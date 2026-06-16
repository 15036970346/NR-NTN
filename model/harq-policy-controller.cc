// SPDX-License-Identifier: GPL-2.0-only
//
// HarqPolicyController 实现 (文档 §3 风险分 + §4 判决 + §8 自愈)。

#include "harq-policy-controller.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HarqPolicyController");
NS_OBJECT_ENSURE_REGISTERED (HarqPolicyController);

TypeId
HarqPolicyController::GetTypeId ()
{
  static TypeId tid =
      TypeId ("ns3::HarqPolicyController")
          .SetParent<Object> ()
          .SetGroupName ("GeoSat")
          .AddConstructor<HarqPolicyController> ()
          // --- 风险权重 (§3) ---
          .AddAttribute ("RiskW1", "风险权重: 电平项 (1-Lnorm)", DoubleValue (0.40),
                         MakeDoubleAccessor (&HarqPolicyController::m_w1),
                         MakeDoubleChecker<double> (0.0))
          .AddAttribute ("RiskW2", "风险权重: 趋势项 (-R·H)", DoubleValue (0.30),
                         MakeDoubleAccessor (&HarqPolicyController::m_w2),
                         MakeDoubleChecker<double> (0.0))
          .AddAttribute ("RiskW3", "风险权重: 不确定度项 (sigma)", DoubleValue (0.20),
                         MakeDoubleAccessor (&HarqPolicyController::m_w3),
                         MakeDoubleChecker<double> (0.0))
          .AddAttribute ("RiskW4", "风险权重: 历史 BLER 项", DoubleValue (0.10),
                         MakeDoubleAccessor (&HarqPolicyController::m_w4),
                         MakeDoubleChecker<double> (0.0))
          // SigmaRef 标定到前向预测 std 的量级: sigma≈H·sqrt(Q) (H=510,Q=0.01 → ~51 稳态, 冷启~510)。
          // 设 300 使稳态 riskUncert≈0.17、冷启≈1, 既保留"不可信→加冗余"又不恒饱和。
          .AddAttribute ("SigmaRef", "不确定度归一化基准 (CQI std, 随 H·sqrt(Q) 量级标定)",
                         DoubleValue (300.0),
                         MakeDoubleAccessor (&HarqPolicyController::m_sigmaRef),
                         MakeDoubleChecker<double> (1e-6))
          // BlerRef 设在工作点之上 (OLLA 目标 0.1), 使正常 BLER 不饱和; 超过 ~0.3 才显著抬风险。
          .AddAttribute ("BlerRef", "历史 BLER 归一化基准", DoubleValue (0.30),
                         MakeDoubleAccessor (&HarqPolicyController::m_blerRef),
                         MakeDoubleChecker<double> (1e-6))
          // --- 阈值 (§4) ---
          .AddAttribute ("ThLow", "risk < 此值 → Tier0 关 HARQ", DoubleValue (0.30),
                         MakeDoubleAccessor (&HarqPolicyController::m_thLow),
                         MakeDoubleChecker<double> (0.0, 1.0))
          .AddAttribute ("ThHigh", "risk ≥ 此值 → (本应 Tier2) 高冗余", DoubleValue (0.65),
                         MakeDoubleAccessor (&HarqPolicyController::m_thHigh),
                         MakeDoubleChecker<double> (0.0, 1.0))
          // --- 各档目标 BLER ---
          .AddAttribute ("BlerSafe", "Tier0 目标 BLER (保守)", DoubleValue (0.01),
                         MakeDoubleAccessor (&HarqPolicyController::m_blerSafe),
                         MakeDoubleChecker<double> (1e-6, 0.999))
          .AddAttribute ("BlerNominal", "Tier1 目标 BLER", DoubleValue (0.10),
                         MakeDoubleAccessor (&HarqPolicyController::m_blerNominal),
                         MakeDoubleChecker<double> (1e-6, 0.999))
          .AddAttribute ("BlerAggr", "高风险首传目标 BLER (激进)", DoubleValue (0.30),
                         MakeDoubleAccessor (&HarqPolicyController::m_blerAggr),
                         MakeDoubleChecker<double> (1e-6, 0.999))
          .AddAttribute ("MaxRetxN", "Tier1 最大重传次数 N", UintegerValue (4),
                         MakeUintegerAccessor (&HarqPolicyController::m_maxRetxN),
                         MakeUintegerChecker<uint32_t> (0))
          .AddAttribute ("FreshMaxAge", "预测新鲜度上限 (超过则回退 REACTIVE)",
                         TimeValue (MilliSeconds (1000)),
                         MakeTimeAccessor (&HarqPolicyController::m_freshMaxAge),
                         MakeTimeChecker ())
          // --- 自愈 / 偏置 ---
          .AddAttribute ("GuardFactor", "自愈闸门1: Tier0 残余失败容错倍数 (×BlerSafe)",
                         DoubleValue (5.0),
                         MakeDoubleAccessor (&HarqPolicyController::m_guardFactor),
                         MakeDoubleChecker<double> (1.0))
          .AddAttribute ("OffsetGain", "blerTarget→CQI 偏置增益 (log10 比例; 越大 Tier0 越保守)",
                         DoubleValue (2.5),
                         MakeDoubleAccessor (&HarqPolicyController::m_offsetGain),
                         MakeDoubleChecker<double> (0.0))
          .AddAttribute ("CqiMax", "CQI 上限 (归一化用)", DoubleValue (15.0),
                         MakeDoubleAccessor (&HarqPolicyController::m_cqiMax),
                         MakeDoubleChecker<double> (1.0));
  return tid;
}

HarqPolicyController::HarqPolicyController () = default;
HarqPolicyController::~HarqPolicyController () = default;

double
HarqPolicyController::Clamp (double v, double lo, double hi)
{
  return std::max (lo, std::min (hi, v));
}

ChannelView
HarqPolicyController::ReadChannelView (uint16_t rnti) const
{
  ChannelView v;
  if (!m_ctrl)
    return v;  // 无控制器: fresh=false → Decide 走安全回退
  v.L = m_ctrl->GetKalmanL (rnti);
  v.R = m_ctrl->GetKalmanR (rnti);
  v.sigma = std::sqrt (std::max (0.0, m_ctrl->GetPredictVariance (rnti)));
  v.bler = m_ctrl->GetMeasuredBler (rnti);
  v.fresh = m_ctrl->HasFreshEstimate (rnti, m_freshMaxAge);
  return v;
}

double
HarqPolicyController::ComputeRisk (const ChannelView& v) const
{
  const double H = m_ctrl ? static_cast<double> (m_ctrl->GetHorizonH ()) : 510.0;
  // 1) 电平项: 预测电平低 → 风险高
  const double Lnorm = Clamp ((v.L - m_cqiMin) / (m_cqiMax - m_cqiMin), 0.0, 1.0);
  const double riskLevel = 1.0 - Lnorm;
  // 2) 趋势项: 预测视界 H 内会掉多少 CQI (R<0 时 drop>0)。GEO 关键: 看 H 步后而非当下。
  const double drop = Clamp (-v.R * H, 0.0, m_cqiMax);
  const double riskTrend = drop / m_cqiMax;
  // 3) 不确定度项: 预测越不可信越保守
  const double riskUncert = Clamp (v.sigma / m_sigmaRef, 0.0, 1.0);
  // 4) 历史可靠性项: 实测 BLER 超标 → 抬风险
  const double riskBler = Clamp (v.bler / m_blerRef, 0.0, 1.0);

  double risk = m_w1 * riskLevel + m_w2 * riskTrend + m_w3 * riskUncert + m_w4 * riskBler;
  return Clamp (risk, 0.0, 1.0);
}

HarqPolicy
HarqPolicyController::MakeOff () const
{
  HarqPolicy p;
  p.mode = HarqMode::OFF;
  p.K = 1;
  p.rvList = {0};
  p.blerTarget = m_blerSafe;
  p.maxRetx = 0;
  return p;
}

HarqPolicy
HarqPolicyController::MakeReactive (double blerTarget, uint8_t maxRetx) const
{
  HarqPolicy p;
  p.mode = HarqMode::REACTIVE;
  p.K = 1;
  p.rvList = {0};
  p.blerTarget = blerTarget;
  p.maxRetx = maxRetx;
  return p;
}

HarqPolicy
HarqPolicyController::Decide (uint16_t rnti, ServicePriority prio)
{
  if (m_thLowInit < 0.0)
    m_thLowInit = m_thLow;  // 懒捕获配置初值, 作自愈松回上限
  ChannelView v = ReadChannelView (rnti);

  // 新鲜度回退: 预测不可信 → 退回安全的 Tier1 (文档 §4)
  if (!v.fresh)
    {
      HarqPolicy p = MakeReactive (m_blerNominal, static_cast<uint8_t> (m_maxRetxN));
      p.risk = 0.0;
      return p;
    }

  const double risk = ComputeRisk (v);
  HarqPolicy p;
  p.sigma = v.sigma;

  const bool latencySensitive =
      (prio == ServicePriority::PRIORITY_EMERGENCY || prio == ServicePriority::PRIORITY_VOICE);

  if (latencySensitive)
    {
      // 时延敏感 (VOICE/EMERGENCY): 反应式 600ms 重传=废包 → 一律单发 (Tier0),
      // 用保守 MCS(blerSafe) 尽量一次成功, 偶发失败交 RLC 兜底。
      // (高风险本应 Tier2 主动冗余以兼顾可靠, 本轮 Tier2 未实现 → 仍单发并打日志。)
      p = MakeOff ();
      if (risk >= m_thLow)
        NS_LOG_INFO ("[HarqPolicy] rnti=" << rnti << " VOICE/EMERG risk=" << risk
                     << " 应 Tier2(PROACTIVE 冗余), 本轮回退 Tier0 单发(保守 MCS)");
    }
  else
    {
      // DATA / BEST_EFFORT: 吞吐导向, 容忍时延
      if (risk < m_thLow)
        {
          p = MakeOff ();  // 信道好且可信 → 关 HARQ 抢吞吐, AMC 保守保首传
        }
      else if (risk < m_thHigh)
        {
          p = MakeReactive (m_blerNominal, static_cast<uint8_t> (m_maxRetxN));
        }
      else
        {
          // TODO(Tier2): 高风险本应 PROACTIVE 主动冗余。本轮回退 REACTIVE + 激进首传。
          p = MakeReactive (m_blerAggr, static_cast<uint8_t> (m_maxRetxN));
          NS_LOG_INFO ("[HarqPolicy] rnti=" << rnti << " DATA risk=" << risk
                       << " ≥TH_HIGH 应 Tier2(PROACTIVE), 本轮回退 REACTIVE");
        }
    }

  p.risk = risk;
  return p;
}

void
HarqPolicyController::OnOutcome (uint16_t rnti, HarqMode lastMode, bool delivered)
{
  // 闸门1 (§8): 只用 Tier0(OFF) 自身的残余失败率 EWMA, 避免被全局 OLLA BLER 误触发。
  if (lastMode != HarqMode::OFF)
    return;
  if (m_thLowInit < 0.0)
    m_thLowInit = m_thLow;
  m_tier0FailEwma = (1.0 - m_ewmaAlpha) * m_tier0FailEwma + m_ewmaAlpha * (delivered ? 0.0 : 1.0);
  const double ref = m_blerSafe * m_guardFactor;   // 容许的 Tier0 残余失败上限
  const double before = m_thLow;
  if (m_tier0FailEwma > ref)
    m_thLow = Clamp (m_thLow * 0.98, 0.5 * m_thLowInit, m_thLowInit);   // Tier0 太烂 → 收紧
  else
    m_thLow = Clamp (m_thLow + 0.002, 0.5 * m_thLowInit, m_thLowInit);  // 健康 → 松回
  if (std::abs (m_thLow - before) > 1e-9)
    NS_LOG_DEBUG ("[HarqPolicy][自愈] rnti=" << rnti << " Tier0FailEwma=" << m_tier0FailEwma
                  << " (ref=" << ref << ") TH_LOW " << before << "→" << m_thLow);
}

double
HarqPolicyController::BlerTargetToCqiOffset (double blerTarget) const
{
  if (blerTarget <= 0.0)
    return -3.0;
  // 相对 nominal: 激进(高BLER)→正偏置抬 MCS; 保守(低BLER)→负偏置降 MCS
  double off = m_offsetGain * std::log10 (blerTarget / m_blerNominal);
  return Clamp (off, -3.0, 3.0);
}

} // namespace ns3
