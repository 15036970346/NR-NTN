// SPDX-License-Identifier: GPL-2.0-only
//
// HarqPolicyController — 预测驱动的 AMC + HARQ 联合策略选择器
//
// 设计文档: contrib/geo-sat/基于CQI预测的HARQ增强伪代码设计.md
//
// 在调度时刻读取 CqiAmcController 的四个信号 (L 电平, R 趋势, P 不确定度→sigma, Δ/实测BLER)
// 结合业务优先级 ServicePriority, 融合成 [0,1] 的"信道风险分" risk, 再据 risk + 业务输出
// 一个 HarqPolicy{mode, K, rvList, blerTarget, maxRetx}; 由调度器/example 据此决定
// MCS 目标 BLER 偏置 + HARQ 模式 + (Tier2 的) 重复份数与 RV 序列。
//
// 本轮(第一轮)实现 Tier0(OFF)/Tier1(REACTIVE) 与自愈闸门 §8;
// Tier2(PROACTIVE) 枚举先预留, Decide 命中高风险时回退 REACTIVE 并打日志 (见 §4)。

#ifndef HARQ_POLICY_CONTROLLER_H
#define HARQ_POLICY_CONTROLLER_H

#include "admit-control.h"      // ServicePriority
#include "cqi-amc-controller.h"

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <vector>

namespace ns3 {

/** HARQ 三档模式 (文档 §2)。 */
enum class HarqMode
{
  OFF = 0,       ///< Tier0: 关 HARQ, 单发 RV0, 不等 ACK, RLC ARQ 兜底
  REACTIVE = 1,  ///< Tier1: 标准反馈式 HARQ, RV 序列 0→2→3→1
  PROACTIVE = 2  ///< Tier2: 主动冗余 (本轮预留; Decide 暂回退 REACTIVE)
};

/** 一次判决输出的策略 (文档 §2)。 */
struct HarqPolicy
{
  HarqMode mode {HarqMode::REACTIVE};
  uint8_t K {1};                   ///< 一次性发送份数 (Tier0/1=1, Tier2=2~3)
  std::vector<uint8_t> rvList {0}; ///< 各份 RV (Tier2 用; 本轮恒 {0})
  double blerTarget {0.1};         ///< 传给 AMC 的目标 BLER → MCS 偏置
  uint8_t maxRetx {4};             ///< 最大重传次数 (Tier0=0, Tier1=N)
  double risk {0.0};               ///< 本次判决的风险分 (供统计/日志)
  double sigma {0.0};              ///< 预测标准差 (供统计/标定 SIGMA_REF)
};

/** 判决用的瞬时信道画像 (文档 §2/§5)。 */
struct ChannelView
{
  double L {0.0};      ///< 预测电平 GetKalmanL
  double R {0.0};      ///< 趋势斜率 GetKalmanR (>0 变好, <0 变差)
  double sigma {0.0};  ///< 预测标准差 = sqrt(GetPredictVariance)
  double bler {0.0};   ///< 实测 BLER GetMeasuredBler
  bool fresh {false};  ///< HasFreshEstimate
};

/**
 * \brief 预测驱动的 AMC+HARQ 策略选择器 (per-UE 判决)。
 */
class HarqPolicyController : public Object
{
public:
  static TypeId GetTypeId ();
  HarqPolicyController ();
  ~HarqPolicyController () override;

  /** 注入 CQI/AMC 控制器 (提供 L/R/P/Δ/bler 信号)。 */
  void SetController (Ptr<CqiAmcController> ctrl) { m_ctrl = ctrl; }

  /** 读取该 rnti 的信道画像 (文档 §5)。 */
  ChannelView ReadChannelView (uint16_t rnti) const;

  /** 把四信号融合成 [0,1] 风险分 (文档 §3); 越高越该上冗余。 */
  double ComputeRisk (const ChannelView& v) const;

  /** 按风险 + 业务优先级输出策略 (文档 §4)。 */
  HarqPolicy Decide (uint16_t rnti, ServicePriority prio);

  /**
   * 自愈闸门 §8 (闸门1): 仅看 **Tier0(OFF) 自身** 的残余失败率 (EWMA)。
   * Tier0 残余失败 EWMA > BLER_SAFE·GUARD → 收紧 TH_LOW (更难进 Tier0); 健康则松回初值。
   * 复用 OLLA 思想 (实测反调阈值), 但用 Tier0 专属口径, 避免被全局 OLLA BLER 误触发。
   * \param delivered 该 OFF TB 是否成功交付 (单发即成功)。
   */
  void OnOutcome (uint16_t rnti, HarqMode lastMode, bool delivered);

  /**
   * blerTarget → CQI 偏置 (相对 nominal): 激进目标(高BLER)→正偏置抬 MCS;
   * 保守目标(低BLER)→负偏置降 MCS。offset = OffsetGain·log10(target/nominal), clamp ±3。
   */
  double BlerTargetToCqiOffset (double blerTarget) const;

  // getter 供 example 统计/输出
  double GetThLow () const { return m_thLow; }
  double GetThHigh () const { return m_thHigh; }

private:
  static double Clamp (double v, double lo, double hi);
  HarqPolicy MakeOff () const;
  HarqPolicy MakeReactive (double blerTarget, uint8_t maxRetx) const;

  Ptr<CqiAmcController> m_ctrl;

  // 风险权重 (文档 §3 建议初值 0.40/0.30/0.20/0.10)
  double m_w1 {0.40};
  double m_w2 {0.30};
  double m_w3 {0.20};
  double m_w4 {0.10};
  // 归一化基准 (SigmaRef 随 H·sqrt(Q) 量级标定, 见 .cc 属性说明)
  double m_sigmaRef {300.0};
  double m_blerRef {0.30};
  double m_cqiMin {0.0};
  double m_cqiMax {15.0};

  // 阈值 (运行时可被自愈修改; attribute 设初值)
  double m_thLow {0.30};
  double m_thHigh {0.65};

  // 各档目标 BLER
  double m_blerSafe {0.01};
  double m_blerNominal {0.10};
  double m_blerAggr {0.30};
  uint32_t m_maxRetxN {4};       // Tier1 最大重传 N
  Time m_freshMaxAge {MilliSeconds (1000)};

  // 自愈 (Tier0 专属残余失败 EWMA + TH_LOW 在 [0.5·init, init] 内收紧/松回)
  double m_guardFactor {5.0};
  double m_offsetGain {2.5};
  double m_tier0FailEwma {0.0};
  double m_ewmaAlpha {0.02};
  double m_thLowInit {-1.0};  // 懒捕获配置初值 (Decide 首次调用时)
};

} // namespace ns3

#endif // HARQ_POLICY_CONTROLLER_H
