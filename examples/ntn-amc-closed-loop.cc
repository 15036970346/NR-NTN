// SPDX-License-Identifier: GPL-2.0-only
//
// NTN AMC 闭环仿真 (OLLA 外环 + Kalman CQI 预测)
//
// 忠实复现 contrib/geo-sat/OLLA外环链路自适应.docx 中 765ms (1.5 RTT) 闭环:
//   t      UE 采 C(t) → CQI, 延迟 255ms 上行
//   t+255  GW 收到 CQI: CqiAmcController::OnCqiReport (Kalman 更新)
//          + 选 MCS = GetMcsFromCqi( PredictCqi(H=510) + Δ )
//          + HarqManager::NewTransmission(rnti, pkt, mcs)
//   t+510  UE 解码 (此刻真值 C(t+510) 正是预测目标), 按 BLER(snr,mcs) 判 ACK/NACK
//   t+765  GW 收到 HARQ 反馈: HarqManager::ReceiveFeedback
//          → m_feedbackCallback → CqiAmcController::OnHarqFeedback (仅新传)
//
// 信道 C(t): Gauss-Markov SINR (per-UE 独立).  解码方案 A: 系统级 BLER 近似
//   BLER(snr, mcs) = clamp(0.5·erfc((snr - threshold(mcs))/√2/σ), 0, 1)
// threshold(mcs) 取每个 MCS 的 BLER=0.5 工作点; OLLA 应将平均工作点拉到 ~10%.
//
// 输出 (ntn-results/):
//   amc_timeline.csv          每次新传一行: t,measuredCqi,predCqi,effCqi,delta,mcs,snr,ack
//   amc_bler_convergence.csv  滑动窗口 BLER vs 时间
//   amc_summary.txt           终值汇总
//
// 用法:
//   ./ns3 run "ntn-amc-closed-loop"
//   ./ns3 run "ntn-amc-closed-loop --predict=0"      // 关 Kalman+OLLA 做基线 A/B
//   ./ns3 run "ntn-amc-closed-loop --numUes=4 --simTime=60"

#include "ns3/admit-control.h"        // ServicePriority
#include "ns3/cqi-amc-controller.h"
#include "ns3/geo-beam-scheduler.h"   // M4: 经 scheduler->GetMcsForNewTx 单一真相源
#include "ns3/harq-manager.h"
#include "ns3/harq-policy-controller.h"  // 预测式 HARQ 策略选择器

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnAmcClosedLoop");

// ============================================================================
// 镜像: SatUtPhy::MapSinrToCqi (sat-ut-phy.cc:223, 3GPP TS 38.214 边界)
// ============================================================================
static uint8_t
MapSinrToCqi (double sinrDb)
{
  if (sinrDb < -6.5) return 1;
  if (sinrDb < -4.5) return 2;
  if (sinrDb < -2.5) return 3;
  if (sinrDb < -0.5) return 4;
  if (sinrDb <  1.0) return 5;
  if (sinrDb <  3.0) return 6;
  if (sinrDb <  5.0) return 7;
  if (sinrDb <  7.0) return 8;
  if (sinrDb <  9.0) return 9;
  if (sinrDb < 11.0) return 10;
  if (sinrDb < 13.0) return 11;
  if (sinrDb < 15.0) return 12;
  if (sinrDb < 17.0) return 13;
  if (sinrDb < 19.0) return 14;
  return 15;
}

// ============================================================================
// 系统级 BLER 近似 (解码方案 A):
//   BLER(snr_dB, mcs) = clamp(0.5·erfc((snr - threshold(mcs))/√2/σ), 0, 1)
// threshold(mcs) = MCS 0 在 -7dB, 每 MCS+1 上 ~1.07dB → MCS 28 在 +23dB
// σ = 2dB 的 BLER 转移宽度
// ============================================================================
static double
McsSinrThreshold (uint8_t mcs)
{
  return -7.0 + 1.07 * static_cast<double> (mcs);
}

static double
BlerForSnrMcs (double snrDb, uint8_t mcs)
{
  const double sigma = 2.0;
  double x = (snrDb - McsSinrThreshold (mcs)) / (std::sqrt (2.0) * sigma);
  double bler = 0.5 * std::erfc (x);
  return std::max (0.0, std::min (1.0, bler));
}

// ============================================================================
// 信道 C(t): Gauss-Markov SINR (per-UE)
// ============================================================================
struct ChannelState
{
  double sinrDb;   // 当前真实 SINR (dB)
};

static double g_rho;          // Gauss-Markov 相关系数 (越接近1越平稳)
static double g_sinrMean;     // SINR 长期均值 (dB)
static double g_sinrSigma;    // SINR 稳态标准差 (dB)
static Ptr<NormalRandomVariable> g_gaussRv;
static std::map<uint16_t, ChannelState> g_channel;

static void
UpdateChannel (uint16_t rnti)
{
  auto& c = g_channel[rnti];
  double n = g_gaussRv->GetValue ();
  c.sinrDb = g_rho * c.sinrDb + (1.0 - g_rho) * g_sinrMean
             + std::sqrt (1.0 - g_rho * g_rho) * g_sinrSigma * n;
}

// ============================================================================
// 全局: HARQ + 控制器 + 每进程跟踪 (isNewTx 在 example 侧重建)
// ============================================================================
static Ptr<CqiAmcController> g_ctrl;
static Ptr<HarqManager> g_harq;
static Ptr<UniformRandomVariable> g_uniRv;
// M4: 轻量 scheduler 实例, 仅为 CQI→MCS 单一真相源 (零复制 MCS 表)
static Ptr<GeoBeamScheduler> g_scheduler;

// 统计/输出
static bool g_predictEnabled = true;   // false → 不用 controller, 直接 CQI→MCS (基线)
static std::ofstream g_timelineFile;
static std::ofstream g_blerFile;
static uint64_t g_newTxTotal = 0;
static uint64_t g_newTxAck = 0;
static uint64_t g_newTxNack = 0;
// 滑动窗口 (最近 N 个新传)
static const size_t kBlerWindowN = 200;
static std::vector<int> g_blerWindow;  // 0=ack, 1=nack
static double g_predErrSumSq = 0.0;
static uint64_t g_predErrCount = 0;
static std::map<uint8_t, uint64_t> g_mcsHistogram;

// ============================================================================
// HARQ 策略实验层 (与上面 OLLA 闭环并行, 不干扰其统计):
//   每个新 TB 按 scheme/policy 决定 mode/maxRetx/blerTarget, 独立做"解码+重传链"核算,
//   产出残余 BLER / 吞吐代理 / 时延(VOICE 预算内率) / 重传浪费 / 预测对错鲁棒性。
//   scheme: a=传统(非预测+HARQ固定N) b=HARQ关 c=仅AMC预测(HARQ固定N) d=本方案策略
// ============================================================================
static char g_harqScheme = 'c';              // 默认 c = 现有行为 (预测+固定N)
static uint32_t g_fixedN = 4;                // a/c 的固定最大重传 N
static double g_voiceFraction = 0.0;         // 设为 VOICE 的 UE 比例 (触发业务分支)
static double g_simTimeS = 30.0;
static Ptr<HarqPolicyController> g_policy;
static std::map<uint16_t, ServicePriority> g_uePrio;

struct PolicyTb
{
  uint16_t rnti;
  uint8_t mcs;
  HarqMode mode;
  uint8_t maxRetx;
  double blerTarget;
  double risk;
  double sigma;          // 预测标准差 (scheme d; 供标定)
  ServicePriority prio;
  double measuredCqi;    // 上报原始 CQI
  double predCqi;        // 发射时 PredictCqi (供 predGood)
  double effCqi;         // effectiveCqi = pred + Δ
  double delta;          // 发射时 OLLA Δ
  double snrAtTx;        // 发射时 (t+255) 真实 SINR
  bool predGood {false}; // 首解时定: |predCqi - 真值CQI| ≤ 1
  uint8_t attempt;       // 当前是第几次发送 (1=首传)
  double accumIrGainDb;  // 重传累计 IR 合并增益 (dB)
  double startMs;
};
static uint64_t g_nextTbId = 0;
static std::map<uint64_t, PolicyTb> g_policyTb;

// 策略实验聚合
static uint64_t g_polTotal = 0, g_polDelivered = 0, g_polFailed = 0, g_polRlcFallback = 0;
static uint64_t g_polAttemptsSum = 0, g_polWastedRetx = 0;
static uint64_t g_polVoiceTotal = 0, g_polVoiceInBudget = 0;
static uint64_t g_polOffDecisions = 0, g_polReactiveDecisions = 0;
static uint64_t g_polPredGoodTotal = 0, g_polPredGoodDeliv = 0;
static uint64_t g_polPredBadTotal = 0, g_polPredBadDeliv = 0;
static double g_polThroughput = 0.0;   // Σ SE(mcs) over delivered TBs
static std::ofstream g_judgeFile;

// 吞吐代理: 每 TB 的频谱效率 (bits/sym 近似, 单调随 MCS)。用于"成功交付×SE"的吞吐对比。
static double
SeForMcs (uint8_t mcs)
{
  return std::max (0.15, std::min (7.4, 0.23 * static_cast<double> (mcs) + 0.15));
}

// ============================================================================
// 事件 1: UE 采样 CQI (每 cqiPeriodMs 周期)
// ============================================================================
static Time g_halfRtt;
static uint32_t g_cqiPeriodMs;
static uint32_t g_numUes;

static void
SampleCqiAtUe (uint16_t rnti);

static void
OnCqiAtGw (uint16_t rnti, double measuredCqi);

static void
PolicyDecode (uint64_t tbId);  // HARQ 策略实验的解码+重传链 (唯一闭环)

static void
SampleCqiAtUe (uint16_t rnti)
{
  UpdateChannel (rnti);
  uint8_t cqi = MapSinrToCqi (g_channel[rnti].sinrDb);
  // 经半 RTT 上行到达 GW
  Simulator::Schedule (g_halfRtt, &OnCqiAtGw, rnti, static_cast<double> (cqi));
  // 周期再采
  Simulator::Schedule (MilliSeconds (g_cqiPeriodMs), &SampleCqiAtUe, rnti);
}

// ============================================================================
// 事件 2: GW 收到 CQI → Kalman 更新 + 选 MCS + HARQ 新传
// ============================================================================
static void
OnCqiAtGw (uint16_t rnti, double measuredCqi)
{
  double predCqi = measuredCqi, effCqi = measuredCqi, delta = 0.0;
  // 喂控制器 (UpdateUeDlCqi 内部喂 Kalman); 基线 a 关预测时只更新 latestDlCqi
  g_scheduler->UpdateUeDlCqi (rnti, measuredCqi);
  if (g_predictEnabled)
    {
      predCqi = g_ctrl->PredictCqi (rnti);
      effCqi = g_ctrl->GetEffectiveCqi (rnti);
      delta = g_ctrl->GetDelta (rnti);
    }

  // ---- HARQ 策略决策: scheme 决定 mode / maxRetx / blerTarget(→MCS 偏置) ----
  HarqMode mode = HarqMode::REACTIVE;
  uint8_t maxRetx = static_cast<uint8_t> (g_fixedN);
  double blerTarget = 0.1, risk = 0.0, bias = 0.0, sigma = 0.0;
  switch (g_harqScheme)
    {
    case 'b': // HARQ 恒关
      mode = HarqMode::OFF; maxRetx = 0; blerTarget = 0.1;
      break;
    case 'd': // 本方案: 预测式联合策略
      {
        HarqPolicy p = g_policy->Decide (rnti, g_uePrio[rnti]);
        mode = p.mode; maxRetx = p.maxRetx; blerTarget = p.blerTarget; risk = p.risk; sigma = p.sigma;
        bias = g_policy->BlerTargetToCqiOffset (p.blerTarget);
      }
      break;
    case 'a': // 传统 (非预测, predict 已全局关) + HARQ 固定 N
    case 'c': // 仅 AMC 预测 + HARQ 固定 N (现有状态)
    default:
      mode = HarqMode::REACTIVE; maxRetx = static_cast<uint8_t> (g_fixedN); blerTarget = 0.1;
      break;
    }

  uint8_t mcs = g_scheduler->GetMcsForNewTx (rnti, bias);

  // ---- HARQ 策略实验 TB = 唯一闭环 ----
  // 首解 (PolicyDecode attempt==1) 时用真值 ACK 驱动 OLLA + 写 AMC 输出 (timeline/BLER);
  // 之后按 mode/maxRetx 走解码-重传链, 终判产出残余 BLER / 吞吐 / 时延 / 鲁棒性。
  uint64_t tbId = g_nextTbId++;
  PolicyTb tb;
  tb.rnti = rnti; tb.mcs = mcs; tb.mode = mode; tb.maxRetx = maxRetx;
  tb.blerTarget = blerTarget; tb.risk = risk; tb.sigma = sigma; tb.prio = g_uePrio[rnti];
  tb.measuredCqi = measuredCqi; tb.predCqi = predCqi; tb.effCqi = effCqi; tb.delta = delta;
  tb.snrAtTx = g_channel[rnti].sinrDb;       // t+255 真实 SINR (供 timeline)
  tb.predGood = false; tb.attempt = 1; tb.accumIrGainDb = 0.0;
  tb.startMs = Simulator::Now ().GetMilliSeconds ();
  g_policyTb[tbId] = tb;
  if (mode == HarqMode::OFF) g_polOffDecisions++; else g_polReactiveDecisions++;
  Simulator::Schedule (g_halfRtt, &PolicyDecode, tbId);   // t+510 首解
}

// ============================================================================
// HARQ 策略实验: 终判核算 + 解码/重传链 (本 example 的唯一闭环)
// ============================================================================
static void
FinalizePolicyTb (const PolicyTb& tb, bool delivered)
{
  g_polTotal++;
  g_polAttemptsSum += tb.attempt;
  g_polWastedRetx += (tb.attempt - 1);   // 首传以外的额外份数 = 重传浪费的时隙
  if (delivered)
    {
      g_polDelivered++;
      g_polThroughput += SeForMcs (tb.mcs);   // 成功交付才计入吞吐代理
    }
  else
    {
      g_polFailed++;
      if (tb.mode == HarqMode::OFF) g_polRlcFallback++;  // Tier0 残余 → RLC ARQ 兜底
    }
  if (tb.prio == ServicePriority::PRIORITY_VOICE)
    {
      g_polVoiceTotal++;
      if (delivered && tb.attempt == 1) g_polVoiceInBudget++;  // 1 RTT 内到达视为预算内
    }
  if (tb.predGood) { g_polPredGoodTotal++; if (delivered) g_polPredGoodDeliv++; }
  else             { g_polPredBadTotal++;  if (delivered) g_polPredBadDeliv++; }

  // 自愈闸门 §8 (仅 scheme d): 用 Tier0 自身交付结果反调 TH_LOW
  if (g_harqScheme == 'd' && g_policy)
    g_policy->OnOutcome (tb.rnti, tb.mode, delivered);

  // 判决明细 (每 TB 一行)
  const char* modeStr = (tb.mode == HarqMode::OFF) ? "OFF"
                        : (tb.mode == HarqMode::REACTIVE ? "REACTIVE" : "PROACTIVE");
  g_judgeFile << std::fixed << std::setprecision (4)
              << Simulator::Now ().GetSeconds () << "," << tb.rnti << ","
              << static_cast<int> (tb.prio) << "," << tb.risk << "," << tb.sigma << ","
              << modeStr << "," << tb.blerTarget << "," << static_cast<unsigned> (tb.mcs) << ","
              << static_cast<unsigned> (tb.attempt) << "," << (delivered ? 1 : 0) << ","
              << (tb.predGood ? 1 : 0) << "\n";
}

static void
PolicyDecode (uint64_t tbId)
{
  auto it = g_policyTb.find (tbId);
  if (it == g_policyTb.end ())
    return;
  PolicyTb& tb = it->second;
  const double snr = g_channel[tb.rnti].sinrDb;
  const double effSnr = snr + tb.accumIrGainDb;          // 重传累计 IR 合并增益
  const double bler = BlerForSnrMcs (effSnr, tb.mcs);
  const bool ack = (g_uniRv->GetValue () > bler);

  if (tb.attempt == 1)
    {
      // 预测对/错判定 (predCqi 对照首解时真值 CQI)
      uint8_t actualCqi = MapSinrToCqi (snr);
      tb.predGood = std::abs (tb.predCqi - static_cast<double> (actualCqi)) <= 1.0;

      // === 用首传真值 ACK 闭合 OLLA 外环 (本 example 唯一驱动点) ===
      if (g_predictEnabled)
        g_ctrl->OnHarqFeedback (tb.rnti, ack, true);

      // === AMC 输出: 预测 RMSE + timeline + 滑窗/累计 BLER (口径同原闭环, 仅新传) ===
      double e = tb.predCqi - static_cast<double> (actualCqi);
      g_predErrSumSq += e * e;
      g_predErrCount++;
      g_newTxTotal++;
      if (ack) g_newTxAck++; else g_newTxNack++;
      g_mcsHistogram[tb.mcs]++;
      g_timelineFile << std::fixed << std::setprecision (3)
                     << tb.startMs / 1000.0 << "," << tb.rnti << "," << tb.measuredCqi << ","
                     << tb.predCqi << "," << tb.effCqi << "," << tb.delta << ","
                     << static_cast<unsigned> (tb.mcs) << "," << tb.snrAtTx << ","
                     << (ack ? 1 : 0) << "\n";
      g_blerWindow.push_back (ack ? 0 : 1);
      if (g_blerWindow.size () > kBlerWindowN)
        g_blerWindow.erase (g_blerWindow.begin ());
      uint32_t winNack = 0;
      for (int v : g_blerWindow) winNack += v;
      g_blerFile << std::fixed << std::setprecision (3) << Simulator::Now ().GetSeconds () << ","
                 << static_cast<double> (winNack) / g_blerWindow.size () << ","
                 << static_cast<double> (g_newTxNack) / g_newTxTotal << "\n";
    }

  if (ack)
    {
      FinalizePolicyTb (tb, true);
      g_policyTb.erase (it);
      return;
    }
  // NACK: 可重传则累计 IR 增益后 ~1 RTT 再解; 否则终判失败 (Tier0/到达上限)
  if (tb.mode != HarqMode::OFF && tb.attempt <= tb.maxRetx)
    {
      static const uint8_t kRv[4] = {0, 2, 3, 1};        // 镜像 HarqManager RV 序
      uint8_t rv = kRv[tb.attempt % 4];                  // 第 attempt 次重传用的 RV
      tb.accumIrGainDb += g_harq->CalculateIrGain (rv);  // RV→IR 合并增益 (dB)
      tb.attempt += 1;
      Simulator::Schedule (g_halfRtt + g_halfRtt, &PolicyDecode, tbId);  // 反馈+重传 ≈ 1 RTT
    }
  else
    {
      FinalizePolicyTb (tb, false);
      g_policyTb.erase (it);
    }
}

// ============================================================================
// main
// ============================================================================
int
main (int argc, char* argv[])
{
  uint32_t numUes = 1;
  double simTime = 30.0;        // s
  uint32_t cqiPeriodMs = 40;     // CQI 上报周期
  uint32_t halfRttMs = 255;      // GEO 透明转发半 RTT
  uint32_t horizonH = 510;       // 预测步数 (slot, 与 RTT 一致)
  // 信道 Gauss-Markov 参数
  double rho = 0.995;            // 相关系数 (1.0=不动, 越小越快)
  double sinrMean = 8.0;         // dB
  double sinrSigma = 4.0;        // dB
  // OLLA/Kalman 参数
  double stepUp = 0.05;
  double blerTarget = 0.1;
  double kalmanQ = 0.01;
  double kalmanRmeas = 0.5;
  bool predict = true;
  uint32_t rngRun = 1;
  bool enableLogs = false;
  // HARQ 策略实验
  std::string harqScheme = "c";   // a=传统 b=HARQ关 c=仅AMC预测 d=本方案
  uint32_t fixedN = 4;            // a/c 固定最大重传 N
  double voiceFraction = 0.0;     // VOICE 业务 UE 比例
  double polThLow = 0.30;         // 策略阈值 TH_LOW
  double polThHigh = 0.65;        // 策略阈值 TH_HIGH

  CommandLine cmd;
  cmd.AddValue ("numUes", "Number of UEs", numUes);
  cmd.AddValue ("simTime", "Simulation time (s)", simTime);
  cmd.AddValue ("cqiPeriodMs", "CQI report period (ms)", cqiPeriodMs);
  cmd.AddValue ("halfRttMs", "Half RTT (ms, GEO transparent ~255)", halfRttMs);
  cmd.AddValue ("horizonH", "Kalman prediction horizon (slots)", horizonH);
  cmd.AddValue ("rho", "Channel Gauss-Markov correlation", rho);
  cmd.AddValue ("sinrMean", "Channel SINR long-run mean (dB)", sinrMean);
  cmd.AddValue ("sinrSigma", "Channel SINR std-dev (dB)", sinrSigma);
  cmd.AddValue ("stepUp", "OLLA step_up (CQI unit)", stepUp);
  cmd.AddValue ("blerTarget", "OLLA target BLER", blerTarget);
  cmd.AddValue ("kalmanQ", "Kalman process noise Q", kalmanQ);
  cmd.AddValue ("kalmanRmeas", "Kalman measurement noise Rmeas", kalmanRmeas);
  cmd.AddValue ("predict", "Enable Kalman+OLLA prediction (0=baseline, no controller). scheme=a 强制关",
                predict);
  cmd.AddValue ("rngRun", "RngRun seed", rngRun);
  cmd.AddValue ("enableLogs", "Enable detailed logging", enableLogs);
  cmd.AddValue ("harqScheme",
               "HARQ 对比方案: a=传统(非预测+固定N) b=HARQ关 c=仅AMC预测(固定N,默认) d=本方案策略",
               harqScheme);
  cmd.AddValue ("fixedN", "a/c 方案的固定最大重传次数 N", fixedN);
  cmd.AddValue ("voiceFraction", "设为 VOICE 业务的 UE 比例 [0,1] (触发业务感知分支)", voiceFraction);
  cmd.AddValue ("polThLow", "策略阈值 TH_LOW (risk<此值→关 HARQ)", polThLow);
  cmd.AddValue ("polThHigh", "策略阈值 TH_HIGH (risk≥此值→高冗余/激进)", polThHigh);
  cmd.Parse (argc, argv);

  // scheme 归一化 + predict 联动 (传统方案 a 强制非预测)
  g_harqScheme = harqScheme.empty () ? 'c' : harqScheme[0];
  if (g_harqScheme != 'a' && g_harqScheme != 'b' && g_harqScheme != 'c' && g_harqScheme != 'd')
    g_harqScheme = 'c';
  g_fixedN = fixedN;
  g_voiceFraction = std::max (0.0, std::min (1.0, voiceFraction));
  g_simTimeS = simTime;
  if (g_harqScheme == 'a')
    predict = false;   // 传统方案: 非预测 AMC (用过时 CQI)

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (rngRun);

  if (enableLogs)
    {
      LogComponentEnable ("NtnAmcClosedLoop", LOG_LEVEL_INFO);
      LogComponentEnable ("CqiAmcController", LOG_LEVEL_DEBUG);
      LogComponentEnable ("HarqManager", LOG_LEVEL_INFO);
    }

  g_halfRtt = MilliSeconds (halfRttMs);
  g_cqiPeriodMs = cqiPeriodMs;
  g_numUes = numUes;
  g_predictEnabled = predict;
  g_rho = rho;
  g_sinrMean = sinrMean;
  g_sinrSigma = sinrSigma;

  g_gaussRv = CreateObject<NormalRandomVariable> ();
  g_gaussRv->SetAttribute ("Mean", DoubleValue (0.0));
  g_gaussRv->SetAttribute ("Variance", DoubleValue (1.0));
  g_uniRv = CreateObject<UniformRandomVariable> ();

  // 创建控制器
  g_ctrl = CreateObject<CqiAmcController> ();
  g_ctrl->SetAttribute ("HorizonH", UintegerValue (horizonH));
  g_ctrl->SetAttribute ("StepUp", DoubleValue (stepUp));
  g_ctrl->SetAttribute ("BlerTarget", DoubleValue (blerTarget));
  g_ctrl->SetAttribute ("Q", DoubleValue (kalmanQ));
  g_ctrl->SetAttribute ("Rmeas", DoubleValue (kalmanRmeas));

  // M4: 创建轻量 scheduler 作为 CQI→MCS 单一真相源
  g_scheduler = CreateObject<GeoBeamScheduler> ();
  g_scheduler->Initialize (1, 1);  // beamId=1, scs index=1; 仅用于 GetMcsForNewTx
  if (predict)
    {
      g_scheduler->SetCqiAmcController (g_ctrl);
    }

  // 创建 HARQ 管理器 (策略实验复用其 CalculateIrGain 作 RV→IR 合并增益模型)
  g_harq = CreateObject<HarqManager> ();
  g_harq->SetHarqEnabled (true);

  // 创建 HARQ 策略选择器 (scheme d 用; 读控制器 L/R/P/Δ/bler 信号)
  g_policy = CreateObject<HarqPolicyController> ();
  g_policy->SetController (g_ctrl);
  g_policy->SetAttribute ("ThLow", DoubleValue (polThLow));
  g_policy->SetAttribute ("ThHigh", DoubleValue (polThHigh));
  g_policy->SetAttribute ("MaxRetxN", UintegerValue (fixedN));

  // 输出目录 + CSV 表头
  mkdir ("ntn-results", 0755);
  g_timelineFile.open ("ntn-results/amc_timeline.csv");
  g_timelineFile << "# OLLA+Kalman closed-loop timeline (per new tx)\n"
                 << "t_s,rnti,measuredCqi,predCqi,effCqi,delta,mcs,snr_db,ack\n";
  g_blerFile.open ("ntn-results/amc_bler_convergence.csv");
  g_blerFile << "# Sliding-window BLER (window=" << kBlerWindowN << " new tx) + cumulative\n"
             << "t_s,window_bler,cumulative_bler\n";
  // HARQ 策略判决明细 (按 scheme 命名)
  g_judgeFile.open ("ntn-results/harq_policy_" + std::string (1, g_harqScheme) + "_judge.csv");
  g_judgeFile << "# per-TB HARQ policy decisions (scheme " << g_harqScheme << ")\n"
              << "t_s,rnti,prio,risk,sigma,mode,blerTarget,mcs,attempts,delivered,predGood\n";

  // 初始化信道并启动事件链 (同时给 scheduler 注册 UE 上下文, 让 baseline 模式
  // 的 latestDlCqi 能正确更新)
  uint32_t numVoice = static_cast<uint32_t> (std::ceil (numUes * g_voiceFraction));
  for (uint16_t rnti = 1; rnti <= numUes; ++rnti)
    {
      g_channel[rnti].sinrDb = sinrMean;  // 初始 SINR
      g_scheduler->AddUeContext (rnti);
      // 前 numVoice 个 UE 设为 VOICE (时延敏感), 其余 DATA → 触发 Decide 的业务分支
      g_uePrio[rnti] = (rnti <= numVoice) ? ServicePriority::PRIORITY_VOICE
                                          : ServicePriority::PRIORITY_DATA;
      // 错开各 UE 起始, 避免事件碰撞
      Simulator::Schedule (MilliSeconds (rnti * 7), &SampleCqiAtUe, rnti);
    }

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  // ====== 汇总输出 ======
  std::ofstream sum ("ntn-results/amc_summary.txt");
  auto writeBoth = [&] (const std::string& s) { std::cout << s; sum << s; };

  writeBoth ("\n=== [AMC-LOOP] OLLA + Kalman closed-loop summary ===\n");
  writeBoth ("predict (Kalman+OLLA) : " + std::string (predict ? "ON" : "OFF (baseline)") + "\n");
  writeBoth ("numUes / simTime      : " + std::to_string (numUes) + " / "
             + std::to_string (simTime) + " s\n");
  writeBoth ("horizon H (slot)      : " + std::to_string (horizonH) + "\n");
  writeBoth ("BLER target           : " + std::to_string (blerTarget) + "\n");
  writeBoth ("\nNew TX total/ACK/NACK : " + std::to_string (g_newTxTotal) + " / "
             + std::to_string (g_newTxAck) + " / " + std::to_string (g_newTxNack) + "\n");
  double measBler = g_newTxTotal ? static_cast<double> (g_newTxNack) / g_newTxTotal : 0.0;
  writeBoth ("Measured BLER (newTx) : " + std::to_string (measBler) + "\n");
  double predRmse = g_predErrCount
                        ? std::sqrt (g_predErrSumSq / g_predErrCount)
                        : 0.0;
  writeBoth ("predCqi RMSE vs actual: " + std::to_string (predRmse) + " (CQI units)\n");

  // 每 UE Δ 收敛
  writeBoth ("\nPer-UE OLLA delta convergence:\n");
  for (uint16_t rnti = 1; rnti <= numUes; ++rnti)
    {
      writeBoth ("  rnti " + std::to_string (rnti) + "  Δ=" + std::to_string (g_ctrl->GetDelta (rnti))
                 + "  measuredBLER=" + std::to_string (g_ctrl->GetMeasuredBler (rnti))
                 + "  newTx=" + std::to_string (g_ctrl->GetNewTxCount (rnti))
                 + "  L=" + std::to_string (g_ctrl->GetKalmanL (rnti))
                 + "  R=" + std::to_string (g_ctrl->GetKalmanR (rnti))
                 + "  predVar=" + std::to_string (g_ctrl->GetPredictVariance (rnti)) + "\n");
    }

  // MCS 直方图
  writeBoth ("\nMCS histogram (new tx):\n");
  for (const auto& kv : g_mcsHistogram)
    writeBoth ("  MCS " + std::to_string (kv.first) + "  count " + std::to_string (kv.second)
               + "\n");

  writeBoth ("====================================================\n");
  writeBoth ("Outputs: ntn-results/amc_{timeline,bler_convergence}.csv  amc_summary.txt\n");

  // ====== HARQ 策略实验汇总 ======
  {
    auto f3 = [] (double v) {
      std::ostringstream os; os << std::fixed << std::setprecision (3) << v; return os.str ();
    };
    const double residualBler = g_polTotal ? static_cast<double> (g_polFailed) / g_polTotal : 0.0;
    const double thr = g_simTimeS > 0 ? g_polThroughput / g_simTimeS : 0.0;
    // 每"占用时隙"的有效频谱效率: ΣSE(交付) / Σ尝试。重传额外占隙→分母变大→效率下降,
    // 因此能体现"关 HARQ 省下重传时隙"的吞吐收益 (时间口径 thr 体现不出)。
    const double thrPerSlot = g_polAttemptsSum ? g_polThroughput / g_polAttemptsSum : 0.0;
    const double avgAtt = g_polTotal ? static_cast<double> (g_polAttemptsSum) / g_polTotal : 0.0;
    const double voiceRate = g_polVoiceTotal
                                 ? static_cast<double> (g_polVoiceInBudget) / g_polVoiceTotal : 0.0;
    const double pgRate = g_polPredGoodTotal
                              ? static_cast<double> (g_polPredGoodDeliv) / g_polPredGoodTotal : 0.0;
    const double pbRate = g_polPredBadTotal
                              ? static_cast<double> (g_polPredBadDeliv) / g_polPredBadTotal : 0.0;
    const std::string sc (1, g_harqScheme);

    writeBoth ("\n=== [HARQ-POLICY] scheme " + sc + " 实验汇总 ===\n");
    writeBoth ("  方案/预测              : " + sc + " / " + (predict ? "ON" : "OFF") + "\n");
    writeBoth ("  TB 总数/交付/失败      : " + std::to_string (g_polTotal) + " / "
               + std::to_string (g_polDelivered) + " / " + std::to_string (g_polFailed) + "\n");
    writeBoth ("  残余 BLER (HARQ后)     : " + f3 (residualBler)
               + "  (RLC兜底=" + std::to_string (g_polRlcFallback) + ")\n");
    writeBoth ("  吞吐代理 (ΣSE/s)       : " + f3 (thr) + "\n");
    writeBoth ("  频谱效率 (ΣSE/占用时隙): " + f3 (thrPerSlot) + "\n");
    writeBoth ("  平均尝试/重传浪费时隙  : " + f3 (avgAtt) + " / " + std::to_string (g_polWastedRetx) + "\n");
    writeBoth ("  判决 OFF/REACTIVE      : " + std::to_string (g_polOffDecisions) + " / "
               + std::to_string (g_polReactiveDecisions) + "\n");
    writeBoth ("  VOICE 预算内率         : " + f3 (voiceRate) + "  (" + std::to_string (g_polVoiceInBudget)
               + "/" + std::to_string (g_polVoiceTotal) + ")\n");
    writeBoth ("  鲁棒性 预测对/错交付率 : " + f3 (pgRate) + " / " + f3 (pbRate) + "\n");
    writeBoth ("  策略阈值 TH_LOW/TH_HIGH: " + f3 (g_policy->GetThLow ()) + " / "
               + f3 (g_policy->GetThHigh ()) + "\n");

    std::ofstream mf ("ntn-results/harq_policy_" + sc + ".csv");
    mf << "scheme,predict,num_ues,sim_time_s,tbs,delivered,failed,residual_bler,throughput_proxy,"
          "throughput_per_slot,avg_attempts,wasted_retx,voice_total,voice_in_budget,voice_in_budget_rate,"
          "off_decisions,reactive_decisions,predgood_total,predgood_deliv,predgood_rate,"
          "predbad_total,predbad_deliv,predbad_rate,th_low_final,th_high_final\n";
    mf << std::fixed << std::setprecision (4)
       << g_harqScheme << "," << (predict ? 1 : 0) << "," << numUes << "," << simTime << ","
       << g_polTotal << "," << g_polDelivered << "," << g_polFailed << "," << residualBler << ","
       << thr << "," << thrPerSlot << "," << avgAtt << "," << g_polWastedRetx << "," << g_polVoiceTotal << ","
       << g_polVoiceInBudget << "," << voiceRate << "," << g_polOffDecisions << ","
       << g_polReactiveDecisions << "," << g_polPredGoodTotal << "," << g_polPredGoodDeliv << ","
       << pgRate << "," << g_polPredBadTotal << "," << g_polPredBadDeliv << "," << pbRate << ","
       << g_policy->GetThLow () << "," << g_policy->GetThHigh () << "\n";
    mf.close ();
    writeBoth ("HARQ-policy outputs: ntn-results/harq_policy_" + sc + ".csv (+ _judge.csv)\n");
  }
  writeBoth ("====================================================\n");

  g_timelineFile.close ();
  g_blerFile.close ();
  g_judgeFile.close ();
  sum.close ();

  Simulator::Destroy ();
  return 0;
}
