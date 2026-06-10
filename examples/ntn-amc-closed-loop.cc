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

#include "ns3/cqi-amc-controller.h"
#include "ns3/geo-beam-scheduler.h"   // M4: 经 scheduler->GetMcsForNewTx 单一真相源
#include "ns3/harq-manager.h"

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
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

struct ProcessRec
{
  bool isNewTx;
  uint8_t mcs;
  double snrAtTx;      // 发射时刻 (t+255) 的真实 SINR (用于 timeline)
  double measuredCqi;  // 上报的原始 CQI (来自 t 时刻)
  double predCqi;      // PredictCqi(H) 当时值
  double effCqi;       // effectiveCqi = pred + Δ
  double deltaAtTx;    // 当时的 OLLA Δ
  double txTimeMs;
};
// key = (rnti << 8) | processId  (HarqManager 的 processId 唯一性是 per-rnti)
static std::map<uint32_t, ProcessRec> g_procRec;

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

static uint32_t
MakeKey (uint16_t rnti, uint8_t pid)
{
  return (static_cast<uint32_t> (rnti) << 8) | pid;
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
OnDecodeAtUe (uint16_t rnti, uint8_t pid);

static void
OnFeedbackAtGw (uint16_t rnti, uint8_t pid, bool ack);

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
  double predCqi = 0.0, effCqi = 0.0, delta = 0.0;
  uint8_t mcs = 0;
  if (g_predictEnabled)
    {
      // M4: scheduler 已注入控制器, UpdateUeDlCqi 内部会喂给 Kalman; 这里直接调即可
      g_scheduler->UpdateUeDlCqi (rnti, measuredCqi);
      predCqi = g_ctrl->PredictCqi (rnti);
      effCqi = g_ctrl->GetEffectiveCqi (rnti);
      delta = g_ctrl->GetDelta (rnti);
      mcs = g_scheduler->GetMcsForNewTx (rnti);   // scheduler 公开包装: effectiveCqi→MCS
    }
  else
    {
      // 基线: scheduler 不注入控制器, GetMcsForNewTx 回退到 latestDlCqi→MCS (过时 CQI)
      predCqi = measuredCqi;
      effCqi = measuredCqi;
      g_scheduler->UpdateUeDlCqi (rnti, measuredCqi);
      mcs = g_scheduler->GetMcsForNewTx (rnti);
    }

  // 发起 HARQ 新传 (载荷大小占位; HarqManager 关心 mcs/状态, 不关心载荷内容)
  Ptr<Packet> pkt = Create<Packet> (1024);
  uint8_t pid = g_harq->NewTransmission (rnti, pkt, mcs);
  if (pid == 255)
    {
      // HARQ 进程满, 跳过此次新传
      return;
    }

  // 记录该进程为新传, 等待 t+510 解码 + t+765 反馈
  ProcessRec rec;
  rec.isNewTx = true;
  rec.mcs = mcs;
  rec.snrAtTx = g_channel[rnti].sinrDb;   // 当前 t+255 的 SINR (供 timeline)
  rec.measuredCqi = measuredCqi;
  rec.predCqi = predCqi;
  rec.effCqi = effCqi;
  rec.deltaAtTx = delta;
  rec.txTimeMs = Simulator::Now ().GetMilliSeconds ();
  g_procRec[MakeKey (rnti, pid)] = rec;

  g_mcsHistogram[mcs]++;

  // 调度解码 (t+510, 即 +half_rtt 后)
  Simulator::Schedule (g_halfRtt, &OnDecodeAtUe, rnti, pid);
}

// ============================================================================
// 事件 3: UE 在 t+510 解码, 用此刻真值 SINR 判 BLER
// ============================================================================
static void
OnDecodeAtUe (uint16_t rnti, uint8_t pid)
{
  auto it = g_procRec.find (MakeKey (rnti, pid));
  if (it == g_procRec.end ())
    return;
  // 此刻 (t+510 from CQI 采样, t+255 from new tx) 的真实 SINR
  double snrNow = g_channel[rnti].sinrDb;
  double bler = BlerForSnrMcs (snrNow, it->second.mcs);
  bool ack = (g_uniRv->GetValue () > bler);

  // 记录预测误差 (predCqi 对照 t+510 的真值 CQI)
  uint8_t actualCqi = MapSinrToCqi (snrNow);
  double err = it->second.predCqi - static_cast<double> (actualCqi);
  g_predErrSumSq += err * err;
  g_predErrCount++;

  // 经半 RTT 回灌反馈
  Simulator::Schedule (g_halfRtt, &OnFeedbackAtGw, rnti, pid, ack);
}

// ============================================================================
// 事件 4: GW 在 t+765 收到反馈 → HarqManager::ReceiveFeedback
//         → m_feedbackCallback (桥接) → OLLA 更新
// ============================================================================
static void
OnFeedbackAtGw (uint16_t rnti, uint8_t pid, bool ack)
{
  SatHarqFeedback fb;
  fb.rnti = rnti;
  fb.processId = pid;
  fb.ack = ack;
  fb.rvIndex = 0;  // 由 HarqManager 内部更新 RV (此处为反馈携带的占位值)
  g_harq->ReceiveFeedback (fb);
  // OLLA 更新走桥接 (HarqFeedbackBridge) 完成, 不在此直接调
}

// ============================================================================
// HARQ 反馈桥接: m_feedbackCallback(rnti, pid, ack) → controller (含 isNewTx 查表)
// ============================================================================
static void
HarqFeedbackBridge (uint16_t rnti, uint8_t pid, bool ack)
{
  auto it = g_procRec.find (MakeKey (rnti, pid));
  bool isNewTx = (it != g_procRec.end ()) ? it->second.isNewTx : false;

  if (g_predictEnabled)
    {
      g_ctrl->OnHarqFeedback (rnti, ack, isNewTx);
    }

  // 写 timeline (仅新传, 与文档定义一致)
  if (isNewTx && it != g_procRec.end ())
    {
      const auto& r = it->second;
      g_timelineFile << std::fixed << std::setprecision (3)
                     << r.txTimeMs / 1000.0 << "," << rnti << "," << r.measuredCqi << ","
                     << r.predCqi << "," << r.effCqi << "," << r.deltaAtTx << ","
                     << static_cast<unsigned> (r.mcs) << "," << r.snrAtTx << ","
                     << (ack ? 1 : 0) << "\n";

      // 滑动窗口 BLER
      g_newTxTotal++;
      if (ack) g_newTxAck++;
      else g_newTxNack++;
      g_blerWindow.push_back (ack ? 0 : 1);
      if (g_blerWindow.size () > kBlerWindowN)
        g_blerWindow.erase (g_blerWindow.begin ());
      uint32_t winNack = 0;
      for (int v : g_blerWindow) winNack += v;
      double winBler = static_cast<double> (winNack) / g_blerWindow.size ();
      double cumBler = static_cast<double> (g_newTxNack) / g_newTxTotal;
      g_blerFile << std::fixed << std::setprecision (3)
                 << Simulator::Now ().GetSeconds () << "," << winBler << "," << cumBler << "\n";
    }

  // 进程记录可清掉 (反馈已处理; HarqManager 重传逻辑由它自己管)
  if (it != g_procRec.end ())
    {
      // 重传由 HarqManager 触发, 我们 example 不直接拉起新一轮 NewTransmission;
      // 简化: 一个进程一次反馈即关闭 (本闭环关注新传统计; 重传链路另议)
      it->second.isNewTx = false;
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
  cmd.AddValue ("predict", "Enable Kalman+OLLA prediction (0=baseline, no controller)", predict);
  cmd.AddValue ("rngRun", "RngRun seed", rngRun);
  cmd.AddValue ("enableLogs", "Enable detailed logging", enableLogs);
  cmd.Parse (argc, argv);

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

  // 创建 HARQ 管理器并接桥接回调
  g_harq = CreateObject<HarqManager> ();
  g_harq->SetHarqEnabled (true);
  g_harq->TraceConnectWithoutContext ("HarqFeedback",
                                      MakeCallback (&HarqFeedbackBridge));

  // 输出目录 + CSV 表头
  mkdir ("ntn-results", 0755);
  g_timelineFile.open ("ntn-results/amc_timeline.csv");
  g_timelineFile << "# OLLA+Kalman closed-loop timeline (per new tx)\n"
                 << "t_s,rnti,measuredCqi,predCqi,effCqi,delta,mcs,snr_db,ack\n";
  g_blerFile.open ("ntn-results/amc_bler_convergence.csv");
  g_blerFile << "# Sliding-window BLER (window=" << kBlerWindowN << " new tx) + cumulative\n"
             << "t_s,window_bler,cumulative_bler\n";

  // 初始化信道并启动事件链 (同时给 scheduler 注册 UE 上下文, 让 baseline 模式
  // 的 latestDlCqi 能正确更新)
  for (uint16_t rnti = 1; rnti <= numUes; ++rnti)
    {
      g_channel[rnti].sinrDb = sinrMean;  // 初始 SINR
      g_scheduler->AddUeContext (rnti);
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
                 + "  R=" + std::to_string (g_ctrl->GetKalmanR (rnti)) + "\n");
    }

  // MCS 直方图
  writeBoth ("\nMCS histogram (new tx):\n");
  for (const auto& kv : g_mcsHistogram)
    writeBoth ("  MCS " + std::to_string (kv.first) + "  count " + std::to_string (kv.second)
               + "\n");

  writeBoth ("\nHarqManager: totalAck=" + std::to_string (g_harq->GetTotalAck ())
             + " totalNack=" + std::to_string (g_harq->GetTotalNack ())
             + " retxRate=" + std::to_string (g_harq->GetRetransmissionRate () * 100.0) + "%\n");
  writeBoth ("====================================================\n");
  writeBoth ("Outputs: ntn-results/amc_{timeline,bler_convergence}.csv  amc_summary.txt\n");

  g_timelineFile.close ();
  g_blerFile.close ();
  sum.close ();

  Simulator::Destroy ();
  return 0;
}
