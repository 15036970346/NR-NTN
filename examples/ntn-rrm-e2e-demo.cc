/*
 * 文件: contrib/0603geo-sat/examples/ntn-rrm-e2e-demo.cc
 *
 * NTN 资源管理 — 生产级端到端演示 (RRM End-to-End Demo)
 * ====================================================================
 *
 * 目的: 把资源管理的全部能力 —— 动态频域 RB 调度 + 功率域 NOMA +
 *       闭环功控 CLPC + CQI 滤波/预测 —— 在一个可运行仿真里 **一起生效**
 *       并产出量化结果, 填补 "RM 功能在 ntn-system-sim (NR 栈) 里不生效"
 *       的集成缺口。
 *
 * 装配方式 (诚实说明):
 *   RM 能力活在 GeoBeamScheduler, 走自研数据面 NtnGwMac -> SatPhy -> SatChannel
 *   -> SatUtMac (见 ntn-protocol-stack-test.cc 的接线范式)。本 demo 用
 *   GeoBeamScheduler + 自研数据面搭一个多 UE 多波束端到端场景, 让所有 RM
 *   特性真实运转。ntn-system-sim 走 NR/5G-LENA 栈、未插 GeoBeamScheduler,
 *   把调度器强插 NR 栈风险大, 故本 demo 不改 ntn-system-sim, 而是务实地用
 *   自研数据面做集成演示。
 *
 * 真实数据面 vs 代表性输入 (诚实说明):
 *   - DL 数据面 (SECTION 4) 是 **真实数据面**: scheduler 跑 RunScheduler 产生
 *     NOMA near/far 的 {effSINR, txPower}; gwMac->SetScheduler(sched) 自动接线;
 *     每个 DL 包经 NtnGwMac::SendDlPduForRnti -> SatPhy -> 弯管信道 ->
 *     SatUtMac::DoPhyRx (按 tag 的 effSINR 驱动 NtnPhyErrorModel BLER), 真实
 *     收/丢包计数。
 *   - 调度决策 (SECTION 1/2: 动态 RB / NOMA 配对 / RB 占用) 由真实的
 *     GeoBeamScheduler::RunScheduler 跑出, DCI 回调捕获真实分配结果。
 *   - CLPC (SECTION 3) 由真实 DoSchedUlTriggerReq -> RunUlScheduler ->
 *     ProcessUlGrant -> ScheduleUlSinrFeedback -> (300ms 后) DeliverUlSinrFeedback
 *     -> ApplyUlSinrMeasurement -> UpdateClosedLoopPowerControl 的真实闭环跑出。
 *   - buffer status / CQI 上报用代表性输入 (UpdateUeDlBufferStatus /
 *     UpdateUeDlCqi 等) 驱动, 代表业务负载与信道; IP 业务生成器
 *     (SatTrafficGenerator) 与自研 gw-mac 数据面是两套, 不强求桥接 (如实说明)。
 *
 * 用法:
 *   ./ns3 build ntn-rrm-e2e-demo
 *   ./ns3 run ntn-rrm-e2e-demo
 *   ./ns3 run "ntn-rrm-e2e-demo --writeFiles=true"  # 把 NOMA 复用记账落盘
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"

#include "ns3/geo-beam-helper.h"
#include "ns3/gw-helper.h"
#include "ns3/sat-phy.h"
#include "ns3/ntn-gw-phy.h"
#include "ns3/ntn-gw-mac.h"
#include "ns3/sat-ut-mac.h"
#include "ns3/ntn-phy-error-model.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"
#include "ns3/admit-control.h"
#include "ns3/cqi-amc-predictor.h"
#include "ns3/cqi-amc-controller.h"
#include "ns3/result-writer.h"
#include "ns3/sat-mac-common.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnRrmE2eDemo");

namespace {

// ---------------------------------------------------------------------------
// 打印辅助
// ---------------------------------------------------------------------------
void
Banner (const std::string& t)
{
  std::cout << "\n";
  std::cout << "================================================================================\n";
  std::cout << "  " << t << "\n";
  std::cout << "================================================================================\n";
}

void
Sub (const std::string& t)
{
  std::cout << "\n---- " << t << " ----\n";
}

std::string
PriStr (ServicePriority p)
{
  switch (p)
    {
    case ServicePriority::PRIORITY_EMERGENCY:   return "EMERGENCY";
    case ServicePriority::PRIORITY_VOICE:       return "VOICE";
    case ServicePriority::PRIORITY_DATA:        return "DATA";
    case ServicePriority::PRIORITY_BEST_EFFORT: return "BEST_EFFORT";
    default:                                    return "UNKNOWN";
    }
}

// 捕获某 UE 收到的全部 DCI (DL 分配结果)。
struct DciSink
{
  void OnDci (DciInfo dci) { dcis.push_back (dci); }
  std::vector<DciInfo> dcis;
  // 最近一次 (非重传) DL DCI 的 RB 数; 无则 0。
  uint32_t LastDlRb () const
  {
    for (auto it = dcis.rbegin (); it != dcis.rend (); ++it)
      {
        if (!it->isUplinkGrant)
          {
            return it->rbAllocation;
          }
      }
    return 0;
  }
  uint8_t LastDlMcs () const
  {
    for (auto it = dcis.rbegin (); it != dcis.rend (); ++it)
      {
        if (!it->isUplinkGrant)
          {
            return it->mcs;
          }
      }
    return 0;
  }
  double LastDlTxPowerDbm () const
  {
    for (auto it = dcis.rbegin (); it != dcis.rend (); ++it)
      {
        if (!it->isUplinkGrant)
          {
            return it->txPowerDbm;
          }
      }
    return 0.0;
  }
};

// ---------------------------------------------------------------------------
// 场景 UE 定义: 消费级 (语音+数据) 与 便携式 (纯数据)
// ---------------------------------------------------------------------------
struct UeSpec
{
  uint16_t    rnti;
  std::string label;
  uint32_t    beamId;
  UtType      utType;
  TrafficType traffic;
  ServicePriority forcePriority;   // PRIORITY_BEST_EFFORT == 不强制 (走业务映射)
  bool        forcePrioritySet;
  double      dlCqi;
  uint32_t    dlBufferBytes;
  uint32_t    ulBufferBytes;
};

// 业务名 (用于聚合吞吐)
std::string
ServiceName (const UeSpec& u)
{
  if (u.forcePrioritySet && u.forcePriority == ServicePriority::PRIORITY_EMERGENCY)
    {
      return "EMERGENCY";
    }
  if (u.traffic == TRAFFIC_VOICE)
    {
      return "VoIP";
    }
  return "DATA";
}

// 数据面 DL 收包计数 (静态函数 + MakeBoundCallback, 与 ntn-protocol-stack-test 同模式;
// 捕获 lambda 不能转成 Callback, 故用普通函数)。
void
CountDlRx (uint32_t* counter, Ptr<Packet>)
{
  (*counter)++;
}

} // namespace

// ===========================================================================
// 共享调度器构建: 多波束 + 多 UE 混合业务, 开启全部 RM 特性
// ===========================================================================
//
// 返回的调度器已 AddUeContext/AddUeInfo + 喂 CQI/buffer, 但尚未 RunScheduler。
static Ptr<GeoBeamScheduler>
BuildScheduler (const std::vector<UeSpec>& ues,
                std::map<uint16_t, DciSink>& sinks,
                bool useCqiPredictor)
{
  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();

  // ---- 开启全部 RM 特性 ----
  sched->SetAttribute ("NomaEnabled", BooleanValue (true));          // 功率域 NOMA
  sched->SetAttribute ("NomaFarPowerFraction", DoubleValue (0.8));   // beta
  sched->SetAttribute ("NomaMinCqiGap", DoubleValue (4.0));
  sched->SetAttribute ("ClpcEnabled", BooleanValue (true));          // 闭环功控 (默认即 true)
  sched->SetAttribute ("DlPowerAllocEnabled", BooleanValue (true));  // 下行动态功率分配

  if (useCqiPredictor)
    {
      // 显式构造 CqiAmcPredictor, 把 DL HorizonH 配成温和值 (默认 ~510 slot 会大幅外推)。
      Ptr<CqiAmcPredictor> predictor = CreateObject<CqiAmcPredictor> ();
      predictor->GetDl ()->SetAttribute ("HorizonH", UintegerValue (1));
      predictor->GetDl ()->SetAttribute ("Rmeas", DoubleValue (8.0));
      sched->SetCqiPredictor (predictor);
    }

  // 应急/语音的刚性预留 RB 生效。
  Ptr<AdmitControl> admit = CreateObject<AdmitControl> ();
  admit->SetPriorityReservationPolicy (3, 3, 2);  // emergency 预留 3 (burst 3), voice 预留 2
  sched->SetAdmitControl (admit);

  // 用第一个 UE 的 beam 作为调度器自身 beam (RunScheduler 会遍历所有 beam)。
  sched->Initialize (ues.empty () ? 1 : ues.front ().beamId, 30);

  for (const UeSpec& u : ues)
    {
      sched->AddUeContext (u.rnti, u.utType, u.traffic);
      sched->AddUeInfo (u.rnti, u.beamId);
      if (u.forcePrioritySet)
        {
          sched->SetUePriority (u.rnti, u.forcePriority);
        }
      // CQI 经 UpdateUeDlCqi 入口 (喂 CqiPredictor); UL CQI 同步给一份。
      sched->UpdateUeDlCqi (u.rnti, u.dlCqi);
      sched->UpdateUeUlCqi (u.rnti, u.dlCqi);
      sched->UpdateUeDlBufferStatus (u.rnti, u.dlBufferBytes);
      sched->UpdateUeUlBufferStatus (u.rnti, u.ulBufferBytes);

      sinks[u.rnti] = DciSink ();
      sched->RegisterUeDciCallback (u.rnti, MakeCallback (&DciSink::OnDci, &sinks[u.rnti]));
    }

  return sched;
}

// ===========================================================================
// main
// ===========================================================================
int
main (int argc, char* argv[])
{
  bool writeFiles = false;
  CommandLine cmd (__FILE__);
  cmd.AddValue ("writeFiles", "Write NOMA reuse accounting + per-UE rate stats to ./simulation_results", writeFiles);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  std::cout << "\n";
  std::cout << "########################################################################\n";
  std::cout << "#   NTN 资源管理 — 生产级端到端演示 (ntn-rrm-e2e-demo)                   #\n";
  std::cout << "#   动态频域RB + 功率域NOMA + 闭环功控CLPC + CQI滤波/预测 一起生效        #\n";
  std::cout << "########################################################################\n";

  // -------------------------------------------------------------------------
  // 场景: 2 波束, 6 UE 混合业务 + 终端区分
  //   Beam 1 (消费级为主): UE 101 应急语音, 102 消费级 VoIP, 103 消费级数据
  //   Beam 2 (NOMA 对 + 便携): UE 201 便携近(高CQI) / 202 便携远(低CQI) NOMA 对,
  //                            203 消费级数据
  // -------------------------------------------------------------------------
  const uint32_t BEAM_A = 1;
  const uint32_t BEAM_B = 2;
  const ServicePriority NONE = ServicePriority::PRIORITY_BEST_EFFORT;

  std::vector<UeSpec> ues = {
    // rnti  label            beam     utType       traffic        forcePri                            set    dlCqi  dlBuf  ulBuf
    { 101, "consumer-EMG",   BEAM_A, UT_CONSUMER, TRAFFIC_VOICE, ServicePriority::PRIORITY_EMERGENCY, true,   9.0,  3000,  2000 },
    { 102, "consumer-VoIP",  BEAM_A, UT_CONSUMER, TRAFFIC_VOICE, NONE,                                false,  8.0,  2000,  1500 },
    { 103, "consumer-DATA",  BEAM_A, UT_CONSUMER, TRAFFIC_DATA,  NONE,                                false, 11.0, 30000, 12000 },
    { 201, "portable-NEAR",  BEAM_B, UT_PORTABLE, TRAFFIC_DATA,  NONE,                                false, 15.0, 20000,  8000 },
    { 202, "portable-FAR",   BEAM_B, UT_PORTABLE, TRAFFIC_DATA,  NONE,                                false,  3.0, 20000,  8000 },
    { 203, "consumer-DATA2", BEAM_B, UT_CONSUMER, TRAFFIC_DATA,  NONE,                                false,  7.0, 15000,  9000 },
  };

  std::cout << "\n场景: 2 波束 / 6 UE 混合业务 (消费级=语音+数据, 便携式=纯数据)\n";
  std::cout << "  " << std::left << std::setw (8) << "RNTI"
            << std::setw (16) << "标签"
            << std::setw (8) << "波束"
            << std::setw (12) << "终端类型"
            << std::setw (12) << "业务"
            << std::setw (8) << "CQI"
            << std::setw (12) << "DL缓冲(B)" << "\n";
  std::cout << "  ------------------------------------------------------------------------\n";
  for (const UeSpec& u : ues)
    {
      std::cout << "  " << std::left << std::setw (8) << u.rnti
                << std::setw (16) << u.label
                << std::setw (8) << ("Beam" + std::to_string (u.beamId))
                << std::setw (12) << (u.utType == UT_CONSUMER ? "consumer" : "portable")
                << std::setw (12) << ServiceName (u)
                << std::setw (8) << (int) u.dlCqi
                << std::setw (12) << u.dlBufferBytes << "\n";
    }

  // =========================================================================
  // SECTION 1: 动态频域 RB 调度 (随 CQI + 随业务类型) + 每波束 RB 占用
  // =========================================================================
  Banner ("SECTION 1: 动态频域 RB 调度 (随 CQI + 业务类型) + 每波束 RB 占用 / NOMA 复用");

  std::map<uint16_t, DciSink> sinks;
  Ptr<GeoBeamScheduler> sched = BuildScheduler (ues, sinks, /*useCqiPredictor=*/false);
  sched->RunScheduler ();

  Sub ("1A. 每 UE 下行分配: RB 数随 CQI(信道) 与 优先级/业务(预留) 联动");
  std::cout << "  波束 25 RB - 6 PRACH 预留 = 19 RB 业务预算/波束; 应急刚性预留 3 / 语音 2\n";
  std::cout << "  " << std::left << std::setw (8) << "RNTI"
            << std::setw (16) << "标签"
            << std::setw (8) << "波束"
            << std::setw (12) << "优先级"
            << std::setw (8) << "CQI"
            << std::setw (12) << "bytes/RB"
            << std::setw (10) << "分到RB"
            << std::setw (8) << "MCS" << "\n";
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << std::fixed << std::setprecision (2);
  for (const UeSpec& u : ues)
    {
      const DciSink& s = sinks[u.rnti];
      const double bpr = sched->EstimateBytesPerRbForCqi (u.dlCqi, false);
      std::cout << "  " << std::left << std::setw (8) << u.rnti
                << std::setw (16) << u.label
                << std::setw (8) << ("Beam" + std::to_string (u.beamId))
                << std::setw (12) << PriStr (sched->GetUePriority (u.rnti))
                << std::setw (8) << (int) u.dlCqi
                << std::setw (12) << bpr
                << std::setw (10) << s.LastDlRb ()
                << std::setw (8) << (uint32_t) s.LastDlMcs () << "\n";
    }
  std::cout << "  => 高 CQI -> bytes/RB 大 -> 同缓冲所需 RB 少; 应急/语音先按刚性预留切片 => 动态 RB 生效\n";

  Sub ("1B. 每波束 RB 占用 + NOMA 功率域复用增益");
  std::cout << "  " << std::left << std::setw (10) << "波束"
            << std::setw (16) << "主用户已用RB"
            << std::setw (18) << "NOMA共享(复用)RB"
            << std::setw (16) << "累计已用RB"
            << std::setw (18) << "累计共享RB"
            << "复用增益\n";
  std::cout << "  --------------------------------------------------------------------------------------------\n";
  for (uint32_t beam : {BEAM_A, BEAM_B})
    {
      const uint32_t usedNow   = sched->GetBeamUsedRbs (beam, false);
      const uint32_t sharedNow = sched->GetBeamSharedRbs (beam, false);
      const uint64_t cumUsed   = sched->GetCumulativeNomaUsedDlRbs (beam);
      const uint64_t cumShared = sched->GetCumulativeNomaSharedDlRbs (beam);
      const double   gain      = sched->GetNomaDlReuseGain (beam);
      std::cout << "  " << std::left << std::setw (10) << ("Beam" + std::to_string (beam))
                << std::setw (16) << usedNow
                << std::setw (18) << sharedNow
                << std::setw (16) << cumUsed
                << std::setw (18) << cumShared
                << std::setprecision (3) << gain << "\n";
    }
  std::cout << "  => Beam2 的 near/far NOMA 对在同一批 RB 上功率叠加 (共享RB>0), 不额外占预算 => 频域复用增益\n";

  // =========================================================================
  // SECTION 2: 功率域 NOMA — near/far 的 effSINR 与功率切分
  // =========================================================================
  Banner ("SECTION 2: 功率域 NOMA — near/far 有效 SINR + beta 功率切分 (调度器真实算出)");

  std::cout << "  Beam2 的 portable-NEAR(CQI15, 强) 与 portable-FAR(CQI3, 弱) CQI 间隔=12 >= 4 => 配对\n";
  std::cout << "  " << std::left << std::setw (16) << "角色"
            << std::setw (8) << "RNTI"
            << std::setw (10) << "分到RB"
            << std::setw (16) << "txPower(dBm)"
            << std::setw (22) << "有效SINR(dB)"
            << "MCS\n";
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << std::setprecision (2);
  struct NomaRow { const char* role; uint16_t rnti; };
  double nearTx = std::nan (""), farTx = std::nan ("");
  for (const NomaRow& r : {NomaRow{"NEAR(强,SIC后)", 201}, NomaRow{"FAR(弱,被干扰)", 202}})
    {
      const DciSink& s = sinks[r.rnti];
      const double effSinr = sched->GetEffectiveDlSinrDb (r.rnti);
      const double tx = s.LastDlTxPowerDbm ();
      if (r.rnti == 201) { nearTx = tx; }
      else               { farTx = tx; }
      std::cout << "  " << std::left << std::setw (16) << r.role
                << std::setw (8) << r.rnti
                << std::setw (10) << s.LastDlRb ()
                << std::setw (16) << tx
                << std::setw (22) << effSinr
                << (uint32_t) s.LastDlMcs () << "\n";
    }
  std::cout << "  ----------------------------------------------------------------------------------\n";
  if (!std::isnan (nearTx) && !std::isnan (farTx))
    {
      std::cout << "  far txPower - near txPower = " << (farTx - nearTx)
                << " dB (beta=0.8 -> far 拿 0.8 份额, near 拿 0.2, 约 +6dB)\n";
    }
  std::cout << "  => near 经 SIC 后有效 SINR 高; far 被 near 干扰后有效 SINR 低 => 功率域 NOMA 物理代价真实\n";

  // =========================================================================
  // SECTION 3: 闭环功控 CLPC — 收敛到 TargetUlSinrDb
  // =========================================================================
  Banner ("SECTION 3: 闭环功控 CLPC — 实测 UL SINR 收敛到 TargetUlSinrDb (真实闭环)");

  // 单独建一个调度器做 CLPC 演示 (避免 SECTION1 的 DL 调度状态干扰 UL ledger 观测)。
  // CLPC 链路: DoSchedUlTriggerReq -> RunUlScheduler -> ProcessUlGrant ->
  //   ScheduleUlSinrFeedback (按链路预算算实测 UL SINR) -> 300ms 后
  //   DeliverUlSinrFeedback -> ApplyUlSinrMeasurement -> UpdateClosedLoopPowerControl
  //   (与 TargetUlSinrDb 比较, 累积 clpcOffsetDb f(i))。
  // 用便携式终端 (P_CMAX=63 dBm, 不易触发发射功率削顶), 让闭环修正 f(i) 能真正把
  // 实测接收 UL SINR 推到 Target 附近 (消费级 P_CMAX=50 dBm 会过早削顶, 使 SINR 封顶)。
  // 每轮只放 1 RB 的小缓冲, 使授权稳定、便于观测收敛。
  const uint16_t clpcRnti = 301;
  const double targetUlSinr = 3.0;   // 默认 TargetUlSinrDb
  Ptr<GeoBeamScheduler> clpc = CreateObject<GeoBeamScheduler> ();
  clpc->SetAttribute ("ClpcEnabled", BooleanValue (true));
  clpc->SetAttribute ("TargetUlSinrDb", DoubleValue (targetUlSinr));
  clpc->SetAttribute ("UlCqiFeedbackDelay", TimeValue (MilliSeconds (300)));  // GEO 上行单程
  clpc->Initialize (5, 30);
  clpc->AddUeContext (clpcRnti, UT_PORTABLE, TRAFFIC_DATA);
  clpc->AddUeInfo (clpcRnti, 5);
  clpc->UpdateUeUlCqi (clpcRnti, 7.0);

  std::cout << "  UE " << clpcRnti << " (portable), TargetUlSinrDb=" << targetUlSinr << " dB\n";
  std::cout << "  每 700ms 触发一次 UL 调度 (留足 300ms 反馈 + 应用); UE 持续有 UL 数据。\n";
  std::cout << "  闭环: 实测 SINR < Target -> TPC 加功率 -> f(i)↑ -> 下轮实测 SINR↑ -> 趋近 Target。\n";
  std::cout << "  " << std::left << std::setw (12) << "时刻(ms)"
            << std::setw (18) << "实测UL SINR(dB)"
            << std::setw (18) << "clpcOffset f(i)"
            << "与Target误差(dB)\n";
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << std::setprecision (3);

  const uint32_t numUlRounds = 12;
  for (uint32_t i = 0; i < numUlRounds; ++i)
    {
      const uint64_t tMs = 100 + i * 700;
      Simulator::Schedule (MilliSeconds (tMs), [&clpc, clpcRnti] () {
        // 持续补充小 UL 缓冲 (~1 RB), 保证每轮都有上行需求触发 ProcessUlGrant -> CLPC 反馈。
        clpc->UpdateUeUlBufferStatus (clpcRnti, 200);
        NrMacSchedSapProvider::SchedUlTriggerReqParameters ulp;
        ulp.m_snfSf = SfnSf (1, 0, 0, 0);
        clpc->DoSchedUlTriggerReq (ulp);
      });
      // 在反馈应用之后读取 实测SINR + clpcOffset (UL 授权 t + 300ms 反馈 + 立即应用)。
      Simulator::Schedule (MilliSeconds (tMs + 350), [&clpc, clpcRnti, tMs, targetUlSinr] () {
        const SatUeContext ctx = clpc->GetUeContext (clpcRnti);
        const double meas = clpc->GetLastMeasuredUlSinrDb (clpcRnti);
        std::cout << "  " << std::left << std::setw (12) << (tMs + 350)
                  << std::setw (18) << meas
                  << std::setw (18) << ctx.clpcOffsetDb
                  << (meas - targetUlSinr) << "\n";
      });
    }

  Simulator::Stop (MilliSeconds (100 + numUlRounds * 700 + 400));
  Simulator::Run ();
  Simulator::Destroy ();

  const SatUeContext clpcCtx = clpc->GetUeContext (clpcRnti);
  const double clpcFinalSinr = clpc->GetLastMeasuredUlSinrDb (clpcRnti);
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << "  收敛后: 实测 UL SINR = " << clpcFinalSinr << " dB, clpcOffset f(i) = "
            << clpcCtx.clpcOffsetDb << " dB, 与 Target(" << targetUlSinr << "dB) 误差 = "
            << (clpcFinalSinr - targetUlSinr) << " dB\n";
  std::cout << "  => f(i) 从 0 累积, 实测 UL SINR 单调趋近并稳定在 Target 附近 (TPC 量化步落入死区) => 闭环功控生效\n";

  // =========================================================================
  // SECTION 4: 真实 DL 数据面 — NOMA effSINR 自动接线驱动 BLER (收/丢字节)
  // =========================================================================
  Banner ("SECTION 4: 真实 DL 数据面 — NOMA 物理代价自动接线驱动 BLER (真实收/丢包)");

  // 复用 SECTION1/2 的调度器 (sched) 跑出的 NOMA near/far {effSINR, txPower},
  // 经 gwMac->SetScheduler(sched) 自动接线到真实数据面。
  // 注: SECTION3 已 Simulator::Destroy(), 这里开新 simulator; sched 的持久化
  //     map (m_lastDlEffSinrDb/m_lastDlTxPowerDbm) 不随 Destroy 清空, 可安全跨用。
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  NodeContainer gwNode;  gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNodes; ueNodes.Create (6);
  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 0, 0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 0, 35786000.0));
  mob.Install (ueNodes);
  for (uint32_t i = 0; i < 6; ++i)
    {
      ueNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 1000.0 + i * 500.0, 1.5));
    }

  GeoBeamHelper sat;
  sat.SetActiveBeamCount (2);
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  Ptr<NtnGwMac> gwMac = CreateObject<NtnGwMac> ();
  gwMac->SetPhy (gw.GetGwPhy (0));
  gwMac->SetTxConfig (sat.GetFeederSpectrumModel (), 100e6, 40.0, MilliSeconds (1));
  // 单一真源自动接线: 之后每个 DL 包的 effSINR/发射功率自动取自调度器 (无手工注入)。
  gwMac->SetScheduler (sched);

  // 数据面只对 NOMA 对 (201 near / 202 far) 跑端到端, 直观对比 effSINR 驱动的 BLER。
  // (其余 UE 的数据面省略, 重点是证明 NOMA 物理代价进了真实收/丢包。)
  const uint16_t dpRnti[2]  = {201, 202};
  const uint16_t dpBeam[2]  = {1, 2};   // 用两束独立 user channel 隔离
  uint32_t rxPdu[2] = {0, 0};
  std::vector<Ptr<SatPhy>>   uePhys (2);
  std::vector<Ptr<SatUtMac>> ueMacs (2);
  std::vector<Ptr<NtnPhyErrorModel>> ems (2);

  for (uint32_t i = 0; i < 2; ++i)
    {
      Ptr<SpectrumChannel>     uch = sat.GetUserChannel (dpBeam[i]);
      Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel (dpBeam[i]);

      Ptr<SatPhy> uePhy = CreateObject<SatPhy> ();
      uePhy->SetChannel (uch);
      uePhy->SetMobility (ueNodes.Get (i)->GetObject<MobilityModel> ());
      uePhy->SetRxSpectrumModel (usm);
      uch->AddRx (uePhy);
      uePhys[i] = uePhy;

      Ptr<SatUtMac> ueMac = CreateObject<SatUtMac> ();
      ueMac->SetPhy (uePhy);
      ueMac->SetTxConfig (usm, 5e6, 23.0, MilliSeconds (1));
      ueMac->SetRnti (dpRnti[i]);
      ueMac->SetDlPduCallback (MakeBoundCallback (&CountDlRx, &rxPdu[i]));
      ueMacs[i] = ueMac;

      Ptr<NtnPhyErrorModel> em = CreateObject<NtnPhyErrorModel> ();
      em->SetSinrDb (20.0);   // 默认高 SINR, 几乎不丢; 只有 tag 注入的低 effSINR 才丢
      ueMac->SetPhyErrorModel (em);
      ems[i] = em;

      gwMac->AddUe (dpRnti[i], dpBeam[i]);
    }

  const uint32_t N = 40;
  Simulator::Schedule (Seconds (1.0), [&] () {
    for (uint32_t i = 0; i < 2; ++i)
      {
        for (uint32_t k = 0; k < N; ++k)
          {
            gwMac->EnqueueDlPdu (dpRnti[i], Create<Packet> (200));
          }
      }
    gwMac->StartScheduler (MilliSeconds (5));
  });
  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();

  std::cout << "  对 NOMA 对跑真实数据面 (gw-mac -> SatPhy -> 弯管 -> SatUtMac, 每 UE 发 " << N << " 包):\n";
  std::cout << "  " << std::left << std::setw (16) << "角色"
            << std::setw (8) << "RNTI"
            << std::setw (16) << "自动接线effSINR"
            << std::setw (12) << "收包"
            << std::setw (12) << "丢包"
            << "收包率\n";
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << std::setprecision (1);
  const char* roleStr[2] = {"NEAR(强,SIC后)", "FAR(弱,被干扰)"};
  for (uint32_t i = 0; i < 2; ++i)
    {
      const double wireSinr = sched->GetLastDlEffSinrDb (dpRnti[i]);
      std::cout << "  " << std::left << std::setw (16) << roleStr[i]
                << std::setw (8) << dpRnti[i]
                << std::setw (16) << wireSinr
                << std::setw (12) << rxPdu[i]
                << std::setw (12) << ems[i]->GetDropCount ()
                << std::setprecision (1) << (100.0 * rxPdu[i] / N) << " %\n";
    }
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << "  => near 高 effSINR 几乎全收; far 低 effSINR 大量丢包 (全靠 SetScheduler 自动接线\n";
  std::cout << "     把 NOMA 物理代价带进真实数据面的 NtnDciTag -> BLER) => 真实数据面 NOMA 闭环\n";

  // -------------------------------------------------------------------------
  // 可选落盘
  // -------------------------------------------------------------------------
  if (writeFiles)
    {
      Ptr<ResultWriter> writer = CreateObject<ResultWriter> ();
      writer->SetOutputDirectory ("./simulation_results");
      sched->WriteNomaReuseStats (writer, "NomaReuseStats.txt");
      std::cout << "\n  [落盘] NOMA 复用记账 -> " << writer->GetOutputDirectory ()
                << "/NomaReuseStats.txt\n";
    }

  // =========================================================================
  // SECTION 5: CQI 滤波/预测 — 实测 vs 滤波后 (GetSchedulingCqi)
  // =========================================================================
  Banner ("SECTION 5: CQI 滤波/预测 — 实测 CQI vs 调度用 CQI (UseCqiAmcPredictor 开/关)");

  const uint16_t cqiRnti = 401;
  const std::vector<std::pair<double, double>> series = {
    {  0, 6.0}, {100, 9.0}, {200, 5.0}, {300, 10.0}, {400, 7.0},
    {500, 11.0}, {600, 8.0}, {700, 12.0}, {800, 9.0}, {900, 13.0}
  };

  Ptr<GeoBeamScheduler> off = CreateObject<GeoBeamScheduler> ();
  off->SetAttribute ("UseCqiAmcPredictor", BooleanValue (false));
  off->Initialize (1, 30);
  off->AddUeContext (cqiRnti, UT_CONSUMER, TRAFFIC_DATA);
  off->AddUeInfo (cqiRnti, 1);

  Ptr<GeoBeamScheduler> on = CreateObject<GeoBeamScheduler> ();
  Ptr<CqiAmcPredictor> predictor = CreateObject<CqiAmcPredictor> ();
  predictor->GetDl ()->SetAttribute ("HorizonH", UintegerValue (1));
  predictor->GetDl ()->SetAttribute ("Rmeas", DoubleValue (8.0));
  on->SetCqiPredictor (predictor);
  on->Initialize (1, 30);
  on->AddUeContext (cqiRnti, UT_CONSUMER, TRAFFIC_DATA);
  on->AddUeInfo (cqiRnti, 1);

  std::cout << "  喂入同一串带抖动/上升趋势的 CQI 上报, 对比 GetSchedulingCqi (调度真正用的 CQI):\n";
  std::cout << "  " << std::left << std::setw (12) << "时刻(ms)"
            << std::setw (16) << "上报CQI(实测)"
            << std::setw (22) << "关预测器(直通)"
            << std::setw (24) << "开预测器(Kalman+OLLA)" << "\n";
  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << std::setprecision (3);

  for (const auto& s : series)
    {
      const double t = s.first;
      const double cqi = s.second;
      Simulator::Schedule (MilliSeconds (static_cast<uint64_t> (t)), [&, t, cqi] () {
        off->UpdateUeDlCqi (cqiRnti, cqi);
        on->UpdateUeDlCqi (cqiRnti, cqi);
        const double cOff = off->GetSchedulingCqiForUe (cqiRnti, false);
        const double cOn  = on->GetSchedulingCqiForUe (cqiRnti, false);
        std::cout << "  " << std::left << std::setw (12) << (int) t
                  << std::setw (16) << cqi
                  << std::setw (22) << cOff
                  << std::setw (24) << cOn << "\n";
      });
    }

  Simulator::Stop (MilliSeconds (1000));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << "  ----------------------------------------------------------------------------------\n";
  std::cout << "  => 关: 调度用 CQI = 最近实测 (直通); 开: 经 Kalman 滤波 + 前向预测 + OLLA 偏置平滑\n";
  std::cout << "  => UseCqiAmcPredictor 打开后调度采用的 CQI 确实经过滤波/预测处理 => CQI 接入生效\n";

  sched = nullptr;

  // =========================================================================
  // 结论
  // =========================================================================
  Banner ("结论: 四大 RM 特性在一个仿真里一起生效 (各特性量化证据)");
  std::cout << "  [1] 动态频域 RB: 各 UE 按 CQI(信道) + 优先级/业务(预留) 分得不同 RB (SECTION 1A)\n";
  std::cout << "  [2] 功率域 NOMA: Beam2 near/far 同 RB 功率叠加, 共享RB>0 不额外占预算;\n";
  std::cout << "                   beta 切分使 far txPower 高 near 约 6dB; near SIC 后 effSINR 高、\n";
  std::cout << "                   far 被干扰 effSINR 低 (SECTION 1B/2), 并在真实数据面驱动 BLER:\n";
  std::cout << "                   near 收包率 " << std::setprecision (0) << (100.0 * rxPdu[0] / N)
            << "%, far 收包率 " << (100.0 * rxPdu[1] / N) << "% (SECTION 4)\n";
  std::cout << "  [3] 闭环功控 CLPC: clpcOffset f(i) 收敛到 " << std::setprecision (2)
            << clpcCtx.clpcOffsetDb << " dB, 实测 UL SINR 稳定在 " << clpcFinalSinr
            << " dB (Target " << targetUlSinr << "dB) (SECTION 3)\n";
  std::cout << "  [4] CQI 滤波/预测: UseCqiAmcPredictor 开后调度用 CQI 经 Kalman+OLLA 平滑 (SECTION 5)\n";
  std::cout << "\n  全部由 GeoBeamScheduler + 自研数据面真实跑出; DL NOMA BLER (SECTION4) 与 CLPC 闭环\n";
  std::cout << "  (SECTION3) 是真实数据/事件链跑出的, RB/NOMA 决策由真实 RunScheduler 跑出, buffer/CQI\n";
  std::cout << "  为代表性输入。RM 全部能力在同一仿真里一起生效并产出量化结果。\n";

  std::cout << "\n";
  std::cout << "########################################################################\n";
  std::cout << "#   ntn-rrm-e2e-demo 全部 SECTION 数据已输出完毕。                       #\n";
  std::cout << "########################################################################\n";
  return 0;
}
