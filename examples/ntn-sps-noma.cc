/*
 * 文件: contrib/geo-sat/examples/ntn-sps-noma.cc
 * 功能: 阶段3 半静态 NOMA(频率域 SPS + 功率域 NOMA 合体)A/B 验证。
 *
 * 验证: 在 SPS near(语音, 强信道, 拿半静态 grant)的 RBG 上, 半持续地叠加一个 far
 *       (数据, 弱信道, 纯动态)——配对一次写入 grant, 跨周期复用(非逐轮重配),
 *       far 经功率域 NOMA 复用 near 的频率资源, 不额外消耗 RB 预算。
 *
 * A/B: SpsNomaEnabled = false vs true。
 *   - 关: 无 NOMA-SPS 配对, 无共享 RB; near 拿满功率(有效 SINR 高)。
 *   - 开: 出现配对发射(nomaSpsPairs>0)+ 累计共享 RB(>0, far 免预算叠加);
 *         near 让出 β 功率 → 有效 SINR 下降(SIC 代价), far 获得服务。
 *
 * 配置: 单波束; 2 个 SPS 语音 near(CQI 13), 3 个动态数据 far(CQI 4, 始终有数据)。
 * 运行: ./ns3 run ntn-sps-noma
 */
#include "ns3/core-module.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"

#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSpsNoma");

namespace {

constexpr uint32_t BEAM_ID = 1;
constexpr double   TICK_MS = 5.0;
constexpr double   STOP_MS = 300.0;

const std::vector<uint16_t> NEAR_UES = {11, 12};            // SPS 语音 near
const std::vector<uint16_t> FAR_UES  = {21, 22, 23};        // 动态数据 far

Ptr<GeoBeamScheduler> g_sched;

void
Tick ()
{
  const double tms = Simulator::Now ().GetMilliSeconds ();
  for (uint16_t r : NEAR_UES) { g_sched->UpdateUeDlBufferStatus (r, 120u); }   // 语音持续有小包
  for (uint16_t r : FAR_UES)  { g_sched->UpdateUeDlBufferStatus (r, 5000u); }  // 数据持续有数据
  g_sched->RunScheduler ();
  if (tms + TICK_MS <= STOP_MS)
    {
      Simulator::Schedule (MilliSeconds (TICK_MS), &Tick);
    }
}

struct NomaResult {
  uint64_t nomaSpsPairs {0};
  uint64_t cumShared {0};
  double   nearEffSinr {0.0};
};

NomaResult
RunScenario (bool nomaOn)
{
  Config::SetDefault ("ns3::ResourceManager::DlSpsRegionRbs", UintegerValue (4)); // 2 near × 2 RB

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->SetAttribute ("SpsMode", UintegerValue (2));                 // Configured
  sched->SetAttribute ("SpsGrantPeriod", TimeValue (MilliSeconds (20)));
  sched->SetAttribute ("SpsVoiceRbs", UintegerValue (2));
  sched->SetAttribute ("SpsNomaEnabled", BooleanValue (nomaOn));      // ← A/B 开关
  sched->SetAttribute ("NomaFarPowerFraction", DoubleValue (0.8));    // β
  sched->SetAttribute ("NomaMinCqiGap", DoubleValue (4.0));
  sched->SetAttribute ("NomaRepairPeriod", UintegerValue (4));
  sched->SetAttribute ("ClpcEnabled", BooleanValue (false));
  sched->Initialize (BEAM_ID, 15);

  for (uint16_t r : NEAR_UES)
    {
      sched->AddUeContext (r, UT_CONSUMER, TRAFFIC_VOICE);  // SPS 合格(语音)
      sched->AddUeInfo (r, BEAM_ID);
      sched->UpdateUeDlCqi (r, 13.0);                       // 强信道 near
    }
  for (uint16_t r : FAR_UES)
    {
      sched->AddUeContext (r, UT_CONSUMER, TRAFFIC_DATA);   // 纯动态(非SPS)
      sched->AddUeInfo (r, BEAM_ID);
      sched->UpdateUeDlCqi (r, 4.0);                        // 弱信道 far
    }

  g_sched = sched;
  Simulator::Schedule (MilliSeconds (0), &Tick);
  Simulator::Stop (MilliSeconds (STOP_MS + 1));
  Simulator::Run ();

  NomaResult res;
  res.nomaSpsPairs = sched->GetSpsStats ().nomaSpsPairs;
  res.cumShared    = sched->GetCumulativeNomaSharedDlRbs (BEAM_ID);
  res.nearEffSinr  = sched->GetLastDlEffSinrDb (NEAR_UES.front ()); // near 末次有效 SINR
  Simulator::Destroy ();
  g_sched = nullptr;
  return res;
}

} // namespace

int
main (int argc, char* argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  std::cout << "==== 阶段3 半静态 NOMA A/B (2 SPS语音near@CQI13 + 3 动态数据far@CQI4) ====" << std::endl;

  NomaResult off = RunScenario (false);
  NomaResult on  = RunScenario (true);

  std::cout << std::fixed << std::setprecision (2);
  std::cout << "                       NOMA-SPS配对   累计共享RB   near末次有效SINR(dB)" << std::endl;
  std::cout << "SpsNomaEnabled=false : "
            << std::setw (10) << off.nomaSpsPairs
            << std::setw (13) << off.cumShared
            << std::setw (16) << off.nearEffSinr << std::endl;
  std::cout << "SpsNomaEnabled=true  : "
            << std::setw (10) << on.nomaSpsPairs
            << std::setw (13) << on.cumShared
            << std::setw (16) << on.nearEffSinr << std::endl;
  std::cout << "------------------------------------------------------------------" << std::endl;

  int pass = 0, fail = 0;
  auto check = [&] (const std::string& name, bool ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
    if (ok) ++pass; else ++fail;
  };

  check ("关: 无半静态 NOMA 配对",            off.nomaSpsPairs == 0);
  check ("关: 无共享 RB",                      off.cumShared == 0);
  check ("开: 出现半静态 NOMA 配对发射 (>0)",  on.nomaSpsPairs > 0);
  check ("开: 累计共享 RB > 0 (far 免预算叠加在 near 的 RBG 上)", on.cumShared > 0);
  check ("开: near 让出 β 功率 → 有效 SINR 低于关时 (SIC 代价)", on.nearEffSinr < off.nearEffSinr);

  std::cout << "------------------------------------------------------------------" << std::endl;
  std::cout << "PASS=" << pass << " FAIL=" << fail << std::endl;
  std::cout << "OVERALL RESULT: " << (fail == 0 ? "PASS" : "FAIL") << std::endl;
  return fail == 0 ? 0 : 1;
}
