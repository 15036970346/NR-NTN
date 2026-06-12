/*
 * 文件: contrib/geo-sat/examples/ntn-sps-dci.cc
 * 功能: 阶段2 真省 DCI(UE 侧 configured grant)验证。
 *
 * 验证: 半静态授权激活一次后, 周期复用【不再下发 DCI】(UE 用 configured grant 周期收发),
 *       只有 激活 / MCS 需重配 / 释放 才发 DCI。相对"每次传输都发 DCI"的动态基线,
 *       节省率 = 无DCI复用 / (激活 + 复用)。
 *
 * A/B(同一稳定信道, 3 语音 UE 持续有数据, ~20 个 20ms 周期):
 *   - SpsMcsReconfigThreshold=2(默认): MCS 稳定 → 复用基本 DCI-free → 高节省。
 *   - SpsMcsReconfigThreshold=0       : 每周期都重配 → 复用都发 DCI → 零节省(对照)。
 *
 * 运行: ./ns3 run ntn-sps-dci
 */
#include "ns3/core-module.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"

#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSpsDci");

namespace {

constexpr uint32_t BEAM_ID = 1;
constexpr double   TICK_MS = 5.0;
constexpr double   STOP_MS = 400.0;
const std::vector<uint16_t> UES = {11, 12, 13};

Ptr<GeoBeamScheduler> g_sched;

void
Tick ()
{
  const double tms = Simulator::Now ().GetMilliSeconds ();
  for (uint16_t r : UES)
    {
      g_sched->UpdateUeDlCqi (r, 9.0);          // 持续喂同一 CQI(稳定信道, 保持新鲜)
      g_sched->UpdateUeUlCqi (r, 9.0);
      g_sched->UpdateUeDlBufferStatus (r, 120u);
      g_sched->UpdateUeUlBufferStatus (r, 5000u);
    }
  g_sched->RunScheduler ();
  g_sched->RunUlSchedulingRound ();
  if (tms + TICK_MS <= STOP_MS)
    {
      Simulator::Schedule (MilliSeconds (TICK_MS), &Tick);
    }
}

struct DciResult {
  uint64_t dlAct {0}, dlReuse {0}, dlDciSent {0}, dlNoDci {0};
  uint64_t ulAct {0}, ulReuse {0}, ulDciSent {0}, ulNoDci {0};
  double dlSavingsPct {0.0}, ulSavingsPct {0.0};
};

DciResult
RunDci (uint8_t reconfigThreshold)
{
  Config::SetDefault ("ns3::ResourceManager::DlSpsRegionRbs", UintegerValue (8));
  Config::SetDefault ("ns3::ResourceManager::UlSpsRegionRbs", UintegerValue (8));

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->SetAttribute ("SpsMode", UintegerValue (2));
  sched->SetAttribute ("SpsGrantPeriod", TimeValue (MilliSeconds (20)));
  sched->SetAttribute ("SpsVoiceRbs", UintegerValue (2));
  sched->SetAttribute ("SpsMcsReconfigThreshold", UintegerValue (reconfigThreshold));
  sched->SetAttribute ("ClpcEnabled", BooleanValue (false));
  sched->Initialize (BEAM_ID, 15);
  for (uint16_t r : UES)
    {
      sched->AddUeContext (r, UT_CONSUMER, TRAFFIC_VOICE);
      sched->AddUeInfo (r, BEAM_ID);
      sched->UpdateUeDlCqi (r, 9.0);
      sched->UpdateUeUlCqi (r, 9.0);
    }

  g_sched = sched;
  Simulator::Schedule (MilliSeconds (0), &Tick);
  Simulator::Stop (MilliSeconds (STOP_MS + 1));
  Simulator::Run ();

  const SatSpsStats& s = sched->GetSpsStats ();
  DciResult r;
  r.dlAct = s.dlActivations; r.dlReuse = s.dlReuse; r.dlDciSent = s.dlSpsDciSent; r.dlNoDci = s.dlSpsReuseNoDci;
  r.ulAct = s.ulActivations; r.ulReuse = s.ulReuse; r.ulDciSent = s.ulSpsDciSent; r.ulNoDci = s.ulSpsReuseNoDci;
  const uint64_t dlTx = s.dlActivations + s.dlReuse;
  const uint64_t ulTx = s.ulActivations + s.ulReuse;
  r.dlSavingsPct = dlTx ? (100.0 * s.dlSpsReuseNoDci / dlTx) : 0.0;
  r.ulSavingsPct = ulTx ? (100.0 * s.ulSpsReuseNoDci / ulTx) : 0.0;
  Simulator::Destroy ();
  g_sched = nullptr;
  return r;
}

void
PrintRow (const std::string& tag, const DciResult& r)
{
  std::cout << std::left << std::fixed << std::setprecision (1)
            << std::setw (26) << tag
            << "DL[act=" << r.dlAct << " reuse=" << r.dlReuse
            << " dci=" << r.dlDciSent << " noDci=" << r.dlNoDci
            << " 省=" << r.dlSavingsPct << "%]  "
            << "UL[act=" << r.ulAct << " reuse=" << r.ulReuse
            << " dci=" << r.ulDciSent << " noDci=" << r.ulNoDci
            << " 省=" << r.ulSavingsPct << "%]" << std::endl;
}

} // namespace

int
main (int argc, char* argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  std::cout << "==== 阶段2 真省 DCI 验证 (3 语音 UE, 稳定信道, ~20 周期) ====" << std::endl;
  std::cout << "节省率 = 无DCI复用 / (激活 + 复用)  [相对'每次传输都发DCI'的动态基线]" << std::endl;
  std::cout << "------------------------------------------------------------------" << std::endl;

  DciResult on  = RunDci (2);  // 默认: MCS 稳定 -> 复用 DCI-free
  DciResult off = RunDci (0);  // 对照: 每周期重配 -> 复用都发 DCI
  PrintRow ("阈值=2 (configured grant)", on);
  PrintRow ("阈值=0 (每周期重配/无节省)", off);

  std::cout << "------------------------------------------------------------------" << std::endl;
  int pass = 0, fail = 0;
  auto check = [&] (const std::string& name, bool ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
    if (ok) ++pass; else ++fail;
  };

  check ("DL: 复用确实有 DCI-free (节省真实发生)", on.dlNoDci > 0);
  check ("DL: 节省率高 (>70%)",                    on.dlSavingsPct > 70.0);
  check ("DL: 阈值=0 时零节省 (复用都发 DCI)",     off.dlNoDci == 0);
  check ("DL: 节省 阈值2 > 阈值0",                 on.dlSavingsPct > off.dlSavingsPct);
  check ("UL: 复用确实有 DCI-free",                on.ulNoDci > 0);
  check ("UL: 节省率高 (>70%)",                    on.ulSavingsPct > 70.0);

  std::cout << "------------------------------------------------------------------" << std::endl;
  std::cout << "PASS=" << pass << " FAIL=" << fail << std::endl;
  std::cout << "OVERALL RESULT: " << (fail == 0 ? "PASS" : "FAIL") << std::endl;
  return fail == 0 ? 0 : 1;
}
