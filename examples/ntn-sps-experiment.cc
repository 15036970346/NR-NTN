/*
 * 文件: contrib/geo-sat/examples/ntn-sps-experiment.cc
 * 功能: 阶段5 SPS 实验矩阵 / 消融出图 —— 一次跑出论文用的对比数据并写 CSV。
 *
 * 在同一业务场景下扫描各 SPS 配置, 汇总三大维度的关键指标到一张表 + 一个 CSV:
 *   - 信令域: 真省 DCI 节省率 (configured grant)
 *   - 频率域: overbooking 账面容量 / 接入路数 / 吞吐 / 冲突率
 *   - 功率域: 半静态 NOMA 配对数 / 累计共享 RB
 *
 * 行(场景):
 *   1 Baseline(无DCI节省)  : SPS 每周期重配(thr=0) —— 代表动态式逐周期发 DCI 的信令基线
 *   2 SPS(configured)       : thr=2 —— 复用 DCI-free, 体现真省 DCI
 *   3 SPS+overbook1.5       : 账面×1.5, 多接入
 *   4 SPS+overbook2.0       : 账面×2.0
 *   5 SPS+NOMA              : near 上半静态叠加 far(功率域)
 *
 * 输出: 控制台表 + sps_experiment.csv(运行目录)。运行: ./ns3 run ntn-sps-experiment
 */
#include "ns3/core-module.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"

#include <fstream>
#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSpsExperiment");

namespace {

constexpr uint32_t BEAM_ID = 1;
constexpr uint32_t REGION  = 8;
constexpr uint32_t VOICE_RBS = 2;
constexpr double   TICK_MS = 5.0;
constexpr double   STOP_MS = 500.0;
constexpr uint32_t VOICE_DUTY = 40;   // 语音占空比 %
constexpr uint32_t FAR_DUTY   = 100;  // 数据 far 持续有数据

struct ExpConfig {
  std::string label;
  uint32_t numVoice;
  uint32_t numFar;
  double   overbook;
  bool     noma;
  uint8_t  reconfigThr;
};

struct ExpResult {
  uint32_t admitCap {0};
  uint32_t peakActive {0};
  double   servedKB {0.0};
  uint64_t dciSent {0};
  double   dciSavingsPct {0.0};
  double   conflictRatePct {0.0};
  uint64_t nomaPairs {0};
  uint64_t cumSharedRb {0};
};

// 全局(Tick 用)
Ptr<GeoBeamScheduler> g_sched;
ExpResult* g_res = nullptr;
uint32_t g_numVoice = 0, g_numFar = 0;

bool
Talking (uint32_t idx, double tms, uint32_t dutyPct)
{
  if (dutyPct >= 100) return true;
  const uint32_t p = static_cast<uint32_t> (tms / 20.0);
  return ((p * 7u + idx * 13u) % 100u) < dutyPct;
}

void
Tick ()
{
  const double tms = Simulator::Now ().GetMilliSeconds ();
  for (uint32_t i = 0; i < g_numVoice; ++i)
    {
      g_sched->UpdateUeDlBufferStatus (100 + i, Talking (i, tms, VOICE_DUTY) ? 120u : 0u);
    }
  for (uint32_t j = 0; j < g_numFar; ++j)
    {
      g_sched->UpdateUeDlBufferStatus (200 + j, Talking (100 + j, tms, FAR_DUTY) ? 5000u : 0u);
    }
  g_sched->RunScheduler ();
  g_res->peakActive = std::max (g_res->peakActive, g_sched->GetDlSpsGrantCount ());
  if (tms + TICK_MS <= STOP_MS)
    {
      Simulator::Schedule (MilliSeconds (TICK_MS), &Tick);
    }
}

ExpResult
RunExp (const ExpConfig& c)
{
  ExpResult r;
  Config::SetDefault ("ns3::ResourceManager::DlSpsRegionRbs", UintegerValue (REGION));
  Config::SetDefault ("ns3::ResourceManager::DlSpsOverbookFactor", DoubleValue (c.overbook));

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->SetAttribute ("SpsMode", UintegerValue (2));
  sched->SetAttribute ("SpsGrantPeriod", TimeValue (MilliSeconds (20)));
  sched->SetAttribute ("SpsVoiceRbs", UintegerValue (VOICE_RBS));
  sched->SetAttribute ("SpsMcsReconfigThreshold", UintegerValue (c.reconfigThr));
  sched->SetAttribute ("SpsNomaEnabled", BooleanValue (c.noma));
  sched->SetAttribute ("NomaFarPowerFraction", DoubleValue (0.8));
  sched->SetAttribute ("NomaMinCqiGap", DoubleValue (4.0));
  sched->SetAttribute ("ClpcEnabled", BooleanValue (false));
  sched->Initialize (BEAM_ID, 15);
  for (uint32_t i = 0; i < c.numVoice; ++i)
    {
      sched->AddUeContext (100 + i, UT_CONSUMER, TRAFFIC_VOICE);
      sched->AddUeInfo (100 + i, BEAM_ID);
      sched->UpdateUeDlCqi (100 + i, 13.0);  // near 强信道
    }
  for (uint32_t j = 0; j < c.numFar; ++j)
    {
      sched->AddUeContext (200 + j, UT_CONSUMER, TRAFFIC_DATA);
      sched->AddUeInfo (200 + j, BEAM_ID);
      sched->UpdateUeDlCqi (200 + j, 4.0);   // far 弱信道
    }

  g_sched = sched; g_res = &r; g_numVoice = c.numVoice; g_numFar = c.numFar;
  Simulator::Schedule (MilliSeconds (0), &Tick);
  Simulator::Stop (MilliSeconds (STOP_MS + 1));
  Simulator::Run ();

  const SatSpsStats& s = sched->GetSpsStats ();
  r.admitCap = static_cast<uint32_t> (REGION * c.overbook);
  r.servedKB = s.dlSpsServedBytes / 1024.0;
  r.dciSent  = s.dlSpsDciSent;
  const uint64_t tx = s.dlActivations + s.dlReuse;
  r.dciSavingsPct = tx ? (100.0 * s.dlSpsReuseNoDci / tx) : 0.0;
  const uint64_t att = s.dlReuse + s.dlConflictEvents;
  r.conflictRatePct = att ? (100.0 * s.dlConflictEvents / att) : 0.0;
  r.nomaPairs = s.nomaSpsPairs;
  r.cumSharedRb = sched->GetCumulativeNomaSharedDlRbs (BEAM_ID);
  Simulator::Destroy ();
  g_sched = nullptr; g_res = nullptr;
  return r;
}

} // namespace

int
main (int argc, char* argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  std::vector<ExpConfig> matrix = {
    {"Baseline(no-DCI-save)", 8,  0, 1.0, false, 0},
    {"SPS(configured)",       8,  0, 1.0, false, 2},
    {"SPS+overbook1.5",       12, 0, 1.5, false, 2},
    {"SPS+overbook2.0",       16, 0, 2.0, false, 2},
    {"SPS+NOMA",              4,  4, 1.0, true,  2},
  };

  std::cout << "==== 阶段5 SPS 实验矩阵 (物理区=8RBG, 语音2RBG/路 duty40%, ~25周期) ====" << std::endl;
  std::cout << std::left
            << std::setw (22) << "场景"
            << std::setw (9)  << "账面cap"
            << std::setw (9)  << "接入路"
            << std::setw (10) << "吞吐KB"
            << std::setw (9)  << "DCI数"
            << std::setw (11) << "DCI省%"
            << std::setw (10) << "冲突%"
            << std::setw (9)  << "NOMA对"
            << std::setw (9)  << "共享RB" << std::endl;

  std::vector<ExpResult> results;
  std::ofstream csv ("sps_experiment.csv");
  csv << "scenario,numVoice,numFar,overbook,noma,reconfigThr,admitCap,peakActive,servedKB,dciSent,dciSavingsPct,conflictRatePct,nomaPairs,cumSharedRb\n";
  csv << std::fixed << std::setprecision (2);

  for (const auto& c : matrix)
    {
      ExpResult r = RunExp (c);
      results.push_back (r);
      std::cout << std::left << std::fixed << std::setprecision (1)
                << std::setw (22) << c.label
                << std::setw (9)  << r.admitCap
                << std::setw (9)  << r.peakActive
                << std::setw (10) << r.servedKB
                << std::setw (9)  << r.dciSent
                << std::setw (11) << r.dciSavingsPct
                << std::setw (10) << r.conflictRatePct
                << std::setw (9)  << r.nomaPairs
                << std::setw (9)  << r.cumSharedRb << std::endl;
      csv << c.label << ',' << c.numVoice << ',' << c.numFar << ',' << c.overbook << ','
          << (c.noma ? 1 : 0) << ',' << (uint32_t) c.reconfigThr << ','
          << r.admitCap << ',' << r.peakActive << ',' << r.servedKB << ',' << r.dciSent << ','
          << r.dciSavingsPct << ',' << r.conflictRatePct << ',' << r.nomaPairs << ',' << r.cumSharedRb << '\n';
    }
  csv.close ();
  std::cout << "CSV 已写出: sps_experiment.csv (运行目录)" << std::endl;
  std::cout << "------------------------------------------------------------------" << std::endl;

  const ExpResult& base = results[0];
  const ExpResult& sps  = results[1];
  const ExpResult& ob20 = results[3];
  const ExpResult& noma = results[4];

  int pass = 0, fail = 0;
  auto check = [&] (const std::string& name, bool ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
    if (ok) ++pass; else ++fail;
  };

  check ("信令域: SPS 显著省 DCI (相对 baseline +50%以上)", sps.dciSavingsPct > base.dciSavingsPct + 50.0);
  check ("频率域: overbook 提升账面容量 (2.0 > base)",       ob20.admitCap > base.admitCap);
  check ("频率域: overbook 提升接入路数 (2.0 >= base)",      ob20.peakActive >= base.peakActive);
  check ("频率域: overbook 提升吞吐 (2.0 >= base)",          ob20.servedKB >= base.servedKB);
  check ("功率域: NOMA 产生半静态配对 (>0)",                 noma.nomaPairs > 0);
  check ("功率域: NOMA 产生累计共享 RB (>0)",                noma.cumSharedRb > 0);

  std::cout << "------------------------------------------------------------------" << std::endl;
  std::cout << "PASS=" << pass << " FAIL=" << fail << std::endl;
  std::cout << "OVERALL RESULT: " << (fail == 0 ? "PASS" : "FAIL") << std::endl;
  return fail == 0 ? 0 : 1;
}
