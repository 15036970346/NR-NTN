/*
 * 文件: contrib/geo-sat/examples/ntn-sps-overbook.cc
 * 功能: 阶段4 SPS overbooking(频率域统计复用)消融实验。
 *
 * 验证 SPS 文档的核心论点:
 *   - 账面超订 (DlSpsOverbookFactor) 把可接入路数从 floor(region/峰值) 放大到
 *     floor(region×factor/峰值) —— factor 越大, 接入越多。
 *   - 占空比 < 1 时 (语音静默期), 多数被接入 UE 不同时活跃, 物理区够用 → 冲突率低
 *     (overbooking 的红利)。
 *   - 高负载/无错峰时, 同时活跃数超过物理区 → 认领失败=冲突 (overbooking 的代价);
 *     但冲突由"连续冲突即隐式释放"的安全阀兜底, 冲突率有界(不随 factor 无限上升),
 *     过度超订的代价更多体现为 churn(冲突释放)与准入波动, 而非冲突率失控。
 *
 * 配置: 单波束, 物理 SPS 区 = 8 RBG, 每路语音峰值 2 RBG (物理同时容纳 4 路)。
 *       18 个语音 UE 竞争; factor ∈ {1.0, 1.5, 2.0} → 账面容量 8/12/16 RBG → 接 4/6/8 路。
 *
 * 运行: ./ns3 run ntn-sps-overbook
 */
#include "ns3/core-module.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"

#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSpsOverbook");

namespace {

constexpr uint32_t BEAM_ID    = 1;
constexpr uint32_t NUM_UE     = 18;
constexpr uint32_t REGION_RBS = 8;    // 物理 SPS 区
constexpr uint32_t VOICE_RBS  = 2;    // 每路语音峰值 RBG (物理容纳 8/2=4 路)
constexpr double   TICK_MS    = 5.0;
constexpr double   STOP_MS    = 500.0;

struct AblationResult {
  uint32_t admitCap {0};      // 账面容量 (RBG)
  uint32_t peakActive {0};    // 运行期间峰值活跃 grant 数 (≈接入路数)
  uint64_t reuse {0};
  uint64_t conflictEvents {0};
  uint64_t admitRejects {0};
  uint64_t releaseEmpty {0};
  uint64_t releaseConflict {0}; // 因连续冲突被隐式释放的次数(超订不稳定度/churn)
  double   conflictRatePct {0.0}; // 冲突率 = 冲突 / (复用 + 冲突) ×100%
};

// 确定性占空比模型: UE i 在第 p 个周期是否讲话 (~dutyPct%, 各 UE/周期去相关)。
bool
IsTalking (uint32_t i, double tms, uint32_t dutyPct)
{
  if (dutyPct >= 100) return true;
  const uint32_t p = static_cast<uint32_t> (tms / 20.0);
  return ((p * 7u + i * 13u) % 100u) < dutyPct;
}

// 全局: 当前这次消融的参数 + 句柄 (Tick 用)。
uint32_t g_dutyPct = 40;
Ptr<GeoBeamScheduler> g_sched;
AblationResult* g_res = nullptr;

void
Tick ()
{
  const double tms = Simulator::Now ().GetMilliSeconds ();
  for (uint32_t i = 0; i < NUM_UE; ++i)
    {
      const uint16_t rnti = 100 + i;
      g_sched->UpdateUeDlBufferStatus (rnti, IsTalking (i, tms, g_dutyPct) ? 120u : 0u);
    }
  g_sched->RunScheduler ();
  g_res->peakActive = std::max (g_res->peakActive, g_sched->GetDlSpsGrantCount ());
  if (tms + TICK_MS <= STOP_MS)
    {
      Simulator::Schedule (MilliSeconds (TICK_MS), &Tick);
    }
}

AblationResult
RunOne (double overbookFactor, uint32_t dutyPct, bool stagger)
{
  AblationResult r;
  Config::SetDefault ("ns3::ResourceManager::DlSpsRegionRbs", UintegerValue (REGION_RBS));
  Config::SetDefault ("ns3::ResourceManager::DlSpsOverbookFactor", DoubleValue (overbookFactor));

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->SetAttribute ("SpsMode", UintegerValue (2));   // Configured
  sched->SetAttribute ("SpsGrantPeriod", TimeValue (MilliSeconds (20)));
  sched->SetAttribute ("SpsImplicitReleaseAfter", UintegerValue (2));
  sched->SetAttribute ("SpsStaggerEnabled", BooleanValue (stagger));
  sched->SetAttribute ("SpsVoiceRbs", UintegerValue (VOICE_RBS));
  sched->SetAttribute ("ClpcEnabled", BooleanValue (false));
  sched->Initialize (BEAM_ID, 15);
  for (uint32_t i = 0; i < NUM_UE; ++i)
    {
      const uint16_t rnti = 100 + i;
      sched->AddUeContext (rnti, UT_CONSUMER, TRAFFIC_VOICE);
      sched->AddUeInfo (rnti, BEAM_ID);
      sched->UpdateUeDlCqi (rnti, 9.0);
    }

  g_sched = sched;
  g_res = &r;
  g_dutyPct = dutyPct;
  Simulator::Schedule (MilliSeconds (0), &Tick);
  Simulator::Stop (MilliSeconds (STOP_MS + 1));
  Simulator::Run ();

  const SatSpsStats& s = sched->GetSpsStats ();
  r.admitCap       = sched->GetSpsAdmittedRbs (BEAM_ID, false); // 末态账面占用(参考)
  r.admitCap       = static_cast<uint32_t> (REGION_RBS * overbookFactor); // 账面容量
  r.reuse          = s.dlReuse;
  r.conflictEvents = s.dlConflictEvents;
  r.admitRejects   = s.dlAdmitRejects;
  r.releaseEmpty   = s.dlReleaseEmpty;
  r.releaseConflict = s.dlReleaseConflict;
  const uint64_t attempts = s.dlReuse + s.dlConflictEvents;
  r.conflictRatePct = (attempts > 0) ? (100.0 * s.dlConflictEvents / attempts) : 0.0;

  Simulator::Destroy ();
  g_sched = nullptr;
  g_res = nullptr;
  return r;
}

void
PrintHeader (const std::string& title)
{
  std::cout << "\n==== " << title << " ====" << std::endl;
  std::cout << std::left
            << std::setw (8)  << "factor"
            << std::setw (10) << "账面cap"
            << std::setw (12) << "峰值接入路"
            << std::setw (10) << "复用次数"
            << std::setw (10) << "冲突次数"
            << std::setw (10) << "冲突率%"
            << std::setw (10) << "冲突释放"
            << std::setw (10) << "准入拒绝"
            << std::endl;
}

void
PrintRow (double f, const AblationResult& r)
{
  std::cout << std::left << std::fixed << std::setprecision (1)
            << std::setw (8)  << f
            << std::setw (10) << r.admitCap
            << std::setw (12) << (r.peakActive)
            << std::setw (10) << r.reuse
            << std::setw (10) << r.conflictEvents
            << std::setw (10) << r.conflictRatePct
            << std::setw (10) << r.releaseConflict
            << std::setw (10) << r.admitRejects
            << std::endl;
}

} // namespace

int
main (int argc, char* argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  std::cout << "SPS overbooking 消融: 物理区=" << REGION_RBS << " RBG, 每路峰值="
            << VOICE_RBS << " RBG (物理容纳 " << (REGION_RBS / VOICE_RBS) << " 路), UE="
            << NUM_UE << std::endl;

  // 场景 A: 低占空比(40%) + 错峰ON —— 体现 overbooking 红利(多接入, 冲突低)。
  PrintHeader ("场景A 低占空比 duty=40% + 错峰ON (语音静默期统计复用)");
  AblationResult a10 = RunOne (1.0, 40, true);  PrintRow (1.0, a10);
  AblationResult a15 = RunOne (1.5, 40, true);  PrintRow (1.5, a15);
  AblationResult a20 = RunOne (2.0, 40, true);  PrintRow (2.0, a20);

  // 场景 B: 满负载(100%) + 错峰OFF —— 体现 overbooking 代价(超物理 → 冲突随 factor 升)。
  PrintHeader ("场景B 满负载 duty=100% + 错峰OFF (压力: 同时活跃超物理区)");
  AblationResult b10 = RunOne (1.0, 100, false);  PrintRow (1.0, b10);
  AblationResult b15 = RunOne (1.5, 100, false);  PrintRow (1.5, b15);
  AblationResult b20 = RunOne (2.0, 100, false);  PrintRow (2.0, b20);

  // ===== 断言 =====
  std::cout << "\n------------------------------------------------------------------" << std::endl;
  int pass = 0, fail = 0;
  auto check = [&] (const std::string& name, bool ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
    if (ok) ++pass; else ++fail;
  };

  // 红利: factor 越大接入越多, 拒绝越少。
  check ("A: factor↑ 接入路数↑ (1.5 > 1.0)",          a15.peakActive > a10.peakActive);
  check ("A: factor↑ 接入路数↑ (2.0 >= 1.5)",         a20.peakActive >= a15.peakActive);
  check ("A: factor↑ 准入拒绝↓ (1.0 > 2.0)",          a10.admitRejects > a20.admitRejects);
  // 安全: 不超订零冲突; 低占空比下即便高 factor 冲突率也很低(overbooking 红利)。
  check ("A: factor=1.0 不超订 → 零冲突",             a10.conflictEvents == 0);
  check ("A: 低占空比下 2.0 冲突率仍低 (<25%)",       a20.conflictRatePct < 25.0);
  // 代价: 满负载超订才产生冲突, 但由隐式释放阀门兜底(冲突有界, 不随 factor 无限上升)。
  check ("B: factor=1.0 不超订 → 零冲突",             b10.conflictEvents == 0);
  check ("B: 满负载超订(1.5/2.0)产生冲突 (>0)",       b15.conflictEvents > 0 && b20.conflictEvents > 0);
  check ("B: factor↑ 准入拒绝↓ (1.0 > 2.0)",          b10.admitRejects > b20.admitRejects);

  std::cout << "------------------------------------------------------------------" << std::endl;
  std::cout << "PASS=" << pass << " FAIL=" << fail << std::endl;
  std::cout << "OVERALL RESULT: " << (fail == 0 ? "PASS" : "FAIL") << std::endl;
  return fail == 0 ? 0 : 1;
}
