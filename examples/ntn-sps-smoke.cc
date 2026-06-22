/*
 * 文件: contrib/geo-sat/examples/ntn-sps-smoke.cc
 * 功能: 阶段1 半静态 configured-grant 状态机的【时间驱动专项冒烟测试】。
 *
 * 目的: 现有 demo 是快照式(只跑几轮),只验到"激活"。本测试在仿真时间上跨多个
 *       SPS 周期(20ms)反复驱动调度器, 把 DL 的 完整生命周期 跑全:
 *         激活 -> 周期复用 -> (静默)隐式释放 -> (恢复)重新激活
 *       并验证 UL 侧 激活 + 复用 也真实发生; 末尾打印 m_spsStats 并做断言。
 *
 * 业务模型(3 个 SPS 合格 UE, 单波束):
 *   - rnti 11 (VOICE)   : 全程讲话      -> 激活 + 持续复用, 不释放
 *   - rnti 12 (VOICE)   : 讲(0-90ms) 静默(90-175ms) 再讲(175ms+)
 *                          -> 激活/复用 -> 静默触发 隐式释放 -> 恢复后 重新激活
 *   - rnti 13 (PORTABLE): 全程有数据    -> 激活 + 复用
 *
 * 运行: ./ns3 run ntn-sps-smoke
 */
#include "ns3/core-module.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"

#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSpsSmoke");

namespace {

constexpr uint32_t BEAM_ID = 1;
constexpr double   TICK_MS = 5.0;     // 采样/驱动步长(< 周期, 保证错峰到期都能被服务)
constexpr double   STOP_MS = 255.0;   // 覆盖 ~12 个 20ms 周期

struct UeSpec { uint16_t rnti; UtType ut; TrafficType tr; };
std::vector<UeSpec> g_ues;

// rnti 12 的讲话/静默时间表; 其余全程讲话。
bool
IsTalking (uint16_t rnti, double tms)
{
  if (rnti == 12)
    {
      return (tms < 90.0) || (tms >= 175.0);
    }
  return true;
}

void
PrintStats (Ptr<GeoBeamScheduler> sched, const std::string& tag)
{
  const SatSpsStats& s = sched->GetSpsStats ();
  std::cout << std::left << std::setw (10) << tag
            << " | DL act=" << s.dlActivations
            << " reuse=" << s.dlReuse
            << " relEmpty=" << s.dlReleaseEmpty
            << " relConf=" << s.dlReleaseConflict
            << " active=" << sched->GetDlSpsGrantCount ()
            << " || UL act=" << s.ulActivations
            << " reuse=" << s.ulReuse
            << " relEmpty=" << s.ulReleaseEmpty
            << " active=" << sched->GetUlSpsGrantCount ()
            << std::endl;
}

void
Tick (Ptr<GeoBeamScheduler> sched)
{
  const double tms = Simulator::Now ().GetMilliSeconds ();

  // 1) 按讲话/静默模型刷新各 UE 的 DL/UL 缓存(模拟业务到达)。
  for (const auto& u : g_ues)
    {
      const bool talk = IsTalking (u.rnti, tms);
      sched->UpdateUeDlBufferStatus (u.rnti, talk ? 120u : 0u);
      // UL 缓存设大值, 让 UL demand 持续为正(本测试不强制 UL 释放; UL 释放与 DL 同构)。
      sched->UpdateUeUlBufferStatus (u.rnti, talk ? 5000u : 0u);
    }

  // 2) 驱动一轮 DL + 一轮 UL 调度。
  sched->RunScheduler ();
  sched->RunUlSchedulingRound ();

  // 3) 每 ~50ms 打印一次快照, 方便观察生命周期推进。
  if (static_cast<int> (tms) % 50 == 0)
    {
      PrintStats (sched, "t=" + std::to_string (static_cast<int> (tms)) + "ms");
    }

  if (tms + TICK_MS <= STOP_MS)
    {
      Simulator::Schedule (MilliSeconds (TICK_MS), &Tick, sched);
    }
}

} // namespace

int
main (int argc, char* argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  // SPS 区大小必须在 ResourceManager 创建前经默认值设好(调度器内部创建 RM)。
  Config::SetDefault ("ns3::ResourceManager::DlSpsRegionRbs", UintegerValue (8));
  Config::SetDefault ("ns3::ResourceManager::UlSpsRegionRbs", UintegerValue (8));

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->SetAttribute ("SpsMode", UintegerValue (2));                  // Configured
  sched->SetAttribute ("SpsGrantPeriod", TimeValue (MilliSeconds (20)));
  sched->SetAttribute ("SpsImplicitReleaseAfter", UintegerValue (2));  // 连续2次空/冲突即释放
  sched->SetAttribute ("SpsStaggerEnabled", BooleanValue (true));
  sched->SetAttribute ("ClpcEnabled", BooleanValue (false));           // 简化: 不挂 CLPC 反馈
  sched->Initialize (BEAM_ID, 15);

  g_ues = {
    {11, UT_CONSUMER, TRAFFIC_VOICE},   // 全程讲话
    {12, UT_CONSUMER, TRAFFIC_VOICE},   // 讲->静默->再讲
    {13, UT_PORTABLE, TRAFFIC_DATA},    // portable, 全程有数据
  };
  for (const auto& u : g_ues)
    {
      sched->AddUeContext (u.rnti, u.ut, u.tr);
      sched->AddUeInfo (u.rnti, BEAM_ID);
      sched->UpdateUeDlCqi (u.rnti, 9.0);
      sched->UpdateUeUlCqi (u.rnti, 9.0);
    }

  std::cout << "==== SPS 时间驱动冒烟测试 (SpsMode=Configured, 周期=20ms, 释放阈=2) ====" << std::endl;
  std::cout << "UE: 11=VOICE(全程) 12=VOICE(讲0-90/静默90-175/再讲175+) 13=PORTABLE(全程)" << std::endl;
  std::cout << "------------------------------------------------------------------" << std::endl;

  Simulator::Schedule (MilliSeconds (0), &Tick, sched);
  Simulator::Stop (MilliSeconds (STOP_MS + 1));
  Simulator::Run ();

  std::cout << "------------------------------------------------------------------" << std::endl;
  PrintStats (sched, "FINAL");

  // ===== 断言 =====
  const SatSpsStats& s = sched->GetSpsStats ();
  int pass = 0, fail = 0;
  auto check = [&] (const std::string& name, bool ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << std::endl;
    if (ok) ++pass; else ++fail;
  };

  check ("DL 激活 >=4 (3 UE 初次 + UE12 恢复后重激活)", s.dlActivations >= 4);
  check ("DL 复用 >0 (周期复用真实发生)",               s.dlReuse > 0);
  check ("DL 隐式释放(空) >=1 (UE12 静默触发)",          s.dlReleaseEmpty >= 1);
  check ("DL 末态活跃 grant ==3 (UE12 已重激活)",        sched->GetDlSpsGrantCount () == 3);
  check ("UL 激活 >=3 (3 UE)",                          s.ulActivations >= 3);
  check ("UL 复用 >0 (周期复用真实发生)",               s.ulReuse > 0);

  std::cout << "------------------------------------------------------------------" << std::endl;
  std::cout << "PASS=" << pass << " FAIL=" << fail << std::endl;
  std::cout << "OVERALL RESULT: " << (fail == 0 ? "PASS" : "FAIL") << std::endl;

  Simulator::Destroy ();
  return fail == 0 ? 0 : 1;
}
