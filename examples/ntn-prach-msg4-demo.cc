/*
 * 文件: contrib/geo-sat/examples/ntn-prach-msg4-demo.cc
 *
 * PRACH 检测模型 + Msg4 解调模型 在 4 步 RA 流程上的集成验证 demo
 *
 *   GW 侧 (GeoBeamScheduler.ProcessPrachWindow):
 *     - 每个聚合窗口对每条 unique (raRnti, preambleId) 调
 *       PrachDetectionModel::DetectActivePreamble(snrDb).
 *       false -> 不发 RAR (PRACH miss), UE 等竞争超时重传
 *
 *   UE 侧 (SatUtMac.ReceiveMsg4 / ReceiveMsgB SUCCESS_RAR):
 *     - 收到 RAR 对应的 Msg4 时调
 *       Msg4DemodModel::DecodeMsg4(prachSnrDb) (PDCCH/PDSCH 两次伯努利).
 *       false -> 静默丢弃, 等竞争超时重传 + m_totalMsg4DemodFail++.
 *
 * 本 demo 跑两次:
 *   1. 关闭两个模型 (基线)
 *   2. 打开 PRACH Pd=0.7 + Msg4 BLER=0.3
 * 对比 PRACH miss 数 / Msg4 demod fail 数 / RA 成功数。
 *
 * 注意: 本 demo 直接调 scheduler->ReceivePrachPreamble + scheduler->RegisterUeRaCallbacks
 *       手工驱动 RA, 不动 PHY/MAC 数据面, 以最小拓扑验证两个 qyh 模型真上场。
 */

#include "ns3/core-module.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/prach-detection-model.h"
#include "ns3/msg4-demod-model.h"
#include "ns3/sat-ut-mac.h"

#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnPrachMsg4Demo");

namespace {

struct Counter
{
  uint32_t rarReceived = 0;
  uint32_t msg4Received = 0;
};
Counter g_cnt;

void OnRar (const RarMessage& rar)        { g_cnt.rarReceived++; (void) rar; }
void OnMsg4 (const RrcSetupMessage& msg4) { g_cnt.msg4Received++; (void) msg4; }

void
PreamblesBurst (Ptr<GeoBeamScheduler> sched, uint32_t n, uint32_t raRnti)
{
  for (uint32_t i = 0; i < n; ++i)
    {
      PrachPreamble p;
      p.rnti = 0;
      p.preambleId = (i % 63) + 1;
      p.format = 0;
      p.transmissionTime = Simulator::Now ();
      p.isRetransmission = false;
      p.raRnti = raRnti;
      sched->ReceivePrachPreamble (p);
    }
}

void
RunOnce (const std::string& tag, bool enableModels, double prachPd, double msg4PdcchBler, double msg4PdschBler)
{
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->Initialize (1, 1);

  // PRACH 检测模型
  if (enableModels)
    {
      Ptr<PrachDetectionModel> prach = CreateObject<PrachDetectionModel> ();
      prach->SetEnabled (true);
      prach->SetFixedProbabilities (prachPd, /*pfa=*/0.0);
      sched->SetPrachDetectionModel (prach);
      sched->SetDefaultPrachSnrDb (8.0);
    }

  // UE 端 Msg4 解调模型 (用一个虚拟 SatUtMac 接 RAR/Msg4 回调统计)
  Ptr<SatUtMac> utMac = CreateObject<SatUtMac> ();
  if (enableModels)
    {
      Ptr<Msg4DemodModel> m4 = CreateObject<Msg4DemodModel> ();
      m4->SetEnabled (true);
      m4->SetFixedBler (msg4PdcchBler, msg4PdschBler);
      utMac->SetMsg4DemodModel (m4);
      utMac->SetPrachSnrDb (8.0);
    }

  // 订阅 RAR + Msg4 计数
  g_cnt = {};
  sched->RegisterUeRaCallbacks (MakeCallback (&OnRar), MakeCallback (&OnMsg4));

  // 50 个 preamble (覆盖 PRACH 窗口聚合)
  const uint32_t N = 50;
  Simulator::Schedule (MilliSeconds (10), &PreamblesBurst, sched, N, /*raRnti=*/1);

  // 手工驱动 N 个 Msg4 进 utMac (脱离 RA 状态机, 只为验证 Msg4DemodModel 起作用):
  // ReceiveMsg4 入口检查 m_msg4DemodModel, 命中 BLER 概率 -> m_totalMsg4DemodFail++
  // 由于 ReceiveMsg4 还会查 m_isRaInitiated 等状态, 我们用一个简化版: 直接调
  // m4->DecodeMsg4 N 次累计 stat (避免引入完整 RA 状态机)。
  uint32_t demodFailDirect = 0;
  if (enableModels)
    {
      auto m4 = utMac->GetMsg4DemodModel ();
      for (uint32_t i = 0; i < N; ++i)
        {
          if (!m4->DecodeMsg4 (8.0)) demodFailDirect++;
        }
    }

  Simulator::Stop (Seconds (5.0));
  Simulator::Run ();

  std::cout << "\n[" << tag << "]\n";
  std::cout << "  preambles sent       : " << N << "\n";
  std::cout << "  RAR received (UE)    : " << g_cnt.rarReceived
            << "  (PRACH 过滤后真生成 RAR 数)\n";
  if (enableModels)
    {
      auto stats = sched->GetPrachDetectionModel ()->GetStats ();
      std::cout << "  PRACH detected groups: " << stats.detectedGroups
                << "  (DetectActivePreamble 返回 true 累计)\n";
      std::cout << "  PRACH missed groups  : " << stats.missedGroups
                << "  (Pd 决策 miss)\n";
      std::cout << "  Msg4 demod fail      : " << demodFailDirect << " / " << N
                << "  (Msg4DemodModel BLER 决策直接累计)\n";
    }
  std::cout << "  (RA 状态机 Msg4 fail : " << utMac->GetTotalMsg4DemodFail ()
            << " 因 demo 没驱动 ReceiveMsg4 路径, 状态机入口 fail=0 属正常)\n";

  Simulator::Destroy ();
}

} // namespace

int
main (int argc, char* argv[])
{
  double prachPd = 0.7;
  double msg4PdcchBler = 0.20;
  double msg4PdschBler = 0.20;

  CommandLine cmd;
  cmd.AddValue ("prachPd",       "PRACH 检测概率 (0-1)", prachPd);
  cmd.AddValue ("msg4PdcchBler", "Msg4 PDCCH BLER (0-1)", msg4PdcchBler);
  cmd.AddValue ("msg4PdschBler", "Msg4 PDSCH BLER (0-1)", msg4PdschBler);
  cmd.Parse (argc, argv);

  std::cout << "========================================\n";
  std::cout << "  PRACH + Msg4 BLER 模型集成验证\n";
  std::cout << "  PRACH Pd      = " << prachPd << "\n";
  std::cout << "  Msg4 PDCCH BLER= " << msg4PdcchBler << "\n";
  std::cout << "  Msg4 PDSCH BLER= " << msg4PdschBler << "\n";
  std::cout << "========================================";

  RunOnce ("baseline (models OFF)", false, 0, 0, 0);
  RunOnce ("models ON",              true,  prachPd, msg4PdcchBler, msg4PdschBler);

  std::cout << "\n========================================\n";
  std::cout << "解读:\n";
  std::cout << "  - models OFF: 全部 50 个 preamble 都生成 RAR + Msg4, RA 不被任何 BLER 影响\n";
  std::cout << "  - models ON : PRACH 按 Pd 概率丢 ~ (1-Pd)*N 个; UE Msg4 解调按 BLER 再丢\n";
  std::cout << "                两个模型真上场 -> PRACH miss > 0 && Msg4 demod fail > 0\n";

  return 0;
}
