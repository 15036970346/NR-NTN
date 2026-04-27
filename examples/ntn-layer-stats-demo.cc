/*
 * ntn-layer-stats-demo.cc
 * 演示分层统计收集器: PHY/MAC/RLC/PDCP/SDAP/RRC 各层统计输出
 *
 * 用法:
 *   ./ns3 run ntn-layer-stats-demo
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

// geo-sat 自定义协议栈
#include "ns3/sat-ut-phy.h"
#include "ns3/sat-ut-mac.h"
#include "ns3/sat-ut-rrc.h"
#include "ns3/sat-pdcp.h"
#include "ns3/sat-sdap.h"
#include "ns3/sat-mac-common.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/resource-manager.h"
#include "ns3/rohc-compressor.h"

// 分层统计
#include "ns3/sat-phy-stats-collector.h"
#include "ns3/sat-mac-stats-collector.h"
#include "ns3/sat-rlc-stats-collector.h"
#include "ns3/sat-pdcp-stats-collector.h"
#include "ns3/sat-sdap-stats-collector.h"
#include "ns3/sat-rrc-stats-collector.h"
#include "ns3/sat-e2e-stats-collector.h"
#include "ns3/sat-stats-connector.h"

#include <sys/stat.h>
#include <iostream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnLayerStatsDemo");

// RA 完成回调: 更新 RRC 状态
static Ptr<SatRrcStatsCollector> g_rrcStats;
static std::map<uint64_t, Ptr<SatUtRrc>> g_ueRrcMap;

void OnRaComplete (uint16_t ueId, SatUtMac::RaResult result)
{
    auto it = g_ueRrcMap.find (ueId);
    if (it == g_ueRrcMap.end ()) return;

    if (result == SatUtMac::RA_SUCCESS)
    {
        it->second->SwitchToState (SatUtRrc::CONNECTED_NORMALLY);
    }
    else
    {
        it->second->SwitchToState (SatUtRrc::IDLE_CAMPED_NORMALLY);
    }
}

int
main (int argc, char* argv[])
{
    uint32_t numUes = 10;
    double simTime = 5.0;

    CommandLine cmd;
    cmd.AddValue ("numUes", "Number of UEs", numUes);
    cmd.AddValue ("simTime", "Simulation time (s)", simTime);
    cmd.Parse (argc, argv);

    // 创建输出目录
    mkdir ("ntn-results", 0755);

    std::cout << "========================================\n";
    std::cout << "  NTN Layer Statistics Demo\n";
    std::cout << "  UEs: " << numUes << "  SimTime: " << simTime << "s\n";
    std::cout << "========================================\n\n";

    // =========================================================================
    // 创建分层统计收集器
    // =========================================================================
    Ptr<SatStatsConnector> connector = CreateObject<SatStatsConnector> ();
    auto phyStats  = CreateObject<SatPhyStatsCollector> ();
    auto macStats  = CreateObject<SatMacStatsCollector> ();
    auto rlcStats  = CreateObject<SatRlcStatsCollector> ();
    auto pdcpStats = CreateObject<SatPdcpStatsCollector> ();
    auto sdapStats = CreateObject<SatSdapStatsCollector> ();
    auto rrcStats  = CreateObject<SatRrcStatsCollector> ();
    auto e2eStats  = CreateObject<SatE2eStatsCollector> ();

    connector->SetPhyStats (phyStats);
    connector->SetMacStats (macStats);
    connector->SetRlcStats (rlcStats);
    connector->SetPdcpStats (pdcpStats);
    connector->SetSdapStats (sdapStats);
    connector->SetRrcStats (rrcStats);
    connector->SetE2eStats (e2eStats);
    g_rrcStats = rrcStats;

    // =========================================================================
    // 创建 gNB 侧调度器
    // =========================================================================
    Ptr<GeoBeamScheduler> scheduler = CreateObject<GeoBeamScheduler> ();
    scheduler->Initialize (1, 1);

    // =========================================================================
    // 创建 UE 协议栈并连接 TracedCallbacks
    // =========================================================================
    std::vector<Ptr<SatUtPhy>> uePhys (numUes);
    std::vector<Ptr<SatUtMac>> ueMacs (numUes);
    std::vector<Ptr<SatUtRrc>> ueRrcs (numUes);
    std::vector<Ptr<SatPdcp>>  uePdcps (numUes);
    std::vector<Ptr<SatSdap>>  ueSdaps (numUes);

    for (uint32_t i = 0; i < numUes; ++i)
    {
        uint16_t rnti = i + 1;

        // --- PHY ---
        uePhys[i] = CreateObject<SatUtPhy> ();
        // 连接 PHY TracedCallbacks
        uePhys[i]->TraceConnectWithoutContext (
            "RsrpTrace", MakeCallback (&SatPhyStatsCollector::OnRsrp, phyStats));
        uePhys[i]->TraceConnectWithoutContext (
            "SinrTrace", MakeCallback (&SatPhyStatsCollector::OnSinr, phyStats));
        uePhys[i]->TraceConnectWithoutContext (
            "RsrqTrace", MakeCallback (&SatPhyStatsCollector::OnRsrq, phyStats));

        // --- MAC ---
        ueMacs[i] = CreateObject<SatUtMac> ();
        ueMacs[i]->SetUtType (UT_CONSUMER);
        ueMacs[i]->SetNumPreambles (64);
        // 连接 MAC TracedCallbacks
        ueMacs[i]->TraceConnectWithoutContext (
            "QueueLength", MakeCallback (&SatMacStatsCollector::OnQueueLength, macStats));
        ueMacs[i]->TraceConnectWithoutContext (
            "QueueDelay", MakeCallback (&SatMacStatsCollector::OnQueueDelay, macStats));

        // --- RRC ---
        ueRrcs[i] = CreateObject<SatUtRrc> ();
        ueRrcs[i]->SetUeId (rnti);
        ueRrcs[i]->SetInactivityTimer (Seconds (3));
        // 连接 RRC TracedCallback
        ueRrcs[i]->TraceConnectWithoutContext (
            "StateTransition", MakeCallback (&SatRrcStatsCollector::OnStateTransition, rrcStats));
        g_ueRrcMap[rnti] = ueRrcs[i];

        // --- PDCP ---
        uePdcps[i] = CreateObject<SatPdcp> ();
        uePdcps[i]->SetRnti (rnti);
        uePdcps[i]->SetLcId (3);
        Ptr<RohcCompressor> rohc = CreateObject<RohcCompressor> ();
        uePdcps[i]->SetRohcCompressor (rohc);
        // 连接 PDCP TracedCallbacks
        uePdcps[i]->TraceConnectWithoutContext (
            "TxPDCP", MakeCallback (&SatPdcpStatsCollector::DlTxPdu, pdcpStats));
        uePdcps[i]->TraceConnectWithoutContext (
            "RxPDCP", MakeCallback (&SatPdcpStatsCollector::DlRxPdu, pdcpStats));
        uePdcps[i]->TraceConnectWithoutContext (
            "RohcCompression", MakeCallback (&SatPdcpStatsCollector::OnRohcCompression, pdcpStats));

        // --- SDAP ---
        ueSdaps[i] = CreateObject<SatSdap> ();
        ueSdaps[i]->SetRnti (rnti);
        ueSdaps[i]->MapQfiToDrb (1, 3);  // Voice QFI=1 -> LCID=3
        ueSdaps[i]->MapQfiToDrb (5, 4);  // Data  QFI=5 -> LCID=4
        // 连接 SDAP TracedCallbacks
        ueSdaps[i]->TraceConnectWithoutContext (
            "TxSDAP", MakeCallback (&SatSdapStatsCollector::OnTxSdap, sdapStats));
        ueSdaps[i]->TraceConnectWithoutContext (
            "RxSDAP", MakeCallback (&SatSdapStatsCollector::OnRxSdap, sdapStats));

        // 注册 gNB UE context
        scheduler->AddUeContext (rnti, UT_CONSUMER, TRAFFIC_DATA);
    }

    // =========================================================================
    // 调度仿真事件: 模拟各层活动
    // =========================================================================
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();

    for (uint32_t i = 0; i < numUes; ++i)
    {
        uint16_t rnti = i + 1;
        double offset = i * 0.01;  // 错开 UE 时间

        // --- RRC: IDLE → RA → CONNECTED ---
        Simulator::Schedule (MilliSeconds (100 + i * 50), [&, i] () {
            ueRrcs[i]->SwitchToState (SatUtRrc::IDLE_CAMPED_NORMALLY);
        });
        Simulator::Schedule (MilliSeconds (200 + i * 50), [&, i] () {
            ueRrcs[i]->SwitchToState (SatUtRrc::IDLE_RANDOM_ACCESS);
        });
        Simulator::Schedule (MilliSeconds (900 + i * 50), [&, i] () {
            ueRrcs[i]->SwitchToState (SatUtRrc::CONNECTED_NORMALLY);
        });

        // --- PHY: 周期性 SINR 测量 ---
        for (uint32_t t = 1; t < (uint32_t)simTime; ++t)
        {
            double rxPower = -85.0 + rng->GetValue (-5.0, 5.0);
            Simulator::Schedule (Seconds (t + offset), [&, i, rxPower] () {
                uePhys[i]->CalculateComprehensiveSinr (rxPower, 1);
            });
        }

        // --- PHY: 传播时延统计 ---
        phyStats->RecordPropDelay (rnti, MilliSeconds (300));

        // --- MAC: 缓冲数据 → 发送 ---
        for (uint32_t t = 1; t < (uint32_t)simTime; ++t)
        {
            uint32_t bytes = 500 + rng->GetInteger (0, 1000);
            Simulator::Schedule (Seconds (t + offset + 0.001), [&, i, bytes] () {
                ueMacs[i]->NotifyDataBuffered (bytes);
            });
        }

        // --- PDCP: 发送和接收数据 ---
        for (uint32_t t = 1; t < (uint32_t)simTime; ++t)
        {
            Simulator::Schedule (Seconds (t + offset + 0.002), [&, i] () {
                Ptr<Packet> p = Create<Packet> (1000);
                uePdcps[i]->DoSendData (p);
            });
            // 模拟 600ms 后接收
            Simulator::Schedule (Seconds (t + offset + 0.602), [&, i] () {
                Ptr<Packet> p = Create<Packet> (800);
                uePdcps[i]->DoReceiveData (p);
            });
        }

        // --- SDAP: QoS 流发送/接收 ---
        for (uint32_t t = 1; t < (uint32_t)simTime; ++t)
        {
            // Voice (QFI=1)
            Simulator::Schedule (Seconds (t + offset + 0.003), [&, i] () {
                Ptr<Packet> p = Create<Packet> (160);
                ueSdaps[i]->DoSendData (1, p);
            });
            Simulator::Schedule (Seconds (t + offset + 0.603), [&, i] () {
                Ptr<Packet> p = Create<Packet> (160);
                ueSdaps[i]->DoReceiveData (1, p);
            });

            // Data (QFI=5)
            Simulator::Schedule (Seconds (t + offset + 0.004), [&, i] () {
                Ptr<Packet> p = Create<Packet> (1200);
                ueSdaps[i]->DoSendData (5, p);
            });
            Simulator::Schedule (Seconds (t + offset + 0.604), [&, i] () {
                Ptr<Packet> p = Create<Packet> (1200);
                ueSdaps[i]->DoReceiveData (5, p);
            });
        }

        // --- RRC: 不活动超时 → INACTIVE → Paging → 恢复 ---
        if (i < numUes / 2)
        {
            // 模拟半数 UE 在仿真中期不活动后被寻呼
            double inactiveTime = 2.0 + i * 0.1;
            Simulator::Schedule (Seconds (inactiveTime), [&, i] () {
                // 不活动定时器触发 (通过 SatUtRrc 内部自动触发)
                // 这里手动演示: CONNECTED → INACTIVE
                ueRrcs[i]->SwitchToState (SatUtRrc::RRC_INACTIVE);
            });
            // 寻呼恢复
            Simulator::Schedule (Seconds (inactiveTime + 1.0), [&, i] () {
                ueRrcs[i]->ReceivePagingMessage ();
            });
            // RA 完成后恢复
            Simulator::Schedule (Seconds (inactiveTime + 1.9), [&, i] () {
                ueRrcs[i]->SwitchToState (SatUtRrc::CONNECTED_NORMALLY);
                rrcStats->RecordRecoveryDelay (i + 1, MilliSeconds (900));
            });
        }

        // --- E2E delay: 聚合各层 ---
        for (uint32_t t = 1; t < (uint32_t)simTime; ++t)
        {
            Simulator::Schedule (Seconds (t + offset + 0.7), [&, rnti] () {
                e2eStats->RecordPhyDelay (rnti, 300.0);    // PHY 传播 300ms
                e2eStats->RecordMacDelay (rnti, 5.0);      // MAC 排队 5ms
                e2eStats->RecordPdcpDelay (rnti, 2.0);     // PDCP 处理 2ms
                e2eStats->RecordE2eDelay (rnti, 307.0);    // 总计 ~307ms (单向)
            });
        }
    }

    // =========================================================================
    // 运行仿真
    // =========================================================================
    std::cout << "Running simulation...\n";
    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();

    // =========================================================================
    // 导出所有分层统计
    // =========================================================================
    std::cout << "\n========================================\n";
    std::cout << "  Exporting Layer Statistics\n";
    std::cout << "========================================\n";

    connector->ExportAll ("ntn-results", Seconds (simTime));

    // =========================================================================
    // 打印摘要到控制台
    // =========================================================================
    std::cout << "\n=== PHY Layer ===\n";
    auto aggRsrp = phyStats->GetAggregateRsrpStats ();
    auto aggSinr = phyStats->GetAggregateSinrStats ();
    auto aggRsrq = phyStats->GetAggregateRsrqStats ();
    std::cout << std::fixed << std::setprecision (2);
    std::cout << "RSRP: avg=" << aggRsrp.avg << " dBm, min=" << aggRsrp.min
              << ", max=" << aggRsrp.max << " (" << aggRsrp.count << " samples)\n";
    std::cout << "SINR: avg=" << aggSinr.avg << " dB, min=" << aggSinr.min
              << ", max=" << aggSinr.max << "\n";
    std::cout << "RSRQ: avg=" << aggRsrq.avg << " dB, min=" << aggRsrq.min
              << ", max=" << aggRsrq.max << "\n";

    std::cout << "\n=== MAC Layer ===\n";
    auto aggQueueDelay = macStats->GetAggregateQueueDelayStats ();
    auto aggQueueLen   = macStats->GetAggregateQueueLengthStats ();
    std::cout << "Queue Delay: avg=" << aggQueueDelay.avg << " ms, P95=" << aggQueueDelay.p95
              << " ms (" << aggQueueDelay.count << " samples)\n";
    std::cout << "Queue Length: avg=" << aggQueueLen.avg << " bytes, max=" << aggQueueLen.max
              << " bytes\n";

    std::cout << "\n=== PDCP Layer ===\n";
    std::cout << "ROHC Compression Ratio: " << pdcpStats->GetAggregateRohcRatio () << "\n";

    std::cout << "\n=== SDAP Layer ===\n";
    auto voiceQfi = sdapStats->GetQfiStats (1);
    auto dataQfi  = sdapStats->GetQfiStats (5);
    std::cout << "Voice (QFI=1): RxBytes=" << voiceQfi.rxBytes
              << ", AvgDelay=" << voiceQfi.avgDelayMs << " ms"
              << ", P95Delay=" << voiceQfi.p95DelayMs << " ms\n";
    std::cout << "Data  (QFI=5): RxBytes=" << dataQfi.rxBytes
              << ", AvgDelay=" << dataQfi.avgDelayMs << " ms"
              << ", P95Delay=" << dataQfi.p95DelayMs << " ms\n";

    std::cout << "\n=== RRC Layer ===\n";
    std::cout << "Total state transitions: " << rrcStats->GetTotalTransitions () << "\n";
    std::cout << "Avg recovery delay: " << rrcStats->GetAverageRecoveryDelayMs () << " ms\n";

    std::cout << "\n=== End-to-End ===\n";
    auto aggE2e = e2eStats->GetAggregateE2eStats ();
    std::cout << "E2E Delay: avg=" << aggE2e.avgDelayMs << " ms, P95=" << aggE2e.p95DelayMs
              << " ms, min=" << aggE2e.minDelayMs << ", max=" << aggE2e.maxDelayMs
              << " (" << aggE2e.sampleCount << " samples)\n";

    auto phyLayer = e2eStats->GetLayerStats ("PHY");
    auto macLayer = e2eStats->GetLayerStats ("MAC");
    auto pdcpLayer = e2eStats->GetLayerStats ("PDCP");
    std::cout << "  PHY: avg=" << phyLayer.avgDelayMs << " ms\n";
    std::cout << "  MAC: avg=" << macLayer.avgDelayMs << " ms\n";
    std::cout << "  PDCP: avg=" << pdcpLayer.avgDelayMs << " ms\n";

    std::cout << "\n========================================\n";
    std::cout << "Files exported to ntn-results/:\n";
    std::cout << "  phy_stats.txt, mac_stats.txt, rlc_stats.txt,\n";
    std::cout << "  pdcp_stats.txt, sdap_stats.txt, rrc_stats.txt,\n";
    std::cout << "  e2e_delay_stats.txt\n";
    std::cout << "========================================\n";

    Simulator::Destroy ();
    return 0;
}
