/*
 * NTN GEO Satellite Random Access Comparison Simulation
 *
 * 对比场景：
 * - 7 个波束, 每波束 N 个 UE (默认 200, 共 1400)
 * - 200 UE 均分到 5 个 RO (Random Access Occasion), 每 RO 40 UE
 * - 相邻 RO 间隔 20ms (符合 NR PRACH 配置典型值)
 * - 每个 RO 内所有 UE 在 100μs 窗口内发起, 落入同一聚合窗口产生碰撞
 * - preambleId 随机从 1-63 中选取
 * - 对比 4 步 RA 与 2 步 RA 的接入成功率和平均时延
 *
 * 关键参数:
 *   --numRos:      每种模式的 RO 数量 (默认 5)
 *   --roSpacingMs: 相邻 RO 的时间间隔 ms (默认 20ms)
 *   --raWindow:    每个 RO 内 UE 发起时间窗口 (秒, 默认 100μs)
 *
 * 用法：
 *   ./ns3 run "ntn-ra-compare"                                    # 默认: 对比模式, 5 RO, 200 UE/beam
 *   ./ns3 run "ntn-ra-compare --rachType=0"                       # 仅 4 步
 *   ./ns3 run "ntn-ra-compare --rachType=1"                       # 仅 2 步
 *   ./ns3 run "ntn-ra-compare --numRos=1 --numUesPerBeam=100"     # 退回单 RO 场景
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include "ns3/geo-beam-scheduler.h"
#include "ns3/sat-ut-mac.h"
#include "ns3/sat-mac-common.h"
#include "ns3/resource-manager.h"
#include "ns3/admit-control.h"

#include <fstream>
#include <iomanip>
#include <map>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sys/stat.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnRaCompare");

// ==================== 统计数据结构 ====================

struct UeRaRecord {
    uint32_t ueIndex;
    uint32_t beamId;
    uint32_t preambleId;      // 初始选择的 preambleId
    std::string rachMode;     // "4-step" or "2-step"
    Time startTime;           // 发起 RA 的时间
    Time endTime;             // RA 完成的时间
    bool success;
    // 来自 SatUtMac 的逐 UE 计数器
    uint32_t totalMsg1Sent;           // 该 UE 总发 Msg1/MsgA 次数
    uint32_t totalMsg4Received;       // 该 UE 收到有效 Msg4/MsgB-SUCCESS 次数
    uint32_t totalContentionTimeouts; // 该 UE 碰撞超时次数
    uint32_t totalMsgBTimeouts;       // 该 UE MsgB 超时次数 (2步专用)
};

static std::vector<UeRaRecord> g_raRecords;
// 保存 (recordIndex, SatUtMac*) 用于仿真结束后读取计数器
static std::vector<std::pair<uint32_t, Ptr<SatUtMac>>> g_utMacRefs;

// RA 发起时记录开始时间
static void
OnRaStarted (uint32_t recordIndex)
{
    g_raRecords[recordIndex].startTime = Simulator::Now ();
}

// RA 完成回调
static void
OnRaComplete (uint32_t recordIndex, SatUtMac::RaResult result)
{
    g_raRecords[recordIndex].endTime = Simulator::Now ();
    g_raRecords[recordIndex].success = (result == SatUtMac::RA_SUCCESS);
}

// ==================== 辅助: 创建并连接一个 UE ====================

static void
SetupUe (Ptr<SatUtMac> utMac,
         Ptr<GeoBeamScheduler> scheduler,
         uint32_t recordIndex,
         RachType rachType,
         uint64_t ueIdentity,
         Time startDelay,
         uint32_t preambleId,
         uint8_t maxRaAttempts = 10,
         uint32_t numPreambles = 64)
{
    utMac->SwitchState (SatUtMac::MAC_IDLE);
    utMac->SetUeIdentity (ueIdentity);
    utMac->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), maxRaAttempts);
    utMac->SetNumPreambles (numPreambles);

    // 4 步回调 (所有 UE 都需要, 2 步 FALLBACK 会复用)
    utMac->SetPrachCallback (MakeCallback (&GeoBeamScheduler::ReceivePrachPreamble, scheduler));
    utMac->SetMsg3Callback (MakeCallback (&GeoBeamScheduler::ReceiveMsg3Packet, scheduler));
    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMac),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMac));

    if (rachType == RachType::TWO_STEP) {
        utMac->SetRachType (RachType::TWO_STEP);
        utMac->SetMsgACallback (MakeCallback (&GeoBeamScheduler::ReceiveMsgA, scheduler));
        scheduler->RegisterUeTwoStepRaCallbacks (
            MakeCallback (&SatUtMac::ReceiveMsgB, utMac));
    } else {
        utMac->SetRachType (RachType::FOUR_STEP);
    }

    utMac->SetRaCompleteCallback (MakeBoundCallback (&OnRaComplete, recordIndex));

    // 调度 RA 发起
    Simulator::Schedule (startDelay, &OnRaStarted, recordIndex);
    Simulator::Schedule (startDelay, &SatUtMac::InitiateRandomAccess, utMac,
                         preambleId, static_cast<uint8_t> (0));
}

// ==================== 碰撞概率计算 (Birthday Problem) ====================

static double
ExpectedCollisionRate (uint32_t numUes, uint32_t numPreambles)
{
    // P(至少一次碰撞) 的补: 所有 UE 都不碰撞的概率
    // P(no collision) = n! / (n^k * (n-k)!) ≈ e^{-k(k-1)/(2n)}
    // 碰撞的 UE 数 ≈ k - n*(1 - (1-1/n)^k) (每个 preamble 被选中的期望)
    // 简化: 期望碰撞 UE 比例
    double k = numUes;
    double n = numPreambles;
    // 期望无碰撞的 UE 数: sum over each preamble of P(exactly 1 UE chose it)
    // P(exactly 1) = C(k,1) * (1/n) * (1-1/n)^(k-1)
    // 期望 "安全" preamble 数 = n * C(k,1) * (1/n)^1 * (1-1/n)^(k-1) = k * (1-1/n)^(k-1)
    double safeUes = k * std::pow (1.0 - 1.0 / n, k - 1);
    double collisionUes = k - safeUes;
    return collisionUes / k * 100.0;  // 百分比
}

// ==================== 统计汇总 ====================

struct RaSummary {
    std::string mode;
    // UE 级统计 (最终结果)
    uint32_t totalUes;
    uint32_t successCount;
    uint32_t failedCount;
    double ueSuccessRate;       // 最终成功 UE 数 / 总 UE 数
    // 时延
    double avgLatencyMs;
    double minLatencyMs;
    double maxLatencyMs;
    double p50LatencyMs;
    double p95LatencyMs;
    // 按你的定义: 基于发送次数的指标
    uint32_t totalMsg1Sent;           // 所有 UE 发送 Msg1/MsgA 总次数
    uint32_t totalMsg4Received;       // 所有 UE 收到有效 Msg4/MsgB-SUCCESS 总次数
    uint32_t totalContentionTimeouts; // 所有 UE 碰撞超时总次数
    double accessSuccessRate;   // = totalMsg4Received / totalMsg1Sent
    double collisionRate;       // = totalContentionTimeouts / totalMsg1Sent
};

static RaSummary
ComputeSummary (const std::string& mode)
{
    RaSummary s;
    s.mode = mode;
    s.totalUes = 0;
    s.successCount = 0;
    s.failedCount = 0;
    s.avgLatencyMs = 0;
    s.minLatencyMs = 1e9;
    s.maxLatencyMs = 0;
    s.totalMsg1Sent = 0;
    s.totalMsg4Received = 0;
    s.totalContentionTimeouts = 0;

    std::vector<double> latencies;

    for (const auto& r : g_raRecords) {
        if (r.rachMode != mode) continue;
        s.totalUes++;
        // 累加计数器
        s.totalMsg1Sent           += r.totalMsg1Sent;
        s.totalMsg4Received       += r.totalMsg4Received;
        s.totalContentionTimeouts += r.totalContentionTimeouts;
        if (r.success) {
            s.successCount++;
            double lat = (r.endTime - r.startTime).GetMilliSeconds ();
            latencies.push_back (lat);
        } else {
            s.failedCount++;
        }
    }

    s.ueSuccessRate = s.totalUes > 0 ? (double)s.successCount / s.totalUes * 100.0 : 0;
    // 按你的定义计算
    s.accessSuccessRate = s.totalMsg1Sent > 0
        ? (double)s.totalMsg4Received / s.totalMsg1Sent * 100.0 : 0;
    s.collisionRate = s.totalMsg1Sent > 0
        ? (double)s.totalContentionTimeouts / s.totalMsg1Sent * 100.0 : 0;

    if (!latencies.empty ()) {
        std::sort (latencies.begin (), latencies.end ());
        double sum = std::accumulate (latencies.begin (), latencies.end (), 0.0);
        s.avgLatencyMs = sum / latencies.size ();
        s.minLatencyMs = latencies.front ();
        s.maxLatencyMs = latencies.back ();
        s.p50LatencyMs = latencies[latencies.size () / 2];
        size_t p95idx = std::min ((size_t)(latencies.size () * 0.95), latencies.size () - 1);
        s.p95LatencyMs = latencies[p95idx];
    } else {
        s.minLatencyMs = 0;
        s.p50LatencyMs = 0;
        s.p95LatencyMs = 0;
    }

    return s;
}

static void
PrintSummary (const RaSummary& s)
{
    std::cout << "| " << std::setw (8) << s.mode
              << " | " << std::setw (6) << s.totalUes
              << " | " << std::setw (7) << s.successCount
              << " | " << std::setw (8) << std::fixed << std::setprecision (1) << s.ueSuccessRate << "%"
              << " | " << std::setw (7) << s.totalMsg1Sent
              << " | " << std::setw (7) << s.totalMsg4Received
              << " | " << std::setw (11) << s.totalContentionTimeouts
              << " | " << std::setw (12) << std::fixed << std::setprecision (1) << s.accessSuccessRate << "%"
              << " | " << std::setw (11) << std::fixed << std::setprecision (1) << s.collisionRate << "%"
              << " | " << std::setw (8) << std::fixed << std::setprecision (1) << s.avgLatencyMs
              << " | " << std::setw (8) << std::fixed << std::setprecision (1) << s.p50LatencyMs
              << " | " << std::setw (8) << std::fixed << std::setprecision (1) << s.p95LatencyMs
              << " |" << std::endl;
}

// ==================== main ====================

int main (int argc, char *argv[])
{
    uint32_t numBeams = 7;
    uint32_t numUesPerBeam = 3000;  // 每波束 3000 UE
    uint32_t numRos = 20;           // 每种模式 20 个 RO
    double roSpacingMs = 1.0;       // 相邻 RO 间隔 1ms
    uint32_t rachType = 2;          // 0=4step, 1=2step, 2=both (对比)
    double raWindowSec = 0.0001;    // 100μs — 强制每个 RO 内所有 UE 落入同一聚合窗口
    double simTime = 60.0;
    bool enableLogs = false;
    uint32_t maxRaAttempts = 5;     // 最大接入尝试次数 (含首次)
    uint32_t numPrachChannels = 1;  // PRACH 频域信道数 (1=64 preambles, 2=128)

    CommandLine cmd;
    cmd.AddValue ("numBeams", "Number of beams (default: 7)", numBeams);
    cmd.AddValue ("numUesPerBeam", "UEs per beam (default: 3000)", numUesPerBeam);
    cmd.AddValue ("numRos", "Number of ROs per mode (default: 20)", numRos);
    cmd.AddValue ("roSpacingMs", "RO spacing in ms (default: 1)", roSpacingMs);
    cmd.AddValue ("rachType", "0=4-step only, 1=2-step only, 2=both (compare)", rachType);
    cmd.AddValue ("raWindow", "RA initiation window per RO in seconds (0.0001=burst)", raWindowSec);
    cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue ("enableLogs", "Enable detailed MAC/scheduler logs", enableLogs);
    cmd.AddValue ("maxRaAttempts", "Max RA attempts per UE (default: 5)", maxRaAttempts);
    cmd.AddValue ("numPrachChannels", "PRACH freq channels per RO (1=64, 2=128 preambles)", numPrachChannels);
    cmd.Parse (argc, argv);

    if (enableLogs) {
        LogComponentEnable ("NtnRaCompare", LOG_LEVEL_INFO);
        LogComponentEnable ("SatUtMac", LOG_LEVEL_INFO);
        LogComponentEnable ("GeoBeamScheduler", LOG_LEVEL_INFO);
    }

    // 确定要跑哪些模式
    std::vector<std::string> modes;
    if (rachType == 0) {
        modes.push_back ("4-step");
    } else if (rachType == 1) {
        modes.push_back ("2-step");
    } else {
        modes.push_back ("4-step");
        modes.push_back ("2-step");
    }

    uint32_t totalUes = numBeams * numUesPerBeam;
    uint32_t numPreambles = numPrachChannels * 64;  // 每信道 64 个 preamble

    // 每个 RO 分到的 UE 数 (均分, 余数放最后一个 RO)
    uint32_t uesPerRo = numUesPerBeam / numRos;
    uint32_t uesLastRo = numUesPerBeam - uesPerRo * (numRos - 1);

    // 计算理论碰撞率 (Birthday Problem, 按每 RO 的 UE 数计算)
    double expectedCollisionPct = ExpectedCollisionRate (uesPerRo, numPreambles);

    std::cout << "========================================" << std::endl;
    std::cout << "  NTN GEO Random Access Comparison" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Beams:            " << numBeams << std::endl;
    std::cout << "  UEs/beam:         " << numUesPerBeam << std::endl;
    std::cout << "  Total UEs:        " << totalUes << std::endl;
    std::cout << "  ROs per mode:     " << numRos << std::endl;
    std::cout << "  UEs/RO (typical): " << uesPerRo << " (last RO: " << uesLastRo << ")" << std::endl;
    std::cout << "  RO spacing:       " << roSpacingMs << " ms" << std::endl;
    std::cout << "  PRACH channels:   " << numPrachChannels << " (" << numPreambles << " preambles/RO)" << std::endl;
    std::cout << "  RA window/RO:     " << raWindowSec * 1e6 << " us" << std::endl;
    std::cout << "  PRACH agg window: 500 us" << std::endl;
    std::cout << "  Sim time:         " << simTime << "s" << std::endl;
    std::cout << "  Max RA attempts:  " << maxRaAttempts << std::endl;
    std::cout << "  Modes:            ";
    for (const auto& m : modes) std::cout << m << " ";
    std::cout << std::endl;
    std::cout << "  Expected 1st-try collision/RO: ~"
              << std::fixed << std::setprecision (1) << expectedCollisionPct
              << "% of UEs" << std::endl;
    std::cout << "  (Birthday Problem: " << uesPerRo << " UEs/RO choosing from "
              << numPreambles << " preambles)" << std::endl;
    std::cout << "========================================" << std::endl;

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();

    double batchOffset = 1.0;  // 从 1s 开始, 留出初始化时间

    // 每种模式用独立的调度器组 (避免状态污染)
    std::vector<Ptr<SatUtMac>> allUtMacs;

    for (const auto& mode : modes) {
        RachType rt = (mode == "2-step") ? RachType::TWO_STEP : RachType::FOUR_STEP;

        // 每种模式新建独立的调度器
        std::vector<Ptr<GeoBeamScheduler>> schedulers (numBeams);
        for (uint32_t b = 0; b < numBeams; b++) {
            schedulers[b] = CreateObject<GeoBeamScheduler> ();
            schedulers[b]->Initialize (b + 1, 1);
            Ptr<AdmitControl> ac = CreateObject<AdmitControl> ();
            ac->SetBeamTotalRbs (b + 1, 25);
            schedulers[b]->SetAdmitControl (ac);
        }

        // 统计本批次 Beam 1 的实际碰撞 (跨所有 RO 汇总, 用于打印)
        // key: roIdx, value: map<preambleId, count>
        std::map<uint32_t, std::map<uint32_t, uint32_t>> roBeam1PreambleCounts;

        uint32_t ueCounter = 0;  // 当前模式内已分配的 UE 编号 (跨 beam、跨 RO)

        for (uint32_t b = 0; b < numBeams; b++) {
            for (uint32_t ro = 0; ro < numRos; ro++) {
                // 最后一个 RO 可能多一些 UE (处理整除余数)
                uint32_t uesThisRo = (ro == numRos - 1) ? uesLastRo : uesPerRo;

                // 该 RO 的时间基准: batchOffset + ro * roSpacing
                double roBaseSec = batchOffset + ro * roSpacingMs / 1000.0;

                for (uint32_t u = 0; u < uesThisRo; u++) {
                    uint32_t globalIdx = g_raRecords.size ();

                    // preambleId 随机 (1-63), 每个 RO 独立竞争
                    uint32_t preambleId = rng->GetInteger (1, numPreambles);
                    if (b == 0) {  // 只统计 beam 1
                        roBeam1PreambleCounts[ro][preambleId]++;
                    }

                    UeRaRecord rec;
                    rec.ueIndex = globalIdx;
                    rec.beamId = b + 1;
                    rec.preambleId = preambleId;
                    rec.rachMode = mode;
                    rec.success = false;
                    rec.startTime = Seconds (0);
                    rec.endTime = Seconds (0);
                    rec.totalMsg1Sent = 0;
                    rec.totalMsg4Received = 0;
                    rec.totalContentionTimeouts = 0;
                    rec.totalMsgBTimeouts = 0;
                    g_raRecords.push_back (rec);

                    Ptr<SatUtMac> utMac = CreateObject<SatUtMac> ();
                    allUtMacs.push_back (utMac);
                    g_utMacRefs.push_back ({globalIdx, utMac});

                    // 在 [roBase, roBase + raWindow] 内随机发起 (100μs 窗口)
                    double startSec = roBaseSec + rng->GetValue (0.0, raWindowSec);

                    uint64_t identity = ((uint64_t)(b + 1) << 40)
                                        | ((uint64_t)ueCounter << 16)
                                        | rng->GetInteger (0, 0xFFFF);
                    ueCounter++;

                    SetupUe (utMac, schedulers[b], globalIdx, rt, identity,
                             Seconds (startSec), preambleId,
                             static_cast<uint8_t> (maxRaAttempts),
                             numPreambles);
                }
            }
        }

        // 打印 Beam 1 各 RO 的实际碰撞情况
        uint32_t totalCollisionUes = 0;
        uint32_t totalBeam1Ues = 0;
        for (uint32_t ro = 0; ro < numRos; ro++) {
            uint32_t roCollision = 0;
            uint32_t roTotal = (ro == numRos - 1) ? uesLastRo : uesPerRo;
            totalBeam1Ues += roTotal;
            for (const auto& kv : roBeam1PreambleCounts[ro]) {
                if (kv.second > 1) roCollision += kv.second;
            }
            totalCollisionUes += roCollision;
            std::cout << "[" << mode << "] Beam 1 RO#" << (ro + 1)
                      << " (" << roTotal << " UE): collision "
                      << roCollision << "/" << roTotal << " ("
                      << std::fixed << std::setprecision (1)
                      << (double)roCollision / roTotal * 100.0 << "%)" << std::endl;
        }
        std::cout << "[" << mode << "] Beam 1 total: " << totalCollisionUes
                  << "/" << totalBeam1Ues << " UEs in collision ("
                  << std::fixed << std::setprecision (1)
                  << (double)totalCollisionUes / totalBeam1Ues * 100.0 << "%)" << std::endl;

        // 下一批次偏移:
        // 本批最后一个 RO 的时间 + 给足 GEO 多次重传的时间
        // 每次重传 ≈ 首次RTT(1.2s) + contention timer(1.5s) + backoff ≈ 3s
        double lastRoOffset = (numRos - 1) * roSpacingMs / 1000.0;
        double retryBudgetSec = maxRaAttempts * 3.5 + 5.0;  // 每次尝试 ~3.5s + 安全裕量
        batchOffset += lastRoOffset + raWindowSec + retryBudgetSec;
    }

    // 自动调整 simTime 使其足够
    double requiredSimTime = batchOffset + 5.0;
    if (simTime < requiredSimTime) {
        std::cout << "[INFO] Adjusting simTime from " << simTime
                  << "s to " << requiredSimTime << "s to cover all batches" << std::endl;
        simTime = requiredSimTime;
    }

    // 运行
    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();

    // 仿真结束后从每个 UE 读取计数器 (必须在 Destroy 之前)
    for (auto& ref : g_utMacRefs) {
        uint32_t idx = ref.first;
        Ptr<SatUtMac> mac = ref.second;
        g_raRecords[idx].totalMsg1Sent           = mac->GetTotalMsg1Sent ();
        g_raRecords[idx].totalMsg4Received       = mac->GetTotalMsg4Received ();
        g_raRecords[idx].totalContentionTimeouts = mac->GetTotalContentionTimeouts ();
        g_raRecords[idx].totalMsgBTimeouts       = mac->GetTotalMsgBTimeouts ();
    }

    Simulator::Destroy ();

    // ==================== 输出结果 ====================
    std::cout << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "          RA Performance Comparison" << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "  [UE级]  最终成功UE数/总UE数" << std::endl;
    std::cout << "  [发次级] 按你的定义: 收到有效Msg4次数/发送Msg1次数, 碰撞率=Msg4超时次数/发送Msg1次数" << std::endl;
    std::cout << std::endl;
    std::cout << "| " << std::setw (8) << "Mode"
              << " | " << std::setw (6) << "UE数"
              << " | " << std::setw (7) << "成功UE"
              << " | " << std::setw (9) << "UE成功率"
              << " | " << std::setw (7) << "发Msg1"
              << " | " << std::setw (7) << "收Msg4"
              << " | " << std::setw (11) << "碰撞超时次"
              << " | " << std::setw (13) << "接入成功率(*)"
              << " | " << std::setw (12) << "碰撞率(*)"
              << " | " << std::setw (8) << "Avg(ms)"
              << " | " << std::setw (8) << "P50(ms)"
              << " | " << std::setw (8) << "P95(ms)"
              << " |" << std::endl;
    std::cout << "|----------|--------|---------|-----------|---------|---------|-------------|---------------|-------------|----------|----------|----------|" << std::endl;

    std::vector<RaSummary> summaries;
    for (const auto& mode : modes) {
        RaSummary s = ComputeSummary (mode);
        summaries.push_back (s);
        PrintSummary (s);
    }
    std::cout << "===========================================" << std::endl;

    // 逐波束统计
    std::cout << std::endl;
    std::cout << "=== Per-Beam Success Rate ===" << std::endl;
    std::cout << "| " << std::setw (6) << "Beam";
    for (const auto& mode : modes) {
        std::cout << " | " << std::setw (16) << mode;
    }
    std::cout << " |" << std::endl;
    std::cout << "|--------";
    for (size_t i = 0; i < modes.size (); i++) {
        std::cout << "|------------------";
    }
    std::cout << "|" << std::endl;

    for (uint32_t b = 1; b <= numBeams; b++) {
        std::cout << "| " << std::setw (6) << b;
        for (const auto& mode : modes) {
            uint32_t total = 0, success = 0;
            for (const auto& r : g_raRecords) {
                if (r.beamId == b && r.rachMode == mode) {
                    total++;
                    if (r.success) success++;
                }
            }
            double rate = total > 0 ? (double)success / total * 100.0 : 0;
            std::ostringstream ss;
            ss << success << "/" << total << " ("
               << std::fixed << std::setprecision (1) << rate << "%)";
            std::cout << " | " << std::setw (16) << ss.str ();
        }
        std::cout << " |" << std::endl;
    }
    std::cout << "=============================" << std::endl;

    // ==================== 逐次尝试成功率 ====================
    // 逻辑: 若 UE 最终 totalMsg1Sent=N 且 success=true, 则第 1..N-1 次失败, 第 N 次成功
    //        若 success=false, 则所有 N 次都失败
    // 第 k 次参与者 = totalMsg1Sent >= k 的 UE 数
    // 第 k 次成功者 = totalMsg1Sent == k 且 success 的 UE 数
    std::cout << std::endl;
    std::cout << "=== Per-Attempt Success Rate ===" << std::endl;
    std::cout << "| " << std::setw (7) << "Attempt";
    for (const auto& mode : modes) {
        std::cout << " | " << std::setw (12) << (mode + " 参与")
                  << " | " << std::setw (12) << (mode + " 成功")
                  << " | " << std::setw (10) << (mode + " 率");
    }
    std::cout << " |" << std::endl;

    // 分隔线
    std::cout << "|--------";
    for (size_t i = 0; i < modes.size (); i++) {
        std::cout << "|--------------|--------------|-----------";
    }
    std::cout << "|" << std::endl;

    for (uint32_t attempt = 1; attempt <= maxRaAttempts; attempt++) {
        std::cout << "| " << std::setw (7) << attempt;
        for (const auto& mode : modes) {
            uint32_t participants = 0;  // totalMsg1Sent >= attempt
            uint32_t successes = 0;     // totalMsg1Sent == attempt && success
            for (const auto& r : g_raRecords) {
                if (r.rachMode != mode) continue;
                if (r.totalMsg1Sent >= attempt) {
                    participants++;
                }
                if (r.totalMsg1Sent == attempt && r.success) {
                    successes++;
                }
            }
            double rate = participants > 0
                ? (double)successes / participants * 100.0 : 0;
            std::cout << " | " << std::setw (12) << participants
                      << " | " << std::setw (12) << successes
                      << " | " << std::setw (8) << std::fixed
                      << std::setprecision (1) << rate << "%";
        }
        std::cout << " |" << std::endl;
    }
    std::cout << "=================================" << std::endl;

    // 写入文件
    mkdir ("ntn-results", 0755);
    {
        std::ofstream f ("ntn-results/ra_compare.txt");
        f << "Mode,Beams,UesPerBeam,Total,Success,Failed,SuccessRate%,AvgLatencyMs,MinLatencyMs,P50LatencyMs,P95LatencyMs,MaxLatencyMs" << std::endl;
        for (const auto& s : summaries) {
            f << s.mode << "," << numBeams << "," << numUesPerBeam
              << "," << s.totalUes << "," << s.successCount << "," << s.failedCount
              << "," << std::fixed << std::setprecision (2) << s.ueSuccessRate
              << "," << s.avgLatencyMs << "," << s.minLatencyMs
              << "," << s.p50LatencyMs << "," << s.p95LatencyMs
              << "," << s.maxLatencyMs << std::endl;
        }
        f.close ();
    }

    // 逐 UE 明细
    {
        std::ofstream f ("ntn-results/ra_compare_detail.txt");
        f << "Index,Beam,PreambleId,Mode,Success,StartTime_s,EndTime_s,Latency_ms" << std::endl;
        for (const auto& r : g_raRecords) {
            double lat = r.success ? (r.endTime - r.startTime).GetMilliSeconds () : -1;
            f << r.ueIndex << "," << r.beamId << "," << r.preambleId << "," << r.rachMode
              << "," << (r.success ? 1 : 0)
              << "," << std::fixed << std::setprecision (6) << r.startTime.GetSeconds ()
              << "," << r.endTime.GetSeconds ()
              << "," << lat << std::endl;
        }
        f.close ();
    }

    std::cout << std::endl;
    std::cout << "Results saved to:" << std::endl;
    std::cout << "  ntn-results/ra_compare.txt        (summary)" << std::endl;
    std::cout << "  ntn-results/ra_compare_detail.txt  (per-UE detail)" << std::endl;

    return 0;
}
