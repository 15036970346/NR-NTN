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
    // 位置 / 卫星仰角链路 (位置分布模型启用时填充)
    double posX;                      // UE 位置 x (米, 本地 ENU)
    double posY;                      // UE 位置 y (米)
    double elevDeg;                   // 对卫星的仰角 (度)
    double offAxisDeg;                // 相对波束 boresight 的偏轴角 (度)
    double prachSnrDb;                // 解析计算的 per-UE PRACH SNR (dB)
};

static std::vector<UeRaRecord> g_raRecords;
// 保存 (recordIndex, SatUtMac*) 用于仿真结束后读取计数器
static std::vector<std::pair<uint32_t, Ptr<SatUtMac>>> g_utMacRefs;
static std::vector<std::pair<std::string, Ptr<GeoBeamScheduler>>> g_schedulerRefs;

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
         uint32_t numPreambles = 64,
         uint32_t raResponseWindowMs = 1600,
         uint32_t contentionResolutionMs = 1500)
{
    utMac->SwitchState (SatUtMac::MAC_IDLE);
    utMac->SetUeIdentity (ueIdentity);
    utMac->SetRaTimers (MilliSeconds (raResponseWindowMs),
                        MilliSeconds (contentionResolutionMs),
                        maxRaAttempts);
    utMac->SetNumPreambles (numPreambles);

    // 4 步回调 (所有 UE 都需要, 2 步 FALLBACK 会复用)
    utMac->SetPrachCallback (MakeCallback (&GeoBeamScheduler::ReceivePrachPreamble, scheduler));
    utMac->SetMsg3Callback (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));
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

struct PrachDetectionSummary {
    std::string mode;
    PrachDetectionStats stats;
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

static PrachDetectionSummary
ComputePrachDetectionSummary (const std::string& mode)
{
    PrachDetectionSummary summary;
    summary.mode = mode;
    for (const auto& ref : g_schedulerRefs) {
        if (ref.first != mode) continue;
        PrachDetectionStats s = ref.second->GetPrachDetectionStats ();
        summary.stats.activePreambleGroups += s.activePreambleGroups;
        summary.stats.detectedGroups += s.detectedGroups;
        summary.stats.missedGroups += s.missedGroups;
        summary.stats.missedUeAttempts += s.missedUeAttempts;
        summary.stats.falseAlarmGroups += s.falseAlarmGroups;
        summary.stats.falseAlarmMsg3GrantRbs += s.falseAlarmMsg3GrantRbs;
    }
    return summary;
}

// ==================== 几何与链路预算 (用户位置分布 + 卫星仰角) ====================
// 说明: ntn-ra-compare 是 MAC 层接入仿真, 不挂完整 PHY/Spectrum 栈。
// 这里用解析方式复现 SatChannel 的同款物理模型:
//   - FSPL 公式与 SatChannel::CalcFspl 完全一致
//   - 抛物面天线增益滚降公式 G=Gmax-12(θ/θ3dB)^2 与 SatChannel::CalcBeamGain 一致
//   - 仰角由 UE→卫星 几何 atan 求出 (与 SatChannel::GetThreeGppTable 思路一致)
// 据此为每个 UE 算出 per-UE PRACH SNR, 经 SatUtMac::SetPrachSnrDb 注入接入流程。

struct BeamGeometry {
    std::vector<Vector> beamCenters;  // 各波束中心 (本地 ENU, z=0, 米)
    Vector satPosition;               // 卫星位置 (本地 ENU, 米)
    double beamRadiusM;               // 波束半径 (米)
    double nominalSlantM;             // 簇中心→卫星 的标称斜距 (米)
    double theta3dbRad;               // 天线 3dB 波束宽度 (弧度)
    double maxGainDbi;                // 主瓣峰值增益
    double sideLobeDbi;               // 旁瓣增益地板
    double freqHz;
    double ueEirpDbm;
    double atmosphericLossDb;
    double noiseFloorDbm;             // 热噪声 = -174 + 10log10(B) + NF
};

struct UeLinkResult { double elevDeg; double offAxisDeg; double snrDb; };

// 7 波束六边形布局的波束中心 (与 ntn-system-sim 一致: 间距 = 1.5*半径)
static std::vector<Vector>
BuildBeamCenters (uint32_t numBeams, double beamRadiusM)
{
    std::vector<Vector> centers;
    centers.push_back (Vector (0.0, 0.0, 0.0));    // 中心波束
    double dist = beamRadiusM * 1.5;
    for (uint32_t i = 1; i < numBeams; i++) {
        double angle = (i - 1) * 60.0 * M_PI / 180.0;
        centers.push_back (Vector (dist * std::cos (angle), dist * std::sin (angle), 0.0));
    }
    return centers;
}

// 由仰角和卫星高度求斜距 (球面地球几何)
static double
SlantRange (double elevDeg, double satAltitudeM)
{
    const double Re = 6371000.0;          // 地球半径 (米)
    double Rs = Re + satAltitudeM;        // 轨道半径
    double el = elevDeg * M_PI / 180.0;
    double ratio = Rs / Re;
    return Re * (std::sqrt (ratio * ratio - std::cos (el) * std::cos (el)) - std::sin (el));
}

// FSPL(dB) = 20log10(d) + 20log10(f) - 147.55  (与 SatChannel::CalcFspl 一致)
static double
Fspl (double distM, double freqHz)
{
    if (distM < 1.0) distM = 1.0;
    return 20.0 * std::log10 (distM) + 20.0 * std::log10 (freqHz) - 147.55;
}

// 在波束圆盘内按面积均匀采样一个位置 (rho 用 sqrt 保证面积均匀)
static Vector
SampleUniformInDisc (Ptr<UniformRandomVariable> rng, const Vector& center, double radiusM)
{
    double rho = radiusM * std::sqrt (rng->GetValue (0.0, 1.0));
    double phi = rng->GetValue (0.0, 2.0 * M_PI);
    return Vector (center.x + rho * std::cos (phi),
                   center.y + rho * std::sin (phi),
                   0.0);
}

// 计算某 UE 的 (仰角, 偏轴角, PRACH SNR)。beamCenter = 该 UE 所属波束中心 (天线 boresight)
static UeLinkResult
ComputeUeLink (const BeamGeometry& geo, const Vector& uePos, const Vector& beamCenter)
{
    UeLinkResult r;
    // 1. UE → 卫星 矢量, 斜距, 仰角
    Vector toSat (geo.satPosition.x - uePos.x,
                  geo.satPosition.y - uePos.y,
                  geo.satPosition.z - uePos.z);
    double dist  = std::sqrt (toSat.x*toSat.x + toSat.y*toSat.y + toSat.z*toSat.z);
    double horiz = std::sqrt (toSat.x*toSat.x + toSat.y*toSat.y);
    r.elevDeg = std::atan2 (toSat.z, horiz) * 180.0 / M_PI;

    // 2. 偏轴角: 卫星→波束中心 (boresight) 与 卫星→UE 的夹角
    Vector satToBeam (beamCenter.x - geo.satPosition.x,
                      beamCenter.y - geo.satPosition.y,
                      beamCenter.z - geo.satPosition.z);
    Vector satToUe  (uePos.x - geo.satPosition.x,
                     uePos.y - geo.satPosition.y,
                     uePos.z - geo.satPosition.z);
    double dotp = satToBeam.x*satToUe.x + satToBeam.y*satToUe.y + satToBeam.z*satToUe.z;
    double nb = std::sqrt (satToBeam.x*satToBeam.x + satToBeam.y*satToBeam.y + satToBeam.z*satToBeam.z);
    double nu = std::sqrt (satToUe.x*satToUe.x + satToUe.y*satToUe.y + satToUe.z*satToUe.z);
    double cosTheta = (nb > 0 && nu > 0)
        ? std::max (-1.0, std::min (1.0, dotp / (nb * nu))) : 1.0;
    double theta = std::acos (cosTheta);       // 偏轴角 (弧度)
    r.offAxisDeg = theta * 180.0 / M_PI;

    // 3. 抛物面天线增益滚降 (与 SatChannel::CalcBeamGain 同式)
    double gainDbi = geo.maxGainDbi - 12.0 * std::pow (theta / geo.theta3dbRad, 2.0);
    gainDbi = std::max (gainDbi, geo.sideLobeDbi);

    // 4. 链路预算 → 接收功率 → SNR
    double fsplDb = Fspl (dist, geo.freqHz);
    double rxPowerDbm = geo.ueEirpDbm + gainDbi - fsplDb - geo.atmosphericLossDb;
    r.snrDb = rxPowerDbm - geo.noiseFloorDbm;
    return r;
}

// ==================== 内置 Pd/Pfa-SNR 曲线 (CFAR 能量检测器系统级近似) ====================
// Pd(SNR) = 0.5*erfc((snr50 - SNR)/(√2·slope)) —— 高斯型 ROC S 曲线
// Pfa 恒定 (CFAR: 门限按目标虚警率设定, 与 SNR 无关)
static double
DefaultCurvePd (double snrDb, double snr50Db, double slopeDb)
{
    return 0.5 * std::erfc ((snr50Db - snrDb) / (std::sqrt (2.0) * slopeDb));
}

// 生成解析曲线并写入 CSV (兼作绘图输入); 列: snr_db,pd,pfa
static bool
WriteDefaultCurve (const std::string& path, double pfa, double snr50Db, double slopeDb,
                   double snrMinDb, double snrMaxDb, double stepDb)
{
    std::ofstream f (path);
    if (!f.is_open ()) return false;
    f << "# Built-in CFAR energy-detector approx: "
         "Pd=0.5*erfc((snr50-snr)/(sqrt2*slope)), Pfa=const" << std::endl;
    f << "# snr50=" << snr50Db << "dB slope=" << slopeDb << "dB pfa=" << pfa << std::endl;
    f << "snr_db,pd,pfa" << std::endl;
    for (double s = snrMinDb; s <= snrMaxDb + 1e-9; s += stepDb) {
        f << std::fixed << std::setprecision (3) << s << ","
          << DefaultCurvePd (s, snr50Db, slopeDb) << ","
          << std::setprecision (6) << pfa << std::endl;
    }
    return true;
}

// ==================== main ====================

int main (int argc, char *argv[])
{
    // 预先扫描 --preset 以决定 dev / stress 两套默认值
    // dev    : 100 UE/beam, 2 RO, 1 PRACH channel (64 preambles)   ← 当前开发默认
    // stress : 3000 UE/beam, 20 RO, 1 PRACH channel (64 preambles) ← 老的大规模回归版
    std::string preset = "dev";
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.rfind ("--preset=", 0) == 0) {
            preset = arg.substr (std::string ("--preset=").size ());
        } else if (arg == "--preset" && i + 1 < argc) {
            preset = argv[++i];
        }
    }

    uint32_t numBeams = 7;
    uint32_t numUesPerBeam;
    uint32_t numRos;
    uint32_t numPrachChannels;
    if (preset == "stress") {
        numUesPerBeam     = 3000;
        numRos            = 20;
        numPrachChannels  = 1;   // 64 preambles/RO
    } else {
        // dev (默认): 小规模快速回归
        numUesPerBeam     = 150;
        numRos            = 2;
        numPrachChannels  = 1;   // 64 preambles/RO (= 1 个 6-RB PRACH 频域信道)
    }
    double roSpacingMs = 1.0;       // 相邻 RO 间隔 1ms
    uint32_t rachType = 2;          // 0=4step, 1=2step, 2=both (对比)
    double raWindowSec = 0.0001;    // 100μs — 强制每个 RO 内所有 UE 落入同一聚合窗口
    double simTime = 60.0;
    bool enableLogs = false;
    uint32_t maxRaAttempts = 6;     // 最大接入尝试次数 (含首次)
    uint32_t raResponseWindowMs = 1600;     // RAR 等待窗口 (GEO RTT 600ms + 余量)
    uint32_t contentionResolutionMs = 1500; // Msg4 等待窗口
    bool enablePrachDetectionErrors = true;   // 默认开启 Msg1 虚警/漏检仿真
    std::string prachDetectionCurveCsv = "";
    double defaultPrachSnrDb = 20.0;
    double prachDetectionPd = 1.0;
    double prachDetectionPfa = 0.0;

    // ---- 内置 Pd/Pfa-SNR 曲线 (CFAR 能量检测器系统级近似) ----
    // 默认开启检测且未提供 CSV 时, 用这条解析曲线, 使 per-UE 仰角/位置 SNR 真正驱动漏检。
    // Pd(SNR)=0.5*erfc((snr50-SNR)/(√2·slope)) 的 S 曲线; Pfa 恒定 (CFAR)。
    double prachCurvePfa = 1.0e-3;       // 目标虚警概率 (恒定)
    double prachCurveSnr50Db = -5.0;     // Pd=0.5 对应的 SNR
    double prachCurveSlopeDb = 2.0;      // S 曲线陡度 (越小越陡)

    // ---- 用户位置分布 + 卫星仰角链路 (per-UE PRACH SNR) ----
    bool enablePositionModel = true;     // 启用后用解析链路 SNR 覆盖固定 defaultPrachSnrDb
    double satAltitudeKm = 35786.0;      // GEO 高度
    double nominalElevDeg = 45.0;        // 波束簇基准仰角
    double beamRadiusKm = 200.0;         // 单波束地面覆盖半径
    double carrierFreqGHz = 2.0;         // S 波段
    double ueEirpDbm = 33.0;             // UE 上行 EIRP
    double satMaxGainDbi = 45.0;         // 卫星波束主瓣峰值增益
    double satBeam3dbDeg = 0.0;          // 天线 3dB 波束宽度; <=0 按波束半径自动推导(边缘≈-3dB)
    double atmosphericLossDb = 0.5;      // 大气/其它固定损耗
    double noiseFigureDb = 2.0;          // 卫星接收噪声系数
    double prachBandwidthHz = 1.08e6;    // PRACH 占用带宽 (热噪声计算)

    CommandLine cmd;
    cmd.AddValue ("preset", "Scale preset: dev (100 UE/beam, 2 RO, 128 preambles) | stress (3000 UE/beam, 20 RO, 64 preambles)", preset);
    cmd.AddValue ("numBeams", "Number of beams (default: 7)", numBeams);
    cmd.AddValue ("numUesPerBeam", "UEs per beam (override preset)", numUesPerBeam);
    cmd.AddValue ("numRos", "Number of ROs per mode (override preset)", numRos);
    cmd.AddValue ("roSpacingMs", "RO spacing in ms (default: 1)", roSpacingMs);
    cmd.AddValue ("rachType", "0=4-step only, 1=2-step only, 2=both (compare)", rachType);
    cmd.AddValue ("raWindow", "RA initiation window per RO in seconds (0.0001=burst)", raWindowSec);
    cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue ("enableLogs", "Enable detailed MAC/scheduler logs", enableLogs);
    cmd.AddValue ("maxRaAttempts", "Max RA attempts per UE (default: 5)", maxRaAttempts);
    cmd.AddValue ("raResponseWindowMs", "RAR window in ms (default: 1600)", raResponseWindowMs);
    cmd.AddValue ("contentionResolutionMs", "Msg4 contention resolution window in ms (default: 1500)", contentionResolutionMs);
    cmd.AddValue ("numPrachChannels", "PRACH freq channels per RO (1=64, 2=128 preambles; override preset)", numPrachChannels);
    cmd.AddValue ("enablePrachDetectionErrors", "Enable system-level Msg1 PRACH missed-detection/false-alarm model", enablePrachDetectionErrors);
    cmd.AddValue ("prachDetectionCurveCsv", "CSV curve path with columns snr_db,pd,pfa. Empty uses fixed probabilities.", prachDetectionCurveCsv);
    cmd.AddValue ("defaultPrachSnrDb", "Default PRACH SNR in dB when UE PHY has no explicit PRACH SNR", defaultPrachSnrDb);
    cmd.AddValue ("prachDetectionPd", "Fixed active-preamble detection probability used when no CSV is supplied", prachDetectionPd);
    cmd.AddValue ("prachDetectionPfa", "Fixed empty-preamble false-alarm probability used when no CSV is supplied", prachDetectionPfa);
    cmd.AddValue ("enablePositionModel", "Place UEs uniformly in each beam disc and derive per-UE PRACH SNR from satellite elevation + beam-gain rolloff", enablePositionModel);
    cmd.AddValue ("satAltitudeKm", "Satellite altitude in km (GEO=35786)", satAltitudeKm);
    cmd.AddValue ("nominalElevDeg", "Nominal beam-cluster elevation angle in degrees", nominalElevDeg);
    cmd.AddValue ("beamRadiusKm", "Per-beam ground coverage radius in km", beamRadiusKm);
    cmd.AddValue ("carrierFreqGHz", "Carrier frequency in GHz (S-band ~2, Ka-band ~20)", carrierFreqGHz);
    cmd.AddValue ("ueEirpDbm", "UE uplink EIRP in dBm", ueEirpDbm);
    cmd.AddValue ("satMaxGainDbi", "Satellite beam peak (boresight) gain in dBi", satMaxGainDbi);
    cmd.AddValue ("satBeam3dbDeg", "Antenna 3dB beamwidth in deg (<=0: auto from beam radius)", satBeam3dbDeg);
    cmd.AddValue ("atmosphericLossDb", "Fixed atmospheric/other loss in dB", atmosphericLossDb);
    cmd.AddValue ("noiseFigureDb", "Satellite receiver noise figure in dB", noiseFigureDb);
    cmd.AddValue ("prachBandwidthHz", "PRACH bandwidth in Hz for thermal-noise calc", prachBandwidthHz);
    cmd.AddValue ("prachCurvePfa", "Built-in curve: constant false-alarm probability (CFAR)", prachCurvePfa);
    cmd.AddValue ("prachCurveSnr50Db", "Built-in curve: SNR (dB) at which Pd=0.5", prachCurveSnr50Db);
    cmd.AddValue ("prachCurveSlopeDb", "Built-in curve: S-curve slope in dB (smaller=steeper)", prachCurveSlopeDb);
    cmd.Parse (argc, argv);

    if (enableLogs) {
        LogComponentEnable ("NtnRaCompare", LOG_LEVEL_INFO);
        LogComponentEnable ("SatUtMac", LOG_LEVEL_INFO);
        LogComponentEnable ("GeoBeamScheduler", LOG_LEVEL_INFO);
    }

    // 默认开启检测且用户未指定 CSV → 生成内置解析曲线并加载 (per-UE SNR 据此驱动漏检)
    mkdir ("ntn-results", 0755);
    bool usingBuiltinCurve = false;
    if (enablePrachDetectionErrors && prachDetectionCurveCsv.empty ()) {
        const std::string builtinCurvePath = "ntn-results/prach_detection_curve.txt";
        if (!WriteDefaultCurve (builtinCurvePath, prachCurvePfa,
                                prachCurveSnr50Db, prachCurveSlopeDb,
                                -20.0, 25.0, 1.0)) {
            NS_FATAL_ERROR ("无法写入内置 PRACH 检测曲线: " << builtinCurvePath);
        }
        prachDetectionCurveCsv = builtinCurvePath;
        usingBuiltinCurve = true;
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

    if (preset != "dev" && preset != "stress") {
        std::cerr << "[WARN] unknown --preset='" << preset
                  << "', falling back to 'dev'" << std::endl;
        preset = "dev";
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  NTN GEO Random Access Comparison" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Preset:           " << preset << std::endl;
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
    std::cout << "  RAR window:       " << raResponseWindowMs << " ms" << std::endl;
    std::cout << "  Contention timer: " << contentionResolutionMs << " ms" << std::endl;
    std::cout << "  PRACH detect err: " << (enablePrachDetectionErrors ? "enabled" : "disabled")
              << " defaultSNR=" << defaultPrachSnrDb
              << " fixedPd=" << prachDetectionPd
              << " fixedPfa=" << prachDetectionPfa
              << (prachDetectionCurveCsv.empty () ? "" : (" csv=" + prachDetectionCurveCsv))
              << std::endl;
    std::cout << "  Modes:            ";
    for (const auto& m : modes) std::cout << m << " ";
    std::cout << std::endl;
    std::cout << "  Expected 1st-try collision/RO: ~"
              << std::fixed << std::setprecision (1) << expectedCollisionPct
              << "% of UEs" << std::endl;
    std::cout << "  (Birthday Problem: " << uesPerRo << " UEs/RO choosing from "
              << numPreambles << " preambles)" << std::endl;
    std::cout << "========================================" << std::endl;

    // ---- 构建几何 (用户位置分布 + 卫星仰角链路) ----
    BeamGeometry geo;
    geo.beamRadiusM       = beamRadiusKm * 1000.0;
    geo.freqHz            = carrierFreqGHz * 1.0e9;
    geo.maxGainDbi        = satMaxGainDbi;
    geo.sideLobeDbi       = -5.0;
    geo.ueEirpDbm         = ueEirpDbm;
    geo.atmosphericLossDb = atmosphericLossDb;
    geo.noiseFloorDbm     = -174.0 + 10.0 * std::log10 (prachBandwidthHz) + noiseFigureDb;
    geo.nominalSlantM     = SlantRange (nominalElevDeg, satAltitudeKm * 1000.0);
    geo.beamCenters       = BuildBeamCenters (numBeams, geo.beamRadiusM);
    // 卫星位置: 使簇中心(原点)以 nominalElevDeg 仰角、+x 方位看到卫星
    {
        double elr = nominalElevDeg * M_PI / 180.0;
        geo.satPosition = Vector (geo.nominalSlantM * std::cos (elr), 0.0,
                                  geo.nominalSlantM * std::sin (elr));
    }
    // 天线 3dB 波束宽度: 默认按波束半径推导, 使边缘 UE 增益约 -3dB (边缘偏轴角=θ3dB/2)
    if (satBeam3dbDeg > 0.0) {
        geo.theta3dbRad = satBeam3dbDeg * M_PI / 180.0;
    } else {
        geo.theta3dbRad = 2.0 * std::atan2 (geo.beamRadiusM, geo.nominalSlantM);
    }

    if (enablePositionModel) {
        // 打印基准 (波束中心) 与 边缘 UE 的链路, 供标定参考
        UeLinkResult boresight = ComputeUeLink (geo, geo.beamCenters[0], geo.beamCenters[0]);
        UeLinkResult edge = ComputeUeLink (geo,
            Vector (geo.beamCenters[0].x + geo.beamRadiusM, geo.beamCenters[0].y, 0.0),
            geo.beamCenters[0]);
        std::cout << "  --- Position / Elevation link model: ENABLED ---" << std::endl;
        std::cout << "  Sat altitude:     " << satAltitudeKm << " km" << std::endl;
        std::cout << "  Nominal elev:     " << nominalElevDeg << " deg"
                  << "  (slant range " << geo.nominalSlantM / 1000.0 << " km)" << std::endl;
        std::cout << "  Beam radius:      " << beamRadiusKm << " km" << std::endl;
        std::cout << "  Carrier:          " << carrierFreqGHz << " GHz"
                  << "  (FSPL@nominal " << Fspl (geo.nominalSlantM, geo.freqHz) << " dB)" << std::endl;
        std::cout << "  Ant 3dB width:    " << geo.theta3dbRad * 180.0 / M_PI << " deg" << std::endl;
        std::cout << "  Noise floor:      " << geo.noiseFloorDbm << " dBm"
                  << "  (B=" << prachBandwidthHz / 1e6 << " MHz, NF=" << noiseFigureDb << " dB)" << std::endl;
        std::cout << "  SNR @ boresight:  " << std::fixed << std::setprecision (2)
                  << boresight.snrDb << " dB  (elev " << boresight.elevDeg << " deg)" << std::endl;
        std::cout << "  SNR @ beam edge:  " << std::fixed << std::setprecision (2)
                  << edge.snrDb << " dB  (off-axis " << edge.offAxisDeg << " deg)" << std::endl;
        std::cout << "========================================" << std::endl;
    } else {
        std::cout << "  --- Position / Elevation link model: DISABLED (fixed SNR="
                  << defaultPrachSnrDb << " dB) ---" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();

    double batchOffset = 1.0;  // 从 1s 开始, 留出初始化时间

    // 每种模式用独立的调度器组 (避免状态污染)
    std::vector<Ptr<SatUtMac>> allUtMacs;

    // qyh ResourceManager 默认每波束 25 RB; 在高负载 4 步 RA 下会卡死 Msg3 分配
    // (ProcessPrachWindow 走 AllocateSpectrum, 预算耗尽就丢 RAR). 抬高预算。
    Config::SetDefault ("ns3::ResourceManager::UlBeamBudgetRbs", UintegerValue (275));
    Config::SetDefault ("ns3::ResourceManager::DlBeamBudgetRbs", UintegerValue (275));

    for (const auto& mode : modes) {
        RachType rt = (mode == "2-step") ? RachType::TWO_STEP : RachType::FOUR_STEP;

        // 每种模式新建独立的调度器
        std::vector<Ptr<GeoBeamScheduler>> schedulers (numBeams);
        for (uint32_t b = 0; b < numBeams; b++) {
            schedulers[b] = CreateObject<GeoBeamScheduler> ();
            schedulers[b]->Initialize (b + 1, 1);
            schedulers[b]->SetNumPrachPreambles (numPreambles);
            schedulers[b]->SetDefaultPrachSnrDb (defaultPrachSnrDb);
            schedulers[b]->SetPrachDetectionFixed (prachDetectionPd, prachDetectionPfa);
            if (!prachDetectionCurveCsv.empty ()) {
                bool curveLoaded = schedulers[b]->SetPrachDetectionCurveCsv (prachDetectionCurveCsv);
                if (!curveLoaded) {
                    NS_FATAL_ERROR ("Failed to load PRACH detection curve CSV: " << prachDetectionCurveCsv);
                }
            }
            schedulers[b]->EnablePrachDetectionErrors (enablePrachDetectionErrors);
            g_schedulerRefs.push_back ({mode, schedulers[b]});
            // 6.2 基准: AdmitControl/SetBeamTotalRbs 已从 GeoBeamScheduler 迁出 (移入 NtnGwRrc)。
            // ProcessPrachWindow 的 Msg3 资源门控走 scheduler 内部自建 ResourceManager 的
            // AllocateSpectrum, 其每波束 UL/DL RB 预算由上面的
            // Config::SetDefault("ns3::ResourceManager::{Ul,Dl}BeamBudgetRbs") 统一设置 (275 RB)。
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

                    // UE 位置: 在所属波束圆盘内均匀分布; 据此算仰角与 per-UE PRACH SNR
                    Vector uePos = SampleUniformInDisc (rng, geo.beamCenters[b], geo.beamRadiusM);
                    UeLinkResult link = ComputeUeLink (geo, uePos, geo.beamCenters[b]);

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
                    rec.posX = uePos.x;
                    rec.posY = uePos.y;
                    rec.elevDeg = link.elevDeg;
                    rec.offAxisDeg = link.offAxisDeg;
                    rec.prachSnrDb = enablePositionModel ? link.snrDb : defaultPrachSnrDb;
                    g_raRecords.push_back (rec);

                    Ptr<SatUtMac> utMac = CreateObject<SatUtMac> ();
                    allUtMacs.push_back (utMac);
                    g_utMacRefs.push_back ({globalIdx, utMac});
                    // 注入 per-UE PRACH SNR (覆盖调度器的固定 defaultPrachSnrDb)
                    if (enablePositionModel) {
                        utMac->SetPrachSnrDb (link.snrDb);
                    }

                    // 在 [roBase, roBase + raWindow] 内随机发起 (100μs 窗口)
                    double startSec = roBaseSec + rng->GetValue (0.0, raWindowSec);

                    uint64_t identity = ((uint64_t)(b + 1) << 40)
                                        | ((uint64_t)ueCounter << 16)
                                        | rng->GetInteger (0, 0xFFFF);
                    ueCounter++;

                    SetupUe (utMac, schedulers[b], globalIdx, rt, identity,
                             Seconds (startSec), preambleId,
                             static_cast<uint8_t> (maxRaAttempts),
                             numPreambles,
                             raResponseWindowMs,
                             contentionResolutionMs);
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
            if (enableLogs) {   // 逐 RO 明细仅在开日志时打印, 默认只给汇总
                std::cout << "[" << mode << "] Beam 1 RO#" << (ro + 1)
                          << " (" << roTotal << " UE): collision "
                          << roCollision << "/" << roTotal << " ("
                          << std::fixed << std::setprecision (1)
                          << (double)roCollision / roTotal * 100.0 << "%)" << std::endl;
            }
        }
        if (enableLogs) {
            std::cout << "[" << mode << "] Beam 1 collision preview: " << totalCollisionUes
                      << "/" << totalBeam1Ues << " UEs ("
                      << std::fixed << std::setprecision (1)
                      << (double)totalCollisionUes / totalBeam1Ues * 100.0 << "%)" << std::endl;
        }

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

    std::cout << std::endl;
    std::cout << "=== Msg1 PRACH Detection Error Summary ===" << std::endl;
    std::cout << "| " << std::setw (8) << "Mode"
              << " | " << std::setw (12) << "ActiveGrp"
              << " | " << std::setw (11) << "Detected"
              << " | " << std::setw (9) << "Missed"
              << " | " << std::setw (12) << "MissedUE"
              << " | " << std::setw (10) << "FalseAlm"
              << " | " << std::setw (14) << "FA_Msg3RB"
              << " |" << std::endl;
    std::cout << "|----------|--------------|-------------|-----------|--------------|------------|----------------|" << std::endl;
    for (const auto& mode : modes) {
        PrachDetectionSummary ds = ComputePrachDetectionSummary (mode);
        std::cout << "| " << std::setw (8) << ds.mode
                  << " | " << std::setw (12) << ds.stats.activePreambleGroups
                  << " | " << std::setw (11) << ds.stats.detectedGroups
                  << " | " << std::setw (9) << ds.stats.missedGroups
                  << " | " << std::setw (12) << ds.stats.missedUeAttempts
                  << " | " << std::setw (10) << ds.stats.falseAlarmGroups
                  << " | " << std::setw (14) << ds.stats.falseAlarmMsg3GrantRbs
                  << " |" << std::endl;
    }
    std::cout << "==========================================" << std::endl;

    // ==================== 检测曲线: 理论 vs 实测 对照 ====================
    if (enablePrachDetectionErrors) {
        // 跨所有波束/模式聚合按 SNR 分箱的实测统计
        std::map<int, uint64_t> aggActive, aggDetected;
        uint64_t aggFaTrials = 0, aggFaGroups = 0;
        for (const auto& ref : g_schedulerRefs) {
            PrachDetectionStats s = ref.second->GetPrachDetectionStats ();
            for (const auto& kv : s.activeGroupsBySnrBin)   aggActive[kv.first]   += kv.second;
            for (const auto& kv : s.detectedGroupsBySnrBin) aggDetected[kv.first] += kv.second;
            aggFaTrials += s.falseAlarmTrials;
            aggFaGroups += s.falseAlarmGroups;
        }

        // 写入实测文件 (绘图用): 每个 SNR 箱的实测 Pd, 并附理论 Pd
        {
            std::ofstream f ("ntn-results/prach_detection_empirical.txt");
            f << "snr_db,active_groups,detected_groups,empirical_pd,theoretical_pd" << std::endl;
            for (const auto& kv : aggActive) {
                int bin = kv.first;
                uint64_t act = kv.second;
                uint64_t det = aggDetected.count (bin) ? aggDetected.at (bin) : 0;
                double empPd = act > 0 ? (double) det / act : 0.0;
                double thyPd = usingBuiltinCurve
                    ? DefaultCurvePd (bin, prachCurveSnr50Db, prachCurveSlopeDb) : -1.0;
                f << bin << "," << act << "," << det << ","
                  << std::fixed << std::setprecision (4) << empPd << "," << thyPd << std::endl;
            }
            f.close ();
        }

        std::cout << std::endl;
        std::cout << "=== Pd Curve: Theoretical vs Empirical (binned by per-UE SNR) ===" << std::endl;
        if (usingBuiltinCurve) {
            std::cout << "  Built-in curve: Pd=0.5*erfc((snr50-SNR)/(sqrt2*slope))"
                      << "  snr50=" << prachCurveSnr50Db << "dB slope=" << prachCurveSlopeDb
                      << "dB  Pfa=" << std::scientific << std::setprecision (1) << prachCurvePfa
                      << std::fixed << " (const)" << std::endl;
        } else {
            std::cout << "  Curve from user CSV (theoretical column = n/a here)" << std::endl;
        }
        std::cout << "| " << std::setw (8) << "SNR(dB)"
                  << " | " << std::setw (10) << "ActiveGrp"
                  << " | " << std::setw (10) << "Detected"
                  << " | " << std::setw (12) << "Emp.Pd"
                  << " | " << std::setw (12) << "Theo.Pd"
                  << " |" << std::endl;
        std::cout << "|----------|------------|------------|--------------|--------------|" << std::endl;
        for (const auto& kv : aggActive) {
            int bin = kv.first;
            uint64_t act = kv.second;
            uint64_t det = aggDetected.count (bin) ? aggDetected.at (bin) : 0;
            double empPd = act > 0 ? (double) det / act : 0.0;
            std::cout << "| " << std::setw (8) << bin
                      << " | " << std::setw (10) << act
                      << " | " << std::setw (10) << det
                      << " | " << std::setw (12) << std::fixed << std::setprecision (4) << empPd
                      << " | " << std::setw (12);
            if (usingBuiltinCurve)
                std::cout << DefaultCurvePd (bin, prachCurveSnr50Db, prachCurveSlopeDb);
            else
                std::cout << "n/a";
            std::cout << " |" << std::endl;
        }
        // Pfa 对照 (空 preamble 检测; 实测分母 = falseAlarmTrials)
        double empPfa = aggFaTrials > 0 ? (double) aggFaGroups / aggFaTrials : 0.0;
        std::cout << "  Pfa: trials=" << aggFaTrials << " falseAlarms=" << aggFaGroups
                  << "  empirical=" << std::scientific << std::setprecision (2) << empPfa;
        if (usingBuiltinCurve)
            std::cout << "  theoretical=" << prachCurvePfa;
        std::cout << std::fixed << std::endl;
        std::cout << "  (curve -> ntn-results/prach_detection_curve.txt, "
                     "empirical -> ntn-results/prach_detection_empirical.txt)" << std::endl;
        std::cout << "================================================================" << std::endl;
    }

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

    // ==================== 位置/仰角/SNR 分布概览 ====================
    if (enablePositionModel && !g_raRecords.empty ()) {
        double snrMin = 1e9, snrMax = -1e9, snrSum = 0;
        double elMin = 1e9, elMax = -1e9, elSum = 0;
        for (const auto& r : g_raRecords) {
            snrMin = std::min (snrMin, r.prachSnrDb); snrMax = std::max (snrMax, r.prachSnrDb);
            snrSum += r.prachSnrDb;
            elMin = std::min (elMin, r.elevDeg); elMax = std::max (elMax, r.elevDeg);
            elSum += r.elevDeg;
        }
        size_t n = g_raRecords.size ();
        std::cout << std::endl;
        std::cout << "=== Per-UE Position / Elevation / SNR Distribution ===" << std::endl;
        std::cout << "  UEs placed (uniform in beam disc): " << n << std::endl;
        std::cout << "  Elevation (deg):  min=" << std::fixed << std::setprecision (3) << elMin
                  << "  mean=" << elSum / n << "  max=" << elMax << std::endl;
        std::cout << "  PRACH SNR (dB):   min=" << std::setprecision (2) << snrMin
                  << "  mean=" << snrSum / n << "  max=" << snrMax << std::endl;
        std::cout << "======================================================" << std::endl;
    }

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
        f << "Index,Beam,PreambleId,Mode,Success,StartTime_s,EndTime_s,Latency_ms,"
             "PosX_m,PosY_m,ElevDeg,OffAxisDeg,PrachSnrDb" << std::endl;
        for (const auto& r : g_raRecords) {
            double lat = r.success ? (r.endTime - r.startTime).GetMilliSeconds () : -1;
            f << r.ueIndex << "," << r.beamId << "," << r.preambleId << "," << r.rachMode
              << "," << (r.success ? 1 : 0)
              << "," << std::fixed << std::setprecision (6) << r.startTime.GetSeconds ()
              << "," << r.endTime.GetSeconds ()
              << "," << lat
              << "," << std::setprecision (1) << r.posX << "," << r.posY
              << "," << std::setprecision (3) << r.elevDeg << "," << r.offAxisDeg
              << "," << r.prachSnrDb << std::endl;
        }
        f.close ();
    }

    std::cout << std::endl;
    std::cout << "Results saved to:" << std::endl;
    std::cout << "  ntn-results/ra_compare.txt        (summary)" << std::endl;
    std::cout << "  ntn-results/ra_compare_detail.txt  (per-UE detail)" << std::endl;

    return 0;
}
