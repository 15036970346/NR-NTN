/*
 * 文件路径：contrib/geo-sat/model/sat-ut-phy.cc
 * 功能：用户终端物理层实现 (支持 C/I 干扰计算与测量反馈)
 */
#include "sat-ut-phy.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatUtPhy");
NS_OBJECT_ENSURE_REGISTERED (SatUtPhy);

TypeId SatUtPhy::GetTypeId (void) {
    static TypeId tid = TypeId ("ns3::SatUtPhy")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatUtPhy> ()
        .AddAttribute ("Bandwidth", "System bandwidth in Hz",
                       DoubleValue (35e6),  // 修正为35MHz
                       MakeDoubleAccessor (&SatUtPhy::m_bandwidthHz),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("NoiseFigure", "Receiver noise figure in dB",
                       DoubleValue (5.0),
                       MakeDoubleAccessor (&SatUtPhy::m_noiseFigure),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("AntennaGain", "Antenna gain in dBi",
                       DoubleValue (0.0),
                       MakeDoubleAccessor (&SatUtPhy::m_antennaGainDbi),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("TraceFilePath", "Path to interference trace data",
                       StringValue ("contrib/geo-sat/data/interference_trace.txt"),
                       MakeStringAccessor (&SatUtPhy::m_traceFilePath),
                       MakeStringChecker ())
        .AddAttribute ("EsssaNumSlots", "Number of slots for ESSA",
                       UintegerValue (16),
                       MakeUintegerAccessor (&SatUtPhy::m_essaNumSlots),
                       MakeUintegerChecker<uint32_t> ())
        .AddTraceSource ("RsrpTrace",
                         "RSRP measurement trace (rnti, rsrp_dBm)",
                         MakeTraceSourceAccessor (&SatUtPhy::m_rsrpTrace),
                         "ns3::TracedCallback::UintDouble")
        .AddTraceSource ("SinrTrace",
                         "SINR measurement trace (rnti, sinr_dB)",
                         MakeTraceSourceAccessor (&SatUtPhy::m_sinrTrace),
                         "ns3::TracedCallback::UintDouble")
        .AddTraceSource ("RsrqTrace",
                         "RSRQ measurement trace (rnti, rsrq_dB)",
                         MakeTraceSourceAccessor (&SatUtPhy::m_rsrqTrace),
                         "ns3::TracedCallback::UintDouble");
    return tid;
}

SatUtPhy::SatUtPhy ()
    : m_macSapUser (nullptr),
      m_rnti (1),
      m_currentBeamId (1),
      m_lastCalculatedSinr (0.0),
      m_bandwidthHz (35e6),
      m_noiseFigure (5.0),
      m_antennaGainDbi (0.0),
      m_preambleId (0),
      m_prachFormat (0),
      m_timingAdvance (MicroSeconds (0)),
      m_maMode (MultipleAccessMode::ESSA),
      m_essaNumSlots (16),
      m_lastRsrq (0.0)
{
    NS_LOG_FUNCTION (this);
}

SatUtPhy::~SatUtPhy () {}

void SatUtPhy::SetMacPhySapUser (SatMacPhySapUser* sapUser) {
    m_macSapUser = sapUser;
}

void SatUtPhy::SetCqiReportCallback (Callback<void, uint8_t> callback) {
    m_cqiReportCallback = callback;
}

// ==============================================================
// 干扰计算辅助函数
// ==============================================================

double SatUtPhy::GetInterferenceFromTrace (uint32_t beamId) {
    double mockedInterferenceDbm = -90.0 + (beamId % 5); 
    NS_LOG_DEBUG ("Read C/I trace for Beam " << beamId << ": " << mockedInterferenceDbm << " dBm");
    return mockedInterferenceDbm;
}

double SatUtPhy::GetCoChannelInterference (uint32_t beamId) {
    // 共道干扰: 来自使用相同频率的其他波束的干扰
    // GEO卫星场景下，同频波束复用会产生共道干扰
    // 模拟: -85dBm ~ -100dBm，取决于频率复用距离
    double interferenceDbm = -90.0 + (beamId % 7) * 2.0;
    
    // 如果有多个活跃干扰源，功率叠加
    uint32_t numInterferers = 3; // 假设3个主要共道干扰源
    double totalInterferenceMw = 0.0;
    for (uint32_t i = 0; i < numInterferers; ++i) {
        double interfDbm = interferenceDbm - i * 3.0; // 每个干扰源递减3dB
        totalInterferenceMw += std::pow(10.0, interfDbm / 10.0);
    }
    
    double totalInterferenceDbm = 10.0 * std::log10(totalInterferenceMw);
    NS_LOG_DEBUG ("Co-channel interference for Beam " << beamId << ": " 
                  << totalInterferenceDbm << " dBm (" << numInterferers << " sources)");
    return totalInterferenceDbm;
}

double SatUtPhy::GetAdjacentChannelInterference (uint32_t beamId) {
    // 临道干扰: 来自相邻频率资源的干扰
    // 通常比共道干扰小15-30dB
    double caciDbm = -100.0 + (beamId % 5);
    NS_LOG_DEBUG ("Adjacent channel interference for Beam " << beamId << ": " << caciDbm << " dBm");
    return caciDbm;
}

double SatUtPhy::CalculateThermalNoise () {
    // 热噪声计算: N = k * T * B
    // k = 1.380649e-23 J/K (玻尔兹曼常数)
    // T = 290 K (标准温度)
    // B = 带宽 (Hz)
    double k = 1.380649e-23;
    double T = 290.0;
    double noisePowerW = k * T * m_bandwidthHz;
    double noisePowerDbm = 10.0 * std::log10(noisePowerW) + 30.0;
    
    // 加上噪声系数
    noisePowerDbm += m_noiseFigure;
    
    return noisePowerDbm;
}

// ==============================================================
// 增强的 SINR 计算 (C/I + 系统干扰)
// ==============================================================

double SatUtPhy::CalculateSinr (double rxPowerDbm, uint32_t beamId) {
    return CalculateComprehensiveSinr (rxPowerDbm, beamId);
}

double SatUtPhy::CalculateComprehensiveSinr (double rxPowerDbm, uint32_t beamId) {
    m_lastRsrp = rxPowerDbm;   // 保存 RSRP
    // 完整的SINR计算: SINR = C / (I_cc + I_adj + N)
    // C: 期望信号功率
    // I_cc: 共道干扰 (Co-channel interference)
    // I_adj: 临道干扰 (Adjacent channel interference)  
    // N: 热噪声
    
    NS_LOG_FUNCTION (this << rxPowerDbm << beamId);
    
    // 1. 获取各干扰分量
    double coChannelInterferenceDbm = GetCoChannelInterference (beamId);
    double adjacentInterferenceDbm = GetAdjacentChannelInterference (beamId);
    double thermalNoiseDbm = CalculateThermalNoise ();
    
    // 2. dBm转线性功率 (mW)
    double cMw = std::pow(10.0, rxPowerDbm / 10.0);
    double iCcMw = std::pow(10.0, coChannelInterferenceDbm / 10.0);
    double iAdjMw = std::pow(10.0, adjacentInterferenceDbm / 10.0);
    double nMw = std::pow(10.0, thermalNoiseDbm / 10.0);
    
    // 3. 总干扰+噪声
    double totalInterferenceAndNoiseMw = iCcMw + iAdjMw + nMw;
    
    // 4. 计算SINR
    double sinrLinear = cMw / totalInterferenceAndNoiseMw;
    double sinrDb = 10.0 * std::log10(sinrLinear);
    
    m_lastCalculatedSinr = sinrDb;

    // 计算 RSRQ: N * RSRP / RSSI, 其中 RSSI = C + I_cc + I_adj + N
    // N = 1 (单个 RB 测量), 简化为 RSRQ = RSRP / RSSI
    double rssiMw = cMw + iCcMw + iAdjMw + nMw;
    double rsrqLinear = cMw / rssiMw;
    m_lastRsrq = 10.0 * std::log10 (rsrqLinear);

    // 触发 PHY 层统计 TracedCallbacks
    m_rsrpTrace (m_rnti, rxPowerDbm);
    m_sinrTrace (m_rnti, sinrDb);
    m_rsrqTrace (m_rnti, m_lastRsrq);

    // 5. 日志输出各分量
    NS_LOG_INFO ("=== Comprehensive SINR Calculation ===");
    NS_LOG_INFO ("Signal (C): " << rxPowerDbm << " dBm");
    NS_LOG_INFO ("Co-channel Interference (I_cc): " << coChannelInterferenceDbm << " dBm");
    NS_LOG_INFO ("Adjacent Interference (I_adj): " << adjacentInterferenceDbm << " dBm");
    NS_LOG_INFO ("Thermal Noise (N): " << thermalNoiseDbm << " dBm");
    NS_LOG_INFO ("Total I+N: " << 10.0 * std::log10(totalInterferenceAndNoiseMw) << " dBm");
    NS_LOG_INFO ("SINR: " << sinrDb << " dB");
    NS_LOG_INFO ("C/I: " << (rxPowerDbm - coChannelInterferenceDbm) << " dB");
    
    return sinrDb;
}

double SatUtPhy::CalculateCarrierToInterferenceRatio (double rxPowerDbm, uint32_t beamId) {
    // C/I = 信号功率 / 共道干扰功率
    double coChannelInterferenceDbm = GetCoChannelInterference (beamId);
    double c_i_Db = rxPowerDbm - coChannelInterferenceDbm;
    NS_LOG_INFO ("C/I Ratio: " << c_i_Db << " dB");
    return c_i_Db;
}

double SatUtPhy::CalculateCarrierToInterferencePlusNoise (double rxPowerDbm, uint32_t beamId) {
    // C/(I+N) = 信号功率 / (共道干扰 + 热噪声)
    double cMw = std::pow(10.0, rxPowerDbm / 10.0);
    double iCcMw = std::pow(10.0, GetCoChannelInterference (beamId) / 10.0);
    double nMw = std::pow(10.0, CalculateThermalNoise () / 10.0);
    
    double cinMw = cMw / (iCcMw + nMw);
    double cinDb = 10.0 * std::log10(cinMw);
    
    NS_LOG_INFO ("C/(I+N): " << cinDb << " dB");
    return cinDb;
}

uint8_t SatUtPhy::MapSinrToCqi (double sinrDb) {
    // 3GPP TS 38.214 规定的CQI映射表
    // 基于SINR (dB) 映射到 1-15 的CQI索引
    if (sinrDb < -6.5) return 1;   // SINR < -6.5dB
    if (sinrDb < -5.0) return 2;   // -6.5 <= SINR < -5.0dB
    if (sinrDb < -3.0) return 3;   // -5.0 <= SINR < -3.0dB
    if (sinrDb < -1.0) return 4;   // -3.0 <= SINR < -1.0dB
    if (sinrDb < 1.0) return 5;    // -1.0 <= SINR < 1.0dB
    if (sinrDb < 3.0) return 6;    // 1.0 <= SINR < 3.0dB
    if (sinrDb < 5.0) return 7;    // 3.0 <= SINR < 5.0dB
    if (sinrDb < 7.0) return 8;    // 5.0 <= SINR < 7.0dB
    if (sinrDb < 9.0) return 9;    // 7.0 <= SINR < 9.0dB
    if (sinrDb < 11.0) return 10;  // 9.0 <= SINR < 11.0dB
    if (sinrDb < 13.0) return 11;  // 11.0 <= SINR < 13.0dB
    if (sinrDb < 15.0) return 12;  // 13.0 <= SINR < 15.0dB
    if (sinrDb < 17.0) return 13;  // 15.0 <= SINR < 17.0dB
    if (sinrDb < 19.0) return 14;  // 17.0 <= SINR < 19.0dB
    return 15;                    // SINR >= 19.0dB
}

void SatUtPhy::TriggerCqiReport () {
    NS_LOG_FUNCTION (this);
    
    uint8_t cqi = MapSinrToCqi (m_lastCalculatedSinr);
    
    NS_LOG_INFO ("CQI Report Triggered! CQI=" << (uint32_t)cqi 
                 << " (Based on SINR=" << m_lastCalculatedSinr << " dB)");
    
    if (!m_cqiReportCallback.IsNull ()) {
        m_cqiReportCallback (cqi);
    }
}

double SatUtPhy::GetLastCalculatedSinr () const {
    return m_lastCalculatedSinr;
}

double SatUtPhy::GetLastRsrp () const {
    return m_lastRsrp;
}

double SatUtPhy::GetLastRsrq () const {
    return m_lastRsrq;
}

double SatUtPhy::CalculateRsrq (double rsrpDbm, uint32_t beamId) {
    // RSRQ = 10*log10(RSRP / RSSI)
    // RSSI = 期望信号 + 共道干扰 + 临道干扰 + 热噪声
    double cMw    = std::pow (10.0, rsrpDbm / 10.0);
    double iCcMw  = std::pow (10.0, GetCoChannelInterference (beamId) / 10.0);
    double iAdjMw = std::pow (10.0, GetAdjacentChannelInterference (beamId) / 10.0);
    double nMw    = std::pow (10.0, CalculateThermalNoise () / 10.0);
    double rssiMw = cMw + iCcMw + iAdjMw + nMw;
    double rsrqDb = 10.0 * std::log10 (cMw / rssiMw);
    m_lastRsrq = rsrqDb;
    return rsrqDb;
}

// ==============================================================
// 随机接入与定时补偿 (DoRandomAccess)
// ==============================================================

void SatUtPhy::DoRandomAccess (uint32_t preambleId, uint8_t format) {
    NS_LOG_FUNCTION (this << preambleId << (uint32_t)format);
    
    // 设置PRACH参数
    m_preambleId = preambleId;
    m_prachFormat = format;
    m_raTransmissionTime = Simulator::Now ();
    
    NS_LOG_INFO ("=== DoRandomAccess Started ===");
    NS_LOG_INFO ("Preamble ID: " << preambleId << ", Format: " << (uint32_t)format);
    
    // GEO卫星场景下，随机接入需要考虑:
    // 1. 长传播延迟 (~600ms RTT for GEO)
    // 2. Timing Advance补偿
    // 3. 多址接入冲突避免
    
    // 根据多址模式选择接入策略
    switch (m_maMode) {
        case MultipleAccessMode::SLOTTED_ALOHA:
            NS_LOG_INFO ("Mode: Slotted ALOHA");
            break;
        case MultipleAccessMode::ESSA:
            NS_LOG_INFO ("Mode: ESSA (Enhanced Slotted ALOHA)");
            break;
        case MultipleAccessMode::FDMA:
            NS_LOG_INFO ("Mode: FDMA");
            break;
        case MultipleAccessMode::TDMA:
            NS_LOG_INFO ("Mode: TDMA");
            break;
    }
    
    // 计算GEO卫星定时提前量
    // 对于GEO卫星，往返时间约600ms，单程约300ms
    // Timing Advance = RTT / 2
    m_timingAdvance = MicroSeconds (300000); // 300ms定时提前
    
    NS_LOG_INFO ("GEO Timing Advance: " << m_timingAdvance.GetMicroSeconds () << " μs");
    
    // 执行PRACH前导码发送
    DoPrachTransmission ();
    
    // 调度RAR响应接收 (在GEO RTT之后)
    ScheduleRaResponse (preambleId);
}

void SatUtPhy::SetRaParams (uint32_t preambleId, uint8_t format, Time transmissionTime) {
    m_preambleId = preambleId;
    m_prachFormat = format;
    m_raTransmissionTime = transmissionTime;
}

Time SatUtPhy::GetTimingAdvance () const {
    return m_timingAdvance;
}

void SatUtPhy::SetMultipleAccessMode (MultipleAccessMode mode) {
    m_maMode = mode;
    NS_LOG_INFO ("Multiple Access Mode set to: " << (uint32_t)mode);
}

void SatUtPhy::DoPrachTransmission () {
    NS_LOG_FUNCTION (this);
    
    NS_LOG_INFO ("[PRACH] 发送前导码 PreambleID=" << m_preambleId 
                 << " Format=" << (uint32_t)m_prachFormat
                 << " Time=" << Simulator::Now ().GetSeconds () << "s"
                 << " TA=" << m_timingAdvance.GetMicroSeconds () << "μs");
    
    // TODO: 调用物理层发送PRACH前导码
    // 这里会触发SpectrumChannel的StartTx
}

void SatUtPhy::ScheduleRaResponse (uint32_t preambleId) {
    NS_LOG_FUNCTION (this << preambleId);
    
    // GEO卫星RTT约600ms，RAR在第一次接入传输后约600ms返回
    Time geoRtt = MilliSeconds (600);
    
    NS_LOG_INFO ("[RAR] 调度RAR响应接收，预计延迟: " << geoRtt.GetMilliSeconds () << " ms");
    
    // 在GEO RTT后模拟接收RAR
    m_raEvent = Simulator::Schedule (geoRtt, [this, preambleId]() {
        NS_LOG_INFO ("[RAR] 接收随机接入响应! PreambleID=" << preambleId);
        NS_LOG_INFO ("[RAR] 包含: TA=" << m_timingAdvance.GetMicroSeconds () << "μs, UL Grant");
        
        // RAR成功接收后，终端可以发送Msg3 (PUSCH)
        // 定时补偿已应用，下一次传输将使用调整后的时序
    });
}

// ==============================================================
// ESSA (Enhanced Slotted ALOHA) 多址接入
// ==============================================================

double SatUtPhy::CalculateCollisionProbability (uint32_t numActiveUsers, uint32_t numSlots) {
    // ESSA碰撞概率分析
    // 在Slotted ALOHA中，用户在每个时隙发送的概率为 P = 1/numSlots
    // 碰撞概率 = 1 - (1 - 1/numSlots)^(numActiveUsers-1)
    
    double slotOccupancyProb = 1.0 / static_cast<double>(numSlots);
    double collisionProb = 1.0 - std::pow(1.0 - slotOccupancyProb, numActiveUsers - 1);
    
    NS_LOG_INFO ("ESSA Collision Analysis:");
    NS_LOG_INFO ("  Active Users: " << numActiveUsers);
    NS_LOG_INFO ("  Num Slots: " << numSlots);
    NS_LOG_INFO ("  Slot Occupancy Prob: " << slotOccupancyProb);
    NS_LOG_INFO ("  Collision Probability: " << collisionProb);
    
    return collisionProb;
}

uint32_t SatUtPhy::SelectEssAlohaSlot () {
    // ESSA时隙选择算法
    // 增强版: 考虑历史碰撞信息，选择空闲概率最高的时隙
    
    NS_LOG_FUNCTION (this);
    
    uint32_t selectedSlot = 0;
    
    if (m_slotAllocationMap.empty ()) {
        // 首次接入，随机选择
        selectedSlot = Simulator::Now ().GetMicroSeconds () % m_essaNumSlots;
        NS_LOG_INFO ("[ESSA] 首次接入，随机选择时隙: " << selectedSlot);
    } else {
        // 查找最少使用的时隙 (最不可能碰撞)
        uint32_t minUsage = UINT32_MAX;
        for (uint32_t i = 0; i < m_essaNumSlots; ++i) {
            uint32_t usage = m_slotAllocationMap.count (i) > 0 ? m_slotAllocationMap[i] : 0;
            if (usage < minUsage) {
                minUsage = usage;
                selectedSlot = i;
            }
        }
        NS_LOG_INFO ("[ESSA] 基于历史选择时隙: " << selectedSlot << " (使用次数: " << minUsage << ")");
    }
    
    // 更新时隙使用计数
    m_slotAllocationMap[selectedSlot]++;
    
    return selectedSlot;
}

// ==============================================================
// 接收与测量上报
// ==============================================================

void SatUtPhy::Receive (Ptr<Packet> packet, double rxPowerDbm, uint32_t beamId) {
    NS_LOG_FUNCTION (this << rxPowerDbm << beamId);

    double sinrDb = CalculateComprehensiveSinr (rxPowerDbm, beamId);
    
    uint8_t currentCqi = MapSinrToCqi (sinrDb);
    
    m_lastCalculatedSinr = sinrDb;
    
    if (m_macSapUser) {
        MeasReport report;
        report.ueId = m_rnti;
        report.bestBeamId = beamId;
        report.rsrp = rxPowerDbm;
        
        NS_LOG_INFO ("MeasReport: RSRP=" << rxPowerDbm << " dBm, CQI=" 
                     << (uint32_t)currentCqi << ", SINR=" << sinrDb << " dB");
        
        m_macSapUser->ReceiveMeasReport (report);
    }
    
    if (!m_cqiReportCallback.IsNull ()) {
        m_cqiReportCallback (currentCqi);
    }
}

void SatUtPhy::PerformHandover (uint32_t targetBeamId) {
    NS_LOG_INFO ("Executing PHY layer Handover. Switching to Beam: " << targetBeamId);
    m_currentBeamId = targetBeamId;
}

void SatUtPhy::AssignNewSatChannels (uint32_t channelId) {
    NS_LOG_INFO ("Assigned new satellite channel ID: " << channelId);
}

void SatUtPhy::UpdateSliceSubscription (uint32_t sliceId, bool subscribe) {
    NS_LOG_INFO ("Updating slice subscription. Slice ID: " << sliceId << ", Subscribe: " << subscribe);
}

} // namespace ns3
