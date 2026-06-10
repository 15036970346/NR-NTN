/*
 * 文件路径：contrib/geo-sat/model/sat-stats-collector.h
 * 功能：NR NTN系统级仿真统计收集器
 * 支持：接入成功率、HARQ重传、峰值速率、系统容量统计
 * 支持：接入真实 NR trace 回调（NrBearerStatsCalculator + NrGnbMac）
 * 支持：导出统计数据到4个TXT文件
 */
#ifndef SAT_STATS_COLLECTOR_H
#define SAT_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/flow-monitor.h"
#include "ns3/nr-bearer-stats-calculator.h"
#include "sat-ut-phy.h"
#include "resource-manager.h"
#include <map>
#include <vector>
#include <string>

namespace ns3 {

struct AccessStatistics {
    uint32_t totalAttempts;       // 总接入尝试次数
    uint32_t successCount;        // 成功接入次数
    uint32_t collisionCount;      // 碰撞失败次数
    uint32_t timeoutCount;        // RAR超时次数
    uint32_t poorChannelCount;    // 因信道质量不足拒绝次数
    double successRate;           // 接入成功率
};

// 每 UE 的信道质量上下文 (用于 NR trace 路径的信道质量门限过滤)
struct UeChannelContext {
    UtType   utType;      // 终端类型
    double   rsrp;        // 最近一次 RSRP (dBm)
    double   sinrDb;      // 最近一次 SINR (dB)
    bool     valid;       // 是否已有有效测量
};

struct HarqStatistics {
    uint32_t totalTransmissions; // 总传输次数(含重传)
    uint32_t firstTransmissions; // 首次传输次数
    uint32_t retransmissions;    // 重传次数
    uint32_t ackCount;          // ACK成功次数
    uint32_t nackCount;         // NACK失败次数
    double retransmissionRate;   // 重传率
};

struct UserRateStatistics {
    uint16_t rnti;
    uint64_t imsi;               // IMSI of the UE
    uint64_t totalBytesTx;       // 总发送字节数
    uint64_t totalBytesRx;       // 总接收字节数 (DL)
    double peakRate;             // 峰值速率 (bps)
    double averageRate;          // 平均速率 (bps)
    Time lastUpdateTime;         // 上次更新时间
    uint64_t lastBytesRx;        // 上次接收字节计数(用于滑动窗口)
    double windowedPeakRate;     // 滑动窗口峰值速率 (bps)
    Time windowStart;            // 当前窗口开始时间
    double currentWindowBytes;   // 当前窗口内字节数
    // Per-LCID byte counters for grouped traffic statistics
    uint64_t voipBytes{0};       // LCID 4
    uint64_t httpBytes{0};       // LCID 5
    uint64_t ftpBytes{0};       // LCID 6
};

struct BeamStatistics {
    uint32_t beamId;
    uint32_t totalActiveUes;    // 总活跃用户数
    uint64_t totalThroughput;   // 总吞吐量 (bits)
    double averageCqi;          // 平均CQI
    uint32_t totalRbAllocated;  // 总RB分配次数
    double spectrumEfficiency;   // 频谱效率 (bps/Hz)
};

class SatStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatStatsCollector ();
    virtual ~SatStatsCollector ();

    // ==================== 接入统计 ====================
    void RecordAccessAttempt (uint16_t rnti);
    void RecordAccessSuccess (uint16_t rnti);

    // ==================== 信道质量注册 (NR trace 路径) ====================
    // 在 UE 接入前/接入时注册终端类型 (供信道质量门限判定使用)
    void RegisterUeType (uint16_t rnti, UtType utType);
    void RegisterUeTypeByImsi (uint64_t imsi, UtType utType);
    // 更新 UE 的信道质量测量值 (从 PHY 测量上报或定期刷新)
    void UpdateUeChannelQuality (uint16_t rnti, double rsrp, double sinrDb);
    void RecordAccessCollision (uint16_t rnti);
    void RecordAccessTimeout (uint16_t rnti);
    AccessStatistics GetAccessStatistics () const;
    void ResetAccessStats ();

    // ==================== HARQ统计 ====================
    void RecordHarqTransmission (uint16_t rnti, bool isRetransmission);
    void RecordHarqAck (uint16_t rnti, bool ack);
    HarqStatistics GetHarqStatistics (uint16_t rnti) const;
    HarqStatistics GetTotalHarqStatistics () const;
    void ResetHarqStats ();

    /**
     * \brief 接收真实 NR HARQ 反馈 (NrGnbMac::m_dlHarqFeedback 回调)
     * \param params HARQ反馈信息，包含 m_rnti, m_harqProcessId, m_harqStatus
     */
    void RecordNrHarqFeedback (const struct DlHarqInfo& params);

    /**
     * \brief 接收 DL 数据转发 trace (LteEnbRrc::m_dlDataForwardingTrace)
     * \param imsi UE IMSI
     * \param rnti UE RNTI
     * \param lcid Logical Channel ID (BID)
     * \param bytes 包大小 (bytes)
     */
    void RecordDlDataForwarding (uint64_t imsi, uint16_t rnti, uint8_t lcid, uint32_t bytes);

    // ==================== 速率统计（来自 NrBearerStatsCalculator）====================
    /**
     * \brief 接收 NR PDCP DL Rx PDU 回调
     * \param cellId Cell ID
     * \param imsi UE IMSI
     * \param rnti UE RNTI
     * \param lcid Logical Channel ID
     * \param packetSize 包大小 (bytes)
     * \param delay 端到端延迟 (ns)
     */
    void RecordDlRxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti,
                        uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief 接收 NR PDCP UL Rx PDU 回调
     */
    void RecordUlRxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti,
                        uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief 接收 NR PDCP DL Tx PDU 回调
     */
    void RecordDlTxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti,
                        uint8_t lcid, uint32_t packetSize);

    /**
     * \brief 接收 NR PDCP UL Tx PDU 回调
     */
    void RecordUlTxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti,
                        uint8_t lcid, uint32_t packetSize);

    /**
     * \brief 接入成功回调（NrBearerStatsConnector::NotifyRandomAccessSuccessfulUe）
     * 带 context string 参数（由 Config::Connect 注入）
     * 签名: void(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
     */
    void OnNrRandomAccessSuccess (std::string context, uint64_t imsi,
                                   uint16_t cellId, uint16_t rnti);

    /**
     * \brief 接入成功回调（无 context 版本，供 device TraceConnectWithoutContext 使用）
     * 签名: void(uint64_t imsi, uint16_t cellId, uint16_t rnti)
     */
    void OnNrRandomAccessSuccessNoCtx (uint64_t imsi, uint16_t cellId, uint16_t rnti);

    /**
     * \brief LTE UE PDCP DL Rx 回调
     * 签名: void (*PduRxTracedCallback)(uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delay)
     */
    void OnLtePdcpDlRx (uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief LTE UE PDCP UL Rx 回调
     * 签名: 同上
     */
    void OnLtePdcpUlRx (uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief LTE UE RLC DL Rx 回调
     * 签名: void (*ReceiveTracedCallback)(uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t delay)
     */
    void OnLteRlcDlRx (uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief LTE UE RLC UL Rx 回调
     * 签名: 同上
     */
    void OnLteRlcUlRx (uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief NR UE RRC PDCP DL Rx trace 回调（通过 device pointer 连接）
     * 签名: void DlRxPdu(uint16_t cellId, uint64_t imsi, uint16_t rnti,
     *                   uint8_t lcid, uint32_t packetSize, uint64_t delay)
     */
    void OnNrRrcDlRxPdu (uint16_t cellId, uint64_t imsi, uint16_t rnti,
                          uint8_t lcid, uint32_t packetSize, uint64_t delay);

    /**
     * \brief NR UE PDCP RxPDU trace 回调（直接从 UE device 连接）
     * 签名: void (std::string path, uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay)
     * 用于处理 LtePdcp::RxPDU trace
     */
    static void OnNrPdcpRxPduFromPath (Ptr<SatStatsCollector> collector,
                                        uint64_t imsi,
                                        std::string path,
                                        uint16_t rnti,
                                        uint8_t lcid,
                                        uint32_t packetSize,
                                        uint64_t delay)
    {
        collector->RecordDlRxPdu (0, imsi, rnti, lcid, packetSize, delay);
    }

    /**
     * \brief UdpServer Rx trace 回调
     * 收到第一个包 = UE 接入成功；持续收到包 = 吞吐数据
     */
    void OnUdpServerRx (Ptr<const Packet> packet, const Address& srcAddress, const Address& dstAddress);

    /**
     * \brief UdpServer Rx trace 回调 - 用于计算真实吞吐量
     * 记录每个收到的包，用于计算实际 DL 吞吐量
     */
    void OnUdpServerRxForThroughput (Ptr<const Packet> packet, const Address& srcAddress, const Address& dstAddress);

    /**
     * \brief 设置 FlowMonitor（用于结束后提取吞吐统计）
     */
    void SetFlowMonitor (Ptr<FlowMonitor> flowMonitor);

    void UpdateRateStatistics ();
    UserRateStatistics GetUserRateStats (uint16_t rnti) const;
    double GetSystemPeakRate () const;
    double GetSystemAverageRate () const;

    // ==================== 峰值用户统计 ====================
    uint16_t GetPeakRateUeRnti () const;
    double GetPeakRateUeThroughput_Mbps () const;

    // ==================== 系统吞吐量统计 ====================
    double GetSystemTotalThroughput_Mbps () const;
    uint32_t GetConnectedUeCount () const;

    /**
     * \brief 静态转发函数，供 Config::Connect 调用
     * 将 LteUeRrc::RandomAccessSuccessful 的 4 参数签名转发到 OnNrRandomAccessSuccess
     */
    static void OnNrRandomAccessSuccessForward (Ptr<SatStatsCollector> collector,
                                               std::string context,
                                               uint64_t imsi,
                                               uint16_t cellId,
                                               uint16_t rnti)
    {
        collector->OnNrRandomAccessSuccess (context, imsi, cellId, rnti);
    }

    // ==================== 波束/系统统计 ====================
    void RecordBeamThroughput (uint32_t beamId, uint64_t bits);
    void RecordRbAllocation (uint32_t beamId, uint32_t rbs);
    void RecordCqiReport (uint32_t beamId, uint8_t cqi);
    BeamStatistics GetBeamStatistics (uint32_t beamId) const;
    uint64_t GetSystemCapacity () const;
    double GetSystemSpectrumEfficiency () const;

    // ==================== 仿真控制 ====================
    void SetSimulationTime (Time start, Time end);
    void SetSlidingWindowSize (Time window);
    void EnablePrintStatistics (bool enable);
    void PrintSummaryReport () const;

    // ==================== 文件导出 ====================
    void SetSimulationParams (uint8_t reuseMode, bool disableHarq, uint32_t totalUes);
    void ExportStatsToFiles ();
    
    // 调试：获取详细的吞吐统计用于诊断
    uint64_t GetTotalDlRxBytes () const;
    uint32_t GetUeRateStatsCount () const;
    /**
     * \brief Get per-UE rate statistics by IMSI
     * \param imsi IMSI of the UE
     * \return UserRateStatistics (fields will be zero-initialized if IMSI not found)
     */
    UserRateStatistics GetUeRateStats (uint64_t imsi) const;
    
    // 设置 PDCP stats calculator 用于获取真实吞吐量
    void SetPdcpStatsCalculator (Ptr<NrBearerStatsCalculator> pdcpStats);

    // 回调函数
    TracedCallback<uint16_t, double> m_accessSuccessTrace;
    TracedCallback<uint16_t, double> m_peakRateTrace;

private:
    void CheckAndUpdatePeakRate (uint16_t rnti);
    void AdvanceWindow (uint16_t rnti, const Time& now);

    // 信道质量门限判定 (与 SatUtMac 使用相同基准)
    // RSRP 门限: -94.5 dBm  SNR 门限: Consumer=1.8dB, Portable=20.8dB
    static constexpr double CQ_RSRP_THRESHOLD     = -94.5;
    static constexpr double CQ_SNR_CONSUMER       = 1.8;
    static constexpr double CQ_SNR_PORTABLE       = 20.8;
    bool IsChannelQualitySufficient (uint16_t rnti) const;

    // 仿真参数
    uint8_t m_reuseMode;
    bool m_disableHarq;
    uint32_t m_totalUes;
    Time m_slidingWindowSize; //!< 峰值速率滑动窗口大小，默认100ms

    // 接入统计
    AccessStatistics m_accessStats;
    std::map<uint16_t, uint32_t> m_ueAccessAttempts;

    // 每 UE 信道质量上下文
    std::map<uint16_t, UeChannelContext> m_ueChannelContext;
    std::map<uint16_t, uint64_t> m_rntiToImsi;  // RNTI → IMSI 映射
    std::map<uint64_t, uint16_t> m_imsiToRnti;  // IMSI → RNTI 映射
    std::set<std::string> m_udpServerConnectedUes;  // 已通过UDP接入的UE地址集合（用于去重）

    // HARQ统计
    std::map<uint16_t, HarqStatistics> m_ueHarqStats;
    std::map<uint16_t, uint8_t> m_ueHarqProcessCount;
    // NR HARQ: RNTI → (进程ID → 状态)
    // 0 = 空闲, 1 = 首传后等待反馈, 2 = 重传后等待反馈
    std::map<uint16_t, std::map<uint8_t, uint8_t>> m_harqProcessState;

    // 速率统计
    std::map<uint16_t, UserRateStatistics> m_ueRateStats;
    std::map<uint64_t, uint16_t> m_imsiToRateRnti; // IMSI → RNTI (for rate stats)
    std::map<uint16_t, std::vector<std::pair<Time, double>>> m_ueRateHistory;
    Time m_simStartTime;
    Time m_simEndTime;
    bool m_printEnabled;
    Ptr<FlowMonitor> m_flowMonitor; //!< FlowMonitor for throughput statistics
    bool m_statsExported; //!< 防止重复导出
    Ptr<NrBearerStatsCalculator> m_pdcpStatsCalculator; //!< PDCP stats calculator for real throughput data

    // 波束统计
    std::map<uint32_t, BeamStatistics> m_beamStats;
};

// forward declaration needed by RecordNrHarqFeedback
struct DlHarqInfo;

} // namespace ns3

#endif /* SAT_STATS_COLLECTOR_H */
