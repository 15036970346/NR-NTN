/*
 * 文件路径：contrib/geo-sat/model/harq-manager.h
 * 功能：HARQ增强管理器 - 支持增量冗余(IR)重传机制
 * 支持：HARQ进程管理、NACK触发重传、IR增量冗余
 * 特性：支持禁用HARQ开关，用于对比性能
 */
#ifndef HARQ_MANAGER_H
#define HARQ_MANAGER_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"
#include <map>
#include <vector>

namespace ns3 {

// HARQ进程状态
enum class HarqState {
    IDLE,           // 空闲
    WAITING_ACK,    // 等待ACK
    RETRANSMITTING, // 重传中
    SUCCESS,        // 传输成功
    FAILED          // 超过最大重传次数
};

// HARQ进程信息 (重命名为SatHarqProcess避免与nr模块冲突)
struct SatHarqProcess {
    uint8_t processId;         // 进程ID (0-7)
    uint16_t rnti;             // 用户ID
    uint32_t txBuffer;        // 发送缓存
    uint32_t txBytes;         // 已发送字节数
    uint8_t mcs;              // 调制编码策略
    uint8_t rvIndex;          // 冗余版本索引 (0-3 for IR)
    HarqState state;          // 当前状态
    uint8_t retransmitCount;  // 重传次数
    Time startTime;           // 开始时间
    Ptr<Packet> packet;       // 数据包副本
};

// HARQ反馈信息 (重命名为SatHarqFeedback避免与nr模块冲突)
struct SatHarqFeedback {
    uint16_t rnti;
    uint8_t processId;
    bool ack;           // true=ACK, false=NACK
    uint8_t rvIndex;   // 下一个冗余版本
};

class HarqManager : public Object
{
public:
    static TypeId GetTypeId (void);
    HarqManager ();
    virtual ~HarqManager ();

    /**
     * \brief 启用/禁用HARQ功能
     * \param enable true: 启用HARQ (默认); false: 禁用HARQ
     * \details 禁用HARQ后，收到NACK时不触发重传，直接标记传输成功
     */
    void SetHarqEnabled (bool enable);

    /**
     * \brief 获取HARQ是否启用
     */
    bool IsHarqEnabled () const;

    // ==================== HARQ进程管理 ====================
    // 开启新传输 (分配HARQ进程)
    uint8_t NewTransmission (uint16_t rnti, Ptr<Packet> packet, uint8_t mcs);
    
    // 收到ACK/NACK反馈
    void ReceiveFeedback (const SatHarqFeedback& feedback);
    
    // 获取进程状态
    HarqState GetProcessState (uint16_t rnti, uint8_t processId) const;
    
    // 获取可用的HARQ进程数
    uint8_t GetAvailableProcessCount (uint16_t rnti) const;

    // ==================== IR增量冗余 ====================
    // 获取下一个冗余版本 (用于重传)
    uint8_t GetNextRedundancyVersion (uint16_t rnti, uint8_t processId);
    
    // 计算IR增益因子
    double CalculateIrGain (uint8_t rvIndex) const;
    
    // 生成重传数据包 (IR puncturing)
    Ptr<Packet> GenerateRetransmission (uint16_t rnti, uint8_t processId);

    // ==================== 统计接口 ====================
    uint32_t GetTotalTransmissions () const;
    uint32_t GetTotalRetransmissions () const;
    uint32_t GetTotalAck () const;
    uint32_t GetTotalNack () const;
    uint32_t GetTotalDropped () const;  //!< 禁用HARQ时因NACK丢弃的包数
    double GetRetransmissionRate () const;
    void ResetStatistics ();

    // 回调函数
    TracedCallback<uint16_t, uint8_t, Ptr<Packet>> m_retransmitCallback;
    TracedCallback<uint16_t, uint8_t, bool> m_feedbackCallback;

private:
    SatHarqProcess* GetProcess (uint16_t rnti, uint8_t processId);
    SatHarqProcess* AllocateProcess (uint16_t rnti);
    void ScheduleRetransmission (uint16_t rnti, uint8_t processId);
    void ProcessTimeout (uint16_t rnti, uint8_t processId);

    static const uint8_t MAX_HARQ_PROCESSES = 8;
    static const uint8_t MAX_RETRANSMISSIONS = 4;
    static const Time HARQ_ACK_TIMEOUT;

    bool m_harqEnabled;  //!< HARQ使能开关

    std::map<std::pair<uint16_t, uint8_t>, SatHarqProcess> m_harqProcesses;
    std::map<uint16_t, std::vector<uint8_t>> m_rntiToProcesses;

    uint32_t m_totalTransmissions;
    uint32_t m_totalRetransmissions;
    uint32_t m_totalAck;
    uint32_t m_totalNack;
    uint32_t m_totalDropped;  //!< 禁用HARQ时丢弃的包数
};

} // namespace ns3

#endif /* HARQ_MANAGER_H */
