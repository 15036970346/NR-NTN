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
    uint8_t processId {0};         // 进程ID
    uint16_t rnti {0};             // 用户ID
    uint32_t txBuffer {0};         // 发送缓存
    uint32_t txBytes {0};          // 已发送字节数
    uint8_t mcs {0};               // 调制编码策略
    uint8_t rvIndex {0};           // 当前冗余版本值 (0/2/3/1)
    uint8_t rvSequenceIndex {0};   // RV 序列位置
    HarqState state {HarqState::IDLE}; // 当前状态
    uint8_t retransmitCount {0};   // 已执行重传次数
    Time startTime {Seconds (0)};  // 开始时间
    Ptr<Packet> packet;            // 数据包副本
    EventId ackTimeoutEvent;       // 等待 ACK/NACK 的超时事件
};

// HARQ反馈信息 (重命名为SatHarqFeedback避免与nr模块冲突)
struct SatHarqFeedback {
    uint16_t rnti;
    uint8_t processId;
    bool ack;           // true=ACK, false=NACK
    uint8_t rvIndex {0};   // 兼容字段: 当前实现会自行推进 RV 序列
};

class HarqManager : public Object
{
public:
    static TypeId GetTypeId (void);
    static constexpr uint8_t INVALID_PROCESS_ID = 0xFF;
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

    /**
     * \brief 设置每 UE HARQ 进程数
     * \param count 仅支持 1 / 2 / 4 / 8 / 16 / 32
     */
    void SetProcessCount (uint8_t count);

    /**
     * \brief 获取当前 HARQ 进程数
     */
    uint8_t GetProcessCount () const;

    /**
     * \brief 检查进程数配置是否合法
     */
    static bool IsSupportedProcessCount (uint8_t count);

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

    // 获取当前冗余版本 (用于构造重传 DCI)
    uint8_t GetCurrentRedundancyVersion (uint16_t rnti, uint8_t processId) const;

    // 获取进程已执行的重传次数 (终态时仍保留, 供 OLLA 判定"首传(rv==0)是否成功":
    // retransmitCount==0 表示该进程从未重传, 终态 SUCCESS 即首传 ACK)。
    uint8_t GetRetransmitCount (uint16_t rnti, uint8_t processId) const;
    
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
    const SatHarqProcess* GetProcess (uint16_t rnti, uint8_t processId) const;
    SatHarqProcess* AllocateProcess (uint16_t rnti);
    void ScheduleRetransmission (uint16_t rnti, uint8_t processId);
    void ProcessTimeout (uint16_t rnti, uint8_t processId);
    void ScheduleAckTimeout (SatHarqProcess& process);
    void FinalizeProcess (SatHarqProcess& process, HarqState finalState);

    static const uint8_t DEFAULT_HARQ_PROCESSES = 8;
    static const uint8_t MAX_RETRANSMISSIONS = 4;
    static const Time HARQ_ACK_TIMEOUT;

    bool m_harqEnabled;  //!< HARQ使能开关
    uint8_t m_processCount; //!< 每 UE HARQ 进程数 (1/2/4/8/16/32)

    std::map<std::pair<uint16_t, uint8_t>, SatHarqProcess> m_harqProcesses;

    uint32_t m_totalTransmissions;
    uint32_t m_totalRetransmissions;
    uint32_t m_totalAck;
    uint32_t m_totalNack;
    uint32_t m_totalDropped;  //!< 禁用HARQ时丢弃的包数
};

} // namespace ns3

#endif /* HARQ_MANAGER_H */
