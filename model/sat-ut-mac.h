// 文件路径：contrib/geo-sat/model/sat-ut-mac.h
#ifndef SAT_UT_MAC_H
#define SAT_UT_MAC_H

#include "ns3/object.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"
#include "sat-mac-common.h"
#include "resource-manager.h"
#include "sat-phy.h" // 引入物理层
#include "sat-ut-phy.h" // 引入终端物理层(含MultipleAccessMode枚举)

namespace ns3 {

class SatUtMac : public Object {
public:
    static TypeId GetTypeId (void);
    SatUtMac ();
    virtual ~SatUtMac ();

    // 定义终端 MAC 层的状态机
    enum UtMacState {
        MAC_IDLE,           // 空闲态
        MAC_CONNECTED,      // 已连接态
        MAC_TX,             // 正在发送
        MAC_RX              // 正在接收
    };

    // 绑定底层物理层指针 (组装网络时使用)
    void SetPhy (Ptr<SatPhy> phy);

    // 状态切换辅助函数
    void SwitchState (UtMacState newState);

    // 核心任务1：解析 DCI 并调度
    void ProcessDciAndSchedule (DciInfo dci);

    // 核心任务2：执行实际的数据传输操作
    void DoTransmit (Time duration, uint32_t rbAllocation, uint8_t mcs, double txPowerDbm);

    // 核心任务3：接收下行数据
    void ReceiveData (Ptr<Packet> packet, const DciInfo& dci);

    // ==================== PUCCH 功能实现 ====================
    // PUCCH Format 0: SR (调度请求) - 1bit信号
    void SendSchedulingRequest ();

    // PUCCH Format 1/2: CSI/CQI 报告
    void SendCqiReport (uint8_t cqi);

    // PUCCH Format 2/3: HARQ ACK/NACK 反馈
    void SendHarqAck (bool ack, uint8_t bitmap = 1);

    // 统一的PUCCH传输接口
    void SendPucch (const PucchInfo& pucchInfo);

    // ==================== PRACH 功能实现 ====================
    // PRACH前导码发送 (随机接入)
    void SendPrachPreamble (uint32_t preambleId, uint8_t format);

    // ==================== BSR 功能实现 ====================
    // 缓冲区状态报告
    void SendBsr (uint8_t lcgId, uint32_t bufferSize);

    // 设置BSR回调 (用于通知调度器有数据需要发送)
    void SetBsrCallback (Callback<void, const BsR_MAC_CE&> callback);

    // 设置PUCCH回调 (用于向信关站发送UCI)
    void SetPucchCallback (Callback<void, const PucchInfo&> callback);

    // 设置PRACH回调 (用于随机接入)
    void SetPrachCallback (Callback<void, const PrachPreamble&> callback);

    // 数据到达触发SR (当终端有数据要发送时调用)
    void NotifyDataBuffered (uint32_t bytes);

    // ==================== 4 步随机接入 (Msg1→Msg2→Msg3→Msg4) ====================
    // 发起随机接入流程: 发 Msg1 (PRACH preamble), 启动 RA Response Timer
    // preambleId = 0 表示自动随机挑选 (1-63)
    void DoRandomAccess (uint32_t preambleId = 0, uint8_t format = 0);

    // Msg2: 接收 RAR, 根据 preambleId 匹配自身 → 触发 Msg3 发送
    void ReceiveRar (const RarMessage& rar);

    // Msg3: 组装 RRCSetupRequest (携带自身 ueIdentity 与收到的 tcRnti) 并上送
    void SendMsg3 ();
    void DeliverPendingMsg3 ();

    // Msg4: 接收 RRCSetup, 比对回显的 ueIdentity 完成竞争解决
    void ReceiveMsg4 (const RrcSetupMessage& msg4);

    // 定时器回调: RA Response Timer 超时 → 未收到 RAR → 重传 Msg1
    void OnRaResponseTimeout ();
    // 定时器回调: Contention Resolution Timer 超时 → 未收到 Msg4 → 重传 Msg1
    void OnContentionResolutionTimeout ();
    // 终止当前接入尝试并根据策略决定是否重试 (传入原因用于日志)
    void AbortOrRetryRa (const std::string& reason);

    // 获取当前定时提前量
    Time GetTimingAdvance () const;

    // 设置多址接入模式
    void SetMultipleAccessMode (MultipleAccessMode mode);

    // 检查是否已接入 (RRC_CONNECTED)
    bool IsConnected () const;

    // 设置本 UE 的唯一身份 (40-bit contention resolution ID)
    void SetUeIdentity (uint64_t ueIdentity);
    uint64_t GetUeIdentity () const;
    void SetUtType (UtType utType);
    void SetRnti (uint16_t rnti);
    uint16_t GetActiveRnti () const;

    // 配置 RA 定时器和最大重传次数
    void SetRaTimers (Time raResponseWindow, Time contentionResolutionTimer, uint8_t maxAttempts);

    // 设置 Msg3 发送回调 (上送到 GeoBeamScheduler::ReceiveMsg3Packet)
    void SetMsg3Callback (Callback<void, Ptr<Packet>> callback);

    // 随机接入完成 (成功/失败) 回调, 便于测试统计
    enum RaResult { RA_SUCCESS, RA_FAILED_MAX_ATTEMPTS };
    void SetRaCompleteCallback (Callback<void, RaResult> callback);

private:
    UtMacState m_state;
    Ptr<SatPhy> m_phy;     // 指向上个月开发的物理层
    EventId m_txEvent;     // 用于 ns-3 的事件调度
    TracedCallback<Ptr<Packet>> m_rxPduCallback;  // 接收数据回调

    // PUCCH相关
    bool m_srPending;       // SR挂起标志
    uint8_t m_pendingCqi;   // 待发送的CQI值
    bool m_pendingHarqAck;  // 待发送的HARQ ACK/NACK
    Callback<void, const BsR_MAC_CE&> m_bsrCallback;
    Callback<void, const PucchInfo&> m_pucchCallback;
    Callback<void, const PrachPreamble&> m_prachCallback;
    uint32_t m_currentBufferBytes; // 当前缓冲区字节数
    
    // ---------- 随机接入状态 ----------
    Time m_timingAdvance;                      // 定时提前量 (GEO ~300ms)
    MultipleAccessMode m_maMode;               // 多址接入模式
    bool m_isRaInitiated;                      // 是否正在执行 RA
    uint32_t m_pendingUlGrant;                 // RAR 中收到的 UL Grant (PRB 数)
    uint8_t m_pendingUlGrantMcs;               // RAR 中收到的 Msg3 MCS
    double m_pendingUlGrantTxPowerDbm;         // RAR 中收到的 Msg3 目标发射功率

    uint64_t m_ueIdentity;                     // 本 UE 的竞争解决身份
    UtType m_utType;                          // 终端类型
    uint16_t m_rnti;                          // 已连接业务面使用的 UE 标识
    uint32_t m_currentPreambleId;              // 当前本次 RA 使用的 preambleId
    uint16_t m_tcRnti;                         // Msg2 中分配给本 UE 的 TC-RNTI (0 表示尚未获得)
    uint16_t m_cRnti;                          // Msg4 成功后晋升的正式 C-RNTI

    uint8_t m_raAttempt;                       // 当前已发起的 Msg1 次数
    uint8_t m_maxRaAttempts;                   // 最大 Msg1 次数
    Time m_raResponseWindow;                   // RA Response Timer 时长
    Time m_contentionResolutionTimeout;        // 竞争解决定时器时长

    EventId m_raResponseTimer;                 // RA Response Timer (等 Msg2)
    EventId m_contentionResolutionTimer;       // 竞争解决定时器 (等 Msg4)
    EventId m_msg3TxEvent;                     // Msg3 延迟发送事件

    Callback<void, Ptr<Packet>>            m_msg3Callback;
    Callback<void, RaResult>                m_raCompleteCallback;
    bool m_hasPendingMsg3;
    RrcSetupRequest m_pendingMsg3Request;
    Ptr<Packet> m_pendingMsg3Packet;
};

} // namespace ns3
#endif /* SAT_UT_MAC_H */
