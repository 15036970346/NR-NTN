// 文件路径：contrib/geo-sat/model/sat-ut-mac.h
#ifndef SAT_UT_MAC_H
#define SAT_UT_MAC_H

#include "ns3/object.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include "sat-mac-common.h"
#include "resource-manager.h"
#include "sat-phy.h"        // 引入物理层
#include "sat-ut-phy.h"     // 引入终端物理层(含MultipleAccessMode枚举)

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
        MAC_RX,             // 正在接收
        MAC_WAITING_MSGB    // 2 步 RA: 等待 MsgB 响应
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

    // 设置 preamble 池大小 (默认 63; 双信道可设 128)
    void SetNumPreambles (uint32_t n);
    uint32_t GetNumPreambles () const;

    // 设置 Msg3 发送回调 (上送到 GeoBeamScheduler::ReceiveMsg3Packet)
    void SetMsg3Callback (Callback<void, Ptr<Packet>> callback);

    // 随机接入完成 (成功/失败) 回调, 便于测试统计
    enum RaResult {
        RA_SUCCESS,                  // 接入成功
        RA_FAILED_MAX_ATTEMPTS,      // 超过最大重传次数
        RA_FAILED_POOR_CHANNEL       // 信道质量不满足门限
    };
    void SetRaCompleteCallback (Callback<void, RaResult> callback);

    // 设置终端类型 (用于信道质量门限选取)
    void SetUtType (UtType utType);
    UtType GetUtType () const;

    // ==================== 2 步随机接入 (MsgA→MsgB) ====================
    // 统一入口: 根据 m_rachType 分派到 4 步或 2 步
    void InitiateRandomAccess (uint32_t preambleId = 0, uint8_t format = 0);
    // 2 步 RA 核心: 构建 MsgA (preamble+payload 打包) 并发送
    void DoTwoStepRandomAccess (uint32_t preambleId = 0, uint8_t format = 0);
    // 接收 MsgB (SUCCESS_RAR 或 FALLBACK_RAR)
    void ReceiveMsgB (const MsgB& msgB);
    // MsgB 响应超时
    void OnMsgBResponseTimeout ();
    // 设置/获取 RA 类型
    void SetRachType (RachType rachType);
    RachType GetRachType () const;
    // 设置 MsgA 发送回调
    void SetMsgACallback (Callback<void, const MsgA&> callback);

    // ==================== 接入过程统计计数器 ====================
    // 总发送 Msg1/MsgA 次数 (每次重传都算一次)
    uint32_t GetTotalMsg1Sent () const { return m_totalMsg1Sent; }
    // 收到有效 Msg4 / MsgB-SUCCESS 的次数
    uint32_t GetTotalMsg4Received () const { return m_totalMsg4Received; }
    // Contention Resolution Timer 超时次数 (= 碰撞导致未收到 Msg4)
    uint32_t GetTotalContentionTimeouts () const { return m_totalContentionTimeouts; }
    // MsgB Response Timer 超时次数 (2 步 RA 专用)
    uint32_t GetTotalMsgBTimeouts () const { return m_totalMsgBTimeouts; }

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
    uint32_t m_currentRaRnti;                  // 当前本次 RA 的 RA-RNTI (区分 PRACH occasion)
    uint16_t m_tcRnti;                         // Msg2 中分配给本 UE 的 TC-RNTI (0 表示尚未获得)
    uint16_t m_cRnti;                          // Msg4 成功后晋升的正式 C-RNTI

    uint8_t m_raAttempt;                       // 当前已发起的 Msg1 次数
    uint8_t m_maxRaAttempts;                   // 最大 Msg1 次数
    uint32_t m_numPreambles;                   // preamble 池大小 (默认 63)
    Time m_raResponseWindow;                   // RA Response Timer 时长
    Time m_contentionResolutionTimeout;        // 竞争解决定时器时长

    EventId m_raResponseTimer;                 // RA Response Timer (等 Msg2)
    EventId m_contentionResolutionTimer;       // 竞争解决定时器 (等 Msg4)
    EventId m_msg3TxEvent;                     // Msg3 延迟发送事件

    Callback<void, Ptr<Packet>>            m_msg3Callback;
    Callback<void, RaResult>                m_raCompleteCallback;

    // ---------- 接入过程统计计数器 ----------
    uint32_t m_totalMsg1Sent;             // 累计发送 Msg1/MsgA 次数
    uint32_t m_totalMsg4Received;         // 累计收到有效 Msg4 / MsgB-SUCCESS 次数
    uint32_t m_totalContentionTimeouts;   // 累计竞争解决超时次数 (碰撞)
    uint32_t m_totalMsgBTimeouts;         // 累计 MsgB 响应超时次数

    // ---------- MAC 层统计 TracedCallbacks ----------
    TracedCallback<uint16_t, uint32_t> m_queueLengthTrace;  // (rnti, bufferBytes)
    TracedCallback<uint16_t, int64_t>  m_queueDelayTrace;   // (rnti, delay_ns)

    // ---------- 信道质量门限 ----------
    static constexpr double RSRP_THRESHOLD_DBM    = -94.5;
    static constexpr double SNR_THRESHOLD_CONSUMER = 1.8;
    static constexpr double SNR_THRESHOLD_PORTABLE = 20.8;

    bool CheckChannelQuality () const;

    // ---------- 2 步随机接入状态 ----------
    RachType m_rachType;                           // 当前 RA 模式 (默认 FOUR_STEP)
    Time m_msgBResponseWindow;                     // MsgB 响应窗口 (UL+DL+margin)
    EventId m_msgBResponseTimer;                   // MsgB 响应定时器
    Callback<void, const MsgA&> m_msgACallback;    // MsgA 发送回调

    // ---------- MAC 层队列时延追踪 ----------
    Time m_bufferArrivalTime;  // 最早未发送数据的入队时间

    // ---------- Msg3 Packet 缓存 (qyh) ----------
    bool m_hasPendingMsg3;
    RrcSetupRequest m_pendingMsg3Request;
    Ptr<Packet> m_pendingMsg3Packet;
};

} // namespace ns3
#endif /* SAT_UT_MAC_H */
