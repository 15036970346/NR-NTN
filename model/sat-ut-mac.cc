// 文件路径：contrib/geo-sat/model/sat-ut-mac.cc
#include "sat-ut-mac.h"
#include "sat-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/spectrum-value.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/antenna-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/uinteger.h"
#include <cmath>
#include <string>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatUtMac");
NS_OBJECT_ENSURE_REGISTERED (SatUtMac);

TypeId SatUtMac::GetTypeId (void) {
    static TypeId tid = TypeId ("ns3::SatUtMac")
        .SetParent<Object> ()
        .SetGroupName ("GeoSat")
        .AddConstructor<SatUtMac> ()
        .AddTraceSource ("QueueLength",
                         "MAC buffer queue length trace (rnti, bufferBytes)",
                         MakeTraceSourceAccessor (&SatUtMac::m_queueLengthTrace),
                         "ns3::TracedCallback::UintUint")
        .AddTraceSource ("QueueDelay",
                         "MAC buffer queue delay trace (rnti, delay_ns)",
                         MakeTraceSourceAccessor (&SatUtMac::m_queueDelayTrace),
                         "ns3::TracedCallback::UintInt64");
    return tid;
}

SatUtMac::SatUtMac ()
    : m_state (MAC_IDLE),
      m_phy (nullptr),
      m_srPending (false),
      m_pendingCqi (1),
      m_pendingHarqAck (false),
      m_currentBufferBytes (0),
      m_timingAdvance (MicroSeconds (0)),
      m_maMode (MultipleAccessMode::ESSA),
      m_isRaInitiated (false),
      m_pendingUlGrant (0),
      m_pendingUlGrantMcs (0),
      m_pendingUlGrantTxPowerDbm (23.0),
      m_ueIdentity (0),
      m_utType (UT_PORTABLE),
      m_rnti (0),
      m_currentPreambleId (0),
      m_currentRaRnti (0),
      m_tcRnti (0),
      m_cRnti (0),
      m_raAttempt (0),
      m_maxRaAttempts (10),
      m_numPreambles (63),
      // RA Response Timer 需 > 单程 RTT (GEO ~300 ms), 留裕量到 1 s
      m_raResponseWindow (MilliSeconds (1000)),
      // 竞争解决定时器: UE 发 Msg3 → gNB → UE 收 Msg4, 需覆盖 1 个 RTT + 处理
      m_contentionResolutionTimeout (MilliSeconds (1500)),
      m_hasPendingMsg3 (false),
      m_totalMsg1Sent (0),
      m_totalMsg4Received (0),
      m_totalContentionTimeouts (0),
      m_totalMsgBTimeouts (0),
      m_rachType (RachType::FOUR_STEP),
      // MsgB 响应窗口: UL prop(300) + processing(2) + DL prop(300) + margin
      m_msgBResponseWindow (MilliSeconds (1500)),
      m_bufferArrivalTime (Seconds (0))
{
    NS_LOG_FUNCTION (this);

    // 默认生成一个随机 40-bit UE Identity
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
    m_ueIdentity = (static_cast<uint64_t> (rng->GetInteger (0, 0xFFFFFFFF)) << 8)
                   | rng->GetInteger (0, 0xFF);
}

SatUtMac::~SatUtMac () {}

void SatUtMac::SetPhy (Ptr<SatPhy> phy) {
    m_phy = phy;
}

void SatUtMac::SwitchState (UtMacState newState) {
    NS_LOG_INFO ("UT MAC State 切换: " << m_state << " -> " << newState);
    m_state = newState;
}

void SatUtMac::ProcessDciAndSchedule (DciInfo dci) {
    NS_LOG_FUNCTION (this);
    
    if (m_state == MAC_IDLE) {
        const bool allowRaMsg3Grant = dci.isUplinkGrant && m_isRaInitiated && m_tcRnti != 0;
        if (!allowRaMsg3Grant) {
            NS_LOG_WARN ("UT 处于 IDLE 状态，忽略 DCI 调度。");
            return;
        }
        NS_LOG_INFO ("UT 处于 IDLE，但当前为 RA Msg3 上行授权，允许继续调度。");
    }

    if (dci.isUplinkGrant) {
        NS_LOG_INFO ("解析到上行授权(PUSCH)。 RB分配: " << dci.rbAllocation 
                     << ", MCS: " << (uint32_t)dci.mcs
                     << ", TxPower: " << dci.txPowerDbm << " dBm"
                     << ", 延迟: " << dci.delayToStart.GetMilliSeconds() << "ms");
        m_srPending = false;
        
        // 如果有待发送的PUCCH信息(PUSCH可以承载BSR)，先发送BSR
        if (m_currentBufferBytes > 0) {
            SendBsr (0, m_currentBufferBytes);
        }
        
        m_txEvent = Simulator::Schedule (dci.delayToStart, 
                                         &SatUtMac::DoTransmit, 
                                         this, 
                                         dci.duration, 
                                         dci.rbAllocation,
                                         dci.mcs,
                                         dci.txPowerDbm);
        SwitchState (MAC_TX);
    } else {
        NS_LOG_INFO ("解析到下行调度(PDSCH)。 RB分配: " << dci.rbAllocation 
                     << ", MCS: " << (uint32_t)dci.mcs
                     << ", 持续时间: " << dci.duration.GetMilliSeconds() << "ms");
        
        Simulator::Schedule (dci.delayToStart, 
                            &SatUtMac::ReceiveData, 
                            this, 
                            Create<Packet>(dci.rbAllocation * 120),  // 模拟接收数据包
                            dci);
        SwitchState (MAC_RX);
    }
}

void SatUtMac::DoTransmit (Time duration, uint32_t rbAllocation, uint8_t mcs, double txPowerDbm) {
    NS_LOG_FUNCTION (this);
    
    SwitchState (MAC_TX);
    
    NS_LOG_INFO ("执行PUSCH数据发送. RB: " << rbAllocation << ", MCS: " << (uint32_t)mcs
                 << ", TxPower: " << txPowerDbm << " dBm"
                 << ", BufferBytes: " << m_currentBufferBytes);
                  
    const uint32_t payloadBytes = (m_currentBufferBytes > 0) ? m_currentBufferBytes : 100;
    Ptr<Packet> macPdu = (m_hasPendingMsg3 && m_pendingMsg3Packet != nullptr) ?
                         m_pendingMsg3Packet->Copy () :
                         Create<Packet> (payloadBytes);

    if (!m_hasPendingMsg3)
      {
        GenericUlMacHeader ulHeader;
        ulHeader.SetRnti (GetActiveRnti ());
        ulHeader.SetPayloadBytes (payloadBytes);
        ulHeader.SetTransmissionTime (Simulator::Now ());
        macPdu->AddHeader (ulHeader);
      }
    
    if (m_phy) {
        Ptr<SatSignalParameters> params = Create<SatSignalParameters> ();
        params->duration = duration;
        
        double bandwidthHz = 180000.0 * rbAllocation;
        double psdValue = txPowerDbm - 10.0 * std::log10 (bandwidthHz);
        
        Ptr<const SpectrumModel> rxSpectrumModel = m_phy->GetRxSpectrumModel ();
        if (rxSpectrumModel)
          {
            Ptr<SpectrumValue> psd = Create<SpectrumValue> (rxSpectrumModel);
            (*psd) = psdValue;
            params->psd = psd;
          }
        
        params->txPhy = m_phy->GetObject<SpectrumPhy> ();
        params->txAntenna = DynamicCast<AntennaModel> (m_phy->GetAntenna ());
        
        m_phy->SendPdu (macPdu, params);
        
        NS_LOG_INFO ("PUSCH MAC PDU已传递给PHY层发送, 数据量: " << macPdu->GetSize () << " bytes");
    } else {
        NS_LOG_ERROR ("PHY层指针为空，无法发送！");
    }

    if (m_hasPendingMsg3 &&
        (!m_msg3Callback.IsNull () || !m_msg3RequestCallback.IsNull ()))
      {
        NS_LOG_INFO ("[Msg3] PUSCH 已发送, 将在 "
                     << (duration + m_timingAdvance).GetMilliSeconds ()
                     << " ms 后送达 gNB");
        Simulator::Schedule (duration + m_timingAdvance,
                             &SatUtMac::DeliverPendingMsg3,
                             this);
      }
    
    // 触发队列时延统计 (从入队到出队)
    if (m_bufferArrivalTime > Seconds (0)) {
        int64_t delayNs = (Simulator::Now () - m_bufferArrivalTime).GetNanoSeconds ();
        m_queueDelayTrace (m_cRnti, delayNs);
    }

    // 发送完成后清空缓冲区
    m_currentBufferBytes = 0;
    m_queueLengthTrace (m_cRnti, 0);
    Simulator::Schedule (duration, &SatUtMac::SwitchState, this, MAC_CONNECTED);
}

void SatUtMac::ReceiveData (Ptr<Packet> packet, const DciInfo& dci)
{
    NS_LOG_FUNCTION (this << packet->GetSize () << (uint32_t)dci.mcs);
    
    SwitchState (MAC_RX);
    
    NS_LOG_INFO ("接收PDSCH下行数据. Packet Size: " << packet->GetSize () 
                 << " bytes, RB: " << dci.rbAllocation 
                 << ", MCS: " << (uint32_t)dci.mcs);
    
    // 模拟PDSCH解码 - 这里应该触发HARQ ACK反馈
    // 简单模拟：假设解码成功
    bool decodeSuccess = true;
    SendHarqAck (decodeSuccess, 1);
    
    NS_LOG_INFO ("ReceiveData: 当前示例未连接上层接收回调，数据在 UT MAC 侧记日志后结束");
    
    Simulator::Schedule (dci.duration, &SatUtMac::SwitchState, this, MAC_CONNECTED);
}

// ==================== PUCCH 功能实现 ====================

void SatUtMac::SendSchedulingRequest ()
{
    NS_LOG_FUNCTION (this);
    const uint16_t activeRnti = GetActiveRnti ();
    if (activeRnti == 0) {
        NS_LOG_WARN ("PUCCH Format 0: 当前没有有效 RNTI，SR 不发送");
        return;
    }
    
    m_srPending = true;
    
    PucchInfo pucch;
    pucch.format = PucchFormatType::FORMAT_0;
    pucch.rnti = activeRnti;
    pucch.srPending = true;
    pucch.transmissionTime = Simulator::Now ();
    
    NS_LOG_INFO ("PUCCH Format 0: SR(调度请求) 发送! 当前缓冲区: " 
                 << m_currentBufferBytes << " bytes");
    
    if (!m_pucchCallback.IsNull ()) {
        m_pucchCallback (pucch);
    }
}

void SatUtMac::SendCqiReport (uint8_t cqi)
{
    NS_LOG_FUNCTION (this << (uint32_t)cqi);
    const uint16_t activeRnti = GetActiveRnti ();
    if (activeRnti == 0) {
        NS_LOG_WARN ("PUCCH Format 1: 当前没有有效 RNTI，CQI 不发送");
        return;
    }
    
    m_pendingCqi = cqi;
    
    PucchInfo pucch;
    pucch.format = PucchFormatType::FORMAT_1;
    pucch.rnti = activeRnti;
    pucch.cqi = cqi;
    pucch.transmissionTime = Simulator::Now ();
    
    NS_LOG_INFO ("PUCCH Format 1: CQI报告发送! CQI=" << (uint32_t)cqi);
    
    if (!m_pucchCallback.IsNull ()) {
        m_pucchCallback (pucch);
    }
}

void SatUtMac::SendHarqAck (bool ack, uint8_t bitmap)
{
    NS_LOG_FUNCTION (this << ack << (uint32_t)bitmap);
    const uint16_t activeRnti = GetActiveRnti ();
    if (activeRnti == 0) {
        NS_LOG_WARN ("PUCCH Format 2: 当前没有有效 RNTI，HARQ 反馈不发送");
        return;
    }
    
    m_pendingHarqAck = ack;
    
    PucchInfo pucch;
    pucch.format = PucchFormatType::FORMAT_2;
    pucch.rnti = activeRnti;
    pucch.harqAck = ack;
    pucch.harqBitMap = bitmap;
    pucch.transmissionTime = Simulator::Now ();
    
    NS_LOG_INFO ("PUCCH Format 2: HARQ " << (ack ? "ACK" : "NACK") 
                 << " 发送! Bitmap: " << (uint32_t)bitmap);
    
    if (!m_pucchCallback.IsNull ()) {
        m_pucchCallback (pucch);
    }
}

void SatUtMac::SendPucch (const PucchInfo& pucchInfo)
{
    NS_LOG_FUNCTION (this);
    
    switch (pucchInfo.format)
    {
        case PucchFormatType::FORMAT_0:
            NS_LOG_INFO ("发送PUCCH Format 0: SR, SR_Pending=" << pucchInfo.srPending);
            break;
        case PucchFormatType::FORMAT_1:
            NS_LOG_INFO ("发送PUCCH Format 1: CQI=" << (uint32_t)pucchInfo.cqi);
            break;
        case PucchFormatType::FORMAT_2:
            NS_LOG_INFO ("发送PUCCH Format 2: HARQ=" << (pucchInfo.harqAck ? "ACK" : "NACK"));
            break;
        default:
            NS_LOG_WARN ("未知的PUCCH格式!");
            break;
    }
    
    if (!m_pucchCallback.IsNull ()) {
        m_pucchCallback (pucchInfo);
    }
}



// ==================== PRACH 功能实现 ====================

void SatUtMac::SendPrachPreamble (uint32_t preambleId, uint8_t format)
{
    NS_LOG_FUNCTION (this << preambleId << (uint32_t)format);
    
    PrachPreamble preamble;
    preamble.preambleId = preambleId;
    preamble.format = format;
    preamble.utType = static_cast<uint8_t> (m_utType);
    preamble.transmissionTime = Simulator::Now ();
    preamble.isRetransmission = false;
    
    NS_LOG_INFO ("PRACH前导码发送! PreambleID=" << preambleId 
                 << ", Format=" << (uint32_t)format);
    
    if (!m_prachCallback.IsNull ()) {
        m_prachCallback (preamble);
    }
}

// ==================== BSR 功能实现 ====================

void SatUtMac::SendBsr (uint8_t lcgId, uint32_t bufferSize)
{
    NS_LOG_FUNCTION (this << (uint32_t)lcgId << bufferSize);
    const uint16_t activeRnti = GetActiveRnti ();
    if (activeRnti == 0) {
        NS_LOG_WARN ("BSR MAC CE: 当前没有有效 RNTI，BSR 不发送");
        return;
    }
    
    BsR_MAC_CE bsr;
    bsr.lcgId = lcgId;
    bsr.bufferSize = bufferSize;
    bsr.rnti = activeRnti;
    
    NS_LOG_INFO ("BSR MAC CE发送! LCG=" << (uint32_t)lcgId 
                 << ", BufferSize=" << bufferSize << " bytes");
    
    if (!m_bsrCallback.IsNull ()) {
        m_bsrCallback (bsr);
    }
}

void SatUtMac::SetBsrCallback (Callback<void, const BsR_MAC_CE&> callback)
{
    m_bsrCallback = callback;
}

void SatUtMac::SetPucchCallback (Callback<void, const PucchInfo&> callback)
{
    m_pucchCallback = callback;
}

void SatUtMac::SetPrachCallback (Callback<void, const PrachPreamble&> callback)
{
    m_prachCallback = callback;
}

void SatUtMac::NotifyDataBuffered (uint32_t bytes)
{
    NS_LOG_FUNCTION (this << bytes);

    // 记录入队时间 (仅在缓冲区从空变非空时更新)
    if (m_currentBufferBytes == 0) {
        m_bufferArrivalTime = Simulator::Now ();
    }
    m_currentBufferBytes += bytes;

    // 触发队列长度统计
    m_queueLengthTrace (m_cRnti, m_currentBufferBytes);

    if (m_state == MAC_CONNECTED && m_srPending == false) {
        NS_LOG_INFO ("数据缓冲达到 " << m_currentBufferBytes << " bytes, 触发SR!");
        SendSchedulingRequest ();
    }
}

// ==================== 4 步随机接入实现 (Msg1→Msg2→Msg3→Msg4) ====================

void SatUtMac::DoRandomAccess (uint32_t preambleId, uint8_t format)
{
    NS_LOG_FUNCTION (this << preambleId << (uint32_t)format);

    if (m_state == MAC_CONNECTED && m_cRnti != 0) {
        NS_LOG_WARN ("UT UE-Id=0x" << std::hex << m_ueIdentity << std::dec
                     << " 已处于 RRC_CONNECTED, 无需再发起 RA");
        return;
    }
    if (m_state == MAC_TX || m_state == MAC_RX) {
        NS_LOG_WARN ("UT MAC 正在收发, 无法发起随机接入!");
        return;
    }

    // 取消任何残留的 RA 定时器 (防止重入)
    if (m_raResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_raResponseTimer);
    }
    if (m_contentionResolutionTimer.IsRunning ()) {
        Simulator::Cancel (m_contentionResolutionTimer);
    }
    if (m_msg3TxEvent.IsRunning ()) {
        Simulator::Cancel (m_msg3TxEvent);
    }

    m_isRaInitiated = true;
    m_raAttempt++;
    m_totalMsg1Sent++;            // 统计: 每次发 Msg1 都计入
    m_tcRnti = 0;   // 清空上一次的 TC-RNTI

    // 若调用者未指定 preambleId, 随机挑选 (1..m_numPreambles)
    if (preambleId == 0) {
        Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
        preambleId = rng->GetInteger (1, m_numPreambles);
    }
    m_currentPreambleId = preambleId;

    // 计算 RA-RNTI: 由 PRACH 发送时刻派生 (毫秒级精度)
    // 同一 RO 内的 UE (100μs 窗口) 在同一毫秒内 → 相同 raRnti → 正确竞争
    // 不同 RO (间隔 ≥1ms) → 不同 raRnti → 互不干扰
    m_currentRaRnti = 1 + static_cast<uint32_t> (Simulator::Now ().GetMilliSeconds () % 0xFFF0);

    NS_LOG_INFO ("=== [RA #" << (uint32_t)m_raAttempt << "] UE-Id=0x"
                 << std::hex << m_ueIdentity << std::dec
                 << " 发起 4 步随机接入 ===");

    // 读取 PHY Timing Advance (若有)
    Ptr<SatUtPhy> utPhy = DynamicCast<SatUtPhy> (m_phy);
    if (utPhy) {
        m_timingAdvance = utPhy->GetTimingAdvance ();
    }

    // ------ Msg1: PRACH Preamble ------
    NS_LOG_INFO ("[Msg1] 发送 PRACH preamble PreambleID=" << preambleId
                 << " RA-RNTI=" << m_currentRaRnti
                 << " Format=" << (uint32_t)format);
    PrachPreamble preamble;
    preamble.rnti             = 0;              // 尚未分配
    preamble.preambleId       = preambleId;
    preamble.format           = format;
    preamble.utType           = static_cast<uint8_t> (m_utType);
    preamble.transmissionTime = Simulator::Now ();
    preamble.isRetransmission = (m_raAttempt > 1);
    preamble.raRnti           = m_currentRaRnti;
    if (!m_prachCallback.IsNull ()) {
        m_prachCallback (preamble);
    } else {
        NS_LOG_WARN ("[Msg1] PRACH callback 未设置, 无法上送 preamble!");
    }

    // 启动 RA Response Timer, 等待 Msg2
    NS_LOG_INFO ("[RA] 启动 RA Response Timer: "
                 << m_raResponseWindow.GetMilliSeconds () << "ms");
    m_raResponseTimer = Simulator::Schedule (m_raResponseWindow,
                                             &SatUtMac::OnRaResponseTimeout,
                                             this);
}

void SatUtMac::ReceiveRar (const RarMessage& rar)
{
    NS_LOG_FUNCTION (this << rar.preambleId << rar.tcRnti);

    // 只处理与本 UE RA-RNTI + preambleId 均匹配的 RAR; 其它直接忽略
    // RA-RNTI 区分不同 PRACH occasion, 防止跨 RO 的 RAR 误匹配
    if (!m_isRaInitiated || rar.raRnti != m_currentRaRnti
        || rar.preambleId != m_currentPreambleId) {
        return;
    }

    // 若已经有 TC-RNTI (重复 RAR), 直接忽略
    if (m_tcRnti != 0) {
        return;
    }

    NS_LOG_INFO ("[Msg2] 收到匹配的 RAR: PreambleID=" << rar.preambleId
                 << " TC-RNTI=0x" << std::hex << rar.tcRnti << std::dec
                 << " TA=" << rar.timingAdvance.GetMicroSeconds () << "μs"
                 << " UL Grant=" << rar.ulGrantRbs << " PRB"
                 << " TxPower=" << rar.ulGrantTxPowerDbm << " dBm");

    // 取消 RA Response Timer
    if (m_raResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_raResponseTimer);
    }

    // 保存 RAR 信息
    m_tcRnti            = rar.tcRnti;
    m_timingAdvance     = rar.timingAdvance;
    m_pendingUlGrant    = rar.ulGrantRbs;
    m_pendingUlGrantMcs = rar.ulGrantMcs;
    m_pendingUlGrantTxPowerDbm = rar.ulGrantTxPowerDbm;

    // 在 Msg3 延迟后组装并发送 RRCSetupRequest
    m_msg3TxEvent = Simulator::Schedule (rar.msg3DelayToStart,
                                         &SatUtMac::SendMsg3, this);
}

void SatUtMac::SendMsg3 ()
{
    NS_LOG_FUNCTION (this);

    if (m_tcRnti == 0) {
        NS_LOG_WARN ("[Msg3] 尚未获得 TC-RNTI, 放弃发送");
        return;
    }

    // 组装 DciInfo (UE 侧基于 RAR 的 UL Grant 推导)
    DciInfo dci;
    dci.isUplinkGrant = true;
    dci.rbAllocation  = m_pendingUlGrant;
    dci.mcs           = m_pendingUlGrantMcs;
    dci.txPowerDbm    = m_pendingUlGrantTxPowerDbm;
    dci.delayToStart  = MicroSeconds (0);
    dci.duration      = MilliSeconds (1);

    NS_LOG_INFO ("[Msg3] 组装 DciInfo: RB=" << dci.rbAllocation
                 << " MCS=" << (uint32_t)dci.mcs
                 << " TxPower=" << dci.txPowerDbm << " dBm"
                 << " TA补偿=" << m_timingAdvance.GetMicroSeconds () << "μs");

    // 组装 RRCSetupRequest
    RrcSetupRequest req;
    req.tcRnti            = m_tcRnti;
    req.ueIdentity        = m_ueIdentity;
    req.establishmentCause = 2;                  // mo-Data
    req.preambleIdUsed    = m_currentPreambleId;
    req.transmissionTime  = Simulator::Now ();

    NS_LOG_INFO ("[Msg3] 发送 RRCSetupRequest: TC-RNTI=0x" << std::hex << m_tcRnti
                 << " UE-Id=0x" << m_ueIdentity << std::dec);
    m_pendingMsg3Request = req;
    m_hasPendingMsg3 = true;
    Msg3MacHeader msg3Header;
    msg3Header.SetRequest (req);
    m_pendingMsg3Packet = Create<Packet> ();
    m_pendingMsg3Packet->AddHeader (msg3Header);
    ProcessDciAndSchedule (dci);

    // 启动竞争解决定时器, 等待 Msg4
    NS_LOG_INFO ("[RA] 启动 Contention Resolution Timer: "
                 << m_contentionResolutionTimeout.GetMilliSeconds () << "ms");
    m_contentionResolutionTimer = Simulator::Schedule (m_contentionResolutionTimeout,
                                                       &SatUtMac::OnContentionResolutionTimeout,
                                                       this);

    SwitchState (MAC_TX);
}

void SatUtMac::DeliverPendingMsg3 ()
{
    NS_LOG_FUNCTION (this);

    if (!m_hasPendingMsg3)
      {
        return;
      }

    bool delivered = false;

    if (!m_msg3Callback.IsNull () && m_pendingMsg3Packet != nullptr)
      {
        NS_LOG_INFO ("[Msg3] PUSCH 已到达 gNB, 开始递交 RRCSetupRequest: TC-RNTI=0x"
                     << std::hex << m_pendingMsg3Request.tcRnti
                     << " UE-Id=0x" << m_pendingMsg3Request.ueIdentity << std::dec);
        m_msg3Callback (m_pendingMsg3Packet->Copy ());
        delivered = true;
      }

    if (!m_msg3RequestCallback.IsNull ())
      {
        m_msg3RequestCallback (m_pendingMsg3Request);
        delivered = true;
      }

    if (!delivered)
      {
        NS_LOG_WARN ("[Msg3] msg3 callback 未设置, 无法向 gNB 递交 Msg3!");
      }

    m_hasPendingMsg3 = false;
    m_pendingMsg3Packet = nullptr;
}

void SatUtMac::ReceiveMsg4 (const RrcSetupMessage& msg4)
{
    NS_LOG_FUNCTION (this << msg4.tcRnti);

    // 仅处理与本 UE TC-RNTI 匹配的 Msg4
    if (!m_isRaInitiated || msg4.tcRnti != m_tcRnti || m_tcRnti == 0) {
        return;
    }

    NS_LOG_INFO ("[Msg4] 收到 RRCSetup: TC-RNTI=0x" << std::hex << msg4.tcRnti
                 << " echoed UE-Id=0x" << msg4.echoedUeIdentity
                 << " (本地 UE-Id=0x" << m_ueIdentity << ")" << std::dec);

    // 关键: 竞争解决 —— 比对回显的 ueIdentity
    if (msg4.echoedUeIdentity != m_ueIdentity) {
        NS_LOG_WARN ("[Msg4] 回显的 UE-Id 与本地不匹配, 竞争失败!");
        // 取消定时器, 交由超时/失败路径处理 (也可直接重传)
        if (m_contentionResolutionTimer.IsRunning ()) {
            Simulator::Cancel (m_contentionResolutionTimer);
        }
        AbortOrRetryRa ("Msg4 UE-Id mismatch");
        return;
    }

    // 竞争解决成功 → 检查信道质量后进入 RRC_CONNECTED
    if (m_contentionResolutionTimer.IsRunning ()) {
        Simulator::Cancel (m_contentionResolutionTimer);
    }

    // 信道质量门限判定 (Msg4 识别 + CQI/RSRP 双重判定)
    if (!CheckChannelQuality ()) {
        NS_LOG_WARN ("[Msg4] 竞争解决成功但信道质量不足，接入失败 (RA_FAILED_POOR_CHANNEL)");
        m_isRaInitiated = false;
        m_raAttempt = 0;
        SwitchState (MAC_IDLE);
        if (!m_raCompleteCallback.IsNull ()) {
            m_raCompleteCallback (RA_FAILED_POOR_CHANNEL);
        }
        return;
    }

    m_totalMsg4Received++;        // 统计: 收到有效 Msg4 且信道质量达标
    m_cRnti = msg4.cRnti;
    m_rnti = m_cRnti;
    m_isRaInitiated = false;
    m_raAttempt = 0;

    NS_LOG_INFO ("=== [RA] 4 步随机接入成功! C-RNTI=0x" << std::hex << m_cRnti
                 << " UE-Id=0x" << m_ueIdentity << std::dec << " ===");
    SwitchState (MAC_CONNECTED);

    if (!m_raCompleteCallback.IsNull ()) {
        m_raCompleteCallback (RA_SUCCESS);
    }
}

void SatUtMac::OnRaResponseTimeout ()
{
    NS_LOG_FUNCTION (this);
    NS_LOG_WARN ("[RA] RA Response Timer 超时 (未收到 RAR)");
    AbortOrRetryRa ("RAR timeout");
}

void SatUtMac::OnContentionResolutionTimeout ()
{
    NS_LOG_FUNCTION (this);
    NS_LOG_WARN ("[RA] Contention Resolution Timer 超时 (未收到 Msg4, 很可能是 Msg3 发生碰撞)");
    m_totalContentionTimeouts++;  // 统计: 碰撞导致 Msg4 超时
    AbortOrRetryRa ("Contention resolution timeout");
}

void SatUtMac::AbortOrRetryRa (const std::string& reason)
{
    NS_LOG_FUNCTION (this << reason);

    // 清理本次尝试的状态
    m_tcRnti = 0;
    if (m_raResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_raResponseTimer);
    }
    if (m_contentionResolutionTimer.IsRunning ()) {
        Simulator::Cancel (m_contentionResolutionTimer);
    }
    if (m_msg3TxEvent.IsRunning ()) {
        Simulator::Cancel (m_msg3TxEvent);
    }
    if (m_msgBResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_msgBResponseTimer);
    }

    // 回到 IDLE, 否则若当前处于 MAC_TX (Msg3 之后) 将阻塞 DoRandomAccess 重试
    SwitchState (MAC_IDLE);

    if (m_raAttempt >= m_maxRaAttempts) {
        NS_LOG_ERROR ("[RA] 达到最大重传次数 " << (uint32_t)m_maxRaAttempts
                      << ", 放弃接入! 原因=" << reason);
        m_isRaInitiated = false;
        m_raAttempt = 0;
        SwitchState (MAC_IDLE);
        if (!m_raCompleteCallback.IsNull ()) {
            m_raCompleteCallback (RA_FAILED_MAX_ATTEMPTS);
        }
        return;
    }

    // 退避后重传 Msg1: 指数退避, 基础窗口 40ms (为避免连续碰撞)
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
    double backoffMs = rng->GetValue (0.0, 40.0 * m_raAttempt);
    Time backoff = MilliSeconds (static_cast<int64_t> (backoffMs));

    NS_LOG_INFO ("[RA] 准备重传 Msg1, 退避 " << backoff.GetMilliSeconds ()
                 << "ms, 下次 attempt=" << (uint32_t)(m_raAttempt + 1)
                 << " / " << (uint32_t)m_maxRaAttempts);

    m_isRaInitiated = false;  // 将由 InitiateRandomAccess 重新置位
    Simulator::Schedule (backoff, &SatUtMac::InitiateRandomAccess, this,
                         static_cast<uint32_t> (0),  // 重新随机选 preambleId
                         static_cast<uint8_t> (0));
}

Time SatUtMac::GetTimingAdvance () const
{
    return m_timingAdvance;
}

void SatUtMac::SetMultipleAccessMode (MultipleAccessMode mode)
{
    m_maMode = mode;
    NS_LOG_INFO ("MAC多址接入模式设置为: " << (mode == MultipleAccessMode::ESSA ? "ESSA" :
                                                   mode == MultipleAccessMode::SLOTTED_ALOHA ? "Slotted ALOHA" : "Other"));
}

bool SatUtMac::IsConnected () const
{
    // 真正接入完成的判据: 完成 4 步 RA 并拿到 C-RNTI
    return m_state == MAC_CONNECTED && m_cRnti != 0;
}

void SatUtMac::SetUeIdentity (uint64_t ueIdentity)
{
    m_ueIdentity = ueIdentity;
}

void SatUtMac::SetUtType (UtType utType)
{
    m_utType = utType;
}

uint64_t SatUtMac::GetUeIdentity () const
{
    return m_ueIdentity;
}

void SatUtMac::SetRnti (uint16_t rnti)
{
    m_rnti = rnti;
}

uint16_t SatUtMac::GetActiveRnti () const
{
    if (m_cRnti != 0) {
        return m_cRnti;
    }

    return m_rnti;
}

void SatUtMac::SetRaTimers (Time raResponseWindow, Time contentionResolutionTimer, uint8_t maxAttempts)
{
    m_raResponseWindow = raResponseWindow;
    m_contentionResolutionTimeout = contentionResolutionTimer;
    m_maxRaAttempts = maxAttempts;
}

void SatUtMac::SetNumPreambles (uint32_t n)
{
    m_numPreambles = n;
}

uint32_t SatUtMac::GetNumPreambles () const
{
    return m_numPreambles;
}

void SatUtMac::SetMsg3Callback (Callback<void, const RrcSetupRequest&> callback)
{
    m_msg3RequestCallback = callback;
}

void SatUtMac::SetMsg3Callback (Callback<void, Ptr<Packet>> callback)
{
    m_msg3Callback = callback;
}

void SatUtMac::SetRaCompleteCallback (Callback<void, RaResult> callback)
{
    m_raCompleteCallback = callback;
}

// ==================== 2 步随机接入实现 (MsgA→MsgB) ====================

void SatUtMac::InitiateRandomAccess (uint32_t preambleId, uint8_t format)
{
    NS_LOG_FUNCTION (this << preambleId << (uint32_t)format);

    if (m_rachType == RachType::TWO_STEP) {
        DoTwoStepRandomAccess (preambleId, format);
    } else {
        DoRandomAccess (preambleId, format);
    }
}

void SatUtMac::DoTwoStepRandomAccess (uint32_t preambleId, uint8_t format)
{
    NS_LOG_FUNCTION (this << preambleId << (uint32_t)format);

    if (m_state == MAC_CONNECTED && m_cRnti != 0) {
        NS_LOG_WARN ("UT UE-Id=0x" << std::hex << m_ueIdentity << std::dec
                     << " 已处于 RRC_CONNECTED, 无需再发起 RA");
        return;
    }
    if (m_state == MAC_TX || m_state == MAC_RX) {
        NS_LOG_WARN ("UT MAC 正在收发, 无法发起随机接入!");
        return;
    }

    if (m_raResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_raResponseTimer);
    }
    if (m_contentionResolutionTimer.IsRunning ()) {
        Simulator::Cancel (m_contentionResolutionTimer);
    }
    if (m_msgBResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_msgBResponseTimer);
    }
    if (m_msg3TxEvent.IsRunning ()) {
        Simulator::Cancel (m_msg3TxEvent);
    }

    m_isRaInitiated = true;
    m_raAttempt++;
    m_totalMsg1Sent++;
    m_tcRnti = 0;

    if (preambleId == 0) {
        Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
        preambleId = rng->GetInteger (1, m_numPreambles);
    }
    m_currentPreambleId = preambleId;
    m_currentRaRnti = 1 + static_cast<uint32_t> (Simulator::Now ().GetMilliSeconds () % 0xFFF0);

    Ptr<SatUtPhy> utPhy = DynamicCast<SatUtPhy> (m_phy);
    if (utPhy) {
        m_timingAdvance = utPhy->GetTimingAdvance ();
    }

    MsgA msgA;
    msgA.preambleId = preambleId;
    msgA.format = format;
    msgA.ueIdentity = m_ueIdentity;
    msgA.bufferStatus = m_currentBufferBytes;
    msgA.establishmentCause = 2;  // mo-Data
    msgA.transmissionTime = Simulator::Now ();
    msgA.isRetransmission = (m_raAttempt > 1);
    msgA.raRnti = m_currentRaRnti;

    NS_LOG_INFO ("[MsgA] 发送 2 步 RA MsgA: PreambleID=" << preambleId
                 << " UE-Id=0x" << std::hex << m_ueIdentity << std::dec
                 << " Format=" << (uint32_t)format);

    if (!m_msgACallback.IsNull ()) {
        m_msgACallback (msgA);
    } else {
        NS_LOG_WARN ("[MsgA] MsgA callback 未设置, 无法上送 MsgA!");
    }

    m_msgBResponseTimer = Simulator::Schedule (m_msgBResponseWindow,
                                                &SatUtMac::OnMsgBResponseTimeout,
                                                this);
    SwitchState (MAC_WAITING_MSGB);
}

void SatUtMac::ReceiveMsgB (const MsgB& msgB)
{
    NS_LOG_FUNCTION (this << msgB.preambleId << (uint32_t)msgB.type);

    if (!m_isRaInitiated || msgB.raRnti != m_currentRaRnti
        || msgB.preambleId != m_currentPreambleId) {
        return;
    }

    if (m_msgBResponseTimer.IsRunning ()) {
        Simulator::Cancel (m_msgBResponseTimer);
    }

    if (msgB.type == MsgBType::SUCCESS_RAR) {
        if (msgB.echoedUeIdentity != m_ueIdentity) {
            NS_LOG_WARN ("[MsgB] SUCCESS_RAR 但 UE-Id 不匹配, 忽略");
            m_msgBResponseTimer = Simulator::Schedule (m_msgBResponseWindow,
                                                        &SatUtMac::OnMsgBResponseTimeout,
                                                        this);
            return;
        }

        if (!CheckChannelQuality ()) {
            NS_LOG_WARN ("[MsgB] SUCCESS_RAR 但信道质量不足，接入失败 (RA_FAILED_POOR_CHANNEL)");
            m_isRaInitiated = false;
            m_raAttempt = 0;
            SwitchState (MAC_IDLE);
            if (!m_raCompleteCallback.IsNull ()) {
                m_raCompleteCallback (RA_FAILED_POOR_CHANNEL);
            }
            return;
        }

        m_cRnti = msgB.cRnti;
        m_rnti = m_cRnti;
        m_timingAdvance = msgB.timingAdvance;
        m_isRaInitiated = false;
        m_raAttempt = 0;
        m_totalMsg4Received++;

        NS_LOG_INFO ("=== [RA] 2 步随机接入成功! C-RNTI=0x" << std::hex << m_cRnti
                     << " UE-Id=0x" << m_ueIdentity << std::dec << " ===");
        SwitchState (MAC_CONNECTED);

        if (!m_raCompleteCallback.IsNull ()) {
            m_raCompleteCallback (RA_SUCCESS);
        }
    } else {
        NS_LOG_INFO ("[MsgB] FALLBACK_RAR: PreambleID=" << msgB.preambleId
                     << " TC-RNTI=0x" << std::hex << msgB.tcRnti << std::dec
                     << " UL Grant=" << msgB.ulGrantRbs << " PRB"
                     << " -> 回退到 4 步 Msg3/Msg4");

        m_tcRnti = msgB.tcRnti;
        m_timingAdvance = msgB.timingAdvance;
        m_pendingUlGrant = msgB.ulGrantRbs;
        m_pendingUlGrantMcs = msgB.ulGrantMcs;
        m_pendingUlGrantTxPowerDbm = msgB.ulGrantTxPowerDbm;

        m_msg3TxEvent = Simulator::Schedule (MicroSeconds (500),
                                              &SatUtMac::SendMsg3, this);
    }
}

void SatUtMac::OnMsgBResponseTimeout ()
{
    NS_LOG_FUNCTION (this);
    NS_LOG_WARN ("[RA] MsgB Response Timer 超时 (未收到 MsgB)");
    m_totalMsgBTimeouts++;
    m_totalContentionTimeouts++;
    AbortOrRetryRa ("MsgB timeout");
}

void SatUtMac::SetRachType (RachType rachType)
{
    m_rachType = rachType;
    NS_LOG_INFO ("RA 模式设置为: " << (rachType == RachType::TWO_STEP ? "2-STEP" : "4-STEP"));
}

RachType SatUtMac::GetRachType () const
{
    return m_rachType;
}

void SatUtMac::SetMsgACallback (Callback<void, const MsgA&> callback)
{
    m_msgACallback = callback;
}

UtType SatUtMac::GetUtType () const
{
    return m_utType;
}

// ==================== 信道质量门限判定 ====================

bool SatUtMac::CheckChannelQuality () const
{
    NS_LOG_FUNCTION (this);

    Ptr<SatUtPhy> utPhy = DynamicCast<SatUtPhy> (m_phy);
    if (!utPhy) {
        NS_LOG_WARN ("[CQ-Check] SatUtPhy 未绑定，跳过信道质量检查");
        return true;
    }

    double rsrp = utPhy->GetLastRsrp ();
    double sinrDb = utPhy->GetLastCalculatedSinr ();

    double snrThreshold = (m_utType == UT_PORTABLE)
                              ? SNR_THRESHOLD_PORTABLE
                              : SNR_THRESHOLD_CONSUMER;

    bool rsrpOk = (rsrp >= RSRP_THRESHOLD_DBM);
    bool snrOk = (sinrDb >= snrThreshold);

    NS_LOG_INFO ("[CQ-Check] UtType=" << (m_utType == UT_PORTABLE ? "PORTABLE" : "CONSUMER")
                 << " | RSRP=" << rsrp << " dBm (thr=" << RSRP_THRESHOLD_DBM << ")"
                 << " | SNR=" << sinrDb << " dB (thr=" << snrThreshold << ")"
                 << " | " << (rsrpOk && snrOk ? "PASS" : "FAIL"));

    return rsrpOk && snrOk;
}

} // namespace ns3
