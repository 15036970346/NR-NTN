/*
 * 文件: contrib/geo-sat/model/ntn-rlc.cc
 *
 * NtnRlc 实现 (TM / UM / AM)。详见头文件注释。
 */
#include "ntn-rlc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/string.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnRlc");

// ===========================================================================
// NtnRlcSnTag
// ===========================================================================
NS_OBJECT_ENSURE_REGISTERED (NtnRlcSnTag);

TypeId
NtnRlcSnTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnRlcSnTag")
    .SetParent<Tag> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnRlcSnTag> ();
  return tid;
}
TypeId NtnRlcSnTag::GetInstanceTypeId (void) const { return GetTypeId (); }
NtnRlcSnTag::NtnRlcSnTag () = default;
NtnRlcSnTag::NtnRlcSnTag (uint16_t sn, bool isStatusPdu)
  : m_sn (sn), m_isStatusPdu (isStatusPdu) {}

uint32_t NtnRlcSnTag::GetSerializedSize (void) const { return sizeof (uint16_t) + sizeof (uint8_t); }
void NtnRlcSnTag::Serialize   (TagBuffer i) const    { i.WriteU16 (m_sn); i.WriteU8 (m_isStatusPdu ? 1 : 0); }
void NtnRlcSnTag::Deserialize (TagBuffer i)          { m_sn = i.ReadU16 (); m_isStatusPdu = (i.ReadU8 () != 0); }
void NtnRlcSnTag::Print       (std::ostream& os) const
{
  os << "NtnRlcSnTag(sn=" << m_sn << (m_isStatusPdu ? ", STATUS" : "") << ")";
}

// ===========================================================================
// NtnRlcTimestampTag
// ===========================================================================
NS_OBJECT_ENSURE_REGISTERED (NtnRlcTimestampTag);

TypeId
NtnRlcTimestampTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnRlcTimestampTag")
    .SetParent<Tag> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnRlcTimestampTag> ();
  return tid;
}
TypeId NtnRlcTimestampTag::GetInstanceTypeId (void) const { return GetTypeId (); }
NtnRlcTimestampTag::NtnRlcTimestampTag () : m_t (Seconds (0)) {}
NtnRlcTimestampTag::NtnRlcTimestampTag (Time t) : m_t (t) {}
uint32_t NtnRlcTimestampTag::GetSerializedSize (void) const { return sizeof (int64_t); }
void NtnRlcTimestampTag::Serialize   (TagBuffer i) const    { i.WriteU64 (static_cast<uint64_t> (m_t.GetNanoSeconds ())); }
void NtnRlcTimestampTag::Deserialize (TagBuffer i)          { m_t = NanoSeconds (static_cast<int64_t> (i.ReadU64 ())); }
void NtnRlcTimestampTag::Print       (std::ostream& os) const { os << "NtnRlcTimestampTag(t=" << m_t.GetSeconds () << "s)"; }

// ===========================================================================
// NtnRlc
// ===========================================================================
NS_OBJECT_ENSURE_REGISTERED (NtnRlc);

TypeId
NtnRlc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnRlc")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnRlc> ()
    .AddAttribute ("AmDropProb",
                   "AM 模式接收端模拟丢包概率 [0,1]; 仅 mode=AM 时生效",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&NtnRlc::m_amDropProb),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("PollRetransmitTimer",
                   "AM 发送侧 t-PollRetransmit, 无 ACK 时对未确认 SDU 主动重传",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&NtnRlc::m_amPollRetransmit),
                   MakeTimeChecker ())
    .AddAttribute ("StatusProhibitTimer",
                   "AM 接收侧 t-StatusProhibit, 限制 Status PDU 发送频率",
                   TimeValue (MilliSeconds (50)),
                   MakeTimeAccessor (&NtnRlc::m_amStatusProhibit),
                   MakeTimeChecker ())
    .AddTraceSource ("TxPdu",
                     "RLC PDU 下推 (含重传), 签名 (rnti, lcid, size)",
                     MakeTraceSourceAccessor (&NtnRlc::m_txTrace),
                     "ns3::TracedCallback::Uint16Uint8Uint32")
    .AddTraceSource ("RxPdu",
                     "RLC PDU 成功上递, 签名 (rnti, lcid, size, delay_ns)",
                     MakeTraceSourceAccessor (&NtnRlc::m_rxTrace),
                     "ns3::TracedCallback::Uint16Uint8Uint32Uint64")
    .AddTraceSource ("RetxPdu",
                     "AM 显式重传发生, 签名 (rnti, lcid, size)",
                     MakeTraceSourceAccessor (&NtnRlc::m_retxTrace),
                     "ns3::TracedCallback::Uint16Uint8Uint32");
  return tid;
}

NtnRlc::NtnRlc ()
{
  NS_LOG_FUNCTION (this);
  m_dropRng = CreateObject<UniformRandomVariable> ();
  m_dropRng->SetAttribute ("Min", DoubleValue (0.0));
  m_dropRng->SetAttribute ("Max", DoubleValue (1.0));
}

NtnRlc::~NtnRlc () = default;

void
NtnRlc::DoDispose ()
{
  if (m_amStatusEvent.IsRunning ()) Simulator::Cancel (m_amStatusEvent);
  if (m_amPollEvent.IsRunning ())   Simulator::Cancel (m_amPollEvent);
  m_amTxBuffer.clear ();
  m_amReorderBuffer.clear ();
  m_recvCb = Callback<void, Ptr<Packet>> ();
  m_txCb   = Callback<void, Ptr<Packet>> ();
  Object::DoDispose ();
}

void NtnRlc::SetRnti (uint16_t rnti) { m_rnti = rnti; }
void NtnRlc::SetLcid (uint8_t  lcid) { m_lcid = lcid; }
void NtnRlc::SetMode (NtnRlcMode mode) { m_mode = mode; }
std::string
NtnRlc::GetModeStr () const
{
  switch (m_mode)
    {
    case NtnRlcMode::TM: return "TM";
    case NtnRlcMode::UM: return "UM";
    case NtnRlcMode::AM: return "AM";
    }
  return "??";
}

void NtnRlc::SetReceiveCallback  (Callback<void, Ptr<Packet>> cb) { m_recvCb = cb; }
void NtnRlc::SetTransmitCallback (Callback<void, Ptr<Packet>> cb) { m_txCb   = cb; }

void NtnRlc::SetAmDropProb           (double p)  { m_amDropProb = p; }
void NtnRlc::SetPollRetransmitTimer  (Time t)    { m_amPollRetransmit = t; }
void NtnRlc::SetStatusProhibitTimer  (Time t)    { m_amStatusProhibit = t; }

// =========================================================================
// 上层入口 (PDCP -> RLC)
// =========================================================================
void
NtnRlc::TransmitPdcpPdu (Ptr<Packet> p)
{
  if (!p) return;
  NS_LOG_FUNCTION (this << p->GetSize () << GetModeStr ());

  // 打 Tx 时间戳 tag, Rx 侧用来算端到端 delay
  NtnRlcTimestampTag tsTag (Simulator::Now ());
  p->AddPacketTag (tsTag);

  switch (m_mode)
    {
    case NtnRlcMode::TM: TmTransmit (p); break;
    case NtnRlcMode::UM: UmTransmit (p); break;
    case NtnRlcMode::AM: AmTransmit (p); break;
    }
}

// =========================================================================
// 下层入口 (MAC -> RLC)
// =========================================================================
void
NtnRlc::ReceiveRlcPdu (Ptr<Packet> p)
{
  if (!p) return;
  NS_LOG_FUNCTION (this << p->GetSize () << GetModeStr ());

  switch (m_mode)
    {
    case NtnRlcMode::TM: TmReceive (p); break;
    case NtnRlcMode::UM: UmReceive (p); break;
    case NtnRlcMode::AM: AmReceive (p); break;
    }
}

// =========================================================================
// TM
// =========================================================================
void
NtnRlc::TmTransmit (Ptr<Packet> p)
{
  uint32_t sz = p->GetSize ();
  m_txBytes += sz;
  m_txPduCount++;
  m_txTrace (m_rnti, m_lcid, sz);
  if (!m_txCb.IsNull ()) m_txCb (p);
}

void
NtnRlc::TmReceive (Ptr<Packet> p)
{
  DeliverToUpper (p);
}

// =========================================================================
// UM
// =========================================================================
void
NtnRlc::UmTransmit (Ptr<Packet> p)
{
  uint16_t sn = m_amTxNextSn++;       // 跟 AM 共用 SN 计数 (本实例只跑一种 mode, 无冲突)
  NtnRlcSnTag tag (sn, false);
  p->AddPacketTag (tag);

  uint32_t sz = p->GetSize ();
  m_txBytes += sz;
  m_txPduCount++;
  m_txTrace (m_rnti, m_lcid, sz);
  if (!m_txCb.IsNull ()) m_txCb (p);
}

void
NtnRlc::UmReceive (Ptr<Packet> p)
{
  NtnRlcSnTag tag;
  if (!p->PeekPacketTag (tag))
    {
      NS_LOG_WARN ("UM Rx: missing SN tag, drop");
      return;
    }
  uint16_t sn = tag.GetSn ();
  // 单调推进: sn < expected 视为乱序/重复, drop
  if (sn < m_umNextExpectedSn)
    {
      NS_LOG_DEBUG ("UM Rx drop stale sn=" << sn << " (expected>=" << m_umNextExpectedSn << ")");
      return;
    }
  m_umNextExpectedSn = sn + 1;
  DeliverToUpper (p);
}

// =========================================================================
// AM
// =========================================================================
void
NtnRlc::AmTransmit (Ptr<Packet> p)
{
  uint16_t sn = m_amTxNextSn++;
  // 缓存原 SDU (重传源, 用 Copy 防上层后续改动)
  m_amTxBuffer[sn] = p->Copy ();
  AmDoTransmit (p, sn, /*isRetx=*/false);

  // 触发 / 重启 t-PollRetransmit (兜底: 无 Status 时也会主动重发)
  if (m_amPollEvent.IsRunning ()) Simulator::Cancel (m_amPollEvent);
  m_amPollEvent = Simulator::Schedule (m_amPollRetransmit, &NtnRlc::AmPollRetransmitTimeout, this);
}

void
NtnRlc::AmDoTransmit (Ptr<Packet> p, uint16_t sn, bool isRetx)
{
  NtnRlcSnTag tag (sn, false);
  p->ReplacePacketTag (tag);    // 重传时覆盖原 tag

  uint32_t sz = p->GetSize ();
  m_txBytes += sz;
  m_txPduCount++;
  m_txTrace (m_rnti, m_lcid, sz);
  if (isRetx)
    {
      m_retxCount++;
      m_retxTrace (m_rnti, m_lcid, sz);
    }
  if (!m_txCb.IsNull ()) m_txCb (p);
}

void
NtnRlc::AmReceive (Ptr<Packet> p)
{
  NtnRlcSnTag tag;
  if (!p->PeekPacketTag (tag))
    {
      NS_LOG_WARN ("AM Rx: missing SN tag, drop");
      return;
    }

  if (tag.IsStatusPdu ())
    {
      AmReceiveStatusPdu (p);
      return;
    }

  // Data PDU: 按 m_amDropProb 模拟丢包 (制造重传场景)
  if (m_amDropProb > 0.0 && m_dropRng->GetValue () < m_amDropProb)
    {
      uint16_t sn = tag.GetSn ();
      NS_LOG_DEBUG ("AM Rx drop sn=" << sn << " (drop prob)");
      m_amMissingSn.insert (sn);
      AmScheduleStatusReport ();
      return;
    }
  AmReceiveDataPdu (p, tag.GetSn ());
}

void
NtnRlc::AmReceiveDataPdu (Ptr<Packet> p, uint16_t sn)
{
  // 重复包 (重传到达后再来一次): 已收过 -> drop
  if (sn < m_amRxNextSn)
    {
      NS_LOG_DEBUG ("AM Rx drop dup sn=" << sn);
      return;
    }
  // 缓存 + 按序投递
  m_amReorderBuffer[sn] = p;
  // 若是缺失列表里的 SN, 表示重传成功到达, 从待 NACK 集合移除
  m_amMissingSn.erase (sn);

  while (m_amReorderBuffer.count (m_amRxNextSn))
    {
      Ptr<Packet> outPkt = m_amReorderBuffer[m_amRxNextSn];
      m_amReorderBuffer.erase (m_amRxNextSn);
      m_amRxNextSn++;
      DeliverToUpper (outPkt);
    }
  // 如果还有 gap, 标记缺失 SN 并安排发 Status
  if (!m_amReorderBuffer.empty ())
    {
      for (uint16_t s = m_amRxNextSn; s < m_amReorderBuffer.rbegin ()->first; ++s)
        {
          if (!m_amReorderBuffer.count (s))
            {
              m_amMissingSn.insert (s);
            }
        }
      if (!m_amMissingSn.empty ()) AmScheduleStatusReport ();
    }
}

void
NtnRlc::AmScheduleStatusReport ()
{
  if (m_amStatusPending) return;
  m_amStatusPending = true;
  if (m_amStatusEvent.IsRunning ()) Simulator::Cancel (m_amStatusEvent);
  m_amStatusEvent = Simulator::Schedule (m_amStatusProhibit, &NtnRlc::AmSendStatusReport, this);
}

void
NtnRlc::AmSendStatusReport ()
{
  m_amStatusPending = false;
  if (m_amMissingSn.empty ()) return;

  // 把 NACK SN 列表写到 payload (4 字节 count + 2 字节/SN)
  std::vector<uint16_t> nacks (m_amMissingSn.begin (), m_amMissingSn.end ());
  uint32_t bytes = 4 + 2 * nacks.size ();
  std::vector<uint8_t> buf (bytes, 0);
  uint32_t n = static_cast<uint32_t> (nacks.size ());
  buf[0] = (n >> 24) & 0xff;
  buf[1] = (n >> 16) & 0xff;
  buf[2] = (n >>  8) & 0xff;
  buf[3] =  n        & 0xff;
  for (uint32_t i = 0; i < nacks.size (); ++i)
    {
      buf[4 + 2 * i + 0] = (nacks[i] >> 8) & 0xff;
      buf[4 + 2 * i + 1] =  nacks[i]       & 0xff;
    }
  Ptr<Packet> statusPdu = Create<Packet> (buf.data (), bytes);
  NtnRlcSnTag tag (0xFFFF, true);
  statusPdu->AddPacketTag (tag);

  // Status PDU 直接经下行回调送出 (Tx 计数累加, 重传到端等价反向 receiveCallback)
  m_txBytes += bytes;
  m_txPduCount++;
  m_txTrace (m_rnti, m_lcid, bytes);
  if (!m_txCb.IsNull ()) m_txCb (statusPdu);

  NS_LOG_INFO ("AM tx Status: " << nacks.size () << " NACK SN(s)");
  // 简化: 假设 Status PDU 100% 送达, 不再保留 m_amMissingSn (等下次再缺再补)
  m_amMissingSn.clear ();
}

void
NtnRlc::AmReceiveStatusPdu (Ptr<Packet> p)
{
  // 解析 NACK 列表
  uint32_t size = p->GetSize ();
  if (size < 4) return;
  std::vector<uint8_t> buf (size, 0);
  p->CopyData (buf.data (), size);
  uint32_t n = (static_cast<uint32_t> (buf[0]) << 24)
             | (static_cast<uint32_t> (buf[1]) << 16)
             | (static_cast<uint32_t> (buf[2]) <<  8)
             |  static_cast<uint32_t> (buf[3]);
  if (size < 4 + 2 * n) return;

  for (uint32_t i = 0; i < n; ++i)
    {
      uint16_t sn = (static_cast<uint16_t> (buf[4 + 2 * i + 0]) << 8)
                  |  static_cast<uint16_t> (buf[4 + 2 * i + 1]);
      auto it = m_amTxBuffer.find (sn);
      if (it == m_amTxBuffer.end ())
        {
          NS_LOG_DEBUG ("AM Rx Status: NACK sn=" << sn << " not in tx buffer (already acked?)");
          continue;
        }
      // 重传 (取出 Copy, buffer 还留着, 万一再次丢包还要再重发)
      Ptr<Packet> reTx = it->second->Copy ();
      AmDoTransmit (reTx, sn, /*isRetx=*/true);
      NS_LOG_INFO ("AM retx sn=" << sn);
    }
}

void
NtnRlc::AmPollRetransmitTimeout ()
{
  // 兜底: tx buffer 里 sn < m_amRxNextSn (理论已接收侧 ACK 的也无)
  // 简化处理: 对 tx buffer 里所有 sn 都视作未确认, 但只在缺失明显时发起 (sn 距 nextSn >= 1)。
  // 实际不要全部重发, 只重发"年龄较老"的, 避免无谓 retx 暴涨。
  // 这里偷一刀: 只看 tx buffer 头部第一个, 重发它一次。
  if (m_amTxBuffer.empty ()) return;
  uint16_t sn = m_amTxBuffer.begin ()->first;
  Ptr<Packet> reTx = m_amTxBuffer.begin ()->second->Copy ();
  AmDoTransmit (reTx, sn, /*isRetx=*/true);
  NS_LOG_INFO ("AM poll-retx sn=" << sn);
  // 续 timer
  m_amPollEvent = Simulator::Schedule (m_amPollRetransmit, &NtnRlc::AmPollRetransmitTimeout, this);
}

// =========================================================================
// 共用上递: 计 rxTrace, 调 receiveCallback
// =========================================================================
void
NtnRlc::DeliverToUpper (Ptr<Packet> p)
{
  uint32_t sz = p->GetSize ();
  m_rxBytes += sz;
  m_rxPduCount++;

  uint64_t delayNs = 0;
  NtnRlcTimestampTag tsTag;
  if (p->PeekPacketTag (tsTag))
    {
      Time d = Simulator::Now () - tsTag.GetTxTime ();
      delayNs = static_cast<uint64_t> (d.GetNanoSeconds ());
    }
  m_rxTrace (m_rnti, m_lcid, sz, delayNs);
  if (!m_recvCb.IsNull ()) m_recvCb (p);
}

} // namespace ns3
