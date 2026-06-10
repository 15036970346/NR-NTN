/*
 * contrib/geo-sat/model/ntn-pdcp.cc
 * GEO satellite PDCP layer with ROHC and NR-compatible TracedCallbacks
 */
#include "ntn-pdcp.h"
#include "ntn-rlc.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnPdcp");
NS_OBJECT_ENSURE_REGISTERED (NtnPdcp);

// ==================== NtnPdcpTag ====================

NS_OBJECT_ENSURE_REGISTERED (NtnPdcpTag);

TypeId
NtnPdcpTag::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::NtnPdcpTag")
        .SetParent<Tag> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<NtnPdcpTag> ();
    return tid;
}

TypeId
NtnPdcpTag::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

NtnPdcpTag::NtnPdcpTag ()
    : m_sendTime (Seconds (0))
{
}

NtnPdcpTag::NtnPdcpTag (Time sendTime)
    : m_sendTime (sendTime)
{
}

void
NtnPdcpTag::Serialize (TagBuffer i) const
{
    int64_t sendTimeNs = m_sendTime.GetNanoSeconds ();
    i.Write (reinterpret_cast<const uint8_t*> (&sendTimeNs), 8);
}

void
NtnPdcpTag::Deserialize (TagBuffer i)
{
    int64_t sendTimeNs;
    i.Read (reinterpret_cast<uint8_t*> (&sendTimeNs), 8);
    m_sendTime = NanoSeconds (sendTimeNs);
}

uint32_t
NtnPdcpTag::GetSerializedSize (void) const
{
    return 8;
}

void
NtnPdcpTag::Print (std::ostream& os) const
{
    os << "NtnPdcpTag sendTime=" << m_sendTime.GetNanoSeconds () << "ns";
}

Time
NtnPdcpTag::GetSendTime () const
{
    return m_sendTime;
}

// ==================== NtnPdcp ====================

TypeId
NtnPdcp::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::NtnPdcp")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<NtnPdcp> ()
        .AddAttribute ("RohcEnabled",
                       "Enable ROHC header compression for NTN",
                       BooleanValue (true),
                       MakeBooleanAccessor (&NtnPdcp::m_rohcEnabled),
                       MakeBooleanChecker ())
        .AddTraceSource ("TxPDCP",
                         "PDCP PDU transmitted (rnti, lcid, size)",
                         MakeTraceSourceAccessor (&NtnPdcp::m_txPdu),
                         "ns3::NtnPdcp::TxPduCallback")
        .AddTraceSource ("RxPDCP",
                         "PDCP PDU received (rnti, lcid, size, delay_ns)",
                         MakeTraceSourceAccessor (&NtnPdcp::m_rxPdu),
                         "ns3::NtnPdcp::RxPduCallback")
        .AddTraceSource ("RohcCompression",
                         "ROHC compression event (rnti, lcid, origSize, compressedSize)",
                         MakeTraceSourceAccessor (&NtnPdcp::m_rohcTrace),
                         "ns3::NtnPdcp::RohcCallback");
    return tid;
}

NtnPdcp::NtnPdcp ()
    : m_rnti (0),
      m_lcId (0),
      m_rohcEnabled (true),
      m_incompletePackets (0)
{
    NS_LOG_FUNCTION (this);
}

NtnPdcp::~NtnPdcp ()
{
    NS_LOG_FUNCTION (this);
}

void
NtnPdcp::SetRohcCompressor (Ptr<RohcCompressor> rohc)
{
    m_rohc = rohc;
    if (rohc)
    {
        m_rohcEnabled = rohc->IsEnabled ();
    }
}

Ptr<RohcCompressor>
NtnPdcp::GetRohcCompressor () const
{
    return m_rohc;
}

void
NtnPdcp::SetPdcpSapUser (Ptr<Object> sapUser)
{
    m_pdcpSapUser = sapUser;
}

void
NtnPdcp::SetRnti (uint16_t rnti)
{
    m_rnti = rnti;
}

void
NtnPdcp::SetLcId (uint8_t lcId)
{
    m_lcId = lcId;
}

void
NtnPdcp::SetRlc (Ptr<NtnRlc> rlc)
{
    m_rlc = rlc;
    if (m_rlc)
    {
        // RLC 上送回 PDCP DoReceiveData
        m_rlc->SetReceiveCallback (MakeCallback (&NtnPdcp::DoReceiveData, this));
    }
}

void
NtnPdcp::SetIpRxCallback (Callback<void, Ptr<Packet>> cb)
{
    m_ipRxCb = cb;
}

void
NtnPdcp::DoSendData (Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << p->GetSize ());

    uint32_t originalSize = p->GetSize ();

    // Add timestamp tag for delay measurement (before compression)
    NtnPdcpTag tag (Simulator::Now ());
    p->AddByteTag (tag);

    if (m_rohcEnabled && m_rohc)
    {
        NS_LOG_INFO ("Applying ROHC compression at PDCP layer");
        p = m_rohc->Compress (p);

        uint32_t compressedSize = p->GetSize ();
        NS_LOG_INFO ("PDCP TX: Original=" << originalSize
                     << " bytes, After ROHC=" << compressedSize
                     << " bytes, Savings=" << (originalSize - compressedSize) << " bytes");

        // Fire ROHC compression ratio trace
        m_rohcTrace (m_rnti, m_lcId, originalSize, compressedSize);
    }
    else
    {
        NS_LOG_DEBUG ("ROHC disabled, sending raw packet");
    }

    // Fire NR-compatible TX trace
    m_txPdu (m_rnti, m_lcId, p->GetSize ());

    // 下推到 RLC (若已注入); 否则只触发 trace, 老 demo 兼容
    if (m_rlc)
    {
        m_rlc->TransmitPdcpPdu (p);
    }
    else
    {
        NS_LOG_INFO ("PDCP send complete (no RLC attached, trace only)");
    }
}

void
NtnPdcp::DoReceiveData (Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << p->GetSize ());

    if (m_rohcEnabled && m_rohc)
    {
        NS_LOG_INFO ("Applying ROHC decompression at PDCP layer");
        uint32_t receivedSize = p->GetSize ();

        Ptr<Packet> decompressed = m_rohc->Decompress (p);

        if (!decompressed || decompressed->GetSize () == 0)
        {
            m_incompletePackets++;
            NS_LOG_WARN ("ROHC decompression failed, incomplete packet count="
                         << m_incompletePackets);
            return;
        }
        p = decompressed;

        uint32_t decompressedSize = p->GetSize ();
        NS_LOG_INFO ("PDCP RX: Received=" << receivedSize
                     << " bytes, After ROHC=" << decompressedSize
                     << " bytes");
    }
    else
    {
        NS_LOG_DEBUG ("ROHC disabled, receiving raw packet");
    }

    // Compute delay from timestamp tag
    uint64_t delayNs = 0;
    NtnPdcpTag tag;
    if (p->FindFirstMatchingByteTag (tag))
    {
        delayNs = (Simulator::Now () - tag.GetSendTime ()).GetNanoSeconds ();
    }

    // Fire NR-compatible RX trace
    m_rxPdu (m_rnti, m_lcId, p->GetSize (), delayNs);

    // 上送到 IP/App (若 callback 已设置)
    if (!m_ipRxCb.IsNull ())
    {
        m_ipRxCb (p);
    }
}

uint32_t
NtnPdcp::GetIncompletePacketCount () const
{
    return m_incompletePackets;
}

} // namespace ns3
