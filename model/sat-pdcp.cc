/*
 * contrib/geo-sat/model/sat-pdcp.cc
 * GEO satellite PDCP layer with ROHC and NR-compatible TracedCallbacks
 */
#include "sat-pdcp.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatPdcp");
NS_OBJECT_ENSURE_REGISTERED (SatPdcp);

// ==================== SatPdcpTag ====================

NS_OBJECT_ENSURE_REGISTERED (SatPdcpTag);

TypeId
SatPdcpTag::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatPdcpTag")
        .SetParent<Tag> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatPdcpTag> ();
    return tid;
}

TypeId
SatPdcpTag::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

SatPdcpTag::SatPdcpTag ()
    : m_sendTime (Seconds (0))
{
}

SatPdcpTag::SatPdcpTag (Time sendTime)
    : m_sendTime (sendTime)
{
}

void
SatPdcpTag::Serialize (TagBuffer i) const
{
    int64_t sendTimeNs = m_sendTime.GetNanoSeconds ();
    i.Write (reinterpret_cast<const uint8_t*> (&sendTimeNs), 8);
}

void
SatPdcpTag::Deserialize (TagBuffer i)
{
    int64_t sendTimeNs;
    i.Read (reinterpret_cast<uint8_t*> (&sendTimeNs), 8);
    m_sendTime = NanoSeconds (sendTimeNs);
}

uint32_t
SatPdcpTag::GetSerializedSize (void) const
{
    return 8;
}

void
SatPdcpTag::Print (std::ostream& os) const
{
    os << "SatPdcpTag sendTime=" << m_sendTime.GetNanoSeconds () << "ns";
}

Time
SatPdcpTag::GetSendTime () const
{
    return m_sendTime;
}

// ==================== SatPdcp ====================

TypeId
SatPdcp::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatPdcp")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatPdcp> ()
        .AddAttribute ("RohcEnabled",
                       "Enable ROHC header compression for NTN",
                       BooleanValue (true),
                       MakeBooleanAccessor (&SatPdcp::m_rohcEnabled),
                       MakeBooleanChecker ())
        .AddTraceSource ("TxPDCP",
                         "PDCP PDU transmitted (rnti, lcid, size)",
                         MakeTraceSourceAccessor (&SatPdcp::m_txPdu),
                         "ns3::SatPdcp::TxPduCallback")
        .AddTraceSource ("RxPDCP",
                         "PDCP PDU received (rnti, lcid, size, delay_ns)",
                         MakeTraceSourceAccessor (&SatPdcp::m_rxPdu),
                         "ns3::SatPdcp::RxPduCallback")
        .AddTraceSource ("RohcCompression",
                         "ROHC compression event (rnti, lcid, origSize, compressedSize)",
                         MakeTraceSourceAccessor (&SatPdcp::m_rohcTrace),
                         "ns3::SatPdcp::RohcCallback");
    return tid;
}

SatPdcp::SatPdcp ()
    : m_rnti (0),
      m_lcId (0),
      m_rohcEnabled (true),
      m_incompletePackets (0)
{
    NS_LOG_FUNCTION (this);
}

SatPdcp::~SatPdcp ()
{
    NS_LOG_FUNCTION (this);
}

void
SatPdcp::SetRohcCompressor (Ptr<RohcCompressor> rohc)
{
    m_rohc = rohc;
    if (rohc)
    {
        m_rohcEnabled = rohc->IsEnabled ();
    }
}

Ptr<RohcCompressor>
SatPdcp::GetRohcCompressor () const
{
    return m_rohc;
}

void
SatPdcp::SetPdcpSapUser (Ptr<Object> sapUser)
{
    m_pdcpSapUser = sapUser;
}

void
SatPdcp::SetRnti (uint16_t rnti)
{
    m_rnti = rnti;
}

void
SatPdcp::SetLcId (uint8_t lcId)
{
    m_lcId = lcId;
}

void
SatPdcp::DoSendData (Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << p->GetSize ());

    uint32_t originalSize = p->GetSize ();

    // Add timestamp tag for delay measurement (before compression)
    SatPdcpTag tag (Simulator::Now ());
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
    NS_LOG_INFO ("PDCP send complete, packet passed to lower layer (RLC)");
}

void
SatPdcp::DoReceiveData (Ptr<Packet> p)
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
    SatPdcpTag tag;
    if (p->FindFirstMatchingByteTag (tag))
    {
        delayNs = (Simulator::Now () - tag.GetSendTime ()).GetNanoSeconds ();
    }

    // Fire NR-compatible RX trace
    m_rxPdu (m_rnti, m_lcId, p->GetSize (), delayNs);
    NS_LOG_INFO ("PDCP receive complete, packet passed to upper layer (IP)");
}

uint32_t
SatPdcp::GetIncompletePacketCount () const
{
    return m_incompletePackets;
}

} // namespace ns3
