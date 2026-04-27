/*
 * contrib/geo-sat/model/sat-sdap.cc
 * SDAP layer implementation: QoS flow <-> DRB mapping
 */
#include "sat-sdap.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatSdap");

// ==================== SatSdapTag ====================
NS_OBJECT_ENSURE_REGISTERED (SatSdapTag);

TypeId
SatSdapTag::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatSdapTag")
        .SetParent<Tag> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatSdapTag> ();
    return tid;
}

TypeId
SatSdapTag::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

SatSdapTag::SatSdapTag ()
    : m_sendTime (Seconds (0)),
      m_qfi (0)
{
}

SatSdapTag::SatSdapTag (Time sendTime, uint8_t qfi)
    : m_sendTime (sendTime),
      m_qfi (qfi)
{
}

void
SatSdapTag::Serialize (TagBuffer i) const
{
    int64_t ns = m_sendTime.GetNanoSeconds ();
    i.Write (reinterpret_cast<const uint8_t*> (&ns), 8);
    i.WriteU8 (m_qfi);
}

void
SatSdapTag::Deserialize (TagBuffer i)
{
    int64_t ns;
    i.Read (reinterpret_cast<uint8_t*> (&ns), 8);
    m_sendTime = NanoSeconds (ns);
    m_qfi = i.ReadU8 ();
}

uint32_t
SatSdapTag::GetSerializedSize (void) const
{
    return 9;  // 8 bytes time + 1 byte QFI
}

void
SatSdapTag::Print (std::ostream& os) const
{
    os << "SatSdapTag sendTime=" << m_sendTime.GetNanoSeconds () << "ns qfi=" << (uint32_t)m_qfi;
}

Time
SatSdapTag::GetSendTime () const
{
    return m_sendTime;
}

uint8_t
SatSdapTag::GetQfi () const
{
    return m_qfi;
}

// ==================== SatSdap ====================
NS_OBJECT_ENSURE_REGISTERED (SatSdap);

TypeId
SatSdap::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatSdap")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatSdap> ()
        .AddTraceSource ("TxSDAP",
                         "SDAP TX trace (rnti, qfi, lcid, size)",
                         MakeTraceSourceAccessor (&SatSdap::m_txSdapTrace),
                         "ns3::SatSdap::TxCallback")
        .AddTraceSource ("RxSDAP",
                         "SDAP RX trace (rnti, qfi, lcid, size, delay_ns)",
                         MakeTraceSourceAccessor (&SatSdap::m_rxSdapTrace),
                         "ns3::SatSdap::RxCallback");
    return tid;
}

SatSdap::SatSdap ()
    : m_rnti (0),
      m_defaultLcid (3)  // default DRB LCID
{
    NS_LOG_FUNCTION (this);
}

SatSdap::~SatSdap ()
{
    NS_LOG_FUNCTION (this);
}

void
SatSdap::SetRnti (uint16_t rnti)
{
    m_rnti = rnti;
}

void
SatSdap::MapQfiToDrb (uint8_t qfi, uint8_t lcid)
{
    m_qfiToDrb[qfi] = lcid;
    NS_LOG_INFO ("SDAP: QFI " << (uint32_t)qfi << " -> DRB LCID " << (uint32_t)lcid);
}

uint8_t
SatSdap::GetDrbForQfi (uint8_t qfi) const
{
    auto it = m_qfiToDrb.find (qfi);
    return (it != m_qfiToDrb.end ()) ? it->second : m_defaultLcid;
}

void
SatSdap::DoSendData (uint8_t qfi, Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << (uint32_t)qfi << p->GetSize ());

    uint8_t lcid = GetDrbForQfi (qfi);

    // Add SDAP timestamp tag
    SatSdapTag tag (Simulator::Now (), qfi);
    p->AddByteTag (tag);

    uint32_t size = p->GetSize ();
    m_txSdapTrace (m_rnti, qfi, lcid, size);

    NS_LOG_INFO ("SDAP TX: RNTI=" << m_rnti << " QFI=" << (uint32_t)qfi
                 << " -> LCID=" << (uint32_t)lcid << " size=" << size);
}

void
SatSdap::DoReceiveData (uint8_t qfi, Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << (uint32_t)qfi << p->GetSize ());

    uint8_t lcid = GetDrbForQfi (qfi);
    uint32_t size = p->GetSize ();

    // Compute E2E delay from SDAP tag
    uint64_t delayNs = 0;
    SatSdapTag tag;
    if (p->FindFirstMatchingByteTag (tag))
    {
        delayNs = (Simulator::Now () - tag.GetSendTime ()).GetNanoSeconds ();
    }

    m_rxSdapTrace (m_rnti, qfi, lcid, size, delayNs);

    NS_LOG_INFO ("SDAP RX: RNTI=" << m_rnti << " QFI=" << (uint32_t)qfi
                 << " LCID=" << (uint32_t)lcid << " size=" << size
                 << " delay=" << delayNs / 1e6 << "ms");
}

} // namespace ns3
