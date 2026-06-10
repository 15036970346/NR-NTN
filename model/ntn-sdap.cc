/*
 * contrib/geo-sat/model/ntn-sdap.cc
 * SDAP layer implementation: QoS flow <-> DRB mapping
 */
#include "ntn-sdap.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnSdap");

// ==================== NtnSdapTag ====================
NS_OBJECT_ENSURE_REGISTERED (NtnSdapTag);

TypeId
NtnSdapTag::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::NtnSdapTag")
        .SetParent<Tag> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<NtnSdapTag> ();
    return tid;
}

TypeId
NtnSdapTag::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

NtnSdapTag::NtnSdapTag ()
    : m_sendTime (Seconds (0)),
      m_qfi (0)
{
}

NtnSdapTag::NtnSdapTag (Time sendTime, uint8_t qfi)
    : m_sendTime (sendTime),
      m_qfi (qfi)
{
}

void
NtnSdapTag::Serialize (TagBuffer i) const
{
    int64_t ns = m_sendTime.GetNanoSeconds ();
    i.Write (reinterpret_cast<const uint8_t*> (&ns), 8);
    i.WriteU8 (m_qfi);
}

void
NtnSdapTag::Deserialize (TagBuffer i)
{
    int64_t ns;
    i.Read (reinterpret_cast<uint8_t*> (&ns), 8);
    m_sendTime = NanoSeconds (ns);
    m_qfi = i.ReadU8 ();
}

uint32_t
NtnSdapTag::GetSerializedSize (void) const
{
    return 9;  // 8 bytes time + 1 byte QFI
}

void
NtnSdapTag::Print (std::ostream& os) const
{
    os << "NtnSdapTag sendTime=" << m_sendTime.GetNanoSeconds () << "ns qfi=" << (uint32_t)m_qfi;
}

Time
NtnSdapTag::GetSendTime () const
{
    return m_sendTime;
}

uint8_t
NtnSdapTag::GetQfi () const
{
    return m_qfi;
}

// ==================== NtnSdap ====================
NS_OBJECT_ENSURE_REGISTERED (NtnSdap);

TypeId
NtnSdap::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::NtnSdap")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<NtnSdap> ()
        .AddTraceSource ("TxSDAP",
                         "SDAP TX trace (rnti, qfi, lcid, size)",
                         MakeTraceSourceAccessor (&NtnSdap::m_txSdapTrace),
                         "ns3::NtnSdap::TxCallback")
        .AddTraceSource ("RxSDAP",
                         "SDAP RX trace (rnti, qfi, lcid, size, delay_ns)",
                         MakeTraceSourceAccessor (&NtnSdap::m_rxSdapTrace),
                         "ns3::NtnSdap::RxCallback");
    return tid;
}

NtnSdap::NtnSdap ()
    : m_rnti (0),
      m_defaultLcid (3)  // default DRB LCID
{
    NS_LOG_FUNCTION (this);
}

NtnSdap::~NtnSdap ()
{
    NS_LOG_FUNCTION (this);
}

void
NtnSdap::SetRnti (uint16_t rnti)
{
    m_rnti = rnti;
}

void
NtnSdap::MapQfiToDrb (uint8_t qfi, uint8_t lcid)
{
    m_qfiToDrb[qfi] = lcid;
    NS_LOG_INFO ("SDAP: QFI " << (uint32_t)qfi << " -> DRB LCID " << (uint32_t)lcid);
}

uint8_t
NtnSdap::GetDrbForQfi (uint8_t qfi) const
{
    auto it = m_qfiToDrb.find (qfi);
    return (it != m_qfiToDrb.end ()) ? it->second : m_defaultLcid;
}

void
NtnSdap::DoSendData (uint8_t qfi, Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << (uint32_t)qfi << p->GetSize ());

    uint8_t lcid = GetDrbForQfi (qfi);

    // Add SDAP timestamp tag
    NtnSdapTag tag (Simulator::Now (), qfi);
    p->AddByteTag (tag);

    uint32_t size = p->GetSize ();
    m_txSdapTrace (m_rnti, qfi, lcid, size);

    NS_LOG_INFO ("SDAP TX: RNTI=" << m_rnti << " QFI=" << (uint32_t)qfi
                 << " -> LCID=" << (uint32_t)lcid << " size=" << size);
}

void
NtnSdap::DoReceiveData (uint8_t qfi, Ptr<Packet> p)
{
    NS_LOG_FUNCTION (this << (uint32_t)qfi << p->GetSize ());

    uint8_t lcid = GetDrbForQfi (qfi);
    uint32_t size = p->GetSize ();

    // Compute E2E delay from SDAP tag
    uint64_t delayNs = 0;
    NtnSdapTag tag;
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
