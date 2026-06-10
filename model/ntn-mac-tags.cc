/*
 * 文件路径: contrib/geo-sat/model/ntn-mac-tags.cc
 */
#include "ntn-mac-tags.h"
#include "ns3/log.h"
#include "ns3/nstime.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (NtnDciTag);

TypeId
NtnDciTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnDciTag")
    .SetParent<Tag> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnDciTag> ();
  return tid;
}

TypeId
NtnDciTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

NtnDciTag::NtnDciTag () = default;

NtnDciTag::NtnDciTag (uint16_t rnti, bool isAck)
  : m_rnti (rnti),
    m_isAck (isAck)
{}

uint32_t
NtnDciTag::GetSerializedSize (void) const
{
  return sizeof (uint16_t) + sizeof (uint8_t) + sizeof (double);
}

void
NtnDciTag::Serialize (TagBuffer i) const
{
  i.WriteU16 (m_rnti);
  i.WriteU8 (m_isAck ? 1 : 0);
  i.WriteDouble (m_effectiveSinrDb);
}

void
NtnDciTag::Deserialize (TagBuffer i)
{
  m_rnti = i.ReadU16 ();
  m_isAck = (i.ReadU8 () != 0);
  m_effectiveSinrDb = i.ReadDouble ();
}

void
NtnDciTag::Print (std::ostream& os) const
{
  os << "NtnDciTag(rnti=" << m_rnti << ", isAck=" << m_isAck
     << ", effSinrDb=" << m_effectiveSinrDb << ")";
}

// ---------------------------------------------------------------------------
// NtnPhyDeliveryTag
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (NtnPhyDeliveryTag);

TypeId
NtnPhyDeliveryTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnPhyDeliveryTag")
    .SetParent<Tag> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnPhyDeliveryTag> ();
  return tid;
}
TypeId NtnPhyDeliveryTag::GetInstanceTypeId (void) const { return GetTypeId (); }
NtnPhyDeliveryTag::NtnPhyDeliveryTag () : m_t (Seconds (0)) {}
NtnPhyDeliveryTag::NtnPhyDeliveryTag (Time t) : m_t (t) {}
uint32_t NtnPhyDeliveryTag::GetSerializedSize () const { return sizeof (int64_t); }
void NtnPhyDeliveryTag::Serialize   (TagBuffer i) const { i.WriteU64 (static_cast<uint64_t> (m_t.GetNanoSeconds ())); }
void NtnPhyDeliveryTag::Deserialize (TagBuffer i)       { m_t = NanoSeconds (static_cast<int64_t> (i.ReadU64 ())); }
void NtnPhyDeliveryTag::Print (std::ostream& os) const  { os << "NtnPhyDeliveryTag(t=" << m_t.GetSeconds () << "s)"; }

// ---------------------------------------------------------------------------
// NtnRxSinrTag
// ---------------------------------------------------------------------------
NS_OBJECT_ENSURE_REGISTERED (NtnRxSinrTag);

TypeId
NtnRxSinrTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnRxSinrTag")
    .SetParent<Tag> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnRxSinrTag> ();
  return tid;
}
TypeId NtnRxSinrTag::GetInstanceTypeId (void) const { return GetTypeId (); }
NtnRxSinrTag::NtnRxSinrTag () = default;
NtnRxSinrTag::NtnRxSinrTag (double rxSinrDb) : m_rxSinrDb (rxSinrDb) {}
uint32_t NtnRxSinrTag::GetSerializedSize () const { return sizeof (double); }
void NtnRxSinrTag::Serialize   (TagBuffer i) const { i.WriteDouble (m_rxSinrDb); }
void NtnRxSinrTag::Deserialize (TagBuffer i)       { m_rxSinrDb = i.ReadDouble (); }
void NtnRxSinrTag::Print (std::ostream& os) const  { os << "NtnRxSinrTag(rxSinrDb=" << m_rxSinrDb << ")"; }

// ---------------------------------------------------------------------------
// NtnBeamIdTag
// ---------------------------------------------------------------------------

NS_OBJECT_ENSURE_REGISTERED (NtnBeamIdTag);

TypeId
NtnBeamIdTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnBeamIdTag")
    .SetParent<Tag> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnBeamIdTag> ();
  return tid;
}

TypeId
NtnBeamIdTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

NtnBeamIdTag::NtnBeamIdTag () = default;

NtnBeamIdTag::NtnBeamIdTag (uint16_t beamId)
  : m_beamId (beamId)
{}

uint32_t
NtnBeamIdTag::GetSerializedSize (void) const
{
  return sizeof (uint16_t);
}

void
NtnBeamIdTag::Serialize (TagBuffer i) const
{
  i.WriteU16 (m_beamId);
}

void
NtnBeamIdTag::Deserialize (TagBuffer i)
{
  m_beamId = i.ReadU16 ();
}

void
NtnBeamIdTag::Print (std::ostream& os) const
{
  os << "NtnBeamIdTag(beamId=" << m_beamId << ")";
}

} // namespace ns3
