/*
 * rohc-compressor.cc
 * ROHC (Robust Header Compression) compressor implementation
 * Simplified implementation for NTN satellite links
 * Reference: RFC 3095, RFC 6846
 */

#include "rohc-compressor.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RohcCompressor");

NS_OBJECT_ENSURE_REGISTERED (RohcCompressor);

RohcCompressor::RohcCompressor ()
  : m_cid (0),
    m_mode (1),
    m_enabled (true)
{
  m_context.mode = RohcMode::IR;
  m_context.isInitialized = false;
}

RohcCompressor::~RohcCompressor ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
RohcCompressor::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RohcCompressor")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<RohcCompressor> ()
    .AddAttribute ("Enabled",
                   "Enable ROHC compression",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RohcCompressor::m_enabled),
                   MakeBooleanChecker ())
    .AddAttribute ("CompressionMode",
                   "ROHC compression mode (0=OFF, 1=IR, 2=IR_DYN, 3=UO_0, 4=UO_1, 5=UO_2)",
                   UintegerValue (1),
                   MakeUintegerAccessor (&RohcCompressor::m_mode),
                   MakeUintegerChecker<uint8_t> ());
  return tid;
}

void
RohcCompressor::SetCid (uint16_t cid)
{
  m_cid = cid;
  m_context.cid = cid;
}

uint16_t
RohcCompressor::GetCid () const
{
  return m_cid;
}

void
RohcCompressor::SetMode (RohcMode mode)
{
  m_mode = static_cast<uint8_t> (mode);
}

RohcMode
RohcCompressor::GetMode () const
{
  return static_cast<RohcMode> (m_mode);
}

bool
RohcCompressor::IsEnabled () const
{
  return m_enabled;
}

void
RohcCompressor::ResetContext ()
{
  NS_LOG_FUNCTION (this);
  m_context.isInitialized = false;
  m_context.sn = 0;
  m_context.ipId = 0;
}

uint32_t
RohcCompressor::CalculateCompressedHeaderSize (RohcPacketType type)
{
  switch (type)
    {
    case RohcPacketType::IR:
      return 10;
    case RohcPacketType::IR_DYN:
      return 6;
    case RohcPacketType::UO_0:
      return 2;
    case RohcPacketType::UO_1:
      return 3;
    case RohcPacketType::UO_1_ID:
      return 4;
    case RohcPacketType::UO_2:
      return 4;
    default:
      return 2;
    }
}

void
RohcCompressor::UpdateContext (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this);
  
  m_context.sn++;
  m_context.ipId = (m_context.ipId + 1) % 65536;
  m_context.isInitialized = true;
}

Ptr<Packet>
RohcCompressor::Compress (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet->GetSize ());

  if (!m_enabled || m_mode == 0)
    {
      return packet;
    }

  uint32_t originalSize = packet->GetSize ();

  Ipv4Header ipv4Header;
  UdpHeader udpHeader;
  bool hasIpv4 = false;
  bool hasUdp = false;

  if (packet->PeekHeader (ipv4Header) == 0)
    {
      hasIpv4 = true;
    }
  if (packet->PeekHeader (udpHeader) == 0)
    {
      hasUdp = true;
    }

  if (!hasIpv4)
    {
      NS_LOG_DEBUG ("No IPv4 header to compress");
      return packet;
    }

  RohcPacketType packetType;
  if (!m_context.isInitialized)
    {
      packetType = RohcPacketType::IR;
      m_context.srcAddr = ipv4Header.GetSource ();
      m_context.dstAddr = ipv4Header.GetDestination ();
      if (hasUdp)
        {
          m_context.srcPort = udpHeader.GetSourcePort ();
          m_context.dstPort = udpHeader.GetDestinationPort ();
        }
    }
  else
    {
      if (hasUdp)
        {
          packetType = RohcPacketType::UO_1;
        }
      else
        {
          packetType = RohcPacketType::UO_0;
        }
    }

  uint32_t payloadSize = originalSize - (hasIpv4 ? 20 : 0) - (hasUdp ? 8 : 0);
  
  Ptr<Packet> compressedPacket;
  
  if (payloadSize > 0)
    {
      uint8_t* payloadBuffer = new uint8_t[payloadSize];
      packet->CopyData (payloadBuffer, payloadSize);
      compressedPacket = Create<Packet> (payloadBuffer, payloadSize);
      delete[] payloadBuffer;
    }
  else
    {
      compressedPacket = Create<Packet> (0);
    }

  RohcHeader rohcHeader;
  rohcHeader.SetPacketType (packetType);
  rohcHeader.SetCid (m_cid);
  rohcHeader.SetSn (m_context.sn);
  rohcHeader.SetIpId (m_context.ipId);
  
  compressedPacket->AddHeader (rohcHeader);

  uint32_t compressedSize = compressedPacket->GetSize ();
  int32_t savings = static_cast<int32_t>(originalSize) - static_cast<int32_t>(compressedSize);

  double savingsPercent = (savings > 0) ? (savings * 100.0 / originalSize) : 0;
  NS_LOG_INFO ("ROHC Compress: Original=" << originalSize 
               << " bytes, Compressed=" << compressedSize 
               << " bytes, Savings=" << savings 
               << " bytes (" << savingsPercent << "%)");

  UpdateContext (packet);

  return compressedPacket;
}

Ptr<Packet>
RohcCompressor::Decompress (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet->GetSize ());

  if (!m_enabled || m_mode == 0)
    {
      return packet;
    }

  RohcHeader rohcHeader;
  packet->RemoveHeader (rohcHeader);

  RohcPacketType packetType = rohcHeader.GetPacketType ();
  uint32_t sn = rohcHeader.GetSn ();

  NS_LOG_DEBUG ("Decompress: CID=" << rohcHeader.GetCid () 
                << ", SN=" << sn 
                << ", Type=" << (uint32_t)packetType);

  Ipv4Header ipv4Header;
  UdpHeader udpHeader;

  ipv4Header.SetSource (m_context.srcAddr);
  ipv4Header.SetDestination (m_context.dstAddr);
  ipv4Header.SetProtocol (17);
  ipv4Header.SetTtl (64);
  ipv4Header.SetIdentification (rohcHeader.GetIpId ());

  if (packetType != RohcPacketType::UO_0)
    {
      udpHeader.SetSourcePort (m_context.srcPort);
      udpHeader.SetDestinationPort (m_context.dstPort);
    }

  Ptr<Packet> decompressedPacket = Create<Packet> ();
  decompressedPacket->AddHeader (ipv4Header);
  if (packetType != RohcPacketType::UO_0)
    {
      decompressedPacket->AddHeader (udpHeader);
    }
  decompressedPacket->AddAtEnd (packet);

  m_context.sn = sn;
  m_context.ipId = rohcHeader.GetIpId ();

  NS_LOG_INFO ("ROHC Decompress: Decompressed=" << decompressedPacket->GetSize () << " bytes");

  return decompressedPacket;
}

NS_OBJECT_ENSURE_REGISTERED (RohcHeader);

RohcHeader::RohcHeader ()
  : m_packetType (RohcPacketType::IR),
    m_cid (0),
    m_sn (0),
    m_ipId (0)
{
}

RohcHeader::~RohcHeader ()
{
}

TypeId
RohcHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

TypeId
RohcHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RohcHeader")
    .SetParent<Header> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<RohcHeader> ();
  return tid;
}

void
RohcHeader::SetPacketType (RohcPacketType type)
{
  m_packetType = type;
}

RohcPacketType
RohcHeader::GetPacketType () const
{
  return m_packetType;
}

void
RohcHeader::SetCid (uint16_t cid)
{
  m_cid = cid;
}

uint16_t
RohcHeader::GetCid () const
{
  return m_cid;
}

void
RohcHeader::SetSn (uint32_t sn)
{
  m_sn = sn & 0xFFFF;
}

uint32_t
RohcHeader::GetSn () const
{
  return m_sn;
}

void
RohcHeader::SetIpId (uint16_t ipId)
{
  m_ipId = ipId;
}

uint16_t
RohcHeader::GetIpId () const
{
  return m_ipId;
}

uint32_t
RohcHeader::GetSerializedSize () const
{
  return 6;
}

void
RohcHeader::Serialize (Buffer::Iterator start) const
{
  start.WriteU8 (static_cast<uint8_t> (m_packetType));
  start.WriteU8 (0);
  start.WriteU16 (m_cid);
  start.WriteU16 (m_sn);
  start.WriteU16 (m_ipId);
}

uint32_t
RohcHeader::Deserialize (Buffer::Iterator start)
{
  m_packetType = static_cast<RohcPacketType> (start.ReadU8 ());
  start.ReadU8 ();
  m_cid = start.ReadU16 ();
  m_sn = start.ReadU16 ();
  m_ipId = start.ReadU16 ();
  return GetSerializedSize ();
}

void
RohcHeader::Print (std::ostream& os) const
{
  os << "ROHC[Type=" << (uint32_t)m_packetType 
     << ", CID=" << m_cid 
     << ", SN=" << m_sn 
     << ", IP_ID=" << m_ipId << "]";
}

} // namespace ns3
