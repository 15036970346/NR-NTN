/*
 * rohc-compressor.h
 * ROHC (Robust Header Compression) compressor implementation
 * Simplified implementation for NTN satellite links
 * Reference: RFC 3095, RFC 6846
 */

#ifndef ROHC_COMPRESSOR_H
#define ROHC_COMPRESSOR_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/header.h"
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h"
#include <map>
#include <cstdint>

namespace ns3 {

enum class RohcMode : uint8_t
{
  OFF = 0,
  IR = 1,
  IR_DYN = 2,
  UO_0 = 3,
  UO_1 = 4,
  UO_1_ID = 5,
  UO_2 = 6
};

enum class RohcPacketType : uint8_t
{
  IR = 1,
  IR_DYN = 2,
  UO_0 = 3,
  UO_1 = 4,
  UO_1_ID = 5,
  UO_2 = 6,
  ESP = 7
};

struct RohcContext
{
  uint16_t cid;
  RohcMode mode;
  uint32_t sn;
  uint32_t ipId;
  uint32_t timestamp;
  Ipv4Address srcAddr;
  Ipv4Address dstAddr;
  uint16_t srcPort;
  uint16_t dstPort;
  bool isInitialized;
};

class RohcCompressor : public Object
{
public:
  RohcCompressor ();
  ~RohcCompressor () override;

  static TypeId GetTypeId ();

  void SetCid (uint16_t cid);
  uint16_t GetCid () const;

  void SetMode (RohcMode mode);
  RohcMode GetMode () const;

  Ptr<Packet> Compress (Ptr<Packet> packet);
  Ptr<Packet> Decompress (Ptr<Packet> packet);

  void ResetContext ();
  bool IsEnabled () const;

private:
  uint16_t m_cid;
  uint8_t m_mode;
  RohcContext m_context;
  bool m_enabled;

  uint32_t CalculateCompressedHeaderSize (RohcPacketType type);
  void UpdateContext (Ptr<Packet> packet);
};

class RohcHeader : public Header
{
public:
  RohcHeader ();
  ~RohcHeader () override;

  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const override;
  void Print (std::ostream& os) const override;
  uint32_t GetSerializedSize () const override;
  void Serialize (Buffer::Iterator start) const override;
  uint32_t Deserialize (Buffer::Iterator start) override;

  void SetPacketType (RohcPacketType type);
  RohcPacketType GetPacketType () const;

  void SetCid (uint16_t cid);
  uint16_t GetCid () const;

  void SetSn (uint32_t sn);
  uint32_t GetSn () const;

  void SetIpId (uint16_t ipId);
  uint16_t GetIpId () const;

private:
  RohcPacketType m_packetType;
  uint16_t m_cid;
  uint16_t m_sn;
  uint16_t m_ipId;
};

} // namespace ns3

#endif
