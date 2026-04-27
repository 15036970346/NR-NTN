/*
 * contrib/geo-sat/model/sat-pdcp.h
 * GEO satellite PDCP layer with ROHC header compression and NR-compatible TracedCallbacks
 * TracedCallback signatures aligned with src/lte/model/lte-rlc.h (rnti, lcid, size, delay_ns)
 */
#ifndef SAT_PDCP_H
#define SAT_PDCP_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/tag.h"
#include "ns3/nstime.h"
#include "rohc-compressor.h"
#include "ns3/traced-callback.h"

namespace ns3 {

/**
 * Timestamp tag applied at PDCP TX, read at PDCP RX to compute per-PDU delay.
 * Modeled on LteRlcTag (src/lte/model/lte-rlc-tag.h).
 */
class SatPdcpTag : public Tag
{
public:
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const override;

    SatPdcpTag ();
    SatPdcpTag (Time sendTime);

    virtual void Serialize (TagBuffer i) const override;
    virtual void Deserialize (TagBuffer i) override;
    virtual uint32_t GetSerializedSize (void) const override;
    virtual void Print (std::ostream& os) const override;

    Time GetSendTime () const;

private:
    Time m_sendTime;
};

class SatPdcp : public Object
{
public:
    static TypeId GetTypeId (void);
    SatPdcp ();
    virtual ~SatPdcp ();

    void SetRohcCompressor (Ptr<RohcCompressor> rohc);
    Ptr<RohcCompressor> GetRohcCompressor () const;

    void SetPdcpSapUser (Ptr<Object> sapUser);
    void SetRnti (uint16_t rnti);
    void SetLcId (uint8_t lcId);

    void DoSendData (Ptr<Packet> p);
    void DoReceiveData (Ptr<Packet> p);

    // NR-compatible TracedCallbacks (matching lte-rlc.h signatures)
    TracedCallback<uint16_t, uint8_t, uint32_t> m_txPdu;            // (rnti, lcid, size)
    TracedCallback<uint16_t, uint8_t, uint32_t, uint64_t> m_rxPdu;  // (rnti, lcid, size, delay_ns)

    // ROHC compression ratio trace
    TracedCallback<uint16_t, uint8_t, uint32_t, uint32_t> m_rohcTrace; // (rnti, lcid, origSize, compressedSize)

    // ROHC decompression failure count
    uint32_t GetIncompletePacketCount () const;

private:
    Ptr<RohcCompressor> m_rohc;
    Ptr<Object> m_pdcpSapUser;
    uint16_t m_rnti;
    uint8_t m_lcId;
    bool m_rohcEnabled;
    uint32_t m_incompletePackets;   // ROHC decompression failure count
};

} // namespace ns3

#endif /* SAT_PDCP_H */
