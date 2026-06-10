/*
 * contrib/geo-sat/model/ntn-pdcp.h
 * GEO satellite PDCP layer with ROHC header compression and NR-compatible TracedCallbacks
 * TracedCallback signatures aligned with src/lte/model/lte-rlc.h (rnti, lcid, size, delay_ns)
 */
#ifndef NTN_PDCP_H
#define NTN_PDCP_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/tag.h"
#include "ns3/nstime.h"
#include "rohc-compressor.h"
#include "ntn-rlc.h"
#include "ns3/traced-callback.h"
#include "ns3/callback.h"

namespace ns3 {

/**
 * Timestamp tag applied at PDCP TX, read at PDCP RX to compute per-PDU delay.
 * Modeled on LteRlcTag (src/lte/model/lte-rlc-tag.h).
 */
class NtnPdcpTag : public Tag
{
public:
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const override;

    NtnPdcpTag ();
    NtnPdcpTag (Time sendTime);

    virtual void Serialize (TagBuffer i) const override;
    virtual void Deserialize (TagBuffer i) override;
    virtual uint32_t GetSerializedSize (void) const override;
    virtual void Print (std::ostream& os) const override;

    Time GetSendTime () const;

private:
    Time m_sendTime;
};

class NtnPdcp : public Object
{
public:
    static TypeId GetTypeId (void);
    NtnPdcp ();
    virtual ~NtnPdcp ();

    void SetRohcCompressor (Ptr<RohcCompressor> rohc);
    Ptr<RohcCompressor> GetRohcCompressor () const;

    void SetPdcpSapUser (Ptr<Object> sapUser);
    void SetRnti (uint16_t rnti);
    void SetLcId (uint8_t lcId);

    /**
     * 接入下层 RLC 实体: SetRlc 后 DoSendData 会把 PDCP PDU 投递到 m_rlc->TransmitPdcpPdu,
     * 同时把 m_rlc 的 receiveCallback 自动接回 DoReceiveData (Rlc -> Pdcp 上送链路)。
     */
    void SetRlc (Ptr<NtnRlc> rlc);
    Ptr<NtnRlc> GetRlc () const { return m_rlc; }

    /// PDCP 解 ROHC 后递交给上层 (IP/App) 的回调; 未设置则只触发 trace 不上送
    void SetIpRxCallback (Callback<void, Ptr<Packet>> cb);

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

    Ptr<NtnRlc> m_rlc;                                  // 下层 RLC 实体 (可选)
    Callback<void, Ptr<Packet>> m_ipRxCb;               // 上送 IP/App
};

} // namespace ns3

#endif /* NTN_PDCP_H */
