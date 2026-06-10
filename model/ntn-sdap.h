/*
 * contrib/geo-sat/model/ntn-sdap.h
 * SDAP (Service Data Adaptation Protocol) layer: QoS flow <-> DRB mapping
 * 3GPP TS 37.324 compliant interface
 */
#ifndef NTN_SDAP_H
#define NTN_SDAP_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/tag.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include <map>
#include <cstdint>

namespace ns3 {

/**
 * Timestamp tag for SDAP-level E2E delay measurement.
 */
class NtnSdapTag : public Tag
{
public:
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const override;

    NtnSdapTag ();
    NtnSdapTag (Time sendTime, uint8_t qfi);

    virtual void Serialize (TagBuffer i) const override;
    virtual void Deserialize (TagBuffer i) override;
    virtual uint32_t GetSerializedSize (void) const override;
    virtual void Print (std::ostream& os) const override;

    Time GetSendTime () const;
    uint8_t GetQfi () const;

private:
    Time m_sendTime;
    uint8_t m_qfi;
};

class NtnSdap : public Object
{
public:
    static TypeId GetTypeId (void);
    NtnSdap ();
    virtual ~NtnSdap ();

    void SetRnti (uint16_t rnti);

    // QoS flow <-> DRB mapping
    void MapQfiToDrb (uint8_t qfi, uint8_t lcid);
    uint8_t GetDrbForQfi (uint8_t qfi) const;

    // Data path
    void DoSendData (uint8_t qfi, Ptr<Packet> p);
    void DoReceiveData (uint8_t qfi, Ptr<Packet> p);

    // TracedCallbacks
    TracedCallback<uint16_t, uint8_t, uint8_t, uint32_t> m_txSdapTrace;           // (rnti, qfi, lcid, size)
    TracedCallback<uint16_t, uint8_t, uint8_t, uint32_t, uint64_t> m_rxSdapTrace; // (rnti, qfi, lcid, size, delay_ns)

private:
    uint16_t m_rnti;
    std::map<uint8_t, uint8_t> m_qfiToDrb;  // QFI -> LCID
    uint8_t m_defaultLcid;                    // default DRB if QFI not mapped
};

} // namespace ns3

#endif /* NTN_SDAP_H */
