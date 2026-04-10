/*
 * 文件路径：contrib/geo-sat/model/sat-pdcp.h
 * 功能：GEO卫星PDCP层实现 - 集成ROHC报头压缩，针对NTN长时延优化
 */
#ifndef SAT_PDCP_H
#define SAT_PDCP_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "rohc-compressor.h"
#include "ns3/traced-callback.h"

namespace ns3 {

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

    TracedCallback<Ptr<Packet>> m_txPdcpTrace;
    TracedCallback<Ptr<Packet>> m_rxPdcpTrace;

private:
    Ptr<RohcCompressor> m_rohc;
    Ptr<Object> m_pdcpSapUser;
    uint16_t m_rnti;
    uint8_t m_lcId;
    bool m_rohcEnabled;
};

} // namespace ns3

#endif /* SAT_PDCP_H */
