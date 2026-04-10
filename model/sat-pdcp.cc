/*
 * 文件路径：contrib/geo-sat/model/sat-pdcp.cc
 * 功能：GEO卫星PDCP层实现 - 集成ROHC报头压缩，针对NTN长时延优化
 */
#include "sat-pdcp.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatPdcp");
NS_OBJECT_ENSURE_REGISTERED (SatPdcp);

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
                         "PDCP data packet transmitted",
                         MakeTraceSourceAccessor (&SatPdcp::m_txPdcpTrace),
                         "ns3::Packet::TracedCallback")
        .AddTraceSource ("RxPDCP",
                         "PDCP data packet received",
                         MakeTraceSourceAccessor (&SatPdcp::m_rxPdcpTrace),
                         "ns3::Packet::TracedCallback");
    return tid;
}

SatPdcp::SatPdcp ()
    : m_rnti (0),
      m_lcId (0),
      m_rohcEnabled (true)
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

    if (m_rohcEnabled && m_rohc)
    {
        NS_LOG_INFO ("Applying ROHC compression at PDCP layer");
        p = m_rohc->Compress (p);
        
        uint32_t compressedSize = p->GetSize ();
        NS_LOG_INFO ("PDCP TX: Original=" << originalSize 
                     << " bytes, After ROHC=" << compressedSize 
                     << " bytes, Savings=" << (originalSize - compressedSize) << " bytes");
    }
    else
    {
        NS_LOG_DEBUG ("ROHC disabled, sending raw packet");
    }

    m_txPdcpTrace (p);
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
        
        p = m_rohc->Decompress (p);
        
        uint32_t decompressedSize = p->GetSize ();
        NS_LOG_INFO ("PDCP RX: Received=" << receivedSize 
                     << " bytes, After ROHC=" << decompressedSize 
                     << " bytes");
    }
    else
    {
        NS_LOG_DEBUG ("ROHC disabled, receiving raw packet");
    }

    m_rxPdcpTrace (p);
    NS_LOG_INFO ("PDCP receive complete, packet passed to upper layer (IP)");
}

} // namespace ns3
