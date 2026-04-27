/*
 * contrib/geo-sat/helper/sat-stats-connector.cc
 * Stats connector implementation
 */
#include "sat-stats-connector.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatStatsConnector");
NS_OBJECT_ENSURE_REGISTERED (SatStatsConnector);

TypeId
SatStatsConnector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatStatsConnector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatStatsConnector> ();
    return tid;
}

SatStatsConnector::SatStatsConnector ()
{
    NS_LOG_FUNCTION (this);
}

SatStatsConnector::~SatStatsConnector ()
{
    NS_LOG_FUNCTION (this);
}

void SatStatsConnector::SetPhyStats (Ptr<SatPhyStatsCollector> stats)  { m_phyStats = stats; }
void SatStatsConnector::SetMacStats (Ptr<SatMacStatsCollector> stats)  { m_macStats = stats; }
void SatStatsConnector::SetRlcStats (Ptr<SatRlcStatsCollector> stats)  { m_rlcStats = stats; }
void SatStatsConnector::SetPdcpStats (Ptr<SatPdcpStatsCollector> stats) { m_pdcpStats = stats; }
void SatStatsConnector::SetSdapStats (Ptr<SatSdapStatsCollector> stats) { m_sdapStats = stats; }
void SatStatsConnector::SetRrcStats (Ptr<SatRrcStatsCollector> stats)  { m_rrcStats = stats; }
void SatStatsConnector::SetE2eStats (Ptr<SatE2eStatsCollector> stats)  { m_e2eStats = stats; }

Ptr<SatPhyStatsCollector>  SatStatsConnector::GetPhyStats () const  { return m_phyStats; }
Ptr<SatMacStatsCollector>  SatStatsConnector::GetMacStats () const  { return m_macStats; }
Ptr<SatRlcStatsCollector>  SatStatsConnector::GetRlcStats () const  { return m_rlcStats; }
Ptr<SatPdcpStatsCollector> SatStatsConnector::GetPdcpStats () const { return m_pdcpStats; }
Ptr<SatSdapStatsCollector> SatStatsConnector::GetSdapStats () const { return m_sdapStats; }
Ptr<SatRrcStatsCollector>  SatStatsConnector::GetRrcStats () const  { return m_rrcStats; }
Ptr<SatE2eStatsCollector>  SatStatsConnector::GetE2eStats () const  { return m_e2eStats; }

void
SatStatsConnector::ExportAll (const std::string& outputDir, Time simDuration) const
{
    NS_LOG_FUNCTION (this << outputDir);

    if (m_phyStats)
    {
        m_phyStats->ExportToFile (outputDir + "/phy_stats.txt");
    }
    if (m_macStats)
    {
        m_macStats->ExportToFile (outputDir + "/mac_stats.txt");
    }
    if (m_rlcStats)
    {
        m_rlcStats->ExportToFile (outputDir + "/rlc_stats.txt", simDuration);
    }
    if (m_pdcpStats)
    {
        m_pdcpStats->ExportToFile (outputDir + "/pdcp_stats.txt");
    }
    if (m_sdapStats)
    {
        m_sdapStats->ExportToFile (outputDir + "/sdap_stats.txt", simDuration);
    }
    if (m_rrcStats)
    {
        m_rrcStats->ExportToFile (outputDir + "/rrc_stats.txt");
    }
    if (m_e2eStats)
    {
        m_e2eStats->ExportToFile (outputDir + "/e2e_delay_stats.txt");
    }

    NS_LOG_INFO ("All layer stats exported to " << outputDir);
}

} // namespace ns3
