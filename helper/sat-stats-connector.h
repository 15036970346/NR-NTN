/*
 * contrib/geo-sat/helper/sat-stats-connector.h
 * Wiring class: connects TracedCallbacks from model objects to per-layer stats collectors
 * Modeled on NrBearerStatsConnector (src/nr/helper/nr-bearer-stats-connector.h)
 */
#ifndef SAT_STATS_CONNECTOR_H
#define SAT_STATS_CONNECTOR_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "sat-phy-stats-collector.h"
#include "sat-mac-stats-collector.h"
#include "sat-rlc-stats-collector.h"
#include "sat-pdcp-stats-collector.h"
#include "sat-sdap-stats-collector.h"
#include "sat-rrc-stats-collector.h"
#include "sat-e2e-stats-collector.h"

namespace ns3 {

class SatStatsConnector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatStatsConnector ();
    virtual ~SatStatsConnector ();

    // Enable per-layer stats collection
    void SetPhyStats (Ptr<SatPhyStatsCollector> stats);
    void SetMacStats (Ptr<SatMacStatsCollector> stats);
    void SetRlcStats (Ptr<SatRlcStatsCollector> stats);
    void SetPdcpStats (Ptr<SatPdcpStatsCollector> stats);
    void SetSdapStats (Ptr<SatSdapStatsCollector> stats);
    void SetRrcStats (Ptr<SatRrcStatsCollector> stats);
    void SetE2eStats (Ptr<SatE2eStatsCollector> stats);

    // Access collectors
    Ptr<SatPhyStatsCollector>  GetPhyStats () const;
    Ptr<SatMacStatsCollector>  GetMacStats () const;
    Ptr<SatRlcStatsCollector>  GetRlcStats () const;
    Ptr<SatPdcpStatsCollector> GetPdcpStats () const;
    Ptr<SatSdapStatsCollector> GetSdapStats () const;
    Ptr<SatRrcStatsCollector>  GetRrcStats () const;
    Ptr<SatE2eStatsCollector>  GetE2eStats () const;

    // Export all stats to a directory
    void ExportAll (const std::string& outputDir, Time simDuration) const;

private:
    Ptr<SatPhyStatsCollector>  m_phyStats;
    Ptr<SatMacStatsCollector>  m_macStats;
    Ptr<SatRlcStatsCollector>  m_rlcStats;
    Ptr<SatPdcpStatsCollector> m_pdcpStats;
    Ptr<SatSdapStatsCollector> m_sdapStats;
    Ptr<SatRrcStatsCollector>  m_rrcStats;
    Ptr<SatE2eStatsCollector>  m_e2eStats;
};

} // namespace ns3

#endif /* SAT_STATS_CONNECTOR_H */
