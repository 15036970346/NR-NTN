/*
 * contrib/geo-sat/helper/sat-e2e-stats-collector.h
 * End-to-end delay aggregation: min/max/avg/P95 across PHY+MAC+RLC+PDCP+SDAP layers
 */
#ifndef SAT_E2E_STATS_COLLECTOR_H
#define SAT_E2E_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

struct E2eDelayStats {
    double minDelayMs;
    double maxDelayMs;
    double avgDelayMs;
    double p95DelayMs;
    uint32_t sampleCount;
};

class SatE2eStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatE2eStatsCollector ();
    virtual ~SatE2eStatsCollector ();

    // Record per-layer delay components for a UE
    void RecordPhyDelay (uint16_t rnti, double delayMs);
    void RecordMacDelay (uint16_t rnti, double delayMs);
    void RecordRlcDelay (uint16_t rnti, double delayMs);
    void RecordPdcpDelay (uint16_t rnti, double delayMs);
    void RecordSdapDelay (uint16_t rnti, double delayMs);

    // Record a complete E2E delay sample (sum of all layers for one packet)
    void RecordE2eDelay (uint16_t rnti, double totalDelayMs);

    // Query per-UE E2E stats
    E2eDelayStats GetE2eStats (uint16_t rnti) const;

    // Aggregate E2E stats across all UEs
    E2eDelayStats GetAggregateE2eStats () const;

    // Per-layer aggregate stats
    E2eDelayStats GetLayerStats (const std::string& layer) const;

    // Export to file
    void ExportToFile (const std::string& filepath) const;

    void Reset ();

private:
    E2eDelayStats ComputeStats (const std::vector<double>& samples) const;

    std::map<uint16_t, std::vector<double>> m_e2eDelaySamples;

    // Per-layer delay pools (aggregate across all UEs)
    std::vector<double> m_phyDelays;
    std::vector<double> m_macDelays;
    std::vector<double> m_rlcDelays;
    std::vector<double> m_pdcpDelays;
    std::vector<double> m_sdapDelays;
};

} // namespace ns3

#endif /* SAT_E2E_STATS_COLLECTOR_H */
