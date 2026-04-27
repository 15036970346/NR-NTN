/*
 * contrib/geo-sat/helper/sat-phy-stats-collector.h
 * PHY layer statistics collector: RSRP, RSRQ, SINR, propagation delay
 * Reuse pattern: RxPacketTraceParams (src/nr/model/nr-phy-mac-common.h)
 */
#ifndef SAT_PHY_STATS_COLLECTOR_H
#define SAT_PHY_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

struct PhyMeasSample {
    double rsrp;      // dBm
    double sinr;      // dB
    double rsrq;      // dB
    Time   timestamp;
};

struct PhyStatsResult {
    double min;
    double max;
    double avg;
    double stddev;
    uint32_t count;
};

class SatPhyStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatPhyStatsCollector ();
    virtual ~SatPhyStatsCollector ();

    // TracedCallback sinks (match SatUtPhy trace signatures)
    void OnRsrp (uint16_t rnti, double rsrpDbm);
    void OnSinr (uint16_t rnti, double sinrDb);
    void OnRsrq (uint16_t rnti, double rsrqDb);

    // Record propagation delay for a UE
    void RecordPropDelay (uint16_t rnti, Time oneWayDelay);

    // Query per-UE stats
    PhyStatsResult GetRsrpStats (uint16_t rnti) const;
    PhyStatsResult GetSinrStats (uint16_t rnti) const;
    PhyStatsResult GetRsrqStats (uint16_t rnti) const;
    PhyStatsResult GetPropDelayStats (uint16_t rnti) const;

    // Query aggregate stats (all UEs)
    PhyStatsResult GetAggregateRsrpStats () const;
    PhyStatsResult GetAggregateSinrStats () const;
    PhyStatsResult GetAggregateRsrqStats () const;

    // Export to file
    void ExportToFile (const std::string& filepath) const;

    // Reset
    void Reset ();

private:
    PhyStatsResult ComputeStats (const std::vector<double>& samples) const;

    std::map<uint16_t, std::vector<double>> m_rsrpSamples;   // per-UE RSRP
    std::map<uint16_t, std::vector<double>> m_sinrSamples;   // per-UE SINR
    std::map<uint16_t, std::vector<double>> m_rsrqSamples;   // per-UE RSRQ
    std::map<uint16_t, std::vector<double>> m_propDelaySamples; // per-UE prop delay (ms)
};

} // namespace ns3

#endif /* SAT_PHY_STATS_COLLECTOR_H */
