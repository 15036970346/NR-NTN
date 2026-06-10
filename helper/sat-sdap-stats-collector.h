/*
 * contrib/geo-sat/helper/sat-sdap-stats-collector.h
 * SDAP layer statistics: per-QFI throughput, per-DRB throughput, E2E delay per QoS flow
 */
#ifndef SAT_SDAP_STATS_COLLECTOR_H
#define SAT_SDAP_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

struct SdapFlowStats {
    uint64_t txBytes;
    uint64_t rxBytes;
    uint32_t txPduCount;
    uint32_t rxPduCount;
    double   avgDelayMs;
    double   maxDelayMs;
    double   p95DelayMs;
};

class SatSdapStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatSdapStatsCollector ();
    virtual ~SatSdapStatsCollector ();

    // TracedCallback sinks (match NtnSdap trace signatures)
    void OnTxSdap (uint16_t rnti, uint8_t qfi, uint8_t lcid, uint32_t size);
    void OnRxSdap (uint16_t rnti, uint8_t qfi, uint8_t lcid, uint32_t size, uint64_t delayNs);

    // Query per-QFI stats (aggregate across all UEs)
    SdapFlowStats GetQfiStats (uint8_t qfi) const;

    // Query per-DRB (LCID) stats (aggregate across all UEs)
    SdapFlowStats GetDrbStats (uint8_t lcid) const;

    // Per-UE per-QFI stats
    SdapFlowStats GetUeQfiStats (uint16_t rnti, uint8_t qfi) const;

    // Throughput (bps) for a QFI over given duration
    double GetQfiThroughput (uint8_t qfi, Time duration) const;
    double GetDrbThroughput (uint8_t lcid, Time duration) const;

    // Export to file
    void ExportToFile (const std::string& filepath, Time simDuration) const;

    void Reset ();

private:
    // Key: (rnti, qfi)
    typedef std::pair<uint16_t, uint8_t> UeQfiKey;
    // Key: (rnti, lcid)
    typedef std::pair<uint16_t, uint8_t> UeLcidKey;

    struct FlowAccumulator {
        uint64_t txBytes = 0;
        uint64_t rxBytes = 0;
        uint32_t txPduCount = 0;
        uint32_t rxPduCount = 0;
        double   totalDelayMs = 0.0;
        double   maxDelayMs = 0.0;
        std::vector<double> delaySamples;
    };

    double ComputeP95 (const std::vector<double>& samples) const;
    SdapFlowStats AccumulatorToStats (const FlowAccumulator& acc) const;

    std::map<UeQfiKey, FlowAccumulator> m_qfiStats;
    std::map<UeLcidKey, FlowAccumulator> m_drbStats;
};

} // namespace ns3

#endif /* SAT_SDAP_STATS_COLLECTOR_H */
