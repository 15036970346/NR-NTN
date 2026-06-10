/*
 * contrib/geo-sat/helper/sat-mac-stats-collector.h
 * MAC layer statistics collector: queue delay, queue length, mux/demux packet stats
 * Reuse pattern: NrMacSchedulingStats (src/nr/helper/nr-mac-scheduling-stats.h)
 */
#ifndef SAT_MAC_STATS_COLLECTOR_H
#define SAT_MAC_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

struct MacStatsResult {
    double min;
    double max;
    double avg;
    double p95;
    uint32_t count;
};

class SatMacStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatMacStatsCollector ();
    virtual ~SatMacStatsCollector ();

    // TracedCallback sinks (match SatUtMac trace signatures)
    void OnQueueLength (uint16_t rnti, uint32_t bufferBytes);
    void OnQueueDelay (uint16_t rnti, int64_t delayNs);

    // Record mux/demux packet sizes
    void RecordMuxPacketSize (uint16_t rnti, uint32_t sizeBytes);
    void RecordDemuxPacketSize (uint16_t rnti, uint32_t sizeBytes);

    // Query per-UE stats
    MacStatsResult GetQueueLengthStats (uint16_t rnti) const;
    MacStatsResult GetQueueDelayStats (uint16_t rnti) const;
    MacStatsResult GetMuxPacketStats (uint16_t rnti) const;
    MacStatsResult GetDemuxPacketStats (uint16_t rnti) const;

    // Query aggregate stats
    MacStatsResult GetAggregateQueueDelayStats () const;
    MacStatsResult GetAggregateQueueLengthStats () const;

    // Export to file
    void ExportToFile (const std::string& filepath) const;

    void Reset ();

private:
    MacStatsResult ComputeStats (const std::vector<double>& samples) const;

    std::map<uint16_t, std::vector<double>> m_queueLengthSamples;  // bytes
    std::map<uint16_t, std::vector<double>> m_queueDelaySamples;   // ms
    std::map<uint16_t, std::vector<double>> m_muxPacketSamples;    // bytes
    std::map<uint16_t, std::vector<double>> m_demuxPacketSamples;  // bytes
};

} // namespace ns3

#endif /* SAT_MAC_STATS_COLLECTOR_H */
