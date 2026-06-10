/*
 * contrib/geo-sat/helper/sat-rlc-stats-collector.h
 * RLC layer statistics: per-mode throughput, AM retransmission estimation
 * Reuse: NrBearerStatsBase pattern, keyed by (IMSI, LCID)
 */
#ifndef SAT_RLC_STATS_COLLECTOR_H
#define SAT_RLC_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <string>
#include <cstdint>

namespace ns3 {

typedef std::pair<uint64_t, uint8_t> ImsiLcidPair;

struct RlcBearerStats {
    uint64_t txBytes;
    uint64_t rxBytes;
    uint32_t txPduCount;
    uint32_t rxPduCount;
    double   avgDelayMs;
    double   maxDelayMs;
    std::string rlcMode;  // "AM", "UM", "TM"
};

class SatRlcStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatRlcStatsCollector ();
    virtual ~SatRlcStatsCollector ();

    // TracedCallback sinks (matching lte-rlc.h signatures)
    void DlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size);
    void DlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs);
    void UlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size);
    void UlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs);

    // IMSI mapping (called when UE attaches)
    void SetImsiForRnti (uint16_t rnti, uint64_t imsi);

    // Set RLC mode for a bearer
    void SetRlcMode (uint64_t imsi, uint8_t lcid, const std::string& mode);

    // Query stats
    RlcBearerStats GetDlBearerStats (uint64_t imsi, uint8_t lcid) const;
    RlcBearerStats GetUlBearerStats (uint64_t imsi, uint8_t lcid) const;

    // Estimated AM retransmission count (TX - RX difference)
    uint32_t GetEstimatedRetxCount (uint64_t imsi, uint8_t lcid) const;

    // Per-mode aggregate throughput (bps) over given duration
    double GetModeThroughput (const std::string& mode, Time duration, bool downlink) const;

    // Export to file
    void ExportToFile (const std::string& filepath, Time simDuration) const;

    void Reset ();

private:
    struct BearerAccumulator {
        uint64_t txBytes = 0;
        uint64_t rxBytes = 0;
        uint32_t txPduCount = 0;
        uint32_t rxPduCount = 0;
        double   totalDelayMs = 0.0;
        double   maxDelayMs = 0.0;
        std::string rlcMode = "UM";  // default
    };

    ImsiLcidPair MakeKey (uint16_t rnti, uint8_t lcid) const;

    std::map<uint16_t, uint64_t> m_rntiToImsi;
    std::map<ImsiLcidPair, BearerAccumulator> m_dlStats;
    std::map<ImsiLcidPair, BearerAccumulator> m_ulStats;
    std::map<ImsiLcidPair, std::string> m_rlcModeMap;
};

} // namespace ns3

#endif /* SAT_RLC_STATS_COLLECTOR_H */
