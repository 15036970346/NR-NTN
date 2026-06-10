/*
 * contrib/geo-sat/helper/sat-pdcp-stats-collector.h
 * PDCP layer statistics: per-bearer delay, ROHC compression ratio, incomplete packets
 * Reuse: NrBearerStatsCalculator pattern, keyed by (IMSI, LCID)
 */
#ifndef SAT_PDCP_STATS_COLLECTOR_H
#define SAT_PDCP_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

typedef std::pair<uint64_t, uint8_t> ImsiLcidPair;

struct PdcpBearerStats {
    uint64_t txBytes;
    uint64_t rxBytes;
    uint32_t txPduCount;
    uint32_t rxPduCount;
    double   avgDelayMs;
    double   maxDelayMs;
    double   p95DelayMs;
    double   avgRohcRatio;   // compressedSize / originalSize (< 1.0 = good)
    uint32_t rohcSampleCount;
    uint32_t incompletePackets;
};

class SatPdcpStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatPdcpStatsCollector ();
    virtual ~SatPdcpStatsCollector ();

    // TracedCallback sinks (matching NR-compatible NtnPdcp signatures)
    void DlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size);
    void DlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs);
    void UlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size);
    void UlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs);

    // ROHC compression trace sink
    void OnRohcCompression (uint16_t rnti, uint8_t lcid, uint32_t origSize, uint32_t compressedSize);

    // Record incomplete packets
    void RecordIncompletePacket (uint16_t rnti, uint8_t lcid);

    // IMSI mapping
    void SetImsiForRnti (uint16_t rnti, uint64_t imsi);

    // Query stats
    PdcpBearerStats GetDlBearerStats (uint64_t imsi, uint8_t lcid) const;
    PdcpBearerStats GetUlBearerStats (uint64_t imsi, uint8_t lcid) const;

    // Aggregate ROHC compression ratio
    double GetAggregateRohcRatio () const;

    // Export to file
    void ExportToFile (const std::string& filepath) const;

    void Reset ();

private:
    struct BearerAccumulator {
        uint64_t txBytes = 0;
        uint64_t rxBytes = 0;
        uint32_t txPduCount = 0;
        uint32_t rxPduCount = 0;
        double   totalDelayMs = 0.0;
        double   maxDelayMs = 0.0;
        std::vector<double> delaySamples; // for P95
    };

    struct RohcAccumulator {
        double   totalRatio = 0.0;
        uint32_t sampleCount = 0;
        uint32_t incompletePackets = 0;
    };

    ImsiLcidPair MakeKey (uint16_t rnti, uint8_t lcid) const;
    double ComputeP95 (const std::vector<double>& samples) const;

    std::map<uint16_t, uint64_t> m_rntiToImsi;
    std::map<ImsiLcidPair, BearerAccumulator> m_dlStats;
    std::map<ImsiLcidPair, BearerAccumulator> m_ulStats;
    std::map<ImsiLcidPair, RohcAccumulator> m_rohcStats;
};

} // namespace ns3

#endif /* SAT_PDCP_STATS_COLLECTOR_H */
