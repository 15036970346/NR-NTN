/*
 * contrib/geo-sat/helper/sat-rrc-stats-collector.h
 * RRC layer statistics: state transitions, time per state, access recovery delay
 */
#ifndef SAT_RRC_STATS_COLLECTOR_H
#define SAT_RRC_STATS_COLLECTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/sat-ut-rrc.h"
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

struct RrcUeStats {
    std::map<SatUtRrc::State, uint32_t> transitionCount; // per-state entry count
    std::map<SatUtRrc::State, double>   timeInStateMs;   // cumulative ms in each state
    double avgRecoveryDelayMs;
    double maxRecoveryDelayMs;
    uint32_t pagingCount;
};

class SatRrcStatsCollector : public Object
{
public:
    static TypeId GetTypeId (void);
    SatRrcStatsCollector ();
    virtual ~SatRrcStatsCollector ();

    // TracedCallback sink (matches SatUtRrc::m_stateTransitionTrace)
    void OnStateTransition (uint16_t ueId, SatUtRrc::State oldState, SatUtRrc::State newState);

    // Record access recovery delay (after paging → CONNECTED)
    void RecordRecoveryDelay (uint16_t ueId, Time delay);

    // Query per-UE stats
    RrcUeStats GetUeStats (uint16_t ueId) const;

    // Aggregate stats
    uint32_t GetTotalTransitions () const;
    double GetAverageRecoveryDelayMs () const;

    // Export to file
    void ExportToFile (const std::string& filepath) const;

    void Reset ();

private:
    struct UeAccumulator {
        std::map<SatUtRrc::State, uint32_t> entryCount;
        std::map<SatUtRrc::State, double>   cumulativeTimeMs;
        SatUtRrc::State lastState = SatUtRrc::IDLE_START;
        Time lastTransitionTime;
        std::vector<double> recoveryDelaysMs;
        uint32_t pagingCount = 0;
    };

    std::map<uint16_t, UeAccumulator> m_ueStats;
};

} // namespace ns3

#endif /* SAT_RRC_STATS_COLLECTOR_H */
