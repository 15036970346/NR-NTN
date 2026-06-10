/*
 * contrib/geo-sat/helper/sat-rrc-stats-collector.cc
 * RRC layer statistics collector implementation
 */
#include "sat-rrc-stats-collector.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <fstream>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatRrcStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatRrcStatsCollector);

TypeId
SatRrcStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatRrcStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatRrcStatsCollector> ();
    return tid;
}

SatRrcStatsCollector::SatRrcStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatRrcStatsCollector::~SatRrcStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

void
SatRrcStatsCollector::OnStateTransition (uint16_t ueId,
                                          SatUtRrc::State oldState,
                                          SatUtRrc::State newState)
{
    auto& acc = m_ueStats[ueId];

    // Accumulate time in old state
    if (acc.lastTransitionTime > Seconds (0))
    {
        double durationMs = (Simulator::Now () - acc.lastTransitionTime).GetMilliSeconds ();
        acc.cumulativeTimeMs[oldState] += durationMs;
    }

    // Count new state entry
    acc.entryCount[newState]++;
    acc.lastState = newState;
    acc.lastTransitionTime = Simulator::Now ();

    // Track paging events (entering IDLE_RANDOM_ACCESS from INACTIVE)
    if (oldState == SatUtRrc::RRC_INACTIVE && newState == SatUtRrc::IDLE_RANDOM_ACCESS)
    {
        acc.pagingCount++;
    }
}

void
SatRrcStatsCollector::RecordRecoveryDelay (uint16_t ueId, Time delay)
{
    m_ueStats[ueId].recoveryDelaysMs.push_back (delay.GetMilliSeconds ());
}

RrcUeStats
SatRrcStatsCollector::GetUeStats (uint16_t ueId) const
{
    RrcUeStats result = {};
    auto it = m_ueStats.find (ueId);
    if (it == m_ueStats.end ()) return result;

    const auto& acc = it->second;
    result.transitionCount = acc.entryCount;
    result.timeInStateMs = acc.cumulativeTimeMs;
    result.pagingCount = acc.pagingCount;

    if (!acc.recoveryDelaysMs.empty ())
    {
        result.avgRecoveryDelayMs = std::accumulate (acc.recoveryDelaysMs.begin (),
                                                      acc.recoveryDelaysMs.end (), 0.0)
                                    / acc.recoveryDelaysMs.size ();
        result.maxRecoveryDelayMs = *std::max_element (acc.recoveryDelaysMs.begin (),
                                                        acc.recoveryDelaysMs.end ());
    }
    return result;
}

uint32_t
SatRrcStatsCollector::GetTotalTransitions () const
{
    uint32_t total = 0;
    for (const auto& kv : m_ueStats)
    {
        for (const auto& sc : kv.second.entryCount)
        {
            total += sc.second;
        }
    }
    return total;
}

double
SatRrcStatsCollector::GetAverageRecoveryDelayMs () const
{
    double totalDelay = 0.0;
    uint32_t totalCount = 0;
    for (const auto& kv : m_ueStats)
    {
        for (double d : kv.second.recoveryDelaysMs)
        {
            totalDelay += d;
            totalCount++;
        }
    }
    return (totalCount > 0) ? totalDelay / totalCount : 0.0;
}

void
SatRrcStatsCollector::ExportToFile (const std::string& filepath) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    ofs << "# RRC Layer Statistics\n\n";

    for (const auto& kv : m_ueStats)
    {
        uint16_t ueId = kv.first;
        const auto& acc = kv.second;

        ofs << "UE " << ueId << ":\n";
        ofs << "  State\t\t\t\tEntries\tTime(ms)\n";

        for (int s = 0; s < SatUtRrc::NUM_STATES; ++s)
        {
            auto state = static_cast<SatUtRrc::State> (s);
            uint32_t entries = 0;
            double timeMs = 0.0;

            auto eit = acc.entryCount.find (state);
            if (eit != acc.entryCount.end ()) entries = eit->second;
            auto tit = acc.cumulativeTimeMs.find (state);
            if (tit != acc.cumulativeTimeMs.end ()) timeMs = tit->second;

            if (entries > 0 || timeMs > 0)
            {
                ofs << "  " << SatUtRrc::StateToString (state)
                    << "\t" << entries << "\t" << timeMs << "\n";
            }
        }

        if (!acc.recoveryDelaysMs.empty ())
        {
            double avg = std::accumulate (acc.recoveryDelaysMs.begin (),
                                          acc.recoveryDelaysMs.end (), 0.0)
                         / acc.recoveryDelaysMs.size ();
            double maxD = *std::max_element (acc.recoveryDelaysMs.begin (),
                                              acc.recoveryDelaysMs.end ());
            ofs << "  Recovery: avg=" << avg << "ms max=" << maxD
                << "ms count=" << acc.recoveryDelaysMs.size () << "\n";
        }
        ofs << "  Paging events: " << acc.pagingCount << "\n\n";
    }

    ofs << "# Total transitions: " << GetTotalTransitions () << "\n";
    ofs << "# Avg recovery delay: " << GetAverageRecoveryDelayMs () << " ms\n";

    ofs.close ();
    NS_LOG_INFO ("RRC stats exported to " << filepath);
}

void
SatRrcStatsCollector::Reset ()
{
    m_ueStats.clear ();
}

} // namespace ns3
