/*
 * contrib/geo-sat/helper/sat-sdap-stats-collector.cc
 * SDAP layer statistics collector implementation
 */
#include "sat-sdap-stats-collector.h"
#include "ns3/log.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <set>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatSdapStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatSdapStatsCollector);

TypeId
SatSdapStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatSdapStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatSdapStatsCollector> ();
    return tid;
}

SatSdapStatsCollector::SatSdapStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatSdapStatsCollector::~SatSdapStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

void
SatSdapStatsCollector::OnTxSdap (uint16_t rnti, uint8_t qfi, uint8_t lcid, uint32_t size)
{
    m_qfiStats[UeQfiKey (rnti, qfi)].txBytes += size;
    m_qfiStats[UeQfiKey (rnti, qfi)].txPduCount++;
    m_drbStats[UeLcidKey (rnti, lcid)].txBytes += size;
    m_drbStats[UeLcidKey (rnti, lcid)].txPduCount++;
}

void
SatSdapStatsCollector::OnRxSdap (uint16_t rnti, uint8_t qfi, uint8_t lcid,
                                   uint32_t size, uint64_t delayNs)
{
    double delayMs = static_cast<double> (delayNs) / 1e6;

    auto& qacc = m_qfiStats[UeQfiKey (rnti, qfi)];
    qacc.rxBytes += size;
    qacc.rxPduCount++;
    qacc.totalDelayMs += delayMs;
    if (delayMs > qacc.maxDelayMs) qacc.maxDelayMs = delayMs;
    qacc.delaySamples.push_back (delayMs);

    auto& dacc = m_drbStats[UeLcidKey (rnti, lcid)];
    dacc.rxBytes += size;
    dacc.rxPduCount++;
    dacc.totalDelayMs += delayMs;
    if (delayMs > dacc.maxDelayMs) dacc.maxDelayMs = delayMs;
    dacc.delaySamples.push_back (delayMs);
}

double
SatSdapStatsCollector::ComputeP95 (const std::vector<double>& samples) const
{
    if (samples.empty ()) return 0.0;
    std::vector<double> sorted = samples;
    std::sort (sorted.begin (), sorted.end ());
    size_t idx = static_cast<size_t> (std::ceil (0.95 * sorted.size ())) - 1;
    return sorted[idx];
}

SdapFlowStats
SatSdapStatsCollector::AccumulatorToStats (const FlowAccumulator& acc) const
{
    SdapFlowStats s = {};
    s.txBytes = acc.txBytes;
    s.rxBytes = acc.rxBytes;
    s.txPduCount = acc.txPduCount;
    s.rxPduCount = acc.rxPduCount;
    s.avgDelayMs = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
    s.maxDelayMs = acc.maxDelayMs;
    s.p95DelayMs = ComputeP95 (acc.delaySamples);
    return s;
}

SdapFlowStats
SatSdapStatsCollector::GetQfiStats (uint8_t qfi) const
{
    FlowAccumulator agg;
    for (const auto& kv : m_qfiStats)
    {
        if (kv.first.second == qfi)
        {
            agg.txBytes += kv.second.txBytes;
            agg.rxBytes += kv.second.rxBytes;
            agg.txPduCount += kv.second.txPduCount;
            agg.rxPduCount += kv.second.rxPduCount;
            agg.totalDelayMs += kv.second.totalDelayMs;
            if (kv.second.maxDelayMs > agg.maxDelayMs) agg.maxDelayMs = kv.second.maxDelayMs;
            agg.delaySamples.insert (agg.delaySamples.end (),
                                     kv.second.delaySamples.begin (),
                                     kv.second.delaySamples.end ());
        }
    }
    return AccumulatorToStats (agg);
}

SdapFlowStats
SatSdapStatsCollector::GetDrbStats (uint8_t lcid) const
{
    FlowAccumulator agg;
    for (const auto& kv : m_drbStats)
    {
        if (kv.first.second == lcid)
        {
            agg.txBytes += kv.second.txBytes;
            agg.rxBytes += kv.second.rxBytes;
            agg.txPduCount += kv.second.txPduCount;
            agg.rxPduCount += kv.second.rxPduCount;
            agg.totalDelayMs += kv.second.totalDelayMs;
            if (kv.second.maxDelayMs > agg.maxDelayMs) agg.maxDelayMs = kv.second.maxDelayMs;
            agg.delaySamples.insert (agg.delaySamples.end (),
                                     kv.second.delaySamples.begin (),
                                     kv.second.delaySamples.end ());
        }
    }
    return AccumulatorToStats (agg);
}

SdapFlowStats
SatSdapStatsCollector::GetUeQfiStats (uint16_t rnti, uint8_t qfi) const
{
    auto it = m_qfiStats.find (UeQfiKey (rnti, qfi));
    if (it == m_qfiStats.end ()) return {};
    return AccumulatorToStats (it->second);
}

double
SatSdapStatsCollector::GetQfiThroughput (uint8_t qfi, Time duration) const
{
    auto stats = GetQfiStats (qfi);
    double sec = duration.GetSeconds ();
    return (sec > 0) ? (stats.rxBytes * 8.0 / sec) : 0.0;
}

double
SatSdapStatsCollector::GetDrbThroughput (uint8_t lcid, Time duration) const
{
    auto stats = GetDrbStats (lcid);
    double sec = duration.GetSeconds ();
    return (sec > 0) ? (stats.rxBytes * 8.0 / sec) : 0.0;
}

void
SatSdapStatsCollector::ExportToFile (const std::string& filepath, Time simDuration) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    double durSec = simDuration.GetSeconds ();

    // Per-QFI summary
    ofs << "# SDAP Layer Statistics\n\n";
    ofs << "# Per-QFI Summary\n";
    ofs << "# QFI\tTxBytes\tRxBytes\tAvgDelay(ms)\tP95Delay(ms)\tThroughput(bps)\n";

    std::set<uint8_t> allQfis;
    for (const auto& kv : m_qfiStats) allQfis.insert (kv.first.second);

    for (uint8_t qfi : allQfis)
    {
        auto s = GetQfiStats (qfi);
        double tput = (durSec > 0) ? (s.rxBytes * 8.0 / durSec) : 0.0;
        ofs << (uint32_t)qfi << "\t" << s.txBytes << "\t" << s.rxBytes << "\t"
            << s.avgDelayMs << "\t" << s.p95DelayMs << "\t" << tput << "\n";
    }

    // Per-DRB summary
    ofs << "\n# Per-DRB (LCID) Summary\n";
    ofs << "# LCID\tTxBytes\tRxBytes\tAvgDelay(ms)\tP95Delay(ms)\tThroughput(bps)\n";

    std::set<uint8_t> allLcids;
    for (const auto& kv : m_drbStats) allLcids.insert (kv.first.second);

    for (uint8_t lcid : allLcids)
    {
        auto s = GetDrbStats (lcid);
        double tput = (durSec > 0) ? (s.rxBytes * 8.0 / durSec) : 0.0;
        ofs << (uint32_t)lcid << "\t" << s.txBytes << "\t" << s.rxBytes << "\t"
            << s.avgDelayMs << "\t" << s.p95DelayMs << "\t" << tput << "\n";
    }

    // Per-UE per-QFI detail
    ofs << "\n# Per-UE Per-QFI Detail\n";
    ofs << "# RNTI\tQFI\tTxBytes\tRxBytes\tAvgDelay(ms)\tMaxDelay(ms)\n";
    for (const auto& kv : m_qfiStats)
    {
        auto s = AccumulatorToStats (kv.second);
        ofs << kv.first.first << "\t" << (uint32_t)kv.first.second << "\t"
            << s.txBytes << "\t" << s.rxBytes << "\t"
            << s.avgDelayMs << "\t" << s.maxDelayMs << "\n";
    }

    ofs.close ();
    NS_LOG_INFO ("SDAP stats exported to " << filepath);
}

void
SatSdapStatsCollector::Reset ()
{
    m_qfiStats.clear ();
    m_drbStats.clear ();
}

} // namespace ns3
