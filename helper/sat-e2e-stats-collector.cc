/*
 * contrib/geo-sat/helper/sat-e2e-stats-collector.cc
 * End-to-end delay aggregation implementation
 */
#include "sat-e2e-stats-collector.h"
#include "ns3/log.h"
#include <fstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <set>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatE2eStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatE2eStatsCollector);

TypeId
SatE2eStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatE2eStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatE2eStatsCollector> ();
    return tid;
}

SatE2eStatsCollector::SatE2eStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatE2eStatsCollector::~SatE2eStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

void SatE2eStatsCollector::RecordPhyDelay (uint16_t rnti, double delayMs)
{
    (void)rnti;
    m_phyDelays.push_back (delayMs);
}

void SatE2eStatsCollector::RecordMacDelay (uint16_t rnti, double delayMs)
{
    (void)rnti;
    m_macDelays.push_back (delayMs);
}

void SatE2eStatsCollector::RecordRlcDelay (uint16_t rnti, double delayMs)
{
    (void)rnti;
    m_rlcDelays.push_back (delayMs);
}

void SatE2eStatsCollector::RecordPdcpDelay (uint16_t rnti, double delayMs)
{
    (void)rnti;
    m_pdcpDelays.push_back (delayMs);
}

void SatE2eStatsCollector::RecordSdapDelay (uint16_t rnti, double delayMs)
{
    (void)rnti;
    m_sdapDelays.push_back (delayMs);
}

void SatE2eStatsCollector::RecordE2eDelay (uint16_t rnti, double totalDelayMs)
{
    m_e2eDelaySamples[rnti].push_back (totalDelayMs);
}

E2eDelayStats
SatE2eStatsCollector::ComputeStats (const std::vector<double>& samples) const
{
    E2eDelayStats result = {0, 0, 0, 0, 0};
    if (samples.empty ()) return result;

    result.sampleCount = static_cast<uint32_t> (samples.size ());
    result.minDelayMs = *std::min_element (samples.begin (), samples.end ());
    result.maxDelayMs = *std::max_element (samples.begin (), samples.end ());
    result.avgDelayMs = std::accumulate (samples.begin (), samples.end (), 0.0) / result.sampleCount;

    std::vector<double> sorted = samples;
    std::sort (sorted.begin (), sorted.end ());
    size_t p95Idx = static_cast<size_t> (std::ceil (0.95 * sorted.size ())) - 1;
    result.p95DelayMs = sorted[p95Idx];

    return result;
}

E2eDelayStats
SatE2eStatsCollector::GetE2eStats (uint16_t rnti) const
{
    auto it = m_e2eDelaySamples.find (rnti);
    if (it == m_e2eDelaySamples.end ()) return {0, 0, 0, 0, 0};
    return ComputeStats (it->second);
}

E2eDelayStats
SatE2eStatsCollector::GetAggregateE2eStats () const
{
    std::vector<double> all;
    for (const auto& kv : m_e2eDelaySamples)
    {
        all.insert (all.end (), kv.second.begin (), kv.second.end ());
    }
    return ComputeStats (all);
}

E2eDelayStats
SatE2eStatsCollector::GetLayerStats (const std::string& layer) const
{
    if (layer == "PHY")  return ComputeStats (m_phyDelays);
    if (layer == "MAC")  return ComputeStats (m_macDelays);
    if (layer == "RLC")  return ComputeStats (m_rlcDelays);
    if (layer == "PDCP") return ComputeStats (m_pdcpDelays);
    if (layer == "SDAP") return ComputeStats (m_sdapDelays);
    return {0, 0, 0, 0, 0};
}

void
SatE2eStatsCollector::ExportToFile (const std::string& filepath) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    ofs << "# End-to-End Delay Statistics\n\n";

    // Per-layer breakdown
    ofs << "# Per-Layer Delay Breakdown\n";
    ofs << "# Layer\tMin(ms)\tMax(ms)\tAvg(ms)\tP95(ms)\tSamples\n";

    for (const std::string layer : {"PHY", "MAC", "RLC", "PDCP", "SDAP"})
    {
        auto s = GetLayerStats (layer);
        if (s.sampleCount > 0)
        {
            ofs << layer << "\t" << s.minDelayMs << "\t" << s.maxDelayMs << "\t"
                << s.avgDelayMs << "\t" << s.p95DelayMs << "\t" << s.sampleCount << "\n";
        }
    }

    // Per-UE E2E
    ofs << "\n# Per-UE End-to-End Delay\n";
    ofs << "# RNTI\tMin(ms)\tMax(ms)\tAvg(ms)\tP95(ms)\tSamples\n";

    for (const auto& kv : m_e2eDelaySamples)
    {
        auto s = ComputeStats (kv.second);
        ofs << kv.first << "\t" << s.minDelayMs << "\t" << s.maxDelayMs << "\t"
            << s.avgDelayMs << "\t" << s.p95DelayMs << "\t" << s.sampleCount << "\n";
    }

    // Aggregate E2E
    auto agg = GetAggregateE2eStats ();
    ofs << "\n# Aggregate E2E: min=" << agg.minDelayMs << " max=" << agg.maxDelayMs
        << " avg=" << agg.avgDelayMs << " P95=" << agg.p95DelayMs
        << " samples=" << agg.sampleCount << "\n";

    ofs.close ();
    NS_LOG_INFO ("E2E stats exported to " << filepath);
}

void
SatE2eStatsCollector::Reset ()
{
    m_e2eDelaySamples.clear ();
    m_phyDelays.clear ();
    m_macDelays.clear ();
    m_rlcDelays.clear ();
    m_pdcpDelays.clear ();
    m_sdapDelays.clear ();
}

} // namespace ns3
