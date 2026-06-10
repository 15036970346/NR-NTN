/*
 * contrib/geo-sat/helper/sat-mac-stats-collector.cc
 * MAC layer statistics collector implementation
 */
#include "sat-mac-stats-collector.h"
#include "ns3/log.h"
#include <cmath>
#include <fstream>
#include <algorithm>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatMacStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatMacStatsCollector);

TypeId
SatMacStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatMacStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatMacStatsCollector> ();
    return tid;
}

SatMacStatsCollector::SatMacStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatMacStatsCollector::~SatMacStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

void
SatMacStatsCollector::OnQueueLength (uint16_t rnti, uint32_t bufferBytes)
{
    m_queueLengthSamples[rnti].push_back (static_cast<double> (bufferBytes));
}

void
SatMacStatsCollector::OnQueueDelay (uint16_t rnti, int64_t delayNs)
{
    double delayMs = static_cast<double> (delayNs) / 1e6;
    m_queueDelaySamples[rnti].push_back (delayMs);
}

void
SatMacStatsCollector::RecordMuxPacketSize (uint16_t rnti, uint32_t sizeBytes)
{
    m_muxPacketSamples[rnti].push_back (static_cast<double> (sizeBytes));
}

void
SatMacStatsCollector::RecordDemuxPacketSize (uint16_t rnti, uint32_t sizeBytes)
{
    m_demuxPacketSamples[rnti].push_back (static_cast<double> (sizeBytes));
}

MacStatsResult
SatMacStatsCollector::ComputeStats (const std::vector<double>& samples) const
{
    MacStatsResult result = {0.0, 0.0, 0.0, 0.0, 0};
    if (samples.empty ())
    {
        return result;
    }

    result.count = static_cast<uint32_t> (samples.size ());
    result.min = *std::min_element (samples.begin (), samples.end ());
    result.max = *std::max_element (samples.begin (), samples.end ());
    result.avg = std::accumulate (samples.begin (), samples.end (), 0.0) / result.count;

    // P95: sort a copy and pick the 95th percentile
    std::vector<double> sorted = samples;
    std::sort (sorted.begin (), sorted.end ());
    size_t p95Idx = static_cast<size_t> (std::ceil (0.95 * sorted.size ())) - 1;
    result.p95 = sorted[p95Idx];

    return result;
}

MacStatsResult
SatMacStatsCollector::GetQueueLengthStats (uint16_t rnti) const
{
    auto it = m_queueLengthSamples.find (rnti);
    if (it == m_queueLengthSamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

MacStatsResult
SatMacStatsCollector::GetQueueDelayStats (uint16_t rnti) const
{
    auto it = m_queueDelaySamples.find (rnti);
    if (it == m_queueDelaySamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

MacStatsResult
SatMacStatsCollector::GetMuxPacketStats (uint16_t rnti) const
{
    auto it = m_muxPacketSamples.find (rnti);
    if (it == m_muxPacketSamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

MacStatsResult
SatMacStatsCollector::GetDemuxPacketStats (uint16_t rnti) const
{
    auto it = m_demuxPacketSamples.find (rnti);
    if (it == m_demuxPacketSamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

MacStatsResult
SatMacStatsCollector::GetAggregateQueueDelayStats () const
{
    std::vector<double> all;
    for (const auto& kv : m_queueDelaySamples)
    {
        all.insert (all.end (), kv.second.begin (), kv.second.end ());
    }
    return ComputeStats (all);
}

MacStatsResult
SatMacStatsCollector::GetAggregateQueueLengthStats () const
{
    std::vector<double> all;
    for (const auto& kv : m_queueLengthSamples)
    {
        all.insert (all.end (), kv.second.begin (), kv.second.end ());
    }
    return ComputeStats (all);
}

void
SatMacStatsCollector::ExportToFile (const std::string& filepath) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    ofs << "# MAC Layer Statistics\n";
    ofs << "# RNTI\tQueueDelay_avg(ms)\tQueueDelay_p95(ms)\t"
           "QueueLen_avg(bytes)\tQueueLen_max(bytes)\t"
           "MuxPkt_avg(bytes)\tDemuxPkt_avg(bytes)\tSamples\n";

    std::set<uint16_t> allRntis;
    for (const auto& kv : m_queueDelaySamples) allRntis.insert (kv.first);
    for (const auto& kv : m_queueLengthSamples) allRntis.insert (kv.first);

    for (uint16_t rnti : allRntis)
    {
        auto delay = GetQueueDelayStats (rnti);
        auto len   = GetQueueLengthStats (rnti);
        auto mux   = GetMuxPacketStats (rnti);
        auto demux = GetDemuxPacketStats (rnti);

        ofs << rnti << "\t"
            << delay.avg << "\t" << delay.p95 << "\t"
            << len.avg << "\t" << len.max << "\t"
            << mux.avg << "\t" << demux.avg << "\t"
            << delay.count << "\n";
    }

    auto aggDelay = GetAggregateQueueDelayStats ();
    auto aggLen   = GetAggregateQueueLengthStats ();
    ofs << "\n# Aggregate\n";
    ofs << "# QueueDelay: avg=" << aggDelay.avg << " p95=" << aggDelay.p95
        << " max=" << aggDelay.max << "\n";
    ofs << "# QueueLength: avg=" << aggLen.avg << " max=" << aggLen.max << "\n";

    ofs.close ();
    NS_LOG_INFO ("MAC stats exported to " << filepath);
}

void
SatMacStatsCollector::Reset ()
{
    m_queueLengthSamples.clear ();
    m_queueDelaySamples.clear ();
    m_muxPacketSamples.clear ();
    m_demuxPacketSamples.clear ();
}

} // namespace ns3
