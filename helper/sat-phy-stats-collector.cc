/*
 * contrib/geo-sat/helper/sat-phy-stats-collector.cc
 * PHY layer statistics collector implementation
 */
#include "sat-phy-stats-collector.h"
#include "ns3/log.h"
#include <cmath>
#include <fstream>
#include <algorithm>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatPhyStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatPhyStatsCollector);

TypeId
SatPhyStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatPhyStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatPhyStatsCollector> ();
    return tid;
}

SatPhyStatsCollector::SatPhyStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatPhyStatsCollector::~SatPhyStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

void
SatPhyStatsCollector::OnRsrp (uint16_t rnti, double rsrpDbm)
{
    m_rsrpSamples[rnti].push_back (rsrpDbm);
}

void
SatPhyStatsCollector::OnSinr (uint16_t rnti, double sinrDb)
{
    m_sinrSamples[rnti].push_back (sinrDb);
}

void
SatPhyStatsCollector::OnRsrq (uint16_t rnti, double rsrqDb)
{
    m_rsrqSamples[rnti].push_back (rsrqDb);
}

void
SatPhyStatsCollector::RecordPropDelay (uint16_t rnti, Time oneWayDelay)
{
    m_propDelaySamples[rnti].push_back (oneWayDelay.GetMilliSeconds ());
}

PhyStatsResult
SatPhyStatsCollector::ComputeStats (const std::vector<double>& samples) const
{
    PhyStatsResult result = {0.0, 0.0, 0.0, 0.0, 0};
    if (samples.empty ())
    {
        return result;
    }

    result.count = static_cast<uint32_t> (samples.size ());
    result.min = *std::min_element (samples.begin (), samples.end ());
    result.max = *std::max_element (samples.begin (), samples.end ());
    result.avg = std::accumulate (samples.begin (), samples.end (), 0.0) / result.count;

    double sumSq = 0.0;
    for (double v : samples)
    {
        double diff = v - result.avg;
        sumSq += diff * diff;
    }
    result.stddev = std::sqrt (sumSq / result.count);

    return result;
}

PhyStatsResult
SatPhyStatsCollector::GetRsrpStats (uint16_t rnti) const
{
    auto it = m_rsrpSamples.find (rnti);
    if (it == m_rsrpSamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

PhyStatsResult
SatPhyStatsCollector::GetSinrStats (uint16_t rnti) const
{
    auto it = m_sinrSamples.find (rnti);
    if (it == m_sinrSamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

PhyStatsResult
SatPhyStatsCollector::GetRsrqStats (uint16_t rnti) const
{
    auto it = m_rsrqSamples.find (rnti);
    if (it == m_rsrqSamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

PhyStatsResult
SatPhyStatsCollector::GetPropDelayStats (uint16_t rnti) const
{
    auto it = m_propDelaySamples.find (rnti);
    if (it == m_propDelaySamples.end ())
    {
        return {0, 0, 0, 0, 0};
    }
    return ComputeStats (it->second);
}

PhyStatsResult
SatPhyStatsCollector::GetAggregateRsrpStats () const
{
    std::vector<double> all;
    for (const auto& kv : m_rsrpSamples)
    {
        all.insert (all.end (), kv.second.begin (), kv.second.end ());
    }
    return ComputeStats (all);
}

PhyStatsResult
SatPhyStatsCollector::GetAggregateSinrStats () const
{
    std::vector<double> all;
    for (const auto& kv : m_sinrSamples)
    {
        all.insert (all.end (), kv.second.begin (), kv.second.end ());
    }
    return ComputeStats (all);
}

PhyStatsResult
SatPhyStatsCollector::GetAggregateRsrqStats () const
{
    std::vector<double> all;
    for (const auto& kv : m_rsrqSamples)
    {
        all.insert (all.end (), kv.second.begin (), kv.second.end ());
    }
    return ComputeStats (all);
}

void
SatPhyStatsCollector::ExportToFile (const std::string& filepath) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    ofs << "# PHY Layer Statistics\n";
    ofs << "# RNTI\tRSRP_avg(dBm)\tRSRP_min\tRSRP_max\t"
           "SINR_avg(dB)\tSINR_min\tSINR_max\t"
           "RSRQ_avg(dB)\tRSRQ_min\tRSRQ_max\t"
           "PropDelay_avg(ms)\tSamples\n";

    // Collect all RNTIs
    std::set<uint16_t> allRntis;
    for (const auto& kv : m_rsrpSamples) allRntis.insert (kv.first);
    for (const auto& kv : m_sinrSamples) allRntis.insert (kv.first);

    for (uint16_t rnti : allRntis)
    {
        auto rsrp = GetRsrpStats (rnti);
        auto sinr = GetSinrStats (rnti);
        auto rsrq = GetRsrqStats (rnti);
        auto prop = GetPropDelayStats (rnti);

        ofs << rnti << "\t"
            << rsrp.avg << "\t" << rsrp.min << "\t" << rsrp.max << "\t"
            << sinr.avg << "\t" << sinr.min << "\t" << sinr.max << "\t"
            << rsrq.avg << "\t" << rsrq.min << "\t" << rsrq.max << "\t"
            << prop.avg << "\t" << rsrp.count << "\n";
    }

    // Aggregate
    auto aggRsrp = GetAggregateRsrpStats ();
    auto aggSinr = GetAggregateSinrStats ();
    auto aggRsrq = GetAggregateRsrqStats ();
    ofs << "\n# Aggregate (all UEs)\n";
    ofs << "# RSRP: avg=" << aggRsrp.avg << " min=" << aggRsrp.min
        << " max=" << aggRsrp.max << " stddev=" << aggRsrp.stddev << "\n";
    ofs << "# SINR: avg=" << aggSinr.avg << " min=" << aggSinr.min
        << " max=" << aggSinr.max << " stddev=" << aggSinr.stddev << "\n";
    ofs << "# RSRQ: avg=" << aggRsrq.avg << " min=" << aggRsrq.min
        << " max=" << aggRsrq.max << " stddev=" << aggRsrq.stddev << "\n";

    ofs.close ();
    NS_LOG_INFO ("PHY stats exported to " << filepath);
}

void
SatPhyStatsCollector::Reset ()
{
    m_rsrpSamples.clear ();
    m_sinrSamples.clear ();
    m_rsrqSamples.clear ();
    m_propDelaySamples.clear ();
}

} // namespace ns3
