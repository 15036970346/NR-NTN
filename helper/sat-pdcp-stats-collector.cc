/*
 * contrib/geo-sat/helper/sat-pdcp-stats-collector.cc
 * PDCP layer statistics collector implementation
 */
#include "sat-pdcp-stats-collector.h"
#include "ns3/log.h"
#include <fstream>
#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatPdcpStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatPdcpStatsCollector);

TypeId
SatPdcpStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatPdcpStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatPdcpStatsCollector> ();
    return tid;
}

SatPdcpStatsCollector::SatPdcpStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatPdcpStatsCollector::~SatPdcpStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

ImsiLcidPair
SatPdcpStatsCollector::MakeKey (uint16_t rnti, uint8_t lcid) const
{
    auto it = m_rntiToImsi.find (rnti);
    uint64_t imsi = (it != m_rntiToImsi.end ()) ? it->second : static_cast<uint64_t> (rnti);
    return ImsiLcidPair (imsi, lcid);
}

void
SatPdcpStatsCollector::SetImsiForRnti (uint16_t rnti, uint64_t imsi)
{
    m_rntiToImsi[rnti] = imsi;
}

void
SatPdcpStatsCollector::DlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size)
{
    auto key = MakeKey (rnti, lcid);
    auto& acc = m_dlStats[key];
    acc.txBytes += size;
    acc.txPduCount++;
}

void
SatPdcpStatsCollector::DlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs)
{
    auto key = MakeKey (rnti, lcid);
    auto& acc = m_dlStats[key];
    acc.rxBytes += size;
    acc.rxPduCount++;
    double delayMs = static_cast<double> (delayNs) / 1e6;
    acc.totalDelayMs += delayMs;
    if (delayMs > acc.maxDelayMs)
    {
        acc.maxDelayMs = delayMs;
    }
    acc.delaySamples.push_back (delayMs);
}

void
SatPdcpStatsCollector::UlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size)
{
    auto key = MakeKey (rnti, lcid);
    auto& acc = m_ulStats[key];
    acc.txBytes += size;
    acc.txPduCount++;
}

void
SatPdcpStatsCollector::UlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs)
{
    auto key = MakeKey (rnti, lcid);
    auto& acc = m_ulStats[key];
    acc.rxBytes += size;
    acc.rxPduCount++;
    double delayMs = static_cast<double> (delayNs) / 1e6;
    acc.totalDelayMs += delayMs;
    if (delayMs > acc.maxDelayMs)
    {
        acc.maxDelayMs = delayMs;
    }
    acc.delaySamples.push_back (delayMs);
}

void
SatPdcpStatsCollector::OnRohcCompression (uint16_t rnti, uint8_t lcid,
                                           uint32_t origSize, uint32_t compressedSize)
{
    auto key = MakeKey (rnti, lcid);
    auto& racc = m_rohcStats[key];
    double ratio = (origSize > 0) ? static_cast<double> (compressedSize) / origSize : 1.0;
    racc.totalRatio += ratio;
    racc.sampleCount++;
}

void
SatPdcpStatsCollector::RecordIncompletePacket (uint16_t rnti, uint8_t lcid)
{
    auto key = MakeKey (rnti, lcid);
    m_rohcStats[key].incompletePackets++;
}

double
SatPdcpStatsCollector::ComputeP95 (const std::vector<double>& samples) const
{
    if (samples.empty ()) return 0.0;
    std::vector<double> sorted = samples;
    std::sort (sorted.begin (), sorted.end ());
    size_t idx = static_cast<size_t> (std::ceil (0.95 * sorted.size ())) - 1;
    return sorted[idx];
}

PdcpBearerStats
SatPdcpStatsCollector::GetDlBearerStats (uint64_t imsi, uint8_t lcid) const
{
    PdcpBearerStats result = {};
    auto key = ImsiLcidPair (imsi, lcid);

    auto it = m_dlStats.find (key);
    if (it != m_dlStats.end ())
    {
        const auto& acc = it->second;
        result.txBytes = acc.txBytes;
        result.rxBytes = acc.rxBytes;
        result.txPduCount = acc.txPduCount;
        result.rxPduCount = acc.rxPduCount;
        result.avgDelayMs = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        result.maxDelayMs = acc.maxDelayMs;
        result.p95DelayMs = ComputeP95 (acc.delaySamples);
    }

    auto rit = m_rohcStats.find (key);
    if (rit != m_rohcStats.end ())
    {
        const auto& racc = rit->second;
        result.avgRohcRatio = (racc.sampleCount > 0) ? racc.totalRatio / racc.sampleCount : 1.0;
        result.rohcSampleCount = racc.sampleCount;
        result.incompletePackets = racc.incompletePackets;
    }
    return result;
}

PdcpBearerStats
SatPdcpStatsCollector::GetUlBearerStats (uint64_t imsi, uint8_t lcid) const
{
    PdcpBearerStats result = {};
    auto key = ImsiLcidPair (imsi, lcid);

    auto it = m_ulStats.find (key);
    if (it != m_ulStats.end ())
    {
        const auto& acc = it->second;
        result.txBytes = acc.txBytes;
        result.rxBytes = acc.rxBytes;
        result.txPduCount = acc.txPduCount;
        result.rxPduCount = acc.rxPduCount;
        result.avgDelayMs = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        result.maxDelayMs = acc.maxDelayMs;
        result.p95DelayMs = ComputeP95 (acc.delaySamples);
    }
    return result;
}

double
SatPdcpStatsCollector::GetAggregateRohcRatio () const
{
    double totalRatio = 0.0;
    uint32_t totalCount = 0;
    for (const auto& kv : m_rohcStats)
    {
        totalRatio += kv.second.totalRatio;
        totalCount += kv.second.sampleCount;
    }
    return (totalCount > 0) ? totalRatio / totalCount : 1.0;
}

void
SatPdcpStatsCollector::ExportToFile (const std::string& filepath) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    ofs << "# PDCP Layer Statistics\n";
    ofs << "# Dir\tIMSI\tLCID\tTxBytes\tRxBytes\tTxPDUs\tRxPDUs\t"
           "AvgDelay(ms)\tMaxDelay(ms)\tP95Delay(ms)\t"
           "ROHC_ratio\tROHC_samples\tIncomplete\n";

    for (const auto& kv : m_dlStats)
    {
        const auto& acc = kv.second;
        double avgDelay = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        double p95 = ComputeP95 (acc.delaySamples);

        double rohcRatio = 1.0;
        uint32_t rohcN = 0, incomplete = 0;
        auto rit = m_rohcStats.find (kv.first);
        if (rit != m_rohcStats.end ())
        {
            rohcRatio = (rit->second.sampleCount > 0)
                            ? rit->second.totalRatio / rit->second.sampleCount : 1.0;
            rohcN = rit->second.sampleCount;
            incomplete = rit->second.incompletePackets;
        }

        ofs << "DL\t" << kv.first.first << "\t" << (uint32_t)kv.first.second << "\t"
            << acc.txBytes << "\t" << acc.rxBytes << "\t"
            << acc.txPduCount << "\t" << acc.rxPduCount << "\t"
            << avgDelay << "\t" << acc.maxDelayMs << "\t" << p95 << "\t"
            << rohcRatio << "\t" << rohcN << "\t" << incomplete << "\n";
    }

    for (const auto& kv : m_ulStats)
    {
        const auto& acc = kv.second;
        double avgDelay = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        double p95 = ComputeP95 (acc.delaySamples);

        ofs << "UL\t" << kv.first.first << "\t" << (uint32_t)kv.first.second << "\t"
            << acc.txBytes << "\t" << acc.rxBytes << "\t"
            << acc.txPduCount << "\t" << acc.rxPduCount << "\t"
            << avgDelay << "\t" << acc.maxDelayMs << "\t" << p95 << "\t"
            << "N/A\t0\t0\n";
    }

    ofs << "\n# Aggregate ROHC compression ratio: " << GetAggregateRohcRatio () << "\n";

    ofs.close ();
    NS_LOG_INFO ("PDCP stats exported to " << filepath);
}

void
SatPdcpStatsCollector::Reset ()
{
    m_dlStats.clear ();
    m_ulStats.clear ();
    m_rohcStats.clear ();
}

} // namespace ns3
