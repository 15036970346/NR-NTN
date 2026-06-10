/*
 * contrib/geo-sat/helper/sat-rlc-stats-collector.cc
 * RLC layer statistics collector implementation
 */
#include "sat-rlc-stats-collector.h"
#include "ns3/log.h"
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatRlcStatsCollector");
NS_OBJECT_ENSURE_REGISTERED (SatRlcStatsCollector);

TypeId
SatRlcStatsCollector::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::SatRlcStatsCollector")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatRlcStatsCollector> ();
    return tid;
}

SatRlcStatsCollector::SatRlcStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

SatRlcStatsCollector::~SatRlcStatsCollector ()
{
    NS_LOG_FUNCTION (this);
}

ImsiLcidPair
SatRlcStatsCollector::MakeKey (uint16_t rnti, uint8_t lcid) const
{
    auto it = m_rntiToImsi.find (rnti);
    uint64_t imsi = (it != m_rntiToImsi.end ()) ? it->second : static_cast<uint64_t> (rnti);
    return ImsiLcidPair (imsi, lcid);
}

void
SatRlcStatsCollector::SetImsiForRnti (uint16_t rnti, uint64_t imsi)
{
    m_rntiToImsi[rnti] = imsi;
}

void
SatRlcStatsCollector::SetRlcMode (uint64_t imsi, uint8_t lcid, const std::string& mode)
{
    m_rlcModeMap[ImsiLcidPair (imsi, lcid)] = mode;
}

void
SatRlcStatsCollector::DlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size)
{
    auto key = MakeKey (rnti, lcid);
    auto& acc = m_dlStats[key];
    acc.txBytes += size;
    acc.txPduCount++;
    if (m_rlcModeMap.count (key))
    {
        acc.rlcMode = m_rlcModeMap[key];
    }
}

void
SatRlcStatsCollector::DlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs)
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
}

void
SatRlcStatsCollector::UlTxPdu (uint16_t rnti, uint8_t lcid, uint32_t size)
{
    auto key = MakeKey (rnti, lcid);
    auto& acc = m_ulStats[key];
    acc.txBytes += size;
    acc.txPduCount++;
    if (m_rlcModeMap.count (key))
    {
        acc.rlcMode = m_rlcModeMap[key];
    }
}

void
SatRlcStatsCollector::UlRxPdu (uint16_t rnti, uint8_t lcid, uint32_t size, uint64_t delayNs)
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
}

RlcBearerStats
SatRlcStatsCollector::GetDlBearerStats (uint64_t imsi, uint8_t lcid) const
{
    RlcBearerStats result = {0, 0, 0, 0, 0.0, 0.0, "UM"};
    auto it = m_dlStats.find (ImsiLcidPair (imsi, lcid));
    if (it != m_dlStats.end ())
    {
        const auto& acc = it->second;
        result.txBytes = acc.txBytes;
        result.rxBytes = acc.rxBytes;
        result.txPduCount = acc.txPduCount;
        result.rxPduCount = acc.rxPduCount;
        result.avgDelayMs = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        result.maxDelayMs = acc.maxDelayMs;
        result.rlcMode = acc.rlcMode;
    }
    return result;
}

RlcBearerStats
SatRlcStatsCollector::GetUlBearerStats (uint64_t imsi, uint8_t lcid) const
{
    RlcBearerStats result = {0, 0, 0, 0, 0.0, 0.0, "UM"};
    auto it = m_ulStats.find (ImsiLcidPair (imsi, lcid));
    if (it != m_ulStats.end ())
    {
        const auto& acc = it->second;
        result.txBytes = acc.txBytes;
        result.rxBytes = acc.rxBytes;
        result.txPduCount = acc.txPduCount;
        result.rxPduCount = acc.rxPduCount;
        result.avgDelayMs = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        result.maxDelayMs = acc.maxDelayMs;
        result.rlcMode = acc.rlcMode;
    }
    return result;
}

uint32_t
SatRlcStatsCollector::GetEstimatedRetxCount (uint64_t imsi, uint8_t lcid) const
{
    auto key = ImsiLcidPair (imsi, lcid);
    uint32_t dlRetx = 0, ulRetx = 0;

    auto dit = m_dlStats.find (key);
    if (dit != m_dlStats.end () && dit->second.txPduCount > dit->second.rxPduCount)
    {
        dlRetx = dit->second.txPduCount - dit->second.rxPduCount;
    }
    auto uit = m_ulStats.find (key);
    if (uit != m_ulStats.end () && uit->second.txPduCount > uit->second.rxPduCount)
    {
        ulRetx = uit->second.txPduCount - uit->second.rxPduCount;
    }
    return dlRetx + ulRetx;
}

double
SatRlcStatsCollector::GetModeThroughput (const std::string& mode, Time duration,
                                          bool downlink) const
{
    uint64_t totalBytes = 0;
    const auto& statsMap = downlink ? m_dlStats : m_ulStats;

    for (const auto& kv : statsMap)
    {
        if (kv.second.rlcMode == mode)
        {
            totalBytes += kv.second.rxBytes;
        }
    }
    double durationSec = duration.GetSeconds ();
    return (durationSec > 0) ? (totalBytes * 8.0 / durationSec) : 0.0;
}

void
SatRlcStatsCollector::ExportToFile (const std::string& filepath, Time simDuration) const
{
    std::ofstream ofs (filepath);
    if (!ofs.is_open ())
    {
        NS_LOG_ERROR ("Cannot open file: " << filepath);
        return;
    }

    ofs << "# RLC Layer Statistics\n";
    ofs << "# Direction\tIMSI\tLCID\tMode\tTxBytes\tRxBytes\tTxPDUs\tRxPDUs\t"
           "AvgDelay(ms)\tMaxDelay(ms)\tEstRetx\n";

    for (const auto& kv : m_dlStats)
    {
        const auto& acc = kv.second;
        double avgDelay = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        uint32_t retx = (acc.txPduCount > acc.rxPduCount) ? (acc.txPduCount - acc.rxPduCount) : 0;
        ofs << "DL\t" << kv.first.first << "\t" << (uint32_t)kv.first.second << "\t"
            << acc.rlcMode << "\t" << acc.txBytes << "\t" << acc.rxBytes << "\t"
            << acc.txPduCount << "\t" << acc.rxPduCount << "\t"
            << avgDelay << "\t" << acc.maxDelayMs << "\t" << retx << "\n";
    }
    for (const auto& kv : m_ulStats)
    {
        const auto& acc = kv.second;
        double avgDelay = (acc.rxPduCount > 0) ? acc.totalDelayMs / acc.rxPduCount : 0.0;
        uint32_t retx = (acc.txPduCount > acc.rxPduCount) ? (acc.txPduCount - acc.rxPduCount) : 0;
        ofs << "UL\t" << kv.first.first << "\t" << (uint32_t)kv.first.second << "\t"
            << acc.rlcMode << "\t" << acc.txBytes << "\t" << acc.rxBytes << "\t"
            << acc.txPduCount << "\t" << acc.rxPduCount << "\t"
            << avgDelay << "\t" << acc.maxDelayMs << "\t" << retx << "\n";
    }

    // Per-mode throughput summary
    double durSec = simDuration.GetSeconds ();
    if (durSec > 0)
    {
        ofs << "\n# Per-mode throughput (bps)\n";
        for (const std::string mode : {"AM", "UM", "TM"})
        {
            double dlTput = GetModeThroughput (mode, simDuration, true);
            double ulTput = GetModeThroughput (mode, simDuration, false);
            if (dlTput > 0 || ulTput > 0)
            {
                ofs << "# " << mode << ": DL=" << dlTput << " UL=" << ulTput << "\n";
            }
        }
    }

    ofs.close ();
    NS_LOG_INFO ("RLC stats exported to " << filepath);
}

void
SatRlcStatsCollector::Reset ()
{
    m_dlStats.clear ();
    m_ulStats.clear ();
    m_rlcModeMap.clear ();
}

} // namespace ns3
