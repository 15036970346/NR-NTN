/*
 * 文件路径：contrib/geo-sat/model/resource-manager.cc
 * 功能：卫星无线资源管理模块实现 (统一 35MHz / 175 PRB / 单波束25 PRB 口径)
 */
#include "resource-manager.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ResourceManager");
NS_OBJECT_ENSURE_REGISTERED (ResourceManager);

TypeId ResourceManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ResourceManager")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<ResourceManager> ()
    .AddAttribute ("DlBeamBudgetRbs",
                   "Per-beam downlink RB budget under the current reuse configuration.",
                   UintegerValue (25),
                   MakeUintegerAccessor (&ResourceManager::m_dlBeamBudgetRbs),
                   MakeUintegerChecker<uint32_t> (1, 275))
    .AddAttribute ("UlBeamBudgetRbs",
                   "Per-beam uplink RB budget under the current reuse configuration.",
                   UintegerValue (25),
                   MakeUintegerAccessor (&ResourceManager::m_ulBeamBudgetRbs),
                   MakeUintegerChecker<uint32_t> (1, 275))
    .AddAttribute ("P0NominalPusch",
                   "Target received power P_0 at the satellite gNB (dBm).",
                   DoubleValue (-90.0),
                   MakeDoubleAccessor (&ResourceManager::m_p0NominalPusch),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("PathLossCompensationAlpha",
                   "Fractional path loss compensation factor.",
                   DoubleValue (0.8), 
                   MakeDoubleAccessor (&ResourceManager::m_alpha),
                   MakeDoubleChecker<double> ());
  return tid;
}

ResourceManager::ResourceManager ()
  : m_dlBeamBudgetRbs (25),
    m_ulBeamBudgetRbs (25),
    m_maxEirpPortable (63.0),   // 对应 33 dBW 的便携式终端发射物理极限
    m_maxEirpConsumer (50.0)    // 对应 20 dBW 的消费级手机发射物理极限
{ 
  NS_LOG_FUNCTION (this); 
}

ResourceManager::~ResourceManager () { NS_LOG_FUNCTION (this); }

// =================================================================
// 物理边界审查 (依据《NR NTN 3.23》核心指标) + beam 预算跟踪
// =================================================================
BeamAllocationUsage&
ResourceManager::GetOrCreateBeamUsage (uint32_t beamId)
{
  return m_beamUsageMap[beamId];
}

void
ResourceManager::ResetBeamAllocation (uint32_t beamId, bool isUplink)
{
  NS_LOG_FUNCTION (this << beamId << isUplink);

  BeamAllocationUsage& usage = GetOrCreateBeamUsage (beamId);
  if (isUplink)
    {
      usage.ulUsedRbs = 0;
    }
  else
    {
      usage.dlUsedRbs = 0;
    }
}

void
ResourceManager::ResetAllBeamAllocations (void)
{
  NS_LOG_FUNCTION (this);
  m_beamUsageMap.clear ();
}

uint32_t
ResourceManager::GetRemainingRbs (uint32_t beamId, bool isUplink) const
{
  const uint32_t budget = isUplink ? m_ulBeamBudgetRbs : m_dlBeamBudgetRbs;

  auto it = m_beamUsageMap.find (beamId);
  if (it == m_beamUsageMap.end ())
    {
      return budget;
    }

  const uint32_t used = isUplink ? it->second.ulUsedRbs : it->second.dlUsedRbs;
  return (used < budget) ? (budget - used) : 0;
}

uint32_t ResourceManager::AllocateSpectrum (UtType utType, uint32_t requestedRbs, bool isUplink)
{
  NS_LOG_FUNCTION (this << utType << requestedRbs << isUplink);

  const uint32_t beamPrbLimit = 25;

  // 核心逻辑：用户需求与物理天花板取最小值
  uint32_t approvedRbs = std::min(requestedRbs, beamPrbLimit);

  NS_LOG_INFO ("RRM Spectrum Review: Requested=" << requestedRbs
               << " | Direction=" << (isUplink ? "UL" : "DL")
               << " | Hardware Ceiling=" << beamPrbLimit
               << " ===> Approved=" << approvedRbs << " RBs");

  return approvedRbs;
}

uint32_t
ResourceManager::GetMaxPowerLimitedUlRbs (UtType utType, double pathLossDb) const
{
  NS_LOG_FUNCTION (this << utType << pathLossDb);

  const double pCmax = (utType == UT_PORTABLE) ? m_maxEirpPortable : m_maxEirpConsumer;
  const double rhsDb = pCmax - m_p0NominalPusch - m_alpha * pathLossDb;
  const double maxLinearRbs = std::pow (10.0, rhsDb / 10.0);

  if (maxLinearRbs < 1.0)
    {
      NS_LOG_INFO ("RRM UL Power Limit | PathLoss=" << pathLossDb
                   << " dB | P_CMAX=" << pCmax
                   << " dBm ===> No non-clipped UL RB feasible");
      return 0;
    }

  const uint32_t cappedRbs = static_cast<uint32_t> (std::floor (maxLinearRbs + 1e-9));
  NS_LOG_INFO ("RRM UL Power Limit | PathLoss=" << pathLossDb
               << " dB | P_CMAX=" << pCmax
               << " dBm ===> Max feasible non-clipped UL RBs=" << cappedRbs);
  return cappedRbs;
}

// =================================================================
// 3GPP PUSCH 上行功率控制
// =================================================================
double ResourceManager::AdjustUtTxPower (UtType utType, uint32_t allocatedRbs, double pathLossDb)
{
  NS_LOG_FUNCTION (this << utType << allocatedRbs << pathLossDb);

  if (allocatedRbs == 0) return 0.0;

  double pCmax = (utType == UT_PORTABLE) ? m_maxEirpPortable : m_maxEirpConsumer;
  double bandwidthCompensation = 10.0 * std::log10 (static_cast<double>(allocatedRbs));
  double pathLossCompensation = m_alpha * pathLossDb;
  double targetPowerDbm = m_p0NominalPusch + bandwidthCompensation + pathLossCompensation;

  double actualTxPowerDbm = std::min (pCmax, targetPowerDbm);

  NS_LOG_INFO ("RRM Power Control | Allocated RBs=" << allocatedRbs
               << " | PathLoss=" << pathLossDb << " dB"
               << " | P_CMAX=" << pCmax << " dBm"
               << " ===> Final TX Power=" << actualTxPowerDbm << " dBm");

  return actualTxPowerDbm;
}

} // namespace ns3
