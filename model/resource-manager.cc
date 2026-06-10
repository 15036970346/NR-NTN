/*
 * 文件路径：contrib/geo-sat/model/resource-manager.cc
 * 功能：卫星无线资源管理模块实现 
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
                   DoubleValue (-110.0),
                   MakeDoubleAccessor (&ResourceManager::m_p0NominalPusch),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("PathLossCompensationAlpha",
                   "Fractional path loss compensation factor.",
                   DoubleValue (0.8),
                   MakeDoubleAccessor (&ResourceManager::m_alpha),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BeamPowerBudgetDbm",
                   "Per-beam total downlink power budget (dBm) shared across scheduled UEs.",
                   DoubleValue (50.0),
                   MakeDoubleAccessor (&ResourceManager::m_beamPowerBudgetDbm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("PowerAllocPolicy",
                   "Downlink power allocation policy: 0=equal, 1=channel-inverse (fairness), "
                   "2=water-filling (capacity).",
                   UintegerValue (POWER_EQUAL),
                   MakeUintegerAccessor (&ResourceManager::m_powerAllocPolicy),
                   MakeUintegerChecker<uint32_t> (0, 2));
  return tid;
}

ResourceManager::ResourceManager () 
  : m_dlBeamBudgetRbs (25),
    m_ulBeamBudgetRbs (25),
    m_maxEirpPortable (63.0),   // 对应 33 dBW 的便携式终端发射物理极限
    m_maxEirpConsumer (50.0),   // 对应 20 dBW 的消费级手机发射物理极限
    m_beamPowerBudgetDbm (50.0),
    m_powerAllocPolicy (POWER_EQUAL)
{
  NS_LOG_FUNCTION (this); 
}

ResourceManager::~ResourceManager () { NS_LOG_FUNCTION (this); }

// =================================================================
// 物理边界审查
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
      usage.ulSharedRbs = 0;
    }
  else
    {
      usage.dlUsedRbs = 0;
      usage.dlSharedRbs = 0;
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

uint32_t
ResourceManager::AllocateSpectrum (uint32_t beamId, uint32_t requestedRbs, bool isUplink)
{
  NS_LOG_FUNCTION (this << beamId << requestedRbs << isUplink);

  BeamAllocationUsage& usage = GetOrCreateBeamUsage (beamId);
  const uint32_t remainingRbs = GetRemainingRbs (beamId, isUplink);
  const uint32_t approvedRbs = std::min (requestedRbs, remainingRbs);

  if (isUplink) 
    {
      usage.ulUsedRbs += approvedRbs;
      NS_LOG_INFO ("RRM Spectrum Review: Requested=" << requestedRbs
                    << " | Direction=UL"
                    << " | Beam=" << beamId
                    << " | RemainingBefore=" << remainingRbs
                    << " | UsedAfter=" << usage.ulUsedRbs
                    << " ===> Approved=" << approvedRbs << " RBs");
    } 
  else 
    {
      usage.dlUsedRbs += approvedRbs;
      NS_LOG_INFO ("RRM Spectrum Review: Requested=" << requestedRbs
                    << " | Direction=DL"
                    << " | Beam=" << beamId
                    << " | RemainingBefore=" << remainingRbs
                    << " | UsedAfter=" << usage.dlUsedRbs
                    << " ===> Approved=" << approvedRbs << " RBs");
    }
               
  return approvedRbs;
}

uint32_t
ResourceManager::AllocateSharedSpectrum (uint32_t beamId, uint32_t requestedRbs, bool isUplink)
{
  NS_LOG_FUNCTION (this << beamId << requestedRbs << isUplink);
  // NOMA 叠加: 这些 RB 必须已被主用户占用, 因此不能超过当前"已用"的 RB 数。
  BeamAllocationUsage& usage = GetOrCreateBeamUsage (beamId);
  const uint32_t usedRbs = isUplink ? usage.ulUsedRbs : usage.dlUsedRbs;
  const uint32_t approvedRbs = std::min (requestedRbs, usedRbs);
  if (isUplink)
    {
      usage.ulSharedRbs += approvedRbs;
    }
  else
    {
      usage.dlSharedRbs += approvedRbs;
    }
  NS_LOG_INFO ("RRM NOMA Overlay: Requested=" << requestedRbs
               << " | Direction=" << (isUplink ? "UL" : "DL")
               << " | Beam=" << beamId
               << " | OnTopOfUsed=" << usedRbs
               << " ===> SharedApproved=" << approvedRbs << " RBs");
  return approvedRbs;
}

uint32_t
ResourceManager::GetSharedRbs (uint32_t beamId, bool isUplink) const
{
  auto it = m_beamUsageMap.find (beamId);
  if (it == m_beamUsageMap.end ())
    {
      return 0;
    }
  return isUplink ? it->second.ulSharedRbs : it->second.dlSharedRbs;
}

uint32_t
ResourceManager::GetUsedRbs (uint32_t beamId, bool isUplink) const
{
  auto it = m_beamUsageMap.find (beamId);
  if (it == m_beamUsageMap.end ())
    {
      return 0;
    }
  return isUplink ? it->second.ulUsedRbs : it->second.dlUsedRbs;
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
double ResourceManager::AdjustUtTxPower (UtType utType, uint32_t allocatedRbs, double pathLossDb,
                                         double closedLoopOffsetDb)
{
  NS_LOG_FUNCTION (this << utType << allocatedRbs << pathLossDb << closedLoopOffsetDb);

  if (allocatedRbs == 0) return 0.0;

  double pCmax = (utType == UT_PORTABLE) ? m_maxEirpPortable : m_maxEirpConsumer;
  double bandwidthCompensation = 10.0 * std::log10 (static_cast<double>(allocatedRbs));
  double pathLossCompensation = m_alpha * pathLossDb;
  // 3GPP PUSCH 功控: P = P0 + 10log10(M) + alpha*PL + f(i)
  //  其中 f(i)=closedLoopOffsetDb 为基于实测 SINR 反馈累积的闭环修正项。
  double targetPowerDbm =
    m_p0NominalPusch + bandwidthCompensation + pathLossCompensation + closedLoopOffsetDb;

  double actualTxPowerDbm = std::min (pCmax, targetPowerDbm);

  NS_LOG_INFO ("RRM Power Control | Allocated RBs=" << allocatedRbs
               << " | PathLoss=" << pathLossDb << " dB"
               << " | f(i)=" << closedLoopOffsetDb << " dB"
               << " | P_CMAX=" << pCmax << " dBm"
               << " ===> Final TX Power=" << actualTxPowerDbm << " dBm");

  return actualTxPowerDbm;
}

// =================================================================
// 动态功率调配 (下行波束总功率约束)
// =================================================================
double
ResourceManager::GetPowerWeight (double cqi) const
{
  const double safeCqi = std::max (1.0, cqi);
  switch (m_powerAllocPolicy)
    {
    case POWER_CHANNEL_INVERSE:
      return 1.0 / safeCqi;   // 弱信道(低CQI)权重大 -> 多给功率, 拉公平
    case POWER_WATER_FILLING:
      return safeCqi;         // 强信道(高CQI)权重大 -> 多给功率, 拉容量(简化注水)
    case POWER_EQUAL:
    default:
      return 1.0;             // 等功率
    }
}

double
ResourceManager::ComputeDlTxPowerDbm (double ueWeight, double sumWeights) const
{
  if (sumWeights <= 0.0 || ueWeight <= 0.0)
    {
      return 0.0;
    }
  // 把总功率预算(dBm)转线性(mW), 按权重比例切给该 UE, 再转回 dBm。
  // 各 UE 份额之和 <= 预算, 因此自动满足"不超总功率"约束。
  const double budgetLinear = std::pow (10.0, m_beamPowerBudgetDbm / 10.0);
  const double share = budgetLinear * (ueWeight / sumWeights);
  return 10.0 * std::log10 (std::max (share, 1e-9));
}

} // namespace ns3
