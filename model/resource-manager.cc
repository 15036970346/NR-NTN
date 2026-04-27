/*
 * 文件路径：contrib/geo-sat/model/resource-manager.cc
 * 功能：卫星无线资源管理模块实现 (统一 35MHz / 175 PRB / 单波束25 PRB 口径)
 */
#include "resource-manager.h"
#include "ns3/log.h"
#include "ns3/double.h"
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
  : m_maxEirpPortable (33.0),   // 对应 3 dBW 的便携式终端发射物理极限
    m_maxEirpConsumer (50.0)    // 对应 20 dBW 的大型终端发射物理极限
{ 
  NS_LOG_FUNCTION (this); 
}

ResourceManager::~ResourceManager () { NS_LOG_FUNCTION (this); }

// =================================================================
// 物理边界审查 (依据《NR NTN 3.23》核心指标)
// =================================================================
uint32_t ResourceManager::AllocateSpectrum (UtType utType, uint32_t requestedRbs, bool isUplink)
{
  NS_LOG_FUNCTION (this << utType << requestedRbs << isUplink);

  const uint32_t beamPrbLimit = 25;
  uint32_t hardwareLimit = beamPrbLimit;

  if (isUplink)
    {
      // 当前统一到每波束 5 MHz / 25 PRB，终端类别差异不再突破该单波束上限。
      hardwareLimit = beamPrbLimit;
    }
  else
    {
      hardwareLimit = beamPrbLimit;
    }

  // 核心逻辑：用户需求与物理天花板取最小值
  uint32_t approvedRbs = std::min(requestedRbs, hardwareLimit);

  NS_LOG_INFO ("RRM Spectrum Review: Requested=" << requestedRbs 
               << " | Direction=" << (isUplink ? "UL" : "DL")
               << " | Hardware Ceiling=" << hardwareLimit 
               << " ===> Approved=" << approvedRbs << " RBs");
               
  return approvedRbs;
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
