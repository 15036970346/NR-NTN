/*
 * 文件路径：contrib/geo-sat/model/resource-manager.cc
 * 功能：卫星无线资源管理模块实现 (S频段专版)
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
    // 支持 S 频段 2GHz 工作频率
    .AddAttribute ("OperatingFrequency",
                   "Operating frequency for S-band in Hz.",
                   DoubleValue (2.0e9), 
                   MakeDoubleAccessor (&ResourceManager::m_operatingFrequency),
                   MakeDoubleChecker<double> ());
  return tid;
}

ResourceManager::ResourceManager () 
  : m_minEirpDbw (3.0),   // 表格限定：下限 3 dBW
    m_maxEirpDbw (20.0)   // 表格限定：上限 20 dBW
{ 
  NS_LOG_FUNCTION (this); 
}

ResourceManager::~ResourceManager () { NS_LOG_FUNCTION (this); }

// =================================================================
// 功能 1：动态分配频谱资源块 (AllocateSpectrum)
// =================================================================
uint32_t ResourceManager::AllocateSpectrum (UtType utType, TrafficType trafficType, double cqi, bool isUplink)
{
  NS_LOG_FUNCTION (this << utType << trafficType << cqi << isUplink);

  uint32_t baseRbs = 0;
  uint32_t allocatedRbs = 0;

  // 1. 根据业务类型设定基准 RB 需求
  if (isUplink) {
      if (trafficType == TRAFFIC_VOICE)            baseRbs = 10;
      else if (trafficType == TRAFFIC_DATA)        baseRbs = 25;
      else if (trafficType == TRAFFIC_HIGH_CAPACITY) baseRbs = 40;
  } else { // 下行带宽通常更宽
      if (trafficType == TRAFFIC_VOICE)            baseRbs = 80;
      else if (trafficType == TRAFFIC_DATA)        baseRbs = 120;
      else if (trafficType == TRAFFIC_HIGH_CAPACITY) baseRbs = 150;
  }

  // 2. 根据终端类型微调 (消费级终端通常处理能力更强，可并发处理更多RB)
  double typeMultiplier = (utType == UT_CONSUMER) ? 1.1 : 1.0;

  // 3. 根据信道质量 (CQI 1~15) 动态补偿
  // CQI 越低，信道越差，为了维持业务速率，需要分配更多的 RB 资源
  double cqiFactor = 1.0 + ((15.0 - std::max(1.0, cqi)) * 0.05); 

  // 计算初步需要的 RB 数量
  allocatedRbs = std::round (baseRbs * typeMultiplier * cqiFactor);

  // 4. 严格执行表格中的硬性边界限制！
  if (isUplink) {
      // 上行：6 ~ 50 RB
      allocatedRbs = std::clamp(allocatedRbs, (uint32_t)6, (uint32_t)50);
  } else {
      // 下行：75 ~ 160 RB
      allocatedRbs = std::clamp(allocatedRbs, (uint32_t)75, (uint32_t)160);
  }

  NS_LOG_INFO ("Allocated " << allocatedRbs << " RBs for " 
               << (isUplink ? "UPLINK" : "DOWNLINK") 
               << " (CQI: " << cqi << ")");
               
  return allocatedRbs;
}

// =================================================================
// 功能 2：动态调整终端发射功率 (AdjustUtTxPower)
// =================================================================
double ResourceManager::AdjustUtTxPower (UtType utType, double currentUplinkSnr, uint8_t targetMcs)
{
  NS_LOG_FUNCTION (this << utType << currentUplinkSnr << (uint16_t)targetMcs);

  // 1. 确定当前终端的 EIRP 物理上限 (dBW 转为 dBm 方便计算：dBm = dBW + 30)
  // 便携式按 3 dBW (33 dBm) 算，消费级按 20 dBW (50 dBm) 算
  double maxEirpDbm = (utType == UT_PORTABLE) ? (m_minEirpDbw + 30.0) : (m_maxEirpDbw + 30.0);

  // 2. 计算目标 MCS 对应的期望 SNR (简单的线性映射估算，MCS越高，需要的SNR越高)
  // 假设 MCS 0 需要 -2dB SNR，MCS 28 需要 22dB SNR
  double targetSnr = -2.0 + (targetMcs * 0.85);

  // 3. 计算 SNR 差距 (如果 currentUplinkSnr 低于 targetSnr，说明功率不够)
  double snrGap = targetSnr - currentUplinkSnr;

  // 4. 闭环功率调整：在原有基准功率上加上缺少的 SNR
  // 假设当前时刻基准发射功率为 23 dBm
  double currentTxPowerDbm = 23.0; 
  double adjustedTxPowerDbm = currentTxPowerDbm + snrGap;

  // 5. 严格限制在终端 EIRP 能力范围内
  // 最小不能低于 0 dBm，最大不能超过该终端的物理上限
  adjustedTxPowerDbm = std::clamp(adjustedTxPowerDbm, 0.0, maxEirpDbm);

  NS_LOG_INFO ("Adjusted TX Power: " << adjustedTxPowerDbm << " dBm "
               << "(Target SNR: " << targetSnr << "dB, Current SNR: " << currentUplinkSnr 
               << "dB, Max EIRP: " << maxEirpDbm << " dBm)");

  return adjustedTxPowerDbm;
}

} // namespace ns3