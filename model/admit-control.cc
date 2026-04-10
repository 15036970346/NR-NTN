/*
 * 文件路径：contrib/geo-sat/model/admit-control.cc
 * 功能：GEO卫星准入控制与波束协调模块实现
 */
#include "admit-control.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("AdmitControl");
NS_OBJECT_ENSURE_REGISTERED (AdmitControl);

TypeId AdmitControl::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::AdmitControl")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<AdmitControl> ()
    .AddAttribute ("EmergencyReservationRatio",
                   "Reserved RBs ratio for emergency services (0.0-1.0)",
                   DoubleValue (0.2),
                   MakeDoubleAccessor (&AdmitControl::m_emergencyReservationRatio),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("AdmissionThreshold",
                   "Resource utilization threshold for admission (0.0-1.0)",
                   DoubleValue (0.9),
                   MakeDoubleAccessor (&AdmitControl::m_admissionThreshold),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("HandoverBenefitThreshold",
                   "Minimum benefit ratio for handover decision",
                   DoubleValue (0.15),
                   MakeDoubleAccessor (&AdmitControl::m_handoverBenefitThreshold),
                   MakeDoubleChecker<double> ());
  return tid;
}

AdmitControl::AdmitControl ()
  : m_emergencyReservationRatio (0.2),
    m_totalDlRbs (160),
    m_totalUlRbs (50),
    m_admissionThreshold (0.9),
    m_handoverBenefitThreshold (0.15)
{
  NS_LOG_FUNCTION (this);
}

AdmitControl::~AdmitControl () { NS_LOG_FUNCTION (this); }

// ==================== 准入控制核心接口 ====================

AdmitDecision AdmitControl::CanAdmitUe (uint32_t targetBeamId, ServicePriority priority,
                                         UtType utType, TrafficType trafficType, uint32_t requiredRbs)
{
  NS_LOG_FUNCTION (this << targetBeamId << (uint32_t)priority << requiredRbs);
  
  auto beamIt = m_beamResources.find (targetBeamId);
  if (beamIt == m_beamResources.end ()) {
      NS_LOG_WARN ("Beam " << targetBeamId << " not found in resource manager, creating new entry");
      BeamResourceStatus newBeam;
      newBeam.beamId = targetBeamId;
      newBeam.totalRbs = m_totalDlRbs;
      newBeam.usedRbs = 0;
      newBeam.emergencyRbs = 0;
      newBeam.guaranteedRbs = 0;
      newBeam.utilizationRatio = 0.0;
      newBeam.activeUeCount = 0;
      m_beamResources[targetBeamId] = newBeam;
      beamIt = m_beamResources.find (targetBeamId);
  }
  
  BeamResourceStatus& beamStatus = beamIt->second;
  
  // 1. 检查应急业务容量
  if (priority == ServicePriority::PRIORITY_EMERGENCY) {
      if (!CheckEmergencyCapacity (targetBeamId)) {
          NS_LOG_WARN ("[AdmitControl] EMERGENCY priority but insufficient emergency capacity!");
          return AdmitDecision::ADMIT_REDIRECT;
      }
  }
  
  // 2. 计算可用资源
  uint32_t availableRbs = GetAvailableRbs (targetBeamId, priority);
  
  // 3. 检查资源是否足够
  if (requiredRbs > availableRbs) {
      NS_LOG_WARN ("[AdmitControl] Insufficient RBs! Required: " << requiredRbs 
                   << ", Available: " << availableRbs);
      
      // 尝试重定向到其他波束
      std::vector<uint32_t> recommendedBeams = GetRecommendedBeams (targetBeamId, priority);
      if (!recommendedBeams.empty ()) {
          NS_LOG_INFO ("[AdmitControl] Redirecting to beam " << recommendedBeams[0]);
          return AdmitDecision::ADMIT_REDIRECT;
      }
      return AdmitDecision::ADMIT_REJECTED;
  }
  
  // 4. 检查资源利用率阈值
  double utilizationAfterAdmit = static_cast<double>(beamStatus.usedRbs + requiredRbs) / beamStatus.totalRbs;
  
  if (utilizationAfterAdmit > m_admissionThreshold) {
      NS_LOG_WARN ("[AdmitControl] Utilization would exceed threshold! "
                   << "After admit: " << utilizationAfterAdmit * 100 << "%");
      
      // 应急业务可以突破阈值
      if (priority != ServicePriority::PRIORITY_EMERGENCY) {
          return AdmitDecision::ADMIT_QUEUE;
      }
  }
  
  NS_LOG_INFO ("[AdmitControl] UE admitted to Beam " << targetBeamId 
               << " | Priority: " << (uint32_t)priority
               << " | Required RBs: " << requiredRbs
               << " | Available: " << availableRbs
               << " | Utilization: " << utilizationAfterAdmit * 100 << "%");
  
  return AdmitDecision::ADMIT_OK;
}

AdmitDecision AdmitControl::CanHandoverUe (uint32_t sourceBeamId, uint32_t targetBeamId,
                                            ServicePriority priority, uint32_t requiredRbs)
{
  NS_LOG_FUNCTION (this << sourceBeamId << targetBeamId << (uint32_t)priority << requiredRbs);
  
  // 1. 先检查目标波束是否能接纳
  AdmitDecision directAdmit = CanAdmitUe (targetBeamId, priority, UT_CONSUMER, TRAFFIC_DATA, requiredRbs);
  
  if (directAdmit == AdmitDecision::ADMIT_OK) {
      // 2. 计算切换收益
      double benefit = CalculateHandoverBenefit (targetBeamId, priority);
      
      NS_LOG_INFO ("[Handover] Direct admit possible | Benefit ratio: " << benefit);
      
      if (benefit > m_handoverBenefitThreshold) {
          return AdmitDecision::ADMIT_OK;
      } else {
          NS_LOG_INFO ("[Handover] Benefit too low, checking load balancing...");
      }
  }
  
  // 3. 检查负载均衡
  CheckBeamLoadBalancing ();
  
  // 4. 重新评估目标波束
  auto targetIt = m_beamResources.find (targetBeamId);
  if (targetIt == m_beamResources.end ()) {
      return AdmitDecision::ADMIT_REJECTED;
  }
  
  BeamResourceStatus& targetBeam = targetIt->second;
  double targetUtilization = static_cast<double>(targetBeam.usedRbs + requiredRbs) / targetBeam.totalRbs;
  
  // 5. 如果目标波束负载过高，尝试寻找更优波束
  if (targetUtilization > m_admissionThreshold) {
      std::vector<uint32_t> alternatives = GetRecommendedBeams (sourceBeamId, priority);
      for (uint32_t altBeam : alternatives) {
          if (altBeam != targetBeamId) {
              auto altIt = m_beamResources.find (altBeam);
              if (altIt != m_beamResources.end ()) {
                  double altUtil = static_cast<double>(altIt->second.usedRbs + requiredRbs) / altIt->second.totalRbs;
                  if (altUtil < targetUtilization - 0.1) {
                      NS_LOG_INFO ("[Handover] Redirecting from beam " << targetBeamId 
                                   << " to better beam " << altBeam);
                      return AdmitDecision::ADMIT_REDIRECT;
                  }
              }
          }
      }
  }
  
  return directAdmit;
}

void AdmitControl::RegisterUeToBeam (uint16_t rnti, uint32_t beamId, ServicePriority priority,
                                      UtType utType, TrafficType trafficType, Vector position)
{
  NS_LOG_FUNCTION (this << rnti << beamId << (uint32_t)priority);
  
  // 更新UE上下文
  UeContextInfo ueInfo;
  ueInfo.rnti = rnti;
  ueInfo.currentBeamId = beamId;
  ueInfo.priority = priority;
  ueInfo.utType = utType;
  ueInfo.trafficType = trafficType;
  ueInfo.position = position;
  ueInfo.latestCqi = 7.0;  // 默认中等CQI
  ueInfo.requiredRbs = 10; // 默认需求10RB
  ueInfo.connectedTime = Simulator::Now ();
  
  m_ueContextMap[rnti] = ueInfo;
  
  // 更新波束资源状态
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) {
      BeamResourceStatus newBeam;
      newBeam.beamId = beamId;
      newBeam.totalRbs = m_totalDlRbs;
      newBeam.usedRbs = 0;
      newBeam.activeUeCount = 0;
      m_beamResources[beamId] = newBeam;
      beamIt = m_beamResources.find (beamId);
  }
  
  BeamResourceStatus& beamStatus = beamIt->second;
  beamStatus.activeUeCount++;
  beamStatus.priorityCount[priority]++;
  UpdateBeamStatistics (beamId);
  
  NS_LOG_INFO ("[AdmitControl] UE " << rnti << " registered to Beam " << beamId
               << " | Priority: " << (uint32_t)priority
               << " | Active UEs: " << beamStatus.activeUeCount);
}

void AdmitControl::UnregisterUeFromBeam (uint16_t rnti, uint32_t beamId)
{
  NS_LOG_FUNCTION (this << rnti << beamId);
  
  auto ueIt = m_ueContextMap.find (rnti);
  if (ueIt != m_ueContextMap.end ()) {
      ServicePriority priority = ueIt->second.priority;
      m_ueContextMap.erase (ueIt);
      
      auto beamIt = m_beamResources.find (beamId);
      if (beamIt != m_beamResources.end ()) {
          BeamResourceStatus& beamStatus = beamIt->second;
          if (beamStatus.activeUeCount > 0) {
              beamStatus.activeUeCount--;
          }
          if (beamStatus.priorityCount[priority] > 0) {
              beamStatus.priorityCount[priority]--;
          }
          UpdateBeamStatistics (beamId);
          
          NS_LOG_INFO ("[AdmitControl] UE " << rnti << " unregistered from Beam " << beamId
                       << " | Active UEs: " << beamStatus.activeUeCount
                       << " | Utilization: " << beamStatus.utilizationRatio * 100 << "%");
      }
  }
}

void AdmitControl::UpdateUeContext (uint16_t rnti, double latestCqi, uint32_t requiredRbs)
{
  auto ueIt = m_ueContextMap.find (rnti);
  if (ueIt != m_ueContextMap.end ()) {
      ueIt->second.latestCqi = latestCqi;
      ueIt->second.requiredRbs = requiredRbs;
  }
}

// ==================== 波束协调接口 ====================

std::vector<uint32_t> AdmitControl::GetRecommendedBeams (uint32_t sourceBeamId, ServicePriority priority)
{
  NS_LOG_FUNCTION (this << sourceBeamId << (uint32_t)priority);
  
  std::vector<std::pair<uint32_t, double>> beamScores;
  
  for (auto& beamPair : m_beamResources) {
      uint32_t beamId = beamPair.first;
      if (beamId == sourceBeamId) continue;
      
      BeamResourceStatus& beamStatus = beamPair.second;
      uint32_t availableRbs = GetAvailableRbs (beamId, priority);
      
      if (availableRbs == 0) continue;
      
      // 计算评分：资源可用性 + 优先级匹配度
      double resourceScore = static_cast<double>(availableRbs) / beamStatus.totalRbs;
      double priorityBonus = (priority == ServicePriority::PRIORITY_EMERGENCY && 
                              beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY] == 0) ? 0.2 : 0.0;
      
      double totalScore = resourceScore + priorityBonus;
      
      beamScores.push_back ({beamId, totalScore});
  }
  
  // 按评分降序排列
  std::sort (beamScores.begin (), beamScores.end (),
             [](const auto& a, const auto& b) { return a.second > b.second; });
  
  std::vector<uint32_t> recommended;
  for (auto& score : beamScores) {
      recommended.push_back (score.first);
  }
  
  NS_LOG_INFO ("[BeamCoord] Recommended beams for priority " << (uint32_t)priority 
               << ": " << recommended.size () << " candidates");
  
  return recommended;
}

BeamResourceStatus AdmitControl::GetBeamResourceStatus (uint32_t beamId)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt != m_beamResources.end ()) {
      return beamIt->second;
  }
  
  BeamResourceStatus emptyStatus;
  emptyStatus.beamId = beamId;
  emptyStatus.totalRbs = m_totalDlRbs;
  emptyStatus.usedRbs = 0;
  emptyStatus.utilizationRatio = 0.0;
  emptyStatus.activeUeCount = 0;
  return emptyStatus;
}

void AdmitControl::CheckBeamLoadBalancing ()
{
  NS_LOG_FUNCTION (this);
  
  double totalUtilization = 0.0;
  double maxUtilization = 0.0;
  double minUtilization = 1.0;
  uint32_t maxUtilBeam = 0;
  uint32_t minUtilBeam = 0;
  
  for (auto& beamPair : m_beamResources) {
      double util = beamPair.second.utilizationRatio;
      totalUtilization += util;
      
      if (util > maxUtilization) {
          maxUtilization = util;
          maxUtilBeam = beamPair.first;
      }
      if (util < minUtilization) {
          minUtilization = util;
          minUtilBeam = beamPair.first;
      }
  }
  
  double avgUtilization = totalUtilization / m_beamResources.size ();
  
  NS_LOG_INFO ("[LoadBalancing] Avg: " << avgUtilization * 100 << "%"
               << " | Max: " << maxUtilization * 100 << "% (Beam " << maxUtilBeam << ")"
               << " | Min: " << minUtilization * 100 << "% (Beam " << minUtilBeam << ")");
  
  // 如果最大负载与最小负载差距超过30%，触发负载均衡
  if (maxUtilization - minUtilization > 0.3) {
      NS_LOG_WARN ("[LoadBalancing] Imbalance detected! Triggering rebalancing...");
      TriggerLoadRebalancing ();
  }
}

void AdmitControl::TriggerLoadRebalancing ()
{
  NS_LOG_FUNCTION (this);
  
  // 找出高负载波束和低负载波束
  std::vector<uint32_t> highLoadBeams, lowLoadBeams;
  
  for (auto& beamPair : m_beamResources) {
      if (beamPair.second.utilizationRatio > 0.8) {
          highLoadBeams.push_back (beamPair.first);
      } else if (beamPair.second.utilizationRatio < 0.4) {
          lowLoadBeams.push_back (beamPair.first);
      }
  }
  
  NS_LOG_INFO ("[LoadBalancing] High load beams: " << highLoadBeams.size ()
               << " | Low load beams: " << lowLoadBeams.size ());
  
  // TODO: 实现UE迁移策略 - 将高负载波束中的低优先级UE迁移到低负载波束
  // 这需要与调度器协同工作
}

// ==================== 资源配置 ====================

void AdmitControl::SetBeamTotalRbs (uint32_t beamId, uint32_t totalRbs)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt != m_beamResources.end ()) {
      beamIt->second.totalRbs = totalRbs;
  } else {
      BeamResourceStatus newBeam;
      newBeam.beamId = beamId;
      newBeam.totalRbs = totalRbs;
      newBeam.usedRbs = 0;
      m_beamResources[beamId] = newBeam;
  }
}

void AdmitControl::SetEmergencyReservationRatio (double ratio)
{
  m_emergencyReservationRatio = ratio;
}

uint32_t AdmitControl::GetTotalActiveUes () const
{
  return m_ueContextMap.size ();
}

// ==================== 私有辅助函数 ====================

uint32_t AdmitControl::GetAvailableRbs (uint32_t beamId, ServicePriority priority)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) {
      return 0;
  }
  
  BeamResourceStatus& beamStatus = beamIt->second;
  uint32_t totalRbs = beamStatus.totalRbs;
  uint32_t usedRbs = beamStatus.usedRbs;
  
  // 计算应急业务预留
  uint32_t emergencyReserved = static_cast<uint32_t>(totalRbs * m_emergencyReservationRatio);
  
  uint32_t availableRbs = totalRbs - usedRbs;
  
  // 非应急业务不能使用应急预留
  if (priority != ServicePriority::PRIORITY_EMERGENCY) {
      availableRbs = (availableRbs > emergencyReserved) ? (availableRbs - emergencyReserved) : 0;
  } else {
      // 应急业务可以使用全部可用资源，但需要检查是否有足够应急容量
      // 应急预留+普通可用资源都可以用
  }
  
  return availableRbs;
}

bool AdmitControl::CheckEmergencyCapacity (uint32_t beamId)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) {
      return true; // 新波束假设有容量
  }
  
  BeamResourceStatus& beamStatus = beamIt->second;
  uint32_t currentEmergencyCount = beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY];
  
  // 限制每个波束应急用户数 (例如最多10个)
  if (currentEmergencyCount >= 10) {
      return false;
  }
  
  return true;
}

double AdmitControl::CalculateHandoverBenefit (uint32_t targetBeamId, ServicePriority priority)
{
  auto targetIt = m_beamResources.find (targetBeamId);
  if (targetIt == m_beamResources.end ()) {
      return 0.0;
  }
  
  BeamResourceStatus& targetBeam = targetIt->second;
  
  // 收益 = 目标波束剩余资源比例 - 当前波束剩余资源比例
  double targetAvailableRatio = 1.0 - targetBeam.utilizationRatio;
  
  // 假设源波束利用率为80%
  double sourceAvailableRatio = 0.2;
  
  double benefit = targetAvailableRatio - sourceAvailableRatio;
  
  return std::max (0.0, benefit);
}

void AdmitControl::UpdateBeamStatistics (uint32_t beamId)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) return;
  
  BeamResourceStatus& beamStatus = beamIt->second;
  
  // 计算保障带宽 (应急+语音用户固定资源)
  uint32_t priorityUsers = beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY] + 
                           beamStatus.priorityCount[ServicePriority::PRIORITY_VOICE];
  beamStatus.guaranteedRbs = priorityUsers * 20; // 每个高优先级用户20RB
  
  // 计算应急业务已用资源
  beamStatus.emergencyRbs = beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY] * 20;
  
  // 计算总利用率
  beamStatus.utilizationRatio = static_cast<double>(beamStatus.usedRbs) / beamStatus.totalRbs;
}

ServicePriority AdmitControl::MapTrafficTypeToPriority (TrafficType trafficType)
{
  switch (trafficType) {
    case TRAFFIC_VOICE:
      return ServicePriority::PRIORITY_VOICE;
    case TRAFFIC_DATA:
      return ServicePriority::PRIORITY_DATA;
    case TRAFFIC_HIGH_CAPACITY:
      return ServicePriority::PRIORITY_DATA;
    default:
      return ServicePriority::PRIORITY_BEST_EFFORT;
  }
}

} // namespace ns3
