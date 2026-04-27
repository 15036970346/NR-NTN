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
    .AddAttribute ("EmergencyReservedRbs",
                   "Rigid PRB reservation for emergency control traffic.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&AdmitControl::m_emergencyReservedRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("EmergencyBurstCapRbs",
                   "Maximum emergency PRBs allowed in burst mode.",
                   UintegerValue (3),
                   MakeUintegerAccessor (&AdmitControl::m_emergencyBurstCapRbs),
                   MakeUintegerChecker<uint32_t> (1, 25))
    .AddAttribute ("VoiceReservedRbs",
                   "Rigid PRB reservation for voice traffic.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&AdmitControl::m_voiceReservedRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("AdmissionThreshold",
                   "Resource utilization threshold for admission (0.0-1.0)",
                   DoubleValue (0.9),
                   MakeDoubleAccessor (&AdmitControl::m_admissionThreshold),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("HandoverBenefitThreshold",
                   "Minimum benefit ratio for handover decision",
                   DoubleValue (0.15),
                   MakeDoubleAccessor (&AdmitControl::m_handoverBenefitThreshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EmergencyUserCapPerBeam",
                   "Maximum number of emergency UEs allowed per beam.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&AdmitControl::m_emergencyUserCapPerBeam),
                   MakeUintegerChecker<uint32_t> (1));
  return tid;
}

AdmitControl::AdmitControl ()
  : m_emergencyReservedRbs (1),
    m_emergencyBurstCapRbs (3),
    m_voiceReservedRbs (2),
    m_totalDlRbs (25),
    m_totalUlRbs (25),
    m_dlReservedRbs (0),
    m_admissionThreshold (0.9),
    m_handoverBenefitThreshold (0.15),
    m_emergencyUserCapPerBeam (10),
    m_resourceManager (nullptr)
{
  NS_LOG_FUNCTION (this);
}

AdmitControl::~AdmitControl () { NS_LOG_FUNCTION (this); }

// ==================== 准入控制核心接口 ====================

AdmitDecision AdmitControl::CanAdmitUe (uint32_t targetBeamId, ServicePriority priority,
                                         UtType utType, TrafficType trafficType, uint32_t requiredRbs,
                                         bool isUplink)
{
  NS_LOG_FUNCTION (this << targetBeamId << (uint32_t)priority << requiredRbs);
  
  auto beamIt = m_beamResources.find (targetBeamId);
  if (beamIt == m_beamResources.end ()) {
      NS_LOG_WARN ("Beam " << targetBeamId << " not found in resource manager, creating new entry");
      BeamResourceStatus newBeam;
      newBeam.beamId = targetBeamId;
      newBeam.totalRbs = isUplink ? m_totalUlRbs : m_totalDlRbs;
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
  const uint32_t effectiveRequiredRbs =
    CalculateEffectiveRequiredRbs (targetBeamId, priority, utType, trafficType, requiredRbs, isUplink);
  uint32_t availableRbs = GetAvailableRbs (targetBeamId, priority, isUplink);
  
  // 3. 检查资源是否足够
  if (effectiveRequiredRbs > availableRbs) {
      NS_LOG_WARN ("[AdmitControl] Insufficient RBs! Required: " << effectiveRequiredRbs
                   << " (raw=" << requiredRbs << ")"
                   << ", Available: " << availableRbs);
      
      // 尝试重定向到其他波束
      std::vector<uint32_t> recommendedBeams =
        GetRecommendedBeams (targetBeamId, priority, utType, trafficType, effectiveRequiredRbs, isUplink);
      if (!recommendedBeams.empty ()) {
          NS_LOG_INFO ("[AdmitControl] Redirecting to beam " << recommendedBeams[0]);
          return AdmitDecision::ADMIT_REDIRECT;
      }
      return AdmitDecision::ADMIT_REJECTED;
  }
  
  // 4. 检查资源利用率阈值
  const uint32_t physicalRemainingBeforeAdmit = GetPhysicalRemainingRbs (targetBeamId, isUplink);
  const uint32_t usedAfterAdmit = beamStatus.totalRbs > physicalRemainingBeforeAdmit ?
                                  (beamStatus.totalRbs - physicalRemainingBeforeAdmit + effectiveRequiredRbs) :
                                  effectiveRequiredRbs;
  double utilizationAfterAdmit = static_cast<double>(usedAfterAdmit) / beamStatus.totalRbs;
  const double admissionThreshold = GetAdmissionThreshold (priority, utType, trafficType, isUplink);
  
  if (utilizationAfterAdmit > admissionThreshold) {
      NS_LOG_WARN ("[AdmitControl] Utilization would exceed threshold! "
                   << "After admit: " << utilizationAfterAdmit * 100 << "%"
                   << " | Threshold=" << admissionThreshold * 100 << "%");
      
      // 应急业务可以突破阈值
      if (priority != ServicePriority::PRIORITY_EMERGENCY) {
          return AdmitDecision::ADMIT_QUEUE;
      }
  }
  
  NS_LOG_INFO ("[AdmitControl] UE admitted to Beam " << targetBeamId 
               << " | Priority: " << (uint32_t)priority
               << " | Required RBs: " << effectiveRequiredRbs << " (raw=" << requiredRbs << ")"
               << " | Available: " << availableRbs
               << " | Utilization: " << utilizationAfterAdmit * 100 << "%");
  
  return AdmitDecision::ADMIT_OK;
}

AdmitDecision AdmitControl::CanHandoverUe (uint32_t sourceBeamId, uint32_t targetBeamId,
                                            ServicePriority priority, UtType utType,
                                            TrafficType trafficType, uint32_t requiredRbs,
                                            bool isUplink)
{
  NS_LOG_FUNCTION (this << sourceBeamId << targetBeamId << (uint32_t)priority << requiredRbs);
  
  // 1. 先检查目标波束是否能接纳
  AdmitDecision directAdmit =
    CanAdmitUe (targetBeamId, priority, utType, trafficType, requiredRbs, isUplink);
  
  if (directAdmit == AdmitDecision::ADMIT_OK) {
      // 2. 计算切换收益
      double benefit = CalculateHandoverBenefit (sourceBeamId,
                                                 targetBeamId,
                                                 priority,
                                                 utType,
                                                 trafficType,
                                                 requiredRbs,
                                                 isUplink);
      
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
  const uint32_t targetRemaining = GetPhysicalRemainingRbs (targetBeamId, isUplink);
  const uint32_t targetUsedAfter = targetBeam.totalRbs > targetRemaining ?
                                   (targetBeam.totalRbs - targetRemaining + requiredRbs) :
                                   requiredRbs;
  double targetUtilization = static_cast<double>(targetUsedAfter) / targetBeam.totalRbs;
  
  // 5. 如果目标波束负载过高，尝试寻找更优波束
  if (targetUtilization > m_admissionThreshold) {
      std::vector<uint32_t> alternatives =
        GetRecommendedBeams (sourceBeamId, priority, utType, trafficType, requiredRbs, isUplink);
      for (uint32_t altBeam : alternatives) {
          if (altBeam != targetBeamId) {
              auto altIt = m_beamResources.find (altBeam);
              if (altIt != m_beamResources.end ()) {
                  const uint32_t altRemaining = GetPhysicalRemainingRbs (altBeam, isUplink);
                  const uint32_t altUsedAfter = altIt->second.totalRbs > altRemaining ?
                                                (altIt->second.totalRbs - altRemaining + requiredRbs) :
                                                requiredRbs;
                  double altUtil = static_cast<double>(altUsedAfter) / altIt->second.totalRbs;
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

  auto existingUeIt = m_ueContextMap.find (rnti);
  if (existingUeIt != m_ueContextMap.end ())
    {
      const uint32_t oldBeamId = existingUeIt->second.currentBeamId;
      const ServicePriority oldPriority = existingUeIt->second.priority;

      if (oldBeamId == beamId)
        {
          existingUeIt->second.priority = priority;
          existingUeIt->second.utType = utType;
          existingUeIt->second.trafficType = trafficType;
          existingUeIt->second.position = position;
          existingUeIt->second.latestCqi = 7.0;
          existingUeIt->second.requiredRbs = (priority == ServicePriority::PRIORITY_EMERGENCY) ? 1 :
                                             (priority == ServicePriority::PRIORITY_VOICE ? 2 : 1);
          existingUeIt->second.connectedTime = Simulator::Now ();

          BeamResourceStatus& beamStatus = m_beamResources[beamId];
          if (oldPriority != priority)
            {
              if (beamStatus.priorityCount[oldPriority] > 0)
                {
                  beamStatus.priorityCount[oldPriority]--;
                }
              beamStatus.priorityCount[priority]++;
            }
          UpdateBeamStatistics (beamId);
          return;
        }

      auto oldBeamIt = m_beamResources.find (oldBeamId);
      if (oldBeamIt != m_beamResources.end ())
        {
          BeamResourceStatus& oldBeamStatus = oldBeamIt->second;
          if (oldBeamStatus.activeUeCount > 0)
            {
              oldBeamStatus.activeUeCount--;
            }
          if (oldBeamStatus.priorityCount[oldPriority] > 0)
            {
              oldBeamStatus.priorityCount[oldPriority]--;
            }
          UpdateBeamStatistics (oldBeamId);
        }
    }
  
  // 更新UE上下文
  UeContextInfo ueInfo;
  ueInfo.rnti = rnti;
  ueInfo.currentBeamId = beamId;
  ueInfo.priority = priority;
  ueInfo.utType = utType;
  ueInfo.trafficType = trafficType;
  ueInfo.position = position;
  ueInfo.latestCqi = 7.0;  // 默认中等CQI
  ueInfo.requiredRbs = (priority == ServicePriority::PRIORITY_EMERGENCY) ? 1 :
                       (priority == ServicePriority::PRIORITY_VOICE ? 2 : 1);
  ueInfo.connectedTime = Simulator::Now ();
  
  m_ueContextMap[rnti] = ueInfo;
  
  // 更新波束资源状态
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) {
      BeamResourceStatus newBeam;
      newBeam.beamId = beamId;
      newBeam.totalRbs = m_totalDlRbs;
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

std::vector<uint32_t> AdmitControl::GetRecommendedBeams (uint32_t sourceBeamId,
                                                         ServicePriority priority,
                                                         UtType utType,
                                                         TrafficType trafficType,
                                                         uint32_t requiredRbs,
                                                         bool isUplink)
{
  NS_LOG_FUNCTION (this << sourceBeamId << (uint32_t)priority);
  
  std::vector<std::pair<uint32_t, double>> beamScores;
  
  for (auto& beamPair : m_beamResources) {
      uint32_t beamId = beamPair.first;
      if (beamId == sourceBeamId) continue;
      
      BeamResourceStatus& beamStatus = beamPair.second;
      uint32_t availableRbs = GetAvailableRbs (beamId, priority, isUplink);
      
      if (availableRbs == 0) continue;

      const uint32_t effectiveRequiredRbs =
        CalculateEffectiveRequiredRbs (beamId, priority, utType, trafficType, requiredRbs, isUplink);
      if (availableRbs < effectiveRequiredRbs)
        {
          continue;
        }
      
      const double totalRbs = std::max (1u, beamStatus.totalRbs);
      const double headroomScore =
        static_cast<double> (availableRbs - effectiveRequiredRbs) / totalRbs;
      const double utilizationPenalty = beamStatus.utilizationRatio;
      const double activeUePenalty = std::min (0.5, beamStatus.activeUeCount / 20.0);
      const double samePriorityPenalty =
        std::min (0.3, beamStatus.priorityCount[priority] * 0.05);
      double trafficPenalty = 0.0;
      if (trafficType == TRAFFIC_HIGH_CAPACITY)
        {
          trafficPenalty += 0.1;
        }
      if (isUplink && utType == UT_CONSUMER)
        {
          trafficPenalty += 0.05;
        }

      double priorityBonus = (priority == ServicePriority::PRIORITY_EMERGENCY &&
                              beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY] == 0) ? 0.2 : 0.0;
      
      double totalScore =
        headroomScore + priorityBonus - utilizationPenalty * 0.4 - activeUePenalty - samePriorityPenalty - trafficPenalty;
      
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
  UpdateBeamStatistics (beamId);
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

  if (m_beamResources.empty ())
    {
      return;
    }
  
  double totalUtilization = 0.0;
  double maxUtilization = 0.0;
  double minUtilization = 1.0;
  uint32_t maxUtilBeam = 0;
  uint32_t minUtilBeam = 0;
  
  for (auto& beamPair : m_beamResources) {
      UpdateBeamStatistics (beamPair.first);
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
      UpdateBeamStatistics (beamPair.first);
      if (beamPair.second.utilizationRatio > 0.8) {
          highLoadBeams.push_back (beamPair.first);
      } else if (beamPair.second.utilizationRatio < 0.4) {
          lowLoadBeams.push_back (beamPair.first);
      }
  }
  
  NS_LOG_INFO ("[LoadBalancing] High load beams: " << highLoadBeams.size ()
               << " | Low load beams: " << lowLoadBeams.size ());

  if (highLoadBeams.empty () || lowLoadBeams.empty () || m_handoverExecutor.IsNull ())
    {
      return;
    }

  for (uint32_t sourceBeamId : highLoadBeams)
    {
      std::vector<uint16_t> candidateUes;
      for (const auto& uePair : m_ueContextMap)
        {
          if (uePair.second.currentBeamId == sourceBeamId)
            {
              candidateUes.push_back (uePair.first);
            }
        }

      std::sort (candidateUes.begin (),
                 candidateUes.end (),
                 [&] (uint16_t lhs, uint16_t rhs) {
                   const UeContextInfo& left = m_ueContextMap.at (lhs);
                   const UeContextInfo& right = m_ueContextMap.at (rhs);
                   if (left.priority != right.priority)
                     {
                       return static_cast<uint32_t> (left.priority) >
                              static_cast<uint32_t> (right.priority);
                     }
                   if (left.requiredRbs != right.requiredRbs)
                     {
                       return left.requiredRbs > right.requiredRbs;
                     }
                   return left.connectedTime > right.connectedTime;
                 });

      for (uint16_t rnti : candidateUes)
        {
          const UeContextInfo& ue = m_ueContextMap.at (rnti);
          const std::vector<uint32_t> recommendedBeams =
            GetRecommendedBeams (sourceBeamId,
                                 ue.priority,
                                 ue.utType,
                                 ue.trafficType,
                                 ue.requiredRbs,
                                 false);
          for (uint32_t targetBeamId : recommendedBeams)
            {
              if (std::find (lowLoadBeams.begin (), lowLoadBeams.end (), targetBeamId) ==
                  lowLoadBeams.end ())
                {
                  continue;
                }

              const AdmitDecision decision =
                CanHandoverUe (sourceBeamId,
                               targetBeamId,
                               ue.priority,
                               ue.utType,
                               ue.trafficType,
                               ue.requiredRbs,
                               false);
              if (decision == AdmitDecision::ADMIT_OK)
                {
                  NS_LOG_INFO ("[LoadBalancing] Execute rebalance UE " << rnti
                               << " | SourceBeam=" << sourceBeamId
                               << " -> TargetBeam=" << targetBeamId);
                  m_handoverExecutor (rnti, targetBeamId);
                  UpdateBeamStatistics (sourceBeamId);
                  UpdateBeamStatistics (targetBeamId);
                  break;
                }
            }
        }
    }
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

void AdmitControl::SetPriorityReservationPolicy (uint32_t emergencyReservedRbs,
                                                 uint32_t emergencyBurstCapRbs,
                                                 uint32_t voiceReservedRbs)
{
  m_emergencyReservedRbs = std::min (emergencyReservedRbs, m_totalDlRbs);
  m_emergencyBurstCapRbs = std::max (m_emergencyReservedRbs,
                                     std::min (emergencyBurstCapRbs, m_totalDlRbs));
  m_voiceReservedRbs = std::min (voiceReservedRbs, m_totalDlRbs);
}

void AdmitControl::SetDlReservedRbs (uint32_t reservedRbs)
{
  m_dlReservedRbs = std::min (reservedRbs, m_totalDlRbs);
}

void AdmitControl::SetResourceManager (Ptr<ResourceManager> resourceManager)
{
  m_resourceManager = resourceManager;
}

uint32_t AdmitControl::GetEmergencyReservedRbs () const
{
  return m_emergencyReservedRbs;
}

uint32_t AdmitControl::GetEmergencyBurstCapRbs () const
{
  return m_emergencyBurstCapRbs;
}

uint32_t AdmitControl::GetVoiceReservedRbs () const
{
  return m_voiceReservedRbs;
}

uint32_t AdmitControl::GetTotalActiveUes () const
{
  return m_ueContextMap.size ();
}

// ==================== 私有辅助函数 ====================

uint32_t AdmitControl::GetAvailableRbs (uint32_t beamId, ServicePriority priority, bool isUplink)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) {
      return 0;
  }
  
  BeamResourceStatus& beamStatus = beamIt->second;
  uint32_t totalRbs = beamStatus.totalRbs;
  uint32_t availableRbs = totalRbs - (totalRbs - GetPhysicalRemainingRbs (beamId, isUplink));
  if (!isUplink)
    {
      availableRbs = (availableRbs > m_dlReservedRbs) ? (availableRbs - m_dlReservedRbs) : 0;
    }
  const uint32_t protectedEmergency = std::min (m_emergencyReservedRbs, totalRbs);
  const uint32_t protectedVoice = std::min (m_voiceReservedRbs, totalRbs - protectedEmergency);

  switch (priority)
    {
    case ServicePriority::PRIORITY_EMERGENCY:
      return std::min (availableRbs, m_emergencyBurstCapRbs);
    case ServicePriority::PRIORITY_VOICE:
      availableRbs = (availableRbs > protectedEmergency) ? (availableRbs - protectedEmergency) : 0;
      return std::min (availableRbs, m_voiceReservedRbs);
    case ServicePriority::PRIORITY_DATA:
    case ServicePriority::PRIORITY_BEST_EFFORT:
    default:
      if (availableRbs <= protectedEmergency + protectedVoice)
        {
          return 0;
        }
      return availableRbs - protectedEmergency - protectedVoice;
    }
}

uint32_t AdmitControl::GetPhysicalRemainingRbs (uint32_t beamId, bool isUplink)
{
  auto beamIt = m_beamResources.find (beamId);
  const uint32_t totalRbs =
    (beamIt != m_beamResources.end ()) ? beamIt->second.totalRbs : (isUplink ? m_totalUlRbs : m_totalDlRbs);

  if (m_resourceManager != nullptr)
    {
      return m_resourceManager->GetRemainingRbs (beamId, isUplink);
    }

  if (beamIt == m_beamResources.end ())
    {
      return totalRbs;
    }

  return (beamIt->second.usedRbs < totalRbs) ? (totalRbs - beamIt->second.usedRbs) : 0;
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
  if (currentEmergencyCount >= m_emergencyUserCapPerBeam) {
      return false;
  }
  
  return true;
}

double AdmitControl::CalculateHandoverBenefit (uint32_t sourceBeamId, uint32_t targetBeamId,
                                               ServicePriority priority, UtType utType,
                                               TrafficType trafficType, uint32_t requiredRbs,
                                               bool isUplink)
{
  auto sourceIt = m_beamResources.find (sourceBeamId);
  auto targetIt = m_beamResources.find (targetBeamId);
  if (sourceIt == m_beamResources.end () || targetIt == m_beamResources.end ()) {
      return 0.0;
  }

  BeamResourceStatus& sourceBeam = sourceIt->second;
  BeamResourceStatus& targetBeam = targetIt->second;

  const uint32_t effectiveRequiredRbs =
    CalculateEffectiveRequiredRbs (targetBeamId, priority, utType, trafficType, requiredRbs, isUplink);
  const uint32_t sourceRemaining = GetPhysicalRemainingRbs (sourceBeamId, isUplink);
  const uint32_t targetRemaining = GetPhysicalRemainingRbs (targetBeamId, isUplink);
  const double sourceAvailableRatio =
    sourceBeam.totalRbs > 0 ? static_cast<double> (sourceRemaining) / sourceBeam.totalRbs : 0.0;
  const double targetHeadroomAfterAdmit =
    targetBeam.totalRbs > 0 ?
    static_cast<double> (targetRemaining > effectiveRequiredRbs ? (targetRemaining - effectiveRequiredRbs) : 0) /
      targetBeam.totalRbs :
    0.0;
  const double sourceReliefRatio =
    sourceBeam.totalRbs > 0 ?
    std::min<uint32_t> (effectiveRequiredRbs, sourceBeam.totalRbs - sourceRemaining) /
      static_cast<double> (sourceBeam.totalRbs) :
    0.0;

  double benefit = (targetHeadroomAfterAdmit - sourceAvailableRatio) + sourceReliefRatio * 0.5;
  if (trafficType == TRAFFIC_HIGH_CAPACITY)
    {
      benefit -= 0.05;
    }
  if (priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      benefit += 0.05;
    }

  return std::max (0.0, benefit);
}

void AdmitControl::UpdateBeamStatistics (uint32_t beamId)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) return;
  
  BeamResourceStatus& beamStatus = beamIt->second;
  
  // 固定业务策略：应急默认1 PRB，语音默认2 PRB，合计3 PRB 刚性预留。
  beamStatus.guaranteedRbs = std::min (beamStatus.totalRbs, m_emergencyReservedRbs + m_voiceReservedRbs);
  
  // 应急业务默认刚性预留与突发上限分开建模。
  beamStatus.emergencyRbs = std::min (beamStatus.totalRbs, m_emergencyReservedRbs);
  
  // 计算总利用率
  beamStatus.usedRbs = beamStatus.totalRbs - GetPhysicalRemainingRbs (beamId, false);
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

uint32_t
AdmitControl::CalculateEffectiveRequiredRbs (uint32_t beamId,
                                             ServicePriority priority,
                                             UtType utType,
                                             TrafficType trafficType,
                                             uint32_t requiredRbs,
                                             bool isUplink) const
{
  const auto beamIt = m_beamResources.find (beamId);
  const uint32_t totalRbs =
    (beamIt != m_beamResources.end ()) ? beamIt->second.totalRbs : (isUplink ? m_totalUlRbs : m_totalDlRbs);

  double multiplier = 1.0;
  switch (trafficType)
    {
    case TRAFFIC_VOICE:
      multiplier = 1.0;
      break;
    case TRAFFIC_HIGH_CAPACITY:
      multiplier = 1.5;
      break;
    case TRAFFIC_DATA:
    default:
      multiplier = 1.0;
      break;
    }

  if (isUplink && utType == UT_CONSUMER &&
      trafficType != TRAFFIC_VOICE &&
      priority != ServicePriority::PRIORITY_EMERGENCY)
    {
      multiplier += 0.15;
    }

  if (priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      multiplier = 1.0;
    }

  const uint32_t effectiveRequiredRbs =
    std::max (1u, static_cast<uint32_t> (std::ceil (requiredRbs * multiplier)));
  return std::min (effectiveRequiredRbs, totalRbs);
}

double
AdmitControl::GetAdmissionThreshold (ServicePriority priority,
                                     UtType utType,
                                     TrafficType trafficType,
                                     bool isUplink) const
{
  double threshold = m_admissionThreshold;

  if (priority == ServicePriority::PRIORITY_VOICE)
    {
      threshold -= 0.03;
    }
  else if (priority == ServicePriority::PRIORITY_BEST_EFFORT)
    {
      threshold -= 0.05;
    }

  if (trafficType == TRAFFIC_HIGH_CAPACITY)
    {
      threshold -= 0.08;
    }

  if (isUplink && utType == UT_CONSUMER)
    {
      threshold -= 0.04;
    }

  if (priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      threshold = std::max (threshold, 0.98);
    }

  return std::max (0.5, std::min (0.99, threshold));
}

void AdmitControl::SetHandoverExecutor (Callback<void, uint16_t, uint32_t> handoverExecutor)
{
  m_handoverExecutor = handoverExecutor;
}

} // namespace ns3
