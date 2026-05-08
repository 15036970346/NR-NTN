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
                   UintegerValue (1),//每个波束为应急业务预留1个 PRB 
                   MakeUintegerAccessor (&AdmitControl::m_emergencyReservedRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("EmergencyBurstCapRbs",
                   "Maximum emergency PRBs allowed in burst mode.",
                   UintegerValue (3),//应急业务在突发情况下最多可以使用3个 PRB
                   MakeUintegerAccessor (&AdmitControl::m_emergencyBurstCapRbs),
                   MakeUintegerChecker<uint32_t> (1, 25))
    .AddAttribute ("VoiceReservedRbs",
                   "Rigid PRB reservation for voice traffic.",
                   UintegerValue (2),//每个波束为语音业务预留2个 PRB
                   MakeUintegerAccessor (&AdmitControl::m_voiceReservedRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("AdmissionThreshold",
                   "Resource utilization threshold for admission (0.0-1.0)",
                   DoubleValue (0.9),//准入控制的资源利用率阈值 默认值是 0.9，也就是 90% 如果某个 UE 接入后，目标波束资源利用率超过这个阈值，系统就可能不允许它直接接入，而是让它排队或重定向。
                   MakeDoubleAccessor (&AdmitControl::m_admissionThreshold),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("HandoverBenefitThreshold",
                   "Minimum benefit ratio for handover decision",
                   DoubleValue (0.15),//切换收益阈值 只有切换收益大于这个阈值，才认为切换是值得的
                   MakeDoubleAccessor (&AdmitControl::m_handoverBenefitThreshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EmergencyUserCapPerBeam",
                   "Maximum number of emergency UEs allowed per beam.",
                   UintegerValue (10),//每个波束最多允许10个应急业务 UE
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
    m_dlReservedRbs (0),//下行没有额外控制信道预留
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
  
  auto beamIt = m_beamResources.find (targetBeamId);//先在 m_beamResources 中查找目标波束
  if (beamIt == m_beamResources.end ()) {//没有找到就创建一个新的 BeamResourceStatus
      NS_LOG_WARN ("Beam " << targetBeamId << " not found in resource manager, creating new entry");
      BeamResourceStatus newBeam;
      newBeam.beamId = targetBeamId;
      newBeam.totalRbs = isUplink ? m_totalUlRbs : m_totalDlRbs;
      m_beamResources[targetBeamId] = newBeam;//把新波束状态放进 map
      beamIt = m_beamResources.find (targetBeamId);//重新查找，拿到迭代器
  }
  
  BeamResourceStatus& beamStatus = beamIt->second;
  
  // 1. 检查应急业务容量
  if (priority == ServicePriority::PRIORITY_EMERGENCY) {
      if (!CheckEmergencyCapacity (targetBeamId)) {//检查当前波束中的应急用户数量是否已经达到上限
          NS_LOG_WARN ("[AdmitControl] EMERGENCY priority but insufficient emergency capacity!");
          return AdmitDecision::ADMIT_REDIRECT;//如果达到上限，就返回 ADMIT_REDIRECT，表示不建议接入当前波束，而应该尝试重定向到其他波束
      }
  }
  
  // 2. 计算可用资源
  const uint32_t effectiveRequiredRbs =
    CalculateEffectiveRequiredRbs (targetBeamId, priority, utType, trafficType, requiredRbs, isUplink);
  uint32_t availableRbs = GetAvailableRbs (targetBeamId, priority, isUplink);//根据业务优先级和预留策略计算当前可用 RB
  
  // 3. 检查资源是否足够
  if (effectiveRequiredRbs > availableRbs) {
      NS_LOG_WARN ("[AdmitControl] Insufficient RBs! Required: " << effectiveRequiredRbs
                   << " (raw=" << requiredRbs << ")"
                   << ", Available: " << availableRbs);
      
      // 尝试重定向到其他波束
      std::vector<uint32_t> recommendedBeams =
        GetRecommendedBeams (targetBeamId, priority, utType, trafficType, effectiveRequiredRbs, isUplink);
      //如果存在推荐波束，就返回 ADMIT_REDIRECT，让调度器去尝试重定向
        if (!recommendedBeams.empty ()) {
          NS_LOG_INFO ("[AdmitControl] Redirecting to beam " << recommendedBeams[0]);
          return AdmitDecision::ADMIT_REDIRECT;
      }
      return AdmitDecision::ADMIT_REJECTED;//如果没有推荐波束，就返回 ADMIT_REJECTED，说明无法接入
  }
  
  // 4. 检查资源利用率阈值
  const uint32_t physicalRemainingBeforeAdmit = GetPhysicalRemainingRbs (targetBeamId, isUplink);//当前物理层还剩多少 RB
  //beamStatus.totalRbs - physicalRemainingBeforeAdmit：当前已经使用了多少 RB 再加上 effectiveRequiredRbs，得到接入后的使用量
  const uint32_t usedAfterAdmit = beamStatus.totalRbs > physicalRemainingBeforeAdmit ?
                                  (beamStatus.totalRbs - physicalRemainingBeforeAdmit + effectiveRequiredRbs) :
                                  effectiveRequiredRbs;
  double utilizationAfterAdmit = static_cast<double>(usedAfterAdmit) / beamStatus.totalRbs;//接入后的资源利用率
  //根据业务类型、终端类型、方向动态计算准入阈值
  const double admissionThreshold = GetAdmissionThreshold (priority, utType, trafficType, isUplink);
  //如果接入后利用率超过阈值，系统会输出警告日志 如果不是应急业务，就返回 ADMIT_QUEUE，表示先进入等待队列
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

//判断 UE 是否可以从源波束切换到目标波束
AdmitDecision AdmitControl::CanHandoverUe (uint32_t sourceBeamId, uint32_t targetBeamId,
                                            ServicePriority priority, UtType utType,
                                            TrafficType trafficType, uint32_t requiredRbs,
                                            bool isUplink)
{
  //记录源波束、目标波束、优先级和请求 RB 数
  NS_LOG_FUNCTION (this << sourceBeamId << targetBeamId << (uint32_t)priority << requiredRbs);
  
  // 1. 先检查目标波束是否能接纳 如果目标波束连基本接入都不允许，就没有必要计算切换收益
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
      
      //如果切换收益大于阈值 m_handoverBenefitThreshold，就允许切换，返回 ADMIT_OK
      if (benefit > m_handoverBenefitThreshold) {
          return AdmitDecision::ADMIT_OK;
      } else {//如果收益不够高，就暂时不直接切换，而是继续看是否存在负载均衡需要
          NS_LOG_INFO ("[Handover] Benefit too low, checking load balancing...");
      }
  }
  
  // 3. 检查负载均衡 如果直接切换收益不够，代码会检查整个系统是否存在波束负载不均衡
  CheckBeamLoadBalancing ();
  
  // 4. 重新评估目标波束
  auto targetIt = m_beamResources.find (targetBeamId);
  if (targetIt == m_beamResources.end ()) {
      return AdmitDecision::ADMIT_REJECTED;//先查找目标波束 如果目标波束不存在，直接拒绝
  }
  //重新计算目标波束在接入该 UE 后的资源利用率
  BeamResourceStatus& targetBeam = targetIt->second;
  const uint32_t targetRemaining = GetPhysicalRemainingRbs (targetBeamId, isUplink);
  const uint32_t targetUsedAfter = targetBeam.totalRbs > targetRemaining ?
                                   (targetBeam.totalRbs - targetRemaining + requiredRbs) :
                                   requiredRbs;
  double targetUtilization = static_cast<double>(targetUsedAfter) / targetBeam.totalRbs;
  
  // 5. 如果目标波束负载过高，尝试寻找更优波束
  if (targetUtilization > m_admissionThreshold) {//如果目标波束接入后会超过默认准入阈值，则调用 GetRecommendedBeams() 找替代波束
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
                  double altUtil = static_cast<double>(altUsedAfter) / altIt->second.totalRbs;//计算候选波束接入后的利用率
                  if (altUtil < targetUtilization - 0.1) {//如果候选波束比当前目标波束低至少 0.1，就返回 ADMIT_REDIRECT
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

//作用是把某个 UE 注册到某个波束中，并更新 UE 上下文和波束统计信息
void AdmitControl::RegisterUeToBeam (uint16_t rnti, uint32_t beamId, ServicePriority priority,
                                      UtType utType, TrafficType trafficType, Vector position)
{
  NS_LOG_FUNCTION (this << rnti << beamId << (uint32_t)priority);

  auto existingUeIt = m_ueContextMap.find (rnti);//先在 m_ueContextMap 中查找这个 UE
  if (existingUeIt != m_ueContextMap.end ())//如果找到了，说明这个 UE 已经注册过
    {//读取旧波束 ID 和旧优先级
      const uint32_t oldBeamId = existingUeIt->second.currentBeamId;
      const ServicePriority oldPriority = existingUeIt->second.priority;

      if (oldBeamId == beamId)//如果旧波束和新波束相同，说明不是切换，只是更新 UE 信息
        {
          existingUeIt->second.priority = priority;
          existingUeIt->second.utType = utType;
          existingUeIt->second.trafficType = trafficType;
          existingUeIt->second.position = position;
          existingUeIt->second.latestDlCqi = 7.0;
          existingUeIt->second.latestUlCqi = 7.0;
          existingUeIt->second.requiredRbs = (priority == ServicePriority::PRIORITY_EMERGENCY) ? 1 :
                                             (priority == ServicePriority::PRIORITY_VOICE ? 2 : 1);
          existingUeIt->second.connectedTime = Simulator::Now ();

          BeamResourceStatus& beamStatus = m_beamResources[beamId];
          if (oldPriority != priority)//处理优先级变化 如果 UE 的优先级变了，就减少旧优先级计数，增加新优先级计数
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
              oldBeamStatus.activeUeCount--;//减少旧波束活跃 UE 数
            }
          if (oldBeamStatus.priorityCount[oldPriority] > 0)
            {
              oldBeamStatus.priorityCount[oldPriority]--;//减少旧业务优先级计数
            }
          UpdateBeamStatistics (oldBeamId);//把 UE 从旧波束中移走
        }
    }
  
  // 更新UE上下文
  UeContextInfo ueInfo;//创建一个新的 UeContextInfo 对象，并写入
  ueInfo.rnti = rnti;
  ueInfo.currentBeamId = beamId;
  ueInfo.priority = priority;
  ueInfo.utType = utType;
  ueInfo.trafficType = trafficType;
  ueInfo.position = position;
  ueInfo.latestDlCqi = 7.0;  // 默认中等DL CQI
  ueInfo.latestUlCqi = 7.0;  // 默认中等UL CQI
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
  
  //更新目标波束统计 活跃 UE 数加 1 对应优先级的 UE 数加 1 更新波束统计信息
  BeamResourceStatus& beamStatus = beamIt->second;
  beamStatus.activeUeCount++;
  beamStatus.priorityCount[priority]++;
  UpdateBeamStatistics (beamId);
  
  //说明 UE 已经注册到哪个波束，当前活跃 UE 数是多少
  NS_LOG_INFO ("[AdmitControl] UE " << rnti << " registered to Beam " << beamId
               << " | Priority: " << (uint32_t)priority
               << " | Active UEs: " << beamStatus.activeUeCount);
}

//从波束注销 UE
void AdmitControl::UnregisterUeFromBeam (uint16_t rnti, uint32_t beamId)
{
  NS_LOG_FUNCTION (this << rnti << beamId);
  
  //如果 UE 存在，就先记录它的优先级，然后从 m_ueContextMap 中删除
  auto ueIt = m_ueContextMap.find (rnti);
  if (ueIt != m_ueContextMap.end ()) {
      ServicePriority priority = ueIt->second.priority;
      m_ueContextMap.erase (ueIt);
      
      //更新波束状态
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

//这个函数用于更新 UE 的实时状态
void AdmitControl::UpdateUeContext (uint16_t rnti,
                                    double latestDlCqi,
                                    double latestUlCqi,
                                    uint32_t requiredRbs)
{
  auto ueIt = m_ueContextMap.find (rnti);
  if (ueIt != m_ueContextMap.end ()) {
      ueIt->second.latestDlCqi = latestDlCqi;//最新下行信道质量
      ueIt->second.latestUlCqi = latestUlCqi;//最新上行信道质量
      ueIt->second.requiredRbs = requiredRbs;//当前所需 RB 数
  }
}

// ==================== 波束协调接口 ====================

//源波束 ID、业务优先级、终端类型、业务类型、所需 RB 和上下行方向
std::vector<uint32_t> AdmitControl::GetRecommendedBeams (uint32_t sourceBeamId,
                                                         ServicePriority priority,
                                                         UtType utType,
                                                         TrafficType trafficType,
                                                         uint32_t requiredRbs,
                                                         bool isUplink)
{
  //初始化评分列表 beamScores 中每个元素是一个二元组 第一个值是波束 ID 第二个值是该波束评分
  NS_LOG_FUNCTION (this << sourceBeamId << (uint32_t)priority);
  
  std::vector<std::pair<uint32_t, double>> beamScores;
  
  //遍历所有波束并排除不可用波束 如果当前波束就是源波束，就跳过 如果可用 RB 为 0，也跳过
  for (auto& beamPair : m_beamResources) {
      uint32_t beamId = beamPair.first;
      if (beamId == sourceBeamId) continue;
      
      BeamResourceStatus& beamStatus = beamPair.second;
      uint32_t availableRbs = GetAvailableRbs (beamId, priority, isUplink);
      
      if (availableRbs == 0) continue;

      //先计算有效 RB 需求 如果该波束可用资源小于有效需求，就不作为候选波束
      const uint32_t effectiveRequiredRbs =
        CalculateEffectiveRequiredRbs (beamId, priority, utType, trafficType, requiredRbs, isUplink);
      if (availableRbs < effectiveRequiredRbs)
        {
          continue;
        }
      
      //计算资源余量和惩罚项 推荐波束不是只看剩余 RB，还综合考虑负载、用户数、同类业务拥挤程度
      const double totalRbs = std::max (1u, beamStatus.totalRbs);//总 RB，至少为 1，防止除 0
      const double headroomScore =
        static_cast<double> (availableRbs - effectiveRequiredRbs) / totalRbs;//接入该 UE 后还剩多少资源，余量越大越好
      const double utilizationPenalty = beamStatus.utilizationRatio;//当前利用率越高，惩罚越大
      const double activeUePenalty = std::min (0.5, beamStatus.activeUeCount / 20.0);//活跃 UE 越多，惩罚越大，最高惩罚 0.5
      const double samePriorityPenalty =
        std::min (0.3, beamStatus.priorityCount[priority] * 0.05);//同优先级用户越多，惩罚越大，最高 0.3
      //根据业务类型增加额外惩罚
        double trafficPenalty = 0.0;
      if (trafficType == TRAFFIC_HIGH_CAPACITY)
        {
          trafficPenalty += 0.1;//高容量业务，额外惩罚 0.1，因为它更容易占用资源
        }
      if (isUplink && utType == UT_CONSUMER)
        {
          trafficPenalty += 0.05;//消费级终端的上行业务，额外惩罚 0.05，因为消费级终端上行能力较弱
        }

      //如果当前业务是应急业务，并且该波束还没有应急 UE，就给这个波束加 0.2 分
      double priorityBonus = (priority == ServicePriority::PRIORITY_EMERGENCY &&
                              beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY] == 0) ? 0.2 : 0.0;
      //计算总评分并加入列表 评分越高，越适合作为目标波束
      double totalScore =
        headroomScore + priorityBonus - utilizationPenalty * 0.4 - activeUePenalty - samePriorityPenalty - trafficPenalty;
      
      beamScores.push_back ({beamId, totalScore});
  }
  
  // 按评分降序排列
  std::sort (beamScores.begin (), beamScores.end (),
             [](const auto& a, const auto& b) { return a.second > b.second; });
  //把排序后的波束 ID 提取出来，形成推荐列表 最后返回推荐波束 ID 列表
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
  UpdateBeamStatistics (beamId);//确保返回前统计信息是最新的
  //如果波束存在，就返回该波束状态
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt != m_beamResources.end ()) {
      return beamIt->second;
  }
  //如果波束不存在，就构造一个空状态返回
  BeamResourceStatus emptyStatus;
  emptyStatus.beamId = beamId;
  emptyStatus.totalRbs = m_totalDlRbs;
  emptyStatus.usedRbs = 0;
  emptyStatus.utilizationRatio = 0.0;
  emptyStatus.activeUeCount = 0;
  return emptyStatus;
}

//检查波束负载均衡
void AdmitControl::CheckBeamLoadBalancing ()
{
  NS_LOG_FUNCTION (this);

  //当前没有任何波束资源记录，直接返回，不做负载均衡检查
  if (m_beamResources.empty ())
    {
      return;
    }
  //初始化统计变量
  double totalUtilization = 0.0;
  double maxUtilization = 0.0;
  double minUtilization = 1.0;
  uint32_t maxUtilBeam = 0;
  uint32_t minUtilBeam = 0;
  
  //遍历所有波束，找最大最小负载
  for (auto& beamPair : m_beamResources) {
    //每遍历一个波束，先更新统计 读取该波束利用率累加到总利用率中
      UpdateBeamStatistics (beamPair.first);
      double util = beamPair.second.utilizationRatio;
      totalUtilization += util;
      
      //如果当前波束利用率比已知最大值更大，就更新最大值和最大负载波束 ID
      if (util > maxUtilization) {
          maxUtilization = util;
          maxUtilBeam = beamPair.first;
      }
      //如果当前波束利用率比已知最小值更小，就更新最小值和最小负载波束 ID
      if (util < minUtilization) {
          minUtilization = util;
          minUtilBeam = beamPair.first;
      }
  }
  
  double avgUtilization = totalUtilization / m_beamResources.size ();//计算平均波束利用率
  
  //输出平均利用率、最大利用率、最小利用率和对应波束 ID
  NS_LOG_INFO ("[LoadBalancing] Avg: " << avgUtilization * 100 << "%"
               << " | Max: " << maxUtilization * 100 << "% (Beam " << maxUtilBeam << ")"
               << " | Min: " << minUtilization * 100 << "% (Beam " << minUtilBeam << ")");
  
  // 如果最大负载与最小负载差距超过30%，触发负载均衡
  if (maxUtilization - minUtilization > 0.3) {
      NS_LOG_WARN ("[LoadBalancing] Imbalance detected! Triggering rebalancing...");
      TriggerLoadRebalancing ();
  }
}

void AdmitControl::TriggerLoadRebalancing ()//负责真正执行重平衡
{
  NS_LOG_FUNCTION (this);
  
  // 找出高负载波束和低负载波束
  std::vector<uint32_t> highLoadBeams, lowLoadBeams;
  
  for (auto& beamPair : m_beamResources) {
      UpdateBeamStatistics (beamPair.first);
      if (beamPair.second.utilizationRatio > 0.8) {//如果利用率大于 80%，加入高负载列表
          highLoadBeams.push_back (beamPair.first);
      } else if (beamPair.second.utilizationRatio < 0.4) {//如果利用率小于 40%，加入低负载列表
          lowLoadBeams.push_back (beamPair.first);
      }
  }
  
  NS_LOG_INFO ("[LoadBalancing] High load beams: " << highLoadBeams.size ()
               << " | Low load beams: " << lowLoadBeams.size ());

  //对每一个高负载波束，遍历所有 UE 如果某个 UE 当前就在这个高负载波束中，就加入候选 UE 列表
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

      //对候选 UE 排序，决定优先迁移谁
      std::sort (candidateUes.begin (),
                 candidateUes.end (),
                 [&] (uint16_t lhs, uint16_t rhs) {
                   const UeContextInfo& left = m_ueContextMap.at (lhs);
                   const UeContextInfo& right = m_ueContextMap.at (rhs);
                   if (left.priority != right.priority)
                     { //比较优先级
                       return static_cast<uint32_t> (left.priority) >
                              static_cast<uint32_t> (right.priority);
                     }
                   if (left.requiredRbs != right.requiredRbs)
                     {//如果优先级相同，比较 requiredRbs
                       return left.requiredRbs > right.requiredRbs;
                     }
                     //如果 RB 需求也相同，比较连接时间
                   return left.connectedTime > right.connectedTime;
                 });

      //为每个候选 UE 获取推荐波束
      for (uint16_t rnti : candidateUes)
        {//遍历候选 UE取出该 UE 的上下文信息 调用 GetRecommendedBeams() 获取适合迁移的目标波束列表
          const UeContextInfo& ue = m_ueContextMap.at (rnti);
          const std::vector<uint32_t> recommendedBeams =
            GetRecommendedBeams (sourceBeamId,
                                 ue.priority,
                                 ue.utType,
                                 ue.trafficType,
                                 ue.requiredRbs,
                                 false);
          //遍历推荐波束
          for (uint32_t targetBeamId : recommendedBeams)
            {
              if (std::find (lowLoadBeams.begin (), lowLoadBeams.end (), targetBeamId) ==
                  lowLoadBeams.end ())
                {
                  continue;
                }

              //检查是否允许切换 如果结果不是 ADMIT_OK，就继续尝试下一个目标波束
              const AdmitDecision decision =
                CanHandoverUe (sourceBeamId,
                               targetBeamId,
                               ue.priority,
                               ue.utType,
                               ue.trafficType,
                               ue.requiredRbs,
                               false);
              if (decision == AdmitDecision::ADMIT_OK)//允许切换
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
{//设置某个波束的总 RB 数 如果该波束已经存在，就直接更新 totalRbs
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt != m_beamResources.end ()) {
      beamIt->second.totalRbs = totalRbs;
  } else {//如果不存在，就创建新的 BeamResourceStatus
      BeamResourceStatus newBeam;
      newBeam.beamId = beamId;
      newBeam.totalRbs = totalRbs;
      newBeam.usedRbs = 0;
      m_beamResources[beamId] = newBeam;
  }
}
//设置应急业务和语音业务的预留策略
void AdmitControl::SetPriorityReservationPolicy (uint32_t emergencyReservedRbs,
                                                 uint32_t emergencyBurstCapRbs,
                                                 uint32_t voiceReservedRbs)
{
  //应急预留不能超过下行总 RB
  m_emergencyReservedRbs = std::min (emergencyReservedRbs, m_totalDlRbs);
  //应急突发上限不能超过总 RB，同时不能低于应急刚性预留
  m_emergencyBurstCapRbs = std::max (m_emergencyReservedRbs,
                                     std::min (emergencyBurstCapRbs, m_totalDlRbs));
  //语音预留也不能超过下行总 RB
  m_voiceReservedRbs = std::min (voiceReservedRbs, m_totalDlRbs);
}
//设置下行保留 RB，例如给 PRACH、控制信道或其他系统开销保留
void AdmitControl::SetDlReservedRbs (uint32_t reservedRbs)
{
  m_dlReservedRbs = std::min (reservedRbs, m_totalDlRbs);
}
//把外部的 ResourceManager 绑定到准入控制模块 通过 ResourceManager::GetRemainingRbs() 获取真实物理剩余 RB
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
//查找波束 如果波束不存在，就认为没有可用资源，返回 0
uint32_t AdmitControl::GetAvailableRbs (uint32_t beamId, ServicePriority priority, bool isUplink)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) {
      return 0;
  }
//计算基础可用 RB
  BeamResourceStatus& beamStatus = beamIt->second;
  uint32_t totalRbs = beamStatus.totalRbs;
  uint32_t availableRbs = totalRbs - (totalRbs - GetPhysicalRemainingRbs (beamId, isUplink));
  if (!isUplink)
    {
      availableRbs = (availableRbs > m_dlReservedRbs) ? (availableRbs - m_dlReservedRbs) : 0;
    }
  //计算保护资源
  const uint32_t protectedEmergency = std::min (m_emergencyReservedRbs, totalRbs);
  const uint32_t protectedVoice = std::min (m_voiceReservedRbs, totalRbs - protectedEmergency);
//根据业务优先级返回可用 RB
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
//获取物理剩余 RB
uint32_t AdmitControl::GetPhysicalRemainingRbs (uint32_t beamId, bool isUplink)
{//波束存在，就使用该波束自己的 totalRbs 波束不存在，就根据上下行方向使用默认值：上行 25，下行 25
  auto beamIt = m_beamResources.find (beamId);
  const uint32_t totalRbs =
    (beamIt != m_beamResources.end ()) ? beamIt->second.totalRbs : (isUplink ? m_totalUlRbs : m_totalDlRbs);
//直接问ResourceManager当前物理剩余 RB
  if (m_resourceManager != nullptr)
    {
      return m_resourceManager->GetRemainingRbs (beamId, isUplink);
    }
//没有 ResourceManager 时使用本地估算
  if (beamIt == m_beamResources.end ())
    {
      return totalRbs;
    }

  return (beamIt->second.usedRbs < totalRbs) ? (totalRbs - beamIt->second.usedRbs) : 0;
}
//检查应急用户容量 判断某个波束还能不能接纳应急业务 UE
bool AdmitControl::CheckEmergencyCapacity (uint32_t beamId)
{
  auto beamIt = m_beamResources.find (beamId);
  //如果波束不存在，返回 true，认为新波束有容量
  if (beamIt == m_beamResources.end ()) {
      return true; // 新波束假设有容量
  }
  //波束存在读取当前应急 UE 数量
  BeamResourceStatus& beamStatus = beamIt->second;
  uint32_t currentEmergencyCount = beamStatus.priorityCount[ServicePriority::PRIORITY_EMERGENCY];
  
  // 限制每个波束应急用户数 (最多10个)
  if (currentEmergencyCount >= m_emergencyUserCapPerBeam) {
      return false;
  }
  
  return true;
}
//计算切换收益
//查找源波束和目标波束 如果源波束或目标波束不存在，就无法计算收益，直接返回 0
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
//计算有效需求和剩余资源
//取出源波束和目标波束状态
  BeamResourceStatus& sourceBeam = sourceIt->second;
  BeamResourceStatus& targetBeam = targetIt->second;
//计算该 UE 的有效 RB 需求
  const uint32_t effectiveRequiredRbs =
    CalculateEffectiveRequiredRbs (targetBeamId, priority, utType, trafficType, requiredRbs, isUplink);
//获取源波束剩余 RB 和目标波束剩余 RB
  const uint32_t sourceRemaining = GetPhysicalRemainingRbs (sourceBeamId, isUplink);
  const uint32_t targetRemaining = GetPhysicalRemainingRbs (targetBeamId, isUplink);
//计算源波束可用比例和目标波束接入后余量
//源波束可用比例 = 源波束剩余 RB / 源波束总 RB
  const double sourceAvailableRatio =
    sourceBeam.totalRbs > 0 ? static_cast<double> (sourceRemaining) / sourceBeam.totalRbs : 0.0;
//目标波束接入该 UE 后的资源余量比例
  const double targetHeadroomAfterAdmit =
    targetBeam.totalRbs > 0 ?
    static_cast<double> (targetRemaining > effectiveRequiredRbs ? (targetRemaining - effectiveRequiredRbs) : 0) /
      targetBeam.totalRbs :
    0.0;
//计算源波束释放比例 表示如果把 UE 从源波束迁走，可以缓解多少负载
  const double sourceReliefRatio =
    sourceBeam.totalRbs > 0 ?
    std::min<uint32_t> (effectiveRequiredRbs, sourceBeam.totalRbs - sourceRemaining) /
      static_cast<double> (sourceBeam.totalRbs) :
    0.0;
//计算最终收益
  double benefit = (targetHeadroomAfterAdmit - sourceAvailableRatio) + sourceReliefRatio * 0.5;
  if (trafficType == TRAFFIC_HIGH_CAPACITY)
    {
      benefit -= 0.05;//高容量业务降低收益，因为高容量业务迁移风险更大、占用资源更多
    }
  if (priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      benefit += 0.05;//应急业务提高收益，因为应急业务优先级更高
    }

  return std::max (0.0, benefit);
}
//更新波束统计信息
void AdmitControl::UpdateBeamStatistics (uint32_t beamId)
{
  auto beamIt = m_beamResources.find (beamId);
  if (beamIt == m_beamResources.end ()) return;
//更新保障资源和应急资源
  BeamResourceStatus& beamStatus = beamIt->second;
  
  // 固定业务策略：应急默认1 PRB，语音默认2 PRB，合计3 PRB 刚性预留。
  beamStatus.guaranteedRbs = std::min (beamStatus.totalRbs, m_emergencyReservedRbs + m_voiceReservedRbs);
  
  // 应急业务默认刚性预留与突发上限分开建模。
  beamStatus.emergencyRbs = std::min (beamStatus.totalRbs, m_emergencyReservedRbs);
  
  // 计算总利用率
  beamStatus.usedRbs = beamStatus.totalRbs - GetPhysicalRemainingRbs (beamId, false);
  beamStatus.utilizationRatio = static_cast<double>(beamStatus.usedRbs) / beamStatus.totalRbs;
}
//业务类型映射优先级
ServicePriority AdmitControl::MapTrafficTypeToPriority (TrafficType trafficType)
{
  switch (trafficType) {
    case TRAFFIC_VOICE:
      return ServicePriority::PRIORITY_VOICE;//语音业务 → 语音优先级
    case TRAFFIC_DATA:
      return ServicePriority::PRIORITY_DATA;//普通数据业务 → 数据优先级
    case TRAFFIC_HIGH_CAPACITY:
      return ServicePriority::PRIORITY_DATA;//高容量业务 → 数据优先级
    default:
      return ServicePriority::PRIORITY_BEST_EFFORT;//其他未知业务 → 尽力而为优先级
  }
}
//计算有效 RB 需求
uint32_t
AdmitControl::CalculateEffectiveRequiredRbs (uint32_t beamId,
                                             ServicePriority priority,
                                             UtType utType,
                                             TrafficType trafficType,
                                             uint32_t requiredRbs,
                                             bool isUplink) const
{
  const auto beamIt = m_beamResources.find (beamId);//波束存在，就用该波束自己的总 RB
  //不存在，就使用默认上下行总 RB
  const uint32_t totalRbs =
    (beamIt != m_beamResources.end ()) ? beamIt->second.totalRbs : (isUplink ? m_totalUlRbs : m_totalDlRbs);
//根据业务类型设置需求倍率
  double multiplier = 1.0;//默认
  switch (trafficType)
    {
    case TRAFFIC_VOICE:
      multiplier = 1.0;
      break;
    case TRAFFIC_HIGH_CAPACITY:
      multiplier = 1.5;
      break;
    case TRAFFIC_DATA://普通数据业务
    default:
      multiplier = 1.0;
      break;
    }
//消费级终端上行增加需求惩罚 消费级终端上行能力较弱，因此准入控制中要更保守
  if (isUplink && utType == UT_CONSUMER &&
      trafficType != TRAFFIC_VOICE &&
      priority != ServicePriority::PRIORITY_EMERGENCY)
    {
      multiplier += 0.15;
    }
//应急业务不放大需求 就强制倍率为 1.0，不进行额外惩罚
  if (priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      multiplier = 1.0;
    }
//计算最终有效 RB 最后再限制不能超过该波束总 RB
  const uint32_t effectiveRequiredRbs =
    std::max (1u, static_cast<uint32_t> (std::ceil (requiredRbs * multiplier)));
  return std::min (effectiveRequiredRbs, totalRbs);
}
//动态计算准入阈值
//从默认准入阈值开始，默认是 0.9
double
AdmitControl::GetAdmissionThreshold (ServicePriority priority,
                                     UtType utType,
                                     TrafficType trafficType,
                                     bool isUplink) const
{
  //根据优先级调整阈值 阈值降低意味着准入更严格
  double threshold = m_admissionThreshold;

  if (priority == ServicePriority::PRIORITY_VOICE)
    {
      threshold -= 0.03;
    }
  else if (priority == ServicePriority::PRIORITY_BEST_EFFORT)
    {
      threshold -= 0.05;
    }
//高容量业务更占资源，因此准入更严格，阈值降低 0.08
  if (trafficType == TRAFFIC_HIGH_CAPACITY)
    {
      threshold -= 0.08;
    }
//消费级终端上行能力弱，所以在上行准入时更加保守
  if (isUplink && utType == UT_CONSUMER)
    {
      threshold -= 0.04;
    }
//应急业务可以在更高负载下被接纳
  if (priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      threshold = std::max (threshold, 0.98);
    }
//最终阈值不能低于 0.5，也不能高于 0.99 防止参数调整后出现过低或过高的不合理阈值
  return std::max (0.5, std::min (0.99, threshold));
}
//设置切换执行回调 需要执行重平衡切换时调用m_handoverExecutor
void AdmitControl::SetHandoverExecutor (Callback<void, uint16_t, uint32_t> handoverExecutor)
{
  m_handoverExecutor = handoverExecutor;
}

} // namespace ns3
