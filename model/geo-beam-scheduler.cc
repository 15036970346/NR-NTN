/*
 * 文件路径：contrib/geo-sat/model/geo-beam-scheduler.cc
 * 功能：GEO 卫星波束感知 MAC 层调度器实现 - 融合 RRM 版
 */
#include "geo-beam-scheduler.h"
#include "ns3/simulator.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GeoBeamScheduler");
NS_OBJECT_ENSURE_REGISTERED (GeoBeamScheduler);

TypeId
GeoBeamScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GeoBeamScheduler")
    .SetParent<NrMacScheduler> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<GeoBeamScheduler> ()
    .AddAttribute ("DefaultK2Delay",
                   "The default scheduling delay for NTN (to compensate round trip time).",
                   TimeValue (MilliSeconds (120)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_defaultK2Delay),
                   MakeTimeChecker ())
    .AddAttribute ("EmergencyDelayThresholdSeconds",
                   "Delay threshold used to trigger emergency burst protection.",
                   DoubleValue (0.15),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_emergencyDelayThresholdSeconds),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("ReferencePathLossDb",
                   "Reference path loss used by scheduler-side UL power control.",
                   DoubleValue (190.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_referencePathLossDb),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("SrGrantRbs",
                   "Minimal uplink grant size issued after SR to bootstrap BSR reporting.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_srGrantRbs),
                   MakeUintegerChecker<uint32_t> (1, 25))
    .AddAttribute ("SrGrantMcs",
                   "MCS used by the minimal uplink grant issued after SR.",
                   UintegerValue (4),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_srGrantMcs),
                   MakeUintegerChecker<uint32_t> (0, 28))
    .AddAttribute ("Msg3RequestedRbs",
                   "Nominal PRBs requested for Msg3 before beam-budget approval.",
                   UintegerValue (6),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_msg3RequestedRbs),
                   MakeUintegerChecker<uint32_t> (1, 25))
    .AddAttribute ("Msg3GrantMcs",
                   "Conservative MCS used for Msg3.",
                   UintegerValue (4),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_msg3GrantMcs),
                   MakeUintegerChecker<uint32_t> (0, 28))
    .AddAttribute ("Msg3DefaultUtType",
                   "Default terminal type assumed for Msg3 power control before UE context exists (0=portable, 1=consumer).",
                   UintegerValue (static_cast<uint32_t> (UT_PORTABLE)),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_msg3DefaultUtTypeValue),
                   MakeUintegerChecker<uint32_t> (0, 1));
  return tid;
}

GeoBeamScheduler::GeoBeamScheduler ()
  : m_prachReservedRbs(0),
    m_wrrCurrentPriority (0),
    m_wrrCurrentIndex (0),
    m_ipfLocationWeight (0.3),
    m_ipfFairnessWeight (0.7),
    m_emergencyDelayThresholdSeconds (0.15),
    m_referencePathLossDb (190.0),
    m_srGrantRbs (1),
    m_srGrantMcs (4),
    m_msg3RequestedRbs (6),
    m_msg3GrantMcs (4),
    m_msg3DefaultUtTypeValue (static_cast<uint32_t> (UT_PORTABLE)),
    m_nextUlSchedulingRoundId (0),
    m_tcRntiCounter (0x8001),                  // 0x8001..0xFFF3 为 TC-RNTI 区间
    m_prachWindowDuration (MicroSeconds (500)),// PRACH 窗口 (gNB 侧去重)
    m_rarProcessingDelay (MilliSeconds (2)),   // RAR 处理时延
    m_rarTxDelay (MilliSeconds (300)),         // GEO 单程 ~300 ms
    m_msg3WindowDuration (MicroSeconds (500)), // Msg3 聚合窗口
    m_msg4ProcessingDelay (MilliSeconds (2)),  // Msg4 处理时延
    m_msg4TxDelay (MilliSeconds (300))         // GEO 单程 ~300 ms
{
  NS_LOG_FUNCTION (this);
  m_resourceManager = CreateObject<ResourceManager> ();
  m_admitControl = CreateObject<AdmitControl> ();
  m_admitControl->SetResourceManager (m_resourceManager);
  m_admitControl->SetHandoverExecutor (MakeCallback (&GeoBeamScheduler::ExecuteHandover, this));
}

GeoBeamScheduler::~GeoBeamScheduler () { NS_LOG_FUNCTION (this); }

// 空的 5G-LENA 接口占位符 (保持不变)
void GeoBeamScheduler::DoCschedCellConfigReq (const NrMacCschedSapProvider::CschedCellConfigReqParameters& params) {}
void GeoBeamScheduler::DoCschedUeConfigReq (const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) {}
void GeoBeamScheduler::DoCschedLcConfigReq (const NrMacCschedSapProvider::CschedLcConfigReqParameters& params) {}
void GeoBeamScheduler::DoCschedLcReleaseReq (const NrMacCschedSapProvider::CschedLcReleaseReqParameters& params) {}
void GeoBeamScheduler::DoCschedUeReleaseReq (const NrMacCschedSapProvider::CschedUeReleaseReqParameters& params) {}
void GeoBeamScheduler::DoSchedDlRlcBufferReq (const NrMacSchedSapProvider::SchedDlRlcBufferReqParameters& params)
{
  const uint32_t totalDlBytes = params.m_rlcTransmissionQueueSize +
                                params.m_rlcRetransmissionQueueSize +
                                params.m_rlcStatusPduSize;
  UpdateUeDlBufferStatus (params.m_rnti, totalDlBytes);
}
void GeoBeamScheduler::DoSchedDlCqiInfoReq (const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlCqiInfoReq (const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlMacCtrlInfoReq (const NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlTriggerReq (const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params)
{
  const uint64_t schedulingRoundId = ++m_nextUlSchedulingRoundId;

  if (m_beamToUesMap.empty ())
    {
      BeginUlSchedulingPeriod (m_myBeamId, schedulingRoundId);
      return;
    }

  for (const auto& beamPair : m_beamToUesMap)
    {
      BeginUlSchedulingPeriod (beamPair.first, schedulingRoundId);
    }

  RunUlScheduler ();
}
void GeoBeamScheduler::DoSchedUlSrInfoReq (const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
  for (uint16_t rnti : params.m_srList)
    {
      auto it = m_ueContextMap.find (rnti);
      if (it != m_ueContextMap.end ())
        {
          it->second.srPending = true;
          if (it->second.currentBeamId != 0)
            {
              BeginUlSchedulingPeriod (it->second.currentBeamId);
              RunUlSchedulerForBeam (it->second.currentBeamId);
            }
        }
    }
}
void GeoBeamScheduler::DoSchedSetMcs (uint32_t mcs) {}
void GeoBeamScheduler::DoSchedDlRachInfoReq (const NrMacSchedSapProvider::SchedDlRachInfoReqParameters& params) {}
uint8_t GeoBeamScheduler::GetDlCtrlSyms () const { return 1; }
uint8_t GeoBeamScheduler::GetUlCtrlSyms () const { return 1; }
int64_t GeoBeamScheduler::AssignStreams (int64_t stream) { return 0; }

void GeoBeamScheduler::DoSchedDlTriggerReq (const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params) 
{
    RunScheduler(); 
}

void GeoBeamScheduler::Initialize (uint32_t beamId, uint8_t scs)
{
  m_myBeamId = beamId;
  NS_LOG_INFO ("Scheduler Initialized for Beam " << beamId << ", SCS config: " << (uint16_t)scs);
}

void GeoBeamScheduler::ConfigPrachResources () { }

void GeoBeamScheduler::ReservePrachResources () 
{ 
  m_prachReservedRbs = 6; 
  if (m_admitControl != nullptr)
    {
      m_admitControl->SetDlReservedRbs (m_prachReservedRbs);
    }
}

void GeoBeamScheduler::AddUeContext (uint16_t rnti, UtType utType, TrafficType trafficType)
{
  if (m_ueContextMap.find(rnti) == m_ueContextMap.end()) {
      SatUeContext ctx;
      ctx.rnti = rnti;
      ctx.latestCqi = 7.0;
      ctx.latestRsrp = -120.0;
      ctx.dlBufferStatus = 0;
      ctx.ulBufferStatus = 0;
      ctx.pendingUlGrantRbs = 0;
      ctx.pendingUlGrantBytes = 0;
      ctx.srPending = false;
      ctx.dlAverageThroughput = 0.001;
      ctx.ulAverageThroughput = 0.001;
      ctx.utType = utType;
      ctx.trafficType = trafficType;
      ctx.position = Vector (0, 0, 0);
      ctx.priority = MapTrafficTypeToPriority (trafficType);
      ctx.wrrWeight = CalculateWrrWeight (ctx.priority, utType);
      ctx.lastDlScheduledTime = Time (0);
      ctx.lastUlScheduledTime = Time (0);
      m_ueContextMap[rnti] = ctx;
      NS_LOG_INFO ("UE Context created for RNTI " << rnti 
                   << " (Type: " << utType << ", Traffic: " << trafficType 
                   << ", Priority: " << (uint32_t)ctx.priority << ")");
  }
}

void GeoBeamScheduler::AddUeInfo (uint16_t rnti, uint32_t targetBeamId)
{
  SatUeContext& ctx = m_ueContextMap[rnti];
  ctx.currentBeamId = targetBeamId;
  m_beamToUesMap[targetBeamId].push_back(rnti);

  if (m_admitControl != nullptr)
    {
      m_admitControl->RegisterUeToBeam (rnti,
                                        targetBeamId,
                                        ctx.priority,
                                        ctx.utType,
                                        ctx.trafficType,
                                        ctx.position);
    }
}

void GeoBeamScheduler::RemoveUt (uint16_t rnti)
{
  uint32_t beamId = m_ueContextMap[rnti].currentBeamId;
  auto& ues = m_beamToUesMap[beamId];
  ues.erase(std::remove(ues.begin(), ues.end(), rnti), ues.end());

  if (m_admitControl != nullptr)
    {
      m_admitControl->UnregisterUeFromBeam (rnti, beamId);
    }

  m_ueDciCallbackMap.erase (rnti);
  m_admissionQueueSince.erase (rnti);
  m_ueContextMap.erase(rnti);
}

// void GeoBeamScheduler::HandoverUe (uint16_t rnti, uint32_t targetBeamId)
// {
//   uint32_t oldBeamId = m_ueContextMap[rnti].currentBeamId;
//   auto& ues = m_beamToUesMap[oldBeamId];
//   ues.erase(std::remove(ues.begin(), ues.end(), rnti), ues.end());
  
//   m_ueContextMap[rnti].currentBeamId = targetBeamId;
//   m_beamToUesMap[targetBeamId].push_back(rnti);
// }
  // 实现接口A：打包导出
HandoverUeContext GeoBeamScheduler::ExportUeContext (uint16_t rnti, uint32_t targetBeamId)
{
    NS_LOG_FUNCTION (this << rnti << targetBeamId);
    
    // 提取当前 UE 的内存快照
    SatUeContext& ctx = m_ueContextMap[rnti];
    HandoverUeContext hoCtx;
    hoCtx.rnti = rnti;
    hoCtx.sourceBeamId = ctx.currentBeamId;
    hoCtx.targetBeamId = targetBeamId;
    hoCtx.latestCqi = ctx.latestCqi;
    hoCtx.latestRsrp = ctx.latestRsrp;
    hoCtx.unsentDlBufferBytes = ctx.dlBufferStatus;
    hoCtx.unsentUlBufferBytes = ctx.ulBufferStatus;
    hoCtx.pendingUlGrantRbs = ctx.pendingUlGrantRbs;
    hoCtx.pendingUlGrantBytes = ctx.pendingUlGrantBytes;
    hoCtx.srPending = ctx.srPending;
    hoCtx.dlAverageThroughput = ctx.dlAverageThroughput;
    hoCtx.ulAverageThroughput = ctx.ulAverageThroughput;
    hoCtx.utType = ctx.utType;
    hoCtx.trafficType = ctx.trafficType;
    hoCtx.lastDlScheduledTime = ctx.lastDlScheduledTime;
    hoCtx.lastUlScheduledTime = ctx.lastUlScheduledTime;
    auto cbIt = m_ueDciCallbackMap.find (rnti);
    if (cbIt != m_ueDciCallbackMap.end ())
      {
        hoCtx.dciCallback = cbIt->second;
      }
    // 从源波束中彻底抹除该 UE 的痕迹
    RemoveUt (rnti);
    
    NS_LOG_INFO ("📦 成功打包 UE " << rnti << " 的上下文 (未发DL="
                 << hoCtx.unsentDlBufferBytes << " Bytes, 未发UL="
                 << hoCtx.unsentUlBufferBytes << " Bytes) 准备切往波束 " << targetBeamId);
                 
    return hoCtx;
}

// 实现接口B：目标波束导入
void GeoBeamScheduler::ImportUeContext (const HandoverUeContext& hoCtx)
{
    NS_LOG_FUNCTION (this << hoCtx.rnti << hoCtx.targetBeamId);
    
    // 在目标波束重建 UE 的调度档案
    SatUeContext ctx;
    ctx.rnti = hoCtx.rnti;
    ctx.currentBeamId = hoCtx.targetBeamId;
    ctx.latestCqi = hoCtx.latestCqi;
    ctx.latestRsrp = hoCtx.latestRsrp;
    ctx.dlBufferStatus = hoCtx.unsentDlBufferBytes;
    ctx.ulBufferStatus = hoCtx.unsentUlBufferBytes;
    ctx.pendingUlGrantRbs = hoCtx.pendingUlGrantRbs;
    ctx.pendingUlGrantBytes = hoCtx.pendingUlGrantBytes;
    ctx.srPending = hoCtx.srPending;
    ctx.dlAverageThroughput = hoCtx.dlAverageThroughput;
    ctx.ulAverageThroughput = hoCtx.ulAverageThroughput;
    ctx.utType = hoCtx.utType;
    ctx.trafficType = hoCtx.trafficType;
    ctx.priority = MapTrafficTypeToPriority (hoCtx.trafficType);
    ctx.wrrWeight = CalculateWrrWeight (ctx.priority, hoCtx.utType);
    ctx.position = Vector (0, 0, 0);
    ctx.lastDlScheduledTime = hoCtx.lastDlScheduledTime;
    ctx.lastUlScheduledTime = hoCtx.lastUlScheduledTime;

    m_ueContextMap[hoCtx.rnti] = ctx;
    m_beamToUesMap[hoCtx.targetBeamId].push_back(hoCtx.rnti);
    m_admissionQueueSince.erase (hoCtx.rnti);
    if (!hoCtx.dciCallback.IsNull ())
      {
        m_ueDciCallbackMap[hoCtx.rnti] = hoCtx.dciCallback;
      }

    if (m_admitControl != nullptr)
      {
        m_admitControl->RegisterUeToBeam (hoCtx.rnti,
                                          hoCtx.targetBeamId,
                                          ctx.priority,
                                          ctx.utType,
                                          ctx.trafficType,
                                          ctx.position);
      }
    
    NS_LOG_INFO ("📥 成功在波束 " << hoCtx.targetBeamId << " 注入 UE " << hoCtx.rnti 
                 << " 的上下文，调度器可直接恢复业务传输！");
}

// 实现接口C：统筹执行
void GeoBeamScheduler::ExecuteHandover (uint16_t rnti, uint32_t targetBeamId)
{
    NS_LOG_INFO ("=== 🚀 发起星地波束间无缝切换 (UE: " << rnti << " -> Beam: " << targetBeamId << ") ===");
    
    // 因为你们目前的信关站调度器是集中式管理所有波束的，所以直接在这里调用导出和导入即可模拟状态转移。
    HandoverUeContext hoCtx = ExportUeContext (rnti, targetBeamId);
    ImportUeContext (hoCtx);
}

void GeoBeamScheduler::UpdateUeCsi (uint16_t rnti, double cqi)
{
  if (m_ueContextMap.find(rnti) != m_ueContextMap.end()) {
      m_ueContextMap[rnti].latestCqi = cqi;
  }
}

void GeoBeamScheduler::UpdateUeDlBufferStatus (uint16_t rnti, uint32_t bufferBytes)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.dlBufferStatus = bufferBytes;
    }
}

void GeoBeamScheduler::UpdateUeUlBufferStatus (uint16_t rnti, uint32_t bufferBytes)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.ulBufferStatus = bufferBytes;
      it->second.pendingUlGrantBytes = std::min (it->second.pendingUlGrantBytes, bufferBytes);
      RefreshPendingUlGrantEstimate (it->second);
    }
}

void GeoBeamScheduler::PreProcessRequests () {}

uint8_t GeoBeamScheduler::GetMcsFromCqi (double cqi) const
{
  double effectiveCqi = std::max(1.0, cqi - 0.5); 
  uint8_t mcs = 0;
  int cqiIndex = std::round(effectiveCqi);

  switch (cqiIndex)
    {
      case 0: case 1: mcs = 0; break; 
      case 2: mcs = 2; break; 
      case 3: mcs = 4; break; 
      case 4: mcs = 6; break; 
      case 5: mcs = 8; break; 
      case 6: mcs = 10; break; 
      case 7: mcs = 12; break; 
      case 8: mcs = 14; break; 
      case 9: mcs = 16; break; 
      case 10: mcs = 18; break; 
      case 11: mcs = 20; break; 
      case 12: mcs = 22; break; 
      case 13: mcs = 24; break; 
      case 14: mcs = 26; break; 
      case 15: default: mcs = 28; break; 
    }
  return mcs;
}

double
GeoBeamScheduler::EstimateBytesPerRb (double cqi) const
{
  const uint8_t mcs = GetMcsFromCqi (cqi);
  // Approximate NR transport efficiency per PRB per slot using a compact
  // spectral-efficiency table. This is intentionally lighter than a full TBS
  // calculation, but avoids the previous overly conservative byte estimates.
  static const double kSpectralEfficiencyByMcs[29] = {
    0.2344, 0.3066, 0.3770, 0.4902, 0.6016, 0.7402, 0.8770, 1.0273, 1.1758, 1.3262,
    1.4766, 1.6953, 1.9141, 2.1602, 2.4063, 2.5703, 2.7305, 3.0293, 3.3223, 3.6094,
    3.9023, 4.2129, 4.5234, 4.8164, 5.1152, 5.3320, 5.5547, 6.2266, 7.4063};
  const uint8_t clampedMcs = std::min<uint8_t> (mcs, 28);
  // 12 subcarriers x 12 data symbols ~= 144 data RE/PRB/slot after DMRS/control overhead.
  const double dataRePerRb = 144.0;
  return std::max (1.0, kSpectralEfficiencyByMcs[clampedMcs] * dataRePerRb / 8.0);
}

double
GeoBeamScheduler::CalculateLocationFactor (const Vector& position) const
{
  const double distanceToCenter = std::sqrt (position.x * position.x + position.y * position.y);
  if (distanceToCenter <= 0.0)
    {
      return 1.0;
    }

  const double rawPenalty = 1.0 / (1.0 + 0.1 * distanceToCenter);
  return (1.0 - m_ipfLocationWeight) + m_ipfLocationWeight * rawPenalty;
}

double
GeoBeamScheduler::CalculateSchedulerMetric (const SatUeContext& ctx,
                                            uint32_t queueBudget,
                                            bool isUplink,
                                            double urgencyBoost,
                                            double demandBytes) const
{
  const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
  const double instRate = bytesPerRb * std::max (1u, queueBudget);
  const double locationFactor = CalculateLocationFactor (ctx.position);
  const Time timeSinceLastSched =
    Simulator::Now () - (isUplink ? ctx.lastUlScheduledTime : ctx.lastDlScheduledTime);
  const double delaySensitivity = std::exp (timeSinceLastSched.GetSeconds () * 1.5);
  const double avgThroughput =
    isUplink ? ctx.ulAverageThroughput : ctx.dlAverageThroughput;
  const double demandWeight = 1.0 + std::log1p (std::max (0.0, demandBytes)) / 8.0;
  const double cqiBoost = 1.0 + std::max (1.0, ctx.latestCqi) / 10.0;

  return (instRate * locationFactor * delaySensitivity * urgencyBoost * ctx.wrrWeight * demandWeight * cqiBoost) /
         std::pow (avgThroughput, m_ipfFairnessWeight);
}

uint32_t
GeoBeamScheduler::GetEffectiveUlDemandBytes (const SatUeContext& ctx) const
{
  return (ctx.ulBufferStatus > ctx.pendingUlGrantBytes) ?
           (ctx.ulBufferStatus - ctx.pendingUlGrantBytes) :
           0u;
}

void
GeoBeamScheduler::RefreshPendingUlGrantEstimate (SatUeContext& ctx) const
{
  ctx.pendingUlGrantBytes = std::min (ctx.pendingUlGrantBytes, ctx.ulBufferStatus);
  if (ctx.pendingUlGrantBytes == 0)
    {
      ctx.pendingUlGrantRbs = 0;
      return;
    }

  const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
  ctx.pendingUlGrantRbs =
    std::max (1u, static_cast<uint32_t> (std::ceil (ctx.pendingUlGrantBytes / bytesPerRb)));
}

void GeoBeamScheduler::RunScheduler ()
{
  NS_LOG_FUNCTION (this);
  ReservePrachResources ();

  for (auto const& beamPair : m_beamToUesMap)
    {
      uint32_t beamId = beamPair.first;
      const std::vector<uint16_t>& ueList = beamPair.second;
      std::vector<uint16_t> ueScheduleOrder;
      std::vector<std::pair<uint16_t, Time>> queuedUes;
      std::vector<uint16_t> freshUes;
      std::vector<std::pair<uint16_t, uint32_t>> redirectActions;

      for (uint16_t rnti : ueList)
        {
          auto queueIt = m_admissionQueueSince.find (rnti);
          if (queueIt != m_admissionQueueSince.end ())
            {
              queuedUes.push_back ({rnti, queueIt->second});
            }
          else
            {
              freshUes.push_back (rnti);
            }
        }

      std::sort (queuedUes.begin (),
                 queuedUes.end (),
                 [] (const std::pair<uint16_t, Time>& a, const std::pair<uint16_t, Time>& b) {
                   return a.second < b.second;
                 });

      ueScheduleOrder.reserve (ueList.size ());
      for (const auto& queuedUe : queuedUes)
        {
          ueScheduleOrder.push_back (queuedUe.first);
        }
      ueScheduleOrder.insert (ueScheduleOrder.end (), freshUes.begin (), freshUes.end ());

      m_resourceManager->ResetBeamAllocation (beamId, false);
      const uint32_t physicalRemainingDlRbs = m_resourceManager->GetRemainingRbs (beamId, false);
      uint32_t availableRbs =
        (physicalRemainingDlRbs > m_prachReservedRbs) ? (physicalRemainingDlRbs - m_prachReservedRbs) : 0;

      NS_LOG_INFO ("=== Beam " << beamId << " Scheduling (WRR+IPF) ===");

      // =====================================================================
      // 步骤 1 (对应文档 3.4): 根据 AdmitControl 准入结果，构建逻辑信道队列
      // =====================================================================
      std::vector<uint16_t> emergencyUes;
      std::vector<uint16_t> voiceUes;
      std::vector<uint16_t> normalUes;
      double maxEmergencyDelay = 0.0;
      double maxVoiceDelay = 0.0;

      for (uint16_t rnti : ueScheduleOrder) {
          SatUeContext& ctx = m_ueContextMap[rnti];
          const uint32_t bufferedBytes = ctx.dlBufferStatus;
          if (bufferedBytes == 0)
            {
              auto queueIt = m_admissionQueueSince.find (rnti);
              if (queueIt != m_admissionQueueSince.end ())
                {
                  NS_LOG_INFO ("[Queue Skip] Beam " << beamId
                               << " | UE " << rnti
                               << " 当前无真实待发数据，移出等待队列");
                  m_admissionQueueSince.erase (queueIt);
                }
              continue;
            }

          const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
          const uint32_t rawRequiredRbs =
            std::max (1u, static_cast<uint32_t> (std::ceil (bufferedBytes / bytesPerRb)));
          const uint32_t gateRequiredRbs =
            (ctx.priority == ServicePriority::PRIORITY_EMERGENCY) ? std::min (3u, rawRequiredRbs) :
            (ctx.priority == ServicePriority::PRIORITY_VOICE) ? std::min (2u, rawRequiredRbs) :
            1u;

          if (m_admitControl != nullptr)
            {
              m_admitControl->UpdateUeContext (rnti, ctx.latestCqi, gateRequiredRbs);
              const AdmitDecision decision =
                m_admitControl->CanAdmitUe (beamId, ctx.priority, ctx.utType, ctx.trafficType,
                                            gateRequiredRbs, false);

              if (decision == AdmitDecision::ADMIT_QUEUE)
                {
                  if (m_admissionQueueSince.find (rnti) == m_admissionQueueSince.end ())
                    {
                      m_admissionQueueSince[rnti] = Simulator::Now ();
                    }

                  NS_LOG_INFO ("[Admission Queue] Beam " << beamId
                               << " | UE " << rnti
                               << " | QueuedSince=" << m_admissionQueueSince[rnti].GetSeconds ()
                               << "s | RequiredRBs=" << gateRequiredRbs);
                  continue;
                }

              if (decision == AdmitDecision::ADMIT_REDIRECT)
                {
                  std::vector<uint32_t> candidates =
                    m_admitControl->GetRecommendedBeams (beamId,
                                                         ctx.priority,
                                                         ctx.utType,
                                                         ctx.trafficType,
                                                         gateRequiredRbs,
                                                         false);

                  uint32_t redirectBeamId = beamId;
                  for (uint32_t candidateBeamId : candidates)
                    {
                      if (candidateBeamId == beamId)
                        {
                          continue;
                        }

                      const AdmitDecision redirectDecision =
                        m_admitControl->CanAdmitUe (candidateBeamId,
                                                    ctx.priority,
                                                    ctx.utType,
                                                    ctx.trafficType,
                                                    gateRequiredRbs,
                                                    false);
                      if (redirectDecision == AdmitDecision::ADMIT_OK)
                        {
                          redirectBeamId = candidateBeamId;
                          break;
                        }
                    }

                  if (redirectBeamId != beamId)
                    {
                      redirectActions.push_back ({rnti, redirectBeamId});
                      m_admissionQueueSince.erase (rnti);
                      NS_LOG_INFO ("[Admission Redirect] Beam " << beamId
                                   << " | UE " << rnti
                                   << " -> Beam " << redirectBeamId
                                   << " | RequiredRBs=" << gateRequiredRbs);
                    }
                  else
                    {
                      if (m_admissionQueueSince.find (rnti) == m_admissionQueueSince.end ())
                        {
                          m_admissionQueueSince[rnti] = Simulator::Now ();
                        }

                      NS_LOG_INFO ("[Admission Redirect] Beam " << beamId
                                   << " | UE " << rnti
                                   << " 没有可执行重定向目标，转入等待队列");
                    }
                  continue;
                }

              if (decision != AdmitDecision::ADMIT_OK)
                {
                  NS_LOG_INFO ("[Admission Gate] Beam " << beamId
                               << " | UE " << rnti
                               << " | Decision=" << static_cast<uint32_t> (decision)
                               << " | RequiredRBs=" << gateRequiredRbs
                               << " -> skip this TTI");
                  continue;
                }

              auto queueIt = m_admissionQueueSince.find (rnti);
              if (queueIt != m_admissionQueueSince.end ())
                {
                  const double queuedMs = (Simulator::Now () - queueIt->second).GetMilliSeconds ();
                  NS_LOG_INFO ("[Admission Queue Exit] Beam " << beamId
                               << " | UE " << rnti
                               << " | Waited=" << queuedMs << " ms");
                  m_admissionQueueSince.erase (queueIt);
                }
            }

          Time delay = Simulator::Now () - ctx.lastDlScheduledTime;
          if (ctx.priority == ServicePriority::PRIORITY_EMERGENCY)
            {
              emergencyUes.push_back (rnti);
              maxEmergencyDelay = std::max (maxEmergencyDelay, delay.GetSeconds ());
            }
          else if (ctx.priority == ServicePriority::PRIORITY_VOICE)
            {
              voiceUes.push_back (rnti);
              maxVoiceDelay = std::max (maxVoiceDelay, delay.GetSeconds ());
            }
          else
            {
              normalUes.push_back (rnti);
            }
      }

      NS_LOG_INFO ("[Queue Build] Beam " << beamId
                   << " | Emergency=" << emergencyUes.size ()
                   << " | Voice=" << voiceUes.size ()
                   << " | Data/BE=" << normalUes.size ());

      // =====================================================================
      // 步骤 2 (对应文档 3.4): 第一级调度 - 基于优先级的加权轮询 (WRR) 动态提权
      // =====================================================================
      NS_LOG_INFO ("[WRR Stage 1] 正在按业务类型切片处理 emergency/voice 队列...");

      uint32_t emergencyBudget =
        std::min (availableRbs,
                  (m_admitControl != nullptr) ?
                  m_admitControl->GetEmergencyReservedRbs () :
                  1u);
      uint32_t voiceBudget =
        std::min (availableRbs - emergencyBudget,
                  (m_admitControl != nullptr) ?
                  m_admitControl->GetVoiceReservedRbs () :
                  2u);
      const double delayThreshold = m_emergencyDelayThresholdSeconds;

      if (maxEmergencyDelay > delayThreshold && !emergencyUes.empty()) {
          emergencyBudget =
            std::min (availableRbs,
                      (m_admitControl != nullptr) ?
                      m_admitControl->GetEmergencyBurstCapRbs () :
                      3u);
          voiceBudget =
            std::min (availableRbs - emergencyBudget,
                      (m_admitControl != nullptr) ?
                      m_admitControl->GetVoiceReservedRbs () :
                      2u);
          NS_LOG_WARN ("[WRR 动态提权] 应急业务时延 (" << maxEmergencyDelay * 1000
                       << " ms) 逼近阈值! 触发确定性保障机制, 应急预算提升到 "
                       << emergencyBudget << " RB!");
      }

      uint32_t trafficBudgetRemaining = availableRbs;

      auto scheduleClassQueue =
        [&] (const std::vector<uint16_t>& classQueue,
             uint32_t& queueBudget,
             const char* queueLabel,
             double urgencyBoost)
        {
          if (classQueue.empty () || queueBudget == 0)
            {
              return;
            }

          std::vector<std::pair<uint16_t, double>> metricQueue;
          for (uint16_t rnti : classQueue)
            {
              SatUeContext& ctx = m_ueContextMap[rnti];
              if (ctx.dlBufferStatus == 0)
                {
                  continue;
                }

              const double classMetric =
                CalculateSchedulerMetric (ctx,
                                          queueBudget,
                                          false,
                                          urgencyBoost,
                                          static_cast<double> (ctx.dlBufferStatus));

              metricQueue.push_back ({rnti, classMetric});
            }

          std::sort (metricQueue.begin (),
                     metricQueue.end (),
                     [] (const std::pair<uint16_t, double>& a, const std::pair<uint16_t, double>& b) {
                       return a.second > b.second;
                     });

          for (const auto& item : metricQueue)
            {
              if (queueBudget == 0)
                {
                  break;
                }

              const uint16_t rnti = item.first;
              SatUeContext& ctx = m_ueContextMap[rnti];
              const uint8_t targetMcs = GetMcsFromCqi (ctx.latestCqi);
              const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
              const uint32_t neededRbs =
                std::max (1u, static_cast<uint32_t> (std::ceil (ctx.dlBufferStatus / bytesPerRb)));
              const uint32_t schedulerProposedRbs = std::min (neededRbs, queueBudget);
              const uint32_t allocatedRb =
                m_resourceManager->AllocateSpectrum (beamId, schedulerProposedRbs, false);

              if (allocatedRb == 0)
                {
                  continue;
                }

              queueBudget -= allocatedRb;
              trafficBudgetRemaining = (trafficBudgetRemaining > allocatedRb) ?
                                       (trafficBudgetRemaining - allocatedRb) :
                                       0;
              const uint32_t allocatedBytes = allocatedRb * bytesPerRb;
              ctx.dlBufferStatus = (ctx.dlBufferStatus > allocatedBytes) ?
                                (ctx.dlBufferStatus - allocatedBytes) :
                                0;
              ctx.dlAverageThroughput =
                (1.0 - 0.1) * ctx.dlAverageThroughput + 0.1 * allocatedBytes;
              ctx.lastDlScheduledTime = Simulator::Now ();

              DciInfo dlDci;
              dlDci.isUplinkGrant = false;
              dlDci.rbAllocation = allocatedRb;
              dlDci.mcs = targetMcs;
              dlDci.txPowerDbm = 0.0;
              dlDci.delayToStart = m_defaultK2Delay;
              dlDci.duration = MilliSeconds (1);

              NS_LOG_INFO ("[IPF Stage 2] Beam " << beamId
                           << " | Queue=" << queueLabel
                           << " | UE=" << rnti
                           << " | MetricP=" << item.second
                           << " | Need=" << neededRbs
                           << " | Proposed=" << schedulerProposedRbs
                           << " | Allocated=" << allocatedRb);
              SendDciToUe (rnti, dlDci);
            }
        };

      scheduleClassQueue (emergencyUes, emergencyBudget, "emergency", 1.5);

      if (maxVoiceDelay > delayThreshold && !voiceUes.empty ())
        {
          NS_LOG_INFO ("[WRR 动态观察] 语音业务最大等待时延="
                       << maxVoiceDelay * 1000 << " ms");
        }

      scheduleClassQueue (voiceUes, voiceBudget, "voice", 1.2);
      uint32_t normalBudget = trafficBudgetRemaining;

      // =====================================================================
      // 步骤 3 (对应文档 3.4): 第二级调度 - 改进型比例公平算法 (IPF)
      // =====================================================================
      NS_LOG_INFO ("[IPF Stage 2] 正在对各业务队列执行同类用户优先级 P 排序...");
      scheduleClassQueue (normalUes, normalBudget, "data-best-effort", 1.0);

      NS_LOG_INFO ("=== Beam " << beamId << " Scheduling Complete | DL Remaining="
                   << m_resourceManager->GetRemainingRbs (beamId, false) << " RBs ===");

      for (const auto& redirectAction : redirectActions)
        {
          ExecuteHandover (redirectAction.first, redirectAction.second);
        }
    }
}

void GeoBeamScheduler::SendControlMsg (uint16_t rnti, uint8_t msgType) { }

void GeoBeamScheduler::ReceiveMeasReport (const MeasReport& report)
{
  if (m_ueContextMap.find (report.ueId) == m_ueContextMap.end ()) {
      AddUeContext (report.ueId);
      AddUeInfo (report.ueId, report.bestBeamId);
  } else if (m_ueContextMap[report.ueId].currentBeamId != report.bestBeamId) {
      ExecuteHandover (report.ueId, report.bestBeamId);
  }
  UpdateUePosition (report.ueId, report.position);
  m_ueContextMap[report.ueId].latestRsrp = report.rsrp;
}

void GeoBeamScheduler::SendDciToUe (uint16_t ueId, const DciInfo& dci)
{
  NS_LOG_INFO ("[DCI] 发送下行控制信息给 UE " << ueId 
               << " | UL Grant: " << dci.isUplinkGrant
               << " | RB: " << dci.rbAllocation 
               << " | MCS: " << (uint32_t)dci.mcs
               << " | TxPower: " << dci.txPowerDbm << " dBm"
               << " | Delay: " << dci.delayToStart.GetMilliSeconds () << "ms");

  auto cbIt = m_ueDciCallbackMap.find (ueId);
  if (cbIt == m_ueDciCallbackMap.end () || cbIt->second.IsNull ())
    {
      NS_LOG_WARN ("[DCI] UE " << ueId << " 未注册 DCI 回调，调度结果仅记录日志");
      return;
    }

  cbIt->second (dci);
}

void GeoBeamScheduler::RegisterUeDciCallback (uint16_t ueId, Callback<void, DciInfo> dciCb)
{
  m_ueDciCallbackMap[ueId] = dciCb;
  NS_LOG_INFO ("[DCI] 已为 UE " << ueId << " 注册调度结果回调");
}

// ==================== PUCCH/BSR 处理实现 ====================

void GeoBeamScheduler::ReceivePucchInfo (const PucchInfo& pucchInfo)
{
  NS_LOG_FUNCTION (this << (uint16_t)pucchInfo.rnti);
  
  switch (pucchInfo.format)
    {
      case PucchFormatType::FORMAT_0:
        NS_LOG_INFO ("[PUCCH] UE " << pucchInfo.rnti << " SR(调度请求) 收到!"
                     << " SR_Pending=" << pucchInfo.srPending);
        if (pucchInfo.srPending)
          {
            auto ctxIt = m_ueContextMap.find (pucchInfo.rnti);
            if (ctxIt != m_ueContextMap.end ())
              {
                ctxIt->second.srPending = true;
                BeginUlSchedulingPeriod (ctxIt->second.currentBeamId);
                RunUlSchedulerForBeam (ctxIt->second.currentBeamId);
              }
          }
        break;
        
      case PucchFormatType::FORMAT_1:
        NS_LOG_INFO ("[PUCCH] UE " << pucchInfo.rnti << " CQI报告收到! CQI=" 
                     << (uint32_t)pucchInfo.cqi);
        UpdateUeCsi (pucchInfo.rnti, pucchInfo.cqi);
        break;
        
      case PucchFormatType::FORMAT_2:
        NS_LOG_INFO ("[PUCCH] UE " << pucchInfo.rnti << " HARQ ACK/NACK收到! ACK=" 
                     << pucchInfo.harqAck << " Bitmap=" << (uint32_t)pucchInfo.harqBitMap);
        break;
        
      case PucchFormatType::FORMAT_3:
        NS_LOG_INFO ("[PUCCH] UE " << pucchInfo.rnti << " HARQ+CSI收到!");
        UpdateUeCsi (pucchInfo.rnti, pucchInfo.cqi);
        break;
        
      default:
        NS_LOG_WARN ("[PUCCH] 未知的PUCCH格式!");
        break;
    }
}

void GeoBeamScheduler::ReceiveBsr (const BsR_MAC_CE& bsr)
{
  NS_LOG_FUNCTION (this << bsr.rnti << bsr.bufferSize);
  
  uint16_t rnti = bsr.rnti;
  if (rnti == 0) {
      NS_LOG_WARN ("[BSR] RNTI为0，BSR可能被丢弃或RNTI尚未分配");
      return;
  }
  
  if (m_ueContextMap.find (rnti) != m_ueContextMap.end ()) {
      SatUeContext& ctx = m_ueContextMap[rnti];
      ctx.ulBufferStatus = bsr.bufferSize;
      ctx.srPending = false;
      ctx.pendingUlGrantBytes = std::min (ctx.pendingUlGrantBytes, ctx.ulBufferStatus);
      RefreshPendingUlGrantEstimate (ctx);
      NS_LOG_INFO ("[BSR] UE " << rnti << " 缓冲区状态更新: " 
                   << bsr.bufferSize << " bytes"
                   << " | LCG: " << (uint32_t)bsr.lcgId
                   << " | PendingGrantBytes=" << ctx.pendingUlGrantBytes
                   << " | EffectiveDemand=" << GetEffectiveUlDemandBytes (ctx));
  } else {
      NS_LOG_WARN ("[BSR] UE " << rnti << " 上下文不存在，BSR被忽略!");
  }
}

void GeoBeamScheduler::ReceiveUlMacPdu (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  if (packet == nullptr)
    {
      NS_LOG_WARN ("[UL Data] 收到空的 UL MAC PDU 指针");
      return;
    }

  Ptr<Packet> packetCopy = packet->Copy ();
  GenericUlMacHeader ulHeader;
  const uint32_t headerBytes = packetCopy->RemoveHeader (ulHeader);
  if (headerBytes != ulHeader.GetSerializedSize ())
    {
      NS_LOG_WARN ("[UL Data] 普通 UL MAC PDU 缺少 GenericUlMacHeader，忽略");
      return;
    }

  const uint16_t rnti = ulHeader.GetRnti ();
  const uint32_t payloadBytes = ulHeader.GetPayloadBytes ();

  auto ctxIt = m_ueContextMap.find (rnti);
  if (ctxIt == m_ueContextMap.end ())
    {
      NS_LOG_WARN ("[UL Data] UE " << rnti << " 上下文不存在，收到 " << payloadBytes
                   << " bytes 普通 UL MAC PDU 但无法入账");
      return;
    }

  SatUeContext& ctx = ctxIt->second;
  const uint32_t bufferedBeforeRx = ctx.ulBufferStatus;
  const uint32_t deliveredBytes = std::min (payloadBytes, bufferedBeforeRx);
  ctx.ulBufferStatus = (bufferedBeforeRx > deliveredBytes) ? (bufferedBeforeRx - deliveredBytes) : 0;
  ctx.pendingUlGrantBytes = (ctx.pendingUlGrantBytes > deliveredBytes) ?
                              (ctx.pendingUlGrantBytes - deliveredBytes) :
                              0;
  RefreshPendingUlGrantEstimate (ctx);
  ctx.srPending = false;
  ctx.ulAverageThroughput = (1.0 - 0.1) * ctx.ulAverageThroughput + 0.1 * deliveredBytes;
  ctx.lastUlScheduledTime = Simulator::Now ();

  NS_LOG_INFO ("[UL Data] UE " << rnti
               << " 普通 PUSCH MAC PDU 已到达 gNB"
               << " | Payload=" << payloadBytes
               << " bytes | BufferBefore=" << bufferedBeforeRx
               << " | Delivered=" << deliveredBytes
               << " | BufferAfter=" << ctx.ulBufferStatus
               << " | PendingGrantBytesAfter=" << ctx.pendingUlGrantBytes
               << " | PendingGrantRbsAfter=" << ctx.pendingUlGrantRbs);
}

// ==================== 4 步随机接入实现 (基站侧) ====================

void GeoBeamScheduler::ReceivePrachPreamble (const PrachPreamble& preamble)
{
  NS_LOG_FUNCTION (this << preamble.preambleId);

  NS_LOG_INFO ("[Msg1] PRACH 前导码收到! PreambleID=" << preamble.preambleId
               << " | Format=" << (uint32_t)preamble.format
               << " | Retx=" << preamble.isRetransmission
               << " | Time=" << Simulator::Now ().GetSeconds () << "s");

  // 将前导码加入当前 PRACH 窗口缓冲
  PreambleArrival arr;
      arr.preambleId  = preamble.preambleId;
      arr.utType      = static_cast<UtType> (preamble.utType);
      arr.arrivalTime = Simulator::Now ();
  m_prachWindowBuf.push_back (arr);

  // 若当前没有活动窗口, 调度一次窗口处理事件
  if (!m_prachWindowEvent.IsRunning ())
    {
      m_prachWindowEvent = Simulator::Schedule (m_prachWindowDuration,
                                                &GeoBeamScheduler::ProcessPrachWindow,
                                                this);
    }
}

void GeoBeamScheduler::ProcessPrachWindow ()
{
  NS_LOG_FUNCTION (this);
  BeginUlSchedulingPeriod (m_myBeamId);

  // 根据 preambleId 统计窗口内的到达次数
  std::map<uint32_t, uint32_t> preambleCnt;
  std::map<uint32_t, UtType> preambleUtType;
  for (const auto& a : m_prachWindowBuf)
    {
      preambleCnt[a.preambleId]++;
      auto utIt = preambleUtType.find (a.preambleId);
      if (utIt == preambleUtType.end ())
        {
          preambleUtType[a.preambleId] = a.utType;
        }
      else if (a.utType == UT_PORTABLE)
        {
          // 若同一前导码对应多个 UE，保守地采用能力更受限的 portable 终端做 Msg3 功控。
          preambleUtType[a.preambleId] = UT_PORTABLE;
        }
    }

  NS_LOG_INFO ("[PRACH] 窗口处理: 共 " << m_prachWindowBuf.size ()
               << " 个前导码, 唯一 PreambleID 数 = " << preambleCnt.size ());

  // 重要: 即使多个 UE 选用了同一个 preambleId, 基站在 Msg1 阶段
  // 解出的还是同一条前导码 (能量叠加 / 去重), 依然会发一条 RAR。
  // 碰撞会推迟到 Msg3 阶段才被发现——这与实际 4 步 RA 的行为一致。
  for (const auto& kv : preambleCnt)
    {
      uint32_t pid   = kv.first;
      uint32_t count = kv.second;

      const UtType msg3UtType =
        (preambleUtType.find (pid) != preambleUtType.end ()) ?
        preambleUtType.at (pid) :
        static_cast<UtType> (m_msg3DefaultUtTypeValue);
      const uint32_t msg3PowerLimitedRbs =
        m_resourceManager->GetMaxPowerLimitedUlRbs (msg3UtType, m_referencePathLossDb);
      const uint32_t msg3RequestedAfterPowerLimit =
        std::min (m_msg3RequestedRbs, msg3PowerLimitedRbs);
      if (msg3RequestedAfterPowerLimit == 0)
        {
          NS_LOG_WARN ("[Msg2] PreambleID=" << pid
                       << " 因终端功率上限受限，无法为 Msg3 分配任何非削顶 UL RB");
          continue;
        }
      RarMessage rar;
      const uint32_t approvedMsg3Rbs =
        m_resourceManager->AllocateSpectrum (m_myBeamId, msg3RequestedAfterPowerLimit, true);
      if (approvedMsg3Rbs == 0)
        {
          NS_LOG_WARN ("[Msg2] Beam " << m_myBeamId
                       << " 上行预算耗尽，无法为 PreambleID=" << pid
                       << " 生成 Msg3 授权");
          continue;
        }

      const double msg3TxPowerDbm =
        m_resourceManager->AdjustUtTxPower (msg3UtType,
                                            approvedMsg3Rbs,
                                            m_referencePathLossDb);
      rar.raRnti            = 1 + (pid % 0x3FF);  // 简单派生
      rar.preambleId        = pid;
      rar.tcRnti            = AllocateTcRnti ();
      rar.timingAdvance     = MilliSeconds (300); // GEO 单程
      rar.ulGrantRbs        = approvedMsg3Rbs;    // Msg3 使用资源管理器批准的 PRB
      rar.ulGrantMcs        = m_msg3GrantMcs;     // 保守 MCS
      rar.ulGrantTxPowerDbm = msg3TxPowerDbm;
      rar.msg3DelayToStart  = MicroSeconds (500); // 处理延迟
      rar.transmissionTime  = Simulator::Now () + m_rarProcessingDelay;

      NS_LOG_INFO ("[Msg2] 分配 RAR: PreambleID=" << pid
                   << " TC-RNTI=0x" << std::hex << rar.tcRnti << std::dec
                   << " Msg3PowerLimitedRB=" << msg3RequestedAfterPowerLimit
                   << " Msg3RB=" << rar.ulGrantRbs
                   << " Msg3TxPower=" << rar.ulGrantTxPowerDbm << " dBm"
                   << " Msg3UtType=" << static_cast<uint32_t> (msg3UtType)
                   << " ULRemainingAfter=" << m_resourceManager->GetRemainingRbs (m_myBeamId, true)
                   << " (窗口内重叠数=" << count << ")");

      // RAR 调度: 处理时延 + 单程传播
      Simulator::Schedule (m_rarProcessingDelay + m_rarTxDelay,
                           &GeoBeamScheduler::DispatchRar, this, rar);
    }

  m_prachWindowBuf.clear ();
}

void GeoBeamScheduler::DispatchRar (RarMessage rar)
{
  NS_LOG_FUNCTION (this << rar.preambleId << rar.tcRnti);
  NS_LOG_INFO ("[Msg2] 广播 RAR 到 " << m_rarSubscribers.size ()
               << " 个 UE (PreambleID=" << rar.preambleId
               << " TC-RNTI=0x" << std::hex << rar.tcRnti << std::dec << ")");

  // 广播给所有订阅 UE, 由 UE 按 preambleId 自行过滤
  for (auto& cb : m_rarSubscribers)
    {
      if (!cb.IsNull ())
        {
          cb (rar);
        }
    }
}

void GeoBeamScheduler::ReceiveMsg3 (const RrcSetupRequest& req)
{
  NS_LOG_FUNCTION (this << req.tcRnti << req.ueIdentity);

  NS_LOG_INFO ("[Msg3] 收到 RRCSetupRequest: TC-RNTI=0x" << std::hex << req.tcRnti
               << " UE-Id=0x" << req.ueIdentity << std::dec
               << " PreambleIdUsed=" << req.preambleIdUsed
               << " Cause=" << (uint32_t)req.establishmentCause);

  // 按 TC-RNTI 聚合: 若窗口内出现多份相同 TC-RNTI 的 Msg3, 即为竞争解决冲突
  m_msg3WindowBuf[req.tcRnti].push_back (req);

  if (!m_msg3WindowEvent.IsRunning ())
    {
      m_msg3WindowEvent = Simulator::Schedule (m_msg3WindowDuration,
                                               &GeoBeamScheduler::ProcessMsg3Window,
                                               this);
    }
}

void GeoBeamScheduler::ReceiveMsg3Packet (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  if (packet == nullptr)
    {
      NS_LOG_WARN ("[Msg3] 收到空 Packet，无法解包");
      return;
    }

  Msg3MacHeader msg3Header;
  Ptr<Packet> packetCopy = packet->Copy ();
  const uint32_t removedBytes = packetCopy->RemoveHeader (msg3Header);
  if (removedBytes == 0)
    {
      NS_LOG_WARN ("[Msg3] Packet 中未找到 Msg3MacHeader，无法还原 RRCSetupRequest");
      return;
    }

  const RrcSetupRequest req = msg3Header.ToRequest ();
  NS_LOG_INFO ("[Msg3] 已从 Packet 解包 RRCSetupRequest: TC-RNTI=0x"
               << std::hex << req.tcRnti
               << " UE-Id=0x" << req.ueIdentity << std::dec
               << " HeaderBytes=" << removedBytes);
  ReceiveMsg3 (req);
}

void GeoBeamScheduler::ProcessMsg3Window ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("[Msg3] 窗口处理: " << m_msg3WindowBuf.size () << " 个 TC-RNTI 待解码");

  for (auto& kv : m_msg3WindowBuf)
    {
      uint16_t tcRnti = kv.first;
      const auto& reqs = kv.second;

      if (reqs.size () > 1)
        {
          // 同一个 TC-RNTI 收到多份 Msg3 → 物理层上视为叠加/乱码
          // 基站无法正确解出任一 UE 的 Identity → 不发 Msg4
          // 两个 UE 的竞争解决定时器都会超时, 都进入 Msg1 重传
          NS_LOG_WARN ("[Msg3] 竞争冲突! TC-RNTI=0x" << std::hex << tcRnti << std::dec
                       << " 收到 " << reqs.size () << " 份 Msg3, 解码失败, 不发 Msg4");
          continue;
        }

      // 单份 Msg3 → 解码成功, 回显 ueIdentity 完成竞争解决
      const RrcSetupRequest& req = reqs.front ();
      RrcSetupMessage msg4;
      msg4.tcRnti           = tcRnti;
      msg4.echoedUeIdentity = req.ueIdentity;
      // 一般而言 TC-RNTI 直接晋升为 C-RNTI, 这里保持相同值
      msg4.cRnti            = tcRnti;
      msg4.transmissionTime = Simulator::Now () + m_msg4ProcessingDelay;

      NS_LOG_INFO ("[Msg4] 竞争解决成功, 回显 UE-Id=0x" << std::hex << req.ueIdentity
                   << " → C-RNTI=0x" << msg4.cRnti << std::dec);

      // 为成功接入的 UE 建立 UE Context (若尚不存在)
      if (m_ueContextMap.find (msg4.cRnti) == m_ueContextMap.end ())
        {
          AddUeContext (msg4.cRnti);
          AddUeInfo (msg4.cRnti, m_myBeamId);
        }

      Simulator::Schedule (m_msg4ProcessingDelay + m_msg4TxDelay,
                           &GeoBeamScheduler::DispatchMsg4, this, msg4);
    }

  m_msg3WindowBuf.clear ();
}

void GeoBeamScheduler::DispatchMsg4 (RrcSetupMessage msg4)
{
  NS_LOG_FUNCTION (this << msg4.tcRnti);
  NS_LOG_INFO ("[Msg4] 广播 RRCSetup 到 " << m_msg4Subscribers.size ()
               << " 个 UE (TC-RNTI=0x" << std::hex << msg4.tcRnti
               << " echoed UE-Id=0x" << msg4.echoedUeIdentity << std::dec << ")");

  for (auto& cb : m_msg4Subscribers)
    {
      if (!cb.IsNull ())
        {
          cb (msg4);
        }
    }
}

void GeoBeamScheduler::RegisterUeRaCallbacks (Callback<void, const RarMessage&> rarCb,
                                              Callback<void, const RrcSetupMessage&> msg4Cb)
{
  NS_LOG_FUNCTION (this);
  m_rarSubscribers.push_back (rarCb);
  m_msg4Subscribers.push_back (msg4Cb);
}

void GeoBeamScheduler::SetRaConfig (Time prachWindow, Time rarProcessingDelay,
                                    Time rarTxDelay, Time msg4ProcessingDelay, Time msg4TxDelay)
{
  m_prachWindowDuration  = prachWindow;
  m_rarProcessingDelay   = rarProcessingDelay;
  m_rarTxDelay           = rarTxDelay;
  m_msg4ProcessingDelay  = msg4ProcessingDelay;
  m_msg4TxDelay          = msg4TxDelay;
}

uint16_t GeoBeamScheduler::AllocateTcRnti ()
{
  uint16_t rnti = m_tcRntiCounter++;
  if (m_tcRntiCounter == 0xFFF4) {
      m_tcRntiCounter = 0x8001;  // 循环回收
  }
  return rnti;
}

uint32_t GeoBeamScheduler::ProcessUlGrant (uint16_t rnti, uint32_t rbAllocation, uint8_t mcs)
{
  NS_LOG_FUNCTION (this << rnti << rbAllocation << (uint32_t)mcs);
  
  if (m_ueContextMap.find (rnti) == m_ueContextMap.end ()) {
      NS_LOG_WARN ("[UL Grant] UE " << rnti << " 上下文不存在!");
      return 0;
  }
  
  SatUeContext& ctx = m_ueContextMap[rnti];
  const uint32_t beamId = ctx.currentBeamId;
  BeginUlSchedulingPeriod (beamId);
  const uint32_t powerLimitedMaxRbs =
    m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
  const uint32_t requestedAfterPowerLimit = std::min (rbAllocation, powerLimitedMaxRbs);
  if (requestedAfterPowerLimit == 0)
    {
      NS_LOG_WARN ("[UL Grant] UE " << rnti
                   << " 因终端功率上限受限，无法在非削顶条件下分配任何 UL RB");
      return 0;
    }
  const uint32_t remainingBeforeGrant = m_resourceManager->GetRemainingRbs (beamId, true);
  const uint32_t approvedRb =
    m_resourceManager->AllocateSpectrum (beamId, requestedAfterPowerLimit, true);

  if (approvedRb == 0)
    {
      NS_LOG_WARN ("[UL Grant] Beam " << beamId << " 无可用上行RB，授权被拒绝!");
      return 0;
    }
  
  // 构建上行授权DCI
  DciInfo dci;
  dci.isUplinkGrant = true;
  dci.rbAllocation = approvedRb;
  dci.mcs = mcs;
  const double estimatedTxPowerDbm =
    m_resourceManager->AdjustUtTxPower (ctx.utType, approvedRb, m_referencePathLossDb);
  dci.txPowerDbm = estimatedTxPowerDbm;
  dci.delayToStart = MicroSeconds (32);  // 典型的K2延迟
  dci.duration = MilliSeconds (1);
  ctx.pendingUlGrantRbs += approvedRb;
  const uint32_t estimatedGrantBytes =
    std::max (1u, static_cast<uint32_t> (std::llround (EstimateBytesPerRb (ctx.latestCqi) * approvedRb)));
  ctx.pendingUlGrantBytes += estimatedGrantBytes;
  ctx.pendingUlGrantBytes = std::min (ctx.pendingUlGrantBytes, ctx.ulBufferStatus);
  RefreshPendingUlGrantEstimate (ctx);
  ctx.srPending = false;
  ctx.lastUlScheduledTime = Simulator::Now ();
  
  NS_LOG_INFO ("[UL Grant] UE " << rnti << " 上行授权! Beam=" << beamId
               << " | RequestedRB=" << rbAllocation
               << " | PowerLimitedRB=" << requestedAfterPowerLimit
               << " | RemainingBefore=" << remainingBeforeGrant
               << " | ApprovedRB=" << approvedRb
               << " | EffectiveGrantBytes=" << estimatedGrantBytes
               << " | MCS=" << (uint32_t)mcs
               << " | EstTxPower=" << estimatedTxPowerDbm << " dBm"
               << " | UL RemainingAfter=" << m_resourceManager->GetRemainingRbs (beamId, true)
               << " | 当前UL Buffer=" << ctx.ulBufferStatus << " bytes"
               << " | PendingGrantBytes=" << ctx.pendingUlGrantBytes);
  
  // 发送DCI给UE
  SendDciToUe (rnti, dci);
  return approvedRb;
}

// ==================== 辅助函数实现 ====================

void GeoBeamScheduler::BeginUlSchedulingPeriod (uint32_t beamId, uint64_t schedulingRoundId)
{
  auto it = m_ulSchedulingRoundIdByBeam.find (beamId);

  if (schedulingRoundId == 0)
    {
      if (it != m_ulSchedulingRoundIdByBeam.end ())
        {
          NS_LOG_INFO ("[UL TTI] Beam " << beamId
                       << " 复用已有上行调度轮次 "
                       << it->second
                       << " | UL Remaining="
                       << m_resourceManager->GetRemainingRbs (beamId, true) << " RB");
          return;
        }

      schedulingRoundId = ++m_nextUlSchedulingRoundId;
    }

  if (it != m_ulSchedulingRoundIdByBeam.end () && it->second == schedulingRoundId)
    {
      return;
    }

  m_resourceManager->ResetBeamAllocation (beamId, true);
  m_ulSchedulingRoundIdByBeam[beamId] = schedulingRoundId;

  NS_LOG_INFO ("[UL TTI] Beam " << beamId
               << " 开启新的上行调度周期 RoundId=" << schedulingRoundId
               << " | UL 预算重置为 "
               << m_resourceManager->GetRemainingRbs (beamId, true) << " RB");
}

void GeoBeamScheduler::RunUlScheduler ()
{
  for (const auto& beamPair : m_beamToUesMap)
    {
      RunUlSchedulerForBeam (beamPair.first);
    }
}

void GeoBeamScheduler::RunUlSchedulerForBeam (uint32_t beamId)
{
  auto beamIt = m_beamToUesMap.find (beamId);
  if (beamIt == m_beamToUesMap.end ())
    {
      return;
    }

  uint32_t availableRbs = m_resourceManager->GetRemainingRbs (beamId, true);
  if (availableRbs == 0)
    {
      return;
    }

  std::vector<uint16_t> emergencyUes;
  std::vector<uint16_t> voiceUes;
  std::vector<uint16_t> normalUes;
  double maxEmergencyDelay = 0.0;

  for (uint16_t rnti : beamIt->second)
    {
      auto ctxIt = m_ueContextMap.find (rnti);
      if (ctxIt == m_ueContextMap.end ())
        {
          continue;
        }

      SatUeContext& ctx = ctxIt->second;
      const uint32_t effectivePendingBytes = GetEffectiveUlDemandBytes (ctx);
      const bool needsBootstrapGrant =
        ctx.srPending && ctx.ulBufferStatus == 0 && ctx.pendingUlGrantRbs == 0;
      const bool hasPendingUlData = effectivePendingBytes > 0;
      if (!needsBootstrapGrant && !hasPendingUlData)
        {
          continue;
        }

      const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
      const uint32_t requestedRbsRaw = hasPendingUlData ?
        std::max (1u, static_cast<uint32_t> (std::ceil (effectivePendingBytes / bytesPerRb))) :
        m_srGrantRbs;
      const uint32_t powerLimitedMaxRbs =
        m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
      const uint32_t requestedRbs = std::min (requestedRbsRaw, powerLimitedMaxRbs);
      if (requestedRbs == 0)
        {
          continue;
        }
      const uint32_t gateRequiredRbs =
        (ctx.priority == ServicePriority::PRIORITY_EMERGENCY) ? std::min (3u, requestedRbs) :
        (ctx.priority == ServicePriority::PRIORITY_VOICE) ? std::min (2u, requestedRbs) :
        std::max (1u, std::min (requestedRbs, m_srGrantRbs));

      if (m_admitControl != nullptr)
        {
          m_admitControl->UpdateUeContext (rnti, ctx.latestCqi, gateRequiredRbs);
          const AdmitDecision decision =
            m_admitControl->CanAdmitUe (beamId,
                                        ctx.priority,
                                        ctx.utType,
                                        ctx.trafficType,
                                        gateRequiredRbs,
                                        true);
          if (decision != AdmitDecision::ADMIT_OK)
            {
              continue;
            }
        }

      const Time delay = Simulator::Now () - ctx.lastUlScheduledTime;
      if (ctx.priority == ServicePriority::PRIORITY_EMERGENCY)
        {
          emergencyUes.push_back (rnti);
          maxEmergencyDelay = std::max (maxEmergencyDelay, delay.GetSeconds ());
        }
      else if (ctx.priority == ServicePriority::PRIORITY_VOICE)
        {
          voiceUes.push_back (rnti);
        }
      else
        {
          normalUes.push_back (rnti);
        }
    }

  uint32_t emergencyBudget =
    std::min (availableRbs,
              (m_admitControl != nullptr) ? m_admitControl->GetEmergencyReservedRbs () : 1u);
  uint32_t voiceBudget =
    std::min (availableRbs - emergencyBudget,
              (m_admitControl != nullptr) ? m_admitControl->GetVoiceReservedRbs () : 2u);

  if (maxEmergencyDelay > m_emergencyDelayThresholdSeconds && !emergencyUes.empty ())
    {
      emergencyBudget =
        std::min (availableRbs,
                  (m_admitControl != nullptr) ? m_admitControl->GetEmergencyBurstCapRbs () : 3u);
      voiceBudget =
        std::min (availableRbs - emergencyBudget,
                  (m_admitControl != nullptr) ? m_admitControl->GetVoiceReservedRbs () : 2u);
    }

  uint32_t ulTrafficBudgetRemaining = availableRbs;

  auto scheduleUlClassQueue =
    [&] (const std::vector<uint16_t>& classQueue, uint32_t& queueBudget, double urgencyBoost)
    {
      if (classQueue.empty () || queueBudget == 0)
        {
          return;
        }

      std::vector<std::pair<uint16_t, double>> metricQueue;
      for (uint16_t rnti : classQueue)
        {
          SatUeContext& ctx = m_ueContextMap[rnti];
          const uint32_t effectivePendingBytes = GetEffectiveUlDemandBytes (ctx);
          const bool needsBootstrapGrant =
            ctx.srPending && ctx.ulBufferStatus == 0 && ctx.pendingUlGrantRbs == 0;
          const bool hasPendingUlData = effectivePendingBytes > 0;
          if (!needsBootstrapGrant && !hasPendingUlData)
            {
              continue;
            }

          const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
          const uint32_t powerLimitedMaxRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
          if (powerLimitedMaxRbs == 0)
            {
              continue;
            }
          const double demandBytes =
            static_cast<double> (hasPendingUlData ?
                                 std::min<uint32_t> (effectivePendingBytes,
                                                     static_cast<uint32_t> (std::llround (bytesPerRb * powerLimitedMaxRbs))) :
                                 std::min (m_srGrantRbs, powerLimitedMaxRbs) * bytesPerRb);
          const double classMetric =
            CalculateSchedulerMetric (ctx, queueBudget, true, urgencyBoost, demandBytes);

          metricQueue.push_back ({rnti, classMetric});
        }

      std::sort (metricQueue.begin (),
                 metricQueue.end (),
                 [] (const std::pair<uint16_t, double>& a, const std::pair<uint16_t, double>& b) {
                   return a.second > b.second;
                 });

      for (const auto& item : metricQueue)
        {
          if (queueBudget == 0)
            {
              break;
            }

          SatUeContext& ctx = m_ueContextMap[item.first];
          const uint32_t effectivePendingBytes = GetEffectiveUlDemandBytes (ctx);
          const bool needsBootstrapGrant =
            ctx.srPending && ctx.ulBufferStatus == 0 && ctx.pendingUlGrantRbs == 0;
          const bool hasPendingUlData = effectivePendingBytes > 0;
          if (!needsBootstrapGrant && !hasPendingUlData)
            {
              continue;
            }

          const double bytesPerRb = EstimateBytesPerRb (ctx.latestCqi);
          const uint32_t neededRbsRaw = hasPendingUlData ?
            std::max (1u, static_cast<uint32_t> (std::ceil (effectivePendingBytes / bytesPerRb))) :
            m_srGrantRbs;
          const uint32_t powerLimitedMaxRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
          const uint32_t neededRbs = std::min (neededRbsRaw, powerLimitedMaxRbs);
          const uint32_t requestedRbs =
            std::min ({neededRbs, queueBudget, m_resourceManager->GetRemainingRbs (beamId, true)});
          if (requestedRbs == 0)
            {
              continue;
            }

          const uint32_t approvedRbs =
            ProcessUlGrant (item.first, requestedRbs, GetMcsFromCqi (ctx.latestCqi));
          if (approvedRbs == 0)
            {
              continue;
            }

          queueBudget = (queueBudget > approvedRbs) ? (queueBudget - approvedRbs) : 0;
          ulTrafficBudgetRemaining = (ulTrafficBudgetRemaining > approvedRbs) ?
                                     (ulTrafficBudgetRemaining - approvedRbs) :
                                     0;
        }
    };

  scheduleUlClassQueue (emergencyUes, emergencyBudget, 1.5);
  scheduleUlClassQueue (voiceUes, voiceBudget, 1.2);
  uint32_t normalBudget = ulTrafficBudgetRemaining;
  scheduleUlClassQueue (normalUes, normalBudget, 1.0);
}

ServicePriority GeoBeamScheduler::MapTrafficTypeToPriority (TrafficType trafficType)
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

double GeoBeamScheduler::CalculateWrrWeight (ServicePriority priority, UtType utType)
{
  // WRR权重计算: 应急业务权重最高
  double baseWeight = 1.0;
  
  switch (priority) {
    case ServicePriority::PRIORITY_EMERGENCY:
      baseWeight = 8.0;  // 应急业务最高权重
      break;
    case ServicePriority::PRIORITY_VOICE:
      baseWeight = 4.0;  // 语音业务次高
      break;
    case ServicePriority::PRIORITY_DATA:
      baseWeight = 2.0;  // 普通数据
      break;
    case ServicePriority::PRIORITY_BEST_EFFORT:
      baseWeight = 1.0;  // 尽力而为最低
      break;
  }
  
  // 终端类型调整: 消费级终端可以处理更多资源
  double utMultiplier = (utType == UT_CONSUMER) ? 1.2 : 1.0;
  
  return baseWeight * utMultiplier;
}

void GeoBeamScheduler::UpdateUePosition (uint16_t rnti, Vector position)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ()) {
      it->second.position = position;
      NS_LOG_DEBUG ("UE " << rnti << " position updated: (" 
                   << position.x << ", " << position.y << ", " << position.z << ")");
  }
}

void GeoBeamScheduler::SetAdmitControl (Ptr<AdmitControl> admitControl)
{
  m_admitControl = admitControl;
  if (m_admitControl != nullptr)
    {
      m_admitControl->SetResourceManager (m_resourceManager);
      m_admitControl->SetHandoverExecutor (
        MakeCallback (&GeoBeamScheduler::ExecuteHandover, this));
      m_admitControl->SetDlReservedRbs (m_prachReservedRbs);
    }
  NS_LOG_INFO ("AdmitControl connected to scheduler");
}

bool GeoBeamScheduler::CheckHandoverAdmission (uint32_t targetBeamId,
                                               ServicePriority priority,
                                               UtType utType,
                                               TrafficType trafficType,
                                               uint32_t requiredRbs,
                                               bool isUplink)
{
  NS_LOG_FUNCTION (this << targetBeamId << (uint32_t)priority << requiredRbs);
  
  if (m_admitControl == nullptr) {
      NS_LOG_WARN ("AdmitControl not set, allowing handover by default");
      return true;
  }
  
  AdmitDecision decision =
    m_admitControl->CanAdmitUe (targetBeamId,
                                priority,
                                utType,
                                trafficType,
                                requiredRbs,
                                isUplink);
  
  bool admitted = (decision == AdmitDecision::ADMIT_OK || 
                   decision == AdmitDecision::ADMIT_REDIRECT);
  
  NS_LOG_INFO ("Handover admission check: Beam " << targetBeamId 
               << " | Decision: " << (uint32_t)decision
               << " | Admitted: " << admitted);
  
  return admitted;
}

double GeoBeamScheduler::CalculateUeDistance (uint16_t rnti1, uint16_t rnti2)
{
  auto it1 = m_ueContextMap.find (rnti1);
  auto it2 = m_ueContextMap.find (rnti2);
  
  if (it1 == m_ueContextMap.end () || it2 == m_ueContextMap.end ()) {
      return 0.0;
  }
  
  Vector pos1 = it1->second.position;
  Vector pos2 = it2->second.position;
  
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double dz = pos1.z - pos2.z;
  
  return std::sqrt (dx * dx + dy * dy + dz * dz);
}

} // namespace ns3
