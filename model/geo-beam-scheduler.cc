/*
 * 文件路径：contrib/geo-sat/model/geo-beam-scheduler.cc
 * 功能：GEO 卫星波束感知 MAC 层调度器实现 - 融合 RRM 版
 */
#include "geo-beam-scheduler.h"
#include "ns3/simulator.h"
#include "ns3/boolean.h"
#include <algorithm>
#include <cmath>
#include <set>

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
                   MakeTimeChecker ());
  return tid;
}

GeoBeamScheduler::GeoBeamScheduler ()
  : m_prachReservedRbs(0),
    m_wrrCurrentPriority (0),
    m_wrrCurrentIndex (0),
    m_ipfLocationWeight (0.3),
    m_ipfFairnessWeight (0.7),
    m_tcRntiCounter (0x8001),                  // 0x8001..0xFFF3 为 TC-RNTI 区间
    m_prachWindowDuration (MicroSeconds (500)),// PRACH 窗口 (gNB 侧去重)
    m_rarProcessingDelay (MilliSeconds (2)),   // RAR 处理时延
    m_rarTxDelay (MilliSeconds (300)),         // GEO 单程 ~300 ms (下行)
    m_msg3WindowDuration (MicroSeconds (500)), // Msg3 聚合窗口
    m_msg4ProcessingDelay (MilliSeconds (2)),  // Msg4 处理时延
    m_msg4TxDelay (MilliSeconds (300)),        // GEO 单程 ~300 ms (下行)
    m_msgAWindowDuration (MicroSeconds (500)), // MsgA 聚合窗口
    m_msgBProcessingDelay (MilliSeconds (2)),  // MsgB 处理时延
    m_msgBTxDelay (MilliSeconds (300)),        // GEO 单程 ~300 ms (下行)
    m_uplinkPropDelay (MilliSeconds (300))     // GEO 单程 ~300 ms (上行, UE→卫星→gNB)
{
  NS_LOG_FUNCTION (this);
  m_resourceManager = CreateObject<ResourceManager> ();
  m_admitControl = CreateObject<AdmitControl> ();
}

GeoBeamScheduler::~GeoBeamScheduler () { NS_LOG_FUNCTION (this); }

// 空的 5G-LENA 接口占位符 (保持不变)
void GeoBeamScheduler::DoCschedCellConfigReq (const NrMacCschedSapProvider::CschedCellConfigReqParameters& params) {}
void GeoBeamScheduler::DoCschedUeConfigReq (const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) {}
void GeoBeamScheduler::DoCschedLcConfigReq (const NrMacCschedSapProvider::CschedLcConfigReqParameters& params) {}
void GeoBeamScheduler::DoCschedLcReleaseReq (const NrMacCschedSapProvider::CschedLcReleaseReqParameters& params) {}
void GeoBeamScheduler::DoCschedUeReleaseReq (const NrMacCschedSapProvider::CschedUeReleaseReqParameters& params) {}
void GeoBeamScheduler::DoSchedDlRlcBufferReq (const NrMacSchedSapProvider::SchedDlRlcBufferReqParameters& params) {}
void GeoBeamScheduler::DoSchedDlCqiInfoReq (const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlCqiInfoReq (const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlMacCtrlInfoReq (const NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlTriggerReq (const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params) {}
void GeoBeamScheduler::DoSchedUlSrInfoReq (const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params) {}
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
}

void GeoBeamScheduler::AddUeContext (uint16_t rnti, UtType utType, TrafficType trafficType)
{
  if (m_ueContextMap.find(rnti) == m_ueContextMap.end()) {
      SatUeContext ctx;
      ctx.rnti = rnti;
      ctx.latestCqi = 7.0;
      ctx.bufferStatus = 0;
      ctx.averageThroughput = 0.001; 
      ctx.utType = utType;
      ctx.trafficType = trafficType;
      ctx.position = Vector (0, 0, 0);
      ctx.priority = MapTrafficTypeToPriority (trafficType);
      ctx.wrrWeight = CalculateWrrWeight (ctx.priority, utType);
      ctx.lastScheduledTime = Time (0);
      m_ueContextMap[rnti] = ctx;
      NS_LOG_INFO ("UE Context created for RNTI " << rnti 
                   << " (Type: " << utType << ", Traffic: " << trafficType 
                   << ", Priority: " << (uint32_t)ctx.priority << ")");
  }
}

void GeoBeamScheduler::AddUeInfo (uint16_t rnti, uint32_t targetBeamId)
{
  m_ueContextMap[rnti].currentBeamId = targetBeamId;
  m_beamToUesMap[targetBeamId].push_back(rnti);
}

void GeoBeamScheduler::RemoveUt (uint16_t rnti)
{
  uint32_t beamId = m_ueContextMap[rnti].currentBeamId;
  auto& ues = m_beamToUesMap[beamId];
  ues.erase(std::remove(ues.begin(), ues.end(), rnti), ues.end());
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
    hoCtx.unsentBufferBytes = ctx.bufferStatus;
    hoCtx.averageThroughput = ctx.averageThroughput;
    hoCtx.utType = ctx.utType;
    hoCtx.trafficType = ctx.trafficType;
    // 从源波束中彻底抹除该 UE 的痕迹
    RemoveUt (rnti);
    
    NS_LOG_INFO ("📦 成功打包 UE " << rnti << " 的上下文 (未发数据: " 
                 << hoCtx.unsentBufferBytes << " Bytes) 准备切往波束 " << targetBeamId);
                 
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
    ctx.bufferStatus = hoCtx.unsentBufferBytes;
    ctx.averageThroughput = hoCtx.averageThroughput;
    ctx.utType = hoCtx.utType;
    ctx.trafficType = hoCtx.trafficType;

    m_ueContextMap[hoCtx.rnti] = ctx;
    m_beamToUesMap[hoCtx.targetBeamId].push_back(hoCtx.rnti);
    
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

void GeoBeamScheduler::PreProcessRequests () {}

uint8_t GeoBeamScheduler::GetMcsFromCqi (double cqi)
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

void GeoBeamScheduler::RunScheduler ()
{
  NS_LOG_FUNCTION (this);
  ReservePrachResources ();

  for (auto const& beamPair : m_beamToUesMap)
    {
      uint32_t beamId = beamPair.first;
      const std::vector<uint16_t>& ueList = beamPair.second;
      uint32_t availableRbs = 25 - m_prachReservedRbs;

      NS_LOG_INFO ("=== Beam " << beamId << " Scheduling (WRR+IPF) ===");

      // =====================================================================
      // 步骤 1 (对应文档 3.4): 根据 AdmitControl 准入结果，构建逻辑信道队列
      // =====================================================================
      std::vector<uint16_t> emergencyUes;
      std::vector<uint16_t> normalUes;
      double maxEmergencyDelay = 0.0;

      for (uint16_t rnti : ueList) {
          SatUeContext& ctx = m_ueContextMap[rnti];

          if (ctx.priority == ServicePriority::PRIORITY_EMERGENCY ||
              ctx.priority == ServicePriority::PRIORITY_VOICE) {
              emergencyUes.push_back (rnti);

              Time delay = Simulator::Now () - ctx.lastScheduledTime;
              if (delay.GetSeconds () > maxEmergencyDelay) {
                  maxEmergencyDelay = delay.GetSeconds ();
              }
          } else {
              normalUes.push_back (rnti);
          }
      }

      // =====================================================================
      // 步骤 2 (对应文档 3.4): 第一级调度 - 基于优先级的加权轮询 (WRR) 动态提权
      // =====================================================================
      NS_LOG_INFO ("[WRR Stage 1] 正在处理高优先级/应急指挥业务...");

      uint32_t wrrEmergencyBudget = availableRbs / 3;
      double delayThreshold = 0.15;

      if (maxEmergencyDelay > delayThreshold && !emergencyUes.empty()) {
          wrrEmergencyBudget = static_cast<uint32_t>(availableRbs * 0.8);
          NS_LOG_WARN ("[WRR 动态提权] 应急业务时延 (" << maxEmergencyDelay * 1000
                       << " ms) 逼近阈值! 触发确定性保障机制, 强制预留 80% 资源!");
      }

      uint32_t normalBudget = availableRbs - wrrEmergencyBudget;

      for (uint16_t rnti : emergencyUes) {
          if (wrrEmergencyBudget <= 0) break;

          SatUeContext& ctx = m_ueContextMap[rnti];
          uint8_t targetMcs = GetMcsFromCqi (ctx.latestCqi);
          double bytesPerRb = std::max (1.0, ctx.latestCqi * 2.5);

          uint32_t neededRbs = std::ceil (ctx.bufferStatus / bytesPerRb);
          uint32_t schedulerProposedRbs = std::min (neededRbs, wrrEmergencyBudget);

          uint32_t allocatedRb = m_resourceManager->AllocateSpectrum (ctx.utType, schedulerProposedRbs, false);

          wrrEmergencyBudget -= allocatedRb;
          ctx.bufferStatus = (ctx.bufferStatus > allocatedRb * bytesPerRb) ?
                            ctx.bufferStatus - allocatedRb * bytesPerRb : 0;

          ctx.averageThroughput = (1.0 - 0.1) * ctx.averageThroughput + 0.1 * (allocatedRb * bytesPerRb);
          ctx.lastScheduledTime = Simulator::Now ();

          DciInfo dlDci;
          dlDci.isUplinkGrant = false;
          dlDci.rbAllocation = allocatedRb;
          dlDci.mcs = targetMcs;
          dlDci.delayToStart = m_defaultK2Delay;
          dlDci.duration = MilliSeconds (1);

          SendDciToUe (rnti, dlDci);
      }

      // =====================================================================
      // 步骤 3 (对应文档 3.4): 第二级调度 - 改进型比例公平算法 (IPF)
      // =====================================================================
      NS_LOG_INFO ("[IPF Stage 2] 正在针对消费级用户执行精细化 IPF 资源分配...");

      std::vector<std::pair<uint16_t, double>> pfQueue;

      for (uint16_t rnti : normalUes)
        {
          SatUeContext& ctx = m_ueContextMap[rnti];
          if (ctx.bufferStatus == 0) ctx.bufferStatus = 5000;

          if (ctx.bufferStatus > 0)
            {
              double bytesPerRb = std::max(1.0, ctx.latestCqi * 2.5);
              double instRate = bytesPerRb * availableRbs;

              double locationFactor = 1.0;
              double distanceToCenter = std::sqrt(ctx.position.x * ctx.position.x +
                                                  ctx.position.y * ctx.position.y);
              if (distanceToCenter > 0) {
                  locationFactor = 1.0 / (1.0 + 0.1 * distanceToCenter);
              }

              double R = instRate * locationFactor;
              double alpha = 1.0;

              Time timeSinceLastSched = Simulator::Now () - ctx.lastScheduledTime;
              double k_delay = 1.5;
              double delaySensitivityD = std::exp (timeSinceLastSched.GetSeconds () * k_delay);

              double ipfMetric = (std::pow(R, alpha) * delaySensitivityD) /
                                 std::pow(ctx.averageThroughput, m_ipfFairnessWeight);

              pfQueue.push_back({rnti, ipfMetric});
            }
        }

      std::sort(pfQueue.begin(), pfQueue.end(),
                [](const std::pair<uint16_t, double>& a, const std::pair<uint16_t, double>& b) {
                    return a.second > b.second;
                });

      for (auto& item : pfQueue)
        {
          if (normalBudget <= 0) break;

          uint16_t rnti = item.first;
          SatUeContext& ctx = m_ueContextMap[rnti];

          uint8_t targetMcs = GetMcsFromCqi (ctx.latestCqi);
          double bytesPerRb = std::max (1.0, ctx.latestCqi * 2.5);

          uint32_t neededRbs = std::ceil (ctx.bufferStatus / bytesPerRb);
          uint32_t schedulerProposedRbs = std::min (neededRbs, normalBudget);

          uint32_t allocatedRb = m_resourceManager->AllocateSpectrum (ctx.utType, schedulerProposedRbs, false);

          normalBudget -= allocatedRb;
          uint32_t allocatedBytes = allocatedRb * bytesPerRb;

          if (ctx.bufferStatus > allocatedBytes) {
              ctx.bufferStatus -= allocatedBytes;
          } else {
              ctx.bufferStatus = 0;
          }

          ctx.averageThroughput = (1.0 - 0.1) * ctx.averageThroughput + 0.1 * allocatedBytes;
          ctx.lastScheduledTime = Simulator::Now ();

          DciInfo dlDci;
          dlDci.isUplinkGrant = false;
          dlDci.rbAllocation = allocatedRb;
          dlDci.mcs = targetMcs;
          dlDci.delayToStart = m_defaultK2Delay;
          dlDci.duration = MilliSeconds (1);

          SendDciToUe (rnti, dlDci);
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
  UpdateUeCsi (report.ueId, report.rsrp);
}

void GeoBeamScheduler::SendDciToUe (uint16_t ueId, const DciInfo& dci)
{
  // 模拟向下发送
  NS_LOG_INFO ("[DCI] 发送下行控制信息给 UE " << ueId 
               << " | UL Grant: " << dci.isUplinkGrant
               << " | RB: " << dci.rbAllocation 
               << " | MCS: " << (uint32_t)dci.mcs
               << " | Delay: " << dci.delayToStart.GetMilliSeconds () << "ms");
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
      m_ueContextMap[rnti].bufferStatus = bsr.bufferSize;
      NS_LOG_INFO ("[BSR] UE " << rnti << " 缓冲区状态更新: " 
                   << bsr.bufferSize << " bytes"
                   << " | LCG: " << (uint32_t)bsr.lcgId);
  } else {
      NS_LOG_WARN ("[BSR] UE " << rnti << " 上下文不存在，BSR被忽略!");
  }
}

// ==================== 4 步随机接入实现 (基站侧) ====================

void GeoBeamScheduler::ReceivePrachPreamble (const PrachPreamble& preamble)
{
  NS_LOG_FUNCTION (this << preamble.preambleId);

  NS_LOG_INFO ("[Msg1] PRACH 前导码发出! PreambleID=" << preamble.preambleId
               << " | Format=" << (uint32_t)preamble.format
               << " | Retx=" << preamble.isRetransmission
               << " | Time=" << Simulator::Now ().GetSeconds () << "s"
               << " → 上行传播 " << m_uplinkPropDelay.GetMilliSeconds () << "ms");

  // 延迟上行传播时间后再入缓冲 (UE → 卫星 → gNB)
  Simulator::Schedule (m_uplinkPropDelay,
                       &GeoBeamScheduler::DoBufferPreamble, this, preamble);
}

void GeoBeamScheduler::DoBufferPreamble (PrachPreamble preamble)
{
  PreambleArrival arr;
  arr.preambleId  = preamble.preambleId;
  arr.arrivalTime = Simulator::Now ();
  arr.raRnti      = preamble.raRnti;
  m_prachWindowBuf.push_back (arr);

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

  // 按 (raRnti, preambleId) 二元组分组, 区分不同 PRACH occasion 的同一 preambleId
  // key = (raRnti, preambleId)
  std::map<std::pair<uint32_t,uint32_t>, uint32_t> preambleCnt;
  for (const auto& a : m_prachWindowBuf)
    {
      preambleCnt[{a.raRnti, a.preambleId}]++;
    }

  NS_LOG_INFO ("[PRACH] 窗口处理: 共 " << m_prachWindowBuf.size ()
               << " 个前导码, 唯一 (raRnti,PreambleID) 数 = " << preambleCnt.size ());

  // 重要: 即使多个 UE 选用了同一个 preambleId, 基站在 Msg1 阶段
  // 解出的还是同一条前导码 (能量叠加 / 去重), 依然会发一条 RAR。
  // 碰撞会推迟到 Msg3 阶段才被发现——这与实际 4 步 RA 的行为一致。
  for (const auto& kv : preambleCnt)
    {
      uint32_t raRnti = kv.first.first;
      uint32_t pid    = kv.first.second;
      uint32_t count  = kv.second;

      RarMessage rar;
      rar.raRnti            = raRnti;             // 保留 UE 的 RA-RNTI (区分 PRACH occasion)
      rar.preambleId        = pid;
      rar.tcRnti            = AllocateTcRnti ();
      rar.timingAdvance     = MilliSeconds (300); // GEO 单程
      rar.ulGrantRbs        = 6;                  // Msg3 使用 6 PRB
      rar.ulGrantMcs        = 4;                  // 保守 MCS
      rar.msg3DelayToStart  = MicroSeconds (500); // 处理延迟
      rar.transmissionTime  = Simulator::Now () + m_rarProcessingDelay;

      NS_LOG_INFO ("[Msg2] 分配 RAR: RA-RNTI=" << raRnti
                   << " PreambleID=" << pid
                   << " TC-RNTI=0x" << std::hex << rar.tcRnti << std::dec
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

  NS_LOG_INFO ("[Msg3] RRCSetupRequest 发出: TC-RNTI=0x" << std::hex << req.tcRnti
               << " UE-Id=0x" << req.ueIdentity << std::dec
               << " PreambleIdUsed=" << req.preambleIdUsed
               << " → 上行传播 " << m_uplinkPropDelay.GetMilliSeconds () << "ms");

  // 延迟上行传播时间后再入缓冲 (UE → 卫星 → gNB)
  Simulator::Schedule (m_uplinkPropDelay,
                       &GeoBeamScheduler::DoBufferMsg3, this, req);
}

void GeoBeamScheduler::DoBufferMsg3 (RrcSetupRequest req)
{
  NS_LOG_INFO ("[Msg3] gNB 收到 RRCSetupRequest: TC-RNTI=0x" << std::hex << req.tcRnti
               << " UE-Id=0x" << req.ueIdentity << std::dec);

  m_msg3WindowBuf[req.tcRnti].push_back (req);

  if (!m_msg3WindowEvent.IsRunning ())
    {
      m_msg3WindowEvent = Simulator::Schedule (m_msg3WindowDuration,
                                               &GeoBeamScheduler::ProcessMsg3Window,
                                               this);
    }
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

void GeoBeamScheduler::ProcessUlGrant (uint16_t rnti, uint32_t rbAllocation, uint8_t mcs)
{
  NS_LOG_FUNCTION (this << rnti << rbAllocation << (uint32_t)mcs);
  
  if (m_ueContextMap.find (rnti) == m_ueContextMap.end ()) {
      NS_LOG_WARN ("[UL Grant] UE " << rnti << " 上下文不存在!");
      return;
  }
  
  SatUeContext& ctx = m_ueContextMap[rnti];
  
  // 构建上行授权DCI
  DciInfo dci;
  dci.isUplinkGrant = true;
  dci.rbAllocation = rbAllocation;
  dci.mcs = mcs;
  dci.delayToStart = MicroSeconds (32);  // 典型的K2延迟
  dci.duration = MilliSeconds (1);
  
  NS_LOG_INFO ("[UL Grant] UE " << rnti << " 上行授权! RB=" << rbAllocation 
               << " | MCS=" << (uint32_t)mcs
               << " | 当前Buffer=" << ctx.bufferStatus << " bytes");
  
  // 发送DCI给UE
  SendDciToUe (rnti, dci);
}

// ==================== 辅助函数实现 ====================

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
  NS_LOG_INFO ("AdmitControl connected to scheduler");
}

bool GeoBeamScheduler::CheckHandoverAdmission (uint32_t targetBeamId, ServicePriority priority, uint32_t requiredRbs)
{
  NS_LOG_FUNCTION (this << targetBeamId << (uint32_t)priority << requiredRbs);
  
  if (m_admitControl == nullptr) {
      NS_LOG_WARN ("AdmitControl not set, allowing handover by default");
      return true;
  }
  
  AdmitDecision decision = m_admitControl->CanHandoverUe (0, targetBeamId, priority, requiredRbs);
  
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

// ==================== 2 步随机接入实现 (基站侧) ====================

void GeoBeamScheduler::ReceiveMsgA (const MsgA& msgA)
{
  NS_LOG_FUNCTION (this << msgA.preambleId << msgA.ueIdentity);

  NS_LOG_INFO ("[MsgA] MsgA 发出: PreambleID=" << msgA.preambleId
               << " UE-Id=0x" << std::hex << msgA.ueIdentity << std::dec
               << " Retx=" << msgA.isRetransmission
               << " → 上行传播 " << m_uplinkPropDelay.GetMilliSeconds () << "ms");

  // 延迟上行传播时间后再入缓冲 (UE → 卫星 → gNB)
  Simulator::Schedule (m_uplinkPropDelay,
                       &GeoBeamScheduler::DoBufferMsgA, this, msgA);
}

void GeoBeamScheduler::DoBufferMsgA (MsgA msgA)
{
  NS_LOG_INFO ("[MsgA] gNB 收到 MsgA: PreambleID=" << msgA.preambleId
               << " UE-Id=0x" << std::hex << msgA.ueIdentity << std::dec);

  MsgAArrival arr;
  arr.preambleId  = msgA.preambleId;
  arr.ueIdentity  = msgA.ueIdentity;
  arr.arrivalTime = Simulator::Now ();
  arr.raRnti      = msgA.raRnti;
  m_msgAWindowBuf.push_back (arr);

  if (!m_msgAWindowEvent.IsRunning ())
    {
      m_msgAWindowEvent = Simulator::Schedule (m_msgAWindowDuration,
                                                &GeoBeamScheduler::ProcessMsgAWindow,
                                                this);
    }
}

void GeoBeamScheduler::ProcessMsgAWindow ()
{
  NS_LOG_FUNCTION (this);

  // 按 (raRnti, preambleId) 分组, 区分不同 PRACH occasion
  // key = (raRnti, preambleId), value = ueIdentity 列表
  std::map<std::pair<uint32_t,uint32_t>, std::vector<uint64_t>> preambleToUeIds;
  for (const auto& a : m_msgAWindowBuf)
    {
      preambleToUeIds[{a.raRnti, a.preambleId}].push_back (a.ueIdentity);
    }

  NS_LOG_INFO ("[MsgA] 窗口处理: 共 " << m_msgAWindowBuf.size ()
               << " 条 MsgA, 唯一 (raRnti,PreambleID) 数 = " << preambleToUeIds.size ());

  for (const auto& kv : preambleToUeIds)
    {
      uint32_t raRnti = kv.first.first;
      uint32_t pid    = kv.first.second;
      const auto& ueIds = kv.second;

      // 统计唯一 ueIdentity 数量
      std::set<uint64_t> uniqueUeIds (ueIds.begin (), ueIds.end ());

      MsgB msgB;
      msgB.preambleId       = pid;
      msgB.raRnti           = raRnti;              // 保留 RA-RNTI (区分 PRACH occasion)
      msgB.timingAdvance    = MilliSeconds (300);  // GEO 单程
      msgB.transmissionTime = Simulator::Now () + m_msgBProcessingDelay;

      if (uniqueUeIds.size () == 1)
        {
          // 无碰撞 → SUCCESS_RAR: 直接分配 C-RNTI
          uint16_t cRnti = AllocateTcRnti ();  // 复用分配器
          msgB.type              = MsgBType::SUCCESS_RAR;
          msgB.cRnti             = cRnti;
          msgB.echoedUeIdentity  = *uniqueUeIds.begin ();
          msgB.tcRnti            = 0;
          msgB.ulGrantRbs        = 0;
          msgB.ulGrantMcs        = 0;

          NS_LOG_INFO ("[MsgB] SUCCESS_RAR: PreambleID=" << pid
                       << " C-RNTI=0x" << std::hex << cRnti
                       << " UE-Id=0x" << msgB.echoedUeIdentity << std::dec);

          // 为成功接入的 UE 建立 UE Context
          if (m_ueContextMap.find (cRnti) == m_ueContextMap.end ())
            {
              AddUeContext (cRnti);
              AddUeInfo (cRnti, m_myBeamId);
            }
        }
      else
        {
          // 碰撞 → FALLBACK_RAR: 分配 TC-RNTI + UL Grant, 回退到 4 步
          uint16_t tcRnti = AllocateTcRnti ();
          msgB.type              = MsgBType::FALLBACK_RAR;
          msgB.cRnti             = 0;
          msgB.echoedUeIdentity  = 0;
          msgB.tcRnti            = tcRnti;
          msgB.ulGrantRbs        = 6;   // Msg3 使用 6 PRB
          msgB.ulGrantMcs        = 4;   // 保守 MCS

          NS_LOG_WARN ("[MsgB] FALLBACK_RAR (碰撞): PreambleID=" << pid
                       << " 唯一 UE 数=" << uniqueUeIds.size ()
                       << " TC-RNTI=0x" << std::hex << tcRnti << std::dec
                       << " → 回退到 4 步 Msg3/Msg4");
        }

      Simulator::Schedule (m_msgBProcessingDelay + m_msgBTxDelay,
                           &GeoBeamScheduler::DispatchMsgB, this, msgB);
    }

  m_msgAWindowBuf.clear ();
}

void GeoBeamScheduler::DispatchMsgB (MsgB msgB)
{
  NS_LOG_FUNCTION (this << msgB.preambleId << (uint32_t)msgB.type);
  NS_LOG_INFO ("[MsgB] 广播 MsgB 到 " << m_msgBSubscribers.size ()
               << " 个 UE (PreambleID=" << msgB.preambleId
               << " Type=" << (msgB.type == MsgBType::SUCCESS_RAR ? "SUCCESS" : "FALLBACK") << ")");

  for (auto& cb : m_msgBSubscribers)
    {
      if (!cb.IsNull ())
        {
          cb (msgB);
        }
    }
}

void GeoBeamScheduler::RegisterUeTwoStepRaCallbacks (Callback<void, const MsgB&> msgBCb)
{
  NS_LOG_FUNCTION (this);
  m_msgBSubscribers.push_back (msgBCb);
}

void GeoBeamScheduler::SetTwoStepRaConfig (Time msgAWindow, Time msgBProcessingDelay, Time msgBTxDelay)
{
  m_msgAWindowDuration   = msgAWindow;
  m_msgBProcessingDelay  = msgBProcessingDelay;
  m_msgBTxDelay          = msgBTxDelay;
}

} // namespace ns3
