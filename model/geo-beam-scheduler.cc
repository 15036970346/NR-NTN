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
    m_wrrCurrentPriority (0),//预留字段
    m_wrrCurrentIndex (0),//预留字段
    m_ipfLocationWeight (0.3),//IPF 算法中的位置权重 计算 UE 距离波束中心越远时的惩罚程度
    m_ipfFairnessWeight (0.7),//比例公平因子 值越大，越强调公平性；值越小，越强调瞬时速率
    m_emergencyDelayThresholdSeconds (0.15),//应急业务等待超过 0.15 秒时，会触发 WRR 动态提权，让应急业务可用更多 RB
    m_referencePathLossDb (190.0),//上行调度侧默认用 190 dB 作为路径损耗估计
    m_srGrantRbs (1),
    m_srGrantMcs (4),
    m_msg3RequestedRbs (6),
    m_msg3GrantMcs (4),
    m_msg3DefaultUtTypeValue (static_cast<uint32_t> (UT_PORTABLE)),
    m_nextUlSchedulingRoundId (0),
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
{//每一轮 UL 调度开始前，先按 beam 重置上行 RB 预算，再运行上行调度算法
  const uint64_t schedulingRoundId = ++m_nextUlSchedulingRoundId;//标识当前调度周期

  if (m_beamToUesMap.empty ())
    {//对当前 beam 开启一个 UL 调度周期，重置该 beam 的 UL 资源预算
      BeginUlSchedulingPeriod (m_myBeamId, schedulingRoundId);
      return;
    }

  for (const auto& beamPair : m_beamToUesMap)
    {
      BeginUlSchedulingPeriod (beamPair.first, schedulingRoundId);
    }

  RunUlScheduler ();//进入上行调度流程
}
//UE 发送 SR 后触发的调度接口 当 UE 通过 SR 请求上行资源时，调度器可以立即对该 beam 进行 UL grant 分配，而不必等全局调度周期
void GeoBeamScheduler::DoSchedUlSrInfoReq (const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
  for (uint16_t rnti : params.m_srList)//遍历所有发送 SR 的 UE
    {
      auto it = m_ueContextMap.find (rnti);//在 UE 上下文表中查找该 RNTI
      if (it != m_ueContextMap.end ())
        {
          it->second.srPending = true;//表示有待处理的上行调度请求
          if (it->second.currentBeamId != 0)
            {
              BeginUlSchedulingPeriod (it->second.currentBeamId);//为该 UE 所在的 beam 开启上行调度周期
              RunUlSchedulerForBeam (it->second.currentBeamId);//对该 UE 所在 beam 运行一次上行调度
            }
        }
    }
}
void GeoBeamScheduler::DoSchedSetMcs (uint32_t mcs) {}
void GeoBeamScheduler::DoSchedDlRachInfoReq (const NrMacSchedSapProvider::SchedDlRachInfoReqParameters& params) {}
uint8_t GeoBeamScheduler::GetDlCtrlSyms () const { return 1; }
uint8_t GeoBeamScheduler::GetUlCtrlSyms () const { return 1; }
int64_t GeoBeamScheduler::AssignStreams (int64_t stream) { return 0; }
//下行调度触发接口 下行调度核心逻辑集中在 RunScheduler() 中
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
//下行预留 6 RB
void GeoBeamScheduler::ReservePrachResources () 
{ 
  m_prachReservedRbs = 6; //每个 beam 的下行资源中，先预留 6 RB，不给普通 DL 业务使用。
  if (m_admitControl != nullptr)//判断准入控制模块是否存在
    {
      m_admitControl->SetDlReservedRbs (m_prachReservedRbs);//把 6 RB 预留值同步给 AdmitControl
    }
}
//UE 上下文创建与 beam 绑定
//创建 UE 调度上下文
void GeoBeamScheduler::AddUeContext (uint16_t rnti, UtType utType, TrafficType trafficType)
{
  if (m_ueContextMap.find(rnti) == m_ueContextMap.end()) {//检查这个 UE 是否已经存在。如果已经存在，就不重复创建
      SatUeContext ctx;
      ctx.rnti = rnti;
      ctx.latestDlCqi = 7.0;//默认 DL CQI 设置为 7
      ctx.latestUlCqi = 7.0;//默认 UL CQI 设置为 7
      ctx.latestRsrp = -120.0;//默认 RSRP 设置为 -120 dBm
      ctx.dlBufferStatus = 0;//下行 buffer 初始为 0，表示暂时没有待发 DL 数据
      ctx.ulBufferStatus = 0;//上行 buffer 初始为 0
      ctx.pendingUlGrantRbs = 0;//当前已授权但尚未实际到达的 UL grant RB 数初始化为 0
      ctx.pendingUlGrantBytes = 0;//当前已授权但尚未实际传输完成的 UL grant 字节数初始化为 0
      ctx.srPending = false;//SR pending 初始为 false，表示 UE 当前没有待处理的调度请求
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
//把 UE 加入某个 beam  完成 UE 与 beam 的绑定，同时让准入控制模块知道这个 UE 属于哪个 beam
void GeoBeamScheduler::AddUeInfo (uint16_t rnti, uint32_t targetBeamId)
{
  SatUeContext& ctx = m_ueContextMap[rnti];//通过 RNTI 取出 UE 上下文
  ctx.currentBeamId = targetBeamId;//把 UE 当前 beam 设置为目标 beam
  m_beamToUesMap[targetBeamId].push_back(rnti);

  if (m_admitControl != nullptr)
    {//把 UE 注册到 AdmitControl，同步 UE 的 RNTI、beam、优先级、终端类型、业务类型和位置
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
//波束切换与上下文迁移
//导出 UE 上下文  切换时不丢业务状态，保证 DL/UL buffer、CQI、吞吐量、DCI 回调都能带到新 beam
HandoverUeContext GeoBeamScheduler::ExportUeContext (uint16_t rnti, uint32_t targetBeamId)
{
    NS_LOG_FUNCTION (this << rnti << targetBeamId);
    
    // 提取当前 UE 的内存快照
    SatUeContext& ctx = m_ueContextMap[rnti];
    HandoverUeContext hoCtx;
    hoCtx.rnti = rnti;
    hoCtx.sourceBeamId = ctx.currentBeamId;
    hoCtx.targetBeamId = targetBeamId;
    hoCtx.latestDlCqi = ctx.latestDlCqi;
    hoCtx.latestUlCqi = ctx.latestUlCqi;
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
    
    NS_LOG_INFO ("成功打包 UE " << rnti << " 的上下文 (未发DL="
                 << hoCtx.unsentDlBufferBytes << " Bytes, 未发UL="
                 << hoCtx.unsentUlBufferBytes << " Bytes) 准备切往波束 " << targetBeamId);
                 
    return hoCtx;
}

// 导入 UE 上下文  完成 UE 从源 beam 到目标 beam 的状态重建
void GeoBeamScheduler::ImportUeContext (const HandoverUeContext& hoCtx)
{
    NS_LOG_FUNCTION (this << hoCtx.rnti << hoCtx.targetBeamId);
    
    // 在目标波束重建 UE 的调度档案
    SatUeContext ctx;
    ctx.rnti = hoCtx.rnti;
    ctx.currentBeamId = hoCtx.targetBeamId;
    ctx.latestDlCqi = hoCtx.latestDlCqi;
    ctx.latestUlCqi = hoCtx.latestUlCqi;
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
    
    NS_LOG_INFO ("成功在波束 " << hoCtx.targetBeamId << " 注入 UE " << hoCtx.rnti 
                 << " 的上下文，调度器可直接恢复业务传输！");
}

// 执行切换
void GeoBeamScheduler::ExecuteHandover (uint16_t rnti, uint32_t targetBeamId)
{
    NS_LOG_INFO ("===  发起星地波束间无缝切换 (UE: " << rnti << " -> Beam: " << targetBeamId << ") ===");
    
    
    HandoverUeContext hoCtx = ExportUeContext (rnti, targetBeamId);//导出 UE 在源 beam 的上下文
    ImportUeContext (hoCtx);//把该上下文导入目标 beam
}
//更新 CQI 和 Buffer 为 WRR/IPF 提供实时调度输入，包括 CQI、DL buffer、UL buffer 和待完成 UL grant
void GeoBeamScheduler::UpdateUeCsi (uint16_t rnti, double cqi)//兼容旧接口：共享 CQI 同时更新 DL/UL
{
  if (m_ueContextMap.find(rnti) != m_ueContextMap.end()) {//确认 UE 存在
      m_ueContextMap[rnti].latestDlCqi = cqi;
      m_ueContextMap[rnti].latestUlCqi = cqi;
  }
}

void GeoBeamScheduler::UpdateUeDlCqi (uint16_t rnti, double cqi)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.latestDlCqi = cqi;
    }
}

void GeoBeamScheduler::UpdateUeUlCqi (uint16_t rnti, double cqi)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.latestUlCqi = cqi;
    }
}

void GeoBeamScheduler::UpdateUeDlBufferStatus (uint16_t rnti, uint32_t bufferBytes)//定义更新下行 buffer 的函数
{
  auto it = m_ueContextMap.find (rnti);//查找 UE
  if (it != m_ueContextMap.end ())
    {
      it->second.dlBufferStatus = bufferBytes;
    }
}

void GeoBeamScheduler::UpdateUeUlBufferStatus (uint16_t rnti, uint32_t bufferBytes)//定义更新上行 buffer 的函数
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.ulBufferStatus = bufferBytes;//更新该 UE 的上行待发字节数
      it->second.pendingUlGrantBytes = std::min (it->second.pendingUlGrantBytes, bufferBytes);
      RefreshPendingUlGrantEstimate (it->second);//重新估算 pending grant 对应的 RB 数
    }
}

void GeoBeamScheduler::PreProcessRequests () {}
//CQI 映射 MCS
uint8_t GeoBeamScheduler::GetMcsFromCqi (double cqi) const//定义 CQI 到 MCS 的映射函数
{
  double effectiveCqi = std::max(1.0, cqi - 0.5); //CQI 减 0.5，保守处理信道质量
  uint8_t mcs = 0;//初始化 MCS 为 0
  int cqiIndex = std::round(effectiveCqi);//CQI 整数索引

  switch (cqiIndex)//根据 CQI index 选择 MCS。CQI 越高，MCS 越高
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
//估算每个 RB 可承载字节数  把 CQI/MCS 转换为资源需求估计
double
GeoBeamScheduler::EstimateBytesPerRb (double cqi) const
{
  const uint8_t mcs = GetMcsFromCqi (cqi);//先根据 CQI 得到 MCS
  // Approximate NR transport efficiency per PRB per slot using a compact
  // spectral-efficiency table. This is intentionally lighter than a full TBS
  // calculation, but avoids the previous overly conservative byte estimates.
  static const double kSpectralEfficiencyByMcs[29] = {//定义 MCS 到频谱效率的简化表
    0.2344, 0.3066, 0.3770, 0.4902, 0.6016, 0.7402, 0.8770, 1.0273, 1.1758, 1.3262,
    1.4766, 1.6953, 1.9141, 2.1602, 2.4063, 2.5703, 2.7305, 3.0293, 3.3223, 3.6094,
    3.9023, 4.2129, 4.5234, 4.8164, 5.1152, 5.3320, 5.5547, 6.2266, 7.4063};
  const uint8_t clampedMcs = std::min<uint8_t> (mcs, 28);
  // 12 subcarriers x 12 data symbols ~= 144 data RE/PRB/slot after DMRS/control overhead.
  const double dataRePerRb = 144.0;//设置每个 RB 的有效资源元素约为 144  简化估计
  return std::max (1.0, kSpectralEfficiencyByMcs[clampedMcs] * dataRePerRb / 8.0);//计算 bytes per RB 每 RB 字节数 = 频谱效率 × 有效 RE 数 / 8
}
//位置因子
double
GeoBeamScheduler::CalculateLocationFactor (const Vector& position) const
{//计算 UE 到波束中心的二维距离
  const double distanceToCenter = std::sqrt (position.x * position.x + position.y * position.y);
  //如果距离为 0，说明 UE 在波束中心，位置因子为 1，不惩罚
  if (distanceToCenter <= 0.0)
    {
      return 1.0;
    }
//距离越远，rawPenalty 越小，表示边缘 UE 的位置条件较差
  const double rawPenalty = 1.0 / (1.0 + 0.1 * distanceToCenter);
  return (1.0 - m_ipfLocationWeight) + m_ipfLocationWeight * rawPenalty;
}
//IPF 调度指标
double
//定义 IPF 指标函数。输入包括 UE 上下文、当前队列预算、上下行方向、紧急度增益和需求字节数
GeoBeamScheduler::CalculateSchedulerMetric (const SatUeContext& ctx,
                                            uint32_t queueBudget,
                                            bool isUplink,
                                            double urgencyBoost,
                                            double demandBytes) const
{
  const double schedulingCqi = GetSchedulingCqi (ctx, isUplink);
  const double bytesPerRb = EstimateBytesPerRb (schedulingCqi);//根据 CQI 估算每 RB 字节数
  const double instRate = bytesPerRb * std::max (1u, queueBudget);//计算瞬时速率近似值。预算越多、CQI 越好，瞬时速率越高
  const double locationFactor = CalculateLocationFactor (ctx.position);//位置因子
  //距离上次被调度过去了多久
  const Time timeSinceLastSched =
    Simulator::Now () - (isUplink ? ctx.lastUlScheduledTime : ctx.lastDlScheduledTime);
  const double delaySensitivity = std::exp (timeSinceLastSched.GetSeconds () * 1.5);//时延敏感因子 等待越久，指数值越大，UE 的调度优先级越高
  //平均吞吐量
  const double avgThroughput =
    isUplink ? ctx.ulAverageThroughput : ctx.dlAverageThroughput;
  const double demandWeight = 1.0 + std::log1p (std::max (0.0, demandBytes)) / 8.0;//需求权重 buffer 越大，需求权重越高
  const double cqiBoost = 1.0 + std::max (1.0, schedulingCqi) / 10.0;//CQI 增益 CQI 越高，调度指标越高
//返回最终 IPF 指标=瞬时速率 × 位置因子 × 等待时延因子 × 业务紧急度 × WRR权重 × 需求权重 × CQI增益÷ 平均吞吐量^公平因子
  return (instRate * locationFactor * delaySensitivity * urgencyBoost * ctx.wrrWeight * demandWeight * cqiBoost) /
         std::pow (avgThroughput, m_ipfFairnessWeight);
}
//计算有效 UL 需求  防止 GEO 长时延场景下，调度器因为旧 BSR 未更新而重复发太多 UL grant
uint32_t
GeoBeamScheduler::GetEffectiveUlDemandBytes (const SatUeContext& ctx) const
{//有效需求 = UL buffer - 已经授权但还没完成的字节数
  return (ctx.ulBufferStatus > ctx.pendingUlGrantBytes) ?
           (ctx.ulBufferStatus - ctx.pendingUlGrantBytes) :
           0u;
}

void//定义刷新 pending grant RB 估计的函数
GeoBeamScheduler::RefreshPendingUlGrantEstimate (SatUeContext& ctx) const
{
  ctx.pendingUlGrantBytes = std::min (ctx.pendingUlGrantBytes, ctx.ulBufferStatus);//pending grant 字节数不能超过当前 UL buffer
  //如果 pending grant 为 0，就把 pending RB 也清零
  if (ctx.pendingUlGrantBytes == 0)
    {
      ctx.pendingUlGrantRbs = 0;
      return;
    }
//根据 CQI 估算每 RB 字节数
  const double bytesPerRb = EstimateBytesPerRb (GetSchedulingCqi (ctx, true));
//把 pending grant 字节数换算成 RB 数
  ctx.pendingUlGrantRbs =
    std::max (1u, static_cast<uint32_t> (std::ceil (ctx.pendingUlGrantBytes / bytesPerRb)));
}

double
GeoBeamScheduler::GetSchedulingCqi (const SatUeContext& ctx, bool isUplink) const
{
  const double primaryCqi = isUplink ? ctx.latestUlCqi : ctx.latestDlCqi;
  const double fallbackCqi = isUplink ? ctx.latestDlCqi : ctx.latestUlCqi;
  return std::max (1.0, primaryCqi > 0.0 ? primaryCqi : fallbackCqi);
}
//下行 WRR + IPF 主流程
//下行调度初始化  每个 beam 独立开启一轮 DL 调度，并先扣除下行保护资源
void GeoBeamScheduler::RunScheduler ()//下行调度主函数
{
  NS_LOG_FUNCTION (this);
  ReservePrachResources ();//预留 PRACH

  for (auto const& beamPair : m_beamToUesMap)//遍历每一个 beam
    {
      uint32_t beamId = beamPair.first;//取出当前 beam ID
      const std::vector<uint16_t>& ueList = beamPair.second;//取出当前 beam 下所有 UE 的列表
      std::vector<uint16_t> ueScheduleOrder;//准备最终的 UE 调度顺序表
      std::vector<std::pair<uint16_t, Time>> queuedUes;//准备等待队列 UE 列表
      std::vector<uint16_t> freshUes;//准备新 UE 或未排队 UE 列表
      std::vector<std::pair<uint16_t, uint32_t>> redirectActions;//准备重定向动作列表

      for (uint16_t rnti : ueList)
        {
          auto queueIt = m_admissionQueueSince.find (rnti);//查找该 UE 是否在准入等待队列中
          //如果该 UE 已经排队，就加入 queuedUes，并保留排队时间
          if (queueIt != m_admissionQueueSince.end ())
            {
              queuedUes.push_back ({rnti, queueIt->second});
            }
          //如果没有排队，就加入新 UE 列表
          else
            {
              freshUes.push_back (rnti);
            }
        }
      //对排队 UE 按进入队列时间排序，排得越早越靠前
      std::sort (queuedUes.begin (),
                 queuedUes.end (),
                 [] (const std::pair<uint16_t, Time>& a, const std::pair<uint16_t, Time>& b) {
                   return a.second < b.second;
                 });
      //给最终调度顺序列表预留空间
      ueScheduleOrder.reserve (ueList.size ());
      //把排队 UE 放入调度顺序
      for (const auto& queuedUe : queuedUes)
        {
          ueScheduleOrder.push_back (queuedUe.first);
        }
      //把普通 UE 接到后面
      ueScheduleOrder.insert (ueScheduleOrder.end (), freshUes.begin (), freshUes.end ());
//重置该 beam 的下行 RB 使用记录 每个 beam 每轮 DL 调度都重新从预算开始
      m_resourceManager->ResetBeamAllocation (beamId, false);
      //查询该 beam 当前下行物理剩余 RB
      const uint32_t physicalRemainingDlRbs = m_resourceManager->GetRemainingRbs (beamId, false);
      //扣除 PRACH 预留 6 RB，得到真正可用于 DL 业务调度的 RB 数
      uint32_t availableRbs =
        (physicalRemainingDlRbs > m_prachReservedRbs) ? (physicalRemainingDlRbs - m_prachReservedRbs) : 0;

      NS_LOG_INFO ("=== Beam " << beamId << " Scheduling (WRR+IPF) ===");

      // =====================================================================
      // 步骤 1 : 根据 AdmitControl 准入结果，构建逻辑信道队列 先通过准入控制筛掉不能调度的 UE，再按业务优先级构建 WRR 的三个逻辑队列
      // =====================================================================
      //创建三个业务队列：应急、语音、普通数据
      std::vector<uint16_t> emergencyUes;
      std::vector<uint16_t> voiceUes;
      std::vector<uint16_t> normalUes;
      //记录应急和语音业务最大等待时延，用于动态提权
      double maxEmergencyDelay = 0.0;
      double maxVoiceDelay = 0.0;
    
      //按前面形成的 UE 调度顺序遍历 UE
      for (uint16_t rnti : ueScheduleOrder) {
          SatUeContext& ctx = m_ueContextMap[rnti];//取出 UE 上下文
          const uint32_t bufferedBytes = ctx.dlBufferStatus;//读取 UE 的下行待发 buffer
          //没有 DL 数据 跳过
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
          //根据 CQI 估算每 RB 可传输字节数
          const double bytesPerRb = EstimateBytesPerRb (GetSchedulingCqi (ctx, false));
          //根据 buffer 大小估算该 UE 理论需要多少 RB
          const uint32_t rawRequiredRbs =
            std::max (1u, static_cast<uint32_t> (std::ceil (bufferedBytes / bytesPerRb)));
          //计算准入控制使用的 gateRequiredRbs
          const uint32_t gateRequiredRbs =
            (ctx.priority == ServicePriority::PRIORITY_EMERGENCY) ? std::min (3u, rawRequiredRbs) :
            (ctx.priority == ServicePriority::PRIORITY_VOICE) ? std::min (2u, rawRequiredRbs) :
            1u;

          if (m_admitControl != nullptr)//如果准入控制模块存在，就执行准入判断
            {
              m_admitControl->UpdateUeContext (rnti,
                                               ctx.latestDlCqi,
                                               ctx.latestUlCqi,
                                               gateRequiredRbs);//把 UE 当前 DL/UL CQI 和 RB 需求同步给准入控制
              //调用 CanAdmitUe() 判断当前 beam 是否允许该 UE 参与调度
              const AdmitDecision decision =
                m_admitControl->CanAdmitUe (beamId, ctx.priority, ctx.utType, ctx.trafficType,
                                            gateRequiredRbs, false);

              if (decision == AdmitDecision::ADMIT_QUEUE)//如果返回 ADMIT_QUEUE，就把 UE 加入等待队列，本轮不调度
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
              //如果返回 ADMIT_REDIRECT，就向 AdmitControl 请求推荐 beam，并尝试找到能接纳该 UE 的目标 beam 找到，就记录重定向动作；找不到，则加入等待队列
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

              if (decision != AdmitDecision::ADMIT_OK)//如果不是 ADMIT_OK，直接跳过本 TTI
                {
                  NS_LOG_INFO ("[Admission Gate] Beam " << beamId
                               << " | UE " << rnti
                               << " | Decision=" << static_cast<uint32_t> (decision)
                               << " | RequiredRBs=" << gateRequiredRbs
                               << " -> skip this TTI");
                  continue;
                }

              auto queueIt = m_admissionQueueSince.find (rnti);
              //如果 UE 原来在等待队列，但现在准入成功，就从等待队列移除
              if (queueIt != m_admissionQueueSince.end ())
                {
                  const double queuedMs = (Simulator::Now () - queueIt->second).GetMilliSeconds ();
                  NS_LOG_INFO ("[Admission Queue Exit] Beam " << beamId
                               << " | UE " << rnti
                               << " | Waited=" << queuedMs << " ms");
                  m_admissionQueueSince.erase (queueIt);
                }
            }
          
            //计算该 UE 距离上次 DL 调度的等待时间
          Time delay = Simulator::Now () - ctx.lastDlScheduledTime;
          //应急业务，加入应急队列，并更新应急最大等待时间
          if (ctx.priority == ServicePriority::PRIORITY_EMERGENCY)
            {
              emergencyUes.push_back (rnti);
              maxEmergencyDelay = std::max (maxEmergencyDelay, delay.GetSeconds ());
            }
            //语音业务，加入语音队列，并更新语音最大等待时间
          else if (ctx.priority == ServicePriority::PRIORITY_VOICE)
            {
              voiceUes.push_back (rnti);
              maxVoiceDelay = std::max (maxVoiceDelay, delay.GetSeconds ());
            }
            //其他业务加入普通数据队列
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
      // 步骤 2 : 第一级调度 基于优先级的加权轮询 (WRR) 动态提权 按业务类型切出确定性保障资源，应急和语音优先得到资源预算；当应急等待过久时，应急预算可动态提高
      // =====================================================================
      //计算 emergency / voice 预算
      NS_LOG_INFO ("[WRR Stage 1] 正在按业务类型切片处理 emergency/voice 队列...");
     //计算应急业务预算
      uint32_t emergencyBudget =//优先读取 AdmitControl 中的应急预留 RB
        std::min (availableRbs,
                  (m_admitControl != nullptr) ?
                  m_admitControl->GetEmergencyReservedRbs () :
                  1u);//没有准入控制，就默认 1 RB
     //语音业务预算
      uint32_t voiceBudget =
        std::min (availableRbs - emergencyBudget,//扣除应急预算
                  (m_admitControl != nullptr) ?
                  m_admitControl->GetVoiceReservedRbs () :
                  2u);//给语音业务预留2RB
      //读取应急业务等待阈值
      const double delayThreshold = m_emergencyDelayThresholdSeconds;

      //如果应急 UE 等待时间超过阈值，并且应急队列不为空，则触发动态提权
      if (maxEmergencyDelay > delayThreshold && !emergencyUes.empty()) {
          emergencyBudget =//提高应急预算到 3 RB
            std::min (availableRbs,
                      (m_admitControl != nullptr) ?
                      m_admitControl->GetEmergencyBurstCapRbs () :
                      3u);
          voiceBudget =//重新计算语音预算
            std::min (availableRbs - emergencyBudget,
                      (m_admitControl != nullptr) ?
                      m_admitControl->GetVoiceReservedRbs () :
                      2u);
          NS_LOG_WARN ("[WRR 动态提权] 应急业务时延 (" << maxEmergencyDelay * 1000
                       << " ms) 逼近阈值! 触发确定性保障机制, 应急预算提升到 "
                       << emergencyBudget << " RB!");
      }

      uint32_t trafficBudgetRemaining = availableRbs;//初始化总业务剩余预算为 availableRbs

      //定义内部函数 调度某一类业务队列
      auto scheduleClassQueue =
        [&] (const std::vector<uint16_t>& classQueue,
             uint32_t& queueBudget,
             const char* queueLabel,
             double urgencyBoost)
        {
          //如果该类队列为空，或者预算为 0，直接返回
          if (classQueue.empty () || queueBudget == 0)
            {
              return;
            }

          //创建 metricQueue，用于保存每个 UE 的 IPF 指标
          std::vector<std::pair<uint16_t, double>> metricQueue;
          for (uint16_t rnti : classQueue)//遍历该业务队列中的所有 UE
            {
              SatUeContext& ctx = m_ueContextMap[rnti];//取出 UE 上下文
              //如果没有 DL buffer，就跳过
              if (ctx.dlBufferStatus == 0)
                {
                  continue;
                }

                //综合 CQI、位置、等待时延、WRR 权重、需求大小、平均吞吐量 计算 UE 的 IPF 指标
              const double classMetric =
                CalculateSchedulerMetric (ctx,
                                          queueBudget,
                                          false,
                                          urgencyBoost,
                                          static_cast<double> (ctx.dlBufferStatus));

              metricQueue.push_back ({rnti, classMetric});//把 RNTI 和指标放入 metricQueue
            }

            //按 IPF 指标从大到小排序。指标越高越先调度
          std::sort (metricQueue.begin (),
                     metricQueue.end (),
                     [] (const std::pair<uint16_t, double>& a, const std::pair<uint16_t, double>& b) {
                       return a.second > b.second;
                     });

          //按排序结果依次调度 UE
          for (const auto& item : metricQueue)
            {
              if (queueBudget == 0)//业务队列预算用完 停止
                {
                  break;
                }

              const uint16_t rnti = item.first;//取当前 UE 的 RNTI
              SatUeContext& ctx = m_ueContextMap[rnti];//UE 的上下文
              const double dlCqi = GetSchedulingCqi (ctx, false);
              const uint8_t targetMcs = GetMcsFromCqi (dlCqi);//根据 CQI 计算目标 MCS
              const double bytesPerRb = EstimateBytesPerRb (dlCqi);//算每 RB 可承载字节数
              //根据 DL buffer 估算该 UE需要多少 RB
              const uint32_t neededRbs =
                std::max (1u, static_cast<uint32_t> (std::ceil (ctx.dlBufferStatus / bytesPerRb)));
              const uint32_t schedulerProposedRbs = std::min (neededRbs, queueBudget);//调度器提出的 RB 数不能超过该业务队列预算
              //调用 ResourceManager::AllocateSpectrum() 进行真正的资源审批
              const uint32_t allocatedRb =
                m_resourceManager->AllocateSpectrum (beamId, schedulerProposedRbs, false);

              //审批结果是 0 跳过该 UE
              if (allocatedRb == 0)
                {
                  continue;
                }

              //扣减该业务队列预算
              queueBudget -= allocatedRb;
              //扣减总业务剩余预算
              trafficBudgetRemaining = (trafficBudgetRemaining > allocatedRb) ?
                                       (trafficBudgetRemaining - allocatedRb) :
                                       0;
              //估算这次分配可以发送多少字节
              const uint32_t allocatedBytes = allocatedRb * bytesPerRb;
              //从 UE 的 DL buffer 中扣除已发送字节
              ctx.dlBufferStatus = (ctx.dlBufferStatus > allocatedBytes) ?
                                (ctx.dlBufferStatus - allocatedBytes) :
                                0;
              //用指数平均更新 UE 的下行平均吞吐量
              ctx.dlAverageThroughput =
                (1.0 - 0.1) * ctx.dlAverageThroughput + 0.1 * allocatedBytes;
              ctx.lastDlScheduledTime = Simulator::Now ();//更新该 UE 上次下行调度时间

            //创建 DCI
              DciInfo dlDci;
              dlDci.isUplinkGrant = false;//下行授权
              dlDci.rbAllocation = allocatedRb;//写入 RB 分配数量
              dlDci.mcs = targetMcs;//写入 MCS
              dlDci.txPowerDbm = 0.0;//下行 DCI UE 发射功率为 0
              dlDci.delayToStart = m_defaultK2Delay;//调度延迟
              dlDci.duration = MilliSeconds (1);//授权持续时间1ms

              NS_LOG_INFO ("[IPF Stage 2] Beam " << beamId
                           << " | Queue=" << queueLabel
                           << " | UE=" << rnti
                           << " | MetricP=" << item.second
                           << " | Need=" << neededRbs
                           << " | Proposed=" << schedulerProposedRbs
                           << " | Allocated=" << allocatedRb);
              SendDciToUe (rnti, dlDci);//把 DCI 发给 UE
            }
        };

      scheduleClassQueue (emergencyUes, emergencyBudget, "emergency", 1.5);//先调度应急队列，紧急度增益为 1.5

      //语音等待时间超过阈值，输出观察日志
      if (maxVoiceDelay > delayThreshold && !voiceUes.empty ())
        {
          NS_LOG_INFO ("[WRR 动态观察] 语音业务最大等待时延="
                       << maxVoiceDelay * 1000 << " ms");
        }

      scheduleClassQueue (voiceUes, voiceBudget, "voice", 1.2);//调度语音队列，紧急度增益为 1.2
      uint32_t normalBudget = trafficBudgetRemaining;//普通数据使用剩余所有业务预算

      // =====================================================================
      // 步骤 3 : 第二级调度 - 改进型比例公平算法 (IPF)
      // =====================================================================
      NS_LOG_INFO ("[IPF Stage 2] 正在对各业务队列执行同类用户优先级 P 排序...");
      scheduleClassQueue (normalUes, normalBudget, "data-best-effort", 1.0);//调度普通数据队列，紧急度增益为 1.0

      //输出当前 beam 下行剩余 RB
      NS_LOG_INFO ("=== Beam " << beamId << " Scheduling Complete | DL Remaining="
                   << m_resourceManager->GetRemainingRbs (beamId, false) << " RBs ===");

      //执行之前记录的重定向动作，把 UE 切换到推荐 beam
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
        UpdateUeDlCqi (pucchInfo.rnti, pucchInfo.cqi);
        break;
        
      case PucchFormatType::FORMAT_2:
        NS_LOG_INFO ("[PUCCH] UE " << pucchInfo.rnti << " HARQ ACK/NACK收到! ACK=" 
                     << pucchInfo.harqAck << " Bitmap=" << (uint32_t)pucchInfo.harqBitMap);
        break;
        
      case PucchFormatType::FORMAT_3:
        NS_LOG_INFO ("[PUCCH] UE " << pucchInfo.rnti << " HARQ+CSI收到!");
        UpdateUeDlCqi (pucchInfo.rnti, pucchInfo.cqi);
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
  arr.utType      = static_cast<UtType> (preamble.utType);
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
  BeginUlSchedulingPeriod (m_myBeamId);

  // 按 (RA-RNTI, preambleId) 统计窗口内的到达次数，避免跨 PRACH occasion 误合并。
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> preambleCnt;
  std::map<std::pair<uint32_t, uint32_t>, UtType> preambleUtType;
  for (const auto& a : m_prachWindowBuf)
    {
      const std::pair<uint32_t, uint32_t> key = {a.raRnti, a.preambleId};
      preambleCnt[key]++;
      auto utIt = preambleUtType.find (key);
      if (utIt == preambleUtType.end ())
        {
          preambleUtType[key] = a.utType;
        }
      else if (a.utType == UT_PORTABLE)
        {
          // 若同一前导码对应多个 UE，保守地采用能力更受限的 portable 终端做 Msg3 功控。
          preambleUtType[key] = UT_PORTABLE;
        }
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

      const std::pair<uint32_t, uint32_t> key = kv.first;
      const UtType msg3UtType =
        (preambleUtType.find (key) != preambleUtType.end ()) ?
        preambleUtType.at (key) :
        static_cast<UtType> (m_msg3DefaultUtTypeValue);
      const uint32_t msg3PowerLimitedRbs =
        m_resourceManager->GetMaxPowerLimitedUlRbs (msg3UtType, m_referencePathLossDb);
      // RA 使用专用控制资源池, 不阻塞于业务 UL 预算 (让碰撞在 Msg3 阶段被检测)
      uint32_t msg3RequestedAfterPowerLimit =
        std::min (m_msg3RequestedRbs, msg3PowerLimitedRbs);
      if (msg3RequestedAfterPowerLimit == 0)
        {
          NS_LOG_WARN ("[Msg2] PreambleID=" << pid
                       << " 终端功率上限受限, 回退使用配置 grant=" << m_msg3RequestedRbs << " RB");
          msg3RequestedAfterPowerLimit = m_msg3RequestedRbs;
        }
      RarMessage rar;
      uint32_t approvedMsg3Rbs =
        m_resourceManager->AllocateSpectrum (m_myBeamId, msg3RequestedAfterPowerLimit, true);
      if (approvedMsg3Rbs == 0)
        {
          NS_LOG_WARN ("[Msg2] Beam " << m_myBeamId
                       << " 业务 UL 预算耗尽 (PreambleID=" << pid
                       << "), RA 走专用控制池, 仍发 RAR (grant=" << msg3RequestedAfterPowerLimit
                       << " RB, 不再扣业务预算)");
          approvedMsg3Rbs = msg3RequestedAfterPowerLimit;
        }

      const double msg3TxPowerDbm =
        m_resourceManager->AdjustUtTxPower (msg3UtType,
                                            approvedMsg3Rbs,
                                            m_referencePathLossDb);
      rar.raRnti            = raRnti;
      rar.preambleId        = pid;
      rar.tcRnti            = AllocateTcRnti ();
      rar.timingAdvance     = MilliSeconds (300); // GEO 单程
      rar.ulGrantRbs        = approvedMsg3Rbs;    // Msg3 使用资源管理器批准的 PRB
      rar.ulGrantMcs        = m_msg3GrantMcs;     // 保守 MCS
      rar.ulGrantTxPowerDbm = msg3TxPowerDbm;
      rar.msg3DelayToStart  = MicroSeconds (500); // 处理延迟
      rar.transmissionTime  = Simulator::Now () + m_rarProcessingDelay;

      NS_LOG_INFO ("[Msg2] 分配 RAR: RA-RNTI=" << raRnti
                   << " PreambleID=" << pid
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
  m_uplinkPropDelay      = rarTxDelay;
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

//上行授权核心函数 UL grant 必须同时满足 beam RB 剩余约束和 UE 功率约束
//定义处理 UL grant 的函数 输入是 UE、请求 RB 数和 MCS
uint32_t GeoBeamScheduler::ProcessUlGrant (uint16_t rnti, uint32_t rbAllocation, uint8_t mcs)
{
  NS_LOG_FUNCTION (this << rnti << rbAllocation << (uint32_t)mcs);
  
  //如果 UE 上下文不存在，返回 0，表示授权失败
  if (m_ueContextMap.find (rnti) == m_ueContextMap.end ()) {
      NS_LOG_WARN ("[UL Grant] UE " << rnti << " 上下文不存在!");
      return 0;
  }
  
  SatUeContext& ctx = m_ueContextMap[rnti];//取出 UE 上下文
  const uint32_t beamId = ctx.currentBeamId;//读取 UE 当前所在 beam
  //确保该 beam 已经进入 UL 调度周期
  BeginUlSchedulingPeriod (beamId);
  //调用 ResourceManager 根据终端类型和路径损耗计算功率受限下最多可分配多少 UL RB
  const uint32_t powerLimitedMaxRbs =
    m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
  const uint32_t requestedAfterPowerLimit = std::min (rbAllocation, powerLimitedMaxRbs);//请求 RB 数不能超过功率允许的最大 RB 数
  if (requestedAfterPowerLimit == 0)
    {
      NS_LOG_WARN ("[UL Grant] UE " << rnti
                   << " 因终端功率上限受限，无法在非削顶条件下分配任何 UL RB");
      return 0;
    }
  const uint32_t remainingBeforeGrant = m_resourceManager->GetRemainingRbs (beamId, true);//记录授权前 beam 剩余 UL RB
  //调用 ResourceManager::AllocateSpectrum() 真正审批 UL RB
  const uint32_t approvedRb =
    m_resourceManager->AllocateSpectrum (beamId, requestedAfterPowerLimit, true);

    //审批结果为 0，说明 beam 上行资源已耗尽
  if (approvedRb == 0)
    {
      NS_LOG_WARN ("[UL Grant] Beam " << beamId << " 无可用上行RB，授权被拒绝!");
      return 0;
    }
  
  // 构建上行授权DCI
  DciInfo dci;
  dci.isUplinkGrant = true;//上行授权
  dci.rbAllocation = approvedRb;//写入最终批准的 RB 数
  dci.mcs = mcs;//写入 MCS
  //调用 AdjustUtTxPower() 估算 UE 发射功率
  const double estimatedTxPowerDbm =
    m_resourceManager->AdjustUtTxPower (ctx.utType, approvedRb, m_referencePathLossDb);
  dci.txPowerDbm = estimatedTxPowerDbm;//把发射功率写入 DCI
  dci.delayToStart = MicroSeconds (32);  // 典型的K2延迟
  dci.duration = MilliSeconds (1);//设置授权持续时间
  ctx.pendingUlGrantRbs += approvedRb;//把批准的 RB 加入 pending UL grant 统计
  //估算这些 RB 可承载多少字节
  const uint32_t estimatedGrantBytes =
    std::max (1u,
              static_cast<uint32_t> (std::llround (EstimateBytesPerRb (GetSchedulingCqi (ctx, true)) *
                                                   approvedRb)));
  ctx.pendingUlGrantBytes += estimatedGrantBytes;//估算字节数加入 pending grant bytes
  ctx.pendingUlGrantBytes = std::min (ctx.pendingUlGrantBytes, ctx.ulBufferStatus);//pending grant bytes 不能超过当前 UL buffer
  RefreshPendingUlGrantEstimate (ctx);//重新估算 pending grant RB
  ctx.srPending = false;//清除 SR pending 标记
  ctx.lastUlScheduledTime = Simulator::Now ();//更新上次 UL 调度时间
  
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
  return approvedRb;//返回最终批准的 RB 数
}

// ==================== 辅助函数实现 ====================
//开启 UL 调度周期  防止一个 TTI 内多次触发 UL 调度时重复重置 RB，避免超分配
void GeoBeamScheduler::BeginUlSchedulingPeriod (uint32_t beamId, uint64_t schedulingRoundId)
{
  auto it = m_ulSchedulingRoundIdByBeam.find (beamId);//查找该 beam 当前记录的 UL 调度轮次

  if (schedulingRoundId == 0)//如果调用者没有传入 roundId，就需要内部处理
    {
      if (it != m_ulSchedulingRoundIdByBeam.end ())//如果该 beam 已经有调度轮次 复用
        {
          NS_LOG_INFO ("[UL TTI] Beam " << beamId
                       << " 复用已有上行调度轮次 "
                       << it->second
                       << " | UL Remaining="
                       << m_resourceManager->GetRemainingRbs (beamId, true) << " RB");
          return;
        }

      schedulingRoundId = ++m_nextUlSchedulingRoundId;//如果没有传入 roundId，就生成一个新 roundId
    }

    //如果该 beam 已经在当前 roundId 下初始化过 直接返回
  if (it != m_ulSchedulingRoundIdByBeam.end () && it->second == schedulingRoundId)
    {
      return;
    }

  m_resourceManager->ResetBeamAllocation (beamId, true);//调用 ResourceManager 重置该 beam 的上行 RB 使用量
  m_ulSchedulingRoundIdByBeam[beamId] = schedulingRoundId;//记录该 beam 当前使用的调度轮次 ID

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

//上行 WRR + IPF 调度 
//构建 UL 业务队列 筛选有 UL 需求的 UE，经过功率约束和准入控制后，按业务类型构建 UL 队列
void GeoBeamScheduler::RunUlSchedulerForBeam (uint32_t beamId)//定义某一个 beam 的上行调度函数
{
  auto beamIt = m_beamToUesMap.find (beamId);
  //如果这个 beam 没有 UE，直接返回
  if (beamIt == m_beamToUesMap.end ())
    {
      return;
    }

  uint32_t availableRbs = m_resourceManager->GetRemainingRbs (beamId, true);//从 ResourceManager 查询该 beam 的上行剩余 RB
  //如果没有上行 RB，就不调度
  if (availableRbs == 0)
    {
      return;
    }
//创建应急、语音、普通数据三个 UL 队列
  std::vector<uint16_t> emergencyUes;
  std::vector<uint16_t> voiceUes;
  std::vector<uint16_t> normalUes;
  double maxEmergencyDelay = 0.0;//记录应急业务最大等待时延

  //遍历该 beam 下的所有 UE
  for (uint16_t rnti : beamIt->second)
    {
      auto ctxIt = m_ueContextMap.find (rnti);
      if (ctxIt == m_ueContextMap.end ())//如果 UE 上下文不存在，跳过
        {
          continue;
        }

      SatUeContext& ctx = ctxIt->second;//取出 UE 上下文
      const uint32_t effectivePendingBytes = GetEffectiveUlDemandBytes (ctx);//计算有效 UL 需求
      //判断是否需要 bootstrap grant 也就是 UE 发了 SR，但还没有 BSR 和 pending grant，此时可以给一个小 grant
      const bool needsBootstrapGrant =
        ctx.srPending && ctx.ulBufferStatus == 0 && ctx.pendingUlGrantRbs == 0;
      const bool hasPendingUlData = effectivePendingBytes > 0;//判断是否真的有待发 UL 数据
      //如果既不需要 bootstrap grant，也没有待发 UL 数据，跳过
      if (!needsBootstrapGrant && !hasPendingUlData)
        {
          continue;
        }

      const double bytesPerRb = EstimateBytesPerRb (GetSchedulingCqi (ctx, true));//估算每 RB 可承载字节数
      //如果有 UL 数据，就按有效 pending bytes 换算所需 RB  否则给 SR bootstrap grant，大小为 m_srGrantRbs
      const uint32_t requestedRbsRaw = hasPendingUlData ?
        std::max (1u, static_cast<uint32_t> (std::ceil (effectivePendingBytes / bytesPerRb))) :
        m_srGrantRbs;
      //调用资源管理器计算功率限制下最多可分配多少 UL RB
      const uint32_t powerLimitedMaxRbs =
        m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
      const uint32_t requestedRbs = std::min (requestedRbsRaw, powerLimitedMaxRbs);//请求 RB 不能超过功率限制
      if (requestedRbs == 0)
        {
          continue;
        }
      //计算准入控制使用的 RB 需求 应急最多 3，语音最多 2，普通数据至少 1 且不超过 SR grant 大小
      const uint32_t gateRequiredRbs =
        (ctx.priority == ServicePriority::PRIORITY_EMERGENCY) ? std::min (3u, requestedRbs) :
        (ctx.priority == ServicePriority::PRIORITY_VOICE) ? std::min (2u, requestedRbs) :
        std::max (1u, std::min (requestedRbs, m_srGrantRbs));

      //调用准入控制判断该 UE 是否可以在当前 beam 上行接入
      if (m_admitControl != nullptr)
        {
          m_admitControl->UpdateUeContext (rnti,
                                           ctx.latestDlCqi,
                                           ctx.latestUlCqi,
                                           gateRequiredRbs);
          const AdmitDecision decision =
            m_admitControl->CanAdmitUe (beamId,
                                        ctx.priority,
                                        ctx.utType,
                                        ctx.trafficType,
                                        gateRequiredRbs,
                                        true);
          //如果不是 ADMIT_OK，跳过
          if (decision != AdmitDecision::ADMIT_OK)
            {
              continue;
            }
        }

      const Time delay = Simulator::Now () - ctx.lastUlScheduledTime;//计算距离上次 UL 调度的时间
      //应急业务加入应急队列，并更新最大等待时间
      if (ctx.priority == ServicePriority::PRIORITY_EMERGENCY)
        {
          emergencyUes.push_back (rnti);
          maxEmergencyDelay = std::max (maxEmergencyDelay, delay.GetSeconds ());
        }
        //语音业务加入语音队列
      else if (ctx.priority == ServicePriority::PRIORITY_VOICE)
        {
          voiceUes.push_back (rnti);
        }
        //普通业务加入普通队列
      else
        {
          normalUes.push_back (rnti);
        }
    }
//UL WRR 预算 先按优先级切出 UL 业务预算，保障应急和语音
//计算 UL 应急业务预算 
  uint32_t emergencyBudget =
    std::min (availableRbs,
              (m_admitControl != nullptr) ? m_admitControl->GetEmergencyReservedRbs () : 1u);
//计算 UL 语音业务预算
  uint32_t voiceBudget =
    std::min (availableRbs - emergencyBudget,
              (m_admitControl != nullptr) ? m_admitControl->GetVoiceReservedRbs () : 2u);
//判断应急业务是否等待太久
  if (maxEmergencyDelay > m_emergencyDelayThresholdSeconds && !emergencyUes.empty ())
    {//如果应急等待超过阈值，就把应急预算提升到 burst cap
      emergencyBudget =
        std::min (availableRbs,
                  (m_admitControl != nullptr) ? m_admitControl->GetEmergencyBurstCapRbs () : 3u);
      //重新计算语音预算
      voiceBudget =
        std::min (availableRbs - emergencyBudget,
                  (m_admitControl != nullptr) ? m_admitControl->GetVoiceReservedRbs () : 2u);
    }

  uint32_t ulTrafficBudgetRemaining = availableRbs;//初始化 UL 总剩余业务预算

//UL 队列内部 IPF 排序与授权 UL 队列内部也使用 IPF 排序，但授权时额外考虑 UE 功率限制、pending grant 和 beam UL 剩余 RB
//定义 UL 队列调度 lambda
  auto scheduleUlClassQueue =
    [&] (const std::vector<uint16_t>& classQueue, uint32_t& queueBudget, double urgencyBoost)
    {//如果队列为空或预算为 0，直接返回
      if (classQueue.empty () || queueBudget == 0)
        {
          return;
        }

      std::vector<std::pair<uint16_t, double>> metricQueue;//创建 IPF 指标队列
      for (uint16_t rnti : classQueue)//遍历该业务类型下所有 UE
        {
          SatUeContext& ctx = m_ueContextMap[rnti];//取出 UE 上下文
          const uint32_t effectivePendingBytes = GetEffectiveUlDemandBytes (ctx);//计算有效 UL 需求
          const bool needsBootstrapGrant =
            ctx.srPending && ctx.ulBufferStatus == 0 && ctx.pendingUlGrantRbs == 0;
          const bool hasPendingUlData = effectivePendingBytes > 0;
          if (!needsBootstrapGrant && !hasPendingUlData)
            {
              continue;
            }

          const double bytesPerRb = EstimateBytesPerRb (GetSchedulingCqi (ctx, true));//估算每 RB 可承载字节数
          //计算该 UE 在功率限制下最多可拿多少 UL RB
          const uint32_t powerLimitedMaxRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
          //如果功率限制导致最多可用 RB 为 0，跳过
          if (powerLimitedMaxRbs == 0)
            {
              continue;
            }
          //计算用于 IPF 指标的需求字节数
          const double demandBytes =
            static_cast<double> (hasPendingUlData ?
                                 std::min<uint32_t> (effectivePendingBytes,
                                                     static_cast<uint32_t> (std::llround (bytesPerRb * powerLimitedMaxRbs))) :
                                 std::min (m_srGrantRbs, powerLimitedMaxRbs) * bytesPerRb);
          //计算 UL IPF 指标
          const double classMetric =
            CalculateSchedulerMetric (ctx, queueBudget, true, urgencyBoost, demandBytes);

          metricQueue.push_back ({rnti, classMetric});//保存 UE 与其指标
        }

        //按指标从高到低排序
      std::sort (metricQueue.begin (),
                 metricQueue.end (),
                 [] (const std::pair<uint16_t, double>& a, const std::pair<uint16_t, double>& b) {
                   return a.second > b.second;
                 });

      //按排序结果逐个分配 UL grant
      for (const auto& item : metricQueue)
        {
          if (queueBudget == 0)//如果当前业务预算用完，就退出
            {
              break;
            }

          SatUeContext& ctx = m_ueContextMap[item.first];//取出当前 UE 上下文
          const uint32_t effectivePendingBytes = GetEffectiveUlDemandBytes (ctx);
          const bool needsBootstrapGrant =
            ctx.srPending && ctx.ulBufferStatus == 0 && ctx.pendingUlGrantRbs == 0;
          const bool hasPendingUlData = effectivePendingBytes > 0;
          if (!needsBootstrapGrant && !hasPendingUlData)
            {
              continue;
            }

          const double bytesPerRb = EstimateBytesPerRb (GetSchedulingCqi (ctx, true));//估算 bytes per RB
          //计算原始所需 RB。如果有 UL 数据，按 buffer 估算；如果只是 SR bootstrap，则给 m_srGrantRbs
          const uint32_t neededRbsRaw = hasPendingUlData ?
            std::max (1u, static_cast<uint32_t> (std::ceil (effectivePendingBytes / bytesPerRb))) :
            m_srGrantRbs;
          //再次计算功率限制最大 RB
          const uint32_t powerLimitedMaxRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, m_referencePathLossDb);
          const uint32_t neededRbs = std::min (neededRbsRaw, powerLimitedMaxRbs);//实际需要 RB 不能超过功率上限
          //最终请求 RB 同时受三个条件限制：UE 需求、业务队列预算、beam 剩余 UL RB
          const uint32_t requestedRbs =
            std::min ({neededRbs, queueBudget, m_resourceManager->GetRemainingRbs (beamId, true)});
          //请求为 0，跳过
          if (requestedRbs == 0)
            {
              continue;
            }

          //调用 ProcessUlGrant() 真正生成 UL grant
          const uint32_t approvedRbs =
            ProcessUlGrant (item.first, requestedRbs, GetMcsFromCqi (GetSchedulingCqi (ctx, true)));
          if (approvedRbs == 0)//批准 RB 为 0，跳过
            {
              continue;
            }

          queueBudget = (queueBudget > approvedRbs) ? (queueBudget - approvedRbs) : 0;//扣减该业务队列预算
          //扣减 UL 总剩余业务预算
          ulTrafficBudgetRemaining = (ulTrafficBudgetRemaining > approvedRbs) ?
                                     (ulTrafficBudgetRemaining - approvedRbs) :
                                     0;
        }
    };

    //执行三类 UL 队列调度 UL 方向同样采用“WRR 业务切片 + IPF 队内排序”的两级调度结构
  scheduleUlClassQueue (emergencyUes, emergencyBudget, 1.5);//先调度上行应急队列，紧急度增益 1.5
  scheduleUlClassQueue (voiceUes, voiceBudget, 1.2);//再调度上行语音队列，紧急度增益 1.2
  uint32_t normalBudget = ulTrafficBudgetRemaining;//普通数据业务使用剩余 UL RB
  scheduleUlClassQueue (normalUes, normalBudget, 1.0);//调度普通数据队列，紧急度增益 1.0
}

//业务类型映射与 WRR 权重
//业务类型映射优先级 给每个 UE 确定 WRR/IPF 中使用的业务优先级
ServicePriority GeoBeamScheduler::MapTrafficTypeToPriority (TrafficType trafficType)//定义业务类型到调度优先级的映射函数
{
  switch (trafficType) {
    case TRAFFIC_VOICE:
      return ServicePriority::PRIORITY_VOICE;//语音业务映射为语音优先级
    case TRAFFIC_DATA:
      return ServicePriority::PRIORITY_DATA;//普通数据业务映射为数据优先级
    case TRAFFIC_HIGH_CAPACITY:
      return ServicePriority::PRIORITY_DATA;//高容量业务也映射为数据优先级
    default:
      return ServicePriority::PRIORITY_BEST_EFFORT;//未知业务映射为尽力而为
  }
}

//WRR 权重计算 给不同业务类型设定差异化权重，并把这个权重注入 IPF 指标中
double GeoBeamScheduler::CalculateWrrWeight (ServicePriority priority, UtType utType)
{
  // WRR权重计算: 应急业务权重最高
  double baseWeight = 1.0;//默认基础权重为 1
  
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

//准入控制外部绑定与切换准入
//设置 AdmitControl 允许外部替换或注入新的 AdmitControl，同时保持它和 ResourceManager、波束切换逻辑一致
void GeoBeamScheduler::SetAdmitControl (Ptr<AdmitControl> admitControl)//定义外部设置准入控制模块的函数
{
  m_admitControl = admitControl;//把外部传进来的 AdmitControl 保存到调度器
  if (m_admitControl != nullptr)//如果准入控制对象不为空，就继续绑定
    {
      m_admitControl->SetResourceManager (m_resourceManager);//把调度器内部的 ResourceManager 绑定给准入控制
      //把调度器的 ExecuteHandover() 作为切换执行回调交给准入控制
      m_admitControl->SetHandoverExecutor (
        MakeCallback (&GeoBeamScheduler::ExecuteHandover, this));
      m_admitControl->SetDlReservedRbs (m_prachReservedRbs);//把当前 PRACH 下行预留 RB 同步给准入控制
    }
  NS_LOG_INFO ("AdmitControl connected to scheduler");
}

//切换准入检查  为波束切换提供资源准入检查接口
bool GeoBeamScheduler::CheckHandoverAdmission (uint32_t targetBeamId,
                                               ServicePriority priority,
                                               UtType utType,
                                               TrafficType trafficType,
                                               uint32_t requiredRbs,
                                               bool isUplink)//定义切换准入检查函数
{
  NS_LOG_FUNCTION (this << targetBeamId << (uint32_t)priority << requiredRbs);
  
  if (m_admitControl == nullptr) {//如果没有准入控制模块，默认允许切换
      NS_LOG_WARN ("AdmitControl not set, allowing handover by default");
      return true;
  }
  
  AdmitDecision decision =//调用 AdmitControl::CanAdmitUe() 判断目标 beam 能否接纳该 UE
    m_admitControl->CanAdmitUe (targetBeamId,
                                priority,
                                utType,
                                trafficType,
                                requiredRbs,
                                isUplink);
  
  //如果返回 ADMIT_OK 或 ADMIT_REDIRECT，认为切换可以继续
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
               << " -> 上行传播 " << m_uplinkPropDelay.GetMilliSeconds () << "ms");

  Simulator::Schedule (m_uplinkPropDelay,
                       &GeoBeamScheduler::DoBufferMsgA, this, msgA);
}

void GeoBeamScheduler::DoBufferMsgA (MsgA msgA)
{
  NS_LOG_INFO ("[MsgA] gNB 收到 MsgA: PreambleID=" << msgA.preambleId
               << " UE-Id=0x" << std::hex << msgA.ueIdentity << std::dec);

  MsgAArrival arr;
  arr.preambleId = msgA.preambleId;
  arr.ueIdentity = msgA.ueIdentity;
  arr.arrivalTime = Simulator::Now ();
  arr.raRnti = msgA.raRnti;
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
  BeginUlSchedulingPeriod (m_myBeamId);

  std::map<std::pair<uint32_t, uint32_t>, std::vector<uint64_t>> preambleToUeIds;
  for (const auto& a : m_msgAWindowBuf)
    {
      preambleToUeIds[{a.raRnti, a.preambleId}].push_back (a.ueIdentity);
    }

  NS_LOG_INFO ("[MsgA] 窗口处理: 共 " << m_msgAWindowBuf.size ()
               << " 条 MsgA, 唯一 (raRnti,PreambleID) 数 = " << preambleToUeIds.size ());

  for (const auto& kv : preambleToUeIds)
    {
      const uint32_t raRnti = kv.first.first;
      const uint32_t pid = kv.first.second;
      const auto& ueIds = kv.second;
      std::set<uint64_t> uniqueUeIds (ueIds.begin (), ueIds.end ());

      MsgB msgB;
      msgB.preambleId = pid;
      msgB.raRnti = raRnti;
      msgB.timingAdvance = MilliSeconds (300);
      msgB.transmissionTime = Simulator::Now () + m_msgBProcessingDelay;
      msgB.ulGrantTxPowerDbm = 0.0;

      if (uniqueUeIds.size () == 1)
        {
          uint16_t cRnti = AllocateTcRnti ();
          msgB.type = MsgBType::SUCCESS_RAR;
          msgB.cRnti = cRnti;
          msgB.echoedUeIdentity = *uniqueUeIds.begin ();
          msgB.tcRnti = 0;
          msgB.ulGrantRbs = 0;
          msgB.ulGrantMcs = 0;

          NS_LOG_INFO ("[MsgB] SUCCESS_RAR: PreambleID=" << pid
                       << " C-RNTI=0x" << std::hex << cRnti
                       << " UE-Id=0x" << msgB.echoedUeIdentity << std::dec);

          if (m_ueContextMap.find (cRnti) == m_ueContextMap.end ())
            {
              AddUeContext (cRnti);
              AddUeInfo (cRnti, m_myBeamId);
            }
        }
      else
        {
          const UtType msg3UtType = static_cast<UtType> (m_msg3DefaultUtTypeValue);
          const uint32_t msg3PowerLimitedRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (msg3UtType, m_referencePathLossDb);
          uint32_t msg3RequestedAfterPowerLimit =
            std::min (m_msg3RequestedRbs, msg3PowerLimitedRbs);
          if (msg3RequestedAfterPowerLimit == 0)
            {
              NS_LOG_WARN ("[MsgB] PreambleID=" << pid
                           << " 终端功率上限受限, 回退使用配置 grant=" << m_msg3RequestedRbs << " RB");
              msg3RequestedAfterPowerLimit = m_msg3RequestedRbs;
            }
          uint32_t approvedMsg3Rbs =
            m_resourceManager->AllocateSpectrum (m_myBeamId, msg3RequestedAfterPowerLimit, true);
          if (approvedMsg3Rbs == 0)
            {
              NS_LOG_WARN ("[MsgB] Beam " << m_myBeamId
                           << " 业务 UL 预算耗尽 (PreambleID=" << pid
                           << "), RA 走专用控制池, 仍发 MsgB (grant=" << msg3RequestedAfterPowerLimit
                           << " RB, 不再扣业务预算)");
              approvedMsg3Rbs = msg3RequestedAfterPowerLimit;
            }

          uint16_t tcRnti = AllocateTcRnti ();
          msgB.type = MsgBType::FALLBACK_RAR;
          msgB.cRnti = 0;
          msgB.echoedUeIdentity = 0;
          msgB.tcRnti = tcRnti;
          msgB.ulGrantRbs = approvedMsg3Rbs;
          msgB.ulGrantMcs = m_msg3GrantMcs;
          msgB.ulGrantTxPowerDbm =
            m_resourceManager->AdjustUtTxPower (msg3UtType,
                                                approvedMsg3Rbs,
                                                m_referencePathLossDb);

          NS_LOG_WARN ("[MsgB] FALLBACK_RAR (碰撞): PreambleID=" << pid
                       << " 唯一 UE 数=" << uniqueUeIds.size ()
                       << " TC-RNTI=0x" << std::hex << tcRnti << std::dec
                       << " Msg3RB=" << msgB.ulGrantRbs
                       << " Msg3TxPower=" << msgB.ulGrantTxPowerDbm
                       << " dBm -> 回退到 4 步 Msg3/Msg4");
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
  m_msgAWindowDuration = msgAWindow;
  m_msgBProcessingDelay = msgBProcessingDelay;
  m_msgBTxDelay = msgBTxDelay;
  m_uplinkPropDelay = msgBTxDelay;
}

} // namespace ns3
