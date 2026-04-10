/*
 * 文件路径：contrib/geo-sat/model/geo-beam-scheduler.h
 * 功能：GEO 卫星波束感知 MAC 层调度器 (透明转发模式) - 融合 RRM 版
 */
#ifndef GEO_BEAM_SCHEDULER_H
#define GEO_BEAM_SCHEDULER_H

#include "ns3/nr-mac-scheduler.h"
#include "ns3/nr-mac-csched-sap.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/log.h"
#include "ns3/vector.h"
#include "sat-mac-common.h"
#include "resource-manager.h"
#include "admit-control.h"
#include <map>
#include <vector>

namespace ns3 {

struct SatUeContext {
  uint16_t rnti;               
  uint32_t currentBeamId;      
  double latestCqi;            
  uint32_t bufferStatus;       
  double averageThroughput;    
  Vector position;             // UE位置 (用于IPF算法)
  
  UtType utType;           
  TrafficType trafficType;
  ServicePriority priority;     // 业务优先级
  double wrrWeight;            // WRR权重
  Time lastScheduledTime;      // 上次调度时间
};

// 用于切换上下文转移的结构体
struct HandoverUeContext {
    uint16_t rnti;
    uint32_t sourceBeamId;
    uint32_t targetBeamId;
    double latestCqi;
    uint32_t unsentBufferBytes;
    double averageThroughput;
    UtType utType;
    TrafficType trafficType;
    Vector position;
};

class GeoBeamScheduler : public NrMacScheduler
{
public:
  static TypeId GetTypeId (void);
  GeoBeamScheduler ();
  virtual ~GeoBeamScheduler ();

  void Initialize (uint32_t beamId, uint8_t scs);
  void ConfigPrachResources ();
  void ReservePrachResources ();

  void AddUeContext (uint16_t rnti, UtType utType = UT_CONSUMER, TrafficType trafficType = TRAFFIC_DATA);
  void AddUeInfo (uint16_t rnti, uint32_t targetBeamId);
  void RemoveUt (uint16_t rnti);
  HandoverUeContext ExportUeContext (uint16_t rnti, uint32_t targetBeamId);
  void ImportUeContext (const HandoverUeContext& hoCtx);
  void ExecuteHandover (uint16_t rnti, uint32_t targetBeamId);

  void UpdateUeCsi (uint16_t rnti, double cqi);
  void UpdateUePosition (uint16_t rnti, Vector position);
  void PreProcessRequests ();

  void RunScheduler ();
  void SendControlMsg (uint16_t rnti, uint8_t msgType);

  void ReceiveMeasReport (const MeasReport& report);
  void SendDciToUe (uint16_t ueId, const DciInfo& dci);

  // ==================== 准入控制接口 ====================
  void SetAdmitControl (Ptr<AdmitControl> admitControl);
  bool CheckHandoverAdmission (uint32_t targetBeamId, ServicePriority priority, uint32_t requiredRbs);

  // ==================== PUCCH/BSR 处理接口 ====================
  void ReceivePucchInfo (const PucchInfo& pucchInfo);
  void ReceiveBsr (const BsR_MAC_CE& bsr);
  void ProcessUlGrant (uint16_t rnti, uint32_t rbAllocation, uint8_t mcs);

  // ==================== 4 步随机接入接口 (基站侧) ====================
  // Msg1: UE → gNB, 前导码收集
  void ReceivePrachPreamble (const PrachPreamble& preamble);
  // Msg3: UE → gNB, RRC 连接建立请求 (承载在 PUSCH 上)
  void ReceiveMsg3 (const RrcSetupRequest& req);
  // UE 订阅下行 RA 消息 (RAR/Msg4) 的广播回调
  void RegisterUeRaCallbacks (Callback<void, const RarMessage&> rarCb,
                              Callback<void, const RrcSetupMessage&> msg4Cb);
  // 配置随机接入参数
  void SetRaConfig (Time prachWindow, Time rarProcessingDelay,
                    Time rarTxDelay, Time msg4ProcessingDelay, Time msg4TxDelay);

  // 必须实现的 5G-LENA 接口
  virtual void DoCschedCellConfigReq (const NrMacCschedSapProvider::CschedCellConfigReqParameters& params) override;
  virtual void DoCschedUeConfigReq (const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) override;
  virtual void DoCschedLcConfigReq (const NrMacCschedSapProvider::CschedLcConfigReqParameters& params) override;
  virtual void DoCschedLcReleaseReq (const NrMacCschedSapProvider::CschedLcReleaseReqParameters& params) override;
  virtual void DoCschedUeReleaseReq (const NrMacCschedSapProvider::CschedUeReleaseReqParameters& params) override;
  virtual void DoSchedDlRlcBufferReq (const NrMacSchedSapProvider::SchedDlRlcBufferReqParameters& params) override;
  virtual void DoSchedDlCqiInfoReq (const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params) override;
  virtual void DoSchedUlCqiInfoReq (const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params) override;
  virtual void DoSchedUlMacCtrlInfoReq (const NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params) override;
  virtual void DoSchedDlTriggerReq (const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params) override;
  virtual void DoSchedUlTriggerReq (const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params) override;
  virtual void DoSchedUlSrInfoReq (const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params) override;
  virtual void DoSchedSetMcs (uint32_t mcs) override;
  virtual void DoSchedDlRachInfoReq (const NrMacSchedSapProvider::SchedDlRachInfoReqParameters& params) override;
  virtual uint8_t GetDlCtrlSyms () const override;
  virtual uint8_t GetUlCtrlSyms () const override;
  virtual int64_t AssignStreams (int64_t stream) override;

private:
  uint8_t GetMcsFromCqi (double cqi);
  ServicePriority MapTrafficTypeToPriority (TrafficType trafficType);
  double CalculateWrrWeight (ServicePriority priority, UtType utType);

  // ---------- 随机接入辅助 ----------
  // 收集窗口内到达的 preamble 并为每个唯一的 preambleId 生成 RAR
  void ProcessPrachWindow ();
  // 真正发送 RAR (广播至所有订阅的 UE)
  void DispatchRar (RarMessage rar);
  // 处理 Msg3 窗口结束, 判定竞争解决并发 Msg4
  void ProcessMsg3Window ();
  // 真正发送 Msg4 (广播至所有订阅的 UE)
  void DispatchMsg4 (RrcSetupMessage msg4);
  // 分配一个 TC-RNTI (简单线性递增)
  uint16_t AllocateTcRnti ();
  
  // WRR调度算法
  void RunWrrScheduler (std::vector<uint16_t>& ueList, uint32_t availableRbs);
  
  // IPF调度算法 (位置辅助改进型比例公平)
  void RunIpfScheduler (std::vector<uint16_t>& ueList, uint32_t availableRbs);
  
  // 两级调度: WRR(应急) + IPF(普通)
  void RunTwoLevelScheduler ();
  
  // 计算UE间距离 (用于IPF)
  double CalculateUeDistance (uint16_t rnti1, uint16_t rnti2);

  std::map<uint16_t, SatUeContext> m_ueContextMap;           
  std::map<uint32_t, std::vector<uint16_t>> m_beamToUesMap;  
  
  Time m_defaultK2Delay; 
  uint32_t m_myBeamId;   
  uint32_t m_prachReservedRbs; 

  Ptr<ResourceManager> m_resourceManager;
  Ptr<AdmitControl> m_admitControl;
  
  // WRR状态
  uint8_t m_wrrCurrentPriority;  // 当前服务的优先级
  uint32_t m_wrrCurrentIndex;    // 当前轮询索引
  
  // IPF参数
  double m_ipfLocationWeight;   // 位置权重因子 (0-1)
  double m_ipfFairnessWeight;    // 公平性权重因子

  // ---------- 4 步 RA 运行时状态 ----------
  // 每条记录一次 preamble 到达；末尾字段为真实发送时间戳 (tie-break 用)
  struct PreambleArrival {
    uint32_t preambleId;
    Time     arrivalTime;
  };
  std::vector<PreambleArrival> m_prachWindowBuf;
  EventId  m_prachWindowEvent;             // 当前 PRACH 窗口处理事件

  // Msg3 聚合缓冲: tcRnti -> 所有同一 TC-RNTI 上的 Msg3
  std::map<uint16_t, std::vector<RrcSetupRequest>> m_msg3WindowBuf;
  EventId  m_msg3WindowEvent;              // 当前 Msg3 窗口处理事件

  // 下行 RA 广播订阅者 (每个 UE 注册一对回调, 自己按 rnti/preambleId 过滤)
  std::vector<Callback<void, const RarMessage&>>      m_rarSubscribers;
  std::vector<Callback<void, const RrcSetupMessage&>> m_msg4Subscribers;

  uint16_t m_tcRntiCounter;                // 下一个可分配的 TC-RNTI (从 0x8001 开始)

  // 可配置时延参数
  Time m_prachWindowDuration;              // 前导码聚合窗口 (1 ms 默认)
  Time m_rarProcessingDelay;               // gNB 处理 RAR 的本地时延
  Time m_rarTxDelay;                       // RAR 单程传播 (GEO ~300 ms)
  Time m_msg3WindowDuration;               // Msg3 聚合/去重窗口
  Time m_msg4ProcessingDelay;              // gNB 处理 Msg4 的本地时延
  Time m_msg4TxDelay;                      // Msg4 单程传播 (GEO ~300 ms)
};

} // namespace ns3

#endif /* GEO_BEAM_SCHEDULER_H */