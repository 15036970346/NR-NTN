/*
 * 文件路径：contrib/geo-sat/model/geo-beam-scheduler.h
 * 功能：GEO 卫星波束感知 MAC 层调度器 (透明转发模式) - 融合 RRM 版
 *
 * 合并说明 (geo-sat-merged):
 *   - 基底 = 0603 版: 完整 4步+2步 RA、PRACH 检测模型、CqiAmcController 钩子、
 *     多目标切换 (波束/地面) 类型、供 NtnGwRrc 调用的 UE 上下文访问器、Msg3 资源门控。
 *   - 嫁接 = qyh 原版重型 RRM: CLPC 闭环功控、功率域 NOMA、下行动态功率分配、
 *     上行动态调度全链路 (UL ledger)、HARQ 集成、DL/UL 双 CQI、NrAmc 的 MCS/字节估计、
 *     DCI 回调、SPS 半持续调度、~28 个可配置 TypeId 属性。
 *   - 切换"决策"归 NtnGwRrc (MeasReport 由 sat-ut-phy 投递给 NtnGwRrc); 调度器仅保留
 *     被 NtnGwRrc 回调的 ExportUeContext / ImportUeContext / ExecuteHandover。
 */
#ifndef GEO_BEAM_SCHEDULER_H
#define GEO_BEAM_SCHEDULER_H

#include "ns3/nr-mac-scheduler.h"
#include "ns3/nr-mac-csched-sap.h"
#include "ns3/nr-mac-sched-sap.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/nr-amc.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include "ns3/random-variable-stream.h"
#include "sat-mac-common.h"
#include "resource-manager.h"
#include "admit-control.h"
#include "cqi-predictor.h"
#include "cqi-amc-controller.h"
#include "cqi-amc-predictor.h"
#include "prach-detection-model.h"
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ns3 {

class HarqManager;
class ResultWriter;

struct SatUeContext {
  uint16_t rnti;
  uint32_t currentBeamId;

  // ----- 兼容字段 (0603): latestCqi == latestDlCqi, bufferStatus == dlBufferStatus -----
  double latestCqi;            // 兼容字段, 等价 latestDlCqi
  uint32_t bufferStatus;       // 兼容字段, 等价 dlBufferStatus
  double averageThroughput;    // 兼容字段, 等价 dlAverageThroughput
  Vector position;             // UE位置 (用于IPF算法)

  UtType utType;
  TrafficType trafficType;
  ServicePriority priority;     // 业务优先级
  double wrrWeight;            // WRR权重
  Time lastScheduledTime;      // 兼容字段 (0603): 上次调度时间

  // ----- qyh 增量: DL/UL 双 CQI / 双 Buffer / 路损 / CLPC / SPS / 吞吐 / 时间戳 -----
  double  latestDlCqi {7.0};
  double  latestUlCqi {7.0};
  double  latestRsrp {-120.0};
  double  pathLossDb {190.0};        // 每-UE 上行路径损耗估计(由 RSRP 推算), 取代全局常量
  uint32_t dlBufferStatus {0};       // = bufferStatus (DL)
  uint32_t ulBufferStatus {0};
  uint32_t pendingUlGrantRbs {0};
  uint32_t pendingUlGrantBytes {0};
  bool     srPending {false};
  double   dlAverageThroughput {0.0};
  double   ulAverageThroughput {0.0};
  Time     lastDlScheduledTime;      // 上次下行调度时间
  Time     lastUlScheduledTime;      // 上次上行调度时间
  Time     lastDlCqiUpdateTime;      // 最近一次收到 DL CQI 的时间
  Time     lastUlCqiUpdateTime;      // 最近一次收到 UL CQI 的时间
  Time     lastDlSpsTime;            // 上次下行 SPS(半持续)授权时间
  Time     lastUlSpsTime;            // 上次上行 SPS(半持续)授权时间
  double   clpcOffsetDb {0.0};       // 上行闭环功控累积修正项 f(i) (dB)
};

// 阶段1: SPS(半静态)调度模式。运行时三选一互斥(详见设计讨论)。
//  - SPS_OFF       : 不启用 SPS(默认; 等价历史 m_spsEnabled=false)
//  - SPS_LEGACY    : 旧"周期性固定带宽"实现(TryScheduleDlSps/UlSps), 仅作 A/B 对比
//  - SPS_CONFIGURED: 新持久 grant 状态机(激活/复用/隐式释放, 频域钉死 RBG)
enum class SpsMode {
  SPS_OFF = 0,
  SPS_LEGACY = 1,
  SPS_CONFIGURED = 2
};

// 阶段1: 半静态授权(configured grant)。一条 grant 跨多个周期复用同一组 RBG。
//  与"现 SPS 每周期重分/重发"的根本区别: 激活一次→存下来→周期复用, 不再逐周期重决策。
struct SatSpsGrant {
  uint32_t rbgSize {0};              // 该 grant 每周期需要的 RBG 数(持久; overbooking 下账面按此准入)
  std::vector<uint32_t> rbgBitmap;   // 本周期 first-fit 认领到的物理 RBG 索引(每周期更新; 供日志/NOMA叠加)
  uint8_t  mcs {0};
  uint32_t tbSize {0};               // 当前 MCS/RBG 下每周期 TBS 字节
  Time     period;                   // 该 grant 的 SPS 周期
  Time     nextTime;                 // 下次到期复用时刻(含错峰相位)
  uint8_t  emptyTxCounter {0};       // 连续到期无数据计数 -> 隐式释放
  uint8_t  conflictCounter {0};      // 连续 RBG 冲突计数 -> 隐式释放
  bool     active {false};
  bool     isUplink {false};
  uint32_t beamId {0};               // 激活时所在波束(复用只在同波束尝试)
  // 功率域半静态 NOMA 绑定(阶段3): 一次配对写入, 跨周期复用, 每 NomaRepairPeriod 重评估。
  uint16_t nomaPartnerRnti {0};      // 绑定的 far(弱信道)UE; 0=未配对
  double   nomaBeta {0.0};           // far 功率份额 β(配对时锁定)
  uint32_t reuseSinceRepair {0};     // 距上次配对重评估的复用次数
};

// 阶段1: SPS 统计(注: DCI 真实节省口径要到阶段2 UE 侧 configured grant 后才成立)。
struct SatSpsStats {
  uint64_t dlActivations {0}, dlReuse {0}, dlReleaseEmpty {0}, dlReleaseConflict {0};
  uint64_t ulActivations {0}, ulReuse {0}, ulReleaseEmpty {0}, ulReleaseConflict {0};
  // 阶段4(overbooking): 每周期物理认领失败次数(冲突事件, 冲突率分子) 与 激活被账面上限拒绝次数。
  uint64_t dlConflictEvents {0}, ulConflictEvents {0};
  uint64_t dlAdmitRejects {0}, ulAdmitRejects {0};
  // 阶段3(半静态 NOMA): SPS grant 上成功叠加 far 的次数(半持续配对发射次数)。
  uint64_t nomaSpsPairs {0};
  // 阶段2(真省 DCI): 实际下发的 SPS DCI 数(激活+重配+释放) 与 无 DCI 复用次数(真实节省)。
  uint64_t dlSpsDciSent {0}, dlSpsReuseNoDci {0};
  uint64_t ulSpsDciSent {0}, ulSpsReuseNoDci {0};
  // 阶段5(统计): SPS DL 累计交付字节(吞吐量指标)。
  uint64_t dlSpsServedBytes {0};
};

// 用于切换上下文转移的结构体 (并集: qyh 丰富字段 + 0603 多目标 ServiceObject)
struct HandoverUeContext {
    uint16_t rnti;
    uint32_t sourceBeamId;
    uint32_t targetBeamId;
    // 0603 多目标切换标识
    ServiceObjectId sourceObject;
    ServiceObjectId targetObject;
    // 0603 兼容字段
    double latestCqi;
    uint32_t unsentBufferBytes;
    double averageThroughput;
    // qyh 丰富字段
    double latestDlCqi;
    double latestUlCqi;
    double latestRsrp;
    double pathLossDb;
    double clpcOffsetDb;
    uint32_t unsentDlBufferBytes;
    uint32_t unsentUlBufferBytes;
    uint32_t pendingUlGrantRbs;
    uint32_t pendingUlGrantBytes;
    bool srPending;
    double dlAverageThroughput;
    double ulAverageThroughput;
    Vector position;
    UtType utType;
    TrafficType trafficType;
    Callback<void, DciInfo> dciCallback;
    Time lastDlScheduledTime;
    Time lastUlScheduledTime;
    Time lastDlCqiUpdateTime;
    Time lastUlCqiUpdateTime;
};

enum class HandoverTargetType {
    HANDOVER_TARGET_UNKNOWN = 0,
    HANDOVER_TARGET_SAT_BEAM,
    HANDOVER_TARGET_GROUND
};

enum class HandoverExecutionType {
    HANDOVER_EXEC_UNKNOWN = 0,
    HANDOVER_EXEC_BEAM_TO_BEAM,
    HANDOVER_EXEC_GROUND_TO_SAT,
    HANDOVER_EXEC_SAT_TO_GROUND
};

struct HandoverTargetInfo {
    HandoverTargetType targetType {HandoverTargetType::HANDOVER_TARGET_UNKNOWN};
    ServiceObjectId targetObject;
    uint32_t targetBeamId {0};
    uint32_t targetGroundId {0};
    bool targetAvailable {true};
};

struct HandoverExecutionRequest {
    uint16_t rnti {0};
    ServiceObjectId sourceObject;
    ServiceObjectId targetObject;
    HandoverExecutionType executionType {HandoverExecutionType::HANDOVER_EXEC_UNKNOWN};
    bool groundAvailable {true};
    bool satelliteAvailable {true};
};

struct HandoverDecisionResult {
    bool shouldHandover {false};
    AdmitDecision admitDecision {AdmitDecision::ADMIT_REJECTED};
    HandoverExecutionRequest request;
    std::string reason;
};

// 单次切换事件记录 (TracedCallback 载荷)
struct HandoverStatsRecord {
    uint16_t rnti {0};
    ServiceObjectId sourceObject;
    ServiceObjectId targetObject;
    HandoverExecutionType executionType {HandoverExecutionType::HANDOVER_EXEC_UNKNOWN};
    MeasEventType triggerEvent {MeasEventType::MEAS_EVENT_NONE};
    Time triggerTime {Seconds (0)};        // 触发该切换的测量上报生成时间
    Time finishTime {Seconds (0)};         // 目标侧上下文注入完成时间
    Time handoverDelay {Seconds (0)};      // finishTime - triggerTime
    Time interruptionDelay {Seconds (0)};  // 数据面中断时延 (源停止调度 -> 目标恢复)
    bool success {false};
    std::string reason;
};

// 切换统计汇总
struct HandoverStatsSummary {
    uint32_t attempts {0};
    uint32_t successes {0};
    uint32_t failures {0};
    double failureRate {0.0};
    double avgHandoverDelayMs {0.0};
    double maxHandoverDelayMs {0.0};
    double avgInterruptionDelayMs {0.0};
    double maxInterruptionDelayMs {0.0};
    std::map<HandoverExecutionType, uint32_t> attemptsByType;
    std::map<HandoverExecutionType, uint32_t> successesByType;
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
  HandoverUeContext ExportUeContext (uint16_t rnti, const HandoverTargetInfo& targetInfo);
  void ImportUeContext (const HandoverUeContext& hoCtx);
  void ExecuteHandover (uint16_t rnti, uint32_t targetBeamId);

  // ==================== UE 上下文查询/修改 (供 RRC 等上层调用) ====================
  bool HasUeContext (uint16_t rnti) const;
  SatUeContext GetUeContext (uint16_t rnti) const;
  void SetUeBeamId (uint16_t rnti, uint32_t beamId);
  ServicePriority GetUePriority (uint16_t rnti) const;
  uint32_t EstimateRequiredRbs (uint16_t rnti) const;

  // ==================== GEO 信令传播参数 (供 RRC 估算中断时延) ====================
  Time GetRarTxDelay () const { return m_rarTxDelay; }
  Time GetUplinkPropDelay () const { return m_uplinkPropDelay; }

  // ==================== CQI 预测器 / AMC 控制器 / PRACH 检测 钩子 ====================
  /// CQI 预测器: 默认直通 (回退实测 CQI)
  void              SetCqiPredictor (Ptr<CqiPredictor> cqiPredictor);
  Ptr<CqiPredictor> GetCqiPredictor () const { return m_cqiPredictor; }

  /// OLLA + Kalman 链路自适应控制器
  void                   SetCqiAmcController (Ptr<CqiAmcController> ctrl) { m_cqiAmc = ctrl; }
  Ptr<CqiAmcController>  GetCqiAmcController () const { return m_cqiAmc; }

  /// PRACH 检测模型 (Pd/Pfa 系统级)
  void                       SetPrachDetectionModel (Ptr<PrachDetectionModel> model);
  Ptr<PrachDetectionModel>   GetPrachDetectionModel () const { return m_prachDetectionModel; }
  void                       SetDefaultPrachSnrDb (double s) { m_defaultPrachSnrDb = s; }
  double                     GetDefaultPrachSnrDb () const { return m_defaultPrachSnrDb; }

  // ==================== 准入控制 / HARQ 注入 (qyh) ====================
  void SetAdmitControl (Ptr<AdmitControl> admitControl);
  bool CheckHandoverAdmission (uint32_t sourceBeamId, uint32_t targetBeamId,
                               ServicePriority priority, UtType utType,
                               TrafficType trafficType, uint32_t requiredRbs, bool isUplink = false);
  void SetHarqManager (Ptr<HarqManager> harqManager);
  void RegisterUeDciCallback (uint16_t ueId, Callback<void, DciInfo> dciCb);

  void UpdateUeCsi (uint16_t rnti, double cqi);
  /// qyh 增量: DL CQI 上报入口 (喂 CqiPredictor + CqiAmcController)
  void UpdateUeDlCqi (uint16_t rnti, double cqi);
  /// qyh 增量: UL CQI 上报入口 (对称 DL)
  void UpdateUeUlCqi (uint16_t rnti, double cqi);
  /// qyh 增量: 上层通知 DL/UL Buffer Status 变化
  void UpdateUeDlBufferStatus (uint16_t rnti, uint32_t bufferBytes);
  void UpdateUeUlBufferStatus (uint16_t rnti, uint32_t bufferBytes);

  /// qyh 增量: 给某 UE 选 MCS。m_cqiAmc 注入则走 EffectiveCqi 路径; 否则回退 latestCqi → GetMcsFromCqi。
  uint8_t GetMcsForNewTx (uint16_t rnti);

  // qyh 增量: PRACH 检测模型包装层 (lazy-create m_prachDetectionModel)
  void   EnablePrachDetectionErrors (bool enabled);
  bool   SetPrachDetectionCurveCsv (const std::string& path);
  void   SetPrachDetectionFixed (double pd, double pfa);
  void   SetNumPrachPreambles (uint32_t numPreambles) { m_numPrachPreambles = numPreambles; }
  uint32_t GetNumPrachPreambles () const { return m_numPrachPreambles; }
  PrachDetectionStats GetPrachDetectionStats () const;
  void   ResetPrachDetectionStats ();

  void UpdateUePosition (uint16_t rnti, Vector position);
  void PreProcessRequests ();

  // ==================== 功率域 NOMA: 有效 SINR / 共享RB 查询 (供数据面/统计读取) ====================
  /// 返回某 UE 本轮下行的"有效接收 SINR"(dB)。NOMA near=SIC后, far=被干扰后,
  /// 普通用户=CQI 等效 SINR。未记录则返回 NaN。供 NtnGwMac 在 SendDlPdu 时打 tag。
  double GetEffectiveDlSinrDb (uint16_t rnti) const;

  // ==================== 数据面单一真源 (跨调度轮持久化, 供 NtnGwMac 自动接线) ====================
  // m_effectiveDlSinrDb 每轮 RunScheduler 开头会清空, 与异步的数据面 (NtnGwMac 调度 tick)
  // 不同步。下面两张表在"真正下发 DL DCI"的位置持久化最近一次的 {有效SINR, 发射功率},
  // 不随调度轮清空, 让 NtnGwMac 能在任意时刻按 rnti 取到与最近一次授权一致的真值。
  /// 某 UE 最近一次 DL 授权的有效接收 SINR(dB)。无记录返回 NaN。
  double GetLastDlEffSinrDb (uint16_t rnti) const;
  /// 某 UE 最近一次 DL 授权的发射功率(dBm)。无记录返回 NaN。
  double GetLastDlTxPowerDbm (uint16_t rnti) const;
  /// 某 UE 最近一次 CLPC 闭环用到的实测接收 UL SINR(dB)。无记录返回 NaN。
  /// (只读观测器: 供 demo 展示 CLPC 收敛, 实测 SINR 趋近 TargetUlSinrDb。)
  double GetLastMeasuredUlSinrDb (uint16_t rnti) const;
  /// 某波束本轮 NOMA 功率域复用的下行/上行共享 RB 数 (读 ResourceManager 记账)。
  uint32_t GetBeamSharedRbs (uint32_t beamId, bool isUplink) const;
  /// 某波束当前已用的下行/上行 RB 数 (主用户占用, 用于算 NOMA 复用增益比例)。
  uint32_t GetBeamUsedRbs (uint32_t beamId, bool isUplink) const;
  uint32_t GetMyBeamId () const { return m_myBeamId; }

  // ----- NOMA 复用累计统计 (跨调度轮次, 供 result-writer / 例子读取并落盘) -----
  /// 某波束 NOMA 下行累计共享(复用) RB 数。
  uint64_t GetCumulativeNomaSharedDlRbs (uint32_t beamId) const;
  /// 某波束下行累计已用(主用户占用) RB 数。
  uint64_t GetCumulativeNomaUsedDlRbs (uint32_t beamId) const;
  /// 某波束 NOMA 下行复用增益 = 累计共享RB / 累计已用RB (无复用为0)。
  double GetNomaDlReuseGain (uint32_t beamId) const;
  /// 把本调度器各波束的 NOMA 复用记账写入 ResultWriter (消除 GetSharedRbs 死代码)。
  /// writer 为空则跳过; filename 默认 "NomaReuseStats.txt"。
  void WriteNomaReuseStats (Ptr<ResultWriter> writer,
                            const std::string& filename = "NomaReuseStats.txt") const;

  void RunScheduler ();
  void SendControlMsg (uint16_t rnti, uint8_t msgType);

  // ---- 阶段1 SPS 测试/统计访问器 ----
  const SatSpsStats& GetSpsStats () const { return m_spsStats; }
  uint32_t GetDlSpsGrantCount () const { return static_cast<uint32_t> (m_dlSpsGrants.size ()); }
  uint32_t GetUlSpsGrantCount () const { return static_cast<uint32_t> (m_ulSpsGrants.size ()); }
  uint32_t GetSpsAdmittedRbs (uint32_t beamId, bool isUplink) const
  { return m_resourceManager ? m_resourceManager->GetSpsAdmittedRbs (beamId, isUplink) : 0u; }
  // 供测试/外部按周期驱动 UL 一轮: 模拟 DoSchedUlTriggerReq 的 per-TTI 行为——
  // 先推进 UL 轮次并重置各 beam 的 UL 预算, 再运行 UL 调度。这样 SPS 的 RBG 预留发生在
  // 重置之后, 不会被 ProcessUlGrant 内的 BeginUlSchedulingPeriod 误清(后者见已有轮次即跳过)。
  void RunUlSchedulingRound ()
  {
    const uint64_t round = ++m_nextUlSchedulingRoundId;
    for (const auto& beamPair : m_beamToUesMap)
      {
        BeginUlSchedulingPeriod (beamPair.first, round);
      }
    RunUlScheduler ();
  }

  // ==================== 功能验证演示访问器 (functional-demo.cc 专用) ====================
  // 谨慎、最小: 只读地暴露调度器内部已有的 "每RB字节估算" 与 "调度用CQI(经预测器处理)",
  // 以及一个 UE 优先级直设接口(供构造应急/语音/数据三类对比)。不改变调度流程。
  /// 公共包装: 走 CQI→MCS→NrAmc 通路返回每 RB 可承载字节数。
  double EstimateBytesPerRbForCqi (double cqi, bool isUplink) const { return EstimateBytesPerRb (cqi, isUplink); }
  /// 公共包装: 返回某 UE 本轮调度实际使用的 CQI(关预测器=实测直通, 开=Kalman+OLLA有效CQI)。
  double GetSchedulingCqiForUe (uint16_t rnti, bool isUplink) const;
  /// 直接设置某 UE 的业务优先级(用于构造 EMERGENCY 等无法由业务类型映射得到的等级)。
  void SetUePriority (uint16_t rnti, ServicePriority priority);

  void SendDciToUe (uint16_t ueId, const DciInfo& dci);

  // ==================== PUCCH/BSR 处理接口 ====================
  void ReceivePucchInfo (const PucchInfo& pucchInfo);
  void ReceiveBsr (const BsR_MAC_CE& bsr);
  void ReceiveUlMacPdu (Ptr<Packet> packet);
  // preReserved=true: rbAllocation 个 RB 已在 RM 预留(SPS 走 ClaimSpsRbgs), 跳过功率受限裁剪与
  // AllocateSpectrum, 直接据此发 UL grant(复用功控/ledger/CLPC 逻辑)。
  // sendDci=false(阶段2 UL configured-grant DCI-free 复用): 完成功控/ledger 记账但不下发 UL DCI。
  uint32_t ProcessUlGrant (uint16_t rnti, uint32_t rbAllocation, uint8_t mcs,
                           bool preReserved = false, bool sendDci = true);

  // ==================== 4 步随机接入接口 (基站侧) ====================
  void ReceivePrachPreamble (const PrachPreamble& preamble);
  void ReceiveMsg3 (const RrcSetupRequest& req);
  void ReceiveMsg3Packet (Ptr<Packet> packet);
  void RegisterUeRaCallbacks (Callback<void, const RarMessage&> rarCb,
                              Callback<void, const RrcSetupMessage&> msg4Cb);
  void SetRaConfig (Time prachWindow, Time rarProcessingDelay,
                    Time rarTxDelay, Time msg4ProcessingDelay, Time msg4TxDelay);

  // ==================== 2 步随机接入接口 (基站侧) ====================
  void ReceiveMsgA (const MsgA& msgA);
  void RegisterUeTwoStepRaCallbacks (Callback<void, const MsgB&> msgBCb);
  void SetTwoStepRaConfig (Time msgAWindow, Time msgBProcessingDelay, Time msgBTxDelay);

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
  uint8_t GetMcsFromCqi (double cqi, bool isUplink) const;  // qyh DL/UL 版 (统一口径, 走 NrAmc)
  // 由 RSRP 推算每-UE 上行路径损耗(dB), 并写回 ctx.pathLossDb
  double DerivePathLossFromRsrp (double rsrpDbm) const;
  // 静态(SPS)固定 RB
  uint32_t GetSpsClassRbs (const SatUeContext& ctx) const;  // 不受 m_spsEnabled 门控的按类 RB(供 Legacy/Configured 共用)
  uint32_t GetSpsFixedRbs (const SatUeContext& ctx) const;  // = m_spsEnabled ? GetSpsClassRbs : 0 (历史语义不变)
  bool TryScheduleDlSps (uint16_t rnti, uint32_t beamId, uint32_t& availableRbs);
  bool TryScheduleUlSps (uint16_t rnti, uint32_t beamId);
  // ---- 阶段1: 半静态 configured-grant 状态机(DL) ----
  SpsMode SpsModeEnum () const { return static_cast<SpsMode> (m_spsMode); } // uint -> 强类型枚举
  uint8_t SpsLcidForUe (const SatUeContext& ctx) const;       // 该 UE 的 SPS 逻辑信道 id(阶段1 单流, 固定)
  bool    IsSpsEligible (const SatUeContext& ctx) const;      // 是否走半静态(周期业务: 应急/语音/便携)
  void    ScheduleDlSpsReuse (uint32_t beamId, std::set<uint16_t>& spsServed);   // 复用到期 grant + 隐式释放
  void    ActivateDlSpsGrants (uint32_t beamId, const std::vector<uint16_t>& order,
                               std::set<uint16_t>& spsServed);                   // 给合格 UE 新建 grant
  // 实发一次(扣 buffer); 阶段3: 绑定 NOMA far 且其本周期有数据则叠加 far, 返回被叠加 far rnti(否则0)。
  // 阶段2(真省 DCI): isActivation=true 必发激活 DCI; 复用时仅当 MCS 需重配才发 DCI, 否则 DCI-free。
  uint16_t ServeDlSpsGrant (uint16_t rnti, SatSpsGrant& grant, bool isActivation);
  void    ReleaseDlSpsGrant (const std::pair<uint16_t,uint8_t>& key, bool dueToConflict);
  Time    ComputeStaggeredNextTime (uint16_t rnti, Time now, Time period) const; // 错峰: 给每 UE 稳定相位偏移
  // ---- 阶段3: 半静态 NOMA(在 SPS DL grant 上叠加 far) ----
  uint16_t FindSpsNomaFar (uint32_t beamId, double nearCqi, const std::set<uint16_t>& exclude) const;
  void     MaybeRepairSpsNomaPair (SatSpsGrant& grant, uint16_t nearRnti, const std::set<uint16_t>& exclude);
  // ---- 阶段1: 半静态 configured-grant 状态机(UL, 与 DL 对称) ----
  void    ScheduleUlSpsReuse (uint32_t beamId, std::set<uint16_t>& spsServed);   // 复用到期 UL grant + 隐式释放
  void    ActivateUlSpsGrants (uint32_t beamId, const std::vector<uint16_t>& order,
                               std::set<uint16_t>& spsServed);                   // 给合格 UE 新建 UL grant
  void    ServeUlSpsGrant (uint16_t rnti, SatSpsGrant& grant, bool isActivation); // 发一次 UL 授权; 阶段2: 仅激活/重配发 DCI
  void    ReleaseUlSpsGrant (const std::pair<uint16_t,uint8_t>& key, bool dueToConflict);
  ServicePriority MapTrafficTypeToPriority (TrafficType trafficType);
  double CalculateWrrWeight (ServicePriority priority, UtType utType);
  double EstimateBytesPerRb (double cqi, bool isUplink) const;
  // 对目标 RB 数求真实 TBS 字节(NR TBS 随 RB 非线性, 直接走 NrAmc::GetPayloadSize(mcs,rbs))。
  // 用于"按已分配 RB 计实发字节"与"按 buffer 反推所需 RB", 取代单RB×N 线性外推。
  uint32_t EstimateTbsBytes (double cqi, uint32_t rbs, bool isUplink) const;
  // 二分: 返回承载 targetBytes 所需的最小 RB 数(clamp 到 maxRbs); targetBytes==0 → 0。
  uint32_t RbsForBytes (double cqi, uint32_t targetBytes, uint32_t maxRbs, bool isUplink) const;
  double CalculateLocationFactor (const Vector& position) const;
  double CalculateSchedulerMetric (const SatUeContext& ctx,
                                   uint32_t queueBudget,
                                   bool isUplink,
                                   double urgencyBoost,
                                   double demandBytes) const;
  double ExtractWidebandDlCqi (const DlCqiInfo& cqiInfo) const;
  double EstimateWidebandUlCqiFromSinr (const UlCqiInfo& ulCqiInfo) const;
  // 闭环功控
  double AverageUlSinrDb (const UlCqiInfo& ulCqiInfo) const;
  // 统一上行实测应用(以 rnti 为键): 更新 UL CQI + (开启时)推进 CLPC。供标准 SAP 入口
  // (反查 rnti 后) 与 CLPC 直透回灌共用; 返回是否成功应用(样本有效)。
  bool ApplyUlSinrMeasurement (uint16_t rnti, const UlCqiInfo& ulCqiInfo);
  void UpdateClosedLoopPowerControl (uint16_t rnti, double measuredSinrDb);
  void ApplyClpcDelta (uint16_t rnti, double deltaDb);
  int QuantizeTpcDb (double errorDb) const;
  // ---------- 上行实测 SINR 反馈 (链路预算法, 闭合 CLPC 回路) ----------
  // 由 ProcessUlGrant 调用: 基于本次授权的 txPower/路损/天线增益/热噪声算出 gNB 侧
  // 接收 UL SINR(dB), 延迟 m_ulCqiFeedbackDelay 后回灌给 DoSchedUlCqiInfoReq。
  void ScheduleUlSinrFeedback (uint16_t rnti, uint32_t approvedRb, uint8_t symStart,
                               const SfnSf& sfnSf, double txPowerDbm,
                               double pathLossDb, double clpcOffsetDb);
  // 链路预算: 计算 gNB 侧接收 UL SINR(dB)。
  double ComputeReceivedUlSinrDb (uint32_t approvedRb, double txPowerDbm,
                                  double pathLossDb) const;
  // 延迟回调: 构造 SchedUlCqiInfoReqParameters 并喂入 DoSchedUlCqiInfoReq。
  void DeliverUlSinrFeedback (uint16_t rnti, uint32_t approvedRb, uint8_t symStart,
                              SfnSf sfnSf, double measuredSinrDb);

  // ---------- 功率域 NOMA 辅助 ----------
  double SinrDbFromCqi (double cqi) const;
  double CqiFromSinrDb (double sinrDb) const;
  // betaFar = 该 far 用户实际占的功率份额 (1:1 时 = m_nomaFarPowerFraction; 1:N 时为
  // m_nomaFarPowerFraction 在各 far 间的等分)。betaFarTotal = 所有 far 占的总份额
  // (near 承受的 (1-betaFarTotal) 自身衰减)。1:1 时两者相等 = m_nomaFarPowerFraction。
  std::pair<double, double> ComputeNomaEffectiveCqi (double nearCqi, double farCqi) const;
  // 功率域 NOMA 的真实功率切分(单位 dBm), 由波束下行总功率预算按 β / (1-β) 切分:
  //   near 用户拿 (1-β) 份额、far 用户拿 β 份额。
  double NomaNearTxPowerDbm () const;
  double NomaFarTxPowerDbm () const;
  // 按给定功率份额(线性, 0..1)把波束下行总功率预算切成 dBm; 供 1:N 的 per-far 份额使用。
  double NomaTxPowerDbmForFraction (double fraction) const;
  // 同时返回该 NOMA 对的 near/far 有效 SINR(dB): {nearSinrDb, farSinrDb}。
  std::pair<double, double> ComputeNomaEffectiveSinrDb (double nearCqi, double farCqi) const;
  // 1:N 版: betaFar=该 far 份额, betaFarTotal=全部 far 总份额(near 承受的自身衰减为
  // 1-betaFarTotal, 该 far 把"near 的 (1-betaFarTotal) 份 + 其它 far 的份额"当作干扰)。
  std::pair<double, double> ComputeNomaEffectiveSinrDb (double nearCqi, double farCqi,
                                                        double betaFar, double betaFarTotal) const;
  // 记录某 UE 本轮下行有效 SINR(dB), 供 GetEffectiveDlSinrDb 读取后随 DL 包下发。
  void RecordEffectiveDlSinrDb (uint16_t rnti, double sinrDb);
  void BuildNomaPairs (uint32_t beamId, const std::vector<uint16_t>& candidates);
  void EmitNomaFarGrant (uint16_t farRnti, uint32_t sharedRb, double farEffCqi,
                         double farEffSinrDb, uint32_t beamId, double farTxPowerDbm);
  uint8_t AllocateUlLedgerSymStart (void) const;
  void RecordPendingUlCqiAllocation (uint16_t rnti, uint32_t approvedRb, uint8_t mcs, uint8_t symStart);
  void BeginUlSchedulingPeriod (uint32_t beamId, uint64_t schedulingRoundId = 0);
  void RunUlScheduler ();
  void RunUlSchedulerForBeam (uint32_t beamId);
  void HandleDlHarqRetransmission (uint16_t rnti, uint8_t processId, Ptr<Packet> packet);
  void HandleDlHarqFeedbackResult (uint16_t rnti, uint8_t processId, bool completedSuccessfully);
  uint32_t GetEffectiveUlDemandBytes (const SatUeContext& ctx) const;
  void RefreshPendingUlGrantEstimate (SatUeContext& ctx) const;
  double GetSchedulingCqi (const SatUeContext& ctx, bool isUplink) const;

  // ---------- 随机接入辅助 ----------
  void DoBufferPreamble (PrachPreamble preamble);
  void DoBufferMsg3 (RrcSetupRequest req);
  void DoBufferMsgA (MsgA msgA);
  void ProcessPrachWindow ();
  void DispatchRar (RarMessage rar);
  void ProcessMsg3Window ();
  void DispatchMsg4 (RrcSetupMessage msg4);
  uint16_t AllocateTcRnti ();

  // 计算UE间距离 (用于IPF)
  double CalculateUeDistance (uint16_t rnti1, uint16_t rnti2);

  std::map<uint16_t, SatUeContext> m_ueContextMap;
  std::map<uint32_t, std::vector<uint16_t>> m_beamToUesMap;
  std::map<uint16_t, Callback<void, DciInfo>> m_ueDciCallbackMap;

  Time m_defaultK2Delay;
  uint32_t m_myBeamId;
  uint32_t m_prachReservedRbs;

  Ptr<ResourceManager> m_resourceManager;
  Ptr<AdmitControl> m_admitControl;
  Ptr<HarqManager> m_harqManager;
  Ptr<NrAmc> m_dlAmc;
  Ptr<NrAmc> m_ulAmc;
  Time m_dlSchedHorizon;  // DL 预测提前量
  Time m_ulSchedHorizon;  // UL 预测提前量

  // WRR状态
  uint8_t m_wrrCurrentPriority;  // 当前服务的优先级
  uint32_t m_wrrCurrentIndex;    // 当前轮询索引

  // IPF参数
  double m_ipfLocationWeight;   // 位置权重因子 (0-1)
  double m_ipfFairnessWeight;    // 公平性权重因子

  // ---------- qyh RRM 配置 ----------
  double m_emergencyDelayThresholdSeconds;
  double m_referencePathLossDb;  // RA(Msg3)阶段无UE上下文时的回退路损
  double m_refSatEirpDbm;        // 卫星每波束参考 EIRP(dBm); 路损 = EIRP - RSRP
  Time m_dlCqiValidityTime;
  Time m_ulCqiValidityTime;
  uint32_t m_srGrantRbs;
  uint8_t m_srGrantMcs;
  uint32_t m_dataAdmitRbCap;     // DATA/BE 准入"粗粒度闸"的宽松度上限(非分配上限)
  // 静态(SPS / 半持续)调度配置
  bool m_spsEnabled;
  uint32_t m_spsVoiceRbs;
  uint32_t m_spsEmergencyRbs;
  uint32_t m_spsPortableFloorRbs;
  Time m_spsPeriod;
  // ---------- 阶段1: 半静态 configured-grant 状态机 ----------
  // 模式以 uint 存储(0=Off/1=Legacy/2=Configured), 经 SpsModeEnum() 取强类型枚举比较。
  // (用 Uinteger 属性而非 Enum 属性, 规避 ns-3.40 EnumValue 对 enum class 成员的设值坑。)
  uint32_t m_spsMode;                 // 运行时三选一(默认 0=SPS_OFF)
  Time     m_spsGrantPeriod;          // configured grant 周期(默认 20ms VoIP)
  uint8_t  m_spsImplicitReleaseAfter; // 连续无数据/冲突 N 次后隐式释放(默认 3)
  uint32_t m_spsActivationMinBytes;   // 激活下限字节(默认 1)
  bool     m_spsStaggerEnabled;       // 错峰开关(默认 true)
  bool     m_spsHarqEnabled;          // SPS 是否挂 HARQ(GEO 周期小包默认 false: 关 HARQ 用保守 MCS)
  bool     m_spsNomaEnabled;          // 阶段3: 在 SPS grant 上叠加半静态 NOMA far(默认 false)
  uint32_t m_nomaRepairPeriod;        // 阶段3: 每多少次复用重评估一次半静态配对(默认 4)
  uint8_t  m_spsMcsReconfigThreshold; // 阶段2: 复用时 MCS 漂移 >= 此阈值才重发 DCI 重配; 否则 DCI-free(默认 2)
  std::map<std::pair<uint16_t,uint8_t>, SatSpsGrant> m_dlSpsGrants; // key=(rnti,lcid)
  std::map<std::pair<uint16_t,uint8_t>, SatSpsGrant> m_ulSpsGrants; // (阶段1 UL 暂留, 下一小步填)
  SatSpsStats m_spsStats;
  // 上行闭环功控(CLPC)配置
  bool m_clpcEnabled;
  double m_targetUlSinrDb;
  double m_clpcMinDb;
  double m_clpcMaxDb;
  Time m_clpcLoopDelay;
  // ---------- 上行 SINR 反馈链路预算参数 ----------
  double m_utTxAntennaGainDb;     // 终端发射天线增益 G_ut_tx (dB)
  double m_satRxAntennaGainDb;    // 卫星接收天线增益 G_sat_rx (dB)
  double m_noiseFigureDb;         // gNB 接收机噪声系数 NF (dB)
  double m_ulSinrMeasErrorSigmaDb;// 接收 SINR 估计误差标准差 (dB), 0=无测量噪声
  Time   m_ulCqiFeedbackDelay;    // UL CQI/SINR 反馈时延 (UE→卫星→gNB 上行单程)
  Ptr<NormalRandomVariable> m_ulSinrMeasErrorRv; // SINR 测量误差随机源
  bool m_dlPowerAllocEnabled;    // 下行动态功率调配
  // 功率域 NOMA 共享配置
  bool m_nomaEnabled;
  double m_nomaFarPowerFraction;
  double m_nomaMinCqiGap;
  uint32_t m_nomaMaxFar {1};                    // 单 near 可叠加的 far 上限 (1=经典 1:1)
  std::map<uint16_t, std::vector<uint16_t>> m_nomaPartner;  // 本轮 near->far(s) 配对
  std::set<uint16_t> m_nomaFarThisRound;       // 本轮作为 far 被 NOMA 服务的 UE
  // 本轮各 UE 下行有效接收 SINR(dB): NOMA near=SIC后, far=被干扰后, 普通=CQI等效。
  // 数据面(NtnGwMac::SendDlPdu)经 GetEffectiveDlSinrDb 读取后写进 NtnDciTag, 驱动 BLER。
  std::map<uint16_t, double> m_effectiveDlSinrDb;
  // 数据面单一真源(不随调度轮清空): 最近一次 DL 授权的 {有效SINR, 发射功率}。
  // 在真正发 DL DCI 的位置(scheduleClassQueue 发 near/普通 DCI、EmitNomaFarGrant 发 far DCI)写入。
  std::map<uint16_t, double> m_lastDlEffSinrDb;
  std::map<uint16_t, double> m_lastDlTxPowerDbm;
  // CLPC 只读观测: 最近一次实测接收 UL SINR(dB) (供 demo 展示收敛, 不参与调度逻辑)。
  std::map<uint16_t, double> m_lastMeasuredUlSinrDb;
  // NOMA 功率域复用累计记账(跨调度轮次, 因每轮 ResetBeamAllocation 会清零瞬时值)。
  // 每波束: 累计共享 RB 数 / 累计主用户已用 RB 数; 二者之比即该波束 NOMA 复用增益。
  std::map<uint32_t, uint64_t> m_cumNomaSharedDlRbs;
  std::map<uint32_t, uint64_t> m_cumNomaUsedDlRbs;

  // ---------- Msg3 资源门控 (0603, 与主代码 ProcessPrachWindow 对齐) ----------
  uint32_t  m_msg3RequestedRbs {6};
  uint8_t   m_msg3GrantMcs {4};
  uint32_t  m_msg3DefaultUtTypeValue {0};      // 0 = UT_PORTABLE
  // 每波束 UL 调度轮次
  std::map<uint32_t, uint64_t> m_ulSchedulingRoundIdByBeam;
  uint64_t                     m_nextUlSchedulingRoundId {0};

  std::map<uint16_t, Time> m_admissionQueueSince;

  struct PendingUlCqiAllocation
  {
    uint16_t rnti {0};
    uint8_t symStart {0};
    uint32_t tbsBytes {0};
  };
  std::map<uint64_t, std::vector<PendingUlCqiAllocation>> m_ulCqiAllocationMap;
  bool m_hasCurrentUlTriggerSfnSf {false};
  SfnSf m_currentUlTriggerSfnSf;

  struct PendingDlHarqTransmission
  {
    DciInfo dci;
    Ptr<Packet> packet;
    uint32_t packetBytes {0};
  };
  std::map<std::pair<uint16_t, uint8_t>, PendingDlHarqTransmission> m_dlHarqProcessMap;
  /// 记录 (rnti,processId) 是否发生过重传 (=首传 NACK), 用于 OLLA 推导初传结果。
  std::map<std::pair<uint16_t, uint8_t>, bool> m_dlHarqEverRetx;

  // ---------- 4 步 RA 运行时状态 ----------
  struct PreambleArrival {
    uint32_t preambleId;
    UtType   utType {UT_PORTABLE};
    Time     arrivalTime;
    uint32_t raRnti;
    double   prachSnrDb {0.0};
  };
  std::vector<PreambleArrival> m_prachWindowBuf;
  EventId  m_prachWindowEvent;

  // Msg3 聚合缓冲
  std::map<uint16_t, std::vector<RrcSetupRequest>> m_msg3WindowBuf;
  EventId  m_msg3WindowEvent;

  // 下行 RA 广播订阅者
  std::vector<Callback<void, const RarMessage&>>      m_rarSubscribers;
  std::vector<Callback<void, const RrcSetupMessage&>> m_msg4Subscribers;

  uint16_t m_tcRntiCounter;

  // 可配置时延参数
  Time m_prachWindowDuration;
  Time m_rarProcessingDelay;
  Time m_rarTxDelay;
  Time m_msg3WindowDuration;
  Time m_msg4ProcessingDelay;
  Time m_msg4TxDelay;

  // ---------- 2 步 RA 运行时状态 ----------
  struct MsgAArrival {
    uint32_t preambleId;
    uint64_t ueIdentity;
    Time     arrivalTime;
    uint32_t raRnti;
  };
  std::vector<MsgAArrival> m_msgAWindowBuf;
  EventId  m_msgAWindowEvent;

  std::vector<Callback<void, const MsgB&>> m_msgBSubscribers;

  Time m_msgAWindowDuration;
  Time m_msgBProcessingDelay;
  Time m_msgBTxDelay;

  // ---------- 上行传播时延 ----------
  Time m_uplinkPropDelay;

  // 2 步 RA 辅助函数
  void ProcessMsgAWindow ();
  void DispatchMsgB (MsgB msgB);

  // ---------- qyh 增量字段 (S2): 链路自适应 / RA 解调钩子 ----------
  Ptr<CqiPredictor>         m_cqiPredictor;
  Ptr<CqiAmcController>     m_cqiAmc;
  Ptr<PrachDetectionModel>  m_prachDetectionModel;
  double                    m_defaultPrachSnrDb {20.0};
  uint32_t                  m_numPrachPreambles {64};
  /// 属性 UseCqiAmcPredictor: 为 true 时把 CqiAmcPredictor 注入为 m_cqiPredictor。
  bool                      m_useCqiAmcPredictor {false};
  /// 是否已按 m_useCqiAmcPredictor 完成惰性注入 (保证属性设置后才生效, 只注入一次)。
  bool                      m_cqiAmcPredictorApplied {false};
  /// 按 m_useCqiAmcPredictor 惰性注入 CqiAmcPredictor (幂等)。
  void MaybeApplyCqiAmcPredictor ();
};

} // namespace ns3

#endif /* GEO_BEAM_SCHEDULER_H */
