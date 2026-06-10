/*
 * 文件路径: contrib/geo-sat/model/ntn-gw-rrc.h
 * 功能: 信关站(基站)侧 RRC 实体。
 *
 * 从 GeoBeamScheduler 拆出, 专门承担 3GPP RRC 在网络侧的职责:
 *   - 收 UE 上报的 MeasReport, 做 A3 / serving-unavailable 决策
 *   - 维护每个 UE 的服务对象 (SAT_BEAM / GROUND)
 *   - 与 AdmitControl 协同做切换准入
 *   - 编排具体的切换执行(波束间 / 星地双向), 通过 scheduler 完成 MAC 层上下文搬运
 *   - 记录 / 输出切换统计
 *
 * scheduler 仅保留调度算法 (WRR + IPF)、ResourceManager 桥接、随机接入相关。
 * RRC 通过 SetScheduler() 持有 scheduler 指针, 借其公开接口完成 UE 上下文操作。
 */
#ifndef NTN_GW_RRC_H
#define NTN_GW_RRC_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"
#include "ns3/callback.h"
#include "sat-mac-common.h"
#include "geo-beam-scheduler.h"   // 复用 HandoverStatsRecord / Summary / Request / Decision / TargetInfo / 枚举
#include "admit-control.h"
#include <map>
#include <string>
#include <vector>

namespace ns3 {

class NtnGwRrc : public Object
{
public:
  static TypeId GetTypeId (void);
  NtnGwRrc ();
  virtual ~NtnGwRrc ();

  // ---------- 装配接口 ----------
  void SetScheduler (Ptr<GeoBeamScheduler> scheduler);
  void SetAdmitControl (Ptr<AdmitControl> admitControl);

  // ---------- 测量上报入口 (从 UE / SatUtRrc 上行经空口送到这里) ----------
  void ReceiveMeasReport (const MeasReport& report);

  // ---------- UE 服务对象查询 / 可用性配置 ----------
  ServiceObjectId GetServingObjectForUe (uint16_t rnti) const;
  void SetGroundAvailability (uint32_t groundId, bool available);
  bool IsGroundAvailable (uint32_t groundId) const;

  // 单独暴露的准入检查 (兼容 scheduler 旧 API 的入口)
  bool CheckHandoverAdmission (uint32_t targetBeamId,
                               ServicePriority priority,
                               uint32_t requiredRbs);

  // ---------- 切换执行编排 ----------
  bool ExecuteHandover (const HandoverExecutionRequest& request);

  // ---------- 切换统计 ----------
  typedef void (*HandoverStatsTracedCallback) (HandoverStatsRecord record);
  HandoverStatsSummary GetHandoverStats () const;
  const std::vector<HandoverStatsRecord>& GetHandoverRecords () const;
  void ExportHandoverStats (const std::string& filepath) const;

  // ---------- 辅助 (公开便于测试 / 后续 RRM 复用) ----------
  HandoverTargetInfo BuildSatBeamTarget (uint32_t beamId) const;
  HandoverTargetInfo BuildGroundTarget (uint32_t groundId) const;
  HandoverExecutionType ClassifyHandoverExecution (const ServiceObjectId& source,
                                                   const ServiceObjectId& target) const;

  // ============================================================
  // Paging (寻呼): GW 收到面向 INACTIVE UE 的 DL 数据时, 主动 SendPaging,
  // UE 端 SatUtRrc::ReceivePagingMessage 收到后 RRC_INACTIVE -> CONNECTED,
  // 并测出 access recovery delay。
  // ============================================================
  /// 注入下行 Paging 发送通路 (典型: 接 SatUtRrc::ReceivePagingMessage)
  void SetPagingCallback (Callback<void, uint16_t> cb);
  /// 主动给某 UE 发 Paging
  void SendPaging (uint16_t ueId);
  /// (ueId, t_ns): GW 每次发出 Paging 的时间戳, 用于和 UE 端 access recovery delay 配对
  TracedCallback<uint16_t, uint64_t> m_pagingTrace;

private:
  void EnsureUeContextFromReport (const MeasReport& report);
  HandoverDecisionResult DecideHandoverTarget (const MeasReport& report);
  bool ExecuteBeamHandover (const HandoverExecutionRequest& request);
  bool ExecuteGroundToSatHandover (const HandoverExecutionRequest& request);
  bool ExecuteSatToGroundHandover (const HandoverExecutionRequest& request);
  Time EstimateInterruptionDelay (HandoverExecutionType type) const;
  void RecordHandoverOutcome (const MeasReport& report,
                              const HandoverDecisionResult& decision,
                              bool success,
                              Time triggerTime,
                              Time finishTime,
                              Time interruptionDelay);

  // ---------- 依赖 ----------
  Ptr<GeoBeamScheduler> m_scheduler;
  Ptr<AdmitControl> m_admitControl;

  // ---------- RRC 自身状态 ----------
  std::map<uint16_t, ServiceObjectId> m_ueServingObjectMap;
  std::map<uint16_t, double>          m_ueGroundMetricMap;
  std::map<uint32_t, bool>            m_groundAvailabilityMap;
  uint32_t                            m_defaultGroundId;

  // ---------- 切换统计 ----------
  std::vector<HandoverStatsRecord> m_handoverRecords;
  uint32_t m_hoAttempts {0};
  uint32_t m_hoSuccesses {0};
  uint32_t m_hoFailures {0};
  TracedCallback<HandoverStatsRecord> m_handoverTrace;

  // Paging
  Callback<void, uint16_t> m_pagingCb;
};

} // namespace ns3

#endif /* NTN_GW_RRC_H */
