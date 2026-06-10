/*
 * 文件路径: contrib/geo-sat/model/ntn-gw-rrc.cc
 * 实现: 信关站(基站)侧 RRC, 见 ntn-gw-rrc.h 顶部说明。
 */
#include "ntn-gw-rrc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <fstream>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnGwRrc");
NS_OBJECT_ENSURE_REGISTERED (NtnGwRrc);

namespace {

// ---- 与原 scheduler 中匿名命名空间一致, 只是搬过来 ----
std::string
ServiceObjectToString (const ServiceObjectId& object)
{
  switch (object.type)
    {
    case ServiceObjectType::SERVICE_OBJECT_GROUND:
      return "GROUND:" + std::to_string (object.objectId);
    case ServiceObjectType::SERVICE_OBJECT_SAT_BEAM:
      return "SAT_BEAM:" + std::to_string (object.objectId);
    default:
      return "UNKNOWN:0";
    }
}

std::string
HandoverExecTypeToString (HandoverExecutionType type)
{
  switch (type)
    {
    case HandoverExecutionType::HANDOVER_EXEC_BEAM_TO_BEAM:
      return "BEAM_TO_BEAM";
    case HandoverExecutionType::HANDOVER_EXEC_GROUND_TO_SAT:
      return "GROUND_TO_SAT";
    case HandoverExecutionType::HANDOVER_EXEC_SAT_TO_GROUND:
      return "SAT_TO_GROUND";
    default:
      return "UNKNOWN";
    }
}

std::string
MeasEventTypeToString (MeasEventType type)
{
  switch (type)
    {
    case MeasEventType::MEAS_EVENT_A3:                  return "A3";
    case MeasEventType::MEAS_EVENT_SERVING_UNAVAILABLE: return "SERVING_UNAVAILABLE";
    default:                                            return "NONE";
    }
}

} // anonymous namespace

TypeId
NtnGwRrc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnGwRrc")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnGwRrc> ()
    .AddTraceSource ("HandoverEvent",
                     "Fired once per handover attempt (success or failure).",
                     MakeTraceSourceAccessor (&NtnGwRrc::m_handoverTrace),
                     "ns3::NtnGwRrc::HandoverStatsTracedCallback");
  return tid;
}

NtnGwRrc::NtnGwRrc ()
  : m_defaultGroundId (1)
{
  NS_LOG_FUNCTION (this);
}

NtnGwRrc::~NtnGwRrc ()
{
  NS_LOG_FUNCTION (this);
}

void
NtnGwRrc::SetScheduler (Ptr<GeoBeamScheduler> scheduler)
{
  m_scheduler = scheduler;
}

void
NtnGwRrc::SetAdmitControl (Ptr<AdmitControl> admitControl)
{
  m_admitControl = admitControl;
  NS_LOG_INFO ("AdmitControl connected to GW RRC");
}

// =============================================================================
// 服务对象查询 / 可用性配置
// =============================================================================

ServiceObjectId
NtnGwRrc::GetServingObjectForUe (uint16_t rnti) const
{
  auto it = m_ueServingObjectMap.find (rnti);
  if (it == m_ueServingObjectMap.end ())
    {
      return ServiceObjectId ();
    }
  return it->second;
}

void
NtnGwRrc::SetGroundAvailability (uint32_t groundId, bool available)
{
  m_groundAvailabilityMap[groundId] = available;
  NS_LOG_INFO ("[Ground] Ground " << groundId << " availability changed to " << available);

  if (m_admitControl != nullptr)
    {
      m_admitControl->SetGroundAvailability (groundId, available);
    }
}

bool
NtnGwRrc::IsGroundAvailable (uint32_t groundId) const
{
  auto it = m_groundAvailabilityMap.find (groundId);
  if (it == m_groundAvailabilityMap.end ())
    {
      return true;
    }
  return it->second;
}

bool
NtnGwRrc::CheckHandoverAdmission (uint32_t targetBeamId,
                                  ServicePriority priority,
                                  uint32_t requiredRbs)
{
  NS_LOG_FUNCTION (this << targetBeamId << (uint32_t) priority << requiredRbs);

  if (m_admitControl == nullptr)
    {
      NS_LOG_WARN ("AdmitControl not set, allowing handover by default");
      return true;
    }

  AdmitDecision decision =
      m_admitControl->CanHandoverUe (0, targetBeamId, priority, requiredRbs);

  bool admitted =
      (decision == AdmitDecision::ADMIT_OK || decision == AdmitDecision::ADMIT_REDIRECT);

  NS_LOG_INFO ("Handover admission check: Beam " << targetBeamId
               << " | Decision: " << (uint32_t) decision
               << " | Admitted: " << admitted);

  return admitted;
}

HandoverTargetInfo
NtnGwRrc::BuildSatBeamTarget (uint32_t beamId) const
{
  HandoverTargetInfo target;
  target.targetType = HandoverTargetType::HANDOVER_TARGET_SAT_BEAM;
  target.targetObject = {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, beamId};
  target.targetBeamId = beamId;
  target.targetAvailable = true;
  return target;
}

HandoverTargetInfo
NtnGwRrc::BuildGroundTarget (uint32_t groundId) const
{
  HandoverTargetInfo target;
  target.targetType = HandoverTargetType::HANDOVER_TARGET_GROUND;
  target.targetObject = {ServiceObjectType::SERVICE_OBJECT_GROUND, groundId};
  target.targetGroundId = groundId;
  target.targetAvailable = IsGroundAvailable (groundId);
  return target;
}

HandoverExecutionType
NtnGwRrc::ClassifyHandoverExecution (const ServiceObjectId& source,
                                     const ServiceObjectId& target) const
{
  if (source.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM &&
      target.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
      return HandoverExecutionType::HANDOVER_EXEC_BEAM_TO_BEAM;
    }
  if (source.type == ServiceObjectType::SERVICE_OBJECT_GROUND &&
      target.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
      return HandoverExecutionType::HANDOVER_EXEC_GROUND_TO_SAT;
    }
  if (source.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM &&
      target.type == ServiceObjectType::SERVICE_OBJECT_GROUND)
    {
      return HandoverExecutionType::HANDOVER_EXEC_SAT_TO_GROUND;
    }
  return HandoverExecutionType::HANDOVER_EXEC_UNKNOWN;
}

// =============================================================================
// MeasReport 入口
// =============================================================================

void
NtnGwRrc::ReceiveMeasReport (const MeasReport& report)
{
  NS_ASSERT_MSG (m_scheduler, "NtnGwRrc::ReceiveMeasReport called before SetScheduler()");

  EnsureUeContextFromReport (report);

  const double csiValue = (report.bestNeighborObject.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
                              ? report.bestNeighborMetric
                              : report.servingMetric;
  m_scheduler->UpdateUeCsi (report.ueId, csiValue);
  m_scheduler->UpdateUePosition (report.ueId, report.position);

  if (report.servingObject.type == ServiceObjectType::SERVICE_OBJECT_GROUND)
    {
      m_ueGroundMetricMap[report.ueId] = report.servingMetric;
    }
  if (report.bestNeighborObject.type == ServiceObjectType::SERVICE_OBJECT_GROUND)
    {
      m_ueGroundMetricMap[report.ueId] = report.bestNeighborMetric;
    }

  HandoverDecisionResult decision = DecideHandoverTarget (report);
  NS_LOG_INFO ("[HO] UE " << report.ueId << " decision result | shouldHandover="
               << decision.shouldHandover << " | reason=" << decision.reason
               << " | admitDecision=" << static_cast<uint32_t> (decision.admitDecision));

  const Time triggerTime = report.measurementTime;

  if (decision.shouldHandover)
    {
      const Time execStart = Simulator::Now ();
      const bool ok = ExecuteHandover (decision.request);
      const Time finishTime = Simulator::Now ();
      // 透明集中式调度下执行是瞬时的; 用 GEO 链路传播参数估算数据面中断时延。
      const Time interruption = ok ? EstimateInterruptionDelay (decision.request.executionType)
                                   : (finishTime - execStart);
      RecordHandoverOutcome (report, decision, ok, triggerTime, finishTime, interruption);
    }
  else if (decision.reason.find ("admission-blocked") != std::string::npos)
    {
      // 准入拒绝: 这是一次真实发生但失败的切换尝试。
      RecordHandoverOutcome (report, decision, false, triggerTime, Simulator::Now (), Time (0));
    }
}

void
NtnGwRrc::EnsureUeContextFromReport (const MeasReport& report)
{
  if (!m_scheduler->HasUeContext (report.ueId))
    {
      m_scheduler->AddUeContext (report.ueId);
      m_ueServingObjectMap[report.ueId] = report.servingObject.IsValid ()
                                              ? report.servingObject
                                              : report.bestNeighborObject;

      if (m_ueServingObjectMap[report.ueId].type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
        {
          m_scheduler->AddUeInfo (report.ueId, m_ueServingObjectMap[report.ueId].objectId);
          NS_LOG_INFO ("[HO] Initial access selected satellite for UE " << report.ueId
                       << " -> " << ServiceObjectToString (m_ueServingObjectMap[report.ueId]));
        }
      else
        {
          NS_LOG_INFO ("[HO] Initial access selected ground for UE " << report.ueId
                       << " -> " << ServiceObjectToString (m_ueServingObjectMap[report.ueId]));
          if (m_admitControl != nullptr)
            {
              const SatUeContext ctx = m_scheduler->GetUeContext (report.ueId);
              m_admitControl->RegisterUeToGround (report.ueId,
                                                  m_ueServingObjectMap[report.ueId].objectId,
                                                  ctx.priority,
                                                  ctx.utType,
                                                  ctx.trafficType,
                                                  report.position);
            }
        }
    }
  else if (report.servingObject.IsValid ())
    {
      m_ueServingObjectMap[report.ueId] = report.servingObject;
    }
}

HandoverDecisionResult
NtnGwRrc::DecideHandoverTarget (const MeasReport& report)
{
  HandoverDecisionResult result;
  result.reason = "no-action";

  if (!report.bestNeighborObject.IsValid ())
    {
      result.reason = "no-valid-neighbor";
      return result;
    }

  if (report.servingObject == report.bestNeighborObject)
    {
      result.reason = "serving-equals-target";
      return result;
    }

  if (report.eventType == MeasEventType::MEAS_EVENT_SERVING_UNAVAILABLE)
    {
      result.reason = "serving-unavailable";
    }
  else if (report.eventType == MeasEventType::MEAS_EVENT_A3)
    {
      result.reason = "a3-triggered";
    }
  else
    {
      result.reason = "unsupported-event";
      return result;
    }

  result.request.rnti = report.ueId;
  result.request.sourceObject = report.servingObject;
  result.request.targetObject = report.bestNeighborObject;
  result.request.executionType = ClassifyHandoverExecution (report.servingObject,
                                                            report.bestNeighborObject);
  result.request.groundAvailable = report.groundAvailable;
  result.request.satelliteAvailable = report.satelliteAvailable;

  if (result.request.executionType == HandoverExecutionType::HANDOVER_EXEC_UNKNOWN)
    {
      result.reason += "|unsupported-path";
      return result;
    }

  const ServicePriority priority = m_scheduler->GetUePriority (report.ueId);
  const uint32_t requiredRbs = m_scheduler->EstimateRequiredRbs (report.ueId);

  if (result.request.targetObject.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM)
    {
      const uint32_t sourceBeamId =
          (report.servingObject.type == ServiceObjectType::SERVICE_OBJECT_SAT_BEAM) ? report.servingObject.objectId : 0;
      result.admitDecision = (m_admitControl != nullptr)
                                 ? m_admitControl->CanHandoverUe (sourceBeamId,
                                                                  report.bestNeighborObject.objectId,
                                                                  priority,
                                                                  requiredRbs)
                                 : AdmitDecision::ADMIT_OK;
    }
  else
    {
      result.admitDecision = (m_admitControl != nullptr)
                                 ? m_admitControl->CanAdmitGroundUe (report.bestNeighborObject.objectId,
                                                                     priority,
                                                                     requiredRbs)
                                 : AdmitDecision::ADMIT_OK;
    }

  NS_LOG_INFO ("[HO] UE " << report.ueId << " candidate decision"
               << " | source=" << ServiceObjectToString (report.servingObject)
               << " | target=" << ServiceObjectToString (report.bestNeighborObject)
               << " | event=" << static_cast<uint32_t> (report.eventType)
               << " | admitDecision=" << static_cast<uint32_t> (result.admitDecision));

  if (result.admitDecision == AdmitDecision::ADMIT_OK)
    {
      result.shouldHandover = true;
      return result;
    }

  result.reason += "|admission-blocked";
  return result;
}

// =============================================================================
// 切换执行编排
// =============================================================================

bool
NtnGwRrc::ExecuteHandover (const HandoverExecutionRequest& request)
{
  switch (request.executionType)
    {
    case HandoverExecutionType::HANDOVER_EXEC_BEAM_TO_BEAM:
      return ExecuteBeamHandover (request);
    case HandoverExecutionType::HANDOVER_EXEC_GROUND_TO_SAT:
      return ExecuteGroundToSatHandover (request);
    case HandoverExecutionType::HANDOVER_EXEC_SAT_TO_GROUND:
      return ExecuteSatToGroundHandover (request);
    default:
      NS_LOG_WARN ("[HO] Unsupported execution type for UE " << request.rnti);
      return false;
    }
}

bool
NtnGwRrc::ExecuteBeamHandover (const HandoverExecutionRequest& request)
{
  NS_LOG_INFO ("[HO] Execute beam -> beam"
               << " | UE=" << request.rnti
               << " | source=" << ServiceObjectToString (request.sourceObject)
               << " | target=" << ServiceObjectToString (request.targetObject));

  const SatUeContext oldCtx = m_scheduler->GetUeContext (request.rnti);
  m_scheduler->ExecuteHandover (request.rnti, request.targetObject.objectId);
  m_ueServingObjectMap[request.rnti] = request.targetObject;

  if (m_admitControl != nullptr)
    {
      m_admitControl->UnregisterUeFromBeam (request.rnti, request.sourceObject.objectId);
      m_admitControl->RegisterUeToBeam (request.rnti,
                                        request.targetObject.objectId,
                                        oldCtx.priority,
                                        oldCtx.utType,
                                        oldCtx.trafficType,
                                        oldCtx.position);
    }
  return true;
}

bool
NtnGwRrc::ExecuteGroundToSatHandover (const HandoverExecutionRequest& request)
{
  NS_LOG_INFO ("[HO] Execute ground -> sat"
               << " | UE=" << request.rnti
               << " | source=" << ServiceObjectToString (request.sourceObject)
               << " | target=" << ServiceObjectToString (request.targetObject));

  if (!m_scheduler->HasUeContext (request.rnti))
    {
      NS_LOG_WARN ("[HO] ground -> sat failed: missing UE context for UE " << request.rnti);
      return false;
    }

  if (m_admitControl != nullptr)
    {
      m_admitControl->UnregisterUeFromGround (request.rnti, request.sourceObject.objectId);
    }

  m_scheduler->AddUeInfo (request.rnti, request.targetObject.objectId);
  m_ueServingObjectMap[request.rnti] = request.targetObject;

  if (m_admitControl != nullptr)
    {
      const SatUeContext ctx = m_scheduler->GetUeContext (request.rnti);
      m_admitControl->RegisterUeToBeam (request.rnti,
                                        request.targetObject.objectId,
                                        ctx.priority,
                                        ctx.utType,
                                        ctx.trafficType,
                                        ctx.position);
    }

  return true;
}

bool
NtnGwRrc::ExecuteSatToGroundHandover (const HandoverExecutionRequest& request)
{
  NS_LOG_INFO ("[HO] Execute sat -> ground"
               << " | UE=" << request.rnti
               << " | source=" << ServiceObjectToString (request.sourceObject)
               << " | target=" << ServiceObjectToString (request.targetObject));

  if (!m_scheduler->HasUeContext (request.rnti))
    {
      NS_LOG_WARN ("[HO] sat -> ground failed: missing UE context for UE " << request.rnti);
      return false;
    }

  const SatUeContext oldCtx = m_scheduler->GetUeContext (request.rnti);

  HandoverTargetInfo targetInfo;
  targetInfo.targetType = HandoverTargetType::HANDOVER_TARGET_GROUND;
  targetInfo.targetObject = request.targetObject;
  targetInfo.targetBeamId = 0;
  targetInfo.targetGroundId = request.targetObject.objectId;
  targetInfo.targetAvailable = request.groundAvailable;

  HandoverUeContext hoCtx = m_scheduler->ExportUeContext (request.rnti, targetInfo);
  // ExportUeContext 不再写 sourceObject, 由 RRC 用自己的 servingObject 记录补齐
  hoCtx.sourceObject = m_ueServingObjectMap.count (request.rnti)
                           ? m_ueServingObjectMap[request.rnti]
                           : request.sourceObject;

  m_scheduler->SetUeBeamId (request.rnti, 0);
  m_ueServingObjectMap[request.rnti] = request.targetObject;
  if (m_ueGroundMetricMap.find (request.rnti) == m_ueGroundMetricMap.end ())
    {
      m_ueGroundMetricMap[request.rnti] = 0.0;
    }

  if (m_admitControl != nullptr)
    {
      m_admitControl->RegisterUeToGround (request.rnti,
                                          request.targetObject.objectId,
                                          oldCtx.priority,
                                          oldCtx.utType,
                                          oldCtx.trafficType,
                                          oldCtx.position);
    }

  NS_LOG_INFO ("[HO] sat -> ground completed"
               << " | UE=" << request.rnti
               << " | bufferedBytes=" << hoCtx.unsentBufferBytes);
  return true;
}

// =============================================================================
// 切换统计
// =============================================================================

Time
NtnGwRrc::EstimateInterruptionDelay (HandoverExecutionType type) const
{
  // 数据面中断 = 下行重配命令 (GEO 单程) + 上行目标接入 (GEO 单程)
  // 直接复用 scheduler 已配的 GEO 链路传播参数。
  switch (type)
    {
    case HandoverExecutionType::HANDOVER_EXEC_BEAM_TO_BEAM:
    case HandoverExecutionType::HANDOVER_EXEC_GROUND_TO_SAT:
    case HandoverExecutionType::HANDOVER_EXEC_SAT_TO_GROUND:
      return m_scheduler->GetRarTxDelay () + m_scheduler->GetUplinkPropDelay ();
    default:
      return Time (0);
    }
}

void
NtnGwRrc::RecordHandoverOutcome (const MeasReport& report,
                                 const HandoverDecisionResult& decision,
                                 bool success,
                                 Time triggerTime,
                                 Time finishTime,
                                 Time interruptionDelay)
{
  HandoverStatsRecord record;
  record.rnti = report.ueId;
  record.sourceObject = decision.request.sourceObject.IsValid ()
                            ? decision.request.sourceObject
                            : report.servingObject;
  record.targetObject = decision.request.targetObject.IsValid ()
                            ? decision.request.targetObject
                            : report.bestNeighborObject;
  record.executionType = decision.request.executionType;
  record.triggerEvent = report.eventType;
  record.triggerTime = triggerTime;
  record.finishTime = finishTime;
  record.handoverDelay = (finishTime >= triggerTime) ? (finishTime - triggerTime) : Time (0);
  record.interruptionDelay = interruptionDelay;
  record.success = success;
  record.reason = decision.reason;

  ++m_hoAttempts;
  if (success)
    {
      ++m_hoSuccesses;
    }
  else
    {
      ++m_hoFailures;
    }

  m_handoverRecords.push_back (record);
  m_handoverTrace (record);

  NS_LOG_INFO ("[HO-STATS] UE " << record.rnti
               << " | " << HandoverExecTypeToString (record.executionType)
               << " | event=" << MeasEventTypeToString (record.triggerEvent)
               << " | result=" << (success ? "SUCCESS" : "FAIL")
               << " | hoDelay=" << record.handoverDelay.GetMilliSeconds () << "ms"
               << " | interrupt=" << record.interruptionDelay.GetMilliSeconds () << "ms"
               << " | reason=" << record.reason);
}

const std::vector<HandoverStatsRecord>&
NtnGwRrc::GetHandoverRecords () const
{
  return m_handoverRecords;
}

HandoverStatsSummary
NtnGwRrc::GetHandoverStats () const
{
  HandoverStatsSummary summary;
  summary.attempts = m_hoAttempts;
  summary.successes = m_hoSuccesses;
  summary.failures = m_hoFailures;
  summary.failureRate = (m_hoAttempts > 0)
                            ? static_cast<double> (m_hoFailures) / m_hoAttempts
                            : 0.0;

  double sumHoDelayMs = 0.0;
  double sumInterruptMs = 0.0;
  uint32_t successCount = 0;
  for (const auto& r : m_handoverRecords)
    {
      ++summary.attemptsByType[r.executionType];
      if (r.success)
        {
          ++summary.successesByType[r.executionType];
          const double hoMs = r.handoverDelay.GetMilliSeconds ();
          const double intMs = r.interruptionDelay.GetMilliSeconds ();
          sumHoDelayMs += hoMs;
          sumInterruptMs += intMs;
          summary.maxHandoverDelayMs = std::max (summary.maxHandoverDelayMs, hoMs);
          summary.maxInterruptionDelayMs = std::max (summary.maxInterruptionDelayMs, intMs);
          ++successCount;
        }
    }
  if (successCount > 0)
    {
      summary.avgHandoverDelayMs = sumHoDelayMs / successCount;
      summary.avgInterruptionDelayMs = sumInterruptMs / successCount;
    }
  return summary;
}

void
NtnGwRrc::ExportHandoverStats (const std::string& filepath) const
{
  std::ofstream out (filepath.c_str ());
  if (!out.is_open ())
    {
      NS_LOG_WARN ("[HO-STATS] Cannot open file for writing: " << filepath);
      return;
    }

  const HandoverStatsSummary summary = GetHandoverStats ();

  out << "# GEO-SAT Handover Statistics\n";
  out << "# attempts successes failures failureRate "
         "avgHoDelayMs maxHoDelayMs avgInterruptMs maxInterruptMs\n";
  out << "SUMMARY "
      << summary.attempts << " "
      << summary.successes << " "
      << summary.failures << " "
      << summary.failureRate << " "
      << summary.avgHandoverDelayMs << " "
      << summary.maxHandoverDelayMs << " "
      << summary.avgInterruptionDelayMs << " "
      << summary.maxInterruptionDelayMs << "\n";

  out << "# per-type: execType attempts successes\n";
  for (const auto& kv : summary.attemptsByType)
    {
      auto sIt = summary.successesByType.find (kv.first);
      const uint32_t succ = (sIt != summary.successesByType.end ()) ? sIt->second : 0;
      out << "BYTYPE " << HandoverExecTypeToString (kv.first) << " "
          << kv.second << " " << succ << "\n";
    }

  out << "# per-event records: time(s) ue execType triggerEvent "
         "source target result hoDelayMs interruptMs reason\n";
  for (const auto& r : m_handoverRecords)
    {
      out << "EVENT "
          << r.finishTime.GetSeconds () << " "
          << r.rnti << " "
          << HandoverExecTypeToString (r.executionType) << " "
          << MeasEventTypeToString (r.triggerEvent) << " "
          << ServiceObjectToString (r.sourceObject) << " "
          << ServiceObjectToString (r.targetObject) << " "
          << (r.success ? "SUCCESS" : "FAIL") << " "
          << r.handoverDelay.GetMilliSeconds () << " "
          << r.interruptionDelay.GetMilliSeconds () << " "
          << r.reason << "\n";
    }

  out.close ();
  NS_LOG_INFO ("[HO-STATS] Exported " << m_handoverRecords.size ()
               << " handover records to " << filepath);
}

// ============================================================
// Paging
// ============================================================

void
NtnGwRrc::SetPagingCallback (Callback<void, uint16_t> cb)
{
  m_pagingCb = cb;
}

void
NtnGwRrc::SendPaging (uint16_t ueId)
{
  NS_LOG_INFO ("[GW RRC] Send Paging for ueId=" << ueId
               << " at t=" << Simulator::Now ().GetSeconds () << " s");
  m_pagingTrace (ueId, static_cast<uint64_t> (Simulator::Now ().GetNanoSeconds ()));
  if (!m_pagingCb.IsNull ())
    {
      // Paging 信令本身: 假设走 SIB/PCH 即可, 这里直接调 UE 回调 (demo/test 接到 SatUtRrc)
      m_pagingCb (ueId);
    }
  else
    {
      NS_LOG_WARN ("[GW RRC] SendPaging: m_pagingCb not set");
    }
}

} // namespace ns3
