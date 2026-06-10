/*
 * 文件路径：contrib/geo-sat/model/geo-beam-scheduler.cc
 * 功能：GEO 卫星波束感知 MAC 层调度器实现 - 融合 RRM 版
 */
#include "geo-beam-scheduler.h"
#include "cqi-predictor.h"
#include "cqi-amc-controller.h"
#include "prach-detection-model.h"
#include "harq-manager.h"
#include "result-writer.h"
#include "ns3/nr-amc.h"
#include "ns3/nr-mac-short-bsr-ce.h"
#include "ns3/spectrum-value.h"
#include "ns3/simulator.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace ns3 {

// (ServiceObjectToString / HandoverExecTypeToString / MeasEventTypeToString
//  已随 RRC 相关逻辑迁移到 ntn-gw-rrc.cc, scheduler.cc 不再需要这些辅助。)

namespace
{

SfnSf
NormalizeUlLedgerSfnSf (const SfnSf& sfnSf)
{
  if (sfnSf.GetFrame () == 0 && sfnSf.GetSubframe () == 0 && sfnSf.GetSlot () == 0)
    {
      return SfnSf (0, 0, 0, 0);
    }

  return sfnSf;
}

} // namespace

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
                   "Fallback path loss for the RA(Msg3) phase before a UE context exists.",
                   DoubleValue (190.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_referencePathLossDb),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("RefSatEirpDbm",
                   "Reference per-beam satellite EIRP (dBm) used to derive per-UE path loss from RSRP: PL = EIRP - RSRP.",
                   DoubleValue (70.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_refSatEirpDbm),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("DlCqiValidityTime",
                   "How long a DL CQI sample remains valid before scheduler falls back to a safer CQI.",
                   TimeValue (Seconds (2)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_dlCqiValidityTime),
                   MakeTimeChecker ())
    .AddAttribute ("UlCqiValidityTime",
                   "How long an UL CQI sample remains valid before scheduler falls back to a safer CQI.",
                   TimeValue (Seconds (2)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_ulCqiValidityTime),
                   MakeTimeChecker ())
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
    .AddAttribute ("DataAdmitRbCap",
                   "Leniency bound (in RB) on the per-TTI DATA/BEST_EFFORT admission gate. "
                   "Admission is a coarse 'will this UE get any service this TTI' gate; actual "
                   "allocation is opportunistic up to the full CQI-driven demand within the beam "
                   "budget. The gate therefore tests min(rawRequiredRbs, this cap) instead of the "
                   "whole buffer, so a UE is not falsely rejected merely because its full buffer "
                   "exceeds the residual budget. Set to 1 for the loosest (channel-agnostic) gate; "
                   "raise it for a stricter gate.",
                   UintegerValue (4),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_dataAdmitRbCap),
                   MakeUintegerChecker<uint32_t> (1, 25))
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
                   MakeUintegerChecker<uint32_t> (0, 1))
    .AddAttribute ("SpsEnabled",
                   "Enable static / semi-persistent scheduling (fixed RB grants for voice/emergency/portable floor). "
                   "When false the scheduler behaves exactly as the dynamic-only baseline.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_spsEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("SpsVoiceRbs",
                   "Fixed RB count granted per SPS period to VOICE traffic.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_spsVoiceRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("SpsEmergencyRbs",
                   "Fixed RB count granted per SPS period to EMERGENCY traffic.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_spsEmergencyRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("SpsPortableFloorRbs",
                   "Fixed RB floor granted per SPS period to UT_PORTABLE terminals.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_spsPortableFloorRbs),
                   MakeUintegerChecker<uint32_t> (0, 25))
    .AddAttribute ("SpsPeriod",
                   "Minimum interval between two SPS fixed grants for the same UE.",
                   TimeValue (MilliSeconds (20)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_spsPeriod),
                   MakeTimeChecker ())
    // ---------- 阶段1: 半静态 configured-grant 状态机 ----------
    .AddAttribute ("SpsMode",
                   "Semi-persistent scheduling mode (mutually exclusive at runtime): "
                   "0=Off (default, dynamic-only baseline), "
                   "1=Legacy (old periodic fixed-RB path TryScheduleDlSps, for A/B), "
                   "2=Configured (persistent configured-grant state machine). "
                   "Configured mode also requires ResourceManager::DlSpsRegionRbs>0.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_spsMode),
                   MakeUintegerChecker<uint32_t> (0, 2))
    .AddAttribute ("SpsGrantPeriod",
                   "Configured-grant period for SPS_CONFIGURED (e.g. 20ms VoIP).",
                   TimeValue (MilliSeconds (20)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_spsGrantPeriod),
                   MakeTimeChecker ())
    .AddAttribute ("SpsImplicitReleaseAfter",
                   "Release a configured grant after this many consecutive empty-or-conflict periods.",
                   UintegerValue (3),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_spsImplicitReleaseAfter),
                   MakeUintegerChecker<uint8_t> (1, 255))
    .AddAttribute ("SpsActivationMinBytes",
                   "Minimum DL buffer bytes required to activate a configured grant.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_spsActivationMinBytes),
                   MakeUintegerChecker<uint32_t> (0))
    .AddAttribute ("SpsStaggerEnabled",
                   "Spread SPS grants' first reuse instant by a per-UE phase offset (load smoothing; "
                   "required for frequency-domain reuse gain once overbooking is enabled).",
                   BooleanValue (true),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_spsStaggerEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("SpsHarqEnabled",
                   "Attach HARQ to SPS transmissions. Default false: GEO periodic small packets "
                   "use a conservative MCS instead of per-packet HARQ (8 processes cannot cover "
                   "20ms-period traffic across a ~600ms RTT).",
                   BooleanValue (false),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_spsHarqEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("ClpcEnabled",
                   "Enable uplink closed-loop power control (TPC from measured SINR feedback). "
                   "When false, UL power control is pure open-loop.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_clpcEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("TargetUlSinrDb",
                   "Target received UL SINR (dB) the closed loop drives each UE towards.",
                   DoubleValue (3.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_targetUlSinrDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ClpcMinDb",
                   "Lower clamp for the accumulated closed-loop offset f(i) (dB).",
                   DoubleValue (-10.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_clpcMinDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ClpcMaxDb",
                   "Upper clamp for the accumulated closed-loop offset f(i) (dB).",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_clpcMaxDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ClpcLoopDelay",
                   "Extra delay before a TPC correction takes effect (models GEO closed-loop latency). "
                   "0 applies immediately (UL feedback is already post-propagation).",
                   TimeValue (MilliSeconds (0)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_clpcLoopDelay),
                   MakeTimeChecker ())
    .AddAttribute ("DlPowerAllocEnabled",
                   "Enable dynamic downlink power allocation across UEs within the beam power budget. "
                   "When false, DL DCI carries txPower=0 (legacy behavior).",
                   BooleanValue (false),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_dlPowerAllocEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("NomaEnabled",
                   "Enable downlink power-domain NOMA (pair a far/weak UE onto a near/strong UE's RBs).",
                   BooleanValue (false),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_nomaEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("NomaFarPowerFraction",
                   "Power fraction beta given to the far (weak) NOMA user; near user gets (1-beta).",
                   DoubleValue (0.8),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_nomaFarPowerFraction),
                   MakeDoubleChecker<double> (0.5, 0.95))
    .AddAttribute ("NomaMinCqiGap",
                   "Minimum CQI gap between near and far UE required to form a NOMA pair.",
                   DoubleValue (4.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_nomaMinCqiGap),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("NomaMaxFar",
                   "Maximum number of far (weak) UEs that may be superposed onto a single near "
                   "(strong) UE's RBs. 1 = classic 1:1 NOMA (default); N>1 enables 1:N where the "
                   "beta power share is split among the far UEs.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&GeoBeamScheduler::m_nomaMaxFar),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("DlSchedHorizon",
                   "Prediction horizon for DL CQI (now -> actual DL transmission instant, ~one DL round trip).",
                   TimeValue (MilliSeconds (300)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_dlSchedHorizon),
                   MakeTimeChecker ())
    .AddAttribute ("UlSchedHorizon",
                   "Prediction horizon for UL CQI (now -> actual UL transmission instant, ~one UL round trip).",
                   TimeValue (MilliSeconds (600)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_ulSchedHorizon),
                   MakeTimeChecker ())
    .AddAttribute ("UtTxAntennaGainDb",
                   "UT transmit antenna gain G_ut_tx (dB) used in the UL SINR link budget feedback.",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_utTxAntennaGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SatRxAntennaGainDb",
                   "Satellite receive antenna gain G_sat_rx (dB) used in the UL SINR link budget feedback.",
                   DoubleValue (30.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_satRxAntennaGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UlNoiseFigureDb",
                   "gNB receiver noise figure NF (dB) for the thermal noise floor in the UL SINR feedback.",
                   DoubleValue (7.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_noiseFigureDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UlSinrMeasErrorSigmaDb",
                   "Std-dev (dB) of the Gaussian measurement error added to the received UL SINR "
                   "estimate (models channel-estimation noise). 0 disables measurement noise.",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&GeoBeamScheduler::m_ulSinrMeasErrorSigmaDb),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("UlCqiFeedbackDelay",
                   "Delay before the measured UL SINR feedback reaches the scheduler "
                   "(UE->satellite->gNB uplink one-way propagation).",
                   TimeValue (MilliSeconds (300)),
                   MakeTimeAccessor (&GeoBeamScheduler::m_ulCqiFeedbackDelay),
                   MakeTimeChecker ())
    .AddAttribute ("UseCqiAmcPredictor",
                   "If true, inject a CqiAmcPredictor (Kalman prediction + OLLA) as the "
                   "scheduler's CQI predictor so resource management uses filtered CQI.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&GeoBeamScheduler::m_useCqiAmcPredictor),
                   MakeBooleanChecker ());
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
    m_refSatEirpDbm (70.0),
    m_dlCqiValidityTime (Seconds (2)),
    m_ulCqiValidityTime (Seconds (2)),
    m_srGrantRbs (1),
    m_srGrantMcs (4),
    m_dataAdmitRbCap (4),
    m_spsEnabled (false),
    m_spsVoiceRbs (2),
    m_spsEmergencyRbs (1),
    m_spsPortableFloorRbs (2),
    m_spsPeriod (MilliSeconds (20)),
    m_spsMode (0),
    m_spsGrantPeriod (MilliSeconds (20)),
    m_spsImplicitReleaseAfter (3),
    m_spsActivationMinBytes (1),
    m_spsStaggerEnabled (true),
    m_spsHarqEnabled (false),
    m_clpcEnabled (true),
    m_targetUlSinrDb (3.0),
    m_clpcMinDb (-10.0),
    m_clpcMaxDb (10.0),
    m_clpcLoopDelay (MilliSeconds (0)),
    m_utTxAntennaGainDb (0.0),
    m_satRxAntennaGainDb (30.0),
    m_noiseFigureDb (7.0),
    m_ulSinrMeasErrorSigmaDb (0.0),
    m_ulCqiFeedbackDelay (MilliSeconds (300)),
    m_dlPowerAllocEnabled (false),
    m_nomaEnabled (false),
    m_nomaFarPowerFraction (0.8),
    m_nomaMinCqiGap (4.0),
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
  m_dlSchedHorizon = MilliSeconds (300);
  m_ulSchedHorizon = MilliSeconds (600);
  m_resourceManager = CreateObject<ResourceManager> ();
  m_admitControl = CreateObject<AdmitControl> ();
  // 默认直通 CQI 预测器, 行为与未引入预测时完全一致; 用户可 SetCqiPredictor 替换
  m_cqiPredictor = CreateObject<CqiPredictor> ();
  m_dlAmc = CreateObject<NrAmc> ();
  m_dlAmc->SetDlMode ();
  m_ulAmc = CreateObject<NrAmc> ();
  m_ulAmc->SetUlMode ();
  m_admitControl->SetResourceManager (m_resourceManager);
  m_admitControl->SetHandoverExecutor (MakeCallback (&GeoBeamScheduler::ExecuteHandover, this));
  // 上行接收 SINR 估计误差随机源 (均值 0; 标准差由 m_ulSinrMeasErrorSigmaDb 在抽样时给出)
  m_ulSinrMeasErrorRv = CreateObject<NormalRandomVariable> ();
  m_ulSinrMeasErrorRv->SetAttribute ("Mean", DoubleValue (0.0));
}

// ============================================================
// qyh 增量钩子 (S2)
// ============================================================
void
GeoBeamScheduler::SetCqiPredictor (Ptr<CqiPredictor> cqiPredictor)
{
  m_cqiPredictor = cqiPredictor;
  // 显式注入视为用户接管: 不再做属性驱动的惰性注入, 避免覆盖。
  m_cqiAmcPredictorApplied = true;
  NS_LOG_INFO ("CqiPredictor " << (cqiPredictor ? "connected" : "cleared"));
}

void
GeoBeamScheduler::MaybeApplyCqiAmcPredictor ()
{
  // 幂等: 只在首次需要时按属性注入。属性在对象构造后才生效, 故惰性处理,
  // 保证"UseCqiAmcPredictor 设了就生效"。
  if (m_cqiAmcPredictorApplied)
    {
      return;
    }
  m_cqiAmcPredictorApplied = true;
  if (m_useCqiAmcPredictor)
    {
      Ptr<CqiAmcPredictor> predictor = CreateObject<CqiAmcPredictor> ();
      // 单一 DL CQI 真源: 若外部已注入 m_cqiAmc, 让适配器复用同一 DL controller,
      // 这样 GetSchedulingCqi(经适配器) 与 GetMcsForNewTx(经 m_cqiAmc) 共享同一
      // DL Kalman/OLLA, 不再状态发散、不再双喂。UL 仍用适配器自建独立实例
      // (m_cqiAmc 语义只覆盖 DL)。
      if (m_cqiAmc != nullptr)
        {
          predictor->SetDlController (m_cqiAmc);
        }
      // horizon 单一化: 自建兜底实例的前向视界来自调度器的 m_dl/ulSchedHorizon,
      // 取代 controller 内部写死的 510 slot (外部复用的 DL controller 保留其自有 H)。
      if (m_cqiAmc == nullptr)
        {
          predictor->SetDlHorizon (m_dlSchedHorizon);
        }
      predictor->SetUlHorizon (m_ulSchedHorizon);
      m_cqiPredictor = predictor;
      NS_LOG_INFO ("CqiAmcPredictor injected (Kalman prediction + OLLA) for beam "
                   << m_myBeamId
                   << (m_cqiAmc != nullptr ? " [reusing external DL controller]" : ""));
    }
}

void
GeoBeamScheduler::SetPrachDetectionModel (Ptr<PrachDetectionModel> model)
{
  m_prachDetectionModel = model;
  NS_LOG_INFO ("PrachDetectionModel " << (model ? "connected" : "cleared"));
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
void
GeoBeamScheduler::DoSchedDlCqiInfoReq (const NrMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);

  for (const auto& cqiInfo : params.m_cqiList)
    {
      if (cqiInfo.m_rnti == 0)
        {
          continue;
        }

      const double widebandCqi = ExtractWidebandDlCqi (cqiInfo);
      if (widebandCqi <= 0.0)
        {
          NS_LOG_WARN ("[DL CQI] UE " << cqiInfo.m_rnti << " CQI payload为空，忽略");
          continue;
        }

      UpdateUeDlCqi (cqiInfo.m_rnti, widebandCqi);
      NS_LOG_INFO ("[DL CQI] UE " << cqiInfo.m_rnti
                   << " 通过 NR SCHED SAP 更新 CQI=" << widebandCqi);
    }
}

void
GeoBeamScheduler::DoSchedUlCqiInfoReq (const NrMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);

  if (params.m_ulCqi.m_type != UlCqiInfo::PUSCH)
    {
      NS_LOG_INFO ("[UL CQI] 仅处理 PUSCH UL CQI, 当前类型="
                   << static_cast<uint32_t> (params.m_ulCqi.m_type));
      return;
    }

  auto itAlloc = m_ulCqiAllocationMap.find (params.m_sfnSf.GetEncoding ());
  if (itAlloc == m_ulCqiAllocationMap.end ())
    {
      NS_LOG_WARN ("[UL CQI] 未找到与 " << params.m_sfnSf
                   << " 对应的 UL allocation ledger");
      return;
    }

  auto& allocations = itAlloc->second;
  auto allocIt = std::find_if (allocations.begin (),
                               allocations.end (),
                               [&] (const PendingUlCqiAllocation& allocation) {
                                 return allocation.symStart == params.m_symStart;
                               });
  if (allocIt == allocations.end ())
    {
      NS_LOG_WARN ("[UL CQI] 在 " << params.m_sfnSf
                   << " 中找不到 symStart=" << static_cast<uint32_t> (params.m_symStart)
                   << " 的 UL allocation");
      return;
    }

  const uint16_t targetRnti = allocIt->rnti;

  // 标准 SAP 入口(PHY 实测)经 (sfnSf,symStart) 反查到 rnti 后, 复用统一的
  // 测量应用逻辑(CQI 更新 + CLPC)。CLPC 回灌(DeliverUlSinrFeedback)不再走此反查,
  // 而直接透传 rnti, 见该函数。
  const bool applied = ApplyUlSinrMeasurement (targetRnti, params.m_ulCqi);

  allocations.erase (allocIt);
  if (allocations.empty ())
    {
      m_ulCqiAllocationMap.erase (itAlloc);
    }
  if (applied)
    {
      NS_LOG_INFO ("[UL CQI] UE " << targetRnti
                   << " 通过 NR SCHED SAP 更新 CQI/CLPC"
                   << " | slot=" << params.m_sfnSf
                   << " | symStart=" << static_cast<uint32_t> (params.m_symStart));
    }
}

// 统一的上行实测应用: 更新该 UE 的 UL CQI 并(开启时)推进 CLPC 修正项 f(i)。
// 直接以 rnti 为键, 不依赖 (sfnSf,symStart) 反查; 标准 SAP 入口与 CLPC 直透回灌共用。
// 注(项 4): 上行链路自适应只走 UL CQI 更新 + CLPC, **没有 OLLA 外环**。OLLA(基于
// HARQ 首传 ACK/NACK 的有效 CQI 偏置, 见 HandleDlHarqFeedbackResult / OnHarqFeedback)
// 仅用于下行。上行 HARQ 反馈不喂 m_cqiAmc/m_cqiPredictor 的 OLLA。
bool
GeoBeamScheduler::ApplyUlSinrMeasurement (uint16_t rnti, const UlCqiInfo& ulCqiInfo)
{
  const double widebandCqi = EstimateWidebandUlCqiFromSinr (ulCqiInfo);
  if (widebandCqi <= 0.0)
    {
      NS_LOG_WARN ("[UL CQI] UE " << rnti << " UL SINR 样本为空或无效");
      return false;
    }

  UpdateUeUlCqi (rnti, widebandCqi);

  // 闭环功控: 用本次实测 UL SINR 与目标比较, 更新该 UE 的累积修正项 f(i)。
  const double measuredSinrDb = AverageUlSinrDb (ulCqiInfo);
  if (m_clpcEnabled && std::isfinite (measuredSinrDb))
    {
      UpdateClosedLoopPowerControl (rnti, measuredSinrDb);
    }
  return true;
}

void
GeoBeamScheduler::DoSchedUlMacCtrlInfoReq (const NrMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);

  for (const auto& element : params.m_macCeList)
    {
      if (element.m_macCeType != MacCeElement::BSR)
        {
          continue;
        }

      uint32_t totalBufferBytes = 0;
      for (uint8_t lcg = 0; lcg < element.m_macCeValue.m_bufferStatus.size (); ++lcg)
        {
          totalBufferBytes +=
            NrMacShortBsrCe::FromLevelToBytes (element.m_macCeValue.m_bufferStatus.at (lcg));
        }

      BsR_MAC_CE bsr;
      bsr.rnti = element.m_rnti;
      bsr.lcgId = 0;
      bsr.bufferSize = totalBufferBytes;
      ReceiveBsr (bsr);
    }
}
void GeoBeamScheduler::DoSchedUlTriggerReq (const NrMacSchedSapProvider::SchedUlTriggerReqParameters& params)
{//每一轮 UL 调度开始前，先按 beam 重置上行 RB 预算，再运行上行调度算法
  m_currentUlTriggerSfnSf = NormalizeUlLedgerSfnSf (params.m_snfSf);
  m_hasCurrentUlTriggerSfnSf = true;
  const uint64_t schedulingRoundId = ++m_nextUlSchedulingRoundId;//标识当前调度周期

  if (m_beamToUesMap.empty ())
    {//对当前 beam 开启一个 UL 调度周期，重置该 beam 的 UL 资源预算
      BeginUlSchedulingPeriod (m_myBeamId, schedulingRoundId);
      m_hasCurrentUlTriggerSfnSf = false;
      return;
    }

  for (const auto& beamPair : m_beamToUesMap)
    {
      BeginUlSchedulingPeriod (beamPair.first, schedulingRoundId);
    }

  RunUlScheduler ();//进入上行调度流程
  m_hasCurrentUlTriggerSfnSf = false;
}
//UE 发送 SR 后触发的调度接口
void GeoBeamScheduler::DoSchedUlSrInfoReq (const NrMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
  m_currentUlTriggerSfnSf = NormalizeUlLedgerSfnSf (params.m_snfSf);
  m_hasCurrentUlTriggerSfnSf = true;
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
  m_hasCurrentUlTriggerSfnSf = false;
}
void GeoBeamScheduler::DoSchedSetMcs (uint32_t mcs) {}
void GeoBeamScheduler::DoSchedDlRachInfoReq (const NrMacSchedSapProvider::SchedDlRachInfoReqParameters& params) {}
uint8_t GeoBeamScheduler::GetDlCtrlSyms () const { return 1; }
uint8_t GeoBeamScheduler::GetUlCtrlSyms () const { return 1; }
int64_t GeoBeamScheduler::AssignStreams (int64_t stream)
{
  if (m_ulSinrMeasErrorRv)
    {
      m_ulSinrMeasErrorRv->SetStream (stream);
      return 1;
    }
  return 0;
}

void GeoBeamScheduler::DoSchedDlTriggerReq (const NrMacSchedSapProvider::SchedDlTriggerReqParameters& params) 
{
    RunScheduler(); 
}

void GeoBeamScheduler::Initialize (uint32_t beamId, uint8_t scs)
{
  m_myBeamId = beamId;
  NS_LOG_INFO ("Scheduler Initialized for Beam " << beamId << ", SCS config: " << (uint16_t)scs);
  MaybeApplyCqiAmcPredictor ();
}

void GeoBeamScheduler::ConfigPrachResources () { MaybeApplyCqiAmcPredictor (); }

void GeoBeamScheduler::ReservePrachResources ()
{
  m_prachReservedRbs = 6; //每个 beam 的下行资源中，先预留 6 RB，不给普通 DL 业务使用。
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
      ctx.currentBeamId = 0;
      ctx.latestDlCqi = 7.0;//默认 DL CQI
      ctx.latestUlCqi = 7.0;//默认 UL CQI
      ctx.latestRsrp = -120.0;//默认 RSRP
      ctx.pathLossDb = DerivePathLossFromRsrp (ctx.latestRsrp);//默认路损=70-(-120)=190dB
      // 兼容字段 (0603): latestCqi == latestDlCqi, bufferStatus == dlBufferStatus
      ctx.latestCqi = ctx.latestDlCqi;
      ctx.bufferStatus = 0;
      ctx.averageThroughput = 0.001;
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
      ctx.lastScheduledTime = Time (0);
      ctx.lastDlScheduledTime = Time (0);
      ctx.lastUlScheduledTime = Time (0);
      ctx.lastDlCqiUpdateTime = NanoSeconds (-1);
      ctx.lastUlCqiUpdateTime = NanoSeconds (-1);
      ctx.lastDlSpsTime = Seconds (-1);
      ctx.lastUlSpsTime = Seconds (-1);
      ctx.clpcOffsetDb = 0.0;
      m_ueContextMap[rnti] = ctx;
      NS_LOG_INFO ("UE Context created for RNTI " << rnti
                   << " (Type: " << utType << ", Traffic: " << trafficType
                   << ", Priority: " << (uint32_t)ctx.priority << ")");
  }
}

void GeoBeamScheduler::AddUeInfo (uint16_t rnti, uint32_t targetBeamId)
{
  // 注意: serving object 的记账已迁移到 NtnGwRrc, 这里只负责 MAC 层
  // 调度所需的 currentBeamId / 波束 UE 列表 + AdmitControl 注册。
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
  for (auto it = m_dlHarqProcessMap.begin (); it != m_dlHarqProcessMap.end ();)
    {
      if (it->first.first == rnti)
        {
          it = m_dlHarqProcessMap.erase (it);
          continue;
        }
      ++it;
    }
  m_ueContextMap.erase(rnti);
}

// ---------- 暴露给 NtnGwRrc 等上层的 UE 上下文访问器 ----------
bool
GeoBeamScheduler::HasUeContext (uint16_t rnti) const
{
  return m_ueContextMap.find (rnti) != m_ueContextMap.end ();
}

SatUeContext
GeoBeamScheduler::GetUeContext (uint16_t rnti) const
{
  auto it = m_ueContextMap.find (rnti);
  if (it == m_ueContextMap.end ())
    {
      return SatUeContext {};
    }
  return it->second;
}

void
GeoBeamScheduler::SetUeBeamId (uint16_t rnti, uint32_t beamId)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.currentBeamId = beamId;
    }
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

    // 提取当前 UE 的内存快照 (并集字段: qyh 丰富字段 + 0603 ServiceObject/兼容字段;
    // serving object 由 NtnGwRrc 在调用前后补齐到 hoCtx.sourceObject / targetObject)
    SatUeContext& ctx = m_ueContextMap[rnti];
    HandoverUeContext hoCtx;
    hoCtx.rnti = rnti;
    hoCtx.sourceBeamId = ctx.currentBeamId;
    hoCtx.targetBeamId = targetBeamId;
    hoCtx.sourceObject = {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, ctx.currentBeamId};
    hoCtx.targetObject = {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, targetBeamId};
    // 0603 兼容字段
    hoCtx.latestCqi = ctx.latestDlCqi;
    hoCtx.unsentBufferBytes = ctx.dlBufferStatus;
    hoCtx.averageThroughput = ctx.dlAverageThroughput;
    // qyh 丰富字段
    hoCtx.latestDlCqi = ctx.latestDlCqi;
    hoCtx.latestUlCqi = ctx.latestUlCqi;
    hoCtx.latestRsrp = ctx.latestRsrp;
    hoCtx.pathLossDb = ctx.pathLossDb;
    hoCtx.clpcOffsetDb = ctx.clpcOffsetDb;
    hoCtx.unsentDlBufferBytes = ctx.dlBufferStatus;
    hoCtx.unsentUlBufferBytes = ctx.ulBufferStatus;
    hoCtx.pendingUlGrantRbs = ctx.pendingUlGrantRbs;
    hoCtx.pendingUlGrantBytes = ctx.pendingUlGrantBytes;
    hoCtx.srPending = ctx.srPending;
    hoCtx.dlAverageThroughput = ctx.dlAverageThroughput;
    hoCtx.ulAverageThroughput = ctx.ulAverageThroughput;
    hoCtx.position = ctx.position;
    hoCtx.utType = ctx.utType;
    hoCtx.trafficType = ctx.trafficType;
    hoCtx.lastDlScheduledTime = ctx.lastDlScheduledTime;
    hoCtx.lastUlScheduledTime = ctx.lastUlScheduledTime;
    hoCtx.lastDlCqiUpdateTime = ctx.lastDlCqiUpdateTime;
    hoCtx.lastUlCqiUpdateTime = ctx.lastUlCqiUpdateTime;
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

HandoverUeContext
GeoBeamScheduler::ExportUeContext (uint16_t rnti, const HandoverTargetInfo& targetInfo)
{
    const uint32_t targetBeamId = (targetInfo.targetType == HandoverTargetType::HANDOVER_TARGET_SAT_BEAM)
                                      ? targetInfo.targetBeamId
                                      : 0;

    HandoverUeContext hoCtx = ExportUeContext (rnti, targetBeamId);
    hoCtx.targetObject = targetInfo.targetObject;
    // sourceObject 由调用方 (NtnGwRrc) 用其 servingObject map 覆盖。
    return hoCtx;
}

// 实现接口B：目标波束导入
void GeoBeamScheduler::ImportUeContext (const HandoverUeContext& hoCtx)
{
    NS_LOG_FUNCTION (this << hoCtx.rnti << hoCtx.targetBeamId);

    // 在目标波束重建 UE 的调度档案 (并集字段)
    SatUeContext ctx;
    ctx.rnti = hoCtx.rnti;
    ctx.currentBeamId = hoCtx.targetBeamId;
    ctx.latestDlCqi = hoCtx.latestDlCqi;
    ctx.latestUlCqi = hoCtx.latestUlCqi;
    ctx.latestRsrp = hoCtx.latestRsrp;
    ctx.pathLossDb = hoCtx.pathLossDb;
    ctx.clpcOffsetDb = hoCtx.clpcOffsetDb;
    // 兼容字段 (0603)
    ctx.latestCqi = hoCtx.latestDlCqi;
    ctx.bufferStatus = hoCtx.unsentDlBufferBytes;
    ctx.averageThroughput = hoCtx.dlAverageThroughput;
    ctx.dlBufferStatus = hoCtx.unsentDlBufferBytes;
    ctx.ulBufferStatus = hoCtx.unsentUlBufferBytes;
    ctx.pendingUlGrantRbs = hoCtx.pendingUlGrantRbs;
    ctx.pendingUlGrantBytes = hoCtx.pendingUlGrantBytes;
    ctx.srPending = hoCtx.srPending;
    ctx.dlAverageThroughput = hoCtx.dlAverageThroughput;
    ctx.ulAverageThroughput = hoCtx.ulAverageThroughput;
    ctx.position = hoCtx.position;
    ctx.utType = hoCtx.utType;
    ctx.trafficType = hoCtx.trafficType;
    ctx.priority = MapTrafficTypeToPriority (hoCtx.trafficType);
    ctx.wrrWeight = CalculateWrrWeight (ctx.priority, hoCtx.utType);
    ctx.lastScheduledTime = Simulator::Now ();
    ctx.lastDlScheduledTime = hoCtx.lastDlScheduledTime;
    ctx.lastUlScheduledTime = hoCtx.lastUlScheduledTime;
    ctx.lastDlCqiUpdateTime = hoCtx.lastDlCqiUpdateTime;
    ctx.lastUlCqiUpdateTime = hoCtx.lastUlCqiUpdateTime;
    ctx.lastDlSpsTime = Seconds (-1);
    ctx.lastUlSpsTime = Seconds (-1);

    m_ueContextMap[hoCtx.rnti] = ctx;
    if (hoCtx.targetBeamId != 0)
      {
        m_beamToUesMap[hoCtx.targetBeamId].push_back(hoCtx.rnti);
      }
    m_admissionQueueSince.erase (hoCtx.rnti);
    if (!hoCtx.dciCallback.IsNull ())
      {
        m_ueDciCallbackMap[hoCtx.rnti] = hoCtx.dciCallback;
      }

    // serving object 的记账已迁移到 NtnGwRrc; 但 AdmitControl 的 beam 记账仍需维护。
    if (m_admitControl != nullptr && hoCtx.targetBeamId != 0)
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

// 实现接口C：统筹执行
void GeoBeamScheduler::ExecuteHandover (uint16_t rnti, uint32_t targetBeamId)
{
    NS_LOG_INFO ("=== 🚀 发起星地波束间无缝切换 (UE: " << rnti << " -> Beam: " << targetBeamId << ") ===");

    // 因为你们目前的信关站调度器是集中式管理所有波束的，所以直接在这里调用导出和导入即可模拟状态转移。
    HandoverUeContext hoCtx = ExportUeContext (rnti, targetBeamId);
    ImportUeContext (hoCtx);
}

void GeoBeamScheduler::UpdateUeCsi (uint16_t rnti, double cqi)//兼容旧接口：共享 CQI 同时更新 DL/UL
{
  if (m_ueContextMap.find(rnti) != m_ueContextMap.end()) {
      SatUeContext& ctx = m_ueContextMap[rnti];
      ctx.latestDlCqi = cqi;
      ctx.latestUlCqi = cqi;
      ctx.latestCqi = cqi;   // 0603 兼容字段
      ctx.lastDlCqiUpdateTime = Simulator::Now ();
      ctx.lastUlCqiUpdateTime = Simulator::Now ();
      // 单一 DL 真源 (与 UpdateUeDlCqi 一致): 适配器优先, 否则退回 m_cqiAmc; 不双喂。
      if (DynamicCast<CqiAmcPredictor> (m_cqiPredictor) != nullptr)
        {
          m_cqiPredictor->RecordDlCqi (rnti, cqi, Simulator::Now ());
          m_cqiPredictor->RecordUlCqi (rnti, cqi, Simulator::Now ());
        }
      else
        {
          if (m_cqiAmc)
            {
              m_cqiAmc->OnCqiReport (rnti, cqi);
            }
          if (m_cqiPredictor != nullptr)
            {
              // 直通预测器的 UL Record 为 no-op; 自定义 UL 预测器仍可累积样本。
              m_cqiPredictor->RecordUlCqi (rnti, cqi, Simulator::Now ());
            }
        }
  }
}

void GeoBeamScheduler::UpdateUeDlCqi (uint16_t rnti, double cqi)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.latestDlCqi = cqi;
      it->second.latestCqi = cqi;   // 0603 兼容字段
      it->second.lastDlCqiUpdateTime = Simulator::Now ();
    }
  // 单一 DL CQI 真源: 一份 DL CQI 只喂一次到"当前生效的 DL controller"。
  //  - 若 m_cqiPredictor 是 CqiAmcPredictor: 经 RecordDlCqi 喂其内部 DL controller。
  //    该 DL controller 可能就是复用的 m_cqiAmc (单一真源), 此时再喂 m_cqiAmc 会
  //    对同一实例双喂 → 状态发散, 故不再单独喂 m_cqiAmc。
  //  - 否则 (直通预测器, 无适配器) 且有 m_cqiAmc: 直接喂 m_cqiAmc->OnCqiReport。
  //    (RecordDlCqi 在直通实现里是 no-op, 故 ntn-amc-closed-loop 仍走此分支)
  if (DynamicCast<CqiAmcPredictor> (m_cqiPredictor) != nullptr)
    {
      m_cqiPredictor->RecordDlCqi (rnti, cqi, Simulator::Now ());
    }
  else if (m_cqiAmc)
    {
      // OLLA+Kalman 更新步: 即使 UE 未注册上下文, 也喂 cqiAmc (Kalman 状态自维护)
      m_cqiAmc->OnCqiReport (rnti, cqi);
    }
}

uint8_t GeoBeamScheduler::GetMcsForNewTx (uint16_t rnti)
{
  if (m_cqiAmc)
    {
      // effectiveCqi = PredictCqi(H) + Δ; 统一走 NrAmc 双参版 (DL)
      return GetMcsFromCqi (m_cqiAmc->GetEffectiveCqi (rnti), false);
    }
  auto it = m_ueContextMap.find (rnti);
  double cqi = (it != m_ueContextMap.end ()) ? it->second.latestCqi : 7.0;
  return GetMcsFromCqi (cqi, false);
}

// ============================================================
// qyh 增量 (P2): UL CQI / Buffer Status
// ============================================================
void GeoBeamScheduler::UpdateUeUlCqi (uint16_t rnti, double cqi)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.latestUlCqi = cqi;
      it->second.lastUlCqiUpdateTime = Simulator::Now ();
    }
  if (m_cqiPredictor != nullptr)
    {
      m_cqiPredictor->RecordUlCqi (rnti, cqi, Simulator::Now ());
    }
}

void GeoBeamScheduler::UpdateUeDlBufferStatus (uint16_t rnti, uint32_t bufferBytes)
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      it->second.bufferStatus = bufferBytes;    // 0603 兼容字段
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

// ============================================================
// qyh 增量 (P2): PRACH 检测模型包装层
// ============================================================
void GeoBeamScheduler::EnablePrachDetectionErrors (bool enabled)
{
  if (!m_prachDetectionModel)
    m_prachDetectionModel = CreateObject<PrachDetectionModel> ();
  m_prachDetectionModel->SetEnabled (enabled);
}

bool GeoBeamScheduler::SetPrachDetectionCurveCsv (const std::string& path)
{
  if (!m_prachDetectionModel)
    m_prachDetectionModel = CreateObject<PrachDetectionModel> ();
  return m_prachDetectionModel->LoadCurveCsv (path);
}

void GeoBeamScheduler::SetPrachDetectionFixed (double pd, double pfa)
{
  if (!m_prachDetectionModel)
    m_prachDetectionModel = CreateObject<PrachDetectionModel> ();
  m_prachDetectionModel->SetFixedProbabilities (pd, pfa);
}

PrachDetectionStats GeoBeamScheduler::GetPrachDetectionStats () const
{
  if (m_prachDetectionModel) return m_prachDetectionModel->GetStats ();
  return PrachDetectionStats {};
}

void GeoBeamScheduler::ResetPrachDetectionStats ()
{
  if (m_prachDetectionModel) m_prachDetectionModel->ResetStats ();
}

void GeoBeamScheduler::PreProcessRequests () {}

// ============================================================
// qyh 重型 RRM 辅助实现
// ============================================================

//CQI 映射 MCS (qyh DL/UL 版, 走 NrAmc)
uint8_t GeoBeamScheduler::GetMcsFromCqi (double cqi, bool isUplink) const
{
  const uint8_t cqiIndex =
    static_cast<uint8_t> (std::clamp (std::lround (cqi), 0l, 15l));
  return isUplink ? m_ulAmc->GetMcsFromCqi (cqiIndex) : m_dlAmc->GetMcsFromCqi (cqiIndex);
}

double
GeoBeamScheduler::DerivePathLossFromRsrp (double rsrpDbm) const
{
  // 路损(dB) = 参考卫星EIRP(dBm) - 接收到的RSRP(dBm)
  const double pl = m_refSatEirpDbm - rsrpDbm;
  return std::clamp (pl, 150.0, 220.0);
}

double
GeoBeamScheduler::ExtractWidebandDlCqi (const DlCqiInfo& cqiInfo) const
{
  if (cqiInfo.m_wbCqi.empty ())
    {
      return 0.0;
    }

  double sum = 0.0;
  for (uint8_t cqi : cqiInfo.m_wbCqi)
    {
      sum += static_cast<double> (cqi);
    }

  return std::max (1.0, sum / static_cast<double> (cqiInfo.m_wbCqi.size ()));
}

double
GeoBeamScheduler::EstimateWidebandUlCqiFromSinr (const UlCqiInfo& ulCqiInfo) const
{
  if (ulCqiInfo.m_sinr.empty ())
    {
      return 0.0;
    }

  if (m_macSchedSapUser != nullptr && m_macSchedSapUser->GetSpectrumModel () != nullptr)
    {
      SpectrumValue sinrSpectrum (m_macSchedSapUser->GetSpectrumModel ());
      auto it = sinrSpectrum.ValuesBegin ();
      uint32_t idx = 0;
      while (it != sinrSpectrum.ValuesEnd ())
        {
          *it = (idx < ulCqiInfo.m_sinr.size ()) ? ulCqiInfo.m_sinr.at (idx) : 0.0;
          ++it;
          ++idx;
        }

      uint8_t mcs = 0;
      const uint8_t cqi = m_ulAmc->CreateCqiFeedbackWbTdma (sinrSpectrum, mcs);
      return static_cast<double> (std::max<uint8_t> (1u, cqi));
    }

  double sum = 0.0;
  uint32_t count = 0;
  for (double linearSinr : ulCqiInfo.m_sinr)
    {
      if (linearSinr > 0.0)
        {
          sum += linearSinr;
          ++count;
        }
    }

  if (count == 0)
    {
      return 0.0;
    }

  const double avgSinrLinear = sum / static_cast<double> (count);
  const double avgSinrDb = 10.0 * std::log10 (avgSinrLinear);
  if (avgSinrDb < -6.5) return 1.0;
  if (avgSinrDb < -5.0) return 2.0;
  if (avgSinrDb < -3.0) return 3.0;
  if (avgSinrDb < -1.0) return 4.0;
  if (avgSinrDb < 1.0) return 5.0;
  if (avgSinrDb < 3.0) return 6.0;
  if (avgSinrDb < 5.0) return 7.0;
  if (avgSinrDb < 7.0) return 8.0;
  if (avgSinrDb < 9.0) return 9.0;
  if (avgSinrDb < 11.0) return 10.0;
  if (avgSinrDb < 13.0) return 11.0;
  if (avgSinrDb < 15.0) return 12.0;
  if (avgSinrDb < 17.0) return 13.0;
  if (avgSinrDb < 19.0) return 14.0;
  return 15.0;
}

// =================================================================
// 上行闭环功率控制 (CLPC)
// =================================================================
double
GeoBeamScheduler::AverageUlSinrDb (const UlCqiInfo& ulCqiInfo) const
{
  double sum = 0.0;
  uint32_t count = 0;
  for (double linearSinr : ulCqiInfo.m_sinr)
    {
      if (linearSinr > 0.0)
        {
          sum += linearSinr;
          ++count;
        }
    }
  if (count == 0)
    {
      return NAN;
    }
  return 10.0 * std::log10 (sum / static_cast<double> (count));
}

int
GeoBeamScheduler::QuantizeTpcDb (double errorDb) const
{
  // 3GPP TPC 命令集 {-1, 0, +1, +3} dB。误差>0 表示收到的功率偏低, 需要加功率。
  if (errorDb > 4.0)  return 3;
  if (errorDb > 0.5)  return 1;
  if (errorDb < -1.0) return -1;
  return 0;
}

void
GeoBeamScheduler::ApplyClpcDelta (uint16_t rnti, double deltaDb)
{
  auto it = m_ueContextMap.find (rnti);
  if (it == m_ueContextMap.end ())
    {
      return;
    }
  it->second.clpcOffsetDb =
    std::clamp (it->second.clpcOffsetDb + deltaDb, m_clpcMinDb, m_clpcMaxDb);
  NS_LOG_INFO ("[CLPC] UE " << rnti << " | applied TPC=" << deltaDb
               << " dB | f(i)=" << it->second.clpcOffsetDb << " dB");
}

void
GeoBeamScheduler::UpdateClosedLoopPowerControl (uint16_t rnti, double measuredSinrDb)
{
  // 记录最近一次实测接收 UL SINR(dB), 供只读 getter 观测 CLPC 收敛 (收敛后 TPC=0
  // 提前返回, 故必须在返回前记录, 才能反映稳态值)。
  m_lastMeasuredUlSinrDb[rnti] = measuredSinrDb;
  const double errorDb = m_targetUlSinrDb - measuredSinrDb;
  const int tpc = QuantizeTpcDb (errorDb);
  if (tpc == 0)
    {
      return;
    }
  NS_LOG_INFO ("[CLPC] UE " << rnti << " | target=" << m_targetUlSinrDb
               << " dB | measured=" << measuredSinrDb << " dB | error=" << errorDb
               << " dB | TPC=" << tpc << " dB");
  if (m_clpcLoopDelay > Time (0))
    {
      Simulator::Schedule (m_clpcLoopDelay, &GeoBeamScheduler::ApplyClpcDelta, this,
                           rnti, static_cast<double> (tpc));
    }
  else
    {
      ApplyClpcDelta (rnti, static_cast<double> (tpc));
    }
}

// =================================================================
// 上行实测 SINR 反馈 (链路预算法) —— 闭合 CLPC 回路
//
//   链路预算:  SINR_rx(dB) = P_tx + G_ut_tx + G_sat_rx - PL - N0
//   其中:
//     P_tx      = AdjustUtTxPower(...) 输出的终端发射功率(dBm), 含闭环修正 f(i);
//     G_ut_tx   = 终端发射天线增益(dB);
//     G_sat_rx  = 卫星接收天线增益(dB);
//     PL        = 该 UE 的上行路径损耗估计(dB);
//     N0        = 热噪声功率(dBm) = -174 + 10log10(BW_Hz) + NF,
//                 BW_Hz = approvedRb * 180kHz (每 RB 12×15kHz)。
//
//   因为 P_tx 中包含 f(i)=clpcOffsetDb, clpcOffset 的任何变化都会等量地传到
//   SINR_rx, 因此该反馈与闭环修正项一一对应, 可让 CLPC 收敛到 m_targetUlSinrDb。
// =================================================================
double
GeoBeamScheduler::ComputeReceivedUlSinrDb (uint32_t approvedRb, double txPowerDbm,
                                           double pathLossDb) const
{
  // 单 RB 带宽 = 12 子载波 × 15 kHz = 180 kHz。
  const double rbBandwidthHz = 12.0 * 15.0e3;
  const double bwHz = std::max (1u, approvedRb) * rbBandwidthHz;
  // 热噪声功率 (dBm): kTB + NF, 其中 -174 dBm/Hz 为 290K 热噪声谱密度。
  const double noiseDbm = -174.0 + 10.0 * std::log10 (bwHz) + m_noiseFigureDb;
  const double sinrDb = txPowerDbm + m_utTxAntennaGainDb + m_satRxAntennaGainDb
                        - pathLossDb - noiseDbm;
  return sinrDb;
}

void
GeoBeamScheduler::ScheduleUlSinrFeedback (uint16_t rnti, uint32_t approvedRb, uint8_t symStart,
                                          const SfnSf& sfnSf, double txPowerDbm,
                                          double pathLossDb, double clpcOffsetDb)
{
  if (!m_clpcEnabled || approvedRb == 0)
    {
      return;
    }

  double measuredSinrDb = ComputeReceivedUlSinrDb (approvedRb, txPowerDbm, pathLossDb);
  // 可选: 叠加估计噪声 (信道估计误差)。
  if (m_ulSinrMeasErrorSigmaDb > 0.0 && m_ulSinrMeasErrorRv)
    {
      measuredSinrDb +=
        m_ulSinrMeasErrorRv->GetValue (0.0, m_ulSinrMeasErrorSigmaDb * m_ulSinrMeasErrorSigmaDb);
    }

  NS_LOG_INFO ("[UL SINR FB] UE " << rnti << " | approvedRB=" << approvedRb
               << " | txPower=" << txPowerDbm << " dBm | PL=" << pathLossDb
               << " dB | f(i)=" << clpcOffsetDb << " dB ===> measuredRxSINR="
               << measuredSinrDb << " dB (回灌延迟 "
               << m_ulCqiFeedbackDelay.GetMilliSeconds () << " ms)");

  // 延迟一个上行单程后, 把实测 SINR 经标准 UL CQI SAP 回灌给调度器, 形成闭环。
  Simulator::Schedule (m_ulCqiFeedbackDelay, &GeoBeamScheduler::DeliverUlSinrFeedback,
                       this, rnti, approvedRb, symStart, sfnSf, measuredSinrDb);
}

void
GeoBeamScheduler::DeliverUlSinrFeedback (uint16_t rnti, uint32_t approvedRb, uint8_t symStart,
                                         SfnSf sfnSf, double measuredSinrDb)
{
  // 项 5: CLPC 回灌直接透传 rnti, 不再经 (sfnSf,symStart) 反查 m_ulCqiAllocationMap。
  // 反查在 sfnSf 回绕或 ledger 被提前 erase 时会错配; 这里本就持有 rnti, 直接走统一的
  // 测量应用入口 ApplyUlSinrMeasurement(rnti, ...) → UpdateUeUlCqi + (CLPC)
  // UpdateClosedLoopPowerControl, 闭合回路。标准 SAP 入口 DoSchedUlCqiInfoReq(PHY 实测)
  // 仍可用, 不受影响。
  UlCqiInfo ulCqi;
  ulCqi.m_type = UlCqiInfo::PUSCH;
  const double linearSinr = std::pow (10.0, measuredSinrDb / 10.0);
  ulCqi.m_sinr.assign (std::max (1u, approvedRb), linearSinr);

  NS_LOG_INFO ("[UL SINR FB] UE " << rnti << " 反馈到达, 直接透传 rnti 应用测量"
               << " | slot=" << sfnSf << " | symStart=" << static_cast<uint32_t> (symStart)
               << " | measuredSINR=" << measuredSinrDb << " dB");

  ApplyUlSinrMeasurement (rnti, ulCqi);

  // 由于本路径不再经 DoSchedUlCqiInfoReq, 需主动清理本次授权登记的 ledger 项, 防止
  // 在 CLPC-only(无真实 PHY UL CQI SAP 回灌) 仿真中 m_ulCqiAllocationMap 无界增长。
  // 用 (sfnSf,symStart) 精确定位本条(仅做清理, 不依赖它反查 rnti); 若已被真实 PHY CQI
  // 消费则为 no-op。
  auto ledgerIt = m_ulCqiAllocationMap.find (sfnSf.GetEncoding ());
  if (ledgerIt != m_ulCqiAllocationMap.end ())
    {
      auto& entries = ledgerIt->second;
      auto entryIt = std::find_if (entries.begin (), entries.end (),
                                   [&] (const PendingUlCqiAllocation& a) {
                                     return a.symStart == symStart && a.rnti == rnti;
                                   });
      if (entryIt != entries.end ())
        {
          entries.erase (entryIt);
        }
      if (entries.empty ())
        {
          m_ulCqiAllocationMap.erase (ledgerIt);
        }
    }
}

// =================================================================
// 功率域 NOMA 共享 (下行)
// =================================================================
double
GeoBeamScheduler::SinrDbFromCqi (double cqi) const
{
  return 2.0 * (std::clamp (cqi, 1.0, 15.0) - 5.0);
}

double
GeoBeamScheduler::CqiFromSinrDb (double sinrDb) const
{
  if (sinrDb < -6.5) return 1.0;
  if (sinrDb < -5.0) return 2.0;
  if (sinrDb < -3.0) return 3.0;
  if (sinrDb < -1.0) return 4.0;
  if (sinrDb < 1.0) return 5.0;
  if (sinrDb < 3.0) return 6.0;
  if (sinrDb < 5.0) return 7.0;
  if (sinrDb < 7.0) return 8.0;
  if (sinrDb < 9.0) return 9.0;
  if (sinrDb < 11.0) return 10.0;
  if (sinrDb < 13.0) return 11.0;
  if (sinrDb < 15.0) return 12.0;
  if (sinrDb < 17.0) return 13.0;
  if (sinrDb < 19.0) return 14.0;
  return 15.0;
}

std::pair<double, double>
GeoBeamScheduler::ComputeNomaEffectiveSinrDb (double nearCqi, double farCqi) const
{
  // 1:1 经典语义: 该 far 独占全部 β 份额, near 承受 (1-β) 自身衰减。
  const double beta = m_nomaFarPowerFraction;
  return ComputeNomaEffectiveSinrDb (nearCqi, farCqi, beta, beta);
}

std::pair<double, double>
GeoBeamScheduler::ComputeNomaEffectiveSinrDb (double nearCqi, double farCqi,
                                             double betaFar, double betaFarTotal) const
{
  const double sNear = std::pow (10.0, SinrDbFromCqi (nearCqi) / 10.0);
  const double sFar = std::pow (10.0, SinrDbFromCqi (farCqi) / 10.0);

  // far(弱用户): 收到自身 betaFar 份功率; 把 near 的 (1-betaFarTotal) 份 + 其它 far 占的
  //   (betaFarTotal - betaFar) 份都当作干扰(SIC 前) ->
  //   SINR_far = (betaFar·S_far) / (1 + (1-betaFar)·S_far)
  // near(强用户): 逐层 SIC 抵消所有 far 信号, 只承受自身 (1-betaFarTotal) 份功率衰减 ->
  //   SINR_near = (1-betaFarTotal)·S_near
  const double sinrFarEffLin = (betaFar * sFar) / (1.0 + (1.0 - betaFar) * sFar);
  const double sinrNearEffLin = (1.0 - betaFarTotal) * sNear;

  const double nearSinrDb = 10.0 * std::log10 (std::max (sinrNearEffLin, 1e-9));
  const double farSinrDb = 10.0 * std::log10 (std::max (sinrFarEffLin, 1e-9));
  return {nearSinrDb, farSinrDb};
}

std::pair<double, double>
GeoBeamScheduler::ComputeNomaEffectiveCqi (double nearCqi, double farCqi) const
{
  const auto sinr = ComputeNomaEffectiveSinrDb (nearCqi, farCqi);
  const double nearEffCqi = CqiFromSinrDb (sinr.first);
  const double farEffCqi = CqiFromSinrDb (sinr.second);
  return {nearEffCqi, farEffCqi};
}

double
GeoBeamScheduler::NomaNearTxPowerDbm () const
{
  // 波束下行总功率预算(dBm)的 (1-β) 份额 -> dBm。
  const double budgetLinear = std::pow (10.0, m_resourceManager->GetBeamPowerBudgetDbm () / 10.0);
  const double share = budgetLinear * (1.0 - m_nomaFarPowerFraction);
  return 10.0 * std::log10 (std::max (share, 1e-9));
}

double
GeoBeamScheduler::NomaFarTxPowerDbm () const
{
  // 波束下行总功率预算(dBm)的 β 份额 -> dBm。
  return NomaTxPowerDbmForFraction (m_nomaFarPowerFraction);
}

double
GeoBeamScheduler::NomaTxPowerDbmForFraction (double fraction) const
{
  // 波束下行总功率预算(dBm)按给定线性份额切分 -> dBm。
  const double budgetLinear = std::pow (10.0, m_resourceManager->GetBeamPowerBudgetDbm () / 10.0);
  const double share = budgetLinear * std::clamp (fraction, 0.0, 1.0);
  return 10.0 * std::log10 (std::max (share, 1e-9));
}

void
GeoBeamScheduler::RecordEffectiveDlSinrDb (uint16_t rnti, double sinrDb)
{
  m_effectiveDlSinrDb[rnti] = sinrDb;
}

double
GeoBeamScheduler::GetEffectiveDlSinrDb (uint16_t rnti) const
{
  auto it = m_effectiveDlSinrDb.find (rnti);
  if (it == m_effectiveDlSinrDb.end ())
    {
      return std::numeric_limits<double>::quiet_NaN ();
    }
  return it->second;
}

double
GeoBeamScheduler::GetLastDlEffSinrDb (uint16_t rnti) const
{
  auto it = m_lastDlEffSinrDb.find (rnti);
  if (it == m_lastDlEffSinrDb.end ())
    {
      return std::numeric_limits<double>::quiet_NaN ();
    }
  return it->second;
}

double
GeoBeamScheduler::GetLastDlTxPowerDbm (uint16_t rnti) const
{
  auto it = m_lastDlTxPowerDbm.find (rnti);
  if (it == m_lastDlTxPowerDbm.end ())
    {
      return std::numeric_limits<double>::quiet_NaN ();
    }
  return it->second;
}

double
GeoBeamScheduler::GetLastMeasuredUlSinrDb (uint16_t rnti) const
{
  auto it = m_lastMeasuredUlSinrDb.find (rnti);
  if (it == m_lastMeasuredUlSinrDb.end ())
    {
      return std::numeric_limits<double>::quiet_NaN ();
    }
  return it->second;
}

uint32_t
GeoBeamScheduler::GetBeamSharedRbs (uint32_t beamId, bool isUplink) const
{
  return m_resourceManager->GetSharedRbs (beamId, isUplink);
}

uint32_t
GeoBeamScheduler::GetBeamUsedRbs (uint32_t beamId, bool isUplink) const
{
  return m_resourceManager->GetUsedRbs (beamId, isUplink);
}

uint64_t
GeoBeamScheduler::GetCumulativeNomaSharedDlRbs (uint32_t beamId) const
{
  auto it = m_cumNomaSharedDlRbs.find (beamId);
  return (it == m_cumNomaSharedDlRbs.end ()) ? 0 : it->second;
}

uint64_t
GeoBeamScheduler::GetCumulativeNomaUsedDlRbs (uint32_t beamId) const
{
  auto it = m_cumNomaUsedDlRbs.find (beamId);
  return (it == m_cumNomaUsedDlRbs.end ()) ? 0 : it->second;
}

double
GeoBeamScheduler::GetNomaDlReuseGain (uint32_t beamId) const
{
  const uint64_t used = GetCumulativeNomaUsedDlRbs (beamId);
  if (used == 0)
    {
      return 0.0;
    }
  return static_cast<double> (GetCumulativeNomaSharedDlRbs (beamId)) / static_cast<double> (used);
}

void
GeoBeamScheduler::WriteNomaReuseStats (Ptr<ResultWriter> writer, const std::string& filename) const
{
  if (writer == nullptr)
    {
      return;
    }
  // 遍历本调度器记账过的所有波束, 把 NOMA 复用记账落盘(真实读取 GetSharedRbs 结果)。
  for (const auto& kv : m_cumNomaUsedDlRbs)
    {
      const uint32_t beamId = kv.first;
      writer->WriteNomaReuseStats (filename, beamId,
                                   GetCumulativeNomaSharedDlRbs (beamId),
                                   GetCumulativeNomaUsedDlRbs (beamId),
                                   GetNomaDlReuseGain (beamId));
    }
}

void
GeoBeamScheduler::BuildNomaPairs (uint32_t beamId, const std::vector<uint16_t>& candidates)
{
  m_nomaPartner.clear ();
  m_nomaFarThisRound.clear ();
  if (!m_nomaEnabled)
    {
      return;
    }

  std::vector<std::pair<uint16_t, double>> pool;
  for (uint16_t rnti : candidates)
    {
      auto it = m_ueContextMap.find (rnti);
      if (it == m_ueContextMap.end () || it->second.dlBufferStatus == 0)
        {
          continue;
        }
      pool.push_back ({rnti, GetSchedulingCqi (it->second, false)});
    }
  if (pool.size () < 2)
    {
      return;
    }

  std::sort (pool.begin (), pool.end (),
             [] (const std::pair<uint16_t, double>& a, const std::pair<uint16_t, double>& b) {
               return a.second > b.second;
             });

  // 强用户从高 CQI 端取, 弱用户从低 CQI 端取。每个 near 最多叠加 m_nomaMaxFar 个 far
  // (按 CQI gap 从最弱往里递减多取); m_nomaMaxFar==1 退化为原 1:1 行为。near 与其所有
  // far 须满足最小 CQI gap (以该 near 的 CQI 减 far 的 CQI 判定)。
  const uint32_t maxFar = std::max (1u, m_nomaMaxFar);
  size_t i = 0;            // near 指针(高 CQI 端)
  size_t j = pool.size (); // far 指针(从末尾向前)
  while (i < j && j > 0)
    {
      const uint16_t nearRnti = pool[i].first;
      const double nearCqi = pool[i].second;
      uint32_t attached = 0;
      while (attached < maxFar && (i + 1) < j)
        {
          const size_t farIdx = j - 1;
          if (nearCqi - pool[farIdx].second < m_nomaMinCqiGap)
            {
              break; // 该 far 与此 near 的 gap 不足, 更里层 far gap 更小, 停止
            }
          m_nomaPartner[nearRnti].push_back (pool[farIdx].first);
          m_nomaFarThisRound.insert (pool[farIdx].first);
          NS_LOG_INFO ("[NOMA] Beam " << beamId << " pair near=" << nearRnti
                       << "(CQI" << nearCqi << ") <- far=" << pool[farIdx].first
                       << "(CQI" << pool[farIdx].second << ") [" << (attached + 1)
                       << "/" << maxFar << "]");
          --j;
          ++attached;
        }
      if (attached == 0)
        {
          break; // 当前最强 near 都配不上最弱 far, 后续更弱 near 更配不上
        }
      ++i;
    }
}

void
GeoBeamScheduler::EmitNomaFarGrant (uint16_t farRnti, uint32_t sharedRb, double farEffCqi,
                                    double farEffSinrDb, uint32_t beamId, double farTxPowerDbm)
{
  auto it = m_ueContextMap.find (farRnti);
  if (it == m_ueContextMap.end () || sharedRb == 0)
    {
      return;
    }
  SatUeContext& far = it->second;
  const uint32_t approvedShared = m_resourceManager->AllocateSharedSpectrum (beamId, sharedRb, false);
  if (approvedShared == 0)
    {
      return;
    }
  const uint8_t mcs = GetMcsFromCqi (farEffCqi, false);
  // 真实多-RB TBS, 不做单RB×N 线性外推。
  const uint32_t allocatedBytes = EstimateTbsBytes (farEffCqi, approvedShared, false);
  far.dlBufferStatus = (far.dlBufferStatus > allocatedBytes) ? (far.dlBufferStatus - allocatedBytes) : 0;
  far.dlAverageThroughput = 0.9 * far.dlAverageThroughput + 0.1 * allocatedBytes;
  far.lastDlScheduledTime = Simulator::Now ();

  // 记录 far 用户被 near 干扰后的有效 SINR, 供数据面打 tag 驱动 BLER。
  RecordEffectiveDlSinrDb (farRnti, farEffSinrDb);

  DciInfo dci;
  dci.isUplinkGrant = false;
  dci.rbAllocation = approvedShared;
  dci.mcs = mcs;
  // 项 A: far(弱用户)拿波束总功率的 β 份额 (1:N 时为 β 在各 far 间的等分, 由调用方传入)。
  dci.txPowerDbm = farTxPowerDbm;
  dci.delayToStart = m_defaultK2Delay;
  dci.duration = MilliSeconds (1);

  // 项 B: far 也走 m_harqManager->NewTransmission 并登记 m_dlHarqProcessMap, 使重传/OLLA
  // 记账与 near 对称(旧实现 far 无 HARQ, 可靠性记账不对称)。
  const uint32_t packetBytes = std::max (1u, allocatedBytes);
  if (m_harqManager != nullptr)
    {
      Ptr<Packet> harqPacket = Create<Packet> (packetBytes);
      const uint8_t harqProcessId = m_harqManager->NewTransmission (farRnti, harqPacket, mcs);
      if (harqProcessId == HarqManager::INVALID_PROCESS_ID)
        {
          NS_LOG_WARN ("[NOMA-DL] far UE " << farRnti
                       << " 未能分配 HARQ 进程，本次 far 授权退化为无 HARQ 发送");
        }
      else
        {
          dci.harqProcessId = harqProcessId;
          dci.harqRv = 0;
          dci.isHarqRetransmission = false;
          m_dlHarqProcessMap[{farRnti, harqProcessId}] =
            PendingDlHarqTransmission{dci, harqPacket, packetBytes};
          // 与 near 路径一致: 新传(rv==0)清零"重传过"标记, 防进程复用串味。
          m_dlHarqEverRetx.erase ({farRnti, harqProcessId});
        }
    }

  NS_LOG_INFO ("[NOMA-DL] Beam " << beamId << " | far UE=" << farRnti
               << " | SharedRB=" << approvedShared << " | farEffCQI=" << farEffCqi
               << " | farEffSINR=" << farEffSinrDb << " dB"
               << " | TxPower=" << dci.txPowerDbm << " dBm"
               << " | MCS=" << (uint32_t) mcs);
  // 数据面单一真源: 记录 far(弱用户)被干扰后的 {有效SINR, β份额发射功率}。
  m_lastDlEffSinrDb[farRnti] = farEffSinrDb;
  m_lastDlTxPowerDbm[farRnti] = dci.txPowerDbm;
  SendDciToUe (farRnti, dci);
}

uint8_t
GeoBeamScheduler::AllocateUlLedgerSymStart (void) const
{
  if (!m_hasCurrentUlTriggerSfnSf)
    {
      return 0;
    }

  auto it = m_ulCqiAllocationMap.find (m_currentUlTriggerSfnSf.GetEncoding ());
  if (it == m_ulCqiAllocationMap.end () || it->second.empty ())
    {
      return 0;
    }

  uint8_t nextSymStart = 0;
  for (const auto& allocation : it->second)
    {
      nextSymStart = std::max<uint8_t> (nextSymStart,
                                        static_cast<uint8_t> (allocation.symStart + 1));
    }
  return nextSymStart;
}

void
GeoBeamScheduler::RecordPendingUlCqiAllocation (uint16_t rnti,
                                                uint32_t approvedRb,
                                                uint8_t mcs,
                                                uint8_t symStart)
{
  if (!m_hasCurrentUlTriggerSfnSf)
    {
      return;
    }

  PendingUlCqiAllocation allocation;
  allocation.rnti = rnti;
  allocation.symStart = symStart;
  allocation.tbsBytes = std::max (1u, m_ulAmc->CalculateTbSize (mcs, approvedRb));
  m_ulCqiAllocationMap[m_currentUlTriggerSfnSf.GetEncoding ()].push_back (allocation);
}

// =================================================================
// 静态(SPS / 半持续)调度
// =================================================================
uint32_t
GeoBeamScheduler::GetSpsClassRbs (const SatUeContext& ctx) const
{
  // 不受 m_spsEnabled 门控的"按业务类固定 RB"。Legacy(经 GetSpsFixedRbs)与
  // Configured(IsSpsEligible / grant 尺寸 cap) 共用此口径。
  if (ctx.priority == ServicePriority::PRIORITY_EMERGENCY)
    {
      return m_spsEmergencyRbs;
    }
  if (ctx.priority == ServicePriority::PRIORITY_VOICE || ctx.trafficType == TRAFFIC_VOICE)
    {
      return m_spsVoiceRbs;
    }
  if (ctx.utType == UT_PORTABLE)
    {
      return m_spsPortableFloorRbs;
    }
  return 0;
}

uint32_t
GeoBeamScheduler::GetSpsFixedRbs (const SatUeContext& ctx) const
{
  // 历史语义不变: 仅 m_spsEnabled(Legacy 开关)为真时返回按类固定 RB。
  return m_spsEnabled ? GetSpsClassRbs (ctx) : 0;
}

bool
GeoBeamScheduler::TryScheduleDlSps (uint16_t rnti, uint32_t beamId, uint32_t& availableRbs)
{
  auto ctxIt = m_ueContextMap.find (rnti);
  if (ctxIt == m_ueContextMap.end ())
    {
      return false;
    }
  SatUeContext& ctx = ctxIt->second;
  const uint32_t spsRbs = GetSpsFixedRbs (ctx);
  if (spsRbs == 0 || ctx.dlBufferStatus == 0 || availableRbs == 0)
    {
      return false;
    }
  if (ctx.lastDlSpsTime >= Time (0) &&
      (Simulator::Now () - ctx.lastDlSpsTime) < m_spsPeriod)
    {
      return false;
    }

  const uint32_t requestRbs = std::min (spsRbs, availableRbs);
  const uint32_t allocatedRb = m_resourceManager->AllocateSpectrum (beamId, requestRbs, false);
  if (allocatedRb == 0)
    {
      return false;
    }
  availableRbs -= allocatedRb;

  const double dlCqi = GetSchedulingCqi (ctx, false);
  const uint8_t mcs = GetMcsFromCqi (dlCqi, false);
  // 真实多-RB TBS, 不做单RB×N 线性外推。
  const uint32_t allocatedBytes = EstimateTbsBytes (dlCqi, allocatedRb, false);
  ctx.dlBufferStatus = (ctx.dlBufferStatus > allocatedBytes) ? (ctx.dlBufferStatus - allocatedBytes) : 0;
  ctx.dlAverageThroughput = 0.9 * ctx.dlAverageThroughput + 0.1 * allocatedBytes;
  ctx.lastDlScheduledTime = Simulator::Now ();
  ctx.lastDlSpsTime = Simulator::Now ();

  DciInfo dci;
  dci.isUplinkGrant = false;
  dci.rbAllocation = allocatedRb;
  dci.mcs = mcs;
  dci.txPowerDbm = 0.0;
  dci.delayToStart = m_defaultK2Delay;
  dci.duration = MilliSeconds (1);
  NS_LOG_INFO ("[SPS-DL] Beam " << beamId << " | UE " << rnti
               << " | FixedRB=" << allocatedRb << " | MCS=" << (uint32_t) mcs);
  SendDciToUe (rnti, dci);
  return true;
}

bool
GeoBeamScheduler::TryScheduleUlSps (uint16_t rnti, uint32_t beamId)
{
  auto ctxIt = m_ueContextMap.find (rnti);
  if (ctxIt == m_ueContextMap.end ())
    {
      return false;
    }
  SatUeContext& ctx = ctxIt->second;
  const uint32_t spsRbs = GetSpsFixedRbs (ctx);
  if (spsRbs == 0)
    {
      return false;
    }
  const bool hasUlNeed = (GetEffectiveUlDemandBytes (ctx) > 0) || ctx.srPending;
  if (!hasUlNeed)
    {
      return false;
    }
  if (ctx.lastUlSpsTime >= Time (0) &&
      (Simulator::Now () - ctx.lastUlSpsTime) < m_spsPeriod)
    {
      return false;
    }

  const uint8_t mcs = GetMcsFromCqi (GetSchedulingCqi (ctx, true), true);
  const uint32_t approvedRb = ProcessUlGrant (rnti, spsRbs, mcs);
  if (approvedRb == 0)
    {
      return false;
    }
  ctx.lastUlSpsTime = Simulator::Now ();
  NS_LOG_INFO ("[SPS-UL] Beam " << beamId << " | UE " << rnti
               << " | FixedRB=" << approvedRb << " | MCS=" << (uint32_t) mcs);
  return true;
}

// =================================================================
// 阶段1: 半静态 configured-grant 状态机 (DL)
// =================================================================
uint8_t
GeoBeamScheduler::SpsLcidForUe (const SatUeContext& /*ctx*/) const
{
  // 阶段1: 调度器上下文是每-rnti 单缓存(一个 dlBufferStatus), 故先用固定 lcid=0。
  // key 类型保留 (rnti,lcid), 阶段3 区分 VoIP/数据多流时再按业务映射真实 lcid。
  return 0;
}

bool
GeoBeamScheduler::IsSpsEligible (const SatUeContext& ctx) const
{
  // 周期业务候选: 应急/语音/便携底(复用现有按类 RB 口径, 但不受 m_spsEnabled 门控)。
  return GetSpsClassRbs (ctx) > 0;
}

Time
GeoBeamScheduler::ComputeStaggeredNextTime (uint16_t rnti, Time now, Time period) const
{
  // 错峰: 给每个 UE 一个稳定的 [0, period) 相位偏移, 把各 grant 首次复用时刻打散到
  // 周期内不同相位 -> 平滑每-slot 峰值负载(并为后续 overbooking 的频域复用做相位基础)。
  // 周期长度不变, 只错开首个相位; 因周期恒定, 相对相位差此后永久保持。
  if (period <= Time (0))
    {
      return now;
    }
  const int64_t periodNs = period.GetNanoSeconds ();
  const uint64_t hash = static_cast<uint64_t> (rnti) * 2654435761ULL + 2166136261ULL;
  const int64_t phaseNs = static_cast<int64_t> (hash % static_cast<uint64_t> (periodNs));
  return now + period + NanoSeconds (phaseNs);
}

void
GeoBeamScheduler::ServeDlSpsGrant (uint16_t rnti, SatSpsGrant& grant)
{
  // 半静态"实发一次": 照搬 grant(RBG 数 + 当前 MCS), 扣 buffer, 记有效 SINR, 发 DCI。
  // 与动态 scheduleClassQueue 的区别: 不算度量、不重选 RB, 直接用持久 grant。
  SatUeContext& ctx = m_ueContextMap[rnti];
  const uint32_t rbs = static_cast<uint32_t> (grant.rbgBitmap.size ());
  const uint32_t allocatedBytes = std::min (ctx.dlBufferStatus, grant.tbSize);
  ctx.dlBufferStatus = (ctx.dlBufferStatus > allocatedBytes) ? (ctx.dlBufferStatus - allocatedBytes) : 0;
  ctx.dlAverageThroughput = 0.9 * ctx.dlAverageThroughput + 0.1 * allocatedBytes;
  ctx.lastDlScheduledTime = Simulator::Now ();
  ctx.lastDlSpsTime = Simulator::Now ();

  // 有效 SINR: 阶段1 SPS 为非 NOMA 普通发送, 等效自身 CQI(阶段3 接半静态 NOMA 再改)。
  const double effSinrDb = SinrDbFromCqi (GetSchedulingCqi (ctx, false));
  RecordEffectiveDlSinrDb (rnti, effSinrDb);
  m_lastDlEffSinrDb[rnti] = effSinrDb;

  DciInfo dci;
  dci.isUplinkGrant = false;
  dci.rbAllocation = rbs;
  dci.mcs = grant.mcs;
  dci.txPowerDbm = 0.0;
  dci.delayToStart = m_defaultK2Delay;
  dci.duration = MilliSeconds (1);
  m_lastDlTxPowerDbm[rnti] = dci.txPowerDbm;

  // GEO 周期小包默认不挂 HARQ(8 进程无法覆盖 20ms 周期 × ~600ms RTT); 开启时按新传登记。
  if (m_spsHarqEnabled && m_harqManager != nullptr &&
      m_harqManager->GetAvailableProcessCount (rnti) > 0)
    {
      const uint32_t packetBytes = std::max (1u, allocatedBytes);
      Ptr<Packet> harqPacket = Create<Packet> (packetBytes);
      const uint8_t pid = m_harqManager->NewTransmission (rnti, harqPacket, grant.mcs);
      if (pid != HarqManager::INVALID_PROCESS_ID)
        {
          dci.harqProcessId = pid;
          dci.harqRv = 0;
          dci.isHarqRetransmission = false;
          m_dlHarqProcessMap[{rnti, pid}] =
            PendingDlHarqTransmission{dci, harqPacket, packetBytes};
          m_dlHarqEverRetx.erase ({rnti, pid});
        }
    }

  NS_LOG_INFO ("[SPS-CFG-DL] Beam " << grant.beamId << " | UE " << rnti
               << " | RBG=" << rbs << " | MCS=" << (uint32_t) grant.mcs
               << " | Bytes=" << allocatedBytes);
  SendDciToUe (rnti, dci);
}

void
GeoBeamScheduler::ScheduleDlSpsReuse (uint32_t beamId, std::set<uint16_t>& spsServed)
{
  const Time now = Simulator::Now ();
  std::vector<std::pair<uint16_t, uint8_t>> releaseEmpty;
  std::vector<std::pair<uint16_t, uint8_t>> releaseConflict;

  for (auto& kv : m_dlSpsGrants)
    {
      const uint16_t rnti = kv.first.first;
      SatSpsGrant& grant = kv.second;
      if (!grant.active || grant.beamId != beamId)
        {
          continue;
        }
      auto ctxIt = m_ueContextMap.find (rnti);
      if (ctxIt == m_ueContextMap.end ())
        {
          releaseEmpty.push_back (kv.first); // UE 不在了 -> 释放
          continue;
        }
      if (now < grant.nextTime)
        {
          continue; // 未到期
        }
      grant.nextTime += grant.period; // 消费这个 due(无论成败, 不在同周期反复重试)

      SatUeContext& ctx = ctxIt->second;
      if (ctx.dlBufferStatus == 0)
        {
          if (++grant.emptyTxCounter >= m_spsImplicitReleaseAfter)
            {
              releaseEmpty.push_back (kv.first);
            }
          continue;
        }
      grant.emptyTxCounter = 0;

      // 频域半静态: 把同一组 RBG 索引按回原位; 冲突(越界/被占/超预算)则累计冲突计数。
      if (!m_resourceManager->ReserveSpsRbgs (beamId, grant.rbgBitmap, false))
        {
          if (++grant.conflictCounter >= m_spsImplicitReleaseAfter)
            {
              releaseConflict.push_back (kv.first);
            }
          continue;
        }
      grant.conflictCounter = 0;

      // 周期性刷新 MCS/TBS(用滤波 CQI), 防止 grant 锁定 MCS 随 ~600ms 环漂移而陈旧。
      const double cqi = GetSchedulingCqi (ctx, false);
      grant.mcs = GetMcsFromCqi (cqi, false);
      grant.tbSize = EstimateTbsBytes (cqi, static_cast<uint32_t> (grant.rbgBitmap.size ()), false);

      ServeDlSpsGrant (rnti, grant);
      spsServed.insert (rnti);
      m_spsStats.dlReuse++;
    }

  for (const auto& k : releaseEmpty)
    {
      ReleaseDlSpsGrant (k, false);
    }
  for (const auto& k : releaseConflict)
    {
      ReleaseDlSpsGrant (k, true);
    }
}

void
GeoBeamScheduler::ActivateDlSpsGrants (uint32_t beamId, const std::vector<uint16_t>& order,
                                       std::set<uint16_t>& spsServed)
{
  const Time now = Simulator::Now ();
  for (uint16_t rnti : order)
    {
      if (spsServed.count (rnti) > 0)
        {
          continue; // 本轮已被复用服务
        }
      auto ctxIt = m_ueContextMap.find (rnti);
      if (ctxIt == m_ueContextMap.end ())
        {
          continue;
        }
      SatUeContext& ctx = ctxIt->second;
      if (ctx.currentBeamId != beamId || !IsSpsEligible (ctx))
        {
          continue;
        }
      const std::pair<uint16_t, uint8_t> key{rnti, SpsLcidForUe (ctx)};
      auto git = m_dlSpsGrants.find (key);
      if (git != m_dlSpsGrants.end () && git->second.active)
        {
          continue; // 已有活跃 grant(尚未到期, 由 reuse 处理), 本轮不重复激活
        }
      if (ctx.dlBufferStatus < m_spsActivationMinBytes)
        {
          continue;
        }

      // grant 尺寸: 按类 cap 与 buffer 反推取小; RBG 来自 SPS 区。
      const double cqi = GetSchedulingCqi (ctx, false);
      const uint32_t cap = std::max (1u, GetSpsClassRbs (ctx));
      const uint32_t needRbs = std::max (1u, RbsForBytes (cqi, ctx.dlBufferStatus, cap, false));
      const uint32_t grantRbs = std::min (needRbs, cap);
      std::vector<uint32_t> rbgs = m_resourceManager->AllocateSpsRbgs (beamId, grantRbs, false);
      if (rbgs.empty ())
        {
          continue; // SPS 区已满 / 未启用(DlSpsRegionRbs=0)
        }

      SatSpsGrant grant;
      grant.active = true;
      grant.isUplink = false;
      grant.beamId = beamId;
      grant.rbgBitmap = rbgs;
      grant.mcs = GetMcsFromCqi (cqi, false);
      grant.tbSize = EstimateTbsBytes (cqi, static_cast<uint32_t> (rbgs.size ()), false);
      grant.period = m_spsGrantPeriod;
      grant.nextTime = m_spsStaggerEnabled ? ComputeStaggeredNextTime (rnti, now, grant.period)
                                           : (now + grant.period);

      ServeDlSpsGrant (rnti, grant); // 首传立即发
      m_dlSpsGrants[key] = grant;
      spsServed.insert (rnti);
      m_spsStats.dlActivations++;
    }
}

void
GeoBeamScheduler::ReleaseDlSpsGrant (const std::pair<uint16_t, uint8_t>& key, bool dueToConflict)
{
  auto it = m_dlSpsGrants.find (key);
  if (it == m_dlSpsGrants.end ())
    {
      return;
    }
  // 清掉本轮可能的 RBG 占用(若未占用则为安全 no-op), 释放回退计数由 RM 处理。
  m_resourceManager->FreeSpsRbgs (it->second.beamId, it->second.rbgBitmap, false);
  if (dueToConflict)
    {
      m_spsStats.dlReleaseConflict++;
    }
  else
    {
      m_spsStats.dlReleaseEmpty++;
    }
  NS_LOG_INFO ("[SPS-CFG-DL] Release grant rnti=" << key.first
               << " lcid=" << (uint32_t) key.second
               << " reason=" << (dueToConflict ? "conflict" : "empty/gone"));
  m_dlSpsGrants.erase (it);
}

//估算每个 RB 可承载字节数 (走 NrAmc, 单RB口径; 仅用于 IPF 速率度量)
double
GeoBeamScheduler::EstimateBytesPerRb (double cqi, bool isUplink) const
{
  const uint8_t mcs = GetMcsFromCqi (cqi, isUplink);
  const Ptr<NrAmc>& amc = isUplink ? m_ulAmc : m_dlAmc;
  return std::max (1.0, static_cast<double> (amc->GetPayloadSize (mcs, 1)));
}

//对目标 RB 数求真实多-RB TBS 字节(NR TBS 随 RB 非线性, 不做单RB×N 线性外推)
uint32_t
GeoBeamScheduler::EstimateTbsBytes (double cqi, uint32_t rbs, bool isUplink) const
{
  if (rbs == 0)
    {
      return 0;
    }
  const uint8_t mcs = GetMcsFromCqi (cqi, isUplink);
  const Ptr<NrAmc>& amc = isUplink ? m_ulAmc : m_dlAmc;
  return amc->GetPayloadSize (mcs, rbs);
}

//二分: 承载 targetBytes 所需最小 RB(真实 TBS 阶梯, clamp 到 maxRbs)
uint32_t
GeoBeamScheduler::RbsForBytes (double cqi, uint32_t targetBytes, uint32_t maxRbs, bool isUplink) const
{
  if (targetBytes == 0)
    {
      return 0;
    }
  if (maxRbs == 0)
    {
      return 0;
    }
  // 若满预算仍不够, 返回 maxRbs(尽力分配)。
  if (EstimateTbsBytes (cqi, maxRbs, isUplink) <= targetBytes)
    {
      return maxRbs;
    }
  uint32_t lo = 1;
  uint32_t hi = maxRbs;
  while (lo < hi)
    {
      const uint32_t mid = lo + (hi - lo) / 2;
      if (EstimateTbsBytes (cqi, mid, isUplink) >= targetBytes)
        {
          hi = mid;
        }
      else
        {
          lo = mid + 1;
        }
    }
  return lo;
}

//位置因子
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

//IPF 调度指标
double
GeoBeamScheduler::CalculateSchedulerMetric (const SatUeContext& ctx,
                                            uint32_t queueBudget,
                                            bool isUplink,
                                            double urgencyBoost,
                                            double demandBytes) const
{
  const double schedulingCqi = GetSchedulingCqi (ctx, isUplink);
  const double bytesPerRb = EstimateBytesPerRb (schedulingCqi, isUplink);
  const double instRate = bytesPerRb * std::max (1u, queueBudget);
  const double locationFactor = CalculateLocationFactor (ctx.position);
  const Time timeSinceLastSched =
    Simulator::Now () - (isUplink ? ctx.lastUlScheduledTime : ctx.lastDlScheduledTime);
  const double delaySensitivity = std::exp (timeSinceLastSched.GetSeconds () * 1.5);
  const double avgThroughput =
    isUplink ? ctx.ulAverageThroughput : ctx.dlAverageThroughput;
  const double demandWeight = 1.0 + std::log1p (std::max (0.0, demandBytes)) / 8.0;
  const double cqiBoost = 1.0 + std::max (1.0, schedulingCqi) / 10.0;
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
  // 用真实 TBS 阶梯反推所需 UL RB(上限取每波束 UL 预算)。
  ctx.pendingUlGrantRbs =
    std::max (1u, RbsForBytes (GetSchedulingCqi (ctx, true), ctx.pendingUlGrantBytes,
                               m_resourceManager->GetUlBeamBudgetRbs (), true));
}

double
GeoBeamScheduler::GetSchedulingCqi (const SatUeContext& ctx, bool isUplink) const
{
  const double primaryCqi = isUplink ? ctx.latestUlCqi : ctx.latestDlCqi;
  const double fallbackCqi = isUplink ? ctx.latestDlCqi : ctx.latestUlCqi;
  const Time primaryUpdateTime = isUplink ? ctx.lastUlCqiUpdateTime : ctx.lastDlCqiUpdateTime;
  const Time fallbackUpdateTime = isUplink ? ctx.lastDlCqiUpdateTime : ctx.lastUlCqiUpdateTime;
  const Time primaryValidity = isUplink ? m_ulCqiValidityTime : m_dlCqiValidityTime;
  const Time fallbackValidity = isUplink ? m_dlCqiValidityTime : m_ulCqiValidityTime;
  const Time now = Simulator::Now ();

  const bool primaryFresh = (primaryUpdateTime >= Time (0)) &&
                            ((now - primaryUpdateTime) <= primaryValidity);
  const bool fallbackFresh = (fallbackUpdateTime >= Time (0)) &&
                             ((now - fallbackUpdateTime) <= fallbackValidity);

  double measuredCqi = 1.0;
  if (primaryFresh && primaryCqi > 0.0)
    {
      measuredCqi = std::max (1.0, primaryCqi);
    }
  else if (fallbackFresh && fallbackCqi > 0.0)
    {
      measuredCqi = std::max (1.0, fallbackCqi);
    }

  if (m_cqiPredictor != nullptr)
    {
      const Time horizon = isUplink ? m_ulSchedHorizon : m_dlSchedHorizon;
      const double predicted = isUplink
        ? m_cqiPredictor->PredictUlCqi (ctx.rnti, measuredCqi, horizon)
        : m_cqiPredictor->PredictDlCqi (ctx.rnti, measuredCqi, horizon);
      return std::clamp (predicted, 1.0, 15.0);
    }
  return measuredCqi;
}

void GeoBeamScheduler::RunScheduler ()
{
  NS_LOG_FUNCTION (this);
  MaybeApplyCqiAmcPredictor ();
  ReservePrachResources ();

  // 每个调度轮次刷新各 UE 的下行有效 SINR(NOMA near/far 与普通用户), 避免跨轮残留。
  m_effectiveDlSinrDb.clear ();

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

      // 步骤 0 : 半静态/静态 预分配 (SpsMode 三选一; 默认 OFF, 与基线逐字节一致)
      std::set<uint16_t> spsServed;
      // 兼容: 历史代码可能只设了 SpsEnabled=true(未设 SpsMode) -> 当作 Legacy。
      const bool legacySps = (SpsModeEnum () == SpsMode::SPS_LEGACY) ||
                             (SpsModeEnum () == SpsMode::SPS_OFF && m_spsEnabled);
      if (SpsModeEnum () == SpsMode::SPS_CONFIGURED)
        {
          // 持久 grant 状态机: 先复用到期 grant(含隐式释放), 再给合格 UE 新建 grant。
          ScheduleDlSpsReuse (beamId, spsServed);
          ActivateDlSpsGrants (beamId, ueScheduleOrder, spsServed);
        }
      else if (legacySps)
        {
          for (uint16_t rnti : ueScheduleOrder)
            {
              if (availableRbs == 0)
                {
                  break;
                }
              if (TryScheduleDlSps (rnti, beamId, availableRbs))
                {
                  spsServed.insert (rnti);
                }
            }
        }
      // SPS(任一模式)可能改变本波束已用 RB; 统一从 RM 重算动态侧可用预算, 保持口径一致。
      {
        const uint32_t remAfterSps = m_resourceManager->GetRemainingRbs (beamId, false);
        availableRbs = (remAfterSps > m_prachReservedRbs) ? (remAfterSps - m_prachReservedRbs) : 0;
      }

      // 步骤 0.5 : NOMA 配对
      m_nomaPartner.clear ();
      m_nomaFarThisRound.clear ();
      if (m_nomaEnabled)
        {
          std::vector<uint16_t> nomaCand;
          for (uint16_t rnti : ueScheduleOrder)
            {
              if (spsServed.count (rnti) == 0)
                {
                  nomaCand.push_back (rnti);
                }
            }
          BuildNomaPairs (beamId, nomaCand);
        }

      // 步骤 1 : 根据 AdmitControl 准入结果, 构建逻辑信道队列
      std::vector<uint16_t> emergencyUes;
      std::vector<uint16_t> voiceUes;
      std::vector<uint16_t> normalUes;
      double maxEmergencyDelay = 0.0;
      double maxVoiceDelay = 0.0;

      for (uint16_t rnti : ueScheduleOrder) {
          if (spsServed.count (rnti) > 0)
            {
              continue;
            }
          if (m_nomaFarThisRound.count (rnti) > 0)
            {
              continue;
            }
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
          // 用真实 TBS 阶梯反推所需 RB(上限取每波束 DL 预算), 不做单RB×N 线性外推。
          const uint32_t rawRequiredRbs =
            std::max (1u, RbsForBytes (GetSchedulingCqi (ctx, false), bufferedBytes,
                                       m_resourceManager->GetDlBeamBudgetRbs (), false));
          // 准入门控语义: 这是一道"粗粒度准入闸"——只判断该 UE 本 TTI 能否拿到至少一片
          // 可用 RB, 而非按完整需求做硬性预留。实际分配在 scheduleClassQueue 里按完整
          // neededRbs(受 queueBudget 限) 机会式占用。为保持门控与分配"是否放行该 UE"
          // 这一决策一致, DATA/BEST_EFFORT 用 min(rawRequiredRbs, gate 上限) 作准入需求,
          // gate 上限取"实际分配每 TTI 可能给该 UE 的上限"(=本波束 DL 流量预算), 这样
          // 只要预算里还放得下分配真正会给出的 RB, 准入就放行, 不会因把整缓存当硬需求而
          // 误判"不够用"。应急/语音仍按各自刚性预留上限(突发3 / 语音2)门控。
          const uint32_t gateRequiredRbs =
            (ctx.priority == ServicePriority::PRIORITY_EMERGENCY) ? std::min (3u, rawRequiredRbs) :
            (ctx.priority == ServicePriority::PRIORITY_VOICE) ? std::min (2u, rawRequiredRbs) :
            std::min (rawRequiredRbs, m_dataAdmitRbCap);

          if (m_admitControl != nullptr)
            {
              m_admitControl->UpdateUeContext (rnti,
                                               GetSchedulingCqi (ctx, false),
                                               GetSchedulingCqi (ctx, true),
                                               gateRequiredRbs);
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

      // 步骤 2 : WRR 业务切片 + 动态提权
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

      double dlPowerSumWeights = 0.0;
      if (m_dlPowerAllocEnabled)
        {
          for (const std::vector<uint16_t>* q : {&emergencyUes, &voiceUes, &normalUes})
            {
              for (uint16_t rnti : *q)
                {
                  const SatUeContext& wctx = m_ueContextMap[rnti];
                  dlPowerSumWeights +=
                    m_resourceManager->GetPowerWeight (GetSchedulingCqi (wctx, false));
                }
            }
        }

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
              const double rawDlCqi = GetSchedulingCqi (ctx, false);
              double dlCqi = rawDlCqi;
              // 默认(普通 UE): 有效 SINR = 自身 CQI 等效 SINR。
              double nearEffSinrDb = SinrDbFromCqi (rawDlCqi);
              bool isNomaNear = false;
              // 本轮该 near 的所有 far 授权参数(1:N), 在 near DCI 发出后逐个 EmitNomaFarGrant。
              struct NomaFarGrant { uint16_t rnti; double effCqi; double effSinrDb; double txPowerDbm; };
              std::vector<NomaFarGrant> nomaFarGrants;
              if (m_nomaEnabled)
                {
                  auto pit = m_nomaPartner.find (rnti);
                  if (pit != m_nomaPartner.end () && !pit->second.empty ())
                    {
                      // 收集本轮仍有效(上下文存在)的 far。
                      std::vector<uint16_t> validFars;
                      for (uint16_t farRnti : pit->second)
                        {
                          if (m_ueContextMap.find (farRnti) != m_ueContextMap.end ())
                            {
                              validFars.push_back (farRnti);
                            }
                        }
                      if (!validFars.empty ())
                        {
                          isNomaNear = true;
                          // β 总份额在各 far 间等分(简单稳健; N==1 时 betaFar==betaFarTotal,
                          // 与原 1:1 完全一致)。
                          const double betaTotal = m_nomaFarPowerFraction;
                          const double betaFar = betaTotal / static_cast<double> (validFars.size ());
                          // near 逐层 SIC 后只承受 (1-betaTotal) 自身衰减 -> near 有效 SINR
                          // 取决于 betaTotal(与具体 far 无关), 用任一 far 的 near 分量即可。
                          double nearSinrSet = false;
                          for (uint16_t farRnti : validFars)
                            {
                              const double farRawCqi =
                                GetSchedulingCqi (m_ueContextMap[farRnti], false);
                              const auto effSinr =
                                ComputeNomaEffectiveSinrDb (rawDlCqi, farRawCqi, betaFar, betaTotal);
                              if (!nearSinrSet)
                                {
                                  nearEffSinrDb = effSinr.first;
                                  nearSinrSet = true;
                                }
                              const double farEffCqi = CqiFromSinrDb (effSinr.second);
                              nomaFarGrants.push_back (
                                {farRnti, farEffCqi, effSinr.second,
                                 NomaTxPowerDbmForFraction (betaFar)});
                            }
                          // near 自身有效 CQI 由 (1-betaTotal) 衰减后的 SINR 决定。
                          dlCqi = CqiFromSinrDb (nearEffSinrDb);
                        }
                    }
                }
              // 记录本轮 near/普通 UE 的下行有效 SINR, 供数据面打 tag 驱动真实 BLER。
              RecordEffectiveDlSinrDb (rnti, nearEffSinrDb);
              const uint8_t targetMcs = GetMcsFromCqi (dlCqi, false);
              if (m_harqManager != nullptr &&
                  m_harqManager->GetAvailableProcessCount (rnti) == 0)
                {
                  NS_LOG_INFO ("[HARQ] UE " << rnti
                               << " 当前无空闲 HARQ 进程，跳过本轮新的 DL 发送");
                  continue;
                }
              // 用真实 TBS 阶梯反推满足 buffer 的最小 RB(不做单RB×N 线性外推)。
              const uint32_t neededRbs =
                std::max (1u, RbsForBytes (dlCqi, ctx.dlBufferStatus, queueBudget, false));
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
              // 实发字节按真实多-RB TBS, 且不超过当前 buffer。
              const uint32_t allocatedBytes =
                std::min (ctx.dlBufferStatus, EstimateTbsBytes (dlCqi, allocatedRb, false));
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
              // 项 A: NOMA near(强用户)拿波束总功率的 (1-β) 份额 (真实功率域切分);
              // 否则按下行动态功率分配(开启时)或默认 0。
              dlDci.txPowerDbm = isNomaNear
                ? NomaNearTxPowerDbm ()
                : (m_dlPowerAllocEnabled
                     ? m_resourceManager->ComputeDlTxPowerDbm (
                         m_resourceManager->GetPowerWeight (dlCqi), dlPowerSumWeights)
                     : 0.0);
              dlDci.delayToStart = m_defaultK2Delay;
              dlDci.duration = MilliSeconds (1);
              const uint32_t packetBytes = std::max (1u, allocatedBytes);

              if (m_harqManager != nullptr)
                {
                  Ptr<Packet> harqPacket = Create<Packet> (packetBytes);
                  const uint8_t harqProcessId =
                    m_harqManager->NewTransmission (rnti, harqPacket, targetMcs);
                  if (harqProcessId == HarqManager::INVALID_PROCESS_ID)
                    {
                      NS_LOG_WARN ("[HARQ] UE " << rnti
                                   << " 未能分配 HARQ 进程，本次 DL 授权退化为无 HARQ 发送");
                    }
                  else
                    {
                      dlDci.harqProcessId = harqProcessId;
                      dlDci.harqRv = 0;
                      dlDci.isHarqRetransmission = false;
                      m_dlHarqProcessMap[{rnti, harqProcessId}] =
                        PendingDlHarqTransmission{dlDci, harqPacket, packetBytes};
                      // 进程在新传(rv==0)时清零"重传过"标记, 防止进程复用串味
                      // (旧实现只在终态 erase, 若复用前未及时清理会污染下一代 OLLA 判定)。
                      m_dlHarqEverRetx.erase ({rnti, harqProcessId});
                    }
                }

              NS_LOG_INFO ("[IPF Stage 2] Beam " << beamId
                           << " | Queue=" << queueLabel
                           << " | UE=" << rnti
                           << " | MetricP=" << item.second
                           << " | Need=" << neededRbs
                           << " | Proposed=" << schedulerProposedRbs
                           << " | Allocated=" << allocatedRb);
              // 数据面单一真源: 记录本次 near/普通 UE 授权的 {有效SINR, 发射功率}。
              // NtnGwMac 自动接线后, 据此打 NtnDciTag 并设 PSD 发射功率, 让 β 切分/
              // DL 动态功率真正进入物理发射 (不随调度轮清空, 与异步数据面对齐)。
              m_lastDlEffSinrDb[rnti] = nearEffSinrDb;
              m_lastDlTxPowerDbm[rnti] = dlDci.txPowerDbm;
              SendDciToUe (rnti, dlDci);

              if (m_nomaEnabled && !nomaFarGrants.empty ())
                {
                  // 所有 far 复用 near 的同一片 RB(NOMA 功率域叠加), 各自拿 betaFar 份额功率。
                  for (const auto& fg : nomaFarGrants)
                    {
                      EmitNomaFarGrant (fg.rnti, allocatedRb, fg.effCqi,
                                        fg.effSinrDb, beamId, fg.txPowerDbm);
                    }
                }
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

      // 步骤 3 : 第二级 IPF
      NS_LOG_INFO ("[IPF Stage 2] 正在对各业务队列执行同类用户优先级 P 排序...");
      scheduleClassQueue (normalUes, normalBudget, "data-best-effort", 1.0);

      // NOMA 复用累计记账: 在本轮 RB 记账被下一轮 ResetBeamAllocation 清零前快照,
      // 把本轮该波束的共享(复用)RB / 已用RB 累加到跨轮次累计量(消除 GetSharedRbs 死代码)。
      m_cumNomaSharedDlRbs[beamId] += m_resourceManager->GetSharedRbs (beamId, false);
      m_cumNomaUsedDlRbs[beamId] += m_resourceManager->GetUsedRbs (beamId, false);

      NS_LOG_INFO ("=== Beam " << beamId << " Scheduling Complete | DL Remaining="
                   << m_resourceManager->GetRemainingRbs (beamId, false) << " RBs"
                   << " | NOMA SharedDL(round)=" << m_resourceManager->GetSharedRbs (beamId, false)
                   << " | NOMA SharedDL(cum)=" << m_cumNomaSharedDlRbs[beamId] << " ===");

      for (const auto& redirectAction : redirectActions)
        {
          ExecuteHandover (redirectAction.first, redirectAction.second);
        }
    }
}

void GeoBeamScheduler::SendControlMsg (uint16_t rnti, uint8_t msgType) { }

// 注: ReceiveMeasReport 已迁移至 NtnGwRrc::ReceiveMeasReport (ntn-gw-rrc.cc)

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

void
GeoBeamScheduler::SetHarqManager (Ptr<HarqManager> harqManager)
{
  m_harqManager = harqManager;
  if (m_harqManager == nullptr)
    {
      return;
    }

  m_harqManager->m_retransmitCallback.ConnectWithoutContext (
    MakeCallback (&GeoBeamScheduler::HandleDlHarqRetransmission, this));
  m_harqManager->m_feedbackCallback.ConnectWithoutContext (
    MakeCallback (&GeoBeamScheduler::HandleDlHarqFeedbackResult, this));
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
                     << pucchInfo.harqAck << " Bitmap=" << (uint32_t)pucchInfo.harqBitMap
                     << " Process=" << (uint32_t)pucchInfo.harqProcessId);
        if (m_harqManager != nullptr &&
            pucchInfo.harqProcessId != PucchInfo::INVALID_HARQ_PROCESS_ID)
          {
            SatHarqFeedback feedback;
            feedback.rnti = pucchInfo.rnti;
            feedback.processId = pucchInfo.harqProcessId;
            feedback.ack = pucchInfo.harqAck;
            feedback.rvIndex =
              m_harqManager->GetNextRedundancyVersion (pucchInfo.rnti, pucchInfo.harqProcessId);
            m_harqManager->ReceiveFeedback (feedback);
          }
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

void
GeoBeamScheduler::HandleDlHarqRetransmission (uint16_t rnti, uint8_t processId, Ptr<Packet> packet)
{
  auto it = m_dlHarqProcessMap.find (std::make_pair (rnti, processId));
  if (it == m_dlHarqProcessMap.end ())
    {
      NS_LOG_WARN ("[HARQ] Missing cached DL transmission state for UE " << rnti
                   << " process " << static_cast<uint32_t> (processId));
      return;
    }

  if (m_harqManager == nullptr)
    {
      return;
    }

  it->second.packet = packet != nullptr ? packet->Copy () : it->second.packet;
  it->second.dci.harqProcessId = processId;
  it->second.dci.harqRv = m_harqManager->GetCurrentRedundancyVersion (rnti, processId);
  it->second.dci.isHarqRetransmission = true;

  // 发生过重传 ⇒ 首传是 NACK; 供终态时推导 OLLA 初传结果。
  m_dlHarqEverRetx[std::make_pair (rnti, processId)] = true;

  NS_LOG_INFO ("[HARQ] Re-sending DL DCI to UE " << rnti
               << " | Process=" << static_cast<uint32_t> (processId)
               << " | RV=" << static_cast<uint32_t> (it->second.dci.harqRv));
  SendDciToUe (rnti, it->second.dci);
}

void
GeoBeamScheduler::HandleDlHarqFeedbackResult (uint16_t rnti,
                                              uint8_t processId,
                                              bool completedSuccessfully)
{
  if (m_harqManager == nullptr)
    {
      return;
    }

  const HarqState state = m_harqManager->GetProcessState (rnti, processId);
  if (state == HarqState::SUCCESS || state == HarqState::FAILED)
    {
      // 终态推导首传(rv==0)结果, 喂 OLLA (isNewTx=true)。判据用 HARQ 进程的真实
      // retransmitCount(终态仍保留, 不会因 FinalizeProcess 复位): 只有"从未重传且
      // 最终成功"(retransmitCount==0 && SUCCESS) 才是首传 ACK; 一旦重传过(说明首传是
      // NACK)或最终失败, 均视为首传 NACK。m_dlHarqEverRetx 仅作日志/兜底对照。
      const std::pair<uint16_t, uint8_t> key = std::make_pair (rnti, processId);
      const uint8_t retransmitCount = m_harqManager->GetRetransmitCount (rnti, processId);
      const bool initialAck = completedSuccessfully && (retransmitCount == 0);
      // 单一 DL OLLA 真源, 与 UpdateUeDlCqi 同源: 适配器(可能复用 m_cqiAmc)优先,
      // 否则直通预测器(no-op)时退回直接喂 m_cqiAmc, 避免双喂同一 OLLA。
      if (DynamicCast<CqiAmcPredictor> (m_cqiPredictor) != nullptr)
        {
          m_cqiPredictor->RecordHarqFeedback (rnti, initialAck, /*isNewTx=*/true);
        }
      else if (m_cqiAmc)
        {
          m_cqiAmc->OnHarqFeedback (rnti, initialAck, /*isNewTx=*/true);
        }
      m_dlHarqEverRetx.erase (key);

      m_dlHarqProcessMap.erase (key);
      NS_LOG_INFO ("[HARQ] DL process finished | UE=" << rnti
                   << " Process=" << static_cast<uint32_t> (processId)
                   << " Result=" << (completedSuccessfully ? "success" : "failed")
                   << " retxCount=" << static_cast<uint32_t> (retransmitCount)
                   << " initialAck=" << initialAck);
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
  // per-UE PRACH SNR: 由 UE 侧位置/仰角解析算出并随前导码携带 (SatUtMac::SetPrachSnrDb)。
  // 若 UE 未注入 (NaN) 则回退到全局默认 m_defaultPrachSnrDb。
  arr.prachSnrDb  = std::isfinite (preamble.prachSnrDb) ?
                    preamble.prachSnrDb :
                    m_defaultPrachSnrDb;
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

  // 按 (RA-RNTI, preambleId) 分组; 同时采集每组的 utType(取最受限)、组检测 SNR(取最大)、
  // 以及每个 RA occasion 已占用的前导码集合(虚警反推空闲前导码用)。
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> preambleCnt;
  std::map<std::pair<uint32_t, uint32_t>, UtType> preambleUtType;
  std::map<std::pair<uint32_t, uint32_t>, double> preambleSnrDb;  // 组的检测 SNR (取组内最大)
  std::map<uint32_t, std::set<uint32_t>> activePreamblesByRaRnti; // 每 RO 已占用的前导码
  for (const auto& a : m_prachWindowBuf)
    {
      const std::pair<uint32_t, uint32_t> key = {a.raRnti, a.preambleId};
      preambleCnt[key]++;
      activePreamblesByRaRnti[a.raRnti].insert (a.preambleId);
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

      // 组的检测 SNR = 组内所有 UE 的 SNR 最大值: 基站只解出一条叠加的前导码,
      // 用最强者代表该组的可检测性 (能检出最强的那个即算检到)。
      auto snrIt = preambleSnrDb.find (key);
      if (snrIt == preambleSnrDb.end () || a.prachSnrDb > snrIt->second)
        {
          preambleSnrDb[key] = a.prachSnrDb;
        }
    }

  NS_LOG_INFO ("[PRACH] 窗口处理: 共 " << m_prachWindowBuf.size ()
               << " 个前导码, 唯一 (raRnti,PreambleID) 数 = " << preambleCnt.size ());

  // 检测误差 (漏检/虚警) 是否启用: 注入了 PRACH 检测模型且 enabled。
  // 关 (false) 时退化为理想检测: 每条有 UE 的前导码都检到、无虚警。
  const bool detectionErrorsEnabled =
    (m_prachDetectionModel != nullptr && m_prachDetectionModel->IsEnabled ());

  // 重要: 即使多个 UE 选用了同一个 preambleId, 基站在 Msg1 阶段
  // 解出的还是同一条前导码 (能量叠加 / 去重), 依然会发一条 RAR。
  // 碰撞会推迟到 Msg3 阶段才被发现——这与实际 4 步 RA 的行为一致。
  for (const auto& kv : preambleCnt)
    {
      uint32_t raRnti = kv.first.first;
      uint32_t pid    = kv.first.second;
      uint32_t count  = kv.second;

      const std::pair<uint32_t, uint32_t> key = kv.first;

      // ---- 漏检判定 (对"有 UE 的前导码组", 仅当检测误差启用) ----
      // 用组 SNR (组内最大 per-UE SNR) 查 Pd 曲线做伯努利抽签。抽中漏检: 记一次漏检
      // (连累组内 count 个 UE)、continue 跳过本组 → 不发 RAR、不占用 Msg3 资源, 这些
      // UE 等不到响应而竞争超时、重传。放在资源分配之前: 漏检的前导码不应消耗 RB。
      if (detectionErrorsEnabled)
        {
          const double snrDb = (preambleSnrDb.find (key) != preambleSnrDb.end ()) ?
                               preambleSnrDb.at (key) :
                               m_defaultPrachSnrDb;
          m_prachDetectionModel->RecordActiveGroup (count, snrDb);   // 分母: 一次检测试验
          if (!m_prachDetectionModel->DetectActivePreamble (snrDb))  // Bernoulli(Pd(snr))
            {
              m_prachDetectionModel->RecordMissedGroup (count);      // 漏检 → 该组 count 个 UE 受连累
              NS_LOG_WARN ("[Msg1] 漏检: RA-RNTI=" << raRnti
                           << " PreambleID=" << pid
                           << " SNR=" << snrDb << " dB UEAttempts=" << count
                           << " -> 不发送 RAR");
              continue;                                              // 不发 RAR, 不占资源
            }
          m_prachDetectionModel->RecordDetectedGroup (snrDb);        // 检到
        }

      // ---- Msg3 资源门控 (功率受限 RB + AllocateSpectrum, 与主代码一致) ----
      const UtType msg3UtType =
        (preambleUtType.find (key) != preambleUtType.end ()) ?
        preambleUtType.at (key) :
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
      rar.raRnti            = raRnti;             // 保留 UE 的 RA-RNTI (区分 PRACH occasion)
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

  // ---- 虚警判定 (对"空闲前导码", 即本窗口根本没人发的前导码; 仅当检测误差启用) ----
  // 虚警 = 噪声越过检测门限被误判为"有前导码", 只可能发生在空闲前导码上。遍历每个
  // RA occasion 的 1..m_numPrachPreambles, 跳过已占用的; 对空闲前导码: 先记一次试验
  // (Pfa 分母), 用默认 SNR 查 Pfa 曲线抽签; 抽中虚警即计 (Pfa 分子, 与是否拿到 RB 无关),
  // 再尝试走资源门控发幽灵 RAR (拿到 RB 才发并累加浪费的 RB; 预算耗尽则只计虚警)。
  if (detectionErrorsEnabled)
    {
      const UtType faUtType = static_cast<UtType> (m_msg3DefaultUtTypeValue);
      for (const auto& raOccasion : activePreamblesByRaRnti)
        {
          const uint32_t raRnti = raOccasion.first;
          const std::set<uint32_t>& activePreambles = raOccasion.second;
          for (uint32_t pid = 1; pid <= m_numPrachPreambles; ++pid)
            {
              if (activePreambles.find (pid) != activePreambles.end ())
                {
                  continue;                                          // 有人发 → 不是虚警对象
                }
              m_prachDetectionModel->RecordFalseAlarmTrial ();       // Pfa 分母 +1
              if (!m_prachDetectionModel->DetectFalseAlarm (m_defaultPrachSnrDb))
                {
                  continue;                                          // 未触发虚警
                }
              const uint32_t faPowerLimitedRbs =
                m_resourceManager->GetMaxPowerLimitedUlRbs (faUtType, m_referencePathLossDb);
              const uint32_t faRequested = std::min (m_msg3RequestedRbs, faPowerLimitedRbs);
              const uint32_t faApprovedRbs =
                (faRequested > 0) ?
                m_resourceManager->AllocateSpectrum (m_myBeamId, faRequested, true) : 0;
              m_prachDetectionModel->RecordFalseAlarmGroup (faApprovedRbs);  // 计虚警 + 浪费的 RB
              if (faApprovedRbs == 0)
                {
                  NS_LOG_WARN ("[Msg1] 虚警: RA-RNTI=" << raRnti << " PreambleID=" << pid
                               << " 触发但上行预算耗尽, 未发幽灵 RAR");
                  continue;
                }
              RarMessage rar;
              rar.raRnti            = raRnti;
              rar.preambleId        = pid;
              rar.tcRnti            = AllocateTcRnti ();
              rar.timingAdvance     = MilliSeconds (300);
              rar.ulGrantRbs        = faApprovedRbs;
              rar.ulGrantMcs        = m_msg3GrantMcs;
              rar.ulGrantTxPowerDbm =
                m_resourceManager->AdjustUtTxPower (faUtType, faApprovedRbs, m_referencePathLossDb);
              rar.msg3DelayToStart  = MicroSeconds (500);
              rar.transmissionTime  = Simulator::Now () + m_rarProcessingDelay;
              NS_LOG_WARN ("[Msg1] 虚警: RA-RNTI=" << raRnti << " PreambleID=" << pid
                           << " 凭空发 RAR, 浪费 Msg3 RB=" << faApprovedRbs);
              Simulator::Schedule (m_rarProcessingDelay + m_rarTxDelay,
                                   &GeoBeamScheduler::DispatchRar, this, rar);
            }
        }
    }

  m_prachWindowBuf.clear ();
}

// 每波束 UL 调度周期: roundId=0 时, 该 beam 首次调用建新轮次并 ResetBeamAllocation 重置
// 上行 RB 预算; 同一轮次内复用 (不再重置)。RA-only 场景下相当于每波束预算重置一次后
// 随 Msg3 授权单调消耗——这正是主代码资源门控下 RA 成功率受约束的来源。
void GeoBeamScheduler::BeginUlSchedulingPeriod (uint32_t beamId, uint64_t schedulingRoundId)
{
  auto it = m_ulSchedulingRoundIdByBeam.find (beamId);
  if (schedulingRoundId == 0)
    {
      if (it != m_ulSchedulingRoundIdByBeam.end ())
        {
          return;   // 已有轮次 → 复用, 不重置预算
        }
      schedulingRoundId = ++m_nextUlSchedulingRoundId;
    }
  if (it != m_ulSchedulingRoundIdByBeam.end () && it->second == schedulingRoundId)
    {
      return;
    }
  m_resourceManager->ResetBeamAllocation (beamId, true);   // 重置该 beam 的上行 RB 使用量
  m_ulSchedulingRoundIdByBeam[beamId] = schedulingRoundId;
  NS_LOG_INFO ("[UL TTI] Beam " << beamId << " 开启上行调度周期 RoundId=" << schedulingRoundId
               << " | UL 预算=" << m_resourceManager->GetRemainingRbs (beamId, true) << " RB");
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
    m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, ctx.pathLossDb);
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
  dci.symStart = AllocateUlLedgerSymStart ();
  const double estimatedTxPowerDbm =
    m_resourceManager->AdjustUtTxPower (ctx.utType, approvedRb, ctx.pathLossDb,
                                        m_clpcEnabled ? ctx.clpcOffsetDb : 0.0);
  dci.txPowerDbm = estimatedTxPowerDbm;
  dci.delayToStart = MicroSeconds (32);  // 典型的K2延迟
  dci.duration = MilliSeconds (1);
  ctx.pendingUlGrantRbs += approvedRb;
  // 授权字节按真实多-RB TBS(已批 RB 数), 不做单RB×N 线性外推。
  const uint32_t estimatedGrantBytes =
    std::max (1u, EstimateTbsBytes (GetSchedulingCqi (ctx, true), approvedRb, true));
  ctx.pendingUlGrantBytes += estimatedGrantBytes;
  ctx.pendingUlGrantBytes = std::min (ctx.pendingUlGrantBytes, ctx.ulBufferStatus);
  RefreshPendingUlGrantEstimate (ctx);
  RecordPendingUlCqiAllocation (rnti, approvedRb, mcs, dci.symStart);
  // 闭环功控反馈: 仅当本次授权确实登记进 UL ledger(有有效 SfnSf)时, 才安排一次
  // 基于链路预算的实测接收 SINR 反馈, 保证回灌的 (sfnSf, symStart) 能反查到 rnti。
  if (m_clpcEnabled && m_hasCurrentUlTriggerSfnSf)
    {
      ScheduleUlSinrFeedback (rnti, approvedRb, dci.symStart, m_currentUlTriggerSfnSf,
                              estimatedTxPowerDbm, ctx.pathLossDb, ctx.clpcOffsetDb);
    }
  ctx.srPending = false;
  ctx.lastUlScheduledTime = Simulator::Now ();

  NS_LOG_INFO ("[UL Grant] UE " << rnti << " 上行授权! Beam=" << beamId
               << " | RequestedRB=" << rbAllocation
               << " | PowerLimitedRB=" << requestedAfterPowerLimit
               << " | RemainingBefore=" << remainingBeforeGrant
               << " | ApprovedRB=" << approvedRb
               << " | SymStart=" << static_cast<uint32_t> (dci.symStart)
               << " | EffectiveGrantBytes=" << estimatedGrantBytes
               << " | MCS=" << (uint32_t)mcs
               << " | EstTxPower=" << estimatedTxPowerDbm << " dBm"
               << " | UL RemainingAfter=" << m_resourceManager->GetRemainingRbs (beamId, true)
               << " | 当前UL Buffer=" << ctx.ulBufferStatus << " bytes"
               << " | PendingGrantBytes=" << ctx.pendingUlGrantBytes);

  SendDciToUe (rnti, dci);
  return approvedRb;
}

void GeoBeamScheduler::RunUlScheduler ()
{
  for (const auto& beamPair : m_beamToUesMap)
    {
      RunUlSchedulerForBeam (beamPair.first);
    }
}

//上行 WRR + IPF 调度
void GeoBeamScheduler::RunUlSchedulerForBeam (uint32_t beamId)
{
  auto beamIt = m_beamToUesMap.find (beamId);
  if (beamIt == m_beamToUesMap.end ())
    {
      return;
    }

  // 步骤 0 : 上行静态(SPS)预分配
  std::set<uint16_t> spsServed;
  if (m_spsEnabled)
    {
      for (uint16_t rnti : beamIt->second)
        {
          if (m_resourceManager->GetRemainingRbs (beamId, true) == 0)
            {
              break;
            }
          if (TryScheduleUlSps (rnti, beamId))
            {
              spsServed.insert (rnti);
            }
        }
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
      if (spsServed.count (rnti) > 0)
        {
          continue;
        }
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

      // 用真实 TBS 阶梯反推所需 UL RB(上限取每波束 UL 预算)。
      const uint32_t requestedRbsRaw = hasPendingUlData ?
        std::max (1u, RbsForBytes (GetSchedulingCqi (ctx, true), effectivePendingBytes,
                                   m_resourceManager->GetUlBeamBudgetRbs (), true)) :
        m_srGrantRbs;
      const uint32_t powerLimitedMaxRbs =
        m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, ctx.pathLossDb);
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
          m_admitControl->UpdateUeContext (rnti,
                                           GetSchedulingCqi (ctx, false),
                                           GetSchedulingCqi (ctx, true),
                                           gateRequiredRbs);
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

          const double ulSchedCqi = GetSchedulingCqi (ctx, true);
          const uint32_t powerLimitedMaxRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, ctx.pathLossDb);
          if (powerLimitedMaxRbs == 0)
            {
              continue;
            }
          // demand 上限用真实多-RB TBS(功率受限 RB 数), 不做单RB×N 线性外推。
          const double demandBytes =
            static_cast<double> (hasPendingUlData ?
                                 std::min<uint32_t> (effectivePendingBytes,
                                                     EstimateTbsBytes (ulSchedCqi, powerLimitedMaxRbs, true)) :
                                 EstimateTbsBytes (ulSchedCqi, std::min (m_srGrantRbs, powerLimitedMaxRbs), true));
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

          // 用真实 TBS 阶梯反推所需 UL RB(上限取每波束 UL 预算)。
          const uint32_t neededRbsRaw = hasPendingUlData ?
            std::max (1u, RbsForBytes (GetSchedulingCqi (ctx, true), effectivePendingBytes,
                                       m_resourceManager->GetUlBeamBudgetRbs (), true)) :
            m_srGrantRbs;
          const uint32_t powerLimitedMaxRbs =
            m_resourceManager->GetMaxPowerLimitedUlRbs (ctx.utType, ctx.pathLossDb);
          const uint32_t neededRbs = std::min (neededRbsRaw, powerLimitedMaxRbs);
          const uint32_t requestedRbs =
            std::min ({neededRbs, queueBudget, m_resourceManager->GetRemainingRbs (beamId, true)});
          if (requestedRbs == 0)
            {
              continue;
            }

          const uint32_t approvedRbs =
            ProcessUlGrant (item.first,
                            requestedRbs,
                            GetMcsFromCqi (GetSchedulingCqi (ctx, true), true));
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

// 注: SetGroundAvailability / IsGroundAvailable / GetServingObjectForUe /
//     BuildSatBeamTarget / BuildGroundTarget / ClassifyHandoverExecution
// 已迁移至 NtnGwRrc (ntn-gw-rrc.cc)。SetAdmitControl / CheckHandoverAdmission
// 由调度器保留 (供 NtnGwRrc / 上层注入与查询)。

//设置 AdmitControl, 并绑定 ResourceManager / 切换执行回调 / 下行预留 RB。
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

//切换准入检查 (供上层调用)
bool GeoBeamScheduler::CheckHandoverAdmission (uint32_t sourceBeamId,
                                               uint32_t targetBeamId,
                                               ServicePriority priority,
                                               UtType utType,
                                               TrafficType trafficType,
                                               uint32_t requiredRbs,
                                               bool isUplink)
{
  NS_LOG_FUNCTION (this << sourceBeamId << targetBeamId << (uint32_t)priority << requiredRbs);

  if (m_admitControl == nullptr) {
      NS_LOG_WARN ("AdmitControl not set, allowing handover by default");
      return true;
  }

  const AdmitDecision decision =
    (sourceBeamId == targetBeamId) ?
      m_admitControl->CanAdmitUe (targetBeamId,
                                  priority,
                                  utType,
                                  trafficType,
                                  requiredRbs,
                                  isUplink) :
      m_admitControl->CanHandoverUe (sourceBeamId,
                                     targetBeamId,
                                     priority,
                                     utType,
                                     trafficType,
                                     requiredRbs,
                                     isUplink);

  bool admitted = (decision == AdmitDecision::ADMIT_OK ||
                   decision == AdmitDecision::ADMIT_REDIRECT);

  NS_LOG_INFO ("Handover admission check: SourceBeam " << sourceBeamId
               << " -> TargetBeam " << targetBeamId
               << " | Decision: " << (uint32_t)decision
               << " | Admitted: " << admitted);

  return admitted;
}

ServicePriority
GeoBeamScheduler::GetUePriority (uint16_t rnti) const
{
  auto it = m_ueContextMap.find (rnti);
  if (it != m_ueContextMap.end ())
    {
      return it->second.priority;
    }
  return ServicePriority::PRIORITY_DATA;
}

// 功能验证演示访问器: 只读暴露调度用 CQI(经 m_cqiPredictor 处理路径)。
double
GeoBeamScheduler::GetSchedulingCqiForUe (uint16_t rnti, bool isUplink) const
{
  auto it = m_ueContextMap.find (rnti);
  if (it == m_ueContextMap.end ())
    {
      return std::numeric_limits<double>::quiet_NaN ();
    }
  return GetSchedulingCqi (it->second, isUplink);
}

// 功能验证演示访问器: 直接设置 UE 优先级, 并据此刷新 WRR 权重(保持一致)。
void
GeoBeamScheduler::SetUePriority (uint16_t rnti, ServicePriority priority)
{
  auto it = m_ueContextMap.find (rnti);
  if (it == m_ueContextMap.end ())
    {
      return;
    }
  it->second.priority = priority;
  it->second.wrrWeight = CalculateWrrWeight (priority, it->second.utType);
}

uint32_t
GeoBeamScheduler::EstimateRequiredRbs (uint16_t rnti) const
{
  auto it = m_ueContextMap.find (rnti);
  if (it == m_ueContextMap.end ())
    {
      return 1;
    }

  const SatUeContext& ctx = it->second;
  // 与主调度路径同口径: 走 NrAmc 真实 TBS 阶梯(RbsForBytes + GetSchedulingCqi),
  // 删除旧的 latestCqi*2.5 魔数与单RB×N 线性外推。切换准入 (ntn-gw-rrc) 调用签名不
  // 带方向, 这里按 DL 与 UL 各自缓存与各自 CQI 口径估算所需 RB, 取较大者以兼顾两个
  // 方向的资源占用。上限取各方向每波束预算。
  const uint32_t dlBuffer = std::max (ctx.dlBufferStatus, 1u);
  const uint32_t ulBuffer = std::max (ctx.ulBufferStatus, 1u);
  const uint32_t dlBudget =
    (m_resourceManager != nullptr) ? m_resourceManager->GetDlBeamBudgetRbs () : 25u;
  const uint32_t ulBudget =
    (m_resourceManager != nullptr) ? m_resourceManager->GetUlBeamBudgetRbs () : 25u;
  const uint32_t dlRawRbs =
    std::max (1u, RbsForBytes (GetSchedulingCqi (ctx, false), dlBuffer, dlBudget, false));
  const uint32_t ulRawRbs =
    std::max (1u, RbsForBytes (GetSchedulingCqi (ctx, true), ulBuffer, ulBudget, true));
  const uint32_t rawRequiredRbs = std::max ({1u, dlRawRbs, ulRawRbs});

  // 与"粗粒度准入闸"保持同一口径: 实际分配总是受每波束预算 clamp(bytesPerRb 很小,
  // 整缓存折算出的 raw RB 往往远超预算), 因此切换准入也用与 DL/UL gate 相同的
  // 优先级上限把 raw 需求收敛成"放行该 UE 所需的最小可服务 RB", 而非把整缓存当硬需求
  // (后者会让任何有数据的 UE 都因 raw≫预算被误判'不够用'而拒切)。
  const ServicePriority priority = ctx.priority;
  const uint32_t gatedRequiredRbs =
    (priority == ServicePriority::PRIORITY_EMERGENCY) ? std::min (3u, rawRequiredRbs) :
    (priority == ServicePriority::PRIORITY_VOICE) ? std::min (2u, rawRequiredRbs) :
    std::min (rawRequiredRbs, m_dataAdmitRbCap);
  return gatedRequiredRbs;
}

// 注: EnsureUeContextFromReport / DecideHandoverTarget /
//     ExecuteBeamHandover / ExecuteGroundToSatHandover /
//     ExecuteSatToGroundHandover / EstimateInterruptionDelay /
//     RecordHandoverOutcome / GetHandoverRecords /
//     GetHandoverStats / ExportHandoverStats
// 已迁移至 NtnGwRrc (ntn-gw-rrc.cc)。

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
