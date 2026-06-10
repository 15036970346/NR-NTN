#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "ns3/sat-ut-rrc.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/admit-control.h"

#include <cmath>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnHandoverDemo");

namespace {

// ===================================================================
// B 阶段: 自包含几何信道模型 (FSPL + 波束/天线增益)
// 参数为演示而缩放(卫星置于 600km、地面 EIRP 调低使星地交叉点 ~3km),
// 物理形式与 SatChannel::DoRxPowerCalculation 一致。
// ===================================================================
constexpr double kFreqHz = 2.0e9;        // S 频段
constexpr double kLightSpeed = 3.0e8;

// 卫星 (波束覆盖半径 200km, 见任务书)
const Vector kSatPosition (0.0, 0.0, 600000.0);
constexpr double kSatTxPowerDbm = 50.0;
constexpr double kSatMaxGainDbi = 30.0;
constexpr double kSatBeamRadiusM = 200000.0;
constexpr double kSatSideLobeDbi = -5.0;

// 地面基站 (地面 EIRP 调低, 使星地 RSRP 交叉点落在 ~3km 处)
const Vector kGroundPosition (0.0, 0.0, 30.0);
constexpr double kGroundTxPowerDbm = 20.0;
constexpr double kGroundMaxGainDbi = 10.0;

// 波束足迹中心 (波束102随时间向UE方向扫描, 模拟GEO波束指向)
const Vector kBeam101Center (0.0, 0.0, 0.0);
const Vector kBeam102Start (300000.0, 0.0, 0.0);
constexpr double kBeam102SteerMps = 30000.0;  // 合成的波束扫描速度

// B1: GEO 单程传播时延 (与调度器 m_uplinkPropDelay 一致)
const Time kUplinkDelay = MilliSeconds (300);
const Time kDownlinkDelay = MilliSeconds (300);

double
FsplDb (double dist3D)
{
  if (dist3D < 1.0)
    {
      dist3D = 1.0;
    }
  return 20.0 * std::log10 (4.0 * M_PI * dist3D * kFreqHz / kLightSpeed);
}

double
HorizDist (const Vector& a, const Vector& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt (dx * dx + dy * dy);
}

Vector
BeamCenterAt (uint32_t beamId, double tSec)
{
  if (beamId == 102)
    {
      const double x = std::max (0.0, kBeam102Start.x - kBeam102SteerMps * tSec);
      return Vector (x, 0.0, 0.0);
    }
  return kBeam101Center;
}

double
SatBeamRsrpDbm (const Vector& uePos, uint32_t beamId, double tSec)
{
  const Vector center = BeamCenterAt (beamId, tSec);
  const double off = HorizDist (uePos, center);
  double gain = kSatMaxGainDbi - 12.0 * std::pow (off / kSatBeamRadiusM, 2.0);
  gain = std::max (gain, kSatSideLobeDbi);
  const double slant = CalculateDistance (uePos, kSatPosition);
  return kSatTxPowerDbm + gain - FsplDb (slant);
}

double
GroundRsrpDbm (const Vector& uePos)
{
  const double d = CalculateDistance (uePos, kGroundPosition);
  return kGroundTxPowerDbm + kGroundMaxGainDbi - FsplDb (d);
}

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

void
OnMeasurementReport (Ptr<GeoBeamScheduler> scheduler,
                     Ptr<SatUtRrc> rrc,
                     std::string label,
                     MeasReport report)
{
  NS_LOG_INFO ("[DEMO] " << label << " generated MeasReport"
               << " | event=" << static_cast<uint32_t> (report.eventType)
               << " | serving=" << ServiceObjectToString (report.servingObject)
               << " | target=" << ServiceObjectToString (report.bestNeighborObject));

  // B1: 测量报告经上行 GEO 传播后才到达 gNB/调度器。
  Simulator::Schedule (kUplinkDelay,
                       [scheduler, rrc, label, report] ()
                       {
                         scheduler->ReceiveMeasReport (report);

                         const ServiceObjectId newServing =
                             scheduler->GetServingObjectForUe (report.ueId);
                         if (newServing == report.bestNeighborObject &&
                             report.bestNeighborObject != report.servingObject)
                           {
                             // B1: RRC 重配命令经下行 GEO 传播后 UE 才应用 (替代旧的 1ms 胶水)。
                             Simulator::Schedule (kDownlinkDelay,
                                                  [rrc, target = newServing, label] ()
                                                  {
                                                    NS_LOG_INFO ("[DEMO] " << label
                                                                 << " applied RRC reconfig -> "
                                                                 << ServiceObjectToString (target));
                                                    rrc->SetServingObject (target);
                                                  });
                           }
                       });
}

struct DemoUe
{
  uint16_t ueId {0};
  Ptr<SatUtRrc> rrc;
  Ptr<MobilityModel> mobility;
  std::string label;
  std::vector<ServiceObjectId> objects;   // 该 UE 需周期测量的服务对象
};

// B2: 周期采样器 — 按 UE 当前位置由几何模型计算 RSRP 并喂给 RRC,
// 替代旧的硬编码 ScheduleMeasurementBurst。
void
SampleMeasurements (const std::vector<DemoUe>* ues, uint32_t stepMs, double simTime)
{
  const double now = Simulator::Now ().GetSeconds ();
  for (const auto& ue : *ues)
    {
      const Vector pos = ue.mobility->GetPosition ();
      for (const auto& obj : ue.objects)
        {
          const double rsrp =
              (obj.type == ServiceObjectType::SERVICE_OBJECT_GROUND)
                  ? GroundRsrpDbm (pos)
                  : SatBeamRsrpDbm (pos, obj.objectId, now);
          ue.rrc->ProcessRawMeasurement (obj, rsrp);
        }
    }
  if (now + (stepMs / 1000.0) < simTime)
    {
      Simulator::Schedule (MilliSeconds (stepMs), &SampleMeasurements, ues, stepMs, simTime);
    }
}

void
SetGroundState (Ptr<GeoBeamScheduler> scheduler,
                const std::vector<DemoUe>& ues,
                uint32_t groundId,
                bool enabled)
{
  NS_LOG_INFO ("[DEMO] Ground " << groundId << " -> " << (enabled ? "ON" : "OFF")
               << " at t=" << Simulator::Now ().GetSeconds () << "s");
  scheduler->SetGroundAvailability (groundId, enabled);
  for (const auto& ue : ues)
    {
      ue.rrc->SetGroundAvailability (enabled);
    }
}

} // namespace

int
main (int argc, char* argv[])
{
  bool enableGroundHandover = true;
  bool enableBeamHandover = true;
  bool groundEnabledAtStart = true;
  double groundOffTime = 2.0;
  double groundOnTime = 5.0;
  double a3Offset = 3.0;
  uint32_t tttMs = 200;
  uint32_t minDwellMs = 1000;
  double simTime = 12.0;
  uint32_t measStepMs = 100;
  std::string scenarioMode = "all";
  std::string hoStatsFile = "HandoverStats.txt";

  CommandLine cmd;
  cmd.AddValue ("enableGroundHandover", "Enable ground <-> satellite handover scenarios", enableGroundHandover);
  cmd.AddValue ("enableBeamHandover", "Enable satellite beam handover scenario", enableBeamHandover);
  cmd.AddValue ("groundEnabledAtStart", "Whether ground starts enabled", groundEnabledAtStart);
  cmd.AddValue ("groundOffTime", "Ground switch OFF time in seconds", groundOffTime);
  cmd.AddValue ("groundOnTime", "Ground switch ON time in seconds", groundOnTime);
  cmd.AddValue ("a3Offset", "A3 offset in dB", a3Offset);
  cmd.AddValue ("tttMs", "Time-to-trigger in ms", tttMs);
  cmd.AddValue ("minDwellMs", "Minimum dwell time in ms", minDwellMs);
  cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue ("scenarioMode",
                "Scenario mode: all | initial-sat | ground-cycle | beam | mobile-star",
                scenarioMode);
  cmd.AddValue ("measStepMs", "Measurement sampling period in ms", measStepMs);
  cmd.AddValue ("hoStatsFile", "Output file for handover statistics", hoStatsFile);
  cmd.Parse (argc, argv);

  LogComponentEnable ("NtnHandoverDemo", LOG_LEVEL_INFO);
  LogComponentEnable ("SatUtRrc", LOG_LEVEL_INFO);
  LogComponentEnable ("GeoBeamScheduler", LOG_LEVEL_INFO);
  LogComponentEnable ("AdmitControl", LOG_LEVEL_INFO);

  const uint32_t groundId = 1;
  const ServiceObjectId groundObject {ServiceObjectType::SERVICE_OBJECT_GROUND, groundId};
  const ServiceObjectId satBeam1 {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, 101};
  const ServiceObjectId satBeam2 {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, 102};

  Ptr<GeoBeamScheduler> scheduler = CreateObject<GeoBeamScheduler> ();
  scheduler->Initialize (satBeam1.objectId, 30);

  Ptr<AdmitControl> admit = CreateObject<AdmitControl> ();
  admit->SetGroundAvailability (groundId, groundEnabledAtStart);
  admit->SetGroundCapacity (groundId, 32);
  scheduler->SetAdmitControl (admit);
  scheduler->SetGroundAvailability (groundId, groundEnabledAtStart);

  MeasConfig measConfig;
  measConfig.a3Offset = a3Offset;
  measConfig.hysteresis = 2.0;
  measConfig.timeToTrigger = MilliSeconds (tttMs);
  measConfig.minDwellTime = MilliSeconds (minDwellMs);
  measConfig.filterCoeff = 0.6;
  measConfig.enableGroundService = true;
  measConfig.enableSatelliteService = true;

  std::vector<DemoUe> ues;
  auto createUe = [&] (uint16_t ueId, const std::string& label, const Vector& pos,
                       const Vector& vel,
                       const std::vector<ServiceObjectId>& objects) -> DemoUe
    {
      DemoUe ue;
      ue.ueId = ueId;
      ue.label = label;
      ue.objects = objects;
      ue.rrc = CreateObject<SatUtRrc> ();
      if (vel.x == 0.0 && vel.y == 0.0 && vel.z == 0.0)
        {
          Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel> ();
          m->SetPosition (pos);
          ue.mobility = m;
        }
      else
        {
          Ptr<ConstantVelocityMobilityModel> m = CreateObject<ConstantVelocityMobilityModel> ();
          m->SetPosition (pos);
          m->SetVelocity (vel);
          ue.mobility = m;
        }
      ue.rrc->SetUeId (ueId);
      ue.rrc->SetMobilityModel (ue.mobility);
      ue.rrc->SetMeasConfig (measConfig);
      ue.rrc->SetGroundAvailability (groundEnabledAtStart);
      ue.rrc->SetSatelliteAvailability (true);
      return ue;
    };

  if (scenarioMode == "all" || scenarioMode == "initial-sat")
    {
      // 静止 UE, 远离地面基站(5km)使地面 RSRP 弱、卫星更优 -> 初始接入选卫星。
      ues.push_back (createUe (1, "InitialSatUe", Vector (5000.0, 0.0, 0.0),
                               Vector (0, 0, 0), {groundObject, satBeam1}));
    }
  if (enableGroundHandover && (scenarioMode == "all" || scenarioMode == "ground-cycle"))
    {
      // 静止 UE, 近地面基站(300m); 由地面可用性 ON/OFF 驱动星地切换。
      ues.push_back (createUe (2, "GroundCycleUe", Vector (300.0, 0.0, 0.0),
                               Vector (0, 0, 0), {groundObject, satBeam2}));
    }
  if (enableBeamHandover && (scenarioMode == "all" || scenarioMode == "beam"))
    {
      // 静止 UE(距波束101中心150km, 增益已衰减); 波束102向UE扫描使其 RSRP 超过波束101 -> A3。
      ues.push_back (createUe (3, "BeamHandoverUe", Vector (150000.0, 0.0, 0.0),
                               Vector (0, 0, 0), {satBeam1, satBeam2}));
    }
  if (scenarioMode == "all" || scenarioMode == "mobile-star")
    {
      // B3: 移动 UE 从地面基站附近(200m)以 500m/s 驶出, 地面 RSRP 跌破卫星 -> A3 星地切换。
      ues.push_back (createUe (4, "MobileStarGroundUe", Vector (200.0, 0.0, 0.0),
                               Vector (500.0, 0.0, 0.0), {groundObject, satBeam1}));
    }

  for (auto& ue : ues)
    {
      ue.rrc->SetReportCallback (
        MakeBoundCallback (&OnMeasurementReport, scheduler, ue.rrc, ue.label));
    }

  for (auto& ue : ues)
    {
      if (ue.label == "InitialSatUe")
        {
          NS_LOG_INFO ("[DEMO] Scenario1: static UE with weak ground, initial access selects satellite");
          ue.rrc->SetServingObject (satBeam1);
        }

      if (ue.label == "GroundCycleUe")
        {
          NS_LOG_INFO ("[DEMO] Scenario2+3: ground -> sat on ground OFF, then sat -> ground after ground ON and A3");
          ue.rrc->SetServingObject (groundObject);

          Simulator::Schedule (Seconds (groundOffTime),
                               [scheduler, &ues, groundId] ()
                               {
                                 SetGroundState (scheduler, ues, groundId, false);
                               });
          Simulator::Schedule (Seconds (groundOnTime),
                               [scheduler, &ues, groundId] ()
                               {
                                 SetGroundState (scheduler, ues, groundId, true);
                               });
        }

      if (ue.label == "BeamHandoverUe")
        {
          NS_LOG_INFO ("[DEMO] Scenario4: satellite beam1 -> beam2 A3 handover (beam steering)");
          ue.rrc->SetServingObject (satBeam1);
        }

      if (ue.label == "MobileStarGroundUe")
        {
          NS_LOG_INFO ("[DEMO] Scenario5: mobile UE drives out of ground coverage -> A3 ground->sat");
          ue.rrc->SetServingObject (groundObject);
        }
    }

  // B2: 启动几何驱动的周期测量采样器。
  Simulator::Schedule (MilliSeconds (measStepMs), &SampleMeasurements, &ues, measStepMs, simTime);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  HandoverStatsSummary summary = scheduler->GetHandoverStats ();
  NS_LOG_INFO ("======== Handover Statistics Summary ========");
  NS_LOG_INFO ("[STATS] attempts=" << summary.attempts
               << " successes=" << summary.successes
               << " failures=" << summary.failures
               << " failureRate=" << summary.failureRate);
  NS_LOG_INFO ("[STATS] handoverDelay avg=" << summary.avgHandoverDelayMs
               << "ms max=" << summary.maxHandoverDelayMs << "ms");
  NS_LOG_INFO ("[STATS] interruptDelay avg=" << summary.avgInterruptionDelayMs
               << "ms max=" << summary.maxInterruptionDelayMs << "ms");
  scheduler->ExportHandoverStats (hoStatsFile);

  Simulator::Destroy ();
  return 0;
}
