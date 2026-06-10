/*
 * 文件: contrib/geo-sat/examples/ntn-nr-ground-handover-demo.cc
 *
 * 阶段C-第一步: 真实 5G NR 地面小区 + 测量接入。
 *
 * - 用 NrHelper + EPC 实例化一个真实的地面 NR gNB 小区, 安装一个真实 NrUeNetDevice
 *   与下行业务流。
 * - 该 NR UE 与 geo-sat 切换 UE (SatUtRrc) 共享同一个移动模型: UE 从地面 gNB
 *   附近以恒定速度驶离。
 * - 周期采样器读取真实 NrUePhy::GetRsrp() 作为 GROUND 服务对象的测量值
 *   (替换原 demo 的解析型 GroundRsrpDbm), 卫星侧仍用解析几何模型。
 * - UE 驶出地面覆盖 -> 真实 NR 地面 RSRP 跌破卫星 -> A3 触发星地切换。
 *
 * 说明: 本步聚焦"地面变成真实 NR 模块 + 真实测量驱动切换决策"。真实数据面
 * 跨系统迁移(NR UE 脱离地面 gNB 接入卫星)与中断时延实测属后续增量。
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-epc-helper.h"
#include "ns3/nr-module.h"
#include "ns3/antenna-module.h"

#include "ns3/sat-ut-rrc.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/ntn-gw-rrc.h"
#include "ns3/admit-control.h"

#include <cmath>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnNrGroundHandoverDemo");

namespace {

// ---- 卫星侧解析几何模型 (与 ntn-handover-demo 一致) ----
constexpr double kFreqHz = 2.0e9;
constexpr double kLightSpeed = 3.0e8;

const Vector kSatPosition (0.0, 0.0, 600000.0);
constexpr double kSatTxPowerDbm = 50.0;
constexpr double kSatMaxGainDbi = 30.0;
constexpr double kSatBeamRadiusM = 200000.0;
constexpr double kSatSideLobeDbi = -5.0;

const Vector kBeam101Center (0.0, 0.0, 0.0);

// 地面 NR gNB 位置
const Vector kGroundGnbPosition (0.0, 0.0, 25.0);

// B1: GEO 单程传播时延
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

double
SatBeamRsrpDbm (const Vector& uePos, uint32_t /*beamId*/)
{
  const double off = HorizDist (uePos, kBeam101Center);
  double gain = kSatMaxGainDbi - 12.0 * std::pow (off / kSatBeamRadiusM, 2.0);
  gain = std::max (gain, kSatSideLobeDbi);
  const double slant = CalculateDistance (uePos, kSatPosition);
  return kSatTxPowerDbm + gain - FsplDb (slant);
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
OnMeasurementReport (Ptr<NtnGwRrc> gwRrc,
                     Ptr<SatUtRrc> rrc,
                     std::string label,
                     MeasReport report)
{
  NS_LOG_INFO ("[DEMO] " << label << " generated MeasReport"
               << " | event=" << static_cast<uint32_t> (report.eventType)
               << " | serving=" << ServiceObjectToString (report.servingObject)
               << " | target=" << ServiceObjectToString (report.bestNeighborObject));

  // B1: 报告经上行 GEO 传播后才到信关站 RRC。
  Simulator::Schedule (kUplinkDelay,
                       [gwRrc, rrc, label, report] ()
                       {
                         gwRrc->ReceiveMeasReport (report);
                         const ServiceObjectId newServing =
                             gwRrc->GetServingObjectForUe (report.ueId);
                         if (newServing == report.bestNeighborObject &&
                             report.bestNeighborObject != report.servingObject)
                           {
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

// 采样器上下文 (打包参数, 规避 Simulator::Schedule 的参数个数上限)
struct SampleCtx
{
  Ptr<SatUtRrc> rrc;
  Ptr<MobilityModel> ueMobility;
  Ptr<NrUePhy> nrUePhy;
  ServiceObjectId groundObject;
  ServiceObjectId satBeam;
  uint32_t stepMs {100};
  double simTime {12.0};
};

// 周期采样: 地面 RSRP 取真实 NrUePhy::GetRsrp(), 卫星 RSRP 用解析模型。
void
SampleMeasurements (SampleCtx* ctx)
{
  const double now = Simulator::Now ().GetSeconds ();
  const Vector pos = ctx->ueMobility->GetPosition ();

  const double groundRsrp = ctx->nrUePhy->GetRsrp ();   // 真实 NR 地面测量
  const double satRsrp = SatBeamRsrpDbm (pos, ctx->satBeam.objectId);

  NS_LOG_INFO ("[SAMPLE] t=" << now << "s | x=" << pos.x << "m"
               << " | groundRsrp(NR)=" << groundRsrp << " dBm"
               << " | satRsrp=" << satRsrp << " dBm");

  ctx->rrc->ProcessRawMeasurement (ctx->groundObject, groundRsrp);
  ctx->rrc->ProcessRawMeasurement (ctx->satBeam, satRsrp);

  if (now + (ctx->stepMs / 1000.0) < ctx->simTime)
    {
      Simulator::Schedule (MilliSeconds (ctx->stepMs), &SampleMeasurements, ctx);
    }
}

} // namespace

int
main (int argc, char* argv[])
{
  double simTime = 12.0;
  double ueStartX = 50.0;
  double ueVelocity = 300.0;
  double gnbTxPowerDbm = 20.0;
  uint32_t measStepMs = 100;
  double attachTime = 1.0;
  double sampleStartTime = 2.0;
  uint32_t rngRun = 1;
  double a3Offset = 3.0;
  uint32_t tttMs = 200;
  uint32_t minDwellMs = 1000;
  std::string hoStatsFile = "NrGroundHandoverStats.txt";

  CommandLine cmd;
  cmd.AddValue ("simTime", "Simulation time (s)", simTime);
  cmd.AddValue ("ueStartX", "UE initial x position (m from ground gNB)", ueStartX);
  cmd.AddValue ("ueVelocity", "UE velocity along +x (m/s)", ueVelocity);
  cmd.AddValue ("gnbTxPowerDbm", "Ground gNB Tx power (dBm)", gnbTxPowerDbm);
  cmd.AddValue ("measStepMs", "Measurement sampling period (ms)", measStepMs);
  cmd.AddValue ("attachTime", "Time to attach NR UE to ground gNB (s)", attachTime);
  cmd.AddValue ("sampleStartTime", "Time to start feeding measurements (s)", sampleStartTime);
  cmd.AddValue ("a3Offset", "A3 offset (dB)", a3Offset);
  cmd.AddValue ("tttMs", "Time-to-trigger (ms)", tttMs);
  cmd.AddValue ("minDwellMs", "Minimum dwell time (ms)", minDwellMs);
  cmd.AddValue ("hoStatsFile", "Handover statistics output file", hoStatsFile);
  cmd.AddValue ("rngRun", "RNG run number (fixes the channel LOS/NLOS draw)", rngRun);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (rngRun);

  LogComponentEnable ("NtnNrGroundHandoverDemo", LOG_LEVEL_INFO);
  LogComponentEnable ("SatUtRrc", LOG_LEVEL_INFO);
  LogComponentEnable ("GeoBeamScheduler", LOG_LEVEL_INFO);
  LogComponentEnable ("NtnGwRrc", LOG_LEVEL_INFO);

  const uint32_t groundId = 1;
  const ServiceObjectId groundObject {ServiceObjectType::SERVICE_OBJECT_GROUND, groundId};
  const ServiceObjectId satBeam1 {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, 101};

  // =====================================================================
  // 1. 真实 NR 地面小区 (gNB + UE + EPC + 下行业务)
  // =====================================================================
  NodeContainer gnbNode;
  gnbNode.Create (1);
  NodeContainer ueNode;
  ueNode.Create (1);

  // gNB 静止于地面位置
  MobilityHelper gnbMobility;
  gnbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  gnbMobility.Install (gnbNode);
  gnbNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (kGroundGnbPosition);

  // UE 以恒定速度沿 +x 驶离地面 gNB; 此移动模型同时供 SatUtRrc 使用。
  Ptr<ConstantVelocityMobilityModel> ueMobility = CreateObject<ConstantVelocityMobilityModel> ();
  ueMobility->SetPosition (Vector (ueStartX, 0.0, 1.5));
  ueMobility->SetVelocity (Vector (ueVelocity, 0.0, 0.0));
  ueNode.Get (0)->AggregateObject (ueMobility);

  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  nrHelper->SetEpcHelper (epcHelper);

  Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper> ();
  beamformingHelper->SetAttribute ("BeamformingMethod",
                                   TypeIdValue (DirectPathBeamforming::GetTypeId ()));
  nrHelper->SetBeamformingHelper (beamformingHelper);

  nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
  nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
  nrHelper->SetHarqEnabled (true);

  // 单 CC: 2.16 GHz, 20 MHz, numerology 1, 地面 UMa 场景
  CcBwpCreator ccBwpCreator;
  CcBwpCreator::SimpleOperationBandConf bandConf (2160.0e6, 20e6, 1, BandwidthPartInfo::UMa);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);
  nrHelper->InitializeOperationBand (&band, 0x07);
  BandwidthPartInfoPtrVector singleBwp = CcBwpCreator::GetAllBwps ({std::ref (band)});

  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("AntennaElement",
                                    PointerValue (CreateObject<IsotropicAntennaModel> ()));
  NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice (gnbNode, singleBwp);
  for (auto it = gnbDevices.Begin (); it != gnbDevices.End (); ++it)
    {
      Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice> (*it);
      gnb->GetPhy (0)->SetNumerology (1);
      gnb->GetPhy (0)->SetTxPower (gnbTxPowerDbm);
      gnb->UpdateConfig ();
    }

  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement",
                                   PointerValue (CreateObject<IsotropicAntennaModel> ()));
  NetDeviceContainer ueDevices = nrHelper->InstallUeDevice (ueNode, singleBwp);
  Ptr<NrUeNetDevice> ueNrDev = DynamicCast<NrUeNetDevice> (ueDevices.Get (0));
  ueNrDev->GetPhy (0)->SetNumerology (1);
  ueNrDev->UpdateConfig ();
  Ptr<NrUePhy> nrUePhy = ueNrDev->GetPhy (0);

  // UE 互联网协议栈 + IP + 默认路由
  InternetStackHelper internet;
  internet.Install (ueNode);
  Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address (ueDevices);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> ueRouting =
      ipv4RoutingHelper.GetStaticRouting (ueNode.Get (0)->GetObject<Ipv4> ());
  ueRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

  // 远程主机 + PGW p2p
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internetRh;
  internetRh.Install (remoteHostContainer);

  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("100Gbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("1ms"));
  NetDeviceContainer internetDevices = p2p.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase (Ipv4Address ("1.0.0.0"), Ipv4Mask ("255.0.0.0"));
  Ipv4InterfaceContainer internetIfaces = ipv4.Assign (internetDevices);
  Ptr<Ipv4StaticRouting> remoteHostRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // 下行业务: remoteHost -> UE (使 NR UE 持续测量并保持连接)
  const uint16_t dlPort = 1234;
  Ipv4Address ueAddr = ueIpIface.GetAddress (0);
  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (ueAddr, dlPort));
  onoff.SetAttribute ("DataRate", StringValue ("2Mbps"));
  onoff.SetAttribute ("PacketSize", UintegerValue (1000));
  onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  ApplicationContainer dlApp = onoff.Install (remoteHost);
  dlApp.Start (Seconds (sampleStartTime));
  dlApp.Stop (Seconds (simTime));

  PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         InetSocketAddress (Ipv4Address::GetAny (), dlPort));
  ApplicationContainer sinkApp = sink.Install (ueNode.Get (0));
  sinkApp.Start (Seconds (sampleStartTime - 0.5));
  sinkApp.Stop (Seconds (simTime));

  // 附着 NR UE 到地面 gNB
  Simulator::Schedule (Seconds (attachTime),
                       [nrHelper, ueDevices, gnbDevices] ()
                       {
                         nrHelper->AttachToClosestEnb (ueDevices, gnbDevices);
                         NS_LOG_INFO ("[NR] UE attached to ground gNB at t="
                                      << Simulator::Now ().GetSeconds () << "s");
                       });

  // =====================================================================
  // 2. geo-sat 切换侧 (调度器 + SatUtRrc, 共享 NR UE 的移动模型)
  // =====================================================================
  Ptr<GeoBeamScheduler> scheduler = CreateObject<GeoBeamScheduler> ();
  scheduler->Initialize (satBeam1.objectId, 30);

  Ptr<AdmitControl> admit = CreateObject<AdmitControl> ();
  admit->SetGroundAvailability (groundId, true);
  admit->SetGroundCapacity (groundId, 32);

  // 网络侧 RRC: 接 scheduler 做调度协作, 接 AdmitControl 做切换准入。
  Ptr<NtnGwRrc> gwRrc = CreateObject<NtnGwRrc> ();
  gwRrc->SetScheduler (scheduler);
  gwRrc->SetAdmitControl (admit);
  gwRrc->SetGroundAvailability (groundId, true);

  MeasConfig measConfig;
  measConfig.a3Offset = a3Offset;
  measConfig.hysteresis = 2.0;
  measConfig.timeToTrigger = MilliSeconds (tttMs);
  measConfig.minDwellTime = MilliSeconds (minDwellMs);
  measConfig.filterCoeff = 0.6;
  measConfig.enableGroundService = true;
  measConfig.enableSatelliteService = true;

  Ptr<SatUtRrc> rrc = CreateObject<SatUtRrc> ();
  rrc->SetUeId (1);
  rrc->SetMobilityModel (ueMobility);            // 与真实 NR UE 同一移动模型
  rrc->SetMeasConfig (measConfig);
  rrc->SetGroundAvailability (true);
  rrc->SetSatelliteAvailability (true);
  rrc->SetServingObject (groundObject);          // 初始驻留地面
  rrc->SetReportCallback (
      MakeBoundCallback (&OnMeasurementReport, gwRrc, rrc, std::string ("NrGroundUe")));

  NS_LOG_INFO ("[DEMO] Mobile UE starts on ground NR cell, drives out -> expect A3 ground->sat");

  // 采样器在附着稳定后启动
  SampleCtx sampleCtx;
  sampleCtx.rrc = rrc;
  sampleCtx.ueMobility = ueMobility;
  sampleCtx.nrUePhy = nrUePhy;
  sampleCtx.groundObject = groundObject;
  sampleCtx.satBeam = satBeam1;
  sampleCtx.stepMs = measStepMs;
  sampleCtx.simTime = simTime;
  Simulator::Schedule (Seconds (sampleStartTime), &SampleMeasurements, &sampleCtx);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  HandoverStatsSummary summary = gwRrc->GetHandoverStats ();
  NS_LOG_INFO ("======== Handover Statistics Summary ========");
  NS_LOG_INFO ("[STATS] attempts=" << summary.attempts
               << " successes=" << summary.successes
               << " failures=" << summary.failures
               << " failureRate=" << summary.failureRate);
  NS_LOG_INFO ("[STATS] handoverDelay avg=" << summary.avgHandoverDelayMs
               << "ms max=" << summary.maxHandoverDelayMs << "ms");
  NS_LOG_INFO ("[STATS] interruptDelay avg=" << summary.avgInterruptionDelayMs
               << "ms max=" << summary.maxInterruptionDelayMs << "ms");
  gwRrc->ExportHandoverStats (hoStatsFile);

  Simulator::Destroy ();
  return 0;
}
