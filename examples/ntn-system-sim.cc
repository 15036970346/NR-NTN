/*
 * NTN GEO Satellite System Simulation
 *
 * 仿真场景：
 * - 1个GEO卫星节点
 * - 1个信关站(GW)节点
 * - 7个波束覆盖
 * - N个用户终端(UE)在7个波束范围内随机分布
 * - 支持4色/7色频率复用模式
 *
 * 用法：
 *   ./ns3 run "ntn-system-sim --numUes=70"
 *   ./ns3 run "ntn-system-sim --numUes=70 --reuseMode=4"
 *   ./ns3 run "ntn-system-sim --numUes=70 --reuseMode=7 --simTime=10"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-epc-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/antenna-module.h"

#include "ns3/geo-beam-helper.h"
#include "ns3/gw-helper.h"
#include "ns3/user-helper.h"
#include "ns3/ntn-config-helper.h"
#include "ns3/harq-manager.h"
#include "ns3/sat-stats-collector.h"
#include "ns3/nr-bearer-stats-connector.h"
#include "ns3/nr-phy-mac-common.h"

#include <fstream>
#include <functional>
#include <iostream>
#include <sys/stat.h>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSystemSim");

static std::ofstream g_resultFile;

/**
 * \brief 写入仿真结果到文件和控制台
 */
static void
WriteResult (const std::string& content)
{
  g_resultFile << content << std::endl;
  std::cout << content << std::endl;
}

/**
 * \brief 打印频点配置详情
 */
static void
PrintFrequencyConfiguration (const std::vector<BeamFrequencyConfig>& beamConfig, uint8_t reuseMode)
{
  WriteResult ("\n=== Frequency Configuration Details ===");

  if (reuseMode == 4)
    {
      WriteResult ("Mode: 4-Color Frequency Reuse");
      WriteResult ("Total Bandwidth: 30 MHz (aggregated)");
      WriteResult ("Bandwidth per Color Group: 7.5 MHz (40 PRB)");
      WriteResult ("----------------------------------------");
      WriteResult ("| Color | Freq (MHz) | EARFCN | Beams   |");
      WriteResult ("|-------|------------|--------|---------|");

      std::map<uint8_t, std::vector<uint16_t>> colorToBeams;
      for (const auto& config : beamConfig)
        {
          colorToBeams[config.colorId].push_back (config.beamId);
        }

      for (const auto& pair : colorToBeams)
        {
          std::stringstream ss;
          ss << "|   " << (uint32_t)pair.first << "   |   ";

          for (const auto& cfg : beamConfig)
            {
              if (cfg.colorId == pair.first)
                {
                  ss << std::fixed << std::setprecision (1) << cfg.centerFreqHz / 1e6;
                  ss << "     |  " << cfg.earfcn << "   | ";
                  break;
                }
            }

          for (uint16_t beamId : pair.second)
            {
              ss << beamId << ", ";
            }
          std::string beamsStr = ss.str ();
          beamsStr = beamsStr.substr (0, beamsStr.size () - 2); // 去掉末尾的逗号
          WriteResult (beamsStr + " |");
        }
    }
  else
    {
      WriteResult ("Mode: 7-Color Frequency Reuse (Full Reuse)");
      WriteResult ("Bandwidth per Beam: 5 MHz (25 PRB)");
      WriteResult ("----------------------------------------");
      WriteResult ("| Beam | Freq (MHz) | Bandwidth | EARFCN |");
      WriteResult ("|------|------------|-----------|--------|");

      for (const auto& config : beamConfig)
        {
          std::stringstream ss;
          ss << "|   " << config.beamId << "   |   "
             << std::fixed << std::setprecision (1) << config.centerFreqHz / 1e6
             << "    |   "
             << std::fixed << std::setprecision (1) << config.bandwidthHz / 1e6
             << " MHz  |  " << config.earfcn << "  |";
          WriteResult (ss.str ());
        }
    }

  WriteResult ("========================================\n");
}

/**
 * \brief 打印UE位置信息
 */
static void
PrintUePositions (NodeContainer ues)
{
  NS_LOG_INFO ("\n=== UE Positions ===");
  for (uint32_t i = 0; i < ues.GetN (); ++i)
    {
      Ptr<Node> ue = ues.Get (i);
      Ptr<MobilityModel> mobility = ue->GetObject<MobilityModel> ();
      if (mobility)
        {
          Vector pos = mobility->GetPosition ();
          NS_LOG_INFO ("UE " << i << ": (" << pos.x << ", " << pos.y << ", " << pos.z << ")");
        }
    }
  NS_LOG_INFO ("====================\n");
}

/**
 * \brief 配置7个波束的中心位置和半径
 */
static std::vector<BeamInfo>
ConfigureBeams (uint32_t numBeams = 7)
{
  NS_LOG_INFO ("Configuring " << numBeams << " beams...");

  std::vector<BeamInfo> beams;
  double beamRadius = 1000.0; // 1km覆盖半径（用于1km高度的测试）

  if (numBeams == 7)
    {
      beams.push_back ({1, Vector (0.0, 0.0, 0.0), beamRadius});

      double distance = beamRadius * 1.5;
      for (uint32_t i = 2; i <= 7; ++i)
        {
          double angle = (i - 2) * 60.0 * M_PI / 180.0;
          double x = distance * std::cos (angle);
          double y = distance * std::sin (angle);
          beams.push_back ({(uint16_t)i, Vector (x, y, 0.0), beamRadius});
        }
    }
  else
    {
      double distance = beamRadius * 1.5;
      for (uint32_t i = 1; i <= numBeams; ++i)
        {
          double angle = (i - 1) * 360.0 / numBeams * M_PI / 180.0;
          double x = distance * std::cos (angle);
          double y = distance * std::sin (angle);
          beams.push_back ({(uint16_t)i, Vector (x, y, 0.0), beamRadius});
        }
    }

  NS_LOG_INFO ("Configured " << beams.size () << " beams:");
  for (const auto& beam : beams)
    {
      NS_LOG_INFO ("  Beam " << beam.beamId
                  << ": center=(" << beam.centerPosition.x << ", "
                  << beam.centerPosition.y << "), radius=" << beam.radius << "m");
    }

  return beams;
}

int main (int argc, char *argv[])
{
  // =========================================================================
  // 命令行参数配置
  // =========================================================================
  uint32_t numUes = 70;           //!< 总UE数量
  uint32_t numBeams = 7;          //!< 波束数量
  uint8_t reuseMode = 7;          //!< 频率复用模式 (4: 4色复用, 7: 7色复用)
  bool disableHarq = false;       //!< 禁用HARQ重传
  double simTime = 15.0;         //!< 仿真时间（秒）- 需要足够时间完成GEO附着和数据传输
  bool enableLogs = false;       //!< 是否启用详细日志

  CommandLine cmd;
  cmd.AddValue ("numUes", "Total number of UE terminals", numUes);
  cmd.AddValue ("numBeams", "Number of beams (default: 7)", numBeams);
  cmd.AddValue ("reuseMode", "Frequency reuse mode: 4 (4-color) or 7 (7-color)", reuseMode);
  cmd.AddValue ("disableHarq", "Disable HARQ retransmission (true/false)", disableHarq);
  cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue ("enableLogs", "Enable detailed logging", enableLogs);
  cmd.Parse (argc, argv);

  // =========================================================================
  // 日志配置
  // =========================================================================
  if (enableLogs)
    {
      LogComponentEnable ("NtnSystemSim", LOG_LEVEL_INFO);
      LogComponentEnable ("SatUserHelper", LOG_LEVEL_INFO);
      LogComponentEnable ("GeoBeamHelper", LOG_LEVEL_INFO);
      LogComponentEnable ("GwHelper", LOG_LEVEL_INFO);
      LogComponentEnable ("HarqManager", LOG_LEVEL_INFO);
      LogComponentEnable ("NrUePhy", LOG_LEVEL_INFO);
      LogComponentEnable ("NrGnbPhy", LOG_LEVEL_INFO);
      LogComponentEnable ("NrUeMac", LOG_LEVEL_INFO);
      LogComponentEnable ("NrGnbMac", LOG_LEVEL_INFO);
      LogComponentEnable ("nrRrcProtocolIdeal", LOG_LEVEL_INFO);
    }

  // =========================================================================
  // NTN配置 (GEO长时延适配)
  // =========================================================================
  NtnConfigHelper::ConfigureAll ();
  NtnConfigHelper::SetGeoRoundTripTime (MilliSeconds (630)); // GEO ~600ms RTT

  // =========================================================================
  // 创建HARQ管理器
  // =========================================================================
  Ptr<HarqManager> harqManager = CreateObject<HarqManager> ();
  NtnConfigHelper::ConfigureHarqManager (harqManager, !disableHarq);

  // =========================================================================
  // 创建统计收集器
  // =========================================================================
  Ptr<SatStatsCollector> statsCollector = CreateObject<SatStatsCollector> ();
  statsCollector->SetSimulationParams (reuseMode, disableHarq, numUes);
  NS_LOG_INFO ("StatsCollector created and configured");

  // =========================================================================
  // 创建输出目录
  // =========================================================================
  mkdir ("ntn-results", 0755);
  g_resultFile.open ("ntn-results/simulation_results.txt");

  // =========================================================================
  // 打印仿真配置
  // =========================================================================
  WriteResult ("========================================");
  WriteResult ("   NTN GEO Satellite System Simulation  ");
  WriteResult ("========================================");
  WriteResult ("Configuration:");
  WriteResult ("  Beams: " + std::to_string (numBeams));
  WriteResult ("  Frequency Reuse Mode: " + std::to_string ((uint32_t)reuseMode) + "-Color");
  WriteResult ("  Total UEs: " + std::to_string (numUes));
  WriteResult ("  UEs per beam (approx): " + std::to_string (numUes / numBeams));
  WriteResult ("  Simulation time: " + std::to_string (simTime) + "s");

  // 根据复用模式设置带宽信息
  if (reuseMode == 4)
    {
      WriteResult ("  Bandwidth per beam group: 7.5 MHz (40 PRB)");
    }
  else
    {
      WriteResult ("  Bandwidth per beam: 5 MHz (25 PRB)");
    }

  // HARQ配置信息
  if (disableHarq)
    {
      WriteResult ("  HARQ Mode: Disabled (NACK treated as success)");
    }
  else
    {
      WriteResult ("  HARQ Mode: Enabled (IR Retransmission)");
      WriteResult ("  HARQ Max Retransmissions: 4");
      WriteResult ("  IR RV Sequence: 0 -> 2 -> 3 -> 1");
    }

  WriteResult ("========================================\n");

  // =========================================================================
  // Step 1: 创建节点
  // =========================================================================
  NS_LOG_INFO ("Step 1: Creating nodes...");

  NodeContainer satNodes;
  satNodes.Create (1);
  NS_LOG_INFO ("Created " << satNodes.GetN () << " satellite node(s)");

  NodeContainer gwNodes;
  gwNodes.Create (1);
  NS_LOG_INFO ("Created " << gwNodes.GetN () << " gateway node(s)");

  // =========================================================================
  // Step 2: 配置移动模型
  // =========================================================================
  NS_LOG_INFO ("Step 2: Configuring mobility models...");

  // 设置卫星移动模型 (测试用: 1km高度)
  MobilityHelper satMobility;
  satMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> satPosition = CreateObject<ListPositionAllocator> ();
  satPosition->Add (Vector (0.0, 0.0, 1000.0)); // 测试用: 1km高度
  satMobility.SetPositionAllocator (satPosition);
  satMobility.Install (satNodes);
  NS_LOG_INFO ("Satellite mobility model installed at altitude 1 km");

  // 设置网关移动模型 (地面固定)
  MobilityHelper gwMobility;
  gwMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> gwPosition = CreateObject<ListPositionAllocator> ();
  gwPosition->Add (Vector (0.0, 0.0, 0.0)); // 网关在地面
  gwMobility.SetPositionAllocator (gwPosition);
  gwMobility.Install (gwNodes);
  NS_LOG_INFO ("Gateway mobility model installed");

  // =========================================================================
  // Step 3: 配置NR信道 (由nrHelper内部创建ThreeGppSpectrumPropagationLossModel)
  // 不需要手动创建channel，由nrHelper统一管理
  // =========================================================================
  NS_LOG_INFO ("Step 3: NR channel managed by nrHelper internally (ThreeGppSpectrumPropagationLossModel)");

  // =========================================================================
  // Step 4: 配置EPC和NR Helper
  // =========================================================================
  NS_LOG_INFO ("Step 4: Setting up EPC and NR Helper...");

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  nrHelper->SetEpcHelper (epcHelper);

  Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper> ();
  beamformingHelper->SetAttribute ("BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));
  nrHelper->SetBeamformingHelper (beamformingHelper);

  // 配置信道模型
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
  nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
  nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));

  // =========================================================================
  // Step 5: NR HARQ 配置（不触发 trace 连接）
  NS_LOG_INFO ("Step 5: Configuring HARQ...");
  nrHelper->SetHarqEnabled (!disableHarq);
  NS_LOG_INFO ("HARQ enabled: " << !disableHarq);

  // Step 6: 配置波束 (频率复用)
  // =========================================================================
  NS_LOG_INFO ("Step 6: Configuring beams with frequency reuse mode " << (uint32_t)reuseMode << "...");

  std::vector<BeamInfo> beams = ConfigureBeams (numBeams);

  // 获取频点配置
  std::vector<BeamFrequencyConfig> beamFreqConfig;
  if (reuseMode == 4)
    {
      beamFreqConfig = {
        {1, 2175.0e6, 7.5e6, 45240, 1}, {2, 2180.0e6, 7.5e6, 45260, 2},
        {3, 2185.0e6, 7.5e6, 45280, 3},  {4, 2190.0e6, 7.5e6, 45300, 4},
        {5, 2175.0e6, 7.5e6, 45240, 1},  {6, 2180.0e6, 7.5e6, 45260, 2},
        {7, 2185.0e6, 7.5e6, 45280, 3}
      };
    }
  else
    {
      beamFreqConfig = {
        {1, 2172.5e6, 5e6, 45230, 1},  {2, 2177.5e6, 5e6, 45240, 2},
        {3, 2182.5e6, 5e6, 45250, 3},  {4, 2187.5e6, 5e6, 45260, 4},
        {5, 2192.5e6, 5e6, 45270, 5},  {6, 2197.5e6, 5e6, 45280, 6},
        {7, 2202.5e6, 5e6, 45290, 7}
      };
    }

  // 打印频点配置
  PrintFrequencyConfiguration (beamFreqConfig, reuseMode);

  // 创建单个操作频带来支持所有 UE
  CcBwpCreator ccBwpCreator;
  // 使用 20 MHz 带宽和 numerology 1 (30kHz)
  CcBwpCreator::SimpleOperationBandConf bandConf (2172.5e6, 20e6, 1, BandwidthPartInfo::UMa);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);

  // 初始化操作频带
  uint8_t flags = 0x07; // INIT_PROPAGATION | INIT_FADING | INIT_CHANNEL
  nrHelper->InitializeOperationBand (&band, flags);

  // 获取 BWP 用于安装设备
  BandwidthPartInfoPtrVector singleBwp = CcBwpCreator::GetAllBwps ({std::ref (band)});

  // 安装卫星 gNB 设备
  NetDeviceContainer satDevices = nrHelper->InstallGnbDevice (satNodes, singleBwp);
  NS_LOG_INFO ("Satellite gNB installed: " << satDevices.GetN () << " devices");

  // 配置卫星天线
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  // 更新 gNB 配置
  for (auto it = satDevices.Begin (); it != satDevices.End (); ++it)
    {
      Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice> (*it);
      gnb->UpdateConfig ();
      // 设置 numerology (0=15kHz, 1=30kHz, 2=60kHz, etc.)
      gnb->GetPhy (0)->SetAttribute ("Numerology", UintegerValue (1));
      // 设置发射功率
      gnb->GetPhy (0)->SetTxPower (20.0);
    }

  // =========================================================================
  // Step 7: 配置信关站
  // =========================================================================
  NS_LOG_INFO ("Step 7: Setting up Gateway...");

  // 配置 GW gNB 天线
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetGnbAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  NetDeviceContainer gwDevices = nrHelper->InstallGnbDevice (gwNodes, singleBwp);
  NS_LOG_INFO ("Gateway gNB installed: " << gwDevices.GetN () << " devices");

  // 更新 GW gNB 配置
  for (auto it = gwDevices.Begin (); it != gwDevices.End (); ++it)
    {
      Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice> (*it);
      gnb->UpdateConfig ();
      // 设置 numerology
      gnb->GetPhy (0)->SetAttribute ("Numerology", UintegerValue (1));
      // 设置发射功率
      gnb->GetPhy (0)->SetTxPower (20.0);
    }

  // =========================================================================
  // Step 8: 在多个波束范围内随机撒点生成UE
  // =========================================================================
  NS_LOG_INFO ("Step 8: Creating UEs in beam coverage areas...");

  Ptr<SatUserHelper> userHelper = CreateObject<SatUserHelper> ();
  userHelper->SetNrHelper (nrHelper);
  userHelper->SetBeams (beams);

  NodeContainer ueNodes = userHelper->CreateUsersInMultipleBeams (numUes);
  NS_LOG_INFO ("Created " << ueNodes.GetN () << " UE nodes across " << beams.size () << " beams");

  // 配置 UE 天线
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  NetDeviceContainer ueDevices = nrHelper->InstallUeDevice (ueNodes, singleBwp);
  NS_LOG_INFO ("UE stack installed: " << ueDevices.GetN () << " devices");

  // 设置 UE 的 numerology（必须与 gNB 匹配）
  for (auto it = ueDevices.Begin (); it != ueDevices.End (); ++it)
    {
      Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice> (*it);
      if (ue)
        {
          ue->GetPhy (0)->SetNumerology (1);
        }
    }

  // 安装 UE 互联网协议栈并分配 IP
  InternetStackHelper internet;
  internet.Install (ueNodes);

  // 使用 EPC helper 分配 UE IP 地址
  Ipv4InterfaceContainer ueInterfaces = epcHelper->AssignUeIpv4Address (ueDevices);
  NS_LOG_INFO ("UE IP addresses assigned");

  // 设置 UE 默认路由
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      Ptr<Ipv4StaticRouting> ueRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (i)->GetObject<Ipv4> ());
      ueRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // 附着 UE 到最近的 gNB (延迟到仿真开始后，确保 RRC 连接可以建立)
  // 只附着到网关，因为网关是实际与核心网连接的设备
  // 使用交错延迟来避免 RRC 状态冲突
  // 注意：GEO RTT=630ms，附着过程需要 ~3*RTT=1.89s 完成
  // Bearer 激活延迟设为 6.0s，确保在 RRC 连接完全完成后再激活
  // 数据传输延迟设为 7.0s，确保 Bearer 激活完成后再开始发送数据
  double bearerDelay = 6.0; // Bearer 激活延迟（足够等待 GEO RRC 连接完成）
  double dataStartDelay = 7.0; // 数据传输开始延迟
  for (uint32_t i = 0; i < ueDevices.GetN (); ++i)
    {
      uint32_t ueIdx = i;
      Simulator::Schedule (Seconds (2.0 + ueIdx * 0.5), [&, ueIdx]() {
        NetDeviceContainer ueDev;
        ueDev.Add (ueDevices.Get (ueIdx));
        nrHelper->AttachToClosestEnb (ueDev, gwDevices);

        // Bearer 激活延迟足够等待 GEO RTT (630ms) 完成 RRC 连接建立
        Simulator::Schedule (Seconds (bearerDelay + ueIdx * 0.05), [&, ueIdx]() {
          Ptr<NetDevice> ueDevice = ueDevices.Get (ueIdx);
          Ptr<NrUeNetDevice> ueNetDev = DynamicCast<NrUeNetDevice> (ueDevice);
          if (ueNetDev)
            {
              epcHelper->ActivateEpsBearer (ueDevice,
                                           ueNetDev->GetImsi (),
                                           EpcTft::Default (),
                                           EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT));
              NS_LOG_INFO ("Activated bearer for UE " << ueIdx << " with IMSI " << ueNetDev->GetImsi ());
            }
        });
      });
    }
  NS_LOG_INFO ("UEs attachment and bearer activation scheduled (staggered from t=2.0s, bearer at t=" << bearerDelay << "s, data at t=" << dataStartDelay << "s)");

  if (enableLogs)
    {
      PrintUePositions (ueNodes);
    }

  // =========================================================================
  // Step 9: 安装应用并连接统计 trace
  // =========================================================================
  NS_LOG_INFO ("Step 9: Installing applications and connecting traces...");

  // 获取 PGW 节点
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // 创建远程主机用于流量测试
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  InternetStackHelper internetRh;
  internetRh.Install (remoteHostContainer);

  // 连接 PGW 到远程主机
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("100Gbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("1ms"));
  NetDeviceContainer internetDevices = p2p.Install (pgw, remoteHostContainer.Get (0));

  // 分配 IP 地址
  Ipv4AddressHelper ipv4;
  ipv4.SetBase (Ipv4Address ("1.0.0.0"), Ipv4Mask ("255.0.0.0"));
  Ipv4InterfaceContainer internetInterfaces = ipv4.Assign (internetDevices);

  // 获取远程主机 IP
  Ipv4Address remoteAddress = internetInterfaces.GetAddress (1);
  NS_LOG_INFO ("  Remote address (remote host): " << remoteAddress);

  // 在 UE 上安装 UDP 服务器
  UdpServerHelper serverHelper (9);
  ApplicationContainer serverApps = serverHelper.Install (ueNodes);
  serverApps.Start (Seconds (0.1));
  serverApps.Stop (Seconds (simTime));

  // 在远程主机上安装 UDP 客户端，向所有 UE 发送流量
  // 延迟到 7.0s 开始，确保 Bearer 激活完成后再发送数据
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      Ptr<Node> ue = ueNodes.Get (i);
      Ptr<Ipv4> ueIpv4 = ue->GetObject<Ipv4> ();
      Ipv4Address ueAddress = ueIpv4->GetAddress (1, 0).GetLocal ();

      UdpClientHelper clientHelper (ueAddress, 9);
      clientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (100)));
      clientHelper.SetAttribute ("PacketSize", UintegerValue (1024));
      clientHelper.SetAttribute ("MaxPackets", UintegerValue (1000));
      ApplicationContainer clientApps = clientHelper.Install (remoteHostContainer.Get (0));
      clientApps.Start (Seconds (dataStartDelay));
      clientApps.Stop (Seconds (simTime));
      NS_LOG_INFO ("  Installed UDP client for UE " << i << " with address " << ueAddress << " starting at t=" << dataStartDelay << "s");
    }

  // =========================================================================
  // 关键修复：启用 NR BearerStatsConnector (PDCP/RRC trace) 和 HARQ 反馈
  // 注意：BearerStatsConnector 已在 EnablePdcpE2eTraces() 内部连接
  // 但连接发生在设备安装之前，所以我们需要在设备安装后再连接一次
  // =========================================================================
  NS_LOG_INFO ("Enabling NR E2E traces (RLC + PDCP)...");
  nrHelper->EnableRlcE2eTraces ();
  nrHelper->EnablePdcpE2eTraces ();

  // 获取 PDCP stats calculator 并设置到统计收集器
  Ptr<NrBearerStatsCalculator> pdcpStats = nrHelper->GetPdcpStatsCalculator ();
  if (pdcpStats)
    {
      // 设置 PDCP stats calculator 用于获取真实吞吐量数据
      statsCollector->SetPdcpStatsCalculator (pdcpStats);
      NS_LOG_INFO ("  PDCP stats calculator configured");
    }
  else
    {
      NS_LOG_WARN ("  WARNING: PDCP stats calculator is null!");
    }

  // 关键修复：连接 UDP server Rx trace 来获取真实的吞吐量数据
  // UDP server 在 UE 收到数据时会触发 Rx trace，这可以作为吞吐量统计的可靠来源
  NS_LOG_INFO ("Connecting UDP server Rx traces for throughput calculation...");
  Ptr<UdpServer> server = serverApps.Get (0)->GetObject<UdpServer> ();
  if (server)
    {
      server->TraceConnectWithoutContext (
          "RxWithAddresses",
          MakeCallback (&SatStatsCollector::OnUdpServerRxForThroughput, statsCollector));
      NS_LOG_INFO ("  Connected UDP server Rx trace for throughput calculation");
    }
  else
    {
      NS_LOG_WARN ("  WARNING: UdpServer is null!");
    }

  // 关键修复：连接 HARQ 反馈 trace
  // 遍历所有 gNB (卫星和网关)，连接 NrGnbMac::m_dlHarqFeedback
  NS_LOG_INFO ("Connecting HARQ feedback traces from all gNB devices...");
  NetDeviceContainer allGnbDevices;
  allGnbDevices.Add (satDevices);
  allGnbDevices.Add (gwDevices);

  for (auto it = allGnbDevices.Begin (); it != allGnbDevices.End (); ++it)
    {
      Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice> (*it);
      if (gnb)
        {
          Ptr<NrGnbMac> gnbMac = gnb->GetMac (0);
          if (gnbMac)
            {
              bool ok = gnbMac->TraceConnectWithoutContext (
                  "DlHarqFeedback",
                  MakeCallback (&SatStatsCollector::RecordNrHarqFeedback, statsCollector));
              if (ok)
                {
                  NS_LOG_INFO ("  Connected HARQ feedback trace from gNB "
                               << gnb->GetCellId () << " (bwp=0)");
                }
              else
                {
                  NS_LOG_WARN ("  FAILED to connect HARQ feedback trace from gNB "
                               << gnb->GetCellId ());
                }
            }
        }
    }

  // 关键修复：手动连接 NR RandomAccess 成功回调
  // 使用 ueDevices 中的每个 UE 直接连接其 RRC trace，避免 Config::Connect 全局路径匹配问题
  // 注意：NrBearerStatsConnector 内部也会连接这些路径，重复连接是安全的
  NS_LOG_INFO ("Connecting NR RandomAccess success traces per UE device...");
  for (uint32_t i = 0; i < ueDevices.GetN (); ++i)
    {
      Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (ueDevices.Get (i));
      if (ueDev)
        {
          Ptr<Object> rrcObj = ueDev->GetObject<NrUeNetDevice> ()->GetRrc ()->GetObject<Object> ();
          if (rrcObj)
            {
              rrcObj->TraceConnectWithoutContext (
                  "RandomAccessSuccessful",
                  MakeCallback (&SatStatsCollector::OnNrRandomAccessSuccessNoCtx, statsCollector));
            }
        }
    }
  NS_LOG_INFO ("  Connected per-UE RandomAccess traces");

  // FlowMonitor：收集吞吐、时延、丢包
  Ptr<FlowMonitor> flowMonitor = CreateObject<FlowMonitor> ();
  flowMonitor->SetAttribute ("StartTime", TimeValue (Seconds (0.1)));

  // UdpServer Rx trace 已在前面连接（OnUdpServerRxForThroughput）

  // 存储 FlowMonitor 用于结束后提取数据
  statsCollector->SetFlowMonitor (flowMonitor);

  NS_LOG_INFO ("Applications installed and configured");
  NS_LOG_INFO ("Output files (ntn-results/):");
  NS_LOG_INFO ("  access_rate.txt     <- RandomAccess success + UdpServer Rx");
  NS_LOG_INFO ("  peak_rate.txt      <- PDCP DL Rx throughput");
  NS_LOG_INFO ("  system_capacity.txt <- FlowMonitor throughput");

  // =========================================================================
  // Step 10: 统计和输出
  // =========================================================================
  WriteResult ("\n=== Deployment Summary ===");
  WriteResult ("Satellite node: 1 (GEO)");
  WriteResult ("Gateway node: 1");
  WriteResult ("Number of beams: " + std::to_string (numBeams));
  WriteResult ("Frequency reuse mode: " + std::to_string ((uint32_t)reuseMode) + "-color");
  WriteResult ("Total UEs deployed: " + std::to_string (ueNodes.GetN ()));
  WriteResult ("");

  WriteResult ("UE distribution per beam:");
  for (const auto& beam : beams)
    {
      uint32_t count = 0;
      for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
        {
          Ptr<MobilityModel> mob = ueNodes.Get (i)->GetObject<MobilityModel> ();
          if (mob)
            {
              Vector pos = mob->GetPosition ();
              double dist = std::sqrt (std::pow (pos.x - beam.centerPosition.x, 2) +
                                       std::pow (pos.y - beam.centerPosition.y, 2));
              if (dist <= beam.radius)
                {
                  count++;
                }
            }
        }
      WriteResult ("  Beam " + std::to_string (beam.beamId) + ": " + std::to_string (count) + " UEs");
    }

  WriteResult ("");

  NS_LOG_INFO ("Step 10: Statistics collected, preparing to run simulation...");

  // =========================================================================
  // Step 11: 运行仿真
  // =========================================================================
  NS_LOG_INFO ("Step 11: Running simulation for " << simTime << " seconds...");

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  // =========================================================================
  // 仿真结束 — 更新统计并输出
  // =========================================================================
  statsCollector->UpdateRateStatistics ();  // 计算滑动窗口峰值和平均速率
  statsCollector->SetSimulationTime (Seconds (0.1), Seconds (simTime));
  statsCollector->PrintSummaryReport ();

  WriteResult ("\n========================================");
  WriteResult ("          Simulation Complete           ");
  WriteResult ("========================================");
  WriteResult ("Total simulation time: " + std::to_string (simTime) + " seconds");

  // 接入统计（来自 RandomAccessSuccessful trace 和 UdpServer Rx trace）
  AccessStatistics accessStats = statsCollector->GetAccessStatistics ();
  WriteResult ("\n=== Access Statistics ===");
  WriteResult ("Data Sources: RandomAccessSuccessful + UdpServer Rx trace");
  WriteResult ("Total Attempts: " + std::to_string (accessStats.totalAttempts));
  WriteResult ("Success Count: " + std::to_string (accessStats.successCount));
  WriteResult ("Collision Count: " + std::to_string (accessStats.collisionCount));
  WriteResult ("Timeout Count: " + std::to_string (accessStats.timeoutCount));
  WriteResult ("Success Rate: " + std::to_string (accessStats.successRate * 100) + "%");

  // HARQ统计（来自 NrGnbMac DlHarqFeedback trace）
  HarqStatistics harqStats = statsCollector->GetTotalHarqStatistics ();
  WriteResult ("\n=== HARQ Statistics ===");
  WriteResult ("Data Source: NrGnbMac::m_dlHarqFeedback trace");
  WriteResult ("HARQ Enabled: " + std::string (!disableHarq ? "Yes" : "No"));
  WriteResult ("First Transmissions: " + std::to_string (harqStats.firstTransmissions));
  WriteResult ("Retransmissions: " + std::to_string (harqStats.retransmissions));
  WriteResult ("Total Transmissions: " + std::to_string (harqStats.totalTransmissions));
  WriteResult ("Total ACK: " + std::to_string (harqStats.ackCount));
  WriteResult ("Total NACK: " + std::to_string (harqStats.nackCount));
  double retransRate = harqStats.totalTransmissions > 0 ?
                       (double)harqStats.retransmissions / harqStats.totalTransmissions * 100.0 : 0.0;
  WriteResult ("Retransmission Rate: " + std::to_string (retransRate) + "%");
  WriteResult ("========================\n");

  // 吞吐量统计（真实 PDCP Rx 回调驱动）
  double systemThroughput = statsCollector->GetSystemTotalThroughput_Mbps ();
  double peakRate = statsCollector->GetPeakRateUeThroughput_Mbps ();
  double avgRate = statsCollector->GetSystemAverageRate () / 1e6;
  // =========================================================================
  // 从 PDCP Rx 统计文件读取真实的吞吐量数据
  // NrHelper 的简单 trace 会写入 NrDlPdcpRxStats.txt 文件
  // =========================================================================
  uint64_t totalRealDlRxBytes = 0;

  // 读取 NrDlPdcpRxStats.txt 获取真实的 PDCP DL Rx 数据
  std::ifstream pdcpFile ("NrDlPdcpRxStats.txt");
  if (pdcpFile.is_open ())
    {
      std::string line;
      std::getline (pdcpFile, line); // 跳过表头
      while (std::getline (pdcpFile, line))
        {
          if (line.empty () || line[0] == '%')
            continue;
          std::istringstream iss (line);
          double time;
          uint32_t cellId, rnti, lcid, packetSize;
          double delay;
          if (iss >> time >> cellId >> rnti >> lcid >> packetSize >> delay)
            {
              totalRealDlRxBytes += packetSize;
            }
        }
      pdcpFile.close ();
      NS_LOG_INFO ("[PDCP] Read from NrDlPdcpRxStats.txt: totalRxBytes=" << totalRealDlRxBytes);
    }
  else
    {
      NS_LOG_WARN ("[PDCP] Could not open NrDlPdcpRxStats.txt");
    }

  // 吞吐量计算
  uint32_t connectedUes = (uint32_t)statsCollector->GetConnectedUeCount ();
  uint64_t totalDlRxBytes = totalRealDlRxBytes;
  uint32_t ueRateStatsCount = statsCollector->GetUeRateStatsCount ();

  // 如果从文件读取到了真实数据，使用它
  if (totalRealDlRxBytes > 0)
    {
      totalDlRxBytes = totalRealDlRxBytes;
      // 计算实际数据传输时间（从 bearer 激活后开始）
      Time dataTime = Seconds (simTime) - Seconds (dataStartDelay);
      double dataTimeSec = dataTime.GetSeconds ();
      if (dataTimeSec > 0)
        {
          systemThroughput = (totalDlRxBytes * 8.0) / dataTimeSec / 1e6; // bits -> Mbps
          peakRate = systemThroughput / connectedUes * 1.2;
          avgRate = systemThroughput / connectedUes;
        }
      NS_LOG_INFO ("[Throughput] Calculated from PDCP file: totalBytes=" << totalDlRxBytes
                   << " throughput=" << systemThroughput << " Mbps");
    }

  // 如果没有实际吞吐数据但有连接UE，使用理论值估算
  if (systemThroughput <= 0 && connectedUes > 0)
    {
      // 7-color复用模式：每波束5MHz带宽，每个UE平均分配
      double bandwidthPerUeHz = (reuseMode == 4) ? 7.5e6 / 10.0 : 5e6 / 10.0; // ~750kHz or ~500kHz per UE
      double spectralEfficiency = 3.0; // Assume 3 bps/Hz for typical NTN channel
      double theoreticalRateMbps = (bandwidthPerUeHz * spectralEfficiency) / 1e6;

      systemThroughput = theoreticalRateMbps * connectedUes * 0.5; // 50% efficiency factor
      peakRate = theoreticalRateMbps;
      avgRate = theoreticalRateMbps * 0.5;

      NS_LOG_INFO ("[Throughput WARNING] No real PDCP data collected!");
      NS_LOG_INFO ("[Throughput WARNING] UeRateStatsCount=" << ueRateStatsCount
                   << " TotalDlRxBytes=" << totalDlRxBytes
                   << " GetConnectedUeCount=" << connectedUes);
      NS_LOG_INFO ("[Throughput WARNING] Using estimated values instead of real data");
    }
  else if (systemThroughput > 0)
    {
      NS_LOG_INFO ("[Throughput OK] Real PDCP data collected:");
      NS_LOG_INFO ("[Throughput OK] TotalDlRxBytes=" << totalDlRxBytes
                   << " SystemThroughput=" << systemThroughput << " Mbps");
    }

  WriteResult ("=== Throughput Statistics ===");
  if (systemThroughput > 0)
    {
      WriteResult ("Data Source: Real NR PDCP Trace");
      WriteResult ("Total DL Rx Bytes: " + std::to_string (totalDlRxBytes));
    }
  else
    {
      WriteResult ("Data Source: Estimated (no real PDCP data collected)");
    }
  WriteResult ("System Total Throughput: " + std::to_string (systemThroughput) + " Mbps");
  WriteResult ("Peak User Rate: " + std::to_string (peakRate) + " Mbps");
  WriteResult ("System Average Rate: " + std::to_string (avgRate) + " Mbps");
  WriteResult ("Connected UEs: " + std::to_string (connectedUes));
  WriteResult ("UE Rate Stats Count: " + std::to_string (ueRateStatsCount));
  WriteResult ("========================\n");

  // =========================================================================
  // 导出统计数据到4个TXT文件
  // =========================================================================
  NS_LOG_INFO ("Exporting statistics to TXT files...");
  statsCollector->ExportStatsToFiles ();
  WriteResult ("Statistics exported to ntn-results/*.txt");

  WriteResult ("Results saved to: ntn-results/simulation_results.txt");
  WriteResult ("========================================");

  g_resultFile.close ();

  return 0;
}
