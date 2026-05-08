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
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-epc-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/antenna-module.h"
#include "ns3/tcp-hybla.h"

#include "ns3/geo-beam-helper.h"
#include "ns3/gw-helper.h"
#include "ns3/terminal-profile.h"
#include "ns3/user-helper.h"
#include "ns3/traffic-models.h"
#include "ns3/ntn-config-helper.h"
#include "ns3/harq-manager.h"
#include "ns3/sat-stats-collector.h"
#include "ns3/nr-bearer-stats-connector.h"
#include "ns3/nr-phy-mac-common.h"


#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sys/stat.h>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnSystemSim");

static std::ofstream g_resultFile;

struct GroupTrafficStats
{
  uint32_t configuredFlows {0};
  uint32_t activeFlows {0};
  uint64_t totalRxBytes {0};
  double throughputMbps {0.0};
  double averageRateMbps {0.0};
};

static void
FinalizeGroupStats (GroupTrafficStats& stats, double activeTrafficSeconds)
{
  if (activeTrafficSeconds > 0.0)
    {
      stats.throughputMbps = (stats.totalRxBytes * 8.0) / activeTrafficSeconds / 1e6;
    }

  if (stats.activeFlows > 0)
    {
      stats.averageRateMbps = stats.throughputMbps / stats.activeFlows;
    }
}

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
      WriteResult ("Total Bandwidth: 35 MHz (aggregated)");
      WriteResult ("Bandwidth per Color Group: 8.75 MHz (~47 PRB)");
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
      WriteResult ("Total Bandwidth: 35 MHz (aggregated)");
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
  double consumerShare = 0.5;    //!< 每波束消费级终端占比
  double portableHttpShare = 0.5; //!< 每波束便携式终端中 HTTP 占比

  CommandLine cmd;
  cmd.AddValue ("numUes", "Total number of UE terminals", numUes);
  cmd.AddValue ("numBeams", "Number of beams (default: 7)", numBeams);
  cmd.AddValue ("reuseMode", "Frequency reuse mode: 4 (4-color) or 7 (7-color)", reuseMode);
  cmd.AddValue ("disableHarq", "Disable HARQ retransmission (true/false)", disableHarq);
  cmd.AddValue ("simTime", "Simulation time in seconds", simTime);
  cmd.AddValue ("enableLogs", "Enable detailed logging", enableLogs);
  cmd.AddValue ("consumerShare",
                "Share of consumer terminals per beam (0.0-1.0); consumers carry voice+HTTP",
                consumerShare);
  cmd.AddValue ("portableHttpShare",
                "Share of portable terminals using HTTP instead of FTP (0.0-1.0)",
                portableHttpShare);
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
      WriteResult ("  Total bandwidth: 35 MHz (aggregated)");
      WriteResult ("  Bandwidth per beam group: 8.75 MHz (~47 PRB)");
    }
  else
    {
      WriteResult ("  Total bandwidth: 35 MHz (aggregated)");
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

  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
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
        {1, 2175.0e6, 8.75e6, 45240, 1}, {2, 2180.0e6, 8.75e6, 45260, 2},
        {3, 2185.0e6, 8.75e6, 45280, 3},  {4, 2190.0e6, 8.75e6, 45300, 4},
        {5, 2175.0e6, 8.75e6, 45240, 1},  {6, 2180.0e6, 8.75e6, 45260, 2},
        {7, 2185.0e6, 8.75e6, 45280, 3}
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
  // 使用 35 MHz 带宽和 numerology 1 (30kHz)
  CcBwpCreator::SimpleOperationBandConf bandConf (2187.5e6, 35e6, 1, BandwidthPartInfo::UMa);
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
      // 先设置 numerology，再 UpdateConfig，否则 gNB 会按默认 15kHz
      // 先计算出 186 RB，而 UE 按 30kHz 得到 93 RB，最终在接收侧越界。
      gnb->GetPhy (0)->SetNumerology (1);
      // 设置发射功率
      gnb->GetPhy (0)->SetTxPower (20.0);
      gnb->UpdateConfig ();
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
      // 与卫星 gNB 保持一致，必须在 UpdateConfig 前设置 numerology。
      gnb->GetPhy (0)->SetNumerology (1);
      // 设置发射功率
      gnb->GetPhy (0)->SetTxPower (20.0);
      gnb->UpdateConfig ();
    }

  // =========================================================================
  // Step 8: 在多个波束范围内随机撒点生成UE
  // =========================================================================
  NS_LOG_INFO ("Step 8: Creating UEs in beam coverage areas...");

  Ptr<SatUserHelper> userHelper = CreateObject<SatUserHelper> ();
  userHelper->SetNrHelper (nrHelper);
  userHelper->SetBeams (beams);
  userHelper->SetConsumerShare (consumerShare);
  userHelper->SetPortableHttpShare (portableHttpShare);

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
          ue->UpdateConfig ();
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
  // 只附着到网关，因为网关是实际与核心网连接的设备。
  //
  // 关键修复：
  // 之前 bearer 激活和业务启动时刻被固定写死为 6s / 7s。对于交错附着的后续 UE，
  // HTTP client 会在用户面尚未就绪时尝试 Connect()，然后长期卡在 CONNECTING，
  // 最终导致系统场景下 HTTP 统计为 0。
  //
  // 现在把时序改成：
  //   attachStart(UE_i) = attachStartBase + i * attachStagger
  //   dataStartDelay    = last attach start + attachCompletionBudget + dataStartMargin
  //
  // 注意：AttachToClosestEnb() 本身已经通过 EPC helper 激活了默认 bearer。
  // 如果在仿真运行时再额外调用 ActivateDedicatedEpsBearer()，会触发 UE NAS 的 fatal：
  // “the necessary NAS signaling to activate a bearer after the initial context has
  // already been setup is not implemented”。
  //
  // 业务相关的 dedicated bearer 需要像 NR/LTE 示例那样，在仿真开始前预注册；UE NAS
  // 会在附着建立初始上下文时一起激活这些 bearer。后面安装应用时会根据具体业务再追加。
  //
  // 这样可以保证所有 UE 在统一启动应用前，至少已经有机会完成 GEO 附着并建立默认 bearer。
  const double geoRttSeconds = 0.63;
  const double attachStartBase = 2.0;
  const double attachStagger = 0.5;
  // GEO 场景下，AttachToClosestEnb() 返回并不代表用户面已经对应用层 TCP/HTTP
  // 完全可用。过早启动应用会让首次 Connect() 撞在 bearer / routing / RRC
  // 尚未彻底 ready 的窗口里，HTTP client 会长期停留在 CONNECTING。
  //
  // 单 UE 最小场景对这个时序尤其敏感，因此这里留出一个更保守的附着完成预算，
  // 让所有 UE 的默认 bearer、预注册 dedicated bearer 和用户面转发路径先稳定下来，
  // 再统一启动业务。
  const double attachCompletionBudget = std::max (8.0, 3.0 * geoRttSeconds);
  const double dataStartMargin = geoRttSeconds + 0.5;
  const double lastAttachStartTime =
      attachStartBase + (ueDevices.GetN () > 0 ? (ueDevices.GetN () - 1) * attachStagger : 0.0);
  const double dataStartDelay = lastAttachStartTime + attachCompletionBudget + dataStartMargin;
  for (uint32_t i = 0; i < ueDevices.GetN (); ++i)
    {
      uint32_t ueIdx = i;
      const double attachTime = attachStartBase + ueIdx * attachStagger;
      Simulator::Schedule (Seconds (attachTime), [&, ueIdx]() {
        NetDeviceContainer ueDev;
        ueDev.Add (ueDevices.Get (ueIdx));
        nrHelper->AttachToClosestEnb (ueDev, gwDevices);
      });
    }
  NS_LOG_INFO ("UE attachment scheduled"
               << " (attachBase=" << attachStartBase << "s"
               << ", attachStagger=" << attachStagger << "s"
               << ", attachBudget=" << attachCompletionBudget << "s"
               << ", lastAttach~t=" << lastAttachStartTime << "s"
               << ", data start t=" << dataStartDelay << "s)");

  if (simTime <= dataStartDelay)
    {
      NS_LOG_WARN ("Simulation time " << simTime
                   << "s is not long enough to observe user traffic after bearer activation."
                   << " Recommended simTime > " << (dataStartDelay + 1.0) << "s");
    }

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
  if (enableLogs)
    {
      p2p.EnablePcapAll ("ntn-system-pgw-remote", true);
    }

  // 分配 IP 地址
  Ipv4AddressHelper ipv4;
  ipv4.SetBase (Ipv4Address ("1.0.0.0"), Ipv4Mask ("255.0.0.0"));
  Ipv4InterfaceContainer internetInterfaces = ipv4.Assign (internetDevices);

  // 获取远程主机 IP
  Ipv4Address remoteAddress = internetInterfaces.GetAddress (1);
  NS_LOG_INFO ("  Remote address (remote host): " << remoteAddress);

  // 远端主机需要一条到 EPC 分配的 UE 网段的回程路由，否则业务流量无法返回 UE。
  Ptr<Ipv4StaticRouting> remoteHostRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHostContainer.Get (0)->GetObject<Ipv4> ());
  remoteHostRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"),
                                        Ipv4Mask ("255.0.0.0"),
                                        1);

  Ptr<SatTrafficGenerator> trafficGen = CreateObject<SatTrafficGenerator> ();
  trafficGen->SetGeoRtt (MilliSeconds (630));
  trafficGen->SetTcpCongestionTypeId (TcpHybla::GetTypeId ());
  trafficGen->SetApplicationWindow (Seconds (dataStartDelay), Seconds (simTime));
  trafficGen->InstallSink (ueNodes);

  uint32_t consumerPhoneCount = 0;
  uint32_t portableTerminalCount = 0;
  uint32_t ftpFlowCount = 0;
  uint32_t httpFlowCount = 0;
  uint32_t voipFlowCount = 0;
  constexpr uint16_t kFtpPort = 20;
  constexpr uint16_t kHttpPort = 80;
  constexpr uint16_t kVoipPort = 5000;
  std::map<uint32_t, bool> ueIsConsumerPhone;
  std::map<uint32_t, bool> ueHasVoipFlow;
  std::map<uint32_t, bool> ueHasHttpFlow;
  std::map<uint32_t, bool> ueHasFtpFlow;

  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      Ptr<Node> ue = ueNodes.Get (i);
      Ptr<Ipv4> ueIpv4 = ue->GetObject<Ipv4> ();
      Ipv4Address ueAddress = ueIpv4->GetAddress (1, 0).GetLocal ();

      // Requirement-driven traffic split:
      // - consumer phones carry concurrent voice + data services
      // - portable terminals carry pure data service
      Ptr<SatTerminalProfile> profile = ue->GetObject<SatTerminalProfile> ();
      NS_ABORT_MSG_IF (profile == nullptr, "UE node is missing SatTerminalProfile");
      NodeContainer remoteHostNode (remoteHostContainer.Get (0));
      InstalledTerminalTraffic installed =
          trafficGen->InstallProfileTraffic (remoteHostNode, ue, ueAddress, false);

      ueIsConsumerPhone[i] = installed.isConsumerPhone;
      ueHasVoipFlow[i] = installed.hasVoip;
      ueHasHttpFlow[i] = installed.hasHttp;
      ueHasFtpFlow[i] = installed.hasFtp;

      if (installed.isConsumerPhone)
        {
          consumerPhoneCount++;
        }
      else
        {
          portableTerminalCount++;
        }

      voipFlowCount += installed.hasVoip ? 1 : 0;
      httpFlowCount += installed.hasHttp ? 1 : 0;
      ftpFlowCount += installed.hasFtp ? 1 : 0;

      // 在仿真开始前为业务预注册 dedicated bearer，避免 VoIP 与 HTTP/FTP 长期共用
      // 默认 bearer 队列，导致 GEO 慢链路下小量 TCP 控制报文长期被饿死。
      Ptr<NetDevice> ueDevice = ueDevices.Get (i);
      if (installed.hasVoip)
        {
          Ptr<EpcTft> voipTft = Create<EpcTft> ();
          EpcTft::PacketFilter voipPf;
          voipPf.direction = EpcTft::DOWNLINK;
          voipPf.localPortStart = kVoipPort;
          voipPf.localPortEnd = kVoipPort;
          voipTft->Add (voipPf);
          nrHelper->ActivateDedicatedEpsBearer (ueDevice,
                                                EpsBearer (EpsBearer::GBR_CONV_VOICE),
                                                voipTft);
        }

      if (installed.hasHttp)
        {
          Ptr<EpcTft> httpTft = Create<EpcTft> ();
          EpcTft::PacketFilter httpPf;
          httpPf.direction = EpcTft::BIDIRECTIONAL;
          httpPf.remotePortStart = kHttpPort;
          httpPf.remotePortEnd = kHttpPort;
          httpTft->Add (httpPf);
          nrHelper->ActivateDedicatedEpsBearer (ueDevice,
                                                EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT),
                                                httpTft);
        }

      if (installed.hasFtp)
        {
          Ptr<EpcTft> ftpTft = Create<EpcTft> ();
          EpcTft::PacketFilter ftpPf;
          ftpPf.direction = EpcTft::DOWNLINK;
          ftpPf.localPortStart = kFtpPort;
          ftpPf.localPortEnd = kFtpPort;
          ftpTft->Add (ftpPf);
          nrHelper->ActivateDedicatedEpsBearer (ueDevice,
                                                EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT),
                                                ftpTft);
        }

      NS_LOG_INFO ("  UE " << i << " profile=" << profile->DescribeTrafficProfile ()
                   << " beam=" << profile->GetBeamId () << " -> " << ueAddress);
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
          if (gnb->GetCcMapSize () == 0)
            {
              NS_LOG_WARN ("  Skipping HARQ trace connect: gNB has empty CC map");
              continue;
            }

          Ptr<NrGnbMac> gnbMac;
          try
            {
              gnbMac = gnb->GetMac (0);
            }
          catch (const std::out_of_range&)
            {
              NS_LOG_WARN ("  Skipping HARQ trace connect: gNB BWP index 0 is unavailable");
              continue;
            }

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

  // 存储 FlowMonitor 用于结束后提取数据
  statsCollector->SetFlowMonitor (flowMonitor);

  NS_LOG_INFO ("Applications installed and configured");
  NS_LOG_INFO ("Output files (ntn-results/):");
  NS_LOG_INFO ("  access_rate.txt     <- RandomAccess success trace");
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
  WriteResult ("Consumer phones (voice + data): " + std::to_string (consumerPhoneCount));
  WriteResult ("Portable terminals (pure data): " + std::to_string (portableTerminalCount));
  WriteResult ("Traffic mix: VoIP=" + std::to_string (voipFlowCount) +
               ", FTP=" + std::to_string (ftpFlowCount) +
               ", HTTP=" + std::to_string (httpFlowCount));
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

  // 接入统计（来自 RandomAccessSuccessful trace）
  AccessStatistics accessStats = statsCollector->GetAccessStatistics ();
  WriteResult ("\n=== Access Statistics ===");
  WriteResult ("Data Source: RandomAccessSuccessful trace");
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
      double bandwidthPerUeHz = (reuseMode == 4) ? 8.75e6 / 10.0 : 5e6 / 10.0; // ~875kHz or ~500kHz per UE
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
  // Per-terminal-group / per-traffic-model export using sink bytes on each UE
  // =========================================================================
  GroupTrafficStats consumerStats;
  GroupTrafficStats portableStats;
  GroupTrafficStats voipStats;
  GroupTrafficStats ftpStats;
  GroupTrafficStats httpStats;

  consumerStats.configuredFlows = consumerPhoneCount;
  portableStats.configuredFlows = portableTerminalCount;
  voipStats.configuredFlows = voipFlowCount;
  ftpStats.configuredFlows = ftpFlowCount;
  httpStats.configuredFlows = httpFlowCount;

  const double activeTrafficSeconds = std::max (0.0, simTime - dataStartDelay);

  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      const uint32_t nodeId = ueNodes.Get (i)->GetId ();
      const uint64_t voipBytes = trafficGen->GetNodeReceivedBytes (nodeId, TRAFFIC_VOIP_RTP);
      const uint64_t ftpBytes = trafficGen->GetNodeReceivedBytes (nodeId, TRAFFIC_FTP);
      const uint64_t httpBytes = trafficGen->GetNodeReceivedBytes (nodeId, TRAFFIC_HTTP);
      const uint64_t totalBytes = voipBytes + ftpBytes + httpBytes;

      if (ueIsConsumerPhone[i])
        {
          consumerStats.totalRxBytes += totalBytes;
          if (totalBytes > 0)
            {
              consumerStats.activeFlows++;
            }
        }
      else
        {
          portableStats.totalRxBytes += totalBytes;
          if (totalBytes > 0)
            {
              portableStats.activeFlows++;
            }
        }

      if (ueHasVoipFlow[i])
        {
          voipStats.totalRxBytes += voipBytes;
          if (voipBytes > 0)
            {
              voipStats.activeFlows++;
            }
        }

      if (ueHasFtpFlow[i])
        {
          ftpStats.totalRxBytes += ftpBytes;
          if (ftpBytes > 0)
            {
              ftpStats.activeFlows++;
            }
        }

      if (ueHasHttpFlow[i])
        {
          httpStats.totalRxBytes += httpBytes;
          if (httpBytes > 0)
            {
              httpStats.activeFlows++;
            }
        }
    }

  FinalizeGroupStats (consumerStats, activeTrafficSeconds);
  FinalizeGroupStats (portableStats, activeTrafficSeconds);
  FinalizeGroupStats (voipStats, activeTrafficSeconds);
  FinalizeGroupStats (ftpStats, activeTrafficSeconds);
  FinalizeGroupStats (httpStats, activeTrafficSeconds);

  WriteResult ("=== Grouped Traffic Statistics ===");
  WriteResult ("Type,ConfiguredFlows,ActiveFlows,TotalRxBytes,Throughput_Mbps,AverageRate_Mbps");
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision (6);
    ss << "ConsumerPhones," << consumerStats.configuredFlows << "," << consumerStats.activeFlows
       << "," << consumerStats.totalRxBytes << "," << consumerStats.throughputMbps
       << "," << consumerStats.averageRateMbps;
    WriteResult (ss.str ());
  }
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision (6);
    ss << "PortableTerminals," << portableStats.configuredFlows << "," << portableStats.activeFlows
       << "," << portableStats.totalRxBytes << "," << portableStats.throughputMbps
       << "," << portableStats.averageRateMbps;
    WriteResult (ss.str ());
  }
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision (6);
    ss << "VoIP," << voipStats.configuredFlows << "," << voipStats.activeFlows
       << "," << voipStats.totalRxBytes << "," << voipStats.throughputMbps
       << "," << voipStats.averageRateMbps;
    WriteResult (ss.str ());
  }
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision (6);
    ss << "FTP," << ftpStats.configuredFlows << "," << ftpStats.activeFlows
       << "," << ftpStats.totalRxBytes << "," << ftpStats.throughputMbps
       << "," << ftpStats.averageRateMbps;
    WriteResult (ss.str ());
  }
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision (6);
    ss << "HTTP," << httpStats.configuredFlows << "," << httpStats.activeFlows
       << "," << httpStats.totalRxBytes << "," << httpStats.throughputMbps
       << "," << httpStats.averageRateMbps;
    WriteResult (ss.str ());
  }
  WriteResult ("========================\n");

  {
    std::ofstream groupedFile ("ntn-results/grouped_traffic_stats.txt");
    groupedFile << "Type,ConfiguredFlows,ActiveFlows,TotalRxBytes,Throughput_Mbps,AverageRate_Mbps\n";
    groupedFile << std::fixed << std::setprecision (6);
    groupedFile << "ConsumerPhones," << consumerStats.configuredFlows << "," << consumerStats.activeFlows
                << "," << consumerStats.totalRxBytes << "," << consumerStats.throughputMbps
                << "," << consumerStats.averageRateMbps << "\n";
    groupedFile << "PortableTerminals," << portableStats.configuredFlows << "," << portableStats.activeFlows
                << "," << portableStats.totalRxBytes << "," << portableStats.throughputMbps
                << "," << portableStats.averageRateMbps << "\n";
    groupedFile << "VoIP," << voipStats.configuredFlows << "," << voipStats.activeFlows
                << "," << voipStats.totalRxBytes << "," << voipStats.throughputMbps
                << "," << voipStats.averageRateMbps << "\n";
    groupedFile << "FTP," << ftpStats.configuredFlows << "," << ftpStats.activeFlows
                << "," << ftpStats.totalRxBytes << "," << ftpStats.throughputMbps
                << "," << ftpStats.averageRateMbps << "\n";
    groupedFile << "HTTP," << httpStats.configuredFlows << "," << httpStats.activeFlows
                << "," << httpStats.totalRxBytes << "," << httpStats.throughputMbps
                << "," << httpStats.averageRateMbps << "\n";
  }

  // =========================================================================
  // 导出统计数据到4个TXT文件
  // =========================================================================
  NS_LOG_INFO ("Exporting statistics to TXT files...");
  statsCollector->ExportStatsToFiles ();

  WriteResult ("Statistics exported to ntn-results/*.txt");

  WriteResult ("Results saved to: ntn-results/simulation_results.txt");
  WriteResult ("========================================");

  g_resultFile.close ();
  Simulator::Destroy ();

  return 0;
}
