/*
 * user-helper.cc
 * Implementation of UserHelper.
 * Fixed: Complete InstallStack using NrHelper->InstallUeDevice with proper BWP setup
 * Enhanced: Added multi-beam random UE deployment support
 */

#include "user-helper.h"
#include "ns3/log.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/nr-mac-scheduler-ue-info.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/epc-helper.h"
#include "ns3/boolean.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-address-helper.h"
#include "terminal-profile.h"
#include <vector>
#include <memory>
#include <functional>
#include <cmath>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatUserHelper");

NS_OBJECT_ENSURE_REGISTERED (SatUserHelper);

TypeId
SatUserHelper::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatUserHelper")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatUserHelper> ();
  return tid;
}

SatUserHelper::SatUserHelper ()
  : m_nrHelper (nullptr),
    m_satChannel (nullptr),
    m_beamId (0),
    m_consumerShare (0.5),
    m_portableHttpShare (0.5)
{
  NS_LOG_FUNCTION (this);
}

SatUserHelper::~SatUserHelper ()
{
  NS_LOG_FUNCTION (this);
}

void
SatUserHelper::SetBeamId (uint16_t beamId)
{
  NS_LOG_FUNCTION (this << beamId);
  m_beamId = beamId;
}

void
SatUserHelper::SetSatelliteChannel (Ptr<SpectrumChannel> channel)
{
  NS_LOG_FUNCTION (this);
  m_satChannel = channel;
}

void
SatUserHelper::SetNrHelper (Ptr<NrHelper> nrHelper)
{
  NS_LOG_FUNCTION (this);
  m_nrHelper = nrHelper;
}

void
SatUserHelper::SetConsumerShare (double consumerShare)
{
  NS_LOG_FUNCTION (this << consumerShare);
  m_consumerShare = std::max (0.0, std::min (1.0, consumerShare));
}

void
SatUserHelper::SetPortableHttpShare (double portableHttpShare)
{
  NS_LOG_FUNCTION (this << portableHttpShare);
  m_portableHttpShare = std::max (0.0, std::min (1.0, portableHttpShare));
}

NodeContainer
SatUserHelper::CreateUserNodes (uint32_t count)
{
  NS_LOG_FUNCTION (this << count);
  NodeContainer ues;
  ues.Create (count);
  AssignTerminalProfiles (ues, m_beamId);
  NS_LOG_INFO ("Created " << count << " UE nodes.");
  return ues;
}

void
SatUserHelper::InstallMobility (NodeContainer ues, std::string positionAllocatorType, std::string paramName, std::string paramValue)
{
  NS_LOG_FUNCTION (this);
  
  m_mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  if (!positionAllocatorType.empty ())
    {
      m_mobility.SetPositionAllocator (positionAllocatorType,
                                       paramName, StringValue (paramValue));
    }
  else
    {
      m_mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                       "MinX", DoubleValue (0.0),
                                       "MinY", DoubleValue (0.0),
                                       "DeltaX", DoubleValue (10.0),
                                       "DeltaY", DoubleValue (10.0),
                                       "GridWidth", UintegerValue (10),
                                       "LayoutType", StringValue ("RowFirst"));
    }

  m_mobility.Install (ues);
  NS_LOG_INFO ("Mobility installed for UEs.");
}

NetDeviceContainer
SatUserHelper::InstallStack (NodeContainer ues)
{
  NS_LOG_FUNCTION (this);
  
  if (!m_nrHelper)
    {
      NS_LOG_ERROR ("NrHelper not set! Returning empty device container.");
      NetDeviceContainer emptyUeDevices; 
      return emptyUeDevices;
    }

  InternetStackHelper internet;
  internet.Install (ues);

  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;
  
  // 统一频谱口径：7个波束，每波束5MHz，总名义带宽35MHz
  CcBwpCreator::SimpleOperationBandConf bandConf (2187.5e6, 35e6, numCcPerBand, BandwidthPartInfo::UMa);
  
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);
  
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
  m_nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
  m_nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
  
  uint8_t numberOfPanels = 1;
  uint8_t flags = 0x07; // INIT_PROPAGATION | INIT_FADING | INIT_CHANNEL
  m_nrHelper->InitializeOperationBand (&band, flags);
  
  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps ({std::ref (band)});
  
  NS_LOG_INFO ("Installing UE devices with BWP: Freq=" << band.m_centralFrequency/1e9 
               << "GHz, BW=" << band.m_channelBandwidth/1e6 << "MHz");
  
  NetDeviceContainer ueDevices = m_nrHelper->InstallUeDevice (ues, allBwps, numberOfPanels);
  
  for (uint32_t i = 0; i < ueDevices.GetN (); ++i)
    {
      Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (ueDevices.Get (i));
      if (ueDev)
        {
          ueDev->GetPhy (0)->SetNumerology (0); // numerology 0 = 15kHz SCS
        }
    }
  
  NS_LOG_INFO ("Installed " << ueDevices.GetN () << " UE NetDevices");
  
  return ueDevices;
}

Ipv4InterfaceContainer
SatUserHelper::AssignIp (NetDeviceContainer ueDevices, Ipv4Address baseAddress, Ipv4Mask mask)
{
  NS_LOG_FUNCTION (this);
  m_ipv4.SetBase (baseAddress, mask);
  return m_ipv4.Assign (ueDevices);
}

void
SatUserHelper::InstallApplication (NodeContainer ues, Ipv4Address destAddress)
{
  NS_LOG_FUNCTION (this);

  uint16_t port = 9;
  UdpClientHelper client (destAddress, port);
  client.SetAttribute ("MaxPackets", UintegerValue (10000));
  client.SetAttribute ("Interval", TimeValue (MilliSeconds (10)));
  client.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer apps = client.Install (ues);
  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (10.0));

  NS_LOG_INFO ("Installed UDP Traffic Application on UEs.");
}

void
SatUserHelper::SetBeams (const std::vector<BeamInfo>& beams)
{
  NS_LOG_FUNCTION (this << beams.size ());
  m_beams = beams;
  NS_LOG_INFO ("Set " << m_beams.size () << " beams for UE deployment.");
}

NodeContainer
SatUserHelper::CreateUsersInMultipleBeams (uint32_t totalCount)
{
  NS_LOG_FUNCTION (this << totalCount);

  if (m_beams.empty ())
    {
      NS_LOG_ERROR ("No beams configured! Please call SetBeams() first.");
      return NodeContainer ();
    }

  NodeContainer allUes;
  std::vector<uint32_t> uesPerBeam (m_beams.size (), 0);
  std::vector<std::pair<double, size_t>> beamRemainders;
  beamRemainders.reserve (m_beams.size ());

  // 按波束面积比例分配UE数量
  double totalArea = 0.0;
  for (const auto& beam : m_beams)
    {
      totalArea += M_PI * beam.radius * beam.radius;
    }

  NS_LOG_INFO ("Distributing " << totalCount << " UEs across " << m_beams.size () << " beams");

  for (size_t i = 0; i < m_beams.size (); ++i)
    {
      double beamArea = M_PI * m_beams[i].radius * m_beams[i].radius;
      double exactShare = totalArea > 0.0 ? (static_cast<double> (totalCount) * beamArea / totalArea)
                                          : 0.0;
      uint32_t floorShare = static_cast<uint32_t> (std::floor (exactShare));
      uesPerBeam[i] = floorShare;
      beamRemainders.emplace_back (exactShare - floorShare, i);
    }

  uint32_t assignedUes = 0;
  for (uint32_t count : uesPerBeam)
    {
      assignedUes += count;
    }

  uint32_t remainingUes = totalCount > assignedUes ? (totalCount - assignedUes) : 0;
  std::stable_sort (beamRemainders.begin (),
                    beamRemainders.end (),
                    [] (const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });

  for (uint32_t extra = 0; extra < remainingUes && extra < beamRemainders.size (); ++extra)
    {
      uesPerBeam[beamRemainders[extra].second]++;
    }

  for (size_t i = 0; i < m_beams.size (); ++i)
    {
      uint32_t uesForThisBeam = uesPerBeam[i];

      NS_LOG_INFO ("Beam " << m_beams[i].beamId << ": " << uesForThisBeam << " UEs "
                          << "(center: " << m_beams[i].centerPosition.x << ", "
                          << m_beams[i].centerPosition.y << ", "
                          << m_beams[i].centerPosition.z << "m, "
                          << "radius: " << m_beams[i].radius << "m)");

      // 在当前波束内创建UE
      NodeContainer beamUes = CreateUsersAroundBeam (m_beams[i], uesForThisBeam);
      allUes.Add (beamUes);
    }

  NS_LOG_INFO ("Created total " << allUes.GetN () << " UE nodes across all beams");
  return allUes;
}

NodeContainer
SatUserHelper::CreateUsersAroundBeam (const BeamInfo& beamInfo, uint32_t count)
{
  NS_LOG_FUNCTION (this << beamInfo.beamId << count);

  NodeContainer ues;
  if (count == 0)
    {
      return ues;
    }

  ues.Create (count);
  AssignTerminalProfiles (ues, beamInfo.beamId);

  // 创建随机圆盘位置分配器，在波束中心附近撒点
  Ptr<RandomDiscPositionAllocator> randomDisc = CreateObject<RandomDiscPositionAllocator> ();

  // 设置圆盘中心为波束中心 (x, y)
  randomDisc->SetX (beamInfo.centerPosition.x);
  randomDisc->SetY (beamInfo.centerPosition.y);

  // 设置圆盘半径为波束半径 (使用均匀随机变量)
  Ptr<UniformRandomVariable> randRho = CreateObject<UniformRandomVariable> ();
  randRho->SetAttribute ("Min", DoubleValue (0.0));
  randRho->SetAttribute ("Max", DoubleValue (beamInfo.radius));
  randomDisc->SetRho (randRho);

  // 创建随机数生成器
  Ptr<UniformRandomVariable> randX = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> randY = CreateObject<UniformRandomVariable> ();
  randX->SetAttribute ("Min", DoubleValue (beamInfo.centerPosition.x - beamInfo.radius));
  randX->SetAttribute ("Max", DoubleValue (beamInfo.centerPosition.x + beamInfo.radius));
  randY->SetAttribute ("Min", DoubleValue (beamInfo.centerPosition.y - beamInfo.radius));
  randY->SetAttribute ("Max", DoubleValue (beamInfo.centerPosition.y + beamInfo.radius));

  // 设置移动模型为常数位置模型
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // 使用随机disc分配器
  mobility.SetPositionAllocator (randomDisc);
  mobility.Install (ues);

  NS_LOG_INFO ("Created " << count << " UEs around beam " << beamInfo.beamId
                          << " at center (" << beamInfo.centerPosition.x << ", "
                          << beamInfo.centerPosition.y << ") with radius " << beamInfo.radius << "m");

  return ues;
}

void
SatUserHelper::AssignTerminalProfiles (NodeContainer ues, uint16_t beamId)
{
  const uint32_t total = ues.GetN ();
  const uint32_t consumerCount = static_cast<uint32_t> (std::round (total * m_consumerShare));
  const uint32_t portableCount = total - consumerCount;
  const uint32_t portableHttpCount =
      static_cast<uint32_t> (std::round (portableCount * m_portableHttpShare));
  std::vector<uint32_t> indices;
  indices.reserve (total);
  for (uint32_t i = 0; i < total; ++i)
    {
      indices.push_back (i);
    }

  // Randomize terminal-type assignment so it is not coupled to node creation
  // order within a beam or a single-batch UE creation call.
  Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable> ();
  for (uint32_t i = total; i > 1; --i)
    {
      uint32_t swapWith = rng->GetInteger (0, i - 1);
      std::swap (indices[i - 1], indices[swapWith]);
    }

  for (uint32_t i = 0; i < total; ++i)
    {
      Ptr<Node> node = ues.Get (indices[i]);
      Ptr<SatTerminalProfile> profile = node->GetObject<SatTerminalProfile> ();
      if (!profile)
        {
          profile = CreateObject<SatTerminalProfile> ();
          node->AggregateObject (profile);
        }

      profile->SetBeamId (beamId);
      if (i < consumerCount)
        {
          profile->SetTerminalType (UT_CONSUMER);
          profile->SetVoiceEnabled (true);
          profile->SetDataServiceType (SAT_DATA_HTTP);
        }
      else
        {
          const uint32_t portableIndex = i - consumerCount;
          profile->SetTerminalType (UT_PORTABLE);
          profile->SetVoiceEnabled (false);
          profile->SetDataServiceType (portableIndex < portableHttpCount ? SAT_DATA_HTTP
                                                                         : SAT_DATA_FTP);
        }
    }
}

} // namespace ns3
