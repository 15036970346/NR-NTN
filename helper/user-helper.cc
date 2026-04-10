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
#include <vector>
#include <memory>
#include <functional>
#include <cmath>

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
    m_beamId (0)
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

NodeContainer
SatUserHelper::CreateUserNodes (uint32_t count)
{
  NS_LOG_FUNCTION (this << count);
  NodeContainer ues;
  ues.Create (count);
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
  
  CcBwpCreator::SimpleOperationBandConf bandConf (2.0e9, 20e6, numCcPerBand, BandwidthPartInfo::UMa);
  
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
  std::vector<uint32_t> uesPerBeam;

  // 按波束面积比例分配UE数量
  double totalArea = 0.0;
  for (const auto& beam : m_beams)
    {
      totalArea += M_PI * beam.radius * beam.radius;
    }

  uint32_t remainingUes = totalCount;
  NS_LOG_INFO ("Distributing " << totalCount << " UEs across " << m_beams.size () << " beams");

  for (size_t i = 0; i < m_beams.size (); ++i)
    {
      uint32_t uesForThisBeam;
      if (i == m_beams.size () - 1)
        {
          // 最后一个波束获取所有剩余UE（确保总数正确）
          uesForThisBeam = remainingUes;
        }
      else
        {
          double beamArea = M_PI * m_beams[i].radius * m_beams[i].radius;
          uesForThisBeam = static_cast<uint32_t> (totalCount * beamArea / totalArea);
          if (uesForThisBeam > remainingUes)
            {
              uesForThisBeam = remainingUes;
            }
        }

      remainingUes -= uesForThisBeam;
      uesPerBeam.push_back (uesForThisBeam);

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

} // namespace ns3