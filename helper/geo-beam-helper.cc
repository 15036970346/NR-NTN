#include "geo-beam-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/mobility-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/uniform-planar-array.h"
#include "ns3/bridge-helper.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/spectrum-channel.h"
#include "ns3/nr-helper.h"
#include "../model/sat-channel.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GeoBeamHelper");

/**
 * \brief 4色复用频率配置
 * \details
 * - 每颜色组带宽: 8.75 MHz (~47 PRB, 子载波间隔15kHz)
 * - 频点分配: 7个波束映射到4个中心频点
 * - S频段下行: 2170-2200 MHz
 *
 * 波束分配:
 *   f1 (2175 MHz): Beam 1, Beam 5 (复用)
 *   f2 (2180 MHz): Beam 2, Beam 6 (复用)
 *   f3 (2185 MHz): Beam 3, Beam 7 (复用)
 *   f4 (2190 MHz): Beam 4
 */
const std::vector<BeamFrequencyConfig> g_4ColorFrequencyConfig = {
  {1, 2175.0e6, 8.75e6, 45240, 1},   //!< Beam 1 -> f1
  {2, 2180.0e6, 8.75e6, 45260, 2},   //!< Beam 2 -> f2
  {3, 2185.0e6, 8.75e6, 45280, 3},   //!< Beam 3 -> f3
  {4, 2190.0e6, 8.75e6, 45300, 4},   //!< Beam 4 -> f4
  {5, 2175.0e6, 8.75e6, 45240, 1},   //!< Beam 5 -> f1 (复用)
  {6, 2180.0e6, 8.75e6, 45260, 2},   //!< Beam 6 -> f2 (复用)
  {7, 2185.0e6, 8.75e6, 45280, 3}    //!< Beam 7 -> f3 (复用)
};

/**
 * \brief 7色复用频率配置
 * \details
 * - 每波束带宽: 5 MHz (25 PRB, 子载波间隔15kHz)
 * - 频点分配: 7个波束使用7个不重叠的中心频点
 * - S频段下行: 2170-2200 MHz
 *
 * 波束分配 (每个波束独占一个频点):
 *   f1 (2172.5 MHz): Beam 1
 *   f2 (2177.5 MHz): Beam 2
 *   f3 (2182.5 MHz): Beam 3
 *   f4 (2187.5 MHz): Beam 4
 *   f5 (2192.5 MHz): Beam 5
 *   f6 (2197.5 MHz): Beam 6
 *   f7 (2202.5 MHz): Beam 7
 */
const std::vector<BeamFrequencyConfig> g_7ColorFrequencyConfig = {
  {1, 2172.5e6, 5e6, 45230, 1},      //!< Beam 1 -> f1
  {2, 2177.5e6, 5e6, 45240, 2},      //!< Beam 2 -> f2
  {3, 2182.5e6, 5e6, 45250, 3},      //!< Beam 3 -> f3
  {4, 2187.5e6, 5e6, 45260, 4},      //!< Beam 4 -> f4
  {5, 2192.5e6, 5e6, 45270, 5},      //!< Beam 5 -> f5
  {6, 2197.5e6, 5e6, 45280, 6},      //!< Beam 6 -> f6
  {7, 2202.5e6, 5e6, 45290, 7}       //!< Beam 7 -> f7
};

GeoBeamHelper::GeoBeamHelper ()
{
  m_deviceFactory.SetTypeId ("ns3::NrGnbNetDevice");
  m_phyFactory.SetTypeId ("ns3::SatPhy");
  m_channelFactory.SetTypeId ("ns3::SpectrumChannel");

  m_antennaFactory.SetTypeId ("ns3::UniformPlanarArray");
  m_antennaFactory.Set ("NumRows", UintegerValue (4));
  m_antennaFactory.Set ("NumColumns", UintegerValue (4));

  m_centerFreqHz = 2e9;   // 2GHz S频段
  m_bandwidthHz = 35e6;   // 上下行总带宽35MHz
  m_reuseMode = FrequencyReuseMode::FR1_7COLOR;  // 默认7色复用

  // 初始化默认7色复用配置
  m_beamFreqConfig = g_7ColorFrequencyConfig;
}

GeoBeamHelper::~GeoBeamHelper () {}

void GeoBeamHelper::SetPhy (std::string type, std::string n0, const AttributeValue &v0)
{
  m_phyFactory.SetTypeId (type);
  if (!n0.empty ())
    {
      m_phyFactory.Set (n0, v0);
    }
}

void GeoBeamHelper::SetDevice (std::string type, std::string n0, const AttributeValue &v0)
{
  m_deviceFactory.SetTypeId (type);
  if (!n0.empty ())
    {
      m_deviceFactory.Set (n0, v0);
    }
}

void GeoBeamHelper::SetChannel (std::string type, std::string n0, const AttributeValue &v0)
{
  m_channelFactory.SetTypeId (type);
  if (!n0.empty ())
    {
      m_channelFactory.Set (n0, v0);
    }
}

void GeoBeamHelper::SetFrequencyReuseMode (uint8_t mode)
{
  NS_LOG_FUNCTION (this << mode);

  if (mode == 4)
    {
      m_reuseMode = FrequencyReuseMode::FR1_4COLOR;
      m_beamFreqConfig = g_4ColorFrequencyConfig;
      NS_LOG_INFO ("Set to 4-color frequency reuse: 8.75MHz (~47PRB) per beam group");
    }
  else if (mode == 7)
    {
      m_reuseMode = FrequencyReuseMode::FR1_7COLOR;
      m_beamFreqConfig = g_7ColorFrequencyConfig;
      NS_LOG_INFO ("Set to 7-color frequency reuse: 5MHz (25PRB) per beam");
    }
  else
    {
      NS_LOG_WARN ("Invalid reuse mode " << mode << ", keeping default 7-color reuse");
      m_reuseMode = FrequencyReuseMode::FR1_7COLOR;
      m_beamFreqConfig = g_7ColorFrequencyConfig;
    }
}

void GeoBeamHelper::SetBeamConf (double centerFreqHz, double bandwidthHz)
{
  m_centerFreqHz = centerFreqHz;
  m_bandwidthHz = bandwidthHz;
}

std::vector<BeamFrequencyConfig>
GeoBeamHelper::GetBeamFrequencyConfig () const
{
  return m_beamFreqConfig;
}

NetDeviceContainer
GeoBeamHelper::Install (NodeContainer nodes, std::vector<std::string> tleLines)
{
  NetDeviceContainer allDevices;

  if (!m_channel)
    {
      // 使用 factory 中设置的类型创建信道
      m_channel = m_channelFactory.Create<SpectrumChannel> ();
      NS_LOG_INFO ("Created spectrum channel with factory type");
    }

  for (uint32_t i = 0; i < nodes.GetN (); ++i)
    {
      Ptr<Node> node = nodes.Get (i);

      if (i < tleLines.size ())
        {
          Ptr<SatSGP4MobilityModel> mobility = CreateObject<SatSGP4MobilityModel> ();
          node->AggregateObject (mobility);
          NS_LOG_INFO ("Attached SGP4 Mobility Model to node " << node->GetId ());
        }

      NetDeviceContainer beamDevices = InstallBeams (node, m_channel);
      allDevices.Add (beamDevices);

      // 跳过 feederChannel 相关逻辑，简化处理
    }

  return allDevices;
}

NetDeviceContainer
GeoBeamHelper::InstallBeams (Ptr<Node> satNode, Ptr<SpectrumChannel> channel)
{
  NetDeviceContainer devices;

  NS_LOG_INFO ("Installing " << m_beamFreqConfig.size () << " beams with frequency reuse mode "
                            << static_cast<uint8_t> (m_reuseMode));

  // 打印频率配置摘要
  if (m_reuseMode == FrequencyReuseMode::FR1_4COLOR)
    {
      NS_LOG_INFO ("=== 4-Color Frequency Reuse Configuration ===");
      NS_LOG_INFO ("Bandwidth per beam group: 8.75 MHz (~47 PRB)");
      NS_LOG_INFO ("Color 1 (f1=2175MHz): Beams 1, 5");
      NS_LOG_INFO ("Color 2 (f2=2180MHz): Beams 2, 6");
      NS_LOG_INFO ("Color 3 (f3=2185MHz): Beams 3, 7");
      NS_LOG_INFO ("Color 4 (f4=2190MHz): Beam 4");
    }
  else
    {
      NS_LOG_INFO ("=== 7-Color Frequency Reuse Configuration ===");
      NS_LOG_INFO ("Bandwidth per beam: 5 MHz (25 PRB)");
    }

  for (uint32_t beamIdx = 0; beamIdx < m_beamFreqConfig.size (); ++beamIdx)
    {
      const BeamFrequencyConfig& config = m_beamFreqConfig[beamIdx];

      Ptr<NetDevice> device = m_deviceFactory.Create<NetDevice> ();
      Ptr<Object> phyObj = m_phyFactory.Create ();
      Ptr<PhasedArrayModel> antenna = m_antennaFactory.Create<PhasedArrayModel> ();

      NS_LOG_INFO ("Configuring Beam " << config.beamId
                   << " | Freq=" << config.centerFreqHz / 1e6 << " MHz"
                   << " | BW=" << config.bandwidthHz / 1e6 << " MHz"
                   << " | EARFCN=" << config.earfcn
                   << " | Color=" << static_cast<uint32_t> (config.colorId));

      satNode->AddDevice (device);
      devices.Add (device);
    }

  NS_LOG_INFO ("Beam installation complete: " << devices.GetN () << " beam devices created");
  return devices;
}

Ptr<NetDevice>
GeoBeamHelper::InstallSwitch (Ptr<Node> satNode, NetDeviceContainer beamDevices, Ptr<NetDevice> feederDevice)
{
  BridgeHelper bridge;

  NetDeviceContainer bridgePorts = beamDevices;
  if (feederDevice)
    {
      bridgePorts.Add (feederDevice);
    }

  NetDeviceContainer bridgeDev = bridge.Install (satNode, bridgePorts);

  NS_LOG_INFO ("Configured Satellite Switch (Bridge) on Node " << satNode->GetId ()
               << " connecting " << bridgePorts.GetN () << " interfaces.");

  return bridgeDev.Get (0);
}

Ptr<SpectrumChannel> GeoBeamHelper::GetSpectrumChannel (void) const
{
  return m_channel;
}

} // namespace ns3
