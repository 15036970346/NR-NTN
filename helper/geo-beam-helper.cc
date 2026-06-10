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
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/spectrum-value.h"
#include "ns3/spectrum-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/nr-helper.h"
#include "../model/sat-channel.h"
#include "../model/sat-phy.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GeoBeamHelper");

// 4色复用频率配置: 7个波束映射到4个中心频点 (S 频段下行 2170-2200 MHz)
const std::vector<BeamFrequencyConfig> g_4ColorFrequencyConfig = {
  {1, 2172.5e6, 5e6, 45230, 1},
  {2, 2177.5e6, 5e6, 45240, 2},
  {3, 2182.5e6, 5e6, 45250, 3},
  {4, 2187.5e6, 5e6, 45260, 4},
  {5, 2172.5e6, 5e6, 45230, 1},
  {6, 2177.5e6, 5e6, 45240, 2},
  {7, 2182.5e6, 5e6, 45250, 3}
};

// 7色复用频率配置: 7波束独立频点
const std::vector<BeamFrequencyConfig> g_7ColorFrequencyConfig = {
  {1, 2172.5e6, 5e6, 45230, 1},
  {2, 2177.5e6, 5e6, 45240, 2},
  {3, 2182.5e6, 5e6, 45250, 3},
  {4, 2187.5e6, 5e6, 45260, 4},
  {5, 2192.5e6, 5e6, 45270, 5},
  {6, 2197.5e6, 5e6, 45280, 6},
  {7, 2202.5e6, 5e6, 45290, 7}
};

GeoBeamHelper::GeoBeamHelper ()
  : m_centerFreqHz (2e9),
    m_bandwidthHz (35e6),
    m_feederCenterFreqHz (30e9),
    m_feederBandwidthHz (100e6),
    m_activeBeamCount (1),   // 默认 1 束; 多束时每束独立 user channel, feederPhy 用 NtnBeamIdTag 路由
    m_reuseMode (FrequencyReuseMode::FR1_7COLOR),
    m_beamFreqConfig (g_7ColorFrequencyConfig)
{
  m_deviceFactory.SetTypeId ("ns3::NrGnbNetDevice");
  m_phyFactory.SetTypeId ("ns3::SatPhy");
  m_channelFactory.SetTypeId ("ns3::SingleModelSpectrumChannel");

  m_antennaFactory.SetTypeId ("ns3::UniformPlanarArray");
  m_antennaFactory.Set ("NumRows", UintegerValue (4));
  m_antennaFactory.Set ("NumColumns", UintegerValue (4));
}

GeoBeamHelper::~GeoBeamHelper () {}

void GeoBeamHelper::SetPhy (std::string type, std::string n0, const AttributeValue &v0)
{
  m_phyFactory.SetTypeId (type);
  if (!n0.empty ())
    m_phyFactory.Set (n0, v0);
}

void GeoBeamHelper::SetDevice (std::string type, std::string n0, const AttributeValue &v0)
{
  m_deviceFactory.SetTypeId (type);
  if (!n0.empty ())
    m_deviceFactory.Set (n0, v0);
}

void GeoBeamHelper::SetChannel (std::string type, std::string n0, const AttributeValue &v0)
{
  m_channelFactory.SetTypeId (type);
  if (!n0.empty ())
    m_channelFactory.Set (n0, v0);
}

void GeoBeamHelper::SetFrequencyReuseMode (uint8_t mode)
{
  NS_LOG_FUNCTION (this << mode);
  if (mode == 4)
    {
      m_reuseMode = FrequencyReuseMode::FR1_4COLOR;
      m_beamFreqConfig = g_4ColorFrequencyConfig;
      NS_LOG_INFO ("Set to 4-color frequency reuse: 5MHz per color, 35MHz total nominal BW");
    }
  else if (mode == 7)
    {
      m_reuseMode = FrequencyReuseMode::FR1_7COLOR;
      m_beamFreqConfig = g_7ColorFrequencyConfig;
      NS_LOG_INFO ("Set to 7-color frequency reuse: 5MHz (25PRB) per beam");
    }
  else
    {
      NS_LOG_WARN ("Invalid reuse mode " << mode << ", keeping 7-color");
      m_reuseMode = FrequencyReuseMode::FR1_7COLOR;
      m_beamFreqConfig = g_7ColorFrequencyConfig;
    }
  // 重置已构造的 FreqReuse, Install 时按新模式重新注册
  m_freqReuse = nullptr;
}

Ptr<FrequencyReuse> GeoBeamHelper::GetFrequencyReuse () const { return m_freqReuse; }

void   GeoBeamHelper::SetMainLobeGainDbi (double v) { m_mainLobeGainDbi = v; }
double GeoBeamHelper::GetMainLobeGainDbi () const   { return m_mainLobeGainDbi; }
void   GeoBeamHelper::SetSideLobeGainDbi (double v) { m_sideLobeGainDbi = v; }
double GeoBeamHelper::GetSideLobeGainDbi () const   { return m_sideLobeGainDbi; }
double GeoBeamHelper::GetCoBeamCIDb       () const   { return m_mainLobeGainDbi - m_sideLobeGainDbi; }

void GeoBeamHelper::SetBeamConf (double centerFreqHz, double bandwidthHz)
{
  m_centerFreqHz = centerFreqHz;
  m_bandwidthHz = bandwidthHz;
}

void GeoBeamHelper::SetFeederConf (double centerFreqHz, double bandwidthHz)
{
  m_feederCenterFreqHz = centerFreqHz;
  m_feederBandwidthHz = bandwidthHz;
}

void GeoBeamHelper::SetActiveBeamCount (uint32_t n)
{
  if (n == 0) n = 1;
  m_activeBeamCount = n;
}

std::vector<BeamFrequencyConfig>
GeoBeamHelper::GetBeamFrequencyConfig () const
{
  return m_beamFreqConfig;
}

Ptr<SpectrumChannel>
GeoBeamHelper::GetSpectrumChannel (void) const
{
  // 兼容旧 API: 返回第一个被建好的 user channel (map 按 beamId 升序, 取第一个)
  return m_userChannels.empty () ? nullptr : m_userChannels.begin ()->second;
}
Ptr<SpectrumChannel> GeoBeamHelper::GetFeederChannel  (void) const { return m_feederChannel; }
Ptr<const SpectrumModel>
GeoBeamHelper::GetUserSpectrumModel (void) const
{
  return m_userSpectrumModels.empty () ? nullptr : m_userSpectrumModels.begin ()->second;
}
Ptr<const SpectrumModel> GeoBeamHelper::GetFeederSpectrumModel (void) const { return m_feederSpectrumModel; }

Ptr<SpectrumChannel>
GeoBeamHelper::GetUserChannel (uint32_t beamId) const
{
  auto it = m_userChannels.find (beamId);
  return (it != m_userChannels.end ()) ? it->second : nullptr;
}

Ptr<const SpectrumModel>
GeoBeamHelper::GetUserSpectrumModel (uint32_t beamId) const
{
  auto it = m_userSpectrumModels.find (beamId);
  return (it != m_userSpectrumModels.end ()) ? it->second : nullptr;
}

Ptr<SatGeoUserPhy>
GeoBeamHelper::GetUserPhy (uint32_t beamId) const
{
  auto it = m_userPhys.find (beamId);
  return (it != m_userPhys.end ()) ? it->second : nullptr;
}

Ptr<SatGeoFeederPhy>
GeoBeamHelper::GetFeederPhy (uint32_t beamId) const
{
  auto it = m_feederPhys.find (beamId);
  return (it != m_feederPhys.end ()) ? it->second : nullptr;
}

Ptr<const SpectrumModel>
GeoBeamHelper::BuildBeamUserSpectrumModel (const BeamFrequencyConfig& cfg) const
{
  Bands bands;
  BandInfo b;
  b.fl = cfg.centerFreqHz - cfg.bandwidthHz / 2.0;
  b.fc = cfg.centerFreqHz;
  b.fh = cfg.centerFreqHz + cfg.bandwidthHz / 2.0;
  bands.push_back (b);
  return Create<SpectrumModel> (bands);
}

Ptr<const SpectrumModel>
GeoBeamHelper::BuildFeederSpectrumModel () const
{
  Bands bands;
  BandInfo b;
  b.fl = m_feederCenterFreqHz - m_feederBandwidthHz / 2.0;
  b.fc = m_feederCenterFreqHz;
  b.fh = m_feederCenterFreqHz + m_feederBandwidthHz / 2.0;
  bands.push_back (b);
  return Create<SpectrumModel> (bands);
}

NetDeviceContainer
GeoBeamHelper::Install (NodeContainer nodes, std::vector<std::string> tleLines)
{
  NetDeviceContainer allDevices;

  // 馈电信道仍共享 (GW <-> 卫星各束 feederPhy)
  if (!m_feederChannel)
    {
      m_feederChannel = m_channelFactory.Create<SpectrumChannel> ();
      Ptr<ConstantSpeedPropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
      m_feederChannel->SetPropagationDelayModel (delay);
      NS_LOG_INFO ("Created shared feeder spectrum channel");
    }
  if (!m_feederSpectrumModel) m_feederSpectrumModel = BuildFeederSpectrumModel ();

  // 频率复用对象: 按当前 reuse 模式给所有波束登记 colorId, 供 SatUtPhy 的 C/I_co 解析模型用
  if (!m_freqReuse)
    {
      m_freqReuse = CreateObject<FrequencyReuse> ();
      // 每束 5 MHz, 7 束 -> 35 MHz / 175 RBs
      const uint8_t reuseFactor = static_cast<uint8_t> (m_reuseMode);
      m_freqReuse->Configure (35e6, 175, reuseFactor);
      for (const auto& cfg : m_beamFreqConfig)
        {
          m_freqReuse->AddBeam (cfg.beamId, cfg.colorId);
        }
    }

  // user 链路: 每束一条独立 SpectrumChannel + SpectrumModel
  // (per-beam 独立 channel 是多波束方案的物理隔离基础: 不同 beam 的 user PHY 不会
  // 在 channel 层串扰, 也避免共享 user channel 时多 satellite userPhy 间的二次转发回路)
  const uint32_t nBeams = std::min<uint32_t> (m_activeBeamCount, m_beamFreqConfig.size ());
  for (uint32_t beamIdx = 0; beamIdx < nBeams; ++beamIdx)
    {
      const BeamFrequencyConfig& cfg = m_beamFreqConfig[beamIdx];
      const uint32_t beamId = cfg.beamId;
      if (m_userChannels.find (beamId) == m_userChannels.end ())
        {
          Ptr<SpectrumChannel> uch = m_channelFactory.Create<SpectrumChannel> ();
          Ptr<ConstantSpeedPropagationDelayModel> d = CreateObject<ConstantSpeedPropagationDelayModel> ();
          uch->SetPropagationDelayModel (d);
          m_userChannels[beamId]       = uch;
          m_userSpectrumModels[beamId] = BuildBeamUserSpectrumModel (cfg);
          NS_LOG_INFO ("Created user channel for beam " << beamId
                       << " (fc=" << cfg.centerFreqHz / 1e6 << " MHz)");
        }
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

      NetDeviceContainer beamDevices = InstallBeams (node);
      allDevices.Add (beamDevices);
    }

  return allDevices;
}

NetDeviceContainer
GeoBeamHelper::InstallBeams (Ptr<Node> satNode)
{
  NetDeviceContainer devices;

  const uint32_t nBeams = std::min<uint32_t> (m_activeBeamCount, m_beamFreqConfig.size ());
  NS_LOG_INFO ("Installing " << nBeams << " active beam(s) (reuse cfg has "
               << m_beamFreqConfig.size () << " entries, reuse mode "
               << static_cast<uint32_t> (m_reuseMode) << ")");

  Ptr<MobilityModel> satMobility = satNode->GetObject<MobilityModel> ();
  if (!satMobility)
    {
      NS_LOG_WARN ("Satellite node has no MobilityModel; SatPhy paths still wired but distances undefined");
    }

  for (uint32_t beamIdx = 0; beamIdx < nBeams; ++beamIdx)
    {
      const BeamFrequencyConfig& cfg = m_beamFreqConfig[beamIdx];

      // 1) 为每束创建一对透明弯管 PHY (Feeder 在共享馈电信道 / User 在本束独立用户信道)
      Ptr<SatGeoFeederPhy> feederPhy = CreateObject<SatGeoFeederPhy> ();
      Ptr<SatGeoUserPhy>   userPhy   = CreateObject<SatGeoUserPhy> ();

      feederPhy->SetChannel (m_feederChannel);
      feederPhy->SetMobility (satMobility);
      feederPhy->SetRxSpectrumModel (m_feederSpectrumModel);
      feederPhy->SetBeamId (static_cast<int> (cfg.beamId));
      m_feederChannel->AddRx (feederPhy);

      Ptr<SpectrumChannel>     userChannel = m_userChannels[cfg.beamId];
      Ptr<const SpectrumModel> userModel   = m_userSpectrumModels[cfg.beamId];
      userPhy->SetChannel (userChannel);
      userPhy->SetMobility (satMobility);
      userPhy->SetRxSpectrumModel (userModel);
      userPhy->SetBeamId (static_cast<int> (cfg.beamId));
      userChannel->AddRx (userPhy);

      // 2) 互为 peer, 形成透明弯管
      feederPhy->SetPeer (userPhy);
      userPhy->SetPeer (feederPhy);

      // 3) 索引保存供上层访问
      m_feederPhys[cfg.beamId] = feederPhy;
      m_userPhys[cfg.beamId]   = userPhy;

      // 4) 不能 AggregateObject (同一 TypeId 只能聚合一次, 这里有 7 束),
      //    helper 自己的 m_feederPhys/m_userPhys map 已持强引用, 寿命由 helper 保证。
      //    也不再创建空壳 NrGnbNetDevice (Simulator::Run 会因子模块缺失崩)。

      NS_LOG_INFO ("Beam " << cfg.beamId
                   << " | userFreq=" << cfg.centerFreqHz / 1e6 << " MHz"
                   << " | userBW=" << cfg.bandwidthHz / 1e6 << " MHz"
                   << " | color=" << static_cast<uint32_t> (cfg.colorId)
                   << " | feederPhy=" << feederPhy << " <-peer-> userPhy=" << userPhy);
    }

  NS_LOG_INFO ("Beam installation complete: " << devices.GetN () << " beam devices created, "
               << m_userPhys.size () << " PHY pairs wired");
  return devices;
}

Ptr<NetDevice>
GeoBeamHelper::InstallSwitch (Ptr<Node> satNode, NetDeviceContainer beamDevices, Ptr<NetDevice> feederDevice)
{
  BridgeHelper bridge;
  NetDeviceContainer bridgePorts = beamDevices;
  if (feederDevice)
    bridgePorts.Add (feederDevice);
  NetDeviceContainer bridgeDev = bridge.Install (satNode, bridgePorts);
  NS_LOG_INFO ("Configured Satellite Switch (Bridge) on Node " << satNode->GetId ()
               << " connecting " << bridgePorts.GetN () << " interfaces.");
  return bridgeDev.Get (0);
}

} // namespace ns3
