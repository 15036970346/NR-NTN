#include "gw-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/mobility-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/phased-array-model.h"
#include "ns3/net-device.h"
#include "../model/ntn-gw-phy.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GwHelper");

GwHelper::GwHelper ()
  : m_feederFreqHz (30e9)
{
  m_deviceFactory.SetTypeId ("ns3::NrGnbNetDevice");
  m_phyFactory.SetTypeId ("ns3::NtnGwPhy");
  m_antennaFactory.SetTypeId ("ns3::UniformPlanarArray");
  m_antennaFactory.Set ("NumRows", UintegerValue (8));
  m_antennaFactory.Set ("NumColumns", UintegerValue (8));
}

GwHelper::~GwHelper () {}

void
GwHelper::SetPhy (std::string type, std::string n0, const AttributeValue &v0)
{
  m_phyFactory.SetTypeId (type);
  if (!n0.empty ()) m_phyFactory.Set (n0, v0);
}

void
GwHelper::SetDevice (std::string type, std::string n0, const AttributeValue &v0)
{
  m_deviceFactory.SetTypeId (type);
  if (!n0.empty ()) m_deviceFactory.Set (n0, v0);
}

void
GwHelper::SetAntenna (std::string type, std::string n0, const AttributeValue &v0)
{
  m_antennaFactory.SetTypeId (type);
  if (!n0.empty ()) m_antennaFactory.Set (n0, v0);
}

void
GwHelper::SetFeederChannel (Ptr<SpectrumChannel> channel)
{
  m_feederChannel = channel;
}

void
GwHelper::SetFeederSpectrumModel (Ptr<const SpectrumModel> model)
{
  m_feederSpectrumModel = model;
}

NetDeviceContainer
GwHelper::Install (NodeContainer gwNodes)
{
  NetDeviceContainer devices;

  if (!m_feederChannel)
    {
      NS_LOG_ERROR ("GwHelper::Install called before SetFeederChannel(); GW PHY not wired");
    }
  if (!m_feederSpectrumModel)
    {
      NS_LOG_WARN ("GwHelper::Install: feeder SpectrumModel not set; PHY rx may not deliver");
    }

  // 节点没有移动模型就给个常量位置, 避免 SpectrumChannel 算距离时崩
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  for (uint32_t i = 0; i < gwNodes.GetN (); ++i)
    {
      if (!gwNodes.Get (i)->GetObject<MobilityModel> ())
        {
          mobility.Install (gwNodes.Get (i));
        }
    }

  for (uint32_t i = 0; i < gwNodes.GetN (); ++i)
    {
      Ptr<Node> node = gwNodes.Get (i);

      // 真正承载数据面的 NtnGwPhy (不再创建空壳 NrGnbNetDevice, 否则
      // Simulator::Run 初始化时会在子模块未配齐处崩)
      Ptr<Object> phyObj = m_phyFactory.Create ();
      Ptr<NtnGwPhy> phy = DynamicCast<NtnGwPhy> (phyObj);
      if (!phy)
        {
          NS_LOG_ERROR ("PhyFactory did not produce NtnGwPhy; check SetPhy() type");
          continue;
        }

      if (m_feederChannel)
        {
          phy->SetChannel (m_feederChannel);
          if (m_feederSpectrumModel)
            {
              phy->SetRxSpectrumModel (m_feederSpectrumModel);
            }
          m_feederChannel->AddRx (phy);
        }
      phy->SetMobility (node->GetObject<MobilityModel> ());
      node->AggregateObject (phy);
      m_gwPhys.push_back (phy);

      NS_LOG_INFO ("GW node " << node->GetId ()
                   << " | NtnGwPhy " << phy
                   << " | feederFreq=" << m_feederFreqHz / 1e9 << " GHz"
                   << " | channel=" << m_feederChannel);
    }
  return devices;  // 空容器: 数据面走聚合在节点上的 NtnGwPhy
}

NetDeviceContainer
GwHelper::ConnectToCore (Ptr<Node> gwNode, Ptr<Node> coreNode)
{
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("100Gbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer link = p2p.Install (gwNode, coreNode);
  NS_LOG_INFO ("Connected GW " << gwNode->GetId () << " <-> Core " << coreNode->GetId ()
               << " (100Gbps, 2ms fiber)");
  return link;
}

Ptr<NtnGwPhy>
GwHelper::GetGwPhy (uint32_t idx) const
{
  return (idx < m_gwPhys.size ()) ? m_gwPhys[idx] : nullptr;
}

} // namespace ns3
