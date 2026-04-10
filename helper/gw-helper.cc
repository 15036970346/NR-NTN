#include "gw-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/mobility-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/phased-array-model.h"
#include "ns3/net-device.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GwHelper");

GwHelper::GwHelper ()
{
  m_deviceFactory.SetTypeId ("ns3::NrGnbNetDevice");
  m_phyFactory.SetTypeId ("ns3::SatPhy");
  m_antennaFactory.SetTypeId ("ns3::UniformPlanarArray");
  m_antennaFactory.Set ("NumRows", UintegerValue (8));
  m_antennaFactory.Set ("NumColumns", UintegerValue (8));
  m_feederFreqHz = 30e9;
}

GwHelper::~GwHelper ()
{
}

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

NetDeviceContainer
GwHelper::Install (NodeContainer gwNodes)
{
  NetDeviceContainer devices;

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (gwNodes);

  for (uint32_t i = 0; i < gwNodes.GetN (); ++i)
    {
      Ptr<Node> node = gwNodes.Get (i);

      Ptr<NetDevice> device = m_deviceFactory.Create<NetDevice> ();
      Ptr<Object> phyObj = m_phyFactory.Create ();
      Ptr<Object> antennaObj = m_antennaFactory.Create ();

      node->AddDevice (device);
      devices.Add (device);

      NS_LOG_INFO ("Installed GW SatNetDevice on Node " << node->GetId ()
                   << " (Freq: " << m_feederFreqHz / 1e9 << " GHz)");
    }
  return devices;
}

NetDeviceContainer
GwHelper::ConnectToCore (Ptr<Node> gwNode, Ptr<Node> coreNode)
{
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("100Gbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer link = p2p.Install (gwNode, coreNode);

  NS_LOG_INFO ("Connected GW " << gwNode->GetId () << " <---> Core " << coreNode->GetId ()
               << " via Fiber (100Gbps, 2ms)");

  return link;
}

} // namespace ns3
