#ifndef GW_HELPER_H
#define GW_HELPER_H

#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/ptr.h"
#include "ns3/object-factory.h"
#include "ns3/attribute.h"

namespace ns3 {

/**
 * \brief 信关站 (Gateway) 助手
 * \details 负责创建 GW 节点并连接核心网。
 */
class GwHelper
{
public:
  GwHelper ();
  virtual ~GwHelper ();

  void SetPhy (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());
  void SetDevice (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());
  void SetAntenna (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());

  /**
   * \brief 安装 GW 节点
   * \param gwNodes 信关站节点容器
   * \return 安装好的 NetDeviceContainer
   */
  NetDeviceContainer Install (NodeContainer gwNodes);

  /**
   * \brief 连接 GW 到核心网 (模拟地面光纤回传)
   * \param gwNode 信关站节点
   * \param coreNode 核心网节点
   * \return 安装好的 NetDeviceContainer
   */
  NetDeviceContainer ConnectToCore (Ptr<Node> gwNode, Ptr<Node> coreNode);

private:
  ObjectFactory m_deviceFactory;
  ObjectFactory m_phyFactory;
  ObjectFactory m_antennaFactory;
  double m_feederFreqHz; // 馈电频率 (例如 30GHz)
};

} // namespace ns3

#endif /* GW_HELPER_H */
