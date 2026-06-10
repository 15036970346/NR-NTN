#ifndef GW_HELPER_H
#define GW_HELPER_H

#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/ptr.h"
#include "ns3/object-factory.h"
#include "ns3/attribute.h"
#include "ns3/spectrum-channel.h"
#include "ns3/spectrum-model.h"
#include "../model/ntn-gw-phy.h"
#include <vector>

namespace ns3 {

/**
 * \brief 信关站 (Gateway) 助手
 * \details 为 GW 节点创建一个真正接到馈电 SpectrumChannel 的 NtnGwPhy。
 *
 * 典型用法 (配合 GeoBeamHelper):
 *   GeoBeamHelper sat;  sat.Install (satNodes);
 *   GwHelper gw;
 *   gw.SetFeederChannel (sat.GetFeederChannel ());
 *   gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
 *   gw.Install (gwNodes);
 */
class GwHelper
{
public:
  GwHelper ();
  virtual ~GwHelper ();

  void SetPhy (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());
  void SetDevice (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());
  void SetAntenna (std::string type, std::string n0 = "", const AttributeValue &v0 = EmptyAttributeValue ());

  /// 设置馈电信道 (必须先调, 否则 Install 会拒绝接线)
  void SetFeederChannel (Ptr<SpectrumChannel> channel);
  /// 设置馈电频段对应的 SpectrumModel (用于 PHY 的 Rx 模型识别)
  void SetFeederSpectrumModel (Ptr<const SpectrumModel> model);

  /**
   * \brief 安装 GW 节点: 给每个节点创建一个 NtnGwPhy, 接到馈电信道。
   * \return 占位 NetDeviceContainer (保持原 API; 数据面 Tx/Rx 走 PHY)
   */
  NetDeviceContainer Install (NodeContainer gwNodes);

  /**
   * \brief 连接 GW 到核心网 (模拟地面光纤回传)
   */
  NetDeviceContainer ConnectToCore (Ptr<Node> gwNode, Ptr<Node> coreNode);

  /// Install 之后, 按节点索引取出对应的 NtnGwPhy
  Ptr<NtnGwPhy> GetGwPhy (uint32_t idx) const;

private:
  ObjectFactory m_deviceFactory;
  ObjectFactory m_phyFactory;
  ObjectFactory m_antennaFactory;
  double m_feederFreqHz;
  Ptr<SpectrumChannel> m_feederChannel;
  Ptr<const SpectrumModel> m_feederSpectrumModel;
  std::vector<Ptr<NtnGwPhy>> m_gwPhys;
};

} // namespace ns3

#endif /* GW_HELPER_H */
