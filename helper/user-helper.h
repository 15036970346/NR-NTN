/*
 * user-helper.h
 * Function: Helper class to create User Terminals (UEs)
 * Fixed: Changed member variable m_satChannel to Ptr<SpectrumChannel> to match the cpp implementation.
 */

#ifndef USER_HELPER_H
#define USER_HELPER_H

#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/spectrum-channel.h"
#include "ns3/vector.h"

namespace ns3 {

/**
 * \brief 波束信息结构
 * \details 描述单个波束的位置参数
 */
struct BeamInfo
{
  uint16_t beamId;        //!< 波束ID
  Vector centerPosition; //!< 波束中心位置 (经度, 纬度, 高度) 单位：度/米
  double radius;          //!< 波束覆盖半径 (米)
};

class SatUserHelper : public Object
{
public:
  static TypeId GetTypeId (void);
  SatUserHelper ();
  virtual ~SatUserHelper ();

  void SetBeamId (uint16_t beamId);
  
  // [Fix] 函数参数改为基类
  void SetSatelliteChannel (Ptr<SpectrumChannel> channel);
  
  void SetNrHelper (Ptr<NrHelper> nrHelper);

  /**
   * \brief 设置多个波束信息
   * \param beams 波束信息向量
   */
  void SetBeams (const std::vector<BeamInfo>& beams);

  /**
   * \brief 在多个波束范围内创建随机分布的用户节点
   * \param totalCount 总共需要创建的UE数量
   * \return 创建的UE节点容器
   *
   * \details 该方法将根据每个波束的覆盖范围，在波束中心附近随机撒点
   *          UE数量按波束面积比例分配到各个波束
   */
  NodeContainer CreateUsersInMultipleBeams (uint32_t totalCount);

  /**
   * \brief 在指定波束中心附近创建随机分布的用户节点
   * \param beamInfo 波束信息
   * \param count 需要创建的UE数量
   * \return 创建的UE节点容器
   */
  NodeContainer CreateUsersAroundBeam (const BeamInfo& beamInfo, uint32_t count);

  NodeContainer CreateUserNodes (uint32_t count);

  void InstallMobility (NodeContainer ues, std::string positionAllocatorType, std::string paramName, std::string paramValue);
  
  NetDeviceContainer InstallStack (NodeContainer ues);
  
  Ipv4InterfaceContainer AssignIp (NetDeviceContainer ueDevices, Ipv4Address baseAddress, Ipv4Mask mask);
  
  void InstallApplication (NodeContainer ues, Ipv4Address destAddress);

private:
  Ptr<NrHelper> m_nrHelper;

  Ptr<SpectrumChannel> m_satChannel;

  uint16_t m_beamId;
  std::vector<BeamInfo> m_beams;  //!< 多个波束信息
  MobilityHelper m_mobility;
  Ipv4AddressHelper m_ipv4;  //!< 保持兼容，保留私有成员
  ObjectFactory m_positionAllocatorFactory;
};

} // namespace ns3

#endif /* USER_HELPER_H */