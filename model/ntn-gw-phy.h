/*
 * 文件路径: contrib/geo-sat/model/ntn-gw-phy.h
 * 功能: 信关站 (基站) 侧物理层。
 *
 * 在 NTN 透明转发架构里:
 *   GW(NtnGwPhy) ──馈电链路──► 卫星(SatGeoFeederPhy ↔ SatGeoUserPhy) ──用户链路──► UE(SatPhy)
 *
 * NtnGwPhy 继承 SatPhy 基类, 不另外引入接口: 它的"GW 角色"是语义上的 (放在 GW 节点上,
 * 挂馈电链路 SpectrumChannel)。
 *
 * 与卫星上的 SatGeoFeederPhy 不同, NtnGwPhy 不做"透明弯管转发":
 * 它收到的包应交给本节点的 MAC/RRC, 这是基类 SatPhy::Receive 默认行为
 * (m_rxTrace 之后调用 m_rxCallback)。MAC 通过 SetRxCallback() 订阅。
 */
#ifndef NTN_GW_PHY_H
#define NTN_GW_PHY_H

#include "sat-phy.h"

namespace ns3 {

class NtnGwPhy : public SatPhy
{
public:
  static TypeId GetTypeId (void);
  NtnGwPhy ();
  virtual ~NtnGwPhy ();
  // SetRxCallback / Receive 全部继承自 SatPhy 基类, 这里只保留 TypeId 用作语义标识。
};

} // namespace ns3

#endif /* NTN_GW_PHY_H */
