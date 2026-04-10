/*
 * 文件路径：contrib/geo-sat/model/sat-mac-phy-sap.h
 * 功能：定义 MAC 层与 PHY 层之间的服务访问点 (SAP) 接口
 */
#ifndef SAT_MAC_PHY_SAP_H
#define SAT_MAC_PHY_SAP_H

#include "sat-mac-common.h" // 包含 MeasReport 和 DciInfo

namespace ns3 {

// PHY 层调用此接口，MAC 层负责实现此接口
class SatMacPhySapUser {
public:
    virtual ~SatMacPhySapUser() {}
    
    // 物理层完成测量后，调用此接口上报给 MAC/调度器/切换算法
    virtual void ReceiveMeasReport (const MeasReport& report) = 0;
};

} // namespace ns3
#endif /* SAT_MAC_PHY_SAP_H */