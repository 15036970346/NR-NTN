/*
 * 文件路径：contrib/geo-sat/model/resource-manager.h
 * 功能：卫星无线资源管理模块 (RRM) - 统一到 35MHz / 175 PRB / 单波束25 PRB 口径
 */
#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "ns3/object.h"

namespace ns3 {

// 终端类型枚举 (便携式 / 消费级)
enum UtType {
    UT_PORTABLE,  // 便携式终端 (天线小，发射功率较小，上行受限 50 PRB)
    UT_CONSUMER   // 消费级终端 (天线大，高 EIRP 能力)
};

// 业务类型 
enum TrafficType {
    TRAFFIC_VOICE,
    TRAFFIC_DATA,
    TRAFFIC_HIGH_CAPACITY
};

class ResourceManager : public Object
{
public:
  static TypeId GetTypeId (void);
  ResourceManager ();
  virtual ~ResourceManager ();

  // =================================================================
  // 功能 1：AllocateSpectrum (物理边界审查)
  // 统一口径：单波束调度容量按 25 PRB 建模
  // =================================================================
  uint32_t AllocateSpectrum (UtType utType, uint32_t requestedRbs, bool isUplink);

  // =================================================================
  // 功能 2：AdjustUtTxPower (3GPP 标准上行功控)
  // 结合终端发射功率较小的事实，执行 P_0 + 10*log10(M_RB) + alpha*PL 分数路损补偿
  // =================================================================
  double AdjustUtTxPower (UtType utType, uint32_t allocatedRbs, double pathLossDb);

private:
  // 3GPP 功控核心参数
  double m_p0NominalPusch; // 基站期望收到的信号基准强度 P_0 (dBm)
  double m_alpha;          // 路径损耗补偿因子 (0.0 ~ 1.0)
  
  // EIRP 物理极限
  double m_maxEirpPortable; // 便携终端上限 (dBm)
  double m_maxEirpConsumer; // 消费终端上限 (dBm)
};

} // namespace ns3

#endif /* RESOURCE_MANAGER_H */
