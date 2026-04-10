/*
 * 文件路径：contrib/geo-sat/model/resource-manager.h
 * 功能：卫星无线资源管理模块 (RRM) - S频段专版
 * 包含：基于终端与业务的频谱资源块(RB)分配、基于目标 MCS 的上行功率控制
 */
#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "ns3/object.h"

namespace ns3 {

// 终端类型枚举 (便携式 / 消费级)
enum UtType {
    UT_PORTABLE,  // 便携式终端 (低 EIRP 能力)
    UT_CONSUMER   // 消费级终端 (高 EIRP 能力)
};

// 业务类型枚举 (语音 / 数据 / 大容量)
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
  // 功能 1：AllocateSpectrum (频谱分配)
  // 根据终端类型、业务类型和信道质量，动态分配 RB 数量。
  // =================================================================
  uint32_t AllocateSpectrum (UtType utType, TrafficType trafficType, double cqi, bool isUplink);

  // =================================================================
  // 功能 2：AdjustUtTxPower (功率控制)
  // 根据终端 EIRP 能力、上行链路质量，调整发射功率以满足目标 MCS。
  // =================================================================
  double AdjustUtTxPower (UtType utType, double currentUplinkSnr, uint8_t targetMcs);

private:
  double m_operatingFrequency; // S频段工作频率 (默认 2GHz)
  
  // EIRP 能力边界 (dBW 转 dBm 内部使用)
  double m_minEirpDbw; // 3 dBW
  double m_maxEirpDbw; // 20 dBW
};

} // namespace ns3

#endif /* RESOURCE_MANAGER_H */