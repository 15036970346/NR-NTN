/*
 * 文件路径：contrib/geo-sat/model/resource-manager.h
 * 功能：卫星无线资源管理模块 (RRM) - 遵循《NR NTN 3.23》S频段 35MHz 专版
 */
#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "ns3/object.h"
#include <map>

namespace ns3 {

// 终端类型枚举 (便携式 / 消费级)
enum UtType {
    UT_PORTABLE,  // 便携式终端 (33 dBW / 63 dBm EIRP)
    UT_CONSUMER   // 消费级手机 (20 dBW / 50 dBm EIRP)
};

// 业务类型 
enum TrafficType {
    TRAFFIC_VOICE,
    TRAFFIC_DATA,
    TRAFFIC_HIGH_CAPACITY
};

struct BeamAllocationUsage
{
  uint32_t dlUsedRbs {0};
  uint32_t ulUsedRbs {0};
  uint32_t dlSharedRbs {0}; // NOMA 功率域复用的下行 RB(叠加, 不额外占预算; 仅统计)
  uint32_t ulSharedRbs {0};
};

// 动态功率调配策略 (下行波束总功率在各 UE 之间的分配方式)
enum PowerAllocPolicy {
    POWER_EQUAL = 0,          // 等功率: 所有 UE 平均分
    POWER_CHANNEL_INVERSE = 1,// 信道反比: 弱信道 UE 多给 (拉公平)
    POWER_WATER_FILLING = 2   // 注水(简化): 强信道 UE 多给 (拉容量)
};

class ResourceManager : public Object
{
public:
  static TypeId GetTypeId (void);
  ResourceManager ();
  virtual ~ResourceManager ();

  // =================================================================
  // 功能 1：AllocateSpectrum (物理边界审查)
  // 当前项目配置：35MHz 带宽下，总RB=175；7色复用时每波束 DL/UL 预算均为25 RB。
  // 资源管理的最小粒度是 beam；每个调度轮次都需要先重置对应 beam 的预算。
  // =================================================================
  void ResetBeamAllocation (uint32_t beamId, bool isUplink);
  void ResetAllBeamAllocations (void);
  uint32_t GetRemainingRbs (uint32_t beamId, bool isUplink) const;
  // 每波束 DL/UL 预算的唯一真源 getter。AdmitControl 据此把"每波束总RB"
  // (利用率分母 / 准入上限) 与 ResourceManager 预算对齐, 避免独立默认值漂移。
  uint32_t GetDlBeamBudgetRbs (void) const { return m_dlBeamBudgetRbs; }
  uint32_t GetUlBeamBudgetRbs (void) const { return m_ulBeamBudgetRbs; }
  uint32_t AllocateSpectrum (uint32_t beamId, uint32_t requestedRbs, bool isUplink);
  // NOMA 功率域复用: 在已分配给主用户(near)的 RB 上叠加从用户(far)。
  // 不消耗 RB 预算(同一频率靠功率域区分), 仅记录复用量。返回批准叠加的 RB 数。
  uint32_t AllocateSharedSpectrum (uint32_t beamId, uint32_t requestedRbs, bool isUplink);
  uint32_t GetSharedRbs (uint32_t beamId, bool isUplink) const;
  // 某波束当前已用(主用户占用)的 RB 数。GetSharedRbs / GetUsedRbs 之比即 NOMA 复用增益。
  uint32_t GetUsedRbs (uint32_t beamId, bool isUplink) const;
  uint32_t GetMaxPowerLimitedUlRbs (UtType utType, double pathLossDb) const;

  // =================================================================
  // 功能 2：AdjustUtTxPower (3GPP 标准上行功控)
  // 结合终端发射功率较小的事实，执行 P_0 + 10*log10(M_RB) + alpha*PL 分数路损补偿
  // =================================================================
  // closedLoopOffsetDb: 闭环功控累积修正项 f(i) (dB); 默认 0 即纯开环。
  double AdjustUtTxPower (UtType utType, uint32_t allocatedRbs, double pathLossDb,
                          double closedLoopOffsetDb = 0.0);

  // =================================================================
  // 功能 3：动态功率调配 (下行波束总功率约束)
  // 给定某 UE 的分配权重与本轮所有被调度 UE 的权重之和，
  // 在波束总功率预算内返回该 UE 的下行发射功率 (dBm)。
  // =================================================================
  double GetPowerWeight (double cqi) const;                 // 按策略计算 UE 权重
  double ComputeDlTxPowerDbm (double ueWeight, double sumWeights) const;
  double GetBeamPowerBudgetDbm (void) const { return m_beamPowerBudgetDbm; }

private:
  // 每波束预算约束
  uint32_t m_dlBeamBudgetRbs; // 当前配置下每波束下行预算
  uint32_t m_ulBeamBudgetRbs; // 当前配置下每波束上行预算

  // 3GPP 功控核心参数
  double m_p0NominalPusch; // 基站期望收到的信号基准强度 P_0 (dBm)
  double m_alpha;          // 路径损耗补偿因子 (0.0 ~ 1.0)
  
  // EIRP 物理极限
  double m_maxEirpPortable; // 便携终端上限 (dBm)
  double m_maxEirpConsumer; // 消费终端上限 (dBm)

  // 动态功率调配 (下行)
  double m_beamPowerBudgetDbm; // 每波束下行总功率预算 (dBm)
  uint32_t m_powerAllocPolicy; // 功率分配策略 (PowerAllocPolicy)

  BeamAllocationUsage& GetOrCreateBeamUsage (uint32_t beamId);

  std::map<uint32_t, BeamAllocationUsage> m_beamUsageMap;
};

} // namespace ns3

#endif /* RESOURCE_MANAGER_H */
