/*
 * 文件路径：contrib/geo-sat/model/admit-control.h
 * 功能：GEO卫星准入控制与波束协调模块
 * 实现：在切换发生时判断目标波束是否有足够资源接纳新用户
 */
#ifndef ADMIT_CONTROL_H
#define ADMIT_CONTROL_H

#include "ns3/object.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include "ns3/callback.h"
#include "resource-manager.h"
#include <map>
#include <vector>

namespace ns3 {

// 业务优先级枚举
enum class ServicePriority {
    PRIORITY_EMERGENCY = 0,  // 应急业务 (最高优先级)
    PRIORITY_VOICE = 1,      // 语音业务
    PRIORITY_DATA = 2,        // 普通数据业务
    PRIORITY_BEST_EFFORT = 3  // 尽力而为 (最低优先级)
};

// 准入决策结果
enum class AdmitDecision {
    ADMIT_OK,              // 允许接入
    ADMIT_REJECTED,        // 拒绝接入
    ADMIT_REDIRECT,        // 重定向到其他波束
    ADMIT_QUEUE            // 排队等待
};

// 波束资源状态
struct BeamResourceStatus {
    uint32_t beamId {0};
    uint32_t totalRbs {25};           // 总RB资源
    uint32_t usedRbs {0};             // 已使用RB
    uint32_t emergencyRbs {0};        // 应急业务预留RB
    uint32_t guaranteedRbs {0};       // 保障带宽RB
    double utilizationRatio {0.0};    // 资源利用率
    uint32_t activeUeCount {0};       // 活跃用户数
    std::map<ServicePriority, uint32_t> priorityCount; // 各优先级用户数
};

// UE上下文信息
struct UeContextInfo {
    uint16_t rnti {0};
    uint32_t currentBeamId {0};
    ServicePriority priority {ServicePriority::PRIORITY_DATA};
    UtType utType {UT_CONSUMER};
    TrafficType trafficType {TRAFFIC_DATA};
    Vector position {0.0, 0.0, 0.0};
    double latestCqi {7.0};
    uint32_t requiredRbs {1};
    Time connectedTime {Seconds (0)};
};

class AdmitControl : public Object
{
public:
    static TypeId GetTypeId (void);
    AdmitControl ();
    virtual ~AdmitControl ();

    // ==================== 准入控制核心接口 ====================
    
    // 准入决策：判断UE是否可以接入目标波束
    AdmitDecision CanAdmitUe (uint32_t targetBeamId, ServicePriority priority, 
                               UtType utType, TrafficType trafficType, uint32_t requiredRbs,
                               bool isUplink = false);
    
    // 切换准入决策：判断切换是否可行
    AdmitDecision CanHandoverUe (uint32_t sourceBeamId, uint32_t targetBeamId,
                                  ServicePriority priority, UtType utType,
                                  TrafficType trafficType, uint32_t requiredRbs,
                                  bool isUplink = false);
    
    // 注册UE到波束
    void RegisterUeToBeam (uint16_t rnti, uint32_t beamId, ServicePriority priority,
                           UtType utType, TrafficType trafficType, Vector position);
    
    // 从波束注销UE
    void UnregisterUeFromBeam (uint16_t rnti, uint32_t beamId);
    
    // 更新UE上下文
    void UpdateUeContext (uint16_t rnti, double latestCqi, uint32_t requiredRbs);

    // ==================== 波束协调接口 ====================
    
    // 获取目标波束的推荐列表 (基于资源状况)
    std::vector<uint32_t> GetRecommendedBeams (uint32_t sourceBeamId,
                                               ServicePriority priority,
                                               UtType utType,
                                               TrafficType trafficType,
                                               uint32_t requiredRbs,
                                               bool isUplink = false);
    
    // 获取波束资源状态
    BeamResourceStatus GetBeamResourceStatus (uint32_t beamId);
    
    // 检查波束间负载均衡
    void CheckBeamLoadBalancing ();
    
    // 触发波束间负载重平衡
    void TriggerLoadRebalancing ();
    void SetHandoverExecutor (Callback<void, uint16_t, uint32_t> handoverExecutor);

    // ==================== 资源配置 ====================
    
    // 设置波束总RB资源
    void SetBeamTotalRbs (uint32_t beamId, uint32_t totalRbs);
    
    // 设置应急/语音的刚性预留策略
    void SetPriorityReservationPolicy (uint32_t emergencyReservedRbs,
                                       uint32_t emergencyBurstCapRbs,
                                       uint32_t voiceReservedRbs);
    void SetDlReservedRbs (uint32_t reservedRbs);

    // 绑定共享的资源管理器，统一读取波束实时剩余资源
    void SetResourceManager (Ptr<ResourceManager> resourceManager);

    uint32_t GetEmergencyReservedRbs () const;
    uint32_t GetEmergencyBurstCapRbs () const;
    uint32_t GetVoiceReservedRbs () const;
    
    // 获取当前系统总活跃UE数
    uint32_t GetTotalActiveUes () const;

private:
    // 内部辅助函数
    uint32_t GetAvailableRbs (uint32_t beamId, ServicePriority priority, bool isUplink);
    uint32_t GetPhysicalRemainingRbs (uint32_t beamId, bool isUplink);
    bool CheckEmergencyCapacity (uint32_t beamId);
    double CalculateHandoverBenefit (uint32_t sourceBeamId, uint32_t targetBeamId,
                                     ServicePriority priority, UtType utType,
                                     TrafficType trafficType, uint32_t requiredRbs,
                                     bool isUplink);
    void UpdateBeamStatistics (uint32_t beamId);
    ServicePriority MapTrafficTypeToPriority (TrafficType trafficType);
    uint32_t CalculateEffectiveRequiredRbs (uint32_t beamId,
                                            ServicePriority priority,
                                            UtType utType,
                                            TrafficType trafficType,
                                            uint32_t requiredRbs,
                                            bool isUplink) const;
    double GetAdmissionThreshold (ServicePriority priority,
                                  UtType utType,
                                  TrafficType trafficType,
                                  bool isUplink) const;

    // 波束资源映射
    std::map<uint32_t, BeamResourceStatus> m_beamResources;
    
    // UE上下文映射
    std::map<uint16_t, UeContextInfo> m_ueContextMap;
    
    // 配置参数
    uint32_t m_emergencyReservedRbs;     // 应急控制默认刚性预留 (默认1 PRB)
    uint32_t m_emergencyBurstCapRbs;     // 应急突发模式上限 (默认3 PRB)
    uint32_t m_voiceReservedRbs;         // 语音业务默认刚性预留 (默认2 PRB)
    uint32_t m_totalDlRbs;              // 7色复用下每波束下行可用RB (25)
    uint32_t m_totalUlRbs;              // 7色复用下每波束上行可用RB (25)
    uint32_t m_dlReservedRbs;           // 下行侧为PRACH/控制信道保留的RB
    double m_admissionThreshold;        // 准入阈值 (资源利用率>90%拒绝)
    double m_handoverBenefitThreshold;   // 切换收益阈值
    uint32_t m_emergencyUserCapPerBeam;  // 每波束允许的应急用户上限
    Ptr<ResourceManager> m_resourceManager;
    Callback<void, uint16_t, uint32_t> m_handoverExecutor;
};

} // namespace ns3

#endif /* ADMIT_CONTROL_H */
