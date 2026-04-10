/*
 * 文件路径：contrib/geo-sat/model/sat-ut-rrc.h
 * 功能：用户终端 RRC 层测量控制模块
 */
#ifndef SAT_UT_RRC_H
#define SAT_UT_RRC_H

#include "ns3/object.h"
#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/callback.h"
#include "ns3/sat-mac-common.h" // 依赖于你在该文件中扩展的 MeasReport 和 MeasConfig

namespace ns3 {

class SatUtRrc : public Object {
public:
    static TypeId GetTypeId (void);
    SatUtRrc ();
    virtual ~SatUtRrc ();

    // 1. 核心接口：接收来自底层（SatUtPhy）的物理层原始测量数据
    void ProcessRawMeasurement (uint32_t beamId, double rawRsrp);

    // 2. 配置接口：设置测量参数 (由基站 SIB 广播或 RRC 重配下发)
    void SetMeasConfig (MeasConfig config);
    
    // 3. 移动性接口：绑定终端的移动性模型，用于获取实时三维坐标
    void SetMobilityModel (Ptr<MobilityModel> mobility);

    // 4. 标识接口：设置当前终端的 RNTI/UE ID
    void SetUeId (uint16_t ueId);

    // 5. 回调接口：当触发测量上报时，通过此回调将报告发给 MAC 层或上层发送模块
    typedef Callback<void, MeasReport> ReportCallback;
    void SetReportCallback (ReportCallback cb);

private:
    // 内部逻辑：评估是否满足测量事件触发条件 (如 Event A2/A4)
    void EvaluateMeasurementEvents ();

    // 内部逻辑：Time-to-Trigger 定时器超时后，真正生成并发送报告
    void TriggerMeasurementReport ();

    Ptr<MobilityModel> m_mobility; // 指向终端的移动性模型
    MeasConfig m_measConfig;       // 测量配置参数
    ReportCallback m_reportCb;     // 上报回调函数
    
    uint16_t m_ueId;           // 终端ID
    uint32_t m_currentBeamId;  // 当前正在测量的波束ID
    double m_filteredRsrp;     // L3 滤波后的 RSRP 平滑值
    bool m_isConditionMet;     // 标记是否已经满足门限条件
    EventId m_tttEvent;        // Time-to-Trigger 定时器句柄
};

} // namespace ns3
#endif /* SAT_UT_RRC_H */