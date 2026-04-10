/*
 * 文件路径：contrib/geo-sat/model/sat-ut-rrc.cc
 * 功能：用户终端 RRC 层测量控制模块实现
 */
#include "sat-ut-rrc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatUtRrc");
NS_OBJECT_ENSURE_REGISTERED (SatUtRrc);

TypeId SatUtRrc::GetTypeId (void) {
    static TypeId tid = TypeId ("ns3::SatUtRrc")
        .SetParent<Object> ()
        .SetGroupName ("SatGeo")
        .AddConstructor<SatUtRrc> ();
    return tid;
}

SatUtRrc::SatUtRrc() 
    : m_mobility(nullptr), 
      m_ueId(0),
      m_currentBeamId(0),
      m_filteredRsrp(-140.0), // 初始化为一个极低的无效值
      m_isConditionMet(false) 
{
    NS_LOG_FUNCTION (this);
    // 设置默认的测量配置（实际中应由基站下发）
    m_measConfig.rsrpThreshold = -105.0; // 门限 -105 dBm
    m_measConfig.hysteresis = 2.0;       // 迟滞 2 dB
    m_measConfig.timeToTrigger = MilliSeconds(200); // TTT为 200ms
    m_measConfig.filterCoeff = 0.5;      // L3滤波系数 (0~1之间)
}

SatUtRrc::~SatUtRrc() {
    NS_LOG_FUNCTION (this);
}

void SatUtRrc::SetMeasConfig (MeasConfig config) {
    m_measConfig = config;
}

void SatUtRrc::SetMobilityModel (Ptr<MobilityModel> mobility) {
    m_mobility = mobility;
}

void SatUtRrc::SetUeId (uint16_t ueId) {
    m_ueId = ueId;
}

void SatUtRrc::SetReportCallback (ReportCallback cb) {
    m_reportCb = cb;
}

void SatUtRrc::ProcessRawMeasurement (uint32_t beamId, double rawRsrp) {
    // 1. Layer 3 滤波 (平滑瞬时信道衰落引起的抖动)
    // 公式: Fn = (1 - a) * Fn-1 + a * Mn
    double a = m_measConfig.filterCoeff;
    if (m_filteredRsrp == -140.0) {
        m_filteredRsrp = rawRsrp; // 第一次测量直接赋值
    } else {
        m_filteredRsrp = (1.0 - a) * m_filteredRsrp + a * rawRsrp;
    }

    m_currentBeamId = beamId;
    
    NS_LOG_DEBUG ("UE " << m_ueId << " | Raw RSRP: " << rawRsrp 
                  << " dBm | L3 Filtered RSRP: " << m_filteredRsrp << " dBm");

    // 2. 评估测量事件
    EvaluateMeasurementEvents();
}

void SatUtRrc::EvaluateMeasurementEvents () {
    // 这里以类似 Event A2 (服务小区质量低于门限) 为例
    // 引入迟滞 (Hysteresis) 以防止乒乓效应
    
    double enterThreshold = m_measConfig.rsrpThreshold - m_measConfig.hysteresis;
    double leaveThreshold = m_measConfig.rsrpThreshold + m_measConfig.hysteresis;

    if (m_filteredRsrp < enterThreshold) {
        // 条件满足：信号变得很差
        if (!m_isConditionMet) {
            m_isConditionMet = true;
            
            // 启动 Time-to-Trigger (TTT) 定时器
            NS_LOG_INFO ("UE " << m_ueId << " RSRP drops below threshold. Starting TTT timer.");
            m_tttEvent = Simulator::Schedule (m_measConfig.timeToTrigger, 
                                              &SatUtRrc::TriggerMeasurementReport, 
                                              this);
        }
    } else if (m_filteredRsrp > leaveThreshold) {
        // 条件不满足：信号恢复良好
        if (m_isConditionMet) {
            m_isConditionMet = false;
            
            // 如果 TTT 定时器还在跑，说明信号是短暂变差后又恢复了，属于假警报，取消定时器
            if (m_tttEvent.IsRunning()) {
                Simulator::Cancel(m_tttEvent);
                NS_LOG_INFO ("UE " << m_ueId << " RSRP recovered. Cancelled TTT timer.");
            }
        }
    }
}

void SatUtRrc::TriggerMeasurementReport () {
    NS_LOG_INFO ("UE " << m_ueId << " Measurement Event Triggered! Generating Report.");
    
    MeasReport report;
    report.ueId = m_ueId;
    report.bestBeamId = m_currentBeamId;
    report.rsrp = m_filteredRsrp;
    
    // 核心特色：为星地切换提供位置维度输入
    if (m_mobility) {
        report.position = m_mobility->GetPosition();
        NS_LOG_INFO ("Attached UT Position to Report: X=" << report.position.x 
                     << ", Y=" << report.position.y 
                     << ", Z=" << report.position.z);
    } else {
        NS_LOG_WARN ("MobilityModel is null. Position not included in report.");
    }
    
    // 触发回调，将报告送出（通常是送给 MAC 层，然后通过 PUSCH 发给信关站）
    if (!m_reportCb.IsNull()) {
        m_reportCb (report);
    }
}

} // namespace ns3