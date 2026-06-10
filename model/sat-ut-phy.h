/*
 * 文件路径：contrib/geo-sat/model/sat-ut-phy.h
 * 功能：用户终端物理层，负责 SINR 计算、同频干扰处理与测量上报
 */
#ifndef SAT_UT_PHY_H
#define SAT_UT_PHY_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"
#include "ns3/ptr.h"
#include "sat-mac-phy-sap.h"
#include "frequency-reuse.h"
#include <string>
#include <vector>
#include <map>

namespace ns3 {

// 多址接入模式枚举
enum class MultipleAccessMode {
    SLOTTED_ALOHA,    // 纯ALOHA
    ESSA,             // 增强型Slotted ALOHA (ESSA)
    FDMA,             // 频分多址
    TDMA              // 时分多址
};

class SatUtPhy : public Object {
public:
    static TypeId GetTypeId (void);
    SatUtPhy ();
    virtual ~SatUtPhy ();

    // 绑定 SAP 接口
    void SetMacPhySapUser (SatMacPhySapUser* sapUser);

    // ==============================================================
    // 设计文档 3.2.1.2 规定的核心功能接口
    // ==============================================================
    void PerformHandover (uint32_t targetBeamId);
    double CalculateSinr (double rxPowerDbm, uint32_t beamId);
    void Receive (Ptr<Packet> packet, double rxPowerDbm, uint32_t beamId);
    void AssignNewSatChannels (uint32_t channelId);
    void UpdateSliceSubscription (uint32_t sliceId, bool subscribe);

    // ==============================================================
    // 增强的 SINR 计算 (C/I + 系统干扰)
    // ==============================================================
    // 完整的SINR计算，包含：
    // - C (期望信号功率)
    // - I (共道干扰)
    // - I_adj (临道干扰)
    // - N (热噪声)
    double CalculateComprehensiveSinr (double rxPowerDbm, uint32_t beamId);
    
    // 计算共道干扰比 (C/I)
    double CalculateCarrierToInterferenceRatio (double rxPowerDbm, uint32_t beamId);
    
    // 计算载干比 (C/(I+N))
    double CalculateCarrierToInterferencePlusNoise (double rxPowerDbm, uint32_t beamId);

    // 测量接口 (供测试使用)
    uint8_t MapSinrToCqi (double sinrDb);

    // ==============================================================
    // 随机接入与定时补偿 (DoRandomAccess)
    // ==============================================================
    // 执行随机接入流程 (PRACH前导码发送)
    void DoRandomAccess (uint32_t preambleId = 0, uint8_t format = 0);
    
    // 设置随机接入参数
    void SetRaParams (uint32_t preambleId, uint8_t format, Time transmissionTime);
    
    // 获取GEO卫星定时提前量 (Timing Advance)
    Time GetTimingAdvance () const;
    
    // 设置多址接入模式
    void SetMultipleAccessMode (MultipleAccessMode mode);
    
    // 计算ESSA碰撞概率
    double CalculateCollisionProbability (uint32_t numActiveUsers, uint32_t numSlots);
    
    // ESSA时隙选择
    uint32_t SelectEssAlohaSlot ();

    // ==============================================================
    // CQI 反馈机制
    // ==============================================================
    // 触发CQI报告
    void TriggerCqiReport ();
    
    // 获取当前SINR (dB)
    double GetLastCalculatedSinr () const;

    // 获取最近一次测量的 RSRP (dBm)
    double GetLastRsrp () const;

    // 获取最近一次计算的 RSRQ (dB)
    double GetLastRsrq () const;

    // 计算 RSRQ (参考 3GPP, 公式: 10*log10(N*RSRP/RSSI))
    double CalculateRsrq (double rsrpDbm, uint32_t beamId);

    // 设置CQI回调
    void SetCqiReportCallback (Callback<void, uint8_t> callback);

    // ==============================================================
    // 多波束同色 C/I 干扰模型 (helper 注入)
    // ==============================================================
    /// 注入 FrequencyReuse 实例 (查同色干扰束). nullptr → 退回到 legacy mock 干扰数。
    void SetFrequencyReuse (Ptr<FrequencyReuse> fr);
    /// 单干扰束 C/I_co (dB). 缺省 18, 即主瓣 43 - 旁瓣 25。
    void SetCoBeamCIDb (double ciDb);
    /// 当前服务波束 ID, 写入后 SinrTrace 才能区分。
    void SetCurrentBeamId (uint32_t beamId);

private:
    double GetInterferenceFromTrace (uint32_t beamId);
    double GetAdjacentChannelInterference (uint32_t beamId);
    double CalculateThermalNoise ();
    void DoPrachTransmission ();
    void ScheduleRaResponse (uint32_t preambleId);
    
    // 共道干扰计算 (同频复用波束)
    double GetCoChannelInterference (uint32_t beamId);
    
    // 临道干扰计算
    double GetAdjacentChannelInterferenceFromBeam (uint32_t beamId);

    SatMacPhySapUser* m_macSapUser;
    Callback<void, uint8_t> m_cqiReportCallback;
    
    uint16_t m_rnti;
    uint32_t m_currentBeamId;
    double m_lastCalculatedSinr;   // 最近一次计算的 SINR (dB)
    double m_lastRsrp;             // 最近一次测量的 RSRP (dBm)
    
    // 物理层参数
    double m_bandwidthHz;
    double m_noiseFigure;
    double m_antennaGainDbi;
    std::string m_traceFilePath;
    
    // 随机接入参数
    uint32_t m_preambleId;
    uint8_t m_prachFormat;
    Time m_raTransmissionTime;
    Time m_timingAdvance;
    EventId m_raEvent;
    
    // 多址接入参数
    MultipleAccessMode m_maMode;
    uint32_t m_essaNumSlots;
    std::map<uint32_t, uint32_t> m_slotAllocationMap;

    // PHY 层统计 TracedCallbacks
    double m_lastRsrq;             // 最近一次计算的 RSRQ (dB)
    TracedCallback<uint16_t, double> m_rsrpTrace;   // (rnti, rsrp_dBm)
    TracedCallback<uint16_t, double> m_sinrTrace;   // (rnti, sinr_dB)
    TracedCallback<uint16_t, double> m_rsrqTrace;   // (rnti, rsrq_dB)

    // 多波束 C/I_co 模型 (helper 注入)
    Ptr<FrequencyReuse> m_freqReuse;
    double              m_coBeamCIDb {18.0};
};

} // namespace ns3
#endif /* SAT_UT_PHY_H */