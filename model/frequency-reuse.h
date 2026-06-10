/*
 * 文件路径：contrib/geo-sat/model/frequency-reuse.h
 * 功能：七色频率复用管理器
 * 支持：1/7频率复用策略、GEO卫星多波束频率分配
 */
#ifndef FREQUENCY_REUSE_H
#define FREQUENCY_REUSE_H

#include "ns3/object.h"
#include <vector>
#include <map>

namespace ns3 {

class FrequencyReuse : public Object
{
public:
    static TypeId GetTypeId (void);
    FrequencyReuse ();
    virtual ~FrequencyReuse ();

    // 设置总带宽和可用RB数
    void Configure (double totalBandwidthHz, uint32_t totalRbs, uint8_t reuseFactor);

    /**
     * \brief 显式登记某波束所属颜色 (例如 helper 按 g_4ColorFrequencyConfig 中的 colorId 注入)。
     *        登记后 GetColorForBeam / GetInterferingBeams 才会返回真实物理布局; 不登记走 fallback (beamId % reuseFactor)。
     */
    void AddBeam (uint32_t beamId, uint8_t colorId);

    // 获取某个波束的频率分配
    std::vector<uint32_t> GetBeamAllocation (uint32_t beamId) const;
    
    // 获取某个UE可用的RB列表 (基于其连接的波束)
    std::vector<uint32_t> GetUeAvailableRbs (uint32_t beamId) const;
    
    // 检查某个RB在目标波束是否可用 (无干扰)
    bool IsRbAvailableAtBeam (uint32_t rbId, uint32_t beamId) const;
    
    // 获取复用因子
    uint8_t GetReuseFactor () const;
    
    // 获取总可用RB数 (考虑频率复用后)
    uint32_t GetAvailableRbsPerBeam () const;
    
    // 获取干扰波束列表
    std::vector<uint32_t> GetInterferingBeams (uint32_t beamId) const;
    
    // 计算频率复用增益
    double CalculateReuseGain () const;
    
    // 打印频率分配表
    void PrintFrequencyAllocation () const;

private:
    void InitializeColorPattern ();
    uint8_t GetColorForBeam (uint32_t beamId) const;

    double m_totalBandwidthHz;
    uint32_t m_totalRbs;
    uint8_t m_reuseFactor;  // 7 for 7-color reuse
    
    std::map<uint32_t, uint8_t> m_beamToColor;  // 波束ID到颜色的映射
    std::map<uint8_t, std::vector<uint32_t>> m_colorToRbs;  // 颜色到RB列表的映射
    std::map<uint8_t, std::vector<uint32_t>> m_colorToBeams;  // 颜色到波束列表的映射
    
    static const uint8_t MAX_COLORS = 7;
};

} // namespace ns3

#endif /* FREQUENCY_REUSE_H */
