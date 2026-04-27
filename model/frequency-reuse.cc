/*
 * 文件路径：contrib/geo-sat/model/frequency-reuse.cc
 * 功能：七色频率复用管理器实现
 */
#include "frequency-reuse.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("FrequencyReuse");
NS_OBJECT_ENSURE_REGISTERED (FrequencyReuse);

const uint8_t FrequencyReuse::MAX_COLORS;

TypeId FrequencyReuse::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::FrequencyReuse")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<FrequencyReuse> ();
  return tid;
}

FrequencyReuse::FrequencyReuse ()
  : m_totalBandwidthHz (35e6),
    m_totalRbs (175),
    m_reuseFactor (7)
{
  NS_LOG_FUNCTION (this);
  InitializeColorPattern ();
}

FrequencyReuse::~FrequencyReuse () { NS_LOG_FUNCTION (this); }

void FrequencyReuse::Configure (double totalBandwidthHz, uint32_t totalRbs, uint8_t reuseFactor)
{
  NS_LOG_FUNCTION (this << totalBandwidthHz << totalRbs << (uint32_t)reuseFactor);
  
  m_totalBandwidthHz = totalBandwidthHz;
  m_totalRbs = totalRbs;
  m_reuseFactor = std::min (reuseFactor, MAX_COLORS);
  
  InitializeColorPattern ();
}

void FrequencyReuse::InitializeColorPattern ()
{
  NS_LOG_FUNCTION (this);
  
  // 清空现有映射
  m_beamToColor.clear ();
  m_colorToRbs.clear ();
  m_colorToBeams.clear ();
  
  // 初始化颜色到RB的映射
  // 每个颜色分配 m_totalRbs / m_reuseFactor 个RB
  uint32_t rbsPerColor = m_totalRbs / m_reuseFactor;
  uint32_t remainder = m_totalRbs % m_reuseFactor;
  
  for (uint8_t color = 0; color < m_reuseFactor; ++color) {
      std::vector<uint32_t> rbsForColor;
      uint32_t startRb = color * rbsPerColor;
      uint32_t endRb = startRb + rbsPerColor + (color < remainder ? 1 : 0);
      
      for (uint32_t rb = startRb; rb < endRb && rb < m_totalRbs; ++rb) {
          rbsForColor.push_back (rb);
      }
      
      m_colorToRbs[color] = rbsForColor;
      m_colorToBeams[color] = std::vector<uint32_t> ();
  }
  
  NS_LOG_INFO ("Frequency Reuse Configuration:");
  NS_LOG_INFO ("  Total Bandwidth: " << m_totalBandwidthHz / 1e6 << " MHz");
  NS_LOG_INFO ("  Total RBs: " << m_totalRbs);
  NS_LOG_INFO ("  Reuse Factor: 1/" << (uint32_t)m_reuseFactor);
  NS_LOG_INFO ("  RBs per Color: " << rbsPerColor);
}

uint8_t FrequencyReuse::GetColorForBeam (uint32_t beamId) const
{
  // 简化的颜色分配：beamId mod reuseFactor
  return beamId % m_reuseFactor;
}

std::vector<uint32_t> FrequencyReuse::GetBeamAllocation (uint32_t beamId) const
{
  uint8_t color = GetColorForBeam (beamId);
  
  auto it = m_colorToRbs.find (color);
  if (it != m_colorToRbs.end ()) {
      return it->second;
  }
  
  return std::vector<uint32_t> ();
}

std::vector<uint32_t> FrequencyReuse::GetUeAvailableRbs (uint32_t beamId) const
{
  return GetBeamAllocation (beamId);
}

bool FrequencyReuse::IsRbAvailableAtBeam (uint32_t rbId, uint32_t beamId) const
{
  std::vector<uint32_t> availableRbs = GetBeamAllocation (beamId);
  return std::find (availableRbs.begin (), availableRbs.end (), rbId) != availableRbs.end ();
}

uint8_t FrequencyReuse::GetReuseFactor () const
{
  return m_reuseFactor;
}

uint32_t FrequencyReuse::GetAvailableRbsPerBeam () const
{
  return m_totalRbs / m_reuseFactor;
}

std::vector<uint32_t> FrequencyReuse::GetInterferingBeams (uint32_t beamId) const
{
  std::vector<uint32_t> interfering;
  
  uint8_t beamColor = GetColorForBeam (beamId);
  
  // 与当前波束颜色相同的其他波束会干扰
  auto it = m_colorToBeams.find (beamColor);
  if (it != m_colorToBeams.end ()) {
      for (uint32_t otherBeam : it->second) {
          if (otherBeam != beamId) {
              interfering.push_back (otherBeam);
          }
      }
  }
  
  return interfering;
}

double FrequencyReuse::CalculateReuseGain () const
{
  // 频率复用增益 = 全频复用时的容量 / 1/7复用时的容量
  // 由于同频干扰减少， SINR提升，容量增加
  // 简化计算: 增益 ≈ 10 * log10(7) ≈ 8.45 dB
  double theoreticalGain = 10.0 * std::log10 (m_reuseFactor);
  
  NS_LOG_INFO ("Frequency Reuse Gain: " << theoreticalGain << " dB");
  
  return theoreticalGain;
}

void FrequencyReuse::PrintFrequencyAllocation () const
{
  NS_LOG_INFO ("");
  NS_LOG_INFO ("========================================");
  NS_LOG_INFO ("    7-Color Frequency Reuse Allocation    ");
  NS_LOG_INFO ("========================================");
  
  for (uint8_t color = 0; color < m_reuseFactor; ++color) {
      NS_LOG_INFO ("Color " << (uint32_t)color << ": "
                   << "RBs=" << m_colorToRbs.at (color).size ()
                   << " | Beams=" << m_colorToBeams.at (color).size ());
  }
  
  NS_LOG_INFO ("Available RBs per Beam: " << GetAvailableRbsPerBeam ());
  NS_LOG_INFO ("Reuse Gain: " << CalculateReuseGain () << " dB");
  NS_LOG_INFO ("========================================");
}

} // namespace ns3
