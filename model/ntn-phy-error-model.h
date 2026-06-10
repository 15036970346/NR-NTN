/*
 * 文件: contrib/geo-sat/model/ntn-phy-error-model.h
 *
 * NTN PHY 错误模型: SINR -> BLER 简化查表 + 概率 drop。
 *
 * 用法:
 *   Ptr<NtnPhyErrorModel> em = CreateObject<NtnPhyErrorModel> ();
 *   em->SetSinrDb (8.0);                  // 静态注入 (demo)
 *   // 或 em->SetSinrProvider (cb);       // 动态查询 (从 SatUtPhy 实时取)
 *   if (em->ShouldDrop ()) drop_packet;
 *
 * 接入 MAC: gwMac/utMac->SetPhyErrorModel(em),DoPhyRx 入口检查 ShouldDrop()。
 * 仅对 user data (NtnDciTag.isAck=false) 起作用,HARQ ACK 视为控制信令不丢。
 */
#ifndef NTN_PHY_ERROR_MODEL_H
#define NTN_PHY_ERROR_MODEL_H

#include "ns3/object.h"
#include "ns3/callback.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ptr.h"

namespace ns3 {

class NtnPhyErrorModel : public Object
{
public:
  static TypeId GetTypeId (void);
  NtnPhyErrorModel ();
  virtual ~NtnPhyErrorModel ();

  /**
   * \brief 5G NR 简化 BLER 查表 (line segments):
   *   SINR < -5 dB           -> BLER 1.0   (基本全丢)
   *   -5 dB <= SINR <  0 dB  -> 线性 1.0 -> 0.5
   *    0 dB <= SINR <  5 dB  -> 线性 0.5 -> 0.1
   *    5 dB <= SINR < 10 dB  -> 线性 0.1 -> 0.01
   *   SINR >= 10 dB          -> BLER 0.001
   * 这只是定性曲线, 用于"用 SINR 驱动重传"的演示。
   */
  static double BlerFromSinr (double sinrDb);

  /// 静态注入 (适合 demo / 离线分析)。一旦被显式调用, HasExplicitSinr() 返回 true,
  /// 表示调用方刻意注入了一条链路/损伤 SINR (例如无路损拓扑里手工设的低 SINR),
  /// 接收侧应优先沿用它而非信道实算 SINR (见 SatUtMac::DoPhyRx 优先级注释)。
  void   SetSinrDb (double sinrDb);
  double GetSinrDb () const;

  /// 调用方是否显式 SetSinrDb 过 (区分"属性默认 20dB"与"用户刻意注入")
  bool   HasExplicitSinr () const { return m_sinrExplicitlySet; }

  /// 动态查询 (例如把 SatUtPhy::GetLastCalculatedSinr 包成 callback)
  void   SetSinrProvider (Callback<double> p);

  /// 当前 BLER (查表结果, 不重投骰子)
  double GetBler () const;

  /// 每次 packet 抵达时调一次: 内部投骰子决定是否丢
  bool ShouldDrop ();

  /// 计数器 (供统计)
  uint32_t GetDropCount () const { return m_dropCount; }
  uint32_t GetCheckCount () const { return m_checkCount; }

private:
  double                     m_sinrDb        {20.0};   // 默认很高 -> BLER ~ 0
  bool                       m_sinrExplicitlySet {false}; // SetSinrDb 是否被显式调用过
  bool                       m_useProvider   {false};
  Callback<double>           m_sinrProvider;
  Ptr<UniformRandomVariable> m_rng;

  uint32_t                   m_checkCount    {0};
  uint32_t                   m_dropCount     {0};
};

} // namespace ns3

#endif /* NTN_PHY_ERROR_MODEL_H */
