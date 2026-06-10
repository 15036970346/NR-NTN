/*
 * 文件路径: contrib/geo-sat/model/ntn-mac-tags.h
 * 功能: NTN MAC 层数据面用到的 Packet Tag。
 *
 * NtnDciTag: 简化的 DCI/调度元信息, 承载三件事:
 *   - rnti: 这个 Packet 的目标 UE (DL) 或 发包 UE (UL)
 *   - isAck: 上行包是不是 HARQ ACK (仅 isAck=true 时, packet 大小可视为空载)
 *   - effectiveSinrDb: 本次 DL 传输该 UE 的"有效接收 SINR"(dB)。功率域 NOMA 下
 *     near 用户取 SIC 后有效 SINR、far 用户取被 near 干扰后的有效 SINR; 普通用户
 *     取其 CQI 等效 SINR。接收侧 (SatUtMac::DoPhyRx) 用它驱动 NtnPhyErrorModel 的
 *     BLER, 让 NOMA 的可靠性代价真实化。NaN = 未携带 (接收侧沿用错误模型默认 SINR)。
 *
 * 这是一个 MVP 简化, 真实 NR 里 DCI 在独立的 PDCCH 上, 跟 PDSCH 数据时序对齐;
 * 本模型把这几件事压成一个 Tag 直接随 Packet 走。
 */
#ifndef NTN_MAC_TAGS_H
#define NTN_MAC_TAGS_H

#include "ns3/tag.h"
#include "ns3/type-id.h"
#include "ns3/nstime.h"
#include <cmath>
#include <cstdint>
#include <limits>

namespace ns3 {

class NtnDciTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const override;

  NtnDciTag ();
  NtnDciTag (uint16_t rnti, bool isAck);

  virtual void Serialize (TagBuffer i) const override;
  virtual void Deserialize (TagBuffer i) override;
  virtual uint32_t GetSerializedSize (void) const override;
  virtual void Print (std::ostream& os) const override;

  uint16_t GetRnti () const { return m_rnti; }
  void     SetRnti (uint16_t rnti) { m_rnti = rnti; }
  bool     IsAck () const { return m_isAck; }
  void     SetIsAck (bool a) { m_isAck = a; }

  /// 有效接收 SINR(dB)。NaN 表示未携带(接收侧不覆盖错误模型默认 SINR)。
  double   GetEffectiveSinrDb () const { return m_effectiveSinrDb; }
  void     SetEffectiveSinrDb (double s) { m_effectiveSinrDb = s; }
  bool     HasEffectiveSinr () const { return !std::isnan (m_effectiveSinrDb); }

private:
  uint16_t m_rnti  {0};
  bool     m_isAck {false};
  double   m_effectiveSinrDb {std::numeric_limits<double>::quiet_NaN ()};
};

/**
 * \brief PHY→MAC 跨层时延戳: SatPhy 收到包 (Receive 入口) 时打, MAC DoPhyRx 入口
 *        读出算 delay = Now - tag.t。GEO 场景这通常 ≈ m_processingDelay (1µs), 但
 *        如果 MAC 调度有 backlog 会变大, 这个 trace 让运维能观察。
 */
class NtnPhyDeliveryTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const override;
  NtnPhyDeliveryTag ();
  explicit NtnPhyDeliveryTag (Time t);

  virtual void Serialize (TagBuffer i) const override;
  virtual void Deserialize (TagBuffer i) override;
  virtual uint32_t GetSerializedSize (void) const override;
  virtual void Print (std::ostream& os) const override;

  Time GetPhyRxTime () const { return m_t; }
private:
  Time m_t;
};

/**
 * \brief PHY→MAC per-packet 接收 SINR 戳 (信道实算)。
 *
 * 端点 PHY (UE 侧 SatPhy) 在 Receive() 入口, 用本次收到信号的 PSD 总功率与本端
 * 热噪声 (k·T·B·NF) 算出 per-packet 接收 SINR(dB), 打到 packet 上, 供 MAC
 * (SatUtMac::DoPhyRx) 作为 BLER 的物理基准。这反映 PSD 功率切分 (NOMA β) + 弯管
 * 增益等"信道能看见"的部分。
 *
 * 与 NtnDciTag::effectiveSinrDb 的分工 (见 SatUtMac::DoPhyRx 注释):
 *   - 本 tag = 信道按 PSD 实算的接收 SINR (物理基准, 非 NOMA 用户直接用)。
 *   - effectiveSinrDb = NOMA SIC残差/配对干扰修正 (单 PSD-per-packet 模型里信道看不到
 *     同 RB 叠加干扰, 由调度器/链路预算算出, 对 NOMA 标记的包覆盖物理基准)。
 *
 * NaN = 未携带 (接收侧退回错误模型自身 SINR, 向后兼容)。
 */
class NtnRxSinrTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const override;

  NtnRxSinrTag ();
  explicit NtnRxSinrTag (double rxSinrDb);

  virtual void Serialize (TagBuffer i) const override;
  virtual void Deserialize (TagBuffer i) override;
  virtual uint32_t GetSerializedSize (void) const override;
  virtual void Print (std::ostream& os) const override;

  double GetRxSinrDb () const { return m_rxSinrDb; }
  void   SetRxSinrDb (double s) { m_rxSinrDb = s; }
  bool   HasRxSinr () const { return !std::isnan (m_rxSinrDb); }

private:
  double m_rxSinrDb {std::numeric_limits<double>::quiet_NaN ()};
};

/**
 * \brief Beam 路由 Tag。
 *
 * GW 端在共享 feeder channel 上发 DL packet 时打此 tag, 卫星侧每束 feederPhy
 * 收到 packet 后比对自身 beamId, 只有匹配才转发到对应 user channel,
 * 否则丢弃, 防止 N 束在共享 feeder channel 上各自全转发产生 fan-out。
 *
 * 没打 tag 的 packet 走"兼容路径":卫星侧 feederPhy 直接转发 (用于单波束 demo)。
 */
class NtnBeamIdTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const override;

  NtnBeamIdTag ();
  explicit NtnBeamIdTag (uint16_t beamId);

  virtual void Serialize (TagBuffer i) const override;
  virtual void Deserialize (TagBuffer i) override;
  virtual uint32_t GetSerializedSize (void) const override;
  virtual void Print (std::ostream& os) const override;

  uint16_t GetBeamId () const { return m_beamId; }
  void     SetBeamId (uint16_t b) { m_beamId = b; }

private:
  uint16_t m_beamId {0};
};

} // namespace ns3

#endif /* NTN_MAC_TAGS_H */
