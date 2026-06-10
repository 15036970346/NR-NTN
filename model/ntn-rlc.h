/*
 * 文件: contrib/geo-sat/model/ntn-rlc.h
 *
 * NTN RLC 层实体 (单文件实现, TM/UM/AM 三模式)
 *
 *   - TM: 透明直通, 无 SN, 无重组, 无重传
 *   - UM: 加 SN, 接收侧按 SN 单调推进, 丢失/乱序直接丢, 无重传
 *   - AM: 加 SN, 接收侧按概率丢包模拟丢失, 周期性 Status PDU (NACK 列表)
 *         触发发送侧重传, 精确累计 m_retxCount
 *
 * 接口与现有 SatRlcStatsCollector::DlTx/RxPdu 签名对齐, demo 直接接 trace 即可。
 * 不真分片 (假设 PDCP SDU < MAC TB), 不接 PDCP/MAC (本期 RLC 独立验证)。
 */
#ifndef NTN_RLC_H
#define NTN_RLC_H

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/tag.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"
#include "ns3/callback.h"
#include "ns3/random-variable-stream.h"
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace ns3 {

enum class NtnRlcMode : uint8_t { TM = 0, UM = 1, AM = 2 };

/**
 * \brief RLC PDU header tag: 携带 SN 和 isStatusPdu 标志。
 *
 * 用 PacketTag (随 packet copy 走) 替代真实 RLC header, 简化但语义足够:
 *   - dataPdu: sn = RLC SDU 序号, isStatusPdu=false
 *   - statusPdu: 携带一组 NACK SN (序列化到 packet payload), isStatusPdu=true
 */
class NtnRlcSnTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const override;
  NtnRlcSnTag ();
  NtnRlcSnTag (uint16_t sn, bool isStatusPdu);

  virtual void Serialize (TagBuffer i) const override;
  virtual void Deserialize (TagBuffer i) override;
  virtual uint32_t GetSerializedSize (void) const override;
  virtual void Print (std::ostream& os) const override;

  uint16_t GetSn () const { return m_sn; }
  void     SetSn (uint16_t sn) { m_sn = sn; }
  bool     IsStatusPdu () const { return m_isStatusPdu; }
  void     SetIsStatusPdu (bool v) { m_isStatusPdu = v; }

private:
  uint16_t m_sn        {0};
  bool     m_isStatusPdu {false};
};

/**
 * \brief Tx 时间戳 tag, Rx 端读出算端到端 delay (sub-ns 精度).
 */
class NtnRlcTimestampTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const override;
  NtnRlcTimestampTag ();
  explicit NtnRlcTimestampTag (Time t);

  virtual void Serialize (TagBuffer i) const override;
  virtual void Deserialize (TagBuffer i) override;
  virtual uint32_t GetSerializedSize (void) const override;
  virtual void Print (std::ostream& os) const override;

  Time GetTxTime () const { return m_t; }
private:
  Time m_t;
};

/**
 * \brief NTN RLC 实体. 一个实例服务一个 RB (rnti + lcid).
 *        TM/UM/AM 三种模式由 SetMode 切换。
 */
class NtnRlc : public Object
{
public:
  static TypeId GetTypeId (void);
  NtnRlc ();
  virtual ~NtnRlc ();

  // ===== Identity =====
  void SetRnti (uint16_t rnti);
  void SetLcid (uint8_t  lcid);
  void SetMode (NtnRlcMode mode);
  NtnRlcMode GetMode () const { return m_mode; }
  std::string GetModeStr () const;

  // ===== Upper SAP (PDCP -> RLC) =====
  /// 上层把 PDCP PDU 推下来, RLC 加 header (UM/AM) 或直透 (TM) 后下推到 transmitCb
  void TransmitPdcpPdu (Ptr<Packet> p);
  /// RLC 重组后递交给 PDCP 的回调
  void SetReceiveCallback (Callback<void, Ptr<Packet>> cb);

  // ===== Lower SAP (MAC -> RLC) =====
  /// 下层 (MAC) 投递 RLC PDU 上来
  void ReceiveRlcPdu (Ptr<Packet> p);
  /// RLC 下推 RLC PDU 用的回调 (上层注入: 通常是 MAC->SendXxx 的桩)
  void SetTransmitCallback (Callback<void, Ptr<Packet>> cb);

  // ===== AM tuning =====
  void SetAmDropProb (double p);
  double GetAmDropProb () const { return m_amDropProb; }
  void SetPollRetransmitTimer (Time t);
  void SetStatusProhibitTimer (Time t);

  // ===== Stats / 查询 =====
  uint32_t GetTxPduCount   () const { return m_txPduCount; }
  uint32_t GetRxPduCount   () const { return m_rxPduCount; }
  uint32_t GetExactRetxCount () const { return m_retxCount; }
  uint64_t GetTxBytes      () const { return m_txBytes; }
  uint64_t GetRxBytes      () const { return m_rxBytes; }

  // ===== Trace 接口 (签名跟 SatRlcStatsCollector 对齐) =====
  // m_txTrace (rnti, lcid, size)            <- 所有 Tx (含重传)
  // m_rxTrace (rnti, lcid, size, delayNs)   <- 成功递交给上层的 Rx
  // m_retxTrace (rnti, lcid, size)          <- 显式重传发生 (AM 专用)
  TracedCallback<uint16_t, uint8_t, uint32_t>           m_txTrace;
  TracedCallback<uint16_t, uint8_t, uint32_t, uint64_t> m_rxTrace;
  TracedCallback<uint16_t, uint8_t, uint32_t>           m_retxTrace;

protected:
  virtual void DoDispose () override;

private:
  // ----- TM -----
  void TmTransmit  (Ptr<Packet> p);
  void TmReceive   (Ptr<Packet> p);
  // ----- UM -----
  void UmTransmit  (Ptr<Packet> p);
  void UmReceive   (Ptr<Packet> p);
  // ----- AM -----
  void AmTransmit  (Ptr<Packet> p);                   // 首次发
  void AmReceive   (Ptr<Packet> p);                   // 下层投递
  void AmReceiveDataPdu   (Ptr<Packet> p, uint16_t sn);
  void AmReceiveStatusPdu (Ptr<Packet> p);            // 收到 NACK 列表 -> 重传
  void AmScheduleStatusReport ();                     // 接收端周期触发
  void AmSendStatusReport ();
  void AmPollRetransmitTimeout ();                    // 发送端无 ACK 兜底
  void AmDoTransmit (Ptr<Packet> p, uint16_t sn, bool isRetx);
  void DeliverToUpper (Ptr<Packet> p);                // 共用上送 + rxTrace

  // ----- 共用 -----
  uint16_t   m_rnti  {0};
  uint8_t    m_lcid  {0};
  NtnRlcMode m_mode  {NtnRlcMode::UM};

  Callback<void, Ptr<Packet>> m_recvCb;
  Callback<void, Ptr<Packet>> m_txCb;

  // 统计
  uint64_t m_txBytes     {0};
  uint64_t m_rxBytes     {0};
  uint32_t m_txPduCount  {0};  // 含重传
  uint32_t m_rxPduCount  {0};
  uint32_t m_retxCount   {0};

  // ----- UM 接收侧 -----
  uint16_t m_umNextExpectedSn {0};

  // ----- AM 接收侧 -----
  uint16_t                            m_amRxNextSn         {0};
  std::map<uint16_t, Ptr<Packet>>     m_amReorderBuffer;
  std::set<uint16_t>                  m_amMissingSn;        // 待 NACK
  Time                                m_amStatusProhibit   {MilliSeconds (50)};
  EventId                             m_amStatusEvent;
  bool                                m_amStatusPending    {false};

  // ----- AM 发送侧 -----
  uint16_t                            m_amTxNextSn         {0};
  std::map<uint16_t, Ptr<Packet>>     m_amTxBuffer;         // sn -> 原 SDU (重传源)
  Time                                m_amPollRetransmit   {MilliSeconds (200)};
  EventId                             m_amPollEvent;

  // 丢包模拟
  double                              m_amDropProb         {0.0};
  Ptr<UniformRandomVariable>          m_dropRng;
};

} // namespace ns3

#endif /* NTN_RLC_H */
