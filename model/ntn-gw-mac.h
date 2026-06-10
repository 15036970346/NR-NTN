/*
 * 文件路径：contrib/geo-sat/model/ntn-gw-mac.h
 * 功能：GEO 卫星信关站 MAC 层实体 (负责系统广播与周期性传输)
 */
#ifndef NTN_GW_MAC_H
#define NTN_GW_MAC_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/spectrum-model.h"
#include "sat-mac-common.h"
#include "ntn-gw-phy.h"
#include "ntn-mac-tags.h"
#include "ntn-rlc.h"
#include "ntn-phy-error-model.h"
#include "ns3/traced-callback.h"
#include <vector>
#include <map>
#include <deque>

namespace ns3 {

// 前置声明: 避免 ntn-gw-mac.h 与 geo-beam-scheduler.h 循环包含 (在 .cc 里 include)。
class GeoBeamScheduler;

class NtnGwMac : public Object
{
public:
  static TypeId GetTypeId (void);
  NtnGwMac ();
  virtual ~NtnGwMac ();

  // 设置当前信关站需要覆盖的活跃波束列表
  void SetActiveBeams (const std::vector<uint32_t>& beams);

  // 核心功能：启动周期性广播传输 (SSB/SIB1)
  void StartPeriodicTransmissions ();

  // 停止广播 (用于安全释放资源)
  void StopPeriodicTransmissions ();

  // ==================== SSB同步信号块传输 ====================
  // 生成并发送完整的SSB (PSS + SSS + PBCH)
  void TransmitSsbBlock (uint32_t beamId);

  // 生成PSS序列 (主同步信号)
  std::vector<uint32_t> GeneratePssSequence (uint32_t pci);

  // 生成SSS序列 (辅同步信号)
  std::vector<uint32_t> GenerateSssSequence (uint32_t pci);

  // 生成PBCH负载 (承载MIB信息)
  Ptr<Packet> GeneratePbchPayload (uint32_t beamId, uint8_t ssbIndex);

  // 计算物理小区ID (PCI) = N_ID_1 * 3 + N_ID_2
  uint32_t CalculatePci (uint32_t nId1, uint32_t nId2);

  // 获取当前SFN (系统帧号)
  uint16_t GetCurrentSfn () const;

  // ============================================================
  // 3B: 数据面接口 (MAC <-> PHY 上下交互)
  // ============================================================
  /// 绑定底层 PHY (馈电链路侧, GW 节点专属)
  void SetPhy (Ptr<NtnGwPhy> phy);

  /// 配置 PHY 发送时所需的参数 (SpectrumModel/带宽/功率/时长), 一次配置长期复用
  void SetTxConfig (Ptr<const SpectrumModel> spectrumModel,
                    double bandwidthHz,
                    double txPowerDbm,
                    Time   duration);

  /// 下行: 单 UE 早期接口 (无 DCI tag), 上层把 packet 直接经 PHY 推到馈电信道
  void SendDlPdu (Ptr<Packet> packet);

  /// 上行: 注册回调, 收到 PHY 上送的上行用户数据时触发
  void SetUlPduCallback (Callback<void, Ptr<Packet>> cb);

  // ============================================================
  // 多 UE 调度 (MVP):
  //  - AddUe(rnti) 把 UE 加入调度器视野, 同时建一个 DL FIFO 队列
  //  - EnqueueDlPdu(rnti, pkt) 让上层往某个 UE 的 DL 队列塞包
  //  - StartScheduler(period) 启周期性轮询, 每个 TTI 选下一个有包的 UE 发 1 包,
  //    自动给 packet 打 NtnDciTag(rnti, isAck=false), 经 PHY SendPdu 下发
  //  - DoPhyRx 区分上行数据 vs HARQ ACK: 是 ACK 则计入 m_ackCount[rnti],
  //    是数据则按原 ulCallback 上送上层
  // ============================================================
  void AddUe (uint16_t rnti);
  /// 多波束版: 同时登记 UE 所属 beam。beamId != 0 时, 该 UE 的 DL 包会被自动打上
  /// NtnBeamIdTag(beamId), 让卫星侧 feederPhy 按 beam 路由 (共享 feeder channel + per-beam user channel)。
  /// beamId == 0 表示不打 tag, 走单波束兼容路径。
  void AddUe (uint16_t rnti, uint16_t beamId);
  void EnqueueDlPdu (uint16_t rnti, Ptr<Packet> packet);
  void StartScheduler (Time period);
  void StopScheduler ();

  /// HARQ ACK 累计 (供 demo / 测试统计)
  uint32_t GetAckCount (uint16_t rnti) const;
  std::vector<uint16_t> GetRegisteredUes () const;

  // ============================================================
  // 全栈集成
  // ============================================================
  /// 注册某 UE 的 UL RLC: DoPhyRx 收到 user data 后按 NtnDciTag.rnti 查表投递 rlc->ReceiveRlcPdu
  void RegisterUlRlc (uint16_t rnti, Ptr<NtnRlc> rlc);
  /// PHY 错误模型: 接收侧对 user data 概率 drop (HARQ ACK 不丢)
  void SetPhyErrorModel (Ptr<NtnPhyErrorModel> em);

  // ============================================================
  // 功率域 NOMA: 下行有效 SINR 注入 (项 B 数据面桥)
  //  调度器(GeoBeamScheduler::GetEffectiveDlSinrDb)算出 NOMA near/far 与普通用户的
  //  本轮下行有效 SINR 后, 由集成层(或调度器侧)调用 SetEffectiveDlSinrDb 写进此处;
  //  SendDlPdu/SendDlPduForRnti 发某 UE 的 DL 数据时把该值打进 NtnDciTag.effectiveSinrDb,
  //  UE 侧 SatUtMac::DoPhyRx 据此驱动 NtnPhyErrorModel 的 BLER (让 NOMA 干扰代价真实化)。
  //  未设置则不打 SINR (接收侧沿用错误模型默认)。
  // ============================================================
  void SetEffectiveDlSinrDb (uint16_t rnti, double sinrDb);
  void ClearEffectiveDlSinrDb (uint16_t rnti);

  // ============================================================
  // 项 A: 每-rnti 下行发射功率 (DCI 功率真正进物理 PSD)
  //  调度器把 NOMA β 切分(NomaNear/FarTxPowerDbm)与 DL 动态功率(ComputeDlTxPowerDbm)
  //  算出的发射功率(dBm)交给此处; SendDlPduForRnti 构造 PSD 时, 若该 rnti 有 per-rnti
  //  功率则用它替代固定 m_txPowerDbm, 让 β 切分/动态功率真正改变发射 (进而改变接收 SINR)。
  //  未设置则回退 m_txPowerDbm。
  // ============================================================
  void SetDlTxPowerDbm (uint16_t rnti, double dbm);
  void ClearDlTxPowerDbm (uint16_t rnti);

  // ============================================================
  // 项 B: 调度器 → gw-mac 单一真源自动接线
  //  持有调度器的弱引用; 接线后, SendDlPduForRnti/StampDlDciTag 自动从调度器取
  //  最近一次该 rnti 的 {有效SINR, 发射功率} (GetLastDlEffSinrDb / GetLastDlTxPowerDbm),
  //  作为 NtnDciTag.effectiveSinrDb 与 PSD 发射功率, 覆盖手工 map 的值。接线后无需任何
  //  手工 SetEffectiveDlSinrDb / SetDlTxPowerDbm 调用。手工 API 保留作向后兼容回退。
  // ============================================================
  void SetScheduler (Ptr<GeoBeamScheduler> sched);

private:
  // 内部发包逻辑：生成广播包并打上波束 ID，发往物理层
  void SendBroadcastMacPdu (uint32_t beamId, bool isSsb);

  // 发送SSB的各个组成部分
  void SendPss (uint32_t beamId, uint32_t pci);
  void SendSss (uint32_t beamId, uint32_t pci);
  void SendPbch (uint32_t beamId, uint32_t pci, uint8_t ssbIndex);

  // PHY 上送上行包的内部入口 (绑到 m_phy->SetRxCallback)
  void DoPhyRx (Ptr<Packet> packet);

  Time m_ssbPeriod;                         // 广播周期 (例如 20ms)
  EventId m_periodicBroadcastEvent;         // 维护定时器句柄，防止事件失控
  std::vector<uint32_t> m_activeBeamIds;    // 活跃波束列表
  uint16_t m_systemFrameNumber;              // 当前系统帧号 (SFN)
  uint8_t m_ssbIndex;                       // SSB索引 (0-63)

  // ---------- 数据面字段 ----------
  Ptr<NtnGwPhy>            m_phy;
  Ptr<const SpectrumModel> m_txSpectrumModel;
  double                   m_txBandwidthHz;
  double                   m_txPowerDbm;
  Time                     m_txDuration;
  Callback<void, Ptr<Packet>> m_ulCallback;

  // ---------- 多 UE 调度运行时状态 ----------
  void ScheduleTick ();                                      // 每 TTI 调用, 轮询发一个包
  void SendDlPduForRnti (uint16_t rnti, Ptr<Packet> packet); // 给 packet 打 tag + PHY 下发

  std::vector<uint16_t>                        m_registeredUes;   // 调度轮询顺序
  std::map<uint16_t, std::deque<Ptr<Packet>>>  m_dlQueues;        // 每 UE 一个 DL FIFO
  std::map<uint16_t, uint32_t>                 m_ackCount;        // 每 UE HARQ ACK 累计
  std::map<uint16_t, uint16_t>                 m_ueBeam;          // rnti -> beamId (0 表示不打 BeamIdTag)
  size_t                                       m_rrCursor {0};    // 轮询游标
  Time                                         m_schedPeriod;
  EventId                                      m_schedEvent;

  // 全栈集成: rnti -> UL RLC, 收 UL 时按 tag.rnti 投递
  std::map<uint16_t, Ptr<NtnRlc>>              m_ulRlc;
  Ptr<NtnPhyErrorModel>                        m_phyErr;

  // 功率域 NOMA: rnti -> 本轮下行有效 SINR(dB)。打 NtnDciTag 时随包带给 UE 驱动 BLER。
  std::map<uint16_t, double>                   m_effectiveDlSinrDb;
  // 项 A: rnti -> 下行发射功率(dBm)。构造 PSD 时若有则替代固定 m_txPowerDbm。
  std::map<uint16_t, double>                   m_dlTxPowerDbm;
  // 项 B: 调度器弱引用 (单一真源自动接线)。前置声明 + 在 .cc include 头文件。
  Ptr<GeoBeamScheduler>                        m_scheduler;
  // 给 packet 打 DCI tag 时附带有效 SINR(若已登记)。供 SendDlPdu/SendDlPduForRnti 复用。
  void StampDlDciTag (Ptr<Packet> packet, uint16_t rnti, bool isAck);
  // 解析某 rnti 本次 DL 的发射功率(dBm): 自动接线优先 -> per-rnti map -> 固定回退。
  double ResolveDlTxPowerDbm (uint16_t rnti) const;

  // ===== 统计 TracedCallbacks =====
  // (rnti, lcid, rlcSize, macPduSize): MAC 下行复用; 当前 1:1 映射时 macPduSize==rlcSize+tag, 暴露便于上层观察 mux 开销
  TracedCallback<uint16_t, uint8_t, uint32_t, uint32_t> m_txMacPduTrace;
  // (rnti, lcid, macPduSize, sduSize): UL 解复用上送
  TracedCallback<uint16_t, uint8_t, uint32_t, uint32_t> m_rxMacPduTrace;
  // GW 端 DL 队列长度 (rnti, bufferBytes)
  TracedCallback<uint16_t, uint32_t>                    m_dlQueueLengthTrace;
  // GW 端 DL 队列时延 (rnti, dequeue_delay_ns)
  TracedCallback<uint16_t, int64_t>                     m_dlQueueDelayTrace;
  // PHY→MAC 跨层时延 (rnti, delay_ns) -- 从 SatPhy::Receive 到 MAC::DoPhyRx
  TracedCallback<uint16_t, int64_t>                     m_phyMacDelayTrace;

  // 每个 DL 队列的入队时间, 跟 m_dlQueues 对齐
  std::map<uint16_t, std::deque<Time>>          m_dlEnqueueTimes;
};

} // namespace ns3

#endif /* NTN_GW_MAC_H */