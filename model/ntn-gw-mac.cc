/*
 * 文件路径：contrib/geo-sat/model/ntn-gw-mac.cc
 * 功能：实现信关站 SSB/SIB1 周期广播; 3B 起增加数据面 SendDlPdu + 上行回调 hook。
 */
#include "ntn-gw-mac.h"
#include "ntn-rlc.h"
#include "ntn-phy-error-model.h"
#include "geo-beam-scheduler.h"   // 项 B: 自动接线需要 GeoBeamScheduler 完整定义 (头里仅前置声明)
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/spectrum-value.h"
#include "ns3/spectrum-signal-parameters.h"
#include "sat-phy.h"
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NtnGwMac");
NS_OBJECT_ENSURE_REGISTERED (NtnGwMac);

TypeId NtnGwMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NtnGwMac")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<NtnGwMac> ()
    .AddAttribute ("SsbPeriod",
                   "Periodicity of SSB and SIB1 broadcast in NTN.",
                   TimeValue (MilliSeconds (20)),
                   MakeTimeAccessor (&NtnGwMac::m_ssbPeriod),
                   MakeTimeChecker ())
    .AddTraceSource ("TxMacPdu",
                     "DL MAC PDU 复用打包发送 (rnti, lcid, rlcSize, macPduSize)",
                     MakeTraceSourceAccessor (&NtnGwMac::m_txMacPduTrace),
                     "ns3::TracedCallback::Uint16Uint8Uint32Uint32")
    .AddTraceSource ("RxMacPdu",
                     "UL MAC PDU 解复用上送 (rnti, lcid, macPduSize, sduSize)",
                     MakeTraceSourceAccessor (&NtnGwMac::m_rxMacPduTrace),
                     "ns3::TracedCallback::Uint16Uint8Uint32Uint32")
    .AddTraceSource ("DlQueueLength",
                     "DL 队列长度 (rnti, bufferBytes)",
                     MakeTraceSourceAccessor (&NtnGwMac::m_dlQueueLengthTrace),
                     "ns3::TracedCallback::Uint16Uint32")
    .AddTraceSource ("DlQueueDelay",
                     "DL 出队时延 (rnti, delay_ns)",
                     MakeTraceSourceAccessor (&NtnGwMac::m_dlQueueDelayTrace),
                     "ns3::TracedCallback::Uint16Int64")
    .AddTraceSource ("PhyMacDelay",
                     "PHY→MAC 跨层时延 (rnti, delay_ns)",
                     MakeTraceSourceAccessor (&NtnGwMac::m_phyMacDelayTrace),
                     "ns3::TracedCallback::Uint16Int64");
  return tid;
}

NtnGwMac::NtnGwMac ()
  : m_txBandwidthHz (100e6),
    m_txPowerDbm (40.0),
    m_txDuration (MilliSeconds (1))
{
  NS_LOG_FUNCTION (this);
  m_ssbPeriod = MilliSeconds (20);
  m_systemFrameNumber = 0;
  m_ssbIndex = 0;
}

NtnGwMac::~NtnGwMac () {}

void NtnGwMac::SetActiveBeams (const std::vector<uint32_t>& beams)
{
  NS_LOG_FUNCTION (this << beams.size());
  m_activeBeamIds = beams;
}

void NtnGwMac::StartPeriodicTransmissions ()
{
  NS_LOG_FUNCTION (this);

  for (uint32_t beamId : m_activeBeamIds)
    {
      NS_LOG_DEBUG ("Generating Broadcast Msgs for Beam " << beamId 
                    << " at Time: " << Simulator::Now ().GetSeconds () << "s");

      TransmitSsbBlock (beamId);
      SendBroadcastMacPdu (beamId, false /* SIB1 */);
    }

  // 更新SFN (每20ms增加)
  m_systemFrameNumber = (m_systemFrameNumber + 1) % 1024;
  m_ssbIndex = (m_ssbIndex + 1) % 64;

  m_periodicBroadcastEvent = Simulator::Schedule (m_ssbPeriod, 
                                                  &NtnGwMac::StartPeriodicTransmissions, 
                                                  this);
}

void NtnGwMac::StopPeriodicTransmissions ()
{
  NS_LOG_FUNCTION (this);
  if (m_periodicBroadcastEvent.IsRunning ())
    {
      Simulator::Cancel (m_periodicBroadcastEvent);
    }
}

// ==================== SSB同步信号块实现 ====================

void NtnGwMac::TransmitSsbBlock (uint32_t beamId)
{
  NS_LOG_FUNCTION (this << beamId);
  
  // 计算PCI (物理小区ID) = N_ID_1 * 3 + N_ID_2
  // N_ID_1: 0-335 (小区组ID)
  // N_ID_2: 0-2 (小区组内ID)
  uint32_t nId1 = beamId % 336;
  uint32_t nId2 = 0;
  uint32_t pci = CalculatePci (nId1, nId2);
  
  NS_LOG_INFO ("=== SSB传输开始 ===");
  NS_LOG_INFO ("Beam " << beamId << " | PCI=" << pci 
               << " (N_ID_1=" << nId1 << ", N_ID_2=" << nId2 << ")");
  NS_LOG_INFO ("SFN=" << m_systemFrameNumber << " | SSB_Index=" << (uint32_t)m_ssbIndex);
  
  // 按顺序发送: PSS -> SSS -> PBCH (在一个SSB时隙内)
  SendPss (beamId, pci);
  Simulator::Schedule (MicroSeconds (4), &NtnGwMac::SendSss, this, beamId, pci);
  Simulator::Schedule (MicroSeconds (8), &NtnGwMac::SendPbch, this, beamId, pci, m_ssbIndex);
  
  NS_LOG_INFO ("=== SSB传输完成 ===");
}

uint32_t NtnGwMac::CalculatePci (uint32_t nId1, uint32_t nId2)
{
  // PCI = N_ID_1 * 3 + N_ID_2 (3GPP 38.211)
  return nId1 * 3 + nId2;
}

std::vector<uint32_t> NtnGwMac::GeneratePssSequence (uint32_t pci)
{
  // PSS序列 (主同步信号)
  // 3GPP 38.211 Section 7.4.2.1
  // PSS基于长度为127的m序列，N_ID_2决定使用哪个根序列
  std::vector<uint32_t> pssSequence (127);
  
  uint32_t nId2 = pci % 3;  // 从PCI提取N_ID_2
  
  // 简化的PSS生成 (实际应使用m序列生成器)
  for (uint32_t i = 0; i < 127; ++i) {
      pssSequence[i] = (i + nId2 * 43) % 127;
  }
  
  NS_LOG_DEBUG ("PSS Generated for N_ID_2=" << nId2 << " (PCI=" << pci << ")");
  return pssSequence;
}

std::vector<uint32_t> NtnGwMac::GenerateSssSequence (uint32_t pci)
{
  // SSS序列 (辅同步信号)
  // 3GPP 38.211 Section 7.4.2.2
  // SSS基于两个长度为127的m序列，N_ID_1决定使用哪个序列对
  std::vector<uint32_t> sssSequence (127);
  
  uint32_t nId1 = pci / 3;  // 从PCI提取N_ID_1
  
  // 简化的SSS生成 (实际应使用m序列生成器)
  for (uint32_t i = 0; i < 127; ++i) {
      sssSequence[i] = (i + nId1 * 31) % 127;
  }
  
  NS_LOG_DEBUG ("SSS Generated for N_ID_1=" << nId1 << " (PCI=" << pci << ")");
  return sssSequence;
}

Ptr<Packet> NtnGwMac::GeneratePbchPayload (uint32_t beamId, uint8_t ssbIndex)
{
  // PBCH负载 (物理广播信道)
  // 3GPP 38.212 Section 7.1
  // 承载MIB (Master Information Block) 信息
  // MIB包含: SFN, subCarrierSpacingCommon, ssbSubcarrierOffset, dmrs-TypeA-Position, etc.
  
  Ptr<Packet> pbchPayload = Create<Packet> (56);  // PBCH负载约56bytes
  
  NS_LOG_DEBUG ("PBCH Payload Generated for Beam " << beamId 
                << " | SSB_Index=" << (uint32_t)ssbIndex
                << " | SFN=" << m_systemFrameNumber
                << " | 包含MIB信息: 系统帧号、子载波间隔等");
  
  return pbchPayload;
}

void NtnGwMac::SendPss (uint32_t beamId, uint32_t pci)
{
  std::vector<uint32_t> pssSeq = GeneratePssSequence (pci);
  
  NS_LOG_INFO ("[SSB-PSS] Beam " << beamId 
               << " | 发送主同步信号 | PCI_N_ID_2=" << (pci % 3)
               << " | 序列长度=" << pssSeq.size ());
  
  // TODO: 调用物理层SAP发送PSS
  // m_macPhySapProvider->TransmitPss (pssSeq, beamId);
}

void NtnGwMac::SendSss (uint32_t beamId, uint32_t pci)
{
  std::vector<uint32_t> sssSeq = GenerateSssSequence (pci);
  
  NS_LOG_INFO ("[SSB-SSS] Beam " << beamId 
               << " | 发送辅同步信号 | PCI_N_ID_1=" << (pci / 3)
               << " | 序列长度=" << sssSeq.size ());
  
  // TODO: 调用物理层SAP发送SSS
  // m_macPhySapProvider->TransmitSss (sssSeq, beamId);
}

void NtnGwMac::SendPbch (uint32_t beamId, uint32_t pci, uint8_t ssbIndex)
{
  Ptr<Packet> pbchPkt = GeneratePbchPayload (beamId, ssbIndex);
  
  NS_LOG_INFO ("[SSB-PBCH] Beam " << beamId 
               << " | 发送物理广播信道 | PCI=" << pci
               << " | SSB_Index=" << (uint32_t)ssbIndex
               << " | SFN=" << m_systemFrameNumber);
  
  // TODO: 调用物理层SAP发送PBCH
  // m_macPhySapProvider->TransmitPbch (pbchPkt, beamId, pci, ssbIndex);
}

void NtnGwMac::SendBroadcastMacPdu (uint32_t beamId, bool isSsb)
{
  uint32_t pduSize = isSsb ? 56 : 250; 
  Ptr<Packet> pdu = Create<Packet> (pduSize);

  // =================================================================
  // [优化点 2] 向下层物理层发送 PDU 的真实接口预留
  // TODO: 后续需调用底层物理层发送接口 (SAP)，例如：
  // m_macPhySapProvider->TransmitPdu (pdu, beamId);
  // =================================================================
  
  static uint32_t count = 0;
  if (count % 10 == 0) {
      NS_LOG_INFO ("Broadcast: Beam " << beamId << " " << (isSsb ? "SSB" : "SIB1"));
  }
  count++;
}

uint16_t NtnGwMac::GetCurrentSfn () const
{
  return m_systemFrameNumber;
}

// =====================================================================
// 3B: 数据面接口
// =====================================================================

void
NtnGwMac::SetPhy (Ptr<NtnGwPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  m_phy = phy;
  if (m_phy)
    {
      // 把 PHY 上送的包统一收口到 DoPhyRx
      m_phy->SetRxCallback (MakeCallback (&NtnGwMac::DoPhyRx, this));
    }
}

void
NtnGwMac::SetTxConfig (Ptr<const SpectrumModel> spectrumModel,
                       double bandwidthHz,
                       double txPowerDbm,
                       Time   duration)
{
  NS_LOG_FUNCTION (this << bandwidthHz << txPowerDbm << duration);
  m_txSpectrumModel = spectrumModel;
  m_txBandwidthHz   = bandwidthHz;
  m_txPowerDbm      = txPowerDbm;
  m_txDuration      = duration;
}

void
NtnGwMac::SendDlPdu (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  if (!m_phy)
    {
      NS_LOG_ERROR ("NtnGwMac::SendDlPdu before SetPhy()");
      return;
    }
  if (!m_txSpectrumModel)
    {
      NS_LOG_ERROR ("NtnGwMac::SendDlPdu before SetTxConfig() (no SpectrumModel)");
      return;
    }

  // 简易 PSD: 在唯一 band 上均匀填入 txPower / BW
  const double txPowerW = std::pow (10.0, (m_txPowerDbm - 30.0) / 10.0);
  Ptr<SpectrumValue> psd = Create<SpectrumValue> (m_txSpectrumModel);
  (*psd)[0] = txPowerW / m_txBandwidthHz;

  Ptr<SatSignalParameters> params = Create<SatSignalParameters> ();
  params->psd = psd;
  params->duration = m_txDuration;
  params->packet = packet;

  NS_LOG_INFO ("[GW MAC] Tx DL size=" << packet->GetSize ()
               << " B, txPower=" << m_txPowerDbm << " dBm"
               << ", t=" << Simulator::Now ().GetSeconds () << " s");
  m_phy->SendPdu (packet, params);
}

void
NtnGwMac::SetUlPduCallback (Callback<void, Ptr<Packet>> cb)
{
  m_ulCallback = cb;
}

void
NtnGwMac::DoPhyRx (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // PHY→MAC delay trace
  NtnPhyDeliveryTag pdt;
  if (packet->PeekPacketTag (pdt))
    {
      Time d = Simulator::Now () - pdt.GetPhyRxTime ();
      // 暂用 tag 里 rnti 之前先 trace 0; 解析 tag 后下面 hasDci 判定时再补 rnti
      m_phyMacDelayTrace (0, d.GetNanoSeconds ());
    }

  // 看是不是 HARQ ACK (UE 端的 SatUtMac 收到 DL 数据后会自动 SendUlPdu 一个 ACK,
  // tag 上带 isAck=true)
  NtnDciTag tag;
  const bool hasDci = packet->PeekPacketTag (tag);
  if (hasDci && tag.IsAck ())
    {
      const uint16_t rnti = tag.GetRnti ();
      m_ackCount[rnti] += 1;
      NS_LOG_INFO ("[GW MAC] Rx HARQ ACK from rnti=" << rnti
                   << " (count=" << m_ackCount[rnti] << "), t="
                   << Simulator::Now ().GetSeconds () << " s");
      return;   // ACK 不上送到 ulCallback
    }

  // PHY 错误模型: 对 user data 概率 drop
  if (m_phyErr && m_phyErr->ShouldDrop ())
    {
      NS_LOG_INFO ("[GW MAC] PHY-error drop UL size=" << packet->GetSize ()
                   << " (sinr=" << m_phyErr->GetSinrDb () << " dB"
                   << ", bler=" << m_phyErr->GetBler () << ")");
      return;
    }

  // 普通上行用户数据
  // RxMacPdu trace: 1:1 映射 macPduSize == sduSize
  uint16_t rxRnti = hasDci ? tag.GetRnti () : 0;
  m_rxMacPduTrace (rxRnti, /*lcid=*/3, packet->GetSize (), packet->GetSize ());

  NS_LOG_INFO ("[GW MAC] Rx UL size=" << packet->GetSize ()
               << " B, t=" << Simulator::Now ().GetSeconds () << " s");

  // 全栈集成: 按 NtnDciTag.rnti 查表投递 UL RLC; 没匹配则走老的 ulCallback
  if (hasDci)
    {
      auto it = m_ulRlc.find (tag.GetRnti ());
      if (it != m_ulRlc.end () && it->second)
        {
          it->second->ReceiveRlcPdu (packet);
          return;
        }
    }
  if (!m_ulCallback.IsNull ())
    {
      m_ulCallback (packet);
    }
}

void
NtnGwMac::RegisterUlRlc (uint16_t rnti, Ptr<NtnRlc> rlc)
{
  m_ulRlc[rnti] = rlc;
  NS_LOG_INFO ("[GW MAC] RegisterUlRlc rnti=" << rnti << " -> " << rlc);
}

void
NtnGwMac::SetPhyErrorModel (Ptr<NtnPhyErrorModel> em)
{
  m_phyErr = em;
}

void
NtnGwMac::SetEffectiveDlSinrDb (uint16_t rnti, double sinrDb)
{
  m_effectiveDlSinrDb[rnti] = sinrDb;
}

void
NtnGwMac::ClearEffectiveDlSinrDb (uint16_t rnti)
{
  m_effectiveDlSinrDb.erase (rnti);
}

void
NtnGwMac::SetDlTxPowerDbm (uint16_t rnti, double dbm)
{
  m_dlTxPowerDbm[rnti] = dbm;
}

void
NtnGwMac::ClearDlTxPowerDbm (uint16_t rnti)
{
  m_dlTxPowerDbm.erase (rnti);
}

void
NtnGwMac::SetScheduler (Ptr<GeoBeamScheduler> sched)
{
  NS_LOG_FUNCTION (this << sched);
  m_scheduler = sched;
}

double
NtnGwMac::ResolveDlTxPowerDbm (uint16_t rnti) const
{
  // 单一真源优先: 已接线调度器且该 rnti 有最近一次授权功率 -> 用调度器值 (β 切分/DL 动态功率)。
  if (m_scheduler)
    {
      const double schedDbm = m_scheduler->GetLastDlTxPowerDbm (rnti);
      if (!std::isnan (schedDbm))
        {
          return schedDbm;
        }
    }
  // 其次: 手工 per-rnti map (向后兼容)。
  auto it = m_dlTxPowerDbm.find (rnti);
  if (it != m_dlTxPowerDbm.end ())
    {
      return it->second;
    }
  // 回退: 固定成员功率。
  return m_txPowerDbm;
}

void
NtnGwMac::StampDlDciTag (Ptr<Packet> packet, uint16_t rnti, bool isAck)
{
  // 项 B: 给 DL 数据包打 DCI tag(rnti, isAck), 附带本次有效 SINR(若有),
  // 让 UE 侧据此驱动 BLER (功率域 NOMA 的 near/far 干扰代价真实化)。
  // 单一真源优先: 已接线调度器且该 rnti 有最近一次授权有效 SINR -> 用调度器值,
  // 否则回退手工 SetEffectiveDlSinrDb 的 map。
  NtnDciTag tag (rnti, isAck);
  bool stamped = false;
  if (m_scheduler)
    {
      const double schedSinr = m_scheduler->GetLastDlEffSinrDb (rnti);
      if (!std::isnan (schedSinr))
        {
          tag.SetEffectiveSinrDb (schedSinr);
          stamped = true;
        }
    }
  if (!stamped)
    {
      auto it = m_effectiveDlSinrDb.find (rnti);
      if (it != m_effectiveDlSinrDb.end ())
        {
          tag.SetEffectiveSinrDb (it->second);
        }
    }
  packet->AddPacketTag (tag);
}

// =====================================================================
// 多 UE 调度
// =====================================================================

void
NtnGwMac::AddUe (uint16_t rnti)
{
  AddUe (rnti, 0);
}

void
NtnGwMac::AddUe (uint16_t rnti, uint16_t beamId)
{
  if (m_dlQueues.find (rnti) == m_dlQueues.end ())
    {
      m_dlQueues[rnti] = {};
      m_ackCount[rnti] = 0;
      m_registeredUes.push_back (rnti);
      NS_LOG_INFO ("[GW MAC] AddUe rnti=" << rnti << " beam=" << beamId
                   << " (total UEs=" << m_registeredUes.size () << ")");
    }
  m_ueBeam[rnti] = beamId;
}

void
NtnGwMac::EnqueueDlPdu (uint16_t rnti, Ptr<Packet> packet)
{
  auto it = m_dlQueues.find (rnti);
  if (it == m_dlQueues.end ())
    {
      NS_LOG_WARN ("[GW MAC] EnqueueDlPdu: rnti " << rnti << " not registered, dropping");
      return;
    }
  it->second.push_back (packet);
  m_dlEnqueueTimes[rnti].push_back (Simulator::Now ());

  // 累计 rnti 的总队列字节
  uint32_t totalBytes = 0;
  for (const auto& p : it->second) totalBytes += p->GetSize ();
  m_dlQueueLengthTrace (rnti, totalBytes);

  NS_LOG_INFO ("[GW MAC] enqueue DL for rnti=" << rnti
               << " size=" << packet->GetSize ()
               << " B, queueLen=" << it->second.size ());
}

void
NtnGwMac::StartScheduler (Time period)
{
  m_schedPeriod = period;
  NS_LOG_INFO ("[GW MAC] StartScheduler period=" << period.GetMilliSeconds () << " ms");
  m_schedEvent = Simulator::Schedule (m_schedPeriod, &NtnGwMac::ScheduleTick, this);
}

void
NtnGwMac::StopScheduler ()
{
  if (m_schedEvent.IsRunning ())
    {
      Simulator::Cancel (m_schedEvent);
    }
}

void
NtnGwMac::ScheduleTick ()
{
  // 轮询: 从 m_rrCursor 开始扫一圈, 找到第一个有包的 UE, 发 1 个; 没有就空转
  const size_t n = m_registeredUes.size ();
  for (size_t step = 0; step < n; ++step)
    {
      const size_t idx = (m_rrCursor + step) % n;
      const uint16_t rnti = m_registeredUes[idx];
      auto& q = m_dlQueues[rnti];
      if (!q.empty ())
        {
          Ptr<Packet> pkt = q.front ();
          q.pop_front ();
          // 出队时延 trace: 当前时间 - 入队时间
          auto& tq = m_dlEnqueueTimes[rnti];
          if (!tq.empty ())
            {
              Time dt = Simulator::Now () - tq.front ();
              tq.pop_front ();
              m_dlQueueDelayTrace (rnti, dt.GetNanoSeconds ());
            }
          // 队列长度 trace: 剩余字节
          uint32_t remain = 0; for (const auto& p : q) remain += p->GetSize ();
          m_dlQueueLengthTrace (rnti, remain);

          m_rrCursor = (idx + 1) % n;
          SendDlPduForRnti (rnti, pkt);
          break;
        }
    }
  // 不管发没发, 都继续 tick
  m_schedEvent = Simulator::Schedule (m_schedPeriod, &NtnGwMac::ScheduleTick, this);
}

void
NtnGwMac::SendDlPduForRnti (uint16_t rnti, Ptr<Packet> packet)
{
  if (!m_phy || !m_txSpectrumModel)
    {
      NS_LOG_ERROR ("[GW MAC] SendDlPduForRnti before SetPhy/SetTxConfig");
      return;
    }

  // 打 DCI tag: rnti = 目标 UE, isAck=false; 若有本轮有效 SINR 一并带上 (项 B)
  StampDlDciTag (packet, rnti, /*isAck=*/false);

  // 多波束: 已登记 beamId 的 UE 包加 NtnBeamIdTag, 让卫星侧 feederPhy 按 beam 路由
  auto bIt = m_ueBeam.find (rnti);
  if (bIt != m_ueBeam.end () && bIt->second != 0)
    {
      NtnBeamIdTag btag (bIt->second);
      packet->AddPacketTag (btag);
    }

  // 构造 SatSignalParameters
  // 项 A: 发射功率取该 rnti 的真值 (调度器 β 切分/DL 动态功率 > 手工 map > 固定回退),
  // 让 DCI 功率真正进入物理 PSD, 而非永远固定 m_txPowerDbm。
  const double txDbm = ResolveDlTxPowerDbm (rnti);
  const double txPowerW = std::pow (10.0, (txDbm - 30.0) / 10.0);
  Ptr<SpectrumValue> psd = Create<SpectrumValue> (m_txSpectrumModel);
  (*psd)[0] = txPowerW / m_txBandwidthHz;

  Ptr<SatSignalParameters> params = Create<SatSignalParameters> ();
  params->psd = psd;
  params->duration = m_txDuration;
  params->packet = packet;

  // Mux trace: 当前 1:1 映射, rlcSize == macPduSize (无 padding/header overhead)
  m_txMacPduTrace (rnti, /*lcid=*/3, packet->GetSize (), packet->GetSize ());

  NS_LOG_INFO ("[GW MAC] Tx DL rnti=" << rnti
               << " size=" << packet->GetSize ()
               << " B, txPower=" << txDbm << " dBm"
               << ", t=" << Simulator::Now ().GetSeconds () << " s");
  m_phy->SendPdu (packet, params);
}

uint32_t
NtnGwMac::GetAckCount (uint16_t rnti) const
{
  auto it = m_ackCount.find (rnti);
  return (it != m_ackCount.end ()) ? it->second : 0;
}

std::vector<uint16_t>
NtnGwMac::GetRegisteredUes () const
{
  return m_registeredUes;
}

} // namespace ns3