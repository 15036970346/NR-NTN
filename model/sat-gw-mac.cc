/*
 * 文件路径：contrib/geo-sat/model/sat-gw-mac.cc
 * 功能：实现信关站周期性生成 SSB 和 SIB1 并向物理层发送的逻辑 - 最终优化版
 */
#include "sat-gw-mac.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatGwMac");
NS_OBJECT_ENSURE_REGISTERED (SatGwMac);

TypeId SatGwMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatGwMac")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatGwMac> ()
    .AddAttribute ("SsbPeriod",
                   "Periodicity of SSB and SIB1 broadcast in NTN.",
                   TimeValue (MilliSeconds (20)),
                   MakeTimeAccessor (&SatGwMac::m_ssbPeriod),
                   MakeTimeChecker ());
  return tid;
}

SatGwMac::SatGwMac ()
{
  NS_LOG_FUNCTION (this);
  m_ssbPeriod = MilliSeconds (20);
  m_systemFrameNumber = 0;
  m_ssbIndex = 0;
}

SatGwMac::~SatGwMac () {}

void SatGwMac::SetActiveBeams (const std::vector<uint32_t>& beams)
{
  NS_LOG_FUNCTION (this << beams.size());
  m_activeBeamIds = beams;
}

void SatGwMac::StartPeriodicTransmissions ()
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
                                                  &SatGwMac::StartPeriodicTransmissions, 
                                                  this);
}

void SatGwMac::StopPeriodicTransmissions ()
{
  NS_LOG_FUNCTION (this);
  if (m_periodicBroadcastEvent.IsRunning ())
    {
      Simulator::Cancel (m_periodicBroadcastEvent);
    }
}

// ==================== SSB同步信号块实现 ====================

void SatGwMac::TransmitSsbBlock (uint32_t beamId)
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
  Simulator::Schedule (MicroSeconds (4), &SatGwMac::SendSss, this, beamId, pci);
  Simulator::Schedule (MicroSeconds (8), &SatGwMac::SendPbch, this, beamId, pci, m_ssbIndex);
  
  NS_LOG_INFO ("=== SSB传输完成 ===");
}

uint32_t SatGwMac::CalculatePci (uint32_t nId1, uint32_t nId2)
{
  // PCI = N_ID_1 * 3 + N_ID_2 (3GPP 38.211)
  return nId1 * 3 + nId2;
}

std::vector<uint32_t> SatGwMac::GeneratePssSequence (uint32_t pci)
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

std::vector<uint32_t> SatGwMac::GenerateSssSequence (uint32_t pci)
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

Ptr<Packet> SatGwMac::GeneratePbchPayload (uint32_t beamId, uint8_t ssbIndex)
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

void SatGwMac::SendPss (uint32_t beamId, uint32_t pci)
{
  std::vector<uint32_t> pssSeq = GeneratePssSequence (pci);
  
  NS_LOG_INFO ("[SSB-PSS] Beam " << beamId 
               << " | 发送主同步信号 | PCI_N_ID_2=" << (pci % 3)
               << " | 序列长度=" << pssSeq.size ());
  
  // TODO: 调用物理层SAP发送PSS
  // m_macPhySapProvider->TransmitPss (pssSeq, beamId);
}

void SatGwMac::SendSss (uint32_t beamId, uint32_t pci)
{
  std::vector<uint32_t> sssSeq = GenerateSssSequence (pci);
  
  NS_LOG_INFO ("[SSB-SSS] Beam " << beamId 
               << " | 发送辅同步信号 | PCI_N_ID_1=" << (pci / 3)
               << " | 序列长度=" << sssSeq.size ());
  
  // TODO: 调用物理层SAP发送SSS
  // m_macPhySapProvider->TransmitSss (sssSeq, beamId);
}

void SatGwMac::SendPbch (uint32_t beamId, uint32_t pci, uint8_t ssbIndex)
{
  Ptr<Packet> pbchPkt = GeneratePbchPayload (beamId, ssbIndex);
  
  NS_LOG_INFO ("[SSB-PBCH] Beam " << beamId 
               << " | 发送物理广播信道 | PCI=" << pci
               << " | SSB_Index=" << (uint32_t)ssbIndex
               << " | SFN=" << m_systemFrameNumber);
  
  // TODO: 调用物理层SAP发送PBCH
  // m_macPhySapProvider->TransmitPbch (pbchPkt, beamId, pci, ssbIndex);
}

void SatGwMac::SendBroadcastMacPdu (uint32_t beamId, bool isSsb)
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

uint16_t SatGwMac::GetCurrentSfn () const
{
  return m_systemFrameNumber;
}

} // namespace ns3