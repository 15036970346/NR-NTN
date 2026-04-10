/*
 * 文件路径：contrib/geo-sat/model/sat-gw-mac.h
 * 功能：GEO 卫星信关站 MAC 层实体 (负责系统广播与周期性传输)
 */
#ifndef SAT_GW_MAC_H
#define SAT_GW_MAC_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/packet.h"
#include "sat-mac-common.h"
#include <vector>

namespace ns3 {

class SatGwMac : public Object
{
public:
  static TypeId GetTypeId (void);
  SatGwMac ();
  virtual ~SatGwMac ();

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

private:
  // 内部发包逻辑：生成广播包并打上波束 ID，发往物理层
  void SendBroadcastMacPdu (uint32_t beamId, bool isSsb);

  // 发送SSB的各个组成部分
  void SendPss (uint32_t beamId, uint32_t pci);
  void SendSss (uint32_t beamId, uint32_t pci);
  void SendPbch (uint32_t beamId, uint32_t pci, uint8_t ssbIndex);

  Time m_ssbPeriod;                         // 广播周期 (例如 20ms)
  EventId m_periodicBroadcastEvent;         // 维护定时器句柄，防止事件失控
  std::vector<uint32_t> m_activeBeamIds;    // 活跃波束列表
  uint16_t m_systemFrameNumber;              // 当前系统帧号 (SFN)
  uint8_t m_ssbIndex;                       // SSB索引 (0-63)
};

} // namespace ns3

#endif /* SAT_GW_MAC_H */