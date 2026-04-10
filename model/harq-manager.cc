/*
 * 文件路径：contrib/geo-sat/model/harq-manager.cc
 * 功能：HARQ增强管理器实现 - 增量冗余(IR)重传机制
 */
#include "harq-manager.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HarqManager");
NS_OBJECT_ENSURE_REGISTERED (HarqManager);

const Time HarqManager::HARQ_ACK_TIMEOUT = MilliSeconds (650); // GEO RTT + Margin

TypeId HarqManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::HarqManager")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<HarqManager> ();
  return tid;
}

HarqManager::HarqManager ()
  : m_harqEnabled (true),  // 默认启用HARQ
    m_totalTransmissions (0),
    m_totalRetransmissions (0),
    m_totalAck (0),
    m_totalNack (0),
    m_totalDropped (0)
{
  NS_LOG_FUNCTION (this);
}

HarqManager::~HarqManager () { NS_LOG_FUNCTION (this); }

void
HarqManager::SetHarqEnabled (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_harqEnabled = enable;

  if (m_harqEnabled)
    {
      NS_LOG_INFO ("[HARQ] HARQ Enabled - NACK will trigger retransmission");
    }
  else
    {
      NS_LOG_INFO ("[HARQ] HARQ Disabled - NACK will be treated as success (packet dropped)");
    }
}

bool
HarqManager::IsHarqEnabled () const
{
  return m_harqEnabled;
}

// ==================== HARQ进程管理 ====================

uint8_t HarqManager::NewTransmission (uint16_t rnti, Ptr<Packet> packet, uint8_t mcs)
{
  NS_LOG_FUNCTION (this << rnti << (uint32_t)mcs);
  
  SatHarqProcess* process = AllocateProcess (rnti);
  if (process == nullptr) {
      NS_LOG_WARN ("[HARQ] No available HARQ process for UE " << rnti);
      return 255; // 无可用进程
  }
  
  process->rnti = rnti;
  process->packet = packet->Copy ();
  process->txBytes = packet->GetSize ();
  process->mcs = mcs;
  process->rvIndex = 0; // 初传使用RV=0
  process->state = HarqState::WAITING_ACK;
  process->retransmitCount = 0;
  process->startTime = Simulator::Now ();
  
  m_totalTransmissions++;
  
  NS_LOG_INFO ("[HARQ] New TX | UE=" << rnti << " Process=" << (uint32_t)process->processId
               << " MCS=" << (uint32_t)mcs << " RV=" << (uint32_t)process->rvIndex
               << " Size=" << process->txBytes << " bytes");
  
  return process->processId;
}

void HarqManager::ReceiveFeedback (const SatHarqFeedback& feedback)
{
  NS_LOG_FUNCTION (this << feedback.rnti << (uint32_t)feedback.processId << feedback.ack);

  SatHarqProcess* process = GetProcess (feedback.rnti, feedback.processId);
  if (process == nullptr || process->state != HarqState::WAITING_ACK) {
      NS_LOG_WARN ("[HARQ] Invalid feedback or process not in WAITING_ACK state");
      return;
  }

  if (feedback.ack) {
      process->state = HarqState::SUCCESS;
      m_totalAck++;

      NS_LOG_INFO ("[HARQ] ACK | UE=" << feedback.rnti << " Process="
                   << (uint32_t)feedback.processId << " TotalACK=" << m_totalAck);

      m_feedbackCallback (feedback.rnti, feedback.processId, true);
  } else {
      m_totalNack++;

      NS_LOG_INFO ("[HARQ] NACK | UE=" << feedback.rnti << " Process="
                   << (uint32_t)feedback.processId << " RetxCount="
                   << (uint32_t)process->retransmitCount);

      // 【核心修改】如果HARQ被禁用，NACK不触发重传
      if (!m_harqEnabled)
        {
          process->state = HarqState::SUCCESS;  // 视作成功（数据包被丢弃）
          m_totalDropped++;

          NS_LOG_INFO ("[HARQ-DISABLED] NACK treated as success | UE=" << feedback.rnti
                       << " Process=" << (uint32_t)feedback.processId
                       << " TotalDropped=" << m_totalDropped);

          // 通知上层（传入true表示传输完成，不触发重传）
          m_feedbackCallback (feedback.rnti, feedback.processId, true);
          return;
        }

      // 【原始逻辑】启用HARQ时，检查是否超过最大重传次数
      if (process->retransmitCount >= MAX_RETRANSMISSIONS) {
          process->state = HarqState::FAILED;
          NS_LOG_WARN ("[HARQ] Max retransmissions reached! UE=" << feedback.rnti
                       << " Process=" << (uint32_t)feedback.processId);
          m_feedbackCallback (feedback.rnti, feedback.processId, false);
          return;
      }

      // 更新RV索引 (增量冗余)
      process->rvIndex = feedback.rvIndex;

      // 触发重传
      ScheduleRetransmission (feedback.rnti, feedback.processId);
  }
}

HarqState HarqManager::GetProcessState (uint16_t rnti, uint8_t processId) const
{
  auto key = std::make_pair (rnti, processId);
  auto it = m_harqProcesses.find (key);
  
  if (it != m_harqProcesses.end ()) {
      return it->second.state;
  }
  return HarqState::IDLE;
}

uint8_t HarqManager::GetAvailableProcessCount (uint16_t rnti) const
{
  auto it = m_rntiToProcesses.find (rnti);
  if (it == m_rntiToProcesses.end ()) {
      return MAX_HARQ_PROCESSES;
  }
  
  uint8_t available = 0;
  for (uint8_t pid : it->second) {
      auto key = std::make_pair (rnti, pid);
      auto procIt = m_harqProcesses.find (key);
      if (procIt != m_harqProcesses.end () && 
          (procIt->second.state == HarqState::IDLE || 
           procIt->second.state == HarqState::SUCCESS ||
           procIt->second.state == HarqState::FAILED)) {
          available++;
      }
  }
  
  return available;
}

// ==================== IR增量冗余 ====================

uint8_t HarqManager::GetNextRedundancyVersion (uint16_t rnti, uint8_t processId)
{
  SatHarqProcess* process = GetProcess (rnti, processId);
  if (process == nullptr) return 0;
  
  // IR HARQ: RV索引循环 0 -> 2 -> 3 -> 1
  // 这是3GPP TS 36.212定义的IR版本序列
  static const uint8_t rvSequence[] = {0, 2, 3, 1};
  uint8_t nextRv = rvSequence[process->rvIndex % 4];
  
  NS_LOG_DEBUG ("[HARQ] RV Index | Current=" << (uint32_t)process->rvIndex 
                << " Next=" << (uint32_t)nextRv);
  
  return nextRv;
}

double HarqManager::CalculateIrGain (uint8_t rvIndex) const
{
  // IR增益: 不同RV版本提供不同的编码增益
  // RV=0: 最高增益 (新数据比例最高)
  // RV>0: 纯冗余，贡献增量增益
  double gainDb[] = {3.0, 0.0, 1.5, 2.0}; // 简化模型
  return gainDb[rvIndex % 4];
}

Ptr<Packet> HarqManager::GenerateRetransmission (uint16_t rnti, uint8_t processId)
{
  SatHarqProcess* process = GetProcess (rnti, processId);
  if (process == nullptr || process->packet == nullptr) {
      NS_LOG_WARN ("[HARQ] Cannot generate retransmission - no packet");
      return nullptr;
  }
  
  // 获取下一个RV版本
  uint8_t newRvIndex = GetNextRedundancyVersion (rnti, processId);
  
  // IR puncturing: 根据RV版本对数据进行冗余穿刺
  // 模拟: 不同RV版本传输不同的编码块
  Ptr<Packet> retxPacket = process->packet->Copy ();
  
  // 更新进程状态
  process->rvIndex = newRvIndex;
  process->retransmitCount++;
  process->state = HarqState::RETRANSMITTING;
  m_totalRetransmissions++;
  
  NS_LOG_INFO ("[HARQ] Retx | UE=" << rnti << " Process=" << (uint32_t)processId
               << " RV=" << (uint32_t)newRvIndex << " IR-Gain=" 
               << CalculateIrGain (newRvIndex) << " dB"
               << " RetxCount=" << (uint32_t)process->retransmitCount);
  
  // 触发重传回调
  m_retransmitCallback (rnti, processId, retxPacket);
  
  // 调度ACK等待
  Simulator::Schedule (HARQ_ACK_TIMEOUT, &HarqManager::ProcessTimeout, 
                      this, rnti, processId);
  
  return retxPacket;
}

// ==================== 内部辅助函数 ====================

SatHarqProcess* HarqManager::GetProcess (uint16_t rnti, uint8_t processId)
{
  auto key = std::make_pair (rnti, processId);
  auto it = m_harqProcesses.find (key);
  
  if (it != m_harqProcesses.end ()) {
      return &it->second;
  }
  return nullptr;
}

SatHarqProcess* HarqManager::AllocateProcess (uint16_t rnti)
{
  // 查找空闲进程
  for (uint8_t pid = 0; pid < MAX_HARQ_PROCESSES; ++pid) {
      auto key = std::make_pair (rnti, pid);
      
      if (m_harqProcesses.find (key) == m_harqProcesses.end ()) {
          // 找到空闲进程，分配
          SatHarqProcess process;
          process.processId = pid;
          process.rnti = rnti;
          process.state = HarqState::IDLE;
          process.txBytes = 0;
          process.retransmitCount = 0;
          process.packet = nullptr;
          
          m_harqProcesses[key] = process;
          m_rntiToProcesses[rnti].push_back (pid);
          
          NS_LOG_DEBUG ("[HARQ] Allocated process " << (uint32_t)pid << " for UE " << rnti);
          return &m_harqProcesses[key];
      }
      
      // 检查现有进程是否完成
      auto it = m_harqProcesses.find (key);
      if (it != m_harqProcesses.end ()) {
          HarqState state = it->second.state;
          if (state == HarqState::IDLE || 
              state == HarqState::SUCCESS || 
              state == HarqState::FAILED) {
              m_harqProcesses.erase (it);
              return AllocateProcess (rnti); // 重新分配
          }
      }
  }
  
  return nullptr; // 无可用进程
}

void HarqManager::ScheduleRetransmission (uint16_t rnti, uint8_t processId)
{
  NS_LOG_FUNCTION (this << rnti << (uint32_t)processId);
  
  SatHarqProcess* process = GetProcess (rnti, processId);
  if (process == nullptr) return;
  
  // 生成重传包
  Ptr<Packet> retxPacket = GenerateRetransmission (rnti, processId);
  
  if (retxPacket == nullptr) {
      NS_LOG_ERROR ("[HARQ] Failed to generate retransmission packet!");
      return;
  }
  
  // 等待GEO卫星RTT后进行重传 (模拟卫星信道延迟)
  // 实际上重传会在物理层根据信道条件自动触发
  Simulator::Schedule (HARQ_ACK_TIMEOUT, [this, rnti, processId]() {
      if (GetProcessState (rnti, processId) == HarqState::RETRANSMITTING) {
          NS_LOG_INFO ("[HARQ] Retransmission timeout, waiting for ACK/NACK");
      }
  });
}

void HarqManager::ProcessTimeout (uint16_t rnti, uint8_t processId)
{
  SatHarqProcess* process = GetProcess (rnti, processId);
  if (process == nullptr) return;

  if (process->state == HarqState::WAITING_ACK ||
      process->state == HarqState::RETRANSMITTING)
    {
      m_totalNack++;

      // 【核心修改】如果HARQ被禁用，超时直接视为成功
      if (!m_harqEnabled)
        {
          process->state = HarqState::SUCCESS;
          m_totalDropped++;

          NS_LOG_INFO ("[HARQ-DISABLED] Timeout treated as success | UE=" << rnti
                       << " Process=" << (uint32_t)processId);

          // 通知上层（传入true表示传输完成）
          m_feedbackCallback (rnti, processId, true);
          return;
        }

      NS_LOG_WARN ("[HARQ] ACK Timeout! UE=" << rnti << " Process=" << (uint32_t)processId);

      process->state = HarqState::FAILED;

      // 触发失败回调
      m_feedbackCallback (rnti, processId, false);
    }
}

// ==================== 统计接口 ====================

uint32_t HarqManager::GetTotalTransmissions () const
{
  return m_totalTransmissions;
}

uint32_t HarqManager::GetTotalRetransmissions () const
{
  return m_totalRetransmissions;
}

uint32_t HarqManager::GetTotalAck () const
{
  return m_totalAck;
}

uint32_t HarqManager::GetTotalNack () const
{
  return m_totalNack;
}

uint32_t HarqManager::GetTotalDropped () const
{
  return m_totalDropped;
}

double HarqManager::GetRetransmissionRate () const
{
  if (m_totalTransmissions == 0) return 0.0;
  return (double)m_totalRetransmissions / m_totalTransmissions;
}

void HarqManager::ResetStatistics ()
{
  m_totalTransmissions = 0;
  m_totalRetransmissions = 0;
  m_totalAck = 0;
  m_totalNack = 0;
  m_totalDropped = 0;
  m_harqProcesses.clear ();
  m_rntiToProcesses.clear ();

  NS_LOG_INFO ("[HARQ] Statistics reset");
}

} // namespace ns3
