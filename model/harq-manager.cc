#include "harq-manager.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <array>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HarqManager");
NS_OBJECT_ENSURE_REGISTERED (HarqManager);

namespace
{

constexpr std::array<uint8_t, 4> kRvSequence = {0, 2, 3, 1};//HARQ 增量冗余（IR）的冗余版本（RV）发送序列

bool
IsTerminalState (HarqState state)//判断给定的 HARQ 进程状态是否属于终态
{//如果状态是空闲（IDLE）、成功（SUCCESS）或失败（FAILED），则返回 true，表示该进程当前没有在等 ACK，可以被分配新任务
  return state == HarqState::IDLE ||
         state == HarqState::SUCCESS ||
         state == HarqState::FAILED;
}

} // namespace

const Time HarqManager::HARQ_ACK_TIMEOUT = MilliSeconds (600); // HARQ 的 ACK 超时时间为 600毫秒 GEO 卫星的往返时延(540)加上处理余量
const uint8_t HarqManager::DEFAULT_HARQ_PROCESSES;//默认 HARQ 进程数
const uint8_t HarqManager::MAX_RETRANSMISSIONS;//最大重传次数常量

TypeId
HarqManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::HarqManager")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<HarqManager> ()
    .AddAttribute ("ProcessCount",//可动态配置的属性 ProcessCount--- HARQ 进程数
                   "Number of parallel HARQ processes per UE. Supported values: 1, 2, 4, 8, 16, 32.",//每个 UE 支持并行的 HARQ 进程数
                   UintegerValue (DEFAULT_HARQ_PROCESSES),
                   MakeUintegerAccessor (&HarqManager::SetProcessCount,
                                         &HarqManager::GetProcessCount),
                   MakeUintegerChecker<uint32_t> (1, 32));
  return tid;
}

HarqManager::HarqManager ()
  : m_harqEnabled (true),//默认 HARQ 开关设为开启
    m_processCount (DEFAULT_HARQ_PROCESSES),//初始化设定的进程数量
    m_totalTransmissions (0),//总传输清零
    m_totalRetransmissions (0),//总重传清零
    m_totalAck (0),//ACK 数清零
    m_totalNack (0),//NACK清零
    m_totalDropped (0)//丢弃数清零
{
  NS_LOG_FUNCTION (this);
}

HarqManager::~HarqManager ()
{
  NS_LOG_FUNCTION (this);
}

void
HarqManager::SetHarqEnabled (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_harqEnabled = enable;//将传入的布尔值赋给成员变量

//打印状态切换的详细信息
  if (m_harqEnabled)//如果开启，明确告知 NACK 或超时会触发增量冗余重传
    {
      NS_LOG_INFO ("[HARQ] HARQ Enabled - NACK/timeout will trigger IR retransmission");
    }
  else//如果关闭，明确告知发生错误时直接丢包并向高层报告失败
    {
      NS_LOG_INFO ("[HARQ] HARQ Disabled - NACK/timeout drops the packet "
                   "and reports FAILED to upper layers (no retransmission)");
    }
}

bool//只读返回当前 HARQ 机制是否开启
HarqManager::IsHarqEnabled () const
{
  return m_harqEnabled;
}

bool//HARQ 进程数
HarqManager::IsSupportedProcessCount (uint8_t count)
{
  return count == 1 || count == 2 || count == 4 ||
         count == 8 || count == 16 || count == 32;
}

void//容错与吸附逻辑
HarqManager::SetProcessCount (uint8_t count)
{
  uint8_t resolved = count;
  if (!IsSupportedProcessCount (count))
    {
      // 合法集 {1,2,4,8,16,32} 都是 2 的幂，非法值就近吸附到最近的 2 的幂并夹在 [1,32]。
      uint8_t lower = 1;
      while ((lower << 1) <= count && (lower << 1) <= 32)//找 ≤count 的最大 2 的幂
        {
          lower <<= 1;
        }
      const uint8_t upper = (lower < 32) ? static_cast<uint8_t> (lower << 1) : 32;//≥count 的最小 2 的幂
      resolved = (count - lower <= upper - count) ? lower : upper;//取更近的一边
      if (resolved < 1)//下溢保护(count=0)
        {
          resolved = 1;
        }
      NS_LOG_WARN ("[HARQ] ProcessCount=" << static_cast<uint32_t> (count)//警告用户配置被强制修正了
                   << " is not in {1,2,4,8,16,32}, snapping to "
                   << static_cast<uint32_t> (resolved));
    }

  const bool hasActiveProcess =
  //使用 std::any_of遍历当前内存中所有的 HARQ 进程
    std::any_of (m_harqProcesses.begin (),
                 m_harqProcesses.end (),
                 [] (const auto& entry) {
                  //利用之前写的 IsTerminalState 函数检查：如果存在任何一个状态不是“终态”的进程则返回 true 给 hasActiveProcess
                   return !IsTerminalState (entry.second.state);
                 });
 //仿真器的断言 如果此时有活动的进程，直接中断报错
  NS_ABORT_MSG_IF (hasActiveProcess,
                   "HARQ process count cannot be changed while processes are active.");

  //当确认没有活动进程，且新的进程数确实与旧值不同时执行重置操作
  if (resolved != m_processCount && !m_harqProcesses.empty ())
    {
      // 仅剩终态进程时允许重配；清掉旧 PID 表，避免进程数切换后保留无意义历史状态。
      for (auto& entry : m_harqProcesses)//遍历底层的 m_harqProcesses 字典
        {
          if (entry.second.ackTimeoutEvent.IsRunning ())//把所有还在运行的定时器（ackTimeoutEvent）强制取消
            {
              entry.second.ackTimeoutEvent.Cancel ();
            }
        }
      m_harqProcesses.clear ();//彻底清空字典 m_harqProcesses.clear()，防止出现旧档位的废弃进程 ID 残留
    }
  m_processCount = resolved;//将最终决议好的进程数安全地赋值给
}

uint8_t
HarqManager::GetProcessCount () const//安全读取当前的进程数设置
{
  return m_processCount;
}


uint8_t
//函数签名及日志 当 MAC 层有新的数据包要发送时调用此函数
HarqManager::NewTransmission (uint16_t rnti, Ptr<Packet> packet, uint8_t mcs)//传入用户的 rnti，数据包指针 packet 和调制编码策略 mcs
{
  NS_LOG_FUNCTION (this << rnti << (uint32_t) mcs);

  SatHarqProcess* process = AllocateProcess (rnti);//调用内部的 AllocateProcess 为该 UE 分配一个空闲的 HARQ 进程
  if (process == nullptr)//如果返回空指针说明该 UE 的所有进程都在等待 ACK
    {
      NS_LOG_WARN ("[HARQ] No available HARQ process for UE " << rnti);//打印警告
      return INVALID_PROCESS_ID;//返回无效的进程 ID 拒绝发送
    }

  //如果该进程身上还挂着之前遗留的超时定时器，立刻将其取消
  if (process->ackTimeoutEvent.IsRunning ())
    {
      process->ackTimeoutEvent.Cancel ();
    }
//初始化新进程的所有状态
  process->rnti = rnti;//绑定 rnti
  process->txBuffer = packet != nullptr ? packet->GetSize () : 0;//记录包大小
  process->txBytes = process->txBuffer;
  process->mcs = mcs;//保存 mcs
  process->rvIndex = kRvSequence.front ();//将冗余版本设为初始态
  process->rvSequenceIndex = 0;//序列索引设为初始态
  process->state = HarqState::WAITING_ACK;//状态切换为 WAITING_ACK
  process->retransmitCount = 0;//重传次数清零
  process->startTime = Simulator::Now ();//记录当前仿真时间
  process->packet = packet != nullptr ? packet->Copy () : Create<Packet> ();//利用 Copy()深度拷贝数据包并缓冲起来 如果失败了靠这个缓存来重传

  ScheduleAckTimeout (*process);//启动该进程的 ACK 超时定时器
  ++m_totalTransmissions;//总传输次数统计量加一

  NS_LOG_INFO ("[HARQ] New TX | UE=" << rnti
               << " Process=" << static_cast<uint32_t> (process->processId)
               << " MCS=" << static_cast<uint32_t> (mcs)
               << " RV=" << static_cast<uint32_t> (process->rvIndex)
               << " Size=" << process->txBytes << " bytes");
  return process->processId;//向调用方返回刚刚分配好的进程 ID
}

void
HarqManager::ReceiveFeedback (const SatHarqFeedback& feedback)//底层（物理层/信道）收到对端 ACK/NACK 反馈时的回调入口
{
  NS_LOG_FUNCTION (this << feedback.rnti << static_cast<uint32_t> (feedback.processId)
                        << feedback.ack);

  SatHarqProcess* process = GetProcess (feedback.rnti, feedback.processId);//根据反馈中的 rnti 和 processId 去查表找对应的进程
  if (process == nullptr || process->state != HarqState::WAITING_ACK)//如果查不到，或者该进程不在 WAITING_ACK 状态
    {//过时的或无效的反馈，打印警告并忽略
      NS_LOG_WARN ("[HARQ] Ignoring feedback for inactive process | UE=" << feedback.rnti
                   << " Process=" << static_cast<uint32_t> (feedback.processId));
      return;
    }

  if (process->ackTimeoutEvent.IsRunning ())//收到了反馈，说明并没有超时，立刻取消挂在身上的超时定时器
    {
      process->ackTimeoutEvent.Cancel ();
    }

  //ACK 成功分支
  if (feedback.ack)
    {
      ++m_totalAck;//收到肯定确认：ACK 统计量加一
      FinalizeProcess (*process, HarqState::SUCCESS);//将进程状态结算为 SUCCESS（并清空缓存）
      NS_LOG_INFO ("[HARQ] ACK | UE=" << feedback.rnti//打日志
                   << " Process=" << static_cast<uint32_t> (feedback.processId)
                   << " TotalACK=" << m_totalAck);
      m_feedbackCallback (feedback.rnti, feedback.processId, true);//触发回调 向上层报告
      return;
    }

    //NACK 失败分支
  ++m_totalNack;//NACK 统计量加一
  NS_LOG_INFO ("[HARQ] NACK | UE=" << feedback.rnti//打印 NACK 日志和当前的重传次数
               << " Process=" << static_cast<uint32_t> (feedback.processId)
               << " RetxCount=" << static_cast<uint32_t> (process->retransmitCount));

  //检查全局的 HARQ 开关
  if (!m_harqEnabled)
    {
     
      ++m_totalDropped;
      FinalizeProcess (*process, HarqState::FAILED);//丢包将状态设为 FAILED
      m_feedbackCallback (feedback.rnti, feedback.processId, false);//向上层报告 false
      return;
    }

  if (process->retransmitCount >= MAX_RETRANSMISSIONS)//达到最大重传次数限制检查
    {
      FinalizeProcess (*process, HarqState::FAILED);//重传次数耗尽则强制结算为 FAILED
      NS_LOG_WARN ("[HARQ] Max retransmissions reached | UE=" << feedback.rnti
                   << " Process=" << static_cast<uint32_t> (feedback.processId));
      m_feedbackCallback (feedback.rnti, feedback.processId, false);
      return;
    }

    //如果以上条件都通过了（需要重传，且还没达到最大重传次数），调用 ScheduleRetransmission 触发真正的重传流程
  ScheduleRetransmission (feedback.rnti, feedback.processId);
}

//进程状态与可用容量查询
HarqState
HarqManager::GetProcessState (uint16_t rnti, uint8_t processId) const
{
  const SatHarqProcess* process = GetProcess (rnti, processId);
  return process != nullptr ? process->state : HarqState::IDLE;
}

uint8_t
HarqManager::GetAvailableProcessCount (uint16_t rnti) const
{
  uint8_t available = 0;//声明并初始化可用进程计数器 available 为 0
  for (uint8_t pid = 0; pid < m_processCount; ++pid)//从 pid = 0 遍历到系统配置的最大进程数
    {
      const SatHarqProcess* process = GetProcess (rnti, pid);//获取对应进程
      if (process == nullptr || IsTerminalState (process->state))//如果这个位置是空的或者这个进程虽然存在但已经处于终态
        {
          ++available;//可以用来发新包 计数器 ++available 加一
        }
    }
  return available;
}

//冗余版本 (RV) 轮转查询
uint8_t
HarqManager::GetNextRedundancyVersion (uint16_t rnti, uint8_t processId)//计算该进程如果下一次重传，应该用哪个RV
{
  SatHarqProcess* process = GetProcess (rnti, processId);//查找进程
  if (process == nullptr)
    {
      return kRvSequence.front ();//找不到直接返回预定义的 RV 序列的第一个值
    }

//读取当前进程的索引 rvSequenceIndex，加 1 后对整个序列长度（4）取模 算出下一个索引
  const uint8_t nextIndex = static_cast<uint8_t> ((process->rvSequenceIndex + 1) %
                                                  kRvSequence.size ());
  return kRvSequence.at (nextIndex);//使用 kRvSequence.at() 查表，返回算出的下一个真正的 RV 值
}

uint8_t
HarqManager::GetCurrentRedundancyVersion (uint16_t rnti, uint8_t processId) const//只读查询当前进程正在使用的 RV 版本
{
  const SatHarqProcess* process = GetProcess (rnti, processId);
  return process != nullptr ? process->rvIndex : kRvSequence.front ();//存在就返回 rvIndex，不存在就返回序列的默认头节点
}

uint8_t
HarqManager::GetRetransmitCount (uint16_t rnti, uint8_t processId) const//只读查询该进程已执行的重传次数(终态仍保留)
{
  const SatHarqProcess* process = GetProcess (rnti, processId);
  return process != nullptr ? process->retransmitCount : 0;
}

//增量冗余 (IR) 增益计算
double
HarqManager::CalculateIrGain (uint8_t rvIndex) const//输入一个 RV 版本，返回它在接收端软合并后带来的信噪比增益
{
  static const double gainDb[] = {3.0, 0.0, 1.5, 2.0};//RV0 增益 3dB，RV2 增益 0dB，RV3 增益 1.5dB，RV1 增益 2.0dB
  switch (rvIndex)//匹配传入的 rvIndex，返回对应的增益值
    {
    case 0:
      return gainDb[0];
    case 2:
      return gainDb[2];
    case 3:
      return gainDb[3];
    case 1:
      return gainDb[1];
    default:
      return 0.0;
    }
}

//重传数据包生成
Ptr<Packet>
HarqManager::GenerateRetransmission (uint16_t rnti, uint8_t processId)//确定需要重传时调用，负责生成包并推进状态
{
  SatHarqProcess* process = GetProcess (rnti, processId);//提取进程 并检查当初发包时缓存的原包数据是否存在
  if (process == nullptr || process->packet == nullptr)//缓存丢了打出警告并返回 nullptr
    {
      NS_LOG_WARN ("[HARQ] Cannot generate retransmission - no buffered packet");
      return nullptr;
    }

  process->rvSequenceIndex =//推进 RV 索引序列
    static_cast<uint8_t> ((process->rvSequenceIndex + 1) % kRvSequence.size ());
  process->rvIndex = kRvSequence.at (process->rvSequenceIndex);//根据算出的索引，去 kRvSequence 中提取出本次真实使用的 RV 编号赋给 rvIndex
  ++process->retransmitCount;//自身重传次数加一
  process->state = HarqState::WAITING_ACK;//状态重新设为正在等待确认
  ++m_totalRetransmissions;//管理器的总重传大盘统计指标加一

  Ptr<Packet> retxPacket = process->packet->Copy ();//深拷贝缓存的报文

  NS_LOG_INFO ("[HARQ] Retx | UE=" << rnti
               << " Process=" << static_cast<uint32_t> (processId)
               << " RV=" << static_cast<uint32_t> (process->rvIndex)
               << " IR-Gain=" << CalculateIrGain (process->rvIndex) << " dB"
               << " RetxCount=" << static_cast<uint32_t> (process->retransmitCount));

  m_retransmitCallback (rnti, processId, retxPacket);

  ScheduleAckTimeout (*process);//为这次重传重新启动超时定时器
  return retxPacket;//返回生成的报文指针
}

//底层字典检索与分配
SatHarqProcess*
HarqManager::GetProcess (uint16_t rnti, uint8_t processId)
{//在内部维护的 m_harqProcesses 字典中，寻找进程对象指针
  auto it = m_harqProcesses.find (std::make_pair (rnti, processId));//找到了返回地址
  return it != m_harqProcesses.end () ? &it->second : nullptr;//，找不到返回 nullptr
}

const SatHarqProcess*///const 重载版本 保证在其他只读方法里调用不破坏状态
HarqManager::GetProcess (uint16_t rnti, uint8_t processId) const
{
  auto it = m_harqProcesses.find (std::make_pair (rnti, processId));
  return it != m_harqProcesses.end () ? &it->second : nullptr;
}


SatHarqProcess*
HarqManager::AllocateProcess (uint16_t rnti)//为一个特定 UE 申请一个新进程
{
  for (uint8_t pid = 0; pid < m_processCount; ++pid)//遍历该 UE 允许分配的所有 ID
    {
      auto [it, inserted] = m_harqProcesses.emplace (std::make_pair (rnti, pid), SatHarqProcess ());//用 emplace 尝试在字典中插入新进程
      SatHarqProcess& process = it->second;
      process.processId = pid;
      process.rnti = rnti;

      if (inserted || IsTerminalState (process.state))//如果是一个刚刚分配出来的新进程或者是旧的但目前是闲置终态
        {
          if (process.ackTimeoutEvent.IsRunning ())
            {
              process.ackTimeoutEvent.Cancel ();//取消它身上所有的残留超时事件
            }
          return &process;
        }
    }
  return nullptr;
}

//统一调度重传与超时处理
void
HarqManager::ScheduleRetransmission (uint16_t rnti, uint8_t processId)
{
  NS_LOG_FUNCTION (this << rnti << static_cast<uint32_t> (processId));

  Ptr<Packet> packet = GenerateRetransmission (rnti, processId);//尝试调用之前的 GenerateRetransmission 去生成新包
  if (packet == nullptr)//生成失败
    {
      SatHarqProcess* process = GetProcess (rnti, processId);
      if (process != nullptr)
        {
          FinalizeProcess (*process, HarqState::FAILED);//把进程强制终结为 FAILED
        }
      m_feedbackCallback (rnti, processId, false);//向上层上报发送失败 (false)
    }
}

void
HarqManager::ProcessTimeout (uint16_t rnti, uint8_t processId)//定时器回调函数
{
  SatHarqProcess* process = GetProcess (rnti, processId);//安全校验
  if (process == nullptr || process->state != HarqState::WAITING_ACK)//如果当时挂载定时器的进程已经找不到了，或者状态已经变成别的了
    {
      return;//直接 return 忽视
    }

  if (!m_harqEnabled)//如果 HARQ 功能是关闭
    {
   
      ++m_totalDropped;//直接统计为丢包
      FinalizeProcess (*process, HarqState::FAILED);//清空进程标记为 FAILED
      m_feedbackCallback (rnti, processId, false);//上报发送失败
      return;
    }

  ++m_totalNack;//系统 NACK 计数器加一 打出超时警告
  NS_LOG_WARN ("[HARQ] ACK timeout | UE=" << rnti
               << " Process=" << static_cast<uint32_t> (processId)
               << " RetxCount=" << static_cast<uint32_t> (process->retransmitCount));

  if (process->retransmitCount >= MAX_RETRANSMISSIONS)//如果当前该进程的重传次数已经达到了最大限制 彻底放弃 宣告失败并上报 false
    {
      FinalizeProcess (*process, HarqState::FAILED);
      m_feedbackCallback (rnti, processId, false);
      return;
    }

  ScheduleRetransmission (rnti, processId);
}

//定时器绑定与终态清理
void
HarqManager::ScheduleAckTimeout (SatHarqProcess& process)
{
  if (process.ackTimeoutEvent.IsRunning ())
    {
      process.ackTimeoutEvent.Cancel ();//检查清理旧定时器
    }

  process.ackTimeoutEvent =
  //使用 ns-3 核心 Simulator::Schedule
  //在 HARQ_ACK_TIMEOUT后，预约执行 ProcessTimeout 函数，并传入环境上下文指针 this，以及 rnti 和 processId
    Simulator::Schedule (HARQ_ACK_TIMEOUT,
                         &HarqManager::ProcessTimeout,
                         this,
                         process.rnti,
                         process.processId);
}

void
HarqManager::FinalizeProcess (SatHarqProcess& process, HarqState finalState)//所有进程走向终结（成功/失败/被抛弃）时的必定调用点
{
  if (process.ackTimeoutEvent.IsRunning ())//强制清理挂接的超时定时器
    {
      process.ackTimeoutEvent.Cancel ();
    }

  process.state = finalState;//赋予进程最终指定的终态
  process.packet = nullptr;//切断智能指针引用
  process.txBuffer = 0;//释放占用的底层报文内存
  process.txBytes = 0;//清理数据大小的统计
}

//统计获取与重置终结
uint32_t
HarqManager::GetTotalTransmissions () const
{
  return m_totalTransmissions;//发起的原始包总数
}

uint32_t
HarqManager::GetTotalRetransmissions () const
{
  return m_totalRetransmissions;//触发的重传总数
}

uint32_t
HarqManager::GetTotalAck () const
{
  return m_totalAck;//成功收到的 ACK 确认总数
}

uint32_t
HarqManager::GetTotalNack () const
{
  return m_totalNack;//收到的 NACK (含超时) 总数
}

uint32_t
HarqManager::GetTotalDropped () const
{
  return m_totalDropped;//因为超出重传限制或开关未开导致的彻底丢弃包总数
}

double
HarqManager::GetRetransmissionRate () const//获取重传率比例
{
  if (m_totalTransmissions == 0)//检查分母是否为 0
    {
      return 0.0;
    }
  return static_cast<double> (m_totalRetransmissions) /
         static_cast<double> (m_totalTransmissions);
}

void
HarqManager::ResetStatistics ()//系统强制重置函数
{
  for (auto& pair : m_harqProcesses)//遍历底层的进程字典，将所有可能还在跑的定时事件强行终止
    {
      if (pair.second.ackTimeoutEvent.IsRunning ())
        {
          pair.second.ackTimeoutEvent.Cancel ();
        }
    }
//清零重置
  m_totalTransmissions = 0;
  m_totalRetransmissions = 0;
  m_totalAck = 0;
  m_totalNack = 0;
  m_totalDropped = 0;
  m_harqProcesses.clear ();
  NS_LOG_INFO ("[HARQ] Statistics reset");
}

} // namespace ns3
