/*
 * traffic-models.cc
 *
 * GEO traffic helper that keeps the original geo-sat API surface while
 * wrapping reusable ns-3 HTTP / FTP / VoIP applications.
 */

#include "traffic-models.h"

#include "terminal-profile.h"

#include "ns3/address.h"
#include "ns3/bulk-send-application.h"
#include "ns3/bulk-send-helper.h"
#include "ns3/application.h"
#include "ns3/attribute.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/log.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/object-factory.h"
#include "ns3/ping-helper.h"
#include "ns3/ping.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/tcp-hybla.h"
#include "ns3/tcp-header.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-congestion-ops.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/three-gpp-http-client.h"      // HTTP:复用的 3GPP HTTP 客户端(浏览状态机)
#include "ns3/three-gpp-http-helper.h"      // HTTP:client/server 的安装 helper
#include "ns3/three-gpp-http-server.h"      // HTTP:复用的 3GPP HTTP 服务端(被动响应)
#include "ns3/three-gpp-http-variables.h"   // HTTP:对象大小/数量/思考时间的随机分布
#include "ns3/traffic-generator.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"
#include "ns3/random-variable-stream.h"

#include <algorithm>
#include <cmath>

namespace ns3 {

namespace {

// 任务1:VoIP 帧大小 / 帧间隔 / 采样率 / payload type 全部改为可配 (见 SatTrafficGenerator
// 成员 m_voip*),默认预设 AMR 12.2k @ 8 kHz:active≈32 B@20 ms、SID≈7 B@160 ms。
// RTP 时间戳按真实采样数推进:timestamp += round(帧间隔ms * sampleRateHz/1000),
// active 20 ms@8 kHz=160,silence 160 ms@8 kHz=1280,不再恒 +160。这里只保留 ON/OFF 时长常量。
//
// 标准语音 ON/OFF 改用指数分布(任务2)。talk spurt 均值取原 [0.8,1.8] 区间中值约 1.3s;
// silence/turn gap 均值取原 [0.15,0.45] 区间中值约 0.3s(3GPP 习惯值)。
static constexpr double kVoipTalkSpurtMeanSeconds = 1.3;
static constexpr double kVoipTurnGapMeanSeconds = 0.3;
// 把帧间隔(ms)与采样率换算成 RTP 时间戳步进(经过的采样数,四舍五入)。
inline uint32_t
VoipRtpTimestampStep (uint32_t frameMs, uint32_t sampleRateHz)
{
  return static_cast<uint32_t> (
    std::llround (static_cast<double> (frameMs) * static_cast<double> (sampleRateHz) / 1000.0));
}
static constexpr uint32_t kIpv4HeaderBytes = 20;
static constexpr uint32_t kUdpHeaderBytes = 8;

std::string//把 nodeId 和 type 拼成字符串
MakeTraceContext (uint32_t nodeId, TrafficModelType type)
{
  return std::to_string (nodeId) + ":" + std::to_string (static_cast<int> (type));
}

bool//把字符串拆回 nodeId 和 type
ParseTraceContext (const std::string& context, uint32_t& nodeId, TrafficModelType& type)
{
  std::size_t separator = context.find (':');//冒号是分隔点
  if (separator == std::string::npos)
    {
      return false;
    }

  nodeId = static_cast<uint32_t> (std::stoul (context.substr (0, separator)));//读取冒号前面的内容，转成 nodeId
  type = static_cast<TrafficModelType> (std::stoi (context.substr (separator + 1)));//读取冒号后面的内容，转成业务类型枚举
  return true;
}

bool
ParseIpv4TcpTraceContext (const std::string& context,
                          uint32_t& nodeId,
                          TrafficModelType& type,
                          uint16_t& trackedPort,
                          bool& matchSourcePort)
{
  std::size_t first = context.find (':');
  std::size_t second = context.find (':', first == std::string::npos ? first : first + 1);
  std::size_t third = context.find (':', second == std::string::npos ? second : second + 1);
  if (first == std::string::npos || second == std::string::npos || third == std::string::npos)
    {
      return false;
    }

  nodeId = static_cast<uint32_t> (std::stoul (context.substr (0, first)));
  type = static_cast<TrafficModelType> (std::stoi (context.substr (first + 1, second - first - 1)));
  trackedPort = static_cast<uint16_t> (std::stoul (context.substr (second + 1, third - second - 1)));
  matchSourcePort = std::stoi (context.substr (third + 1)) != 0;
  return true;
}

// 任务3:为"内嵌对象数"选择分布。当截断泊松均值较大(>~10)时,ThreeGppHttpVariables
// 内部的 Knuth 乘积法平均需要 (mean+1) 次均匀采样,效率随 mean 线性下降;此时退回到
// 上游自带的截断 Pareto(SetUsePoissonEmbeddedObjects(false)),既保持重尾页面结构又避免
// 低效合成。阈值以下仍用截断泊松(语义与原配置一致)。
//
// 注意:这里配置的是"单页内嵌对象数"分布,与 SatTrafficGenerator 的
// "会话/页面到达 Poisson"(HttpSessionArrivalRate)是两个完全不同的概念。
constexpr double kHttpEmbeddedPoissonEfficientMaxMean = 10.0;

void
ApplyEmbeddedObjectCountDistribution (Ptr<ThreeGppHttpVariables> variables,
                                      double poissonMean,
                                      uint32_t maxObjects)
{
  if (poissonMean <= kHttpEmbeddedPoissonEfficientMaxMean)
    {
      variables->SetUsePoissonEmbeddedObjects (true);
      variables->SetNumOfEmbeddedObjectsPoissonMean (poissonMean);
      variables->SetNumOfEmbeddedObjectsMax (maxObjects);
    }
  else
    {
      // 大 mean:退回截断 Pareto(上游默认 body),避免 Knuth 低效采样。
      variables->SetUsePoissonEmbeddedObjects (false);
      variables->SetNumOfEmbeddedObjectsMax (maxObjects);
    }
}

TrafficModelType
NormalizeTrafficModelType (TrafficModelType type)
{
  switch (type)
    {
    case TRAFFIC_LEGACY_DATA:
      return TRAFFIC_HTTP;
    case TRAFFIC_LEGACY_VOICE:
      return TRAFFIC_VOIP_RTP;
    default:
      return type;
    }
}

} // namespace

NS_LOG_COMPONENT_DEFINE ("SatTrafficGenerator");
NS_OBJECT_ENSURE_REGISTERED (SatTrafficGenerator);

// ============================================================================
// SatRtpHeader (12 B RFC 3550)
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED (SatRtpHeader);

TypeId
SatRtpHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::SatRtpHeader")
    .SetParent<Header> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatRtpHeader> ();
  return tid;
}

TypeId
SatRtpHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
SatRtpHeader::GetSerializedSize () const
{
  return 12;
}

void//把时间戳、序列号等信息压缩进 12 个字节的二进制报头里
SatRtpHeader::Serialize (Buffer::Iterator start) const
{
  // Byte 0: V(2)=2, P(1)=0, X(1)=0, CC(4)=0
  start.WriteU8 (0x80);//处理 RTP 报头的第 0 个字节
  // Byte 1: M(1) | PT(7)
  start.WriteU8 (static_cast<uint8_t> ((m_marker ? 0x80 : 0x00) | (m_payloadType & 0x7F)));//处理 RTP 报头的第 1 个字节
  start.WriteHtonU16 (m_seq);//处理剩下的 10 个字节
  start.WriteHtonU32 (m_timestamp);
  start.WriteHtonU32 (m_ssrc);
}

uint32_t
SatRtpHeader::Deserialize (Buffer::Iterator start)
{
  start.ReadU8 ();                         // V/P/X/CC, 当前实现忽略扩展
  const uint8_t mpt = start.ReadU8 ();
  m_marker = (mpt & 0x80) != 0;
  m_payloadType = mpt & 0x7F;
  m_seq = start.ReadNtohU16 ();
  m_timestamp = start.ReadNtohU32 ();
  m_ssrc = start.ReadNtohU32 ();
  return 12;
}

void
SatRtpHeader::Print (std::ostream& os) const
{
  os << "RTP[seq=" << m_seq << " ts=" << m_timestamp
     << " ssrc=0x" << std::hex << m_ssrc << std::dec
     << (m_marker ? " M" : "") << "]";
}

// ============================================================================
// SatFlowDriverApp: 把 driver 挂到 Application 生命周期上
// ============================================================================

NS_OBJECT_ENSURE_REGISTERED (SatFlowDriverApp);

TypeId
SatFlowDriverApp::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::SatFlowDriverApp")
    .SetParent<Application> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatFlowDriverApp> ();
  return tid;
}

SatFlowDriverApp::SatFlowDriverApp () = default;
SatFlowDriverApp::~SatFlowDriverApp () = default;

void
SatFlowDriverApp::SetStartCallback (std::function<void ()> cb)
{
  m_startCb = std::move (cb);
}

void
SatFlowDriverApp::SetStopCallback (std::function<void ()> cb)
{
  m_stopCb = std::move (cb);
}

void
SatFlowDriverApp::StartApplication ()
{
  if (m_startCb)
    {
      m_startCb ();
    }
}

void
SatFlowDriverApp::StopApplication ()
{
  if (m_stopCb)
    {
      m_stopCb ();
    }
}

TypeId
SatTrafficGenerator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatTrafficGenerator")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatTrafficGenerator> ()
    .AddAttribute ("TargetRate",//注册 TargetRate 属性
                   "Target data rate for Full Buffer mode (bits/s)",
                   UintegerValue (10000000),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_targetRateBps),
                   MakeUintegerChecker<uint64_t> (1000, 1000000000))
    .AddAttribute ("GeoRtt",
                   "GEO satellite round-trip time",
                   TimeValue (MilliSeconds (630)),//GEO 卫星往返时延属性
                   MakeTimeAccessor (&SatTrafficGenerator::m_geoRtt),
                   MakeTimeChecker ())
    .AddAttribute ("MaxPrbPerTti",
                   "Maximum PRBs per TTI for payload calculation",
                   UintegerValue (25),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_maxPrbPerTti),
                   MakeUintegerChecker<uint32_t> (1, 273))
    .AddAttribute ("RohcOverhead",//注册 ROHC 头部压缩开销
                   "ROHC compressed header overhead (bytes)",//估算一个 TTI 中除去头部开销后还能承载多少有效负载
                   UintegerValue (2),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_rohcOverheadBytes),
                   MakeUintegerChecker<uint32_t> (0, 100))
    .AddAttribute ("EnableClosedLoopControl",//注册闭环速率控制开关
                   "Enable closed-loop rate control (MAC queue monitoring)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&SatTrafficGenerator::m_closedLoopControlEnabled),
                   MakeBooleanChecker ())
    // ===== 3GPP TR 37.901 §5.4.2 FTP 模型属性 =====
    .AddAttribute ("FtpModel",
                   "3GPP TR 37.901 FTP traffic model: 1/2/3",
                   UintegerValue (FTP_MODEL_3),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_ftpModel),
                   MakeUintegerChecker<uint8_t> (1, 3))
    .AddAttribute ("FtpFileSizeMean",
                   "FTP truncated log-normal file size mean (bytes)",
                   DoubleValue (2'000'000.0),
                   MakeDoubleAccessor (&SatTrafficGenerator::m_ftpFileSizeMeanBytes),
                   MakeDoubleChecker<double> (1.0))
    .AddAttribute ("FtpFileSizeStdDev",
                   "FTP truncated log-normal file size standard deviation (bytes)",
                   DoubleValue (1'000'000.0),
                   MakeDoubleAccessor (&SatTrafficGenerator::m_ftpFileSizeStdDevBytes),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("FtpFileSizeMin",
                   "FTP file size truncation lower bound (bytes)",
                   UintegerValue (100'000),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_ftpFileSizeMinBytes),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("FtpFileSizeMax",
                   "FTP file size truncation upper bound (bytes)",
                   UintegerValue (5'000'000),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_ftpFileSizeMaxBytes),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("FtpFixedFileSize",
                   "FTP fixed file size in bytes (>0 disables the log-normal distribution)",
                   UintegerValue (0),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_ftpFixedFileSizeBytes),
                   MakeUintegerChecker<uint32_t> (0))
    .AddAttribute ("FtpArrivalRate",
                   "FTP Poisson file arrival rate lambda (files/s) for Model 1/3",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&SatTrafficGenerator::m_ftpArrivalRate),
                   MakeDoubleChecker<double> (1e-6))
    .AddAttribute ("FtpReadingTimeMean",
                   "FTP exponential reading time mean (s) for Model 2",
                   DoubleValue (5.0),
                   MakeDoubleAccessor (&SatTrafficGenerator::m_ftpReadingTimeMeanSeconds),
                   MakeDoubleChecker<double> (0.0))
    .AddAttribute ("FtpTimeoutSafetyFactor",
                   "FTP watchdog timeout = estimated transfer time * this factor + RTT margin",
                   DoubleValue (4.0),
                   MakeDoubleAccessor (&SatTrafficGenerator::m_ftpTimeoutSafetyFactor),
                   MakeDoubleChecker<double> (1.0))
    // ===== 任务1:VoIP codec 属性(默认 AMR 12.2k @ 8 kHz)=====
    .AddAttribute ("VoipActivePayloadBytes",
                   "VoIP active speech frame payload size (bytes); default AMR 12.2k ~32 B",
                   UintegerValue (32),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_voipActivePayloadBytes),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("VoipFrameMs",
                   "VoIP active frame interval (ms); default 20 ms",
                   UintegerValue (20),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_voipFrameMs),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("VoipSilencePayloadBytes",
                   "VoIP silence/comfort-noise (SID) frame payload size (bytes); default ~7 B",
                   UintegerValue (7),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_voipSilencePayloadBytes),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("VoipSilenceFrameMs",
                   "VoIP silence/SID frame interval (ms); default 160 ms (DTX)",
                   UintegerValue (160),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_voipSilenceFrameMs),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("VoipPayloadType",
                   "VoIP RTP payload type (PT); 96=dynamic (AMR), 0=PCMU(G.711)",
                   UintegerValue (96),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_voipPayloadType),
                   MakeUintegerChecker<uint8_t> (0, 127))
    .AddAttribute ("VoipSampleRateHz",
                   "VoIP codec sample rate (Hz); default 8000 (narrowband)",
                   UintegerValue (8000),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_voipSampleRateHz),
                   MakeUintegerChecker<uint32_t> (1))
    // ===== 任务3:HTTP 会话/页面到达 Poisson(独立于内嵌对象数 Poisson)=====
    .AddAttribute ("HttpSessionArrivalRate",
                   "HTTP new-session/page Poisson arrival rate lambda (sessions/s); "
                   "0 disables (pure chained browsing). Distinct from the embedded-object "
                   "count Poisson configured via ThreeGppHttpVariables.",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&SatTrafficGenerator::m_httpSessionArrivalRate),
                   MakeDoubleChecker<double> (0.0));
  return tid;
}

SatTrafficGenerator::SatTrafficGenerator ()
  : m_tcpCongestionTypeId (TcpHybla::GetTypeId ())//把默认 TCP 拥塞控制算法设置为 TcpHybla  Hybla 更适合高 RTT 链路
{
  NS_LOG_FUNCTION (this);
}

SatTrafficGenerator::~SatTrafficGenerator ()
{
  NS_LOG_FUNCTION (this);

  for (const auto& flow : m_flows)
    {
      if (flow)
        {
          flow->active = false;//把 flow 的 active 设为 false，表示停止继续发包
        }
    }
}

TrafficModelType//返回当前主业务模型类型
SatTrafficGenerator::GetTrafficModel () const
{
  if (m_installedTypes.size () == 1)
    {
      return *m_installedTypes.begin ();
    }

  if (m_installedTypes.size () > 1)
    {
      return TRAFFIC_MIXED;
    }

  return m_modelType;//只安装了 FTP，就返回 TRAFFIC_FTP
}

std::vector<TrafficModelType>//返回已经安装过的全部业务类型
SatTrafficGenerator::GetInstalledTrafficModels () const
{
  return std::vector<TrafficModelType> (m_installedTypes.begin (), m_installedTypes.end ());
}

std::string//把已安装业务转换成字符串
SatTrafficGenerator::DescribeInstalledTrafficModels () const
{
  if (m_installedTypes.empty ())
    {
      return "none";//没有安装任何业务
    }

  std::string description;
  for (auto it = m_installedTypes.begin (); it != m_installedTypes.end (); ++it)//遍历所有已安装业务类型
    {
      if (!description.empty ())
        {
          description += "+";//+用于连接多个业务
        }
      description += TrafficModelTypeToString (*it);//业务枚举转换成字符串并拼接
    }
  return description;
}

uint32_t
SatTrafficGenerator::CalculateMaxPayloadBytesPerTti () const//估算每个 TTI 可以承载的最大 payload 字节数
{
  const uint32_t overheadPerPacket = m_rohcOverheadBytes + 2 + 4 + 3 + 20;//简化估算ROHC 开销
  const uint32_t maxBitsPerTti = m_maxPrbPerTti * 12 * 14 * 6;//估算每 TTI 的最大比特数 PRB 数 × 每 PRB 12 个子载波 × 每 slot 14 个符号 × 每 RE 6 bit
  const uint32_t maxBytesPerTti = maxBitsPerTti / 8;//把 bit 转换成 byte
  const uint32_t theoreticalPayload =
    (maxBytesPerTti > overheadPerPacket) ? (maxBytesPerTti - overheadPerPacket) : 1U;
  const uint32_t mtuLimitedPayload =
    (m_mtu > (kIpv4HeaderBytes + kUdpHeaderBytes)) ?
      (m_mtu - kIpv4HeaderBytes - kUdpHeaderBytes) :
      1U;
  return std::max (1U, std::min (theoreticalPayload, mtuLimitedPayload));//返回可承载 payload
}

uint32_t
SatTrafficGenerator::CalculateTcpWindowBytes () const//根据 GEO 链路 RTT 和估计带宽计算 TCP 窗口
{
  const double bandwidthBps = m_maxPrbPerTti * 12 * 14 * 6 * 1000.0;//估算链路带宽 按每毫秒一个 TTI，把每 TTI bit 数转换成 bit/s
  const double bdpBytes = bandwidthBps * m_geoRtt.GetSeconds () / 8.0;//计算 BDP即带宽时延积 单位换成字节
  const uint32_t windowBytes = static_cast<uint32_t> (bdpBytes * 3.0);//TCP 窗口取 BDP 的 3 倍，留出足够空间
  return std::max (windowBytes, 1048576U);//返回 TCP 窗口大小，至少为 1 MB
}

//业务开始前先让网络解析地址，避免第一个业务包因为 ARP 过程被延迟或丢失
ApplicationContainer
InstallArpSeedPings (NodeContainer sourceNodes, Ipv4Address sinkAddress, Time appStartTime)//安装一组 ping 应用，用于在真正业务开始前触发 ARP 解析
{
  PingHelper ping (sinkAddress);//创建 ping helper，目标地址是 sinkAddress
  ping.SetAttribute ("Count", UintegerValue (8));//ping 发送 8 次
  // ping.SetAttribute ("Interval", TimeValue (MilliSeconds (250)));/每 250 ms 一次
  ping.SetAttribute ("VerboseMode", EnumValue (Ping::VerboseMode::SILENT));//静默输出
  ApplicationContainer pingApps = ping.Install (sourceNodes);//把 ping 安装到所有源节点上
  Time pingStart = appStartTime > Seconds (2.0) ? appStartTime - Seconds (2.0) : Seconds (0.1);//如果业务开始时间大于 2 秒，就提前 2 秒启动 ping；否则 0.1 秒启动
  Time pingStop = appStartTime > MilliSeconds (100) ? appStartTime - MilliSeconds (100)//ping 在业务开始前 100 ms 停止 如果业务开始太早，就让 ping 持续 1 秒
                                                    : pingStart + Seconds (1.0);
  //防止停止时间早于启动时间
  if (pingStop <= pingStart)
    {
      pingStop = pingStart + Seconds (1.0);
    }
  pingApps.Start (pingStart);
  pingApps.Stop (pingStop);
  return pingApps;
}

//InstallFullBuffer，安装 Full Buffer UDP 流
ApplicationContainer//安装 Full Buffer 类型流量 固定目标速率的 UDP 发包器
SatTrafficGenerator::InstallFullBuffer (NodeContainer sourceNodes,
                                        Ipv4Address sinkAddress,
                                        bool isUplink)
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);

  m_modelType = TRAFFIC_FULL_BUFFER;//把当前模型设为 Full Buffer
  MarkTrafficModelInstalled (TRAFFIC_FULL_BUFFER);//把 Full Buffer 记录到已安装模型集合里

  ApplicationContainer apps;//创建应用容器
  PruneInactiveFlows ();
  apps.Add (InstallArpSeedPingsOnce (sourceNodes, sinkAddress));//先安装 ARP 预热 ping
  //为每个源节点创建 UDP socket
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)//遍历所有源节点
    {
      Ptr<Node> node = sourceNodes.Get (i);

      Ptr<Socket> udpSocket = Socket::CreateSocket (node, UdpSocketFactory::GetTypeId ());//在当前节点创建 UDP socket
      udpSocket->SetAllowBroadcast (false);//禁止广播
      udpSocket->Connect (InetSocketAddress (sinkAddress, LEGACY_PORT));//连接到目标地址的 LEGACY_PORT

      const uint32_t packetSize = CalculateMaxPayloadBytesPerTti ();//计算每个包大小
      //根据目标速率计算发送间隔 发送间隔 = 包大小 bit / 目标速率 bit/s
      const double intervalSeconds =
          (static_cast<double> (packetSize) * 8.0) / static_cast<double> (m_targetRateBps);
      const Time baseInterval = Seconds (std::max (0.001, intervalSeconds));

      Ptr<SatFlowDriverApp> app = CreateObject<SatFlowDriverApp> ();
      node->AddApplication (app);
      app->SetStartTime (m_appStartTime);
      app->SetStopTime (m_appStopTime);
      m_driverApps.push_back (app);

      auto flow = std::make_shared<FlowState> ();
      flow->socket = udpSocket;
      flow->type = TRAFFIC_FULL_BUFFER;
      flow->nodeId = node->GetId ();
      flow->packetSize = packetSize;
      flow->baseInterval = baseInterval;
      flow->active = false; // 待 StartApplication 触发后置 true

      std::weak_ptr<FlowState> weak = flow;
      flow->driver = [this, weak] () {
        auto self = weak.lock ();
        if (!self || !self->active)
          {
            return;
          }

        Ptr<Packet> pkt = Create<Packet> (self->packetSize);
        int sent = self->socket->Send (pkt);
        if (sent > 0)
          {
            // wire = L4 payload + UDP(8) + IPv4(20),与 TCP 业务的 L3+ 统计口径对齐
            const uint32_t wireBytes =
              static_cast<uint32_t> (sent) + kIpv4HeaderBytes + kUdpHeaderBytes;
            AccountTx (*self, wireBytes, static_cast<uint32_t> (sent));
          }
        else
          {
            m_totalStats.sendBlockedCount++;
            m_statsByType[TRAFFIC_FULL_BUFFER].sendBlockedCount++;
          }

        if (self->active)
          {
            const Time nextInterval = ScaledInterval (self->baseInterval);
            Simulator::Schedule (nextInterval, self->driver);
          }
      };

      m_flows.push_back (flow);
      app->SetStartCallback ([weak] () {
        if (auto self = weak.lock ())
          {
            self->started = true;
            self->active = true;
            self->driver ();
          }
      });
      app->SetStopCallback ([this, weak] () {
        if (auto self = weak.lock ())
          {
            self->active = false;
          }
        PruneInactiveFlows ();
      });
      apps.Add (app);
    }

  return apps;
}

// 惰性创建 FTP 随机变量(文件大小=截断对数正态;到达/阅读时间=指数),全部用 ns-3
// 标准随机变量类 CreateObject。3GPP TR 37.901 §5.4.2。
void
SatTrafficGenerator::EnsureFtpRandomVariables ()
{
  if (m_ftpFileSizeRv == nullptr)
    {
      m_ftpFileSizeRv = CreateObject<LogNormalRandomVariable> ();
    }
  // LogNormalRandomVariable 的 Mu/Sigma 是底层正态的参数,需由目标(均值,标准差)换算:
  //   sigma^2 = ln(1 + var/mean^2)
  //   mu      = ln(mean) - sigma^2 / 2
  const double mean = std::max (1.0, m_ftpFileSizeMeanBytes);
  const double stddev = std::max (0.0, m_ftpFileSizeStdDevBytes);
  const double variance = stddev * stddev;
  const double sigma2 = std::log (1.0 + variance / (mean * mean));
  const double sigma = std::sqrt (std::max (0.0, sigma2));
  const double mu = std::log (mean) - 0.5 * sigma2;
  m_ftpFileSizeRv->SetAttribute ("Mu", DoubleValue (mu));
  m_ftpFileSizeRv->SetAttribute ("Sigma", DoubleValue (sigma));

  if (m_ftpArrivalRv == nullptr)
    {
      m_ftpArrivalRv = CreateObject<ExponentialRandomVariable> ();
    }
  // 指数到达间隔均值 = 1/λ (Poisson 文件到达过程)
  m_ftpArrivalRv->SetAttribute ("Mean",
                                DoubleValue (1.0 / std::max (1e-6, m_ftpArrivalRate)));

  if (m_ftpReadingTimeRv == nullptr)
    {
      m_ftpReadingTimeRv = CreateObject<ExponentialRandomVariable> ();
    }
  m_ftpReadingTimeRv->SetAttribute ("Mean",
                                    DoubleValue (std::max (0.0, m_ftpReadingTimeMeanSeconds)));
}

// 采样一个文件大小:固定模式直接返回;否则用截断对数正态 do-while 截断到 [min,max]。
uint64_t
SatTrafficGenerator::SampleFtpFileSize ()
{
  if (m_ftpFixedFileSizeBytes > 0)
    {
      return m_ftpFixedFileSizeBytes;   // 固定大小模式
    }

  EnsureFtpRandomVariables ();
  const double lo = static_cast<double> (m_ftpFileSizeMinBytes);
  const double hi = static_cast<double> (std::max (m_ftpFileSizeMinBytes, m_ftpFileSizeMaxBytes));
  double sample = 0.0;
  uint32_t guard = 0;
  do
    {
      sample = m_ftpFileSizeRv->GetValue ();   // 截断对数正态:循环重采样到落在 [min,max]
    }
  while ((sample < lo || sample > hi) && ++guard < 1000);
  sample = std::min (std::max (sample, lo), hi);
  return static_cast<uint64_t> (sample);
}

// ===================================================================
// 公共采样/查询访问器 (functional-demo.cc 专用) - 见 .h 注释
// ===================================================================
uint64_t
SatTrafficGenerator::SampleFtpFileSizePublic ()
{
  return SampleFtpFileSize ();   // 复用真实业务的截断对数正态采样
}

void
SatTrafficGenerator::GetFtpLogNormalParams (double& mu, double& sigma)
{
  EnsureFtpRandomVariables ();   // 确保 Mu/Sigma 已按当前 mean/std 反算并写入
  DoubleValue muVal;
  DoubleValue sigmaVal;
  m_ftpFileSizeRv->GetAttribute ("Mu", muVal);
  m_ftpFileSizeRv->GetAttribute ("Sigma", sigmaVal);
  mu = muVal.Get ();
  sigma = sigmaVal.Get ();
}

double
SatTrafficGenerator::SampleVoipTalkSpurtSeconds ()
{
  // 与真实 VoIP 会话状态机使用相同的指数分布均值(kVoipTalkSpurtMeanSeconds),
  // 但用独立采样实例, 不触动任何运行中的会话。
  if (m_voipTalkSampleRv == nullptr)
    {
      m_voipTalkSampleRv = CreateObject<ExponentialRandomVariable> ();
      m_voipTalkSampleRv->SetAttribute ("Mean", DoubleValue (kVoipTalkSpurtMeanSeconds));
    }
  return m_voipTalkSampleRv->GetValue ();
}

double
SatTrafficGenerator::SampleVoipSilenceGapSeconds ()
{
  if (m_voipGapSampleRv == nullptr)
    {
      m_voipGapSampleRv = CreateObject<ExponentialRandomVariable> ();
      m_voipGapSampleRv->SetAttribute ("Mean", DoubleValue (kVoipTurnGapMeanSeconds));
    }
  return m_voipGapSampleRv->GetValue ();
}

// 为某 FTP driver 动态创建一个新文件的 BulkSendApplication:采样文件大小、装到节点、
// 配 GEO TCP socket(TcpHybla + 大窗口)。
// 任务2:完成检测改用 socket NormalClose(BulkSend 发完去重传字节后会主动关闭 socket),
// 而不是累计 Tx trace 字节(含重传会偏早完成);并排一个超时看门狗兜底防止卡死。
Ptr<BulkSendApplication>
SatTrafficGenerator::FtpSpawnFile (const std::shared_ptr<FtpDriverState>& st, Time startDelay)
{
  if (!st || !st->active || st->node == nullptr)
    {
      return nullptr;
    }

  const uint64_t fileSize = SampleFtpFileSize ();
  ++m_ftpFilesSpawned;   // 功能验证: 统计本生成器发起的 FTP 文件数(Model 1/2/3 节奏对比)
  const uint32_t tcpWindowBytes = CalculateTcpWindowBytes ();

  BulkSendHelper bulkHelper ("ns3::TcpSocketFactory",
                             InetSocketAddress (st->sinkAddress, FTP_PORT));
  bulkHelper.SetAttribute ("SendSize", UintegerValue (m_mtu - TCP_HEADER_BYTES));
  bulkHelper.SetAttribute ("MaxBytes", UintegerValue (fileSize));   // 每文件按采样大小发送

  ApplicationContainer bulkApps = bulkHelper.Install (st->node);
  Ptr<BulkSendApplication> bulk = DynamicCast<BulkSendApplication> (bulkApps.Get (0));
  if (bulk == nullptr)
    {
      return nullptr;
    }

  const Time now = Simulator::Now ();
  Time start = now + startDelay;
  if (start < m_appStartTime)
    {
      start = m_appStartTime;     // 不早于业务窗口开始
    }
  bulk->SetStartTime (start);
  bulk->SetStopTime (m_appStopTime);

  ScheduleManagedTcpSocketRegistration (bulk, tcpWindowBytes, tcpWindowBytes);
  m_managedApps.push_back (bulk);

  // 为本文件分配单调递增 id,完成回调与看门狗都带上它,只对"当前在飞文件"生效。
  const uint64_t fileId = st->nextFileId++;
  st->currentFileId = fileId;
  st->currentFileBytesSent = 0;
  st->currentFileSize = fileSize;
  st->fileInFlight = true;
  st->currentBulk = bulk;

  std::weak_ptr<FtpDriverState> weak = st;
  // Tx trace 仅累计进度(不再用于判完成),便于诊断/统计。
  Callback<void, Ptr<const Packet>> txCb =
    [this, weak] (Ptr<const Packet> packet) { FtpOnFileTx (weak, packet); };
  bulk->TraceConnectWithoutContext ("Tx", txCb);

  // 任务2-完成检测:socket 创建在 StartApplication 之后,延迟轮询挂 NormalClose 回调。
  FtpScheduleCloseHook (weak, bulk, fileId);

  // 任务2-超时兜底:估计传输时间 = fileSize*8 / 有效速率,乘安全系数,再加一个 RTT 余量;
  // 还要算上 start 相对 now 的延迟(reading time / 排队)。到期若该文件仍在飞则视为失败。
  const double rateBps = std::max<double> (1.0, static_cast<double> (m_targetRateBps));
  const double estTransferSec = (static_cast<double> (fileSize) * 8.0) / rateBps;
  Time timeout = Seconds (estTransferSec * std::max (1.0, m_ftpTimeoutSafetyFactor))
                 + m_geoRtt * 2          // 慢启动 / 关闭握手余量
                 + (start - now);        // 文件实际起始相对当前的延迟
  if (!timeout.IsStrictlyPositive ())
    {
      timeout = m_geoRtt * 2 + Seconds (1.0);
    }
  Simulator::Schedule (timeout, &SatTrafficGenerator::FtpWatchdog, this, weak, fileId);

  return bulk;
}

// socket 创建在 StartApplication 之后,这里延迟轮询取 socket 并挂 NormalClose 回调。
void
SatTrafficGenerator::FtpScheduleCloseHook (std::weak_ptr<FtpDriverState> weak,
                                           Ptr<BulkSendApplication> bulk,
                                           uint64_t fileId,
                                           uint32_t retriesRemaining)
{
  if (bulk == nullptr)
    {
      return;
    }

  Time delay;
  if (Simulator::Now () < m_appStartTime)
    {
      delay = (m_appStartTime - Simulator::Now ()) + MicroSeconds (1);
    }
  else
    {
      delay = MilliSeconds (10);
    }

  Simulator::Schedule (delay, [this, weak, bulk, fileId, retriesRemaining] () mutable {
    auto st = weak.lock ();
    if (!st || !st->active)
      {
        return;
      }
    // 文件已被更新的 spawn 取代(id 不匹配)→ 本 hook 失效,无需再挂。
    if (st->currentFileId != fileId)
      {
        return;
      }
    Ptr<Socket> socket = bulk->GetSocket ();
    if (socket != nullptr)
      {
        // NormalClose:BulkSend 发完(去重传)后主动 Close() → 真正完成信号。
        // ErrorClose 也兜底推进(异常关闭不能让 driver 卡死),与正常完成同样处理。
        Callback<void, Ptr<Socket>> closeCb =
          [this, weak, fileId] (Ptr<Socket>) { FtpOnSocketClose (weak, fileId); };
        socket->SetCloseCallbacks (closeCb, closeCb);
        return;
      }
    if (retriesRemaining == 0)
      {
        return;   // 取不到 socket 也无妨:看门狗仍会兜底推进。
      }
    FtpScheduleCloseHook (weak, bulk, fileId, retriesRemaining - 1);
  });
}

// BulkSend 的 Tx trace 回调:仅累计当前文件已发字节作进度参考(不再据此判完成)。
void
SatTrafficGenerator::FtpOnFileTx (std::weak_ptr<FtpDriverState> weak, Ptr<const Packet> packet)
{
  auto st = weak.lock ();
  if (!st || !st->active || packet == nullptr)
    {
      return;
    }
  st->currentFileBytesSent += packet->GetSize ();
}

// 任务2:socket NormalClose 回调——真正发完(去重传)才触发完成,只对当前在飞文件生效。
void
SatTrafficGenerator::FtpOnSocketClose (std::weak_ptr<FtpDriverState> weak, uint64_t fileId)
{
  auto st = weak.lock ();
  if (!st || !st->active)
    {
      return;
    }
  // 迟到/串台保护:只有 fileId 与当前在飞文件一致且仍 inFlight 才推进。
  if (!st->fileInFlight || st->currentFileId != fileId)
    {
      return;
    }
  st->fileInFlight = false;
  ++st->filesCompleted;
  FtpOnFileComplete (st);
}

// 任务2:超时兜底——到期若该文件(fileId)仍在飞,则视为传输失败,复位 fileInFlight
// 并按模型推进,避免 socket 异常关闭时 fileInFlight 永真导致 driver 卡死、
// Model 3 的 pendingArrivals 无界堆积。
void
SatTrafficGenerator::FtpWatchdog (std::weak_ptr<FtpDriverState> weak, uint64_t fileId)
{
  auto st = weak.lock ();
  if (!st || !st->active)
    {
      return;
    }
  // 该文件已正常完成(被新文件取代或 fileInFlight 已清)→ 看门狗无需动作。
  if (!st->fileInFlight || st->currentFileId != fileId)
    {
      return;
    }
  NS_LOG_WARN ("FTP file " << fileId << " on node " << st->nodeId
               << " timed out; failing over to keep the driver alive.");
  st->fileInFlight = false;
  ++st->filesTimedOut;
  FtpOnFileComplete (st);   // 与正常完成同样推进模型(Model3 取队首 / Model2 排下一个)
}

// 一个文件传输完成后的处理,按 3GPP 三种模型语义推进:
void
SatTrafficGenerator::FtpOnFileComplete (const std::shared_ptr<FtpDriverState>& st)
{
  if (!st || !st->active)
    {
      return;
    }

  switch (m_ftpModel)
    {
    case FTP_MODEL_2:
      // Model 2:串行——等一个指数 reading time 后再传下一个文件。
      {
        EnsureFtpRandomVariables ();
        const Time readingTime = Seconds (m_ftpReadingTimeRv->GetValue ());
        FtpSpawnFile (st, readingTime);
      }
      break;
    case FTP_MODEL_3:
      // Model 3:Poisson 到达但同一时刻只允许一个活动传输;若有排队的到达则立即接着传队首。
      if (st->pendingArrivals > 0)
        {
          st->pendingArrivals--;
          FtpSpawnFile (st, Time (0));
        }
      break;
    case FTP_MODEL_1:
    default:
      // Model 1:文件间彼此独立、由 Poisson 到达驱动,完成事件无需触发下一个。
      break;
    }
}

// Model 1/3:排下一个 Poisson 文件到达(指数到达间隔)。
void
SatTrafficGenerator::FtpScheduleNextArrival (const std::shared_ptr<FtpDriverState>& st)
{
  if (!st || !st->active)
    {
      return;
    }

  EnsureFtpRandomVariables ();
  const Time interArrival = Seconds (m_ftpArrivalRv->GetValue ());
  std::weak_ptr<FtpDriverState> weak = st;
  Simulator::Schedule (interArrival, &SatTrafficGenerator::FtpOnArrival, this, weak);
}

// Model 1/3 的文件到达事件。
void
SatTrafficGenerator::FtpOnArrival (std::weak_ptr<FtpDriverState> weak)
{
  auto st = weak.lock ();
  if (!st || !st->active)
    {
      return;
    }

  if (m_ftpModel == FTP_MODEL_1)
    {
      // Model 1:每次到达就发起一个新文件传输(允许并发/独立)。
      FtpSpawnFile (st, Time (0));
    }
  else if (m_ftpModel == FTP_MODEL_3)
    {
      // Model 3:到达时若上一个未完成则排队,否则立即开始。
      if (st->fileInFlight)
        {
          st->pendingArrivals++;
        }
      else
        {
          FtpSpawnFile (st, Time (0));
        }
    }

  FtpScheduleNextArrival (st);   // 继续排下一个到达,形成 Poisson 过程
}

// 在节点上安装一个 FTP driver(SatFlowDriverApp 承载生命周期),并返回第一个文件的 BulkSend
// 供调用方/验证程序取用(保持 ftpApps 第 1 项是 BulkSendApplication 的历史行为)。
Ptr<SatFlowDriverApp>
SatTrafficGenerator::InstallFtpDriver (Ptr<Node> node,
                                       Ipv4Address sinkAddress,
                                       Ptr<BulkSendApplication>& firstBulkOut)
{
  auto st = std::make_shared<FtpDriverState> ();
  st->node = node;
  st->sinkAddress = sinkAddress;
  st->nodeId = node->GetId ();
  st->active = false;   // 待 StartApplication 触发
  m_ftpDrivers.push_back (st);

  Ptr<SatFlowDriverApp> app = CreateObject<SatFlowDriverApp> ();
  node->AddApplication (app);
  app->SetStartTime (m_appStartTime);
  app->SetStopTime (m_appStopTime);
  m_driverApps.push_back (app);

  std::weak_ptr<FtpDriverState> weak = st;
  app->SetStartCallback ([this, weak] () {
    auto self = weak.lock ();
    if (!self)
      {
        return;
      }
    self->active = true;
    // 第一个文件已在 InstallFtpDriver 里预创建(其 BulkSend 会按 m_appStartTime 自启动)。
    //  - Model 2:仅靠 FtpOnFileComplete 串行链式触发下一个文件,这里无需额外动作。
    //  - Model 1/3:在第一个文件之外,还要启动 Poisson 文件到达流。
    if (m_ftpModel != FTP_MODEL_2)
      {
        FtpScheduleNextArrival (self);
      }
  });
  app->SetStopCallback ([weak] () {
    if (auto self = weak.lock ())
      {
        self->active = false;
      }
  });

  // 预创建第一个文件的 BulkSend,使 InstallFtp 返回的容器第 1 项始终是 BulkSendApplication
  // (兼容 traffic-model-validation.cc)。对 Model 2 不预创建,改由 StartCallback 触发;
  // 但仍创建一个以保证返回项类型,Model 2 的 StartCallback 再创建后续文件。
  st->active = true;                                  // 允许 FtpSpawnFile 预创建
  Ptr<BulkSendApplication> firstBulk = FtpSpawnFile (st, Time (0));
  st->active = false;                                 // 复位,等 StartApplication 正式置 true
  // FtpSpawnFile 已把 fileInFlight 置 true、并挂好 Tx trace 与窗口配置。
  firstBulkOut = firstBulk;
  return app;
}

ApplicationContainer
SatTrafficGenerator::InstallFtp (NodeContainer sourceNodes,
                                 Ipv4Address sinkAddress,
                                 bool isUplink)
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);

  m_modelType = TRAFFIC_FTP;
  MarkTrafficModelInstalled (TRAFFIC_FTP);

  ApplicationContainer apps;
  PruneInactiveFlows ();
  EnsureFtpRandomVariables ();
  apps.Add (InstallArpSeedPingsOnce (sourceNodes, sinkAddress));
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);

      Ptr<BulkSendApplication> firstBulk;
      InstallFtpDriver (node, sinkAddress, firstBulk);

      RegisterIpv4TcpTxTrace (node, TRAFFIC_FTP, FTP_PORT, false);
      // 返回容器第 1 项放第一个文件的 BulkSend(保持原 ftpApps.Get(1) 语义)。
      if (firstBulk)
        {
          apps.Add (firstBulk);
        }
    }

  return apps;
}

// HTTP 业务对外入口:在 sourceNodes 上安装标准浏览(portable 档,非轻量)HTTP 流量。
ApplicationContainer
SatTrafficGenerator::InstallHttp (NodeContainer sourceNodes,   // 承载 HTTP 的源节点集合(下行时为内容服务器)
                                  Ipv4Address sinkAddress,     // 对端地址(下行时为 UE/client 地址)
                                  bool isUplink)               // true=上行上传,false=下行浏览
{
  // 直接转调内部实现,最后一个参数 lightweightProfile=false 表示使用宽带 portable 浏览档位。
  return InstallHttpInternal (sourceNodes, sinkAddress, isUplink, false);
}

// HTTP 业务真正实现:复用 ns-3 的 ThreeGppHttpClient/Server,按上行/下行两套角色部署。
ApplicationContainer
SatTrafficGenerator::InstallHttpInternal (NodeContainer sourceNodes,   // 源节点集合
                                          Ipv4Address sinkAddress,     // 对端地址
                                          bool isUplink,               // true=上行上传 false=下行浏览
                                          bool lightweightProfile)     // true=消费级手机轻量档 false=宽带 portable 档
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);   // ns-3 函数级日志,打印入参便于调试

  m_modelType = TRAFFIC_HTTP;                  // 把当前生成器的“当前模型”标记为 HTTP
  MarkTrafficModelInstalled (TRAFFIC_HTTP);    // 把 HTTP 登记进“已安装模型集合”,供统计/描述使用

  Ptr<Node> sinkNode = FindNodeByAddress (sinkAddress);   // 由 IP 反查对端节点对象(需先 InstallSink 登记地址)
  if (sinkNode == nullptr)                                // 找不到说明地址尚未登记
    {
      NS_LOG_ERROR ("Unable to map sink address " << sinkAddress   // 报错:必须在 IP 分配后先调用 InstallSink()
                    << " to a node. Call InstallSink() after IP assignment first.");
      return ApplicationContainer ();                     // 直接返回空容器,放弃本次安装
    }

  ApplicationContainer apps;     // 收集本次安装产生的全部应用(client/ARP 预热等),最终返回
  PruneInactiveFlows ();         // 清理已停止的旧流,避免重复安装时残留
  if (isUplink)                  // ===== 上行上传分支:UE 作 client 上传,sink 端作 server 接收 =====
    {

      Ptr<ThreeGppHttpServer> sinkServer = EnsureHttpServer (sinkNode,            // 在 sink 节点上确保有一个 HTTP server
                                                             sinkAddress,         // server 监听的本地地址
                                                             lightweightProfile,  // 透传画像档位
                                                             true);               // uploadProfile=true:用上传档对象大小
      if (sinkServer == nullptr)        // server 创建失败(无 IP 等)
        {
          return ApplicationContainer ();   // 放弃安装
        }

      for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)   // 遍历每个上传发起方(远端节点)
        {
          Ptr<Node> remoteNode = sourceNodes.Get (i);                       // 取出当前远端节点
          Ipv4Address remoteAddress = GetPrimaryIpv4Address (remoteNode);   // 取该节点主 IPv4 地址
          if (remoteAddress == Ipv4Address::GetAny ())                      // 无有效地址则跳过
            {
              NS_LOG_WARN ("Skipping HTTP upload requester on node " << remoteNode->GetId ()
                           << " because it has no IPv4 address.");
              continue;
            }

          ThreeGppHttpClientHelper clientHelper{Address (sinkAddress)};                 // 创建 HTTP client helper,目标=sink server
          clientHelper.SetAttribute ("RemoteServerPort", UintegerValue (HTTP_PORT));    // 指定连接端口 80

          ApplicationContainer clientApps = clientHelper.Install (remoteNode);                       // 在远端节点装上 HTTP client
          apps.Add (InstallArpSeedPingsOnce (NodeContainer (remoteNode), sinkAddress));              // 远端→sink 方向预热 ARP
          apps.Add (InstallArpSeedPingsOnce (NodeContainer (sinkNode), remoteAddress));              // sink→远端 方向预热 ARP
          clientApps.Start (m_appStartTime);    // client 启动时间
          clientApps.Stop (m_appStopTime);      // client 停止时间

          Ptr<ThreeGppHttpClient> client = clientApps.Get (0)->GetObject<ThreeGppHttpClient> ();   // 取出 client 对象指针
          if (client)
            {
              m_managedApps.push_back (client);    // 纳入受管应用列表,ResetStats 时统一停掉
              RegisterTraceConnectWithoutContext (                          // 挂“连接建立”回调:握手成功时下发 GEO 窗口
                client,
                "ConnectionEstablished",
                MakeCallback (&SatTrafficGenerator::OnHttpClientConnectionEstablished, this));
              PointerValue variablesPtr;                          // 用于接收 client 内部的随机变量对象指针
              client->GetAttribute ("Variables", variablesPtr);   // 取出 client 的 ThreeGppHttpVariables
              ConfigureHttpVariables (variablesPtr.Get<ThreeGppHttpVariables> (),   // 按画像配置 client 侧请求体等参数
                                      lightweightProfile,
                                      true);                                       

              const uint32_t remoteNodeId = remoteNode->GetId ();                       // 当前节点 id,用于统计上下文
              std::string rxContext = MakeTraceContext (remoteNodeId, TRAFFIC_HTTP);    
              RegisterTraceConnect (                          // 挂“收到主对象分片”trace → 计入接收统计
                client,
                "RxMainObjectPacket",
                rxContext,
                MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
              RegisterTraceConnect (                          // 挂“收到内嵌对象分片”trace → 计入接收统计
                client,
                "RxEmbeddedObjectPacket",
                rxContext,
                MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
            }

          RegisterIpv4TcpTxTrace (sinkNode, TRAFFIC_HTTP, HTTP_PORT, true);   // 在 sink(server)IPv4 层按源端口80统计发出流量
          apps.Add (clientApps);     // 把 client 加入返回容器

          // 任务3:登记上行上传方(远端)的 Poisson 会话到达 spawn 上下文(λ>0 时启用)。
          HttpArrivalContext arrivalCtx;
          arrivalCtx.clientNode = remoteNode;
          arrivalCtx.serverAddress = sinkAddress;
          arrivalCtx.clientAddress = remoteAddress;
          arrivalCtx.rxNodeId = remoteNode->GetId ();
          arrivalCtx.statType = TRAFFIC_HTTP;
          arrivalCtx.serverPort = HTTP_PORT;
          arrivalCtx.lightweightProfile = lightweightProfile;
          arrivalCtx.uploadProfile = true;
          RegisterHttpSessionArrival (arrivalCtx);
        }
      return apps;       // 上行分支安装完成,返回
    }

  // ===== 下行浏览分支:sourceNodes 作内容服务器,sinkAddress 所在节点(UE)作 HTTP client 拉取内容 =====
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)   // 遍历每个内容服务器节点
    {
      Ptr<Node> sourceNode = sourceNodes.Get (i);                       // 当前内容服务器节点
      Ipv4Address localAddress = GetPrimaryIpv4Address (sourceNode);    // 取其主 IPv4 地址(server 监听地址)
      if (localAddress == Ipv4Address::GetAny ())                       // 无地址则跳过
        {
          NS_LOG_WARN ("Skipping HTTP install on node " << sourceNode->GetId ()
                       << " because it has no IPv4 address.");
          continue;
        }


      Ptr<ThreeGppHttpServer> server =
        EnsureHttpServer (sourceNode, localAddress, lightweightProfile, false);   // 在内容服务器节点上确保有 HTTP server(下行档)
      if (server == nullptr)        // 创建失败则跳过该节点
        {
          continue;
        }

      ThreeGppHttpClientHelper clientHelper{Address (localAddress)};                 // 创建 client helper,目标=该内容服务器
      clientHelper.SetAttribute ("RemoteServerPort", UintegerValue (HTTP_PORT));     // 连接端口

      ApplicationContainer clientApps = clientHelper.Install (sinkNode);             // 在 UE(sinkNode)上装 HTTP client
      // HTTP in this model has the UE as TCP client and the remote host as server.
      // The critical return path is server -> UE (SYN-ACK / HTTP response), so
      // pre-warm the path from content server toward the UE, matching the setup
      // used by the upstream NR mixed-traffic examples.
      apps.Add (InstallArpSeedPingsOnce (NodeContainer (sourceNode), sinkAddress));   // server→UE 方向预热 ARP(关键返回路径)
      apps.Add (InstallArpSeedPingsOnce (NodeContainer (sinkNode), localAddress));    // UE→server 方向预热 ARP
      clientApps.Start (m_appStartTime);    // client 启动时间
      clientApps.Stop (m_appStopTime);      // client 停止时间

      Ptr<ThreeGppHttpClient> client = clientApps.Get (0)->GetObject<ThreeGppHttpClient> ();   // 取出 client 对象
      if (client)
        {
          m_managedApps.push_back (client);   
          RegisterTraceConnectWithoutContext (                          // 挂“连接建立”回调:下发 GEO 窗口
            client,
            "ConnectionEstablished",
            MakeCallback (&SatTrafficGenerator::OnHttpClientConnectionEstablished, this));
          PointerValue variablesPtr;                          // 接收随机变量对象指针
          client->GetAttribute ("Variables", variablesPtr);   // 取出 client 的 Variables
          ConfigureHttpVariables (variablesPtr.Get<ThreeGppHttpVariables> (), lightweightProfile);   // 按画像配置 client 侧参数

          const uint32_t sinkNodeId = sinkNode->GetId ();                       // UE 节点 id(下行接收方)
          std::string rxContext = MakeTraceContext (sinkNodeId, TRAFFIC_HTTP);  // 拼接 "nodeId:HTTP" 统计上下文
          RegisterTraceConnect (                          // 挂主对象收包统计
            client,
            "RxMainObjectPacket",
            rxContext,
            MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
          RegisterTraceConnect (                          // 挂内嵌对象收包统计
            client,
            "RxEmbeddedObjectPacket",
            rxContext,
            MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
        }

      RegisterIpv4TcpTxTrace (sourceNode, TRAFFIC_HTTP, HTTP_PORT, true);   // 在内容服务器 IPv4 层按源端口80统计 HTTP 响应流量
      apps.Add (clientApps);     // 把 client 加入返回容器

      // 任务3:登记下行浏览方(UE)的 Poisson 会话/页面到达 spawn 上下文(λ>0 时启用)。
      HttpArrivalContext arrivalCtx;
      arrivalCtx.clientNode = sinkNode;
      arrivalCtx.serverAddress = localAddress;
      arrivalCtx.clientAddress = sinkAddress;
      arrivalCtx.rxNodeId = sinkNode->GetId ();
      arrivalCtx.statType = TRAFFIC_HTTP;
      arrivalCtx.serverPort = HTTP_PORT;
      arrivalCtx.lightweightProfile = lightweightProfile;
      arrivalCtx.uploadProfile = false;
      RegisterHttpSessionArrival (arrivalCtx);
    }

  return apps;     // 下行分支安装完成,返回
}

// HTTP 下行下载的便捷封装:contentServerNodes 作服务器,clientAddress 所在节点作 client。
ApplicationContainer
SatTrafficGenerator::InstallHttpDownload (NodeContainer contentServerNodes,   // 内容服务器节点集合
                                          Ipv4Address clientAddress)          // 拉取内容的 client 地址
{
  // 等价于下行、宽带档的 InstallHttpInternal。
  return InstallHttpInternal (contentServerNodes, clientAddress, false, false);
}

// ===================================================================
// 任务3:HTTP 会话/页面到达 Poisson(独立于"内嵌对象数 Poisson")
// ===================================================================
// 重要区分:
//  - "内嵌对象数 Poisson"(ThreeGppHttpVariables::SetUsePoissonEmbeddedObjects)决定
//    单个页面里有几个内嵌对象,是页面内部结构;
//  - 这里的"会话/页面到达 Poisson"(HttpSessionArrivalRate)决定多久发起一个全新的
//    页面/会话请求,是会话之间的到达过程。两者命名/语义/默认值完全分开。
void
SatTrafficGenerator::EnsureHttpSessionArrivalRv ()
{
  if (m_httpSessionArrivalRv == nullptr)
    {
      m_httpSessionArrivalRv = CreateObject<ExponentialRandomVariable> ();
    }
  m_httpSessionArrivalRv->SetAttribute (
    "Mean", DoubleValue (1.0 / std::max (1e-6, m_httpSessionArrivalRate)));
}

// 登记一个浏览方的 Poisson 到达 spawn 上下文,并排第一个到达。λ<=0 时不登记(关闭)。
void
SatTrafficGenerator::RegisterHttpSessionArrival (const HttpArrivalContext& ctx)
{
  if (m_httpSessionArrivalRate <= 0.0)
    {
      return;   // 默认关闭:保持纯链式浏览行为
    }
  EnsureHttpSessionArrivalRv ();
  m_httpArrivalActive = true;
  const std::size_t index = m_httpArrivalContexts.size ();
  m_httpArrivalContexts.push_back (ctx);
  HttpScheduleNextSessionArrival (index);
}

// 排下一个 Poisson 会话到达(指数到达间隔),形成 Poisson 过程。
void
SatTrafficGenerator::HttpScheduleNextSessionArrival (std::size_t ctxIndex)
{
  if (!m_httpArrivalActive || ctxIndex >= m_httpArrivalContexts.size ())
    {
      return;
    }
  EnsureHttpSessionArrivalRv ();
  const Time interArrival = Seconds (m_httpSessionArrivalRv->GetValue ());
  Simulator::Schedule (interArrival,
                       &SatTrafficGenerator::HttpOnSessionArrival, this, ctxIndex);
}

// 会话到达事件:在浏览方节点新建一个 ThreeGppHttpClient 会话(取一个页面),
// 复用已运行的 server。短连接式:client 完成一页浏览后自然结束。
void
SatTrafficGenerator::HttpOnSessionArrival (std::size_t ctxIndex)
{
  if (!m_httpArrivalActive || ctxIndex >= m_httpArrivalContexts.size ())
    {
      return;
    }
  const HttpArrivalContext ctx = m_httpArrivalContexts[ctxIndex];

  const Time now = Simulator::Now ();
  if (now >= m_appStopTime)
    {
      return;   // 业务窗口已结束,不再发起新会话
    }
  ++m_httpSessionArrivals;

  ThreeGppHttpClientHelper clientHelper{Address (ctx.serverAddress)};
  clientHelper.SetAttribute ("RemoteServerPort", UintegerValue (ctx.serverPort));
  ApplicationContainer clientApps = clientHelper.Install (ctx.clientNode);
  Ptr<ThreeGppHttpClient> client = clientApps.Get (0)->GetObject<ThreeGppHttpClient> ();
  if (client == nullptr)
    {
      HttpScheduleNextSessionArrival (ctxIndex);
      return;
    }

  // 立即起、跟随业务窗口停;到达驱动的会话由 Poisson 过程节奏控制。
  client->SetStartTime (now + MicroSeconds (1));
  client->SetStopTime (m_appStopTime);
  m_managedApps.push_back (client);

  RegisterTraceConnectWithoutContext (
    client,
    "ConnectionEstablished",
    MakeCallback (&SatTrafficGenerator::OnHttpClientConnectionEstablished, this));
  PointerValue variablesPtr;
  client->GetAttribute ("Variables", variablesPtr);
  ConfigureHttpVariables (variablesPtr.Get<ThreeGppHttpVariables> (),
                          ctx.lightweightProfile, ctx.uploadProfile);

  std::string rxContext = MakeTraceContext (ctx.rxNodeId, ctx.statType);
  RegisterTraceConnect (client, "RxMainObjectPacket", rxContext,
                        MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
  RegisterTraceConnect (client, "RxEmbeddedObjectPacket", rxContext,
                        MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));

  HttpScheduleNextSessionArrival (ctxIndex);   // 继续排下一个到达
}

ApplicationContainer
SatTrafficGenerator::InstallVoipRtp (NodeContainer sourceNodes,
                                     Ipv4Address sinkAddress,
                                     bool isUplink)
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);

  m_modelType = TRAFFIC_VOIP_RTP;
  MarkTrafficModelInstalled (TRAFFIC_VOIP_RTP);
  PruneInactiveFlows ();

  Ptr<Node> sinkNode = FindNodeByAddress (sinkAddress);
  if (sinkNode == nullptr)
    {
      NS_LOG_ERROR ("Unable to map VoIP sink address " << sinkAddress
                    << " to a node. Call InstallSink() after IP assignment first.");
      return ApplicationContainer ();
    }

  auto installVoipLeg =
    [&] (Ptr<Node> node,
         Ipv4Address peerAddress,
         const std::shared_ptr<VoipSessionState>& session,
         bool isLegA) -> ApplicationContainer
    {
      ApplicationContainer legApps;
      Ptr<Socket> udpSocket = Socket::CreateSocket (node, UdpSocketFactory::GetTypeId ());
      udpSocket->SetAllowBroadcast (false);
      udpSocket->Connect (InetSocketAddress (peerAddress, VOIP_PORT));

      Ptr<SatFlowDriverApp> app = CreateObject<SatFlowDriverApp> ();
      node->AddApplication (app);
      app->SetStartTime (m_appStartTime);
      app->SetStopTime (m_appStopTime);
      m_driverApps.push_back (app);

      // 任务1:帧大小/间隔/PT/采样率全部取自可配 codec 参数(默认 AMR 12.2k @ 8 kHz)。
      const uint32_t activePayload = m_voipActivePayloadBytes;
      const uint32_t silencePayload = m_voipSilencePayloadBytes;
      const Time activeInterval = MilliSeconds (m_voipFrameMs);
      const Time silenceInterval = MilliSeconds (m_voipSilenceFrameMs);
      const uint8_t payloadType = m_voipPayloadType;
      // RTP 时间戳步进(经过的采样数),active / silence 各自按真实帧间隔换算。
      const uint32_t activeTsStep = VoipRtpTimestampStep (m_voipFrameMs, m_voipSampleRateHz);
      const uint32_t silenceTsStep = VoipRtpTimestampStep (m_voipSilenceFrameMs, m_voipSampleRateHz);

      auto flow = std::make_shared<FlowState> ();
      flow->socket = udpSocket;
      flow->type = TRAFFIC_VOIP_RTP;
      flow->nodeId = node->GetId ();
      flow->packetSize = activePayload;
      flow->baseInterval = activeInterval;
      flow->voiceVadEnabled = true;
      flow->voiceActive = true;
      flow->isVoipLegA = isLegA;
      flow->activePacketSize = activePayload;
      flow->silencePacketSize = silencePayload;
      flow->silenceInterval = silenceInterval;
      flow->voipSession = session;
      flow->stateChangeTime = session ? session->stateChangeTime : Time (0);
      flow->active = false;
      // Issue 4: SSRC 从生成器内全局计数器分配,与 nodeId 解耦,避免大 nodeId
      // 位移溢出;不同 leg 自然拿到不同值,符合 RFC 3550 的不冲突原则。
      const uint32_t ssrc = ++m_nextSsrc;

      std::weak_ptr<FlowState> weak = flow;
      flow->driver = [this, weak, ssrc, payloadType, activeTsStep, silenceTsStep] () {
        auto self = weak.lock ();
        if (!self || !self->active)
          {
            return;
          }

        const Time now = Simulator::Now ();
        if (self->voiceVadEnabled && self->voipSession)//VAD (静音检测) 状态机
          {
            // 任务4:单时钟幂等推进。两条 leg 共享会话状态,但只在 lastAdvancedTime<now
            // 时由 AdvanceVoipSession 真正推进一次;后到的 leg 读到一致快照,
            // 不再各自写共享状态,消除切换瞬间多发/漏发一帧的竞态。
            AdvanceVoipSession (*self->voipSession, now);

            self->voiceActive =//结算当前这条流是否活跃:非互相静音且当前说话人正好是自己这个方向
              !self->voipSession->inMutualSilence &&
              (self->isVoipLegA == self->voipSession->legATalking);
            self->stateChangeTime = self->voipSession->stateChangeTime;//发包时据此决定帧大小
          }

        const uint32_t payloadBytes = self->voiceActive ? self->activePacketSize : self->silencePacketSize;
        const Time pacingInterval = self->voiceActive ? self->baseInterval : self->silenceInterval;
        const uint32_t tsStep = self->voiceActive ? activeTsStep : silenceTsStep;
        const bool startOfTalkSpurt = self->voiceActive && !self->lastVoiceActive;

        Ptr<Packet> pkt = Create<Packet> (payloadBytes);
        SatRtpHeader rtp;
        rtp.SetPayloadType (payloadType);
        rtp.SetSequenceNumber (self->rtpSeq);
        rtp.SetTimestamp (self->rtpTimestamp);
        rtp.SetSsrc (ssrc);
        rtp.SetMarker (startOfTalkSpurt);
        pkt->AddHeader (rtp);

        int sent = self->socket->Send (pkt);
        if (sent > 0)
          {
            // wire = L4 payload(含 12 B RTP) + UDP(8) + IPv4(20);useful = 仅 codec payload
            const uint32_t wireBytes =
              static_cast<uint32_t> (sent) + kIpv4HeaderBytes + kUdpHeaderBytes;
            AccountTx (*self, wireBytes, payloadBytes);
          }
        else
          {
            m_totalStats.sendBlockedCount++;
            m_statsByType[TRAFFIC_VOIP_RTP].sendBlockedCount++;
          }

        self->rtpSeq++;
        // 任务1:时间戳按本帧真实经过采样数推进(active 20 ms@8kHz=160,silence 160 ms@8kHz=1280),
        // 不再恒 +160,与 PT/采样率自洽。
        self->rtpTimestamp += tsStep;
        self->lastVoiceActive = self->voiceActive;

        if (!self->active)
          {
            return;
          }
        // VoIP 是 conversational 业务,严格按 RTP/codec 节拍发包,不受闭环速率
        // 控制影响;否则 RTP 时间戳步进与实际发送间隔不一致,接收侧 jitter buffer
        // 与采样率会失准。
        const Time delta = self->stateChangeTime - now;
        Time nextWake = pacingInterval;
        if (delta.IsStrictlyPositive ())
          {
            nextWake = std::min (nextWake, delta);
          }
        if (!nextWake.IsStrictlyPositive ())
          {
            nextWake = MilliSeconds (1);   // 状态机零步推进保护
          }
        Simulator::Schedule (nextWake, self->driver);
      };

      m_flows.push_back (flow);
      app->SetStartCallback ([weak] () {
        if (auto self = weak.lock ())
          {
            self->started = true;
            self->active = true;
            self->driver ();
          }
      });
      app->SetStopCallback ([this, weak] () {
        if (auto self = weak.lock ())
          {
            self->active = false;
          }
        PruneInactiveFlows ();
      });
      legApps.Add (app);
      return legApps;
    };

  ApplicationContainer apps;
  apps.Add (InstallArpSeedPingsOnce (sourceNodes, sinkAddress));
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);
      Ipv4Address sourceAddress = GetPrimaryIpv4Address (node);
      if (sourceAddress == Ipv4Address::GetAny ())
        {
          NS_LOG_WARN ("Skipping VoIP session install on node " << node->GetId ()
                       << " because it has no IPv4 address.");
          continue;
        }

      auto session = std::make_shared<VoipSessionState> ();
      session->legATalking = true;
      session->inMutualSilence = false;
      // 任务2:标准语音 ON/OFF 状态机改用指数分布(ExponentialRandomVariable)。
      // talk spurt 与 silence/turn gap 各取一条指数分布,均值见上面常量。
      session->talkSpurtRv = CreateObject<ExponentialRandomVariable> ();
      session->talkSpurtRv->SetAttribute ("Mean", DoubleValue (kVoipTalkSpurtMeanSeconds));
      session->turnGapRv = CreateObject<ExponentialRandomVariable> ();
      session->turnGapRv->SetAttribute ("Mean", DoubleValue (kVoipTurnGapMeanSeconds));
      session->stateChangeTime = Time (0);

      apps.Add (installVoipLeg (node, sinkAddress, session, true));
      if (node != sinkNode)
        {
          apps.Add (InstallArpSeedPingsOnce (NodeContainer (sinkNode), sourceAddress));
          apps.Add (installVoipLeg (sinkNode, sourceAddress, session, false));
        }
    }

  return apps;
}

ApplicationContainer
SatTrafficGenerator::InstallLegacyConsumerData (NodeContainer sourceNodes,
                                                Ipv4Address sinkAddress,
                                                bool isUplink)
{
  return InstallHttp (sourceNodes, sinkAddress, isUplink);
}

InstalledTerminalTraffic
SatTrafficGenerator::InstallProfileTraffic (NodeContainer sourceNodes,
                                            Ptr<Node> terminalNode,
                                            Ipv4Address terminalAddress,
                                            bool isUplink)
{
  InstalledTerminalTraffic installed;

  Ptr<SatTerminalProfile> profile = terminalNode->GetObject<SatTerminalProfile> ();
  NS_ABORT_MSG_IF (profile == nullptr, "UE node is missing SatTerminalProfile");

  installed.isConsumerPhone = profile->GetTerminalType () == UT_CONSUMER;

  if (profile->HasVoiceService ())
    {
      InstallVoipRtp (sourceNodes, terminalAddress, isUplink);
      installed.hasVoip = true;
    }

  switch (profile->GetDataServiceType ())
    {
    case SAT_DATA_HTTP:
      // Issue 2: 之前这里写死 false,导致 isUplink 在 HTTP 业务里被吞掉。
      InstallHttpInternal (sourceNodes,
                           terminalAddress,
                           isUplink,
                           installed.isConsumerPhone);
      installed.hasHttp = true;
      break;
    case SAT_DATA_FTP:
      InstallFtp (sourceNodes, terminalAddress, isUplink);
      installed.hasFtp = true;
      break;
    default:
      NS_ABORT_MSG ("Unknown terminal data service type");
    }

  return installed;
}

ApplicationContainer
SatTrafficGenerator::GenerateVoiceTraffic (NodeContainer sourceNodes, Ipv4Address sinkAddress)
{
  return InstallVoipRtp (sourceNodes, sinkAddress, false);
}

ApplicationContainer
SatTrafficGenerator::GenerateConsumerDataTraffic (NodeContainer sourceNodes,
                                                  Ipv4Address sinkAddress,
                                                  bool isUplink)
{
  return InstallHttp (sourceNodes, sinkAddress, isUplink);
}

ApplicationContainer
SatTrafficGenerator::GeneratePortableDataTraffic (NodeContainer sourceNodes,
                                                  Ipv4Address sinkAddress,
                                                  bool isUplink)
{
  return InstallFtp (sourceNodes, sinkAddress, isUplink);
}

void
SatTrafficGenerator::AttachUserToBeam (uint16_t beamId,
                                       Ptr<Node> userNode,
                                       Ipv4Address destAddress,
                                       TrafficModelType type,
                                       bool isUplink)
{
  NS_LOG_FUNCTION (this << beamId << static_cast<int> (type));

  NodeContainer nodeCont (userNode);
  switch (type)
    {
    case TRAFFIC_FULL_BUFFER:
      InstallFullBuffer (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_FTP:
      InstallFtp (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_HTTP:
      // Issue 3: 之前固定走 InstallHttpDownload,使得 AttachUserToBeam 无法装上行 HTTP。
      InstallHttp (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_VOIP_RTP:
      InstallVoipRtp (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_LEGACY_DATA:
      InstallLegacyConsumerData (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_LEGACY_VOICE:
      GenerateVoiceTraffic (nodeCont, destAddress);
      break;
    default:
      NS_LOG_WARN ("Unknown Traffic Type: " << static_cast<int> (type));
      break;
    }
}

ApplicationContainer
SatTrafficGenerator::InstallSink (NodeContainer nodes)
{
  NS_LOG_FUNCTION (this);

  RegisterNodeAddresses (nodes);

  ApplicationContainer apps;
  const Time sinkStopTime = m_appStopTime + Seconds (1.0);

  for (uint32_t i = 0; i < nodes.GetN (); ++i)
    {
      Ptr<Node> node = nodes.Get (i);
      const uint32_t nodeId = node->GetId ();

      PacketSinkHelper fullBufferSink ("ns3::UdpSocketFactory",
                                       InetSocketAddress (Ipv4Address::GetAny (), LEGACY_PORT));
      ApplicationContainer fullBufferApps = fullBufferSink.Install (node);
      fullBufferApps.Start (Seconds (0.0));
      fullBufferApps.Stop (sinkStopTime);
      Ptr<PacketSink> fullBuffer = DynamicCast<PacketSink> (fullBufferApps.Get (0));
      if (fullBuffer)
        {
          RegisterTraceConnect (
            fullBuffer,
            "Rx",
            MakeTraceContext (nodeId, TRAFFIC_FULL_BUFFER),
            MakeCallback (&SatTrafficGenerator::NotifyPacketRx, this));
        }
      apps.Add (fullBufferApps);

      PacketSinkHelper ftpSink ("ns3::TcpSocketFactory",
                                InetSocketAddress (Ipv4Address::GetAny (), FTP_PORT));
      ApplicationContainer ftpApps = ftpSink.Install (node);
      ftpApps.Start (Seconds (0.0));
      ftpApps.Stop (sinkStopTime);
      Ptr<PacketSink> ftp = DynamicCast<PacketSink> (ftpApps.Get (0));
      if (ftp)
        {
          RegisterTraceConnect (
            ftp,
            "Rx",
            MakeTraceContext (nodeId, TRAFFIC_FTP),
            MakeCallback (&SatTrafficGenerator::NotifyPacketRx, this));
        }
      apps.Add (ftpApps);

      PacketSinkHelper voipSink ("ns3::UdpSocketFactory",
                                 InetSocketAddress (Ipv4Address::GetAny (), VOIP_PORT));
      ApplicationContainer voipApps = voipSink.Install (node);
      voipApps.Start (Seconds (0.0));
      voipApps.Stop (sinkStopTime);
      Ptr<PacketSink> voip = DynamicCast<PacketSink> (voipApps.Get (0));
      if (voip)
        {
          RegisterTraceConnect (
            voip,
            "Rx",
            MakeTraceContext (nodeId, TRAFFIC_VOIP_RTP),
            MakeCallback (&SatTrafficGenerator::NotifyPacketRx, this));
        }
      apps.Add (voipApps);
    }

  return apps;
}

ApplicationContainer
SatTrafficGenerator::InstallArpSeedPingsOnce (NodeContainer sourceNodes, Ipv4Address sinkAddress)
{
  ApplicationContainer apps;
  const int64_t startKey = m_appStartTime.GetNanoSeconds ();
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> sourceNode = sourceNodes.Get (i);
      if (sourceNode == nullptr)
        {
          continue;
        }

      const auto key = std::make_tuple (sourceNode->GetId (), sinkAddress.Get (), startKey);
      if (!m_arpSeedRegistry.insert (key).second)
        {
          continue;
        }
      apps.Add (InstallArpSeedPings (NodeContainer (sourceNode), sinkAddress, m_appStartTime));
    }
  return apps;
}

uint64_t
SatTrafficGenerator::GetNodeReceivedBytes (uint32_t nodeId, TrafficModelType type) const
{
  auto byType = m_rxBytesByType.find (type);
  if (byType == m_rxBytesByType.end ())
    {
      return 0;
    }

  auto byNode = byType->second.find (nodeId);
  if (byNode == byType->second.end ())
    {
      return 0;
    }

  return byNode->second;
}

TrafficModelStats
SatTrafficGenerator::GetStats (TrafficModelType type) const
{
  auto it = m_statsByType.find (type);
  if (it == m_statsByType.end ())
    {
      return TrafficModelStats {};
    }
  return it->second;
}

Ptr<Node>
SatTrafficGenerator::FindNodeByAddress (Ipv4Address address) const
{
  auto it = m_nodesByAddress.find (address);
  return it == m_nodesByAddress.end () ? nullptr : it->second;
}

Ipv4Address
SatTrafficGenerator::GetPrimaryIpv4Address (Ptr<Node> node) const
{
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  if (!ipv4)
    {
      return Ipv4Address::GetAny ();
    }

  for (uint32_t i = 0; i < ipv4->GetNInterfaces (); ++i)
    {
      for (uint32_t j = 0; j < ipv4->GetNAddresses (i); ++j)
        {
          Ipv4Address address = ipv4->GetAddress (i, j).GetLocal ();
          if (address != Ipv4Address::GetLoopback () && address != Ipv4Address::GetAny ())
            {
              return address;
            }
        }
    }

  return Ipv4Address::GetAny ();
}

void
SatTrafficGenerator::RegisterNodeAddresses (NodeContainer nodes)
{
  for (uint32_t i = 0; i < nodes.GetN (); ++i)
    {
      RegisterNodeAddresses (nodes.Get (i));
    }
}

void
SatTrafficGenerator::RegisterNodeAddresses (Ptr<Node> node)
{
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
  if (!ipv4)
    {
      return;
    }

  for (uint32_t i = 0; i < ipv4->GetNInterfaces (); ++i)
    {
      for (uint32_t j = 0; j < ipv4->GetNAddresses (i); ++j)
        {
          Ipv4Address address = ipv4->GetAddress (i, j).GetLocal ();
          if (address != Ipv4Address::GetLoopback () && address != Ipv4Address::GetAny ())
            {
              m_nodesByAddress[address] = node;
            }
        }
    }
}

Ptr<ThreeGppHttpServer>
// 确保某节点上存在一个 HTTP server(已存在则复用,否则新建并配置画像/窗口/统计回调)。
SatTrafficGenerator::EnsureHttpServer (Ptr<Node> node,            // 要承载 server 的节点
                                       Ipv4Address localAddress,  // server 绑定的本地地址
                                       bool lightweightProfile,   // 画像档位:轻量 vs 宽带
                                       bool uploadProfile)        // 是否上传档(对象大小不同)
{
  auto existing = m_httpServersByNodeId.find (node->GetId ());        // 先查该节点是否已建过 HTTP server
  if (existing != m_httpServersByNodeId.end ())                       // 已存在则复用,不重复创建
    {
      auto profileIt = m_httpServerProfilesByNodeId.find (node->GetId ());   // 查已建 server 的画像档位
      if (profileIt != m_httpServerProfilesByNodeId.end () &&                // 若请求的档位与现有不一致
          (profileIt->second.lightweightProfile != lightweightProfile ||
           profileIt->second.uploadProfile != uploadProfile))
        {
          NS_LOG_WARN ("Reusing existing HTTP server on node " << node->GetId ()   // 仅告警,沿用原档位不重写
                       << " with original profile instead of rewriting active variables.");
        }
      return existing->second;        // 返回已有 server 指针
    }

  ThreeGppHttpServerHelper serverHelper{Address (localAddress)};          // 新建 HTTP server helper,绑定本地地址
  serverHelper.SetAttribute ("LocalPort", UintegerValue (HTTP_PORT));     // server 监听端口设为 80
  ApplicationContainer serverApps = serverHelper.Install (node);          // 在节点上安装 server 应用

  const Time serverStart =                                                   // server 比 client 早 100ms 启动
      (m_appStartTime > MilliSeconds (100)) ? (m_appStartTime - MilliSeconds (100))   // 提前 100ms,确保监听 socket 先就绪
                                            : Seconds (0.0);                            // 业务太早则从 0 时刻起
  serverApps.Start (serverStart);     // 设置 server 启动时间
  serverApps.Stop (m_appStopTime);    // 设置 server 停止时间

  Ptr<ThreeGppHttpServer> server = serverApps.Get (0)->GetObject<ThreeGppHttpServer> ();   // 取出 server 对象指针
  if (!server)                        // 创建失败
    {
      NS_LOG_ERROR ("Failed to create HTTP server on node " << node->GetId ());   // 报错
      return nullptr;                 // 返回空
    }

  PointerValue variablesPtr;                                // 接收 server 内部随机变量对象指针
  server->GetAttribute ("Variables", variablesPtr);         // 取出 server 的 ThreeGppHttpVariables
  ConfigureHttpVariables (variablesPtr.Get<ThreeGppHttpVariables> (),   // 按画像配置 server 侧对象大小(决定真实页面大小)
                          lightweightProfile,
                          uploadProfile);
  RegisterTraceConnectWithoutContext (                       // 挂 server “连接建立”回调:给 accept 出的子 socket 配窗口
    server,
    "ConnectionEstablished",
    MakeCallback (&SatTrafficGenerator::OnHttpServerConnectionEstablished, this));

 
  const uint32_t window = CalculateTcpWindowBytes ();              // 计算 GEO 链路 BDP×3 的 TCP 窗口
  ScheduleManagedTcpSocketRegistration (server, window, window);  // 延迟取 listen socket 并配大窗口、纳入受管
  m_managedApps.push_back (server);   // Issue 6: ResetStats 时需要一并停掉

  m_httpServersByNodeId[node->GetId ()] = server;                                       // 记录该节点的 server,供下次复用
  m_httpServerProfilesByNodeId[node->GetId ()] = {lightweightProfile, uploadProfile};   // 记录该 server 的画像档位
  return server;     // 返回新建的 server
}

// 给 TCP socket 替换为指定的拥塞控制算法
void
SatTrafficGenerator::ApplyTcpCongestionType (Ptr<Socket> socket) const   // 目标 socket
{
  Ptr<TcpSocketBase> tcpSocket = DynamicCast<TcpSocketBase> (socket);   // 转成 TCP socket 基类
  if (tcpSocket == nullptr)        // 不是 TCP socket(如 UDP)则跳过
    {
      return;
    }

  ObjectFactory congestionFactory;                                       // 用工厂按 TypeId 创建算法对象
  congestionFactory.SetTypeId (m_tcpCongestionTypeId);                   // 设为配置的拥塞算法类型
  Ptr<TcpCongestionOps> algo = congestionFactory.Create<TcpCongestionOps> ();   // 实例化算法
  if (algo != nullptr)
    {
      tcpSocket->SetCongestionControlAlgorithm (algo);                   // 装到 socket 上
    }
}

// 一站式配置 TCP socket:换拥塞算法 + 设窗口并纳入受管列表。
void
SatTrafficGenerator::ConfigureTcpSocket (Ptr<Socket> socket,             // 目标 socket
                                         uint32_t preferredSndBufBytes,  // 期望发送缓冲(0=用默认)
                                         uint32_t preferredRcvBufBytes)  // 期望接收缓冲(0=用默认)
{
  ApplyTcpCongestionType (socket);                                          // 第一步:换拥塞算法
  RegisterManagedTcpSocket (socket, preferredSndBufBytes, preferredRcvBufBytes);   // 第二步:配窗口+纳管
}

// 【HTTP/FTP 共用】把 socket 纳入受管列表,并按 BDP×3 配置收发窗口(GEO 高时延关键)。
void
SatTrafficGenerator::RegisterManagedTcpSocket (Ptr<Socket> socket,             // 目标 socket
                                               uint32_t preferredSndBufBytes,  // 期望发送缓冲
                                               uint32_t preferredRcvBufBytes)  // 期望接收缓冲
{
  if (socket == nullptr)        // 空指针保护
    {
      return;
    }

  // std::map 自带 O(log N) 去重,重复注册同一个 socket 时直接返回。
  if (m_managedTcpSockets.find (socket) != m_managedTcpSockets.end ())   // 已纳管则不重复处理
    {
      return;
    }

  UintegerValue sndBufValue;                              // 读取 socket 当前发送缓冲值(兜底用)
  UintegerValue rcvBufValue;                              // 读取 socket 当前接收缓冲值(兜底用)
  socket->GetAttribute ("SndBufSize", sndBufValue);
  socket->GetAttribute ("RcvBufSize", rcvBufValue);

  ManagedTcpSocketState state;                                                          // 记录该 socket 的基准窗口
  state.baseSndBufBytes = preferredSndBufBytes > 0 ? preferredSndBufBytes : sndBufValue.Get ();   // 优先用期望值,否则用当前值
  state.baseRcvBufBytes = preferredRcvBufBytes > 0 ? preferredRcvBufBytes : rcvBufValue.Get ();
  m_managedTcpSockets.emplace (socket, state);                                          // 存入受管表,供闭环统一缩放

  // 只为新加入的 socket 应用当前 multiplier;不要遍历全部 socket。全局重设由
  // AdjustRateForClosedLoop 在 multiplier 变化时统一触发。
  const double multiplier = m_currentRateMultiplier > 0.0 ? m_currentRateMultiplier : 1.0;   // 当前闭环倍率
  const uint32_t sndBuf =
    std::max (16384u, static_cast<uint32_t> (std::llround (state.baseSndBufBytes * multiplier)));   // 基准×倍率,下限16KB
  const uint32_t rcvBuf =
    std::max (16384u, static_cast<uint32_t> (std::llround (state.baseRcvBufBytes * multiplier)));
  socket->SetAttribute ("SndBufSize", UintegerValue (sndBuf));   // 写回发送窗口
  socket->SetAttribute ("RcvBufSize", UintegerValue (rcvBuf));   // 写回接收窗口
}

// 【HTTP/FTP 共用】延迟+重试地从应用拿到 TCP socket 并配置它(socket 在 StartApplication 后才创建)。
void
SatTrafficGenerator::ScheduleManagedTcpSocketRegistration (Ptr<Application> app,             // BulkSend / HTTP client / HTTP server
                                                           uint32_t preferredSndBufBytes,    // 期望发送缓冲
                                                           uint32_t preferredRcvBufBytes,    // 期望接收缓冲
                                                           uint32_t retriesRemaining)        // 剩余重试次数
{
  if (app == nullptr)        // 空应用保护
    {
      return;
    }

  // 第一次注册要落在 Application::StartApplication 之后才能从 BulkSend/HTTP
  // helper 那里拿到 socket。计算"距业务开始的剩余时间 + 1us",保证首次尝试
  // 不会过早;若已过业务开始,则用 10ms 短间隔继续重试。
  Time registrationDelay;                              // 本次尝试的延迟时间
  if (Simulator::Now () < m_appStartTime)              // 业务还没开始
    {
      registrationDelay = (m_appStartTime - Simulator::Now ()) + MicroSeconds (1);   // 排到业务开始后 1us
    }
  else                                                 // 业务已开始
    {
      registrationDelay = MilliSeconds (10);           // 10ms 后再试
    }
  Simulator::Schedule (                                // 安排一个延迟事件去取 socket
    registrationDelay,
    [this, app, preferredSndBufBytes, preferredRcvBufBytes, retriesRemaining] () mutable {
      Ptr<Socket> socket;                                                              // 尝试取出的 socket
      if (Ptr<BulkSendApplication> bulkApp = DynamicCast<BulkSendApplication> (app))   // 若是 FTP 的 BulkSend
        {
          socket = bulkApp->GetSocket ();
        }
      else if (Ptr<ThreeGppHttpClient> httpClient = DynamicCast<ThreeGppHttpClient> (app))   // 若是 HTTP client
        {
          socket = httpClient->GetSocket ();
        }
      else if (Ptr<ThreeGppHttpServer> httpServer = DynamicCast<ThreeGppHttpServer> (app))   // 若是 HTTP server(listen socket)
        {
          socket = httpServer->GetSocket ();
        }

      if (socket != nullptr)        // 取到了 socket
        {
          ConfigureTcpSocket (socket, preferredSndBufBytes, preferredRcvBufBytes);   // 配窗口+拥塞算法+纳管,完成
          return;
        }

      if (retriesRemaining == 0)        // 重试次数用尽仍没取到
        {
          return;                       // 放弃
        }

      ScheduleManagedTcpSocketRegistration (app,                              // 否则再排一次,次数减一
                                            preferredSndBufBytes,
                                            preferredRcvBufBytes,
                                            retriesRemaining - 1);
    });
}

// 【HTTP/FTP 共用】在节点 IPv4 层挂 SendOutgoing trace,按端口过滤统计某业务的发包量。
void
SatTrafficGenerator::RegisterIpv4TcpTxTrace (Ptr<Node> node,            // 要监听的节点
                                             TrafficModelType type,     // 该业务类型(HTTP/FTP)
                                             uint16_t trackedPort,      // 跟踪的端口(HTTP=80, FTP=20)
                                             bool matchSourcePort)      // true=按源端口匹配,false=按目的端口
{
  if (node == nullptr)        // 空节点保护
    {
      return;
    }

  const auto key = std::make_tuple (node->GetId (), type, trackedPort, matchSourcePort);   // 注册去重键
  if (m_ipv4TcpTxTraceRegistry.find (key) != m_ipv4TcpTxTraceRegistry.end ())               // 已注册则跳过
    {
      return;
    }

  Ptr<Ipv4L3Protocol> ipv4 = node->GetObject<Ipv4L3Protocol> ();   // 取该节点的 IPv4 协议栈
  if (ipv4 == nullptr)        // 没有 IPv4 则跳过
    {
      return;
    }

  const std::string traceContext =                                                          // 拼接 "nodeId:type:port:matchSrc" 上下文
    std::to_string (node->GetId ()) + ":" + std::to_string (static_cast<int> (type)) + ":" +
    std::to_string (trackedPort) + ":" + (matchSourcePort ? "1" : "0");
  RegisterTraceConnect (                          // 连接 IPv4 SendOutgoing,每个出站包都会回调 NotifyIpv4TcpTx
    ipv4,
    "SendOutgoing",
    traceContext,
    MakeCallback (&SatTrafficGenerator::NotifyIpv4TcpTx, this));
  m_ipv4TcpTxTraceRegistry.insert (key);          // 记录已注册,防重复
}

// 【HTTP/FTP 共用】IPv4 出站包回调:过滤出目标业务的 TCP 包,按 wire/useful 双口径计入发送统计。
void
SatTrafficGenerator::NotifyIpv4TcpTx (std::string context,        // 注册时的上下文字符串
                                      const Ipv4Header& header,   // IPv4 头
                                      Ptr<const Packet> packet,   // 出站包(不含 IP 头)
                                      uint32_t interface)         // 出口网卡索引(未用)
{
  uint32_t nodeId = 0;                              // 解析出的节点 id
  TrafficModelType type = TRAFFIC_FULL_BUFFER;      // 解析出的业务类型
  uint16_t trackedPort = 0;                         // 解析出的跟踪端口
  bool matchSourcePort = false;                     // 解析出的匹配方式
  if (!ParseIpv4TcpTraceContext (context, nodeId, type, trackedPort, matchSourcePort))   // 解析上下文,失败则丢弃
    {
      return;
    }

  if (packet == nullptr || header.GetProtocol () != TcpL4Protocol::PROT_NUMBER)   // 只统计 TCP 包
    {
      return;
    }

  TcpHeader tcpHeader;                             
  Ptr<Packet> packetCopy = packet->Copy ();         
  if (packetCopy->PeekHeader (tcpHeader) == 0)      
    {
      return;
    }

  const uint16_t observedPort =                                                       // 取出待比较的端口
    matchSourcePort ? tcpHeader.GetSourcePort () : tcpHeader.GetDestinationPort ();    // 按注册时的匹配方式
  if (observedPort != trackedPort)        // 端口不匹配 → 不是本业务的包,丢弃
    {
      return;
    }

  const uint32_t tcpHeaderBytes = tcpHeader.GetSerializedSize ();                      // TCP 头长度
  const uint32_t usefulBytes =                                                         // 有效载荷 = 包大小 - TCP 头
    packet->GetSize () > tcpHeaderBytes ? (packet->GetSize () - tcpHeaderBytes) : 0u;  // ACK 等纯头包 useful=0
  const uint32_t bytesSentOnWire = packet->GetSize () + header.GetSerializedSize ();   // 上线字节 = 包 + IP 头
  AccountTx (type, nodeId, bytesSentOnWire, usefulBytes);   // 计入发送统计(双口径)
  (void) interface;                                         // 显式消除未使用参数告警
}

void
SatTrafficGenerator::ApplyManagedTcpSocketRateControl ()
{
  const double multiplier = m_currentRateMultiplier > 0.0 ? m_currentRateMultiplier : 1.0;
  for (auto& [socket, state] : m_managedTcpSockets)
    {
      if (socket == nullptr)
        {
          continue;
        }

      const uint32_t sndBuf =
        std::max (16384u, static_cast<uint32_t> (std::llround (state.baseSndBufBytes * multiplier)));
      const uint32_t rcvBuf =
        std::max (16384u, static_cast<uint32_t> (std::llround (state.baseRcvBufBytes * multiplier)));
      socket->SetAttribute ("SndBufSize", UintegerValue (sndBuf));
      socket->SetAttribute ("RcvBufSize", UintegerValue (rcvBuf));
    }
}

// HTTP client 连接建立(三次握手成功)回调:此刻 socket 才存在,立即配 GEO 大窗口。
void
SatTrafficGenerator::OnHttpClientConnectionEstablished (Ptr<const ThreeGppHttpClient> httpClient)   // 触发回调的 client
{
  if (httpClient == nullptr)   // 防御性判空
    {
      return;
    }


  const uint32_t window = CalculateTcpWindowBytes ();                 // 计算 BDP×3 窗口
  ConfigureTcpSocket (httpClient->GetSocket (), window, window);      // 给 client socket 配收发窗口 + 换拥塞算法 + 纳管
}

// HTTP server 接受新连接回调:对每个 accept 出的子 socket 配 GEO 大窗口并纳入闭环。
void
SatTrafficGenerator::OnHttpServerConnectionEstablished (Ptr<const ThreeGppHttpServer> httpServer,   // 触发回调的 server
                                                        Ptr<Socket> socket)                         // 新接受的子 socket
{

  const uint32_t window = CalculateTcpWindowBytes ();   // 计算 BDP×3 窗口
  ConfigureTcpSocket (socket, window, window);          // 给子 socket 配窗口 + 拥塞算法 + 纳管(响应闭环)
  (void) httpServer;                                    // server 参数本函数未用,显式消除未使用告警
}

void
// 按终端画像把对象大小/数量/思考时间等参数写进 ThreeGppHttpVariables(决定 HTTP 流量形态)。
SatTrafficGenerator::ConfigureHttpVariables (Ptr<ThreeGppHttpVariables> variables,   // 待配置的随机变量对象
                                             bool lightweightProfile,                // true=消费级手机轻量档
                                             bool uploadProfile) const               // true=上传方向(请求体更大)
{
  if (!variables)        // 空指针保护(client/server 未暴露 Variables 时)
    {
      return;
    }

  if (uploadProfile)     // ===== 上传方向画像 =====
    {
      if (lightweightProfile)     // --- 上传 · 轻量手机档 ---
        {
          // Consumer phones can only sustain lightweight form/text/image uploads.
          variables->SetRequestSize (512);                      // 请求包(此处是上传请求)固定 512B
          variables->SetMainObjectSizeMean (12288);             // 上传主体均值 12KB
          variables->SetMainObjectSizeStdDev (6144);            // 上传主体标准差 6KB(对数正态)
          variables->SetEmbeddedObjectSizeMean (2048);          // 内嵌对象均值 2KB
          variables->SetEmbeddedObjectSizeStdDev (6144);        // 内嵌对象标准差 6KB
          // 内嵌对象数:截断泊松均值 2(很少),mean 较小走高效泊松。
          ApplyEmbeddedObjectCountDistribution (variables, 2.0, 8);
          variables->SetReadingTimeMean (Seconds (7));          // 阅读思考时间均值 7s(指数分布)
          variables->SetParsingTimeMean (Seconds (0.15));       // 解析时间均值 0.15s(指数分布)
          return;
        }

      // Portable terminals can upload richer page payloads or batched content.
      variables->SetRequestSize (512);                      // 请求固定 512B
      variables->SetMainObjectSizeMean (49152);             // 上传主体均值 48KB(比手机大)
      variables->SetMainObjectSizeStdDev (24576);           // 标准差 24KB
      variables->SetEmbeddedObjectSizeMean (12288);         // 内嵌对象均值 12KB
      variables->SetEmbeddedObjectSizeStdDev (32768);       // 内嵌对象标准差 32KB
      // 内嵌对象数:截断泊松均值 6。
      ApplyEmbeddedObjectCountDistribution (variables, 6.0, 24);
      variables->SetReadingTimeMean (Seconds (10));         // 阅读时间均值 10s
      variables->SetParsingTimeMean (Seconds (0.2));        // 解析时间均值 0.2s
      return;
    }

  if (lightweightProfile)     // ===== 下行 · 轻量手机档(边打电话边轻浏览) =====
    {
      // Consumer phones only browse lightweight pages while voice is active.
      variables->SetMainObjectSizeMean (16384);             // 主对象(正文)均值 16KB
      variables->SetMainObjectSizeStdDev (8192);            // 标准差 8KB
      variables->SetEmbeddedObjectSizeMean (4096);          // 内嵌对象均值 4KB
      variables->SetEmbeddedObjectSizeStdDev (12288);       // 内嵌对象标准差 12KB
      // 内嵌对象数:截断泊松均值 4(图片较少)。
      ApplyEmbeddedObjectCountDistribution (variables, 4.0, 16);
      variables->SetReadingTimeMean (Seconds (5));          // 阅读时间均值 5s
      variables->SetParsingTimeMean (Seconds (0.1));        // 解析时间均值 0.1s
      return;
    }

  // ===== 下行 · 宽带 portable 档(突发性更强的宽带浏览) =====
  // Portable terminals use a burstier broadband browsing profile.
  variables->SetMainObjectSizeMean (65536);             // 主对象均值 64KB(页面更大)
  variables->SetMainObjectSizeStdDev (32768);           // 标准差 32KB
  variables->SetEmbeddedObjectSizeMean (16384);         // 内嵌对象均值 16KB
  variables->SetEmbeddedObjectSizeStdDev (49152);       // 内嵌对象标准差 48KB
  // 内嵌对象数:截断泊松均值 10(图片多);=阈值,仍走高效泊松。>10 会自动退回 Pareto。
  ApplyEmbeddedObjectCountDistribution (variables, 10.0, 64);
  variables->SetReadingTimeMean (Seconds (8));          // 阅读时间均值 8s
  variables->SetParsingTimeMean (Seconds (0.2));        // 解析时间均值 0.2s
}

void
SatTrafficGenerator::NotifyPacketTx (std::string context, Ptr<const Packet> packet)
{
  if (!packet)
    {
      return;
    }

  uint32_t nodeId = 0;
  TrafficModelType type = TRAFFIC_FULL_BUFFER;
  ParseTraceContext (context, nodeId, type);

  AccountTx (type, nodeId, packet->GetSize (), packet->GetSize ());

  if (!m_txTraceCallback.IsNull ())
    {
      Address empty;
      m_txTraceCallback (packet, empty);
    }
  (void) nodeId;
  (void) type;
}

// 【HTTP/FTP/VoIP 共用】接收清算:把收到的有效载荷累加进各级接收统计(HTTP 直接累加,无需剥头)。
void
SatTrafficGenerator::RecordReceivedPacket (uint32_t nodeId,        // 接收节点 id
                                           TrafficModelType type,  // 业务类型
                                           Ptr<const Packet> packet)   // 收到的包
{
  if (!packet)        // 空包保护
    {
      return;
    }

  // 对 VoIP 业务剥掉 12 B RTP 头,使收发双方的 usefulBytes 都只统计 codec
  // payload,避免投递率 > 100% 的错觉。其余业务的 PacketSink::Rx 已是应用层
  // payload,不需要再剥。
  //流量接收清算
  uint32_t effectiveSize = packet->GetSize ();//获取收到的整个数据包的大小
  if (type == TRAFFIC_VOIP_RTP)
    {
      static constexpr uint32_t kRtpHeaderBytes = 12;//强制将有效大小（effectiveSize）减去 12 个字节的 RTP 头部开销
      effectiveSize = (effectiveSize > kRtpHeaderBytes) ? (effectiveSize - kRtpHeaderBytes) : 0;
    }//将扣减后的真实净荷（比如 60 - 12 = 48 字节）累加到接收统计字典（m_rxBytesByType）和总交付量（usefulBytesDelivered）中

  m_rxBytesByType[type][nodeId] += effectiveSize;              // 按业务+节点累计接收字节
  m_totalStats.usefulBytesDelivered += effectiveSize;          // 总交付有效字节
  m_totalStats.perUserUsefulBytes[nodeId] += effectiveSize;    // 总账按用户累计
  TrafficModelStats& typeStats = m_statsByType[type];          // 取该业务类型的统计槽
  typeStats.usefulBytesDelivered += effectiveSize;             // 该业务交付有效字节
  typeStats.perUserUsefulBytes[nodeId] += effectiveSize;       // 该业务按用户累计
}

// 【共用,带地址版】PacketSink::Rx 回调:解析上下文后转交 RecordReceivedPacket(FTP/VoIP/FullBuffer 用)。
void
SatTrafficGenerator::NotifyPacketRx (std::string context,        // "nodeId:type" 上下文
                                     Ptr<const Packet> packet,   // 收到的包
                                     const Address& from)        // 源地址(未用)
{
  uint32_t nodeId = 0;                              // 解析出的节点 id
  TrafficModelType type = TRAFFIC_FULL_BUFFER;      // 解析出的业务类型
  if (ParseTraceContext (context, nodeId, type))    // 解析成功才记账
    {
      RecordReceivedPacket (nodeId, type, packet);
    }
  (void) from;                                      // 显式消除未使用参数告警
}

// 【HTTP 用,无地址版】HTTP client 的 RxMainObjectPacket/RxEmbeddedObjectPacket trace 回调。
void
SatTrafficGenerator::NotifyPacketRxNoAddress (std::string context, Ptr<const Packet> packet)   // 上下文 + 收到的对象分片
{
  uint32_t nodeId = 0;                              // 解析出的节点 id
  TrafficModelType type = TRAFFIC_FULL_BUFFER;      // 解析出的业务类型
  if (ParseTraceContext (context, nodeId, type))    // 解析成功才记账
    {
      RecordReceivedPacket (nodeId, type, packet);  // HTTP 收到的对象字节计入接收统计
    }
}

Time
SatTrafficGenerator::ScaledInterval (Time base) const
{
  double multiplier = m_currentRateMultiplier > 0 ? m_currentRateMultiplier : 1.0;
  double us = base.GetMicroSeconds () / multiplier;
  if (us < 100.0)
    {
      us = 100.0;
    }
  return MicroSeconds (static_cast<uint64_t> (us));
}

void
SatTrafficGenerator::AccountTx (FlowState& flow, uint32_t bytesSentOnWire, uint32_t usefulBytes)
{
  AccountTx (flow.type, flow.nodeId, bytesSentOnWire, usefulBytes);
}

void
SatTrafficGenerator::AccountTx (TrafficModelType type,
                                uint32_t nodeId,
                                uint32_t bytesSentOnWire,
                                uint32_t usefulBytes)
{
  const Time now = Simulator::Now ();
  UpdateTxStats (m_totalStats, bytesSentOnWire, usefulBytes, now);
  UpdateTxStats (m_statsByType[type], bytesSentOnWire, usefulBytes, now);
  UpdatePeakStats (m_totalStats,
                   nodeId,
                   m_totalWindowBytesByNode[nodeId],
                   m_totalWindowStartByNode[nodeId],
                   m_totalFlowPeakBpsByNode[nodeId],
                   usefulBytes,
                   now);

  const auto key = std::make_pair (type, nodeId);
  UpdatePeakStats (m_statsByType[type],
                   nodeId,
                   m_typeWindowBytesByFlow[key],
                   m_typeWindowStartByFlow[key],
                   m_typeFlowPeakBpsByFlow[key],
                   usefulBytes,
                   now);
}

void
SatTrafficGenerator::UpdateTxStats (TrafficModelStats& stats,
                                    uint32_t bytesSentOnWire,
                                    uint32_t usefulBytes,
                                    Time now)
{
  stats.totalBytesGenerated += usefulBytes;
  stats.totalBytesSent += bytesSentOnWire;
  stats.totalPacketsGenerated++;
  stats.ipv4Packets++;

  if (stats.lastGenerateTime != Time (0))
    {
      double dtUs = (now - stats.lastGenerateTime).GetMicroSeconds ();
      double alpha = 0.05;
      if (stats.avgGenerateIntervalUs == 0)
        {
          stats.avgGenerateIntervalUs = dtUs;
        }
      else
        {
          stats.avgGenerateIntervalUs =
              (1.0 - alpha) * stats.avgGenerateIntervalUs + alpha * dtUs;
        }
    }
  stats.lastGenerateTime = now;
  (void) usefulBytes;
}

void
SatTrafficGenerator::UpdatePeakStats (TrafficModelStats& stats,
                                      uint32_t nodeId,
                                      uint64_t& windowBytes,
                                      Time& windowStart,
                                      double& flowPeakBytesPerSec,
                                      uint32_t usefulBytes,
                                      Time now)
{
  if (windowStart == Time (0) || (now - windowStart) >= Seconds (1.0))
    {
      if (windowStart != Time (0))
        {
          double wSecs = (now - windowStart).GetSeconds ();
          double bps = (windowBytes * 8.0) / std::max (0.001, wSecs);
          flowPeakBytesPerSec = std::max (flowPeakBytesPerSec, bps);
          stats.perUserPeakBps[nodeId] = std::max (stats.perUserPeakBps[nodeId], bps);
          stats.peakThroughputBps = std::max (stats.peakThroughputBps, bps);
        }
      windowStart = now;
      windowBytes = 0;
    }
  windowBytes += usefulBytes;
}

void
SatTrafficGenerator::SetQueueStateCallback (Callback<void, double> cb)
{
  m_queueStateCallback = cb;
}

void
SatTrafficGenerator::ReportQueueUtilization (double utilizationPercent)
{
  if (!m_queueStateCallback.IsNull ())
    {
      m_queueStateCallback (utilizationPercent);
    }
  AdjustRateForClosedLoop (utilizationPercent);
}

void
SatTrafficGenerator::SetTxTraceCallback (Callback<void, Ptr<const Packet>, const Address&> cb)
{
  m_txTraceCallback = cb;
}

void
SatTrafficGenerator::OnTypedPacketTx (TrafficModelType type,
                                      uint32_t nodeId,
                                      Ptr<const Packet> packet)
{
  if (!packet)
    {
      return;
    }

  AccountTx (type, nodeId, packet->GetSize (), packet->GetSize ());

  if (!m_txTraceCallback.IsNull ())
    {
      Address empty;
      m_txTraceCallback (packet, empty);
    }
}

void
SatTrafficGenerator::OnQueueStateFeedback (double utilizationPercent)
{
  ReportQueueUtilization (utilizationPercent);
}

void
SatTrafficGenerator::AdjustRateForClosedLoop (double utilizationPercent)
{
  if (!m_closedLoopControlEnabled)
    {
      return;
    }

  double newMultiplier = m_currentRateMultiplier;
  if (utilizationPercent < m_lowWatermarkPercent)
    {
      newMultiplier = std::min (3.0, m_currentRateMultiplier * 1.5);
    }
  else if (utilizationPercent > m_highWatermarkPercent)
    {
      newMultiplier = std::max (0.5, m_currentRateMultiplier * 0.8);
    }

  m_currentRateMultiplier = newMultiplier;
  ApplyManagedTcpSocketRateControl ();
}

void
SatTrafficGenerator::EnableClosedLoopControl (bool enable, uint8_t lowWatermarkPercent)
{
  m_closedLoopControlEnabled = enable;
  m_lowWatermarkPercent = lowWatermarkPercent;
  m_highWatermarkPercent = std::min<uint8_t> (100, lowWatermarkPercent + 15);
}

void
SatTrafficGenerator::SetTrafficModel (TrafficModelType type)
{
  m_modelType = NormalizeTrafficModelType (type);
  MarkTrafficModelInstalled (type);
}

void
SatTrafficGenerator::SetTargetRate (uint64_t rateBitsPerSec)
{
  m_targetRateBps = rateBitsPerSec;
}

void
SatTrafficGenerator::SetGeoRtt (Time rtt)
{
  m_geoRtt = rtt;
}

void
SatTrafficGenerator::SetMaxPrbPerTti (uint32_t numPrb)
{
  m_maxPrbPerTti = numPrb;
}

void
SatTrafficGenerator::SetRohcOverheadBytes (uint32_t bytes)
{
  m_rohcOverheadBytes = bytes;
}

void
SatTrafficGenerator::SetTcpCongestionTypeId (TypeId typeId)
{
  m_tcpCongestionTypeId = typeId;
}

TypeId
SatTrafficGenerator::GetTcpCongestionTypeId () const
{
  return m_tcpCongestionTypeId;
}

// ===== 3GPP TR 37.901 §5.4.2 FTP 模型可配置项的 setter =====
void
SatTrafficGenerator::SetFtpModel (FtpModelType model)
{
  m_ftpModel = static_cast<uint8_t> (model);
}

FtpModelType
SatTrafficGenerator::GetFtpModel () const
{
  return static_cast<FtpModelType> (m_ftpModel);
}

void
SatTrafficGenerator::SetFtpFileSize (double meanBytes, double stdDevBytes)
{
  m_ftpFileSizeMeanBytes = meanBytes;
  m_ftpFileSizeStdDevBytes = stdDevBytes;
  m_ftpFixedFileSizeBytes = 0;            // 切回对数正态分布模式
  if (m_ftpFileSizeRv != nullptr)
    {
      EnsureFtpRandomVariables ();         // 立即用新参数刷新 Mu/Sigma
    }
}

void
SatTrafficGenerator::SetFtpFileSizeBounds (uint32_t minBytes, uint32_t maxBytes)
{
  m_ftpFileSizeMinBytes = minBytes;
  m_ftpFileSizeMaxBytes = maxBytes;
}

void
SatTrafficGenerator::SetFtpFixedFileSize (uint32_t fixedBytes)
{
  m_ftpFixedFileSizeBytes = fixedBytes;   // >0 时启用固定大小模式
}

void
SatTrafficGenerator::SetFtpArrivalRate (double filesPerSecond)
{
  m_ftpArrivalRate = filesPerSecond;
  if (m_ftpArrivalRv != nullptr)
    {
      m_ftpArrivalRv->SetAttribute ("Mean",
                                    DoubleValue (1.0 / std::max (1e-6, m_ftpArrivalRate)));
    }
}

void
SatTrafficGenerator::SetFtpReadingTimeMean (double seconds)
{
  m_ftpReadingTimeMeanSeconds = seconds;
  if (m_ftpReadingTimeRv != nullptr)
    {
      m_ftpReadingTimeRv->SetAttribute ("Mean",
                                        DoubleValue (std::max (0.0, m_ftpReadingTimeMeanSeconds)));
    }
}

// ===== 任务1:VoIP codec setter =====
void
SatTrafficGenerator::SetVoipCodec (uint32_t activePayloadBytes,
                                   uint32_t frameMs,
                                   uint32_t silencePayloadBytes,
                                   uint32_t silenceFrameMs,
                                   uint8_t payloadType,
                                   uint32_t sampleRateHz)
{
  m_voipActivePayloadBytes = std::max (1u, activePayloadBytes);
  m_voipFrameMs = std::max (1u, frameMs);
  m_voipSilencePayloadBytes = std::max (1u, silencePayloadBytes);
  m_voipSilenceFrameMs = std::max (1u, silenceFrameMs);
  m_voipPayloadType = payloadType;
  m_voipSampleRateHz = std::max (1u, sampleRateHz);
}

void
SatTrafficGenerator::SetVoipActivePayloadBytes (uint32_t bytes)
{
  m_voipActivePayloadBytes = std::max (1u, bytes);
}

void
SatTrafficGenerator::SetVoipFrameMs (uint32_t ms)
{
  m_voipFrameMs = std::max (1u, ms);
}

void
SatTrafficGenerator::SetVoipSilencePayloadBytes (uint32_t bytes)
{
  m_voipSilencePayloadBytes = std::max (1u, bytes);
}

void
SatTrafficGenerator::SetVoipSilenceFrameMs (uint32_t ms)
{
  m_voipSilenceFrameMs = std::max (1u, ms);
}

void
SatTrafficGenerator::SetVoipPayloadType (uint8_t pt)
{
  m_voipPayloadType = pt;
}

void
SatTrafficGenerator::SetVoipSampleRateHz (uint32_t hz)
{
  m_voipSampleRateHz = std::max (1u, hz);
}

// ===== 任务3:HTTP 会话/页面到达 Poisson setter/getter =====
void
SatTrafficGenerator::SetHttpSessionArrivalRate (double sessionsPerSecond)
{
  m_httpSessionArrivalRate = std::max (0.0, sessionsPerSecond);
  if (m_httpSessionArrivalRv != nullptr && m_httpSessionArrivalRate > 0.0)
    {
      m_httpSessionArrivalRv->SetAttribute ("Mean",
                                            DoubleValue (1.0 / m_httpSessionArrivalRate));
    }
}

double
SatTrafficGenerator::GetHttpSessionArrivalRate () const
{
  return m_httpSessionArrivalRate;
}

void
SatTrafficGenerator::SetApplicationWindow (Time start, Time stop)
{
  m_appStartTime = start;
  m_appStopTime = stop;
  NS_LOG_INFO ("Application window set to: start=" << start.GetSeconds ()
               << "s stop=" << stop.GetSeconds () << "s");
}

void
SatTrafficGenerator::PruneInactiveFlows ()
{
  m_flows.erase (std::remove_if (m_flows.begin (),
                                 m_flows.end (),
                                 [] (const std::shared_ptr<FlowState>& flow) {
                                   return flow == nullptr || (flow->started && !flow->active);
                                 }),
                 m_flows.end ());
}

void
SatTrafficGenerator::PrimeVoipSession (VoipSessionState& session, Time now) const
{
  if (session.stateChangeTime != Time (0))
    {
      return;
    }

  session.stateChangeTime = now + Seconds (session.talkSpurtRv->GetValue ());
}

// 任务4:单时钟幂等推进会话状态到时刻 now。两条 leg 在同一仿真时刻都会调用,但只有
// 第一个调用(lastAdvancedTime < now)会真正推进 talk/silence 状态机;后到的 leg
// 看到 lastAdvancedTime >= now 直接返回,读取已推进后的一致快照。这样切换瞬间两条
// leg 看到完全相同的 (inMutualSilence, legATalking),不会一条已切、一条未切而多发/漏发。
void
SatTrafficGenerator::AdvanceVoipSession (VoipSessionState& session, Time now) const
{
  PrimeVoipSession (session, now);   // 首次惰性初始化首个 talk spurt 截止时刻

  // 幂等保护:本时刻已推进过则跳过(另一条 leg 已推进)。用 >= 保证同一 now 只推进一次。
  if (session.lastAdvancedTime >= now)
    {
      return;
    }
  session.lastAdvancedTime = now;

  // 推进 talk/silence 状态机直到 stateChangeTime 落在 now 之后。
  while (now >= session.stateChangeTime)
    {
      if (session.inMutualSilence)        // 之前双方静音 → 静音结束,切换说话人
        {
          session.inMutualSilence = false;
          session.legATalking = !session.legATalking;
          session.stateChangeTime = now + Seconds (session.talkSpurtRv->GetValue ());
        }
      else                                // 之前有人说话 → 进入互相静音(turn gap)
        {
          session.inMutualSilence = true;
          session.stateChangeTime = now + Seconds (session.turnGapRv->GetValue ());
        }
    }
}

void
SatTrafficGenerator::ResetStats ()
{
  for (const auto& flow : m_flows)
    {
      if (flow)
        {
          flow->active = false;
        }
    }

  // FTP driver:停掉后续文件的到达/排队,避免统计清空后继续偷偷传文件。
  for (const auto& ftp : m_ftpDrivers)
    {
      if (ftp)
        {
          ftp->active = false;
        }
    }
  m_ftpDrivers.clear ();

  // 任务3:停掉 HTTP 会话/页面 Poisson 到达过程,避免统计清空后继续 spawn 新会话。
  m_httpArrivalActive = false;
  m_httpArrivalContexts.clear ();

  // Issue 1: 在清空各 registry 之前,先断开所有 ns-3 trace 连接。否则旧的
  // PacketSink::Rx / Ipv4L3Protocol::SendOutgoing / ThreeGppHttpClient 等 trace
  // 仍然存活,后续 Install 会再连一次,同一个数据包会触发两次 AccountTx → 双重计数。
  for (auto& disconnect : m_traceDisconnects)
    {
      if (disconnect)
        {
          disconnect ();
        }
    }
  m_traceDisconnects.clear ();

  // 清理已挂在 node 上的 SatFlowDriverApp 与标准 ns-3 业务应用:
  //  - 把 stop time 提前到 now+1us 让 ns-3 尽快触发 StopApplication
  //  - 清空 driver app 的 start/stop callback,使后续触发的事件成为 no-op
  //  - Issue 6: BulkSendApplication / ThreeGppHttpClient/Server 同样要停,
  //    否则它们会在统计被清空之后继续偷偷跑、形成隐形网络负载
  const Time stopAt = Simulator::Now () + MicroSeconds (1);
  for (auto& app : m_driverApps)
    {
      if (app == nullptr)
        {
          continue;
        }
      app->SetStartCallback ({});
      app->SetStopCallback ({});
      app->SetStopTime (stopAt);
    }
  m_driverApps.clear ();

  for (auto& app : m_managedApps)
    {
      if (app == nullptr)
        {
          continue;
        }
      app->SetStopTime (stopAt);
    }
  m_managedApps.clear ();

  m_totalStats = TrafficModelStats {};
  m_statsByType.clear ();
  m_modelType = TRAFFIC_FULL_BUFFER;
  m_installedTypes.clear ();
  m_flows.clear ();
  m_nodesByAddress.clear ();
  m_httpServersByNodeId.clear ();
  m_httpServerProfilesByNodeId.clear ();
  m_managedTcpSockets.clear ();
  m_ipv4TcpTxTraceRegistry.clear ();
  m_arpSeedRegistry.clear ();
  m_rxBytesByType.clear ();
  m_currentRateMultiplier = 1.0;
  m_nextSsrc = 0xC0DE0000u;
  m_totalWindowBytes = 0;
  m_totalWindowStart = Time (0);
  m_totalWindowBytesByNode.clear ();
  m_totalWindowStartByNode.clear ();
  m_totalFlowPeakBpsByNode.clear ();
  m_typeWindowBytesByFlow.clear ();
  m_typeWindowStartByFlow.clear ();
  m_typeFlowPeakBpsByFlow.clear ();
}

void
SatTrafficGenerator::DumpStats (std::ostream& os) const
{
  os << "=== Traffic Model Statistics ===" << std::endl;
  os << "Installed Models: " << DescribeInstalledTrafficModels () << std::endl;
  os << "Primary Model Enum: " << static_cast<int> (GetTrafficModel ()) << std::endl;
  os << "TCP Congestion Control: " << m_tcpCongestionTypeId.GetName () << std::endl;
  os << "Generated Payload Bytes (L5+): " << m_totalStats.totalBytesGenerated << std::endl;
  os << "Transmitted Bytes (L3+): " << m_totalStats.totalBytesSent << std::endl;
  os << "Useful Bytes Delivered: " << m_totalStats.usefulBytesDelivered << std::endl;
  os << "Total Packets Generated: " << m_totalStats.totalPacketsGenerated << std::endl;
  os << "Send Blocked Events: " << m_totalStats.sendBlockedCount << std::endl;

  double simSecs = Simulator::Now ().GetSeconds ();
  if (simSecs > 0.0)
    {
      os << "Wire Throughput (L3+): "
         << (m_totalStats.totalBytesSent * 8.0) / simSecs / 1e6 << " Mbps" << std::endl;
      os << "Useful Throughput (L5+): "
         << (m_totalStats.usefulBytesDelivered * 8.0) / simSecs / 1e6 << " Mbps" << std::endl;
      // Issue 7: 之前还输出 "Beam Peak",但 SatTrafficGenerator 并不知道 beam→UE
      // 映射,该字段实际只是和 Per-user Peak 同源重复。beam 级峰值应由上层
      // (scheduler/stats collector,持有 beam 拓扑) 自行聚合。
      os << "Per-user Peak (1s window): " << m_totalStats.peakThroughputBps / 1e6 << " Mbps"
         << std::endl;
    }
  for (const auto& [type, stats] : m_statsByType)
    {
      os << "Type " << TrafficModelTypeToString (type) << " (" << static_cast<int> (type) << ")"
         << " delivered=" << stats.usefulBytesDelivered
         << " bytes, payloadGenerated=" << stats.totalBytesGenerated
         << " bytes, tx=" << stats.totalBytesSent << " bytes" << std::endl;
    }
  os << "================================" << std::endl;
}

void
SatTrafficGenerator::MarkTrafficModelInstalled (TrafficModelType type)
{
  m_installedTypes.insert (NormalizeTrafficModelType (type));
}

const char*
SatTrafficGenerator::TrafficModelTypeToString (TrafficModelType type)
{
  switch (type)
    {
    case TRAFFIC_FULL_BUFFER:
      return "FULL_BUFFER";
    case TRAFFIC_FTP:
      return "FTP";
    case TRAFFIC_HTTP:
      return "HTTP";
    case TRAFFIC_VOIP_RTP:
      return "VOIP_RTP";
    case TRAFFIC_LEGACY_DATA:
      return "HTTP_ALIAS";
    case TRAFFIC_LEGACY_VOICE:
      return "VOIP_RTP_ALIAS";
    case TRAFFIC_MIXED:
      return "MIXED";
    default:
      return "UNKNOWN";
    }
}

} // namespace ns3
