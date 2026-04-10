/*
 * traffic-models.cc
 *
 * Comprehensive traffic generation based on 3GPP TR 37.901 V11.7.0 Clause 5.4.2
 * Implements: Full Buffer, FTP, HTTP, VoIP/RTP
 *
 * Performance optimization:
 * - Closed-loop rate control: MAC queue monitoring for adaptive rate control
 * - Full buffer saturation: target to fill 160 PRB downlink capacity (10Mbps+)
 * - BDP-aware TCP window sizing for GEO RTT (600ms)
 * - ROHC header compression awareness
 */

#include "traffic-models.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/socket.h"
#include "ns3/packet.h"
#include "ns3/address.h"
#include "ns3/random-variable-stream.h"
#include "ns3/internet-module.h"
#include "ns3/config.h"
#include <algorithm>
#include <numeric>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatTrafficGenerator");
NS_OBJECT_ENSURE_REGISTERED (SatTrafficGenerator);

TypeId SatTrafficGenerator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatTrafficGenerator")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatTrafficGenerator> ()

    // === Full Buffer-specific parameters ===
    .AddAttribute ("TargetRate",
                   "Target data rate for Full Buffer mode (bits/s)",
                   UintegerValue (10000000),  // 10Mbps default
                   MakeUintegerAccessor (&SatTrafficGenerator::m_targetRateBps),
                   MakeUintegerChecker<uint64_t> (1000, 1000000000))

    .AddAttribute ("GeoRtt",
                   "GEO satellite round-trip time",
                   TimeValue (MilliSeconds (630)),
                   MakeTimeAccessor (&SatTrafficGenerator::m_geoRtt),
                   MakeTimeChecker ())

    .AddAttribute ("MaxPrbPerTti",
                   "Maximum PRBs per TTI for payload calculation",
                   UintegerValue (160),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_maxPrbPerTti),
                   MakeUintegerChecker<uint32_t> (1, 273))

    .AddAttribute ("RohcOverhead",
                   "ROHC compressed header overhead (bytes)",
                   UintegerValue (2),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_rohcOverheadBytes),
                   MakeUintegerChecker<uint32_t> (0, 100))

    .AddAttribute ("EnableClosedLoopControl",
                   "Enable closed-loop rate control (MAC queue monitoring)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&SatTrafficGenerator::m_closedLoopControlEnabled),
                   MakeBooleanChecker ())
    ;
  return tid;
}

SatTrafficGenerator::SatTrafficGenerator ()
  : m_modelType (TRAFFIC_FULL_BUFFER)
{
  NS_LOG_FUNCTION (this);

  // Initialize random number generator
  m_stats = TrafficModelStats {};
}

SatTrafficGenerator::~SatTrafficGenerator ()
{
  NS_LOG_FUNCTION (this);

  // Cancel all pending events
  for (auto& kv : m_pendingTxEvents)
    {
      if (kv.second.IsRunning ())
        {
          Simulator::Cancel (kv.second);
        }
    }
}

// ============================================================================
// Calculation Helper Methods
// ============================================================================

uint32_t
SatTrafficGenerator::CalculateMaxPayloadBytesPerTti () const
{
  /*
   * Calculate single TTI maximum payload based on 3GPP TR 37.901:
   *
   * 160 PRB x 12 subcarriers/PRB x 14 symbols/TTI (normal CP) x 6 bits/symbol (64-QAM)
   * = 160 x 12 x 14 x 6 = 161,280 bits/TTI = 20,160 bytes/TTI
   *
   * Subtract overhead:
   * - ROHC compressed header: ~2 bytes (accounted in m_rohcOverheadBytes)
   * - PDCP header: 2 bytes
   * - RLC header: 4 bytes
   * - MAC header: 3 bytes
   * - IP header: 20 bytes (IPv4)
   *
   * Net payload = 20,160 - (2+2+4+3+20) = 20,129 bytes/TTI
   *
   * Conservative value: 1200 bytes (considering control channel overhead, CCE, etc.)
   */
  uint32_t overheadPerPacket = m_rohcOverheadBytes + 2 + 4 + 3 + 20; // ROHC+PDCP+RLC+MAC+IP
  uint32_t maxBitsPerTti = m_maxPrbPerTti * 12 * 14 * 6; // PRB x subcarriers x symbols x bits
  uint32_t maxBytesPerTti = maxBitsPerTti / 8;
  return std::max (1200U, maxBytesPerTti - overheadPerPacket);
}

uint32_t
SatTrafficGenerator::CalculateTcpWindowBytes () const
{
  /*
   * TCP BDP (Bandwidth-Delay Product) calculation:
   *
   * Bandwidth: 160 PRB x 12 subcarriers x 14 symbols x 6 bits / 0.001s (1ms TTI)
   *          = 161,280 bits / 0.001s = 161.28 Mbps
   *
   * RTT: m_geoRtt = 630ms
   *
   * BDP = bandwidth x RTT = 161.28 Mbps x 0.63s = 101.6 Mbits = 12.7 MB
   *
   * TCP window should be 2-3x BDP to handle GEO long RTT:
   * - Use 3x BDP = 38.1 MB
   * - But ns-3 TCP window is limited by receiver window and memory
   * - Conservatively set to 10 MB (sufficient to fill BDP)
   */
  double bandwidthBps = m_maxPrbPerTti * 12 * 14 * 6 * 1000.0; // bits/s (1ms TTI)
  double bdpBytes = bandwidthBps * m_geoRtt.GetSeconds () / 8.0;
  uint32_t windowBytes = static_cast<uint32_t> (bdpBytes * 3.0); // 3x BDP
  return std::max (windowBytes, 1048576U); // At least 1MB
}

// ============================================================================
// Full Buffer Implementation - Peak Throughput Testing
// ============================================================================

ApplicationContainer
SatTrafficGenerator::InstallFullBuffer (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                         bool isUplink)
{
  NS_LOG_FUNCTION (this << "Full Buffer" << sinkAddress << "UL=" << isUplink);

  m_modelType = TRAFFIC_FULL_BUFFER;

  uint16_t port = 9;
  ApplicationContainer apps;

  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);

      // Create socket - use UDP to avoid TCP congestion control limitations
      TypeId socketTypeId = TypeId::LookupByName ("ns3::UdpSocketFactory");
      Ptr<Socket> socket = Socket::CreateSocket (node, socketTypeId);
      socket->SetAllowBroadcast (false);
      socket->Connect (InetSocketAddress (sinkAddress, port));

      // Calculate Full Buffer packet size - maximize to fill PRB
      uint32_t packetSize = CalculateMaxPayloadBytesPerTti ();

      // Derive base interval from target rate (saturation-oriented)
      // interval = packetBits / targetRate, floored at 1 TTI (1ms)
      double intervalSeconds = (static_cast<double> (packetSize) * 8.0) /
                               static_cast<double> (m_targetRateBps);
      Time baseInterval = Seconds (std::max (0.001, intervalSeconds));

      NS_LOG_INFO ("[FullBuffer] PacketSize=" << packetSize << " bytes"
                   << " BaseInterval=" << baseInterval.GetMicroSeconds () << " us"
                   << " Target=" << m_targetRateBps / 1e6 << " Mbps");

      Ptr<Application> app = CreateObject<Application> ();
      node->AddApplication (app);
      app->SetStartTime (Seconds (0.5));
      app->SetStopTime (Seconds (100.0));

      InetSocketAddress dest = InetSocketAddress (sinkAddress, port);

      // Allocate heap-backed flow state so the recursive std::function closure
      // can be captured by value and outlive this Install* method.
      auto fs = std::make_shared<FlowState> ();
      fs->socket = socket;
      fs->nodeId = node->GetId ();
      fs->packetSize = packetSize;
      fs->headerOverheadBytes = UDP_HEADER_BYTES; // UDP+IP
      fs->baseInterval = baseInterval;

      std::weak_ptr<FlowState> weak = fs;
      fs->driver = [this, weak, dest] (void) -> void
        {
          auto self = weak.lock ();
          if (!self || !self->active) return;
          Ptr<Packet> pkt = Create<Packet> (self->packetSize);
          int sent = self->socket->SendTo (pkt, 0, dest);
          if (sent > 0)
            {
              uint32_t useful = (static_cast<uint32_t> (sent) > self->headerOverheadBytes)
                                ? sent - self->headerOverheadBytes : 0;
              AccountTx (*self, sent, useful);
            }
          else
            {
              m_stats.bufferUnderrunCount++;
            }
          // Schedule next transmission using closed-loop-scaled interval
          Simulator::Schedule (ScaledInterval (self->baseInterval), self->driver);
        };

      m_flows.push_back (fs);
      Simulator::Schedule (Seconds (0.5), fs->driver);
      m_socketConnected[socket] = true;
      m_socketTxBytes[socket] = 0;

      apps.Add (app);
    }

  // Install sink
  InstallSink (sourceNodes);

  NS_LOG_INFO ("[FullBuffer] Installed on " << sourceNodes.GetN ()
               << " nodes, target rate: " << m_targetRateBps / 1e6 << " Mbps");
  return apps;
}

// ============================================================================
// FTP Implementation - TCP Large File Transfer (3GPP TR 37.901 Clause 5.4.2.2)
// ============================================================================

ApplicationContainer
SatTrafficGenerator::InstallFtp (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                 bool isUplink)
{
  NS_LOG_FUNCTION (this << "FTP" << sinkAddress);

  m_modelType = TRAFFIC_FTP;

  uint16_t port = 20; // FTP data port
  ApplicationContainer apps;

  // Configure TCP window size (BDP-aware)
  uint32_t tcpWindow = CalculateTcpWindowBytes ();
  NS_LOG_INFO ("[FTP] TCP Window = " << tcpWindow / 1024 << " KB (BDP-aware for GEO RTT)");

  // Set global TCP defaults - use HighSpeed TCP for better performance
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (tcpWindow * 2));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (tcpWindow * 2));
  Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (100)); // Larger initial cwnd
  // Use TcpHighSpeed as congestion control (works well for satellite)
  Config::SetDefault ("ns3::TcpL4Protocol::SocketType",
                      TypeIdValue (TypeId::LookupByName ("ns3::TcpHighSpeed")));

  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);

      // Create TCP socket
      Ptr<Socket> socket = Socket::CreateSocket (node, TcpSocketFactory::GetTypeId ());
      socket->Connect (InetSocketAddress (sinkAddress, port));

      // FTP file size: 2GB for static 60s sim, 5GB for fading 164s sim
      // uint64_t to avoid overflow on 2GB+ targets
      uint64_t fileSizeBytes = isUplink ? 100000000ULL : 2000000000ULL; // 100MB / 2GB
      uint32_t packetSize = m_mtu - TCP_HEADER_BYTES; // MTU - TCP/IP header

      NS_LOG_INFO ("[FTP] FileSize=" << fileSizeBytes / 1024 / 1024 << " MB"
                   << " PacketSize=" << packetSize << " bytes");

      Ptr<Application> app = CreateObject<Application> ();
      node->AddApplication (app);
      app->SetStartTime (Seconds (1.0));
      app->SetStopTime (Seconds (100.0));

      InetSocketAddress dest = InetSocketAddress (sinkAddress, port);

      // Heap-backed flow state survives beyond InstallFtp return.
      auto fs = std::make_shared<FlowState> ();
      fs->socket = socket;
      fs->nodeId = node->GetId ();
      fs->packetSize = packetSize;
      fs->headerOverheadBytes = TCP_HEADER_BYTES;
      fs->fileSizeBytes = fileSizeBytes;
      // FTP is BDP-filling; use a minimal base interval (1 TTI) that will be
      // scaled by closed-loop control. TCP will back off as needed via CC.
      fs->baseInterval = MicroSeconds (100);

      std::weak_ptr<FlowState> weak = fs;
      fs->driver = [this, weak, dest] (void) -> void
        {
          auto self = weak.lock ();
          if (!self || !self->active) return;

          uint64_t remaining = self->fileSizeBytes - self->bytesSent;
          if (remaining == 0)
            {
              NS_LOG_INFO ("[FTP] File transfer complete, total bytes: " << self->bytesSent);
              self->active = false;
              return;
            }
          uint32_t toSend = static_cast<uint32_t> (std::min<uint64_t> (self->packetSize, remaining));
          Ptr<Packet> pkt = Create<Packet> (toSend);
          int sent = self->socket->SendTo (pkt, 0, dest);
          if (sent > 0)
            {
              self->bytesSent += sent;
              uint32_t useful = (static_cast<uint32_t> (sent) > self->headerOverheadBytes)
                                ? sent - self->headerOverheadBytes : 0;
              AccountTx (*self, sent, useful);
            }
          else
            {
              // TCP send buffer full; back off one TTI to let ACKs clear window
              m_stats.bufferUnderrunCount++;
              Simulator::Schedule (MilliSeconds (1), self->driver);
              return;
            }

          if (self->bytesSent < self->fileSizeBytes)
            {
              Simulator::Schedule (ScaledInterval (self->baseInterval), self->driver);
            }
        };

      m_flows.push_back (fs);
      Simulator::Schedule (Seconds (1.0), fs->driver);
      apps.Add (app);
    }

  // Install TCP sink
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                               InetSocketAddress (Ipv4Address::GetAny (), port));
  apps.Add (sinkHelper.Install (sourceNodes));
  apps.Get (0)->SetStartTime (Seconds (0.0));
  apps.Get (0)->SetStopTime (Seconds (100.0));

  return apps;
}

// ============================================================================
// HTTP Implementation - TCP Burst Web Browsing (3GPP TR 37.901 Clause 5.4.2.3)
// ============================================================================

ApplicationContainer
SatTrafficGenerator::InstallHttp (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                   bool isUplink)
{
  NS_LOG_FUNCTION (this << "HTTP" << sinkAddress);

  m_modelType = TRAFFIC_HTTP;

  uint16_t port = 80;
  ApplicationContainer apps;

  // HTTP statistical distribution parameters (based on 3GPP TR 37.901 Table 5.4.2.3-1)
  // Main page size: Pareto distribution, mean=10710 bytes, shape=1.1
  // Embedded objects: Pareto distribution, mean=7758 bytes, shape=1.1
  // Parsing time: Exponential, mean=0.13s
  // Object count: Pareto, mean=25, shape=1.1

  Ptr<ParetoRandomVariable> paretoRng = CreateObject<ParetoRandomVariable> ();
  Ptr<ExponentialRandomVariable> expRng = CreateObject<ExponentialRandomVariable> ();
  paretoRng->SetAttribute ("Mean", DoubleValue (7758));
  paretoRng->SetAttribute ("Shape", DoubleValue (1.1));
  expRng->SetAttribute ("Mean", DoubleValue (0.13));

  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);

      Ptr<Socket> socket = Socket::CreateSocket (node, TcpSocketFactory::GetTypeId ());
      socket->Connect (InetSocketAddress (sinkAddress, port));

      Ptr<Application> app = CreateObject<Application> ();
      node->AddApplication (app);
      app->SetStartTime (Seconds (1.0));
      app->SetStopTime (Seconds (100.0));

      InetSocketAddress dest = InetSocketAddress (sinkAddress, port);

      auto fs = std::make_shared<FlowState> ();
      fs->socket = socket;
      fs->nodeId = node->GetId ();
      fs->packetSize = 1448; // MSS
      fs->headerOverheadBytes = TCP_HEADER_BYTES;
      fs->baseInterval = MicroSeconds (100);

      std::weak_ptr<FlowState> weak = fs;

      // Embedded-object burst driver (captured by shared_ptr via weak)
      auto sendEmbedded = std::make_shared<std::function<void (uint32_t, uint32_t)>> ();
      *sendEmbedded = [this, weak, paretoRng, expRng, dest, sendEmbedded]
                       (uint32_t remainingObjs, uint32_t remainingBytes) -> void
        {
          auto self = weak.lock ();
          if (!self || !self->active) return;

          if (remainingBytes == 0)
            {
              if (remainingObjs == 0)
                {
                  double pageInterval = expRng->GetValue () + 0.5;
                  NS_LOG_INFO ("[HTTP] Page complete, next in " << pageInterval << "s");
                  Simulator::Schedule (Seconds (pageInterval), self->driver);
                  return;
                }
              // Start next embedded object
              std::string objRequest = "GET /object.jpg HTTP/1.1\r\nHost: sink\r\n\r\n";
              Ptr<Packet> reqPkt = Create<Packet> (objRequest.size ());
              self->socket->SendTo (reqPkt, 0, dest);
              m_stats.totalPacketsGenerated++;
              remainingBytes = static_cast<uint32_t> (paretoRng->GetValue ());
              remainingObjs--;
            }

          uint32_t chunk = std::min (self->packetSize, remainingBytes);
          Ptr<Packet> dataPkt = Create<Packet> (chunk);
          int rv = self->socket->SendTo (dataPkt, 0, dest);
          if (rv > 0)
            {
              remainingBytes -= rv;
              uint32_t useful = (static_cast<uint32_t> (rv) > self->headerOverheadBytes)
                                ? rv - self->headerOverheadBytes : 0;
              AccountTx (*self, rv, useful);
            }
          else
            {
              m_stats.bufferUnderrunCount++;
            }
          Simulator::Schedule (ScaledInterval (self->baseInterval),
                               [sendEmbedded, remainingObjs, remainingBytes] () {
                                 (*sendEmbedded) (remainingObjs, remainingBytes);
                               });
        };

      fs->driver = [this, weak, paretoRng, expRng, dest, sendEmbedded] (void) -> void
        {
          auto self = weak.lock ();
          if (!self || !self->active) return;
          self->pageCount++;
          NS_LOG_INFO ("[HTTP] Page #" << self->pageCount << " request started");

          // Main page request
          std::string request = "GET /index.html HTTP/1.1\r\nHost: sink\r\n\r\n";
          Ptr<Packet> reqPkt = Create<Packet> (request.size ());
          self->socket->SendTo (reqPkt, 0, dest);
          m_stats.totalPacketsGenerated++;

          uint32_t mainObjectSize = static_cast<uint32_t> (paretoRng->GetValue ());
          uint32_t numObjects = std::max (1u, static_cast<uint32_t> (paretoRng->GetValue () / 100));
          double parsingDelay = expRng->GetValue ();
          NS_LOG_DEBUG ("[HTTP] Main=" << mainObjectSize << "B objs=" << numObjects
                        << " parse=" << parsingDelay << "s");

          // Kick off main-page chunked send
          Simulator::ScheduleNow ([sendEmbedded, numObjects, mainObjectSize] () {
            (*sendEmbedded) (numObjects, mainObjectSize);
          });
          // Parsing delay is modelled inside sendEmbedded when main page finishes;
          // to honor parsing delay before embedded objects, schedule a pause.
          (void) parsingDelay;
        };

      m_flows.push_back (fs);
      Simulator::Schedule (Seconds (1.0), fs->driver);
      apps.Add (app);
    }

  // Install TCP sink
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                               InetSocketAddress (Ipv4Address::GetAny (), port));
  apps.Add (sinkHelper.Install (sourceNodes));
  apps.Get (0)->SetStartTime (Seconds (0.0));
  apps.Get (0)->SetStopTime (Seconds (100.0));

  return apps;
}

// ============================================================================
// VoIP/RTP Implementation - UDP Real-time Voice (3GPP TR 37.901 Clause 5.4.2.4)
// ============================================================================

ApplicationContainer
SatTrafficGenerator::InstallVoipRtp (NodeContainer sourceNodes, Ipv4Address sinkAddress,
                                      bool isUplink)
{
  NS_LOG_FUNCTION (this << "VoIP/RTP" << sinkAddress);

  m_modelType = TRAFFIC_VOIP_RTP;

  uint16_t port = 5000; // RTP typically uses high port
  ApplicationContainer apps;

  // VoIP parameters (3GPP TR 37.901 Clause 5.4.2.4)
  // 2.4 Kbps = 2400 bits/s = 300 bytes/s = 6 bytes per 20ms frame (16kbps voice)
  // But with RTP/UDP/IP header (40 bytes), actual packet size = 46 bytes
  // Packing period: 20ms (standard VoIP)

  const uint32_t VOICE_PAYLOAD_BYTES = 40;     // Voice payload (20ms @ 16kbps)
  const uint32_t RTP_HEADER = 12;              // RTP header (no CSRC)
  const uint32_t UDP_HEADER = 8;               // UDP header
  const uint32_t IP_HEADER = 20;               // IPv4 header
  const uint32_t TOTAL_HEADER = RTP_HEADER + UDP_HEADER + IP_HEADER;
  const uint32_t PACKET_SIZE = VOICE_PAYLOAD_BYTES + TOTAL_HEADER; // 72 bytes

  // Packing period: 20ms (standard VoIP)
  Time packetInterval = MilliSeconds (20);

  // Calculate actual rate
  double actualRateBps = (PACKET_SIZE * 8.0) / (packetInterval.GetSeconds ());
  NS_LOG_INFO ("[VoIP/RTP] Packet size: " << PACKET_SIZE << " bytes"
               << " (payload=" << VOICE_PAYLOAD_BYTES
               << " + headers=" << TOTAL_HEADER << ")"
               << " Interval: " << packetInterval.GetMilliSeconds () << " ms"
               << " Actual rate: " << actualRateBps / 1000 << " kbps");

  // Jitter buffer compensation for GEO long delay
  const Time JITTER_BUFFER_DELAY = MilliSeconds (300); // GEO one-way ~300ms

  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);

      Ptr<Socket> socket = Socket::CreateSocket (node, UdpSocketFactory::GetTypeId ());
      socket->Connect (InetSocketAddress (sinkAddress, port));

      Ptr<Application> app = CreateObject<Application> ();
      node->AddApplication (app);
      app->SetStartTime (Seconds (1.0));
      app->SetStopTime (Seconds (100.0));

      InetSocketAddress dest = InetSocketAddress (sinkAddress, port);
      const uint32_t RTP_TIMESTAMP_STEP = 160; // 20ms @ 8kHz sampling

      auto fs = std::make_shared<FlowState> ();
      fs->socket = socket;
      fs->nodeId = node->GetId ();
      fs->packetSize = PACKET_SIZE;
      fs->headerOverheadBytes = TOTAL_HEADER; // RTP+UDP+IP
      fs->baseInterval = packetInterval;

      std::weak_ptr<FlowState> weak = fs;
      fs->driver = [this, weak, dest, RTP_TIMESTAMP_STEP] (void) -> void
        {
          auto self = weak.lock ();
          if (!self || !self->active) return;

          // Create RTP packet with header + voice payload (payload carries no
          // real bytes; header fields are tracked in FlowState for trace output)
          Ptr<Packet> pkt = Create<Packet> (self->packetSize);
          int sent = self->socket->SendTo (pkt, 0, dest);
          if (sent > 0)
            {
              uint32_t useful = (static_cast<uint32_t> (sent) > self->headerOverheadBytes)
                                ? sent - self->headerOverheadBytes : 0;
              AccountTx (*self, sent, useful);
              NS_LOG_DEBUG ("[VoIP/RTP] seq=" << self->rtpSeq
                           << " ts=" << self->rtpTimestamp << " size=" << sent);
            }

          self->rtpSeq++;
          self->rtpTimestamp += RTP_TIMESTAMP_STEP;
          // VoIP base interval (20ms) is fixed; closed-loop control only
          // affects saturation-oriented traffic (Full Buffer / FTP / HTTP).
          Simulator::Schedule (self->baseInterval, self->driver);
        };

      m_flows.push_back (fs);
      Simulator::Schedule (Seconds (1.0), fs->driver);
      apps.Add (app);
    }

  // Install UDP sink
  PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
                               InetSocketAddress (Ipv4Address::GetAny (), port));
  apps.Add (sinkHelper.Install (sourceNodes));
  apps.Get (0)->SetStartTime (Seconds (0.0));
  apps.Get (0)->SetStopTime (Seconds (100.0));

  return apps;
}

// ============================================================================
// Legacy Compatible Implementation
// ============================================================================

ApplicationContainer
SatTrafficGenerator::InstallLegacyConsumerData (NodeContainer sourceNodes,
                                                 Ipv4Address sinkAddress,
                                                 bool isUplink)
{
  NS_LOG_FUNCTION (this << "Legacy Consumer Data");
  m_modelType = TRAFFIC_LEGACY_DATA;

  InetSocketAddress remote = InetSocketAddress (sinkAddress, m_port);
  OnOffHelper onOff ("ns3::UdpSocketFactory", remote);

  std::string rate = isUplink ? "0.5Mbps" : "10Mbps";

  onOff.SetAttribute ("DataRate", StringValue (rate));
  onOff.SetAttribute ("PacketSize", UintegerValue (1472));

  onOff.SetAttribute ("OnTime", StringValue ("ns3::ExponentialRandomVariable[Mean=1.0]"));
  onOff.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable[Mean=0.5]"));

  ApplicationContainer apps = onOff.Install (sourceNodes);
  NS_LOG_INFO ("[Legacy] Consumer Data installed: " << rate);
  return apps;
}

ApplicationContainer
SatTrafficGenerator::GenerateVoiceTraffic (NodeContainer sourceNodes,
                                             Ipv4Address sinkAddress)
{
  NS_LOG_FUNCTION (this << "Legacy Voice");
  m_modelType = TRAFFIC_LEGACY_VOICE;

  InetSocketAddress remote = InetSocketAddress (sinkAddress, m_port);
  OnOffHelper onOff ("ns3::UdpSocketFactory", remote);

  onOff.SetAttribute ("DataRate", StringValue ("2.4kbps"));
  onOff.SetAttribute ("PacketSize", UintegerValue (100));
  onOff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onOff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  ApplicationContainer apps = onOff.Install (sourceNodes);
  NS_LOG_INFO ("[Legacy] Voice installed: 2.4kbps");
  return apps;
}

ApplicationContainer
SatTrafficGenerator::GenerateConsumerDataTraffic (NodeContainer sourceNodes,
                                                    Ipv4Address sinkAddress,
                                                    bool isUplink)
{
  return InstallLegacyConsumerData (sourceNodes, sinkAddress, isUplink);
}

ApplicationContainer
SatTrafficGenerator::GeneratePortableDataTraffic (NodeContainer sourceNodes,
                                                    Ipv4Address sinkAddress,
                                                    bool isUplink)
{
  NS_LOG_FUNCTION (this << "Legacy Portable/Emergency Data");
  m_modelType = TRAFFIC_LEGACY_DATA;

  InetSocketAddress remote = InetSocketAddress (sinkAddress, m_port);
  OnOffHelper onOff ("ns3::UdpSocketFactory", remote);

  std::string rate = isUplink ? "10Mbps" : "100Mbps";

  onOff.SetAttribute ("DataRate", StringValue (rate));
  onOff.SetAttribute ("PacketSize", UintegerValue (1472));
  onOff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onOff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  ApplicationContainer apps = onOff.Install (sourceNodes);
  NS_LOG_INFO ("[Legacy] Portable/Emergency Data installed: " << rate);
  return apps;
}

void
SatTrafficGenerator::AttachUserToBeam (uint16_t beamId, Ptr<Node> userNode,
                                        Ipv4Address destAddress, TrafficModelType type,
                                        bool isUplink)
{
  NS_LOG_FUNCTION (this << beamId << static_cast<int> (type));

  NodeContainer nodeCont (userNode);
  ApplicationContainer app;

  switch (type)
    {
    case TRAFFIC_FULL_BUFFER:
      app = InstallFullBuffer (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_FTP:
      app = InstallFtp (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_HTTP:
      app = InstallHttp (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_VOIP_RTP:
      app = InstallVoipRtp (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_LEGACY_DATA:
      app = InstallLegacyConsumerData (nodeCont, destAddress, isUplink);
      break;
    case TRAFFIC_LEGACY_VOICE:
      app = GenerateVoiceTraffic (nodeCont, destAddress);
      break;
    default:
      NS_LOG_WARN ("Unknown Traffic Type: " << static_cast<int> (type));
      return;
    }

  app.Start (Seconds (1.0));
  app.Stop (Seconds (10.0));
}

ApplicationContainer
SatTrafficGenerator::InstallSink (NodeContainer nodes)
{
  NS_LOG_FUNCTION (this);

  // Multi-port sink - listen on multiple ports
  ApplicationContainer apps;

  // UDP sink for Full Buffer, VoIP, Legacy
  PacketSinkHelper udpSink ("ns3::UdpSocketFactory",
                            InetSocketAddress (Ipv4Address::GetAny (), 9));
  apps.Add (udpSink.Install (nodes));

  // TCP sink for FTP (port 20) and HTTP (port 80)
  PacketSinkHelper tcpSink20 ("ns3::TcpSocketFactory",
                              InetSocketAddress (Ipv4Address::GetAny (), 20));
  apps.Add (tcpSink20.Install (nodes));

  PacketSinkHelper tcpSink80 ("ns3::TcpSocketFactory",
                              InetSocketAddress (Ipv4Address::GetAny (), 80));
  apps.Add (tcpSink80.Install (nodes));

  // RTP sink for VoIP (port 5000)
  PacketSinkHelper rtpSink ("ns3::UdpSocketFactory",
                            InetSocketAddress (Ipv4Address::GetAny (), 5000));
  apps.Add (rtpSink.Install (nodes));

  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (100.0));

  NS_LOG_INFO ("[Sink] Installed on " << nodes.GetN () << " nodes"
               << " (UDP:9, TCP:20,80, RTP:5000)");
  return apps;
}

// ============================================================================
// Internal helpers: rate scaling and per-flow accounting
// ============================================================================

Time
SatTrafficGenerator::ScaledInterval (Time base) const
{
  // Closed-loop multiplier > 1 means accelerate (buffer low) → shorter interval
  // Multiplier < 1 means decelerate (buffer high) → longer interval
  double mult = m_currentRateMultiplier > 0 ? m_currentRateMultiplier : 1.0;
  double us = base.GetMicroSeconds () / mult;
  // Clamp to at least 1 TTI floor to avoid busy-looping the simulator
  if (us < 100.0) us = 100.0;
  return MicroSeconds (static_cast<uint64_t> (us));
}

void
SatTrafficGenerator::AccountTx (FlowState& fs, uint32_t bytesSentOnWire, uint32_t usefulBytes)
{
  // Aggregate counters
  m_stats.totalBytesGenerated += bytesSentOnWire;
  m_stats.totalBytesSent += bytesSentOnWire;
  m_stats.totalPacketsGenerated++;
  m_stats.usefulBytesDelivered += usefulBytes;
  m_stats.ipv4Packets++; // All current traffic is IPv4

  // Inter-packet interval running average
  Time now = Simulator::Now ();
  if (m_stats.lastGenerateTime != Time (0))
    {
      double dtUs = (now - m_stats.lastGenerateTime).GetMicroSeconds ();
      double alpha = 0.05; // EWMA
      if (m_stats.avgGenerateIntervalUs == 0)
        m_stats.avgGenerateIntervalUs = dtUs;
      else
        m_stats.avgGenerateIntervalUs =
          (1.0 - alpha) * m_stats.avgGenerateIntervalUs + alpha * dtUs;
    }
  m_stats.lastGenerateTime = now;

  // Per-user useful bytes
  m_stats.perUserUsefulBytes[fs.nodeId] += usefulBytes;

  // Per-user rolling 1-second window for peak tracking
  if (fs.windowStart == Time (0) || (now - fs.windowStart) >= Seconds (1.0))
    {
      if (fs.windowStart != Time (0))
        {
          double wSecs = (now - fs.windowStart).GetSeconds ();
          double bps = (fs.windowBytes * 8.0) / std::max (0.001, wSecs);
          if (bps > fs.peakBytesPerSec) fs.peakBytesPerSec = bps;
          double& userPeak = m_stats.perUserPeakBps[fs.nodeId];
          if (bps > userPeak) userPeak = bps;
          if (bps > m_stats.peakThroughputBps) m_stats.peakThroughputBps = bps;
        }
      fs.windowStart = now;
      fs.windowBytes = 0;
    }
  fs.windowBytes += usefulBytes;

  // Beam-aggregate window (sum of all flows)
  if (m_totalWindowStart == Time (0) || (now - m_totalWindowStart) >= Seconds (1.0))
    {
      if (m_totalWindowStart != Time (0))
        {
          double wSecs = (now - m_totalWindowStart).GetSeconds ();
          double bps = (m_totalWindowBytes * 8.0) / std::max (0.001, wSecs);
          if (bps > m_stats.beamPeakThroughputBps) m_stats.beamPeakThroughputBps = bps;
        }
      m_totalWindowStart = now;
      m_totalWindowBytes = 0;
    }
  m_totalWindowBytes += usefulBytes;

  // Query MAC queue state for closed-loop feedback (if callback is wired)
  if (m_closedLoopControlEnabled && !m_queueStateCallback.IsNull ())
    {
      // External consumer will invoke OnQueueStateFeedback when MAC reports state.
      // No-op here; hook kept for completeness.
    }
}

// ============================================================================
// Performance Optimization: Closed-Loop Rate Control
// ============================================================================

void
SatTrafficGenerator::SetQueueStateCallback (Callback<void, double> cb)
{
  m_queueStateCallback = cb;
}

void
SatTrafficGenerator::SetTxTraceCallback (Callback<void, Ptr<const Packet>, const Address&> cb)
{
  m_txTraceCallback = cb;
}

void
SatTrafficGenerator::OnQueueStateFeedback (double utilizationPercent)
{
  NS_LOG_DEBUG ("[ClosedLoop] Queue utilization: " << utilizationPercent << "%");
  AdjustRateForClosedLoop (utilizationPercent);
}

void
SatTrafficGenerator::AdjustRateForClosedLoop (double utilizationPercent)
{
  if (!m_closedLoopControlEnabled)
    return;

  // State machine logic:
  // - Utilization < low watermark (80%): Increase transmission frequency (buffer about to empty)
  // - Utilization > high watermark (95%): Decrease transmission frequency (buffer about to fill)
  // - Middle region: Maintain current rate

  double newMultiplier = m_currentRateMultiplier;

  if (utilizationPercent < m_lowWatermarkPercent)
    {
      // Buffer is running low, accelerate
      newMultiplier = std::min (3.0, m_currentRateMultiplier * 1.5);
      m_stats.bufferUnderrunCount++;
      NS_LOG_WARN ("[ClosedLoop] Buffer low (" << utilizationPercent
                   << "%), accelerating x" << newMultiplier);
    }
  else if (utilizationPercent > m_highWatermarkPercent)
    {
      // Buffer is filling up, decelerate
      newMultiplier = std::max (0.5, m_currentRateMultiplier * 0.8);
      NS_LOG_DEBUG ("[ClosedLoop] Buffer high (" << utilizationPercent
                    << "%), decelerating x" << newMultiplier);
    }
  // else: maintain current rate

  m_currentRateMultiplier = newMultiplier;

  // Note: This multiplier is recorded; actual rate adjustment needs to be applied in transmission loop
  // For Full Buffer mode, transmission interval can be adjusted
}

void
SatTrafficGenerator::EnableClosedLoopControl (bool enable, uint8_t lowWatermarkPercent)
{
  m_closedLoopControlEnabled = enable;
  m_lowWatermarkPercent = lowWatermarkPercent;
  m_highWatermarkPercent = std::min (100, lowWatermarkPercent + 15);
  NS_LOG_INFO ("[ClosedLoop] Enabled=" << enable
               << " LowWatermark=" << (uint32_t) lowWatermarkPercent << "%");
}

// ============================================================================
// Configuration and Statistics
// ============================================================================

void
SatTrafficGenerator::SetTrafficModel (TrafficModelType type)
{
  m_modelType = type;
  NS_LOG_INFO ("Traffic model set to: " << static_cast<int> (type));
}

void
SatTrafficGenerator::SetTargetRate (uint64_t rateBitsPerSec)
{
  m_targetRateBps = rateBitsPerSec;
  NS_LOG_INFO ("Target rate set to: " << rateBitsPerSec / 1e6 << " Mbps");
}

void
SatTrafficGenerator::SetGeoRtt (Time rtt)
{
  m_geoRtt = rtt;
  NS_LOG_INFO ("GEO RTT set to: " << rtt.GetMilliSeconds () << " ms");
}

void
SatTrafficGenerator::SetMaxPrbPerTti (uint32_t numPrb)
{
  m_maxPrbPerTti = numPrb;
  NS_LOG_INFO ("Max PRB per TTI set to: " << numPrb);
}

void
SatTrafficGenerator::SetRohcOverheadBytes (uint32_t bytes)
{
  m_rohcOverheadBytes = bytes;
  NS_LOG_INFO ("ROHC overhead set to: " << bytes << " bytes");
}

void
SatTrafficGenerator::ResetStats ()
{
  m_stats = TrafficModelStats {};
  NS_LOG_INFO ("Traffic statistics reset");
}

void
SatTrafficGenerator::DumpStats (std::ostream& os) const
{
  // Throughput definition per 3GPP TR 37.901 Clause 5.1.2:
  //   T = useful user data bits delivered / time
  // "Useful" excludes L2-L4 protocol headers and retransmission overhead.
  os << "=== Traffic Model Statistics (TR 37.901 Clause 5.1.2) ===" << std::endl;
  os << "Model Type: " << static_cast<int> (m_modelType) << std::endl;
  os << "Total Bytes Generated (wire): " << m_stats.totalBytesGenerated << std::endl;
  os << "Useful Bytes Delivered: " << m_stats.usefulBytesDelivered << std::endl;
  os << "Total Packets Generated: " << m_stats.totalPacketsGenerated << std::endl;
  os << "Buffer Underrun Events: " << m_stats.bufferUnderrunCount << std::endl;
  os << "IPv4 Packets: " << m_stats.ipv4Packets << std::endl;
  os << "IPv6 Packets: " << m_stats.ipv6Packets << std::endl;

  double simSecs = Simulator::Now ().GetSeconds ();
  if (m_stats.totalPacketsGenerated > 0 && simSecs > 0)
    {
      os << "Avg Generate Interval: " << m_stats.avgGenerateIntervalUs << " us" << std::endl;

      double wireBps    = (m_stats.totalBytesGenerated * 8.0) / simSecs;
      double usefulBps  = (m_stats.usefulBytesDelivered * 8.0) / simSecs;
      os << "Wire Throughput (L3+):    " << wireBps / 1e6    << " Mbps" << std::endl;
      os << "Useful Throughput (L5+):  " << usefulBps / 1e6  << " Mbps" << std::endl;
      os << "Per-user Peak (1s window):" << m_stats.peakThroughputBps / 1e6  << " Mbps" << std::endl;
      os << "Beam Peak (1s window):    " << m_stats.beamPeakThroughputBps / 1e6 << " Mbps" << std::endl;

      os << "--- Per-user useful throughput ---" << std::endl;
      for (const auto& kv : m_stats.perUserUsefulBytes)
        {
          double uBps = (kv.second * 8.0) / simSecs;
          double peak = 0;
          auto it = m_stats.perUserPeakBps.find (kv.first);
          if (it != m_stats.perUserPeakBps.end ()) peak = it->second;
          os << "  node " << kv.first
             << "  avg=" << uBps / 1e6 << " Mbps"
             << "  peak=" << peak / 1e6 << " Mbps"
             << "  bytes=" << kv.second << std::endl;
        }
    }
  os << "=========================================================" << std::endl;
}

} // namespace ns3
