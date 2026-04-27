/*
 * traffic-models.cc
 *
 * GEO traffic helper that keeps the original geo-sat API surface while
 * wrapping reusable ns-3 HTTP / FTP / VoIP applications.
 */

#include "traffic-models.h"

#include "terminal-profile.h"

#include "ns3/address.h"
#include "ns3/bulk-send-helper.h"
#include "ns3/application.h"
#include "ns3/attribute.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/ping-helper.h"
#include "ns3/ping.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/tcp-hybla.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/three-gpp-http-client.h"
#include "ns3/three-gpp-http-helper.h"
#include "ns3/three-gpp-http-server.h"
#include "ns3/three-gpp-http-variables.h"
#include "ns3/traffic-generator.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <algorithm>

namespace ns3 {

namespace {

std::string
MakeTraceContext (uint32_t nodeId, TrafficModelType type)
{
  return std::to_string (nodeId) + ":" + std::to_string (static_cast<int> (type));
}

bool
ParseTraceContext (const std::string& context, uint32_t& nodeId, TrafficModelType& type)
{
  std::size_t separator = context.find (':');
  if (separator == std::string::npos)
    {
      return false;
    }

  nodeId = static_cast<uint32_t> (std::stoul (context.substr (0, separator)));
  type = static_cast<TrafficModelType> (std::stoi (context.substr (separator + 1)));
  return true;
}

void
ForwardTypedPacketTx (SatTrafficGenerator* generator,
                      TrafficModelType type,
                      uint32_t nodeId,
                      Ptr<const Packet> packet)
{
  generator->OnTypedPacketTx (type, nodeId, packet);
}

} // namespace

NS_LOG_COMPONENT_DEFINE ("SatTrafficGenerator");
NS_OBJECT_ENSURE_REGISTERED (SatTrafficGenerator);

TypeId
SatTrafficGenerator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatTrafficGenerator")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<SatTrafficGenerator> ()
    .AddAttribute ("TargetRate",
                   "Target data rate for Full Buffer mode (bits/s)",
                   UintegerValue (10000000),
                   MakeUintegerAccessor (&SatTrafficGenerator::m_targetRateBps),
                   MakeUintegerChecker<uint64_t> (1000, 1000000000))
    .AddAttribute ("GeoRtt",
                   "GEO satellite round-trip time",
                   TimeValue (MilliSeconds (630)),
                   MakeTimeAccessor (&SatTrafficGenerator::m_geoRtt),
                   MakeTimeChecker ())
    .AddAttribute ("MaxPrbPerTti",
                   "Maximum PRBs per TTI for payload calculation",
                   UintegerValue (25),
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
                   MakeBooleanChecker ());
  return tid;
}

SatTrafficGenerator::SatTrafficGenerator ()
  : m_tcpCongestionTypeId (TcpHybla::GetTypeId ())
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
          flow->active = false;
        }
    }
}

TrafficModelType
SatTrafficGenerator::GetTrafficModel () const
{
  if (m_installedTypes.size () == 1)
    {
      return *m_installedTypes.begin ();
    }

  // Mixed traffic profiles no longer have a single authoritative model type.
  return m_modelType;
}

std::vector<TrafficModelType>
SatTrafficGenerator::GetInstalledTrafficModels () const
{
  return std::vector<TrafficModelType> (m_installedTypes.begin (), m_installedTypes.end ());
}

std::string
SatTrafficGenerator::DescribeInstalledTrafficModels () const
{
  if (m_installedTypes.empty ())
    {
      return "none";
    }

  std::string description;
  for (auto it = m_installedTypes.begin (); it != m_installedTypes.end (); ++it)
    {
      if (!description.empty ())
        {
          description += "+";
        }
      description += TrafficModelTypeToString (*it);
    }
  return description;
}

uint32_t
SatTrafficGenerator::CalculateMaxPayloadBytesPerTti () const
{
  const uint32_t overheadPerPacket = m_rohcOverheadBytes + 2 + 4 + 3 + 20;
  const uint32_t maxBitsPerTti = m_maxPrbPerTti * 12 * 14 * 6;
  const uint32_t maxBytesPerTti = maxBitsPerTti / 8;
  return std::max (1200U, maxBytesPerTti - overheadPerPacket);
}

uint32_t
SatTrafficGenerator::CalculateTcpWindowBytes () const
{
  const double bandwidthBps = m_maxPrbPerTti * 12 * 14 * 6 * 1000.0;
  const double bdpBytes = bandwidthBps * m_geoRtt.GetSeconds () / 8.0;
  const uint32_t windowBytes = static_cast<uint32_t> (bdpBytes * 3.0);
  return std::max (windowBytes, 1048576U);
}

ApplicationContainer
InstallArpSeedPings (NodeContainer sourceNodes, Ipv4Address sinkAddress, Time appStartTime)
{
  PingHelper ping (sinkAddress);
  ping.SetAttribute ("Count", UintegerValue (8));
  ping.SetAttribute ("Interval", TimeValue (MilliSeconds (250)));
  ping.SetAttribute ("VerboseMode", EnumValue (Ping::VerboseMode::SILENT));
  ApplicationContainer pingApps = ping.Install (sourceNodes);
  Time pingStart = appStartTime > Seconds (2.0) ? appStartTime - Seconds (2.0) : Seconds (0.1);
  Time pingStop = appStartTime > MilliSeconds (100) ? appStartTime - MilliSeconds (100)
                                                    : pingStart + Seconds (1.0);
  if (pingStop <= pingStart)
    {
      pingStop = pingStart + Seconds (1.0);
    }
  pingApps.Start (pingStart);
  pingApps.Stop (pingStop);
  return pingApps;
}

ApplicationContainer
SatTrafficGenerator::InstallFullBuffer (NodeContainer sourceNodes,
                                        Ipv4Address sinkAddress,
                                        bool isUplink)
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);

  m_modelType = TRAFFIC_FULL_BUFFER;
  MarkTrafficModelInstalled (TRAFFIC_FULL_BUFFER);

  ApplicationContainer apps;
  apps.Add (InstallArpSeedPings (sourceNodes, sinkAddress, m_appStartTime));
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);

      Ptr<Socket> udpSocket = Socket::CreateSocket (node, UdpSocketFactory::GetTypeId ());
      udpSocket->SetAllowBroadcast (false);
      udpSocket->Connect (InetSocketAddress (sinkAddress, LEGACY_PORT));

      const uint32_t packetSize = CalculateMaxPayloadBytesPerTti ();
      const double intervalSeconds =
          (static_cast<double> (packetSize) * 8.0) / static_cast<double> (m_targetRateBps);
      const Time baseInterval = Seconds (std::max (0.001, intervalSeconds));

      Ptr<Application> app = CreateObject<Application> ();
      node->AddApplication (app);
      app->SetStartTime (m_appStartTime);
      app->SetStopTime (m_appStopTime);

      auto flow = std::make_shared<FlowState> ();
      flow->socket = udpSocket;
      flow->type = TRAFFIC_FULL_BUFFER;
      flow->nodeId = node->GetId ();
      flow->packetSize = packetSize;
      flow->baseInterval = baseInterval;

      std::weak_ptr<FlowState> weak = flow;
      flow->driver = [this, weak] () {
        auto self = weak.lock ();
        if (!self || !self->active)
          {
            return;
          }

        if (Simulator::Now () >= m_appStopTime)
          {
            self->active = false;
            return;
          }

        Ptr<Packet> pkt = Create<Packet> (self->packetSize);
        int sent = self->socket->Send (pkt);
        if (sent > 0)
          {
            AccountTx (*self, static_cast<uint32_t> (sent), static_cast<uint32_t> (sent));
          }
        else
          {
            m_totalStats.sendBlockedCount++;
            m_statsByType[TRAFFIC_FULL_BUFFER].sendBlockedCount++;
          }

        const Time nextInterval = ScaledInterval (self->baseInterval);
        if (Simulator::Now () + nextInterval < m_appStopTime)
          {
            Simulator::Schedule (nextInterval, self->driver);
          }
        else
          {
            self->active = false;
          }
      };

      m_flows.push_back (flow);
      Simulator::Schedule (m_appStartTime, flow->driver);
      apps.Add (app);
    }

  return apps;
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
  apps.Add (InstallArpSeedPings (sourceNodes, sinkAddress, m_appStartTime));
  const uint32_t tcpWindowBytes = CalculateTcpWindowBytes ();
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (tcpWindowBytes));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (tcpWindowBytes));
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);
      ApplyTcpCongestionType (node);
      BulkSendHelper bulkHelper ("ns3::TcpSocketFactory",
                                 InetSocketAddress (sinkAddress, FTP_PORT));
      bulkHelper.SetAttribute ("SendSize", UintegerValue (m_mtu - TCP_HEADER_BYTES));
      bulkHelper.SetAttribute ("MaxBytes",
                               UintegerValue (isUplink ? 100000000U : 2000000000U));

      ApplicationContainer bulkApps = bulkHelper.Install (node);
      bulkApps.Start (m_appStartTime);
      bulkApps.Stop (m_appStopTime);

      Ptr<Application> app = bulkApps.Get (0);
      if (app)
        {
          const uint32_t nodeId = node->GetId ();
          app->TraceConnectWithoutContext ("Tx",
                                           MakeBoundCallback (&ForwardTypedPacketTx,
                                                              this,
                                                              TRAFFIC_FTP,
                                                              nodeId));
        }
      apps.Add (bulkApps);
    }

  return apps;
}

ApplicationContainer
SatTrafficGenerator::InstallHttp (NodeContainer sourceNodes,
                                  Ipv4Address sinkAddress,
                                  bool isUplink)
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);

  m_modelType = TRAFFIC_HTTP;
  MarkTrafficModelInstalled (TRAFFIC_HTTP);

  if (isUplink)
    {
      NS_LOG_WARN ("InstallHttp() does not model HTTP upload semantics explicitly; "
                   "treating this call as a download-style content fetch toward "
                   << sinkAddress << ". Prefer InstallHttpDownload() in new code.");
    }

  Ptr<Node> sinkNode = FindNodeByAddress (sinkAddress);
  if (sinkNode == nullptr)
    {
      NS_LOG_ERROR ("Unable to map sink address " << sinkAddress
                    << " to a node. Call InstallSink() after IP assignment first.");
      return ApplicationContainer ();
    }

  // For HTTP, sourceNodes host the content servers and the node owning
  // sinkAddress runs the HTTP client that fetches content from them.
  ApplicationContainer apps;
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> sourceNode = sourceNodes.Get (i);
      Ipv4Address localAddress = GetPrimaryIpv4Address (sourceNode);
      if (localAddress == Ipv4Address::GetAny ())
        {
          NS_LOG_WARN ("Skipping HTTP install on node " << sourceNode->GetId ()
                       << " because it has no IPv4 address.");
          continue;
        }

      Ptr<ThreeGppHttpServer> server = EnsureHttpServer (sourceNode, localAddress);
      if (server == nullptr)
        {
          continue;
        }

      ThreeGppHttpClientHelper clientHelper{Address (localAddress)};
      clientHelper.SetAttribute ("RemoteServerPort", UintegerValue (HTTP_PORT));
      ApplyTcpCongestionType (sinkNode);

      ApplicationContainer clientApps = clientHelper.Install (sinkNode);
      // HTTP in this model has the UE as TCP client and the remote host as server.
      // The critical return path is server -> UE (SYN-ACK / HTTP response), so
      // pre-warm the path from content server toward the UE, matching the setup
      // used by the upstream NR mixed-traffic examples.
      apps.Add (InstallArpSeedPings (NodeContainer (sourceNode), sinkAddress, m_appStartTime));
      clientApps.Start (m_appStartTime);
      clientApps.Stop (m_appStopTime);

      Ptr<ThreeGppHttpClient> client = clientApps.Get (0)->GetObject<ThreeGppHttpClient> ();
      if (client)
        {
          PointerValue variablesPtr;
          client->GetAttribute ("Variables", variablesPtr);
          ConfigureHttpVariables (variablesPtr.Get<ThreeGppHttpVariables> ());

          const uint32_t sinkNodeId = sinkNode->GetId ();
          std::string rxContext = MakeTraceContext (sinkNodeId, TRAFFIC_HTTP);
          client->TraceConnect ("RxMainObjectPacket",
                                rxContext,
                                MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
          client->TraceConnect ("RxEmbeddedObjectPacket",
                                rxContext,
                                MakeCallback (&SatTrafficGenerator::NotifyPacketRxNoAddress, this));
        }

      apps.Add (clientApps);
    }

  return apps;
}

ApplicationContainer
SatTrafficGenerator::InstallHttpDownload (NodeContainer contentServerNodes,
                                          Ipv4Address clientAddress)
{
  return InstallHttp (contentServerNodes, clientAddress, false);
}

ApplicationContainer
SatTrafficGenerator::InstallVoipRtp (NodeContainer sourceNodes,
                                     Ipv4Address sinkAddress,
                                     bool isUplink)
{
  NS_LOG_FUNCTION (this << sinkAddress << isUplink);

  m_modelType = TRAFFIC_VOIP_RTP;
  MarkTrafficModelInstalled (TRAFFIC_VOIP_RTP);

  static constexpr uint32_t kVoicePayloadBytes = 40;
  static constexpr uint32_t kRtpTimestampStep = 160;

  ApplicationContainer apps;
  apps.Add (InstallArpSeedPings (sourceNodes, sinkAddress, m_appStartTime));
  for (uint32_t i = 0; i < sourceNodes.GetN (); ++i)
    {
      Ptr<Node> node = sourceNodes.Get (i);
      Ptr<Socket> udpSocket = Socket::CreateSocket (node, UdpSocketFactory::GetTypeId ());
      udpSocket->Connect (InetSocketAddress (sinkAddress, VOIP_PORT));

      Ptr<Application> app = CreateObject<Application> ();
      node->AddApplication (app);
      app->SetStartTime (m_appStartTime);
      app->SetStopTime (m_appStopTime);

      auto flow = std::make_shared<FlowState> ();
      flow->socket = udpSocket;
      flow->type = TRAFFIC_VOIP_RTP;
      flow->nodeId = node->GetId ();
      flow->packetSize = kVoicePayloadBytes;
      flow->baseInterval = MilliSeconds (20);

      std::weak_ptr<FlowState> weak = flow;
      flow->driver = [this, weak] () {
        auto self = weak.lock ();
        if (!self || !self->active)
          {
            return;
          }

        if (Simulator::Now () >= m_appStopTime)
          {
            self->active = false;
            return;
          }

        Ptr<Packet> pkt = Create<Packet> (self->packetSize);
        int sent = self->socket->Send (pkt);
        if (sent > 0)
          {
            AccountTx (*self, static_cast<uint32_t> (sent), static_cast<uint32_t> (sent));
          }
        else
          {
            m_totalStats.sendBlockedCount++;
            m_statsByType[TRAFFIC_VOIP_RTP].sendBlockedCount++;
          }

        self->rtpSeq++;
        self->rtpTimestamp += kRtpTimestampStep;
        if (Simulator::Now () + self->baseInterval < m_appStopTime)
          {
            Simulator::Schedule (self->baseInterval, self->driver);
          }
        else
          {
            self->active = false;
          }
      };

      m_flows.push_back (flow);
      Simulator::Schedule (m_appStartTime, flow->driver);
      apps.Add (app);
    }

  return apps;
}

ApplicationContainer
SatTrafficGenerator::InstallLegacyConsumerData (NodeContainer sourceNodes,
                                                Ipv4Address sinkAddress,
                                                bool isUplink)
{
  return InstallHttpDownload (sourceNodes, sinkAddress);
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
      InstallHttpDownload (sourceNodes, terminalAddress);
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
  return InstallHttpDownload (sourceNodes, sinkAddress);
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
      InstallHttpDownload (nodeCont, destAddress);
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
  const Time sinkStopTime = std::max (m_appStopTime + Seconds (1.0), Seconds (100.0));

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
          fullBuffer->TraceConnect ("Rx",
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
          ftp->TraceConnect ("Rx",
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
          voip->TraceConnect ("Rx",
                              MakeTraceContext (nodeId, TRAFFIC_VOIP_RTP),
                              MakeCallback (&SatTrafficGenerator::NotifyPacketRx, this));
        }
      apps.Add (voipApps);
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
SatTrafficGenerator::EnsureHttpServer (Ptr<Node> node, Ipv4Address localAddress)
{
  ApplyTcpCongestionType (node);

  auto existing = m_httpServersByNodeId.find (node->GetId ());
  if (existing != m_httpServersByNodeId.end ())
    {
      return existing->second;
    }

  ThreeGppHttpServerHelper serverHelper{Address (localAddress)};
  serverHelper.SetAttribute ("LocalPort", UintegerValue (HTTP_PORT));
  ApplicationContainer serverApps = serverHelper.Install (node);
  // HTTP client and server used to start at exactly the same time. In system
  // scenarios the client could attempt Connect() before the server had bound
  // its listening socket, leading to a silent connection failure and zero HTTP
  // traffic. Start the server slightly earlier to make the handshake reliable.
  const Time serverStart =
      (m_appStartTime > MilliSeconds (100)) ? (m_appStartTime - MilliSeconds (100))
                                            : Seconds (0.0);
  serverApps.Start (serverStart);
  serverApps.Stop (m_appStopTime);

  Ptr<ThreeGppHttpServer> server = serverApps.Get (0)->GetObject<ThreeGppHttpServer> ();
  if (!server)
    {
      NS_LOG_ERROR ("Failed to create HTTP server on node " << node->GetId ());
      return nullptr;
    }

  PointerValue variablesPtr;
  server->GetAttribute ("Variables", variablesPtr);
  ConfigureHttpVariables (variablesPtr.Get<ThreeGppHttpVariables> ());

  server->TraceConnect ("Tx",
                        MakeTraceContext (node->GetId (), TRAFFIC_HTTP),
                        MakeCallback (&SatTrafficGenerator::NotifyPacketTx, this));

  m_httpServersByNodeId[node->GetId ()] = server;
  return server;
}

void
SatTrafficGenerator::ConfigureHttpVariables (Ptr<ThreeGppHttpVariables> variables) const
{
  if (!variables)
    {
      return;
    }

  variables->SetMainObjectSizeMean (10710);
  variables->SetMainObjectSizeStdDev (25032);
  variables->SetEmbeddedObjectSizeMean (7758);
  variables->SetEmbeddedObjectSizeStdDev (126168);
  variables->SetNumOfEmbeddedObjectsMax (55);
  variables->SetNumOfEmbeddedObjectsScale (2);
  variables->SetNumOfEmbeddedObjectsShape (1.1);
  variables->SetReadingTimeMean (Seconds (30));
  variables->SetParsingTimeMean (Seconds (0.13));
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

void
SatTrafficGenerator::RecordReceivedPacket (uint32_t nodeId,
                                           TrafficModelType type,
                                           Ptr<const Packet> packet)
{
  if (!packet)
    {
      return;
    }

  m_rxBytesByType[type][nodeId] += packet->GetSize ();
  m_totalStats.usefulBytesDelivered += packet->GetSize ();
  m_totalStats.perUserUsefulBytes[nodeId] += packet->GetSize ();
  TrafficModelStats& typeStats = m_statsByType[type];
  typeStats.usefulBytesDelivered += packet->GetSize ();
  typeStats.perUserUsefulBytes[nodeId] += packet->GetSize ();
}

void
SatTrafficGenerator::NotifyPacketRx (std::string context,
                                     Ptr<const Packet> packet,
                                     const Address& from)
{
  uint32_t nodeId = 0;
  TrafficModelType type = TRAFFIC_FULL_BUFFER;
  if (ParseTraceContext (context, nodeId, type))
    {
      RecordReceivedPacket (nodeId, type, packet);
    }
  (void) from;
}

void
SatTrafficGenerator::NotifyPacketRxNoAddress (std::string context, Ptr<const Packet> packet)
{
  uint32_t nodeId = 0;
  TrafficModelType type = TRAFFIC_FULL_BUFFER;
  if (ParseTraceContext (context, nodeId, type))
    {
      RecordReceivedPacket (nodeId, type, packet);
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
  stats.totalBytesGenerated += bytesSentOnWire;
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
          stats.beamPeakThroughputBps = std::max (stats.beamPeakThroughputBps, bps);
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
  m_modelType = type;
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
SatTrafficGenerator::SetApplicationWindow (Time start, Time stop)
{
  m_appStartTime = start;
  m_appStopTime = stop;
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

void
SatTrafficGenerator::ResetStats ()
{
  m_totalStats = TrafficModelStats {};
  m_statsByType.clear ();
  m_installedTypes.clear ();
  m_rxBytesByType.clear ();
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
  os << "Total Bytes Generated: " << m_totalStats.totalBytesGenerated << std::endl;
  os << "Useful Bytes Delivered: " << m_totalStats.usefulBytesDelivered << std::endl;
  os << "Total Packets Generated: " << m_totalStats.totalPacketsGenerated << std::endl;
  os << "Send Blocked Events: " << m_totalStats.sendBlockedCount << std::endl;

  double simSecs = Simulator::Now ().GetSeconds ();
  if (simSecs > 0.0)
    {
      os << "Wire Throughput (L3+): "
         << (m_totalStats.totalBytesGenerated * 8.0) / simSecs / 1e6 << " Mbps" << std::endl;
      os << "Useful Throughput (L5+): "
         << (m_totalStats.usefulBytesDelivered * 8.0) / simSecs / 1e6 << " Mbps" << std::endl;
      os << "Per-user Peak (1s window): " << m_totalStats.peakThroughputBps / 1e6 << " Mbps"
         << std::endl;
      os << "Beam Peak (1s window): " << m_totalStats.beamPeakThroughputBps / 1e6 << " Mbps"
         << std::endl;
    }
  for (const auto& [type, stats] : m_statsByType)
    {
      os << "Type " << TrafficModelTypeToString (type) << " (" << static_cast<int> (type) << ")"
         << " delivered=" << stats.usefulBytesDelivered
         << " bytes, generated=" << stats.totalBytesGenerated << " bytes" << std::endl;
    }
  os << "================================" << std::endl;
}

void
SatTrafficGenerator::MarkTrafficModelInstalled (TrafficModelType type)
{
  m_installedTypes.insert (type);
}

void
SatTrafficGenerator::ApplyTcpCongestionType (Ptr<Node> node) const
{
  if (!node)
    {
      return;
    }

  Ptr<TcpL4Protocol> tcp = node->GetObject<TcpL4Protocol> ();
  if (!tcp)
    {
      return;
    }

  tcp->SetAttribute ("SocketType", TypeIdValue (m_tcpCongestionTypeId));
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
      return "LEGACY_DATA";
    case TRAFFIC_LEGACY_VOICE:
      return "LEGACY_VOICE";
    default:
      return "UNKNOWN";
    }
}

} // namespace ns3
