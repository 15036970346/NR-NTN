#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/bulk-send-application.h"
#include "ns3/tcp-hybla.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/uinteger.h"

#include "ns3/traffic-models.h"
#include "ns3/terminal-profile.h"

#include <iostream>

using namespace ns3;

namespace
{

struct CheckContext
{
  uint32_t pass {0};
  uint32_t fail {0};
};

uint32_t gQueueFeedbackCount = 0;
double gLastQueueFeedback = -1.0;

void
ExpectTrue (CheckContext& ctx, bool cond, const std::string& label)
{
  if (cond)
    {
      ++ctx.pass;
      std::cout << "[PASS] " << label << std::endl;
    }
  else
    {
      ++ctx.fail;
      std::cout << "[FAIL] " << label << std::endl;
  }
}

void
OnQueueFeedback (double utilization)
{
  ++gQueueFeedbackCount;
  gLastQueueFeedback = utilization;
}

struct BasicTopology
{
  NodeContainer nodes;
  Ipv4InterfaceContainer interfaces;
  Ptr<SatTrafficGenerator> tg;
};

BasicTopology
CreateBasicTopology (uint32_t subnetIndex)
{
  BasicTopology topo;
  topo.nodes.Create (2);

  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("1ms"));

  InternetStackHelper internet;
  internet.Install (topo.nodes);

  NetDeviceContainer devices = p2p.Install (topo.nodes);
  Ipv4AddressHelper ipv4;
  std::ostringstream network;
  network << "10." << (subnetIndex + 1) << ".1.0";
  ipv4.SetBase (network.str ().c_str (), "255.255.255.0");
  topo.interfaces = ipv4.Assign (devices);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  return topo;
}

} // namespace

int
main (int argc, char* argv[])
{
  CheckContext ctx;
  std::string mode = "voip";
  CommandLine cmd (__FILE__);
  cmd.AddValue ("mode", "Validation mode: voip | ftp | http", mode);
  cmd.Parse (argc, argv);

  {
    Ptr<SatTrafficGenerator> metaTg = CreateObject<SatTrafficGenerator> ();
    metaTg->SetTrafficModel (TRAFFIC_HTTP);
    metaTg->SetTrafficModel (TRAFFIC_VOIP_RTP);
    const auto installedTypes = metaTg->GetInstalledTrafficModels ();
    const std::string description = metaTg->DescribeInstalledTrafficModels ();
    ExpectTrue (ctx, installedTypes.size () == 2, "Installed model set tracks mixed traffic types");
    ExpectTrue (ctx,
                description.find ("HTTP") != std::string::npos &&
                    description.find ("VOIP_RTP") != std::string::npos,
                "Installed model description includes both HTTP and VOIP");

    gQueueFeedbackCount = 0;
    gLastQueueFeedback = -1.0;
    metaTg->SetQueueStateCallback (MakeCallback (&OnQueueFeedback));
    metaTg->EnableClosedLoopControl (true, 80);
    metaTg->ReportQueueUtilization (92.5);
    ExpectTrue (ctx, gQueueFeedbackCount == 1, "Queue utilization callback fires");
    ExpectTrue (ctx, gLastQueueFeedback == 92.5, "Queue utilization callback keeps original value");
  }

  {
    BasicTopology topo = CreateBasicTopology (10);
    topo.tg = CreateObject<SatTrafficGenerator> ();
    topo.tg->SetApplicationWindow (Seconds (1.0), Seconds (3.0));
    topo.tg->InstallSink (topo.nodes);

    Ptr<SatTerminalProfile> profile = CreateObject<SatTerminalProfile> ();
    profile->SetTerminalType (UT_CONSUMER);
    profile->SetVoiceEnabled (true);
    profile->SetDataServiceType (SAT_DATA_HTTP);
    topo.nodes.Get (1)->AggregateObject (profile);

    InstalledTerminalTraffic installed =
        topo.tg->InstallProfileTraffic (NodeContainer (topo.nodes.Get (0)),
                                        topo.nodes.Get (1),
                                        topo.interfaces.GetAddress (1),
                                        false);

    Simulator::Stop (Seconds (4.0));
    Simulator::Run ();

    ExpectTrue (ctx, installed.isConsumerPhone, "Consumer profile is detected correctly");
    ExpectTrue (ctx, installed.hasVoip && installed.hasHttp && !installed.hasFtp,
                "Consumer profile installs VoIP + HTTP only");
    ExpectTrue (ctx,
                topo.tg->GetNodeReceivedBytes (topo.nodes.Get (1)->GetId (), TRAFFIC_VOIP_RTP) > 0,
                "Consumer UE receives downlink VoIP bytes");
    ExpectTrue (ctx,
                topo.tg->GetNodeReceivedBytes (topo.nodes.Get (0)->GetId (), TRAFFIC_VOIP_RTP) > 0,
                "Consumer profile also produces uplink VoIP bytes");
    const TrafficModelStats profileVoipStats = topo.tg->GetStats (TRAFFIC_VOIP_RTP);
    ExpectTrue (ctx,
                profileVoipStats.totalPacketsGenerated >= 70 &&
                    profileVoipStats.totalPacketsGenerated < 150,
                "Consumer profile installs bidirectional VoIP with turn-taking and VAD/DTX suppression");
    ExpectTrue (ctx,
                topo.tg->GetNodeReceivedBytes (topo.nodes.Get (1)->GetId (), TRAFFIC_HTTP) > 0,
                "Consumer UE receives HTTP bytes under the mixed profile");

    Simulator::Destroy ();
  }

  if (mode == "voip")
    {
      BasicTopology topo = CreateBasicTopology (0);
      topo.tg = CreateObject<SatTrafficGenerator> ();
      topo.tg->SetApplicationWindow (Seconds (1.0), Seconds (3.0));
      topo.tg->InstallSink (topo.nodes);
      topo.tg->InstallVoipRtp (NodeContainer (topo.nodes.Get (0)),
                               topo.interfaces.GetAddress (1),
                               false);

      Simulator::Stop (Seconds (4.0));
      Simulator::Run ();

      const uint32_t calleeNodeId = topo.nodes.Get (1)->GetId ();
      const uint32_t callerNodeId = topo.nodes.Get (0)->GetId ();
      const uint64_t rxBytesDown = topo.tg->GetNodeReceivedBytes (calleeNodeId, TRAFFIC_VOIP_RTP);
      const uint64_t rxBytesUp = topo.tg->GetNodeReceivedBytes (callerNodeId, TRAFFIC_VOIP_RTP);
      const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_VOIP_RTP);
      ExpectTrue (ctx, rxBytesDown > 0, "VoIP sink receives bytes");
      ExpectTrue (ctx,
                  rxBytesUp > 0,
                  "VoIP caller also receives reverse-direction bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == (rxBytesDown + rxBytesUp),
                  "VoIP per-type delivered stats equal bidirectional received bytes");
      ExpectTrue (ctx,
                  topo.tg->GetStats ().usefulBytesDelivered >= (rxBytesDown + rxBytesUp),
                  "VoIP contributes to aggregate delivered stats");
      ExpectTrue (ctx,
                  typeStats.totalPacketsGenerated >= 70 &&
                      typeStats.totalPacketsGenerated < 150,
                  "VoIP session alternates active speech and suppresses packets during silence");
      ExpectTrue (ctx,
                  typeStats.avgGenerateIntervalUs > 10000.0 &&
                      typeStats.avgGenerateIntervalUs < 80000.0,
                  "VoIP bidirectional session keeps a turn-taking speech-like aggregate packet cadence");
    }
  else if (mode == "ftp")
    {
      BasicTopology topo = CreateBasicTopology (1);
      topo.tg = CreateObject<SatTrafficGenerator> ();
      topo.tg->SetApplicationWindow (Seconds (1.0), Seconds (3.0));
      topo.tg->InstallSink (topo.nodes);
      ApplicationContainer ftpApps =
          topo.tg->InstallFtp (NodeContainer (topo.nodes.Get (0)), topo.interfaces.GetAddress (1), false);

      Ptr<BulkSendApplication> bulkApp = DynamicCast<BulkSendApplication> (ftpApps.Get (1));
      uint32_t ftpSndBuf = 0;
      Simulator::Schedule (Seconds (1.2), [&ftpSndBuf, bulkApp] {
        if (!bulkApp)
          {
            return;
          }
        Ptr<Socket> socket = bulkApp->GetSocket ();
        if (!socket)
          {
            return;
          }
        UintegerValue sndBufValue;
        socket->GetAttribute ("SndBufSize", sndBufValue);
        ftpSndBuf = sndBufValue.Get ();
      });

      Simulator::Stop (Seconds (4.0));
      Simulator::Run ();

      const uint32_t nodeId = topo.nodes.Get (1)->GetId ();
      const uint64_t rxBytes = topo.tg->GetNodeReceivedBytes (nodeId, TRAFFIC_FTP);
      const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_FTP);
      Ptr<TcpL4Protocol> senderTcp = topo.nodes.Get (0)->GetObject<TcpL4Protocol> ();
      TypeIdValue senderTcpType;
      senderTcp->GetAttribute ("SocketType", senderTcpType);
      ExpectTrue (ctx, ftpApps.GetN () == 2, "FTP install also adds ARP seed ping app");
      ExpectTrue (ctx, rxBytes > 0, "FTP sink receives bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == rxBytes,
                  "FTP per-type delivered stats equal received bytes");
      ExpectTrue (ctx,
                  typeStats.totalBytesGenerated >= rxBytes,
                  "FTP generated bytes cover delivered bytes");
      ExpectTrue (ctx, ftpSndBuf >= 1000000, "FTP sender TCP buffer is GEO-optimized");
      ExpectTrue (ctx,
                  senderTcpType.Get () == TcpHybla::GetTypeId (),
                  "FTP sender uses GEO-oriented TCP Hybla");
    }
  else if (mode == "http")
    {
      {
        BasicTopology topo = CreateBasicTopology (2);
        topo.tg = CreateObject<SatTrafficGenerator> ();
        topo.tg->SetApplicationWindow (Seconds (1.0), Seconds (5.0));
        topo.tg->InstallSink (topo.nodes);
        topo.tg->InstallHttpDownload (NodeContainer (topo.nodes.Get (1)),
                                      topo.interfaces.GetAddress (0));

        Simulator::Stop (Seconds (6.0));
        Simulator::Run ();

        const uint32_t nodeId = topo.nodes.Get (0)->GetId ();
        const uint64_t rxBytes = topo.tg->GetNodeReceivedBytes (nodeId, TRAFFIC_HTTP);
        const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_HTTP);
        Ptr<TcpL4Protocol> clientTcp = topo.nodes.Get (0)->GetObject<TcpL4Protocol> ();
        TypeIdValue clientTcpType;
        clientTcp->GetAttribute ("SocketType", clientTcpType);
        ExpectTrue (ctx, rxBytes > 0, "HTTP download client receives bytes");
        ExpectTrue (ctx,
                    typeStats.usefulBytesDelivered == rxBytes,
                    "HTTP download per-type delivered stats equal received bytes");
        ExpectTrue (ctx,
                    typeStats.totalBytesGenerated >= rxBytes,
                    "HTTP download generated bytes cover delivered bytes");
        ExpectTrue (ctx,
                    clientTcpType.Get () == TcpHybla::GetTypeId (),
                    "HTTP download client uses GEO-oriented TCP Hybla");

        Simulator::Destroy ();
      }

      {
        BasicTopology topo = CreateBasicTopology (3);
        topo.tg = CreateObject<SatTrafficGenerator> ();
        topo.tg->SetApplicationWindow (Seconds (1.0), Seconds (5.0));
        topo.tg->InstallSink (topo.nodes);
        topo.tg->InstallHttp (NodeContainer (topo.nodes.Get (0)),
                              topo.interfaces.GetAddress (1),
                              true);

        Simulator::Stop (Seconds (6.0));
        Simulator::Run ();

        const uint32_t remoteClientNodeId = topo.nodes.Get (0)->GetId ();
        const uint32_t ueServerNodeId = topo.nodes.Get (1)->GetId ();
        const uint64_t rxBytes = topo.tg->GetNodeReceivedBytes (remoteClientNodeId, TRAFFIC_HTTP);
        const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_HTTP);
        Ptr<TcpL4Protocol> clientTcp = topo.nodes.Get (0)->GetObject<TcpL4Protocol> ();
        TypeIdValue clientTcpType;
        clientTcp->GetAttribute ("SocketType", clientTcpType);
        ExpectTrue (ctx, rxBytes > 0, "HTTP upload requester receives uploaded object bytes");
        ExpectTrue (ctx,
                    topo.tg->GetNodeReceivedBytes (ueServerNodeId, TRAFFIC_HTTP) == 0,
                    "HTTP upload useful payload is not mis-attributed to the UE server");
        ExpectTrue (ctx,
                    typeStats.usefulBytesDelivered == rxBytes,
                    "HTTP upload per-type delivered stats equal remote received bytes");
        ExpectTrue (ctx,
                    typeStats.totalBytesGenerated >= rxBytes,
                    "HTTP upload generated bytes cover delivered bytes");
        ExpectTrue (ctx,
                    clientTcpType.Get () == TcpHybla::GetTypeId (),
                    "HTTP upload requester also uses GEO-oriented TCP Hybla");
      }
    }
  else
    {
      std::cerr << "Unknown mode: " << mode << std::endl;
      return 1;
    }

  Simulator::Destroy ();

  std::cout << "PASS=" << ctx.pass << std::endl;
  std::cout << "FAIL=" << ctx.fail << std::endl;
  std::cout << "OVERALL RESULT: " << (ctx.fail == 0 ? "PASS" : "FAIL") << std::endl;
  return ctx.fail == 0 ? 0 : 1;
}
