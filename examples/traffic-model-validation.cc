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

      const uint32_t nodeId = topo.nodes.Get (1)->GetId ();
      const uint64_t rxBytes = topo.tg->GetNodeReceivedBytes (nodeId, TRAFFIC_VOIP_RTP);
      const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_VOIP_RTP);
      ExpectTrue (ctx, rxBytes > 0, "VoIP sink receives bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == rxBytes,
                  "VoIP per-type delivered stats equal received bytes");
      ExpectTrue (ctx,
                  topo.tg->GetStats ().usefulBytesDelivered >= rxBytes,
                  "VoIP contributes to aggregate delivered stats");
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
      ExpectTrue (ctx, rxBytes > 0, "HTTP client receives bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == rxBytes,
                  "HTTP per-type delivered stats equal received bytes");
      ExpectTrue (ctx,
                  typeStats.totalBytesGenerated >= rxBytes,
                  "HTTP generated bytes cover delivered bytes");
      ExpectTrue (ctx,
                  clientTcpType.Get () == TcpHybla::GetTypeId (),
                  "HTTP client uses GEO-oriented TCP Hybla");
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
