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
  // 任务3:可选地启用 HTTP 会话/页面到达 Poisson(默认 0=关闭,不影响既有断言)。
  double httpArrivalRate = 0.0;
  CommandLine cmd (__FILE__);
  cmd.AddValue ("mode", "Validation mode: voip | ftp | http", mode);
  cmd.AddValue ("httpArrivalRate",
                "HTTP session/page Poisson arrival rate (sessions/s); 0 = disabled",
                httpArrivalRate);
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

      // VoIP 是双向会话业务: InstallVoipRtp 装 Leg A (node0->node1) 与 Leg B
      // (node1->node0) 两条腿 (带 VAD 轮流通话), 因此两端都会收到对向语音。
      // per-type 的 usefulBytesDelivered 是该业务两个方向的交付总和, 应当等于
      // 两端各自收到字节之和, 而非单端收到字节。
      const uint32_t node1Id = topo.nodes.Get (1)->GetId ();
      const uint32_t node0Id = topo.nodes.Get (0)->GetId ();
      const uint64_t rxBytesNode1 = topo.tg->GetNodeReceivedBytes (node1Id, TRAFFIC_VOIP_RTP);
      const uint64_t rxBytesNode0 = topo.tg->GetNodeReceivedBytes (node0Id, TRAFFIC_VOIP_RTP);
      const uint64_t rxBytesBothLegs = rxBytesNode1 + rxBytesNode0;
      const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_VOIP_RTP);
      ExpectTrue (ctx, rxBytesNode1 > 0, "VoIP sink receives bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == rxBytesBothLegs,
                  "VoIP per-type delivered stats equal received bytes across both call legs");
      ExpectTrue (ctx,
                  topo.tg->GetStats ().usefulBytesDelivered >= rxBytesBothLegs,
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
      ExpectTrue (ctx, ftpApps.GetN () == 2, "FTP install also adds ARP seed ping app");
      ExpectTrue (ctx, rxBytes > 0, "FTP sink receives bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == rxBytes,
                  "FTP per-type delivered stats equal received bytes");
      ExpectTrue (ctx,
                  typeStats.totalBytesGenerated >= rxBytes,
                  "FTP generated bytes cover delivered bytes");
      ExpectTrue (ctx, ftpSndBuf >= 1000000, "FTP sender TCP buffer is GEO-optimized");
      // Hybla 是逐 socket 经 SetCongestionControlAlgorithm 安装(不是节点 TcpL4Protocol::SocketType 默认),
      // 故验证生成器配置的拥塞算法——它由 ApplyTcpCongestionType 装到每条 FTP socket 上。
      ExpectTrue (ctx,
                  topo.tg->GetTcpCongestionTypeId () == TcpHybla::GetTypeId (),
                  "FTP sender uses GEO-oriented TCP Hybla");
    }
  else if (mode == "http")
    {
      BasicTopology topo = CreateBasicTopology (2);
      topo.tg = CreateObject<SatTrafficGenerator> ();
      topo.tg->SetApplicationWindow (Seconds (1.0), Seconds (5.0));
      // 任务3:可选启用独立的 HTTP 会话/页面到达 Poisson(须在 Install 前设置)。
      topo.tg->SetHttpSessionArrivalRate (httpArrivalRate);
      topo.tg->InstallSink (topo.nodes);
      topo.tg->InstallHttpDownload (NodeContainer (topo.nodes.Get (1)),
                                    topo.interfaces.GetAddress (0));

      Simulator::Stop (Seconds (6.0));
      Simulator::Run ();

      if (httpArrivalRate > 0.0)
        {
          // 启用时:验证 Poisson 会话到达过程确实触发了独立的新会话(>0 次)。
          const uint64_t httpArrivals = topo.tg->GetHttpSessionArrivals ();
          std::cout << "[INFO] HTTP session/page Poisson arrivals = " << httpArrivals
                    << " (rate=" << httpArrivalRate << "/s)" << std::endl;
          ExpectTrue (ctx, httpArrivals > 0,
                      "HTTP session/page Poisson arrivals fire when enabled");
        }

      const uint32_t nodeId = topo.nodes.Get (0)->GetId ();
      const uint64_t rxBytes = topo.tg->GetNodeReceivedBytes (nodeId, TRAFFIC_HTTP);
      const TrafficModelStats typeStats = topo.tg->GetStats (TRAFFIC_HTTP);
      ExpectTrue (ctx, rxBytes > 0, "HTTP client receives bytes");
      ExpectTrue (ctx,
                  typeStats.usefulBytesDelivered == rxBytes,
                  "HTTP per-type delivered stats equal received bytes");
      ExpectTrue (ctx,
                  typeStats.totalBytesGenerated >= rxBytes,
                  "HTTP generated bytes cover delivered bytes");
      // 同 FTP: 验证生成器配置的逐 socket 拥塞算法(ApplyTcpCongestionType 装到 HTTP client socket)。
      ExpectTrue (ctx,
                  topo.tg->GetTcpCongestionTypeId () == TcpHybla::GetTypeId (),
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
