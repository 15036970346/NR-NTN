// http-live-validate.cc
// 端到端验证: 跑真实的 ThreeGppHttpClient/Server, 用 trace 抓应用"实际生成"的 5 个量,
// 落盘成与 http-dist-sample.cc 完全相同的格式, 复用 plot_http_dist.py 与理论曲线比对。
//
// 与 http-dist-sample.cc 的区别:
//   - sample 版直接调 ThreeGppHttpVariables::Get*(), 只验证"分布库+参数"。
//   - 本版让真实 client/server 在仿真里跑, 从 trace 抓实际产生的对象大小/内嵌数/think time,
//     验证"我的 HTTP 仿真产生的流量确实服从这三种分布"。
//
// 5 个量的来源:
//   主对象大小   ← server "MainObject" trace
//   内嵌对象大小 ← server "EmbeddedObject" trace
//   内嵌对象数   ← client "RxPage" trace 的 numObjects 实参(=该页内嵌对象数)
//   解析时间     ← client "StateTransition": 停留在 PARSING_MAIN_OBJECT 的时长
//   阅读时间     ← client "StateTransition": 停留在 READING 的时长
//
// 用法:
//   ./ns3 run "http-live-validate --numClients=200 --simTime=2000 --profile=dl-portable --out=http-live-samples.txt"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-http-client.h"
#include "ns3/three-gpp-http-helper.h"
#include "ns3/three-gpp-http-server.h"
#include "ns3/three-gpp-http-variables.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HttpLiveValidate");

// ---- 收集器 ----
static std::vector<double> g_mainSize;
static std::vector<double> g_embSize;
static std::vector<double> g_numEmb;
static std::vector<double> g_reading;
static std::vector<double> g_parsing;

// 每个 client 的 think-time 状态: 上次状态转移的时刻。
struct ThinkState
{
    Time lastTs{Seconds(0)};
};

static std::map<const void*, ThinkState> g_think;

static void
OnMainObject(uint32_t bytes)
{
    g_mainSize.push_back(static_cast<double>(bytes));
}

static void
OnEmbeddedObject(uint32_t bytes)
{
    g_embSize.push_back(static_cast<double>(bytes));
}

// RxPage(client, pageTime, numEmbedded, numBytes): 第 3 实参是该页内嵌对象数。
static void
OnRxPage(Ptr<const ThreeGppHttpClient>, const Time&, uint32_t numEmbedded, uint32_t)
{
    g_numEmb.push_back(static_cast<double>(numEmbedded));
}

// StateTransition(oldState, newState): oldState 的停留时长 = now - 上次转移时刻。
static void
OnStateTransition(const void* key, const std::string& oldState, const std::string&)
{
    const Time now = Simulator::Now();
    ThinkState& st = g_think[key];
    const double dur = (now - st.lastTs).GetSeconds();
    if (oldState == "PARSING_MAIN_OBJECT")
    {
        g_parsing.push_back(dur);
    }
    else if (oldState == "READING")
    {
        g_reading.push_back(dur);
    }
    st.lastTs = now;
}

// 与 http-dist-sample.cc 一致: 按 4 个画像配置 variables。
static void
ConfigureProfile(Ptr<ThreeGppHttpVariables> v, const std::string& profile)
{
    if (profile == "ul-light")
    {
        v->SetRequestSize(512);
        v->SetMainObjectSizeMean(12288);
        v->SetMainObjectSizeStdDev(6144);
        v->SetEmbeddedObjectSizeMean(2048);
        v->SetEmbeddedObjectSizeStdDev(6144);
        v->SetUsePoissonEmbeddedObjects(true);
        v->SetNumOfEmbeddedObjectsPoissonMean(2.0);
        v->SetNumOfEmbeddedObjectsMax(8);
        v->SetReadingTimeMean(Seconds(7));
        v->SetParsingTimeMean(Seconds(0.15));
    }
    else if (profile == "ul-portable")
    {
        v->SetRequestSize(512);
        v->SetMainObjectSizeMean(49152);
        v->SetMainObjectSizeStdDev(24576);
        v->SetEmbeddedObjectSizeMean(12288);
        v->SetEmbeddedObjectSizeStdDev(32768);
        v->SetUsePoissonEmbeddedObjects(true);
        v->SetNumOfEmbeddedObjectsPoissonMean(6.0);
        v->SetNumOfEmbeddedObjectsMax(24);
        v->SetReadingTimeMean(Seconds(10));
        v->SetParsingTimeMean(Seconds(0.2));
    }
    else if (profile == "dl-light")
    {
        v->SetMainObjectSizeMean(16384);
        v->SetMainObjectSizeStdDev(8192);
        v->SetEmbeddedObjectSizeMean(4096);
        v->SetEmbeddedObjectSizeStdDev(12288);
        v->SetUsePoissonEmbeddedObjects(true);
        v->SetNumOfEmbeddedObjectsPoissonMean(4.0);
        v->SetNumOfEmbeddedObjectsMax(16);
        v->SetReadingTimeMean(Seconds(5));
        v->SetParsingTimeMean(Seconds(0.1));
    }
    else // dl-portable
    {
        v->SetMainObjectSizeMean(65536);
        v->SetMainObjectSizeStdDev(32768);
        v->SetEmbeddedObjectSizeMean(16384);
        v->SetEmbeddedObjectSizeStdDev(49152);
        v->SetUsePoissonEmbeddedObjects(true);
        v->SetNumOfEmbeddedObjectsPoissonMean(10.0);
        v->SetNumOfEmbeddedObjectsMax(64);
        v->SetReadingTimeMean(Seconds(8));
        v->SetParsingTimeMean(Seconds(0.2));
    }
}

static void
MeanStdToMuSigma(double mean, double stddev, double& mu, double& sigma)
{
    const double a = std::log(1.0 + (stddev * stddev) / (mean * mean));
    mu = std::log(mean) - 0.5 * a;
    sigma = std::sqrt(a);
}

int
main(int argc, char* argv[])
{
    uint32_t numClients = 200;
    double simTime = 2000.0;
    std::string profile = "dl-portable";
    std::string out = "http-live-samples.txt";

    CommandLine cmd(__FILE__);
    cmd.AddValue("numClients", "并发 HTTP 客户端数(越多采样越快)", numClients);
    cmd.AddValue("simTime", "仿真时长(秒)", simTime);
    cmd.AddValue("profile", "dl-portable|dl-light|ul-portable|ul-light", profile);
    cmd.AddValue("out", "输出文件名", out);
    cmd.Parse(argc, argv);

    // ---- 拓扑: numClients 对独立的 server<->client 点对点链路。
    // 每条链路只承载自己那一对的流量 (O(N), 无 CSMA 的 O(N^2) 总线扇出)。
    // 对象大小/内嵌数/think time 的分布与链路无关, 所以这种最小拓扑足以验证"应用产生的分布"。
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("1ms"));

    InternetStackHelper internet;
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.255.252"); // 每对一个 /30 子网

    for (uint32_t i = 0; i < numClients; ++i)
    {
        Ptr<Node> sNode = CreateObject<Node>();
        Ptr<Node> cNode = CreateObject<Node>();
        internet.Install(sNode);
        internet.Install(cNode);

        NetDeviceContainer dev = p2p.Install(sNode, cNode);
        Ipv4InterfaceContainer ic = ipv4.Assign(dev);
        ipv4.NewNetwork();
        Ipv4Address sAddr = ic.GetAddress(0); // server 在本链路上的地址

        // server
        ThreeGppHttpServerHelper serverHelper(sAddr);
        ApplicationContainer serverApp = serverHelper.Install(sNode);
        Ptr<ThreeGppHttpServer> server = serverApp.Get(0)->GetObject<ThreeGppHttpServer>();
        {
            PointerValue pv;
            server->GetAttribute("Variables", pv);
            ConfigureProfile(pv.Get<ThreeGppHttpVariables>(), profile);
        }
        server->TraceConnectWithoutContext("MainObject", MakeCallback(&OnMainObject));
        server->TraceConnectWithoutContext("EmbeddedObject", MakeCallback(&OnEmbeddedObject));
        serverApp.Start(Seconds(0.5));
        serverApp.Stop(Seconds(simTime));

        // client
        ThreeGppHttpClientHelper clientHelper(sAddr);
        ApplicationContainer clientApp = clientHelper.Install(cNode);
        Ptr<ThreeGppHttpClient> client = clientApp.Get(0)->GetObject<ThreeGppHttpClient>();
        {
            PointerValue pv;
            client->GetAttribute("Variables", pv);
            ConfigureProfile(pv.Get<ThreeGppHttpVariables>(), profile);
        }
        client->TraceConnectWithoutContext("RxPage", MakeCallback(&OnRxPage));
        client->TraceConnectWithoutContext(
            "StateTransition",
            MakeBoundCallback(&OnStateTransition, static_cast<const void*>(PeekPointer(client))));
        clientApp.Start(Seconds(1.0));
        clientApp.Stop(Seconds(simTime));
    }

    Simulator::Stop(Seconds(simTime + 1.0));
    Simulator::Run();
    Simulator::Destroy();

    // ---- 还原理论参数(与 profile 一一对应), 写文件头 ----
    struct
    {
        double mainMean, mainStd, embeddedMean, embeddedStd, poissonMean, readMean, parseMean;
        uint32_t bound;
    } pp;
    if (profile == "ul-light")
        pp = {12288, 6144, 2048, 6144, 2.0, 7.0, 0.15, 8};
    else if (profile == "ul-portable")
        pp = {49152, 24576, 12288, 32768, 6.0, 10.0, 0.2, 24};
    else if (profile == "dl-light")
        pp = {16384, 8192, 4096, 12288, 4.0, 5.0, 0.1, 16};
    else
        pp = {65536, 32768, 16384, 49152, 10.0, 8.0, 0.2, 64};

    double mainMu, mainSigma, embMu, embSigma;
    MeanStdToMuSigma(pp.mainMean, pp.mainStd, mainMu, mainSigma);
    MeanStdToMuSigma(pp.embeddedMean, pp.embeddedStd, embMu, embSigma);

    // 写成与 http-dist-sample.cc 相同的宽表: 各列截到最短长度, 保证等长。
    const size_t minN = std::min({g_mainSize.size(),
                                  g_embSize.size(),
                                  g_numEmb.size(),
                                  g_reading.size(),
                                  g_parsing.size()});

    std::cout << "captured counts: main=" << g_mainSize.size() << " emb=" << g_embSize.size()
              << " numEmb=" << g_numEmb.size() << " reading=" << g_reading.size()
              << " parsing=" << g_parsing.size() << " -> using minN=" << minN << "\n";

    if (minN == 0)
    {
        std::cerr << "ERROR: 某个量没采到样本, 增大 --numClients 或 --simTime 再试。\n";
        return 1;
    }

    std::ofstream f(out);
    f << "# profile=" << profile << " n=" << minN << " (LIVE from real client/server traces)\n";
    f << "# main_object_size dist=lognormal mu=" << mainMu << " sigma=" << mainSigma
      << " min=100 max=2000000\n";
    f << "# embedded_object_size dist=lognormal mu=" << embMu << " sigma=" << embSigma
      << " min=50 max=2000000\n";
    f << "# num_embedded dist=poisson mean=" << pp.poissonMean << " bound=" << pp.bound << "\n";
    f << "# reading_time dist=exponential mean=" << pp.readMean << "\n";
    f << "# parsing_time dist=exponential mean=" << pp.parseMean << "\n";
    f << "main_object_size\tembedded_object_size\tnum_embedded\treading_time\tparsing_time\n";
    for (size_t i = 0; i < minN; ++i)
    {
        f << g_mainSize[i] << '\t' << g_embSize[i] << '\t' << g_numEmb[i] << '\t' << g_reading[i]
          << '\t' << g_parsing[i] << '\n';
    }
    f.close();

    std::cout << "wrote " << minN << " rows to " << out << " (profile=" << profile << ")\n";
    return 0;
}
