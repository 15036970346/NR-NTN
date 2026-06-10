// http-dist-sample.cc
// 对 ns-3 3GPP HTTP 模型用到的三类分布族采样，导出供 Python 画拟合曲线：
//   1) 主/内嵌对象大小 : LogNormal (截断) —— 连续
//   2) 内嵌对象数      : Poisson(截断)    —— 离散
//   3) 阅读/解析时间   : Exponential      —— 连续
//
// 直接调用真实的 ThreeGppHttpVariables::Get*() 接口，所以采样里已包含截断/重采样逻辑。
// 文件头同时写出理论分布参数，Python 端据此画解析曲线。
//
// 用法:
//   ./ns3 run "http-dist-sample --n=200000 --profile=dl-portable --out=http-dist-samples.txt"
//   profile ∈ {dl-portable, dl-light, ul-portable, ul-light}

#include "ns3/core-module.h"
#include "ns3/three-gpp-http-variables.h"

#include <cmath>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HttpDistSample");

// 按 SatTrafficGenerator::ConfigureHttpVariables 里的四个画像配置 variables。
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
    else // dl-portable (默认)
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

// 把用户设置的 (Mean, StdDev) 转成 LogNormal 底层正态参数 (mu, sigma)，
// 与 ThreeGppHttpVariables::UpdateMainObjectMuAndSigma 完全一致。
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
    uint32_t n = 200000;
    std::string profile = "dl-portable";
    std::string out = "http-dist-samples.txt";

    // 这些是上面各 profile 写死的均值/标准差，用于在文件头还原理论参数。
    // 简单起见这里只复算 dl-portable 一档的主对象大小参数；其它档由 CLI 透传也行，
    // 但为保证文件头与采样一致，下面统一从已知的 means 表里取。
    CommandLine cmd(__FILE__);
    cmd.AddValue("n", "采样次数", n);
    cmd.AddValue("profile", "画像 dl-portable|dl-light|ul-portable|ul-light", profile);
    cmd.AddValue("out", "输出文件名", out);
    cmd.Parse(argc, argv);

    Ptr<ThreeGppHttpVariables> v = CreateObject<ThreeGppHttpVariables>();
    ConfigureProfile(v, profile);
    v->Initialize();

    // 还原理论参数(与各 profile 配置一一对应)。
    struct
    {
        double mainMean, mainStd;
        double embeddedMean, embeddedStd;
        double poissonMean, readMean, parseMean;
        uint32_t bound;
    } p;
    if (profile == "ul-light")
        p = {12288, 6144, 2048, 6144, 2.0, 7.0, 0.15, 8};
    else if (profile == "ul-portable")
        p = {49152, 24576, 12288, 32768, 6.0, 10.0, 0.2, 24};
    else if (profile == "dl-light")
        p = {16384, 8192, 4096, 12288, 4.0, 5.0, 0.1, 16};
    else
        p = {65536, 32768, 16384, 49152, 10.0, 8.0, 0.2, 64};

    double mainMu, mainSigma;
    double embeddedMu, embeddedSigma;
    MeanStdToMuSigma(p.mainMean, p.mainStd, mainMu, mainSigma);
    MeanStdToMuSigma(p.embeddedMean, p.embeddedStd, embeddedMu, embeddedSigma);

    std::ofstream f(out);
    f << "# profile=" << profile << " n=" << n << "\n";
    f << "# main_object_size dist=lognormal mu=" << mainMu << " sigma=" << mainSigma
      << " min=100 max=2000000\n";
    f << "# embedded_object_size dist=lognormal mu=" << embeddedMu << " sigma=" << embeddedSigma
      << " min=50 max=2000000\n";
    f << "# num_embedded dist=poisson mean=" << p.poissonMean << " bound=" << p.bound << "\n";
    f << "# reading_time dist=exponential mean=" << p.readMean << "\n";
    f << "# parsing_time dist=exponential mean=" << p.parseMean << "\n";
    f << "main_object_size\tembedded_object_size\tnum_embedded\treading_time\tparsing_time\n";

    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t mainSize = v->GetMainObjectSize();
        uint32_t embeddedSize = v->GetEmbeddedObjectSize();
        uint32_t numEmb = v->GetNumOfEmbeddedObjects();
        double readT = v->GetReadingTime().GetSeconds();
        double parseT = v->GetParsingTime().GetSeconds();
        f << mainSize << '\t' << embeddedSize << '\t' << numEmb << '\t' << readT << '\t'
          << parseT << '\n';
    }
    f.close();

    std::cout << "wrote " << n << " samples to " << out << " (profile=" << profile << ")\n";
    return 0;
}
