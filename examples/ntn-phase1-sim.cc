#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include "ns3/applications-module.h"
#include "ns3/lte-module.h" 

#include "ns3/point-to-point-epc-helper.h" 
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/ideal-beamforming-algorithm.h"

// [整合] 引入你和你队友的自定义模块
#include "ns3/user-helper.h"
#include "ns3/sat-gw-mac.h"     
#include "ns3/traffic-models.h"
#include "ns3/sat-phy.h" 
#include "ns3/multi-model-spectrum-channel.h" 

// [新增测试] 引入队友新写的调度器与资源管理模块头文件
#include "ns3/resource-manager.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/admit-control.h"    // [新增] 准入控制模块

// [新增] NTN 配置助手 - 独立封装 RLC/PDCP 长时延适配
#include "ns3/ntn-config-helper.h"

// [新增] 谢昀松部分 - SatUtMac 终端MAC层
#include "ns3/sat-ut-mac.h"

// [新增] SatPdcp PDCP层 + ROHC集成
#include "ns3/sat-pdcp.h"

// [新增] 崔博开的测量模块
#include "ns3/sat-ut-phy.h"
#include "ns3/sat-ut-rrc.h"
#include "ns3/sat-mac-common.h"
#include "ns3/constant-position-mobility-model.h"

using namespace ns3;

// 测量报告回调函数
void ReceiveMeasReportCallback (MeasReport report)
{
    std::cout << "[RRC Callback] UE=" << report.ueId 
              << " Beam=" << report.bestBeamId 
              << " RSRP=" << report.rsrp 
              << " Pos=(" << report.position.x << "," << report.position.y << "," << report.position.z << ")" << std::endl;
}

// [新增] CQI报告回调
void CqiReportCallback (uint8_t cqi)
{
    std::cout << "[CQI Callback] Received CQI: " << (uint32_t)cqi << std::endl;
}

// [新增] 4 步 RA 完成回调
static uint32_t g_raSuccessCount = 0;
static uint32_t g_raFailedCount  = 0;
void RaCompleteCallback (std::string tag, SatUtMac::RaResult result)
{
    if (result == SatUtMac::RA_SUCCESS) {
        g_raSuccessCount++;
        std::cout << "[RA Complete] " << tag << " SUCCESS" << std::endl;
    } else {
        g_raFailedCount++;
        std::cout << "[RA Complete] " << tag << " FAILED (max attempts)" << std::endl;
    }
}

NS_LOG_COMPONENT_DEFINE ("NtnPhase1Sim");

int main (int argc, char *argv[])
{
    LogComponentEnable ("NtnPhase1Sim", LOG_LEVEL_INFO);
    LogComponentEnable ("SatUserHelper", LOG_LEVEL_INFO);
    //LogComponentEnable ("SatGwMac", LOG_LEVEL_INFO); 
    
    // [新增测试] 开启队友调度器和 RRM 模块的日志，方便观察打出的数据
    LogComponentEnable ("GeoBeamScheduler", LOG_LEVEL_INFO); 
    LogComponentEnable ("ResourceManager", LOG_LEVEL_INFO);
    LogComponentEnable ("AdmitControl", LOG_LEVEL_INFO);  // [新增] 准入控制日志
    
    // [新增] 开启 SatUtMac 和 SatPdcp 日志
    LogComponentEnable ("SatUtMac", LOG_LEVEL_INFO);
    LogComponentEnable ("SatPdcp", LOG_LEVEL_INFO);
    LogComponentEnable ("RohcCompressor", LOG_LEVEL_INFO);
    LogComponentEnable ("NtnConfigHelper", LOG_LEVEL_INFO);
    // SatGwMac使用DEBUG级别，只在每10次广播时打印一次
    // LogComponentEnable ("SatGwMac", LOG_LEVEL_INFO);
    LogComponentEnable ("SatUtPhy", LOG_LEVEL_INFO);

    // 使用独立的 NTN 配置助手
    NtnConfigHelper::ConfigureAll ();

    // 1. 基础物理环境
    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel> ();
    Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel> ();
    channel->AddPropagationLossModel (lossModel);
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
    channel->SetPropagationDelayModel (delayModel);

    // 2. 准备 Helper 与 EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
    nrHelper->SetEpcHelper (epcHelper); 

    Ptr<IdealBeamformingHelper> bfmHelper = CreateObject<IdealBeamformingHelper> ();
    bfmHelper->SetAttribute ("BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));
    nrHelper->SetBeamformingHelper (bfmHelper);

    // 3. 实例化队友的信关站 (GW) MAC 层
    NS_LOG_INFO ("Setting up Satellite Gateway MAC...");
    Ptr<SatGwMac> gwMac = CreateObject<SatGwMac> ();
    std::vector<uint32_t> beams = {1, 2, 3};
    gwMac->SetActiveBeams (beams);
    gwMac->StartPeriodicTransmissions ();

    // 4. 实例化你的用户 (UE) 及其 Helper
    NS_LOG_INFO ("Setting up User Terminals...");
    Ptr<SatUserHelper> userHelper = CreateObject<SatUserHelper> ();
    userHelper->SetNrHelper (nrHelper);
    userHelper->SetSatelliteChannel (channel); 
    userHelper->SetBeamId (1); 

    NodeContainer ues = userHelper->CreateUserNodes (2);
    userHelper->InstallMobility (ues, "", "", "");      
    NetDeviceContainer ueDevs = userHelper->InstallStack (ues); 
    userHelper->AssignIp (ueDevs, Ipv4Address ("10.0.1.0"), Ipv4Mask ("255.255.255.0"));

    // 5. 虚拟网关 (Sink)
    NodeContainer gwNode;
    gwNode.Create (1);
    InternetStackHelper internet;
    internet.Install (gwNode);
    Ipv4Address gwAddress ("10.0.2.1"); 

    // 6. 注入业务流量
    NS_LOG_INFO ("Generating Traffic...");
    Ptr<SatTrafficGenerator> trafficGen = CreateObject<SatTrafficGenerator> ();
    trafficGen->SetApplicationWindow (Seconds (1.0), Seconds (10.0));
    trafficGen->InstallSink (gwNode);
    trafficGen->InstallVoipRtp (NodeContainer (ues.Get (0)), gwAddress, true);
    trafficGen->InstallFtp (NodeContainer (ues.Get (1)), gwAddress, true);

    // ------------------------------------------------------------
    // [新增测试] 7. 验证队友的准入控制模块 (AdmitControl)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing AdmitControl (准入控制) ===");
    Ptr<AdmitControl> admitControl = CreateObject<AdmitControl> ();
    admitControl->SetBeamTotalRbs (1, 25);   // 波束1: 25 RB
    admitControl->SetBeamTotalRbs (2, 25);   // 波束2: 25 RB
    admitControl->SetBeamTotalRbs (3, 25);   // 波束3: 25 RB
    
    // 测试1: 应急业务接入判断
    NS_LOG_INFO ("Test AdmitControl: Emergency UE to Beam 1");
    AdmitDecision decision1 = admitControl->CanAdmitUe (1, ServicePriority::PRIORITY_EMERGENCY, 
                                                          UT_CONSUMER, TRAFFIC_DATA, 12);
    NS_LOG_INFO ("  Decision: " << (uint32_t)decision1);
    
    // 测试2: 普通数据接入判断
    NS_LOG_INFO ("Test AdmitControl: Normal DATA UE to Beam 1");
    AdmitDecision decision2 = admitControl->CanAdmitUe (1, ServicePriority::PRIORITY_DATA, 
                                                          UT_CONSUMER, TRAFFIC_DATA, 10);
    NS_LOG_INFO ("  Decision: " << (uint32_t)decision2);
    
    // 测试3: 切换准入判断
    NS_LOG_INFO ("Test AdmitControl: Handover from Beam 1 to Beam 2");
    admitControl->RegisterUeToBeam (1, 1, ServicePriority::PRIORITY_DATA, UT_CONSUMER, TRAFFIC_DATA, Vector (100, 100, 0));
    AdmitDecision decision3 = admitControl->CanHandoverUe (1, 2, ServicePriority::PRIORITY_DATA, 12);
    NS_LOG_INFO ("  Decision: " << (uint32_t)decision3);
    
    // 测试4: 获取推荐波束
    NS_LOG_INFO ("Test AdmitControl: Get Recommended Beams");
    std::vector<uint32_t> recommendedBeams = admitControl->GetRecommendedBeams (1, ServicePriority::PRIORITY_EMERGENCY);
    std::cout << "  Recommended beams: ";
    for (uint32_t beam : recommendedBeams) {
        std::cout << beam << " ";
    }
    std::cout << std::endl;
    
    // 测试5: 负载均衡检查
    admitControl->RegisterUeToBeam (2, 1, ServicePriority::PRIORITY_DATA, UT_CONSUMER, TRAFFIC_DATA, Vector (200, 200, 0));
    admitControl->RegisterUeToBeam (3, 1, ServicePriority::PRIORITY_DATA, UT_CONSUMER, TRAFFIC_DATA, Vector (300, 300, 0));
    admitControl->CheckBeamLoadBalancing ();
    
    // ------------------------------------------------------------
    // [新增测试] 8. 验证两级调度策略 (WRR + IPF)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing Two-Level Scheduling (WRR + IPF) ===");
    Ptr<GeoBeamScheduler> scheduler = CreateObject<GeoBeamScheduler> ();
    scheduler->Initialize (1, 1);
    scheduler->SetAdmitControl (admitControl);  // 连接准入控制

    // 模拟 UE 1：大容量业务 (消费级终端) + 优秀的信道质量 + 近波束中心
    uint16_t rnti1 = 1;
    scheduler->AddUeContext (rnti1, UT_CONSUMER, TRAFFIC_HIGH_CAPACITY);
    scheduler->AddUeInfo (rnti1, 1);
    scheduler->UpdateUeCsi (rnti1, 14.0);
    scheduler->UpdateUePosition (rnti1, Vector (10.0, 10.0, 0.0));  // 近中心

    // 模拟 UE 2：应急业务 (消费级终端) - WRR优先调度
    uint16_t rnti2 = 2;
    scheduler->AddUeContext (rnti2, UT_CONSUMER, TRAFFIC_VOICE);  // 语音 -> 高优先级
    scheduler->AddUeInfo (rnti2, 1);
    scheduler->UpdateUeCsi (rnti2, 10.0);
    scheduler->UpdateUePosition (rnti2, Vector (50.0, 50.0, 0.0));  // 中等距离

    // 模拟 UE 3：便携式终端 + 边缘位置
    uint16_t rnti3 = 3;
    scheduler->AddUeContext (rnti3, UT_PORTABLE, TRAFFIC_DATA);
    scheduler->AddUeInfo (rnti3, 1);
    scheduler->UpdateUeCsi (rnti3, 6.0);
    scheduler->UpdateUePosition (rnti3, Vector (500.0, 500.0, 0.0));  // 边缘

    // 利用仿真引擎触发调度
    Simulator::Schedule (Seconds (1.0), &GeoBeamScheduler::RunScheduler, scheduler);
    Simulator::Schedule (Seconds (1.5), &GeoBeamScheduler::RunScheduler, scheduler);
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // [新增测试] 8. 验证谢昀松的 SatUtMac 终端MAC层
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing SatUtMac (谢昀松) ===");
    
    // 创建终端MAC层
    Ptr<SatUtMac> utMac = CreateObject<SatUtMac> ();
    utMac->SwitchState (SatUtMac::MAC_CONNECTED);  // 先设置为CONNECTED状态
    
    // 模拟接收上行授权 DCI (UL Grant)
    DciInfo ulGrant;
    ulGrant.isUplinkGrant = true;
    ulGrant.rbAllocation = 10;
    ulGrant.mcs = 16;
    ulGrant.delayToStart = MilliSeconds (10);
    ulGrant.duration = MilliSeconds (1);
    
    NS_LOG_INFO ("Test 1: Process UL Grant DCI");
    Simulator::Schedule (Seconds (2.0), &SatUtMac::ProcessDciAndSchedule, utMac, ulGrant);
    
    // 模拟接收下行调度 DCI (DL Scheduling)
    DciInfo dlSchedule;
    dlSchedule.isUplinkGrant = false;
    dlSchedule.rbAllocation = 20;
    dlSchedule.mcs = 14;
    dlSchedule.delayToStart = MilliSeconds (5);
    dlSchedule.duration = MilliSeconds (1);
    
    NS_LOG_INFO ("Test 2: Process DL Scheduling DCI");
    Simulator::Schedule (Seconds (2.5), &SatUtMac::ProcessDciAndSchedule, utMac, dlSchedule);
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // [新增测试] 9. 验证 SatPdcp + ROHC 集成 (谢昀松)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing SatPdcp with ROHC (谢昀松) ===");
    
    // 使用 NtnConfigHelper 创建 PDCP + ROHC 实例
    Ptr<SatPdcp> pdcp = NtnConfigHelper::CreatePdcpWithRohc (1, 1);
    
    // 创建一个模拟的 UDP/IP 数据包
    Ptr<Packet> packet = Create<Packet> (500);
    Ipv4Header ipv4;
    ipv4.SetSource (Ipv4Address ("10.0.1.1"));
    ipv4.SetDestination (Ipv4Address ("10.0.2.1"));
    ipv4.SetProtocol (17);  // UDP
    packet->AddHeader (ipv4);
    
    UdpHeader udp;
    udp.SetSourcePort (1234);
    udp.SetDestinationPort (5678);
    packet->AddHeader (udp);
    
    NS_LOG_INFO ("Original packet size: " << packet->GetSize () << " bytes");
    
    // 测试发送 (压缩)
    NS_LOG_INFO ("Test 3: PDCP Send with ROHC Compression");
    pdcp->DoSendData (packet);
    
    // 测试接收 (解压) - 创建另一个包来模拟接收
    Ptr<Packet> rxPacket = Create<Packet> (500);
    rxPacket->AddHeader (ipv4);
    rxPacket->AddHeader (udp);
    NS_LOG_INFO ("Test 4: PDCP Receive with ROHC Decompression");
    pdcp->DoReceiveData (rxPacket);
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // [新增测试] 10. 验证崔博开的测量模块 (SatUtPhy) - 增强SINR计算
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing SatUtPhy Enhanced SINR Calculation (C/I + System Interference) ===");
    
    Ptr<SatUtPhy> utPhy = CreateObject<SatUtPhy> ();
    utPhy->SetAttribute ("Bandwidth", DoubleValue (5e6));  // 每波束5MHz
    utPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));
    utPhy->SetCqiReportCallback (MakeCallback (&CqiReportCallback));
    
    // 模拟接收信号，触发测量和CQI计算
    double rxPowerDbm = -70.0;
    uint32_t beamId = 1;
    
    // Test 5a: 基础SINR计算
    NS_LOG_INFO ("Test 5a: Basic SINR Calculation");
    double sinrDb = utPhy->CalculateSinr (rxPowerDbm, beamId);
    uint8_t cqi = utPhy->MapSinrToCqi (sinrDb);
    NS_LOG_INFO ("  RSRP=" << rxPowerDbm << " dBm -> SINR=" << sinrDb << " dB -> CQI=" << (uint16_t)cqi);
    
    // Test 5b: 增强型综合SINR计算 (C/I + 临道干扰 + 热噪声)
    NS_LOG_INFO ("Test 5b: Comprehensive SINR (C/I + I_adj + N)");
    double comprehensiveSinr = utPhy->CalculateComprehensiveSinr (rxPowerDbm, beamId);
    NS_LOG_INFO ("  Comprehensive SINR: " << comprehensiveSinr << " dB");
    
    // Test 5c: 载干比计算
    NS_LOG_INFO ("Test 5c: Carrier to Interference Ratio (C/I)");
    double cToI = utPhy->CalculateCarrierToInterferenceRatio (rxPowerDbm, beamId);
    NS_LOG_INFO ("  C/I: " << cToI << " dB");
    
    // Test 5d: 载干噪比计算
    NS_LOG_INFO ("Test 5d: Carrier to Interference + Noise (C/(I+N))");
    double cToIn = utPhy->CalculateCarrierToInterferencePlusNoise (rxPowerDbm, beamId);
    NS_LOG_INFO ("  C/(I+N): " << cToIn << " dB");
    
    // Test 5e: CQI触发报告
    NS_LOG_INFO ("Test 5e: Trigger CQI Report");
    utPhy->TriggerCqiReport ();
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // [新增测试] 11. 验证崔博开的波束切换 (Export/Import UeContext)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing Beam Handover (崔博开) ===");
    
    NS_LOG_INFO ("Test 6: Export UE Context for Handover");
    HandoverUeContext hoCtx = scheduler->ExportUeContext (rnti1, 2);
    scheduler->ImportUeContext (hoCtx);
    
    NS_LOG_INFO ("Test 7: Remove UE");
    scheduler->RemoveUt (rnti2);
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // [新增测试] 12. 验证崔博开的RRC测量模块 (SatUtRrc)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing SatUtRrc (崔博开) ===");
    
    Ptr<SatUtRrc> utRrc = CreateObject<SatUtRrc> ();
    utRrc->SetUeId (1);
    
    // 创建移动模型用于获取位置
    Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel> ();
    mobility->SetPosition (Vector (100.0, 200.0, 10.0));
    utRrc->SetMobilityModel (mobility);
    
    // 设置测量报告回调
    utRrc->SetReportCallback (MakeCallback (&ReceiveMeasReportCallback));
    
    // 配置测量参数
    MeasConfig config;
    config.rsrpThreshold = -100.0;
    config.hysteresis = 2.0;
    config.timeToTrigger = MilliSeconds (100);
    config.filterCoeff = 0.5;
    utRrc->SetMeasConfig (config);
    
    // 模拟第一次测量：信号良好，不触发上报
    NS_LOG_INFO ("Test 8: RRC Measurement - Good Signal (no report)");
    utRrc->ProcessRawMeasurement (1, -80.0);
    
    // 模拟第二次测量：信号变差，触发TTT
    NS_LOG_INFO ("Test 9: RRC Measurement - Poor Signal (trigger TTT)");
    utRrc->ProcessRawMeasurement (1, -110.0);
    
    // ------------------------------------------------------------
    // [新增测试] 13. 验证 4 步随机接入 (Msg1→Msg2→Msg3→Msg4)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing 4-Step Random Access (GEO ~600ms RTT) ===");

    // --- 场景 A: 单个 UE 成功完成 4 步 RA ---
    NS_LOG_INFO ("--- Scenario A: Single UE, 4-step RA should SUCCEED ---");

    Ptr<SatUtMac> utMacA = CreateObject<SatUtMac> ();
    utMacA->SetMultipleAccessMode (MultipleAccessMode::ESSA);
    utMacA->SwitchState (SatUtMac::MAC_IDLE);
    utMacA->SetUeIdentity (0xA1A1A1A1A1ULL);
    // GEO 场景: RAR 窗口 1000 ms, 竞争解决 1500 ms, 最多 5 次
    utMacA->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), 5);
    // 连接 UE → gNB 上行回调
    utMacA->SetPrachCallback (MakeCallback (&GeoBeamScheduler::ReceivePrachPreamble, scheduler));
    utMacA->SetMsg3Callback  (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));
    // 将本 UE 订阅到 gNB 的下行 RAR/Msg4 广播
    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMacA),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMacA));
    utMacA->SetRaCompleteCallback (MakeBoundCallback (&RaCompleteCallback, std::string ("UE-A")));

    // t=3.0 s 发起 RA, PreambleId=17 (唯一, 不会碰撞)
    Simulator::Schedule (Seconds (3.0), &SatUtMac::DoRandomAccess, utMacA,
                         static_cast<uint32_t> (17), static_cast<uint8_t> (0));

    // --- 场景 B: 两个 UE 选择同一 PreambleId, Msg3 碰撞 → 两者均应重传 ---
    NS_LOG_INFO ("--- Scenario B: Two UEs pick SAME PreambleId, Msg3 collision ---");

    Ptr<SatUtMac> utMacB1 = CreateObject<SatUtMac> ();
    Ptr<SatUtMac> utMacB2 = CreateObject<SatUtMac> ();
    utMacB1->SwitchState (SatUtMac::MAC_IDLE);
    utMacB2->SwitchState (SatUtMac::MAC_IDLE);
    utMacB1->SetUeIdentity (0xB1B1B1B1B1ULL);
    utMacB2->SetUeIdentity (0xB2B2B2B2B2ULL);
    utMacB1->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), 3);
    utMacB2->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), 3);

    utMacB1->SetPrachCallback (MakeCallback (&GeoBeamScheduler::ReceivePrachPreamble, scheduler));
    utMacB2->SetPrachCallback (MakeCallback (&GeoBeamScheduler::ReceivePrachPreamble, scheduler));
    utMacB1->SetMsg3Callback  (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));
    utMacB2->SetMsg3Callback  (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));

    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMacB1),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMacB1));
    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMacB2),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMacB2));

    utMacB1->SetRaCompleteCallback (MakeBoundCallback (&RaCompleteCallback, std::string ("UE-B1")));
    utMacB2->SetRaCompleteCallback (MakeBoundCallback (&RaCompleteCallback, std::string ("UE-B2")));

    // 两个 UE 几乎同时选同一个 preambleId=42:
    //   - 基站聚合到一份 preamble, 发一条 RAR (同一 TC-RNTI)
    //   - 两个 UE 都据此发 Msg3, 基站收到两份相同 TC-RNTI 的 Msg3 → 解码冲突
    //   - 不发 Msg4 → 两边竞争解决定时器都超时 → 退避后重发 Msg1
    Simulator::Schedule (Seconds (6.0), &SatUtMac::DoRandomAccess, utMacB1,
                         static_cast<uint32_t> (42), static_cast<uint8_t> (0));
    Simulator::Schedule (Seconds (6.0) + MicroSeconds (50), &SatUtMac::DoRandomAccess, utMacB2,
                         static_cast<uint32_t> (42), static_cast<uint8_t> (0));
    
    // ------------------------------------------------------------
    // [新增测试] 14. 验证 2 步随机接入 (MsgA→MsgB)
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing 2-Step Random Access (MsgA/MsgB) ===");

    // --- 场景 C: 单个 UE, 2 步 RA 成功 (SUCCESS_RAR) ---
    NS_LOG_INFO ("--- Scenario C: Single UE, 2-step RA should SUCCEED ---");

    Ptr<SatUtMac> utMacC = CreateObject<SatUtMac> ();
    utMacC->SwitchState (SatUtMac::MAC_IDLE);
    utMacC->SetUeIdentity (0xC1C1C1C1C1ULL);
    utMacC->SetRachType (RachType::TWO_STEP);
    utMacC->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), 5);
    // MsgA 回调 → gNB 的 ReceiveMsgA
    utMacC->SetMsgACallback (MakeCallback (&GeoBeamScheduler::ReceiveMsgA, scheduler));
    // 同时也需要 Msg3 回调 (FALLBACK 时复用)
    utMacC->SetMsg3Callback (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));
    // 订阅 MsgB 广播
    scheduler->RegisterUeTwoStepRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveMsgB, utMacC));
    // 也订阅 Msg4 (FALLBACK 时需要)
    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMacC),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMacC));
    utMacC->SetRaCompleteCallback (MakeBoundCallback (&RaCompleteCallback, std::string ("UE-C(2step)")));

    // t=10s 发起 2 步 RA, PreambleId=55 (唯一, 无碰撞 → SUCCESS_RAR)
    Simulator::Schedule (Seconds (10.0), &SatUtMac::InitiateRandomAccess, utMacC,
                         static_cast<uint32_t> (55), static_cast<uint8_t> (0));

    // --- 场景 D: 两个 UE, 2 步 RA, 同一 PreambleId → FALLBACK_RAR ---
    NS_LOG_INFO ("--- Scenario D: Two UEs pick SAME PreambleId, 2-step FALLBACK ---");

    Ptr<SatUtMac> utMacD1 = CreateObject<SatUtMac> ();
    Ptr<SatUtMac> utMacD2 = CreateObject<SatUtMac> ();
    utMacD1->SwitchState (SatUtMac::MAC_IDLE);
    utMacD2->SwitchState (SatUtMac::MAC_IDLE);
    utMacD1->SetUeIdentity (0xD1D1D1D1D1ULL);
    utMacD2->SetUeIdentity (0xD2D2D2D2D2ULL);
    utMacD1->SetRachType (RachType::TWO_STEP);
    utMacD2->SetRachType (RachType::TWO_STEP);
    utMacD1->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), 5);
    utMacD2->SetRaTimers (MilliSeconds (1000), MilliSeconds (1500), 5);

    // MsgA 回调
    utMacD1->SetMsgACallback (MakeCallback (&GeoBeamScheduler::ReceiveMsgA, scheduler));
    utMacD2->SetMsgACallback (MakeCallback (&GeoBeamScheduler::ReceiveMsgA, scheduler));
    // Msg3 回调 (FALLBACK 复用)
    utMacD1->SetMsg3Callback (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));
    utMacD2->SetMsg3Callback (MakeCallback (&GeoBeamScheduler::ReceiveMsg3, scheduler));

    // 订阅 MsgB 广播
    scheduler->RegisterUeTwoStepRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveMsgB, utMacD1));
    scheduler->RegisterUeTwoStepRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveMsgB, utMacD2));
    // 订阅 4 步 RAR/Msg4 (FALLBACK 后需要)
    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMacD1),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMacD1));
    scheduler->RegisterUeRaCallbacks (
        MakeCallback (&SatUtMac::ReceiveRar, utMacD2),
        MakeCallback (&SatUtMac::ReceiveMsg4, utMacD2));

    utMacD1->SetRaCompleteCallback (MakeBoundCallback (&RaCompleteCallback, std::string ("UE-D1(2step)")));
    utMacD2->SetRaCompleteCallback (MakeBoundCallback (&RaCompleteCallback, std::string ("UE-D2(2step)")));

    // 两个 UE 几乎同时发 MsgA, 同一 PreambleId=60 → FALLBACK_RAR
    Simulator::Schedule (Seconds (13.0), &SatUtMac::InitiateRandomAccess, utMacD1,
                         static_cast<uint32_t> (60), static_cast<uint8_t> (0));
    Simulator::Schedule (Seconds (13.0) + MicroSeconds (50), &SatUtMac::InitiateRandomAccess, utMacD2,
                         static_cast<uint32_t> (60), static_cast<uint8_t> (0));

    // ------------------------------------------------------------
    // [新增测试] 15. 验证ESSA (Enhanced Slotted ALOHA) 多址接入
    // ------------------------------------------------------------
    NS_LOG_INFO ("=== Testing ESSA (Enhanced Slotted ALOHA) ===");
    
    Ptr<SatUtPhy> utPhyForEssa = CreateObject<SatUtPhy> ();
    utPhyForEssa->SetAttribute ("Bandwidth", DoubleValue (5e6));
    utPhyForEssa->SetAttribute ("EsssaNumSlots", UintegerValue (16));
    
    // Test 11a: 计算ESSA碰撞概率
    NS_LOG_INFO ("Test 11a: ESSA Collision Probability");
    double collisionProb = utPhyForEssa->CalculateCollisionProbability (10, 16);  // 10用户, 16时隙
    NS_LOG_INFO ("  Collision Probability: " << collisionProb * 100 << "%");
    
    // Test 11b: ESSA时隙选择
    NS_LOG_INFO ("Test 11b: ESSA Slot Selection (multiple users)");
    for (uint32_t i = 0; i < 5; i++) {
        uint32_t selectedSlot = utPhyForEssa->SelectEssAlohaSlot ();
        NS_LOG_INFO ("  User " << i << " selected slot: " << selectedSlot);
    }
    
    // Test 11c: 切换多址模式 - Slotted ALOHA
    NS_LOG_INFO ("Test 11c: Switch to Slotted ALOHA Mode");
    utPhyForEssa->SetMultipleAccessMode (MultipleAccessMode::SLOTTED_ALOHA);
    double collisionProbSaloha = utPhyForEssa->CalculateCollisionProbability (10, 16);
    NS_LOG_INFO ("  Slotted ALOHA Collision Probability: " << collisionProbSaloha * 100 << "%");
    
    // ------------------------------------------------------------

    // 10. 运行仿真
    // 4 步 + 2 步 RA 场景: 碰撞重传额外 1-2 次 → 预留 25 秒
    Simulator::Stop (Seconds (25.0));
    Simulator::Run ();

    gwMac->StopPeriodicTransmissions ();

    Simulator::Destroy ();

    NS_LOG_INFO ("=== RA Summary (4-Step + 2-Step) ===");
    NS_LOG_INFO ("  SUCCESS = " << g_raSuccessCount);
    NS_LOG_INFO ("  FAILED  = " << g_raFailedCount);
    NS_LOG_INFO ("Simulation Finished.");
    return 0;
}
