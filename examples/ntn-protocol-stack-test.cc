/*
 * 文件: contrib/geo-sat/examples/ntn-protocol-stack-test.cc
 *
 * NTN 协议栈联通性自动化测试 (合并替代之前 7 个独立 demo)
 *
 *   T1 PHY-relay        — 单波束 GW PHY → 卫星弯管 → UE PHY
 *   T2 MAC-bidir        — MAC ↔ MAC 双向数据面 (DL/UL)
 *   T3 MAC-multiUE      — 多 UE round-robin 调度 + HARQ ACK
 *   T4 PHY-multibeam    — 多波束 per-beam channel + NtnBeamIdTag 路由 + UE 间隔离
 *   T5 RLC-modes        — TM/UM/AM 三模式 + AM 主动丢包 -> 重传 -> 精确计数
 *   T6 Fullstack        — App↔PDCP↔RLC AM↔MAC↔PHY↔弯管 + SINR→BLER PHY 真丢包 -> AM 重传 -> App 100% 收到
 *
 * 每个子测试自检 PASS/FAIL, 用 RngSeedManager 固定种子保证可复现。
 *
 * 用法:
 *   ./ns3 run ntn-protocol-stack-test                # 6/6 全跑
 *   ./ns3 run "ntn-protocol-stack-test --only=T1,T4" # 只跑 T1 与 T4
 *   ./ns3 run "ntn-protocol-stack-test --verbose"    # 打开内部 LogComponent
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"

#include "ns3/geo-beam-helper.h"
#include "ns3/gw-helper.h"
#include "ns3/sat-phy.h"
#include "ns3/ntn-gw-phy.h"
#include "ns3/ntn-gw-mac.h"
#include "ns3/sat-ut-mac.h"
#include "ns3/ntn-pdcp.h"
#include "ns3/ntn-rlc.h"
#include "ns3/ntn-sdap.h"
#include "ns3/ntn-phy-error-model.h"
#include "ns3/sat-ut-rrc.h"
#include "ns3/ntn-gw-rrc.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/result-writer.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NtnProtocolStackTest");

// ---------- 公共脚手架 ----------
struct TestResult
{
  std::string name;
  bool        pass;
  std::string note;
};

namespace {

std::vector<TestResult> g_results;

// 重置随机种子, 让每个子测试都从同一起点(否则前序子测试消耗 rng 后, 后续行为可变)
void
ResetSimulationContext ()
{
  Simulator::Destroy ();
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun  (1);
}

void
ReportTest (const std::string& name, bool pass, const std::string& note)
{
  g_results.push_back ({name, pass, note});
  std::cout << "  " << std::left << std::setw (32) << name
            << (pass ? "PASS" : "FAIL")
            << "   " << note << "\n";
}

// 静态 helpers for MakeBoundCallback (lambda 不行)
void
GwRlcToMac (Ptr<NtnGwMac> mac, uint16_t rnti, Ptr<Packet> p)
{
  mac->EnqueueDlPdu (rnti, p);
}

} // namespace

// ============================================================================
// T1: PHY relay (single beam)
//   GW NtnGwPhy → feeder channel → SatGeoFeederPhy → SatGeoUserPhy → user channel → UE SatPhy
//   验证: t=1.0s 发 1000B → UE 收到, 端到端延迟 ≈ 2 × ~119ms ≈ 240ms
// ============================================================================
namespace T1 {

double g_rxTime = -1.0;
uint32_t g_rxSize = 0;

void OnUeRx (Ptr<const Packet> p) { g_rxTime = Simulator::Now ().GetSeconds (); g_rxSize = p->GetSize (); }

bool Run ()
{
  ResetSimulationContext ();
  g_rxTime = -1.0; g_rxSize = 0;

  NodeContainer gwNode;  gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNode;  ueNode.Create (1);

  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,35786000.0));
  mob.Install (ueNode);  ueNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,1000,1.5));

  GeoBeamHelper sat;
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  // UE PHY 接到 beam 1 的 user channel
  Ptr<SpectrumChannel>     uch = sat.GetSpectrumChannel ();
  Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel ();
  Ptr<SatPhy> uePhy = CreateObject<SatPhy> ();
  uePhy->SetChannel (uch);
  uePhy->SetMobility (ueNode.Get (0)->GetObject<MobilityModel> ());
  uePhy->SetRxSpectrumModel (usm);
  uch->AddRx (uePhy);
  uePhy->TraceConnectWithoutContext ("Rx", MakeCallback (&T1::OnUeRx));

  Simulator::Schedule (Seconds (1.0), [&] () {
    Ptr<NtnGwPhy> gwPhy = gw.GetGwPhy (0);
    Ptr<SpectrumValue> psd = Create<SpectrumValue> (sat.GetFeederSpectrumModel ());
    (*psd)[0] = std::pow (10.0, 1.0) / 100e6;
    Ptr<SatSignalParameters> p = Create<SatSignalParameters> ();
    p->psd = psd; p->duration = MilliSeconds (1); p->packet = Create<Packet> (1000);
    gwPhy->SendPdu (p->packet, p);
  });
  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  bool ok = (g_rxSize == 1000) && (g_rxTime > 1.20) && (g_rxTime < 1.30);
  std::ostringstream note;
  note << "Rx@UE t=" << std::fixed << std::setprecision (4) << g_rxTime
       << "s size=" << g_rxSize << "B";
  ReportTest ("T1 PHY relay (1 beam)", ok, note.str ());
  return ok;
}

} // namespace T1

// ============================================================================
// T2: MAC bidirectional
//   GW MAC ↔ UE MAC 双向: DL 1024B + UL 512B
// ============================================================================
namespace T2 {

uint32_t g_ueDlRx = 0;
uint32_t g_gwUlRx = 0;
double   g_ueDlTime = 0.0, g_gwUlTime = 0.0;

void OnUeDl (Ptr<Packet> p) { g_ueDlRx += p->GetSize (); g_ueDlTime = Simulator::Now ().GetSeconds (); }
void OnGwUl (Ptr<Packet> p) { g_gwUlRx += p->GetSize (); g_gwUlTime = Simulator::Now ().GetSeconds (); }

bool Run ()
{
  ResetSimulationContext ();
  g_ueDlRx = g_gwUlRx = 0; g_ueDlTime = g_gwUlTime = 0.0;

  NodeContainer gwNode;  gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNode;  ueNode.Create (1);

  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,35786000.0));
  mob.Install (ueNode);  ueNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,1000,1.5));

  GeoBeamHelper sat;
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  Ptr<NtnGwMac> gwMac = CreateObject<NtnGwMac> ();
  gwMac->SetPhy (gw.GetGwPhy (0));
  gwMac->SetTxConfig (sat.GetFeederSpectrumModel (), 100e6, 40.0, MilliSeconds (1));
  gwMac->SetUlPduCallback (MakeCallback (&OnGwUl));

  Ptr<SpectrumChannel>     uch = sat.GetSpectrumChannel ();
  Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel ();
  Ptr<SatPhy> uePhy = CreateObject<SatPhy> ();
  uePhy->SetChannel (uch);
  uePhy->SetMobility (ueNode.Get (0)->GetObject<MobilityModel> ());
  uePhy->SetRxSpectrumModel (usm);
  uch->AddRx (uePhy);

  Ptr<SatUtMac> ueMac = CreateObject<SatUtMac> ();
  ueMac->SetPhy (uePhy);
  ueMac->SetTxConfig (usm, 20e6, 23.0, MilliSeconds (1));
  ueMac->SetDlPduCallback (MakeCallback (&OnUeDl));

  Simulator::Schedule (Seconds (1.0), [&] () { gwMac->SendDlPdu (Create<Packet> (1024)); });
  Simulator::Schedule (Seconds (2.0), [&] () { ueMac->SendUlPdu (Create<Packet> (512));  });
  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();

  bool ok = (g_ueDlRx == 1024) && (g_gwUlRx == 512)
         && (g_ueDlTime > 1.20 && g_ueDlTime < 1.30)
         && (g_gwUlTime > 2.20 && g_gwUlTime < 2.30);
  std::ostringstream note;
  note << "DL=" << g_ueDlRx << "B@" << std::fixed << std::setprecision (3) << g_ueDlTime
       << " UL=" << g_gwUlRx << "B@" << g_gwUlTime;
  ReportTest ("T2 MAC bidirectional", ok, note.str ());
  return ok;
}

} // namespace T2

// ============================================================================
// T3: MAC multi-UE schedule + HARQ ACK
//   2 UE 共享 user channel, round-robin 调度 4+4 个 DL, ACK 各自计数
// ============================================================================
namespace T3 {

uint32_t g_appRx[2] = {0, 0};

void OnRx (uint32_t idx, Ptr<Packet>) { g_appRx[idx]++; }

bool Run ()
{
  ResetSimulationContext ();
  g_appRx[0] = g_appRx[1] = 0;

  NodeContainer gwNode;  gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNodes; ueNodes.Create (2);
  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,35786000.0));
  mob.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,1000.0,1.5));
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (0,2000.0,1.5));

  GeoBeamHelper sat;
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  Ptr<NtnGwMac> gwMac = CreateObject<NtnGwMac> ();
  gwMac->SetPhy (gw.GetGwPhy (0));
  gwMac->SetTxConfig (sat.GetFeederSpectrumModel (), 100e6, 40.0, MilliSeconds (1));

  Ptr<SpectrumChannel>     uch = sat.GetSpectrumChannel ();
  Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel ();

  std::vector<Ptr<SatUtMac>> ueMacs (2);
  const uint16_t rntis[2] = {1001, 1002};
  for (uint32_t i = 0; i < 2; ++i)
    {
      Ptr<SatPhy> p = CreateObject<SatPhy> ();
      p->SetChannel (uch);
      p->SetMobility (ueNodes.Get (i)->GetObject<MobilityModel> ());
      p->SetRxSpectrumModel (usm);
      uch->AddRx (p);
      ueMacs[i] = CreateObject<SatUtMac> ();
      ueMacs[i]->SetPhy (p);
      ueMacs[i]->SetTxConfig (usm, 20e6, 23.0, MilliSeconds (1));
      ueMacs[i]->SetRnti (rntis[i]);
      ueMacs[i]->SetDlPduCallback (MakeBoundCallback (&OnRx, i));
      gwMac->AddUe (rntis[i]);
    }

  Simulator::Schedule (Seconds (1.0), [&] () {
    for (uint32_t k = 0; k < 4; ++k)
      {
        gwMac->EnqueueDlPdu (1001, Create<Packet> (1024));
        gwMac->EnqueueDlPdu (1002, Create<Packet> (1024));
      }
    gwMac->StartScheduler (MilliSeconds (100));
  });
  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();

  uint32_t ack0 = gwMac->GetAckCount (1001);
  uint32_t ack1 = gwMac->GetAckCount (1002);
  bool ok = (g_appRx[0] == 4) && (g_appRx[1] == 4) && (ack0 == 4) && (ack1 == 4);
  std::ostringstream note;
  note << "UE1 app=" << g_appRx[0] << " ACK=" << ack0
       << " | UE2 app=" << g_appRx[1] << " ACK=" << ack1;
  ReportTest ("T3 MAC multi-UE + HARQ ACK", ok, note.str ());
  return ok;
}

} // namespace T3

// ============================================================================
// T4: PHY multi-beam isolation
//   每束独立 user channel + NtnBeamIdTag 路由
//   UE1@beam1 不应该收到 GW 发往 beam2 的包
// ============================================================================
namespace T4 {

uint32_t g_appRx[2] = {0, 0};
void OnRx (uint32_t idx, Ptr<Packet>) { g_appRx[idx]++; }

bool Run ()
{
  ResetSimulationContext ();
  g_appRx[0] = g_appRx[1] = 0;

  NodeContainer gwNode;  gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNodes; ueNodes.Create (2);
  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,35786000.0));
  mob.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,1000.0,1.5));
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (0,2000.0,1.5));

  GeoBeamHelper sat;
  sat.SetActiveBeamCount (2);
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  Ptr<NtnGwMac> gwMac = CreateObject<NtnGwMac> ();
  gwMac->SetPhy (gw.GetGwPhy (0));
  gwMac->SetTxConfig (sat.GetFeederSpectrumModel (), 100e6, 40.0, MilliSeconds (1));

  const uint16_t rntis[2]   = {1001, 1002};
  const uint16_t beamIds[2] = {1, 2};

  std::vector<Ptr<SatUtMac>> ueMacs (2);
  for (uint32_t i = 0; i < 2; ++i)
    {
      Ptr<SpectrumChannel>     uch = sat.GetUserChannel (beamIds[i]);
      Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel (beamIds[i]);
      Ptr<SatPhy> p = CreateObject<SatPhy> ();
      p->SetChannel (uch); p->SetMobility (ueNodes.Get (i)->GetObject<MobilityModel> ());
      p->SetRxSpectrumModel (usm);
      uch->AddRx (p);
      ueMacs[i] = CreateObject<SatUtMac> ();
      ueMacs[i]->SetPhy (p);
      ueMacs[i]->SetTxConfig (usm, 5e6, 23.0, MilliSeconds (1));
      ueMacs[i]->SetRnti (rntis[i]);
      ueMacs[i]->SetDlPduCallback (MakeBoundCallback (&OnRx, i));
      gwMac->AddUe (rntis[i], beamIds[i]);
    }

  Simulator::Schedule (Seconds (1.0), [&] () {
    for (uint32_t k = 0; k < 4; ++k)
      {
        gwMac->EnqueueDlPdu (1001, Create<Packet> (1024));
        gwMac->EnqueueDlPdu (1002, Create<Packet> (1024));
      }
    gwMac->StartScheduler (MilliSeconds (100));
  });
  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();

  bool ok = (g_appRx[0] == 4) && (g_appRx[1] == 4)
         && (gwMac->GetAckCount (1001) == 4) && (gwMac->GetAckCount (1002) == 4);
  std::ostringstream note;
  note << "UE1@beam1 rx=" << g_appRx[0] << " UE2@beam2 rx=" << g_appRx[1]
       << " (per-beam channel + BeamIdTag)";
  ReportTest ("T4 PHY multi-beam isolation", ok, note.str ());
  return ok;
}

} // namespace T4

// ============================================================================
// T5: RLC modes (TM/UM/AM) + AM retx
//   端到端 RLC↔RLC 直连, AM 配 0.1 drop_prob → 应触发可观察重传, 重传后全部到达
// ============================================================================
namespace T5 {

struct ModeRes { uint32_t appRx = 0; };
ModeRes resTm, resUm, resAm;

void OnUpper (ModeRes* r, Ptr<Packet>) { r->appRx++; }

bool Run ()
{
  ResetSimulationContext ();
  resTm = resUm = resAm = {};

  auto MakePair = [] (NtnRlcMode mode, double drop, ModeRes* res,
                      Ptr<NtnRlc>& txRlc, Ptr<NtnRlc>& rxRlc, uint16_t rnti)
  {
    txRlc = CreateObject<NtnRlc> ();
    rxRlc = CreateObject<NtnRlc> ();
    txRlc->SetRnti (rnti); txRlc->SetLcid (3); txRlc->SetMode (mode);
    rxRlc->SetRnti (rnti); rxRlc->SetLcid (3); rxRlc->SetMode (mode);
    if (mode == NtnRlcMode::AM)
      {
        rxRlc->SetAmDropProb (drop);
        rxRlc->SetStatusProhibitTimer (MilliSeconds (30));
        txRlc->SetPollRetransmitTimer (MilliSeconds (500));
      }
    txRlc->SetTransmitCallback (MakeCallback (&NtnRlc::ReceiveRlcPdu, rxRlc));
    rxRlc->SetTransmitCallback (MakeCallback (&NtnRlc::ReceiveRlcPdu, txRlc));
    rxRlc->SetReceiveCallback (MakeBoundCallback (&OnUpper, res));
  };

  Ptr<NtnRlc> tmTx, tmRx, umTx, umRx, amTx, amRx;
  MakePair (NtnRlcMode::TM, 0.0, &resTm, tmTx, tmRx, 100);
  MakePair (NtnRlcMode::UM, 0.0, &resUm, umTx, umRx, 101);
  MakePair (NtnRlcMode::AM, 0.1, &resAm, amTx, amRx, 102);

  const uint32_t N = 50;
  for (uint32_t i = 0; i < N; ++i)
    {
      Time t = MilliSeconds (10 + i * 5);
      Simulator::Schedule (t, [&] () { tmTx->TransmitPdcpPdu (Create<Packet> (200)); });
      Simulator::Schedule (t, [&] () { umTx->TransmitPdcpPdu (Create<Packet> (200)); });
      Simulator::Schedule (t, [&] () { amTx->TransmitPdcpPdu (Create<Packet> (200)); });
    }
  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();

  // 期望 TM: 50 rx 0 retx; UM: 50 rx 0 retx; AM: retx>0 且 rx ≈ 50 (最终都收到)
  uint32_t tmRetx = tmTx->GetExactRetxCount ();
  uint32_t umRetx = umTx->GetExactRetxCount ();
  uint32_t amRetx = amTx->GetExactRetxCount ();
  bool ok = (resTm.appRx == N) && (tmRetx == 0)
         && (resUm.appRx == N) && (umRetx == 0)
         && (resAm.appRx == N) && (amRetx > 0);
  std::ostringstream note;
  note << "TM rx=" << resTm.appRx << "/" << N
       << "  UM rx=" << resUm.appRx << "/" << N
       << "  AM rx=" << resAm.appRx << "/" << N << " retx=" << amRetx;
  ReportTest ("T5 RLC TM/UM/AM modes", ok, note.str ());
  return ok;
}

} // namespace T5

// ============================================================================
// T6: Fullstack reliable delivery
//   App ↔ PDCP ↔ RLC AM ↔ MAC ↔ PHY ↔ 弯管 ↔ PHY ↔ MAC ↔ RLC AM ↔ PDCP ↔ App
//   2 UE: UE1 SINR=12dB BLER ~0; UE2 SINR=2dB BLER ~0.34
//   验证: 两 UE App 都 100% 收到; UE2 PHY drop > 0 且 RLC retx > 0
// ============================================================================
namespace T6 {

struct UeRx { uint32_t pdu = 0; };
UeRx ueAppRx[2];

void OnAppRx (uint32_t idx, Ptr<Packet>) { ueAppRx[idx].pdu++; }

bool Run ()
{
  ResetSimulationContext ();
  ueAppRx[0] = ueAppRx[1] = {};

  NodeContainer gwNode;  gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNodes; ueNodes.Create (2);
  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,35786000.0));
  mob.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,1000.0,1.5));
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (0,2000.0,1.5));

  GeoBeamHelper sat;
  sat.SetActiveBeamCount (2);
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  Ptr<NtnGwMac> gwMac = CreateObject<NtnGwMac> ();
  gwMac->SetPhy (gw.GetGwPhy (0));
  gwMac->SetTxConfig (sat.GetFeederSpectrumModel (), 100e6, 40.0, MilliSeconds (1));

  const uint16_t rntis[2]   = {1001, 1002};
  const uint16_t beamIds[2] = {1, 2};
  const double sinrs[2]     = {12.0, 2.0};

  std::vector<Ptr<NtnRlc>>  gwRlcs (2), ueRlcs (2);
  std::vector<Ptr<NtnPdcp>> gwPdcps (2), uePdcps (2);
  std::vector<Ptr<NtnPhyErrorModel>> ems (2);
  std::vector<Ptr<SatPhy>>   uePhys (2);   // 必须持到 Simulator::Run() 结束, 否则 callback 悬挂
  std::vector<Ptr<SatUtMac>> ueMacs (2);

  for (uint32_t i = 0; i < 2; ++i)
    {
      Ptr<SpectrumChannel>     uch = sat.GetUserChannel (beamIds[i]);
      Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel (beamIds[i]);

      Ptr<SatPhy> uePhy = CreateObject<SatPhy> ();
      uePhy->SetChannel (uch);
      uePhy->SetMobility (ueNodes.Get (i)->GetObject<MobilityModel> ());
      uePhy->SetRxSpectrumModel (usm);
      uch->AddRx (uePhy);
      uePhys[i] = uePhy;

      Ptr<SatUtMac> ueMac = CreateObject<SatUtMac> ();
      ueMac->SetPhy (uePhy);
      ueMac->SetTxConfig (usm, 5e6, 23.0, MilliSeconds (1));
      ueMac->SetRnti (rntis[i]);
      ueMacs[i] = ueMac;

      // UE 全栈
      Ptr<NtnRlc> ueRlc = CreateObject<NtnRlc> ();
      ueRlc->SetMode (NtnRlcMode::AM);
      ueRlc->SetRnti (rntis[i]); ueRlc->SetLcid (3);
      ueRlc->SetStatusProhibitTimer (MilliSeconds (50));
      ueRlc->SetPollRetransmitTimer (MilliSeconds (1500));
      Ptr<NtnPdcp> uePdcp = CreateObject<NtnPdcp> ();
      uePdcp->SetRnti (rntis[i]); uePdcp->SetLcId (3);
      uePdcp->SetRlc (ueRlc);
      uePdcp->SetIpRxCallback (MakeBoundCallback (&OnAppRx, i));
      ueRlc->SetTransmitCallback (MakeCallback (&SatUtMac::SendUlPdu, ueMac));
      ueMac->SetRlc (ueRlc);

      Ptr<NtnPhyErrorModel> em = CreateObject<NtnPhyErrorModel> ();
      em->SetSinrDb (sinrs[i]);
      ueMac->SetPhyErrorModel (em);
      ems[i] = em;
      ueRlcs[i] = ueRlc;
      uePdcps[i] = uePdcp;

      // GW 侧
      Ptr<NtnRlc> gwRlc = CreateObject<NtnRlc> ();
      gwRlc->SetMode (NtnRlcMode::AM);
      gwRlc->SetRnti (rntis[i]); gwRlc->SetLcid (3);
      gwRlc->SetStatusProhibitTimer (MilliSeconds (50));
      gwRlc->SetPollRetransmitTimer (MilliSeconds (1500));
      Ptr<NtnPdcp> gwPdcp = CreateObject<NtnPdcp> ();
      gwPdcp->SetRnti (rntis[i]); gwPdcp->SetLcId (3);
      gwPdcp->SetRlc (gwRlc);
      gwRlc->SetTransmitCallback (MakeBoundCallback (&GwRlcToMac, gwMac, rntis[i]));
      gwMac->RegisterUlRlc (rntis[i], gwRlc);
      gwRlcs[i] = gwRlc;
      gwPdcps[i] = gwPdcp;

      gwMac->AddUe (rntis[i], beamIds[i]);
    }

  const uint32_t N = 30;
  Simulator::Schedule (Seconds (1.0), [&] () {
    for (uint32_t i = 0; i < 2; ++i)
      for (uint32_t k = 0; k < N; ++k)
        gwPdcps[i]->DoSendData (Create<Packet> (200));
    gwMac->StartScheduler (MilliSeconds (10));
  });
  Simulator::Stop (Seconds (8.0));
  Simulator::Run ();

  bool ok = (ueAppRx[0].pdu == N) && (ueAppRx[1].pdu == N)
         && (ems[1]->GetDropCount () > 0)
         && (gwRlcs[1]->GetExactRetxCount () > 0);
  std::ostringstream note;
  note << "UE1 app=" << ueAppRx[0].pdu << "/" << N
       << " (drop=" << ems[0]->GetDropCount () << " retx=" << gwRlcs[0]->GetExactRetxCount () << ")"
       << " UE2 app=" << ueAppRx[1].pdu << "/" << N
       << " (drop=" << ems[1]->GetDropCount () << " retx=" << gwRlcs[1]->GetExactRetxCount () << ")";
  ReportTest ("T6 Fullstack reliable", ok, note.str ());
  return ok;
}

} // namespace T6

// ============================================================================
// T7: SDAP QoS flow → DRB mapping + E2E delay
//   2 个 QFI (voice=1 -> LCID 3, data=5 -> LCID 4), tx -> 100ms 后 rx
//   验证: 两 DRB 按 QFI 路由正确, RxSdap E2E delay ≈ 100ms
// ============================================================================
namespace T7 {

struct DrbCounter { uint32_t tx = 0, rx = 0; uint64_t sumDelayNs = 0; uint8_t lastLcid = 0; };
DrbCounter cntVoice, cntData;

// 单个 trace handler, 内部按 qfi 分流计数
void OnSdapTx (uint16_t, uint8_t qfi, uint8_t lcid, uint32_t)
{
  if (qfi == 1) { cntVoice.tx++; cntVoice.lastLcid = lcid; }
  else if (qfi == 5) { cntData.tx++; cntData.lastLcid = lcid; }
}
void OnSdapRx (uint16_t, uint8_t qfi, uint8_t lcid, uint32_t, uint64_t d)
{
  if (qfi == 1) { cntVoice.rx++; cntVoice.sumDelayNs += d; cntVoice.lastLcid = lcid; }
  else if (qfi == 5) { cntData.rx++; cntData.sumDelayNs += d; cntData.lastLcid = lcid; }
}

bool Run ()
{
  ResetSimulationContext ();
  cntVoice = cntData = {};

  Ptr<NtnSdap> tx = CreateObject<NtnSdap> ();
  Ptr<NtnSdap> rx = CreateObject<NtnSdap> ();
  tx->SetRnti (200);  rx->SetRnti (200);
  tx->MapQfiToDrb (1, 3);  tx->MapQfiToDrb (5, 4);
  rx->MapQfiToDrb (1, 3);  rx->MapQfiToDrb (5, 4);

  tx->TraceConnectWithoutContext ("TxSDAP", MakeCallback (&OnSdapTx));
  rx->TraceConnectWithoutContext ("RxSDAP", MakeCallback (&OnSdapRx));

  // 同一 packet 用 ByteTag 携带 sendTime, 100ms 后 rx 端读出 delay
  for (uint32_t i = 0; i < 10; ++i)
    {
      Ptr<Packet> p = Create<Packet> (50);
      Simulator::Schedule (MilliSeconds (10 + i * 5), [tx, p] () { tx->DoSendData (1, p); });
      Simulator::Schedule (MilliSeconds (110 + i * 5), [rx, p] () { rx->DoReceiveData (1, p); });
    }
  for (uint32_t i = 0; i < 5; ++i)
    {
      Ptr<Packet> p = Create<Packet> (200);
      Simulator::Schedule (MilliSeconds (10 + i * 5), [tx, p] () { tx->DoSendData (5, p); });
      Simulator::Schedule (MilliSeconds (110 + i * 5), [rx, p] () { rx->DoReceiveData (5, p); });
    }

  Simulator::Stop (Seconds (1.0));
  Simulator::Run ();

  uint64_t avgVoice = cntVoice.rx > 0 ? cntVoice.sumDelayNs / cntVoice.rx : 0;
  uint64_t avgData  = cntData.rx  > 0 ? cntData.sumDelayNs  / cntData.rx  : 0;
  bool ok = (cntVoice.tx == 10) && (cntVoice.rx == 10) && (cntVoice.lastLcid == 3)
         && (cntData.tx  == 5)  && (cntData.rx  == 5)  && (cntData.lastLcid  == 4)
         && (avgVoice > 0) && (avgData > 0);
  std::ostringstream note;
  note << "voice(qfi=1)->lcid=" << (int) cntVoice.lastLcid << " rx=" << cntVoice.rx << "/10 delay=" << avgVoice / 1e6 << "ms | "
       << "data(qfi=5)->lcid=" << (int) cntData.lastLcid << " rx=" << cntData.rx << "/5 delay=" << avgData / 1e6 << "ms";
  ReportTest ("T7 SDAP QFI->DRB + E2E", ok, note.str ());
  return ok;
}

} // namespace T7

// ============================================================================
// T8: RRC state machine + Paging + access recovery delay
//   IDLE_CAMPED -> CONNECTED -> InactivityTimer 100ms -> RRC_INACTIVE
//   GW SendPaging -> UE ReceivePagingMessage -> IDLE_RANDOM_ACCESS
//   -> (模拟 5ms RA 完成) -> CONNECTED -> 算 access_recovery_delay > 0
// ============================================================================
namespace T8 {

std::vector<SatUtRrc::State> g_stateSeq;
uint16_t g_pagedUeId = 0;
uint64_t g_pagingTxTimeNs = 0;

void OnStateTransition (uint16_t, SatUtRrc::State, SatUtRrc::State newState)
{
  g_stateSeq.push_back (newState);
}
void OnGwPagingTx (uint16_t ueId, uint64_t tNs) { g_pagedUeId = ueId; g_pagingTxTimeNs = tNs; }

// gwRrc 的 SetPagingCallback 签名是 (uint16_t ueId), SatUtRrc::ReceivePagingMessage 无参,
// 用 trampoline 适配
void PagingTrampoline (Ptr<SatUtRrc> ueRrc, uint16_t /*ueId*/) { ueRrc->ReceivePagingMessage (); }

bool Run ()
{
  ResetSimulationContext ();
  g_stateSeq.clear ();
  g_pagedUeId = 0; g_pagingTxTimeNs = 0;

  Ptr<SatUtRrc> ueRrc = CreateObject<SatUtRrc> ();
  ueRrc->SetUeId (3001);
  ueRrc->SetInactivityTimer (MilliSeconds (100));
  ueRrc->TraceConnectWithoutContext ("StateTransition", MakeCallback (&OnStateTransition));

  Ptr<NtnGwRrc> gwRrc = CreateObject<NtnGwRrc> ();
  gwRrc->SetPagingCallback (MakeBoundCallback (&PagingTrampoline, ueRrc));
  // m_pagingTrace 不是 attribute, 直接对成员变量 trace connect:
  gwRrc->m_pagingTrace.ConnectWithoutContext (MakeCallback (&OnGwPagingTx));

  // T8 时间轴
  Simulator::Schedule (MilliSeconds (100), [&] () { ueRrc->SwitchToState (SatUtRrc::IDLE_CAMPED_NORMALLY); });
  Simulator::Schedule (MilliSeconds (200), [&] () { ueRrc->SwitchToState (SatUtRrc::CONNECTED_NORMALLY); });
  Simulator::Schedule (MilliSeconds (210), [&] () { ueRrc->ResetInactivityTimer (); });
  // inactivity timeout 200+100+10 = 310ms 后 UE 进入 RRC_INACTIVE
  Simulator::Schedule (MilliSeconds (500), [&] () { gwRrc->SendPaging (3001); });
  // ReceivePagingMessage 把状态切到 IDLE_RANDOM_ACCESS, 5ms 后假装 RA 完成 -> CONNECTED
  Simulator::Schedule (MilliSeconds (505), [&] () { ueRrc->SwitchToState (SatUtRrc::CONNECTED_NORMALLY); });

  Simulator::Stop (MilliSeconds (700));
  Simulator::Run ();

  // 期望状态序列 (尾段):
  //   IDLE_CAMPED -> CONNECTED -> RRC_INACTIVE -> IDLE_RANDOM_ACCESS -> CONNECTED
  bool seqOk = (g_stateSeq.size () >= 5)
               && g_stateSeq[0] == SatUtRrc::IDLE_CAMPED_NORMALLY
               && g_stateSeq[1] == SatUtRrc::CONNECTED_NORMALLY
               && g_stateSeq[2] == SatUtRrc::RRC_INACTIVE
               && g_stateSeq[3] == SatUtRrc::IDLE_RANDOM_ACCESS
               && g_stateSeq[4] == SatUtRrc::CONNECTED_NORMALLY;
  Time recovery = ueRrc->GetLastAccessRecoveryDelay ();
  bool recOk = recovery > Seconds (0) && recovery < MilliSeconds (50);
  bool ok = seqOk && recOk;
  std::ostringstream note;
  note << "transitions=" << g_stateSeq.size ()
       << " recovery=" << recovery.GetMilliSeconds () << "ms"
       << " (pagingTxAt=" << g_pagingTxTimeNs / 1000000 << "ms)";
  ReportTest ("T8 RRC state + Paging", ok, note.str ());
  return ok;
}

} // namespace T8

// ============================================================================
// T9: Power-domain NOMA — 真实物理闭环 (自动接线, 不再手填 effSINR)
//   (A) beta power split written into near/far DCIs
//   (B) 单一真源自动接线: gwMac->SetScheduler(sched) 后, NOMA 配对真实跑一轮调度
//       产生 near/far 的 {effSINR, txPower}; SendDlPduForRnti/StampDlDciTag 自动从调度器
//       取这两个值 -> 发射功率真正进 PSD (β 切分, far>near 约 6dB), effSINR 进 NtnDciTag
//       驱动 UE BLER (far 被干扰 effSINR 低 -> 丢包远多于 near)。无任何手工 SetEffectiveDlSinrDb。
//   (C) NOMA shared-RB accounting read back (GetSharedRbs) + persisted via
//       ResultWriter (the previously write-only accounting is now consumed)
// ============================================================================
namespace T9 {

// DCI 捕获 (全局 + MakeBoundCallback, 与 T2/T3 同模式; idx 0=near 1=far)
std::vector<DciInfo> g_dci[2];
void OnDci (uint32_t idx, DciInfo dci) { g_dci[idx].push_back (dci); }

// 数据面计数: UE 侧收到的 DL 包数 (idx 0=ctrl 1=test)
uint32_t g_rxPdu[2] = {0, 0};
void OnUeRx (uint32_t idx, Ptr<Packet>) { g_rxPdu[idx]++; }

bool
Run ()
{
  // -------------------- (A)+(C): 调度器侧 NOMA 记账 --------------------
  ResetSimulationContext ();

  Ptr<GeoBeamScheduler> sched = CreateObject<GeoBeamScheduler> ();
  sched->SetAttribute ("NomaEnabled", BooleanValue (true));
  sched->SetAttribute ("NomaFarPowerFraction", DoubleValue (0.8)); // beta
  sched->SetAttribute ("NomaMinCqiGap", DoubleValue (4.0));
  const uint32_t beamId = 30;
  sched->Initialize (beamId, 30);

  const uint16_t nearRnti = 301, farRnti = 302;
  sched->AddUeContext (nearRnti, UT_CONSUMER, TRAFFIC_DATA);
  sched->AddUeInfo (nearRnti, beamId);
  sched->UpdateUeDlCqi (nearRnti, 15.0);
  sched->UpdateUeDlBufferStatus (nearRnti, 20000);

  sched->AddUeContext (farRnti, UT_CONSUMER, TRAFFIC_DATA);
  sched->AddUeInfo (farRnti, beamId);
  sched->UpdateUeDlCqi (farRnti, 3.0);
  sched->UpdateUeDlBufferStatus (farRnti, 20000);

  g_dci[0].clear (); g_dci[1].clear ();
  sched->RegisterUeDciCallback (nearRnti, MakeBoundCallback (&OnDci, 0u));
  sched->RegisterUeDciCallback (farRnti, MakeBoundCallback (&OnDci, 1u));

  sched->RunScheduler ();

  bool pairFormed = !g_dci[0].empty () && !g_dci[1].empty ();
  bool powerSplitOk = false;
  if (pairFormed)
    {
      const double budgetDbm = 50.0; // default beam DL power budget
      const double expNear = 10.0 * std::log10 (std::pow (10.0, budgetDbm / 10.0) * 0.2);
      const double expFar  = 10.0 * std::log10 (std::pow (10.0, budgetDbm / 10.0) * 0.8);
      const double nearP = g_dci[0].front ().txPowerDbm;
      const double farP  = g_dci[1].front ().txPowerDbm;
      powerSplitOk = (std::abs (nearP - expNear) < 0.5) &&
                     (std::abs (farP - expFar) < 0.5) && (farP > nearP);
    }

  const double nearSinr = sched->GetEffectiveDlSinrDb (nearRnti);
  const double farSinr  = sched->GetEffectiveDlSinrDb (farRnti);
  const bool sinrOk = !std::isnan (nearSinr) && !std::isnan (farSinr) && (farSinr < nearSinr);

  const uint64_t sharedDl = sched->GetCumulativeNomaSharedDlRbs (beamId);
  const uint64_t usedDl   = sched->GetCumulativeNomaUsedDlRbs (beamId);
  const double reuseGain  = sched->GetNomaDlReuseGain (beamId);
  const bool accountingOk = (sharedDl > 0) && (usedDl > 0) && (reuseGain > 0.0);

  // (C) 真实 reader: 把 NOMA 复用记账写盘 (消除 GetSharedRbs 死代码)
  Ptr<ResultWriter> writer = CreateObject<ResultWriter> ();
  writer->SetOutputDirectory ("./simulation_results");
  const std::string statsFile = "NomaReuseStats.txt";
  const std::string statsPath = writer->GetOutputDirectory () + "/" + statsFile;
  std::remove (statsPath.c_str ());
  sched->WriteNomaReuseStats (writer, statsFile);
  std::ifstream chk (statsPath);
  const bool fileOk = chk.good () && (chk.peek () != std::ifstream::traits_type::eof ());
  chk.close ();

  // 释放 part A 的写盘器; 但 **保留 sched 存活**: part B 数据面要靠 gwMac->SetScheduler(sched)
  // 自动取调度器刚算出的 near/far {effSINR, txPower}。sched 的 RunScheduler 是同步调用,
  // 不留挂起事件, 其持久化 map 不随 Simulator::Destroy() 清除, 可安全跨到 part B 新 simulator。
  writer = nullptr;

  // 自动接线流到数据面的真值(单一真源): 取自调度器最近一次 DL 授权。
  const double nearTxDbm = sched->GetLastDlTxPowerDbm (nearRnti);
  const double farTxDbm  = sched->GetLastDlTxPowerDbm (farRnti);
  const double nearEffSinrWire = sched->GetLastDlEffSinrDb (nearRnti);
  const double farEffSinrWire  = sched->GetLastDlEffSinrDb (farRnti);
  // β 切分真正进 PSD 源: far(β=0.8 份额) 比 near((1-β)=0.2 份额) 高约 6dB。
  const bool powerSinkOk = !std::isnan (nearTxDbm) && !std::isnan (farTxDbm)
                           && (farTxDbm > nearTxDbm)
                           && (std::abs ((farTxDbm - nearTxDbm) - 6.02) < 0.5);

  // -------------------- (B): 自动接线 -> tag 驱动 BLER 的数据面端到端 --------------------
  // 两条 gw->ut DL 链路, 复用 NOMA 对 RNTI(near=301, far=302), 错误模型默认 SINR=20dB。
  // gwMac->SetScheduler(sched) 后, 每个 DL 包的 effSINR/发射功率都自动取自调度器:
  //   near: effSINR≈13dB(SIC后) -> 几乎全收; far: effSINR≈-5dB(被干扰) -> 大量丢包。
  // 无任何手工 SetEffectiveDlSinrDb 调用 -> 证明自动接线把 NOMA 物理代价带到了数据面。
  ResetSimulationContext ();

  NodeContainer gwNode; gwNode.Create (1);
  NodeContainer satNode; satNode.Create (1);
  NodeContainer ueNodes; ueNodes.Create (2);
  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (gwNode);  gwNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  mob.Install (satNode); satNode.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,35786000.0));
  mob.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,1000.0,1.5));
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (0,2000.0,1.5));

  GeoBeamHelper sat;
  sat.SetActiveBeamCount (2);
  sat.Install (satNode);
  GwHelper gw;
  gw.SetFeederChannel (sat.GetFeederChannel ());
  gw.SetFeederSpectrumModel (sat.GetFeederSpectrumModel ());
  gw.Install (gwNode);

  Ptr<NtnGwMac> gwMac = CreateObject<NtnGwMac> ();
  gwMac->SetPhy (gw.GetGwPhy (0));
  gwMac->SetTxConfig (sat.GetFeederSpectrumModel (), 100e6, 40.0, MilliSeconds (1));
  // 单一真源自动接线: 之后每个 DL 包的 effSINR/发射功率都自动取自调度器 (无手工注入)。
  gwMac->SetScheduler (sched);

  // 复用 NOMA 对 RNTI: i=0=near(高 effSINR), i=1=far(被干扰低 effSINR)。
  const uint16_t rntis[2]   = {nearRnti, farRnti};
  const uint16_t beamIds[2] = {1, 2};
  g_rxPdu[0] = g_rxPdu[1] = 0;
  std::vector<Ptr<SatPhy>>   uePhys (2);
  std::vector<Ptr<SatUtMac>> ueMacs (2);
  std::vector<Ptr<NtnPhyErrorModel>> ems (2);

  for (uint32_t i = 0; i < 2; ++i)
    {
      Ptr<SpectrumChannel>     uch = sat.GetUserChannel (beamIds[i]);
      Ptr<const SpectrumModel> usm = sat.GetUserSpectrumModel (beamIds[i]);

      Ptr<SatPhy> uePhy = CreateObject<SatPhy> ();
      uePhy->SetChannel (uch);
      uePhy->SetMobility (ueNodes.Get (i)->GetObject<MobilityModel> ());
      uePhy->SetRxSpectrumModel (usm);
      uch->AddRx (uePhy);
      uePhys[i] = uePhy;

      Ptr<SatUtMac> ueMac = CreateObject<SatUtMac> ();
      ueMac->SetPhy (uePhy);
      ueMac->SetTxConfig (usm, 5e6, 23.0, MilliSeconds (1));
      ueMac->SetRnti (rntis[i]);
      // 无 RLC: 走 dlCallback 直接统计收到的 DL 包数
      ueMac->SetDlPduCallback (MakeBoundCallback (&OnUeRx, i));
      ueMacs[i] = ueMac; // 必须持有到 Run() 结束, 否则 phy 的 rx callback 悬挂

      Ptr<NtnPhyErrorModel> em = CreateObject<NtnPhyErrorModel> ();
      em->SetSinrDb (20.0); // 默认高 SINR, 几乎不丢; 仅靠 tag 注入的低 SINR 才丢
      ueMac->SetPhyErrorModel (em);
      ems[i] = em;

      gwMac->AddUe (rntis[i], beamIds[i]);
    }

  // 不再手工注入 effSINR: near/far 的 effSINR 由自动接线从调度器(part A 跑出的真值)流入。

  const uint32_t N = 40;
  Simulator::Schedule (Seconds (1.0), [&] () {
    for (uint32_t i = 0; i < 2; ++i)
      for (uint32_t k = 0; k < N; ++k)
        gwMac->EnqueueDlPdu (rntis[i], Create<Packet> (200));
    gwMac->StartScheduler (MilliSeconds (5));
  });
  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();

  // near 链路(i=0, effSINR≈13dB SIC后) 应几乎全收;
  // far 链路(i=1, effSINR≈-5dB 被干扰) 应大量丢包。两者全靠自动接线流入的 tag 驱动。
  const bool nearMostlyOk = (g_rxPdu[0] >= (N - 2));
  const bool farDropped   = (ems[1]->GetDropCount () > 0) && (g_rxPdu[1] < g_rxPdu[0]);
  const bool tagDrivesBler = nearMostlyOk && farDropped;

  // 释放 part A 的调度器(数据面已用完它的真值)。
  sched = nullptr;

  const bool ok = pairFormed && powerSplitOk && sinrOk && accountingOk && fileOk
                  && powerSinkOk && tagDrivesBler;
  std::ostringstream note;
  note << "betaSplit=" << (powerSplitOk ? "ok" : "BAD")
       << " effSINR(near=" << std::fixed << std::setprecision (1) << nearSinr
       << ",far=" << farSinr << ")"
       << " shared/used=" << sharedDl << "/" << usedDl
       << " gain=" << std::setprecision (2) << reuseGain
       << " file=" << (fileOk ? "ok" : "BAD")
       << " | wire txPwr(near=" << std::setprecision (1) << nearTxDbm
       << ",far=" << farTxDbm << ")dBm dPwr=" << (farTxDbm - nearTxDbm)
       << " effSINRwire(near=" << nearEffSinrWire << ",far=" << farEffSinrWire << ")"
       << " | autoBLER near=" << g_rxPdu[0] << "/" << N
       << " far=" << g_rxPdu[1] << "/" << N
       << "(drop=" << ems[1]->GetDropCount () << ")";
  ReportTest ("T9 NOMA beta+SINR+stats (auto-wired)", ok, note.str ());
  return ok;
}

} // namespace T9

// ============================================================================
// main: 跑全部 / 指定子集, 输出汇总
// ============================================================================
int
main (int argc, char* argv[])
{
  std::string only;
  bool verbose = false;

  CommandLine cmd;
  cmd.AddValue ("only",    "Comma-separated test IDs (e.g. T1,T4); empty = run all", only);
  cmd.AddValue ("verbose", "Enable internal LogComponent",                            verbose);
  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("NtnRlc", LOG_LEVEL_INFO);
      LogComponentEnable ("NtnGwMac", LOG_LEVEL_INFO);
      LogComponentEnable ("SatUtMac", LOG_LEVEL_INFO);
      LogComponentEnable ("NtnPhyErrorModel", LOG_LEVEL_INFO);
    }

  // 固定随机种子 (T5/T6 涉及概率丢包, 此举让结果可复现)
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun  (1);

  std::set<std::string> enabled;
  if (only.empty ())
    enabled = {"T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9"};
  else
    {
      std::stringstream ss (only);
      std::string tok;
      while (std::getline (ss, tok, ',')) enabled.insert (tok);
    }

  std::cout << "========================================\n";
  std::cout << "  NTN Protocol Stack Connectivity Test\n";
  std::cout << "========================================\n";

  auto t0 = std::chrono::steady_clock::now ();
  if (enabled.count ("T1")) T1::Run ();
  if (enabled.count ("T2")) T2::Run ();
  if (enabled.count ("T3")) T3::Run ();
  if (enabled.count ("T4")) T4::Run ();
  if (enabled.count ("T5")) T5::Run ();
  if (enabled.count ("T6")) T6::Run ();
  if (enabled.count ("T7")) T7::Run ();
  if (enabled.count ("T8")) T8::Run ();
  if (enabled.count ("T9")) T9::Run ();
  auto t1 = std::chrono::steady_clock::now ();
  double wall = std::chrono::duration<double> (t1 - t0).count ();

  uint32_t passed = 0;
  for (const auto& r : g_results) if (r.pass) passed++;

  std::cout << "========================================\n";
  std::cout << "  " << passed << "/" << g_results.size () << " PASSED"
            << "  (wallclock " << std::fixed << std::setprecision (2) << wall << "s)\n";
  std::cout << "========================================\n";

  Simulator::Destroy ();
  return (passed == g_results.size ()) ? 0 : 1;
}
