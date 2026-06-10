#include "ns3/core-module.h"
#include "ns3/resource-manager.h"
#include "ns3/admit-control.h"
#include "ns3/geo-beam-scheduler.h"
#include "ns3/ntn-gw-rrc.h"
#include "ns3/frequency-reuse.h"
#include "ns3/sat-mac-common.h"
#include "ns3/nr-amc.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

bool g_allPassed = true;
uint32_t g_passCount = 0;
uint32_t g_failCount = 0;

void
PrintSection (const std::string& title)
{
  std::cout << "\n=== " << title << " ===" << std::endl;
}

void
ExpectTrue (bool condition, const std::string& checkName, const std::string& detail = "")
{
  if (condition)
    {
      ++g_passCount;
      std::cout << "[PASS] " << checkName;
      if (!detail.empty ())
        {
          std::cout << " | " << detail;
        }
      std::cout << std::endl;
      return;
    }

  g_allPassed = false;
  ++g_failCount;
  std::cout << "[FAIL] " << checkName;
  if (!detail.empty ())
    {
      std::cout << " | " << detail;
    }
  std::cout << std::endl;
}

template <typename T, typename U>
void
ExpectEq (const T& actual, const U& expected, const std::string& checkName)
{
  std::ostringstream oss;
  oss << "actual=" << actual << " expected=" << expected;
  ExpectTrue (actual == expected, checkName, oss.str ());
}

std::string
ToString (AdmitDecision decision)
{
  switch (decision)
    {
    case AdmitDecision::ADMIT_OK:
      return "ADMIT_OK";
    case AdmitDecision::ADMIT_REJECTED:
      return "ADMIT_REJECTED";
    case AdmitDecision::ADMIT_REDIRECT:
      return "ADMIT_REDIRECT";
    case AdmitDecision::ADMIT_QUEUE:
      return "ADMIT_QUEUE";
    default:
      return "UNKNOWN";
    }
}

struct DciCapture
{
  explicit DciCapture (uint16_t ueId)
    : m_ueId (ueId)
  {
  }

  void
  OnDci (DciInfo dci)
  {
    m_dcis.push_back (dci);
  }

  uint16_t m_ueId;
  std::vector<DciInfo> m_dcis;
};

struct RoundGrantCapture
{
  RoundGrantCapture (uint16_t ueId, uint32_t beamId, bool isVoice)
    : m_ueId (ueId),
      m_beamId (beamId),
      m_isVoice (isVoice)
  {
  }

  void
  ResetRound ()
  {
    m_roundDlRb = 0;
    m_roundUlRb = 0;
    m_roundDlGrantCount = 0;
    m_roundUlGrantCount = 0;
  }

  void
  OnDci (DciInfo dci)
  {
    if (dci.isUplinkGrant)
      {
        m_roundUlRb += dci.rbAllocation;
        ++m_roundUlGrantCount;
        return;
      }

    m_roundDlRb += dci.rbAllocation;
    ++m_roundDlGrantCount;
  }

  uint16_t m_ueId;
  uint32_t m_beamId;
  bool m_isVoice;
  uint32_t m_roundDlRb = 0;
  uint32_t m_roundUlRb = 0;
  uint32_t m_roundDlGrantCount = 0;
  uint32_t m_roundUlGrantCount = 0;
};

struct RarCapture
{
  void
  OnRar (const RarMessage& rar)
  {
    m_rars.push_back (rar);
  }

  void
  OnMsg4 (const RrcSetupMessage&)
  {
  }

  std::vector<RarMessage> m_rars;
};

void
ExpectAdmitDecision (AdmitDecision actual, AdmitDecision expected, const std::string& checkName)
{
  std::ostringstream oss;
  oss << "actual=" << ToString (actual) << " expected=" << ToString (expected);
  ExpectTrue (actual == expected, checkName, oss.str ());
}

// 参考 AMC: 复刻 GeoBeamScheduler::EstimateBytesPerRb 走的 NrAmc DL/UL 通路,
// 让测试侧能算出与调度器一致的"每 RB 字节数"(避免硬编码常量与真实值脱节)。
Ptr<NrAmc>
GetReferenceAmc (bool isUplink)
{
  static Ptr<NrAmc> dlAmc = [] () {
    Ptr<NrAmc> amc = CreateObject<NrAmc> ();
    amc->SetDlMode ();
    return amc;
  } ();
  static Ptr<NrAmc> ulAmc = [] () {
    Ptr<NrAmc> amc = CreateObject<NrAmc> ();
    amc->SetUlMode ();
    return amc;
  } ();
  return isUplink ? ulAmc : dlAmc;
}

uint8_t
GetReferenceMcsFromCqi (double cqi, bool isUplink)
{
  const uint8_t cqiIndex =
    static_cast<uint8_t> (std::clamp (std::lround (cqi), 0l, 15l));
  return GetReferenceAmc (isUplink)->GetMcsFromCqi (cqiIndex);
}

double
GetReferenceBytesPerRb (double cqi, bool isUplink)
{
  const uint8_t mcs = GetReferenceMcsFromCqi (cqi, isUplink);
  return std::max (1.0,
                   static_cast<double> (GetReferenceAmc (isUplink)->GetPayloadSize (mcs, 1)));
}

} // namespace

int
main (int argc, char* argv[])
{
  bool verbose = false;
  CommandLine cmd;
  cmd.AddValue ("verbose", "Enable component logs while running the validation example.", verbose);
  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("GeoBeamScheduler", LOG_LEVEL_INFO);
      LogComponentEnable ("AdmitControl", LOG_LEVEL_INFO);
      LogComponentEnable ("ResourceManager", LOG_LEVEL_INFO);
    }

  PrintSection ("Test 1: ResourceManager beam budget accounting");

  Ptr<ResourceManager> resourceManager = CreateObject<ResourceManager> ();

  resourceManager->ResetBeamAllocation (1, false);
  ExpectEq (resourceManager->AllocateSpectrum (1, 10, false), 10u,
            "DL first allocation should grant 10 RB");
  ExpectEq (resourceManager->AllocateSpectrum (1, 20, false), 15u,
            "DL second allocation should be clipped by shared 25 RB budget");
  ExpectEq (resourceManager->GetRemainingRbs (1, false), 0u,
            "DL remaining RB should reach 0 after exhausting one beam");

  resourceManager->ResetBeamAllocation (1, true);
  ExpectEq (resourceManager->AllocateSpectrum (1, 12, true), 12u,
            "UL first allocation should grant 12 RB");
  ExpectEq (resourceManager->AllocateSpectrum (1, 20, true), 13u,
            "UL second allocation should share the same 25 RB pool");
  ExpectEq (resourceManager->GetRemainingRbs (1, true), 0u,
            "UL remaining RB should reach 0 after exhausting one beam");

  const double portablePower = resourceManager->AdjustUtTxPower (UT_PORTABLE, 5, 190.0);
  const double consumerPower = resourceManager->AdjustUtTxPower (UT_CONSUMER, 5, 190.0);
  const uint32_t portableMaxNonClippedRbs = resourceManager->GetMaxPowerLimitedUlRbs (UT_PORTABLE, 190.0);
  const uint32_t consumerMaxNonClippedRbs = resourceManager->GetMaxPowerLimitedUlRbs (UT_CONSUMER, 190.0);
  ExpectTrue (portablePower <= 63.0 + 1e-6,
              "Portable UE Tx power should be capped by 33 dBW / 63 dBm");
  ExpectTrue (consumerPower <= 50.0 + 1e-6,
              "Consumer UE Tx power should be capped by 20 dBW / 50 dBm");
  ExpectTrue (portablePower >= consumerPower,
              "Portable UE power cap should not be lower than consumer UE when portable uses 33 dBW");
  ExpectEq (portableMaxNonClippedRbs, 125u,
            "Portable UE should sustain a much larger non-clipped UL RB count at 190 dB path loss");
  ExpectEq (consumerMaxNonClippedRbs, 6u,
            "Consumer UE should be power-limited to 6 non-clipped UL RBs at 190 dB path loss");

  PrintSection ("Test 2: AdmitControl reservation and admission policy");

  Ptr<AdmitControl> admitControl = CreateObject<AdmitControl> ();
  admitControl->SetResourceManager (resourceManager);
  admitControl->SetBeamTotalRbs (1, 25);
  admitControl->SetBeamTotalRbs (2, 25);
  admitControl->SetBeamTotalRbs (3, 25);

  resourceManager->ResetBeamAllocation (1, false);
  resourceManager->ResetBeamAllocation (2, false);

  ExpectAdmitDecision (
    admitControl->CanAdmitUe (1,
                              ServicePriority::PRIORITY_VOICE,
                              UT_PORTABLE,
                              TRAFFIC_VOICE,
                              2),
    AdmitDecision::ADMIT_OK,
    "Voice UE should be admitted on an empty beam");

  resourceManager->AllocateSpectrum (1, 24, false);
  ExpectAdmitDecision (
    admitControl->CanAdmitUe (1,
                              ServicePriority::PRIORITY_DATA,
                              UT_CONSUMER,
                              TRAFFIC_DATA,
                              1),
    AdmitDecision::ADMIT_QUEUE,
    "Data UE should be queued when only opportunistic (emergency-reserved) headroom remains");

  ExpectAdmitDecision (
    admitControl->CanAdmitUe (1,
                              ServicePriority::PRIORITY_EMERGENCY,
                              UT_PORTABLE,
                              TRAFFIC_DATA,
                              1),
    AdmitDecision::ADMIT_OK,
    "Emergency UE should still be admitted within reserved RB");

  resourceManager->ResetBeamAllocation (2, false);
  resourceManager->AllocateSpectrum (2, 22, false);
  ExpectAdmitDecision (
    admitControl->CanAdmitUe (2,
                              ServicePriority::PRIORITY_VOICE,
                              UT_PORTABLE,
                              TRAFFIC_VOICE,
                              1),
    AdmitDecision::ADMIT_QUEUE,
    "Voice UE should be queued when admission threshold would be exceeded");

  resourceManager->ResetBeamAllocation (2, false);
  resourceManager->AllocateSpectrum (2, 21, false);
  resourceManager->ResetBeamAllocation (3, false);
  ExpectAdmitDecision (
    admitControl->CanAdmitUe (2,
                              ServicePriority::PRIORITY_DATA,
                              UT_CONSUMER,
                              TRAFFIC_DATA,
                              1),
    AdmitDecision::ADMIT_OK,
    "Regular data UE should still be admitted with 1 RB headroom");
  ExpectAdmitDecision (
    admitControl->CanAdmitUe (2,
                              ServicePriority::PRIORITY_DATA,
                              UT_CONSUMER,
                              TRAFFIC_HIGH_CAPACITY,
                              1),
    AdmitDecision::ADMIT_QUEUE,
    "High-capacity data UE should queue when its amplified RB demand would borrow protected headroom");

  PrintSection ("Test 3: GeoBeamScheduler DL slicing and 25 RB sharing");

  Ptr<GeoBeamScheduler> dlScheduler = CreateObject<GeoBeamScheduler> ();
  dlScheduler->Initialize (7, 30);

  const uint16_t voiceRnti = 101;
  const uint16_t dataRnti = 102;

  dlScheduler->AddUeContext (voiceRnti, UT_PORTABLE, TRAFFIC_VOICE);
  dlScheduler->AddUeInfo (voiceRnti, 7);
  dlScheduler->UpdateUeCsi (voiceRnti, 10.0);
  dlScheduler->UpdateUeDlBufferStatus (voiceRnti, 1000);

  dlScheduler->AddUeContext (dataRnti, UT_CONSUMER, TRAFFIC_DATA);
  dlScheduler->AddUeInfo (dataRnti, 7);
  dlScheduler->UpdateUeCsi (dataRnti, 10.0);
  dlScheduler->UpdateUeDlBufferStatus (dataRnti, 10000);

  DciCapture voiceDlCapture (voiceRnti);
  DciCapture dataDlCapture (dataRnti);
  dlScheduler->RegisterUeDciCallback (voiceRnti, MakeCallback (&DciCapture::OnDci, &voiceDlCapture));
  dlScheduler->RegisterUeDciCallback (dataRnti, MakeCallback (&DciCapture::OnDci, &dataDlCapture));

  dlScheduler->RunScheduler ();

  ExpectEq (voiceDlCapture.m_dcis.size (), 1u,
            "Voice UE should receive exactly one DL DCI");
  ExpectEq (dataDlCapture.m_dcis.size (), 1u,
            "Data UE should receive exactly one DL DCI");

  if (!voiceDlCapture.m_dcis.empty () && !dataDlCapture.m_dcis.empty ())
    {
      const DciInfo& voiceDci = voiceDlCapture.m_dcis.front ();
      const DciInfo& dataDci = dataDlCapture.m_dcis.front ();

      ExpectTrue (!voiceDci.isUplinkGrant && !dataDci.isUplinkGrant,
                  "DL scheduler should emit downlink DCIs in RunScheduler()");
      ExpectEq (voiceDci.rbAllocation, 2u,
                "Voice queue should consume its reserved 2 RB budget");
      ExpectEq (dataDci.rbAllocation, 17u,
                "Data queue should consume the remaining 17 RB after PRACH reserve and voice slice");
      ExpectEq (voiceDci.rbAllocation + dataDci.rbAllocation, 19u,
                "Voice and data UEs should share the 19 RB traffic budget after reserving 6 PRACH RBs");
    }

  PrintSection ("Test 4: GeoBeamScheduler UL scheduler sharing");

  Ptr<GeoBeamScheduler> ulScheduler = CreateObject<GeoBeamScheduler> ();
  ulScheduler->Initialize (9, 30);

  const uint16_t ulRnti1 = 201;
  const uint16_t ulRnti2 = 202;

  ulScheduler->AddUeContext (ulRnti1, UT_PORTABLE, TRAFFIC_VOICE);
  ulScheduler->AddUeInfo (ulRnti1, 9);
  ulScheduler->UpdateUeCsi (ulRnti1, 10.0);
  ulScheduler->ReceiveBsr ({ulRnti1, 0, 2400});

  ulScheduler->AddUeContext (ulRnti2, UT_CONSUMER, TRAFFIC_DATA);
  ulScheduler->AddUeInfo (ulRnti2, 9);
  ulScheduler->UpdateUeCsi (ulRnti2, 10.0);
  ulScheduler->ReceiveBsr ({ulRnti2, 0, 3200});

  DciCapture ulCapture1 (ulRnti1);
  DciCapture ulCapture2 (ulRnti2);
  ulScheduler->RegisterUeDciCallback (ulRnti1, MakeCallback (&DciCapture::OnDci, &ulCapture1));
  ulScheduler->RegisterUeDciCallback (ulRnti2, MakeCallback (&DciCapture::OnDci, &ulCapture2));

  NrMacSchedSapProvider::SchedUlTriggerReqParameters ulTrigger;
  ulScheduler->DoSchedUlTriggerReq (ulTrigger);

  ExpectEq (ulCapture1.m_dcis.size (), 1u,
            "First UL UE should receive one UL grant");
  ExpectEq (ulCapture2.m_dcis.size (), 1u,
            "Second UL UE should receive one UL grant");

  if (!ulCapture1.m_dcis.empty () && !ulCapture2.m_dcis.empty ())
    {
      const DciInfo& ulDci1 = ulCapture1.m_dcis.front ();
      const DciInfo& ulDci2 = ulCapture2.m_dcis.front ();

      ExpectTrue (ulDci1.isUplinkGrant && ulDci2.isUplinkGrant,
                  "UL path should emit uplink grants");
      ExpectEq (ulDci1.rbAllocation, 2u,
                "Voice UL UE should consume its reserved 2 RB budget");
      ExpectEq (ulDci2.rbAllocation, 6u,
                "Consumer-data UL UE should be capped by its power-feasible 6 RB limit");
      ExpectEq (ulDci1.rbAllocation + ulDci2.rbAllocation, 8u,
                "UL scheduler should not over-allocate RBs beyond the consumer UE power cap");
      ExpectTrue (ulDci1.txPowerDbm > 0.0 && ulDci2.txPowerDbm > 0.0,
                  "UL grants should carry non-zero transmit power");
    }

  PrintSection ("Test 5: UL grants should not over-issue against stale GEO BSR");

  Ptr<GeoBeamScheduler> ulGrantWindowScheduler = CreateObject<GeoBeamScheduler> ();
  ulGrantWindowScheduler->Initialize (10, 30);

  const uint16_t ulWindowRnti = 203;
  ulGrantWindowScheduler->AddUeContext (ulWindowRnti, UT_CONSUMER, TRAFFIC_DATA);
  ulGrantWindowScheduler->AddUeInfo (ulWindowRnti, 10);
  ulGrantWindowScheduler->UpdateUeCsi (ulWindowRnti, 15.0);
  ulGrantWindowScheduler->ReceiveBsr ({ulWindowRnti, 0, 5000});

  DciCapture ulWindowCapture (ulWindowRnti);
  ulGrantWindowScheduler->RegisterUeDciCallback (ulWindowRnti,
                                                 MakeCallback (&DciCapture::OnDci, &ulWindowCapture));

  NrMacSchedSapProvider::SchedUlTriggerReqParameters ulWindowTrigger;
  for (uint32_t round = 0; round < 10; ++round)
    {
      ulGrantWindowScheduler->DoSchedUlTriggerReq (ulWindowTrigger);
    }

  ExpectTrue (!ulWindowCapture.m_dcis.empty (),
              "Stale-BSR GEO test should still issue at least one UL grant");
  // 用与调度器一致的 NrAmc UL 通路算每 RB 字节数 (旧版硬编码 133.3 是 DL 口径,
  // 与真实 UL EstimateBytesPerRb 严重偏离, 导致 backlog 覆盖所需 RB 被低估)。
  const double windowBytesPerRb = GetReferenceBytesPerRb (15.0, true);
  const uint32_t expectedGrantRbsUpperBound =
    static_cast<uint32_t> (std::ceil (5000.0 / windowBytesPerRb));
  const uint32_t consumerPerGrantUpperBound = 6u;
  uint32_t totalGrantedRbs = 0;
  for (const auto& dci : ulWindowCapture.m_dcis)
    {
      totalGrantedRbs += dci.rbAllocation;
    }

  ExpectTrue (totalGrantedRbs <= expectedGrantRbsUpperBound + 1,
              "UL scheduler should stop granting once in-flight capacity covers the stale BSR backlog",
              "grantedRbs=" + std::to_string (totalGrantedRbs) +
              " upperBound=" + std::to_string (expectedGrantRbsUpperBound + 1));
  ExpectTrue (ulWindowCapture.m_dcis.size () <=
                static_cast<size_t> (std::ceil (static_cast<double> (expectedGrantRbsUpperBound) /
                                                consumerPerGrantUpperBound)) + 1,
              "5000-byte stale GEO BSR should only trigger the finite number of grants needed to cover the known backlog",
              "grantCount=" + std::to_string (ulWindowCapture.m_dcis.size ()));

  PrintSection ("Test 6: MeasReport should not overwrite established CQI (via NtnGwRrc)");

  // 架构适配: ReceiveMeasReport 已从调度器迁到 NtnGwRrc。NtnGwRrc::ReceiveMeasReport
  // 会用上报里的 serving/bestNeighbor 度量经 UpdateUeCsi 喂回调度器, 并不再使用 legacy
  // rsrp 字段。本测试断言: 一份上报当前服务波束、且 servingMetric 与既有 CQI 一致的
  // MeasReport 不会污染调度器已建立的 DL CQI(13.0), DL MCS 仍按 CQI=13 选出。
  Ptr<GeoBeamScheduler> measScheduler = CreateObject<GeoBeamScheduler> ();
  measScheduler->Initialize (11, 30);

  const uint16_t measRnti = 301;
  measScheduler->AddUeContext (measRnti, UT_CONSUMER, TRAFFIC_DATA);
  measScheduler->AddUeInfo (measRnti, 11);
  measScheduler->UpdateUeCsi (measRnti, 13.0);
  measScheduler->UpdateUeDlBufferStatus (measRnti, 4000);

  DciCapture measCapture (measRnti);
  measScheduler->RegisterUeDciCallback (measRnti, MakeCallback (&DciCapture::OnDci, &measCapture));

  Ptr<NtnGwRrc> measRrc = CreateObject<NtnGwRrc> ();
  measRrc->SetScheduler (measScheduler);

  MeasReport report;
  report.ueId = measRnti;
  report.bestBeamId = 11;
  report.rsrp = -85.0;   // legacy 字段, 新架构忽略
  report.servingObject = {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, 11};
  report.servingMetric = 13.0;  // 与既有 CQI 一致的服务波束度量
  report.position = Vector (120.0, 40.0, 0.0);
  measRrc->ReceiveMeasReport (report);
  measScheduler->RunScheduler ();

  ExpectEq (measCapture.m_dcis.size (), 1u,
            "MeasReport test UE should still receive one DL DCI");
  if (!measCapture.m_dcis.empty ())
    {
      const DciInfo& measDci = measCapture.m_dcis.front ();
      ExpectEq (measDci.mcs, GetReferenceMcsFromCqi (13.0, false),
                "Serving-metric MeasReport should keep CQI=13 driven DL MCS (legacy RSRP ignored)");
    }

  PrintSection ("Test 7: PRACH reserve should reduce DL traffic budget");

  Ptr<GeoBeamScheduler> prachScheduler = CreateObject<GeoBeamScheduler> ();
  prachScheduler->Initialize (12, 30);

  const uint16_t prachDataRnti = 401;
  prachScheduler->AddUeContext (prachDataRnti, UT_CONSUMER, TRAFFIC_DATA);
  prachScheduler->AddUeInfo (prachDataRnti, 12);
  prachScheduler->UpdateUeCsi (prachDataRnti, 10.0);
  prachScheduler->UpdateUeDlBufferStatus (prachDataRnti, 20000);

  DciCapture prachCapture (prachDataRnti);
  prachScheduler->RegisterUeDciCallback (prachDataRnti, MakeCallback (&DciCapture::OnDci, &prachCapture));
  prachScheduler->RunScheduler ();

  ExpectEq (prachCapture.m_dcis.size (), 1u,
            "Single data UE should receive exactly one DL DCI under PRACH reserve");
  if (!prachCapture.m_dcis.empty ())
    {
      const DciInfo& prachDci = prachCapture.m_dcis.front ();
      ExpectEq (prachDci.rbAllocation, 19u,
                "Single data UE should see only 19 RB traffic budget after reserving 6 PRACH RBs");
    }

  PrintSection ("Test 8: UL multi-user contention should not exceed one beam budget");

  Ptr<GeoBeamScheduler> ulContentionScheduler = CreateObject<GeoBeamScheduler> ();
  ulContentionScheduler->Initialize (13, 30);

  const uint16_t contentionVoiceRnti = 501;
  const uint16_t contentionDataRntiFast = 502;
  const uint16_t contentionDataRntiSlow = 503;

  ulContentionScheduler->AddUeContext (contentionVoiceRnti, UT_PORTABLE, TRAFFIC_VOICE);
  ulContentionScheduler->AddUeInfo (contentionVoiceRnti, 13);
  ulContentionScheduler->UpdateUeCsi (contentionVoiceRnti, 10.0);
  ulContentionScheduler->ReceiveBsr ({contentionVoiceRnti, 0, 2400});

  ulContentionScheduler->AddUeContext (contentionDataRntiFast, UT_CONSUMER, TRAFFIC_DATA);
  ulContentionScheduler->AddUeInfo (contentionDataRntiFast, 13);
  ulContentionScheduler->UpdateUeCsi (contentionDataRntiFast, 12.0);
  ulContentionScheduler->ReceiveBsr ({contentionDataRntiFast, 0, 8000});

  ulContentionScheduler->AddUeContext (contentionDataRntiSlow, UT_CONSUMER, TRAFFIC_DATA);
  ulContentionScheduler->AddUeInfo (contentionDataRntiSlow, 13);
  ulContentionScheduler->UpdateUeCsi (contentionDataRntiSlow, 4.0);
  ulContentionScheduler->ReceiveBsr ({contentionDataRntiSlow, 0, 8000});

  DciCapture contentionVoiceCapture (contentionVoiceRnti);
  DciCapture contentionFastCapture (contentionDataRntiFast);
  DciCapture contentionSlowCapture (contentionDataRntiSlow);
  ulContentionScheduler->RegisterUeDciCallback (contentionVoiceRnti, MakeCallback (&DciCapture::OnDci, &contentionVoiceCapture));
  ulContentionScheduler->RegisterUeDciCallback (contentionDataRntiFast, MakeCallback (&DciCapture::OnDci, &contentionFastCapture));
  ulContentionScheduler->RegisterUeDciCallback (contentionDataRntiSlow, MakeCallback (&DciCapture::OnDci, &contentionSlowCapture));

  NrMacSchedSapProvider::SchedUlTriggerReqParameters contentionUlTrigger;
  ulContentionScheduler->DoSchedUlTriggerReq (contentionUlTrigger);

  ExpectEq (contentionVoiceCapture.m_dcis.size (), 1u,
            "UL contention voice UE should receive one grant");
  ExpectEq (contentionFastCapture.m_dcis.size (), 1u,
            "Higher-metric data UE should receive one grant");
  ExpectEq (contentionSlowCapture.m_dcis.size (), 1u,
            "Lower-metric data UE can still receive a grant when power caps leave residual beam budget");

  if (!contentionVoiceCapture.m_dcis.empty () &&
      !contentionFastCapture.m_dcis.empty () &&
      !contentionSlowCapture.m_dcis.empty ())
    {
      const DciInfo& voiceContentionDci = contentionVoiceCapture.m_dcis.front ();
      const DciInfo& fastContentionDci = contentionFastCapture.m_dcis.front ();
      const DciInfo& slowContentionDci = contentionSlowCapture.m_dcis.front ();
      ExpectEq (voiceContentionDci.rbAllocation, 2u,
                "Voice UE should still keep its 2 RB UL reservation under contention");
      ExpectEq (fastContentionDci.rbAllocation, 6u,
                "Best consumer-data UE should still be capped by the 6 RB power-feasible limit");
      ExpectEq (slowContentionDci.rbAllocation, 6u,
                "Lower-metric consumer-data UE should also be capped by the 6 RB power-feasible limit");
      ExpectEq (voiceContentionDci.rbAllocation + fastContentionDci.rbAllocation + slowContentionDci.rbAllocation, 14u,
                "UL contention scheduling should reflect the sum of power-feasible grants actually issued");
    }

  PrintSection ("Test 8: UL/DL buffers should remain separated");

  Ptr<GeoBeamScheduler> splitBufferScheduler = CreateObject<GeoBeamScheduler> ();
  splitBufferScheduler->Initialize (14, 30);

  const uint16_t splitDlOnlyRnti = 601;
  const uint16_t splitUlOnlyRnti = 602;

  splitBufferScheduler->AddUeContext (splitDlOnlyRnti, UT_CONSUMER, TRAFFIC_DATA);
  splitBufferScheduler->AddUeInfo (splitDlOnlyRnti, 14);
  splitBufferScheduler->UpdateUeCsi (splitDlOnlyRnti, 10.0);
  splitBufferScheduler->UpdateUeDlBufferStatus (splitDlOnlyRnti, 5000);
  splitBufferScheduler->ReceiveBsr ({splitDlOnlyRnti, 0, 2400});

  splitBufferScheduler->AddUeContext (splitUlOnlyRnti, UT_CONSUMER, TRAFFIC_DATA);
  splitBufferScheduler->AddUeInfo (splitUlOnlyRnti, 14);
  splitBufferScheduler->UpdateUeCsi (splitUlOnlyRnti, 10.0);
  splitBufferScheduler->ReceiveBsr ({splitUlOnlyRnti, 0, 1800});

  Ptr<Packet> splitUlPdu = Create<Packet> (1200);
  GenericUlMacHeader splitUlHeader;
  splitUlHeader.SetRnti (splitDlOnlyRnti);
  splitUlHeader.SetPayloadBytes (1200);
  splitUlHeader.SetTransmissionTime (Simulator::Now ());
  splitUlPdu->AddHeader (splitUlHeader);
  splitBufferScheduler->ReceiveUlMacPdu (splitUlPdu);

  DciCapture splitDlCapture (splitDlOnlyRnti);
  DciCapture splitUlOnlyCapture (splitUlOnlyRnti);
  splitBufferScheduler->RegisterUeDciCallback (splitDlOnlyRnti,
                                               MakeCallback (&DciCapture::OnDci, &splitDlCapture));
  splitBufferScheduler->RegisterUeDciCallback (splitUlOnlyRnti,
                                               MakeCallback (&DciCapture::OnDci, &splitUlOnlyCapture));
  splitBufferScheduler->RunScheduler ();

  ExpectEq (splitDlCapture.m_dcis.size (), 1u,
            "DL scheduler should still serve UE whose DL buffer is non-zero after UL delivery");
  ExpectEq (splitUlOnlyCapture.m_dcis.size (), 0u,
            "UE with only UL buffer and zero DL buffer should not enter DL scheduling");

  PrintSection ("Test 9: A3 MeasReport via NtnGwRrc should trigger beam handover without corrupting context");

  // 架构适配: 切换"决策"已迁到 NtnGwRrc。这里构造 NtnGwRrc + AdmitControl + ResourceManager,
  // 投递一份 A3 上报 (serving=SAT_BEAM:21, bestNeighbor=SAT_BEAM:22), 由 NtnGwRrc::ReceiveMeasReport
  // 做准入并经 scheduler->ExecuteHandover 把 UE 从波束 21 搬到波束 22, 同时保留 DL/UL 待发缓存。
  Ptr<GeoBeamScheduler> handoverScheduler = CreateObject<GeoBeamScheduler> ();
  handoverScheduler->Initialize (21, 30);

  Ptr<ResourceManager> handoverRm = CreateObject<ResourceManager> ();
  Ptr<AdmitControl> handoverAdmit = CreateObject<AdmitControl> ();
  handoverAdmit->SetResourceManager (handoverRm);
  handoverAdmit->SetBeamTotalRbs (21, 25);
  handoverAdmit->SetBeamTotalRbs (22, 25);
  handoverRm->ResetBeamAllocation (21, false);
  handoverRm->ResetBeamAllocation (22, false);
  handoverScheduler->SetAdmitControl (handoverAdmit);

  Ptr<NtnGwRrc> handoverRrc = CreateObject<NtnGwRrc> ();
  handoverRrc->SetScheduler (handoverScheduler);
  handoverRrc->SetAdmitControl (handoverAdmit);

  const uint16_t handoverRnti = 701;
  handoverScheduler->AddUeContext (handoverRnti, UT_CONSUMER, TRAFFIC_DATA);
  handoverScheduler->AddUeInfo (handoverRnti, 21);
  handoverScheduler->UpdateUeCsi (handoverRnti, 11.0);
  // 新架构对切换也走准入控制: 目标波束须能保证该 UE 的 RB 需求。EstimateRequiredRbs
  // 现走 NrAmc 同主路径口径(已删除 latestCqi*2.5 魔数), 整缓存折算出的 raw RB 远大于
  // 波束预算, 因此对 DATA UE 收敛到准入闸上限 m_dataAdmitRbCap(=4), 低于空波束的保证
  // 余量, 切换被准入。缓存值仍要被切换完整保留。
  handoverScheduler->UpdateUeDlBufferStatus (handoverRnti, 500);
  handoverScheduler->ReceiveBsr ({handoverRnti, 0, 400});

  MeasReport handoverReport;
  handoverReport.ueId = handoverRnti;
  handoverReport.bestBeamId = 22;
  handoverReport.rsrp = -82.0;   // legacy 字段, 新架构忽略
  handoverReport.eventType = MeasEventType::MEAS_EVENT_A3;
  handoverReport.servingObject = {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, 21};
  // 注意: NtnGwRrc::ReceiveMeasReport 把 serving/bestNeighbor 度量当作 CQI 喂给
  // scheduler->UpdateUeCsi, 因此这里给 CQI 量纲 (1-15) 而非 dBm RSRP。邻区(目标)CQI 更优。
  handoverReport.servingMetric = 8.0;
  handoverReport.bestNeighborObject = {ServiceObjectType::SERVICE_OBJECT_SAT_BEAM, 22};
  handoverReport.bestNeighborMetric = 11.0;
  handoverReport.position = Vector (300.0, 120.0, 0.0);
  handoverRrc->ReceiveMeasReport (handoverReport);

  // 切换成功 -> UE 现处于波束 22; 服务对象记录也应指向波束 22。
  const HandoverStatsSummary hoSummary = handoverRrc->GetHandoverStats ();
  ExpectEq (hoSummary.successes, 1u,
            "A3 MeasReport should yield exactly one successful beam handover");
  ExpectEq (handoverScheduler->GetUeContext (handoverRnti).currentBeamId, 22u,
            "NtnGwRrc handover should move the UE context onto the reported best beam");

  HandoverUeContext exportedAfterReport = handoverScheduler->ExportUeContext (handoverRnti, 23);
  ExpectEq (exportedAfterReport.sourceBeamId, 22u,
            "Post-handover export should see the UE on the new best beam 22");
  ExpectEq (exportedAfterReport.unsentDlBufferBytes, 500u,
            "Handover should preserve unsent DL buffer bytes through the context move");
  ExpectEq (exportedAfterReport.unsentUlBufferBytes, 400u,
            "Handover should preserve unsent UL buffer bytes through the context move");

  PrintSection ("Test 10: Load balancing should move low-priority UE to a lighter beam");

  Ptr<GeoBeamScheduler> rebalanceScheduler = CreateObject<GeoBeamScheduler> ();
  rebalanceScheduler->Initialize (31, 30);
  Ptr<AdmitControl> rebalanceAdmit = CreateObject<AdmitControl> ();
  rebalanceScheduler->SetAdmitControl (rebalanceAdmit);

  const uint16_t rebalanceDataRnti = 801;
  const uint16_t rebalanceVoiceRnti1 = 802;
  const uint16_t rebalanceVoiceRnti2 = 803;

  rebalanceScheduler->AddUeContext (rebalanceDataRnti, UT_CONSUMER, TRAFFIC_DATA);
  rebalanceScheduler->AddUeInfo (rebalanceDataRnti, 31);
  rebalanceScheduler->UpdateUeDlBufferStatus (rebalanceDataRnti, 4000);

  rebalanceScheduler->AddUeContext (rebalanceVoiceRnti1, UT_PORTABLE, TRAFFIC_VOICE);
  rebalanceScheduler->AddUeInfo (rebalanceVoiceRnti1, 31);
  rebalanceScheduler->UpdateUeDlBufferStatus (rebalanceVoiceRnti1, 500);

  rebalanceScheduler->AddUeContext (rebalanceVoiceRnti2, UT_PORTABLE, TRAFFIC_VOICE);
  rebalanceScheduler->AddUeInfo (rebalanceVoiceRnti2, 31);
  rebalanceScheduler->UpdateUeDlBufferStatus (rebalanceVoiceRnti2, 500);

  Ptr<ResourceManager> rebalanceRm = CreateObject<ResourceManager> ();
  rebalanceAdmit->SetResourceManager (rebalanceRm);
  rebalanceAdmit->SetBeamTotalRbs (31, 25);
  rebalanceAdmit->SetBeamTotalRbs (32, 25);
  rebalanceRm->ResetBeamAllocation (31, false);
  rebalanceRm->ResetBeamAllocation (32, false);
  rebalanceRm->AllocateSpectrum (31, 24, false);

  rebalanceAdmit->CheckBeamLoadBalancing ();

  HandoverUeContext rebalanceExport = rebalanceScheduler->ExportUeContext (rebalanceDataRnti, 33);
  ExpectEq (rebalanceExport.sourceBeamId, 32u,
            "Load balancing should move the low-priority data UE to the lighter beam");

  PrintSection ("Test 11: PRACH/RAR should honor Msg3 power-control UE type");

  // 时序适配: 当前调度器把 RA 建模为真实 GEO 链路, ReceivePrachPreamble 会先延迟
  // m_uplinkPropDelay (~300 ms 单程) 才把前导码入窗, 再经 PRACH 窗口 + RAR 处理/发送
  // 时延派发 RAR。SetRaConfig 不再压缩上行传播时延 (旧版会把它收成 rarTxDelay), 因此停
  // 机时间必须覆盖完整 GEO RA 往返 (~300 ms+), 这里用 400 ms。
  const Time raGeoSettleTime = MilliSeconds (400);

  Ptr<GeoBeamScheduler> raScheduler = CreateObject<GeoBeamScheduler> ();
  raScheduler->Initialize (41, 30);
  raScheduler->SetRaConfig (MicroSeconds (100),
                            MicroSeconds (100),
                            MicroSeconds (100),
                            MicroSeconds (100),
                            MicroSeconds (100));

  RarCapture portableRarCapture;
  raScheduler->RegisterUeRaCallbacks (MakeCallback (&RarCapture::OnRar, &portableRarCapture),
                                      MakeCallback (&RarCapture::OnMsg4, &portableRarCapture));

  PrachPreamble portablePreamble;
  portablePreamble.preambleId = 11;
  portablePreamble.format = 0;
  portablePreamble.utType = static_cast<uint8_t> (UT_PORTABLE);
  portablePreamble.transmissionTime = Simulator::Now ();
  portablePreamble.isRetransmission = false;
  raScheduler->ReceivePrachPreamble (portablePreamble);

  Simulator::Stop (raGeoSettleTime);
  Simulator::Run ();

  ExpectEq (portableRarCapture.m_rars.size (), 1u,
            "Single portable PRACH preamble should produce one RAR");
  if (!portableRarCapture.m_rars.empty ())
    {
      ExpectTrue (portableRarCapture.m_rars.front ().ulGrantTxPowerDbm <= 63.0 + 1e-6,
                  "Portable Msg3 grant power should be capped by the portable EIRP");
    }

  Ptr<GeoBeamScheduler> raSchedulerConsumer = CreateObject<GeoBeamScheduler> ();
  raSchedulerConsumer->Initialize (42, 30);
  raSchedulerConsumer->SetRaConfig (MicroSeconds (100),
                                    MicroSeconds (100),
                                    MicroSeconds (100),
                                    MicroSeconds (100),
                                    MicroSeconds (100));

  RarCapture consumerRarCapture;
  raSchedulerConsumer->RegisterUeRaCallbacks (MakeCallback (&RarCapture::OnRar, &consumerRarCapture),
                                              MakeCallback (&RarCapture::OnMsg4, &consumerRarCapture));

  PrachPreamble consumerPreamble;
  consumerPreamble.preambleId = 12;
  consumerPreamble.format = 0;
  consumerPreamble.utType = static_cast<uint8_t> (UT_CONSUMER);
  consumerPreamble.transmissionTime = Simulator::Now ();
  consumerPreamble.isRetransmission = false;
  raSchedulerConsumer->ReceivePrachPreamble (consumerPreamble);

  Simulator::Stop (raGeoSettleTime);
  Simulator::Run ();

  ExpectEq (consumerRarCapture.m_rars.size (), 1u,
            "Single consumer PRACH preamble should produce one RAR");
  if (!portableRarCapture.m_rars.empty () && !consumerRarCapture.m_rars.empty ())
    {
      ExpectTrue (consumerRarCapture.m_rars.front ().ulGrantTxPowerDbm >= 0.0,
                  "Consumer Msg3 grant power should remain a valid positive value");
      ExpectTrue (consumerRarCapture.m_rars.front ().ulGrantTxPowerDbm <= 50.0 + 1e-6,
                  "Consumer Msg3 grant power should remain capped by the consumer EIRP");
      ExpectTrue (portableRarCapture.m_rars.front ().ulGrantTxPowerDbm >=
                  consumerRarCapture.m_rars.front ().ulGrantTxPowerDbm,
                  "Portable Msg3 grant power cap should not be lower than consumer under same path loss");
    }

  PrintSection ("Test 12: 7-color reuse configuration should match project assumptions");

  Ptr<FrequencyReuse> frequencyReuse = CreateObject<FrequencyReuse> ();
  ExpectEq (static_cast<uint32_t> (frequencyReuse->GetReuseFactor ()), 7u,
            "Frequency reuse factor should default to 7");
  ExpectEq (frequencyReuse->GetAvailableRbsPerBeam (), 25u,
            "Each beam should expose 25 RB under 7-color reuse");

  std::map<uint32_t, uint32_t> colorCounts;
  for (uint32_t beamId = 1; beamId <= 7; ++beamId)
    {
      std::vector<uint32_t> allocation = frequencyReuse->GetBeamAllocation (beamId);
      ExpectEq (allocation.size (), 25u,
                "Every reuse color should map to 25 RB");
      if (!allocation.empty ())
        {
          colorCounts[allocation.front () / 25] += 1;
        }
    }
  ExpectEq (colorCounts.size (), 7u,
            "Beam IDs 1-7 should cover all 7 reuse colors exactly once");

  PrintSection ("Test 13: 7 beams with 4 users each should respect per-beam DL budgets");

  Ptr<GeoBeamScheduler> multiBeamDlScheduler = CreateObject<GeoBeamScheduler> ();
  multiBeamDlScheduler->Initialize (1, 30);

  std::map<uint16_t, uint32_t> dlRntiToBeam;
  std::map<uint16_t, DciCapture*> dlCaptureByRnti;
  std::vector<std::unique_ptr<DciCapture>> dlCaptures;
  uint16_t nextDlRnti = 900;

  for (uint32_t beamId = 1; beamId <= 7; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextDlRnti++;
      multiBeamDlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      multiBeamDlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      multiBeamDlScheduler->UpdateUeCsi (voiceRntiPerBeam, 10.0);
      multiBeamDlScheduler->UpdateUeDlBufferStatus (voiceRntiPerBeam, 2000);
      dlCaptures.emplace_back (std::make_unique<DciCapture> (voiceRntiPerBeam));
      multiBeamDlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&DciCapture::OnDci, dlCaptures.back ().get ()));
      dlRntiToBeam[voiceRntiPerBeam] = beamId;
      dlCaptureByRnti[voiceRntiPerBeam] = dlCaptures.back ().get ();

      for (uint32_t userIndex = 0; userIndex < 3; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextDlRnti++;
          multiBeamDlScheduler->AddUeContext (dataRntiPerBeam,
                                              userIndex == 2 ? UT_PORTABLE : UT_CONSUMER,
                                              userIndex == 0 ? TRAFFIC_HIGH_CAPACITY : TRAFFIC_DATA);
          multiBeamDlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          multiBeamDlScheduler->UpdateUeCsi (dataRntiPerBeam, 12.0 - userIndex * 2.0);
          multiBeamDlScheduler->UpdateUeDlBufferStatus (dataRntiPerBeam, 12000 + userIndex * 2000);
          dlCaptures.emplace_back (std::make_unique<DciCapture> (dataRntiPerBeam));
          multiBeamDlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&DciCapture::OnDci, dlCaptures.back ().get ()));
          dlRntiToBeam[dataRntiPerBeam] = beamId;
          dlCaptureByRnti[dataRntiPerBeam] = dlCaptures.back ().get ();
        }
    }

  multiBeamDlScheduler->RunScheduler ();

  std::map<uint32_t, uint32_t> beamDlRbTotals;
  std::map<uint32_t, uint32_t> beamDlGrantCount;
  for (const auto& entry : dlRntiToBeam)
    {
      const uint16_t rnti = entry.first;
      const uint32_t beamId = entry.second;
      for (const auto& dci : dlCaptureByRnti.at (rnti)->m_dcis)
        {
          if (!dci.isUplinkGrant)
            {
              beamDlRbTotals[beamId] += dci.rbAllocation;
              beamDlGrantCount[beamId] += 1;
            }
        }
    }

  for (uint32_t beamId = 1; beamId <= 7; ++beamId)
    {
      ExpectTrue (beamDlGrantCount[beamId] >= 2,
                  "Each beam should schedule at least voice and one data UE in DL",
                  "beam=" + std::to_string (beamId));
      ExpectEq (beamDlRbTotals[beamId], 19u,
                "Each beam should consume exactly 19 DL traffic RB after PRACH reserve");
    }

  PrintSection ("Test 14: 7 beams with 5 users each should respect per-beam UL budgets");

  Ptr<GeoBeamScheduler> multiBeamUlScheduler = CreateObject<GeoBeamScheduler> ();
  multiBeamUlScheduler->Initialize (1, 30);

  std::map<uint16_t, uint32_t> ulRntiToBeam;
  std::map<uint16_t, DciCapture*> ulCaptureByRnti;
  std::vector<std::unique_ptr<DciCapture>> ulCaptures;
  uint16_t nextUlRnti = 1200;

  for (uint32_t beamId = 1; beamId <= 7; ++beamId)
    {
      const uint16_t voiceUlRnti = nextUlRnti++;
      multiBeamUlScheduler->AddUeContext (voiceUlRnti, UT_PORTABLE, TRAFFIC_VOICE);
      multiBeamUlScheduler->AddUeInfo (voiceUlRnti, beamId);
      multiBeamUlScheduler->UpdateUeCsi (voiceUlRnti, 10.0);
      multiBeamUlScheduler->ReceiveBsr ({voiceUlRnti, 0, 2400});
      ulCaptures.emplace_back (std::make_unique<DciCapture> (voiceUlRnti));
      multiBeamUlScheduler->RegisterUeDciCallback (
        voiceUlRnti,
        MakeCallback (&DciCapture::OnDci, ulCaptures.back ().get ()));
      ulRntiToBeam[voiceUlRnti] = beamId;
      ulCaptureByRnti[voiceUlRnti] = ulCaptures.back ().get ();

      for (uint32_t userIndex = 0; userIndex < 4; ++userIndex)
        {
          const uint16_t dataUlRnti = nextUlRnti++;
          multiBeamUlScheduler->AddUeContext (dataUlRnti,
                                              userIndex >= 2 ? UT_PORTABLE : UT_CONSUMER,
                                              TRAFFIC_DATA);
          multiBeamUlScheduler->AddUeInfo (dataUlRnti, beamId);
          multiBeamUlScheduler->UpdateUeCsi (dataUlRnti, 13.0 - userIndex * 2.5);
          multiBeamUlScheduler->ReceiveBsr ({dataUlRnti, 0, 4000 + userIndex * 1500});
          ulCaptures.emplace_back (std::make_unique<DciCapture> (dataUlRnti));
          multiBeamUlScheduler->RegisterUeDciCallback (
            dataUlRnti,
            MakeCallback (&DciCapture::OnDci, ulCaptures.back ().get ()));
          ulRntiToBeam[dataUlRnti] = beamId;
          ulCaptureByRnti[dataUlRnti] = ulCaptures.back ().get ();
        }
    }

  NrMacSchedSapProvider::SchedUlTriggerReqParameters multiBeamUlTrigger;
  multiBeamUlScheduler->DoSchedUlTriggerReq (multiBeamUlTrigger);

  std::map<uint32_t, uint32_t> beamUlRbTotals;
  std::map<uint32_t, uint32_t> beamUlGrantCount;
  std::map<uint32_t, uint32_t> beamVoiceUlRb;
  for (const auto& entry : ulRntiToBeam)
    {
      const uint16_t rnti = entry.first;
      const uint32_t beamId = entry.second;
      for (const auto& dci : ulCaptureByRnti.at (rnti)->m_dcis)
        {
          if (dci.isUplinkGrant)
            {
              beamUlRbTotals[beamId] += dci.rbAllocation;
              beamUlGrantCount[beamId] += 1;
              if ((rnti - 1200) % 5 == 0)
                {
                  beamVoiceUlRb[beamId] += dci.rbAllocation;
                }
            }
        }
    }

  for (uint32_t beamId = 1; beamId <= 7; ++beamId)
    {
      ExpectTrue (beamUlGrantCount[beamId] >= 2,
                  "Each beam should produce at least one voice UL grant and one data UL grant",
                  "beam=" + std::to_string (beamId));
      ExpectEq (beamVoiceUlRb[beamId], 2u,
                "Each beam voice UE should preserve the 2 RB UL reservation");
      ExpectEq (beamUlRbTotals[beamId], 25u,
                "Each beam should consume exactly one beam UL budget of 25 RB");
    }

  PrintSection ("Test 15: 7 beams with 3000 users each should complete one DL scheduling round");

  constexpr uint32_t stressBeamCount = 7;
  constexpr uint32_t stressUsersPerBeam = 3000;
  constexpr uint32_t stressTotalUsers = stressBeamCount * stressUsersPerBeam;

  Ptr<GeoBeamScheduler> stressDlScheduler = CreateObject<GeoBeamScheduler> ();
  stressDlScheduler->Initialize (1, 30);

  std::map<uint16_t, uint32_t> stressDlRntiToBeam;
  std::map<uint16_t, DciCapture*> stressDlCaptureByRnti;
  std::vector<std::unique_ptr<DciCapture>> stressDlCaptures;
  stressDlCaptures.reserve (stressTotalUsers);
  uint16_t nextStressDlRnti = 2000;

  for (uint32_t beamId = 1; beamId <= stressBeamCount; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextStressDlRnti++;
      stressDlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      stressDlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      stressDlScheduler->UpdateUeCsi (voiceRntiPerBeam, 12.0);
      stressDlScheduler->UpdateUeDlBufferStatus (voiceRntiPerBeam, 4000);
      stressDlCaptures.emplace_back (std::make_unique<DciCapture> (voiceRntiPerBeam));
      stressDlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&DciCapture::OnDci, stressDlCaptures.back ().get ()));
      stressDlRntiToBeam[voiceRntiPerBeam] = beamId;
      stressDlCaptureByRnti[voiceRntiPerBeam] = stressDlCaptures.back ().get ();

      for (uint32_t userIndex = 1; userIndex < stressUsersPerBeam; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextStressDlRnti++;
          stressDlScheduler->AddUeContext (dataRntiPerBeam,
                                           (userIndex % 3 == 0) ? UT_PORTABLE : UT_CONSUMER,
                                           (userIndex % 10 == 0) ? TRAFFIC_HIGH_CAPACITY : TRAFFIC_DATA);
          stressDlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          stressDlScheduler->UpdateUeCsi (dataRntiPerBeam, 14.0 - (userIndex % 5));
          stressDlScheduler->UpdateUeDlBufferStatus (dataRntiPerBeam, 8000 + (userIndex % 7) * 1000);
          stressDlCaptures.emplace_back (std::make_unique<DciCapture> (dataRntiPerBeam));
          stressDlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&DciCapture::OnDci, stressDlCaptures.back ().get ()));
          stressDlRntiToBeam[dataRntiPerBeam] = beamId;
          stressDlCaptureByRnti[dataRntiPerBeam] = stressDlCaptures.back ().get ();
        }
    }

  const auto dlStressStart = std::chrono::steady_clock::now ();
  stressDlScheduler->RunScheduler ();
  const auto dlStressMs =
    std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () -
                                                           dlStressStart)
      .count ();

  std::map<uint32_t, uint32_t> stressBeamDlRbTotals;
  std::map<uint32_t, uint32_t> stressBeamDlGrantCount;
  for (const auto& entry : stressDlRntiToBeam)
    {
      const uint16_t rnti = entry.first;
      const uint32_t beamId = entry.second;
      for (const auto& dci : stressDlCaptureByRnti.at (rnti)->m_dcis)
        {
          if (!dci.isUplinkGrant)
            {
              stressBeamDlRbTotals[beamId] += dci.rbAllocation;
              stressBeamDlGrantCount[beamId] += 1;
            }
        }
    }

  ExpectTrue (true,
              "Large-scale DL scheduling round should complete without crashing",
              "7 beams x 3000 users | wall-clock=" + std::to_string (dlStressMs) + " ms");
  for (uint32_t beamId = 1; beamId <= stressBeamCount; ++beamId)
    {
      ExpectTrue (stressBeamDlGrantCount[beamId] >= 2,
                  "Large-scale DL scheduling should still serve at least voice and one data UE",
                  "beam=" + std::to_string (beamId));
      ExpectEq (stressBeamDlRbTotals[beamId], 19u,
                "Large-scale DL scheduling should still respect the 19 RB traffic budget per beam");
    }

  PrintSection ("Test 16: 7 beams with 3000 users each should complete one UL scheduling round");

  Ptr<GeoBeamScheduler> stressUlScheduler = CreateObject<GeoBeamScheduler> ();
  stressUlScheduler->Initialize (1, 30);

  std::map<uint16_t, uint32_t> stressUlRntiToBeam;
  std::map<uint16_t, DciCapture*> stressUlCaptureByRnti;
  std::vector<std::unique_ptr<DciCapture>> stressUlCaptures;
  stressUlCaptures.reserve (stressTotalUsers);
  uint16_t nextStressUlRnti = 30000;

  for (uint32_t beamId = 1; beamId <= stressBeamCount; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextStressUlRnti++;
      stressUlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      stressUlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      stressUlScheduler->UpdateUeCsi (voiceRntiPerBeam, 12.0);
      stressUlScheduler->ReceiveBsr ({voiceRntiPerBeam, 0, 2400});
      stressUlCaptures.emplace_back (std::make_unique<DciCapture> (voiceRntiPerBeam));
      stressUlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&DciCapture::OnDci, stressUlCaptures.back ().get ()));
      stressUlRntiToBeam[voiceRntiPerBeam] = beamId;
      stressUlCaptureByRnti[voiceRntiPerBeam] = stressUlCaptures.back ().get ();

      for (uint32_t userIndex = 1; userIndex < stressUsersPerBeam; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextStressUlRnti++;
          stressUlScheduler->AddUeContext (dataRntiPerBeam,
                                           (userIndex % 4 == 0) ? UT_PORTABLE : UT_CONSUMER,
                                           TRAFFIC_DATA);
          stressUlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          stressUlScheduler->UpdateUeCsi (dataRntiPerBeam, 14.0 - (userIndex % 6));
          stressUlScheduler->ReceiveBsr ({dataRntiPerBeam, 0, 4000 + (userIndex % 9) * 1200});
          stressUlCaptures.emplace_back (std::make_unique<DciCapture> (dataRntiPerBeam));
          stressUlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&DciCapture::OnDci, stressUlCaptures.back ().get ()));
          stressUlRntiToBeam[dataRntiPerBeam] = beamId;
          stressUlCaptureByRnti[dataRntiPerBeam] = stressUlCaptures.back ().get ();
        }
    }

  NrMacSchedSapProvider::SchedUlTriggerReqParameters stressUlTrigger;
  const auto ulStressStart = std::chrono::steady_clock::now ();
  stressUlScheduler->DoSchedUlTriggerReq (stressUlTrigger);
  const auto ulStressMs =
    std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () -
                                                           ulStressStart)
      .count ();

  std::map<uint32_t, uint32_t> stressBeamUlRbTotals;
  std::map<uint32_t, uint32_t> stressBeamUlGrantCount;
  std::map<uint32_t, uint32_t> stressBeamVoiceUlRb;
  for (const auto& entry : stressUlRntiToBeam)
    {
      const uint16_t rnti = entry.first;
      const uint32_t beamId = entry.second;
      for (const auto& dci : stressUlCaptureByRnti.at (rnti)->m_dcis)
        {
          if (dci.isUplinkGrant)
            {
              stressBeamUlRbTotals[beamId] += dci.rbAllocation;
              stressBeamUlGrantCount[beamId] += 1;
              if ((rnti - 30000) % stressUsersPerBeam == 0)
                {
                  stressBeamVoiceUlRb[beamId] += dci.rbAllocation;
                }
            }
        }
    }

  ExpectTrue (true,
              "Large-scale UL scheduling round should complete without crashing",
              "7 beams x 3000 users | wall-clock=" + std::to_string (ulStressMs) + " ms");
  for (uint32_t beamId = 1; beamId <= stressBeamCount; ++beamId)
    {
      ExpectTrue (stressBeamUlGrantCount[beamId] >= 2,
                  "Large-scale UL scheduling should still serve at least voice and one data UE",
                  "beam=" + std::to_string (beamId));
      ExpectEq (stressBeamVoiceUlRb[beamId], 2u,
                "Large-scale UL scheduling should preserve the 2 RB voice reservation per beam");
      ExpectEq (stressBeamUlRbTotals[beamId], 25u,
                "Large-scale UL scheduling should still respect the 25 RB budget per beam");
    }

  PrintSection ("Test 17: 300 beams with 200 users each should complete one DL scheduling round");

  constexpr uint32_t hugeBeamCount = 300;
  constexpr uint32_t hugeUsersPerBeam = 200;
  constexpr uint32_t hugeTotalUsers = hugeBeamCount * hugeUsersPerBeam;
  constexpr uint16_t hugeDlRntiBase = 1000;

  Ptr<GeoBeamScheduler> hugeDlScheduler = CreateObject<GeoBeamScheduler> ();
  hugeDlScheduler->Initialize (1, 30);

  std::vector<uint32_t> hugeDlRntiToBeam (65536, 0);
  std::vector<DciCapture*> hugeDlCaptureByRnti (65536, nullptr);
  std::vector<std::unique_ptr<DciCapture>> hugeDlCaptures;
  hugeDlCaptures.reserve (hugeTotalUsers);
  uint16_t nextHugeDlRnti = hugeDlRntiBase;

  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextHugeDlRnti++;
      hugeDlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      hugeDlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      hugeDlScheduler->UpdateUeCsi (voiceRntiPerBeam, 12.0);
      hugeDlScheduler->UpdateUeDlBufferStatus (voiceRntiPerBeam, 4000);
      hugeDlCaptures.emplace_back (std::make_unique<DciCapture> (voiceRntiPerBeam));
      hugeDlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&DciCapture::OnDci, hugeDlCaptures.back ().get ()));
      hugeDlRntiToBeam[voiceRntiPerBeam] = beamId;
      hugeDlCaptureByRnti[voiceRntiPerBeam] = hugeDlCaptures.back ().get ();

      for (uint32_t userIndex = 1; userIndex < hugeUsersPerBeam; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextHugeDlRnti++;
          hugeDlScheduler->AddUeContext (dataRntiPerBeam,
                                         (userIndex % 3 == 0) ? UT_PORTABLE : UT_CONSUMER,
                                         (userIndex % 10 == 0) ? TRAFFIC_HIGH_CAPACITY : TRAFFIC_DATA);
          hugeDlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          hugeDlScheduler->UpdateUeCsi (dataRntiPerBeam, 14.0 - (userIndex % 5));
          hugeDlScheduler->UpdateUeDlBufferStatus (dataRntiPerBeam, 8000 + (userIndex % 7) * 1000);
          hugeDlCaptures.emplace_back (std::make_unique<DciCapture> (dataRntiPerBeam));
          hugeDlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&DciCapture::OnDci, hugeDlCaptures.back ().get ()));
          hugeDlRntiToBeam[dataRntiPerBeam] = beamId;
          hugeDlCaptureByRnti[dataRntiPerBeam] = hugeDlCaptures.back ().get ();
        }
    }

  const auto hugeDlStressStart = std::chrono::steady_clock::now ();
  hugeDlScheduler->RunScheduler ();
  const auto hugeDlStressMs =
    std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () -
                                                           hugeDlStressStart)
      .count ();

  std::map<uint32_t, uint32_t> hugeBeamDlRbTotals;
  std::map<uint32_t, uint32_t> hugeBeamDlGrantCount;
  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      for (uint32_t userOffset = 0; userOffset < hugeUsersPerBeam; ++userOffset)
        {
          const uint16_t rnti = hugeDlRntiBase + (beamId - 1) * hugeUsersPerBeam + userOffset;
          DciCapture* capture = hugeDlCaptureByRnti[rnti];
          if (capture == nullptr)
            {
              continue;
            }
          for (const auto& dci : capture->m_dcis)
            {
              if (!dci.isUplinkGrant)
                {
                  hugeBeamDlRbTotals[beamId] += dci.rbAllocation;
                  hugeBeamDlGrantCount[beamId] += 1;
                }
            }
        }
    }

  ExpectTrue (true,
              "Huge-scale DL scheduling round should complete without crashing",
              "300 beams x 200 users | wall-clock=" + std::to_string (hugeDlStressMs) + " ms");
  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      ExpectTrue (hugeBeamDlGrantCount[beamId] >= 2,
                  "Huge-scale DL scheduling should still serve at least voice and one data UE",
                  "beam=" + std::to_string (beamId));
      ExpectEq (hugeBeamDlRbTotals[beamId], 19u,
                "Huge-scale DL scheduling should still respect the 19 RB traffic budget per beam");
    }

  PrintSection ("Test 18: 300 beams with 200 users each should complete one UL scheduling round");

  constexpr uint16_t hugeUlRntiBase = 1000;
  Ptr<GeoBeamScheduler> hugeUlScheduler = CreateObject<GeoBeamScheduler> ();
  hugeUlScheduler->Initialize (1, 30);

  std::vector<DciCapture*> hugeUlCaptureByRnti (65536, nullptr);
  std::vector<std::unique_ptr<DciCapture>> hugeUlCaptures;
  hugeUlCaptures.reserve (hugeTotalUsers);
  uint16_t nextHugeUlRnti = hugeUlRntiBase;

  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextHugeUlRnti++;
      hugeUlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      hugeUlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      hugeUlScheduler->UpdateUeCsi (voiceRntiPerBeam, 12.0);
      hugeUlScheduler->ReceiveBsr ({voiceRntiPerBeam, 0, 2400});
      hugeUlCaptures.emplace_back (std::make_unique<DciCapture> (voiceRntiPerBeam));
      hugeUlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&DciCapture::OnDci, hugeUlCaptures.back ().get ()));
      hugeUlCaptureByRnti[voiceRntiPerBeam] = hugeUlCaptures.back ().get ();

      for (uint32_t userIndex = 1; userIndex < hugeUsersPerBeam; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextHugeUlRnti++;
          hugeUlScheduler->AddUeContext (dataRntiPerBeam,
                                         (userIndex % 4 == 0) ? UT_PORTABLE : UT_CONSUMER,
                                         TRAFFIC_DATA);
          hugeUlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          hugeUlScheduler->UpdateUeCsi (dataRntiPerBeam, 14.0 - (userIndex % 6));
          hugeUlScheduler->ReceiveBsr ({dataRntiPerBeam, 0, 4000 + (userIndex % 9) * 1200});
          hugeUlCaptures.emplace_back (std::make_unique<DciCapture> (dataRntiPerBeam));
          hugeUlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&DciCapture::OnDci, hugeUlCaptures.back ().get ()));
          hugeUlCaptureByRnti[dataRntiPerBeam] = hugeUlCaptures.back ().get ();
        }
    }

  NrMacSchedSapProvider::SchedUlTriggerReqParameters hugeUlTrigger;
  const auto hugeUlStressStart = std::chrono::steady_clock::now ();
  hugeUlScheduler->DoSchedUlTriggerReq (hugeUlTrigger);
  const auto hugeUlStressMs =
    std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () -
                                                           hugeUlStressStart)
      .count ();

  std::map<uint32_t, uint32_t> hugeBeamUlRbTotals;
  std::map<uint32_t, uint32_t> hugeBeamUlGrantCount;
  std::map<uint32_t, uint32_t> hugeBeamVoiceUlRb;
  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      for (uint32_t userOffset = 0; userOffset < hugeUsersPerBeam; ++userOffset)
        {
          const uint16_t rnti = hugeUlRntiBase + (beamId - 1) * hugeUsersPerBeam + userOffset;
          DciCapture* capture = hugeUlCaptureByRnti[rnti];
          if (capture == nullptr)
            {
              continue;
            }
          for (const auto& dci : capture->m_dcis)
            {
              if (dci.isUplinkGrant)
                {
                  hugeBeamUlRbTotals[beamId] += dci.rbAllocation;
                  hugeBeamUlGrantCount[beamId] += 1;
                  if (userOffset == 0)
                    {
                      hugeBeamVoiceUlRb[beamId] += dci.rbAllocation;
                    }
                }
            }
        }
    }

  ExpectTrue (true,
              "Huge-scale UL scheduling round should complete without crashing",
              "300 beams x 200 users | wall-clock=" + std::to_string (hugeUlStressMs) + " ms");
  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      ExpectTrue (hugeBeamUlGrantCount[beamId] >= 2,
                  "Huge-scale UL scheduling should still serve at least voice and one data UE",
                  "beam=" + std::to_string (beamId));
      ExpectEq (hugeBeamVoiceUlRb[beamId], 2u,
                "Huge-scale UL scheduling should preserve the 2 RB voice reservation per beam");
      ExpectEq (hugeBeamUlRbTotals[beamId], 25u,
                "Huge-scale UL scheduling should still respect the 25 RB budget per beam");
    }

  PrintSection ("Test 19: 300 beams with 200 users each should remain stable for 100 DL rounds");

  constexpr uint32_t longStressRounds = 100;
  Ptr<GeoBeamScheduler> longDlScheduler = CreateObject<GeoBeamScheduler> ();
  longDlScheduler->Initialize (1, 30);

  std::vector<std::unique_ptr<RoundGrantCapture>> longDlCaptures;
  longDlCaptures.reserve (hugeTotalUsers);
  uint16_t nextLongDlRnti = hugeDlRntiBase;

  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextLongDlRnti++;
      longDlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      longDlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      longDlScheduler->UpdateUeCsi (voiceRntiPerBeam, 12.0);
      longDlScheduler->UpdateUeDlBufferStatus (voiceRntiPerBeam, 1000000);
      longDlCaptures.emplace_back (std::make_unique<RoundGrantCapture> (voiceRntiPerBeam, beamId, true));
      longDlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&RoundGrantCapture::OnDci, longDlCaptures.back ().get ()));

      for (uint32_t userIndex = 1; userIndex < hugeUsersPerBeam; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextLongDlRnti++;
          longDlScheduler->AddUeContext (dataRntiPerBeam,
                                         (userIndex % 3 == 0) ? UT_PORTABLE : UT_CONSUMER,
                                         (userIndex % 10 == 0) ? TRAFFIC_HIGH_CAPACITY : TRAFFIC_DATA);
          longDlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          longDlScheduler->UpdateUeCsi (dataRntiPerBeam, 14.0 - (userIndex % 5));
          longDlScheduler->UpdateUeDlBufferStatus (dataRntiPerBeam, 1000000);
          longDlCaptures.emplace_back (std::make_unique<RoundGrantCapture> (dataRntiPerBeam, beamId, false));
          longDlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&RoundGrantCapture::OnDci, longDlCaptures.back ().get ()));
        }
    }

  bool allLongDlRoundsValid = true;
  const auto longDlStart = std::chrono::steady_clock::now ();
  for (uint32_t round = 0; round < longStressRounds; ++round)
    {
      for (auto& capture : longDlCaptures)
        {
          capture->ResetRound ();
        }

      longDlScheduler->RunScheduler ();

      std::vector<uint32_t> beamDlRbTotals (hugeBeamCount + 1, 0);
      std::vector<uint32_t> beamDlGrantCount (hugeBeamCount + 1, 0);
      for (const auto& capture : longDlCaptures)
        {
          beamDlRbTotals[capture->m_beamId] += capture->m_roundDlRb;
          beamDlGrantCount[capture->m_beamId] += capture->m_roundDlGrantCount;
        }

      for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
        {
          if (beamDlGrantCount[beamId] < 2 || beamDlRbTotals[beamId] != 19u)
            {
              allLongDlRoundsValid = false;
              break;
            }
        }

      if (!allLongDlRoundsValid)
        {
          break;
        }
    }
  const auto longDlMs =
    std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () -
                                                           longDlStart)
      .count ();

  ExpectTrue (allLongDlRoundsValid,
              "Huge-scale DL scheduling should remain budget-correct for 100 rounds",
              "300 beams x 200 users x 100 rounds | wall-clock=" + std::to_string (longDlMs) + " ms");

  PrintSection ("Test 20: 300 beams with 200 users each should remain stable for 100 UL rounds");

  Simulator::Destroy ();

  Ptr<GeoBeamScheduler> longUlScheduler = CreateObject<GeoBeamScheduler> ();
  longUlScheduler->Initialize (1, 30);

  std::vector<std::unique_ptr<RoundGrantCapture>> longUlCaptures;
  longUlCaptures.reserve (hugeTotalUsers);
  uint16_t nextLongUlRnti = hugeUlRntiBase;

  for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
    {
      const uint16_t voiceRntiPerBeam = nextLongUlRnti++;
      longUlScheduler->AddUeContext (voiceRntiPerBeam, UT_PORTABLE, TRAFFIC_VOICE);
      longUlScheduler->AddUeInfo (voiceRntiPerBeam, beamId);
      longUlScheduler->UpdateUeCsi (voiceRntiPerBeam, 12.0);
      longUlScheduler->ReceiveBsr ({voiceRntiPerBeam, 0, 1000000});
      longUlCaptures.emplace_back (std::make_unique<RoundGrantCapture> (voiceRntiPerBeam, beamId, true));
      longUlScheduler->RegisterUeDciCallback (
        voiceRntiPerBeam,
        MakeCallback (&RoundGrantCapture::OnDci, longUlCaptures.back ().get ()));

      for (uint32_t userIndex = 1; userIndex < hugeUsersPerBeam; ++userIndex)
        {
          const uint16_t dataRntiPerBeam = nextLongUlRnti++;
          longUlScheduler->AddUeContext (dataRntiPerBeam,
                                         (userIndex % 4 == 0) ? UT_PORTABLE : UT_CONSUMER,
                                         TRAFFIC_DATA);
          longUlScheduler->AddUeInfo (dataRntiPerBeam, beamId);
          longUlScheduler->UpdateUeCsi (dataRntiPerBeam, 14.0 - (userIndex % 6));
          longUlScheduler->ReceiveBsr ({dataRntiPerBeam, 0, 1000000});
          longUlCaptures.emplace_back (std::make_unique<RoundGrantCapture> (dataRntiPerBeam, beamId, false));
          longUlScheduler->RegisterUeDciCallback (
            dataRntiPerBeam,
            MakeCallback (&RoundGrantCapture::OnDci, longUlCaptures.back ().get ()));
        }
    }

  bool allLongUlRoundsValid = true;
  const auto longUlStart = std::chrono::steady_clock::now ();
  for (uint32_t round = 0; round < longStressRounds; ++round)
    {
      for (auto& capture : longUlCaptures)
        {
          capture->ResetRound ();
        }

      NrMacSchedSapProvider::SchedUlTriggerReqParameters longUlTrigger;
      longUlScheduler->DoSchedUlTriggerReq (longUlTrigger);

      std::vector<uint32_t> beamUlRbTotals (hugeBeamCount + 1, 0);
      std::vector<uint32_t> beamUlGrantCount (hugeBeamCount + 1, 0);
      std::vector<uint32_t> beamVoiceUlRb (hugeBeamCount + 1, 0);
      for (const auto& capture : longUlCaptures)
        {
          beamUlRbTotals[capture->m_beamId] += capture->m_roundUlRb;
          beamUlGrantCount[capture->m_beamId] += capture->m_roundUlGrantCount;
          if (capture->m_isVoice)
            {
              beamVoiceUlRb[capture->m_beamId] += capture->m_roundUlRb;
            }
        }

      for (uint32_t beamId = 1; beamId <= hugeBeamCount; ++beamId)
        {
          if (beamUlGrantCount[beamId] < 2 || beamVoiceUlRb[beamId] != 2u ||
              beamUlRbTotals[beamId] != 25u)
            {
              allLongUlRoundsValid = false;
              break;
            }
        }

      if (!allLongUlRoundsValid)
        {
          break;
        }

      Simulator::Stop (MilliSeconds (3));
      Simulator::Run ();
    }
  const auto longUlMs =
    std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now () -
                                                           longUlStart)
      .count ();

  ExpectTrue (allLongUlRoundsValid,
              "Huge-scale UL scheduling should remain budget-correct for 100 rounds",
              "300 beams x 200 users x 100 rounds | wall-clock=" + std::to_string (longUlMs) + " ms");

  Simulator::Destroy ();

  PrintSection ("Validation Summary");
  std::cout << "PASS=" << g_passCount << " FAIL=" << g_failCount << std::endl;
  std::cout << (g_allPassed ? "OVERALL RESULT: PASS" : "OVERALL RESULT: FAIL") << std::endl;

  return g_allPassed ? 0 : 1;
}
