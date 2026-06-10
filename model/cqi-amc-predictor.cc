/*
 * 文件路径：contrib/0603geo-sat/model/cqi-amc-predictor.cc
 * 功能：CqiAmcPredictor 实现 — 把 CqiAmcController 适配为 CqiPredictor。
 */
#include "cqi-amc-predictor.h"

#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("CqiAmcPredictor");
NS_OBJECT_ENSURE_REGISTERED (CqiAmcPredictor);

TypeId
CqiAmcPredictor::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CqiAmcPredictor")
    .SetParent<CqiPredictor> ()
    .SetGroupName ("GeoSat")
    .AddConstructor<CqiAmcPredictor> ();
  return tid;
}

CqiAmcPredictor::CqiAmcPredictor ()
  : m_maxEstimateAge (Time (0))   // 默认不做陈旧直通: 初始化后即用预测 (语义不变)
{
  NS_LOG_FUNCTION (this);
  // DL/UL 各建一个兜底实例 (信道不同, 绝不共用)。
  // 若外部注入了 controller, SetDl/UlController 会替换为复用同一真源。
  m_dl = CreateObject<CqiAmcController> ();
  m_ul = CreateObject<CqiAmcController> ();
}

CqiAmcPredictor::~CqiAmcPredictor ()
{
  NS_LOG_FUNCTION (this);
}

void
CqiAmcPredictor::SetDlController (Ptr<CqiAmcController> ctrl)
{
  if (ctrl == nullptr)
    return;
  // 复用外部 DL controller → 单一 DL CQI 真源。沿用其外部配置的 HorizonH,
  // 不在此覆盖 (外部注入语义优先; horizon 单一化仅作用于自建兜底实例)。
  m_dl = ctrl;
  NS_LOG_INFO ("CqiAmcPredictor reuses external DL controller (single DL CQI source)");
}

void
CqiAmcPredictor::SetUlController (Ptr<CqiAmcController> ctrl)
{
  if (ctrl == nullptr)
    return;
  m_ul = ctrl;
  NS_LOG_INFO ("CqiAmcPredictor reuses external UL controller");
}

void
CqiAmcPredictor::SetDlHorizon (Time h)
{
  if (m_dl)
    m_dl->SetHorizonFromTime (h);
}

void
CqiAmcPredictor::SetUlHorizon (Time h)
{
  if (m_ul)
    m_ul->SetHorizonFromTime (h);
}

void
CqiAmcPredictor::RecordDlCqi (uint16_t rnti, double cqi, Time when)
{
  // OnCqiReport 内部用 Simulator::Now 计算不规则到达的 dt, 故无需传 when。
  m_dl->OnCqiReport (rnti, cqi);
}

void
CqiAmcPredictor::RecordUlCqi (uint16_t rnti, double cqi, Time when)
{
  m_ul->OnCqiReport (rnti, cqi);
}

double
CqiAmcPredictor::PredictDlCqi (uint16_t rnti, double measuredCqi, Time horizon) const
{
  // 陈旧/未初始化回退: 直通调度器已算好的新鲜度回退 measuredCqi, 不用过时外推。
  if (!m_dl->HasFreshEstimate (rnti, m_maxEstimateAge))
    return measuredCqi;
  // EffectiveCqi = Kalman 向前预测 (置信门控+饱和) + OLLA 偏置 Δ。
  return m_dl->GetEffectiveCqi (rnti);
}

double
CqiAmcPredictor::PredictUlCqi (uint16_t rnti, double measuredCqi, Time horizon) const
{
  if (!m_ul->HasFreshEstimate (rnti, m_maxEstimateAge))
    return measuredCqi;
  return m_ul->GetEffectiveCqi (rnti);
}

void
CqiAmcPredictor::RecordHarqFeedback (uint16_t rnti, bool ack, bool isNewTx)
{
  // 仅 DL 有 HARQ→OLLA 闭环; CqiAmcController 内部已对 isNewTx==false 直接忽略。
  m_dl->OnHarqFeedback (rnti, ack, isNewTx);
}

} // namespace ns3
