/*
 * 文件路径：contrib/geo-sat/model/cqi-predictor.cc
 * 功能：CQI 预测器默认(直通)实现
 */
#include "cqi-predictor.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("CqiPredictor");
NS_OBJECT_ENSURE_REGISTERED (CqiPredictor);

TypeId
CqiPredictor::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CqiPredictor")
    .SetParent<Object> ()
    .SetGroupName ("SatGeo")
    .AddConstructor<CqiPredictor> ();
  return tid;
}

CqiPredictor::CqiPredictor () { NS_LOG_FUNCTION (this); }
CqiPredictor::~CqiPredictor () { NS_LOG_FUNCTION (this); }

double
CqiPredictor::PredictDlCqi (uint16_t rnti, double measuredCqi, Time horizon) const
{
  // 直通：未引入真实预测时，预测值即实测值，保证行为不变。
  return measuredCqi;
}

double
CqiPredictor::PredictUlCqi (uint16_t rnti, double measuredCqi, Time horizon) const
{
  return measuredCqi;
}

void
CqiPredictor::RecordDlCqi (uint16_t rnti, double cqi, Time when)
{
  // 直通实现不维护历史。
}

void
CqiPredictor::RecordUlCqi (uint16_t rnti, double cqi, Time when)
{
}

void
CqiPredictor::RecordHarqFeedback (uint16_t rnti, bool ack, bool isNewTx)
{
  // 直通实现不维护 OLLA, 行为不变。
}

} // namespace ns3
