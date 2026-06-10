/*
 * 文件路径：contrib/geo-sat/model/cqi-predictor.h
 * 功能：CQI 预测器抽象接口 + 直通(passthrough)默认实现
 *
 * 设计意图：
 *  - 把"预测 CQI"与调度器解耦。调度器只依赖本接口，不关心实现。
 *  - 默认实现为直通：直接返回实测 CQI，行为与未引入预测时完全一致。
 *  - 同事的 ML/统计预测模型(例如经 ns3-ai 与 Python 交互)只需派生本类，
 *    重写 PredictDlCqi/PredictUlCqi，再通过 GeoBeamScheduler::SetCqiPredictor 注入，
 *    调度器代码零改动。
 *
 * GEO 场景关键点：CQI 反馈经过 ~一个环回(~600ms)才可用，
 *  调度时应预测"实际传输时刻"的 CQI，因此接口带 horizon(预测提前量)。
 */
#ifndef CQI_PREDICTOR_H
#define CQI_PREDICTOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"

namespace ns3 {

class CqiPredictor : public Object
{
public:
  static TypeId GetTypeId (void);
  CqiPredictor ();
  virtual ~CqiPredictor ();

  /**
   * 预测下行 CQI。
   * \param rnti        目标 UE
   * \param measuredCqi 调度器当前可用的实测/最近 CQI(已做新鲜度回退)
   * \param horizon     预测提前量(从现在到实际传输时刻的时延)
   * \return 预测得到的全带宽标量 CQI
   *
   * 默认实现：直通，忽略 rnti/horizon，直接返回 measuredCqi。
   */
  virtual double PredictDlCqi (uint16_t rnti, double measuredCqi, Time horizon) const;

  /** 预测上行 CQI，语义同上。默认直通。 */
  virtual double PredictUlCqi (uint16_t rnti, double measuredCqi, Time horizon) const;

  /**
   * 记录一次新的 CQI 观测，供需要历史序列的预测器(如时间序列模型)累积样本。
   * 直通实现为空操作。调度器在每次更新 CQI 时调用。
   */
  virtual void RecordDlCqi (uint16_t rnti, double cqi, Time when);
  virtual void RecordUlCqi (uint16_t rnti, double cqi, Time when);

  /**
   * 记录一次 HARQ 反馈, 供需要外环 (OLLA) 的预测器调整 MCS 偏置。
   * \param rnti     目标 UE
   * \param ack      初传是否成功 (true=ACK, false=NACK)
   * \param isNewTx  是否为新传反馈 (仅新传更新 OLLA, 重传合并不污染)
   * 直通实现为空操作 (不维护 OLLA, 行为不变)。
   */
  virtual void RecordHarqFeedback (uint16_t rnti, bool ack, bool isNewTx);
};

} // namespace ns3

#endif /* CQI_PREDICTOR_H */
