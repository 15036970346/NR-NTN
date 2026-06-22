/*
 * 文件路径：contrib/geo-sat/model/cqi-amc-predictor.h
 * 功能：把 CqiAmcController (OLLA 外环 + Kalman CQI 预测) 适配为 CqiPredictor 接口，
 *       从而经 GeoBeamScheduler::SetCqiPredictor 无侵入地接入资源管理。
 *
 * 设计意图：
 *  - GeoBeamScheduler::GetSchedulingCqi 是资源管理 CQI 总闸, 它已经在调
 *    m_cqiPredictor->PredictDl/UlCqi; UpdateUeDl/UlCqi 已经在调
 *    m_cqiPredictor->RecordDl/UlCqi。只要把 m_cqiPredictor 换成本适配器,
 *    RM 就自动用上 Kalman 预测 + OLLA 偏置的滤波 CQI, 调度器逻辑零改动。
 *  - DL/UL 信道不同, 因此各持有一个 CqiAmcController 实例, 绝不共用。
 *  - 默认 DL/UL 实例为构造时自建的兜底; 若 scheduler 已注入外部 m_cqiAmc,
 *    MaybeApplyCqiAmcPredictor 会经 SetDlController 让本适配器复用同一 DL 实例,
 *    实现"单一 DL CQI 真源": GetSchedulingCqi(经适配器) 与 GetMcsForNewTx(经
 *    m_cqiAmc) 读同一 DL Kalman/OLLA, 不再状态发散、不再对同一/不同实例双喂。
 */
#ifndef CQI_AMC_PREDICTOR_H
#define CQI_AMC_PREDICTOR_H

#include "cqi-predictor.h"
#include "cqi-amc-controller.h"
#include "ns3/ptr.h"

namespace ns3 {

/**
 * \brief 把 CqiAmcController 适配到 CqiPredictor 接口的预测器。
 *
 * DL/UL 各持有一个独立 CqiAmcController:
 *   RecordDlCqi → m_dl->OnCqiReport;   PredictDlCqi → m_dl->GetEffectiveCqi
 *   RecordUlCqi → m_ul->OnCqiReport;   PredictUlCqi → m_ul->GetEffectiveCqi
 *   RecordHarqFeedback → m_dl->OnHarqFeedback (仅 DL 有 HARQ)
 */
class CqiAmcPredictor : public CqiPredictor
{
public:
  static TypeId GetTypeId (void);
  CqiAmcPredictor ();
  ~CqiAmcPredictor () override;

  // --- CqiPredictor 接口重写 ---
  void RecordDlCqi (uint16_t rnti, double cqi, Time when) override;
  void RecordUlCqi (uint16_t rnti, double cqi, Time when) override;
  double PredictDlCqi (uint16_t rnti, double measuredCqi, Time horizon) const override;
  double PredictUlCqi (uint16_t rnti, double measuredCqi, Time horizon) const override;
  void RecordHarqFeedback (uint16_t rnti, bool ack, bool isNewTx) override;

  // --- getter (供配置 / 统计) ---
  Ptr<CqiAmcController> GetDl () const { return m_dl; }
  Ptr<CqiAmcController> GetUl () const { return m_ul; }

  /**
   * 复用外部注入的 DL/UL controller (单一 DL CQI 真源)。
   * 设了之后, 适配器读写同一实例, 不再用构造时自建的兜底实例; 这样
   * GetSchedulingCqi(经适配器) 与 GetMcsForNewTx(经 scheduler->m_cqiAmc) 共享
   * 同一 DL Kalman/OLLA 状态, 不再状态发散、不再双喂。未设时保持自建兜底。
   */
  void SetDlController (Ptr<CqiAmcController> ctrl);
  void SetUlController (Ptr<CqiAmcController> ctrl);

  /**
   * 把前向预测视界写入 controller (取代 controller 内部写死的 510 slot)。
   * 由 scheduler 用 m_dlSchedHorizon / m_ulSchedHorizon 调用, 实现 horizon 单一化。
   */
  void SetDlHorizon (Time h);
  void SetUlHorizon (Time h);

  /**
   * 陈旧阈值: 距上次 OnCqiReport 超过该时长则 PredictDl/UlCqi 直通 measuredCqi
   * (用调度器已算好的新鲜度回退值), 不再用过时的 Kalman 外推。
   * 默认 0 表示只要 controller 已初始化即用预测 (与"加适配器即用滤波"语义一致)。
   */
  void SetMaxEstimateAge (Time age) { m_maxEstimateAge = age; }
  Time GetMaxEstimateAge () const { return m_maxEstimateAge; }

private:
  Ptr<CqiAmcController> m_dl;  ///< 下行信道 Kalman+OLLA 实例 (默认自建; 可被外部复用替换)
  Ptr<CqiAmcController> m_ul;  ///< 上行信道 Kalman+OLLA 实例 (无 HARQ→OLLA)
  Time m_maxEstimateAge;       ///< 陈旧阈值; <=0 表示不做陈旧直通
};

} // namespace ns3

#endif // CQI_AMC_PREDICTOR_H
