// SPDX-License-Identifier: GPL-2.0-only
//
// CqiAmcController — OLLA 外环 + Kalman CQI 预测 闭环链路自适应
//
// 用于 GEO 透明转发 (~510ms RTT) 下补偿 CQI 上报过时:
//   * Kalman (每 UE, 二维状态 x=[L 电平, R 每 slot 变化率]):
//     CQI 上报到达时做更新步; 调度选 MCS 时向前预测 H 步 predCqi=L+R·H。
//   * OLLA (每 UE 偏置 Δ): 仅对新传 ACK/NACK 更新, 锁定目标 BLER。
//   * 组合: effectiveCqi = predCqi + Δ, 由外部 (scheduler) 转 MCS, 避免在此
//     重复 CQI→MCS 表。
//
// 设计文档: contrib/geo-sat/OLLA外环链路自适应 md版.docx

#ifndef CQI_AMC_CONTROLLER_H
#define CQI_AMC_CONTROLLER_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include <cstdint>
#include <map>

namespace ns3 {

/**
 * \brief per-UE Kalman 状态: 二维 x=[L,R] (CQI 电平, 每 slot 变化率)。
 */
struct CqiKalmanState
{
  double L {7.0};                ///< CQI 电平 (默认与 SatUeContext.latestDlCqi 默认 7.0 对齐)
  double R {0.0};                ///< 每 slot 变化率
  double P[2][2] {{1.0, 0.0}, {0.0, 1.0}};  ///< 协方差
  double lastUpdateSlot {-1.0};  ///< 上次更新所在 slot, 用于 dt 计算 (<0 表示未初始化)
  bool initialized {false};
};

/**
 * \brief per-UE OLLA 状态: 偏置 Δ + 仅新传统计。
 */
struct OllaState
{
  double delta {0.0};
  uint64_t newTxAck {0};
  uint64_t newTxNack {0};
};

/**
 * \brief OLLA + Kalman 闭环 CQI/AMC 控制器 (per-UE)。
 *
 * 接口对应文档中:
 *   OnCqiReport     ← GW 收到 CQI 上报 (经半 RTT 延迟)
 *   PredictCqi      → 调度时向前 H 步预测的 CQI
 *   GetEffectiveCqi → predCqi + OLLA Δ, 由 scheduler 转 MCS
 *   OnHarqFeedback  ← GW 收到 HARQ 反馈 (经 1.5 RTT 延迟); 仅新传更新
 */
class CqiAmcController : public Object
{
public:
  static TypeId GetTypeId ();
  CqiAmcController ();
  ~CqiAmcController () override;

  // --- 闭环主接口 ---
  /** 收到 CQI 上报: Kalman 预测步 + 更新步 (用当前 slot 与上次的 dt)。 */
  void OnCqiReport (uint16_t rnti, double measuredCqi);

  /** 不修改状态: 返回当前预测的 H 步前向 CQI (clamp [0,15])。 */
  double PredictCqi (uint16_t rnti) const;

  /**
   * 该 rnti 是否已初始化且估计新鲜 (距上次 OnCqiReport 不超过 maxAge)。
   * 适配器据此决定走预测还是直通调度器算好的新鲜度回退 measuredCqi。
   * \param rnti   目标 UE
   * \param maxAge 允许的最大陈旧时长; <=0 表示只要初始化即视为新鲜 (不做陈旧判断)。
   */
  bool HasFreshEstimate (uint16_t rnti, Time maxAge) const;

  /** 上次 OnCqiReport 所在 slot (slotNow = now_ms / DtSlots); 未初始化返回 <0。 */
  double GetLastUpdateSlot (uint16_t rnti) const;

  /** 不修改状态: predCqi + Δ, clamp [0,15], 供 scheduler 转 MCS。 */
  double GetEffectiveCqi (uint16_t rnti) const;

  /**
   * HARQ 反馈更新 OLLA Δ。**仅当 isNewTx==true 时更新**, 重传 (IR 合并) 不污染。
   * ACK→Δ+=stepUp; NACK→Δ-=stepDown; stepDown=stepUp*(1-blerTarget)/blerTarget; clamp。
   */
  void OnHarqFeedback (uint16_t rnti, bool ack, bool isNewTx);

  // --- 统计/调试 getter ---
  double GetDelta (uint16_t rnti) const;
  /** newTxNack / (newTxAck + newTxNack), 应收敛到 blerTarget。 */
  double GetMeasuredBler (uint16_t rnti) const;
  uint64_t GetNewTxCount (uint16_t rnti) const;
  /** Kalman 内部状态 (L, R), 供 example/日志读取。 */
  double GetKalmanL (uint16_t rnti) const;
  double GetKalmanR (uint16_t rnti) const;
  /**
   * H 步前向预测的方差 Var(L + R·H) = P00 + 2·H·P01 + H²·P11 (由当前协方差外推)。
   * 未初始化/无估计返回 0。供 HarqPolicyController 的不确定度风险项 (sigma=sqrt(此值)):
   * 预测越不可信 → 方差越大 → 越该上冗余。
   */
  double GetPredictVariance (uint16_t rnti) const;

  // --- 参数 getter (供 example 输出/记录) ---
  uint32_t GetHorizonH () const { return m_horizonH; }
  double GetBlerTarget () const { return m_blerTarget; }

  /**
   * 按时间设置前向预测视界: H = round(h_ms / DtSlots), 至少 1 slot。
   * 用于让适配器把调度器的 m_dl/ulSchedHorizon 单一化到 controller, 取代写死的 510。
   */
  void SetHorizonFromTime (Time h);

private:
  /** F=[[1,dt],[0,1]] 作用于 x 与 P; Q 注入对角项 (P[1][1] 加更多, R 噪声较大)。 */
  void KalmanPredictTo (CqiKalmanState& s, double slotNow) const;

  // --- 属性参数 ---
  double m_Q;            ///< 过程噪声 (信道变化快慢)
  double m_Rmeas;        ///< 测量噪声 (CQI 量化+估计误差)
  uint32_t m_horizonH;   ///< 前向预测步数 (slot), 默认 510 (~GEO RTT)
  double m_dt;           ///< 一个 slot 的 (相对) 时长基准, 默认 1.0
  double m_stepUp;       ///< OLLA ACK 上升步, 默认 0.05 CQI
  double m_blerTarget;   ///< 目标 BLER, 默认 0.1 → stepDown=9·stepUp=0.45
  double m_deltaMin;     ///< Δ 下限 (默认 -3)
  double m_deltaMax;     ///< Δ 上限 (默认 +3)
  double m_maxPredictDeltaCqi;  ///< 前向外推增量 R·H 的饱和限幅 (CQI 单位, 默认 2.0):
                                ///< 缓变信道下 R 估计抖动被 H(=510) 放大会使 RMSE 反而变差,
                                ///< 把 |R·H| 钳到此值, 既保留趋势性信道的预测增益, 又压住噪声放大。

  std::map<uint16_t, CqiKalmanState> m_kalman;
  std::map<uint16_t, OllaState> m_olla;

  static double Clamp (double v, double lo, double hi);
};

} // namespace ns3

#endif // CQI_AMC_CONTROLLER_H
