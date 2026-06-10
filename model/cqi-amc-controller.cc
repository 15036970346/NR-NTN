// SPDX-License-Identifier: GPL-2.0-only
//
// CqiAmcController 实现: Kalman 二维状态 [L,R] + OLLA 偏置 Δ。
//
// 算法忠实 contrib/geo-sat/OLLA外环链路自适应 md版.docx:
//   F = [[1, dt], [0, 1]]
//   x_pred = F·x;  P_pred = F·P·Fᵀ + Q
//   y = z - H·x_pred;  S = H·P_pred·Hᵀ + Rmeas   (H = [1, 0])
//   K = P_pred·Hᵀ / S                  → K = [P_pred[0][0]/S, P_pred[1][0]/S]ᵀ
//   x = x_pred + K·y;  P = (I - K·H)·P_pred
//   predCqi(H) = L + R·H                (调度时不改状态)
//   ACK  → Δ += stepUp
//   NACK → Δ -= stepDown = stepUp · (1 - blerTarget) / blerTarget
//   Δ ∈ [deltaMin, deltaMax];          仅对新传 (rv==0) 反馈更新

#include "cqi-amc-controller.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("CqiAmcController");
NS_OBJECT_ENSURE_REGISTERED (CqiAmcController);

TypeId
CqiAmcController::GetTypeId ()
{
  static TypeId tid =
      TypeId ("ns3::CqiAmcController")
          .SetParent<Object> ()
          .SetGroupName ("GeoSat")
          .AddConstructor<CqiAmcController> ()
          .AddAttribute ("Q", "Kalman 过程噪声 (信道变化快慢, 注入对角)",
                         DoubleValue (0.01),
                         MakeDoubleAccessor (&CqiAmcController::m_Q),
                         MakeDoubleChecker<double> (0.0))
          .AddAttribute ("Rmeas", "Kalman 测量噪声 (CQI 量化+估计误差)",
                         DoubleValue (0.5),
                         MakeDoubleAccessor (&CqiAmcController::m_Rmeas),
                         MakeDoubleChecker<double> (1e-9))
          .AddAttribute ("HorizonH",
                         "前向预测步数 (slot), GEO 透明转发约 510 slot ≈ 510ms",
                         UintegerValue (510),
                         MakeUintegerAccessor (&CqiAmcController::m_horizonH),
                         MakeUintegerChecker<uint32_t> (1))
          .AddAttribute ("DtSlots", "一个 slot 的相对时长 (用于 dt 计算)",
                         DoubleValue (1.0),
                         MakeDoubleAccessor (&CqiAmcController::m_dt),
                         MakeDoubleChecker<double> (1e-9))
          .AddAttribute ("StepUp", "OLLA ACK 上升步 (CQI 单位)",
                         DoubleValue (0.05),
                         MakeDoubleAccessor (&CqiAmcController::m_stepUp),
                         MakeDoubleChecker<double> (0.0))
          .AddAttribute ("BlerTarget",
                         "目标 BLER. stepDown = stepUp·(1-target)/target (默认 0.1→stepDown=0.45)",
                         DoubleValue (0.1),
                         MakeDoubleAccessor (&CqiAmcController::m_blerTarget),
                         MakeDoubleChecker<double> (1e-6, 0.999999))
          .AddAttribute ("DeltaMin", "Δ 下限 (CQI 单位)",
                         DoubleValue (-3.0),
                         MakeDoubleAccessor (&CqiAmcController::m_deltaMin),
                         MakeDoubleChecker<double> ())
          .AddAttribute ("DeltaMax", "Δ 上限 (CQI 单位)",
                         DoubleValue (3.0),
                         MakeDoubleAccessor (&CqiAmcController::m_deltaMax),
                         MakeDoubleChecker<double> ())
          .AddAttribute ("MaxPredictDeltaCqi",
                         "前向外推增量 R·H 的饱和限幅 (CQI 单位). 缓变信道下 R 抖动被 H 放大会令 "
                         "RMSE 退化; 钳住 |R·H| 可保留趋势预测增益同时压噪. 设很大=关闭饱和.",
                         DoubleValue (2.0),
                         MakeDoubleAccessor (&CqiAmcController::m_maxPredictDeltaCqi),
                         MakeDoubleChecker<double> (0.0));
  return tid;
}

CqiAmcController::CqiAmcController () = default;
CqiAmcController::~CqiAmcController () = default;

double
CqiAmcController::Clamp (double v, double lo, double hi)
{
  return std::max (lo, std::min (hi, v));
}

// 把 Kalman 状态从 lastUpdateSlot 推进到 slotNow: 多步 F 折算成一次 dt。
void
CqiAmcController::KalmanPredictTo (CqiKalmanState& s, double slotNow) const
{
  if (!s.initialized)
    return;
  double dt = (s.lastUpdateSlot < 0) ? m_dt : (slotNow - s.lastUpdateSlot);
  if (dt <= 0.0)
    return;
  // F = [[1, dt],[0, 1]]; x_pred = F·x
  double L_pred = s.L + s.R * dt;
  double R_pred = s.R;
  // P_pred = F·P·Fᵀ + Q   (Q 主要注入 R 维 = P[1][1], 同时给 P[0][0] 一点保持非奇异)
  double P00 = s.P[0][0] + dt * (s.P[1][0] + s.P[0][1]) + dt * dt * s.P[1][1];
  double P01 = s.P[0][1] + dt * s.P[1][1];
  double P10 = s.P[1][0] + dt * s.P[1][1];
  double P11 = s.P[1][1];
  P00 += m_Q * dt;
  P11 += m_Q;
  s.L = L_pred;
  s.R = R_pred;
  s.P[0][0] = P00;
  s.P[0][1] = P01;
  s.P[1][0] = P10;
  s.P[1][1] = P11;
  s.lastUpdateSlot = slotNow;
}

void
CqiAmcController::OnCqiReport (uint16_t rnti, double measuredCqi)
{
  // 当前 slot = 仿真时间(ms) / m_dt  (DtSlots=1 → 1 slot=1ms 自洽)
  double slotNow = Simulator::Now ().GetMilliSeconds () / m_dt;
  auto& s = m_kalman[rnti];

  if (!s.initialized)
    {
      // 首报: 直接用 z 初始化 L, R=0
      s.L = measuredCqi;
      s.R = 0.0;
      s.P[0][0] = 1.0;
      s.P[0][1] = 0.0;
      s.P[1][0] = 0.0;
      s.P[1][1] = 1.0;
      s.lastUpdateSlot = slotNow;
      s.initialized = true;
      NS_LOG_DEBUG ("[Kalman] init rnti=" << rnti << " L=" << s.L);
      return;
    }

  // 预测步: F·x, F·P·Fᵀ + Q
  KalmanPredictTo (s, slotNow);
  // 更新步: H=[1,0]
  double y = measuredCqi - s.L;                  // 残差
  double S = s.P[0][0] + m_Rmeas;                // 残差协方差 (标量)
  double K0 = s.P[0][0] / S;
  double K1 = s.P[1][0] / S;
  s.L += K0 * y;
  s.R += K1 * y;
  // P = (I - K·H) · P_pred,  K·H = [[K0, 0],[K1, 0]]
  double newP00 = (1.0 - K0) * s.P[0][0];
  double newP01 = (1.0 - K0) * s.P[0][1];
  double newP10 = s.P[1][0] - K1 * s.P[0][0];
  double newP11 = s.P[1][1] - K1 * s.P[0][1];
  s.P[0][0] = newP00;
  s.P[0][1] = newP01;
  s.P[1][0] = newP10;
  s.P[1][1] = newP11;

  NS_LOG_DEBUG ("[Kalman] rnti=" << rnti << " z=" << measuredCqi << " → L=" << s.L
                                 << " R=" << s.R);
}

double
CqiAmcController::PredictCqi (uint16_t rnti) const
{
  auto it = m_kalman.find (rnti);
  if (it == m_kalman.end () || !it->second.initialized)
    return 7.0;  // 与 SatUeContext.latestDlCqi 默认值一致
  // 前向外推: L + R·H, 但做两道防噪处理, 避免缓变信道下 R 估计抖动被 H(~510) 放大
  // 导致预测围绕真值大幅摆动(RMSE 反劣于直通):
  //  1) 置信度门控: 用速率信噪比 R²/(R²+Var(R)) 加权——纯噪声的 R(方差 P[1][1] 大)权重→0,
  //     预测退回滤波电平 L; 只有真实趋势(|R| 显著大于其不确定度)才被外推。
  //  2) 饱和限幅: 再把外推增量钳到 ±MaxPredictDeltaCqi, 兜底防极端外推。
  const double R = it->second.R;
  const double varR = std::max (0.0, it->second.P[1][1]);
  const double confidence = (R * R) / (R * R + varR + 1e-12);   // ∈[0,1)
  double delta = R * static_cast<double> (m_horizonH) * confidence;
  delta = Clamp (delta, -m_maxPredictDeltaCqi, m_maxPredictDeltaCqi);
  double pred = it->second.L + delta;
  return Clamp (pred, 0.0, 15.0);
}

bool
CqiAmcController::HasFreshEstimate (uint16_t rnti, Time maxAge) const
{
  auto it = m_kalman.find (rnti);
  if (it == m_kalman.end () || !it->second.initialized)
    return false;
  if (maxAge <= Time (0))
    return true;  // 不做陈旧判断: 只要初始化即新鲜
  // lastUpdateSlot 单位是 slot (= now_ms / DtSlots); 折回时间做年龄比较。
  const double slotNow = Simulator::Now ().GetMilliSeconds () / m_dt;
  const double ageSlots = slotNow - it->second.lastUpdateSlot;
  if (ageSlots < 0.0)
    return true;  // 时钟回退/同 slot 视为新鲜
  const double ageMs = ageSlots * m_dt;
  return MilliSeconds (static_cast<int64_t> (ageMs)) <= maxAge;
}

double
CqiAmcController::GetLastUpdateSlot (uint16_t rnti) const
{
  auto it = m_kalman.find (rnti);
  if (it == m_kalman.end () || !it->second.initialized)
    return -1.0;
  return it->second.lastUpdateSlot;
}

void
CqiAmcController::SetHorizonFromTime (Time h)
{
  // H(slot) = round(h_ms / DtSlots), 至少 1。DtSlots=1 → 1 slot=1ms 自洽。
  double slots = h.GetMilliSeconds () / m_dt;
  if (slots < 1.0)
    slots = 1.0;
  m_horizonH = static_cast<uint32_t> (std::llround (slots));
  if (m_horizonH < 1)
    m_horizonH = 1;
}

double
CqiAmcController::GetEffectiveCqi (uint16_t rnti) const
{
  double pred = PredictCqi (rnti);
  auto it = m_olla.find (rnti);
  double d = (it == m_olla.end ()) ? 0.0 : it->second.delta;
  return Clamp (pred + d, 0.0, 15.0);
}

void
CqiAmcController::OnHarqFeedback (uint16_t rnti, bool ack, bool isNewTx)
{
  if (!isNewTx)
    return;  // 仅新传反馈更新 OLLA (IR 合并的重传 ACK/NACK 不污染)
  auto& o = m_olla[rnti];
  double stepDown = m_stepUp * (1.0 - m_blerTarget) / m_blerTarget;
  if (ack)
    {
      o.delta += m_stepUp;
      o.newTxAck++;
    }
  else
    {
      o.delta -= stepDown;
      o.newTxNack++;
    }
  o.delta = Clamp (o.delta, m_deltaMin, m_deltaMax);
  NS_LOG_DEBUG ("[OLLA] rnti=" << rnti << " " << (ack ? "ACK" : "NACK") << " Δ=" << o.delta);
}

double
CqiAmcController::GetDelta (uint16_t rnti) const
{
  auto it = m_olla.find (rnti);
  return (it == m_olla.end ()) ? 0.0 : it->second.delta;
}

double
CqiAmcController::GetMeasuredBler (uint16_t rnti) const
{
  auto it = m_olla.find (rnti);
  if (it == m_olla.end ())
    return 0.0;
  uint64_t tot = it->second.newTxAck + it->second.newTxNack;
  return (tot == 0) ? 0.0 : static_cast<double> (it->second.newTxNack) / tot;
}

uint64_t
CqiAmcController::GetNewTxCount (uint16_t rnti) const
{
  auto it = m_olla.find (rnti);
  if (it == m_olla.end ())
    return 0;
  return it->second.newTxAck + it->second.newTxNack;
}

double
CqiAmcController::GetKalmanL (uint16_t rnti) const
{
  auto it = m_kalman.find (rnti);
  return (it == m_kalman.end ()) ? 0.0 : it->second.L;
}

double
CqiAmcController::GetKalmanR (uint16_t rnti) const
{
  auto it = m_kalman.find (rnti);
  return (it == m_kalman.end ()) ? 0.0 : it->second.R;
}

} // namespace ns3
