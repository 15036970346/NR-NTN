# 基于 CQI 预测的 HARQ 增强 — 伪代码设计

> 面向 ns-3 `contrib/geo-sat`(6.5 版本)的实现蓝图。
> 本文聚焦**判决逻辑**:风险分公式、阈值、状态机、与现有 `CqiAmcController` / `HarqManager` / `GeoBeamScheduler` 的接缝。
> 代码级实现按本文"填空"即可。

---

## 0. 设计总纲(一句话)

把"全禁/全开 HARQ""主动重传""跨 RV"三个想法,重构为**一个预测驱动的 AMC+HARQ 联合策略选择器**:
在调度时刻读取 Kalman/OLLA 的四个信号 `(L, R, P, Δ)`,结合业务类型,输出一个 `HarqPolicy`,
再由调度器据此决定 **MCS 目标 BLER + HARQ 模式 + 重复份数 + RV 序列**。

```
                 ┌──────────────── CqiAmcController (已存在) ───────────────┐
   CQI 上报 ───▶ │  Kalman: L(电平) R(趋势) P(不确定度)   OLLA: Δ / 实测BLER │
                 └───────────────┬──────────────────────────┬──────────────┘
                                 │ (L,R,P,Δ)                 │
                                 ▼                           ▼
                    ┌─────────────────────────┐   业务类型(ServicePriority)
                    │  HarqPolicyController     │◀──────────────┘
                    │  (本文要新增的判决器)      │
                    └────────────┬─────────────┘
                                 │ HarqPolicy{mode,K,rvList,blerTarget}
                                 ▼
              ┌──────────────────────────────────────────────┐
   调度时刻 ─▶│ GeoBeamScheduler                              │
              │  1) MCS = f(effectiveCqi, blerTarget)         │  ← :911
              │  2) HarqManager.NewTransmission(...)          │  ← :2345
              │  3) 若 Tier2: 背靠背再发 K-1 份, 指定 RV      │
              └──────────────────────────────────────────────┘
```

---

## 1. 复用的现有接口(不要重造)

### 1.1 `CqiAmcController`(`model/cqi-amc-controller.h`)
| 接口 | 用途 |
|------|------|
| `PredictCqi(rnti)` | H 步前向预测 CQI(纯电平 L)|
| `GetEffectiveCqi(rnti)` | `predCqi + Δ`,scheduler 转 MCS 用(:912)|
| `GetKalmanL(rnti)` | **L** 预测电平 |
| `GetKalmanR(rnti)` | **R** 每 slot 变化率(趋势)← HARQ 增强核心信号 |
| `GetMeasuredBler(rnti)` | 实测 BLER,做误判闸门 |
| `GetDelta(rnti)` | OLLA Δ |
| `HasFreshEstimate(rnti, maxAge)` | 估计是否新鲜,决定走预测还是直通 |

> **缺口**:当前没有暴露 Kalman 协方差 P(不确定度)。需新增一个 getter,例如
> `double GetPredictVariance(rnti)`,返回 `P[0][0] + H^2 * P[1][1]`(前向预测方差)。
> 这是本设计唯一需要 controller 暴露的新信号。

### 1.2 `HarqManager`(`model/harq-manager.h`)
| 接口 | 现状 | 需要的改动 |
|------|------|-----------|
| `SetHarqEnabled(bool)` | **全局** `m_harqEnabled` | 改成**按 RNTI/进程**:`SetHarqEnabled(rnti, bool)` |
| `NewTransmission(rnti,pkt,mcs)` | 分配进程、首传 | 加可选入参 `rv0Override`(Tier2 指定首发 RV)|
| `GetNextRedundancyVersion(rnti,pid)` | 推进 0→2→3→1 | 加变体 `ForceRedundancyVersion(rnti,pid,rv)`(Tier2 跨 RV)|
| `GetAvailableProcessCount(rnti)` | 可用进程数 | 用于 Tier2 申请 K 份前的容量检查 |
| `CalculateIrGain(rvIndex)` | RV→增益因子(简化模型)| 不改;Tier2 的 RV 选择靠它体现增益差异 |

### 1.3 `GeoBeamScheduler`(`model/geo-beam-scheduler.cc`)接缝
- `:911-912` MCS 选择:`GetMcsFromCqi(GetEffectiveCqi(rnti), false)` → 改为 `GetMcsFromCqi(effCqi, false, policy.blerTarget)`
- `:2345` 新传:`NewTransmission(rnti, pkt, mcs)` → 前面插入 `policy = harqPolicy->Decide(...)`,后面据 `policy.mode` 决定是否背靠背补发
- `:2511` 重传 RV:`GetNextRedundancyVersion(...)` 保持(Tier1 用);Tier2 走 `ForceRedundancyVersion`

---

## 2. 核心数据结构

```cpp
enum class HarqMode {
    OFF,          // Tier 0: 关 HARQ, 单发 RV0, 不等 ACK, RLC ARQ 兜底
    REACTIVE,     // Tier 1: 标准反馈式 HARQ, RV 序列 0→2→3→1
    PROACTIVE     // Tier 2: 主动冗余, 调度时一次性发 K 份, 不等 NACK
};

struct HarqPolicy {
    HarqMode mode;
    uint8_t  K;            // 一次性发送份数 (Tier0=1, Tier1=1, Tier2=2~3)
    vector<uint8_t> rvList;// 各份的 RV, 如 {0,2}(IR合并) 或 {0,3}(自解码)
    double   blerTarget;   // 传给 AMC 的目标 BLER (Tier0 低/保守, Tier1/2 高/激进)
    uint8_t  maxRetx;      // 最大重传次数 (Tier0=0, Tier1=N, Tier2=0 或小)
};

// 判决用的瞬时信道画像
struct ChannelView {
    double L;        // 预测电平 (GetKalmanL)
    double R;        // 趋势斜率 (GetKalmanR), >0 变好, <0 变差
    double sigma;    // 预测标准差 = sqrt(GetPredictVariance)
    double bler;     // 实测 BLER (GetMeasuredBler)
    bool   fresh;    // HasFreshEstimate
};
```

---

## 3. 风险分公式(判决的数学核心)

把四个信号融合成一个 `[0,1]` 的**信道风险分** `risk`。越高越该上冗余。

```
设:
  cqiMin, cqiMax            = 0, 15
  H                         = controller 的前向预测步数 (m_horizonH, ~510)
  L_norm = clamp((L - cqiMin) / (cqiMax - cqiMin), 0, 1)   // 电平越高越安全

  // 1) 电平项: 预测电平低 → 风险高
  riskLevel = 1 - L_norm

  // 2) 趋势项: 在预测视界内信道会掉多少 CQI
  //    drop = -R * H (R<0 时为正, 代表掉落量); 归一化并截断
  drop      = clamp(-R * H, 0, cqiMax)
  riskTrend = drop / cqiMax

  // 3) 不确定度项: 预测越不可信, 越该保守上冗余
  riskUncert = clamp(sigma / SIGMA_REF, 0, 1)     // SIGMA_REF 经验值, 如 2.0 CQI

  // 4) 历史可靠性项: 实测 BLER 已超标 → 提高风险 (误判闸门的一部分)
  riskBler   = clamp(bler / BLER_REF, 0, 1)        // BLER_REF 如 0.1

  // 加权融合 (权重可调, 建议初值)
  risk = w1*riskLevel + w2*riskTrend + w3*riskUncert + w4*riskBler
         w1=0.40, w2=0.30, w3=0.20, w4=0.10
  risk = clamp(risk, 0, 1)
```

> **设计要点**:`riskTrend` 用了 `R*H`,即"趋势 × 预测视界"。这正是 GEO 特性 ——
> 一次重传要 600ms 后才落地,所以要看的不是"现在好不好",而是"600ms 后好不好"。
> 这是本方案区别于地面 HARQ 的关键。

---

## 4. 判决状态机(`HarqPolicyController::Decide`)

```
function Decide(rnti, ServicePriority prio) -> HarqPolicy:

    view = ReadChannelView(rnti)           // 见 §5

    // --- 新鲜度回退: 预测不可信时退回安全的 Tier1 ---
    if not view.fresh:
        return Policy(REACTIVE, K=1, rv={0}, blerTarget=BLER_NOMINAL, maxRetx=N)

    risk = ComputeRisk(view)               // §3

    // --- 业务感知优先 (GEO 下重传对语音无意义) ---
    if prio in {EMERGENCY, VOICE}:
        // 时延敏感: 永不走反应式 (600ms 重传 = 废包)
        if risk < TH_LOW:
            return Policy(OFF, K=1, rv={0}, blerTarget=BLER_SAFE, maxRetx=0)
        else:
            // 一次性前置冗余, 不等反馈
            rvs = SelectRv(view, risk)     // §6
            return Policy(PROACTIVE, K=len(rvs), rv=rvs, blerTarget=BLER_AGGR, maxRetx=0)

    // --- DATA / BEST_EFFORT: 吞吐导向, 容忍时延 ---
    if risk < TH_LOW:
        // 信道好且可信 → 关 HARQ 抢吞吐, AMC 调保守保证首传成功
        return Policy(OFF, K=1, rv={0}, blerTarget=BLER_SAFE, maxRetx=0)

    else if risk < TH_HIGH:
        // 中等 → 标准反馈式 HARQ
        return Policy(REACTIVE, K=1, rv={0}, blerTarget=BLER_NOMINAL, maxRetx=N)

    else:
        // 高风险/趋势下行/不可信 → 主动冗余, 不赌 600ms 后的重传机会
        rvs = SelectRv(view, risk)
        return Policy(PROACTIVE, K=len(rvs), rv=rvs,
                      blerTarget=BLER_AGGR, maxRetx=1)   // 留 1 次反应式兜底
```

### 阈值与参数初值(可在 Attribute 暴露,便于扫参)

| 参数 | 含义 | 建议初值 |
|------|------|----------|
| `TH_LOW` | risk < 此值 → Tier0 关 HARQ | 0.30 |
| `TH_HIGH` | risk ≥ 此值 → Tier2 主动冗余 | 0.65 |
| `BLER_SAFE` | Tier0 的 AMC 目标 BLER(保守)| 0.01 |
| `BLER_NOMINAL` | Tier1 的目标 BLER | 0.10 |
| `BLER_AGGR` | Tier2 首传目标 BLER(激进,有冗余兜底)| 0.30 |
| `SIGMA_REF` | 不确定度归一化基准 | 2.0 CQI |
| `BLER_REF` | BLER 项归一化基准 | 0.10 |
| `N` | Tier1 最大重传 | 4(对齐 `MAX_RETRANSMISSIONS`)|

---

## 5. 读取信道画像(`ReadChannelView`)

```
function ReadChannelView(rnti) -> ChannelView:
    H = ctrl.GetHorizonH()
    return {
        L     : ctrl.GetKalmanL(rnti),
        R     : ctrl.GetKalmanR(rnti),
        sigma : sqrt(ctrl.GetPredictVariance(rnti)),   // ← 需新增 getter
        bler  : ctrl.GetMeasuredBler(rnti),
        fresh : ctrl.HasFreshEstimate(rnti, MAX_AGE)
    }
```

---

## 6. RV 选择(模式 C,作为 Tier2 子模块)

```
function SelectRv(view, risk) -> vector<uint8_t>:
    // 第一份永远 RV0 (系统比特最全, 自解码)
    if risk < TH_RV_SELFDEC:
        // 略差: 第二份走 IR 合并增益
        return {0, 2}
    else:
        // 很差/不可信: 第二份用自解码 RV3, 整包丢也能独立站住
        return {0, 3}

    // 极端高风险且 K 可为 3 时, 可扩展 {0, 3, 2}: 自解码 + 合并双保险
```

> 真实 LDPC 中 RV{0,3} 偏自解码、RV{1,2} 偏合并增益。
> 你的 `CalculateIrGain(rvIndex)` 是简化增益模型,故此处体现为"不同 RV 给不同增益因子",逻辑成立、无需比特级精确。

---

## 7. 调度器集成(伪代码,对应 `geo-beam-scheduler.cc`)

```
// ===== MCS 选择处 (:911) =====
policy = harqPolicy->Decide(rnti, GetServicePriority(rnti))
effCqi = m_cqiAmc->GetEffectiveCqi(rnti)
mcs    = GetMcsFromCqi(effCqi, isUplink, policy.blerTarget)   // blerTarget 新增入参

// ===== 新传处 (:2345) =====
switch policy.mode:

  case OFF:
      harqManager->SetHarqEnabled(rnti, false)
      pid = harqManager->NewTransmission(rnti, pkt, mcs)      // 单发 RV0, 不登记等待
      // 偶发错误 → RLC AM ARQ 兜底, scheduler 不安排重传

  case REACTIVE:
      harqManager->SetHarqEnabled(rnti, true)
      pid = harqManager->NewTransmission(rnti, pkt, mcs)      // 等 ACK, RV 序列 0→2→3→1
      // 后续 NACK 走现有 :2511 GetNextRedundancyVersion 流程

  case PROACTIVE:
      harqManager->SetHarqEnabled(rnti, true)
      if harqManager->GetAvailableProcessCount(rnti) < policy.K:
          fallback → REACTIVE                                 // 进程不够, 降级
      else:
          for i in 0 .. policy.K-1:
              rv = policy.rvList[i]
              // 同一 TB 的 K 份占用连续时隙 (slot aggregation)
              pid = harqManager->NewTransmission(rnti, pkt, mcs, /*rv0Override=*/rv)
          // 不等 NACK; 若 maxRetx>0 则保留一次反应式兜底
```

---

## 8. 误判保护闭环(必须有,评审会问)

预测错的代价不对称,要让系统**自愈**:

```
在 OnHarqFeedback / RLC 状态回调里:
  measuredBler = ctrl.GetMeasuredBler(rnti)

  // 闸门 1: Tier0 关了 HARQ 却频繁出错 → 抬高关 HARQ 的门槛
  if lastMode == OFF and measuredBler > BLER_SAFE * GUARD_FACTOR:
      TH_LOW *= 0.9        // 收紧 (更难进 Tier0); 或对该 rnti 临时禁用 Tier0 一段时间

  // 闸门 2: Tier2 频繁"白发"(K 份里第一份其实就成功) → 降低主动冗余倾向
  if lastMode == PROACTIVE and firstCopyAckRate > 0.95:
      TH_HIGH *= 1.05      // 放宽 (更难进 Tier2), 省频谱
```

> 本质是复用 OLLA 的思想:让**实测 BLER 反过来调 HARQ 模式阈值**。
> Tier0 的最终安全网始终是 **RLC AM ARQ**(慢但保证正确)。

---

## 9. 评估指标与对比基线

**指标**:残余 BLER(HARQ 后)、吞吐/频谱效率、时延(语音"预算内到达率")、重传次数/浪费时隙、**预测对/错时各自表现**(鲁棒性)。

**基线**:
| 方案 | 说明 |
|------|------|
| (a) | HARQ 恒开固定 N(传统)|
| (b) | HARQ 恒关 |
| (c) | 仅 AMC 预测、HARQ 不动(你当前状态)|
| (d) | **本方案:预测式联合策略** |

目标:(d) 在"吞吐–时延–可靠性"三角里同时压住 (a)(b),且对 misprediction 鲁棒。

---

## 10. 实现清单(落地顺序)

1. `CqiAmcController` 新增 `GetPredictVariance(rnti)`(暴露 P 的前向方差)。
2. `HarqManager`:`m_harqEnabled` 改 per-RNTI;`NewTransmission` 加 `rv0Override`;新增 `ForceRedundancyVersion`。
3. 新建 `model/harq-policy-controller.{h,cc}`:`ChannelView` / `HarqPolicy` / `ComputeRisk` / `Decide` / `SelectRv` + 自愈闸门。
4. `GeoBeamScheduler`:`SetHarqPolicyController(...)`;MCS 处接 `blerTarget`;新传处接 `policy.mode` 分支。
5. `GetMcsFromCqi` 增加 `blerTarget` 入参(或用一组 CQI→MCS 表按目标 BLER 切换)。
6. 示例 + 统计:扩 `examples/ntn-amc-closed-loop.cc` 或新建 demo;在 `sat-stats-collector` 记录每次判决的 `(risk, mode, K, rvList)` 供分析。
7. 扫参对比 (a)(b)(c)(d),出三角折中图。

---

## 附:与你原始三模式的映射

| 你的原始想法 | 在本设计中的位置 |
|--------------|------------------|
| ① 全禁/全开 HARQ | **Tier 0 (OFF) ↔ Tier 1 (REACTIVE)**,且对齐 3GPP Rel-17 NTN 的"动态化"卖点 |
| ② 主动重传(不等反馈)| **Tier 2 (PROACTIVE)**,触发判据改为"调度时刻 margin/不确定度",而非"发完 5ms 后回看" |
| ③ 跨 RV(0→3 替 0→2)| **§6 SelectRv**,作为 Tier 2 子模块(自解码 RV{0,3} vs 合并 RV{1,2})|
