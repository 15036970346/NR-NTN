# GEO 卫星资源管理模块升级说明文档

> 适用范围：`contrib/geo-sat` 模块的资源管理三件套
> （`resource-manager`、`admit-control`、`geo-beam-scheduler`）+ CQI 预测接入 + NOMA + 闭环功控。
> 编写目的：完整记录本次改动的"做了什么、改在哪、起什么作用、是否满足需求、后续怎么走、CQI 预测如何接入"。

---

## 0. 一句话总览

本次工作围绕你的两个核心诉求展开：

1. **消费级/便携式终端的静态 + 动态频率域调度**（根据业务类型、信道质量动态分配 RB 数量）；
2. **功率域：动态功率调配 + 把开环功控改成基于实测 SNR 反馈的闭环功控**；

并额外打通了 **功率域 NOMA 资源共享** 与 **系统吞吐量 / 公平性统计**，同时把和同事的 **CQI 预测** 的对接口子预先钉好（接口先行、解耦）。

全部按 7 个步骤（Step 0~6）落地，工程可编译可运行，验证 **1689 项全部 PASS / 0 FAIL**。

---

## 1. 改动涉及的文件清单

| 文件 | 类型 | 改了什么 |
|---|---|---|
| `model/cqi-predictor.h` / `.cc` | **新增** | CQI 预测器抽象接口 + 直通默认实现 |
| `model/geo-beam-scheduler.h` / `.cc` | 改动（最大） | 预测注入、每-UE 路损、SPS、闭环功控、动态功率调配、NOMA |
| `model/resource-manager.h` / `.cc` | 改动 | 闭环功控项、波束功率预算+分配策略、NOMA 的 RB 复用记账 |
| `model/admit-control.h` / `.cc` | 改动 | 准入门控的 RB 倍率改可配 + 与调度器对齐 |
| `examples/resource-manager-validation.cc` | 改动 | OMA vs NOMA 的 A/B 对照（吞吐量 + 被服务用户数 + Jain 公平性）+ CLI 开关 |
| `CMakeLists.txt` | 改动 | 注册新增的 `cqi-predictor.{h,cc}` |
| `contrib/ns3-ai-main` → `contrib/ai` | **目录重命名** | 修复 ns3-ai 与 ns-3.40 的 CMake 不兼容（详见 §9） |

---

## 2. 按步骤：实现了哪些功能、改在哪、起什么作用

### Step 0 — CQI 预测器接口（解耦地基，行为零变化）

**目的**：把"预测 CQI"和调度器解耦。调度器只依赖一个抽象接口，不关心实现；同事的预测模型做好后派生它再注入即可，调度器一行不用改。

**新增 `model/cqi-predictor.{h,cc}`**：
- 抽象类 `CqiPredictor : public Object`：
  - `PredictDlCqi(rnti, measuredCqi, horizon)`、`PredictUlCqi(...)`：返回预测 CQI；
  - `RecordDlCqi(rnti, cqi, when)`、`RecordUlCqi(...)`：供预测器累积历史样本；
  - **默认实现是"直通"**：`Predict*` 原样返回 `measuredCqi`，`Record*` 空操作 → **等价于没有预测，行为与改动前完全一致**。

**`geo-beam-scheduler`**：
- 新增成员 `Ptr<CqiPredictor> m_cqiPredictor`（构造函数里 `CreateObject<CqiPredictor>()` 建一个直通实例）；
- 新增 `SetCqiPredictor() / GetCqiPredictor()`；
- 新增 attribute `DlSchedHorizon`(默认 300ms) / `UlSchedHorizon`(默认 600ms)：预测"提前量"，即从现在到实际传输时刻的时延（GEO 一个环回）。

**作用**：插座装好了。直通实现下系统行为不变；同事交付后只需 `SetCqiPredictor(自定义预测器)`。

---

### Step 1 — 每-UE 真实路径损耗（公共前置）

**问题**：原来上行功控/功率受限 RB 计算用一个**全局常量** `m_referencePathLossDb = 190 dB`，对所有终端一视同仁，不是真正的"按链路动态"。

**改动**：
- `SatUeContext` / `HandoverUeContext` 增加字段 `double pathLossDb`（切换时随上下文转移）；
- 新增 `DerivePathLossFromRsrp(rsrp)`：`路损 = RefSatEirpDbm − RSRP`，限幅 `[150, 220] dB`；
- 新增 attribute `RefSatEirpDbm`(默认 70 dBm)，使**默认 RSRP −120 → 路损 190 dB，与历史行为对齐**；
- `m_referencePathLossDb` 改为只在 **RA(Msg3) 阶段（此时还没有 UE 上下文）** 当回退值用；
- 在 `AddUeContext` 初始化、`ReceiveMeasReport` 用最新 RSRP 刷新 `pathLossDb`；
- 把 5 处带 UE 上下文的上行调用（`GetMaxPowerLimitedUlRbs`、`AdjustUtTxPower`）从 `m_referencePathLossDb` 换成 `ctx.pathLossDb`。

**作用**：上行功控/功率受限 RB 现在**随每个 UE 的实际 RSRP 变化**——消费级和便携式终端的上行 RB 上限按各自链路收敛，这是后续闭环功控和动态功率调配的基础。

> 注意：这是用"下行 RSRP"近似"上行路损"（GEO 上下行几何相同、频率不同），残余误差由 Step 3 的闭环功控纠正。`RefSatEirpDbm` 若知道卫星真实 EIRP 应改成真值。

---

### Step 2 — 频率域调度（接预测 CQI + 静态 SPS + 准入对齐）

这一步对应你的核心诉求"静态 + 动态频率域调度"。分三个子部分：

#### 2a. 把 `GetSchedulingCqi` 接到预测器（核心切口）

- 重写 `GetSchedulingCqi(ctx, isUplink)`：先取"实测/最近 CQI（带新鲜度回退）"，**再交给 `m_cqiPredictor` 预测"实际传输时刻"的 CQI**，结果 clamp 到 `[1,15]`；
- `UpdateUeDlCqi / UpdateUeUlCqi / UpdateUeCsi` 增加 `RecordDl/UlCqi` 钩子，供未来预测器累积历史。

**作用（关键）**：因为 `EstimateBytesPerRb → neededRbs`、`GetMcsFromCqi`、`CalculateSchedulerMetric`、上行功控 **全部经过 `GetSchedulingCqi` 这一个函数**，所以改这一处，**DL/UL 的 RB 数量、MCS、IPF 排序全链路自动变成"CQI/预测驱动"**。直通实现下行为不变；预测注入后立刻全链路生效。
> 你说的"动态分配 RB 数量"本来就有骨架（`neededRbs = ceil(buffer / bytesPerRb)`），本步让它真正吃上准确（可预测）的 CQI。

#### 2b. 静态 SPS（半持续调度，覆盖便携保底/应急/语音）

- `SatUeContext` 增加 `lastDlSpsTime / lastUlSpsTime`；
- 新增 `GetSpsFixedRbs(ctx)`（按 UE 类别返回固定 RB，0=不走 SPS）、`TryScheduleDlSps(...)`、`TryScheduleUlSps(...)`（UL 复用现成的 `ProcessUlGrant`）；
- 在 `RunScheduler`（DL）和 `RunUlSchedulerForBeam`（UL）的**动态调度之前**插入 SPS 预分配：对**便携式终端 / 应急 / 语音**按 `SpsPeriod` 周期发固定 RB 授权，从 `availableRbs` 扣除，被服务的 UE 本轮**跳过动态队列**（用 `spsServed` 集合）；
- 新增 attribute：`SpsEnabled`(默认 **false**)、`SpsVoiceRbs`(2)、`SpsEmergencyRbs`(1)、`SpsPortableFloorRbs`(2)、`SpsPeriod`(20ms)。

**作用**：实现"静态调度"——周期性小包业务（语音/应急/便携保底）用固定 RB、不每 TTI 算，降低开销、保证确定性时延；其余业务走动态。**对单个 UE 是二选一，对整个波束是静态+动态并行**。默认关，开启后才生效。

#### 2c. 准入控制与调度器对齐

- `admit-control` 把两个写死的倍率改为可配 attribute：`HighCapacityRbMultiplier`(1.5)、`ConsumerUlRbMargin`(0.15)，默认值保留原行为，可设 1.0/0 与调度器完全对齐；
- `geo-beam-scheduler` 新增 attribute `DataAdmitRbCap`(默认 4)：**DL 数据业务的准入门控从原来硬编码的"1 RB"改成 `min(CQI驱动需求, DataAdmitRbCap)`**。

**作用**：原来数据业务无论需多少 RB，只要 1 个 RB 空闲就放行（准入完全不看信道）。现在准入**真正看信道/业务量**（CQI 驱动且与调度器对齐）。设 `DataAdmitRbCap=1` 可恢复旧的宽松行为。

> ⚠️ 这是本步唯一的**默认行为变更**（你已确认接受）。

---

### Step 3 — 闭环功率控制（开环 → 闭环，CLPC）

这一步对应你的诉求"把开环功率估算改成基于实测 SNR 反馈的闭环功控"。

**功控公式升级**（`resource-manager` 的 `AdjustUtTxPower`）：
- 由纯开环 `P = P0 + 10·log10(M) + α·PL` → `P = P0 + 10·log10(M) + α·PL + f(i)`；
- 函数签名增加 `closedLoopOffsetDb`（默认 0，所以 Msg3/RA 阶段调用不受影响）。

**闭环回路**（`geo-beam-scheduler`）：
- `SatUeContext` / `HandoverUeContext` 增加 `clpcOffsetDb`（即每-UE 的累积修正项 `f(i)`，初值 0，切换时转移）；
- `DoSchedUlCqiInfoReq`（gNB 收到上行 CQI/SINR 的入口）里，拿到**实测 UL SINR** 后：
  - `AverageUlSinrDb(ulCqiInfo)` 求平均实测 SINR(dB)；
  - `UpdateClosedLoopPowerControl(rnti, measuredSinrDb)`：算 `error = 目标SINR − 实测SINR` → `QuantizeTpcDb(error)` 量化成 **3GPP TPC 命令集 {-1,0,+1,+3} dB** → `ApplyClpcDelta` 累积到 `clpcOffsetDb` 并限幅 `[ClpcMinDb, ClpcMaxDb]`；
- `ProcessUlGrant` 计算发射功率时把 `ctx.clpcOffsetDb` 传入 `AdjustUtTxPower`；
- **GEO 环路时延**：`ClpcLoopDelay` > 0 时用 `Simulator::Schedule` 延迟应用修正；默认 0（UL 反馈本身已是传播后、授权 DCI 下行还有时延，已隐含一个环回）。
- 新增 attribute：`ClpcEnabled`(默认 **true**)、`TargetUlSinrDb`(3)、`ClpcMinDb`(-10)、`ClpcMaxDb`(10)、`ClpcLoopDelay`(0)。

**作用**：把"发出去就不管"的开环，升级成"用实测 SINR 反向纠偏"的闭环——能纠正慢变（雨衰、仰角变化）。`ClpcEnabled=false` 可退回纯开环，供 A/B 对照。

> ⚠️ `ClpcEnabled` 默认 **true**（即闭环默认开启，正是你要的"改成闭环"）。GEO 长时延下闭环只能跟慢变、追不上快衰落，步长保守 + 限幅防发散。

---

### Step 4 — 波束总功率预算 + 动态功率调配

**问题**：原来每个 UE 独立算功率、**没有波束级总功率约束**，多个 UE 加起来可能超卫星功率。

**`resource-manager` 新增**：
- attribute `BeamPowerBudgetDbm`(默认 50 dBm)：每波束下行总功率预算；
- `PowerAllocPolicy` 枚举 + attribute `PowerAllocPolicy`：`POWER_EQUAL`(0,等功率) / `POWER_CHANNEL_INVERSE`(1,弱信道多给→拉公平) / `POWER_WATER_FILLING`(2,强信道多给→拉容量，简化注水)；
- `GetPowerWeight(cqi)`：按策略算 UE 权重；
- `ComputeDlTxPowerDbm(ueWeight, sumWeights)`：把预算转线性 mW，按权重比例切分再转回 dBm。**各 UE 份额之和 ≤ 预算 → 自动满足"不超总功率"约束**。

**`geo-beam-scheduler`**：
- 新增 attribute `DlPowerAllocEnabled`(默认 **false**)；
- 每轮统计候选 DL UE 的权重和 `dlPowerSumWeights`，DCI 构建时把 `txPowerDbm` 设为按预算+策略切分的功率。
- **UL 侧**：按你的要求受 `P_CMAX` 约束——已由 Step 3 的 `AdjustUtTxPower`(`min(pCmax,…)`) 保证。

**作用**：下行卫星总功率在各 UE 间动态分配（可选等功率/公平/容量策略）。默认关（因为 DL 功率值会影响下游 SINR，稳妥起见默认不动），开启后生效。

---

### Step 5 — 功率域 NOMA 共享（下行）

这一步对应你的诉求"动态功率域资源共享调度"。

**`resource-manager` 的 RB 复用模型**：
- `BeamAllocationUsage` 增加 `dlSharedRbs / ulSharedRbs`（NOMA 叠加量，仅统计、不占预算）；
- `AllocateSharedSpectrum(beamId, rbs, isUplink)`：在主用户已占的 RB 上**叠加**从用户，**不消耗 RB 预算**（同频靠功率域区分），返回批准叠加数；
- `GetSharedRbs(...)` 供统计；`ResetBeamAllocation` 一并清零。

**`geo-beam-scheduler` 的 NOMA 流程**（默认关）：
1. **配对** `BuildNomaPairs(beam, candidates)`：每波束在非 SPS 候选里按 CQI 排序，**最强 near 配最弱 far**，要求 CQI 差 ≥ `NomaMinCqiGap`，2 用户一对，贪心向内收；
2. **far 剔除**：配对的 far UE 放进 `m_nomaFarThisRound`，从动态队列跳过；
3. **SIC 有效 SINR** `ComputeNomaEffectiveCqi(nearCqi, farCqi)`（标准两用户线性模型，β=`NomaFarPowerFraction`）：
   - far：`SINR_eff = β·γ_far / (1 + (1-β)·γ_far)`（把近端当干扰）；
   - near：`SINR_eff = (1-β)·γ_near`（SIC 后无组内干扰）；
   - `SinrDbFromCqi / CqiFromSinrDb` 做 CQI↔SINR 近似互换（约 2dB/级）；
4. **near 降级 + far 叠加**：near 用降级后的有效 CQI 算 MCS/RB；near DCI 发出后，`EmitNomaFarGrant` 在**同一组 RB** 上给 far 发叠加授权（`AllocateSharedSpectrum`，不占预算）。
- 新增 attribute：`NomaEnabled`(默认 **false**)、`NomaFarPowerFraction`(0.8)、`NomaMinCqiGap`(4)。

**作用**：让一块 RB 同时服务"近+远"两个用户，在 RB 紧张的卫星场景里多服务弱用户。near 的功率降级在调度时就反映进 bytesPerRb（无事后修正问题）。

> 建模简化（如实记录）：β 功率劈分通过有效 CQI 体现；CQI↔SINR 用线性近似；只做 DL、2 用户配对（结构可扩展）。

---

### Step 6 — 系统吞吐量统计 + A/B 对照开关

**`examples/resource-manager-validation.cc`**：
- 新增 `DlScenarioResult { totalBytes, servedUes, totalUes, jainIndex }`；
- 新增 `RunDlThroughputScenario(noma, clpc, sps)`：构造固定混合信道场景（3 强 + 3 中等 UE，单波束）跑一轮 DL 调度，按 DCI 累计 `bytes = TBS(mcs, RB)`（自动涵盖 NOMA far 叠加授权），并算被服务用户数与 **Jain 公平性指数**；
- 新增 CLI 开关：`--enableNoma`、`--enableSps`、`--powerControl=openloop|closedloop`；
- 新增 `Test 12` 区段：打印 OMA vs NOMA 的 **吞吐量 / 被服务用户数 / 公平性** 对照 + 增量，并断言 `NOMA 服务用户数 ≥ OMA`、`NOMA 公平性 ≥ OMA`。

**作用**：可量化对比开/关各特性的系统级效果，是你出图/答辩用的对照实验入口。

**当前 A/B 实测结果**（默认场景）：
```
            服务用户数(共6)   Jain公平性    系统吞吐
OMA              3            0.4999        127 bytes
NOMA             4            0.5179        111 bytes
→ 吞吐 -12.6% | 被服务 +1 | 公平性 +0.018
```
**解读**：NOMA 在当前简化模型里体现为"**牺牲少量峰值吞吐，换取多服务边缘用户 + 更高公平性**"——这是 NOMA 的真实价值，需用"服务用户数/公平性"而非单看吞吐量来评价（详见 §7、§8）。

---

## 3. 配置速查表（所有新增/改动的 attribute）

| 模块 | Attribute | 默认 | 作用 |
|---|---|---|---|
| scheduler | `DlSchedHorizon` / `UlSchedHorizon` | 300ms / 600ms | CQI 预测提前量（GEO 环回） |
| scheduler | `RefSatEirpDbm` | 70 | 由 RSRP 反推路损的参考 EIRP（路损=EIRP−RSRP） |
| scheduler | `ReferencePathLossDb` | 190 | 仅 RA(Msg3) 阶段的回退路损 |
| scheduler | `DataAdmitRbCap` | 4 | 数据业务准入门控的 CQI 驱动 RB 上限（设 1 恢复旧行为） |
| scheduler | `SpsEnabled` | false | 是否启用静态 SPS |
| scheduler | `SpsVoiceRbs` / `SpsEmergencyRbs` / `SpsPortableFloorRbs` | 2 / 1 / 2 | 语音/应急/便携保底的 SPS 固定 RB |
| scheduler | `SpsPeriod` | 20ms | SPS 授权周期 |
| scheduler | `ClpcEnabled` | **true** | 是否启用上行闭环功控 |
| scheduler | `TargetUlSinrDb` | 3 | 闭环目标 SINR |
| scheduler | `ClpcMinDb` / `ClpcMaxDb` | -10 / 10 | 闭环修正项 f(i) 限幅 |
| scheduler | `ClpcLoopDelay` | 0 | 闭环额外环路时延 |
| scheduler | `DlPowerAllocEnabled` | false | 是否启用下行动态功率调配 |
| scheduler | `NomaEnabled` | false | 是否启用下行 NOMA |
| scheduler | `NomaFarPowerFraction` | 0.8 | NOMA 远端功率占比 β |
| scheduler | `NomaMinCqiGap` | 4 | NOMA 配对最小 CQI 差 |
| resource-manager | `BeamPowerBudgetDbm` | 50 | 每波束下行总功率预算 |
| resource-manager | `PowerAllocPolicy` | 0(equal) | 功率分配策略(0等功率/1信道反比/2注水) |
| admit-control | `HighCapacityRbMultiplier` | 1.5 | 高容量业务准入 RB 余量(1.0=完全对齐) |
| admit-control | `ConsumerUlRbMargin` | 0.15 | 消费级终端上行额外 RB 余量 |

> **默认行为小结**：除 `ClpcEnabled`(true) 和 `DataAdmitRbCap`(变信道感知) 外，SPS / 动态功率调配 / NOMA **默认全关**，CQI 预测为直通——即默认行为基本与改动前一致，新特性按需开启。

---

## 4. 如何构建与验证

```bash
# 从 ns-3-dev 根目录
./ns3 build geo-sat
./ns3 build resource-manager-validation
./ns3 run resource-manager-validation                       # 全量单测 + A/B
./ns3 run "resource-manager-validation --enableNoma=1"      # 自定义 A/B
./ns3 run "resource-manager-validation --powerControl=openloop"
```
当前验证结果：**PASS=1689 / FAIL=0**。

---

## 5. 现在是否满足你的要求？

| 你的诉求 | 状态 | 说明 |
|---|---|---|
| 消费级/便携式 **静态**频率域调度 | ✅ | SPS（便携保底/应急/语音），`SpsEnabled` 开启 |
| **动态**频率域调度（按业务类型+信道质量分 RB 数量） | ✅ | 骨架已有，本次接上（预测）CQI 驱动 + 准入对齐 |
| 与同事 **CQI 预测**结合 | ✅(接口就绪) | 直通占位，待同事交付后注入（见 §6） |
| **动态功率调配** | ✅ | 波束总功率预算 + 三种分配策略，`DlPowerAllocEnabled` 开启 |
| 开环 → **闭环功控**（基于实测 SNR 反馈） | ✅ | CLPC 默认开启 |
| **频率域调度 + 功率域 NOMA 共享** | ✅ | 两者分层叠加，`NomaEnabled` 开启 |
| **统计系统吞吐量** | ✅ | A/B：吞吐量 + 被服务用户数 + Jain 公平性 |

**结论**：功能层面**全部实现且可运行**。两个保留项：①CQI 预测目前是直通占位（等同事）；②NOMA 在当前简化模型下体现为"公平/覆盖增益"而非"吞吐增益"（见 §7、§8）。

---

## 6. 同事的 CQI 预测做好后，我该怎么加进来？

**只需三步，调度器代码一行不改：**

**① 派生一个预测器类**（例如 `model/ml-cqi-predictor.{h,cc}`）：
```cpp
class MlCqiPredictor : public CqiPredictor {
public:
  static TypeId GetTypeId();
  // 重写预测：内部调用同事的模型(可经 contrib/ai / ns3-ai 与 Python 交互)
  double PredictDlCqi(uint16_t rnti, double measuredCqi, Time horizon) const override;
  double PredictUlCqi(uint16_t rnti, double measuredCqi, Time horizon) const override;
  // 用历史样本喂模型(可选)
  void RecordDlCqi(uint16_t rnti, double cqi, Time when) override;
  void RecordUlCqi(uint16_t rnti, double cqi, Time when) override;
};
```
- `Record*`：调度器每次更新 CQI 时会自动调用 → 在这里把样本喂给同事的时间序列/ML 模型；
- `Predict*`：返回"`horizon` 之后那一刻"的预测 CQI（同事模型的输出）。

**② 在场景里注入**：
```cpp
Ptr<MlCqiPredictor> predictor = CreateObject<MlCqiPredictor>();
geoScheduler->SetCqiPredictor(predictor);
```

**③（可选）调时延**：把 `DlSchedHorizon / UlSchedHorizon` 设成你场景的实际环回时延。

**就这样。** 因为 `GetSchedulingCqi` 是 CQI 的唯一汇聚点，注入后 RB 数量 / MCS / IPF 排序 / 上行功控 / NOMA 配对全链路自动变成"预测驱动"。

---

## 7. 接入 CQI 预测后会起什么作用？与现在的结果有何不同？

**现在（直通）**：`GetSchedulingCqi` 返回的是**最近一次实测 CQI**；GEO 下这个反馈**滞后约一个环回(~600ms)**，过期还会回退到保守值(CQI=1)。也就是说，调度器是在"**用旧信道信息**"做决策。

**接入预测后**：返回的是"**传输那一刻的预测 CQI**"，抵消 GEO 长时延导致的滞后。具体差异：

| 维度 | 现在(直通/实测) | 接入预测后 |
|---|---|---|
| RB 数量估计 | 基于旧 CQI，可能多分/少分 | 基于预测 CQI，更贴合实际信道 |
| MCS 选择 | 易选错(信道已变) → 误块/重传 | 更准 → HARQ 重传减少 |
| 吞吐量 | 受滞后拖累 | 提升（尤其信道变化快/雨衰场景） |
| 边缘/移动用户 | 旧 CQI 偏差大 | 预测补偿，分配更合理 |
| NOMA 配对 | 用旧 CQI 配对，可能选错近/远端 | 配对更准，SIC 更稳 |

**一句话**：预测把"看着后视镜开车"变成"看着前方开车"。在 GEO ~600ms 环回下，这正是预测最大的价值所在。**结果差异**主要体现在：**有效吞吐量上升、HARQ 重传率下降、MCS 误选减少**——你可以用同一个 A/B 框架，对比"直通预测器 vs 同事的预测器"来量化这个增益。

---

## 8. 后续还有哪些可改进的地方

1. **NOMA 吞吐增益**：当前"全带宽标量 CQI + RB 数量级 + winner-take-all"模型下，NOMA 体现为公平/覆盖增益而非吞吐增益。若要 NOMA 连吞吐都正增益，需升级到 **per-RB / 子带 SINR 的链路-系统映射**（需引入子带 CQI，与当前"全带宽标量"相反，是较大的模型升级）。
2. **下行功率对 SINR 的闭环影响**：`DlPowerAllocEnabled` 目前默认关，因为 DL 功率值会影响下游 SINR 计算，建议在能完整跑端到端场景后再开启验证。
3. **`RefSatEirpDbm` 标定**：用卫星真实每波束 EIRP 替换默认 70，路损估计才更准。
4. **闭环功控的发散保护**：GEO 长时延下若把 `ClpcLoopDelay` 设为真实环回，需复核步长/限幅，必要时引入更平滑的 TPC 策略。
5. **NOMA 多用户(K>2)**：当前 2 用户配对，结构可扩展到 K>2（需权衡 SIC 误差传播）。
6. **统计维度扩展**：可加边缘用户吞吐量、HARQ 重传率、SINR 收敛曲线等，配合 `SatStatsCollector`。
7. **端到端场景验证**：当前主要用 `resource-manager-validation` 直接驱动调度器 API；建议在 `ntn-system-sim` 这类完整网络场景里再跑一遍，验证与 RLC/PDCP/信道的联动。

---

## 9. 构建环境修复记录（protobuf + ns3-ai）

本次还顺带打通了构建环境：

1. **protobuf**：你已在终端 `sudo apt install -y protobuf-compiler libprotobuf-dev`，CMake 现在能找到 Protobuf 3.6.1。
2. **ns3-ai 目录重命名**：`contrib/ns3-ai-main` 的 `build_lib(LIBNAME ai)` 期望目录名为 `ai`（ns-3 用目录名预设 `lib<目录名>` 变量），目录名不匹配导致 `${libai}` 为空 → 三个 CMake 报错。**已把目录重命名为 `contrib/ai`**，三个报错消失，`./ns3 build ai` 与 `geo-sat` 均通过。
   > 提示：若有脚本/文档按旧名 `ns3-ai-main` 引用路径，需同步改成 `ai`；C++ 侧 `#include "ns3/ai-module.h"` 不受影响（按模块名 `ai` 生成）。

---

## 10. 关键名词速查

- **OMA（正交多址）**：一块 RB 同一时刻只服务一个用户（现网做法）。
- **NOMA（非正交多址）**：多个用户共享同一块 RB，靠功率高低 + 接收端 SIC 区分。
- **SIC（串行干扰消除）**：接收端先解功率大的信号、减掉，再解自己的。
- **SPS（半持续调度）**：周期性固定 RB 授权，不每 TTI 决策，适合语音等周期小包。
- **CLPC（闭环功控）**：用实测 SINR 与目标比较，量化 TPC 命令累积修正发射功率。
- **TPC 命令**：3GPP 功率控制命令集 {-1, 0, +1, +3} dB。
- **Jain 公平性指数**：`(Σx)²/(n·Σx²)`，值域 (0,1]，1=完全公平，1/n=一人独吞最不公平；被饿死的用户(0)会拉低它。
- **f(i)**：3GPP PUSCH 功控公式里的闭环累积修正项。

---

*本文档由本次升级工作自动整理，覆盖 Step 0~6 全部改动。如需进一步细化某一步或某个函数的实现细节，可在对应文件按上述函数名定位。*
