# Resource Management Code Explanation

本文针对以下三份核心代码做联合说明：

- [resource-manager.cc](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:1)
- [admit-control.cc](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:1)
- [geo-beam-scheduler.cc](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1)

目标不是逐行翻译，而是把这三份代码在“资源管理模块”中的职责、调用关系、关键决策点和实际执行流程讲清楚。

## 1. 总体定位

这三份代码组成的是一个三层结构：

1. `ResourceManager` 负责最底层的“物理预算审批”。
2. `AdmitControl` 负责“是否允许接入/是否建议切换/是否要负载均衡”。
3. `GeoBeamScheduler` 负责“真正按 TTI 组织下行/上行调度，并把前两者串起来执行”。

可以把它理解成：

- `ResourceManager` 回答的是：`还能不能批 RB，最多能批多少，功率上限允许多少`。
- `AdmitControl` 回答的是：`这个 UE 现在该不该进这个 beam，是否该排队，是否该重定向`。
- `GeoBeamScheduler` 回答的是：`这一轮先调谁，给多少 RB，什么时候发 DCI，什么时候执行切换`。

三者的直接调用关系如下：

- `GeoBeamScheduler -> AdmitControl`：做准入判断、候选 beam 推荐、切换收益判断、负载均衡触发。
- `GeoBeamScheduler -> ResourceManager`：做每轮 RB 重置、RB 审批、UE 上行功率限制和发射功率计算。
- `AdmitControl -> ResourceManager`：读取当前 beam 的物理剩余 RB，作为准入与负载判断的底层依据。

关键绑定点在：

- [GeoBeamScheduler::GeoBeamScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:69)
- [GeoBeamScheduler::SetAdmitControl()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1743)
- [AdmitControl::SetResourceManager()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:636)

## 2. `GeoBeamScheduler` 中哪些部分属于你的资源管理逻辑

`GeoBeamScheduler` 文件很大，但不是所有代码都属于你的资源管理核心。

### 2.1 属于“框架外壳 / 对接接口”的部分

这些部分主要是为了接入 ns-3 / nr 框架，本身不是你的资源管理算法核心：

- `TypeId` 注册与属性声明的壳：[GetTypeId()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:19)
- 空的 SAP 接口占位符：
  - [DoCschedCellConfigReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:105)
  - [DoCschedUeConfigReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:106)
  - [DoCschedLcConfigReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:107)
  - [DoCschedLcReleaseReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:108)
  - [DoCschedUeReleaseReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:109)
  - [DoSchedDlCqiInfoReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:117)
  - [DoSchedUlCqiInfoReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:118)
  - [DoSchedUlMacCtrlInfoReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:119)
  - [DoSchedSetMcs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:154)
  - [DoSchedDlRachInfoReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:155)
- 一些简单桥接函数：
  - [DoSchedDlTriggerReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:160)
  - [SendDciToUe()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:895)
  - [RegisterUeDciCallback()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:914)

这些代码是必要的，但不是你的资源调度思想本身。

### 2.2 属于“你的资源管理模块核心”的部分

下面这些才是 `GeoBeamScheduler` 中真正和你这套资源管理/调度设计直接相关的实现：

- 资源管理参数入口：
  - `DefaultK2Delay`、`EmergencyDelayThresholdSeconds`、`ReferencePathLossDb`
  - `SrGrantRbs`、`Msg3RequestedRbs`、`Msg3GrantMcs`
  - 见 [GetTypeId()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:19)
- 调度器内部资源管理组件初始化：
  - [GeoBeamScheduler::GeoBeamScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:69)
- PRACH 下行预留：
  - [ReservePrachResources()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:173)
- UE 上下文、beam 绑定、切换上下文迁移：
  - [AddUeContext()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:183)
  - [AddUeInfo()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:211)
  - [RemoveUt()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:228)
  - [ExportUeContext()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:256)
  - [ImportUeContext()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:295)
  - [ExecuteHandover()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:343)
- 调度输入维护：
  - [UpdateUeCsi()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:352)
  - [UpdateUeDlBufferStatus()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:359)
  - [UpdateUeUlBufferStatus()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:368)
- 核心算法：
  - [GetMcsFromCqi()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:381)
  - [EstimateBytesPerRb()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:409)
  - [CalculateLocationFactor()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:426)
  - [CalculateSchedulerMetric()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:441)
  - [GetEffectiveUlDemandBytes()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:465)
  - [RefreshPendingUlGrantEstimate()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:473)
- 下行主调度：
  - [RunScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:490)
- 上行输入与收敛：
  - [ReceivePucchInfo()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:922)
  - [ReceiveBsr()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:965)
  - [ReceiveUlMacPdu()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:991)
- 上行授权与上行主调度：
  - [ProcessUlGrant()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1336)
  - [BeginUlSchedulingPeriod()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1412)
  - [RunUlScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1446)
  - [RunUlSchedulerForBeam()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1456)
- 优先级和权重：
  - [MapTrafficTypeToPriority()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1689)
  - [CalculateWrrWeight()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1704)
- 准入控制绑定：
  - [SetAdmitControl()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1743)
  - [CheckHandoverAdmission()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1758)
- 接入过程中的资源审批：
  - 4 步 RA： [ProcessPrachWindow()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1077) 和 [ProcessMsg3Window()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1244)
  - 2 步 RA： [ProcessMsgAWindow()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1845)

## 3. `ResourceManager` 详解

### 3.1 模块职责

`ResourceManager` 是最底层的资源审批器。它不做“谁优先”这种策略问题，它只做两件事：

1. 这个 beam 当前还剩多少 RB。
2. 这个 UE 在给定路径损耗和终端功率上限下，最多能拿多少 UL RB。

对应代码入口在 [GetTypeId()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:17)。

### 3.2 核心数据

- `m_dlBeamBudgetRbs` / `m_ulBeamBudgetRbs`
  - 每波束 DL / UL 的总预算。
  - 默认都是 `25` RB。
- `m_beamUsageMap`
  - 记录每个 beam 本轮已经用了多少 DL / UL RB。
- `m_maxEirpPortable = 63 dBm`
  - 对应便携式终端 `33 dBW`。
- `m_maxEirpConsumer = 50 dBm`
  - 对应消费级终端 `20 dBW`。

定义位置见 [ResourceManager::ResourceManager()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:46)。

### 3.3 关键函数

#### 3.3.1 `ResetBeamAllocation()`

位置：[ResetBeamAllocation()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:67)

作用：

- 每一轮调度开始时，把某个 beam 在某个方向上的“已用 RB”清零。
- 调度器每轮要先重置，再重新审批，避免上一轮账目污染下一轮。

#### 3.3.2 `GetRemainingRbs()`

位置：[GetRemainingRbs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:90)

作用：

- 返回 `预算 - 已用量`。
- 如果这个 beam 还没有建账，默认认为剩余就是整波束预算。

这是 `AdmitControl` 和 `GeoBeamScheduler` 都要读的最基础接口。

#### 3.3.3 `AllocateSpectrum()`

位置：[AllocateSpectrum()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:105)

作用：

- 输入：`beamId + requestedRbs + isUplink`
- 输出：真实批准的 `approvedRbs`

逻辑：

1. 读取当前剩余 RB。
2. 取 `min(requestedRbs, remainingRbs)`。
3. 把批准值写回 beam 用量。
4. 返回审批结果。

这是真正的“记账落地”动作。调度器只能提议，最终还是它审批。

#### 3.3.4 `GetMaxPowerLimitedUlRbs()`

位置：[GetMaxPowerLimitedUlRbs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:138)

作用：

- 根据终端类型、路径损耗、功控参数，计算“在不发生功率削顶时，上行最多能拿多少 RB”。

这是你这套资源管理里最关键的物理约束之一。

公式对应思想是：

- UE 总发射功率有上限。
- 分配 RB 越多，总功率需求越高。
- 路损越大，所需补偿功率越高。
- 所以 `UL RB` 不是只看 beam 预算，还要看终端是不是“带得动”。

#### 3.3.5 `AdjustUtTxPower()`

位置：[AdjustUtTxPower()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:164)

作用：

- 在给定 `allocatedRbs` 和路径损耗下，计算最终 UE 发射功率。
- 结果被 `pCmax` 截断，也就是不能超过终端物理上限。

这个函数主要被上行 DCI / Msg3 / MsgB 授权时调用，用来把“预估发射功率”写到控制信息里。

## 4. `AdmitControl` 详解

### 4.1 模块职责

`AdmitControl` 解决的是“逻辑上是否应该让 UE 进入这个 beam”的问题。

它不负责真正扣账，只负责在调度前做门控：

- 当前 beam 能不能收这个 UE。
- 收了以后会不会超过阈值。
- 不合适的话，是排队、重定向还是拒绝。
- 全局看起来负载差太大时，要不要触发重平衡。

入口见 [CanAdmitUe()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:76)。

### 4.2 关键策略参数

在 [GetTypeId()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:18) 中定义了这套策略的静态参数：

- `EmergencyReservedRbs = 1`
- `EmergencyBurstCapRbs = 3`
- `VoiceReservedRbs = 2`
- `AdmissionThreshold = 0.9`
- `HandoverBenefitThreshold = 0.15`
- `EmergencyUserCapPerBeam = 10`

这决定了你的准入控制是“刚性预留 + 阈值门控 + 收益驱动切换”的风格。

### 4.3 `CanAdmitUe()` 的完整逻辑

位置：[CanAdmitUe()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:76)

执行顺序：

1. 如果目标 beam 还没建档，就先创建资源状态。
2. 如果是应急业务，先检查该波束应急 UE 数量是否超上限。
3. 调用 [CalculateEffectiveRequiredRbs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:821) 计算“有效需求”。
4. 调用 [GetAvailableRbs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:663) 计算这个优先级下真正可用的 RB。
5. 如果 `需求 > 可用`：
   - 尝试 [GetRecommendedBeams()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:361)
   - 有替代 beam 就 `ADMIT_REDIRECT`
   - 没有就 `ADMIT_REJECTED`
6. 如果资源够，再算“接入后的利用率”。
7. 如果接入后超过该业务对应的动态阈值：
   - 应急业务允许突破
   - 非应急业务返回 `ADMIT_QUEUE`
8. 否则返回 `ADMIT_OK`

这里可以看出：

- `ResourceManager` 解决的是“物理还能不能分”。
- `AdmitControl` 解决的是“逻辑上现在该不该分”。

### 4.4 资源可用量是怎么按优先级切的

位置：[GetAvailableRbs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:663)

这段代码体现了你的“业务资源策略”：

- 应急业务：
  - 可见 `1 RB` 刚性预留
  - 紧急时最多放大到 `3 RB`
- 语音业务：
  - 看到的可用资源要先扣掉应急预留
  - 再最多拿到 `2 RB`
- 普通数据：
  - 只能使用扣掉 `应急预留 + 语音预留 + DL 保留` 之后的剩余资源

也就是说，`AdmitControl` 不是简单看“beam 还剩多少”，而是看“在当前业务等级下，这部分剩余里有多少是你能碰的”。

### 4.5 推荐 beam 的打分逻辑

位置：[GetRecommendedBeams()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:361)

这段代码不是只看剩余 RB，而是多项打分：

- `headroomScore`
  - 接入后还剩多少资源
- `utilizationPenalty`
  - 当前利用率越高越差
- `activeUePenalty`
  - UE 数太多越差
- `samePriorityPenalty`
  - 同类业务过多越差
- `trafficPenalty`
  - 高容量业务、消费级手机 UL 会被额外保守化
- `priorityBonus`
  - 应急业务更愿意去一个没有应急 UE 的 beam

这就是你的“负载均衡候选 beam 选择器”。

### 4.6 切换收益判断

位置：[CanHandoverUe()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:155) 和 [CalculateHandoverBenefit()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:738)

思想是：

1. 目标 beam 先得接得住。
2. 接得住以后，还要看切过去到底值不值。
3. 收益不足时，不直接切，先看全局负载均衡是否需要。

收益由三部分构成：

- 目标 beam 接入后的剩余空间
- 源 beam 被释放后的缓解程度
- 业务类型带来的额外奖励或惩罚

### 4.7 负载均衡流程

入口：

- [CheckBeamLoadBalancing()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:454)
- [TriggerLoadRebalancing()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/admit-control.cc:503)

执行逻辑：

1. 遍历所有 beam，更新当前利用率。
2. 找出最大负载和最小负载 beam。
3. 如果两者差超过 `0.3`，触发重平衡。
4. 把 `>80%` 的 beam 视为高负载，`<40%` 的 beam 视为低负载。
5. 从高负载 beam 中挑候选 UE。
6. 候选 UE 先按“优先级、RB 需求、连接时间”排序。
7. 对每个 UE 查询推荐 beam，再调用 `CanHandoverUe()` 复核。
8. 真允许切换时，回调 `GeoBeamScheduler::ExecuteHandover()` 执行。

注意这里的职责边界非常清楚：

- `AdmitControl` 只决定“谁该迁、迁去哪里”。
- 真正的上下文搬运在 `GeoBeamScheduler` 里完成。

## 5. `GeoBeamScheduler` 详解

### 5.1 模块职责

`GeoBeamScheduler` 是三者中的执行中枢。

它维护：

- UE 上下文
- beam 到 UE 的映射
- DL / UL buffer
- CQI / RSRP / 位置
- pending UL grant
- 调度历史吞吐

核心数据入口：

- [AddUeContext()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:183)
- [AddUeInfo()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:211)

其中 `SatUeContext` 里最关键的字段有：

- `dlBufferStatus`
- `ulBufferStatus`
- `pendingUlGrantRbs`
- `pendingUlGrantBytes`
- `latestCqi`
- `priority`
- `wrrWeight`
- `dlAverageThroughput`
- `ulAverageThroughput`
- `lastDlScheduledTime`
- `lastUlScheduledTime`

### 5.2 调度前的输入是怎么进入调度器的

#### 下行输入

来自 [DoSchedDlRlcBufferReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:110)：

- RLC 把传输队列、重传队列、状态 PDU 大小上报给调度器。
- 调度器把这些加总，写入 `dlBufferStatus`。

#### 上行输入

上行有三条输入链：

- `PUCCH SR / CQI`
  - [ReceivePucchInfo()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:922)
- `BSR`
  - [ReceiveBsr()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:965)
- 真实 UL MAC PDU 到达
  - [ReceiveUlMacPdu()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:991)

这三条链分别提供：

- `SR`：告诉调度器“我需要上行调度了”
- `BSR`：告诉调度器“我缓存里大概有多少待发字节”
- `UL MAC PDU`：告诉调度器“真实数据已经送达，应该冲减缓存和 pending grant”

### 5.3 速率与优先级是怎么量化的

#### CQI -> MCS

位置：[GetMcsFromCqi()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:381)

作用：

- 把 `CQI` 映射成一个较粗粒度的 `MCS`。
- 先做一点保守化 `cqi - 0.5`，再分段映射到 MCS。

#### MCS -> 每 RB 可承载字节数

位置：[EstimateBytesPerRb()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:409)

作用：

- 用一个简化频谱效率表，把 `MCS` 转成 `bytesPerRb`。
- 这是整个调度器做“buffer -> RB 需求”换算的核心。

#### WRR 权重

位置：[CalculateWrrWeight()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1704)

当前权重：

- 应急：`8.0`
- 语音：`4.0`
- 数据：`2.0`
- 尽力而为：`1.0`
- 消费级终端额外乘 `1.2`

#### IPF 指标

位置：[CalculateSchedulerMetric()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:441)

它综合了：

- 估计瞬时速率
- 位置因子
- 距离上次调度的等待时间
- 当前平均吞吐
- 需求大小
- CQI 提升
- 业务紧急度增益
- WRR 权重

也就是说，你这里的“第二级调度”本质上是一个带 GEO 场景修正项的比例公平调度。

### 5.4 下行调度主流程

入口：[RunScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:490)

完整逻辑如下。

#### 第一步：预留 PRACH 资源

- 通过 [ReservePrachResources()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:173) 固定扣 `6 RB`。
- 同时把这件事同步给 `AdmitControl`。

#### 第二步：为每个 beam 重置本轮 DL 账本

- 调用 [ResourceManager::ResetBeamAllocation()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:67)
- 再调用 [GetRemainingRbs()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:90)
- 然后减去 `6 RB PRACH`，得到本轮真正可用于业务的 DL 预算

对应代码在 [RunScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:533)。

#### 第三步：准入门控并构建业务队列

对应代码在 [RunScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:543)。

每个 UE 先经历：

1. 检查是否有真实 `dlBufferStatus`
2. 估算 `rawRequiredRbs`
3. 收敛出用于准入门控的 `gateRequiredRbs`
4. 把 `latestCqi + gateRequiredRbs` 同步给 `AdmitControl`
5. 调用 `CanAdmitUe()`

根据返回值处理：

- `ADMIT_OK`
  - 进入正常业务队列
- `ADMIT_QUEUE`
  - 进入等待队列，不参加本轮调度
- `ADMIT_REDIRECT`
  - 查询候选 beam，找到能接的就记录为稍后执行的重定向动作

最后形成三个逻辑队列：

- `emergencyUes`
- `voiceUes`
- `normalUes`

#### 第四步：第一级调度，先切业务预算

对应代码在 [RunScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:707)

当前切片策略是：

- 应急先拿 `1 RB`
- 语音再拿 `2 RB`
- 当应急等待超过阈值时，应急可以提到 `3 RB`
- 剩余全部给普通数据

这就是你现在的“刚性预留 + 应急动态提权”机制。

#### 第五步：第二级调度，队内按 IPF 排序

对应代码在 [scheduleClassQueue lambda](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:747)

对每个业务队列：

1. 为每个 UE 算 `metric`
2. 按 `metric` 从高到低排序
3. 估算 `neededRbs`
4. 提议 `schedulerProposedRbs`
5. 最终调用 [ResourceManager::AllocateSpectrum()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/resource-manager.cc:105) 审批
6. 扣 `DL buffer`
7. 更新平均吞吐与上次调度时间
8. 生成 DCI

这里最关键的一点是：

- `GeoBeamScheduler` 负责“排序和提议”
- `ResourceManager` 负责“审批和记账”

#### 第六步：调度后执行重定向

位置：[RunScheduler()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:873)

下行调度结束后，再统一执行本轮已经判定好的 beam 重定向，避免在调度过程中改队列结构。

### 5.5 上行调度主流程

入口：

- 全局触发：[DoSchedUlTriggerReq()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:120)
- 按 beam 执行：[RunUlSchedulerForBeam()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1456)

#### 第一步：开启 UL 调度周期

位置：[BeginUlSchedulingPeriod()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1412)

作用：

- 防止同一个 TTI 内反复触发 UL 调度时重复清零预算。
- 同一个 `RoundId` 只重置一次 beam 的 UL 账本。

这是你修正“一个 TTI 内多次 SR/UL 触发导致重复清账”的关键机制。

#### 第二步：计算真实上行需求

位置：

- [GetEffectiveUlDemandBytes()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:465)
- [RefreshPendingUlGrantEstimate()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:473)

逻辑：

- `有效需求 = ulBufferStatus - pendingUlGrantBytes`

这样做的目的，是避免 GEO 长 RTT 下因为旧 BSR 没更新，调度器重复给同一批未到达的数据连续发授权。

#### 第三步：构建 UL 业务队列

位置：[RunUlSchedulerForBeam()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1477)

对每个 UE：

1. 判断是否需要 `bootstrap grant`
   - 条件是 `SR 已到，但还没 BSR，且也没有 pending grant`
2. 如果有真实 UL 数据，则按 `effectivePendingBytes` 估算需求
3. 先套上功率限制 `GetMaxPowerLimitedUlRbs()`
4. 再交给 `AdmitControl::CanAdmitUe()`
5. 通过后进入 `emergency / voice / normal` 三类队列

#### 第四步：先切 UL 预算

位置：[RunUlSchedulerForBeam()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1553)

策略和 DL 一致：

- 应急保底 `1 RB`
- 语音保底 `2 RB`
- 应急超时则可提到 `3 RB`
- 剩余给普通数据

#### 第五步：队内按 IPF 排序后逐个发 UL grant

位置：[scheduleUlClassQueue lambda](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1578)

对每个 UE：

1. 重新检查 `effectivePendingBytes`
2. 重新套一次功率上限
3. 结合 `queueBudget + beam 剩余 UL RB` 形成最终请求
4. 调用 [ProcessUlGrant()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1336)

#### 第六步：`ProcessUlGrant()` 真正落授权

位置：[ProcessUlGrant()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1336)

这一步是上行方向最关键的闭环：

1. 检查 UE 是否存在。
2. 读取 UE 当前 beam。
3. 再次套功率上限。
4. 调用 `ResourceManager::AllocateSpectrum()` 做最终审批。
5. 生成 UL DCI。
6. 用 `EstimateBytesPerRb()` 把本次授权折算成 `pendingUlGrantBytes`。
7. 更新 `pendingUlGrantRbs / pendingUlGrantBytes / lastUlScheduledTime`

这一步之后，调度器就认为“这部分 UL 资源已经发出，但真实数据还没回来”。

#### 第七步：真实 UL 数据回来后冲减账目

位置：[ReceiveUlMacPdu()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:991)

收到真实 PDU 后：

1. 解析 `GenericUlMacHeader`
2. 找到 `rnti`
3. 按 `payloadBytes` 冲减 `ulBufferStatus`
4. 同时冲减 `pendingUlGrantBytes`
5. 重新估算 `pendingUlGrantRbs`
6. 更新 `ulAverageThroughput`

所以 UL 闭环是：

`BSR/SR -> UL grant -> pending grant记账 -> 真实PDU到达 -> 反向冲减pending`

### 5.6 切换和上下文迁移

你这套资源管理不是简单“改 beamId”，而是显式做上下文打包和重建。

关键函数：

- 导出：[ExportUeContext()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:256)
- 导入：[ImportUeContext()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:295)
- 执行：[ExecuteHandover()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:343)

迁移内容包括：

- DL / UL buffer
- pending UL grant
- CQI / RSRP
- 平均吞吐
- 上次调度时间
- DCI 回调

这意味着你的切换不只是“接入关系切过去”，而是“调度状态机整体搬过去”。

### 5.7 随机接入为什么也属于资源管理逻辑的一部分

严格说，RA 不完全等于调度算法，但你这里的 RA 已经和资源管理强绑定，所以也属于这套模块的一部分。

#### 4 步 RA

关键函数：

- Msg1 -> 窗口聚合：[ReceivePrachPreamble()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1045)
- Msg2 / RAR 分配：[ProcessPrachWindow()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1077)
- Msg3 解包：[ReceiveMsg3Packet()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1217)
- Msg3 冲突判断与 Msg4：[ProcessMsg3Window()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1244)

资源管理点在于：

- Msg3 先受终端功率上限约束
- 再受 beam UL 剩余预算约束
- 最终才生成 `RAR`

#### 2 步 RA

关键函数：

- [ProcessMsgAWindow()](/home/cgf/Downloads/ns-3-dev/contrib/geo-sat/model/geo-beam-scheduler.cc:1845)

逻辑是：

- 单个 UE 成功时直接 `SUCCESS_RAR`
- 多个 UE 抢同一前导码时，退回 `FALLBACK_RAR`，再走 Msg3/Msg4

这里同样调用了 `ResourceManager` 审批 Msg3 的 UL 资源。

## 6. 三个文件合起来的主流程

### 6.1 下行主流程

1. RLC 上报 DL buffer 到 `GeoBeamScheduler`。
2. `GeoBeamScheduler` 每轮先重置 beam DL 账本。
3. 扣除 `PRACH 6 RB`。
4. 对每个 UE 先调用 `AdmitControl::CanAdmitUe()`。
5. 把通过门控的 UE 分成 `应急/语音/普通数据` 三类。
6. 先按 WRR 思想切保底预算。
7. 再按 IPF 指标对每类 UE 排序。
8. 每次真正分配前调用 `ResourceManager::AllocateSpectrum()`。
9. 调度完后，如果本轮产生了重定向动作，再执行 `ExecuteHandover()`。

### 6.2 上行主流程

1. `SR / BSR / PUCCH / CQI / UL PDU` 进入 `GeoBeamScheduler`。
2. 调度器基于 `ulBufferStatus - pendingUlGrantBytes` 计算有效需求。
3. 每轮 UL 调度开始先调用 `BeginUlSchedulingPeriod()` 清当前 beam 的 UL 账本。
4. 构建 `应急/语音/普通数据` 三类 UL 队列。
5. 每个 UE 先受 `AdmitControl` 门控，再受 `ResourceManager` 的功率上限裁剪。
6. 调度器按 WRR 切预算，再用 IPF 排序。
7. 调用 `ProcessUlGrant()` 生成授权，更新 `pendingUlGrantBytes`。
8. 真实 UL MAC PDU 回到 gNB 后，再反向冲减 `pending` 和 `ulBufferStatus`。

## 7. 这三份代码的职责边界总结

### `ResourceManager`

- 管物理预算
- 管 UL 功率上限
- 不管优先级
- 不管 beam 推荐
- 不管切换

### `AdmitControl`

- 管逻辑门控
- 管预留策略
- 管推荐 beam
- 管负载均衡
- 不直接分配 RB

### `GeoBeamScheduler`

- 管执行时序
- 管 UE 上下文
- 管 DL/UL 排序与预算切片
- 管 DCI 生成
- 管切换上下文迁移
- 真正把 `AdmitControl` 和 `ResourceManager` 用起来

## 8. 一句话总结这套实现

你的实现不是“一个简单调度器”，而是一套分层资源管理系统：

- `ResourceManager` 守住物理边界。
- `AdmitControl` 守住策略边界。
- `GeoBeamScheduler` 负责在每个 TTI 内把这两层策略转成实际的 DL/UL 调度结果。

如果后续还要继续扩展，这份结构最自然的扩展方向有两个：

- 在 `AdmitControl` 中继续增强“负载判据”和“方向独立统计”。
- 在 `GeoBeamScheduler` 中继续增强“真实 PHY 闭环下的 UL 授权-到达一致性”和“更多业务类别的队列模型”。
