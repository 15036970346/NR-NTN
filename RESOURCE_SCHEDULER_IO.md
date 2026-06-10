# Geo-Sat 资源管理与调度模块输入输出说明

本文档整理当前 `geo-sat` 项目中资源管理与调度主链路的输入、输出、关键用户参数，以及模块之间的数据流向。

适用模块：

- `GeoBeamScheduler`
- `AdmitControl`
- `ResourceManager`
- `SatUtMac`
- `SatPhy / SatGeoUserPhy / SatGeoFeederPhy`

## 1. 总体链路

当前主链路可概括为：

1. UE 上报业务需求、CQI、SR、BSR、PRACH。
2. `GeoBeamScheduler` 收集 UE 上下文并执行准入控制。
3. 调度器按 `emergency / voice / data` 构建逻辑队列。
4. 调度器执行两级调度：
   - 一级：业务级 WRR 切片
   - 二级：类内 IPF/优先级排序
5. 调度器向 `ResourceManager` 申请实际 RB。
6. `ResourceManager` 返回批准 RB，并在 UL 时给出发射功率。
7. 调度器生成 `DciInfo` 下发到 UE。
8. `SatUtMac` 按调度结果发起 UL 发送或执行 DL 接收。
9. `SatPhy` 将 UL 包送到 gNB 侧，调度器更新 UE 状态。

## 2. GeoBeamScheduler

### 2.1 模块职责

`GeoBeamScheduler` 是调度总控模块，负责：

- 维护 UE 上下文
- 按 beam 组织 DL/UL 调度
- 调用 `AdmitControl`
- 调用 `ResourceManager`
- 生成 `DciInfo`
- 处理 BSR、PUCCH、PRACH、Msg3、普通 UL MAC PDU

### 2.2 主要输入

#### 2.2.1 来自 UE 上下文的输入

这些参数主要存放在调度器内部的 `m_ueContextMap[rnti]` 中：

- `rnti`
  - UE 标识
- `currentBeamId`
  - 当前所属波束
- `priority`
  - 业务优先级，决定进入哪类逻辑队列
- `trafficType`
  - 业务类型
- `utType`
  - 终端类型
- `dlBufferStatus`
  - 当前 gNB 侧待下发给该 UE 的数据量
  - 只被 DL 调度使用
- `ulBufferStatus`
  - 当前 gNB 侧看到的 UE 待上行发送数据量
  - 由 `BSR` 和普通 UL PDU 接收闭环维护
- `pendingUlGrantRbs`
  - 当前 UE 已经拿到、但尚未在 gNB 侧确认消耗的 UL 授权 RB
  - 用于避免重复授权
- `srPending`
  - 当前 UE 是否存在待处理的 SR 请求
- `latestCqi`
  - 最近的链路质量指标
- `latestRsrp`
  - 最近一次测量上报中的 RSRP
  - 只作为测量状态存储，不直接当 CQI 使用
- `position`
  - UE 位置，用于位置因子计算
- `lastDlScheduledTime`
  - 最近一次被下行调度时间
- `lastUlScheduledTime`
  - 最近一次被上行调度或成功接收上行数据时间
- `dlAverageThroughput`
  - 下行吞吐统计量，用于 DL 比例公平
- `ulAverageThroughput`
  - 上行吞吐统计量，用于 UL 比例公平
- `wrrWeight`
  - 权重参数

#### 2.2.2 来自 UE 上报消息的输入

- `PucchInfo`
  - `SR`
  - `CQI`
  - `HARQ ACK/NACK`
- `BsR_MAC_CE`
  - UE 上行缓冲状态
- `MeasReport`
  - 测量上报
- `PrachPreamble`
  - 随机接入前导
- 普通 UL `Packet`
  - 通过 `GenericUlMacHeader` 携带 `rnti` 和 `payloadBytes`
- `Msg3 Packet`
  - 通过 `Msg3MacHeader` 携带 RA 相关信息

#### 2.2.3 来自其他模块的输入

- `AdmitControl`
  - 准入结果
- `ResourceManager`
  - beam 剩余 RB
  - 实际批准 RB
  - UL 发射功率估计

### 2.3 主要输出

- `DciInfo`
  - 给 UE 的调度结果
  - 主要字段：
    - `isUplinkGrant`
    - `rbAllocation`
    - `mcs`
    - `txPowerDbm`
    - `delayToStart`
    - `duration`
- `RAR`
  - 给 RA UE 的随机接入响应
- `Msg4`
  - RA 竞争解决结果
- UE 状态更新
  - `dlBufferStatus`
  - `ulBufferStatus`
  - `pendingUlGrantRbs`
  - `srPending`
  - `dlAverageThroughput`
  - `ulAverageThroughput`
  - `lastScheduledTime`
- 调度动作
  - `QUEUE`
  - `REDIRECT`
  - `REJECT`

### 2.4 调度时真正直接依赖的用户参数

#### 2.4.1 影响“是否进入调度”

- `dlBufferStatus`（DL）
- `ulBufferStatus`（UL）
- `pendingUlGrantRbs`
- `srPending`
- `priority`
- `currentBeamId`
- `trafficType`
- `utType`

#### 2.4.2 影响“同类用户排序”

- `latestCqi`
- `position`
- `lastDlScheduledTime`
- `dlAverageThroughput`
- `wrrWeight`

#### 2.4.3 影响“最终分配多少 RB”

- `dlBufferStatus` 或 `ulBufferStatus`
- `latestCqi`
- 所属 `beamId`
- 当前队列预算
- 当前 beam 剩余 RB

### 2.5 Buffer、吞吐与统计口径

当前调度器已经将上下行缓存状态拆开，语义如下。

#### 2.5.1 `dlBufferStatus`

- 表示 gNB 侧仍待给该 UE 下发的数据量
- 主要由 `DoSchedDlRlcBufferReq()` 或测试入口 `UpdateUeDlBufferStatus()` 写入
- 在 `RunScheduler()` 中按实际分配的 `DL RB * bytesPerRb` 扣减
- 只参与 DL 准入、DL 队列构建、DL 排序与 DL 资源分配

#### 2.5.2 `ulBufferStatus`

- 表示 gNB 侧保存的该 UE 上行待发数据量视图
- 由 `ReceiveBsr()` 更新为 UE 最近一次上报的缓存值
- 在 gNB 收到普通 UL MAC PDU 后，由 `ReceiveUlMacPdu()` 按真实收到的 `payloadBytes` 扣减
- 只参与 UL 准入、UL 队列构建、UL 排序与 UL 资源分配

#### 2.5.3 `pendingUlGrantRbs`

- 表示当前 UE 已经拿到但尚未完成闭环消耗的 UL grant
- `ProcessUlGrant()` 授权后增加
- `ReceiveUlMacPdu()` 收到 UL MAC PDU 后清零
- 主要目的不是表示缓存量，而是防止同一 UE 在同一轮 UL 传输尚未完成前重复拿 grant

#### 2.5.4 `srPending`

- 表示 UE 已经发送 SR，等待调度器分配上行机会
- `ReceivePucchInfo()` 收到 `FORMAT_0` 且 `srPending=true` 时置位
- `ProcessUlGrant()` 发出授权后清零

#### 2.5.5 `dlAverageThroughput / ulAverageThroughput`

- `dlAverageThroughput`
  - 在 `RunScheduler()` 中按 `allocatedRb * bytesPerRb` 更新
  - 只参与 DL 类内排序的比例公平分母
- `ulAverageThroughput`
  - 在 `ReceiveUlMacPdu()` 中按 `deliveredBytes` 更新
  - 只参与 UL 类内排序的比例公平分母
- 这两个统计量已经拆开，避免 UL 活跃度直接污染 DL 公平性，反之亦然

#### 2.5.6 `lastDlScheduledTime / lastUlScheduledTime`

- `lastDlScheduledTime`
  - 当 UE 拿到 DL DCI 时更新
  - 只参与 DL 等待时延敏感度计算
- `lastUlScheduledTime`
  - 当 UE 拿到 UL grant，或 gNB 收到其普通 UL MAC PDU 时更新
  - 只参与 UL 等待时延敏感度计算
- 作用：
  - 避免 UL 活跃度直接抬高 DL 排序优先级
  - 避免 DL 频繁服务直接污染 UL 等待时间统计

#### 2.5.7 `latestRsrp`

- 由 `ReceiveMeasReport()` 更新
- 当前只用于测量状态保存和波束/位置相关流程
- 不再直接写入 `latestCqi`

## 3. AdmitControl

### 3.1 模块职责

`AdmitControl` 负责：

- 准入判断
- 排队判断
- 重定向候选 beam 推荐
- 读取物理剩余资源
- 基于保留资源策略判断当前 UE 是否可进入本轮调度
- 区分当前判断针对的是 `DL` 还是 `UL`

### 3.2 输入

- `beamId`
- `priority`
- `trafficType`
- `utType`
- `requiredRbs`
- `isUplink`
- 当前 beam 资源状态
- `ResourceManager` 返回的剩余 RB

### 3.3 输出

- `AdmitDecision`
  - `ADMIT_OK`
  - `ADMIT_QUEUE`
  - `ADMIT_REDIRECT`
  - `ADMIT_REJECTED`
- 候选目标 beam 列表

### 3.4 当前资源策略

- 应急业务
  - 默认保留 `1 PRB`
  - 突发最多 `3 PRB`
- 语音业务
  - 默认保留 `2 PRB`
- 数据业务
  - 使用剩余资源
- DL 侧还可额外扣除 `m_dlReservedRbs`
  - 当前主要用于 PRACH/控制保留

### 3.5 当前准入与重定向判定逻辑

`AdmitControl` 当前不会直接拿裸 `requiredRbs` 去判断，而是会先结合：

- `priority`
- `utType`
- `trafficType`
- `isUplink`

计算一个 `effectiveRequiredRbs`，再执行准入或重定向判断。

当前规则可以概括为：

- `voice`
  - 基本按原始需求处理
- `data`
  - 默认按原始需求处理
- `high-capacity`
  - 会被放大为更高的有效需求，更容易触发 `REDIRECT/REJECT`
- `UL + portable`
  - 对非应急、非语音业务，会额外提高有效需求，体现便携终端上行更紧

同时，准入阈值也已方向化和业务化：

- `emergency`
  - 最宽，允许接近满载
- `voice`
  - 比普通数据更保守
- `best-effort / high-capacity`
  - 更容易排队或重定向

`GetRecommendedBeams()` 当前评分主要考虑：

- 目标 beam 的剩余资源头间距
- 当前利用率
- 当前活跃 UE 数
- 同优先级 UE 数
- 业务类型惩罚/奖励

因此当前推荐已经是“带业务语义的 beam 推荐”，不再只是“谁剩余 RB 多就选谁”。

## 4. ResourceManager

### 4.1 模块职责

`ResourceManager` 当前负责两类功能：

- 每个 beam 的 RB 记账与剩余资源查询
- 上行发射功率计算

### 4.2 输入

#### 4.2.1 RB 分配相关输入

- `beamId`
- `requestedRbs`
- `isUplink`

#### 4.2.2 功率计算相关输入

- `utType`
- `allocatedRbs`
- `pathLossDb`

### 4.3 输出

- `AllocateSpectrum()`
  - 输出：`approvedRbs`
- `GetRemainingRbs()`
  - 输出：当前 beam 剩余 RB
- `AdjustUtTxPower()`
  - 输出：`txPowerDbm`

### 4.4 内部状态

内部维护按 beam 的使用状态：

- `dlUsedRbs`
- `ulUsedRbs`

即：

- 每个 beam 已使用多少下行 RB
- 每个 beam 已使用多少上行 RB

### 4.5 当前固定资源口径

当前项目口径固定为：

- 总带宽：`35 MHz`
- 总 RB：`175`
- 7 色复用
- 每波束：
  - `DL = 25 RB`
  - `UL = 25 RB`

### 4.6 参考路径损耗口径

当前调度与功控链路里使用的参考路径损耗为：

- `ReferencePathLossDb = 190 dB`

它的用途是：

- 普通 `UL grant` 的功控计算
- `Msg3` 的功控计算

当前它不是通过传播模型实时计算出来的，而是作为一个**工程参考值**使用。

这个值的来源是当前项目采用的 GEO 场景口径：

- 轨道类型：`GEO`
- 高度：约 `35786 km`
- 载频：约 `2 GHz`
- 星地链路：典型路径损耗按 `190 dB` 建模

因此，当前 `190 dB` 的意义应理解为：

- `GEO + 2 GHz` 场景下的**典型参考路损**
- 用于调度器侧和 `ResourceManager` 功控公式的统一输入
- 不是每个 UE、每个 beam、每个时刻都重新计算的瞬时路径损耗

这意味着：

- 现在的功控是“基于参考链路预算的工程默认值”
- 不是“基于实时几何位置和传播环境的动态功控”

如果后续要做更高真实性的链路仿真，建议把该参数从固定值升级为：

- 按 UE 位置、波束几何关系、传播条件动态计算的 `pathLossDb`

### 4.7 与 Scheduler 的边界

`ResourceManager` 不负责决定“哪个 UE 更优先”，它只负责：

- 当前 beam 还有多少 `DL/UL RB`
- 这次请求最多还能批多少
- UL 若获批，应使用多大发射功率

换句话说：

- `GeoBeamScheduler` 决定“想给谁分、想分多少”
- `ResourceManager` 决定“当前 beam 还能不能批、最终批多少”

## 5. Scheduler 指标口径

当前 `GeoBeamScheduler` 已将 `DL/UL` 里复用的辅助估算统一收口成内部函数：

- `EstimateBytesPerRb(cqi)`
- `CalculateLocationFactor(position)`
- `CalculateSchedulerMetric(ctx, queueBudget, isUplink, urgencyBoost, demandBytes)`

### 5.1 `EstimateBytesPerRb`

它不是标准频谱效率公式，而是当前项目内部统一的调度辅助估算：

- 同时参考 `CQI` 和 `CQI -> MCS`
- 用于把缓存字节量换算成 RB 需求

这样做的目标是保证 `DL/UL` 两侧用一致的需求估算，而不是输出可直接当标准值的吞吐。

### 5.2 `CalculateLocationFactor`

位置因子当前由 `m_ipfLocationWeight` 控制强度：

- 越靠近波束中心，因子越接近 `1.0`
- 越远离中心，因子越低

它目前仍是启发式位置加权，不是严格波束增益模型。

### 5.3 `CalculateSchedulerMetric`

当前类内排序指标 `P` 综合考虑：

- 瞬时可得速率
- 位置因子
- 等待时延敏感度
- 业务紧急度
- WRR 权重
- 需求量温和加权
- CQI 正向增益
- 历史平均吞吐公平性分母

因此当前 `DL/UL` 调度的排序趋势是统一的：

- 高优先级、久未服务、CQI 更好、平均吞吐更低的 UE 更容易被排前
- 需求量只做温和偏置，不会因为缓存大就反向奖励差链路 UE

## 6. SatUtMac

### 5.1 模块职责

`SatUtMac` 负责：

- 接收调度器下发的 `DciInfo`
- 发起普通 UL 发送
- 发送 `SR / CQI / HARQ / BSR`
- 发起 PRACH
- 发送 Msg3

### 5.2 输入

- `DciInfo`
- `RAR`
- `Msg4`
- 本地缓冲数据量 `m_currentBufferBytes`
- 当前 UE 有效 `RNTI`

### 5.3 输出

- `PucchInfo`
- `BsR_MAC_CE`
- `PrachPreamble`
- 普通 UL MAC PDU
- `Msg3 Packet`

## 7. SatPhy / SatGeoUserPhy / SatGeoFeederPhy

### 6.1 模块职责

这些模块负责物理层包传输：

- UE 侧 PHY 发送 UL 包
- gNB 侧 PHY 接收 UL 包
- 将接收到的包通过回调上送调度器

### 6.2 输入

- `Packet`
- 物理参数 `SatSignalParameters`

### 6.3 输出

- gNB 侧接收回调
  - 普通 UL PDU -> `ReceiveUlMacPdu()`
  - Msg3 Packet -> `ReceiveMsg3Packet()`

## 8. 关键输入输出表

| 模块 | 主要输入 | 直接依赖的用户参数 | 主要输出 | 输出流向 |
|---|---|---|---|---|
| `GeoBeamScheduler::RunScheduler()` | UE上下文、`AdmitControl` 决策、`ResourceManager` 剩余RB | `rnti`、`currentBeamId`、`priority`、`trafficType`、`utType`、`dlBufferStatus`、`latestCqi`、`position`、`lastDlScheduledTime`、`dlAverageThroughput`、`wrrWeight` | 每个UE本轮是否参与调度、排序结果、建议分配多少RB | `ResourceManager`、`SendDciToUe()` |
| `GeoBeamScheduler::RunUlSchedulerForBeam()` | UE上下文、`AdmitControl` 决策、`ResourceManager` 剩余RB | `rnti`、`currentBeamId`、`priority`、`trafficType`、`utType`、`ulBufferStatus`、`pendingUlGrantRbs`、`srPending`、`latestCqi`、`position`、`lastUlScheduledTime`、`ulAverageThroughput`、`wrrWeight` | 每个UE本轮是否参与 UL 调度、排序结果、建议 UL grant | `ResourceManager`、`ProcessUlGrant()` |
| `GeoBeamScheduler::ReceivePucchInfo()` | `PucchInfo` | `rnti`、`cqi`、`srPending`、`harqAck` | 更新 `latestCqi`，或触发 UL 调度入口 | UE上下文、`RunUlSchedulerForBeam()` |
| `GeoBeamScheduler::ReceiveBsr()` | `BsR_MAC_CE` | `rnti`、`bufferSize` | 更新该UE的 `ulBufferStatus` | UE上下文，后续被 UL 调度使用 |
| `GeoBeamScheduler::ReceiveMeasReport()` | `MeasReport` | `ueId`、`bestBeamId`、`rsrp`、`position` | 更新 `latestRsrp`、位置和可能的切换 | UE上下文、切换逻辑 |
| `GeoBeamScheduler::ProcessUlGrant()` | `rnti`、请求RB、MCS | `rnti` 对应的 `beamId`、`utType` | UL `DciInfo`，包含 `rbAllocation`、`mcs`、`txPowerDbm` | `SendDciToUe()` |
| `GeoBeamScheduler::ReceiveUlMacPdu()` | 普通 UL `Packet` | `rnti`、`payloadBytes` | 扣减 `ulBufferStatus`，更新吞吐和最近服务时间 | UE上下文 |
| `AdmitControl::CanAdmitUe()` | `beamId`、优先级、请求RB、方向、当前资源状态 | `priority`、`utType`、`trafficType`、`requiredRbs`、`isUplink` | `ADMIT_OK / QUEUE / REDIRECT / REJECTED` | `GeoBeamScheduler` |
| `ResourceManager::AllocateSpectrum()` | `beamId`、`requestedRbs`、`isUplink` | 间接依赖用户所属beam | 实际批准的 `approvedRbs` | `GeoBeamScheduler` |
| `ResourceManager::GetRemainingRbs()` | `beamId`、方向 | 用户所属beam | 当前剩余RB | `Scheduler`、`AdmitControl` |
| `ResourceManager::AdjustUtTxPower()` | `utType`、`allocatedRbs`、`pathLossDb` | `utType` | `txPowerDbm` | `ProcessUlGrant()`、`Msg3 grant` |
| `SatUtMac::ProcessDciAndSchedule()` | `DciInfo` | UE当前 `rnti`、本地缓存 `m_currentBufferBytes` | 触发UL发送或DL接收 | `DoTransmit()` / `ReceiveData()` |
| `SatUtMac::SendSchedulingRequest()` | 本地有待发数据 | `activeRnti`、`m_currentBufferBytes` | `PUCCH SR` | `GeoBeamScheduler::ReceivePucchInfo()` |
| `SatUtMac::SendCqiReport()` | CQI测量值 | `activeRnti`、`cqi` | `PUCCH CQI` | `GeoBeamScheduler::ReceivePucchInfo()` |
| `SatUtMac::SendBsr()` | 本地缓存大小 | `activeRnti`、`bufferSize` | `BSR` | `GeoBeamScheduler::ReceiveBsr()` |
| `SatUtMac::DoTransmit()` | UL授权参数、MAC缓存 | `activeRnti`、`m_currentBufferBytes` | 普通 UL PDU 或 `Msg3 Packet` | `SatPhy::SendPdu()` |

## 8. 当前设计上最重要的理解

### 8.1 Scheduler 的本质

`Scheduler` 的本质是：

- 输入：用户状态、用户需求、链路质量、准入结果、剩余资源
- 输出：本轮给哪个用户分多少 RB、用什么 MCS、何时发、UL 时用多大功率

### 8.2 ResourceManager 的本质

`ResourceManager` 的本质是：

- 输入：某个 beam 的资源申请、方向、功控参数
- 输出：批多少 RB、还剩多少 RB、UL 发射功率是多少

### 8.3 两者关系

- `GeoBeamScheduler` 决定“想给谁分、想分多少”
- `ResourceManager` 决定“当前 beam 还能不能批、最终批多少”

换句话说：

- `Scheduler` 偏“决策”
- `ResourceManager` 偏“执行约束与记账”

## 9. 示例运行模式

为避免 `ntn-phase1-sim` 中 `DL / UL / RA / 杂项模块测试` 互相干扰，现在示例支持按模式单独运行：

- `--mode=all`
  - 运行综合联调场景
- `--mode=dl`
  - 只看 `AdmitControl + DL WRR/IPF + DL DCI`
- `--mode=ul`
  - 只看 `SR / BSR / UL 多用户调度 / UL PUSCH 入账`
- `--mode=ra`
  - 只看 `Msg1 -> Msg2 -> Msg3 -> Msg4`
- `--mode=misc`
  - 只看 `PDCP / PHY / RRC / ESSA` 等独立模块测试

便捷脚本：

- `contrib/geo-sat/examples/run-ntn-phase1-dl.sh`
- `contrib/geo-sat/examples/run-ntn-phase1-ul.sh`
- `contrib/geo-sat/examples/run-ntn-phase1-ra.sh`
