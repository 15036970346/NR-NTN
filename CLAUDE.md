# CLAUDE.md — geosat-merged

GEO Satellite NTN 仿真平台 **合并版本**，整合了 geosat-qyh 和 geosat-xys 两个开发分支的最佳实现。

## 合并来源

| 分支 | 主要贡献 |
|------|----------|
| **geosat-qyh** | 精细化资源管理（每波束独立RB预算）、完整的4步RA、性能验证工具 |
| **geosat-xys** | 分层统计收集器体系、RSRP/RSRQ测量、2步RA、SDAP层 |

## 合并策略

| 模块 | 采用版本 | 原因 |
|------|----------|------|
| `model/sat-phy.h/.cc` | xys | 基础PHY，两版本一致 |
| `model/sat-ut-phy.h/.cc` | xys | RSRP/RSRQ TracedCallback 更完整 |
| `model/sat-ut-mac.h/.cc` | xys | 支持2步RA、MAC_WAITING_MSGB状态 |
| `model/sat-gw-mac.h/.cc` | xys | 一致 |
| `model/geo-beam-scheduler.h/.cc` | xys | UL Grant处理更规范 |
| `model/resource-manager.h/.cc` | **qyh** | 每波束独立RB预算更精细 |
| `model/admit-control.h/.cc` | xys | 简化接口，移除isUplink参数 |
| `model/frequency-reuse.h/.cc` | xys | 一致 |
| `model/harq-manager.h/.cc` | xys | 一致 |
| `model/sat-channel.h/.cc` | xys | 一致 |
| `model/sat-pdcp.h/.cc` | xys | 一致 |
| `model/sat-sdap.h/.cc` | xys | xys独有 |
| `model/sat-ut-rrc.h/.cc` | xys | 一致 |
| `model/rohc-compressor.h/.cc` | xys | 一致 |
| `model/sat-stats-collector.h/.cc` | xys | 增强：UeChannelContext |
| `model/result-writer.h/.cc` | xys | 一致 |
| helper/ | xys | 含完整分层统计收集器 |
| examples/ | 合并 | 所有示例程序均保留 |

---

## Build & Run

```bash
# 从 ns-3-dev 根目录
./ns3 configure --enable-modules=geo-sat
./ns3 build geo-sat

# 集成测试
./ns3 run ntn-phase1-sim

# 生产仿真
./ns3 run "ntn-system-sim --numUes=70 --reuseMode=7 --simTime=10"

# 4色复用高峰值速率
./ns3 run "ntn-system-sim --numUes=14 --reuseMode=4 --simTime=30"

# 接入对比测试
./ns3 run ntn-ra-compare

# 分层统计演示
./ns3 run ntn-layer-stats-demo

# 资源管理器验证
./ns3 run resource-manager-validation

# 业务模型验证
./ns3 run traffic-model-validation

# 拓扑可视化 (需在 examples/ 目录执行)
python3 generate_topology.py   # → topology.xml (NetAnim)
python3 visualize_topology.py # → PNG 图
```

---

## 系统架构

### 协议栈

```
Application (OnOff/Voice)
    ↓
RLC  ←→  PDCP (RohcCompressor)
    ↓
SatUtMac  ←  GeoBeamScheduler
    |              ↑ MeasReport SAP
SatUtPhy      ResourceManager + AdmitControl
    ↓
SatChannel (3GPP TR 38.811 NTN tables)
    ↓
SatGeoFeederPhy (gateway) / SatGeoUserPhy
```

### 两级调度器 (`model/geo-beam-scheduler.h`)

`GeoBeamScheduler` 继承 `NrMacScheduler`，结合：
- **WRR (加权轮询)** — 应急/语音优先级队列
- **IPF (改进型比例公平)** — 位置感知的数据业务调度

持有 `ResourceManager`（每波束RB分配）和 `AdmitControl`（切换准入、应急预留）。

### GEO 时延适配

所有 RLC/PDCP 定时器通过 `NtnConfigHelper` 静态方法适配 ~600ms RTT。关键配置：扩展 T-Reordering、T-StatusProhibit、HARQ RTT。

### NTN 信道模型 (`model/sat-channel.h`)

继承 `ThreeGppChannelModel`，重写 `GetThreeGppTable()` 嵌入 3GPP TR 38.811 仰角依赖LUT（S/Ka频段，10°–90°，城市LOS）。

### 切换流程

1. `SatUtPhy` 测量 SINR → `SatUtRrc` L3滤波 + Event A4
2. `MeasReport` 通过 `SatMacPhySapUser` SAP 传递
3. `GeoBeamScheduler::ReceiveMeasReport()` → `AdmitControl` 评估目标波束
4. `ExecuteHandover()`: 上下文导出/导入，RNTI更新

### 频率复用 (`model/frequency-reuse.h`)

- **7色**: 无同频干扰，完全隔离
- **4色**: +6dB 复用增益，含波束间干扰；可通过 `--reuseMode` 选择

---

## 关键枚举 (均在 `model/sat-mac-common.h`)

| 枚举 | 值 |
|------|-----|
| `MultipleAccessMode` | SLOTTED_ALOHA, ESSA, FDMA, TDMA |
| `UtType` | UT_PORTABLE, UT_CONSUMER |
| `TrafficType` | TRAFFIC_VOICE, TRAFFIC_DATA, TRAFFIC_HIGH_CAPACITY |
| `ServicePriority` | EMERGENCY, VOICE, DATA, BEST_EFFORT |
| `AdmitDecision` | OK, REJECTED, REDIRECT, QUEUE |
| `HarqState` | IDLE, WAITING_ACK, RETRANSMITTING, SUCCESS, FAILED |

---

## 关键文件

| 文件 | 用途 |
|------|------|
| `model/geo-beam-scheduler.{h,cc}` | WRR+IPF两级调度 + 切换 |
| `model/resource-manager.{h,cc}` | **每波束独立RB预算** (qyh特性) |
| `model/sat-ut-mac.{h,cc}` | 终端MAC状态机 (IDLE→CONNECTED, 支持2步RA) |
| `model/sat-ut-phy.{h,cc}` | **RSRP/RSRQ TracedCallback** (xys特性) |
| `model/sat-channel.{h,cc}` | 3GPP TR 38.811 NTN 信道 |
| `model/harq-manager.{h,cc}` | 8进程 HARQ-IR，RV循环 |
| `model/rohc-compressor.{h,cc}` | RFC 3095 报头压缩 |
| `model/sat-ut-rrc.{h,cc}` | RRC L3滤波 + A2/A4事件 |
| `model/sat-sdap.{h,cc}` | SDAP层 QoS Flow映射 (xys特性) |
| `helper/ntn-config-helper.{h,cc}` | GEO RLC/PDCP定时器配置 |
| `helper/sat-*-stats-collector.{h,cc}` | **分层统计收集器** (xys特性) |
| `examples/ntn-system-sim.cc` | 生产仿真：--numUes, --reuseMode, --simTime |
| `examples/ntn-phase1-sim.cc` | 集成组件测试 |
| `examples/ntn-ra-compare.cc` | 2步/4步RA对比 |
| `examples/ntn-layer-stats-demo.cc` | 分层统计演示 |
| `examples/resource-manager-validation.cc` | 资源管理器验证 |
| `examples/traffic-model-validation.cc` | 业务模型验证 |

---

## 分层统计收集器体系 (xys 特性)

```
helper/sat-stats-connector.cc      # 统一连接器
    ├── sat-phy-stats-collector    # PHY层: SINR/RSRP/RSRQ
    ├── sat-mac-stats-collector    # MAC层: BSR/PUCCH/DCI
    ├── sat-rlc-stats-collector    # RLC层: PDU/缓存
    ├── sat-pdcp-stats-collector   # PDCP层: 压缩比/吞吐
    ├── sat-sdap-stats-collector   # SDAP层: QoS Flow
    ├── sat-rrc-stats-collector    # RRC层: 测量/切换
    └── sat-e2e-stats-collector    # E2E: 端到端性能
```

---

## 实现约束

- `GeoBeamScheduler` 继承 `NrMacScheduler`，但大多数虚拟调度器SAP方法为桩实现
- `RohcCompressor` 为简化参考实现（非比特级精确）
- `SatChannel` 仅使用城市LOS；未参数化NLOS和郊区NTN场景
- 固定7波束六边形星座；无动态波束赋形

---

## 主要参数

| 参数 | 值 |
|------|-----|
| 卫星类型 | GEO |
| 轨道高度 | 35786 km |
| 波束数量 | 7 |
| 总带宽 | 35 MHz (175 PRB) |
| 每波束带宽 | 25 PRB (5 MHz @ 15kHz SCS) |
| 子载波间隔 | 15 kHz |
| GEO RTT | ~600 ms |
| HARQ进程 | 8 |
| 最大HARQ重传 | 4 |
| 频率复用 | 4色 / 7色 |
| ROHC压缩 | 启用 |

---

## 实现注意事项

1. **resource-manager**: 采用 qyh 每波束独立RB预算，需在 `GeoBeamScheduler` 中正确维护每波束的 `m_beamUsageMap`
2. **sat-ut-phy**: xys版本的 RSRP/RSRQ TracedCallback 需要上册连接统计收集器
3. **2步RA**: `MAC_WAITING_MSGB` 状态需在状态机中正确处理
4. **统计收集**: 分层统计收集器需通过 `SatStatsConnector` 统一管理
