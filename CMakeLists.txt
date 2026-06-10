# contrib/geo-sat/CMakeLists.txt
# GEO Satellite NTN 仿真平台
# 合并版本：geosat-qyh + geosat-xys

# 1. 定义模块名称
# (检查修复: 原交付为 "0603geo-sat" — 数字开头的模块名会让 ns-3 的 lib<name> 变量为空,
#  导致 add_library ALIAS 报错、configure 失败。模块名不能以数字开头, 改回 geo-sat。)
set(name geo-sat)

# 2. 源文件列表 (.cc)
set(source_files
    # ==================== 核心物理层 ====================
    model/sat-phy.cc                      # 卫星 PHY (基类 + UserPhy/FeederPhy 透明弯管)
    model/sat-ut-phy.cc                   # 用户终端物理层 (含 RSRP/RSRQ/SINR TracedCallback)
    model/ntn-gw-phy.cc                   # 信关站 PHY (馈电链路侧, SatPhy 薄壳子类)

    # ==================== MAC 层 ====================
    model/sat-ut-mac.cc                   # 终端MAC (支持4步/2步RA, MAC状态机)
    model/ntn-gw-mac.cc                   # 信关站MAC (SSB广播)

    # ==================== 调度与资源管理 ====================
    model/geo-beam-scheduler.cc           # 两级调度器 (WRR+IPF, 位置感知)
    model/resource-manager.cc             # 资源管理器 (每波束独立RB预算)
    model/admit-control.cc                # 准入控制与切换判决
    model/frequency-reuse.cc             # 4色/7色频率复用

    # ==================== HARQ 增强 ====================
    model/harq-manager.cc                # 8进程 HARQ-IR 重传

    # ==================== qyh 增量: 链路自适应 / RA 解调 ====================
    model/cqi-predictor.cc               # CQI 预测器抽象接口 (直通默认)
    model/cqi-amc-controller.cc          # OLLA 外环 + Kalman CQI 闭环
    model/cqi-amc-predictor.cc           # 把 CqiAmcController 适配为 CqiPredictor
    model/msg4-demod-model.cc            # Msg4 PDCCH/PDSCH BLER-SNR 解调
    model/prach-detection-model.cc       # PRACH Pd/Pfa 系统级检测

    # ==================== 信道模型 ====================
    model/sat-channel.cc                 # 3GPP TR 38.811 NTN 信道

    # ==================== 协议栈 ====================
    model/ntn-pdcp.cc                    # PDCP 层 (集成 ROHC)
    model/ntn-sdap.cc                    # SDAP 层 (QoS Flow 映射)
    model/ntn-rlc.cc                     # RLC 层 (TM/UM/AM, AM 模拟丢包+重传)
    model/ntn-phy-error-model.cc         # SINR -> BLER 简化模型 (MAC DoPhyRx 入口丢包)
    model/ntn-mac-tags.cc                # NTN MAC 数据面 Packet Tag (NtnDciTag)

    # ==================== RRC ====================
    model/sat-ut-rrc.cc                  # 用户终端 RRC (L3滤波, A2/A4事件)
    model/ntn-gw-rrc.cc                  # 信关站 RRC (切换决策/执行/统计)

    # ==================== 报头压缩 ====================
    model/rohc-compressor.cc             # ROHC 压缩 (RFC 3095)

    # ==================== 统计与输出 ====================
    model/sat-stats-collector.cc         # 系统级统计收集
    model/result-writer.cc              # 仿真结果输出

    # ==================== Helper 辅助类 ====================
    helper/geo-beam-helper.cc           # 波束配置助手
    helper/gw-helper.cc                 # 信关站助手
    helper/user-helper.cc               # 用户终端助手
    helper/ntn-config-helper.cc         # NTN 配置助手 (GEO RTT 定时器)
    helper/terminal-profile.cc          # 终端配置
    helper/traffic-models.cc            # 业务模型 (VoIP/FTP/HTTP/FullBuffer)

    # ==================== 分层统计收集器 ====================
    helper/sat-phy-stats-collector.cc
    helper/sat-mac-stats-collector.cc
    helper/sat-rlc-stats-collector.cc
    helper/sat-pdcp-stats-collector.cc
    helper/sat-sdap-stats-collector.cc
    helper/sat-rrc-stats-collector.cc
    helper/sat-e2e-stats-collector.cc
    helper/sat-stats-connector.cc
)

# 3. 头文件列表 (.h)
set(header_files
    # ==================== 核心物理层 ====================
    model/sat-phy.h
    model/sat-ut-phy.h
    model/ntn-gw-phy.h
    model/sat-mac-phy-sap.h             # MAC-PHY SAP 接口

    # ==================== MAC 层 ====================
    model/sat-ut-mac.h
    model/ntn-gw-mac.h
    model/sat-mac-common.h              # MAC 公共结构体

    # ==================== 调度与资源管理 ====================
    model/geo-beam-scheduler.h
    model/resource-manager.h
    model/admit-control.h
    model/frequency-reuse.h

    # ==================== HARQ ====================
    model/harq-manager.h

    # ==================== qyh 增量: 链路自适应 / RA 解调 ====================
    model/cqi-predictor.h
    model/cqi-amc-controller.h
    model/cqi-amc-predictor.h
    model/msg4-demod-model.h
    model/prach-detection-model.h

    # ==================== 信道模型 ====================
    model/sat-channel.h

    # ==================== 协议栈 ====================
    model/ntn-pdcp.h
    model/ntn-sdap.h
    model/ntn-rlc.h
    model/ntn-phy-error-model.h
    model/ntn-mac-tags.h

    # ==================== RRC ====================
    model/sat-ut-rrc.h
    model/ntn-gw-rrc.h

    # ==================== 报头压缩 ====================
    model/rohc-compressor.h

    # ==================== 统计与输出 ====================
    model/sat-stats-collector.h
    model/result-writer.h

    # ==================== Helper ====================
    helper/geo-beam-helper.h
    helper/gw-helper.h
    helper/user-helper.h
    helper/ntn-config-helper.h
    helper/terminal-profile.h
    helper/traffic-models.h

    # ==================== 分层统计收集器 ====================
    helper/sat-phy-stats-collector.h
    helper/sat-mac-stats-collector.h
    helper/sat-rlc-stats-collector.h
    helper/sat-pdcp-stats-collector.h
    helper/sat-sdap-stats-collector.h
    helper/sat-rrc-stats-collector.h
    helper/sat-e2e-stats-collector.h
    helper/sat-stats-connector.h
)

# 4. 依赖库
set(libraries_to_link
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libantenna}
    ${libspectrum}
    ${libinternet}
    ${libapplications}
    ${libbridge}
    ${libpoint-to-point}
    ${liblte}            # LTE RLC/PDCP
    ${libnr}             # 5G NR 模块 (已集成全套卫星类: geo-coordinate / satellite-sgp4-mobility-model 等)
    # 注: 不再依赖独立 ${libsgp4} 模块——nr 已内置同名卫星类, 同时链接会导致
    #     geo-coordinate / SatConstantPropagationDelayModel 等日志组件重复注册而启动崩溃。
    #     与原版 geo-sat 运行时行为一致(仅从 nr 解析卫星功能)。
)

# 5. 构建库
build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)

# 6. 构建示例程序

# --- 生产仿真 ---
build_lib_example(
    NAME ntn-system-sim
    SOURCE_FILES examples/ntn-system-sim.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- 接入对比测试 ---
build_lib_example(
    NAME ntn-ra-compare
    SOURCE_FILES examples/ntn-ra-compare.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- 分层统计演示 ---
build_lib_example(
    NAME ntn-layer-stats-demo
    SOURCE_FILES examples/ntn-layer-stats-demo.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- 单主服务切换演示 ---
# 注意: 暂时禁用。ntn-handover-demo.cc 按旧 API 在 GeoBeamScheduler 上调用
#   ReceiveMeasReport/GetServingObjectForUe/GetHandoverStats/ExportHandoverStats,
#   这些方法在当前架构属于 NtnGwRrc(非调度器), 该示例合并前即不兼容。属示例层问题。
# build_lib_example(
#     NAME ntn-handover-demo
#     SOURCE_FILES examples/ntn-handover-demo.cc
#     LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
# )

# --- 真实 NR 地面小区 + 测量驱动星地切换演示 (阶段C) ---
# 注意: 暂时禁用, 原因同 ntn-handover-demo(按旧 API 调用已迁移到 NtnGwRrc 的方法)。
# build_lib_example(
#     NAME ntn-nr-ground-handover-demo
#     SOURCE_FILES examples/ntn-nr-ground-handover-demo.cc
#     LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
# )

# --- 协议栈联通性自动化测试 (T1-T6: PHY/MAC/RLC/PDCP 全栈) ---
# 合并替代:
#   ntn-sat-relay-demo / ntn-relay-helper-demo (T1 PHY relay)
#   ntn-mac-relay-demo (T2 MAC bidir)
#   ntn-multi-ue-demo (T3 多 UE + HARQ ACK)
#   ntn-multi-beam-demo (T4 多波束隔离)
#   ntn-rlc-demo (T5 RLC TM/UM/AM)
#   ntn-fullstack-demo (T6 全栈 + SINR→BLER 真重传)
build_lib_example(
    NAME ntn-protocol-stack-test
    SOURCE_FILES examples/ntn-protocol-stack-test.cc
    LIBRARIES_TO_LINK ${libgeo-sat}
)

# --- 多波束 4色 vs 7色 SINR / 容量 解析比对 (独立功能, 非协议栈) ---
build_lib_example(
    NAME ntn-reuse-sinr-demo
    SOURCE_FILES examples/ntn-reuse-sinr-demo.cc
    LIBRARIES_TO_LINK ${libgeo-sat}
)

# --- qyh 增量: OLLA 外环 + Kalman CQI 闭环 链路自适应仿真 ---
build_lib_example(
    NAME ntn-amc-closed-loop
    SOURCE_FILES examples/ntn-amc-closed-loop.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- qyh 增量: HTTP 分布采样 (参数化层, 直接采 ThreeGppHttpVariables) ---
build_lib_example(
    NAME http-dist-sample
    SOURCE_FILES examples/http-dist-sample.cc
    LIBRARIES_TO_LINK ${libcore} ${libapplications}
)

# --- qyh 增量: HTTP 端到端 trace 验证 (真实 client/server) ---
build_lib_example(
    NAME http-live-validate
    SOURCE_FILES examples/http-live-validate.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- qyh 增量: PRACH 检测 + Msg4 解调模型上场验证 ---
build_lib_example(
    NAME ntn-prach-msg4-demo
    SOURCE_FILES examples/ntn-prach-msg4-demo.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- 业务模型验证 (qyh 流量恢复后重新启用) ---
build_lib_example(
    NAME traffic-model-validation
    SOURCE_FILES examples/traffic-model-validation.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- 资源管理模块综合验证 20项 (ReceiveMeasReport 迁 NtnGwRrc + 恢复 RegisterUeDciCallback) ---
build_lib_example(
    NAME resource-manager-validation
    SOURCE_FILES examples/resource-manager-validation.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- 功能验证演示 (FTP/VoIP 业务模型 + 动态RB/NOMA 调度 + CQI 接入, 产出可观测数据) ---
build_lib_example(
    NAME functional-demo
    SOURCE_FILES examples/functional-demo.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

# --- RM 生产级端到端演示 (动态RB + 功率域NOMA + 闭环功控CLPC + CQI滤波/预测 一起生效出结果) ---
build_lib_example(
    NAME ntn-rrm-e2e-demo
    SOURCE_FILES examples/ntn-rrm-e2e-demo.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)
