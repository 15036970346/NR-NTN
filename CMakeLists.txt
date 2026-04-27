# contrib/geo-sat/CMakeLists.txt

# 1. 定义模块名称
set(name geo-sat)

# contrib/geo-sat/CMakeLists.txt

# ... 前面的定义保持不变 ...

# 2. 合并所有源文件 (.cc)
set(source_files
    # --- 谢昀松部分 ---
    model/sat-phy.cc
    model/sat-ut-phy.cc        # [新增] 你的终端物理层
    model/sat-ut-mac.cc        # [新增] 终端MAC层
    helper/user-helper.cc
    helper/terminal-profile.cc
    helper/traffic-models.cc
    
    # --- 秦宇航部分 ---
    model/sat-channel.cc
    model/sat-gw-mac.cc
    model/resource-manager.cc
    model/geo-beam-scheduler.cc
    model/admit-control.cc     # [新增] 准入控制与波束协调
    helper/geo-beam-helper.cc
    helper/gw-helper.cc

    # --- NTN 配置助手 ---
    helper/ntn-config-helper.cc

    # --- ROHC 报头压缩 ---
    model/rohc-compressor.cc
    
    # --- [新增] PDCP 层集成 ROHC ---
    model/sat-pdcp.cc
    
    # --- [新增] 崔博开 RRC 测量模块 ---
    model/sat-ut-rrc.cc
    
    # --- [新增] 系统级仿真统计 ---
    model/sat-stats-collector.cc  # 统计收集器
    
    # --- [新增] 七色频率复用 ---
    model/frequency-reuse.cc        # 频率复用管理器
    
    # --- [新增] HARQ增强管理器 ---
    model/harq-manager.cc          # HARQ IR重传
    
    # --- [新增] 结果输出管理器 ---
    model/result-writer.cc         # 仿真结果输出

    # --- [新增] SDAP 层 ---
    model/sat-sdap.cc

    # --- [新增] 分层统计收集器 ---
    helper/sat-phy-stats-collector.cc
    helper/sat-mac-stats-collector.cc
    helper/sat-rlc-stats-collector.cc
    helper/sat-pdcp-stats-collector.cc
    helper/sat-sdap-stats-collector.cc
    helper/sat-rrc-stats-collector.cc
    helper/sat-e2e-stats-collector.cc
    helper/sat-stats-connector.cc
)

# 3. 合并所有头文件 (.h)
set(header_files
    # --- 谢昀松部分 ---
    model/sat-phy.h
    model/sat-mac-phy-sap.h    # [新增] SAP 接口规范
    model/sat-ut-phy.h         # [新增] 你的终端物理层头文件
    model/sat-ut-mac.h         # [新增] 终端MAC层头文件
    helper/user-helper.h
    helper/terminal-profile.h
    helper/traffic-models.h
    
    # --- 秦宇航部分 ---
    model/sat-channel.h
    model/sat-gw-mac.h
    model/resource-manager.h
    model/geo-beam-scheduler.h
    model/admit-control.h       # [新增] 准入控制与波束协调
    model/sat-mac-common.h
    helper/geo-beam-helper.h
    helper/gw-helper.h

    # --- NTN 配置助手 ---
    helper/ntn-config-helper.h

    # --- ROHC 报头压缩 ---
    model/rohc-compressor.h
    
    # --- [新增] PDCP 层集成 ROHC ---
    model/sat-pdcp.h
    
    # --- [新增] 崔博开 RRC 测量模块 ---
    model/sat-ut-rrc.h
    
    # --- [新增] 系统级仿真统计 ---
    model/sat-stats-collector.h  # 统计收集器
    
    # --- [新增] 七色频率复用 ---
    model/frequency-reuse.h       # 频率复用管理器
    
    # --- [新增] HARQ增强管理器 ---
    model/harq-manager.h          # HARQ IR重传
    
    # --- [新增] 结果输出管理器 ---
    model/result-writer.h         # 仿真结果输出

    # --- [新增] SDAP 层 ---
    model/sat-sdap.h

    # --- [新增] 分层统计收集器 ---
    helper/sat-phy-stats-collector.h
    helper/sat-mac-stats-collector.h
    helper/sat-rlc-stats-collector.h
    helper/sat-pdcp-stats-collector.h
    helper/sat-sdap-stats-collector.h
    helper/sat-rrc-stats-collector.h
    helper/sat-e2e-stats-collector.h
    helper/sat-stats-connector.h
)
# ... 后面的构建命令保持不变 ...

# 4. 合并所有依赖库
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
    ${liblte}           # [关键] 必须添加，因为 RLC/PDCP 优化依赖 LTE 模块
    ${libnr}            # 5G-LENA 模块
    ${libsgp4}          # SGP4 卫星轨道模块
)

# 5. 构建库
build_lib(
    LIBNAME ${name}
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
)

# 6. 构建 Example (确保所有库都链接到位)
build_lib_example(
    NAME ntn-phase1-sim
    SOURCE_FILES examples/ntn-phase1-sim.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

build_lib_example(
    NAME ntn-system-sim
    SOURCE_FILES examples/ntn-system-sim.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

build_lib_example(
    NAME ntn-ra-compare
    SOURCE_FILES examples/ntn-ra-compare.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)

build_lib_example(
    NAME ntn-layer-stats-demo
    SOURCE_FILES examples/ntn-layer-stats-demo.cc
    LIBRARIES_TO_LINK ${libgeo-sat} ${liblte} ${libnr}
)
