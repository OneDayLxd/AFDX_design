> 生成时间：2026-10-24 15:36

# AFDX_design

AFDX 端系统 RTL 与验证代码。当前生产入口为 [AFDX_top.v](0-rtl/5-top/AFDX_top.v)，采用单个 125MHz 时钟、应用载荷接口及独立 A/B GMII 网络扩展。MAC 使用工程内的 `gmii_mac_minimal` 重构内核和 `afdx_gmii_mac` 适配器，固定千兆全双工；运行时不依赖外部厂商 IP、网表、原语库或 MDIO。

本 README 主要说明代码组织和阅读方式，不作为全部竞赛指标已达标的声明。

## 1. 仓库范围与目录

Git 只追踪 `0-rtl/`、`1-tb/`、`.gitignore` 和本 README。

```text
0-rtl/
├─ 0-common/                  消息队列、帧缓存、CRC、仲裁和时基
│  └─ include/                协议常量、数学函数头文件
├─ 1-config/                  VL 参数、静态配置 ROM、运行时配置 RAM
├─ 2-es_core/
│  ├─ afdx_es_core.v          AFDX 发送和接收核心
│  ├─ tx/                     BAG 调度、相位控制、组帧、冗余复制
│  ├─ rx/                     并行解析、完整性检查、去重和消息交付
│  └─ mac/                    队友 GMII MAC 内核、适配器、BAG 跟踪及基线接收模块
├─ 3-management/
│  ├─ afdx_mgmt_plane.v       管理面集成入口
│  ├─ afdx_mgmt_events.v      事件记录与 Trap 控制
│  ├─ buffer/                 管理请求/响应缓冲
│  ├─ snmp/                   SNMP 解析、MIB 查找和响应编码
│  ├─ loader/                 TFTP 服务与加载/卸载对象管理
│  └─ rtc/                    RTC 计时与网络时间同步
├─ 4-application/             应用输入缓存、消息发生器、接收槽及应用/管理仲裁
└─ 5-top/                     系统集成和生产顶层

1-tb/
├─ tb_afdx_*.sv               单元、协议、集成和压力测试
├─ *.svh                     测试公共定义和连接辅助
├─ *_config.v、*.vh           小规模及专用测试配置
├─ scale/                    满规模专用配置
├─ stimuli/                  管理协议测试激励
└─ checkers/                 抓包、吞吐、时延和协议独立检查器
```

这是功能目录，不是严格的实例树；公共模块可以被多个路径复用。

本地还可保留 `2-sim/`、`3-constraints/`、`4-elinx/`、`docs/`、`lint_work/` 和编辑器配置，它们不随仓库保存。因此仅克隆本仓库不会获得完整的本机仿真启动脚本、eLinx 工程和板级约束。

## 2. 从顶层理解模块关系

下面只列主要实例，省略配置 ROM、公共计数器和辅助缓冲：

```text
AFDX_top
├─ u_app_ingress : afdx_app_ingress    valid-only 输入消息缓存
└─ u_mac_system : afdx_mac_system
   ├─ u_system : afdx_system
   │  ├─ u_gen     : afdx_app_gen       内部应用发送源
   │  ├─ u_sink    : afdx_app_sink      内部应用接收槽
   │  ├─ u_mgmt    : afdx_mgmt_plane    管理协议与事件处理
   │  ├─ u_app_mux : afdx_app_mux       应用/管理发送仲裁
   │  └─ u_core    : afdx_es_core       VL 队列、调度、组帧与接收处理
   ├─ g_net[0].u_link : afdx_gmii_mac   A 网适配器（内含 gmii_mac_minimal）
   └─ g_net[1].u_link : afdx_gmii_mac   B 网适配器（内含 gmii_mac_minimal）
```

- [AFDX_top.v](0-rtl/5-top/AFDX_top.v)：正式应用载荷端口及 A/B 网络扩展，固定时钟参数并处理 ready 兼容语义。
- [afdx_app_ingress.v](0-rtl/4-application/afdx_app_ingress.v)：完整消息缓存，将 valid-only 应用输入转换为内部握手流。
- [afdx_mac_system.v](0-rtl/5-top/afdx_mac_system.v)：连接协议系统与两路 MAC；保留 host 应用接口供测试使用。
- [afdx_system.v](0-rtl/5-top/afdx_system.v)：集成应用、管理面和协议核心；正式路径采用 `EXTERNAL_MAC=1`，通过两组独立帧流连接核心外的 MAC 适配器。
- [afdx_es_core.v](0-rtl/2-es_core/afdx_es_core.v)：发送队列/BAG 调度，以及接收解析、完整性检查、冗余管理和消息交付。
- [gmii_mac_minimal.v](0-rtl/2-es_core/mac/gmii_mac_minimal.v)：从队友提供的同名 RTL 复制，保留锁定参考的功能逻辑；仅规范文件头和端口格式，来源哈希见文件头。
- [afdx_gmii_mac.v](0-rtl/2-es_core/mac/afdx_gmii_mac.v)：TX 整帧缓存和 ready 限定 valid，RX 末字节保留/错误裁决与整帧缓存，线路时间戳、首尾反馈及统计。原始 RX 无反压，适配器在消息交给协议核心前丢弃坏帧、长短帧及缓存溢出帧。

适配器在线路侧锁存整个帧期间的 RX_ER，覆盖内核不报告的前导码 PHY 错误。外层仍执行至少七字节前导码、60..1514 字节无 FCS 帧体的策略。默认每网保留外层四槽 TX FIFO，再经过内核双银行 4KiB RAM；因此增加了缓存资源和一段整帧发送等待，不能将此次替换当作时延优化。MAC 的 SOF/EOF 反馈来自实际线路包络；内核反相 TX 时钟输出在当前单时钟仿真入口未使用。

生产顶层采用 `APP_EXTERNAL=1`、关闭内部演示发生器；应用输入经 `afdx_app_ingress` 接入 `host_tx_*`，`host_rx_*` 直接交付应用。`host_*` 保留内部 valid/ready 握手，外部模板端口不使用 ready 握手。旧测试仍可在 MAC 集成层使用内部发生器和接收槽。

## 3. 一条消息如何发送与接收

### 发送方向

```text
rx_data/rx_valid/rx_vl/rx_last → 输入完整消息缓存
  → 应用载荷 / 管理响应
  → afdx_app_mux：仲裁消息来源
  → afdx_msg_queue：按 VL 缓存消息
  → afdx_bag_scheduler：按队列状态、BAG 和管理窗口选择 VL
  → afdx_frame_builder：封装 MAC/IP/UDP 头、载荷和序列号
  → afdx_redundancy_ctrl：按网络使能复制为 A/B 副本
  → afdx_mac_tx × 2：发送元数据与实际线路 SOF/BAG 跟踪
  → afdx_gmii_mac × 2：完整 TX 缓存 → gmii_mac_minimal：前导码/SFD、填充、FCS 和 IFG
  → a_gmii_tx_* / b_gmii_tx_*
```

发送调度默认使用快照轮询；`TX_PHASE_ENABLE=1` 为可选相位模式，需要明确的业务相位和突发约束。不能只打开该参数就认定所有任意负载的抖动都满足要求。

### 接收方向

```text
a_gmii_rx_* / b_gmii_rx_*
  → gmii_mac_minimal × 2：分别接收并给出末尾错误标志
  → afdx_gmii_mac × 2：七字节前导码/帧长策略、错误回滚、完整好帧缓存
  → afdx_rx_parallel
      ├─ afdx_rx_frontend × 2：A/B 独立解析、IP/UDP 校验、载荷暂存
      └─ afdx_rx_integrity：共享完整性检查与冗余去重
  → afdx_msg_queue：保存通过检查的有效消息
  → afdx_rx_deliver：选择并交付消息
  → 应用 tx_data/tx_valid/tx_vl/tx_last / 管理协议处理
```

A/B 是同一逻辑消息的两个网络副本，不是应用需要接收两次。共享去重在消息池成功提交后才确认交付状态，避免提交失败后误丢另一副本。

端系统不是默认透明转发器：普通业务接收后交付应用，不保证自动从 TX 原样发回；部分 TB 的“应用回送”是测试激励行为。

## 4. 外部接口与关键配置

模板接口是应用数据接口，不能再把 `tx_data/rx_data` 当作 GMII：

- `clk`：125MHz，单时钟；`rst`：高有效同步复位。
- 输入 `rx_data/rx_valid/rx_vl/rx_last`：应用提交待发送的 UDP 载荷。`rx_vl` 是发送配置索引（生产配置 7 位），不是 16 位 VLID。
- 输出 `tx_data/tx_valid/tx_vl/tx_last`：向应用交付通过校验、去重后的载荷。`tx_vl` 是接收配置索引（生产配置 12 位）。当前管理面也旁路观察接收流，不对应用输出额外隐藏管理消息。
- `valid=1` 接收一个字节；`last` 仅在 valid 有效时表示末字节。消息内 VL 不变，允许 valid 空洞，不允许消息交织。零字节消息不编码。
- `tx_ready=1` 仅为模板兼容值，不表示容量无限。`rx_ready` 不使用，应用必须接收每一个 `tx_valid` 字节。
- 输入默认 16 个 2048 字节物理槽（32KiB），每消息最多 1471 字节；整条到齐才提交。每 VL 的运行时 Lmax、核心池容量仍由协议核心约束。
- 输入满、超长、非法/中途变化的 VL 均整条丢弃，直到有效 last 才恢复下一条；`app_drop` 脉冲、累计 `app_drop_count` 报告一次拒绝。`app_drop_reason[0/1/2]` 分别为满/超长/VL 错误，可以同时置位。它们不是全系统丢弃计数；核心后续拒绝由管理统计记录。
- 默认槽位最多同时保留 16 条消息（包括读出中消息）。允许的业务还须满足 VL BAG、Lmax、核心池容量与平均服务速率；不能向有限缓存无限提交而期待无损。
- 网络扩展 `a_gmii_tx/rx_data/valid/er`、`b_gmii_tx/rx_data/valid/er`：独立 A/B 链路，与 clk 同步。数据包含前导码、SFD、MAC 帧及 FCS；帧内 valid 连续为高，IFG 时 valid=0。
- 内部 `host_*` 使用 valid/ready，输入缓存负责协议转换；RAM 使用同步读，不复位数据阵列，背压时保持输出。

这是同步千兆字节接口约定，不是任意真实 PHY 的直接上板接口；125MHz × 8bit/拍对应 1Gbps 线路字节节拍，不等于宣称 100Mbps 物理模式已实现。

主要配置入口：

- [afdx_cfg_params.vh](0-rtl/1-config/afdx_cfg_params.vh)：生产 VL 数量/位宽和管理窗口宏；当前为 128 TX / 4096 RX VL。
- [afdx_vl_config.v](0-rtl/1-config/afdx_vl_config.v)：静态 VL 配置 ROM，包含多个配置模块，属于已有的多模块文件。
- [afdx_tx_cfg_ram.v](0-rtl/1-config/afdx_tx_cfg_ram.v)：运行时 TX 配置。
- 顶层默认 `MGMT_EN=1`、`TX_PHASE_ENABLE=0`、`APP_SLOT_LOG=4`；内部演示发生器固定关闭。

部分内部模块仍保留旧模式的 100MHz 默认值，正式顶层通过参数覆盖为 125MHz。历史共享总线路径仍保留，但不是当前 A/B 生产接口。

## 5. 测试代码怎么找

按要理解的功能选择测试入口：

| 关注点 | 主要测试文件 |
| --- | --- |
| 应用缓存、溢出、背压和指针回卷 | [tb_afdx_app_ingress.sv](1-tb/tb_afdx_app_ingress.sv) |
| 两个生产顶层双向通信、吞吐及冗余故障 | [tb_afdx_two_endpoints.sv](1-tb/tb_afdx_two_endpoints.sv)、[独立抓包检查器](1-tb/checkers/check_two_endpoints.py) |
| MAC 集成层双网、ready 兼容与历史回环 | [tb_afdx_mac_top.sv](1-tb/tb_afdx_mac_top.sv) |
| MAC 适配、FCS、帧缓存、错误与反压边界 | [tb_afdx_gmii_mac.sv](1-tb/tb_afdx_gmii_mac.sv) |
| A/B 并行接收与共享去重 | [tb_afdx_rx_parallel.sv](1-tb/tb_afdx_rx_parallel.sv) |
| 快照轮询和管理窗口 | [tb_afdx_tx_snapshot.sv](1-tb/tb_afdx_tx_snapshot.sv)、[tb_afdx_mgmt_window.sv](1-tb/tb_afdx_mgmt_window.sv) |
| 正常应用发送到应用接收 | [tb_afdx_app_loopback.sv](1-tb/tb_afdx_app_loopback.sv) |
| 排队、突发、背压及回送时延 | [tb_afdx_load_latency.sv](1-tb/tb_afdx_load_latency.sv) |
| VL 规模和综合业务诊断 | [tb_afdx_scale.sv](1-tb/tb_afdx_scale.sv)、[tb_afdx_acceptance.sv](1-tb/tb_afdx_acceptance.sv) |
| 管理协议端到端 | [tb_afdx_mac_management.sv](1-tb/tb_afdx_mac_management.sv)、[stimuli/](1-tb/stimuli/) |

测试配置和生产配置存在同名模块/头文件，编译时必须选择一套，不能把所有配置 ROM 同时加入工程。`tb_afdx_latency.sv` 是旧单总线时延基线。旧官方 IP/MDIO 适配器及其专用 TB、PHY 管理模型已移除，当前 MAC 内核只使用工程内的 `gmii_mac_minimal`。

`afdx_mac_tx` 的生产分支负责 VL 的 BAG 计时、发送元数据和实际线路反馈，不能随旧 MAC 删除。`afdx_mac_rx`、串行接收仲裁及 `EXTERNAL_MAC=0` 分支仍供原协议基线测试使用，生产入口固定选择 `EXTERNAL_MAC=1`；旧队列前级和串行解析连线仅在对应分支生成。管理协议、应用输入缓存和 MAC 错误回滚缓存仍是现用功能。

双端测试实例化两个生产 `AFDX_top`（各自含 128 TX / 4096 RX 配置），经四条独立单向网络通道双向通信，不自动回送应用数据。业务使用 VL1..127，VL0 保留管理；基础通道整帧到齐后延迟 A=256ns、B=512ns，另含序列化及排队时间，不模拟电气 PHY。

本地 `2-sim/run_TwoEndpoints.ps1` 跑四种场景：0 为 8 VL 变长帧，1 为 127 VL/1471 字节吞吐，2 为单路丢帧/坏 FCS/到达顺序变化，3 为额外 5us 网络延迟。变长集为 17/465/977/1471 字节 UDP 载荷，对应 64/512/1024/1518 字节 MAC 帧。独立检查应用内容、IP/UDP/FCS、BAG、序列号、去重及时间因果，并包含篡改抓包的负对照。

应用时延从输入首字节到远端输出末字节，抖动取同一 VL 多消息时延极差；这些是应用级工程指标，不替代官方 GMII 处理时延口径。应用 2ms/50us 工程检查和有效载荷 100Mbps 吞吐检查分开报告，吞吐仅在专门负载场景判定，每方向独立且 A/B 只计一次。低负载场景的低吞吐不代表带宽能力不足；工程检查失败不会改成通过。

历史 `tb_afdx_post_synth.sv` 及现存完整综合网表使用旧网络顶层端口，只能作为旧版本证据；不能直接用于当前应用顶层后仿。小规模后仿 fixture 也不证明新应用适配器已经综合验证。下一次综合须使用更新后的生产清单，并重新生成网表/匹配 TB。

## 6. 建议阅读顺序与修改注意

1. 先读 `AFDX_top → afdx_app_ingress / afdx_mac_system → afdx_system`，区分应用载荷、内部握手和网络扩展接口。
2. 再读 `afdx_es_core`，沿发送队列、调度、组帧、A/B 复制理解 TX。
3. 阅读 `afdx_gmii_mac`，区分核心帧流与含前导码/FCS 的线路帧。
4. 沿 `afdx_rx_parallel → afdx_rx_frontend / afdx_rx_integrity → afdx_rx_deliver` 理解 RX。
5. 最后看管理面和上述对应 TB，确认异常处理与检查口径。

新增生产 RTL 后同步维护本地仿真/综合清单，不要漏编译。正式配置来自 `0-rtl/1-config`，不要用 TB 的小规模配置替换生产配置。eLinx 的导入目录只是副本，代码修改仍在 `0-rtl` 进行，综合前核对副本是否最新。

当前已有功能回归与压力验证，但部分时延/抖动工程检查仍失败；完整器件映射、布局布线和 125MHz 时序收敛尚未完成。TFTP/加载管理不等于完整 ARINC615A/665，协议仿真也不等于物理授时精度已验证。
