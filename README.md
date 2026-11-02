> 生成时间：2026-11-02 16:40

# AFDX_design

AFDX 端系统 RTL 与验证代码。当前生产入口为 [AFDX_es.v](0-rtl/5-AFDX_es/AFDX_es.v)，采用单个 125MHz 时钟、应用载荷接口及独立 A/B GMII 网络扩展。MAC 使用工程内的 `gmii_mac_minimal` 重构内核和 `afdx_gmii_mac` 适配器，固定千兆全双工；运行时不依赖外部厂商 IP、网表、原语库或 MDIO。

仿真应用封装为 [AFDX_top.sv](0-rtl/6-top/AFDX_top.sv)，内部仅例化两个相同的 `AFDX_es`，A/B 的 TX/RX 数据、有效和错误信号分别交叉直连，不增加传输周期或交换延迟。TB 驱动应用消息，故障和延迟对照由 TB 单独注入。综合时选择 `AFDX_es`，不编译双端系统验收封装。

本 README 主要说明代码组织和阅读方式，不作为全部竞赛指标已达标的声明。

## 1. 仓库范围与目录

Git 只追踪 `0-rtl/`、`1-tb/`、`.gitignore` 和本 README。

```text
0-rtl/
├─ 0-common/                  消息队列、帧缓存、CRC、仲裁和时基
│  └─ include/                协议常量、数学函数头文件
├─ 1-config/                  VL 参数、静态配置 ROM、接收 VLID 逆映射、运行时配置 RAM
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
├─ 4-application/             应用封装、共享字节缓存、IP 分片/重组及管理仲裁
├─ 5-AFDX_es/                 单个可综合端系统：AFDX_es、afdx_mac_system、afdx_system
└─ 6-top/                     AFDX_top.sv：两个端系统的 A/B GMII 直连验收封装

1-tb/
├─ tb_afdx_*.sv               单元、协议、集成和压力测试
├─ *.svh                     测试公共定义和连接辅助
├─ *_config.v、*.vh           小规模及专用测试配置
├─ scale/                    满规模专用配置
├─ stimuli/                  管理协议测试激励
└─ checkers/                 抓包、吞吐、时延和协议独立检查器
```

这是功能目录，不是严格的实例树；公共模块可以被多个路径复用。

本地还可保留 `2-sim/`、`3-constraints/`、`4-AFDX_es_synth/`、`5-data/`、`docs/`、`lint_work/` 和编辑器配置，它们不随仓库保存。`2-sim/` 保留仿真启动脚本、源码清单及配置；`4-AFDX_es_synth/` 保留当前单端系统综合工程及源码刷新、核对脚本；`5-data/results/` 保存已归档的日志、抓包和统计结果，`5-data/history/` 保存历史快照和开发辅助脚本。因此仅克隆本仓库不会获得完整的本机仿真启动脚本、eLinx 工程、约束和结果数据。

## 2. 从顶层理解模块关系

端到端验证层次：

```text
tb_afdx_two_endpoints
└─ dut : AFDX_top
   ├─ g_endpoint[0].u_es : AFDX_es
   └─ g_endpoint[1].u_es : AFDX_es
      两端的 A 网互连、B 网互连；两个方向均可同时通信
```

下面列出单个端系统的主要实例，省略配置 ROM、公共计数器和辅助缓冲：

```text
AFDX_es
├─ u_app_tx : afdx_app_tx_adapter      VLID 首部解析、配置索引查找及完整消息缓存
│  ├─ u_buffer : afdx_app_ingress      完整消息缓存、长度与全消息校验和
│  └─ g_fragment.u_fragment            按 VL Lmax 切分 IPv4 数据报
├─ u_app_rx : afdx_app_rx_adapter      接收索引还原 VLID、应用消息封装
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

- [AFDX_es.v](0-rtl/5-AFDX_es/AFDX_es.v)：正式应用载荷端口及 A/B 网络扩展，固定时钟参数并处理 ready 兼容语义。
- [afdx_app_ingress.v](0-rtl/4-application/afdx_app_ingress.v)：完整消息缓存，将 valid-only 应用输入转换为内部握手流。
- [afdx_app_tx_adapter.v](0-rtl/4-application/afdx_app_tx_adapter.v)：解析两字节大端 VLID，比较配置 ROM 后寄存，经平衡选择树得到内部发送索引；拒绝未知 VLID、截断首部和空载荷消息。
- [afdx_app_rx_adapter.v](0-rtl/4-application/afdx_app_rx_adapter.v)：将内部接收索引转换为实际 VLID，先输出 VLID，再输出载荷。
- [afdx_mac_system.v](0-rtl/5-AFDX_es/afdx_mac_system.v)：连接协议系统与两路 MAC；保留 host 应用接口供测试使用。
- [afdx_system.v](0-rtl/5-AFDX_es/afdx_system.v)：集成应用、管理面和协议核心；正式路径采用 `EXTERNAL_MAC=1`，通过两组独立帧流连接核心外的 MAC 适配器。
- [afdx_es_core.v](0-rtl/2-es_core/afdx_es_core.v)：发送队列/BAG 调度，以及接收解析、完整性检查、冗余管理和消息交付。
- [gmii_mac_minimal.v](0-rtl/2-es_core/mac/gmii_mac_minimal.v)：从队友提供的同名 RTL 复制，保留锁定参考的功能逻辑；仅规范文件头和端口格式，来源哈希见文件头。
- [afdx_gmii_mac.v](0-rtl/2-es_core/mac/afdx_gmii_mac.v)：TX 整帧缓存和 ready 限定 valid，RX 末字节保留/错误裁决与整帧缓存，线路时间戳、首尾反馈及统计。原始 RX 无反压，适配器在消息交给协议核心前丢弃坏帧、长短帧及缓存溢出帧。

适配器在线路侧锁存整个帧期间的 RX_ER，覆盖内核不报告的前导码 PHY 错误。外层仍执行至少七字节前导码、60..1514 字节无 FCS 帧体的策略。默认每网保留外层四槽 TX FIFO，再经过内核双银行 4KiB RAM；因此增加了缓存资源和一段整帧发送等待，不能将此次替换当作时延优化。MAC 的 SOF/EOF 反馈来自实际线路包络；内核反相 TX 时钟输出在当前单时钟仿真入口未使用。

生产顶层采用 `APP_EXTERNAL=1`、关闭内部演示发生器；应用输入经发送适配器和消息缓存接入 `host_tx_*`，`host_rx_*` 经接收适配器交付应用。`host_*` 保留内部 valid/ready 握手，外部应用端口不使用 ready 握手。旧测试仍可在 MAC 集成层使用内部发生器和接收槽。

## 3. 一条消息如何发送与接收

### 发送方向

```text
tx_data/tx_valid/tx_last（VLID + 载荷）→ VLID 查表 → 输入完整消息缓存
  → afdx_ip_fragment_tx：长消息切片（短消息保持单帧）
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

生产模式默认启用 `TX_EARLY_PREPARE=1`：在 BAG 到期前预备组帧，按当前帧长度计算两级完整帧存储的安全提前量，避免每个 BAG 后再累积一段组帧等待。实际 GMII 的同 VL 帧间隔仍逐帧检查，不通过缩短 BAG 降低时延。该优化不改动队友的 MAC 内核。

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
  → afdx_ip_reassembly：分片重组与整包 UDP 校验（短消息透传）
  → 接收索引逆映射 → 应用 rx_data/rx_valid/rx_last（VLID + 载荷）/ 管理协议处理
```

A/B 是同一逻辑消息的两个网络副本，不是应用需要接收两次。共享去重在消息池成功提交后才确认交付状态，避免提交失败后误丢另一副本。

端系统不是默认透明转发器：普通业务接收后交付应用，不保证自动从 TX 原样发回；部分 TB 的“应用回送”是测试激励行为。

## 4. 外部接口与关键配置

模板接口是应用数据接口，不能再把 `tx_data/rx_data` 当作 GMII：

- `clk`：125MHz，单时钟；`rst`：高有效同步复位。
- 输入 `tx_data/tx_valid/tx_last`：应用提交待发送消息。第一个有效字节为 VLID[15:8]，第二个为 VLID[7:0]，其后为 1..8192 字节 UDP 载荷（默认 `APP_MAX_LEN=8192`）；`tx_last` 标记载荷末字节。
- 输出 `rx_data/rx_valid/rx_last`：以相同的“VLID 高字节、VLID 低字节、载荷”格式交付通过检查、去重的消息。当前管理面也旁路观察接收流，不对应用输出额外隐藏管理消息。
- `valid=1` 接收一个字节；`last` 仅在 valid 有效时结束消息。首部和载荷均允许 valid 空洞，不允许消息交织。VLID 属于应用封装元数据，不进入 UDP 载荷，也不计入有效载荷吞吐。
- 两字节大端 VLID 和 last 是本工程明确采用的封装约定；答疑未规定唯一字节编码。配置索引不再暴露为应用端口。当前接收订阅为 0x1000..0x1fff，修改订阅表须同步维护 `afdx_rx_vl_id_rom` 逆映射。
- `tx_ready=1` 仅为兼容值，不表示容量无限。`rx_ready` 不使用，应用必须接收每一个 `rx_valid` 字节。
- 输入默认 16 个消息描述符、32KiB 环形共享字节池；按实际长度占用空间，每消息最多 8192 字节，整条到齐才发布。超静态 Lmax 的消息使用 IPv4 分片，每片仍受 VL BAG 与核心容量约束。
- 输入满、超长、未知 VLID、截断首部或空载荷均整条拒绝；有效 last 恢复下一条。`app_drop` 脉冲、累计 `app_drop_count` 报告一次拒绝。`app_drop_reason[0/1/2]` 分别为满/超长/VLID 或封装错误，可以同时置位。它们不是全系统丢弃计数；核心后续拒绝由管理统计记录。
- 默认槽位最多同时保留 16 条消息（包括读出中消息）。允许的业务还须满足 VL BAG、Lmax、核心池容量与平均服务速率；不能向有限缓存无限提交而期待无损。
- 网络扩展 `a_gmii_tx/rx_data/valid/er`、`b_gmii_tx/rx_data/valid/er`：独立 A/B 链路，与 clk 同步。数据包含前导码、SFD、MAC 帧及 FCS；帧内 valid 连续为高，IFG 时 valid=0。
- 内部 `host_*` 使用 valid/ready，输入缓存负责协议转换；RAM 使用同步读，不复位数据阵列，背压时保持输出。

这是同步千兆字节接口约定，不是任意真实 PHY 的直接上板接口；125MHz × 8bit/拍对应 1Gbps 线路字节节拍，不等于宣称 100Mbps 物理模式已实现。

`AFDX_top` 的应用端口是两路打包总线：data[7:0]对应端点0，data[15:8]对应端点1，valid/last/ready/drop各位按端点索引对应；两个方向均可独立注入和接收。业务格式与单端系统完全相同。

基础连接为零附加延迟的 GMII 直连，仍经过两个端系统的完整 MAC、校验、缓存和调度通路。顶层无网络故障/延迟控制口；TB 的故障场景覆盖接收连线以丢弃单网整帧或翻转字节制造坏 FCS，延迟对照仅在 TB 使用固定 5us 延迟。TB 记录故障计划，检查器与实际线路字节、SFD/EOF 时间戳交叉核对。普通场景要求对端 RX 与源 TX 在同一采样沿数据完全一致。


主要配置入口：

- [afdx_cfg_params.vh](0-rtl/1-config/afdx_cfg_params.vh)：生产 VL 数量/位宽和管理窗口宏；当前为 128 TX / 4096 RX VL。
- [afdx_vl_config.v](0-rtl/1-config/afdx_vl_config.v)：静态 VL 配置 ROM，包含多个配置模块，属于已有的多模块文件。
- [afdx_tx_cfg_ram.v](0-rtl/1-config/afdx_tx_cfg_ram.v)：运行时 TX 配置。
- 顶层默认 `MGMT_EN=1`、`TX_PHASE_ENABLE=0`、`TX_EARLY_PREPARE=1`、`APP_SLOT_LOG=4`、`APP_MAX_LEN=8192`；内部演示发生器固定关闭。


### 分片与重组的边界

`afdx_ip_fragment_tx` 产生真正的 IPv4 MF、八字节偏移与统一 IP ID；仅首片携带 UDP 首部。整包 UDP 校验和在完整消息缓存时计算，每个分片独立经过调度、序列号、A/B 复制与 MAC 校验。

`afdx_ip_reassembly` 位于共享完整性/去重与应用/管理分流之间。默认四个上下文，以接收 VL、源 IP 和 IP ID 区分数据报，允许乱序，拒绝重叠、越界、长度不符和整包 UDP 校验错误；完整通过后才交付应用。缺片不交付，无重组定时超时；四个上下文耗尽时轮询替换未完成消息，内部计数可观察。长消息并发和输入突发须纳入业务规划。

静态配置与运行时 Lmax 应保持一致；修改 Lmax 前应停止并清空相关 VL 的业务，再同步配置。单帧时延/抖动与跨多个 BAG 的长消息完成时间分别记录，不宣称任意突发消息均在 2ms 内完成。答疑第 96 行明确跨多个 BAG 的分片消息完成时间不受该单帧 2ms 限制。


## 5. 验证入口

[测试平台说明](1-tb/README.md)列出当前42个TB的分组。标准回归运行39项单元和集成测试；管理专项及综合后测试分别运行。容量测试以125 MHz逐项检查128条发送VL和4096条接收订阅。

在交付包的2-sim目录运行run_RtlRegression.ps1、run_TwoEndpoints.ps1和run_ManagementRegression.ps1。仿真清单为flist.f，单端综合清单为rtl_production.f。生产和测试配置ROM定义同名模块，编译时选择对应的一套配置。

双端测试采用两个生产AFDX_es的A/B GMII直连；VL0保留管理，业务VL1..127。七个场景覆盖变长、127 VL并发、单网丢帧及坏FCS、TB固定5us延迟、同长度重复、100 Mbps输入和全部业务VL每BAG最大帧。帧长包含64/512/1024/1518 B。

时延从源应用首个VLID字节输入至对端对应消息首字节输出；抖动按同VL同帧长分组计算时延极差，包含冷启动。基础业务BAG为2ms、Lmax为1471 B；长消息跨BAG的完成时间单列。输入有效载荷100 Mbps的稳定性、首字节时延小于2ms及抖动小于100us通过相应场景检查。已有原始抓包、逐帧统计及额外诊断值归档在5-data/results；重新运行仿真时，新结果仍生成在2-sim。

## 6. 阅读顺序

1. AFDX_top：理解两个端系统的应用接口与双网连接。
2. AFDX_es：沿应用适配、共享字节缓存和分片进入afdx_mac_system。
3. afdx_system及afdx_es_core：理解VL队列、BAG调度、组帧及复制。
4. afdx_gmii_mac：理解帧流与包含前导码/FCS的GMII线路数据。
5. afdx_rx_parallel、afdx_rx_integrity、afdx_rx_deliver及afdx_ip_reassembly：理解校验、去重、缓存及完整消息交付。
6. 管理面和对应TB：理解SNMP、TFTP、RTC与Trap。
