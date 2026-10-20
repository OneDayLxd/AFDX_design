> 生成时间：2026-10-17 15:03

# AFDX_design

本项目用于 AFDX RTL 开发与功能仿真。日常仿真不需要 Python，不涉及综合、布局布线或网表仿真。

## 目录与编辑器配置

- `0-rtl/`：设计源代码。
- `1-tb/`：testbench、测试配置和激励。
- `2-sim/`：ModelSim 仿真脚本、仿真编译库和输出文件。
- `lint_work/`：VS Code 保存文件时进行语法检查的独立 ModelSim 编译库。
- `.vscode/settings.json`：项目级编辑器配置。

使用 VS Code 打开 `AFDX_design` 文件夹，项目配置才会生效。
Verilog 扩展使用 ModelSim 进行检查：打开文件时不检查，保存时检查；头文件搜索路径覆盖 `0-rtl/0-common/include`、`0-rtl/1-config`、`0-rtl/3-management/snmp` 和 `1-tb`。
`lint_work` 与 `2-sim/work` 相互独立，均为工具生成的编译库，不应手工修改其中的文件。
保存时的语法检查不会运行 testbench，也不能代替功能仿真。

当前配置已指定本机的绝对编译库路径。以后再次移动或重命名项目目录时，需要更新 `.vscode/settings.json` 中的 `verilog.linting.modelsim.work`。

## RTL 目录与模块关系

```text
0-rtl/
├─ 0-common/                  公共 CRC、轮询仲裁、消息队列、微秒时基
│  └─ include/                公共协议常量和数学函数头文件
├─ 1-config/                  静态 VL 配置、参数宏、运行时双银行配置 RAM
├─ 2-es_core/
│  ├─ afdx_es_core.v          端系统收发核心
│  ├─ tx/                    BAG 调度、帧组装、发送冗余复制
│  ├─ rx/                    接收仲裁、帧解析、完整性检查与冗余管理、消息交付
│  └─ mac/                   MAC 收发、FCS 与帧缓冲
├─ 3-management/
│  ├─ afdx_mgmt_plane.v       管理功能集成与作业调度
│  ├─ afdx_mgmt_events.v      事件记录与 Trap 控制
│  ├─ buffer/                管理请求和响应缓冲
│  ├─ snmp/                  SNMP 解析、MIB 查找、取值与响应编码
│  ├─ loader/                TFTP 服务与数据加载/卸载对象管理
│  └─ rtc/                   64 位 RTC 与网络时间同步
├─ 4-application/             应用消息发生器、接收槽、应用/管理消息仲裁
└─ 5-top/
   └─ AFDX_top.v             连接应用接口、管理功能和收发核心
```

目录按功能分组，不是严格的模块实例树。公共组件由多个模块复用，文件只保留一份。
建议先阅读 `5-top/AFDX_top.v`，再看 `2-es_core/afdx_es_core.v` 的收发路径；管理功能从 `3-management/afdx_mgmt_plane.v` 进入。

每个 `.v/.vh` 文件头包含 `File Name`、`Module Name`、`Function` 和 `Sub Module`：

- `Function` 保留原有功能与接口说明。
- `Module Name` 列出本文件定义的模块；纯头文件填写 `none`。
- `Sub Module` 列出本文件内直接实例化的模块类型，不展开间接依赖；没有子模块时填写 `none`。

新增 RTL 文件后，把新路径加入 `2-sim/flist.f`；新增头文件目录时，同步更新文件清单和 `.vscode/settings.json` 中的搜索路径。

## 日常使用

在 PowerShell 中进入 `2-sim`，然后选择一个 testbench：

```powershell
cd D:\Desktop\CCWork\FPGA\AFDX_design\2-sim
.\run_ModelSim tb_afdx_top.sv
```

ModelSim 会打开界面，编译、运行到 testbench 结束，并保留波形。
Transcript 应出现 `tb_afdx_top: ... checks, 0 errors -> PASS`。
命令行立即返回不代表仿真已结束；不要连续启动多个实例争用同一 `work` 库。

更换测试，只需更换文件名，例如：

```powershell
.\run_ModelSim tb_afdx_protocol.sv
.\run_ModelSim tb_afdx_mac_tx_bag.sv
```

已有 ModelSim 窗口且工作目录为 `2-sim` 时，也可在 Transcript 中运行：

```tcl
set TB_TOP tb_afdx_top
set SIM_ARGS {}
do ModelSim.do
```

改 RTL 后重新执行 `do ModelSim.do` 即可重新编译、仿真。
更换测试时建议清空 `SIM_ARGS`，避免沿用上次手工设置的参数。

## 文件作用

以下文件均位于 `2-sim`：

| 文件 | 作用 |
| --- | --- |
| `run_ModelSim.bat` | 指定 testbench，启动 ModelSim 图形界面 |
| `ModelSim.do` | 选择配置，编译 RTL/TB，启动并运行仿真 |
| `flist.f` | 公共 RTL 文件清单，新增 RTL 模块时在这里添加 |
| `wave.do` | 默认波形配置，可追加需要观察的内部信号 |
| `clean.bat` | 关闭 ModelSim 后清理本目录日志、波形和输出数据；不删除源码或 `work` |
| `work/` | ModelSim 编译库，自动生成，不修改里面的文件 |
| `sim.log` | 日常启动的编译和仿真日志 |
| `modelsim_console.log` | 启动控制台输出，打不开软件时查看 |
| `vsim.wlf` | 波形记录，自动生成 |

`flist.f` 不列配置 ROM 和 testbench，这两项由 `ModelSim.do` 自动添加，避免同名模块重复编译。
默认波形显示 testbench 信号；实例名为 `dut` 时还显示其端口。可在 ModelSim 中手工展开内部层级或编辑 `wave.do`。

## AFDX 配置选择

- `tb_afdx_top`、`tb_afdx_protocol`：使用 `1-tb` 中的小规模测试配置。
- `tb_afdx_mgmt_e2e`：使用 `1-tb` 中的管理测试配置，默认读取 `1-tb/stimuli/smoke.txt`，输出 `capture.txt`。
- `tb_afdx_scale`：使用 `1-tb/scale` 中的满规模专用配置。
- 其他 testbench：使用 `0-rtl/1-config` 中的正式配置。

配置选择不修改 `0-rtl` 中的文件。不需要手动复制或覆盖配置 ROM。
管理端到端 testbench 在本流程中只运行 RTL 和自身检查；不调用旧 Python 协议检查器，因此其 PASS 不能替代完整管理协议校验。
`tb_afdx_latency` 默认仿真 20 ms，输出 `latency.csv`，本流程不分析延迟或抖动指标。

## 命令行运行（可选）

不看波形时可在 `2-sim` 目录前台运行，等待仿真完成：

```powershell
$env:MGLS_LICENSE_FILE = 'D:\Softwares\ModelSim_SE_2020.4\LICENSE.TXT'
vsim -c -l batch.log -do "set TB_TOP tb_afdx_top; set BATCH_MODE 1; do ModelSim.do"
```

检查日志中的明确 PASS，而不是仅凭退出码判断。

## 旧流程归档

原来的自动回归、配置生成、Python 分析及历史结果完整保存在：

`D:\Desktop\CCWork\FPGA\_archive\AFDX_2-sim_20261017_143302`

不需要打开归档就能使用当前 RTL 功能仿真流程。
