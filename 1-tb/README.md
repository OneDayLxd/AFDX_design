> 生成时间：2026-11-02 16:40

# 测试平台使用说明

本目录保留当前端系统的单元、集成、管理和综合后功能测试，共42个TB。标准RTL回归运行39个；管理专项和综合后测试分别运行。

| 测试组 | 入口 | 检查内容 |
| --- | --- | --- |
| 应用端到端 | `tb_afdx_two_endpoints.sv` | 两个AFDX_es双向收发、变长、多VL、100 Mbps、BAG、时延和抖动 |
| VL容量 | `tb_afdx_scale.sv`、`scale/` | 125 MHz核心，128发送VL和4096接收订阅逐项检查 |
| 应用适配 | `tb_afdx_app_*.sv` | VLID映射、完整消息缓存、8192 B、满池、超长和复位 |
| 分片重组 | `tb_afdx_ip_*.sv`、`tb_afdx_fragment_rx.sv`、`tb_afdx_fragment_e2e.sv` | IPv4分片字段、乱序重组、整包校验、异常片和双端长消息 |
| MAC与调度 | `tb_afdx_gmii_mac.sv`、`tb_afdx_mac_*.sv`、`tb_afdx_tx_*.sv` | FCS/IFG、缓存、实际线路BAG和提前预备 |
| 接收与消息池 | `tb_afdx_rx_*.sv`、`tb_afdx_queue_*.sv`、`tb_afdx_integrity_age.sv` | 并行接收、IC/RM、原子提交、队列回收和状态老化 |
| 管理服务 | `tb_afdx_mac_management.sv`、`stimuli/` | SNMP、TFTP、RTC、Trap十二类场景 |
| 综合后功能 | `tb_afdx_fragment_post_synth.sv`、`post_synth/` | 同一夹具对照RTL和综合功能网表 |
| 完整端系统综合后功能 | `tb_afdx_es_post_synth.sv` | 生产默认参数、应用与GMII外部接口收发、CRC错误隔离、去重与接收分片重组 |

完整端系统后仿采用同一TB分别驱动生产RTL和导出的综合功能网表，使用`checkers/check_es_post_synth.py`独立检查抓包并比较外部事件及周期。运行入口为交付包中的`2-sim/run_PostSynthProduction.ps1`。本次提交时RTL基准已通过，完整生产网表测试仍在运行，尚未形成最终通过结论；此专项不替代容量、吞吐率或管理协议回归。

在工程`2-sim`目录运行：

```powershell
.\run_RtlRegression.ps1
.\run_TwoEndpoints.ps1
.\run_ManagementRegression.ps1
```

单独运行容量测试：`run_RtlRegression.ps1 -Testbench tb_afdx_scale`。

`checkers/check_two_endpoints.py`与`check_two_endpoints_timing.py`独立核对应用/GMII内容、校验和、BAG、消息交付及时间分解；`check_mgmt_capture.py`与`mib_model.py`核对管理协议。脚本和仿真器配置随交付包中的`2-sim`提供。

正式性能判定采用首字节时延小于2 ms、同VL同帧长抖动小于100 µs。原始抓包和附加诊断值均保存在结果文件；`run_TwoEndpoints.ps1 -StrictEngineeringGuards`可额外检查保留的内部诊断门限。
