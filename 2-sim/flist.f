// 配置 ROM 和所选 testbench 由 ModelSim.do 单独添加。
+incdir+../0-rtl/0-common/include
+incdir+../0-rtl/1-config
+incdir+../0-rtl/3-management/snmp
+incdir+../1-tb

// 公共组件：校验、仲裁、消息队列和时基。
../0-rtl/0-common/afdx_crc32.v
../0-rtl/0-common/afdx_msg_queue.v
../0-rtl/0-common/afdx_rr_arbiter.v
../0-rtl/0-common/afdx_timer.v

// VL 运行时配置；静态配置 ROM 由仿真入口选择。
../0-rtl/1-config/afdx_tx_cfg_ram.v

// 端系统核心及发送、接收、MAC 路径。
../0-rtl/2-es_core/afdx_es_core.v
../0-rtl/2-es_core/mac/afdx_mac_rx.v
../0-rtl/2-es_core/mac/afdx_mac_tx.v
../0-rtl/2-es_core/rx/afdx_rx_arb.v
../0-rtl/2-es_core/rx/afdx_rx_deliver.v
../0-rtl/2-es_core/rx/afdx_rx_integrity.v
../0-rtl/2-es_core/rx/afdx_rx_parser.v
../0-rtl/2-es_core/tx/afdx_bag_scheduler.v
../0-rtl/2-es_core/tx/afdx_frame_builder.v
../0-rtl/2-es_core/tx/afdx_redundancy_ctrl.v

// 管理功能及其请求缓冲、服务引擎。
../0-rtl/3-management/afdx_mgmt_events.v
../0-rtl/3-management/afdx_mgmt_plane.v
../0-rtl/3-management/buffer/afdx_mgmt_rxbuf.v
../0-rtl/3-management/buffer/afdx_mgmt_txbuf.v
../0-rtl/3-management/loader/afdx_loader.v
../0-rtl/3-management/loader/afdx_tftp_server.v
../0-rtl/3-management/rtc/afdx_rtc_sync.v
../0-rtl/3-management/rtc/afdx_rtc64.v
../0-rtl/3-management/snmp/afdx_snmp_agent.v
../0-rtl/3-management/snmp/afdx_snmp_ber.v
../0-rtl/3-management/snmp/afdx_snmp_enc.v
../0-rtl/3-management/snmp/afdx_snmp_mib.v
../0-rtl/3-management/snmp/afdx_snmp_mibrom.v
../0-rtl/3-management/snmp/afdx_snmp_values.v

// 应用消息接口。
../0-rtl/4-application/afdx_app_gen.v
../0-rtl/4-application/afdx_app_mux.v
../0-rtl/4-application/afdx_app_sink.v

// 端系统顶层。
../0-rtl/5-top/AFDX_top.v
