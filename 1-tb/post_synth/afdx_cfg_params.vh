// 小规模综合后数据路径测试；不替代生产 128 TX / 4096 RX VL 容量验证。
`ifndef AFDX_CFG_PARAMS_VH
`define AFDX_CFG_PARAMS_VH
`define AFDX_N_TX_VL 4
`define AFDX_TX_VLW 2
`define AFDX_N_RX_VL 4
`define AFDX_RX_VLW 2
`define AFDX_MGMT_LMAX 516
`define AFDX_MGMT_WIN_VL 0
`define AFDX_MGMT_WIN_PERIOD_US 2000
`define AFDX_MGMT_WIN_START_US 0
`define AFDX_MGMT_WIN_OPEN_US 8
`endif
