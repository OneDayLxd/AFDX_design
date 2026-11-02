/*
 * ================================================================
 * File Name    : AFDX_top.sv
 * Module Name  : AFDX_top
 *
 * Function:
 *   应用级端到端仿真封装：两个相同端系统的 A/B GMII 双向直连。
 *   应用数据仍为大端VLID两字节及载荷，last结束消息，valid即有效。
 *   总线低8位为端点0，高8位为端点1；两端均可独立收发。
 *   连接不增加时钟周期或网络延迟；单端系统综合仍选择 AFDX_es。
 *
 * Sub Module:
 *   AFDX_es
 * ================================================================
 */
`timescale 1ns/1ps
module AFDX_top #(
    parameter         TFTP_TIMEOUT_US      = 1000000,
    parameter         SELFTEST_US          = 50000,
    parameter         APP_AW               = 12,
    parameter [127:0] SNMP_WRITE_COMM      = 128'd0,
    parameter [4:0]   SNMP_WRITE_COMM_LEN  = 5'd0,
    parameter         MGMT_EN              = 1,
    parameter         SKEW_MAX_US          = 10000,
    parameter         TX_PHASE_ENABLE      = 0,
    parameter         APP_SLOT_LOG         = 4
) (
    input  wire         clk,
    input  wire         rst,
    input  wire [15:0]  tx_data,
    input  wire [1:0]   tx_valid,
    input  wire [1:0]   tx_last,
    output wire [1:0]   tx_ready,
    output wire [15:0]  rx_data,
    output wire [1:0]   rx_valid,
    output wire [1:0]   rx_last,
    input  wire [1:0]   rx_ready,
    output wire [1:0]   app_drop,
    output wire [5:0]   app_drop_reason,
    output wire [63:0]  app_drop_count
);
    wire [31:0] gmii_tx_data, gmii_rx_data;
    wire [3:0] gmii_tx_valid, gmii_tx_er, gmii_rx_valid, gmii_rx_er;
    // 每端点低一路为 A、高一路为 B；对端 TX 连接本端 RX，包含错误信号。
    assign gmii_rx_data  = {gmii_tx_data[15:0], gmii_tx_data[31:16]};
    assign gmii_rx_valid = {gmii_tx_valid[1:0], gmii_tx_valid[3:2]};
    assign gmii_rx_er    = {gmii_tx_er[1:0], gmii_tx_er[3:2]};
    genvar e;
    generate for (e=0;e<2;e=e+1) begin : g_endpoint
        AFDX_es #(
            .TFTP_TIMEOUT_US     (TFTP_TIMEOUT_US),
            .SELFTEST_US         (SELFTEST_US),
            .APP_AW              (APP_AW),
            .SNMP_WRITE_COMM     (SNMP_WRITE_COMM),
            .SNMP_WRITE_COMM_LEN (SNMP_WRITE_COMM_LEN),
            .MGMT_EN             (MGMT_EN),
            .SKEW_MAX_US         (SKEW_MAX_US),
            .TX_PHASE_ENABLE     (TX_PHASE_ENABLE),
            .APP_SLOT_LOG        (APP_SLOT_LOG)
        ) u_es (
            .clk             (clk),
            .rst             (rst),
            .tx_data         (tx_data[e*8 +: 8]),
            .rx_data         (rx_data[e*8 +: 8]),
            .tx_valid        (tx_valid[e]),
            .rx_valid        (rx_valid[e]),
            .tx_ready        (tx_ready[e]),
            .rx_ready        (rx_ready[e]),
            .tx_last         (tx_last[e]),
            .rx_last         (rx_last[e]),
            .app_drop        (app_drop[e]),
            .app_drop_reason (app_drop_reason[e*3 +: 3]),
            .app_drop_count  (app_drop_count[e*32 +: 32]),
            .a_gmii_tx_data  (gmii_tx_data[(e*2)*8 +: 8]),
            .a_gmii_tx_valid (gmii_tx_valid[e*2]),
            .a_gmii_tx_er    (gmii_tx_er[e*2]),
            .a_gmii_rx_data  (gmii_rx_data[(e*2)*8 +: 8]),
            .a_gmii_rx_valid (gmii_rx_valid[e*2]),
            .a_gmii_rx_er    (gmii_rx_er[e*2]),
            .b_gmii_tx_data  (gmii_tx_data[(e*2+1)*8 +: 8]),
            .b_gmii_tx_valid (gmii_tx_valid[e*2+1]),
            .b_gmii_tx_er    (gmii_tx_er[e*2+1]),
            .b_gmii_rx_data  (gmii_rx_data[(e*2+1)*8 +: 8]),
            .b_gmii_rx_valid (gmii_rx_valid[e*2+1]),
            .b_gmii_rx_er    (gmii_rx_er[e*2+1])
        );
    end endgenerate
endmodule
