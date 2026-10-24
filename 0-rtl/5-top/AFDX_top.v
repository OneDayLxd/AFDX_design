/*
 * ================================================================
 * File Name    : AFDX_top.v
 * Module Name  : AFDX_top
 *
 * Function:
 *   125MHz 单时钟端系统；rst 高有效。模板端口为应用载荷，不是 GMII。
 *   rx_* 提交发送消息，tx_* 交付去重后消息；last 是唯一消息边界。
 *   应用 valid 即有效，tx_ready 固定 1，rx_ready 不参与握手。
 *   A/B GMII 独立网络扩展包含前导码/SFD/FCS，帧内连续有效。
 *   输入缓存丢弃报错；每 VL Lmax/核心容量丢弃由管理统计记录。
 *
 * Sub Module:
 *   afdx_app_ingress
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
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
    input  wire                    clk,
    input  wire                    rst,
    output wire [7:0]              tx_data,
    input  wire [7:0]              rx_data,
    output wire                    tx_valid,
    input  wire                    rx_valid,
    output wire                    tx_ready,
    input  wire                    rx_ready,
    // vl 是配置索引，不是线路上的 16 位 VLID；消息内保持不变。
    input  wire [`AFDX_TX_VLW-1:0] rx_vl,
    input  wire                    rx_last,
    output wire [`AFDX_RX_VLW-1:0] tx_vl,
    output wire                    tx_last,
    output wire                    app_drop,
    output wire [2:0]              app_drop_reason,
    output wire [31:0]             app_drop_count,
    // 网络扩展：每拍一字节，IFG 时 valid=0，不使用应用 ready。
    output wire [7:0]              a_gmii_tx_data,
    output wire                    a_gmii_tx_valid,
    output wire                    a_gmii_tx_er,
    input  wire [7:0]              a_gmii_rx_data,
    input  wire                    a_gmii_rx_valid,
    input  wire                    a_gmii_rx_er,
    output wire [7:0]              b_gmii_tx_data,
    output wire                    b_gmii_tx_valid,
    output wire                    b_gmii_tx_er,
    input  wire [7:0]              b_gmii_rx_data,
    input  wire                    b_gmii_rx_valid,
    input  wire                    b_gmii_rx_er
);
    wire [`AFDX_TX_VLW-1:0] host_tx_vl;
    wire [7:0] host_tx_data;
    wire host_tx_valid, host_tx_last, host_tx_ready;
    // 核心同步复位有流水：保护期间可以收消息，暂不向核心提交。
    reg [4:0] startup;
    always @(posedge clk) begin
        if (rst) startup <= 0;
        else if (!startup[4]) startup <= startup + 1'b1;
    end
    assign tx_ready = 1'b1;
    afdx_app_ingress #(
        .VLW      (`AFDX_TX_VLW),
        .N_VL     (`AFDX_N_TX_VL),
        .SLOT_LOG (APP_SLOT_LOG)
    ) u_app_ingress (
        .clk          (clk),
        .rst          (rst),
        .in_vl        (rx_vl),
        .in_data      (rx_data),
        .in_valid     (rx_valid),
        .in_last      (rx_last),
        .out_vl       (host_tx_vl),
        .out_data     (host_tx_data),
        .out_valid    (host_tx_valid),
        .out_last     (host_tx_last),
        .out_ready    (host_tx_ready && startup[4]),
        .drop_pulse   (app_drop),
        .drop_reason  (app_drop_reason),
        .drop_count   (app_drop_count),
        .accept_count ()
    );
    afdx_mac_system #(
        .TFTP_TIMEOUT_US     (TFTP_TIMEOUT_US),
        .SELFTEST_US         (SELFTEST_US),
        .APP_AW              (APP_AW),
        .APP_EXTERNAL        (1),
        .SNMP_WRITE_COMM     (SNMP_WRITE_COMM),
        .SNMP_WRITE_COMM_LEN (SNMP_WRITE_COMM_LEN),
        .CLK_HZ              (125_000_000),
        .APP_GEN_EN          (0),
        .MGMT_EN             (MGMT_EN),
        .TX_PHASE_ENABLE     (TX_PHASE_ENABLE),
        .SKEW_MAX_US         (SKEW_MAX_US)
    ) u_mac_system (
        .clk            (clk),
        .reset          (rst),
        .tx_data        (a_gmii_tx_data),
        .rx_data        (a_gmii_rx_data),
        .tx_valid       (a_gmii_tx_valid),
        .rx_valid       (a_gmii_rx_valid),
        .tx_ready       (),
        .rx_ready       (1'b0),
        .rx_er          (a_gmii_rx_er),
        .tx_clk         (),
        .tx_er          (a_gmii_tx_er),
        .b_tx_data      (b_gmii_tx_data),
        .b_rx_data      (b_gmii_rx_data),
        .b_tx_valid     (b_gmii_tx_valid),
        .b_rx_valid     (b_gmii_rx_valid),
        .b_tx_ready     (),
        .b_rx_ready     (1'b0),
        .b_rx_er        (b_gmii_rx_er),
        .b_tx_clk       (),
        .b_tx_er        (b_gmii_tx_er),
        .host_tx_vl     (host_tx_vl),
        .host_tx_valid  (host_tx_valid && startup[4]),
        .host_tx_data   (host_tx_data),
        .host_tx_last   (host_tx_last),
        .host_tx_ready  (host_tx_ready),
        .host_rx_vl     (tx_vl),
        .host_rx_valid  (tx_valid),
        .host_rx_data   (tx_data),
        .host_rx_last   (tx_last),
        .host_rx_ready  (1'b1),
        .host_rx_len    (),
        .host_rx_sn     (),
        .host_rx_net    (),
        .host_rx_ts     (),
        .host_rx_src_ip (),
        .host_rx_sport  (),
        .host_rx_dport  ()
    );
endmodule
