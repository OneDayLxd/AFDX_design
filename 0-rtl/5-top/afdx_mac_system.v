/*
 * ================================================================
 * File Name    : afdx_mac_system.v
 * Module Name  : afdx_mac_system
 *
 * Function:
 *   集成应用、管理面、AFDX 核心与两路千兆全双工 MAC，提供同步 A/B GMII。
 *   保留外部应用边界供规模与管理测试使用；所有数据逻辑同步于 clk。
 *   模板 ready 不参与网络握手；reset 同时复位核心和数据面。
 *   固定千兆模式，不依赖 MDIO、厂商 IP、PHY 管理模型或派生时钟。
 *
 * Sub Module:
 *   afdx_system
 *   afdx_gmii_mac
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module afdx_mac_system #(
    parameter             TFTP_TIMEOUT_US      = 1000000,
    parameter             SELFTEST_US          = 50000,
    parameter             APP_AW               = 12,
    parameter             APP_EXTERNAL         = 0,
    parameter [127:0]     SNMP_WRITE_COMM      = 128'd0,
    parameter [4:0]       SNMP_WRITE_COMM_LEN  = 5'd0,
    parameter             CLK_HZ               = 125_000_000,
    parameter             APP_GEN_EN           = 1,
    parameter             MGMT_EN              = 1,
    parameter             SKEW_MAX_US          = 10000,
    parameter             TX_PHASE_ENABLE      = 0
) (
    // A/B 同步网络接口：valid 即有效，不与 ready 相与。
    input  wire                    clk,             // 核心/host 时钟，默认 125MHz
    input  wire                    reset,           // 高有效，复位核心及两路 MAC 数据面
    output wire [7:0]              tx_data,         // A 网 GMII TXD，同步于 tx_clk
    input  wire [7:0]              rx_data,         // A 网 GMII RXD，同步于 clk
    output wire                    tx_valid,        // A 网 GMII TX_EN
    input  wire                    rx_valid,        // A 网 GMII RX_DV
    output wire                    tx_ready,        // 模板兼容输出，固定 1，不表示缓存余量
    input  wire                    rx_ready,        // 模板兼容输入，按答疑不使用

    // 集成层保留错误注入/观察端口；外层同步入口将 RX_ER 固定为 0。
    // 数据范围为前导码、SFD、MAC 帧及 FCS；
    // IFG 期间 valid=0，不作为有效字节传输。RX 在 clk 上升沿采样。
    input  wire                    rx_er,           // GMII RX_ER
    output wire                    tx_clk,          // MAC 输出的发送时钟，千兆为 125MHz
    output wire                    tx_er,           // GMII TX_ER

    // B 网独立 GMII，与 A 网数据语义相同，不复用 A 网端口。
    output wire [7:0]              b_tx_data,
    input  wire [7:0]              b_rx_data,
    output wire                    b_tx_valid,
    input  wire                    b_rx_valid,
    output wire                    b_tx_ready,      // 兼容端口，固定 1
    input  wire                    b_rx_ready,      // 不参与网络字节握手
    input  wire                    b_rx_er,
    output wire                    b_tx_clk,
    output wire                    b_tx_er,

    // 应用侧载荷接口，全部同步于 clk，使用 valid && ready 握手。
    // APP_EXTERNAL=1 时启用外部应用发送；AFDX_top 在此边界接入应用适配器。
    input  wire [`AFDX_TX_VLW-1:0] host_tx_vl,
    input  wire                    host_tx_valid,
    input  wire [7:0]              host_tx_data,
    input  wire                    host_tx_last,
    output wire                    host_tx_ready,
    output wire [`AFDX_RX_VLW-1:0] host_rx_vl,
    output wire                    host_rx_valid,
    output wire [7:0]              host_rx_data,
    output wire                    host_rx_last,
    input  wire                    host_rx_ready,
    output wire [10:0]             host_rx_len,
    output wire [7:0]              host_rx_sn,
    output wire                    host_rx_net,
    output wire [31:0]             host_rx_ts,
    output wire [31:0]             host_rx_src_ip,
    output wire [15:0]             host_rx_sport,
    output wire [15:0]             host_rx_dport
);
    wire [15:0] core_tx_data, core_rx_data;
    wire [1:0] core_tx_valid, tx_last, core_tx_ready, wire_sof, wire_done;
    wire [1:0] core_rx_valid, rx_last, core_rx_ready;

    // 端口拆分/拼接；内部按低位 A、高位 B 连接两路 GMII MAC 适配器。
    wire [1:0] gmii_rx_er = {b_rx_er, rx_er};
    wire [1:0] gmii_rx_dv = {b_rx_valid, rx_valid};
    wire [15:0] gmii_rxd = {b_rx_data, rx_data};
    wire [1:0] gmii_tx_er, gmii_tx_en;
    wire [15:0] gmii_txd;
    assign {b_tx_data, tx_data} = gmii_txd;
    assign {b_tx_valid, tx_valid} = gmii_tx_en;
    assign {b_tx_clk, tx_clk} = {2{clk}};
    assign {b_tx_er, tx_er} = gmii_tx_er;
    assign tx_ready = 1'b1;
    assign b_tx_ready = 1'b1;
    wire [63:0] rx_ts, rx_good_count, rx_error_count, rx_length_count, rx_overflow_count;
    wire [31:0] ts_us;
    afdx_system #(
        .TFTP_TIMEOUT_US            (TFTP_TIMEOUT_US),
        .SELFTEST_US                (SELFTEST_US),
        .APP_AW                     (APP_AW),
        .APP_EXTERNAL               (APP_EXTERNAL),
        .SNMP_WRITE_COMM            (SNMP_WRITE_COMM),
        .SNMP_WRITE_COMM_LEN        (SNMP_WRITE_COMM_LEN),
        .EXTERNAL_MAC               (1),
        .CLK_HZ                     (CLK_HZ),
        .APP_GEN_EN                 (APP_GEN_EN),
        .MGMT_EN                    (MGMT_EN),
        .TX_PHASE_ENABLE            (TX_PHASE_ENABLE),
        .SKEW_MAX_US                (SKEW_MAX_US)
    ) u_system (
        .host_tx_vl                 (host_tx_vl),
        .host_tx_valid              (host_tx_valid),
        .host_tx_data               (host_tx_data),
        .host_tx_last               (host_tx_last),
        .host_tx_ready              (host_tx_ready),
        .host_rx_vl                 (host_rx_vl),
        .host_rx_valid              (host_rx_valid),
        .host_rx_data               (host_rx_data),
        .host_rx_last               (host_rx_last),
        .host_rx_ready              (host_rx_ready),
        .host_rx_len                (host_rx_len),
        .host_rx_sn                 (host_rx_sn),
        .host_rx_net                (host_rx_net),
        .host_rx_ts                 (host_rx_ts),
        .host_rx_src_ip             (host_rx_src_ip),
        .host_rx_sport              (host_rx_sport),
        .host_rx_dport              (host_rx_dport),
        .clk                        (clk),
        .reset                      (reset),
        .tx_data                    (),
        .rx_data                    (8'd0),
        .tx_valid                   (),
        .rx_valid                   (1'b0),
        .tx_ready                   (),
        .rx_ready                   (1'b0),
        .ext_ts_us                  (ts_us),
        .ext_a_tx_data              (core_tx_data[0 +: 8]),
        .ext_a_tx_valid             (core_tx_valid[0]),
        .ext_a_tx_last              (tx_last[0]),
        .ext_a_tx_ready             (core_tx_ready[0]),
        .ext_a_wire_sof             (wire_sof[0]),
        .ext_a_wire_done            (wire_done[0]),
        .ext_a_rx_data              (core_rx_data[0 +: 8]),
        .ext_a_rx_valid             (core_rx_valid[0]),
        .ext_a_rx_last              (rx_last[0]),
        .ext_a_rx_ready             (core_rx_ready[0]),
        .ext_a_rx_ts                (rx_ts[0 +: 32]),
        .ext_a_rx_good_count        (rx_good_count[0 +: 32]),
        .ext_a_rx_error_count       (rx_error_count[0 +: 32]),
        .ext_a_rx_length_count      (rx_length_count[0 +: 32]),
        .ext_a_rx_overflow_count    (rx_overflow_count[0 +: 32]),
        .ext_b_tx_data              (core_tx_data[8 +: 8]),
        .ext_b_tx_valid             (core_tx_valid[1]),
        .ext_b_tx_last              (tx_last[1]),
        .ext_b_tx_ready             (core_tx_ready[1]),
        .ext_b_wire_sof             (wire_sof[1]),
        .ext_b_wire_done            (wire_done[1]),
        .ext_b_rx_data              (core_rx_data[8 +: 8]),
        .ext_b_rx_valid             (core_rx_valid[1]),
        .ext_b_rx_last              (rx_last[1]),
        .ext_b_rx_ready             (core_rx_ready[1]),
        .ext_b_rx_ts                (rx_ts[32 +: 32]),
        .ext_b_rx_good_count        (rx_good_count[32 +: 32]),
        .ext_b_rx_error_count       (rx_error_count[32 +: 32]),
        .ext_b_rx_length_count      (rx_length_count[32 +: 32]),
        .ext_b_rx_overflow_count    (rx_overflow_count[32 +: 32])
    );
    genvar n;
    generate for (n=0; n<2; n=n+1) begin : g_net
        afdx_gmii_mac u_link (
            .clk                (clk),
            .reset              (reset),
            .ts_us              (ts_us),
            .tx_data            (core_tx_data[n*8 +: 8]),
            .tx_valid           (core_tx_valid[n]),
            .tx_last            (tx_last[n]),
            .tx_ready           (core_tx_ready[n]),
            .tx_sof             (wire_sof[n]),
            .tx_done            (wire_done[n]),
            .rx_data            (core_rx_data[n*8 +: 8]),
            .rx_valid           (core_rx_valid[n]),
            .rx_last            (rx_last[n]),
            .rx_ts              (rx_ts[n*32 +: 32]),
            .rx_ready           (core_rx_ready[n]),
            .rx_good_count      (rx_good_count[n*32 +: 32]),
            .rx_error_count     (rx_error_count[n*32 +: 32]),
            .rx_length_count    (rx_length_count[n*32 +: 32]),
            .rx_overflow_count  (rx_overflow_count[n*32 +: 32]),
            .eth_rx_er          (gmii_rx_er[n]),
            .eth_rx_dv          (gmii_rx_dv[n]),
            .eth_rxd            (gmii_rxd[n*8 +: 8]),
            .eth_tx_er          (gmii_tx_er[n]),
            .eth_tx_en          (gmii_tx_en[n]),
            .eth_txd            (gmii_txd[n*8 +: 8])
        );
    end endgenerate
endmodule
