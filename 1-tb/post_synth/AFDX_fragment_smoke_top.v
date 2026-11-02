/*
 * ================================================================
 * File Name    : AFDX_fragment_smoke_top.v
 * Module Name  : AFDX_fragment_smoke_top
 * Function:
 *   同版本生产端系统的小规模综合后夹具，应用端与 GMII 端均可直接观察。
 *   4 TX/4 RX VL、1472 字节上限，仅用于功能对照，不代表生产容量或时序。
 * Sub Module:
 *   AFDX_es
 * ================================================================
 */
module AFDX_fragment_smoke_top (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  app_tx_data,
    input  wire        app_tx_valid,
    input  wire        app_tx_last,
    output wire [7:0]  app_rx_data,
    output wire        app_rx_valid,
    output wire        app_rx_last,
    output wire [15:0] gmii_tx_data,
    output wire [1:0]  gmii_tx_valid,
    input  wire [15:0] gmii_rx_data,
    input  wire [1:0]  gmii_rx_valid
);
    AFDX_es #(
        .MGMT_EN          (0),
        .APP_MAX_LEN      (1472),
        .APP_SLOT_LOG     (2)
    ) u_es (
        .clk              (clk),
        .rst              (rst),
        .tx_data          (app_tx_data),
        .rx_data          (app_rx_data),
        .tx_valid         (app_tx_valid),
        .rx_valid         (app_rx_valid),
        .tx_ready         (),
        .rx_ready         (1'b0),
        .tx_last          (app_tx_last),
        .rx_last          (app_rx_last),
        .app_drop         (),
        .app_drop_reason  (),
        .app_drop_count   (),
        .a_gmii_tx_data   (gmii_tx_data[7:0]),
        .a_gmii_tx_valid  (gmii_tx_valid[0]),
        .a_gmii_tx_er     (),
        .a_gmii_rx_data   (gmii_rx_data[7:0]),
        .a_gmii_rx_valid  (gmii_rx_valid[0]),
        .a_gmii_rx_er     (1'b0),
        .b_gmii_tx_data   (gmii_tx_data[15:8]),
        .b_gmii_tx_valid  (gmii_tx_valid[1]),
        .b_gmii_tx_er     (),
        .b_gmii_rx_data   (gmii_rx_data[15:8]),
        .b_gmii_rx_valid  (gmii_rx_valid[1]),
        .b_gmii_rx_er     (1'b0)
    );
`ifndef SYNTHESIS
    defparam u_es.u_mac_system.g_net[0].u_link.RX_BYTE_LOG = 12;
    defparam u_es.u_mac_system.g_net[1].u_link.RX_BYTE_LOG = 12;
    defparam u_es.u_mac_system.g_net[0].u_link.RX_SLOT_LOG = 3;
    defparam u_es.u_mac_system.g_net[1].u_link.RX_SLOT_LOG = 3;
`endif
    defparam u_es.u_mac_system.u_system.TX_POOL_LOG = 3;
    defparam u_es.u_mac_system.u_system.RX_POOL_LOG = 3;
    defparam u_es.u_mac_system.u_system.TX_MAX_MSGS = 16;
endmodule
