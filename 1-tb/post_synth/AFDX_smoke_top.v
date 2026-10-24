/*
 * ================================================================
 * File Name    : AFDX_smoke_top.v
 * Module Name  : AFDX_smoke_top
 *
 * Function:
 *   综合后功能测试夹具：生产顶层逻辑不变，缩小缓存的合法参数取值。
 *   与小规模配置头一起使用；不代表生产容量、吞吐或时序验证。
 * Sub Module:
 *   AFDX_top
 * ================================================================
 */
`timescale 1ns/1ps
module AFDX_smoke_top (
    input  wire       clk,
    input  wire       rst,
    output wire [7:0] tx_data,
    input  wire [7:0] rx_data,
    output wire       tx_valid,
    input  wire       rx_valid,
    output wire       tx_ready,
    input  wire       rx_ready,
    output wire [7:0] b_tx_data,
    input  wire [7:0] b_rx_data,
    output wire       b_tx_valid,
    input  wire       b_rx_valid,
    output wire       b_tx_ready,
    input  wire       b_rx_ready
);
    AFDX_top u_production (
        .clk         (clk),
        .rst         (rst),
        .tx_data     (tx_data),
        .rx_data     (rx_data),
        .tx_valid    (tx_valid),
        .rx_valid    (rx_valid),
        .tx_ready    (tx_ready),
        .rx_ready    (rx_ready),
        .b_tx_data   (b_tx_data),
        .b_rx_data   (b_rx_data),
        .b_tx_valid  (b_tx_valid),
        .b_rx_valid  (b_rx_valid),
        .b_tx_ready  (b_tx_ready),
        .b_rx_ready  (b_rx_ready)
    );
    // 只对测试实例覆盖已存在的容量参数，不替换模块或加速时基。
    // 旧综合器的 generate 实例参数由工程脚本覆盖；RTL 使用相同值。
`ifndef SYNTHESIS
    defparam u_production.u_mac_system.g_net[0].u_link.RX_BYTE_LOG = 12;
    defparam u_production.u_mac_system.g_net[1].u_link.RX_BYTE_LOG = 12;
    defparam u_production.u_mac_system.g_net[0].u_link.RX_SLOT_LOG = 3;
    defparam u_production.u_mac_system.g_net[1].u_link.RX_SLOT_LOG = 3;
`endif
    defparam u_production.u_mac_system.u_system.TX_POOL_LOG = 3;
    defparam u_production.u_mac_system.u_system.RX_POOL_LOG = 3;
    defparam u_production.u_mac_system.u_system.TX_MAX_MSGS = 16;
endmodule
