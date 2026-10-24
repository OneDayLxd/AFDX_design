/*
 * ================================================================
 * File Name    : afdx_vl_config.v
 * Module Name  : afdx_tx_vl_rom / afdx_tx_vl_id_rom
 *                afdx_rx_vl_lookup / afdx_gen_rom
 *
 * Function:
 *   小规模测试配置：四个配置模块共同组成一份可替换的测试 ROM。
 *   BAG/Lmax 保持标准取值；三个应用 VL 首次同时提交。
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_tx_vl_rom #(parameter VLW = 2) (
    input  wire [VLW-1:0] idx,
    output wire [15:0]    vlid,
    output wire [19:0]    bag_us,
    output wire [10:0]    lmax,
    output wire [15:0]    udp_src,
    output wire [15:0]    udp_dst,
    output wire [1:0]     net_en,
    output wire [4:0]     partition
);
    assign vlid      = 16'h1000 + idx;
    assign bag_us    = 20'd2000;
    assign lmax      = idx == 0 ? 11'd516 : 11'd1471;
    assign udp_src   = 16'd1000 + idx;
    assign udp_dst   = 16'd2000 + idx;
    assign net_en    = 2'b11;
    assign partition = 5'd1;
endmodule

module afdx_tx_vl_id_rom #(parameter VLW = 2) (
    input  wire [VLW-1:0] idx,
    output wire [15:0]    vlid,
    output wire [10:0]    lmax
);
    assign vlid = 16'h1000 + idx;
    assign lmax = idx == 0 ? 11'd516 : 11'd1471;
endmodule

module afdx_rx_vl_lookup #(parameter VLW = 2) (
    input  wire [15:0]    vlid,
    output wire          hit,
    output wire [VLW-1:0] idx
);
    assign hit = vlid >= 16'h1000 && vlid <= 16'h1003;
    assign idx = vlid[VLW-1:0];
endmodule

module afdx_gen_rom #(parameter VLW = 2) (
    input  wire [VLW-1:0] idx,
    output wire [19:0]    period_us,
    output wire [19:0]    phase_us,
    output wire [10:0]    len
);
    assign period_us = idx == 0 ? 20'd0 : 20'd2000;
    assign phase_us  = 20'd0;
    assign len       = 11'd17;
endmodule
