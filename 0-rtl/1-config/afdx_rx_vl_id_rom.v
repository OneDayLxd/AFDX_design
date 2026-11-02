/*
 * ================================================================
 * File Name    : afdx_rx_vl_id_rom.v
 * Module Name  : afdx_rx_vl_id_rom
 *
 * Function:
 *   将接收订阅索引还原为应用侧实际 VLID。
 *   与现有 afdx_rx_vl_lookup 的 0x1000 起连续订阅表配套维护。
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_rx_vl_id_rom #(
    parameter VLW = 12
) (
    input  wire [VLW-1:0] idx,
    output wire [15:0]    vlid
);
    assign vlid = 16'h1000 + idx;
endmodule
