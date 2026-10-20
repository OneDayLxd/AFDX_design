/*
 * ================================================================
 * File Name    : afdx_crc32.v
 * Module Name  : afdx_crc32
 *
 * Function:
 *   以太网 CRC-32（IEEE 802.3）逐字节更新，纯组合逻辑。
 *   反射算法：初值 0xFFFFFFFF，FCS = ~crc，低字节先发；
 *   接收端把 FCS 一并计入后余数固定为 0xDEBB20E3。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_defs.vh"
module afdx_crc32 (
    input  wire [31:0] crc_in,    // 当前 CRC 状态
    input  wire [7:0]  data,      // 输入字节
    output reg  [31:0] crc_out    // 更新后的 CRC 状态
);
    integer i;
    always @* begin
        crc_out = crc_in ^ {24'd0, data};
        for (i = 0; i < 8; i = i + 1)
            crc_out = crc_out[0] ? ((crc_out >> 1) ^ `AFDX_CRC_POLY_REV) : (crc_out >> 1);
    end
endmodule
