/*
 * ================================================================
 * File Name    : afdx_defs.vh
 * Module Name  : none
 *
 * Function:
 *   AFDX 端系统公共常量定义（ARINC 664 Part 7）
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`ifndef AFDX_DEFS_VH
`define AFDX_DEFS_VH
`define AFDX_ETH_TYPE_IPV4   16'h0800      // 以太网类型：IPv4
`define AFDX_IP_PROTO_UDP    8'd17         // IP 协议号：UDP
`define AFDX_MIN_FRAME       14'd64        // 最小帧长（含 FCS）
`define AFDX_MAX_FRAME       14'd1518      // 最大帧长（含 FCS）
`define AFDX_MIN_PAYLOAD     11'd17        // 最小 AFDX 有效载荷
`define AFDX_MAX_PAYLOAD     11'd1471      // 最大 AFDX 有效载荷
`define AFDX_HDR_LEN         14'd42        // MAC(14)+IP(20)+UDP(8)
`define AFDX_CRC_INIT        32'hFFFFFFFF
`define AFDX_CRC_RESIDUE     32'hDEBB20E3  // 含 FCS 计算后的 CRC 余数
`define AFDX_CRC_POLY_REV    32'hEDB88320  // CRC-32 反射多项式
`endif
