/*
 * ================================================================
 * File Name    : afdx_redundancy_ctrl.v
 * Module Name  : afdx_redundancy_ctrl
 *
 * Function:
 *   冗余控制器（发送侧）：把一帧复制到网络 A 和网络 B，
 *   并改写源 MAC 地址最后一个字节的接口号（A=001, B=010），两路 MAC 各自计算 FCS。
 *   仅当所有使能网络的 FIFO 都可接收时才推进，保证两份拷贝逐字节同步。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_redundancy_ctrl (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] in_data,
    input  wire       in_valid,
    input  wire       in_last,
    input  wire [1:0] in_net_en,       // bit0: 发往 A, bit1: 发往 B
    output wire       in_ready,
    output wire [7:0] a_data,
    output wire       a_valid,
    output wire       a_last,
    input  wire       a_ready,
    output wire [7:0] b_data,
    output wire       b_valid,
    output wire       b_last,
    input  wire       b_ready
);
    reg [3:0] byte_idx;                  // 帧内字节计数（仅需识别第 11 字节）
    wire en_a = in_net_en[0];
    wire en_b = in_net_en[1];
    wire both_ready = (~en_a | a_ready) & (~en_b | b_ready);
    wire is_sa_iface = (byte_idx == 4'd11);

    assign in_ready = both_ready;
    assign a_valid  = in_valid & en_a & both_ready;
    assign b_valid  = in_valid & en_b & both_ready;
    assign a_last   = in_last;
    assign b_last   = in_last;
    assign a_data   = is_sa_iface ? {3'b001, in_data[4:0]} : in_data;
    assign b_data   = is_sa_iface ? {3'b010, in_data[4:0]} : in_data;

    always @(posedge clk) begin
        if (rst)
            byte_idx <= 4'd0;
        else if (in_valid && in_ready) begin
            if (in_last)
                byte_idx <= 4'd0;
            else if (byte_idx != 4'd15)
                byte_idx <= byte_idx + 1'b1;
        end
    end
endmodule
