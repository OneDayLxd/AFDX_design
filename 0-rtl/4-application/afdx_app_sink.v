/*
 * ================================================================
 * File Name    : afdx_app_sink.v
 * Module Name  : afdx_app_sink
 *
 * Function:
 *   内部应用接收槽：接收交付的消息并计数（模拟应用分区消费数据）。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_app_sink #(
    parameter VLW = 3
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [VLW-1:0] msg_vl,
    input  wire           msg_valid,
    input  wire [7:0]     msg_data,
    input  wire           msg_last,
    output wire           msg_ready,
    output reg  [31:0]    cnt_msgs,
    output reg  [31:0]    cnt_bytes
);
    assign msg_ready = 1'b1;
    always @(posedge clk) begin
        if (rst) begin
            cnt_msgs <= 32'd0; cnt_bytes <= 32'd0;
        end else if (msg_valid) begin
            cnt_bytes <= cnt_bytes + 1'b1;
            if (msg_last) cnt_msgs <= cnt_msgs + 1'b1;
        end
    end
endmodule
