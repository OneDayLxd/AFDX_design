/*
 * ================================================================
 * File Name    : afdx_timer.v
 * Module Name  : afdx_timer
 *
 * Function:
 *   微秒节拍发生器与实时时钟（RTC）计数器。
 *   tick_us 每微秒产生 1 个时钟周期宽的脉冲，供 BAG 定时、冗余管理 SkewMax 计时使用；
 *   rtc_us 为 32 位微秒计数器（可作为 RTC 时钟同步的本地时基，支持外部装载）。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_timer #(
    parameter CLK_HZ = 100_000_000          // 系统时钟频率
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        rtc_load,            // RTC 装载使能（时钟同步）
    input  wire [31:0] rtc_load_val,        // 装载值（微秒）
    output reg         tick_us,             // 微秒脉冲
    output reg  [31:0] rtc_us,              // 可校准的 RTC 微秒计数
    output reg  [31:0] monotonic_us         // 不受 RTC 装载影响，用于协议超时
);
    `include "afdx_math.vh"
    localparam integer DIV = CLK_HZ / 1_000_000;
    localparam integer CW = afdx_clog2(DIV);
    reg [CW-1:0] div_cnt;
    always @(posedge clk) begin
        if (rst) begin
            div_cnt <= {CW{1'b0}};
            tick_us <= 1'b0;
            rtc_us  <= 32'd0;
            monotonic_us <= 32'd0;
        end else begin
            if (div_cnt == DIV - 1) begin
                div_cnt <= {CW{1'b0}};
                tick_us <= 1'b1;
            end else begin
                div_cnt <= div_cnt + 1'b1;
                tick_us <= 1'b0;
            end
            if (rtc_load)
                rtc_us <= rtc_load_val;
            else if (tick_us)
                rtc_us <= rtc_us + 32'd1;
            if (tick_us) monotonic_us <= monotonic_us + 32'd1;
        end
    end
endmodule
