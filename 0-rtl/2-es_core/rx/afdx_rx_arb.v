/*
 * ================================================================
 * File Name    : afdx_rx_arb.v
 * Module Name  : afdx_rx_arb
 *
 * Function:
 *   接收帧仲裁器：把网络 A / 网络 B 两路 MAC 接收帧流合并为一路送往帧解析器，
 *   以帧为单位切换（轮询），输出附带网络标识。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_rx_arb (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] a_data,
    input  wire       a_valid,
    input  wire       a_last,
    output wire       a_ready,
    input  wire [7:0] b_data,
    input  wire       b_valid,
    input  wire       b_last,
    output wire       b_ready,
    output reg  [7:0] out_data,
    output reg        out_valid,
    output reg        out_last,
    output reg        out_net,               // 0: 网络 A, 1: 网络 B
    input  wire [31:0] a_ts,
    input  wire [31:0] b_ts,
    output reg  [31:0] out_ts,
    input  wire       out_ready
);
    reg sel_valid, sel, rr;
    // Register the selected byte before parsing it. The MAC FIFO block RAM
    // output and A/B mux otherwise share a clock path with the parser's
    // header comparisons and 32-bit checksum adders.
    wire stage_ready = !out_valid || out_ready;
    wire selected_valid = sel ? b_valid : a_valid;
    wire selected_last = sel ? b_last : a_last;
    wire take = sel_valid && selected_valid && stage_ready;
    assign a_ready = sel_valid && !sel && stage_ready;
    assign b_ready = sel_valid &&  sel && stage_ready;

    always @(posedge clk) begin
        if (rst) begin
            sel_valid <= 1'b0;
            sel       <= 1'b0;
            rr        <= 1'b0;
            out_valid <= 1'b0;
            out_data  <= 8'd0;
            out_last  <= 1'b0;
            out_net   <= 1'b0;
            out_ts    <= 0;
        end else begin
            if (stage_ready) begin
                out_valid <= sel_valid && selected_valid;
                if (take) begin
                    out_data <= sel ? b_data : a_data;
                    out_last <= selected_last;
                    out_net <= sel;
                    out_ts <= sel ? b_ts : a_ts;
                end
            end
            if (!sel_valid) begin
                if (a_valid && (!b_valid || !rr)) begin
                    sel <= 1'b0; sel_valid <= 1'b1;
                end else if (b_valid) begin
                    sel <= 1'b1; sel_valid <= 1'b1;
                end
            end else if (take && selected_last) begin
                // 对端已有完整帧时直接交接；不能重选仍显示 last 的当前读口。
                if (sel ? a_valid : b_valid) begin
                    sel <= ~sel;
                    sel_valid <= 1'b1;
                end else sel_valid <= 1'b0;
                rr        <= ~sel;
            end
        end
    end
endmodule
