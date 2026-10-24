/*
 * ================================================================
 * File Name    : afdx_phase_gate.v
 * Module Name  : afdx_phase_gate
 *
 * Function:
 *   周期性 VL 发送窗口；每个 VL 每轮最多接受一次授权。
 *   窗口只约束组帧开始，不替代出口 BAG，也不推断应用消息与 RX 的关系。
 *   复位确定相位；RTC 校时不改变调度时基。
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_phase_gate #(
    parameter N_VL          = 128,
    parameter VLW           = 7,
    parameter SLOT_CYCLES   = 8000,
    parameter OPEN_CYCLE    = 7500,
    parameter WINDOW_CYCLES = 125
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            grant_ack,
    input  wire [VLW-1:0]  grant_vl,
    output wire [N_VL-1:0] permit,
    output wire            window_open,
    output reg  [VLW-1:0]  window_vl
);
    `include "afdx_math.vh"
    localparam CW = afdx_clog2(SLOT_CYCLES);
    reg [CW-1:0] cycle;
    reg [VLW-1:0] slot;
    reg used;
    reg open_q;
    reg [N_VL-1:0] selected_q;
    wire last_cycle = cycle == SLOT_CYCLES - 1;

    // 仅一个窄计数器；窗口和 VL 译码各寄存一拍后驱动调度流水线。
    always @(posedge clk) begin
        if (rst) begin
            cycle <= 0;
            slot <= 0;
            used <= 1'b0;
            open_q <= 1'b0;
            window_vl <= 0;
        end else begin
            cycle <= last_cycle ? {CW{1'b0}} : cycle + 1'b1;
            if (last_cycle) begin
                slot <= slot == N_VL - 1 ? {VLW{1'b0}} : slot + 1'b1;
                used <= 1'b0;
            end else if (grant_ack && grant_vl == slot) used <= 1'b1;
            open_q <= cycle >= OPEN_CYCLE && cycle < OPEN_CYCLE + WINDOW_CYCLES;
            window_vl <= slot;
        end
    end
    genvar v;
    generate for (v = 0; v < N_VL; v = v + 1) begin : g_decode
        always @(posedge clk) begin
            if (rst) selected_q[v] <= 1'b0;
            else selected_q[v] <= slot == v;
        end
    end endgenerate
    assign window_open = open_q && !used;
    assign permit = selected_q & {N_VL{window_open}};
endmodule
