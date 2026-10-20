/*
 * ================================================================
 * File Name    : afdx_rtc64.v
 * Module Name  : afdx_rtc64
 *
 * Function:
 *   64 位微秒实时时钟（RTC）：与单调时基共用 tick_us，可被网络同步校准。
 *   - 频率微调：每微秒把 freq_corr（单位 2^-32 us/us，有符号）累加到小数相位，进位/借位时多/少走 1 us；
 *   - 相位微调：slew_remaining 为待消除的相位（us，有符号），每 SLEW_DIV 个 tick 消除 1 us；
 *   - 步进：step 把 step_delta（有符号）一次加到 RTC；set 直接装载绝对值。
 *   微调时单个 tick 的增量不小于 0，因此业务时间戳不会回拨；只有步进/装载允许回拨。
 *   64 位更新分两拍（低 32 位 + 进位到高 32 位），rtc_us 在两半都更新后一起刷新，读出不撕裂。
 *   小数相位和与相位微调判定每拍预先寄存（frac/slew 只在 tick 处理时改变，tick 间隔远大于 1 拍），
 *   tick 路径只剩一次 33 位加法。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_rtc64 #(
    parameter SLEW_DIV = 1000                  // 最大相位微调速率 1/SLEW_DIV（默认 1000 ppm）
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick_us,
    input  wire signed [31:0] freq_corr,
    input  wire               slew_load,       // 装载新的待消除相位
    input  wire signed [31:0] slew_value,
    input  wire               step,
    input  wire signed [63:0] step_delta,
    input  wire               set,
    input  wire [63:0]        set_value,
    output reg  [63:0]        rtc_us,
    output reg  signed [31:0] slew_remaining,
    output wire               adjust_busy      // 步进/装载尚未生效
);
    `include "afdx_math.vh"
    localparam DW = afdx_clog2(SLEW_DIV < 2 ? 2 : SLEW_DIV);
    reg [31:0] lo, hi, frac;
    reg [DW-1:0] slew_cnt;
    reg tick_pending, carry_pending, publish;
    reg step_pending, set_pending, step_phase;
    reg [63:0] step_value;
    reg [32:0] step_lo_sum;
    // The microsecond source is in the data plane, far from the RTC. End
    // that route at a local flop before the tick/step/slew priority logic.
    (* preserve *) reg tick_local;
    always @(posedge clk) tick_local <= !rst && tick_us;
    assign adjust_busy = step_pending || set_pending || step_phase;
    // 一个 tick 的增量：1 + 频率进位 + 相位微调，取值 0..3（判定项均为上一拍寄存值）
    reg [32:0] frac_sum;
    reg frac_neg, slew_due, slew_neg, slew_fresh;
    wire f_up = !frac_neg && frac_sum[32];                   // 与写回 frac 的同一和值一致
    wire f_dn = frac_neg && frac_sum[32];
    wire s_up = slew_due && !slew_neg && !slew_fresh;
    wire s_dn = slew_due && slew_neg && !f_dn && !slew_fresh;  // 同拍两次借位时推迟相位微调
    wire [1:0] inc = 2'd1 + {1'b0, f_up} + {1'b0, s_up} - {1'b0, f_dn} - {1'b0, s_dn};
    wire [32:0] lo_sum = {1'b0, lo} + {31'd0, inc};
    always @(posedge clk) begin
        frac_sum <= {1'b0, frac} + {freq_corr[31], freq_corr};
        frac_neg <= freq_corr[31];
        slew_due <= (slew_cnt == 0) && (slew_remaining != 0);
        slew_neg <= slew_remaining[31];
        slew_fresh <= slew_load;                                // 刚装载：判定尚未反映新值
    end
    always @(posedge clk) begin
        if (rst) begin
            lo <= 32'd0; hi <= 32'd0; frac <= 32'd0; rtc_us <= 64'd0;
            slew_cnt <= {DW{1'b0}}; slew_remaining <= 32'sd0;
            tick_pending <= 1'b0; carry_pending <= 1'b0; publish <= 1'b0;
            step_pending <= 1'b0; set_pending <= 1'b0; step_phase <= 1'b0;
        end else begin
            publish <= 1'b0;
            if (tick_local) tick_pending <= 1'b1;
            if (step) begin step_pending <= 1'b1; step_value <= step_delta; end
            if (set) begin set_pending <= 1'b1; step_value <= set_value; end
            if (slew_load) slew_remaining <= slew_value;
            if (carry_pending) begin
                hi <= hi + 1'b1;
                carry_pending <= 1'b0;
                publish <= 1'b1;
            end else if (step_phase) begin
                lo <= step_lo_sum[31:0];
                hi <= set_pending ? step_value[63:32] : hi + step_value[63:32] + {31'd0, step_lo_sum[32]};
                set_pending <= 1'b0; step_pending <= 1'b0; step_phase <= 1'b0;
                publish <= 1'b1;
            end else if (tick_pending || tick_local) begin
                tick_pending <= 1'b0;
                frac <= frac_sum[31:0];
                slew_cnt <= (slew_cnt == SLEW_DIV - 1) ? {DW{1'b0}} : slew_cnt + 1'b1;
                if ((s_up || s_dn) && !slew_load)
                    slew_remaining <= slew_remaining + (s_up ? -32'sd1 : 32'sd1);
                lo <= lo_sum[31:0];
                carry_pending <= lo_sum[32];
                publish <= !lo_sum[32];
            end else if ((set_pending || step_pending) && !step && !set) begin
                step_lo_sum <= set_pending ? {1'b0, step_value[31:0]} :
                               {1'b0, lo} + {1'b0, step_value[31:0]};
                step_phase <= 1'b1;
            end
            if (publish) rtc_us <= {hi, lo};
        end
    end
endmodule
