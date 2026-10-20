/*
 * ================================================================
 * File Name    : afdx_mgmt_events.v
 * Module Name  : afdx_mgmt_events
 *
 * Function:
 *   管理面事件 / Trap 控制：每类事件一个待发记录，限速窗口内同类事件合并（累计次数、首次/末次时间），
 *   被屏蔽的事件只累计 trapDropped；记录数固定为事件类数，故障风暴不会占用更多资源或挤占实时业务。
 *   多类待发时按严重级别选择：加载故障 > RTC 故障 > 接收错误 > 缓冲高水位/溢出 > 链路变化 > 配置变更。
 *   事件源：
 *     1 链路：网络 A/B 接收帧计数在 link_timeout 内无变化判为中断，恢复计数变化判为恢复（均产生事件）；
 *     2 接收错误合计较上次事件增加 err_threshold；3 缓冲高水位上升沿或丢弃计数变化（两者由管理面寄存后输入）；
 *     4 TFTP/615A 故障脉冲；5 RTC 故障脉冲；6 配置应用/回滚脉冲。
 *   时序：链路超时与限速窗口均为微秒节拍递减计数（装载时同拍清零零标志）；错误增量先寄存差值，
 *        门限比较再寄存一拍，触发后停两拍再比较，避免用到尚未更新的基线。窗口/超时配置修改时，正在计时的计数器按新值重新开始。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_mgmt_events (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] mono_us,
    input  wire [31:0] rx_frames_a,
    input  wire [31:0] rx_frames_b,
    input  wire [31:0] err_total,
    input  wire        buf_high,
    input  wire [31:0] drop_total,
    input  wire        ev_loader,
    input  wire        ev_rtc,
    input  wire        ev_cfg,
    input  wire [31:0] mask,                // bit t 屏蔽事件 t
    input  wire [31:0] interval_ms,
    input  wire [31:0] err_threshold,
    input  wire [31:0] link_timeout_ms,
    output reg         pending,
    output reg  [2:0]  pick,
    input  wire        take,                // Trap 作业开始：锁存 pick 的记录并清除
    output reg  [2:0]  tv_type,
    output reg  [31:0] tv_count,
    output reg  [31:0] tv_first,
    output reg  [31:0] tv_last,
    output reg  [31:0] cnt_dropped,
    output reg  [1:0]  link_up
);
    // ms -> us：x*1000 = (x<<10) - (x<<4) - (x<<3)，寄存一拍
    reg [31:0] interval_us, link_us, interval_q, link_q;
    always @(posedge clk) begin
        interval_q <= interval_us; link_q <= link_us;
        interval_us <= (interval_ms << 10) - (interval_ms << 4) - (interval_ms << 3);
        link_us <= (link_timeout_ms << 10) - (link_timeout_ms << 4) - (link_timeout_ms << 3);
    end
    reg mono0_q;
    wire tick = mono_us[0] != mono0_q;
    always @(posedge clk) mono0_q <= mono_us[0];
    // ---------------- 事件检测 ----------------
    reg [31:0] ra_q, rb_q, left_a, left_b, err_base, err_diff, drop_q;
    reg zero_a, zero_b, rx_a, rx_b, err_ge, thr_nz;
    reg [1:0] err_hold;
    reg buf_q;
    reg [6:1] ev;
    always @(posedge clk) begin
        if (rst) begin
            ra_q <= 0; rb_q <= 0; left_a <= 0; left_b <= 0; zero_a <= 1'b1; zero_b <= 1'b1;
            rx_a <= 1'b0; rx_b <= 1'b0; err_base <= 0; err_diff <= 0; err_hold <= 2'd2; drop_q <= 0;
            buf_q <= 1'b0; link_up <= 2'b00; ev <= 6'd0;
        end else begin
            ra_q <= rx_frames_a; rb_q <= rx_frames_b; drop_q <= drop_total; buf_q <= buf_high;
            rx_a <= ra_q != rx_frames_a; rx_b <= rb_q != rx_frames_b;
            ev <= 6'd0;
            if (link_q != link_us) begin
                if (!zero_a) left_a <= link_us;
                if (!zero_b) left_b <= link_us;
            end else begin
                if (tick && !zero_a) left_a <= left_a - 1'b1;
                if (tick && !zero_b) left_b <= left_b - 1'b1;
            end
            zero_a <= left_a == 32'd0;
            zero_b <= left_b == 32'd0;
            if (rx_a) begin
                left_a <= link_us; zero_a <= 1'b0;
                if (!link_up[0]) begin link_up[0] <= 1'b1; ev[1] <= 1'b1; end
            end else if (link_up[0] && zero_a) begin link_up[0] <= 1'b0; ev[1] <= 1'b1; end
            if (rx_b) begin
                left_b <= link_us; zero_b <= 1'b0;
                if (!link_up[1]) begin link_up[1] <= 1'b1; ev[1] <= 1'b1; end
            end else if (link_up[1] && zero_b) begin link_up[1] <= 1'b0; ev[1] <= 1'b1; end
            err_diff <= err_total - err_base;
            err_ge <= err_diff >= err_threshold;
            thr_nz <= err_threshold != 32'd0;
            // 触发后停两拍：err_ge 已对应新的基线
            if (err_hold != 2'd0) err_hold <= err_hold - 1'b1;
            else if (err_ge && thr_nz) begin
                ev[2] <= 1'b1; err_base <= err_total; err_hold <= 2'd2;
            end
            if ((buf_high && !buf_q) || drop_q != drop_total) ev[3] <= 1'b1;
            ev[4] <= ev_loader;
            ev[5] <= ev_rtc;
            ev[6] <= ev_cfg;
        end
    end
    // ---------------- 待发记录 ----------------
    reg [6:1] pend;
    reg [31:0] cnt [1:6];
    reg [31:0] first [1:6];
    reg [31:0] last [1:6];
    reg [31:0] win [1:6];                      // 限速窗口剩余微秒
    reg [6:1] win_zero;
    reg [6:1] elig;
    integer t;
    // 取走的记录（独热），及按独热与-或选出的记录内容
    // pick_oh 与 pick 同拍寄存，记录内容的选择不经过 take（take 只作装载使能）
    reg [6:1] take_oh, pick_oh;
    (* preserve *) reg pick_valid;
    reg [31:0] sel_cnt, sel_first, sel_last;
    always @* begin
        sel_cnt = 32'd0; sel_first = 32'd0; sel_last = 32'd0;
        for (t = 1; t <= 6; t = t + 1) begin
            take_oh[t] = take && pick_oh[t];
            sel_cnt = sel_cnt | ({32{pick_oh[t]}} & cnt[t]);
            sel_first = sel_first | ({32{pick_oh[t]}} & first[t]);
            sel_last = sel_last | ({32{pick_oh[t]}} & last[t]);
        end
    end
    always @(posedge clk) begin
        for (t = 1; t <= 6; t = t + 1)
            elig[t] <= pend[t] && !take_oh[t] && win_zero[t];
    end
    always @(posedge clk) begin
        if (rst) begin
            pend <= 6'd0; win_zero <= 6'h3f; cnt_dropped <= 32'd0; pending <= 1'b0; pick <= 3'd0; pick_oh <= 6'd0; pick_valid <= 1'b0;
            for (t = 1; t <= 6; t = t + 1) win[t] <= 32'd0;
            tv_type <= 3'd0; tv_count <= 32'd0; tv_first <= 32'd0; tv_last <= 32'd0;
        end else begin
            for (t = 1; t <= 6; t = t + 1) begin
                if (interval_q != interval_us) begin if (!win_zero[t]) win[t] <= interval_us; end
                else if (tick && !win_zero[t]) win[t] <= win[t] - 1'b1;
                win_zero[t] <= win[t] == 32'd0;
            end
            for (t = 1; t <= 6; t = t + 1) begin
                if (ev[t]) begin
                    if (mask[t]) cnt_dropped <= cnt_dropped + 1'b1;
                    else if (pend[t] && !take_oh[t]) begin
                        cnt[t] <= cnt[t] + 1'b1; last[t] <= mono_us;
                    end else begin
                        pend[t] <= 1'b1; cnt[t] <= 32'd1; first[t] <= mono_us; last[t] <= mono_us;
                    end
                end
            end
            if (take && pick_valid) begin
                tv_type <= pick; tv_count <= sel_cnt; tv_first <= sel_first; tv_last <= sel_last;
            end
            for (t = 1; t <= 6; t = t + 1)
                if (take_oh[t]) begin
                    if (!ev[t] || mask[t]) pend[t] <= 1'b0;
                    win[t] <= interval_us; win_zero[t] <= 1'b0;
                end
            // 严重级别优先
            pick_valid <= |elig;
            pending <= |elig && !take;
            pick <= elig[4] ? 3'd4 : elig[5] ? 3'd5 : elig[2] ? 3'd2 : elig[3] ? 3'd3 :
                    elig[1] ? 3'd1 : elig[6] ? 3'd6 : 3'd0;
            pick_oh <= elig[4] ? 6'b001000 : elig[5] ? 6'b010000 : elig[2] ? 6'b000010 :
                       elig[3] ? 6'b000100 : elig[1] ? 6'b000001 : elig[6] ? 6'b100000 : 6'd0;
        end
    end
endmodule
