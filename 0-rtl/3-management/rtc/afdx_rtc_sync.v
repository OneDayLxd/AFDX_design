/*
 * ================================================================
 * File Name    : afdx_rtc_sync.v
 * Module Name  : afdx_rtc_sync
 *
 * Function:
 *   RTC 网络同步引擎（管理 VL 上的版本化 UDP 时间协议，报文 36 字节，大端）：
 *     0 "ATIM" | 4 ver=2 | 5 type | 6 len=36 | 8 seq | 12 source_id | 14 quality | 15 log2 周期(us)
 *     16 ts_a(64) | 24 ts_b(64) | 32 status | 33..35 保留
 *   type 1 SYNC(主→从, ts_a=t1)  2 DELAY_REQ(从→主, ts_a=t2)  3 DELAY_RESP(主→从, ts_a=t4)
 *        4 ONEWAY(主→从, ts_a=主时间，用配置的链路时延校正，状态为 DEGRADED)
 *   t2 为请求报文进入解析器时的 RTC，t3 为 DELAY_REQ 帧实际 SOF 时的 RTC（带标记帧的 SOF 脉冲）。
 *     offset = ((t2-t1) - (t4-t3)) / 2，path_delay = ((t2-t1) + (t4-t3)) / 2，全部在 66 位有符号域计算。
 *   伺服：首次同步或 |offset| > step_threshold 时步进；否则相位微调并按 offset/周期 积分频率校正。
 *   序号按 32 位模比较：重复（同内容）只重发 DELAY_REQ，内容冲突记协议错误，旧序号拒绝，
 *   前跳累计丢失数；连续 4 个旧序号视为主端序号复位。所有超时均基于单调时间 mono_us 的微秒节拍。
 *   时序：报文字段比较、序号差、SYNC 分支判定、偏差饱和/门限比较、频率移位（分两拍）与限幅比较均各占一级寄存，
 *   超时用微秒节拍递减计数器（门限在计时开始时采样；SNMP 修改门限时正在计时的计数器按新值重新开始）。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_rtc_sync #(
    parameter [15:0] LOCAL_ID = 16'h0101,
    parameter LOCK_US = 30,                     // 锁定判据 |offset|
    parameter LOCK_COUNT = 4,                   // 连续满足判据的样本数
    parameter RESP_TIMEOUT_US = 100000,
    parameter signed [31:0] FREQ_LIMIT = 32'sd4194304   // ±约 976 ppm
) (
    input  wire               clk,
    input  wire               rst,
    input  wire [31:0]        mono_us,
    input  wire [63:0]        rtc_us,
    // 作业接口
    input  wire               go,
    output reg                done,
    input  wire [10:0]        req_len,
    input  wire [31:0]        req_ts,
    input  wire [15:0]        req_sport,
    output reg  [10:0]        rq_addr,
    input  wire [7:0]         rq_data,
    output reg                tb_wr_en,
    output reg  [10:0]        tb_wr_addr,
    output reg  [7:0]         tb_wr_data,
    output reg                tb_post,
    output wire [10:0]        tb_len,
    output reg  [15:0]        tb_dport,
    input  wire               sof_tag,
    // RTC 控制
    output reg  signed [31:0] freq_corr,
    output reg                slew_load,
    output reg  signed [31:0] slew_value,
    output reg                step,
    output reg  signed [63:0] step_delta,
    output reg                set,
    output reg  [63:0]        set_value,
    // 配置（SNMP 可写）
    input  wire [31:0]        step_threshold,
    input  wire [31:0]        holdover_us,
    input  wire [31:0]        unsync_us,
    input  wire [31:0]        oneway_delay_us,
    input  wire               manual_set,
    input  wire [63:0]        manual_value,
    // 状态与统计
    output reg  [2:0]         state,            // 0 UNSYNCED 1 ACQUIRING 2 LOCKED 3 HOLDOVER 4 DEGRADED 5 FAULT
    output reg  [31:0]        cnt_sync, cnt_step, cnt_slew, cnt_switch,
    output reg  [31:0]        cnt_lost, cnt_dup, cnt_ooo, cnt_bad, cnt_timeout, cnt_reject,
    output reg  [31:0]        last_seq,
    output reg  [31:0]        last_sync_mono,
    output reg  signed [31:0] last_offset,      // 饱和到 32 位
    output reg  [31:0]        max_abs_offset,
    output reg  [31:0]        path_delay,
    output reg  [15:0]        source_id,
    output reg  [7:0]         source_quality,
    output reg  [7:0]         last_error,
    output reg                ev_fault          // 失锁/偏差越限/超时/序号异常事件脉冲
);
    localparam S_UNSYNC = 3'd0, S_ACQ = 3'd1, S_LOCK = 3'd2, S_HOLD = 3'd3, S_DEGR = 3'd4, S_FAULT = 3'd5;
    localparam R_IDLE = 5'd0, R_READ = 5'd1, R_PRE = 5'd2, R_CHECK = 5'd3, R_SYNC = 5'd4, R_T2 = 5'd5,
               R_SEND = 5'd6, R_EMIT = 5'd7, R_POST = 5'd8, R_RESP = 5'd9, R_CALC1 = 5'd10, R_CALC2 = 5'd11,
               R_CALC3 = 5'd12, R_CALC4 = 5'd13, R_SERVO = 5'd14, R_FREQ = 5'd15, R_FREQ2 = 5'd16,
               R_DONE = 5'd17, R_SYNC0 = 5'd18, R_CALC5 = 5'd19, R_FREQ3 = 5'd20;
    localparam E_FORMAT = 8'd1, E_TYPE = 8'd2, E_CONFLICT = 8'd3, E_OLD = 8'd4, E_SOURCE = 8'd5,
               E_MISMATCH = 8'd6, E_TIMEOUT = 8'd7, E_FAULT = 8'd8;
    reg [4:0] r;
    reg [5:0] rd_i;
    reg rd_v, rd_v2;
    (* preserve *) reg [7:0] rq_q;              // 读出数据就近寄存一拍（各引擎各一份）
    always @(posedge clk) rq_q <= rq_data;
    reg [5:0] got;
    reg [287:0] msg;
    wire [31:0] m_magic = msg[287:256];
    wire [7:0] m_ver = msg[255:248], m_type = msg[247:240];
    wire [15:0] m_len = msg[239:224];
    wire [31:0] m_seq = msg[223:192];
    wire [15:0] m_src = msg[191:176];
    wire [7:0] m_q = msg[175:168], m_logp = msg[167:160];
    wire [63:0] m_ts_a = msg[159:96];
    wire [7:0] m_status = msg[31:24];
    assign tb_len = 11'd36;

    // 交换状态
    reg pending, t3_valid, t3_armed, seq_valid, have_src, oneway;
    reg [31:0] pend_seq, done_seq;
    reg [63:0] t1, t2, t3, t4, last_t1;
    reg [7:0] log_period;
    reg [2:0] ooo_run;
    reg [3:0] lock_cnt;
    reg signed [65:0] d1, d2, off2, dly2;
    reg signed [64:0] offset;
    reg [63:0] abs_off, neg_off, shv;
    reg signed [63:0] freq_step, freq_sum;
    wire signed [63:0] freq_lim = FREQ_LIMIT;
    // R_PRE 寄存的报文判定
    reg fmt_ok, is_sync, is_ow, is_resp, st_bad, src_diff, q_ge, q_gt, q_ff, ts_same, pend_hit, done_hit;
    reg [7:0] ts_byte_same;
    genvar tsb;
    generate for (tsb = 0; tsb < 8; tsb = tsb + 1) begin : g_ts_compare
        always @(posedge clk)
            if (r == R_PRE) ts_byte_same[tsb] <= m_ts_a[8*tsb +: 8] == last_t1[8*tsb +: 8];
    end endgenerate
    reg [31:0] seq_d, lost_add, t2_hi, t2_hi_now;
    reg t2_same_hi;
    reg seq_zero, seq_old, seq_gap;
    // R_SYNC0 寄存的 SYNC 处理分支（独热）
    reg sy_src, sy_acc, sy_reset, sy_lost, sy_dup, sy_conf;
    reg f_hi, f_lo;
    reg [63:0] shv1;
    // R_CALC 寄存的伺服判定
    reg sh_right, big, near, newmax, abs_sat, sat_pos, sat_neg;
    reg [5:0] sh_amt;
    // 步进门限上限 2^30 us，保证微调量可用 32 位有符号数表示（配置值寄存一拍）
    reg [31:0] step_limit;
    // 超时：微秒节拍递减计数，零标志寄存一拍
    reg mono0_q;
    wire tick = mono_us[0] != mono0_q;
    reg [31:0] resp_left, hold_left, uns_left, hold_cfg, uns_cfg;
    reg resp_zero, hold_zero, uns_zero;
    // DELAY_REQ 报文，逐字节从最高位移出
    reg [287:0] out;
    reg [5:0] wr_i;

    always @(posedge clk) begin
        if (rst) begin
            r <= R_IDLE; done <= 1'b0; rq_addr <= 11'd0; rd_i <= 6'd0; rd_v <= 1'b0; rd_v2 <= 1'b0; got <= 6'd0;
            tb_wr_en <= 1'b0; tb_post <= 1'b0; tb_dport <= 16'd0;
            freq_corr <= 32'sd0; slew_load <= 1'b0; slew_value <= 32'sd0; step <= 1'b0;
            step_delta <= 64'sd0; set <= 1'b0; set_value <= 64'd0;
            state <= S_UNSYNC; cnt_sync <= 0; cnt_step <= 0; cnt_slew <= 0; cnt_switch <= 0;
            cnt_lost <= 0; cnt_dup <= 0; cnt_ooo <= 0; cnt_bad <= 0; cnt_timeout <= 0; cnt_reject <= 0;
            last_seq <= 0; last_sync_mono <= 0; last_offset <= 0; max_abs_offset <= 0; path_delay <= 0;
            source_id <= 0; source_quality <= 8'hff; last_error <= 0; ev_fault <= 1'b0;
            pending <= 1'b0; t3_valid <= 1'b0; t3_armed <= 1'b0; seq_valid <= 1'b0; have_src <= 1'b0;
            oneway <= 1'b0; ooo_run <= 3'd0; lock_cnt <= 4'd0; log_period <= 8'd20; done_seq <= 0;
            mono0_q <= 1'b0; resp_left <= 0; hold_left <= 0; uns_left <= 0;
            resp_zero <= 1'b1; hold_zero <= 1'b1; uns_zero <= 1'b1; step_limit <= 32'd0;
        end else begin
            done <= 1'b0; tb_wr_en <= 1'b0; tb_post <= 1'b0; ev_fault <= 1'b0;
            slew_load <= 1'b0; step <= 1'b0; set <= 1'b0;
            step_limit <= (step_threshold > 32'h40000000) ? 32'h40000000 : step_threshold;
            if (sof_tag && t3_armed) begin t3 <= rtc_us; t3_valid <= 1'b1; t3_armed <= 1'b0; end
            // 超时计数（单调时间，不受 RTC 步进影响）；装载处的赋值在后，覆盖此处
            mono0_q <= mono_us[0];
            if (tick && !resp_zero) resp_left <= resp_left - 1'b1;
            hold_cfg <= holdover_us; uns_cfg <= unsync_us;
            if (hold_cfg != holdover_us) begin if (!hold_zero) hold_left <= holdover_us; end
            else if (tick && !hold_zero) hold_left <= hold_left - 1'b1;
            if (uns_cfg != unsync_us) begin if (!uns_zero) uns_left <= unsync_us; end
            else if (tick && !uns_zero) uns_left <= uns_left - 1'b1;
            resp_zero <= resp_left == 32'd0;
            hold_zero <= hold_left == 32'd0;
            uns_zero <= uns_left == 32'd0;
            if (r == R_IDLE) begin
                if (pending && resp_zero) begin
                    pending <= 1'b0; t3_armed <= 1'b0; cnt_timeout <= cnt_timeout + 1'b1;
                    last_error <= E_TIMEOUT;
                end else if ((state == S_LOCK || state == S_DEGR) && hold_zero) begin
                    state <= S_HOLD; ev_fault <= 1'b1;
                end else if (state != S_UNSYNC && state != S_FAULT && uns_zero) begin
                    state <= S_UNSYNC; cnt_timeout <= cnt_timeout + 1'b1; last_error <= E_TIMEOUT;
                    lock_cnt <= 4'd0; ev_fault <= 1'b1;
                end
                if (manual_set) begin
                    set <= 1'b1; set_value <= manual_value; state <= S_UNSYNC; lock_cnt <= 4'd0;
                    cnt_step <= cnt_step + 1'b1;
                end
            end
            case (r)
                R_IDLE: if (go) begin
                    rd_i <= 6'd0; got <= 6'd0; rd_v <= 1'b0; rd_v2 <= 1'b0; rq_addr <= 11'd0;
                    if (req_len < 11'd36) begin
                        cnt_bad <= cnt_bad + 1'b1; last_error <= E_FORMAT; r <= R_DONE;
                    end else r <= R_READ;
                end
                R_READ: begin
                    // 地址逐拍递增，数据（RAM 读出后再寄存一拍）晚两拍移入
                    rd_v <= rd_i != 6'd36;
                    rd_v2 <= rd_v;
                    if (rd_i != 6'd36) begin rq_addr <= {5'd0, rd_i} + 11'd1; rd_i <= rd_i + 1'b1; end
                    if (rd_v2) begin msg <= {msg[279:0], rq_q}; got <= got + 1'b1; end
                    if (got == 6'd36) r <= R_PRE;
                end
                R_PRE: begin
                    fmt_ok <= m_magic == 32'h4154494D && m_ver == 8'd2 && m_len == 16'd36;
                    is_sync <= m_type == 8'd1 || m_type == 8'd4;
                    is_ow <= m_type == 8'd4;
                    is_resp <= m_type == 8'd3;
                    st_bad <= m_status != 8'd0;
                    src_diff <= m_src != source_id;
                    q_ge <= m_q >= source_quality;
                    q_gt <= m_q > source_quality;
                    q_ff <= m_q == 8'hff;
                    pend_hit <= pending && m_seq == pend_seq;
                    done_hit <= m_seq == done_seq;
                    seq_d <= m_seq - last_seq;
                    // t2：接收时间戳低 32 位扩展为 64 位（接收发生在当前时刻之前）；先比较并采样高位
                    t2_same_hi <= req_ts <= rtc_us[31:0];
                    t2_hi_now <= rtc_us[63:32];
                    r <= R_CHECK;
                end
                R_CHECK: begin
                    ts_same <= &ts_byte_same;
                    t2_hi <= t2_same_hi ? t2_hi_now : t2_hi_now - 1'b1;
                    seq_zero <= seq_d == 32'd0;
                    seq_old <= seq_d[31];
                    seq_gap <= !seq_d[31] && |seq_d[30:1];
                    lost_add <= seq_d - 1'b1;
                    if (!fmt_ok) begin
                        cnt_bad <= cnt_bad + 1'b1; last_error <= E_FORMAT; r <= R_DONE;
                    end else if (is_sync) begin
                        if (st_bad) begin
                            state <= S_FAULT; last_error <= E_FAULT; cnt_reject <= cnt_reject + 1'b1;
                            ev_fault <= state != S_FAULT; r <= R_DONE;
                        end else if (q_ff || (have_src && src_diff && q_ge && !hold_zero)) begin
                            cnt_reject <= cnt_reject + 1'b1; last_error <= E_SOURCE; r <= R_DONE;
                        end else r <= R_SYNC0;
                    end else if (is_resp) begin
                        r <= R_RESP;
                    end else begin
                        cnt_bad <= cnt_bad + 1'b1; last_error <= E_TYPE; r <= R_DONE;
                    end
                end
                R_SYNC0: begin
                    sy_src <= !have_src || src_diff;
                    sy_acc <= have_src && !src_diff && (!seq_valid || (!seq_zero && !seq_old) || ooo_run == 3'd4);
                    sy_reset <= seq_valid && seq_old;
                    sy_lost <= seq_valid && !seq_old && seq_gap;
                    sy_dup <= have_src && !src_diff && seq_valid && seq_zero && ooo_run != 3'd4 && ts_same;
                    sy_conf <= have_src && !src_diff && seq_valid && seq_zero && ooo_run != 3'd4 && !ts_same;
                    r <= R_SYNC;
                end
                R_SYNC: begin
                    oneway <= is_ow;
                    if (sy_src) begin
                        // 首个源或更优源：切换并重新采集序号
                        if (have_src) cnt_switch <= cnt_switch + 1'b1;
                        have_src <= 1'b1; source_id <= m_src; source_quality <= m_q;
                        seq_valid <= 1'b1; last_seq <= m_seq; last_t1 <= m_ts_a; ooo_run <= 3'd0;
                        lock_cnt <= 4'd0; if (state == S_LOCK) state <= S_ACQ;
                        r <= R_T2;
                    end else if (sy_acc) begin
                        if (sy_reset) begin
                            // 连续旧序号：主端序号复位
                            lock_cnt <= 4'd0; if (state == S_LOCK) state <= S_ACQ; ev_fault <= 1'b1;
                        end else if (sy_lost)
                            cnt_lost <= cnt_lost + lost_add;
                        if (q_gt) begin
                            lock_cnt <= 4'd0; if (state == S_LOCK) state <= S_ACQ;
                        end
                        source_quality <= m_q;
                        seq_valid <= 1'b1; last_seq <= m_seq; last_t1 <= m_ts_a; ooo_run <= 3'd0;
                        r <= R_T2;
                    end else if (sy_dup) begin
                        cnt_dup <= cnt_dup + 1'b1;
                        // 只重发 DELAY_REQ（重新取 t3），不重复校时
                        r <= (pend_hit && !is_ow) ? R_SEND : R_DONE;
                        if (pend_hit) t3_valid <= 1'b0;
                    end else if (sy_conf) begin
                        cnt_bad <= cnt_bad + 1'b1; last_error <= E_CONFLICT; ev_fault <= 1'b1; r <= R_DONE;
                    end else begin
                        cnt_ooo <= cnt_ooo + 1'b1; last_error <= E_OLD; ooo_run <= ooo_run + 1'b1; r <= R_DONE;
                    end
                    log_period <= m_logp;
                end
                R_T2: begin
                    t1 <= m_ts_a;
                    t2 <= {t2_hi, req_ts};
                    if (oneway) begin
                        pending <= 1'b0;
                        t4 <= 64'd0;
                        r <= R_CALC1;
                    end else begin
                        pending <= 1'b1; pend_seq <= m_seq; t3_valid <= 1'b0;
                        resp_left <= RESP_TIMEOUT_US; resp_zero <= 1'b0;
                        r <= R_SEND;
                    end
                end
                R_SEND: begin
                    out <= {32'h4154494D, 8'd2, 8'd2, 16'd36, pend_seq, LOCAL_ID, {5'd0, state},
                            log_period, t2, 64'd0, 8'd0, 24'd0};
                    wr_i <= 6'd0;
                    tb_dport <= req_sport;
                    r <= R_EMIT;
                end
                R_EMIT: begin                           // 逐字节写入发送缓冲
                    tb_wr_en <= 1'b1; tb_wr_addr <= {5'd0, wr_i}; tb_wr_data <= out[287:280];
                    out <= {out[279:0], 8'd0};
                    wr_i <= wr_i + 1'b1;
                    if (wr_i == 6'd35) r <= R_POST;
                end
                R_POST: begin
                    tb_post <= 1'b1; t3_armed <= 1'b1; r <= R_DONE;
                end
                R_RESP: begin
                    if (pend_hit && t3_valid) begin
                        t4 <= m_ts_a; pending <= 1'b0; done_seq <= m_seq; r <= R_CALC1;
                    end else begin
                        if (done_hit) cnt_dup <= cnt_dup + 1'b1;
                        else begin cnt_ooo <= cnt_ooo + 1'b1; last_error <= E_MISMATCH; end
                        r <= R_DONE;
                    end
                end
                R_CALC1: begin
                    d1 <= $signed({2'b00, t2}) - $signed({2'b00, t1});
                    d2 <= oneway ? $signed({34'd0, oneway_delay_us}) : $signed({2'b00, t4}) - $signed({2'b00, t3});
                    // 频率积分移位量：offset/2^log_period，增益 1/4，单位 2^-32
                    sh_right <= log_period >= 8'd30;
                    sh_amt <= (log_period >= 8'd30) ? ((log_period > 8'd93) ? 6'd63 : log_period[5:0] - 6'd30)
                                                    : 6'd30 - log_period[5:0];
                    r <= R_CALC2;
                end
                R_CALC2: begin
                    // 单向：offset = (t2-t1) - delay，统一写成 ((d1-d2) + (d1-d2))/2
                    off2 <= oneway ? (d1 - d2) <<< 1 : d1 - d2;
                    dly2 <= oneway ? d2 <<< 1 : d1 + d2;
                    r <= R_CALC3;
                end
                R_CALC3: begin
                    offset <= off2[65:1];
                    neg_off <= -off2[64:1];
                    abs_off <= off2[65] ? -off2[64:1] : off2[64:1];
                    path_delay <= dly2[65] ? 32'd0 : (|dly2[64:33] ? 32'hffffffff : dly2[32:1]);
                    r <= R_CALC4;
                end
                R_CALC4: begin
                    big <= |abs_off[63:32] || abs_off[31:0] > step_limit;
                    near <= !(|abs_off[63:32]) && abs_off[31:0] <= LOCK_US;
                    newmax <= |abs_off[63:32] || abs_off[31:0] > max_abs_offset;
                    abs_sat <= |abs_off[63:32];
                    sat_pos <= !offset[64] && |offset[63:31];
                    sat_neg <= offset[64] && !(&offset[63:31]);
                    // 频率积分移位分两拍：先按 8 位整倍数，再按余数
                    shv1 <= sh_right ? $signed(neg_off) >>> {sh_amt[5:3], 3'b000} : neg_off << {sh_amt[5:3], 3'b000};
                    r <= R_CALC5;
                end
                R_CALC5: begin
                    shv <= sh_right ? $signed(shv1) >>> sh_amt[2:0] : shv1 << sh_amt[2:0];
                    r <= R_SERVO;
                end
                R_SERVO: begin
                    cnt_sync <= cnt_sync + 1'b1;
                    last_sync_mono <= mono_us;
                    hold_left <= holdover_us; hold_zero <= 1'b0;
                    uns_left <= unsync_us; uns_zero <= 1'b0;
                    last_offset <= sat_pos ? 32'sh7fffffff : sat_neg ? 32'sh80000000 : offset[31:0];
                    if (newmax) max_abs_offset <= abs_sat ? 32'hffffffff : abs_off[31:0];
                    if (state == S_UNSYNC || state == S_FAULT || big) begin
                        step <= 1'b1; step_delta <= neg_off;
                        cnt_step <= cnt_step + 1'b1; lock_cnt <= 4'd0;
                        if (state == S_LOCK) ev_fault <= 1'b1;
                        state <= oneway ? S_DEGR : S_ACQ;
                        r <= R_DONE;
                    end else begin
                        slew_load <= 1'b1; slew_value <= neg_off[31:0];
                        cnt_slew <= cnt_slew + 1'b1;
                        freq_step <= shv;
                        if (near) begin
                            if (lock_cnt >= LOCK_COUNT - 1) state <= oneway ? S_DEGR : S_LOCK;
                            else begin
                                lock_cnt <= lock_cnt + 1'b1;
                                if (state != S_LOCK) state <= oneway ? S_DEGR : S_ACQ;
                            end
                        end else begin
                            lock_cnt <= 4'd0;
                            if (state == S_LOCK) ev_fault <= 1'b1;
                            state <= oneway ? S_DEGR : S_ACQ;
                        end
                        r <= oneway ? R_DONE : R_FREQ;
                    end
                end
                R_FREQ: begin
                    freq_sum <= {{32{freq_corr[31]}}, freq_corr} + freq_step;
                    r <= R_FREQ2;
                end
                R_FREQ2: begin
                    f_hi <= freq_sum > freq_lim;
                    f_lo <= freq_sum < -freq_lim;
                    r <= R_FREQ3;
                end
                R_FREQ3: begin
                    if (f_hi) freq_corr <= FREQ_LIMIT;
                    else if (f_lo) freq_corr <= -FREQ_LIMIT;
                    else freq_corr <= freq_sum[31:0];
                    r <= R_DONE;
                end
                R_DONE: begin done <= 1'b1; r <= R_IDLE; end
                default: r <= R_IDLE;
            endcase
        end
    end
endmodule
