/*
 * ================================================================
 * File Name    : afdx_app_gen.v
 * Module Name  : afdx_app_gen
 *
 * Function:
 *   内部应用消息发生器：为每个发送 VL 按配置周期和相位产生测试消息（模拟航电分区应用）。
 *   消息格式：[0]=VL 索引 [1..2]=消息序号 [3]=长度低 8 位 [4..]=递增测试图样。
 *   释放时刻为理想网格 phase + k×period（微秒，k ≥ 0，从复位起算）：逐 VL 在同步 RAM 中保存
 *   上一个理想释放时刻 start，触发后按 start += period 推进，不以实际触发时刻重新定基准，
 *   因此扫描延迟不会逐周期累积成相位漂移；落后超过一个周期（仅在暂停后）时改以当前时刻为基准。
 *   扫描为 4 级流水、每拍一个 VL（128 VL 一轮 1.28 us）：读 start/周期 → 模减 → 比较 → 触发。
 *   只有已检出但尚未开始发送的 VL（pending）会暂停扫描；发送期间扫描继续。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_app_gen #(
    parameter N_VL = 8,
    parameter VLW  = 3
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            enable,
    input  wire            tick_us,
    output wire [VLW-1:0]  scan_idx,          // 周期扫描 ROM 索引
    input  wire [19:0]     rom_period_us,     // scan_idx 对应 VL 的消息周期（0：不发送）
    input  wire [19:0]     rom_phase_us,      // scan_idx 对应 VL 的释放相位（< 周期）
    output wire [VLW-1:0]  len_idx,           // 当前发送 VL 的 ROM 索引
    input  wire [10:0]     rom_len,           // len_idx 对应 VL 的消息长度
    output reg  [VLW-1:0]  msg_vl,
    output reg             msg_valid,
    output reg  [7:0]      msg_data,
    output reg             msg_last,
    input  wire            msg_ready,
    output reg  [31:0]     cnt_msgs
);
    (* ramstyle = "M4K, no_rw_check" *) reg [19:0] start_us [0:N_VL-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [15:0] seq      [0:N_VL-1];
    reg [19:0] us_count;                      // 全局微秒计数（自由回绕，模 2^20）
    reg [19:0] start_q;
    reg [15:0] seq_q;
    // 复位后逐项写入 start = phase - period，使首个释放时刻为 phase。
    (* preserve *) reg [VLW-1:0] init_addr;     // 与队列的复位清扫同构，避免被合并
    (* preserve *) reg init_run;
    always @(posedge clk) begin
        if (rst) begin init_run <= 1'b1; init_addr <= {VLW{1'b0}}; end
        else if (init_run) begin
            init_addr <= init_addr + 1'b1;
            if (init_addr == N_VL-1) init_run <= 1'b0;
        end
    end
    reg pending;
    reg [VLW-1:0] pending_vl;
    reg [VLW-1:0] scan;
    reg [10:0] byte_idx, len_l, len_m2;
    reg [7:0] pat;                            // 下一个图样字节 = byte_idx + 1 + 序号低字节（随字节递增）
    reg        sending;
    reg seq_fetch;
    reg [15:0] sequence_l;
    reg [VLW-1:0] seq_wr_idx;
    reg [15:0] seq_wr_val;
    reg seq_wr;
    // 扫描流水
    reg s1_v, s2_v, s3_v, s3_due, s3_late;
    reg [VLW-1:0] s1_vl, s2_vl, s3_vl;
    reg [19:0] s1_period, s2_period, s2_start, s2_diff, s3_next;

    assign scan_idx = init_run ? init_addr : scan;
    assign len_idx  = msg_vl;

    wire advance = !init_run && !pending;
    wire fire = advance && s3_v && s3_due;
    // 暂停时重读第 1 级 VL，使 RAM 输出始终与第 1 级对齐。
    wire [VLW-1:0]  start_ra = advance ? scan : s1_vl;
    wire            start_we = init_run || fire;
    wire [VLW-1:0]  start_wa = init_run ? init_addr : s3_vl;
    wire [19:0]     start_wd = init_run ? rom_phase_us - rom_period_us : (s3_late ? us_count : s3_next);
    wire            seq_we   = init_run || seq_wr;
    wire [VLW-1:0]  seq_wa   = init_run ? init_addr : seq_wr_idx;
    wire [15:0]     seq_wd   = init_run ? 16'd0 : seq_wr_val;
    always @(posedge clk) begin
        start_q <= start_us[start_ra];
        if (start_we) start_us[start_wa] <= start_wd;
    end
    always @(posedge clk) begin
        seq_q <= seq[msg_vl];
        if (seq_we) seq[seq_wa] <= seq_wd;
    end

    always @(posedge clk) begin
        if (rst) begin
            s1_v <= 1'b0; s2_v <= 1'b0; s3_v <= 1'b0; s3_due <= 1'b0; s3_late <= 1'b0;
            pending <= 1'b0; pending_vl <= {VLW{1'b0}}; scan <= {VLW{1'b0}}; sending <= 1'b0;
            msg_vl <= {VLW{1'b0}}; msg_valid <= 1'b0; msg_data <= 8'h00; msg_last <= 1'b0;
            byte_idx <= 11'd0; len_l <= 11'd0; cnt_msgs <= 32'd0; sequence_l <= 16'd0;
            us_count <= 20'd0; seq_wr <= 1'b0; seq_wr_idx <= {VLW{1'b0}}; seq_wr_val <= 16'd0;
            seq_fetch <= 1'b0;
        end else begin
            seq_wr <= 1'b0;
            if (tick_us && enable) us_count <= us_count + 1'b1;
            if (advance) begin
                s1_v <= 1'b1; s1_vl <= scan; s1_period <= rom_period_us;
                scan <= scan == N_VL-1 ? {VLW{1'b0}} : scan + 1'b1;
                s2_v <= s1_v; s2_vl <= s1_vl; s2_period <= s1_period;
                s2_start <= start_q; s2_diff <= us_count - start_q;
                s3_v <= s2_v; s3_vl <= s2_vl; s3_next <= s2_start + s2_period;
                s3_due <= enable && s2_period != 20'd0 && s2_diff >= s2_period;
                s3_late <= {1'b0, s2_diff} >= {s2_period, 1'b0};
                if (fire) begin
                    // 已触发 VL 的新 start 刚写入；清空流水，避免 VL 数很少时用旧值再次触发。
                    pending <= 1'b1; pending_vl <= s3_vl;
                    s1_v <= 1'b0; s2_v <= 1'b0; s3_v <= 1'b0;
                end
            end
            if (!sending) begin
                if (pending && !init_run) begin
                    msg_vl   <= pending_vl;
                    sending  <= 1'b1;
                    pending  <= 1'b0;
                    seq_fetch <= 1'b0;
                    byte_idx <= 11'd0;
                    msg_valid <= 1'b0;
                end
            end else if (!msg_valid) begin
                // 等一拍让 seq_q 对应新的 msg_vl。
                seq_fetch <= 1'b1;
                if (seq_fetch) begin
                    len_l     <= rom_len;
                    len_m2    <= rom_len - 11'd2;
                    sequence_l <= seq_q;
                    pat       <= seq_q[7:0] + 8'd1;
                    msg_data  <= 8'h00 | msg_vl;
                    msg_last  <= (rom_len == 11'd1);
                    msg_valid <= 1'b1;
                end
            end else if (msg_ready) begin
                if (msg_last) begin
                    msg_valid <= 1'b0;
                    sending   <= 1'b0;
                    seq_wr <= 1'b1; seq_wr_idx <= msg_vl; seq_wr_val <= sequence_l + 1'b1;
                    cnt_msgs  <= cnt_msgs + 1'b1;
                end else begin
                    byte_idx <= byte_idx + 1'b1;
                    pat <= pat + 8'd1;
                    case (byte_idx)
                        11'd0: msg_data <= sequence_l[15:8];
                        11'd1: msg_data <= sequence_l[7:0];
                        11'd2: msg_data <= len_l[7:0];
                        default: msg_data <= pat;
                    endcase
                    msg_last <= byte_idx == len_m2;
                end
            end
        end
    end
endmodule
