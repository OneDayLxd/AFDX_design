/*
 * ================================================================
 * File Name    : afdx_mac_rx.v
 * Module Name  : afdx_mac_rx
 *
 * Function:
 *   MAC 接收模块：8 位 MAC 总线输入 -> 帧 FIFO（存储转发）。
 *   总线协议：rx_frame 为帧包络（对应顶层 rx_ready），rx_valid 为字节有效；帧结束 = 包络拉低，
 *     若 MAC 不提供包络（常高），则以 rx_valid 空闲超过 GAP_CYCLES 周期作为帧结束。
 *   接收过程中计算 CRC-32，帧结束时校验 FCS 余数、帧长（64..1518），
 *   校验失败或 FIFO 溢出则回退写指针丢弃整帧。输出流去掉 4 字节 FCS，最后一字节带 out_last。
 *
 * Sub Module:
 *   afdx_crc32
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_defs.vh"
module afdx_mac_rx #(
    parameter FIFO_AW    = 12,
    parameter GAP_CYCLES = 32                  // 无包络时的帧结束空闲判定周期数
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] rx_data,
    input  wire       rx_valid,
    input  wire       rx_frame,
    output wire [7:0] out_data,
    output reg        out_valid,
    output wire       out_last,
    input  wire       out_ready,
    output wire       frame_avail,             // FIFO 中存在完整帧
    output reg        stat_frame_ok,
    output reg        stat_fcs_err,
    output reg        stat_len_err,
    output reg        stat_ovf
);
    `include "afdx_math.vh"
    localparam DEPTH = (1 << FIFO_AW);
    // Only committed bytes are read; full detection prevents overwriting them.
    (* ramstyle = "no_rw_check" *) reg [8:0] mem [0:DEPTH-1];
    reg [FIFO_AW:0]   wr_ptr, wr_commit, rd_ptr;
    reg [FIFO_AW-1:0] frame_cnt;
    // 只读取已提交字节，不能越过 wr_commit 进入尚未校验的帧。
    wire empty = (wr_commit == rd_ptr);
    // 满标志寄存：上一拍已用 ≥ DEPTH-1 即视为满。每拍至多写入 1 字节，真正满（已用 = DEPTH）时
    // 上一拍必然已用 ≥ DEPTH-1，故只会提前 1 字节判满，不会覆盖未读数据；减法与比较不进入写口路径。
    wire [FIFO_AW:0] used_bytes = wr_ptr - rd_ptr;
    reg full;
    always @(posedge clk)
        full <= !rst && (used_bytes[FIFO_AW] || &used_bytes[FIFO_AW-1:0]);
    assign frame_avail = (frame_cnt != 0);

    // ---------------- 接收写入 ----------------
    reg         in_frame;
    reg [13:0]  byte_cnt;                      // 已接收字节数（含 FCS）
    reg         frame_len_good, payload_delay_full;
    reg [7:0]   dl0, dl1, dl2, dl3, dl4;       // 5 字节延迟线，dl4 最旧
    reg [31:0]  crc;
    reg         ovf_flag, long_flag;
    wire [31:0] crc_next;
    afdx_crc32 u_crc (
        .crc_in  (crc),
        .data    (rx_data),
        .crc_out (crc_next)
    );

    localparam GAP_W = afdx_clog2(GAP_CYCLES + 1);
    reg [GAP_W-1:0] gap_cnt;                   // rx_valid 空闲周期计数
    reg gap_near;                              // 寄存的 gap_cnt >= GAP_CYCLES - 1
    wire eof = in_frame & ~rx_valid & (~rx_frame | gap_near);
    wire frame_good = frame_len_good && (crc == `AFDX_CRC_RESIDUE) &&
                      !ovf_flag && !long_flag;
    wire wr_data_en = rx_valid & payload_delay_full & ~ovf_flag & ~long_flag;
    wire wr_last_en = eof & frame_good;
    wire inc = wr_last_en && !full;
    wire mem_write = (wr_data_en || wr_last_en) && !full;

    // 写口寄存一拍：RAM 写使能/地址/数据直接来自触发器（存储器跨多个 RAM 块，写使能布线较长）。
    // 延后一拍写入不影响读出：读侧只读到 wr_commit 之前，而本帧提交时其余字节早已写入，
    // 帧尾标记字节在提交后一拍写入时读指针至多位于本帧首字节。
    reg mem_we_q;
    reg [FIFO_AW-1:0] mem_wa_q;
    reg [8:0] mem_wd_q;
    always @(posedge clk) begin
        mem_we_q <= !rst && mem_write;
        mem_wa_q <= wr_ptr[FIFO_AW-1:0];
        mem_wd_q <= {wr_last_en, dl4};
    end
    always @(posedge clk) begin
        if (mem_we_q)
            mem[mem_wa_q] <= mem_wd_q;
    end

    always @(posedge clk) begin
        if (rst) begin
            in_frame  <= 1'b0;
            byte_cnt  <= 14'd0;
            frame_len_good <= 1'b0;
            payload_delay_full <= 1'b0;
            crc       <= `AFDX_CRC_INIT;
            wr_ptr    <= {(FIFO_AW+1){1'b0}};
            wr_commit <= {(FIFO_AW+1){1'b0}};
            ovf_flag  <= 1'b0;
            long_flag <= 1'b0;
            stat_frame_ok <= 1'b0;
            stat_fcs_err  <= 1'b0;
            stat_len_err  <= 1'b0;
            stat_ovf      <= 1'b0;
            {dl0, dl1, dl2, dl3, dl4} <= 40'd0;
            gap_cnt <= 0;
            gap_near <= (GAP_CYCLES < 2);
        end else begin
            if (rx_valid) gap_cnt <= 0;
            else if (gap_cnt < GAP_CYCLES) gap_cnt <= gap_cnt + 1'b1;
            gap_near <= (GAP_CYCLES < 2) || (!rx_valid && gap_cnt >= GAP_CYCLES - 2);
            stat_frame_ok <= 1'b0;
            stat_fcs_err  <= 1'b0;
            stat_len_err  <= 1'b0;
            stat_ovf      <= 1'b0;
            if (rx_valid) begin
                in_frame <= 1'b1;
                crc      <= crc_next;
                {dl4, dl3, dl2, dl1, dl0} <= {dl3, dl2, dl1, dl0, rx_data};
                if (byte_cnt != 14'h3FFF) byte_cnt <= byte_cnt + 1'b1;
                // Qualify the count that will be visible on the following
                // clock, before the EOF decision drives the RAM write port.
                frame_len_good <= (byte_cnt >= `AFDX_MIN_FRAME - 1) &&
                                  (byte_cnt < `AFDX_MAX_FRAME);
                if (byte_cnt == 14'd4) payload_delay_full <= 1'b1;
                if (byte_cnt > `AFDX_MAX_FRAME - 1) long_flag <= 1'b1;
                if (wr_data_en) begin
                    if (full) ovf_flag <= 1'b1;
                    else      wr_ptr   <= wr_ptr + 1'b1;
                end
            end else if (eof) begin
                in_frame  <= 1'b0;
                byte_cnt  <= 14'd0;
                frame_len_good <= 1'b0;
                payload_delay_full <= 1'b0;
                crc       <= `AFDX_CRC_INIT;
                ovf_flag  <= 1'b0;
                long_flag <= 1'b0;
                if (frame_good && !full) begin
                    wr_ptr        <= wr_ptr + 1'b1;
                    wr_commit     <= wr_ptr + 1'b1;
                    stat_frame_ok <= 1'b1;
                end else begin
                    wr_ptr <= wr_commit;          // 回退，丢弃整帧
                    if (ovf_flag || full)       stat_ovf     <= 1'b1;
                    else if (long_flag || byte_cnt < `AFDX_MIN_FRAME) stat_len_err <= 1'b1;
                    else                        stat_fcs_err <= 1'b1;
                end
            end
        end
    end

    // ---------------- 读出 ----------------
    // Separate the RAM read stage from the A/B arbiter with a local elastic
    // output stage. Both stages hold under backpressure; one byte/clock is
    // sustained when ready. frame_cnt retires only on the external last beat.
    wire stage_ready = !out_valid || out_ready;
    reg ram_valid;
    wire out_take = !ram_valid || stage_ready;
    wire dec = out_valid && out_ready && out_last;
    reg [8:0] mem_data_q;
    (* preserve *) reg [8:0] output_q;
    assign out_data = output_q[7:0];
    assign out_last = out_valid && output_q[8];
    always @(posedge clk)
        if (out_take && !empty) mem_data_q <= mem[rd_ptr[FIFO_AW-1:0]];
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else if (stage_ready) out_valid <= ram_valid;
        if (stage_ready && ram_valid) output_q <= mem_data_q;
    end

    always @(posedge clk) begin
        if (rst) begin
            rd_ptr    <= {(FIFO_AW+1){1'b0}};
            frame_cnt <= {FIFO_AW{1'b0}};
            ram_valid <= 1'b0;
        end else begin
            frame_cnt <= frame_cnt + {{(FIFO_AW-1){1'b0}}, inc} - {{(FIFO_AW-1){1'b0}}, dec};
            if (out_take) begin
                ram_valid <= !empty;
                if (!empty) rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end
endmodule
