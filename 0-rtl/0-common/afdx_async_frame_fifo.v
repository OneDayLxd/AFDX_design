/*
 * ================================================================
 * File Name    : afdx_async_frame_fifo.v
 * Module Name  : afdx_async_frame_fifo
 *
 * Function:
 *   双时钟整帧缓存；仅通过校验的完整帧可见，满或超长时丢弃整帧。
 *   帧指针以 Gray 码跨域；数据和描述符在读侧释放该槽前保持稳定。
 *   两侧复位必须同时断言，分别在各自时钟域同步释放。
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_async_frame_fifo #(
    parameter SLOT_LOG = 2,
    parameter BYTE_LOG = 11,
    parameter MIN_LEN  = 60,
    parameter MAX_LEN  = 1514,
    parameter DROP_FULL = 0
) (
    input  wire        wr_clk,
    input  wire        wr_rst,
    input  wire [7:0]  in_data,
    input  wire        in_valid,
    input  wire        in_last,
    input  wire        in_error,
    input  wire [31:0] in_ts,
    output wire        in_ready,
    output reg         stat_good,
    output reg         stat_error,
    output reg         stat_length,
    output reg         stat_overflow,
    input  wire        rd_clk,
    input  wire        rd_rst,
    output reg  [7:0]  out_data,
    output reg         out_valid,
    output reg         out_last,
    output reg  [31:0] out_ts,
    input  wire        out_ready
);
    localparam SLOTS = 1 << SLOT_LOG;
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] mem [0:(1 << (SLOT_LOG+BYTE_LOG))-1];
    // 少量描述符采用寄存器，载荷阵列保留独立的 RAM 推断边界。
    (* mem2reg *) reg [BYTE_LOG:0] lengths [0:SLOTS-1];
    (* mem2reg *) reg [31:0] timestamps [0:SLOTS-1];
    reg [SLOT_LOG:0] wp, wg, rp, rg;
    (* async_reg = "true" *) reg [SLOT_LOG:0] rg1, rg2, wg1, wg2;
    wire [SLOT_LOG:0] wp_next = wp + 1'b1;
    wire [SLOT_LOG:0] rp_next = rp + 1'b1;
    wire [SLOT_LOG:0] full_gray = rg2 ^ ((1 << SLOT_LOG) | (1 << (SLOT_LOG-1)));
    wire full = wg == full_gray;
    wire empty = rg == wg2;
    reg [BYTE_LOG:0] count;
    reg dropping, bad, too_long;
    reg [31:0] frame_ts;
    wire over_now = full || dropping;
    wire long_now = too_long || count >= MAX_LEN;
    wire bad_now = bad || in_error;
    assign in_ready = DROP_FULL || count != 0 || dropping || !full;

    always @(posedge wr_clk) begin
        if (wr_rst) begin rg1 <= 0; rg2 <= 0; end
        else begin rg1 <= rg; rg2 <= rg1; end
    end
    always @(posedge rd_clk) begin
        if (rd_rst) begin wg1 <= 0; wg2 <= 0; end
        else begin wg1 <= wg; wg2 <= wg1; end
    end
    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wp <= 0; wg <= 0; count <= 0;
            dropping <= 0; bad <= 0; too_long <= 0; frame_ts <= 0;
            stat_good <= 0; stat_error <= 0; stat_length <= 0; stat_overflow <= 0;
        end else begin
            stat_good <= 0; stat_error <= 0; stat_length <= 0; stat_overflow <= 0;
            if (in_valid && in_ready) begin
                if (count == 0) frame_ts <= in_ts;
                if (!over_now && !long_now)
                    mem[{wp[SLOT_LOG-1:0], count[BYTE_LOG-1:0]}] <= in_data;
                dropping <= over_now;
                bad <= bad_now;
                too_long <= long_now;
                // Saturate until tlast, including malformed overlong streams.
                if (count < MAX_LEN+1) count <= count + 1'b1;
                if (in_last) begin
                    if (!over_now && !long_now && !bad_now && count+1 >= MIN_LEN) begin
                        lengths[wp[SLOT_LOG-1:0]] <= count + 1'b1;
                        timestamps[wp[SLOT_LOG-1:0]] <= count == 0 ? in_ts : frame_ts;
                        wp <= wp_next;
                        wg <= (wp_next >> 1) ^ wp_next;
                        stat_good <= 1;
                    end else begin
                        stat_overflow <= over_now;
                        stat_length <= !over_now && (long_now || count+1 < MIN_LEN);
                        stat_error <= !over_now && !long_now && count+1 >= MIN_LEN && bad_now;
                    end
                    count <= 0; dropping <= 0; bad <= 0; too_long <= 0;
                end
            end
        end
    end
    reg [BYTE_LOG-1:0] read_index;
    reg last_loaded;
    wire read_load = !rd_rst && (!out_valid || out_ready) && !empty && !last_loaded;
    // 独立同步读口不复位，便于 RAM 推断；无效周期忽略 out_data。
    always @(posedge rd_clk)
        if (read_load) out_data <= mem[{rp[SLOT_LOG-1:0], read_index}];
    // One synchronous RAM read per accepted beat; payload has no bubbles.
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rp <= 0; rg <= 0; read_index <= 0; last_loaded <= 0;
            out_valid <= 0; out_last <= 0; out_ts <= 0;
        end else begin
            if (!out_valid || out_ready) begin
                out_valid <= !empty && !last_loaded;
                if (!empty && !last_loaded) begin
                    out_ts <= timestamps[rp[SLOT_LOG-1:0]];
                    out_last <= ({1'b0, read_index}+1 == lengths[rp[SLOT_LOG-1:0]]);
                    last_loaded <= ({1'b0, read_index}+1 == lengths[rp[SLOT_LOG-1:0]]);
                    read_index <= read_index + 1'b1;
                end
            end
            if (out_valid && out_ready && out_last) begin
                rp <= rp_next;
                rg <= (rp_next >> 1) ^ rp_next;
                read_index <= 0; last_loaded <= 0;
            end
        end
    end
endmodule
