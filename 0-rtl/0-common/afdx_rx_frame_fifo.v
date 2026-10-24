/*
 * ================================================================
 * File Name    : afdx_rx_frame_fifo.v
 * Module Name  : afdx_rx_frame_fifo
 *
 * Function:
 *   单时钟、按字节紧凑存储的 RX 整帧缓存，短帧不独占 2KB 槽。
 *   校验成功才发布长度/时间戳；错误、超长、空间不足均回滚整帧。
 *   载荷和描述符使用同步读 RAM，不复位数据阵列，支持输出背压。
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_rx_frame_fifo #(
    parameter BYTE_LOG = 17,
    parameter SLOT_LOG = 11,
    parameter MIN_LEN  = 60,
    parameter MAX_LEN  = 1514
) (
    input  wire        clk,
    input  wire        reset,
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
    output reg  [7:0]  out_data,
    output reg         out_valid,
    output reg         out_last,
    output reg  [31:0] out_ts,
    input  wire        out_ready
);
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] data_mem [0:(1<<BYTE_LOG)-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [43:0] desc_mem [0:(1<<SLOT_LOG)-1];
    reg [BYTE_LOG:0] byte_wp, byte_rp, frame_start;
    reg [SLOT_LOG:0] wp, rp;
    wire [BYTE_LOG:0] used_bytes = byte_wp-byte_rp;
    wire byte_full = byte_wp[BYTE_LOG]!=byte_rp[BYTE_LOG] &&
                     byte_wp[BYTE_LOG-1:0]==byte_rp[BYTE_LOG-1:0];
    wire [SLOT_LOG:0] used_frames = wp-rp;
    wire desc_full = wp[SLOT_LOG]!=rp[SLOT_LOG] && wp[SLOT_LOG-1:0]==rp[SLOT_LOG-1:0];
    wire full = byte_full || desc_full;
    reg [11:0] count;
    wire [11:0] next_count = count+12'd1;
    reg dropping, bad, too_long;
    reg [31:0] frame_ts;
    wire over_now = dropping || full;
    wire long_now = too_long || count >= MAX_LEN;
    wire bad_now = bad || in_error;
    wire [BYTE_LOG:0] rollback = count == 0 ? byte_wp : frame_start;
    assign in_ready = 1'b1; // GMII 不可背压；无空间时排空并丢弃整帧。

    always @(posedge clk) begin
        if (reset) begin
            byte_wp<=0; wp<=0; frame_start<=0; count<=0;
            dropping<=0; bad<=0; too_long<=0; frame_ts<=0;
            stat_good<=0; stat_error<=0; stat_length<=0; stat_overflow<=0;
        end else begin
            stat_good<=0; stat_error<=0; stat_length<=0; stat_overflow<=0;
            if (in_valid) begin
                if (count==0) begin frame_start<=byte_wp; frame_ts<=in_ts; end
                if (!over_now && !long_now) begin
                    data_mem[byte_wp[BYTE_LOG-1:0]]<=in_data;
                    byte_wp<=byte_wp+1'b1;
                end
                dropping<=over_now; bad<=bad_now; too_long<=long_now;
                if (count<MAX_LEN+1) count<=count+1'b1;
                if (in_last) begin
                    if (!over_now && !long_now && !bad_now && count+1>=MIN_LEN) begin
                        desc_mem[wp[SLOT_LOG-1:0]]<={count==0 ? in_ts : frame_ts,next_count};
                        wp<=wp+1'b1; stat_good<=1;
                    end else begin
                        byte_wp<=rollback;
                        stat_overflow<=over_now;
                        stat_length<=!over_now && (long_now || count+1<MIN_LEN);
                        stat_error<=!over_now && !long_now && count+1>=MIN_LEN && bad_now;
                    end
                    count<=0; dropping<=0; bad<=0; too_long<=0;
                end
            end
        end
    end

    reg active;
    reg [11:0] remaining;
    reg [BYTE_LOG-1:0] read_addr;
    wire take = out_valid && out_ready;
    wire finish = take && out_last;
    wire stage_ready = !out_valid || out_ready;
    // 帧尾握手时预取下一项，连续帧只需一个空拍；同拍提交的新项延后一拍可见。
    wire [SLOT_LOG:0] desc_addr = finish ? rp+1'b1 : rp;
    reg [SLOT_LOG:0] desc_addr_q;
    reg desc_valid;
    reg [43:0] desc_q;
    wire head_ready = desc_valid && desc_addr_q==rp;
    wire read_load = !reset && stage_ready && (active ? remaining!=0 : head_ready);
    always @(posedge clk) begin
        desc_q<=desc_mem[desc_addr[SLOT_LOG-1:0]];
        desc_addr_q<=desc_addr;
        desc_valid<=!reset && wp!=desc_addr;
        if (read_load) out_data<=data_mem[read_addr];
    end
    always @(posedge clk) begin
        if (reset) begin
            byte_rp<=0; rp<=0; read_addr<=0; active<=0; remaining<=0;
            out_valid<=0; out_last<=0; out_ts<=0;
        end else begin
            if (stage_ready) out_valid<=read_load;
            if (read_load) begin
                read_addr<=read_addr+1'b1;
                if (!active) begin
                    active<=1;
                    remaining<=desc_q[11:0]-1'b1;
                    out_last<=desc_q[11:0]==1;
                    out_ts<=desc_q[43:12];
                end else begin
                    remaining<=remaining-1'b1;
                    out_last<=remaining==1;
                end
            end
            if (take) byte_rp<=byte_rp+1'b1;
            if (finish) begin rp<=rp+1'b1; active<=0; end
        end
    end
endmodule
