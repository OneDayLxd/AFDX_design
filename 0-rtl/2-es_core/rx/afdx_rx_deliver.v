/*
 * ================================================================
 * File Name    : afdx_rx_deliver.v
 * Module Name  : afdx_rx_deliver
 *
 * Function:
 *   接收交付模块：轮询各接收 VL 队列，把队首消息以字节流形式交付给应用（主机）接口。
 *
 * Sub Module:
 *   afdx_rr_arbiter
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_rx_deliver #(
    parameter N_VL    = 8,
    parameter VLW     = 3,
    parameter SLOT_AW = 11,
    parameter MW      = 9,
    // 1: 非空位来自队列的分组 RAM（ne_group -> ne_bits，1 拍读延迟），
    //    不再对 N_VL 位向量做 N_VL/16:1 的 16 位宽选择。
    parameter NONEMPTY_RAM = 0,
    parameter REGISTER_RESET = 0
) (
    input  wire               clk,
    input  wire               rst,
    input  wire [N_VL-1:0]    nonempty,
    output wire [(VLW>4 ? VLW-4 : 1)-1:0] ne_group,
    input  wire [15:0]        ne_bits,
    input  wire               q_init_busy,
    output reg  [VLW-1:0]     q_rd_vl,
    output wire               q_rd_start,
    output wire               q_rd_pop,
    input  wire               q_rd_info_valid,
    input  wire               q_rd_pop_ready,
    input  wire [SLOT_AW-1:0] q_rd_len,
    input  wire [MW-1:0]      q_rd_meta,
    input  wire [7:0]         q_rd_data,
    input  wire               q_rd_valid,
    input  wire               q_rd_last,
    output wire               q_rd_ready,
    output wire [VLW-1:0]     msg_vl,
    output wire               msg_valid,
    output wire [7:0]         msg_data,
    output wire               msg_last,
    input  wire               msg_ready,
    output reg  [SLOT_AW-1:0] msg_len,
    output reg  [MW-1:0]      msg_meta,
    output reg                stat_deliver
);
    `include "afdx_math.vh"
    localparam S_IDLE = 3'd0, S_SETUP = 3'd1, S_STREAM = 3'd2,
               S_POP = 3'd3, S_RELEASE = 3'd4, S_SCAN_GROUP = 3'd5, S_DRAIN = 3'd6;
    localparam GROUPS = (N_VL + 15) / 16;
    localparam GW = afdx_clog2(GROUPS);
    localparam [GW-1:0] LAST_GROUP = GROUPS[GW-1:0] - {{(GW-1){1'b0}},1'b1};
    // In the core, replace the parent reset stage with same-depth local
    // copies. Standalone users retain the original direct-reset interface.
    wire rst_ctrl, rst_rr;
    generate if (REGISTER_RESET != 0) begin : g_local_reset
        (* preserve, maxfan = 32 *) reg ctrl_reset;
        (* preserve, maxfan = 16 *) reg rr_reset;
        always @(posedge clk) begin
            ctrl_reset <= rst;
            rr_reset <= rst;
        end
        assign rst_ctrl = ctrl_reset;
        assign rst_rr = rr_reset;
    end else begin : g_direct_reset
        assign rst_ctrl = rst;
        assign rst_rr = rst;
    end endgenerate
    reg [2:0] state;
    // Queue may invalidate rd_info_valid on the following edge when a commit
    // changes this VL's head. Start the read on the edge that accepts it.
    assign q_rd_start = (state == S_SETUP) && q_rd_info_valid;
    assign q_rd_pop = (state == S_RELEASE) && q_rd_pop_ready;
    reg [VLW-1:0] rr;
    reg [GW-1:0] scan_group;
    reg scan_wrapped;
    reg scan_request_valid, scan_request_done;
    reg [15:0] scan_requests_q;
    reg [GW-1:0] scan_request_group;
    reg scan_result_valid, scan_result_found, scan_result_done;
    reg [3:0] scan_result_pick;
    reg [GW-1:0] scan_result_group;
    reg [15:0] scan_mask;
    reg scan_at_rr, scan_at_last;
    wire [GW-1:0] rr_group;
    wire [3:0] rr_lane;
    generate if (VLW > 4) begin : g_wide_rr
        assign rr_group = rr[VLW-1:4];
        assign rr_lane = rr[3:0];
    end else begin : g_narrow_rr
        assign rr_group = {GW{1'b0}};
        if (VLW == 4) begin : g_four_lane_bits
            assign rr_lane = rr;
        end else begin : g_short_lane_bits
            assign rr_lane = {{(4-VLW){1'b0}},rr};
        end
    end endgenerate
    wire [15:0] group_bits;
    // Prefetch the next group each clock. Its RAM bits, local pick, and
    // controller decision occupy separate registered stages.
    // 本模块出队后的清零最迟在 S_DRAIN 结束时写入，S_IDLE 起的读取不会看到过期的置位。
    wire [GW-1:0] next_scan_group =
        (state == S_IDLE) ? rr_group :
        (state == S_SCAN_GROUP) ?
            ((scan_at_last) ? {GW{1'b0}} : scan_group + 1'b1) :
        scan_group;
    generate if (NONEMPTY_RAM != 0) begin : g_ne_ram
        assign group_bits = ne_bits;
        assign ne_group = next_scan_group;
    end else begin : g_ne_vec
        wire [GROUPS*16-1:0] padded_nonempty;
        genvar v;
        for (v=0; v<GROUPS*16; v=v+1) begin : g_pad_nonempty
            if (v < N_VL) begin : g_valid_vl
                assign padded_nonempty[v] = nonempty[v];
            end else begin : g_unused_vl
                assign padded_nonempty[v] = 1'b0;
            end
        end
        assign group_bits = padded_nonempty[scan_group*16 +: 16];
        assign ne_group = {(VLW>4 ? VLW-4 : 1){1'b0}};
    end endgenerate
    wire [15:0] upper_mask = 16'hffff << rr_lane;
    wire [15:0] lower_mask = ~upper_mask;
    // The lane mask and group comparisons are registered from the same
    // speculative next group as the nonempty read, so they align with it.
    wire next_wrapped = (state == S_IDLE) ? 1'b0 :
        (state == S_SCAN_GROUP && scan_at_last) ? 1'b1 : scan_wrapped;
    always @(posedge clk) begin
        scan_at_rr <= next_scan_group == rr_group;
        scan_at_last <= next_scan_group == LAST_GROUP;
        scan_mask <= (next_scan_group == rr_group) ? (next_wrapped ? lower_mask : upper_mask) : 16'hffff;
    end
    wire [15:0] requests = group_bits & scan_mask;
    wire found;
    wire [3:0] pick;
    afdx_rr_arbiter #(
        .N (16),
        .W (4)
    ) u_rr (
        .request     (scan_requests_q),
        .start_index (4'd0),
        .valid       (found),
        .index       (pick)
    );
    always @(posedge clk) begin
        if (state == S_SCAN_GROUP) begin
            scan_requests_q <= requests;
            scan_request_group <= scan_group;
            scan_request_done <= scan_wrapped && scan_at_rr;
            scan_result_found <= found;
            scan_result_pick <= pick;
            scan_result_group <= scan_request_group;
            scan_result_done <= scan_request_done;
        end
    end

    always @(posedge clk) begin
        if (rst_rr) rr <= {VLW{1'b0}};
        else if (state == S_STREAM && q_rd_valid && msg_ready && q_rd_last)
            rr <= (q_rd_vl == N_VL - 1) ? {VLW{1'b0}} : q_rd_vl + 1'b1;
    end

    assign msg_vl     = q_rd_vl;
    assign msg_valid  = (state == S_STREAM) & q_rd_valid;
    assign msg_data   = q_rd_data;
    assign msg_last   = q_rd_last;
    assign q_rd_ready = (state == S_STREAM) & msg_ready;

    always @(posedge clk) begin
        if (rst_ctrl) begin
            state <= S_IDLE; q_rd_vl <= {VLW{1'b0}};
            scan_group <= {GW{1'b0}}; scan_wrapped <= 1'b0;
            scan_request_valid <= 1'b0; scan_result_valid <= 1'b0;
            msg_len <= {SLOT_AW{1'b0}}; msg_meta <= {MW{1'b0}};
            stat_deliver <= 1'b0;
        end else begin
            stat_deliver <= 1'b0;
            case (state)
                S_IDLE: begin
                    scan_request_valid <= 1'b0;
                    scan_result_valid <= 1'b0;
                    if (!q_init_busy) begin
                        scan_group <= rr_group;
                        scan_wrapped <= 1'b0;
                        state <= S_SCAN_GROUP;
                    end
                end
                S_SCAN_GROUP: begin
                    scan_request_valid <= 1'b1;
                    scan_result_valid <= scan_request_valid;
                    scan_group <= next_scan_group;
                    scan_wrapped <= next_wrapped;
                    if (scan_result_valid && scan_result_found) begin
                        q_rd_vl <= {scan_result_group, 4'b0000} | scan_result_pick;
                        state <= S_SETUP;
                    end else if (scan_result_valid && scan_result_done) begin
                        // Completed the wrapped portion of the starting group.
                        state <= S_IDLE;
                    end
                end
                S_SETUP: if (q_rd_info_valid) begin
                    msg_len    <= q_rd_len;
                    msg_meta   <= q_rd_meta;
                    state      <= S_STREAM;
                end
                S_STREAM: if (q_rd_valid && msg_ready && q_rd_last) begin
                    stat_deliver <= 1'b1;
                    state        <= S_RELEASE;
                end
                S_RELEASE: if (q_rd_pop_ready) state <= S_POP;
                // The queue applies the pop on the next clock, deferred one
                // more when it meets a commit. In RAM mode S_DRAIN keeps the
                // first nonempty read after that write; the vector has
                // already cleared when S_SCAN_GROUP samples it.
                S_POP: state <= (NONEMPTY_RAM != 0) ? S_DRAIN : S_IDLE;
                S_DRAIN: state <= S_IDLE;
            endcase
        end
    end
endmodule
