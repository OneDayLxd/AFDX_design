/*
 * ================================================================
 * File Name    : afdx_mac_tx.v
 * Module Name  : afdx_mac_tx
 *
 * Function:
 *   MAC 发送：共享小块帧存储、逐 VL FIFO、按实际出口 BAG 的轮询选择。
 *   BAG 未到期的 VL 不会挡住其它 VL；同 VL 帧顺序保持不变。
 *   队列弹性读口直接驱动数据阶段，连续 tx_slot 时每时钟输出一个字节。
 *   in_tag 随帧保存；带标记帧的 SOF 同时给出 stat_sof_tag，用于时间同步报文的发送时间戳。
 *
 * Sub Module:
 *   afdx_crc32
 *   afdx_msg_queue
 *   afdx_first16_nz
 *   afdx_rr_arbiter
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_defs.vh"
module afdx_mac_tx #(
    parameter FIFO_AW = 12,
    parameter IFG = 12,
    parameter N_VL = 8,
    parameter VLW = 3,
    parameter CLK_HZ = 100_000_000,
    parameter STRICT_BAG = 1
) (
    input wire clk,
    input wire rst,
    input wire [7:0] in_data,
    input wire in_valid,
    input wire in_last,
    input wire [VLW-1:0] in_vl,
    input wire [19:0] in_bag_us,
    input wire in_tag,
    output wire in_ready,
    output wire [7:0] tx_data,
    output wire tx_valid,
    input wire tx_slot,
    output wire tx_frame,
    output wire tx_busy,
    output reg stat_frame,
    output wire [N_VL-1:0] bag_open,
    output reg stat_sof,
    output reg [VLW-1:0] stat_sof_vl,
    output reg stat_sof_tag,
    output wire bag_wait
);
    `include "afdx_math.vh"
    localparam SLOT_AW = (FIFO_AW < 11) ? FIFO_AW : 11;
    localparam BLOCK_AW = (SLOT_AW < 6) ? SLOT_AW : 6;
    localparam BLOCK_LOG = FIFO_AW - BLOCK_AW;
    localparam DESC_LOG = (BLOCK_LOG < 1) ? 1 : BLOCK_LOG;
    localparam IFG_W = afdx_clog2(IFG + 1);
    localparam CYCLES_PER_US = CLK_HZ / 1_000_000;
    // 截止时间模差位宽：最大 BAG 128 ms 的周期数再加 1 位符号。原 32 位时每个 VL
    // 各一条 32 位减法链，两路 MAC 共 256 条，是布线拥塞的主要来源之一。
    localparam TW = afdx_clog2(128000 * CYCLES_PER_US + 1) + 1;
    localparam S_IDLE = 4'd0, S_DATA = 4'd1, S_FCS = 4'd2, S_IFG = 4'd3,
               S_SETUP = 4'd4, S_POP = 4'd5, S_BAG = 4'd6, S_REFRESH = 4'd7,
               S_BAG_PREP = 4'd8;
    reg [3:0] state;
    reg [VLW-1:0] frame_vl, rr_ptr;
    reg [19:0] frame_bag_us;
    reg frame_tag;
    reg first_byte;
    wire read_start;
    reg frame_due;
    wire data_allowed = !first_byte || frame_due;
    reg [31:0] crc;
    reg [1:0] fcs_idx;
    reg [IFG_W-1:0] ifg_cnt;
    wire [7:0] q_data;
    wire q_valid, q_last, q_info_valid, q_pop_ready;
    // Start the queue read on the edge where the descriptor is valid.
    assign read_start = (state == S_BAG) && q_info_valid;
    wire [19:0] q_bag_us;
    wire q_tag;
    wire [N_VL-1:0] nonempty;
    reg found;
    reg [VLW-1:0] pick;
    wire [N_VL-1:0] ready_requests, prepare_requests;
    wire take = tx_valid && tx_slot;
    wire data_take = state == S_DATA && data_allowed && q_valid && tx_slot;
    wire pop = state == S_POP && q_pop_ready;
    wire start_frame = data_take && first_byte;
    wire [31:0] crc_next;
    reg [24:0] bag_times24;
    wire [24:0] bag_times25 = bag_times24 + {5'd0,frame_bag_us};
    // At 100 MHz, 100*x = 4*(24*x+x). Capture the first sum alongside
    // the descriptor, leaving only one carry chain in each cycle.
    always @(posedge clk)
        bag_times24 <= {1'b0,q_bag_us,4'b0} + {2'b0,q_bag_us,3'b0};
    wire [31:0] bag_cycles_now = (CYCLES_PER_US == 100) ?
        {5'd0,bag_times25,2'b0} : frame_bag_us * CYCLES_PER_US;
    wire [7:0] fcs_byte = (fcs_idx == 0) ? ~crc[7:0] :
                          (fcs_idx == 1) ? ~crc[15:8] :
                          (fcs_idx == 2) ? ~crc[23:16] : ~crc[31:24];
    assign tx_data = (state == S_FCS) ? fcs_byte : q_data;
    assign tx_valid = (state == S_DATA && data_allowed && q_valid) || state == S_FCS;
    assign tx_frame = (state == S_DATA && data_allowed) || state == S_FCS;
    assign tx_busy = state != S_IDLE;
    // Registered SOF report; the redundancy credit clears one clock later.
    always @(posedge clk) begin
        if (rst) begin stat_sof <= 1'b0; stat_sof_tag <= 1'b0; end
        else begin stat_sof <= start_frame; stat_sof_tag <= start_frame && frame_tag; end
        stat_sof_vl <= frame_vl;
    end
    // 此信号表示已有完整帧，但所有候选 VL 均在等待 BAG。
    assign bag_wait = (state == S_IDLE && (|nonempty) && !(|ready_requests) && !(|prepare_requests)) ||
                      (state == S_DATA && first_byte && !frame_due);
    afdx_crc32 u_crc (
        .crc_in  (crc),
        .data    (q_data),
        .crc_out (crc_next)
    );

    // Only data bytes enter the CRC; FCS selection and frame arbitration do
    // not belong in its feedback path. Initialize throughout the idle state.
    always @(posedge clk) begin
        if (rst || state == S_IDLE) crc <= `AFDX_CRC_INIT;
        else if (data_take) crc <= crc_next;
    end

    // Two-entry circular input buffer. Queue admission only advances the
    // read pointer; speculative writes touch only the unoccupied tail slot.
    // Keep the last consumed entry selected while empty for queue prefetch.
    localparam IW = VLW+30;
    reg [IW-1:0] input_e0, input_e1;
    reg input_wsel, input_rsel, input_hsel;
    reg input_valid, input_spare;
    wire queue_ready;
    wire input_put = in_valid && in_ready;
    reg queue_valid;
    (* preserve *) reg [IW-1:0] queue_entry;
    wire queue_stage_ready = !queue_valid || queue_ready;
    wire input_get = input_valid && queue_stage_ready;
    wire [VLW-1:0] queue_vl;
    wire [19:0] queue_bag;
    wire queue_last, queue_tag;
    wire [7:0] queue_data;
    wire [IW-1:0] input_head = input_hsel ? input_e1 : input_e0;
    assign {queue_vl,queue_tag,queue_bag,queue_last,queue_data} = queue_entry;
    // End the circular-buffer mux at a local elastic stage before queue
    // admission/commit logic. Hold the last entry while invalid for prefetch.
    always @(posedge clk) begin
        if (rst) queue_valid <= 1'b0;
        else if (queue_stage_ready) queue_valid <= input_valid;
        if (queue_stage_ready && input_valid) queue_entry <= input_head;
    end
    assign in_ready = !input_spare;
    always @(posedge clk) begin
        // in_valid includes upstream A/B ready logic. Keep that round trip
        // off the entry enables; input_put alone publishes the sampled data.
        if (!input_spare) begin
            if (input_wsel) input_e1 <= {in_vl,in_tag,in_bag_us,in_last,in_data};
            else input_e0 <= {in_vl,in_tag,in_bag_us,in_last,in_data};
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            input_valid<=0; input_spare<=0;
            input_wsel<=0; input_rsel<=0; input_hsel<=1;
        end
        else begin
            if (input_put) input_wsel <= ~input_wsel;
            if (input_get) begin
                input_rsel <= ~input_rsel;
                input_hsel <= (input_spare || input_put) ? ~input_rsel : input_rsel;
                if (input_spare) input_spare<=0;
                else if (!input_put) input_valid<=0;
            end else if (input_put) begin
                if (!input_valid) begin
                    input_hsel<=input_rsel; input_valid<=1;
                end else input_spare<=1;
            end
        end
    end

    afdx_msg_queue #(
        .N_VL     (N_VL),
        .VLW      (VLW),
        .DEPTH    (1 << BLOCK_LOG),
        .DW       (DESC_LOG),
        .SLOT_AW  (SLOT_AW),
        .MW       (21),
        .POOL_AW  (FIFO_AW-SLOT_AW),
        .BLOCK_AW (BLOCK_AW)
    ) u_frames (
        .clk            (clk),
        .rst            (rst),
        .wr_vl          (queue_vl),
        .wr_vl_same     (1'b1),
        .wr_valid       (queue_valid),
        .wr_data        (queue_data),
        .wr_ready       (queue_ready),
        .wr_commit      (queue_valid && queue_last),
        .wr_meta        ({queue_tag,queue_bag}),
        .wr_abort       (1'b0),
        .wr_ovf         (),
        .rd_vl          (frame_vl),
        .rd_start       (read_start),
        .rd_info_valid  (q_info_valid),
        .rd_pop_ready   (q_pop_ready),
        .reclaim_busy   (),
        .rd_pop         (pop),
        .rd_len         (),
        .rd_meta        ({q_tag,q_bag_us}),
        .rd_data        (q_data),
        .rd_valid       (q_valid),
        .rd_last        (q_last),
        .rd_ready       (state == S_DATA && data_allowed && tx_slot),
        .nonempty       (nonempty),
        .pool_used      (),
        .pool_peak      (),
        .pool_msgs      (),
        .pool_msgs_peak (),
        .ne_group       ({(VLW>4 ? VLW-4 : 1){1'b0}}),
        .ne_bits        (),
        .init_busy      ()
    );

    // BAG state. Each 16-VL region keeps its VL deadlines in a small RAM and
    // compares one of them per clock with its local clock count, so a VL is
    // revisited every 16 clocks and its bag_active/prepare_open flags follow
    // a few clocks after the comparison. These flags only steer arbitration.
    // The SOF clock itself comes from one exact comparison with the selected
    // VL's own deadline (frame_due below). A full-width deadline register,
    // subtractor and comparators per VL took about two thirds of each MAC.
    localparam GROUPS = (N_VL + 15) / 16;
    localparam GW = afdx_clog2(GROUPS);
    localparam SCAN_N = (N_VL < 16) ? N_VL : 16;
    // prepare_open sets between 2^PREP_LOG and 2^PREP_LOG-SCAN_N-4 clocks
    // before the deadline; bag_active clears at most SCAN_N+4 clocks after it.
    localparam PREP_LOG = 6;
    localparam [TW-1:0] PREP_AHEAD = {{(TW-1){1'b0}}, 1'b1} << PREP_LOG;
    // Distinct constant phases prevent synthesis from merging the region
    // counters; each region stores its deadlines with the same phase.
    (* preserve *) reg [TW-1:0] cycle_count_group [0:GROUPS-1];
    (* preserve *) reg [TW-1:0] prep_count_group [0:GROUPS-1];
    wire [TW-1:0] cycle_count = cycle_count_group[0];
    reg [TW-1:0] deadline_base, deadline_q, deadline_m1;
    wire [TW-1:0] deadline_base_plus1 = deadline_base + 1'b1;
    wire deadline_step = (state == S_BAG_PREP) || ((state == S_DATA) && first_byte);
    // The SOF clock captures deadline_q = SOF clock + BAG (region-0 time) and
    // deadline_m1 = deadline_q - 1; both hold until the next frame's setup.
    always @(posedge clk) begin
        if (rst) deadline_base <= 0;
        else if (state == S_BAG && q_info_valid)
            deadline_base <= cycle_count + bag_cycles_now[TW-1:0];
        else if (deadline_step) deadline_base <= deadline_base_plus1;
        if (deadline_step) begin
            deadline_q <= deadline_base_plus1;
            deadline_m1 <= deadline_base;
        end
    end
    reg [N_VL-1:0] bag_active, prepare_open;
    assign bag_open = ~bag_active;
    assign ready_requests = nonempty & bag_open;
    assign prepare_requests = nonempty & prepare_open;
    // Warm the descriptor/payload pipeline before expiry (prepare_open).
    // Already eligible VLs have priority over these preparation candidates.
    // Arbitration is a registered pipeline with one 16-way or group-level
    // choice per stage: M rr_ptr masks, A0 local requests, A1 request bits, A2 first VL in each
    // group, B1 ready/preparation class, B2 group round robin, B3 VL.
    // Nonempty only falls after this MAC's own POP (two clocks later in the
    // queue); S_REFRESH waits for ARB_FLUSH clocks so that the pick always
    // comes from post-POP state. Later arrivals only add requests.
    localparam ARB_FLUSH = 8;
    localparam FLUSH_W = afdx_clog2(ARB_FLUSH + 1);
    wire [GW-1:0] rr_group;
    wire [3:0] rr_lane;
    generate if (VLW > 4) begin : g_wide_rr
        assign rr_group = rr_ptr[VLW-1:4];
        assign rr_lane = rr_ptr[3:0];
    end else begin : g_narrow_rr
        assign rr_group = {GW{1'b0}};
        if (VLW == 4) begin : g_four_lane
            assign rr_lane = rr_ptr;
        end else begin : g_short_lane
            assign rr_lane = {{(4-VLW){1'b0}}, rr_ptr};
        end
    end endgenerate
    reg [GW-1:0] rr_group_m;
    reg [GROUPS-1:0] start_group_m;
    always @(posedge clk) begin
        rr_group_m <= rr_group;
    end
    reg [GROUPS-1:0] ready_group_q, prep_group_q;
    reg [GROUPS-1:0] ready_after_q, prep_after_q;
    reg [3:0] ready_pick_q [0:GROUPS-1];
    reg [3:0] prep_pick_q [0:GROUPS-1];
    wire [GROUPS-1:0] ready_found_all;
    function [3:0] nibble_any;
        input [15:0] x;
        nibble_any = {|x[15:12], |x[11:8], |x[7:4], |x[3:0]};
    endfunction
    genvar grp, lane;
    generate for (grp = 0; grp < GROUPS; grp = grp + 1) begin : g_rr_group
        localparam [GW-1:0] GROUP_ID = grp[GW-1:0];
        wire [15:0] ready_bits, prep_bits;
        reg [15:0] ready_local, prep_local;
        // A local lane mask avoids a shared upper-mask flop driving all
        // groups and a second decode on the request-to-nibble path.
        (* preserve *) reg [15:0] after_mask_m;
        // A1 also keeps the requests at or after rr_lane of the start group.
        reg [15:0] ready_bits_q, prep_bits_q, ready_after_bits_q, prep_after_bits_q;
        wire ready_found, prep_found, ready_after_found, prep_after_found;
        wire [3:0] ready_first, prep_first, ready_after_first, prep_after_first;
        for (lane = 0; lane < 16; lane = lane + 1) begin : g_lane
            always @(posedge clk)
                after_mask_m[lane] <= (rr_group != GROUP_ID) || (rr_lane <= lane);
            if (grp*16 + lane < N_VL) begin : g_real
                assign ready_bits[lane] = ready_requests[grp*16 + lane];
                assign prep_bits[lane] = prepare_requests[grp*16 + lane];
            end else begin : g_pad
                assign ready_bits[lane] = 1'b0;
                assign prep_bits[lane] = 1'b0;
            end
        end
        wire [15:0] ready_after_bits = ready_local & after_mask_m;
        wire [15:0] prep_after_bits = prep_local & after_mask_m;
        // Per-nibble ORs are registered with the bits so the Q-to-pick path
        // is only the nibble-local priority and two small muxes.
        reg [3:0] ready_nz_q, prep_nz_q, ready_after_nz_q, prep_after_nz_q;
        always @(posedge clk) begin
            start_group_m[grp] <= rr_group == GROUP_ID;
            ready_local <= ready_bits;
            prep_local <= prep_bits;
            ready_bits_q <= ready_local;
            prep_bits_q <= prep_local;
            ready_after_bits_q <= ready_after_bits;
            prep_after_bits_q <= prep_after_bits;
            ready_nz_q <= nibble_any(ready_local);
            prep_nz_q <= nibble_any(prep_local);
            ready_after_nz_q <= nibble_any(ready_after_bits);
            prep_after_nz_q <= nibble_any(prep_after_bits);
        end
        afdx_first16_nz u_ready (
            .x       (ready_bits_q),
            .nonzero (ready_nz_q),
            .any     (ready_found),
            .index   (ready_first)
        );
        afdx_first16_nz u_ready_after (
            .x       (ready_after_bits_q),
            .nonzero (ready_after_nz_q),
            .any     (ready_after_found),
            .index   (ready_after_first)
        );
        afdx_first16_nz u_prepare (
            .x       (prep_bits_q),
            .nonzero (prep_nz_q),
            .any     (prep_found),
            .index   (prep_first)
        );
        afdx_first16_nz u_prepare_after (
            .x       (prep_after_bits_q),
            .nonzero (prep_after_nz_q),
            .any     (prep_after_found),
            .index   (prep_after_first)
        );
        assign ready_found_all[grp] = ready_found;
        always @(posedge clk) begin
            if (rst) begin
                ready_group_q[grp] <= 1'b0;
                prep_group_q[grp] <= 1'b0;
                ready_after_q[grp] <= 1'b0;
                prep_after_q[grp] <= 1'b0;
            end else begin
                ready_group_q[grp] <= ready_found;
                prep_group_q[grp] <= prep_found;
                ready_after_q[grp] <= ready_after_found;
                prep_after_q[grp] <= prep_after_found;
            end
            ready_pick_q[grp] <= ready_after_found ? ready_after_first : ready_first;
            prep_pick_q[grp] <= prep_after_found ? prep_after_first : prep_first;
        end
    end endgenerate

    // |ready_group_q, registered alongside it: the class choice and the
    // S_IDLE enable no longer need an 8-input OR from the group registers.
    reg ready_any_q;
    always @(posedge clk) begin
        if (rst) ready_any_q <= 1'b0;
        else ready_any_q <= |ready_found_all;
    end
    wire choose_ready = ready_any_q;
    reg class_ready_b1;
    reg [GROUPS-1:0] valid_b1, after_b1;
    reg [3:0] lane_b1 [0:GROUPS-1];
    always @(posedge clk) begin
        if (rst) begin class_ready_b1 <= 1'b0; valid_b1 <= {GROUPS{1'b0}}; end
        else begin
            class_ready_b1 <= choose_ready;
            valid_b1 <= choose_ready ? ready_group_q : prep_group_q;
        end
        after_b1 <= choose_ready ? ready_after_q : prep_after_q;
    end
    // Only the start group needs lanes at or after rr_lane.
    wire [GROUPS-1:0] group_request = valid_b1 & (~start_group_m | after_b1);
    wire group_found;
    wire [GW-1:0] group_pick;
    afdx_rr_arbiter #(
        .N (GROUPS),
        .W (GW)
    ) u_group_rr (
        .request     (group_request),
        .start_index (rr_group_m),
        .valid       (group_found),
        .index       (group_pick)
    );
    reg any_b2, class_ready_b2;
    reg [GW-1:0] group_b2;
    reg [3:0] lane_b2 [0:GROUPS-1];
    always @(posedge clk) begin
        if (rst) begin any_b2 <= 1'b0; class_ready_b2 <= 1'b0; end
        else begin any_b2 <= |valid_b1; class_ready_b2 <= class_ready_b1; end
        // If the only requests are below rr_lane in rr_group, use its wrapped
        // local pick. Otherwise the group arbiter implements the global RR order.
        group_b2 <= group_found ? group_pick : rr_group_m;
    end
    generate for (grp = 0; grp < GROUPS; grp = grp + 1) begin : g_lane_pipe
        always @(posedge clk) begin
            lane_b1[grp] <= choose_ready ? ready_pick_q[grp] : prep_pick_q[grp];
            lane_b2[grp] <= lane_b1[grp];
        end
    end endgenerate
    wire [3:0] selected_lane = lane_b2[group_b2];
    wire [GW+3:0] selected_full = {group_b2, selected_lane};
    wire [VLW-1:0] selected_vl;
    reg selected_ready;
    // One-hot copies of the pick select per-VL state without a VL decoder.
    reg [GROUPS-1:0] pick_group_oh, frame_group_oh;
    reg [15:0] pick_lane_oh, frame_lane_oh;
    generate if (VLW <= GW+4) begin : g_pick_narrow
        assign selected_vl = selected_full[VLW-1:0];
    end else begin : g_pick_wide
        assign selected_vl = {{(VLW-GW-4){1'b0}}, selected_full};
    end endgenerate
    always @(posedge clk) begin
        if (rst) begin found <= 1'b0; pick <= {VLW{1'b0}}; selected_ready <= 1'b0; end
        else begin
            found <= any_b2;
            pick <= selected_vl;
            selected_ready <= class_ready_b2;
        end
        pick_lane_oh <= 16'd1 << selected_lane;
    end
    generate for (grp = 0; grp < GROUPS; grp = grp + 1) begin : g_pick_group
        localparam [GW-1:0] GROUP_ID = grp[GW-1:0];
        always @(posedge clk) pick_group_oh[grp] <= group_b2 == GROUP_ID;
    end endgenerate

    // SOF (stat_sof, one clock after the first byte) is registered once per
    // 16-VL region before it updates BAG state; frame_vl, frame_lane_oh and
    // the captured deadline are stable until the next frame is selected.
    (* preserve *) reg [GROUPS-1:0] rst_group, sof_group;
    wire [3:0] frame_lane;
    generate if (VLW >= 4) begin : g_frame_lane_wide
        assign frame_lane = frame_vl[3:0];
    end else begin : g_frame_lane_narrow
        assign frame_lane = {{(4-VLW){1'b0}}, frame_vl};
    end endgenerate
    genvar cg, fl;
    generate for (cg = 0; cg < GROUPS; cg = cg + 1) begin : g_deadline_region
        // Region number in the top bits, built at exactly TW bits.
        localparam [GW-1:0] REGION = cg[GW-1:0];
        localparam [TW-1:0] PHASE = {REGION, {(TW-GW){1'b0}}};
        localparam [3:0] SCAN_FIRST = cg[3:0];
        localparam [3:0] SCAN_LAST = (SCAN_N == 16) ? 4'd15 :
                                     (SCAN_N[3:0] - 1'b1);
        (* ramstyle = "M4K, no_rw_check" *) reg [TW-1:0] deadline_mem [0:15];
        (* preserve *) reg [3:0] scan_lane;
        reg [TW-1:0] scan_read, scan_deadline;
        reg [3:0] scan_lane1, scan_lane2;
        reg scan_valid1, scan_valid2, scan_expired, scan_near;
        reg [15:0] scan_hit;
        wire [TW-1:0] elapsed = cycle_count_group[cg] - scan_deadline;
        wire [TW-1:0] prep_elapsed = prep_count_group[cg] - scan_deadline;
        always @(posedge clk) begin
            rst_group[cg] <= rst;
            if (rst) sof_group[cg] <= 1'b0;
            else sof_group[cg] <= stat_sof && frame_group_oh[cg];
            if (rst) begin
                cycle_count_group[cg] <= PHASE;
                prep_count_group[cg] <= PHASE + PREP_AHEAD;
            end else begin
                cycle_count_group[cg] <= cycle_count_group[cg] + 1'b1;
                prep_count_group[cg] <= prep_count_group[cg] + 1'b1;
            end
            if (rst) scan_lane <= SCAN_FIRST;
            else if (scan_lane == SCAN_LAST) scan_lane <= 4'd0;
            else scan_lane <= scan_lane + 1'b1;
        end
        always @(posedge clk) begin
            if (sof_group[cg]) deadline_mem[frame_lane] <= deadline_q + PHASE;
            scan_read <= deadline_mem[scan_lane];
        end
        // Read, register, compare, then apply. A read on or before the clock
        // of a deadline write is stale, so an SOF in this region drops the
        // three reads still in flight; the fourth applies with the SOF, which
        // takes priority.
        always @(posedge clk) begin
            scan_lane1 <= scan_lane;
            scan_valid1 <= !sof_group[cg];
            scan_deadline <= scan_read;
            scan_lane2 <= scan_lane1;
            scan_valid2 <= scan_valid1 && !sof_group[cg];
            scan_expired <= !elapsed[TW-1];
            scan_near <= !prep_elapsed[TW-1];
            scan_hit <= (scan_valid2 && !sof_group[cg]) ? (16'd1 << scan_lane2) : 16'd0;
        end
        // Only this MAC's SOF sets bag_active, so a VL seen as ready stays
        // ready until it is sent. Uninitialized deadlines only reach VLs
        // whose flags already hold the reset values.
        for (fl = 0; fl < 16; fl = fl + 1) begin : g_lane_flags
            if (cg*16 + fl < N_VL) begin : g_real
                always @(posedge clk) begin
                    if (rst_group[cg]) begin
                        bag_active[cg*16 + fl] <= 1'b0;
                        prepare_open[cg*16 + fl] <= 1'b1;
                    end else if (STRICT_BAG != 0) begin
                        if (sof_group[cg] && frame_lane_oh[fl]) begin
                            bag_active[cg*16 + fl] <= 1'b1;
                            prepare_open[cg*16 + fl] <= 1'b0;
                        end else if (scan_hit[fl]) begin
                            if (scan_expired) bag_active[cg*16 + fl] <= 1'b0;
                            if (scan_near) prepare_open[cg*16 + fl] <= 1'b1;
                        end
                    end
                end
            end
        end
    end endgenerate

    // Exact BAG boundary of the selected VL. A ready-class pick has no running
    // BAG. A prepare-class pick had bag_active set while no VL was ready, so
    // its stored deadline lies within the preparation window; the second
    // deadline copy holds deadline-1, and frame_due opens on the clock
    // SOF + BAG like the per-VL comparators did. The lookup follows the pick
    // in S_IDLE; frame_due is valid three clocks after selection, before the
    // earliest S_DATA clock.
    (* ramstyle = "M4K, no_rw_check" *) reg [TW-1:0] frame_deadline_mem [0:N_VL-1];
    reg [TW-1:0] frame_deadline_read, frame_deadline_m1;
    reg frame_ready_class;
    wire [VLW-1:0] frame_deadline_addr = (state == S_IDLE) ? pick : frame_vl;
    wire [TW-1:0] frame_elapsed = cycle_count - frame_deadline_m1;
    always @(posedge clk) begin
        if (stat_sof) frame_deadline_mem[frame_vl] <= deadline_m1;
        frame_deadline_read <= frame_deadline_mem[frame_deadline_addr];
    end
    always @(posedge clk) begin
        frame_deadline_m1 <= frame_deadline_read;
        if (rst) frame_due <= 1'b0;
        else frame_due <= (STRICT_BAG == 0) || frame_ready_class || !frame_elapsed[TW-1];
    end

    reg [FLUSH_W-1:0] arb_flush;
    always @(posedge clk) begin
        if (rst) arb_flush <= {FLUSH_W{1'b0}};
        else if (pop) arb_flush <= ARB_FLUSH[FLUSH_W-1:0];
        else if (arb_flush != 0) arb_flush <= arb_flush - 1'b1;
    end
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; frame_vl <= 0; frame_bag_us <= 0; frame_tag <= 0; rr_ptr <= 0;
            frame_group_oh <= {GROUPS{1'b0}}; frame_lane_oh <= 16'd0;
            first_byte <= 0; frame_ready_class <= 0;
            fcs_idx <= 0; ifg_cnt <= 0; stat_frame <= 0;
        end else begin
            stat_frame <= 0;
            case (state)
                // A ready VL appearing in the newer group snapshot takes
                // priority over a still-pipelined preparation-only choice.
                S_IDLE: if (found && (!ready_any_q || selected_ready)) begin
                    frame_vl <= pick;
                    frame_group_oh <= pick_group_oh;
                    frame_lane_oh <= pick_lane_oh;
                    frame_ready_class <= selected_ready;
                    first_byte <= 1;
                    state <= S_SETUP;
                end
                S_SETUP: if (q_info_valid) begin
                    frame_bag_us <= q_bag_us;
                    frame_tag <= q_tag;
                    state <= S_BAG;
                end
                S_BAG: if (q_info_valid) begin
                    state <= S_BAG_PREP;
                end
                S_BAG_PREP: state <= S_DATA;
                S_DATA: begin
                    if (data_take) begin
                        first_byte <= 0;
                        if (q_last) begin
                            fcs_idx <= 0;
                            state <= S_FCS;
                        end
                    end
                end
                S_FCS: if (take) begin
                    fcs_idx <= fcs_idx + 1'b1;
                    if (fcs_idx == 3) begin
                        stat_frame <= 1;
                        ifg_cnt <= 0;
                        rr_ptr <= (frame_vl == N_VL-1) ? {VLW{1'b0}} : frame_vl + 1'b1;
                        state <= S_POP;
                    end
                end
                S_POP: if (q_pop_ready) state <= S_IFG;
                S_IFG: if (tx_slot) begin
                    ifg_cnt <= ifg_cnt + 1'b1;
                    if (ifg_cnt == IFG-1) state <= S_REFRESH;
                end
                // Wait until the arbitration pipeline has observed the
                // completed pop, including when IFG is only one byte slot.
                S_REFRESH: if (arb_flush == 0) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
