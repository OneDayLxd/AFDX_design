/*
 * ================================================================
 * File Name    : afdx_msg_queue.v
 * Module Name  : afdx_msg_queue
 *
 * Function:
 *   多 VL 消息队列：固定总容量的字节 RAM，按需分配小块，逐 VL 描述符 FIFO。
 *   BLOCK_AW=SLOT_AW 时与原整槽分配等价；小块模式下长消息跨块且帧内读出不插气泡。
 *   Synchronous metadata, bitmap allocation and background block reclamation.
 *   rd_start requires rd_info_valid; the read begins on the next clock.
 *   rd_pop requires rd_pop_ready; the pop takes effect on the next clock and nonempty
 *   (NONEMPTY_RAM=0) falls one clock after that. Hold rd_vl until the pop has taken effect.
 *   pool_used includes retired blocks until reclaim_busy deasserts.
 *
 * Sub Module:
 *   afdx_rr_arbiter
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_msg_queue #(
    parameter N_VL = 8,
    parameter VLW = 3,
    parameter DEPTH = 4,
    parameter DW = 2,
    parameter SLOT_AW = 11,                 // 消息长度位宽；同时保持旧容量参数的含义
    parameter MW = 9,
    parameter POOL_AW = 5,                 // 总字节容量仍为 2^(POOL_AW+SLOT_AW)
    parameter BLOCK_AW = SLOT_AW,
    parameter MAX_MSGS = 1 << (POOL_AW + SLOT_AW - BLOCK_AW),
    // RX parser sets wr_vl during the header, long before the first payload
    // byte. Register its admission tag match to shorten the ready feedback.
    parameter PIPE_ADMISSION_MATCH = 0,
    // 1: the producer registers wr_vl_same = (wr_vl == wr_vl one clock ago)
    //    and only presents in-range VLs as valid, so the ready path sees no
    //    wr_vl comparator (the TX input slice holds wr_vl in a register).
    parameter EXT_VL_SAME = 0,
    // 1: rd_info_valid omits the live desc_vl == rd_vl tag. desc_valid already
    //    drops one clock after rd_vl changes, so the consumer must only ignore
    //    rd_info_valid on the first clock after it changes rd_vl.
    parameter RD_INFO_UNTAGGED = 0,
    // 1: 大 VL 数时不再生成 N_VL 位 nonempty 寄存器，
    //    改为 16 路 1 位宽 RAM，按 16 个 VL 一组读出（ne_group -> ne_bits，1 拍延迟）。
    //    两种模式复位后都逐项清零状态 RAM，init_busy 期间不接收写入。
    parameter NONEMPTY_RAM = 0
) (
    input wire clk, rst,
    input wire [VLW-1:0] wr_vl,
    input wire wr_vl_same,
    input wire wr_valid,
    input wire [7:0] wr_data,
    output wire wr_ready,
    input wire wr_commit,
    input wire [MW-1:0] wr_meta,
    input wire wr_abort,
    output reg wr_ovf,
    input wire [VLW-1:0] rd_vl,
    input wire rd_start, rd_pop,
    output wire rd_info_valid,
    output wire reclaim_busy,
    output wire rd_pop_ready,
    output wire [SLOT_AW-1:0] rd_len,
    output wire [MW-1:0] rd_meta,
    output wire [7:0] rd_data,
    output wire rd_valid, rd_last,
    input wire rd_ready,
    output reg [N_VL-1:0] nonempty,
    output wire [POOL_AW+SLOT_AW-BLOCK_AW:0] pool_used,
    output reg [POOL_AW+SLOT_AW-BLOCK_AW:0] pool_peak,
    output wire [POOL_AW+SLOT_AW-BLOCK_AW:0] pool_msgs,
    output reg [POOL_AW+SLOT_AW-BLOCK_AW:0] pool_msgs_peak,
    input wire [(VLW>4 ? VLW-4 : 1)-1:0] ne_group,
    output reg [15:0] ne_bits,
    output wire init_busy
);
    `include "afdx_math.vh"
    localparam AW = POOL_AW + SLOT_AW;
    localparam BW = AW - BLOCK_AW;
    localparam BLOCKS = 1 << BW;
    localparam [BW:0] BLOCKS_COUNT = BLOCKS[BW:0];
    // An active writer reserves one of its VL's DEPTH entries. No reachable
    // state can use more than N_VL*DEPTH descriptors (or BLOCKS payload blocks).
    localparam QUOTA_MSGS = (MAX_MSGS < N_VL*DEPTH) ? MAX_MSGS : N_VL*DEPTH;
    localparam MSGS = (QUOTA_MSGS < BLOCKS) ? QUOTA_MSGS : BLOCKS;
    localparam [BW:0] MSGS_COUNT = MSGS[BW:0];
    localparam MDW = afdx_clog2(MSGS);
    localparam MAX_LEN = ((1 << SLOT_AW)-1 < (1 << AW)) ? (1 << SLOT_AW)-1 : (1 << AW);
    // Committed blocks cannot be allocated for writing until rd_pop releases
    // them; active reads and writes therefore never address the same byte.
    // The legacy mapper leaves 16384x8 arrays at AUTO and warns about M-RAM
    // power-up even with ramstyle=M4K. 4096x8 banks infer without that warning.
    localparam PAYLOAD_BANK_AW = AW > 12 ? 12 : AW;
    localparam PAYLOAD_BANKS = 1 << (AW-PAYLOAD_BANK_AW);
    localparam PAYLOAD_BANK_W = afdx_clog2(PAYLOAD_BANKS);
    // Only the writer changes payload links; two synchronous read ports serve
    // the stream prefetch and background reclamation (the tool may replicate).
    (* ramstyle = "M4K, no_rw_check" *) reg [BW-1:0] next_block [0:BLOCKS-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [MDW-1:0] next_msg [0:MSGS-1];
    localparam DESC_W = BW + (BW+1) + SLOT_AW + MW;
    (* ramstyle = "M4K, no_rw_check" *) reg [DESC_W-1:0] descriptor [0:MSGS-1];
    reg [BLOCKS-1:0] block_free;
    reg [MSGS-1:0] msg_free;
    // Independent single-writer memories allow simultaneous enqueue/dequeue
    // on different VLs without turning the pointer table into flip-flops.
    (* ramstyle = "M4K, no_rw_check" *) reg [MDW-1:0] first_head [0:N_VL-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [MDW-1:0] popped_head [0:N_VL-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [MDW-1:0] tail [0:N_VL-1];
    reg [MDW-1:0] first_head_q, popped_head_q, tail_q;
    reg first_head_we;
    reg [VLW-1:0] first_head_write_vl;
    reg [MDW-1:0] first_head_write_msg;
    reg head_from_pop_q, head_valid;
    reg [VLW-1:0] head_vl;
    wire [MDW-1:0] head_q = head_from_pop_q ? popped_head_q : first_head_q;
    // Keep two identical single-port copies of the VL state.  Each copy has
    // one synchronous lookup and the same write; this maps to block RAM even
    // on tools which expand a two-read/two-write array into logic cells.
    // Both copies are swept to zero after reset (init_active).
    localparam STATE_W = DW + 2;
    (* ramstyle = "M4K, no_rw_check" *) reg [STATE_W-1:0] writer_vl_state [0:N_VL-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [STATE_W-1:0] reader_vl_state [0:N_VL-1];
    reg [STATE_W-1:0] writer_state_q, reader_state_q;
    localparam NE_GW = VLW > 4 ? VLW-4 : 1;
    // 各实例（及发生器）的复位清扫逻辑完全相同；preserve 防止综合合并为一个远距离共享寄存器
    (* preserve *) reg init_active;
    (* preserve *) reg [VLW-1:0] init_vl;
    assign init_busy = init_active;
    reg [VLW-1:0] writer_vl_q, reader_vl_q;
    reg writer_lookup_valid, reader_lookup_valid;
    reg [STATE_W-1:0] last_commit_state, last_pop_state;
    reg pending_pop;
    reg [VLW-1:0] pending_pop_vl;
    reg [STATE_W-1:0] pending_pop_state;
    // Forwarding of the state written on the previous clock. The VL tag
    // comparisons are registered on the writing clock (the lookup tags do
    // not move on a write), so no comparator sits after the RAM output.
    reg writer_hit_commit, writer_hit_pop, reader_hit_commit, reader_hit_pop;
    wire [STATE_W-1:0] writer_state =
        writer_hit_commit ? last_commit_state :
        writer_hit_pop ? last_pop_state :
        writer_state_q;
    wire [STATE_W-1:0] reader_state =
        reader_hit_commit ? last_commit_state :
        reader_hit_pop ? last_pop_state :
        reader_state_q;
    wire [DW:0] writer_count = writer_state[STATE_W-1:1];
    wire [DW:0] reader_count = reader_state[STATE_W-1:1];
    reg [BW-1:0] free_block_head;
    reg [MDW-1:0] free_msg_head;
    reg block_candidate_valid, msg_candidate_valid;
    wire [BW-1:0] block_pick;
    wire [MDW-1:0] msg_pick;
    wire block_found, msg_found;
    // Two-level registered selection limits each local bitmap fan-in to 16.
    localparam BLOCK_GROUPS = (BLOCKS+15)/16;
    localparam BGW = afdx_clog2(BLOCK_GROUPS);
    reg [BLOCK_GROUPS-1:0] group_valid;
    reg [3:0] group_index [0:BLOCK_GROUPS-1];
    reg [3:0] group_index_q [0:BLOCK_GROUPS-1];
    wire [BGW-1:0] chosen_group;
    reg [BGW-1:0] chosen_group_q;
    genvar bg, bitn;
    generate for (bg=0; bg<BLOCK_GROUPS; bg=bg+1) begin : g_allocator
        wire [15:0] requests;
        wire found;
        wire [3:0] chosen;
        for (bitn=0; bitn<16; bitn=bitn+1) begin : g_request
            if (bg*16+bitn < BLOCKS) assign requests[bitn] = block_free[bg*16+bitn];
            else assign requests[bitn] = 1'b0;
        end
        afdx_rr_arbiter #(
            .N (16),
            .W (4)
        ) local_pick (
            .request     (requests),
            .start_index (4'd0),
            .valid       (found),
            .index       (chosen)
        );
        always @(posedge clk) begin
            if (rst) group_valid[bg] <= 0;
            else group_valid[bg] <= found;
            group_index[bg] <= chosen;
            group_index_q[bg] <= group_index[bg];
        end
    end endgenerate
    afdx_rr_arbiter #(
        .N (BLOCK_GROUPS),
        .W (BGW)
    ) u_free_blocks (
        .request     (group_valid),
        .start_index ({BGW{1'b0}}),
        .valid       (block_found),
        .index       (chosen_group)
    );
    assign block_pick = (chosen_group_q << 4) | group_index_q[chosen_group_q];
    reg block_alloc_delayed, block_alloc_delayed2, block_alloc_delayed3;
    reg block_found_q;
    always @(posedge clk) begin
        chosen_group_q <= chosen_group;
        if (rst) block_found_q <= 0;
        else block_found_q <= block_found;
    end
    reg msg_alloc_delayed, msg_alloc_delayed2, msg_alloc_delayed3;
    reg [BW-1:0] allocated_block;
    reg [MDW-1:0] allocated_msg;
    always @(posedge clk) begin
        allocated_block <= free_block_head;
        allocated_msg <= free_msg_head;
    end
    // Descriptor slots use the same registered two-level search as blocks;
    // a single MSGS-way priority tree was too deep for 256 receive slots.
    localparam MSG_GROUPS = (MSGS+15)/16;
    localparam MGW = afdx_clog2(MSG_GROUPS);
    reg [MSG_GROUPS-1:0] msg_group_valid;
    reg [3:0] msg_group_index [0:MSG_GROUPS-1];
    reg [3:0] msg_group_index_q [0:MSG_GROUPS-1];
    wire [MGW-1:0] msg_chosen_group;
    reg [MGW-1:0] msg_chosen_group_q;
    reg msg_found_q;
    genvar mg, mbit;
    generate for (mg=0; mg<MSG_GROUPS; mg=mg+1) begin : g_msg_allocator
        wire [15:0] requests;
        wire found;
        wire [3:0] chosen;
        for (mbit=0; mbit<16; mbit=mbit+1) begin : g_request
            if (mg*16+mbit < MSGS) begin : g_slot
                assign requests[mbit] = msg_free[mg*16+mbit];
            end else begin : g_pad
                assign requests[mbit] = 1'b0;
            end
        end
        afdx_rr_arbiter #(
            .N (16),
            .W (4)
        ) local_pick (
            .request     (requests),
            .start_index (4'd0),
            .valid       (found),
            .index       (chosen)
        );
        always @(posedge clk) begin
            if (rst) msg_group_valid[mg] <= 0;
            else msg_group_valid[mg] <= found;
            msg_group_index[mg] <= chosen;
            msg_group_index_q[mg] <= msg_group_index[mg];
        end
    end endgenerate
    afdx_rr_arbiter #(
        .N (MSG_GROUPS),
        .W (MGW)
    ) u_free_msgs (
        .request     (msg_group_valid),
        .start_index ({MGW{1'b0}}),
        .valid       (msg_found),
        .index       (msg_chosen_group)
    );
    assign msg_pick = (msg_chosen_group_q << 4) | msg_group_index_q[msg_chosen_group_q];
    always @(posedge clk) begin
        msg_chosen_group_q <= msg_chosen_group;
        if (rst) msg_found_q <= 0;
        else msg_found_q <= msg_found;
    end
    reg [1:0] reclaim_state;
    reg [BW-1:0] reclaim_block, reclaim_next;
    reg [BW:0] reclaim_left;
    reg cancel_pending;
    reg [BW-1:0] cancel_first;
    reg [BW:0] cancel_blocks;
    reg [MDW-1:0] next_msg_raw, next_msg_forward;
    reg next_msg_bypass;
    wire [MDW-1:0] next_msg_q = next_msg_bypass ? next_msg_forward : next_msg_raw;
    wire release_block;
    reg [BW:0] free_blocks, free_msgs;
    reg wr_active, wr_bad;
    reg [MDW-1:0] wr_msg;
    reg [BW-1:0] wr_first, wr_block;
    reg [BW:0] wr_blocks;
    // Cache the increment before admission. block_candidate_valid remains
    // low for four clocks after allocation; commit/cancel also block the
    // next message. Thus wr_blocks has settled for at least one clock when
    // another allocation can occur, and this lookahead is always current.
    // The handshake now selects two registers, with no carry chain after it.
    reg [BW:0] wr_blocks_plus1;
    always @(posedge clk) wr_blocks_plus1 <= wr_blocks + 1'b1;
    wire alloc_block;
    wire [BW:0] wr_blocks_next = alloc_block ? wr_blocks_plus1 : wr_blocks;
    // No maxfan: the count now feeds only its incrementer, the payload
    // address and commit_len, and counters ignore it (warning 17031).
    reg [SLOT_AW:0] wr_count;
    reg commit, cancel;
    wire wr_index_ok = wr_vl < N_VL;
    wire rd_index_ok = rd_vl < N_VL;
    reg [VLW-1:0] admission_vl;
    reg admission_quota_ok, admission_match_q, commit_delayed, commit_delayed2;
    // A state write blocks the count lookup for one clock. A pop only lowers
    // a count, and a commit to another VL leaves writer_vl_q unchanged, so
    // the last result still holds; only a commit to this VL needs a new
    // lookup. Otherwise a delivery pop could close admission just as the RX
    // parser, which cannot wait, delivers its first payload byte.
    reg writer_intact;
    // Tag the synchronous count lookup; a changed VL waits for its own result.
    // The commit guard covers the RAM write and its following lookup cycle.
    // A second guard clock keeps commits four clocks apart, so the writer
    // lookup on the clock before a commit is always current (see
    // commit_plus_q below).
    always @(posedge clk) begin
        if (rst) begin
            admission_vl <= 0; admission_quota_ok <= 0; admission_match_q <= 0;
            commit_delayed <= 0; commit_delayed2 <= 0;
        end else begin
            // Prefetch during the header/idle interval too: the RX parser
            // requires ready on its first payload byte. Ignore invalid VLs.
            admission_vl <= writer_vl_q;
            admission_match_q <= writer_vl_q == wr_vl;
            if (writer_lookup_valid && writer_vl_q < N_VL)
                admission_quota_ok <= writer_count < DEPTH;
            else if (!writer_intact)
                admission_quota_ok <= 0;
            commit_delayed <= commit;
            commit_delayed2 <= commit_delayed;
        end
    end
    // Count properties are registered with wr_count, keeping the constant
    // comparisons out of the write handshake.
    reg wr_at_max, wr_block_edge;
    wire need_block = !wr_active || wr_block_edge;
    // Non-pipelined producers may change wr_vl while idle. Require both the
    // live tag comparison and the registered lookup match: admission_vl alone
    // cannot distinguish an X-Y-X tag sequence and could pair X with Y's RAM
    // result in the one-clock-early commit calculation. The RX parser holds
    // wr_vl throughout its header pipeline, so its registered-only mode keeps
    // the short ready feedback path.
    // EXT_VL_SAME: match_q covers writer_vl_q == wr_vl(t-1), wr_vl_same covers
    // wr_vl(t) == wr_vl(t-1); together they equal the live comparison.
    wire admission_match = (PIPE_ADMISSION_MATCH != 0) ? admission_match_q :
                           (EXT_VL_SAME != 0) ? (admission_match_q && wr_vl_same) :
                           (admission_match_q && admission_vl == wr_vl);
    // 超长输入继续接收至 commit/abort，不能在越界字节上永久反压。
    // Producers hold wr_vl for a whole message, so continuation beats are not
    // re-qualified by VL; a new message still waits for its own VL lookup.
    // wr_hold 复位期间为 1，此外与 commit || cancel 同值，是只供握手使用的独立寄存器
    // （commit 与全局复位本身扇出很大）
    (* preserve *) reg wr_hold;
    assign wr_ready = !init_active && (EXT_VL_SAME != 0 || wr_index_ok) && !cancel_pending && !wr_hold &&
        (wr_active ? (wr_bad || wr_at_max || !need_block || block_candidate_valid) :
                     (block_candidate_valid && msg_candidate_valid && !commit_delayed &&
                      !commit_delayed2 && admission_match && admission_quota_ok));
    wire beat = wr_valid && wr_ready;
    wire over = beat && wr_at_max;
    assign alloc_block = beat && need_block && !wr_abort && !wr_bad && !wr_at_max;
    wire alloc_msg = alloc_block && !wr_active;
    // Address selection does not depend on the admission handshake. The
    // registered payload enable separately qualifies whether it is written.
    wire [BW-1:0] write_block = need_block ? free_block_head : wr_block;
    wire [MDW-1:0] write_msg = wr_active ? wr_msg : free_msg_head;
    wire [SLOT_AW:0] count_plus1 = wr_count + 1'b1;
    wire [SLOT_AW-1:0] new_count = beat ? count_plus1[SLOT_AW-1:0] : wr_count[SLOT_AW-1:0];
    wire finish_write = wr_commit && (!wr_valid || wr_ready);
    // finish_write already qualifies an asserted wr_valid with wr_ready.
    // With no active writer, that accepted byte allocates the first block
    // and descriptor. Do not route the same handshake through alloc_msg again.
    wire commit_now = !rst && finish_write && !wr_abort && (wr_active || wr_valid) &&
                  !wr_bad && !(wr_valid && wr_at_max);
    // wr_active implies at least one stored byte; alloc_msg stores the first.
    // MAX_LEN/over prevents count wrap, so commit never needs an adder-zero test.
    wire cancel_now = !rst && wr_active &&
                      (wr_abort || (finish_write && (wr_bad || (wr_valid && wr_at_max))));
    wire hold_now = (wr_active && wr_abort) ||
                    (finish_write && (wr_active || (wr_valid && !wr_abort && !wr_bad && !wr_at_max)));
    // Retire a complete message one cycle after admission. Snapshot data is
    // sampled without an enable so the admission cone only drives event flops.
    reg [VLW-1:0] commit_vl;
    reg [MDW-1:0] retire_msg;
    reg [BW-1:0] retire_first;
    reg [BW:0] retire_blocks;
    reg [SLOT_AW-1:0] commit_len;
    reg [MW-1:0] commit_meta;
    always @(posedge clk) begin
        if (rst) begin commit <= 0; cancel <= 0; wr_hold <= 1; end
        else begin commit <= commit_now; cancel <= cancel_now; wr_hold <= hold_now; end
        commit_vl <= wr_vl;
        retire_msg <= write_msg;
        retire_first <= alloc_msg ? write_block : wr_first;
        retire_blocks <= wr_blocks_next;
        commit_len <= new_count;
        commit_meta <= wr_meta;
    end
    // An accepted pop executes on the next clock from a register, so the
    // consumer handshake does not fan out to every pop consequence. The
    // consumer has left its release state by then and still holds rd_vl.
    wire pop_accept = !rst && rd_pop && rd_pop_ready;
    reg pop;
    always @(posedge clk) begin
        if (rst) pop <= 1'b0;
        else pop <= pop_accept;
    end
    wire [MDW-1:0] pop_msg = head_q;
    // Registered VL match: rd_vl is held while a pop can be issued.
    reg commit_rd_match;
    always @(posedge clk) commit_rd_match <= wr_vl == rd_vl;
    wire same_vl = commit && pop && commit_rd_match;
    // State written on this clock by a pop: a pop without a same-VL commit,
    // or a pop deferred behind a commit to another VL.
    wire pop_write = (pop && !same_vl) || pending_pop;
    wire [VLW-1:0] pop_write_vl = pending_pop ? pending_pop_vl : rd_vl;
    always @(posedge clk) begin
        if (rst) begin
            writer_hit_commit <= 1'b0; writer_hit_pop <= 1'b0;
            reader_hit_commit <= 1'b0; reader_hit_pop <= 1'b0;
        end else begin
            writer_hit_commit <= commit && commit_vl == writer_vl_q;
            writer_hit_pop <= pop_write && pop_write_vl == writer_vl_q;
            reader_hit_commit <= commit && commit_vl == reader_vl_q;
            reader_hit_pop <= pop_write && pop_write_vl == reader_vl_q;
        end
    end
    // Commit and pop results are formed one clock ahead, so the state RAM
    // write data is selected from registers and no path runs from a state
    // RAM read port through the +/-1 logic back to its write port.
    // - A pop is accepted on a clock without a state write, and rd_vl is held
    //   (see header), so the reader state on the accepting clock is the
    //   state that the pop updates on the next clock.
    // - Commits are at least four clocks apart (commit_delayed2), so the
    //   writer state on the clock before a commit is current. Only a pop on
    //   that clock can change it; writer_hit_pop then selects the "count
    //   unchanged" result, which is also the same-VL commit+pop result.
    reg [STATE_W-1:0] commit_plus_q, commit_keep_q, pop_state_q;
    reg writer_zero_q, writer_one_q, writer_two_q, reader_last_q;
    always @(posedge clk) begin
        commit_plus_q <= {writer_count + 1'b1, (writer_count == 0) ? 1'b0 : writer_state[0]};
        commit_keep_q <= {writer_count, writer_count != 1};
        writer_zero_q <= writer_count == 0;
        writer_one_q <= writer_count == 1;
        writer_two_q <= writer_count == 2;
        pop_state_q <= {reader_count - 1'b1, 1'b1};
        reader_last_q <= reader_count <= 1;
    end
    // Count of the committing VL on the commit clock is zero/one.
    wire commit_empty = writer_hit_pop ? writer_one_q : writer_zero_q;
    wire commit_single = writer_hit_pop ? writer_two_q : writer_one_q;
    // The +/-1 values are formed from the registers; the handshakes only select.
    wire [BW:0] free_blocks_up = free_blocks + 1'b1, free_blocks_down = free_blocks - 1'b1;
    wire [BW:0] next_free_blocks = (alloc_block == release_block) ? free_blocks :
                                   alloc_block ? free_blocks_down : free_blocks_up;
    wire [BW:0] free_msgs_up = free_msgs + 1'b1, free_msgs_up2 = free_msgs_up + 1'b1;
    wire [BW:0] free_msgs_down = free_msgs - 1'b1;
    wire [1:0] msgs_released = {pop && cancel, pop ^ cancel};
    wire [BW:0] next_free_msgs = alloc_msg ? (msgs_released[1] ? free_msgs_up :
                                              msgs_released[0] ? free_msgs : free_msgs_down) :
                                             (msgs_released[1] ? free_msgs_up2 :
                                              msgs_released[0] ? free_msgs_up : free_msgs);
    assign pool_used = BLOCKS_COUNT - free_blocks;
    assign pool_msgs = MSGS_COUNT - free_msgs;
    // Isolate admission/allocation logic from the high-fanout physical RAM
    // address and byte-enable nets. Metadata becomes readable only after its
    // synchronous lookup, by which time the last payload write has completed.
    reg payload_we;
    (* preserve, maxfan = 16 *) reg [AW-1:0] payload_addr;
    reg [7:0] payload_data;
    always @(posedge clk) begin
        if (rst) payload_we <= 0;
        else payload_we <= beat && !wr_abort && !wr_bad && !over;
        payload_addr <= {write_block, wr_count[BLOCK_AW-1:0]};
        payload_data <= wr_data;
    end

    integer i;
    always @(posedge clk) begin
        if (rst) begin

            free_blocks <= BLOCKS_COUNT; free_msgs <= MSGS_COUNT;
            wr_active <= 0; wr_bad <= 0; wr_msg <= 0;
            wr_first <= 0; wr_block <= 0; wr_blocks <= 0; wr_count <= 0; wr_ovf <= 0;
            wr_at_max <= 0; wr_block_edge <= 1;
            pool_peak <= 0; pool_msgs_peak <= 0;
        end else begin
            wr_ovf <= 0;
            free_blocks <= next_free_blocks; free_msgs <= next_free_msgs;
            // Peaks follow the registered occupancy one clock later.
            if (pool_used > pool_peak) pool_peak <= pool_used;
            if (pool_msgs > pool_msgs_peak) pool_msgs_peak <= pool_msgs;


            // These fields are only consumed while active. Capture them on
            // every idle cycle, so their enable is independent of admission.
            if (!wr_active) begin
                wr_msg <= free_msg_head; wr_first <= free_block_head;
            end
            if (alloc_msg) wr_active <= 1;
            if (alloc_block) wr_block <= write_block;
            wr_blocks <= wr_blocks_next;
            if (beat && !wr_bad) begin
                if (over) begin wr_bad <= 1; wr_ovf <= 1; end
                else begin
                    wr_count <= count_plus1;
                    wr_at_max <= count_plus1 == MAX_LEN;
                    wr_block_edge <= count_plus1[BLOCK_AW-1:0] == 0;
                end
            end
            if (wr_abort || finish_write) begin
                wr_active <= 0; wr_bad <= 0; wr_count <= 0; wr_blocks <= 0;
                wr_at_max <= 0; wr_block_edge <= 1;
            end

        end
    end

    // RAM output tags invalidate stale data immediately when VL/head changes.
    // The head pointer (a mux of two RAM outputs) is registered before it
    // addresses the descriptor/next_msg RAMs; head_changed clears every stage.
    reg [MDW-1:0] head_r;
    reg [VLW-1:0] head_vl_r;
    reg head_valid_r;
    wire [MDW-1:0] rd_msg_c = head_r;
    // The head-pointer write executes one clock after the commit decision.
    // Invalidate both lookup stages until the new pointer and descriptor have
    // reached their read ports, including the pending write clock.
    wire head_changed = pop || (commit && commit_vl == rd_vl) ||
                        (first_head_we && first_head_write_vl == rd_vl);
    always @(posedge clk) begin
        if (rst) first_head_we <= 1'b0;
        else first_head_we <= commit && (commit_empty || (same_vl && commit_single));
        first_head_write_vl <= commit_vl;
        first_head_write_msg <= retire_msg;
        if (commit) begin
            tail[commit_vl] <= retire_msg;
        end
        if (first_head_we) first_head[first_head_write_vl] <= first_head_write_msg;
        if (pop) popped_head[rd_vl] <= next_msg_q;
        first_head_q <= first_head[rd_vl];
        popped_head_q <= popped_head[rd_vl];
        head_from_pop_q <= reader_state[0];
        // commit_vl is delayed by the same cycle as this tail lookup.
        tail_q <= tail[wr_vl];
        head_vl <= rd_vl;
        if (rst) head_valid <= 0;
        else head_valid <= rd_index_ok && reader_lookup_valid && reader_vl_q == rd_vl &&
                           reader_count != 0 && !head_changed;
    end
    reg [DESC_W-1:0] desc_q;
    reg [VLW-1:0] desc_vl;
    reg desc_valid;
    wire [BW-1:0] desc_first;
    wire [BW:0] desc_blocks;
    // The descriptor RAM output is available well before a streamed message
    // can be popped. Cache the recycler fields separately so the RAM output
    // does not feed the reclaim-state mux and counter in the same cycle.
    reg [BW-1:0] reclaim_desc_first;
    reg [BW:0] reclaim_desc_blocks;
    reg msg_link_write;
    assign {desc_first, desc_blocks, rd_len, rd_meta} = desc_q;
    assign rd_info_valid = desc_valid && (RD_INFO_UNTAGGED != 0 || desc_vl == rd_vl);
    // A just-appended link is written on the next edge. Hold pop during
    // that edge so the RAM read/forwarding result is available to the head;
    // a commit now would append such a link on the executing clock.
    // Consumers pop only after streaming the message, with rd_vl unchanged
    // for many clocks, so readiness omits the rd_vl tag comparison.
    assign rd_pop_ready = desc_valid && !reclaim_busy && !msg_link_write && !pending_pop &&
                          !pop && !commit;
    wire link_msg = commit && !commit_empty && !(same_vl && commit_single);
    reg [MDW-1:0] msg_link_from, msg_link_to;
    always @(posedge clk) begin
        msg_link_write <= !rst && link_msg;
        msg_link_from <= tail_q;
        msg_link_to <= retire_msg;
        if (msg_link_write) next_msg[msg_link_from] <= msg_link_to;
    end
    reg block_link_write;
    reg [BW-1:0] block_link_from, block_link_to;
    always @(posedge clk) begin
        block_link_write <= !rst && alloc_block && wr_active;
        block_link_from <= wr_block;
        block_link_to <= write_block;
        if (block_link_write) next_block[block_link_from] <= block_link_to;
    end
    always @(posedge clk) begin
        if (commit) descriptor[retire_msg] <= {retire_first, retire_blocks, commit_len, commit_meta};
        desc_q <= descriptor[rd_msg_c];
        // Forward the physical write collision. Pop waits for this registered
        // result, so no long current-append selection feeds the data register.
        next_msg_raw <= next_msg[rd_msg_c];
        next_msg_bypass <= msg_link_write && msg_link_from == rd_msg_c;
        next_msg_forward <= msg_link_to;
        head_r <= head_q; head_vl_r <= head_vl;
        if (rst) head_valid_r <= 0;
        else head_valid_r <= head_valid && head_vl == rd_vl && !head_changed;
        desc_vl <= head_vl_r;
        if (rst) desc_valid <= 0;
        else desc_valid <= head_valid_r && head_vl_r == rd_vl && !head_changed;
        if (rd_info_valid || pop) begin
            reclaim_desc_first <= desc_first;
            reclaim_desc_blocks <= desc_blocks;
        end
    end

    // Registered allocation candidates isolate the priority tree from RAM writes.
    // A just-allocated candidate is invalid for a cycle until the tree catches up.
    always @(posedge clk) begin
        if (rst) begin
            free_block_head <= 0; free_msg_head <= 0;
            block_candidate_valid <= 0; msg_candidate_valid <= 0; block_alloc_delayed <= 0; block_alloc_delayed2 <= 0; block_alloc_delayed3 <= 0;
            msg_alloc_delayed <= 0; msg_alloc_delayed2 <= 0; msg_alloc_delayed3 <= 0;
        end else begin
            free_block_head <= block_pick; free_msg_head <= msg_pick;
            block_candidate_valid <= block_found_q && !alloc_block && !block_alloc_delayed && !block_alloc_delayed2 && !block_alloc_delayed3;
            block_alloc_delayed <= alloc_block;
            block_alloc_delayed2 <= block_alloc_delayed;
            block_alloc_delayed3 <= block_alloc_delayed2;
            msg_alloc_delayed <= alloc_msg;
            msg_alloc_delayed2 <= msg_alloc_delayed;
            msg_alloc_delayed3 <= msg_alloc_delayed2;
            msg_candidate_valid <= msg_found_q && !alloc_msg && !msg_alloc_delayed && !msg_alloc_delayed2 && !msg_alloc_delayed3;
        end
    end
    // Both commit results are formed before the pop handshake selects one.
    wire [STATE_W-1:0] commit_state_next = (same_vl || writer_hit_pop) ? commit_keep_q : commit_plus_q;
    // A same-VL commit/pop writes the combined state. For different VLs,
    // write the commit now and retire the pop on the next clock. The pending
    // pop backpressures further pops; a commit cannot follow on that clock
    // because wr_ready is low while commit is asserted. Both RAM copies thus
    // have exactly one address operation per clock and no read/write collision.
    // 初始化期间写入被阻塞、队列为空，因此不会与 commit/pop 冲突。
    wire state_write = !rst && (init_active || commit || pending_pop || pop);
    wire [VLW-1:0] state_write_vl = init_active ? init_vl : commit ? commit_vl :
                                     pending_pop ? pending_pop_vl : rd_vl;
    always @(posedge clk)
        writer_intact <= state_write && !init_active && !(commit && commit_vl == writer_vl_q);
    wire [STATE_W-1:0] state_write_data = init_active ? {STATE_W{1'b0}} : commit ? commit_state_next :
                                           pending_pop ? pending_pop_state : pop_state_q;
    // Both modes sweep the state RAMs to zero after reset, so every entry is
    // initialized and the lookups need no nonempty mask (in vector mode that
    // was an N_VL:1 multiplexer on wr_vl and rd_vl).
    always @(posedge clk) begin
        if (rst) begin init_active <= 1'b1; init_vl <= {VLW{1'b0}}; end
        else if (init_active) begin
            init_vl <= init_vl + 1'b1;
            if (init_vl == N_VL-1) init_active <= 1'b0;
        end
    end
    generate if (NONEMPTY_RAM != 0) begin : g_init_sweep
        // This delayed bit is needed only by the RAM-backed nonempty mode.
        // Keeping it in this generated scope avoids unused-register warnings
        // in the vector-backed queue specializations.
        reg pending_pop_nonzero;
        always @(posedge clk)
            if (commit && pop && !same_vl)
                pending_pop_nonzero <= !reader_last_q;
        // Nonempty after the state write: a commit always leaves a message.
        wire ne_write_data = !init_active && (commit ||
            (pending_pop ? pending_pop_nonzero : !reader_last_q));
        // 每个 lane 保存 VL[3:0]==lane 的非空位；状态 RAM 每拍至多一次写，
        // 因此纯写即可维护，无需读-改-写。
        genvar ln;
        for (ln = 0; ln < 16; ln = ln + 1) begin : g_ne_lane
            (* ramstyle = "M4K, no_rw_check" *) reg ne_mem [0:(1<<NE_GW)-1];
            wire [NE_GW-1:0] waddr;
            if (VLW > 4) begin : g_wide_addr
                assign waddr = state_write_vl[VLW-1:4];
            end else begin : g_narrow_addr
                assign waddr = {NE_GW{1'b0}};
            end
            always @(posedge clk) begin
                if (state_write && state_write_vl % 16 == ln)
                    ne_mem[waddr] <= ne_write_data;
                ne_bits[ln] <= ne_mem[ne_group];
            end
        end
    end else begin : g_no_init_sweep
        always @(posedge clk) ne_bits <= 16'd0;
    end endgenerate
    always @(posedge clk) begin
        if (state_write) writer_vl_state[state_write_vl] <= state_write_data;
        else if (wr_index_ok) writer_state_q <= writer_vl_state[wr_vl];
    end
    always @(posedge clk) begin
        if (state_write) reader_vl_state[state_write_vl] <= state_write_data;
        else if (rd_index_ok) reader_state_q <= reader_vl_state[rd_vl];
    end
    always @(posedge clk) begin
        if (!state_write) begin
            if (wr_index_ok) writer_vl_q <= wr_vl;
            if (rd_index_ok) reader_vl_q <= rd_vl;
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            writer_lookup_valid <= 0;
            reader_lookup_valid <= 0;
            pending_pop <= 0;
        end else begin
            writer_lookup_valid <= !state_write && wr_index_ok;
            reader_lookup_valid <= !state_write && rd_index_ok;
            pending_pop <= commit && pop && !same_vl;
            if (commit && pop && !same_vl) begin
                pending_pop_vl <= rd_vl;
                pending_pop_state <= pop_state_q;
            end
        end
    end
    // An emptying pop clears its nonempty bit one clock later, so the N_VL
    // decoder is driven from registers. A later commit to that VL wins.
    // Lookups are blocked on the pop's state write, and consumers wait for
    // the queue before choosing again, so none observes the delayed bit.
    reg ne_clear;
    reg [VLW-1:0] ne_clear_vl;
    always @(posedge clk) begin
        if (rst) begin
            nonempty <= 0;
            ne_clear <= 0;
        end else begin
            ne_clear <= pop && !same_vl && reader_last_q;
            ne_clear_vl <= rd_vl;
            if (ne_clear && NONEMPTY_RAM == 0) nonempty[ne_clear_vl] <= 1'b0;
            if (commit) begin
                last_commit_state <= commit_state_next;
                if (NONEMPTY_RAM == 0) nonempty[commit_vl] <= 1'b1;
            end
            if (pop_write)
                last_pop_state <= pending_pop ? pending_pop_state : pop_state_q;
        end
    end
    // Releases reach the descriptor bitmap one clock later. Freed slots are
    // only reused later, and the pop/cancel handshakes no longer fan out to
    // every bitmap bit.
    reg release_pop, release_cancel;
    reg [MDW-1:0] release_pop_msg, release_cancel_msg;
    always @(posedge clk) begin
        if (rst) begin release_pop <= 0; release_cancel <= 0; end
        else begin release_pop <= pop; release_cancel <= cancel; end
        release_pop_msg <= pop_msg;
        release_cancel_msg <= retire_msg;
    end
    genvar b, m;
    generate
        for (b = 0; b < BLOCKS; b = b+1) begin : g_block_free
            localparam [BW-1:0] INDEX = b[BW-1:0];
            always @(posedge clk) begin
                if (rst) block_free[b] <= 1;
                else if (block_alloc_delayed && allocated_block == INDEX) block_free[b] <= 0;
                else if (release_block && reclaim_block == INDEX) block_free[b] <= 1;
            end
        end
        for (m = 0; m < MSGS; m = m+1) begin : g_msg_free
            localparam [MDW-1:0] INDEX = m[MDW-1:0];
            always @(posedge clk) begin
                if (rst) msg_free[m] <= 1;
                else if (msg_alloc_delayed && allocated_msg == INDEX) msg_free[m] <= 0;
                else if ((release_pop && release_pop_msg == INDEX) ||
                         (release_cancel && release_cancel_msg == INDEX)) msg_free[m] <= 1;
            end
        end
    endgenerate

    // A pop can start only while the recycler is idle. An independent writer
    // can abort during that pop chain; one pending slot preserves both returns.
    // No new write starts until that pending abort has entered the recycler.
    assign reclaim_busy = reclaim_state != 0 || cancel_pending || cancel;
    assign release_block = !rst && reclaim_state == 2;
    always @(posedge clk) reclaim_next <= next_block[reclaim_block];
    always @(posedge clk) begin
        if (rst) begin
            reclaim_state <= 0; reclaim_block <= 0; reclaim_left <= 0;
            cancel_pending <= 0; cancel_first <= 0; cancel_blocks <= 0;
        end else begin
            case (reclaim_state)
                0: begin
                    if (pop) begin
                        // The descriptor fields are captured above on this
                        // edge. State 3 consumes only those registers, keeping
                        // the RAM output off the reclaim_left timing path.
                        reclaim_state <= 3;
                    end else if (cancel_pending) begin
                        reclaim_block <= cancel_first; reclaim_left <= cancel_blocks;
                        cancel_pending <= 0; reclaim_state <= 1;
                    end else if (cancel) begin
                        reclaim_block <= retire_first; reclaim_left <= retire_blocks; reclaim_state <= 1;
                    end
                end
                1: reclaim_state <= 2; // synchronous link read
                2: begin
                    if (reclaim_left == 1) reclaim_state <= 0;
                    else begin
                        reclaim_block <= reclaim_next; reclaim_left <= reclaim_left-1'b1; reclaim_state <= 1;
                    end
                end
                3: begin
                    reclaim_block <= reclaim_desc_first;
                    reclaim_left <= reclaim_desc_blocks;
                    reclaim_state <= 1;
                end
                default: reclaim_state <= 0;
            endcase
            if (cancel && (reclaim_state != 0 || pop || cancel_pending)) begin
                cancel_pending <= 1; cancel_first <= retire_first; cancel_blocks <= retire_blocks;
            end
        end
    end

    reg rd_active;
    // An accepted read start takes effect on the next clock from a register;
    // the head descriptor cannot change while its message is unread.
    reg read_start;
    always @(posedge clk) begin
        if (rst) read_start <= 1'b0;
        else read_start <= rd_start && rd_info_valid;
    end
    (* preserve *) reg [BW-1:0] rd_block;
    reg [BW-1:0] rd_next;
    (* preserve *) reg [BLOCK_AW-1:0] rd_off;
    reg [SLOT_AW-1:0] rd_pos, rd_len_l;
    wire [7:0] payload_q;
    wire [AW-1:0] read_address = {rd_block,rd_off};
    reg [PAYLOAD_BANK_W-1:0] read_bank, read_bank_pipe;
    wire [7:0] bank_data [0:PAYLOAD_BANKS-1];
    reg read_pending, last_pending;
    reg payload_valid, payload_last;
    reg [8:0] output_fifo [0:3];
    reg [1:0] output_rptr, output_wptr;
    reg [2:0] output_count, read_occupancy;
    assign rd_valid = output_count != 0;
    assign {rd_last,rd_data} = output_fifo[output_rptr];
    wire output_pop = rd_valid && rd_ready;
    wire transfer = payload_valid;
    // Reserve space for an outstanding RAM read. Read enable depends only on
    // registered occupancy, isolating downstream ready from the physical RAM.
    wire advance = rd_active && read_occupancy < 4;
    always @(posedge clk) rd_next <= next_block[rd_block];
    // Local bank output registers keep the RAM-to-global-mux route out of a
    // single cycle. Banking changes neither byte capacity nor block addresses.
    genvar pb;
    generate for (pb=0; pb<PAYLOAD_BANKS; pb=pb+1) begin : g_payload_bank
        // Keep the M4K preference and permit undefined same-address reads;
        // allocation prevents those reads from being consumed.
        (* ramstyle = "M4K, no_rw_check" *) reg [7:0] bytes [0:(1<<PAYLOAD_BANK_AW)-1];
        reg [7:0] raw_data;
        (* preserve *) reg [7:0] registered_data;
        (* preserve, maxfan = 8 *) reg bank_write;
        (* preserve *) reg [PAYLOAD_BANK_AW-1:0] bank_write_address;
        reg [7:0] bank_write_data;
        always @(posedge clk) begin
            bank_write <= !rst && payload_we && (payload_addr >> PAYLOAD_BANK_AW)==pb;
            bank_write_address <= payload_addr[PAYLOAD_BANK_AW-1:0];
            bank_write_data <= payload_data;
        end
        // Keep both ports in one clocked process so synthesis recognizes a
        // common clock. Allocation prevents a live read from using a block
        // which is being written.
        always @(posedge clk) begin
            if (bank_write) bytes[bank_write_address] <= bank_write_data;
            // The validity pipeline, not this RAM output, determines whether
            // the fetched byte may enter the FIFO. An unconditional read keeps
            // both RAM ports on the same ungated clock in legacy synthesis.
            raw_data <= bytes[read_address[PAYLOAD_BANK_AW-1:0]];
        end
        always @(posedge clk) registered_data <= raw_data;
        assign bank_data[pb] = registered_data;
    end endgenerate
    assign payload_q = bank_data[read_bank_pipe];
    generate if (PAYLOAD_BANKS == 1) begin : g_single_read_bank
        always @(posedge clk) read_bank <= {PAYLOAD_BANK_W{1'b0}};
    end else begin : g_multiple_read_banks
        always @(posedge clk)
            if (advance) read_bank <= read_address[AW-1:PAYLOAD_BANK_AW];
    end endgenerate
    always @(posedge clk) read_bank_pipe <= read_bank;
    always @(posedge clk) if (transfer) output_fifo[output_wptr] <= {payload_last,payload_q};
    always @(posedge clk) begin
        if (rst) begin
            rd_active<=0; payload_valid<=0; payload_last<=0; read_pending<=0; last_pending<=0;
            output_rptr<=0; output_wptr<=0; output_count<=0; read_occupancy<=0;
            rd_block<=0; rd_off<=0; rd_pos<=0; rd_len_l<=0;
        end else if (read_start) begin
            rd_active<=1; payload_valid<=0; read_pending<=0;
            output_rptr<=0; output_wptr<=0; output_count<=0; read_occupancy<=0;
            rd_block<=desc_first; rd_off<=0; rd_pos<=0; rd_len_l<=rd_len;
        end else begin
            if (output_pop) output_rptr <= output_rptr+1'b1;
            if (transfer) output_wptr <= output_wptr+1'b1;
            case ({transfer,output_pop})
                2'b10: output_count <= output_count+1'b1;
                2'b01: output_count <= output_count-1'b1;
                default: ;
            endcase
            case ({advance,output_pop})
                2'b10: read_occupancy <= read_occupancy+1'b1;
                2'b01: read_occupancy <= read_occupancy-1'b1;
                default: ;
            endcase
            read_pending <= advance;
            payload_valid <= read_pending;
            payload_last <= last_pending;
            if (advance) begin
                last_pending<=rd_pos==rd_len_l-1'b1;
                rd_pos<=rd_pos+1'b1; rd_off<=rd_off+1'b1;
                if (&rd_off) rd_block<=rd_next;
                if (rd_pos==rd_len_l-1'b1) rd_active<=0;
            end
        end
    end
endmodule
