`timescale 1ns/1ps

// Random traffic on a small multi-VL queue: per-VL FIFO order, payload,
// length and metadata against a model, quota and pool back-pressure, aborts,
// and commit/pop on the same clock. Every commit and pop also checks the
// state results formed one clock early against the direct formulas on the
// looked-up state of that clock.
module tb_afdx_queue_stress;
    `include "tb_common.svh"

    parameter N_VL = 8;
    parameter VLW = 3;
    parameter NONEMPTY_RAM = 0;
    localparam DEPTH = 4, DW = 2, SLOT_AW = 6, POOL_AW = 3, BLOCK_AW = 4;
    localparam STATE_W = DW + 2;
    localparam MAX_LEN = (1 << SLOT_AW) - 1;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    reg [VLW-1:0] wr_vl = 0, rd_vl = 0;
    reg wr_valid = 0, wr_commit = 0, wr_abort = 0;
    reg [7:0] wr_data = 0;
    reg [8:0] wr_meta = 0;
    wire wr_ready, wr_ovf;
    reg rd_start = 0, rd_pop = 0, rd_ready = 0;
    wire rd_info_valid, rd_pop_ready, reclaim_busy, init_busy;
    wire [SLOT_AW-1:0] rd_len;
    wire [8:0] rd_meta;
    wire [7:0] rd_data;
    wire rd_valid, rd_last;
    wire [N_VL-1:0] nonempty;
    wire [15:0] ne_bits;
    reg [(VLW>4 ? VLW-4 : 1)-1:0] ne_group = 0;
    wire [POOL_AW+SLOT_AW-BLOCK_AW:0] pool_used, pool_peak, pool_msgs, pool_msgs_peak;

    afdx_msg_queue #(.N_VL(N_VL), .VLW(VLW), .DEPTH(DEPTH), .DW(DW), .SLOT_AW(SLOT_AW), .MW(9),
                     .POOL_AW(POOL_AW), .BLOCK_AW(BLOCK_AW), .MAX_MSGS(16),
                     .NONEMPTY_RAM(NONEMPTY_RAM)) dut (
        .clk(clk), .rst(rst), .wr_vl(wr_vl), .wr_valid(wr_valid), .wr_data(wr_data),
        .wr_ready(wr_ready), .wr_commit(wr_commit), .wr_meta(wr_meta), .wr_abort(wr_abort),
        .wr_ovf(wr_ovf), .rd_vl(rd_vl), .rd_start(rd_start), .rd_pop(rd_pop),
        .rd_info_valid(rd_info_valid), .reclaim_busy(reclaim_busy), .rd_pop_ready(rd_pop_ready),
        .rd_len(rd_len), .rd_meta(rd_meta), .rd_data(rd_data), .rd_valid(rd_valid),
        .rd_last(rd_last), .rd_ready(rd_ready), .nonempty(nonempty), .pool_used(pool_used),
        .pool_peak(pool_peak), .pool_msgs(pool_msgs), .pool_msgs_peak(pool_msgs_peak),
        .ne_group(ne_group), .ne_bits(ne_bits), .init_busy(init_busy),
        .wr_commit_ok ());

    // ---------------- model ----------------
    localparam QMAX = 64;
    integer q_len [0:N_VL-1][0:QMAX-1];
    integer q_seed [0:N_VL-1][0:QMAX-1];
    integer q_head [0:N_VL-1];
    integer q_tail [0:N_VL-1];
    integer committed = 0, aborted = 0, delivered = 0, same_clock = 0, adjusted = 0;
    integer data_errors = 0, formula_errors = 0, spacing_errors = 0;
    integer cycle = 0, last_commit_cycle = -100;
    function automatic [7:0] pattern(input integer seed, input integer position);
        pattern = (seed * 13 + position * 7 + (position >> 2)) & 8'hff;
    endfunction
    function automatic integer queued(input integer vl);
        queued = q_tail[vl] - q_head[vl];
    endfunction

    // ---------------- one-clock-early state results ----------------
    reg [STATE_W-1:0] direct_state;
    reg [DW:0] direct_count;
    always @(posedge clk) begin
        cycle <= cycle + 1;
        if (!rst && dut.commit === 1'b1) begin
            if (cycle - last_commit_cycle < 4) spacing_errors = spacing_errors + 1;
            last_commit_cycle = cycle;
            direct_count = dut.writer_count;
            direct_state = dut.same_vl ? {direct_count, direct_count != 1} :
                           {direct_count + 1'b1, (direct_count == 0) ? 1'b0 : dut.writer_state[0]};
            if (dut.commit_state_next !== direct_state || dut.commit_empty !== (direct_count == 0) ||
                dut.commit_single !== (direct_count == 1)) begin
                formula_errors = formula_errors + 1;
                $display("commit formula mismatch t=%0d vl=%0d state=%b expected=%b same=%b hitpop=%b",
                         cycle, dut.commit_vl, dut.commit_state_next, direct_state, dut.same_vl,
                         dut.writer_hit_pop);
            end
            if (dut.same_vl) same_clock = same_clock + 1;
            if (dut.writer_hit_pop) adjusted = adjusted + 1;
        end
        if (!rst && dut.pop === 1'b1) begin
            direct_count = dut.reader_count;
            if (dut.pop_state_q !== {direct_count - 1'b1, 1'b1} ||
                dut.reader_last_q !== (direct_count <= 1)) begin
                formula_errors = formula_errors + 1;
                $display("pop formula mismatch t=%0d vl=%0d state=%b count=%0d",
                         cycle, rd_vl, dut.pop_state_q, direct_count);
            end
        end
    end

    // ---------------- writer ----------------
    integer seed_counter = 1;
    task automatic write_message(input integer vl, input integer len, input bit abort_it,
                                 input integer gap_pct);
        integer i, seed, timeout;
        begin
            seed = seed_counter; seed_counter = seed_counter + 1;
            for (i = 0; i < len; i = i + 1) begin
                @(negedge clk);
                while ($urandom_range(0, 99) < gap_pct) begin
                    wr_valid = 0; wr_commit = 0; wr_abort = 0; @(negedge clk);
                end
                wr_vl = vl; wr_valid = 1; wr_data = pattern(seed, i);
                wr_meta = seed & 9'h1ff;
                wr_commit = (i == len-1) && !abort_it;
                wr_abort = (i == len-1) && abort_it;
                #1;
                timeout = 0;
                while (!wr_ready && timeout < 5000) begin @(negedge clk); #1; timeout = timeout + 1; end
                if (timeout >= 5000) $fatal(1, "writer stalled vl=%0d", vl);
                if (i == len-1) begin
                    if (abort_it) aborted = aborted + 1;
                    else begin
                        q_len[vl][q_tail[vl] % QMAX] = len;
                        q_seed[vl][q_tail[vl] % QMAX] = seed;
                        q_tail[vl] = q_tail[vl] + 1;
                        committed = committed + 1;
                    end
                end
                @(posedge clk);
            end
            @(negedge clk);
            wr_valid = 0; wr_commit = 0; wr_abort = 0;
        end
    endtask

    // ---------------- reader ----------------
    task automatic read_message(input integer vl, input integer ready_pct);
        integer i, timeout, len, seed;
        begin
            @(negedge clk);
            rd_vl = vl;
            len = q_len[vl][q_head[vl] % QMAX];
            seed = q_seed[vl][q_head[vl] % QMAX];
            #1;
            timeout = 0;
            while (!rd_info_valid && timeout < 5000) begin @(negedge clk); #1; timeout = timeout + 1; end
            if (timeout >= 5000) $fatal(1, "descriptor stalled vl=%0d", vl);
            if (rd_len != len || rd_meta != (seed & 9'h1ff)) begin
                data_errors = data_errors + 1;
                $display("descriptor mismatch vl=%0d len=%0d/%0d meta=%0d/%0d", vl, rd_len, len,
                         rd_meta, seed & 9'h1ff);
            end
            rd_start = 1;
            @(posedge clk); @(negedge clk);
            rd_start = 0;
            i = 0;
            timeout = 0;
            while (i < len && timeout < 20000) begin
                rd_ready = $urandom_range(0, 99) < ready_pct;
                #1;
                if (rd_ready && rd_valid) begin
                    if (rd_data !== pattern(seed, i) || rd_last !== (i == len-1)) data_errors = data_errors + 1;
                    i = i + 1;
                end
                @(posedge clk); @(negedge clk);
                timeout = timeout + 1;
            end
            rd_ready = 0;
            if (timeout >= 20000) $fatal(1, "payload stalled vl=%0d", vl);
            #1;
            timeout = 0;
            while (!rd_pop_ready && timeout < 5000) begin @(negedge clk); #1; timeout = timeout + 1; end
            if (timeout >= 5000) $fatal(1, "pop stalled vl=%0d", vl);
            rd_pop = 1;
            @(posedge clk); @(negedge clk);
            rd_pop = 0;
            q_head[vl] = q_head[vl] + 1;
            delivered = delivered + 1;
            repeat ($urandom_range(0, 2)) @(negedge clk);   // hold rd_vl past the pop
        end
    endtask

    // A synchronous lookup must not be accepted with a stale state merely
    // because the live tag revisits the value from two clocks earlier.
    task automatic check_lookup_revisit;
        integer timeout;
        begin
            @(negedge clk);
            wr_valid = 0; wr_commit = 0; wr_abort = 0; wr_vl = 0;
            repeat (4) @(negedge clk);
            #1;
            tb_check(wr_ready === 1'b1, "stable VL has a valid admission lookup");

            wr_vl = 1;
            #1;
            tb_check(wr_ready === 1'b0, "changed VL waits for its own admission lookup");
            @(posedge clk); @(negedge clk);
            wr_vl = 0;
            #1;
            tb_check(wr_ready === 1'b0,
                     "X-Y-X VL revisit cannot reuse the stale X admission tag");

            timeout = 0;
            while (wr_ready !== 1'b1 && timeout < 20) begin
                @(negedge clk); #1; timeout = timeout + 1;
            end
            tb_check(timeout < 20, "revisited VL admission lookup recovers");
        end
    endtask

    task automatic wait_descriptor(input integer vl);
        integer timeout;
        begin
            @(negedge clk);
            rd_vl = vl;
            #1;
            timeout = 0;
            while (rd_info_valid !== 1'b1 && timeout < 5000) begin
                @(negedge clk); #1; timeout = timeout + 1;
            end
            if (timeout >= 5000) $fatal(1, "directed descriptor stalled vl=%0d", vl);
        end
    endtask

    // Accept the final byte and a pop together. The queue executes both on
    // the next clock and must use the combined same-VL state transition.
    task automatic commit_and_pop_together(input integer vl);
        integer seed, timeout;
        begin
            wait_descriptor(vl);
            seed = seed_counter; seed_counter = seed_counter + 1;
            wr_vl = vl; wr_data = pattern(seed, 0); wr_meta = seed & 9'h1ff;
            wr_valid = 1; wr_commit = 1; wr_abort = 0;
            #1;
            timeout = 0;
            while ((wr_ready !== 1'b1 || rd_pop_ready !== 1'b1) && timeout < 5000) begin
                @(negedge clk); #1; timeout = timeout + 1;
            end
            if (timeout >= 5000) $fatal(1, "directed simultaneous commit/pop stalled");
            rd_pop = 1;
            q_len[vl][q_tail[vl] % QMAX] = 1;
            q_seed[vl][q_tail[vl] % QMAX] = seed;
            q_tail[vl] = q_tail[vl] + 1;
            committed = committed + 1;
            q_head[vl] = q_head[vl] + 1;
            delivered = delivered + 1;
            @(posedge clk); @(negedge clk);
            wr_valid = 0; wr_commit = 0; rd_pop = 0;
            #1;
            tb_check(dut.commit === 1'b1 && dut.pop === 1'b1 && dut.same_vl === 1'b1,
                     "same-VL commit and pop execute together");
        end
    endtask

    // Accept a pop one clock before a final byte. The pop writes the state on
    // the byte-accept clock, so the following commit must use its registered
    // pop-adjusted result instead of the stale RAM output.
    task automatic commit_after_pop(input integer vl);
        integer seed, timeout;
        begin
            wait_descriptor(vl);
            wr_vl = vl; wr_valid = 0; wr_commit = 0; wr_abort = 0;
            repeat (3) @(negedge clk);
            #1;
            timeout = 0;
            while (rd_pop_ready !== 1'b1 && timeout < 5000) begin
                @(negedge clk); #1; timeout = timeout + 1;
            end
            if (timeout >= 5000) $fatal(1, "directed pop stalled");
            rd_pop = 1;
            @(posedge clk); @(negedge clk);
            rd_pop = 0;

            seed = seed_counter; seed_counter = seed_counter + 1;
            wr_data = pattern(seed, 0); wr_meta = seed & 9'h1ff;
            wr_valid = 1; wr_commit = 1;
            #1;
            timeout = 0;
            while (wr_ready !== 1'b1 && timeout < 5000) begin
                @(negedge clk); #1; timeout = timeout + 1;
            end
            if (timeout >= 5000) $fatal(1, "directed pop-adjusted commit stalled");
            q_head[vl] = q_head[vl] + 1;
            delivered = delivered + 1;
            q_len[vl][q_tail[vl] % QMAX] = 1;
            q_seed[vl][q_tail[vl] % QMAX] = seed;
            q_tail[vl] = q_tail[vl] + 1;
            committed = committed + 1;
            @(posedge clk); @(negedge clk);
            wr_valid = 0; wr_commit = 0;
            #1;
            tb_check(dut.commit === 1'b1 && dut.writer_hit_pop === 1'b1,
                     "commit uses the preceding pop's registered state");
        end
    endtask

    integer phase;
    integer rng_seed;
    bit writer_done;
    initial begin
        integer init_v;
        if (!$value$plusargs("SEED=%d", rng_seed)) rng_seed = 32'h51a7e55;
        rng_seed = $urandom(rng_seed);
        for (init_v = 0; init_v < N_VL; init_v = init_v + 1) begin
            q_head[init_v] = 0;
            q_tail[init_v] = 0;
        end
        repeat (5) @(negedge clk);
        rst = 0;
        while (init_busy) @(negedge clk);
        check_lookup_revisit();
        write_message(0, 1, 0, 0);
        write_message(0, 1, 0, 0);
        commit_and_pop_together(0);
        commit_after_pop(0);
        for (phase = 0; phase < 3; phase = phase + 1) begin
            writer_done = 0;
            fork
                begin
                    integer wn, wv, wlen, wbusy, wpick;
                    // phase 0: 1-byte messages back to back; 1: mixed; 2: long with gaps
                    for (wn = 0; wn < 400; wn = wn + 1) begin
                        wv = $urandom_range(0, N_VL-1);
                        while (queued(wv) >= DEPTH) wv = (wv + 1) % N_VL;
                        wlen = (phase == 0) ? 1 : (phase == 1) ? $urandom_range(1, 20) : $urandom_range(10, MAX_LEN);
                        write_message(wv, wlen, (phase != 0) && ($urandom_range(0, 19) == 0),
                                      (phase == 2) ? 20 : 0);
                        // keep the model within the per-VL quota before the next pick
                        wbusy = 0;
                        while (wbusy < 20000) begin
                            wpick = 0;
                            for (wv = 0; wv < N_VL; wv = wv + 1)
                                if (queued(wv) < DEPTH) wpick = 1;
                            if (wpick) wbusy = 20000;
                            else begin @(negedge clk); wbusy = wbusy + 1; end
                        end
                    end
                    writer_done = 1;
                end
                begin
                    integer rv, rpick, roffset;
                    roffset = 0;
                    while (!writer_done || committed != delivered) begin
                        rpick = -1;
                        for (rv = 0; rv < N_VL; rv = rv + 1)
                            if (rpick < 0 && queued((rv + roffset) % N_VL) > 0)
                                rpick = (rv + roffset) % N_VL;
                        if (rpick >= 0) begin
                            read_message(rpick, (phase == 1) ? 60 : 100);
                            roffset = (rpick + 1) % N_VL;
                        end
                        else @(negedge clk);
                    end
                end
            join
            repeat (40) @(negedge clk);
            tb_check(committed == delivered, "every committed message is delivered once");
            tb_check(data_errors == 0, "payload, length, metadata and order match the model");
            tb_check(pool_used == 0 && pool_msgs == 0, "pool drains after aborts and pops");
            tb_check(NONEMPTY_RAM != 0 || nonempty == 0, "nonempty clears after draining");
        end
        tb_check(formula_errors == 0, "early commit/pop state equals the direct formulas");
        tb_check(spacing_errors == 0, "commits stay four clocks apart");
        tb_check(same_clock > 0 && adjusted > 0, "same-clock and pop-adjusted commits are exercised");
        $display("queue stress: committed=%0d aborted=%0d delivered=%0d same_clock=%0d pop_adjusted=%0d",
                 committed, aborted, delivered, same_clock, adjusted);
        tb_finish("tb_afdx_queue_stress");
        $finish;
    end

    initial begin
        #200_000_000;
        $fatal(1, "tb_afdx_queue_stress timeout");
    end
endmodule
