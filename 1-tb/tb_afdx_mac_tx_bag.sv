`timescale 1ns/1ps

// MAC transmit BAG at the wire with 128 VLs: exact SOF spacing for queued
// frames of one VL, byte-slot pacing, and a random all-VL stress in which
// every SOF interval must be at least the frame's BAG.
module tb_afdx_mac_tx_bag;
    `include "tb_common.svh"

    localparam N_VL = 128;
    localparam VLW = 7;
    localparam CLK_HZ = 10_000_000;
    localparam CPU = CLK_HZ / 1_000_000;
    localparam MAX_FRAMES = 4096;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #50 clk = ~clk;

    reg [7:0] in_data = 0;
    reg in_valid = 0, in_last = 0;
    reg [VLW-1:0] in_vl = 0;
    reg [19:0] in_bag_us = 0;
    wire in_ready;
    wire [7:0] tx_data;
    wire tx_valid, tx_frame, tx_busy, stat_frame, stat_sof, bag_wait;
    wire [VLW-1:0] stat_sof_vl;
    wire [N_VL-1:0] bag_open;
    reg tx_slot = 1'b1;

    afdx_mac_tx #(.FIFO_AW(12), .IFG(12), .N_VL(N_VL), .VLW(VLW), .CLK_HZ(CLK_HZ)) dut (
        .clk(clk), .rst(rst), .in_data(in_data), .in_valid(in_valid), .in_last(in_last),
        .in_vl(in_vl), .in_bag_us(in_bag_us), .in_tag(1'b0), .in_ready(in_ready),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_slot(tx_slot), .tx_frame(tx_frame),
        .tx_busy(tx_busy), .stat_frame(stat_frame), .bag_open(bag_open),
        .stat_sof(stat_sof), .stat_sof_vl(stat_sof_vl), .stat_sof_tag(), .bag_wait(bag_wait));

    integer cycle = 0;
    always @(posedge clk) cycle <= cycle + 1;

    // Wire monitor: byte 0 carries the VL, byte 1 the per-VL frame number.
    integer sent_bags [0:N_VL-1];
    integer last_sof [0:N_VL-1];
    integer last_bag [0:N_VL-1];
    integer next_rx [0:N_VL-1];
    integer sof_log [0:MAX_FRAMES-1];
    integer sof_vl_log [0:MAX_FRAMES-1];
    integer frames_seen = 0, bag_errors = 0, order_errors = 0, fcs_errors = 0;
    integer max_late = 0;
    integer pos = 0, cur_vl = 0, cur_sof = 0;
    // +SOF_TRACE prints every start of frame, e.g. to compare two RTL versions.
    bit sof_trace = $test$plusargs("SOF_TRACE");
    reg [31:0] crc = 32'hffff_ffff;
    bit active = 0;
    integer expected_len [0:N_VL-1][0:63];

    always @(posedge clk) begin
        if (rst) begin
            active = 0;
        end else begin
            if (tx_valid && tx_slot) begin
                if (!active) begin
                    active = 1; pos = 0; crc = 32'hffff_ffff;
                    cur_vl = tx_data; cur_sof = cycle;
                    if (last_sof[cur_vl] >= 0) begin
                        if (cycle - last_sof[cur_vl] < last_bag[cur_vl] * CPU) begin
                            bag_errors = bag_errors + 1;
                            $display("BAG violation vl=%0d interval=%0d bag=%0d", cur_vl,
                                     cycle - last_sof[cur_vl], last_bag[cur_vl] * CPU);
                        end
                        if (cycle - last_sof[cur_vl] - last_bag[cur_vl] * CPU > max_late)
                            max_late = cycle - last_sof[cur_vl] - last_bag[cur_vl] * CPU;
                    end
                    last_sof[cur_vl] = cycle;
                    last_bag[cur_vl] = sent_bags[cur_vl];
                    if (sof_trace) $display("SOF %0d %0d", cycle, cur_vl);
                    if (frames_seen < MAX_FRAMES) begin
                        sof_log[frames_seen] = cycle;
                        sof_vl_log[frames_seen] = cur_vl;
                    end
                end
                if (pos == 1 && tx_data != (next_rx[cur_vl] & 255)) order_errors = order_errors + 1;
                crc = tb_crc32_byte(crc, tx_data);
                pos = pos + 1;
            end
            if (active && !tx_frame && !tx_valid) begin
                if (crc != 32'hdebb20e3 || pos != expected_len[cur_vl][next_rx[cur_vl] % 64] + 4)
                    fcs_errors = fcs_errors + 1;
                next_rx[cur_vl] = next_rx[cur_vl] + 1;
                frames_seen = frames_seen + 1;
                active = 0;
            end
        end
    end

    integer next_tx [0:N_VL-1];
    task automatic push_frame(input integer vl, input integer len, input integer bag_us);
        integer i;
        begin
            expected_len[vl][next_tx[vl] % 64] = len;
            sent_bags[vl] = bag_us;
            for (i = 0; i < len; i = i + 1) begin
                @(negedge clk);
                in_valid = 1'b1;
                in_vl = vl;
                in_bag_us = bag_us;
                in_last = (i == len-1);
                in_data = (i == 0) ? vl : (i == 1) ? next_tx[vl] : (vl*7 + i);
                #1;
                while (!in_ready) begin @(negedge clk); #1; end
                @(posedge clk);
            end
            @(negedge clk);
            in_valid = 1'b0; in_last = 1'b0;
            next_tx[vl] = next_tx[vl] + 1;
        end
    endtask

    task automatic reset_all;
        integer v;
        begin
            @(negedge clk); rst = 1'b1; in_valid = 1'b0;
            for (v = 0; v < N_VL; v = v + 1) begin
                last_sof[v] = -1; last_bag[v] = 0; next_rx[v] = 0; next_tx[v] = 0; sent_bags[v] = 0;
            end
            frames_seen = 0; bag_errors = 0; order_errors = 0; fcs_errors = 0; max_late = 0;
            repeat (8) @(negedge clk);
            rst = 1'b0;
            repeat (200) @(negedge clk);   // queue state sweep
        end
    endtask

    task automatic wait_frames(input integer target);
        integer timeout;
        begin
            timeout = 0;
            while (frames_seen < target && timeout < 2_000_000) begin
                @(negedge clk); timeout = timeout + 1;
            end
            tb_check(timeout < 2_000_000, "all queued frames reach the wire");
        end
    endtask

    integer k, v, n, order_ok;
    integer vls [0:N_VL*3-1];
    integer tmp, j, slot_mode;
    always @(posedge clk) begin
        if (slot_mode == 1) tx_slot <= (cycle % 8) == 7;
        else if (slot_mode == 2) tx_slot <= $urandom_range(0, 1);
        else tx_slot <= 1'b1;
    end

    initial begin
        slot_mode = 0;
        // 1) Four queued frames of one VL leave exactly one BAG apart.
        reset_all();
        for (k = 0; k < 4; k = k + 1) push_frame(37, 60, 20);
        wait_frames(4);
        for (k = 1; k < 4; k = k + 1)
            tb_check(sof_log[k] - sof_log[k-1] == 20 * CPU, "queued frames start exactly one BAG apart");
        $display("exact: SOF intervals %0d %0d %0d", sof_log[1]-sof_log[0], sof_log[2]-sof_log[1], sof_log[3]-sof_log[2]);

        // 2) Byte slots every 8 clocks: the SOF is the first slot at or after the BAG.
        slot_mode = 1;
        reset_all();
        for (k = 0; k < 4; k = k + 1) push_frame(100, 60, 100);
        wait_frames(4);
        for (k = 1; k < 4; k = k + 1)
            tb_check(sof_log[k] - sof_log[k-1] >= 100 * CPU && sof_log[k] - sof_log[k-1] < 100 * CPU + 8,
                     "paced frames start on the first byte slot after the BAG");
        $display("paced: SOF intervals %0d %0d %0d", sof_log[1]-sof_log[0], sof_log[2]-sof_log[1], sof_log[3]-sof_log[2]);

        // 3) Two VLs in different regions alternate and each keeps its BAG.
        slot_mode = 0;
        reset_all();
        for (k = 0; k < 4; k = k + 1) begin push_frame(3, 60, 20); push_frame(99, 60, 20); end
        wait_frames(8);
        tb_check(bag_errors == 0 && order_errors == 0 && fcs_errors == 0, "two regions keep BAG, order and FCS");
        $display("two VLs: max lateness %0d clocks", max_late);

        // 4) Two full regions whose BAG outlasts a round of their frames: every
        //    VL waits for its own boundary while its neighbours start frames.
        reset_all();
        for (k = 0; k < 3; k = k + 1)
            for (v = 0; v < 32; v = v + 1) push_frame((v < 16) ? v : 96 + v, 40, 150);
        wait_frames(96);
        tb_check(bag_errors == 0 && order_errors == 0 && fcs_errors == 0,
                 "long-BAG regions keep BAG, order and FCS");
        $display("long BAG: max lateness %0d clocks", max_late);

        // 5) Random stress over all 128 VLs with random slots and lengths.
        for (n = 0; n < 3; n = n + 1) begin
            slot_mode = (n == 1) ? 2 : 0;
            reset_all();
            for (k = 0; k < N_VL*3; k = k + 1) vls[k] = k % N_VL;
            for (k = N_VL*3-1; k > 0; k = k - 1) begin
                j = $urandom_range(0, k); tmp = vls[k]; vls[k] = vls[j]; vls[j] = tmp;
            end
            for (k = 0; k < N_VL*3; k = k + 1)
                push_frame(vls[k], $urandom_range(8, 100), (vls[k] % 4 == 0) ? 3 : 10 + (vls[k] % 5) * 7);
            wait_frames(N_VL*3);
            repeat (100) @(negedge clk);
            tb_check(frames_seen == N_VL*3, "random stress sends every frame once");
            tb_check(bag_errors == 0, "random stress never starts a VL frame inside its BAG");
            tb_check(order_errors == 0, "per-VL frame order is preserved");
            tb_check(fcs_errors == 0, "random stress frames keep length and FCS");
            $display("stress %0d: frames=%0d max lateness %0d clocks", n, frames_seen, max_late);
        end
        tb_finish("tb_afdx_mac_tx_bag");
        $finish;
    end

    initial begin
        #2_000_000_000;
        $fatal(1, "tb_afdx_mac_tx_bag timeout");
    end
endmodule
