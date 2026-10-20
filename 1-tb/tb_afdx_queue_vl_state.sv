`timescale 1ns/1ps

module tb_afdx_queue_vl_state;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg [11:0] wr_vl = 0, rd_vl = 0;
    reg wr_valid = 0, wr_commit = 0, wr_abort = 0;
    reg [7:0] wr_data = 0;
    wire wr_ready, wr_ovf;
    reg rd_start = 0, rd_pop = 0, rd_ready = 0;
    wire rd_info_valid, rd_pop_ready, reclaim_busy, init_busy;
    wire [3:0] rd_len;
    wire [8:0] rd_meta;
    wire [7:0] rd_data;
    wire rd_valid, rd_last;
    wire [4095:0] nonempty;
    wire [4:0] pool_used, pool_peak, pool_msgs, pool_msgs_peak;
    integer checks = 0;

    afdx_msg_queue #(.N_VL(4096), .VLW(12), .DEPTH(4), .DW(2),
                     .SLOT_AW(4), .MW(9), .POOL_AW(4), .BLOCK_AW(4), .MAX_MSGS(16)) dut (
        .clk(clk), .rst(rst), .wr_vl(wr_vl), .wr_valid(wr_valid), .wr_data(wr_data),
        .wr_ready(wr_ready), .wr_commit(wr_commit), .wr_meta(9'h12a),
        .wr_abort(wr_abort), .wr_ovf(wr_ovf),
        .rd_vl(rd_vl), .rd_start(rd_start), .rd_pop(rd_pop),
        .rd_info_valid(rd_info_valid), .reclaim_busy(reclaim_busy), .rd_pop_ready(rd_pop_ready),
        .rd_len(rd_len), .rd_meta(rd_meta), .rd_data(rd_data),
        .rd_valid(rd_valid), .rd_last(rd_last), .rd_ready(rd_ready),
        .nonempty(nonempty), .pool_used(pool_used), .pool_peak(pool_peak),
        .pool_msgs(pool_msgs), .pool_msgs_peak(pool_msgs_peak),
        .ne_group(8'd0), .ne_bits(), .init_busy(init_busy));

    task automatic push(input integer vl, input [7:0] data);
        integer timeout;
        begin
            @(negedge clk);
            wr_vl = vl;
            wr_data = data;
            wr_valid = 1;
            wr_commit = 1;
            #1;
            timeout = 0;
            while (wr_ready !== 1'b1 && timeout < 100) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 100) $fatal(1, "write admission stalled on VL %0d", vl);
            @(posedge clk);
            @(negedge clk);
            wr_valid = 0;
            wr_commit = 0;
            checks = checks + 1;
        end
    endtask

    task automatic wait_info(input integer vl);
        integer timeout;
        begin
            @(negedge clk);
            rd_vl = vl;
            #1;
            timeout = 0;
            while (rd_info_valid !== 1'b1 && timeout < 100) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 100) $fatal(1, "descriptor lookup stalled on VL %0d", vl);
            if (rd_len !== 4'd1 || rd_meta !== 9'h12a)
                $fatal(1, "descriptor mismatch on VL %0d", vl);
        end
    endtask

    task automatic read_one(input integer vl, input [7:0] expected);
        integer timeout;
        begin
            wait_info(vl);
            rd_start = 1;
            @(posedge clk);
            @(negedge clk);
            rd_start = 0;
            rd_ready = 1;
            timeout = 0;
            while (rd_valid !== 1'b1 && timeout < 100) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 100) $fatal(1, "payload read stalled on VL %0d", vl);
            if (rd_data !== expected || rd_last !== 1'b1)
                $fatal(1, "VL %0d returned %02x, expected %02x", vl, rd_data, expected);
            @(posedge clk);
            @(negedge clk);
            rd_ready = 0;
            timeout = 0;
            while (rd_pop_ready !== 1'b1 && timeout < 100) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 100) $fatal(1, "pop stalled on VL %0d", vl);
            rd_pop = 1;
            @(posedge clk);
            @(negedge clk);
            rd_pop = 0;
            checks = checks + 2;
        end
    endtask

    // A pop accepted with the committing beat takes effect on the next
    // clock, together with that commit.
    task automatic push_and_pop_together(input integer pop_vl, input integer push_vl,
                                          input [7:0] data, input bit expect_same);
        integer timeout;
        begin
            wait_info(pop_vl);
            @(negedge clk);
            wr_vl = push_vl;
            wr_data = data;
            wr_valid = 1;
            wr_commit = 1;
            #1;
            timeout = 0;
            while ((wr_ready !== 1'b1 || rd_pop_ready !== 1'b1) && timeout < 100) begin
                @(negedge clk);
                #1;
                timeout = timeout + 1;
            end
            if (timeout >= 100) $fatal(1, "commit/pop could not be aligned");
            rd_pop = 1;
            @(posedge clk);
            @(negedge clk);
            wr_valid = 0;
            wr_commit = 0;
            rd_pop = 0;
            #1;
            if (dut.commit !== 1'b1 || dut.pop !== 1'b1 || dut.same_vl !== expect_same)
                $fatal(1, "wrong simultaneous commit/pop classification");
            checks = checks + 2;
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        rst = 0;
        while (init_busy !== 1'b0) @(negedge clk);
        push(4095,8'h11); push(4095,8'h22);
        push(4095,8'h33); push(4095,8'h44);
        repeat (6) @(negedge clk);
        if (wr_ready !== 1'b0 || dut.admission_quota_ok !== 1'b0)
            $fatal(1, "four-message quota did not close");
        checks = checks + 1;
        read_one(4095,8'h11);
        push(4095,8'h55);
        read_one(4095,8'h22); read_one(4095,8'h33);
        read_one(4095,8'h44); read_one(4095,8'h55);
        repeat (8) @(negedge clk);
        if (nonempty[4095] !== 1'b0) $fatal(1, "highest VL did not drain");
        checks = checks + 1;

        // After a reset the state RAM is swept again, so the stale count of
        // the drained VL cannot leak into the new queue state.
        rst = 1;
        repeat (5) @(negedge clk);
        rst = 0;
        while (init_busy !== 1'b0) @(negedge clk);
        push(4095,8'h66);
        read_one(4095,8'h66);

        push(0,8'ha0);
        push_and_pop_together(0,4095,8'h77,1'b0);
        repeat (3) @(negedge clk);
        if (nonempty[0] !== 1'b0 || nonempty[4095] !== 1'b1)
            $fatal(1, "different-VL simultaneous update lost state");
        checks = checks + 1;

        push_and_pop_together(4095,4095,8'h88,1'b1);
        repeat (3) @(negedge clk);
        if (nonempty[4095] !== 1'b1)
            $fatal(1, "same-VL simultaneous update lost state");
        read_one(4095,8'h88);
        repeat (8) @(negedge clk);
        if (nonempty[4095] !== 1'b0 || pool_used !== 0 || pool_msgs !== 0)
            $fatal(1, "pool or VL state did not drain");
        checks = checks + 2;
        $display("tb_afdx_queue_vl_state: %0d checks, 0 errors -> PASS", checks);
        $finish;
    end
endmodule
