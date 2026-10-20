`timescale 1ns/1ps

module tb_afdx_rx_deliver_scan;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg [4095:0] nonempty = 0;
    wire [11:0] q_rd_vl, msg_vl;
    wire q_rd_start, q_rd_pop, q_rd_ready;
    wire msg_valid, msg_last, stat_deliver;
    wire [7:0] msg_data;
    wire [10:0] msg_len;
    wire [8:0] msg_meta;
    integer remaining [0:4095];
    integer expected [0:9];
    integer delivered = 0;
    integer checks = 0;
    integer cycle = 0;
    integer i;

    afdx_rx_deliver #(.N_VL(4096), .VLW(12)) dut (
        .clk(clk), .rst(rst), .nonempty(nonempty), .ne_group(), .ne_bits(16'd0), .q_init_busy(1'b0),
        .q_rd_vl(q_rd_vl), .q_rd_start(q_rd_start), .q_rd_pop(q_rd_pop),
        .q_rd_info_valid(1'b1), .q_rd_pop_ready(1'b1),
        .q_rd_len(11'd1), .q_rd_meta(9'h055),
        .q_rd_data(8'ha5), .q_rd_valid(1'b1), .q_rd_last(1'b1), .q_rd_ready(q_rd_ready),
        .msg_vl(msg_vl), .msg_valid(msg_valid), .msg_data(msg_data), .msg_last(msg_last),
        .msg_ready(1'b1), .msg_len(msg_len), .msg_meta(msg_meta), .stat_deliver(stat_deliver));

    always @(posedge clk) begin
        cycle = cycle + 1;
        if (!rst) begin
            if (msg_valid) begin
                if (delivered >= 10 || msg_vl !== expected[delivered])
                    $fatal(1, "delivery %0d picked VL %0d, expected %0d", delivered, msg_vl, expected[delivered]);
                if (msg_data !== 8'ha5 || !msg_last || msg_len !== 11'd1 || msg_meta !== 9'h055)
                    $fatal(1, "delivery metadata or payload mismatch");
                delivered = delivered + 1;
                checks = checks + 2;
            end
            if (q_rd_pop) begin
                if (remaining[q_rd_vl] <= 0) $fatal(1, "unexpected pop on VL %0d", q_rd_vl);
                remaining[q_rd_vl] = remaining[q_rd_vl] - 1;
                if (remaining[q_rd_vl] == 0) nonempty[q_rd_vl] = 0;
            end
        end
    end

    task automatic enqueue(input integer vl, input integer count);
        begin
            remaining[vl] = count;
            nonempty[vl] = 1;
        end
    endtask

    task automatic wait_deliveries(input integer target, input integer limit);
        integer start_cycle;
        begin
            start_cycle = cycle;
            while (delivered < target && cycle - start_cycle < limit) @(negedge clk);
            if (delivered != target) $fatal(1, "only %0d of %0d deliveries within %0d cycles", delivered, target, limit);
        end
    endtask

    integer high_start;
    initial begin
        for (i=0; i<4096; i=i+1) remaining[i] = 0;
        expected[0]=0; expected[1]=15; expected[2]=16;
        expected[3]=255; expected[4]=4080; expected[5]=4095;
        expected[6]=0; expected[7]=16; expected[8]=0; expected[9]=4095;
        repeat (4) @(negedge clk);
        rst = 0;
        enqueue(0,1); enqueue(15,1); enqueue(16,1);
        enqueue(255,1); enqueue(4080,1); enqueue(4095,1);
        wait_deliveries(6, 1800);
        repeat (5) @(negedge clk);
        if (nonempty !== 0) $fatal(1, "first group did not drain");

        // After VL 0, a waiting VL in the next group precedes a second
        // message from VL 0. This checks true wraparound round-robin order.
        enqueue(0,2); enqueue(16,1);
        wait_deliveries(9, 800);
        repeat (5) @(negedge clk);
        if (nonempty !== 0) $fatal(1, "second group did not drain");

        high_start = cycle;
        enqueue(4095,1);
        wait_deliveries(10, 270);
        if (cycle - high_start > 265) $fatal(1, "high VL scan took %0d cycles", cycle - high_start);
        checks = checks + 3;
        $display("tb_afdx_rx_deliver_scan: %0d checks, 0 errors -> PASS", checks);
        $finish;
    end
endmodule
