`timescale 1ns/1ps
module tb_afdx_ip_fragment_tx;
    reg clk = 0, rst = 1;
    always #4 clk = ~clk;
    integer tb_errors = 0, checks = 0;
    bit tb_done = 0;
    reg [7:0] data = 0;
    reg valid = 0, last = 0;
    reg [15:0] length = 0;
    reg [31:0] sum = 0;
    wire ready;
    wire [7:0] result;
    wire result_valid, result_last;
    wire [79:0] fragment;
    reg result_ready = 0;
    integer received = 0, chunk_pos = 0, chunks = 0, expected_offset = 0;
    integer wanted_chunk = 0, remaining = 0, expected_id = 0;
    integer cycle = 0;
    function automatic [7:0] pattern(input integer i);
        pattern = (i*37 + (i>>8) + 19) & 255;
    endfunction
    task automatic check(input bit ok, input string message);
        checks++;
        if (!ok) begin tb_errors++; $display("ERROR %s at %0t",message,$time); end
    endtask
    afdx_ip_fragment_tx #(
        .VLW          (7)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .in_vl        (7'd3),
        .in_data      (data),
        .in_valid     (valid),
        .in_last      (last),
        .in_len       (length),
        .in_sum       (sum),
        .in_lmax      (11'd1471),
        .in_ready     (ready),
        .out_vl       (),
        .out_data     (result),
        .out_valid    (result_valid),
        .out_last     (result_last),
        .out_fragment (fragment),
        .out_ready    (result_ready)
    );
    reg [89:0] held;
    reg stalled = 0;
    always @(negedge clk) begin
        cycle++;
        result_ready = cycle%7 != 0 && cycle%7 != 1;
    end
    always @(posedge clk) if (!rst) begin
        if (stalled) check({result_valid,result_last,result,fragment} == held,"backpressure stable");
        stalled = result_valid && !result_ready;
        held = {result_valid,result_last,result,fragment};
        if (result_valid && result_ready) begin
            check(result == pattern(received),"byte order");
            check(result_last == (chunk_pos+1 == wanted_chunk),"fragment boundary");
            check(fragment[79] == (length>1471),"fragment enable");
            if (length>1471) begin
                check(fragment[78] == (remaining>wanted_chunk),"MF flag");
                check(fragment[77:65] == expected_offset,"offset in eight-byte units");
                check(fragment[64:49] == expected_id,"datagram ID stable across pieces");
                check(fragment[48:33] == length+8,"whole UDP length");
                check(fragment[32:1] == sum,"whole payload checksum sum");
            end else check(fragment == 0,"short message metadata zero");
            received++;
            chunk_pos++;
            if (result_last) begin
                remaining -= wanted_chunk;
                expected_offset += 183;
                chunk_pos = 0;
                wanted_chunk = remaining>1464 ? 1464 : remaining;
                chunks++;
            end
        end
    end
    task automatic run_message(input integer n);
        @(negedge clk);
        length = n;
        sum = 0;
        for (int i=0;i<n;i++) sum += (i%2 ? {24'd0,pattern(i)} : {16'd0,pattern(i),8'd0});
        received = 0; remaining = n; chunks = 0; chunk_pos = 0; expected_offset = 0;
        wanted_chunk = n>1471 ? 1456 : n;
        for (int i=0;i<n;i++) begin
            data = pattern(i); valid = 1; last = i==n-1;
            do @(posedge clk); while (!ready);
            @(negedge clk);
            valid = 0;
            // Some source gaps, in addition to downstream stalls.
            if (i%19 == 0) @(negedge clk);
        end
        last = 0;
        repeat(4) @(negedge clk);
        check(received == n,"all input bytes delivered exactly once");
        check(remaining == 0,"complete datagram");
        expected_id++;
    endtask
    initial begin
        repeat(4) @(negedge clk);
        rst = 0;
        run_message(1);
        run_message(1471);
        run_message(1472);
        run_message(2920);
        run_message(8191);
        run_message(8192);
        tb_done = 1;
        $display("tb_afdx_ip_fragment_tx: %0d checks, %0d errors -> %s", checks,tb_errors,tb_errors ? "FAIL" : "PASS");
        if (tb_errors) $fatal(1,"fragment packetizer failed");
        $finish;
    end
    initial begin #2000000; $fatal(1,"watchdog"); end
endmodule
