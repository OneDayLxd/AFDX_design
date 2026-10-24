`timescale 1ns/1ps
module tb_afdx_integrity_age;
    `include "tb_common.svh"
    reg clk = 0;
    always #4 clk = ~clk;
    wire [1:0] done;
    genvar test;
    generate for (test=0; test<2; test=test+1) begin : g_case
        localparam LIMIT = test == 0 ? 50 : 65550;
        reg rst = 1, req = 0;
        reg [31:0] now_us = 0;
        wire resp, accept, ic_drop, rm_drop, busy;
        reg [31:0] last_time, offset;
        reg expected_accept;
        reg complete = 0;
        integer i, timeout, seed;
        assign done[test] = complete;
        // 禁用 IC，单独验证 RM 时间边界，包括低半字借位与 32 位回绕。
        afdx_rx_integrity #(
            .N_VL        (2),
            .VLW         (1),
            .IC_EN       (0),
            .SKEW_MAX_US (LIMIT)
        ) dut (
            .clk          (clk),
            .rst          (rst),
            .now_us       (now_us),
            .req          (req),
            .vl           (1'b0),
            .net          (1'b0),
            .sn           (8'd1),
            .resp         (resp),
            .accept       (accept),
            .stat_ic_drop (ic_drop),
            .stat_rm_drop (rm_drop),
            .init_busy     (busy),
            .delivery_done (1'b0),
            .delivery_ok   (1'b0)
        );
        initial begin
            seed = 241 + test;
            repeat (4) @(negedge clk);
            rst = 0;
            wait (!busy);
            last_time = 32'hffffff00;
            for (i=0; i<300; i=i+1) begin
                case (i % 8)
                    0: offset = 0;
                    1: offset = LIMIT-1;
                    2: offset = LIMIT;
                    3: offset = LIMIT+1;
                    4: offset = 65535;
                    5: offset = 65536;
                    default: offset = $random(seed) & 32'h0003ffff;
                endcase
                @(negedge clk);
                now_us = last_time + offset;
                expected_accept = i == 0 || offset > LIMIT;
                req = 1;
                @(negedge clk);
                req = 0;
                timeout = 0;
                while (!resp && timeout < 20) begin
                    @(negedge clk);
                    timeout = timeout + 1;
                end
                if (!resp || accept !== expected_accept || ic_drop !== 0 || rm_drop !== !expected_accept)
                    $fatal(1, "age mismatch case=%0d i=%0d offset=%0d accept=%b expected=%b", test, i, offset, accept, expected_accept);
                if (expected_accept) last_time = now_us;
            end
            complete = 1;
        end
    end endgenerate
    initial begin
        wait (&done);
        tb_check(1, "600 RM age decisions match strict unsigned 32-bit reference");
        tb_finish("tb_afdx_integrity_age");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1, "integrity age timeout");
    end
endmodule
