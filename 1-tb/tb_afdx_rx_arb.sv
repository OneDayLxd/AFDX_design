`timescale 1ns/1ps
module tb_afdx_rx_arb;
    // 批处理统一检查完成标志，避免提前 $fatal 被误判为成功。
    integer tb_errors = 0;
    bit tb_done = 0;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    reg out_ready = 0;
    wire a_ready, b_ready;
    wire [7:0] out_data;
    wire out_valid, out_last, out_net;
    integer a_idx = 0, b_idx = 0, out_idx = 0, cycle = 0, checks = 0;
    reg held = 0;
    reg [7:0] held_data;
    reg held_last, held_net;

    wire a_valid = a_idx < 3;
    wire b_valid = b_idx < 3;
    wire [7:0] a_data = a_idx == 0 ? 8'ha1 : a_idx == 1 ? 8'ha2 : 8'ha3;
    wire [7:0] b_data = b_idx == 0 ? 8'hb1 : b_idx == 1 ? 8'hb2 : 8'hb3;
    wire a_last = a_idx != 0;
    wire b_last = b_idx != 0;

    afdx_rx_arb dut (
        .a_ts                      (32'd0),
        .b_ts                      (32'd0),
        .out_ts                    (),
        .clk(clk), .rst(rst),
        .a_data(a_data), .a_valid(a_valid), .a_last(a_last), .a_ready(a_ready),
        .b_data(b_data), .b_valid(b_valid), .b_last(b_last), .b_ready(b_ready),
        .out_data(out_data), .out_valid(out_valid), .out_last(out_last),
        .out_net(out_net), .out_ready(out_ready));

    always @(posedge clk) begin
        if (rst) begin
            a_idx <= 0; b_idx <= 0; out_idx <= 0; cycle <= 0;
            held <= 0; out_ready <= 0;
        end else begin
            cycle <= cycle + 1;
            out_ready <= cycle >= 5 && (cycle % 7) < 4;
            if (a_ready && b_ready) $fatal(1, "both sources accepted together");
            if (held && (!out_valid || out_data !== held_data ||
                         out_last !== held_last || out_net !== held_net))
                $fatal(1, "output changed under backpressure");
            held <= out_valid && !out_ready;
            held_data <= out_data; held_last <= out_last; held_net <= out_net;
            if (a_valid && a_ready) a_idx <= a_idx + 1;
            if (b_valid && b_ready) b_idx <= b_idx + 1;
            if (out_valid && out_ready) begin
                case (out_idx)
                    0: if ({out_net,out_last,out_data} !== {1'b0,1'b0,8'ha1}) $fatal(1, "A frame byte 0");
                    1: if ({out_net,out_last,out_data} !== {1'b0,1'b1,8'ha2}) $fatal(1, "A frame byte 1");
                    2: if ({out_net,out_last,out_data} !== {1'b1,1'b0,8'hb1}) $fatal(1, "B frame byte 0");
                    3: if ({out_net,out_last,out_data} !== {1'b1,1'b1,8'hb2}) $fatal(1, "B frame byte 1");
                    4: if ({out_net,out_last,out_data} !== {1'b0,1'b1,8'ha3}) $fatal(1, "A second frame");
                    5: if ({out_net,out_last,out_data} !== {1'b1,1'b1,8'hb3}) $fatal(1, "B second frame");
                    default: $fatal(1, "extra output byte");
                endcase
                out_idx <= out_idx + 1;
                checks = checks + 1;
            end
            if (cycle > 200) $fatal(1, "arbiter made no progress");
        end
    end

    initial begin
        repeat (4) @(negedge clk);
        rst = 0;
        wait (out_idx == 6);
        repeat (5) @(negedge clk);
        if (a_idx != 3 || b_idx != 3 || out_valid)
            $fatal(1, "arbiter did not drain both sources");
        checks = checks + 1;
        tb_done = 1;
        $display("tb_afdx_rx_arb: %0d checks, 0 errors -> PASS", checks);
        $finish;
    end
endmodule
