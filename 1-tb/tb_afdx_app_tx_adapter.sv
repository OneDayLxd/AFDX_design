/*
 * ================================================================
 * File Name    : tb_afdx_app_tx_adapter.sv
 * Module Name  : tb_afdx_app_tx_adapter
 * Function:
 *   128 VLID 映射、未知/截断首部、输入空洞、背压、超长、溢出和复位。
 * Sub Module:
 *   afdx_app_tx_adapter
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_app_tx_adapter;
    `include "tb_common.svh"
    reg clk=0, rst=1, valid=0, last=0, ready=1;
    reg [7:0] data=0;
    wire [6:0] vl;
    wire [7:0] out_data;
    wire out_valid, out_last, drop;
    wire [2:0] reason;
    wire [31:0] drops;
    always #4 clk=~clk;
    afdx_app_tx_adapter #(
        .SLOT_LOG (1),
        .MAX_LEN  (16)
    ) dut (
        .clk         (clk),
        .rst         (rst),
        .in_data     (data),
        .in_valid    (valid),
        .in_last     (last),
        .out_vl      (vl),
        .out_data    (out_data),
        .out_valid   (out_valid),
        .out_last    (out_last),
        .out_ready   (ready),
        .drop_pulse  (drop),
        .drop_reason (reason),
        .drop_count  (drops)
    );
    byte expected[$];
    int expected_vl[$], expected_last[$];
    int got=0, pulses=0;
    bit held=0;
    reg [15:0] held_beat;
    always @(posedge clk) if(rst) held=0; else begin
        if(held) tb_check(out_valid && {vl,out_last,out_data}===held_beat,"stable under backpressure");
        held=out_valid && !ready; held_beat={vl,out_last,out_data};
        if(drop) pulses++;
        if(out_valid && ready) begin
            tb_check(expected.size()>0,"no rejected/partial message");
            if(expected.size()==0) $fatal(1,"unexpected output");
            tb_check(out_data===expected.pop_front(),"payload ordered without VLID prefix");
            tb_check(vl==expected_vl.pop_front(),"actual VLID mapped to config index");
            tb_check(out_last==expected_last.pop_front(),"payload boundary");
            got++;
        end
    end
    task automatic beat(input byte b, input bit end_message);
        @(negedge clk); valid=1; data=b; last=end_message;
    endtask
    task automatic idle;
        @(negedge clk); valid=0; last=0;
    endtask
    task automatic message(input int id, n, input bit good, gaps);
        if(good) for(int p=0;p<n;p++) begin
            expected.push_back(byte'(p+id)); expected_vl.push_back(id-16'h1000);
            expected_last.push_back(p==n-1);
        end
        beat(byte'(id>>8),0);
        if(gaps) begin idle(); repeat(3) @(negedge clk); end
        beat(byte'(id),n==0);
        for(int p=0;p<n;p++) begin
            if(gaps && p%3==0) begin idle(); repeat(2) @(negedge clk); end
            beat(byte'(p+id),p==n-1);
        end
        idle();
    endtask
    task automatic drain;
        ready=1; wait(expected.size()==0); repeat(12) @(negedge clk);
    endtask
    initial begin
        repeat(6) @(negedge clk); rst=0;
        for(int v=0;v<128;v++) begin message(16'h1000+v,1+(v%16),1,v%2); drain(); end
        message(16'h0001,4,0,1); message(16'h1080,1,0,0); message(16'h1fff,16,0,1);
        message(16'h1001,0,0,0); beat(8'h10,1); idle();
        message(16'h1002,17,0,1); drain();
        tb_check(drops==6 && pulses==6,"unknown ID, malformed header and overlength each counted once");
        ready=0;
        message(16'h1003,8,1,0); message(16'h1004,4,1,0);
        message(16'h1005,4,0,0);
        repeat(10) @(negedge clk);
        tb_check(drops==7 && reason[0],"full input drops whole message");
        drain();
        beat(8'h10,0); beat(8'h06,0); beat(8'haa,0); idle();
        rst=1; repeat(6) @(negedge clk); rst=0; pulses=0;
        message(16'h107f,16,1,1); drain();
        tb_check(drops==0,"reset clears partial message and statistics");
        tb_finish("tb_afdx_app_tx_adapter"); $finish;
    end
    initial begin #2_000_000; $fatal(1,"TX adapter timeout"); end
endmodule
