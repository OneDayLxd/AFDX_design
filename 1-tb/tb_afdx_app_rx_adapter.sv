/*
 * ================================================================
 * File Name    : tb_afdx_app_rx_adapter.sv
 * Module Name  : tb_afdx_app_rx_adapter
 * Function:
 *   4096 个接收索引逆映射、VLID 大端首部、输入空洞和复位恢复。
 * Sub Module:
 *   afdx_app_rx_adapter
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_app_rx_adapter;
    `include "tb_common.svh"
    reg clk=0, rst=1, valid=0, last=0;
    reg [11:0] vl=0;
    reg [7:0] data=0;
    wire ready, out_valid, out_last;
    wire [7:0] out_data;
    always #4 clk=~clk;
    afdx_app_rx_adapter dut (
        .clk       (clk),
        .rst       (rst),
        .in_vl     (vl),
        .in_data   (data),
        .in_valid  (valid),
        .in_last   (last),
        .in_ready  (ready),
        .out_data  (out_data),
        .out_valid (out_valid),
        .out_last  (out_last)
    );
    byte expected[$];
    int expected_last[$];
    always @(posedge clk) if(!rst && out_valid) begin
        tb_check(expected.size()>0,"no extra output");
        if(expected.size()==0) $fatal(1,"unexpected output");
        tb_check(out_data===expected.pop_front(),"actual VLID prefix and payload");
        tb_check(out_last==expected_last.pop_front(),"last on payload only");
    end
    task automatic message(input int index, length);
        int id;
        id=16'h1000+index;
        expected.push_back(byte'(id>>8)); expected_last.push_back(0);
        expected.push_back(byte'(id)); expected_last.push_back(0);
        for(int p=0;p<length;p++) begin expected.push_back(byte'(index+p)); expected_last.push_back(p==length-1); end
        for(int p=0;p<length;p++) begin
            @(negedge clk); valid=1; last=p==length-1; vl=index; data=index+p;
            @(posedge clk); while(!ready) @(posedge clk);
            if(p%3==1) begin @(negedge clk); valid=0; last=1; repeat(2) @(negedge clk); end
        end
        @(negedge clk); valid=0; last=0;
    endtask
    initial begin
        repeat(6) @(negedge clk); rst=0;
        for(int i=0;i<4096;i++) message(i,1+i%4);
        wait(expected.size()==0);
        expected.push_back(8'h1f); expected_last.push_back(0);
        @(negedge clk); valid=1; vl=12'hfff;
        @(negedge clk); rst=1; valid=0;
        repeat(6) @(negedge clk); rst=0;
        message(12'habc,17); repeat(6) @(negedge clk);
        tb_check(expected.size()==0,"all serialized messages complete");
        tb_finish("tb_afdx_app_rx_adapter"); $finish;
    end
    initial begin #2_000_000; $fatal(1,"RX adapter timeout"); end
endmodule
