`timescale 1ns/1ps
module tb_afdx_mgmt_rxbuf;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    always #5 clk=~clk;
    reg valid=0, last=0, release_req=0;
    reg [7:0] data=0;
    reg [15:0] port=161;
    reg [10:0] addr=0;
    wire req_valid;
    wire [10:0] len;
    wire [7:0] rd;
    wire [31:0] accepted, full_drops, port_drops, long_drops;
    integer held_len=0;
    afdx_mgmt_rxbuf dut (
        .clk(clk), .rst(rst), .enable(1'b1), .snmp_port(16'd161), .rtc_port(16'd3190),
        .rx_vl(3'd0), .rx_valid(valid), .rx_data(data), .rx_last(last),
        .rx_src_ip(32'h01020304), .rx_sport(16'd4000), .rx_dport(port), .rx_ts(32'd42),
        .req_valid(req_valid), .req_len(len), .req_release(release_req),
        .rd_addr(addr), .rd_data(rd), .cnt_accept(accepted), .cnt_drop_full(full_drops),
        .cnt_drop_port(port_drops), .cnt_drop_long(long_drops)
    );
    always @(negedge clk)
        if (held_len != 0) tb_check(req_valid && len == held_len, "occupied head length stays stable");
    task automatic send_msg(input integer n, input integer seed, input bit gaps);
        integer i;
        begin
            for(i=0;i<n;i=i+1) begin
                @(negedge clk); valid=1; last=(i==n-1); data=seed+i;
                if(gaps && i%3==1) begin @(negedge clk); valid=0; last=0; end
            end
            @(negedge clk); valid=0; last=0;
            repeat(5) @(negedge clk);
        end
    endtask
    task automatic read_msg(input integer n, input integer seed);
        integer i;
        begin
            tb_check(req_valid && len==n, "committed length matches complete message");
            for(i=0;i<n;i=i+1) begin
                @(negedge clk); addr=i;
                @(posedge clk); #1;
                tb_check(rd == ((seed+i)&255), "RAM byte order and final byte");
            end
        end
    endtask
    task automatic pop;
        begin
            @(negedge clk); release_req=1;
            @(negedge clk); release_req=0;
            repeat(4) @(negedge clk);
        end
    endtask
    integer j;
    initial begin
        repeat(4) @(negedge clk); rst=0;
        repeat(4) @(negedge clk);
        send_msg(1,17,0); held_len=1;
        send_msg(1471,33,1);
        send_msg(2,77,0); // Full: neither occupied length may change.
        held_len=0;
        tb_check(accepted==2 && full_drops==1, "two slots full drops the next message");
        read_msg(1,17); pop(); read_msg(1471,33); pop();
        tb_check(!req_valid, "both requests released");
        send_msg(1472,99,0);
        tb_check(!req_valid && long_drops==1, "1472-byte message rejected");
        send_msg(2000,88,1);
        tb_check(!req_valid && long_drops==2, "overlong counter remains saturated");
        port=999; send_msg(3,1,0); port=161;
        tb_check(!req_valid && port_drops==1, "unknown port is not published");
        for(j=1;j<=8;j=j+1) begin
            send_msg(j,100+j,j%2); read_msg(j,100+j); pop();
        end
        tb_check(accepted==10, "valid traffic recovers after overflow and rejection");
        tb_finish("tb_afdx_mgmt_rxbuf"); $finish;
    end
    initial begin #1000000; $fatal(1,"rxbuf timeout"); end
endmodule
