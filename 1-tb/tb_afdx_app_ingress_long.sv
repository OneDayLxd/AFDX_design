/*
 * ================================================================
 * File Name    : tb_afdx_app_ingress_long.sv
 * Module Name  : tb_afdx_app_ingress_long
 * Function:
 *   检查 32KiB 环形字节池、8192 字节边界、跨地址回卷及全消息校验和。
 * Sub Module:
 *   afdx_app_ingress
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_app_ingress_long;
    `include "tb_common.svh"
    reg clk=0, rst=1, valid=0, last=0, ready=0;
    reg [2:0] vl=0;
    reg [7:0] data=0;
    wire [2:0] out_vl, reason;
    wire [7:0] out_data;
    wire out_valid, out_last, drop;
    wire [31:0] drops, accepted;
    wire [15:0] out_len;
    wire [31:0] out_sum;
    always #4 clk=~clk;
    afdx_app_ingress #(
        .VLW      (3),
        .N_VL     (5),
        .SLOT_LOG (4),
        .MAX_LEN  (8192)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .in_vl        (vl),
        .in_data      (data),
        .in_valid     (valid),
        .in_last      (last),
        .out_vl       (out_vl),
        .out_data     (out_data),
        .out_valid    (out_valid),
        .out_last     (out_last),
        .out_ready    (ready),
        .drop_pulse   (drop),
        .drop_reason  (reason),
        .drop_count   (drops),
        .accept_count (accepted),
        .out_len      (out_len),
        .out_sum      (out_sum)
    );
    byte expected_data[$];
    int expected_vl[$], expected_last[$], expected_len[$];
    int unsigned expected_sum[$];
    integer got=0, pulses=0;
    bit held=0;
    reg [11:0] held_beat;
    always @(posedge clk) begin
        if (rst) held=0;
        else begin
            if (held) tb_check(out_valid && {out_vl,out_last,out_data}===held_beat,"stable while stalled");
            held=out_valid && !ready;
            held_beat={out_vl,out_last,out_data};
            if (drop) pulses++;
            if (out_valid && ready) begin
                tb_check(expected_data.size()>0,"no partial/rejected message escapes");
                if (expected_data.size()==0) $fatal(1,"unexpected ingress output");
                tb_check(out_data===expected_data.pop_front(),"ordered payload");
                tb_check(out_vl==expected_vl.pop_front(),"VL metadata");
                tb_check(out_last==expected_last.pop_front(),"last metadata");
                tb_check(out_len==expected_len[0],"full-message length descriptor");
                tb_check(out_sum==expected_sum[0],"full-message payload sum descriptor");
                if(out_last)begin void'(expected_len.pop_front());void'(expected_sum.pop_front());end
                got++;
            end
        end
    end
    task automatic message(input int v, n, seed, gap, change_at, input bit expect_good);
        int unsigned sum=0;
        if(expect_good) begin
            for(int p=0;p<n;p++) sum+=p%2 ? int'(unsigned'(8'(seed+p))) : int'(unsigned'(8'(seed+p)))<<8;
            expected_len.push_back(n);expected_sum.push_back(sum);
        end
        if (expect_good) for(int p=0;p<n;p++) begin
            expected_data.push_back(byte'(seed+p));
            expected_vl.push_back(v);
            expected_last.push_back(p==n-1);
        end
        for(int p=0;p<n;p++) begin
            @(negedge clk);
            valid=1; last=p==n-1; vl=(p==change_at) ? v+1 : v; data=seed+p;
            if (gap && p%3==1) begin
                @(negedge clk); valid=0; last=1;
                repeat(3) @(negedge clk);
            end
        end
        @(negedge clk); valid=0; last=0;
    endtask
    task automatic drain;
        ready=1;
        wait(expected_data.size()==0);
        repeat(12) @(negedge clk);
        ready=0;
    endtask
    initial begin
        repeat(5) @(negedge clk); rst=0;
        for(int m=0;m<4;m++)message(m,8192,m*17,0,-1,1);
        message(4,1,99,0,-1,0);
        tb_check(drops==1 && reason[0],"shared byte pool full, even with free descriptors");
        drain();
        // 大消息与小消息混合、多次穿越 RAM 环形地址与扩展指针回卷。
        for(int r=0;r<10;r++)begin
            message(1,8191,11+r,1,-1,1);
            message(2,1472,32+r,0,-1,1);
            message(3,1,77+r,0,-1,1);
            drain();
            message(1,8193,99,0,-1,0);
            tb_check(reason[1],"overlength rolls back all reserved bytes");
            message(4,8192,r,0,-1,1);drain();
        end
        tb_check(drops==11,"one discard per rejected long message");
        tb_check(expected_len.size()==0 && expected_sum.size()==0,"all descriptors consumed");
        tb_finish("tb_afdx_app_ingress_long"); $finish;
    end
    initial begin #20000000;$fatal(1,"long ingress watchdog");end
endmodule
