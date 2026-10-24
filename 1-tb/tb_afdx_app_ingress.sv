/*
 * ================================================================
 * File Name    : tb_afdx_app_ingress.sv
 * Module Name  : tb_afdx_app_ingress
 * Function:
 *   检查消息原子提交、输入空洞、输出背压、超长/非法 VL/溢出及复位恢复。
 * Sub Module:
 *   afdx_app_ingress
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_app_ingress;
    `include "tb_common.svh"
    reg clk=0, rst=1, valid=0, last=0, ready=0;
    reg [2:0] vl=0;
    reg [7:0] data=0;
    wire [2:0] out_vl, reason;
    wire [7:0] out_data;
    wire out_valid, out_last, drop;
    wire [31:0] drops, accepted;
    always #4 clk=~clk;
    afdx_app_ingress #(
        .VLW      (3),
        .N_VL     (5),
        .SLOT_LOG (1),
        .MAX_LEN  (16)
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
        .accept_count (accepted)
    );
    byte expected_data[$];
    int expected_vl[$], expected_last[$];
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
                got++;
            end
        end
    end
    task automatic message(input int v, n, seed, gap, change_at, input bit expect_good);
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
        message(1,16,20,1,-1,1);
        repeat(8) @(negedge clk);
        tb_check(got==0 && accepted==1,"complete message waits for consumer");
        message(2,1,90,0,-1,1);
        message(3,7,100,0,-1,0);
        tb_check(drops==1 && reason==1,"full buffer drops whole third message");
        drain();
        message(1,17,40,0,-1,0);
        tb_check(drops==2 && reason==2,"MAX_LEN+1 rejected");
        message(1,9,60,1,4,0);
        tb_check(drops==3 && reason==4,"VL change rejected");
        message(7,3,10,0,-1,0);
        tb_check(drops==4 && reason==4,"out of range VL rejected");
        for(int m=0;m<70;m++) begin
            message(m%5,1+m%16,m*11,m%2,-1,1);
            // 随机背压但不依赖随机种子决定覆盖范围。
            while(expected_data.size()) begin
                @(negedge clk); ready=$urandom_range(0,3)!=0;
            end
            repeat(5) @(negedge clk); ready=0;
        end
        tb_check(drops==4 && pulses==4,"one error pulse/count per rejected message");
        // 多次跨越扩展指针回卷，满时不得误判为空或覆盖尚未读出的消息。
        for(int wrap=0;wrap<6;wrap++) begin
            message(1,3,wrap+10,0,-1,1);
            message(2,2,wrap+40,0,-1,1);
            message(3,4,wrap+90,0,-1,0);
            tb_check(drops==5+wrap && reason==1,"full detection survives pointer wrap");
            drain();
        end
        tb_check(pulses==10,"wrapped overflow pulses counted exactly once");
        // 未完成消息不能提交；复位也不得泄漏残留数据。
        @(negedge clk); valid=1; vl=1; data=99; last=0;
        @(negedge clk); valid=0; ready=1;
        repeat(12) @(negedge clk);
        tb_check(!out_valid,"partial message invisible");
        rst=1; repeat(4) @(negedge clk); rst=0; ready=0;
        message(4,16,101,0,-1,1); drain();
        tb_check(accepted==1 && drops==0,"reset flush and recovery");
        tb_finish("tb_afdx_app_ingress"); $finish;
    end
    initial begin #2000000; $fatal(1,"ingress watchdog"); end
endmodule
