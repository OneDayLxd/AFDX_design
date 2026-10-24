/*
 * ================================================================
 * File Name    : tb_afdx_tx_snapshot.sv
 * Module Name  : tb_afdx_tx_snapshot
 *
 * Function:
 *   复现晚到低编号 VL 被持续高编号到达流越过的问题；检查快照轮询、
 *   授权背压稳定、128 VL 公平性、出队撤销和中途复位。
 *
 * Sub Module:
 *   afdx_bag_scheduler
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_tx_snapshot;
    `include "tb_common.svh"
    reg clk = 0, rst = 1, ack = 0, queue_pop = 0;
    always #4 clk = ~clk;
    reg [127:0] nonempty = 0;
    wire valid;
    wire [6:0] vl;
    afdx_bag_scheduler #(
        .N_VL         (128),
        .VLW          (7)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .tick_us      (1'b0),
        .nonempty     (nonempty),
        .queue_pop    (queue_pop),
        .grant_bag_us (20'd0),
        .grant_valid  (valid),
        .grant_vl     (vl),
        .grant_ack    (ack),
        .bag_open     ()
    );
    task automatic candidate(input integer expected_vl);
        begin
            wait(valid);
            @(negedge clk);
            tb_check(vl == expected_vl, $sformatf("snapshot candidate expected=%0d actual=%0d", expected_vl, vl));
        end
    endtask
    task automatic consume(input bit remove_message);
        begin
            if(remove_message) nonempty[vl] = 0;
            ack = 1;
            @(negedge clk); ack = 0;
        end
    endtask
    integer seen [0:127];
    integer held_vl;
    initial begin
        for(integer v=0; v<128; v++) seen[v]=0;
        repeat(5) @(negedge clk);
        rst=0; nonempty[40]=1;
        candidate(40); consume(1);
        nonempty[41]=1;
        candidate(41);
        nonempty[1]=1; nonempty[42]=1;
        repeat(32) begin
            @(negedge clk);
            tb_check(valid && vl==41, "held grant is stable despite late arrivals");
        end
        consume(1);
        candidate(42); nonempty[43]=1;
        repeat(32) @(negedge clk);
        consume(1);
        candidate(1); consume(1);
        candidate(43);
        // 出队撤销未接受的授权，不能丢掉该 VL 的待服务资格。
        queue_pop=1;
        @(negedge clk); queue_pop=0;
        tb_check(!valid, "queue pop withdraws unaccepted candidate");
        candidate(43); consume(1);
        repeat(24) @(negedge clk);
        tb_check(!valid, "no phantom grant after queue drains");
        // 持续非空的 VL 每轮只被接受一次，不能被同轮新消息反复抢占。
        nonempty='1;
        for(integer i=0; i<128*4; i++) begin
            wait(valid); @(negedge clk);
            seen[vl]++;
            consume(0);
        end
        for(integer v=0; v<128; v++)
            tb_check(seen[v]==4, "all 128 continuously eligible VLs receive four grants");
        wait(valid); @(negedge clk); held_vl=vl;
        repeat(12) begin
            @(negedge clk);
            tb_check(valid && vl==held_vl, "continuous traffic cannot change stalled grant");
        end
        rst=1; nonempty=0;
        repeat(5) @(negedge clk);
        tb_check(!valid, "reset clears stalled grant");
        rst=0; nonempty[127]=1;
        candidate(127); consume(1);
        repeat(24) @(negedge clk);
        tb_check(!valid, "reset clears stale snapshot members");
        tb_finish("tb_afdx_tx_snapshot"); $finish;
    end
    initial begin #1000000; $fatal(1,"snapshot scheduler watchdog"); end
endmodule
