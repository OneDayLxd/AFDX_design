/*
 * ================================================================
 * File Name    : tb_afdx_phase_scheduler.sv
 * Module Name  : tb_afdx_phase_scheduler
 *
 * Function:
 *   时隙授权、窗口关闭撤销、非二次幂 VL 数量、BAG=0 突发和复位验证。
 * Sub Module:
 *   afdx_bag_scheduler
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_phase_scheduler;
    `include "tb_common.svh"
    reg clk = 0, rst = 1;
    always #4 clk = ~clk;
    reg [16:0] nonempty = '1;
    wire grant_valid;
    wire [4:0] grant_vl;
    reg grant_ack = 0;
    reg accept = 1;
    integer cycle = 0, slot = 0, round = 0, grants = 0;
    integer accepted[0:16];
    wire bag_grant, bag_open;
    wire bag_ack = bag_grant;
    integer us_cycle=0, bag_grants=0;
    time previous_bag_ack=0;
    afdx_bag_scheduler #(
        .N_VL                (1),
        .VLW                 (1),
        .PHASE_ENABLE        (1),
        .PHASE_SLOT_CYCLES   (64),
        .PHASE_OPEN_CYCLE    (8),
        .PHASE_WINDOW_CYCLES (24)
    ) u_bag_intersection (
        .clk          (clk),
        .rst          (rst),
        .tick_us      (us_cycle==124),
        .nonempty     (1'b1),
        .queue_pop    (1'b0),
        .grant_bag_us (20'd5),
        .grant_valid  (bag_grant),
        .grant_vl     (),
        .grant_ack    (bag_ack),
        .bag_open     (bag_open)
    );
    always @(posedge clk) begin
        if(rst) begin us_cycle<=0; previous_bag_ack=0; end
        else begin
            us_cycle<=us_cycle==124 ? 0 : us_cycle+1;
            if(bag_ack) begin
                tb_check(bag_open, "phase opening does not bypass closed BAG");
                if(previous_bag_ack)
                    tb_check($time-previous_bag_ack>=4000, "5us coarse BAG cannot release every 512ns slot");
                previous_bag_ack=$time; bag_grants++;
            end
        end
    end
    afdx_bag_scheduler #(
        .N_VL                (17),
        .VLW                 (5),
        .PHASE_ENABLE        (1),
        .PHASE_SLOT_CYCLES   (80),
        .PHASE_OPEN_CYCLE    (20),
        .PHASE_WINDOW_CYCLES (20)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .tick_us      (1'b0),
        .nonempty     (nonempty),
        .queue_pop    (1'b0),
        .grant_bag_us (20'd0),
        .grant_valid  (grant_valid),
        .grant_vl     (grant_vl),
        .grant_ack    (grant_ack),
        .bag_open     ()
    );
    // 第三轮不确认，第四轮在闭窗后才入队；两种情况都必须等待下轮。
    reg pending = 0;
    always @(negedge clk) begin
        if (rst) begin grant_ack=0; pending=0; end
        else begin
            nonempty=(round==3 && cycle<60) ? 17'd0 : {17{1'b1}};
            grant_ack=pending && accept;
            pending=grant_valid && !grant_ack && accept;
        end
    end
    always @(posedge clk) begin
        if (rst) begin cycle=0; slot=0; round=0; grants=0; end
        else begin
            if (grant_ack) begin
                tb_check(grant_vl==slot, "grant matches slot, including N_VL=17 wrap");
                tb_check(cycle>=20 && cycle<=42, "ack bounded by grant window and handshake latency");
                grants++;
                accepted[slot]++;
                tb_check(grants==1, "at most one grant per slot even with BAG=0");
            end
            if (cycle==79) begin
                if (round!=2 && round!=3) tb_check(grants==1, "every eligible VL served once");
                else tb_check(grants==0, "no fabricated acknowledgment under stall");
                cycle=0; grants=0;
                if (slot==16) begin slot=0; round++; end
                else slot++;
            end else cycle++;
            if (cycle>=50) tb_check(!grant_valid, "no stale grant outside window, including late arrival");
        end
    end
    initial begin
        for (integer v=0;v<17;v++) accepted[v]=0;
        repeat(8) @(negedge clk);
        rst=0;
        wait(round==2); @(negedge clk); accept=0;
        wait(round==3); @(negedge clk); accept=1;
        wait(round==5); @(negedge clk);
        for (integer v=0;v<17;v++) tb_check(accepted[v]==3, "all VLs have three accepted rounds");
        rst=1; repeat(4) @(negedge clk); rst=0;
        wait(round==1); @(negedge clk);
        tb_check(bag_grants>5, "BAG/phase intersection exercised repeatedly");
        tb_finish("tb_afdx_phase_scheduler");
        $finish;
    end
    initial begin #1000000; $fatal(1,"phase scheduler timeout"); end
endmodule
