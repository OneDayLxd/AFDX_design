/*
 * ================================================================
 * File Name    : tb_afdx_mgmt_window.sv
 * Module Name  : tb_afdx_mgmt_window
 *
 * Function:
 *   连续 125MHz、自研双 MAC：复现 VL0 的 BAG 到期错过管理窗口，
 *   检查窗口预约、两网 BAG、载荷/SN/FCS、其他 VL 进展与复位。
 *
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module tb_afdx_mgmt_window #(
    parameter WINDOW_PERIOD_US = 2000,
    parameter WINDOW_OPEN_US   = 8
);
    `include "tb_common.svh"
    reg clk=0, reset=1;
    always #4 clk=~clk;
    reg [6:0] host_vl=0;
    reg [7:0] host_data=0;
    reg host_valid=0, host_last=0;
    wire host_ready;
    wire [15:0] txd;
    wire [1:0] valid, error;
    afdx_mac_system #(
        .APP_EXTERNAL (1),
        .APP_GEN_EN   (0),
        .MGMT_EN      (0)
    ) dut (
        .clk            (clk),
        .reset          (reset),
        .tx_data        (txd[7:0]),
        .rx_data        (8'd0),
        .tx_valid       (valid[0]),
        .rx_valid       (1'b0),
        .tx_ready       (),
        .rx_ready       (1'b0),
        .rx_er          (1'b0),
        .tx_clk         (),
        .tx_er          (error[0]),
        .b_tx_data      (txd[15:8]),
        .b_rx_data      (8'd0),
        .b_tx_valid     (valid[1]),
        .b_rx_valid     (1'b0),
        .b_tx_ready     (),
        .b_rx_ready     (1'b0),
        .b_rx_er        (1'b0),
        .b_tx_clk       (),
        .b_tx_er        (error[1]),
        .host_tx_vl     (host_vl),
        .host_tx_valid  (host_valid),
        .host_tx_data   (host_data),
        .host_tx_last   (host_last),
        .host_tx_ready  (host_ready),
        .host_rx_vl     (),
        .host_rx_valid  (),
        .host_rx_data   (),
        .host_rx_last   (),
        .host_rx_ready  (1'b1),
        .host_rx_len    (),
        .host_rx_sn     (),
        .host_rx_net    (),
        .host_rx_ts     (),
        .host_rx_src_ip (),
        .host_rx_sport  (),
        .host_rx_dport  ()
    );
    // 仅配置真实 DUT 参数，分别验证窄窗、全时开窗和关闭窗口机制。
    defparam dut.u_system.u_core.MGMT_WIN_PERIOD_US = WINDOW_PERIOD_US;
    defparam dut.u_system.u_core.MGMT_WIN_OPEN_US = WINDOW_OPEN_US;
    localparam LIMITED_WINDOW = WINDOW_PERIOD_US!=0 && WINDOW_OPEN_US<WINDOW_PERIOD_US;
    task automatic submit(input integer vl, length, id);
        for(integer i=0; i<length; i++) begin
            @(negedge clk);
            host_vl=vl; host_valid=1; host_last=i==length-1;
            host_data=8'(id+i*7);
            @(posedge clk);
            while(!host_ready) @(posedge clk);
        end
        @(negedge clk); host_valid=0; host_last=0;
    endtask
    integer count[0:1][0:1]='{'{0,0},'{0,0}};
    integer pos[0:1]='{0,0};
    bit active[0:1]='{0,0};
    byte frame[0:1][0:1525];
    time sfd[0:1], previous[0:1][0:1];
    integer epoch=0;
    time reset_end=0;
    bit previous_open=0;
    integer grants_this_window=0;
    integer waiting_oracle=0;
    bit oracle_update=0;
    bit previous_credit=0;
    always @(posedge clk) begin
        if(reset) begin
            previous_open=0; grants_this_window=0; waiting_oracle=0; oracle_update=0; previous_credit=0;
        end
        else begin
            if(LIMITED_WINDOW && dut.u_system.u_core.win_credit && !previous_credit)
                tb_check(waiting_oracle>0, "reservation never belongs to an already granted frame");
            previous_credit=dut.u_system.u_core.win_credit;
            if(oracle_update || dut.u_system.u_core.u_txq.commit || dut.u_system.u_core.grant_ack)
                tb_check(dut.u_system.u_core.win_waiting==waiting_oracle,
                         "ungranted count agrees with independent queue descriptor events");
            oracle_update=dut.u_system.u_core.u_txq.commit || dut.u_system.u_core.grant_ack;
            if(dut.u_system.u_core.u_txq.commit && dut.u_system.u_core.u_txq.commit_vl==0)
                waiting_oracle++;
            if(dut.u_system.u_core.win_open && !previous_open) grants_this_window=0;
            previous_open=dut.u_system.u_core.win_open;
            if(dut.u_system.u_core.grant_ack && dut.u_system.u_core.grant_vl==0) begin
                tb_check(waiting_oracle>0, "grant cannot underflow ungranted count");
                waiting_oracle--;
                grants_this_window++;
                if(LIMITED_WINDOW)
                    tb_check(grants_this_window<=1, "at most one grant per management window");
            end
        end
    end
    always @(posedge clk) if(!reset) begin
        for(integer n=0;n<2;n++) begin
            if(valid[n]) begin
                if(!active[n]) begin active[n]=1; pos[n]=0; end
                frame[n][pos[n]]=txd[n*8+:8];
                if(pos[n]==7) sfd[n]=$time;
                pos[n]++;
                tb_check(!error[n], "MAC output has no error");
            end else if(active[n]) begin : finish_frame
                integer vl, length, id;
                reg [31:0] crc;
                time gap;
                active[n]=0;
                vl=frame[n][13]; length=vl==0 ? 516 : 17;
                id=count[n][vl]; crc=32'hffffffff;
                tb_check(vl<2 && pos[n]==8+42+length+1+4, "wire frame length and VL");
                tb_check(frame[n][7]==8'hd5, "SFD");
                for(integer i=8;i<pos[n];i++) crc=tb_crc32_byte(crc,frame[n][i]);
                tb_check(crc==32'hdebb20e3, "independent FCS check");
                for(integer i=0;i<length;i++)
                    tb_check(frame[n][50+i]===8'(id+i*7), "payload FIFO order");
                tb_check(frame[n][50+length]===8'(id+1), "SN FIFO order");
                if(vl==0 && id==0) begin
                    if(LIMITED_WINDOW && WINDOW_PERIOD_US==2000)
                        tb_check(sfd[n]>=reset_end+2000000, "empty initial window grants no future credit");
                    else if(!LIMITED_WINDOW)
                        tb_check(sfd[n]<reset_end+2000000, "disabled/full window cannot block the first frame");
                end
                if(id!=0) begin
                    gap=sfd[n]-previous[n][vl];
                    tb_check(gap>=2000000, "physical same-VL BAG >=2ms");
                    if(vl==0 && epoch==0) begin
                        $display("WINDOW_GAP net=%0d frame=%0d sfd_ns=%0d gap_ns=%0d",n,id,sfd[n],gap);
                        tb_check(gap<2200000, "VL0 does not wait an extra 2ms window");
                    end
                end
                previous[n][vl]=sfd[n]; count[n][vl]++;
            end
        end
    end
    initial begin
        for(integer n=0;n<2;n++) for(integer v=0;v<2;v++) previous[n][v]=0;
        repeat(20) @(negedge clk); reset=0; reset_end=$time;
        // 错过复位后的空窗口，第一帧仍须等待 2ms 窗口；随后不得隔窗发送。
        #100000;
        for(integer i=0;i<6;i++) begin submit(0,516,i); submit(1,17,i); end
        wait(count[0][0]==6 && count[1][0]==6 && count[0][1]==6 && count[1][1]==6);
        tb_check(previous[0][0]<12500000, "six VL0 frames finish without skipped window");
        // 无排队消息的窗口不能为未来的窗口外到达预存资格。
        epoch=1;
        #3000000;
        @(negedge clk); reset=1;
        repeat(20) @(negedge clk);
        for(integer n=0;n<2;n++) begin
            active[n]=0;
            for(integer v=0;v<2;v++) begin count[n][v]=0; previous[n][v]=0; end
        end
        reset=0; reset_end=$time;
        #100000; submit(0,516,0);
        wait(count[0][0]==1 && count[1][0]==1);
        // 在预约已产生、上一帧 BAG 尚未到期时复位，旧预约和旧消息均不得泄漏。
        submit(0,516,1);
        wait(dut.u_system.u_core.win_open);
        repeat(3) @(negedge clk);
        tb_check(!dut.u_system.u_core.mac_a_bag_open[0] &&
                 !dut.u_system.u_core.mac_b_bag_open[0], "reset exercises pending BAG-bound reservation");
        reset=1; epoch=2;
        repeat(20) @(negedge clk);
        for(integer n=0;n<2;n++) begin
            active[n]=0;
            for(integer v=0;v<2;v++) begin count[n][v]=0; previous[n][v]=0; end
        end
        reset=0; reset_end=$time;
        #100000; submit(0,516,0);
        wait(count[0][0]==1 && count[1][0]==1);
        #20000;
        tb_check(count[0][0]==1 && count[1][0]==1, "no old message or reservation survives reset");
        tb_finish("tb_afdx_mgmt_window"); $finish;
    end
    initial begin #30000000; $fatal(1,"management window watchdog"); end
endmodule
