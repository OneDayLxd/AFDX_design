/*
 * ================================================================
 * File Name    : tb_afdx_mac_top.sv
 * Module Name  : tb_afdx_mac_top
 *
 * Function:
 *   连接 clk/rst、A/B 数据有效接口及兼容 ready，检查 125MHz MAC 集成层。
 *   无 PHY 管理应答或层次 force；检查自研双 MAC 收发、CRC/IFG、去重、
 *   真实微秒节拍、复位恢复及 rx_ready 为 0/1 时均能收发。
 *
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`default_nettype none
module tb_afdx_mac_top;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    reg rx_ready=0;
    wire tx_ready, b_tx_ready;
    reg b_rx_ready=1;
    always #4 clk=~clk;
    wire [15:0] txd;
    wire [1:0] txv;
    reg [1:0] loop_enable=2'b11;
    reg corrupt_a=0;
    integer frames[0:1]='{0,0};
    integer pos[0:1]='{0,0};
    integer idle_slots[0:1]='{100,100};
    reg active[0:1]='{0,0};
    reg [31:0] crc[0:1];
    integer net, sample_mono;
    wire [15:0] loop_data={txd[15:8], txd[7:0] ^ ((corrupt_a && pos[0]==20) ? 8'h01 : 8'h00)};

    afdx_mac_system #(
        .APP_GEN_EN  (1),
        .MGMT_EN     (1),
        .SKEW_MAX_US (100)
    ) dut (
        .clk         (clk),
        .reset       (rst),
        .tx_data     (txd[7:0]),
        .rx_data     (loop_data[7:0]),
        .tx_valid    (txv[0]),
        .rx_valid    (txv[0] && loop_enable[0]),
        .tx_ready    (tx_ready),
        .rx_ready    (rx_ready),
        .b_tx_data   (txd[15:8]),
        .b_rx_data   (loop_data[15:8]),
        .b_tx_valid  (txv[1]),
        .b_rx_valid  (txv[1] && loop_enable[1]),
        .b_tx_ready  (b_tx_ready),
        .b_rx_ready  (b_rx_ready),
        .rx_er          (1'b0),
        .b_rx_er        (1'b0),
        .tx_clk         (),
        .tx_er          (),
        .b_tx_clk       (),
        .b_tx_er        (),
        .host_tx_vl     ({`AFDX_TX_VLW{1'b0}}),
        .host_tx_valid  (1'b0),
        .host_tx_data   (8'd0),
        .host_tx_last   (1'b0),
        .host_tx_ready  (),
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


    // 在统一 clk 的上升沿读取 TX；同一数据同时作为同步 RX 回环输入。
    always @(posedge clk) begin
        if(rst) begin
            for(net=0;net<2;net++) begin
                frames[net]=0; pos[net]=0; active[net]=0; idle_slots[net]=100;
            end
        end else for(net=0;net<2;net++) begin
            if(txv[net]) begin
                if(!active[net]) begin
                    tb_check(idle_slots[net]>=12,"GMII IFG >=12 byte times");
                    active[net]=1; pos[net]=0; crc[net]=32'hffffffff;
                end
                if(pos[net]<7) tb_check(txd[net*8 +: 8]===8'h55,"GMII preamble");
                else if(pos[net]==7) tb_check(txd[net*8 +: 8]===8'hd5,"GMII SFD");
                else crc[net]=tb_crc32_byte(crc[net],txd[net*8 +: 8]);
                pos[net]++;
                idle_slots[net]=0;
            end else begin
                if(active[net]) begin
                    tb_check(pos[net]>=72 && pos[net]<=1526,"complete GMII frame length");
                    tb_check(crc[net]===32'hdebb20e3,"Ethernet FCS residue");
                    frames[net]++; active[net]=0;
                end
                idle_slots[net]++;
            end
        end
    end

    task automatic check_phase;
        tb_check(tx_ready===1'b1 && b_tx_ready===1'b1,"both template tx_ready tied high");
        tb_check(frames[0]>0 && frames[1]>0,"both network outputs active");
        tb_check(dut.u_system.cnt_sink_msgs>=8,"application receives generated messages");
        tb_check(dut.u_system.cnt_rx_rm_drop>=4,"A/B copies delivered only once");
        tb_check(dut.u_system.cnt_rx_mac_error==0 &&
                 dut.u_system.cnt_rx_fcs_err==0,"no MAC/FCS error");
        tb_check(dut.u_system.cnt_rx_hdr_err==0 &&
                 dut.u_system.cnt_rx_csum_err==0,"no header/checksum error");
        tb_check(dut.u_system.cnt_rx_mac_ovf==0 &&
                 dut.u_system.cnt_rx_full_drop==0,"no overflow");
        tb_check(dut.u_system.cnt_rx_ic_drop==0,"no sequence integrity error");
    endtask

    task automatic check_run;
        wait(!rst && !dut.u_system.rst);
        repeat(25000) @(negedge clk);
        check_phase();
        sample_mono=dut.u_system.u_core.mono_us;
        repeat(12500) @(negedge clk);
        tb_check(dut.u_system.u_core.mono_us-sample_mono==100,
            "12500 input cycles represent 100us, BAG/RTC timebase preserved");
    endtask

    initial begin
        repeat(20) @(negedge clk); rst=0;
        check_run();
        // A/B ready 取相反值，重复复位后交换，均不影响收发。
        rst=1; rx_ready=1; b_rx_ready=0; repeat(20) @(negedge clk); rst=0;
        check_run();
        // 单网静默、单网坏 FCS 仍由另一网交付，随后恢复双网并去重。
        loop_enable=2'b01;
        rst=1; repeat(20) @(negedge clk); rst=0;
        repeat(25000) @(negedge clk);
        tb_check(dut.u_system.cnt_sink_msgs>=8,"A-only link delivers");
        tb_check(dut.u_system.cnt_rx_rm_drop==0,"no duplicate when B silent");
        loop_enable=2'b10;
        rst=1; repeat(20) @(negedge clk); rst=0;
        repeat(25000) @(negedge clk);
        tb_check(dut.u_system.cnt_sink_msgs>=8,"B-only link delivers");
        tb_check(dut.u_system.cnt_rx_rm_drop==0,"no duplicate when A silent");
        loop_enable=2'b11; corrupt_a=1;
        rst=1; repeat(20) @(negedge clk); rst=0;
        repeat(25000) @(negedge clk);
        tb_check(dut.u_system.cnt_sink_msgs>=8,"good B survives corrupt A");
        tb_check(dut.u_system.cnt_rx_mac_error>0 &&
                 dut.u_system.cnt_rx_rm_drop==0,"corrupt A rejected before duplicate merge");
        corrupt_a=0;
        rst=1; repeat(20) @(negedge clk); rst=0;
        check_run();
        tb_finish("tb_afdx_mac_top"); $finish;
    end
    initial begin #20000000; $fatal(1,"synchronous MAC top watchdog"); end
endmodule
`default_nettype wire
