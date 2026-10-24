/*
 * ================================================================
 * File Name    : tb_afdx_post_synth_smoke.sv
 * Module Name  : tb_afdx_post_synth_smoke
 *
 * Function:
 *   小规模综合前后数据路径对照；125MHz，4 TX / 4 RX VL，BAG=2ms。
 *   真实 GMII 回环检查发送、接收、冗余去重和坏帧隔离。
 *   接收计数只读观察，不强制或修改 DUT 内部状态。
 * Sub Module:
 *   AFDX_smoke_top
 * ================================================================
 */
`timescale 1ns/1ps
`default_nettype none
module tb_afdx_post_synth_smoke;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    wire [15:0] txd;
    wire [1:0] txv, txready;
    reg [1:0] loop_en=2'b11, ready=2'b01;
    reg bad_a=0;
    integer pos[2]='{0,0}, idle[2]='{100,100}, frames[2]='{0,0};
    wire [15:0] rxd = {txd[15:8], (bad_a && pos[0]==20) ? txd[7:0]^8'h01 : txd[7:0]};
    always #4 clk=~clk;
    AFDX_smoke_top dut (
        .clk         (clk),
        .rst         (rst),
        .tx_data     (txd[7:0]),
        .rx_data     (rxd[7:0]),
        .tx_valid    (txv[0]),
        .rx_valid    (txv[0] && loop_en[0]),
        .tx_ready    (txready[0]),
        .rx_ready    (ready[0]),
        .b_tx_data   (txd[15:8]),
        .b_rx_data   (rxd[15:8]),
        .b_tx_valid  (txv[1]),
        .b_rx_valid  (txv[1] && loop_en[1]),
        .b_tx_ready  (txready[1]),
        .b_rx_ready  (ready[1])
    );
`ifdef POST_SYNTH
    // 官方 .vo 中保留的计数器总线；仅用于观测实际综合网表。
    wire [31:0] accepted  = dut.\u_production|u_mac_system|u_system|u_core|cnt_rx_accepted ;
    wire [31:0] delivered = dut.\u_production|u_mac_system|u_system|u_core|cnt_rx_delivered ;
    wire [31:0] rm_drop   = dut.\u_production|u_mac_system|u_system|u_core|cnt_rx_rm_drop ;
    wire [31:0] error_a   = dut.\u_production|u_mac_system|g_net[0].u_link|rx_error_count ;
    wire [31:0] error_b   = dut.\u_production|u_mac_system|g_net[1].u_link|rx_error_count ;
`else
    wire [31:0] accepted  = dut.u_production.u_mac_system.u_system.cnt_rx_accepted;
    wire [31:0] delivered = dut.u_production.u_mac_system.u_system.cnt_rx_delivered;
    wire [31:0] rm_drop   = dut.u_production.u_mac_system.u_system.cnt_rx_rm_drop;
    wire [31:0] error_a   = dut.u_production.u_mac_system.g_net[0].u_link.rx_error_count;
    wire [31:0] error_b   = dut.u_production.u_mac_system.g_net[1].u_link.rx_error_count;
`endif
    bit active[2]='{0,0};
    reg [31:0] crc[2];
    byte unsigned cap[2][1600];
    integer net,j,fd,phase=0;
    reg [1023:0] cap_name;

    always @(posedge clk) begin
        if(!rst) begin
            tb_check(txready===2'b11,"ready outputs fixed high");
            tb_check(!$isunknown(txv),"TX valid is known");
            for(net=0;net<2;net++) begin
                if(txv[net]===1'b1) begin
                    tb_check(!$isunknown(txd[net*8 +:8]),"TX byte is known");
                    if(!active[net]) begin
                        tb_check(idle[net]>=12,"GMII IFG >=12 cycles");
                        active[net]=1; pos[net]=0; crc[net]=32'hffffffff;
                    end
                    if(pos[net]>=1600) $fatal(1,"unterminated frame");
                    cap[net][pos[net]]=txd[net*8 +:8];
                    if(pos[net]<7) tb_check(txd[net*8 +:8]===8'h55,"preamble");
                    else if(pos[net]==7) tb_check(txd[net*8 +:8]===8'hd5,"SFD");
                    else crc[net]=tb_crc32_byte(crc[net],txd[net*8 +:8]);
                    pos[net]++; idle[net]=0;
                end else begin
                    if(active[net]) begin
                        tb_check(pos[net]>=72 && pos[net]<=1526,"GMII length");
                        tb_check(crc[net]===32'hdebb20e3,"FCS residue");
                        $fwrite(fd,"F %0t %0d %0d",$time,net,pos[net]);
                        for(j=0;j<pos[net];j++) $fwrite(fd," %02x",cap[net][j]);
                        $fwrite(fd,"\n"); $fflush(fd);
                        active[net]=0; frames[net]++;
                    end
                    idle[net]++;
                end
            end
        end
    end

    task automatic run_phase(input [1:0] mask, input bit corrupt);
        integer k;
        begin
            @(negedge clk); rst=1; loop_en=mask; bad_a=corrupt; ready=~ready;
            repeat(40) @(negedge clk);
            for(k=0;k<2;k++) begin frames[k]=0; pos[k]=0; idle[k]=100; active[k]=0; end
            rst=0; phase++;
            $display("PHASE %0d start, mask=%b bad_a=%0d at %0t",phase,mask,corrupt,$time);
            // 等待初始化、首轮三个应用 VL 以及接收交付；不缩短 BAG。
            repeat(3750) @(negedge clk);
            tb_check(frames[0]>=3 && frames[1]>=3,"both network TX paths active");
            tb_check(!active[0] && !active[1],"frames completed before check");
            tb_check(accepted===frames[0] && delivered===frames[0],"each unique message delivered once");
            tb_check(rm_drop===(mask==2'b11 && !corrupt ? frames[1] : 0),"A/B redundancy duplicate count");
            tb_check(error_a===(corrupt ? frames[0] : 0) && error_b===0,"corrupt A rejected, good B unaffected");
            $display("PHASE %0d frames=%0d/%0d accepted=%0d delivered=%0d rm=%0d errors=%0d/%0d",
                phase,frames[0],frames[1],accepted,delivered,rm_drop,error_a,error_b);
        end
    endtask

    initial begin
        if(!$value$plusargs("CAP=%s",cap_name)) cap_name="post_synth_smoke_capture.txt";
        fd=$fopen(cap_name,"w"); if(fd==0) $fatal(1,"cannot open capture");
        run_phase(2'b11,0);
        run_phase(2'b01,0);
        run_phase(2'b11,1);
        $fclose(fd); tb_finish("tb_afdx_post_synth_smoke"); $finish;
    end
    initial begin #100000; $fatal(1,"small netlist watchdog"); end
endmodule
`default_nettype wire
