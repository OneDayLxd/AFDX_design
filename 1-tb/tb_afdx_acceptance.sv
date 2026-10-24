/*
 * ================================================================
 * File Name    : tb_afdx_acceptance.sv
 * Module Name  : tb_afdx_acceptance
 * Function:
 *   双 GMII 工程诊断：应用有效吞吐、128 TX/4096 RX VL、
 *   变长帧应用回送的 SFD 延迟与抖动；此回送不是官方同帧边界验收。
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
`default_nettype none
module tb_afdx_acceptance;
    `include "tb_common.svh"
    reg clk=0, reset=1;
    always #4 clk=~clk;
    wire gtx_clk=clk;
    wire [1:0] rclk={2{clk}};
    wire [1:0] tx_clk, tx_en, tx_er;
    wire [15:0] txd;
    reg loop_enable=1, inject_valid=0;
    reg [7:0] inject_data=0;
    wire [1:0] rx_dv=loop_enable ? tx_en : {1'b0,inject_valid};
    wire [15:0] rxd=loop_enable ? txd : {8'd0,inject_data};
    reg [6:0] host_tx_vl=0;
    reg host_tx_valid=0, host_tx_last=0;
    reg [7:0] host_tx_data=0;
    wire host_tx_ready;
    wire [11:0] host_rx_vl;
    wire host_rx_valid, host_rx_last, host_rx_net;
    wire [7:0] host_rx_data, host_rx_sn;
    wire [10:0] host_rx_len;
    wire [31:0] host_rx_ts;
    afdx_mac_system #(
        .APP_EXTERNAL (1),
        .APP_GEN_EN   (0),
        .MGMT_EN      (0),
        .SKEW_MAX_US  (10000)
    ) dut (
        .clk                         (clk),
        .reset                       (reset),
        .tx_data                     (txd[7:0]),
        .rx_data                     (rxd[7:0]),
        .tx_valid                    (tx_en[0]),
        .rx_valid                    (rx_dv[0]),
        .tx_ready                    (),
        .rx_ready                    (1'b0),
        .rx_er                       (1'b0),
        .tx_clk                      (tx_clk[0]),
        .tx_er                       (tx_er[0]),
        .b_tx_data                   (txd[15:8]),
        .b_rx_data                   (rxd[15:8]),
        .b_tx_valid                  (tx_en[1]),
        .b_rx_valid                  (rx_dv[1]),
        .b_tx_ready                  (),
        .b_rx_ready                  (1'b0),
        .b_rx_er                     (1'b0),
        .b_tx_clk                    (tx_clk[1]),
        .b_tx_er                     (tx_er[1]),
        .host_tx_vl                  (host_tx_vl),
        .host_tx_valid               (host_tx_valid),
        .host_tx_data                (host_tx_data),
        .host_tx_last                (host_tx_last),
        .host_tx_ready               (host_tx_ready),
        .host_rx_vl                  (host_rx_vl),
        .host_rx_valid               (host_rx_valid),
        .host_rx_data                (host_rx_data),
        .host_rx_last                (host_rx_last),
        .host_rx_ready               (1'b1),
        .host_rx_len                 (host_rx_len),
        .host_rx_sn                  (host_rx_sn),
        .host_rx_net                 (host_rx_net),
        .host_rx_ts                  (host_rx_ts),
        .host_rx_src_ip              (),
        .host_rx_sport               (),
        .host_rx_dport               ()
    );

    integer phase=0, received=0, rxpos=0, perf_received=0;
    integer next_rx_sn[0:4095], next_tx_sn[0:127];
    bit tx_covered[0:127], rx_covered[0:4095];
    integer expected_length[0:4095], expected_seed[0:4095];
    byte echo_payload[0:1470];
    time perf_start, perf_end, injection_sfd, delay_ns[0:3];
    time app_rx_first, app_rx_end, app_tx_first, app_tx_end;
    integer echo_index=0, echo_completed=0;
    function automatic byte pattern(input integer seed, pos);
        return byte'(seed+pos*7);
    endfunction
    always @(posedge clk) if(!reset && host_rx_valid) begin
        integer errors_before;
        errors_before=tb_errors;
        if(rxpos==0) app_rx_first=$time;
        tb_check(host_rx_sn==next_rx_sn[host_rx_vl],"SN order / duplicate suppression");
        tb_check(host_rx_data===pattern(expected_seed[host_rx_vl],rxpos),"application payload bytes");
        tb_check(host_rx_len==expected_length[host_rx_vl],"application message length");
        tb_check(host_rx_last===(rxpos==host_rx_len-1),"application message boundary");
        if(tb_errors!=errors_before) $fatal(1,"payload mismatch phase=%0d vl=%0d sn=%0d pos=%0d",phase,host_rx_vl,host_rx_sn,rxpos);
        echo_payload[rxpos]=host_rx_data;
        rxpos++;
        if(host_rx_last) begin
            app_rx_end=$time;
            next_rx_sn[host_rx_vl]=host_rx_sn==255?1:host_rx_sn+1;
            rx_covered[host_rx_vl]=1; received++; rxpos=0;
            if(phase==1) begin
                perf_received++; perf_end=$time;
                if(perf_received%32==0) $display("PERF delivered=%0d time=%0t",perf_received,$time);
            end
        end
    end
    integer wp[0:1], vf[0:1], wire_vl[0:1], idle_slots[0:1];
    reg wire_active[0:1];
    reg [31:0] crc[0:1];
    time sof[0:1], previous_sof[0:1][0:127];
    always @(negedge gtx_clk) if(!reset) begin
        for(integer k=0;k<2;k++) begin
            if(!tx_en[k]) idle_slots[k]++;
            if(tx_en[k]) begin
                if(!wire_active[k]) begin
                    wire_active[k]=1; wp[k]=0; crc[k]=32'hffffffff; sof[k]=$time;
                    tb_check(idle_slots[k]>=12,"GMII IFG >= 12 byte times");
                end
                if(wp[k]<7) tb_check(txd[k*8 +: 8]===8'h55,"GMII preamble");
                else if(wp[k]==7) begin
                    tb_check(txd[k*8 +: 8]===8'hd5,"GMII SFD");
                    if(phase==4 && k==0) delay_ns[echo_index]=$time-injection_sfd;
                end else crc[k]=tb_crc32_byte(crc[k],txd[k*8 +: 8]);
                if(wp[k]==13) wire_vl[k]=txd[k*8 +: 8];
                if(phase==4 && wp[k]>=50 && wp[k]<50+expected_length[1])
                    tb_check(txd[k*8 +: 8]===echo_payload[wp[k]-50],"GMII echo payload identity");
                tb_check(!tx_er[k],"GMII no underflow/error");
                wp[k]++; idle_slots[k]=0;
                if(wp[k]>1526) $fatal(1,"overlong GMII frame");
            end else if(wire_active[k]) begin
                tb_check(crc[k]===32'hdebb20e3,"GMII independent FCS check");
                tb_check(wp[k]>=72 && wp[k]<=1526,"standard Ethernet frame length");
                tb_check(wire_vl[k]<128,"TX VL in range");
                if(previous_sof[k][wire_vl[k]]!=0)
                    tb_check(sof[k]-previous_sof[k][wire_vl[k]]>=2000000,"per-VL BAG at GMII");
                previous_sof[k][wire_vl[k]]=sof[k];
                tx_covered[wire_vl[k]]=1;
                wire_active[k]=0; vf[k]++;
                if(phase==4 && k==0) echo_completed++;
            end
        end
    end
    task automatic send_message(input integer vl, length, seed, input bit from_echo=0);
        expected_length[vl]=length; expected_seed[vl]=seed;
        for(integer j=0;j<length;j++) begin
            @(negedge clk);
            host_tx_vl=vl; host_tx_valid=1; host_tx_last=j==length-1;
            host_tx_data=from_echo ? echo_payload[j] : pattern(seed,j);
            @(posedge clk);
            while(!host_tx_ready) @(posedge clk);
            if(j==0) app_tx_first=$time;
            if(j==length-1) app_tx_end=$time;
        end
        @(negedge clk); host_tx_valid=0; host_tx_last=0;
        next_tx_sn[vl]=next_tx_sn[vl]==255?1:next_tx_sn[vl]+1;
    endtask
    task automatic wait_messages(input integer total);
        integer cyc=0;
        while(received<total && cyc<2000000) begin @(negedge clk); cyc++; end
        tb_check(received==total,"all offered messages delivered");
        if(received!=total) $fatal(1,"delivery stalled %0d/%0d hdr=%0d csum=%0d ic=%0d full=%0d",
            received,total,dut.u_system.cnt_rx_hdr_err,dut.u_system.cnt_rx_csum_err,
            dut.u_system.cnt_rx_ic_drop,dut.u_system.cnt_rx_full_drop);
    endtask
    function automatic [15:0] checksum(input [31:0] s);
        reg [31:0] t;
        t=s[15:0]+s[31:16]; t=t[15:0]+t[31:16]; return ~t[15:0];
    endfunction
    task automatic inject(input integer vl, length, seed);
        byte f[0:1517];
        reg [31:0] sum,c;
        reg [15:0] v,iplen,uplen,ck;
        integer last_body;
        for(integer j=0;j<1518;j++) f[j]=0;
        v=16'h1000+vl; iplen=28+length; uplen=8+length;
        f[0]=3; f[4]=v>>8; f[5]=v;
        f[6]=2; f[9]=1; f[10]=1; f[11]=8'h21; f[12]=8'h08;
        f[14]=8'h45; f[16]=iplen>>8; f[17]=iplen; f[22]=1; f[23]=17;
        f[26]=10; f[27]=1; f[28]=1; f[29]=1;
        f[30]=8'he0; f[31]=8'he0; f[32]=v>>8; f[33]=v;
        sum=0; for(integer j=14;j<34;j+=2) sum+={f[j],f[j+1]};
        ck=checksum(sum); f[24]=ck>>8; f[25]=ck;
        f[34]=16'(1000+vl)>>8; f[35]=1000+vl;
        f[36]=16'(2000+vl)>>8; f[37]=2000+vl;
        f[38]=uplen>>8; f[39]=uplen; // IPv4 UDP zero checksum is allowed.
        for(integer j=0;j<length;j++) f[42+j]=pattern(seed,j);
        last_body=42+((length<17)?17:length);
        f[last_body]=next_rx_sn[vl];
        c=32'hffffffff;
        for(integer j=0;j<=last_body;j++) c=tb_crc32_byte(c,f[j]);
        for(integer j=0;j<4;j++) f[last_body+1+j]=(~c)>>(j*8);
        expected_length[vl]=length; expected_seed[vl]=seed;
        for(integer j=0;j<8;j++) begin
            @(negedge rclk[0]); inject_valid=1; inject_data=j==7?8'hd5:8'h55;
        end
        @(posedge rclk[0]); injection_sfd=$time;
        for(integer j=0;j<last_body+5;j++) begin
            @(negedge rclk[0]); inject_valid=1; inject_data=f[j];
        end
        @(negedge rclk[0]); inject_valid=0; inject_data=0;
        repeat(12) @(negedge rclk[0]);
    endtask
    real throughput;
    time min_delay,max_delay;
    integer target, sizes[0:3]='{17,465,977,1471};
    initial begin
        for(integer k=0;k<2;k++) begin
            wp[k]=0; vf[k]=0; wire_active[k]=0; idle_slots[k]=100;
            for(integer v=0;v<128;v++) previous_sof[k][v]=0;
        end
        for(integer v=0;v<4096;v++) begin
            next_rx_sn[v]=1; expected_length[v]=0; expected_seed[v]=0; rx_covered[v]=0;
            if(v<128) begin next_tx_sn[v]=1; tx_covered[v]=0; end
        end
        repeat(40) @(negedge clk); reset=0;
        wait(!dut.u_system.u_core.rxq_init_busy);
        repeat(200) @(negedge clk);
        if(!$test$plusargs("LATENCY_ONLY")) begin
        phase=1; perf_start=$time;
        for(integer r=0;r<3;r++) for(integer v=1;v<=32;v++) send_message(v,1471,v);
        wait_messages(96);
        throughput=96.0*1471*8*1000.0/(perf_end-perf_start);
        $display("METRIC effective_payload_Mbps=%0.3f bytes=%0d interval_ns=%0d",throughput,96*1471,perf_end-perf_start);
        tb_check(perf_received==96 && throughput>=100.0,"effective payload >=100 Mbps, A/B counted once");
        repeat(5000) @(negedge clk);
        phase=2;
        for(integer v=0;v<128;v++) send_message(v,17,v);
        wait_messages(224);
        // 最后一条逻辑消息交付不代表另一网络副本已解析完；有界排空再核对精确计数。
        begin
            integer drain_cycles;
            drain_cycles=0;
            while(dut.u_system.cnt_rx_rm_drop<224 && drain_cycles<250000) begin
                @(negedge clk); drain_cycles++;
            end
        end
        for(integer v=0;v<128;v++) tb_check(tx_covered[v],"all 128 TX VLs reach both physical links");
        tb_check(vf[0]==224 && vf[1]==224,"both physical networks transmit every message");
        tb_check(dut.u_system.cnt_rx_rm_drop==224,"exact duplicate count");
        phase=3; loop_enable=0;
        for(integer v=0;v<4096;v++) begin
            inject(v,17,v); wait_messages(225+v);
            if(v%512==511) $display("RX coverage %0d/4096",v+1);
        end
        for(integer v=0;v<4096;v++) tb_check(rx_covered[v],"all 4096 RX subscriptions deliver");
        end
        phase=4; loop_enable=0; target=received;
        for(integer i=0;i<4;i++) begin
            echo_index=i; inject(1,sizes[i],90+i); target++;
            wait_messages(target);
            send_message(1,sizes[i],90+i,1);
            wait(echo_completed==i+1);
            tb_check(delay_ns[i]<2000000,"GMII SFD input to re-emitted frame SFD <2ms");
            $display("METRIC frame_body=%0d GMII_echo_delay_ns=%0d rx_first_ns=%0d rx_done_ns=%0d tx_first_ns=%0d tx_done_ns=%0d",
                43+sizes[i],delay_ns[i],app_rx_first-injection_sfd,app_rx_end-injection_sfd,
                app_tx_first-injection_sfd,app_tx_end-injection_sfd);
            // 保持 2.2ms 静默，而非沿用 100MHz 下的固定周期数。
            repeat(2200*125) @(negedge clk);
        end
        min_delay=delay_ns[0]; max_delay=delay_ns[0];
        for(integer i=1;i<4;i++) begin
            if(delay_ns[i]<min_delay) min_delay=delay_ns[i];
            if(delay_ns[i]>max_delay) max_delay=delay_ns[i];
        end
        $display("METRIC same_VL_variable_length_jitter_ns=%0d",max_delay-min_delay);
        // Engineering guard for 'tens of us'; not an official numeric limit.
        tb_check(max_delay-min_delay<=50000,"variable frame echo jitter <=50us engineering guard");
        tb_check(dut.u_system.cnt_rx_mac_error==0 && dut.u_system.cnt_rx_mac_len_err==0 &&
            dut.u_system.cnt_rx_mac_ovf==0 && dut.u_system.cnt_rx_hdr_err==0 &&
            dut.u_system.cnt_rx_csum_err==0 && dut.u_system.cnt_rx_full_drop==0 &&
            dut.u_system.cnt_rx_ic_drop==0,"no corruption, overflow, or integrity drops");
        if(!$test$plusargs("LATENCY_ONLY")) $display("METRIC TX_VL=128 RX_VL=4096 delivered=%0d",received);
        tb_finish("tb_afdx_acceptance"); $finish;
    end
    initial begin #100000000; $fatal(1,"acceptance watchdog phase=%0d received=%0d",phase,received); end
endmodule
`default_nettype wire
