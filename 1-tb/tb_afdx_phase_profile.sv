/*
 * ================================================================
 * File Name    : tb_afdx_phase_profile.sv
 * Module Name  : tb_afdx_phase_profile
 *
 * Function:
 *   独立时钟计划驱动 128 TX/4096 RX VL 的有界相位业务。
 *   实测完整 A/B CRC、去重、载荷吞吐和每 VL 变长回送 SFD 时延极差。
 *   PHASE=0 提供相同业务的轮询对照，不替代原压力测试。
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module tb_afdx_phase_profile;
    `include "tb_common.svh"
    parameter PHASE = 1;
    localparam ROUNDS=8, TARGETS=128*ROUNDS, BACKGROUND=3968;
    localparam SLOT_NS=64000, PERIOD_NS=8192000;
    reg clk=0, reset=1, started=0;
    always #4 clk=~clk;
    wire [1:0] tx_en, tx_er;
    wire [15:0] txd;
    reg [1:0] rx_dv=0;
    reg [15:0] rxd=0;
    reg [6:0] host_tx_vl=0;
    reg host_tx_valid=0, host_tx_last=0, host_rx_ready=1;
    reg [7:0] host_tx_data=0;
    wire host_tx_ready, host_rx_valid, host_rx_last;
    wire [11:0] host_rx_vl;
    wire [7:0] host_rx_data, host_rx_sn;
    wire [10:0] host_rx_len;
    afdx_mac_system #(
        .APP_EXTERNAL    (1),
        .APP_GEN_EN      (0),
        .MGMT_EN         (0),
        .TX_PHASE_ENABLE (PHASE)
    ) dut (
        .clk                        (clk),
        .reset                      (reset),
        .tx_data                    (txd[7:0]),
        .rx_data                    (rxd[7:0]),
        .tx_valid                   (tx_en[0]),
        .rx_valid                   (rx_dv[0]),
        .tx_ready                   (),
        .rx_ready                   (1'b0),
        .rx_er                      (1'b0),
        .tx_clk                     (),
        .tx_er                      (tx_er[0]),
        .b_tx_data                  (txd[15:8]),
        .b_rx_data                  (rxd[15:8]),
        .b_tx_valid                 (tx_en[1]),
        .b_rx_valid                 (rx_dv[1]),
        .b_tx_ready                 (),
        .b_rx_ready                 (1'b0),
        .b_rx_er                    (1'b0),
        .b_tx_clk                   (),
        .b_tx_er                    (tx_er[1]),
        .host_tx_vl                 (host_tx_vl),
        .host_tx_valid              (host_tx_valid),
        .host_tx_data               (host_tx_data),
        .host_tx_last               (host_tx_last),
        .host_tx_ready              (host_tx_ready),
        .host_rx_vl                 (host_rx_vl),
        .host_rx_valid              (host_rx_valid),
        .host_rx_data               (host_rx_data),
        .host_rx_last               (host_rx_last),
        .host_rx_ready              (host_rx_ready),
        .host_rx_len                (host_rx_len),
        .host_rx_sn                 (host_rx_sn),
        .host_rx_net                (),
        .host_rx_ts                 (),
        .host_rx_src_ip             (),
        .host_rx_sport              (),
        .host_rx_dport              ()
    );
    mailbox #(integer) replies=new();
    time epoch, base, source_sfd[0:TARGETS-1], app_last[0:TARGETS-1];
    time commit[0:TARGETS-1], output_sfd[0:1][0:TARGETS-1];
    time dmin[0:127], dmax[0:127], prior_sfd[0:1][0:127];
    bit seen_rx[0:4095][0:ROUNDS-1], seen_tx[0:1][0:TARGETS-1];
    integer source_sn[0:4095], tx_sn[0:1][0:127];
    integer rx_count=0, out_count[0:1]='{0,0}, queued=0, committed=0;
    integer payload_total=0, fd, rx_peak[0:1]='{0,0};
    time rx_max=0, commit_offset_max=0;
    function automatic integer length_for(input integer vl, round);
        integer l;
        case(round)
            0: l=17;
            1: l=465;
            2: l=977;
            default: l=1471;
        endcase
        return vl==0 && l>516 ? 516 : l;
    endfunction
    function automatic byte payload(input integer vl, round, p);
        return byte'(vl*13+round*7+p*3);
    endfunction
    function automatic [15:0] checksum(input [31:0] sum);
        reg [31:0] s;
        s=sum[15:0]+sum[31:16]; s=s[15:0]+s[31:16]; return ~s[15:0];
    endfunction
    task automatic until_time(input time due);
        if($time<due) #(due-$time);
    endtask
    task automatic inject(input integer net, vl, round, length, sn, skew, id);
        byte f[0:1517];
        reg [31:0] s,c;
        reg [15:0] v,iplen,uplen,ck;
        integer last_body;
        for(integer i=0;i<1518;i++) f[i]=0;
        v=16'h1000+vl; iplen=28+length; uplen=8+length;
        f[0]=3; f[4]=v>>8; f[5]=v;
        f[6]=2; f[9]=1; f[10]=1; f[11]=8'h21+net; f[12]=8'h08;
        f[14]=8'h45; f[16]=iplen>>8; f[17]=iplen; f[22]=1; f[23]=17;
        f[26]=10; f[27]=1; f[28]=1; f[29]=1;
        f[30]=8'he0; f[31]=8'he0; f[32]=v>>8; f[33]=v;
        s=0; for(integer i=14;i<34;i+=2) s+={f[i],f[i+1]};
        ck=checksum(s); f[24]=ck>>8; f[25]=ck;
        f[34]=(1000+vl)>>8; f[35]=1000+vl;
        f[36]=(2000+vl)>>8; f[37]=2000+vl;
        f[38]=uplen>>8; f[39]=uplen;
        for(integer i=0;i<length;i++) f[42+i]=payload(vl,round,i);
        last_body=42+length; f[last_body]=sn;
        c=32'hffffffff; for(integer i=0;i<=last_body;i++) c=tb_crc32_byte(c,f[i]);
        for(integer i=0;i<4;i++) f[last_body+1+i]=(~c)>>(8*i);
        repeat(skew) @(negedge clk);
        for(integer i=0;i<8;i++) begin
            @(negedge clk); rx_dv[net]=1; rxd[8*net +: 8]=i==7 ? 8'hd5 : 8'h55;
        end
        @(posedge clk);
        if(net==0 && id>=0) source_sfd[id]=$time;
        for(integer i=0;i<last_body+5;i++) begin
            @(negedge clk); rxd[8*net +: 8]=f[i];
        end
        @(negedge clk); rx_dv[net]=0; rxd[8*net +: 8]=0;
        repeat(12) @(negedge clk);
    endtask
    // 唯一 host TX 驱动；逐帧回送，测量包括这段应用复制耗时。
    initial begin : tx_driver
        integer id,v,r,l;
        wait(started);
        forever begin
            replies.get(id); v=id%128; r=id/128; l=length_for(v,r);
            for(integer p=0;p<l;p++) begin
                @(negedge clk);
                host_tx_vl=v; host_tx_valid=1; host_tx_last=p==l-1;
                host_tx_data=payload(v,r,p);
                @(posedge clk); while(!host_tx_ready) @(posedge clk);
                if(p==l-1) begin commit[id]=$time; committed++; end
            end
            @(negedge clk); host_tx_valid=0; host_tx_last=0;
        end
    end
    integer rx_pos=0;
    always @(negedge clk) if(started) begin
        // 每 64us 最多 2us 的应用停顿，刻意覆盖长帧交付附近。
        host_rx_ready=(($time-epoch)%SLOT_NS<32000 || ($time-epoch)%SLOT_NS>=34000);
    end
    always @(posedge clk) if(started && host_rx_valid && host_rx_ready) begin : rx_check
        integer v,r,l,id;
        v=host_rx_vl; r=host_rx_sn-1; l=v<128 ? length_for(v,r) : 17;
        if(v>4095 || r<0 || r>=ROUNDS) $fatal(1,"unexpected RX identity");
        tb_check(host_rx_len==l && host_rx_last==(rx_pos==l-1), "RX length/last");
        tb_check(host_rx_data===payload(v,r,rx_pos), "RX byte scoreboard");
        if(tb_errors) $fatal(1,"profile RX data failed vl=%0d",v);
        if(host_rx_last) begin
            tb_check(!seen_rx[v][r], "A/B delivered only once"); seen_rx[v][r]=1;
            rx_count++; rx_pos=0;
            if(v<128) begin
                id=r*128+v; app_last[id]=$time;
                if($time-source_sfd[id]>rx_max) rx_max=$time-source_sfd[id];
                replies.put(id); queued++;
            end
        end else rx_pos++;
    end
    byte bytes[0:1][0:1525];
    integer pos[0:1]='{0,0}, idle[0:1]='{100,100};
    bit active[0:1]='{0,0};
    reg [31:0] crc[0:1];
    time sfd[0:1];
    task automatic finish_frame(input integer n);
        integer v,r,l,id;
        time d,offset;
        v={bytes[n][12],bytes[n][13]}-16'h1000;
        if(v<0 || v>=128) $fatal(1,"unexpected TX VL");
        r=tx_sn[n][v]-1; l=length_for(v,r); id=r*128+v;
        if(r>=ROUNDS) $fatal(1,"extra TX frame");
        tb_check(pos[n]==l+55 && crc[n]===32'hdebb20e3, "TX length and independent CRC");
        tb_check(bytes[n][pos[n]-5]==tx_sn[n][v], "TX SN");
        for(integer p=0;p<l;p++) tb_check(bytes[n][50+p]===payload(v,r,p), "TX payload scoreboard");
        tb_check(!seen_tx[n][id], "one copy per network");
        if(prior_sfd[n][v]) tb_check(sfd[n]-prior_sfd[n][v]>=2000000, "actual wire BAG >=2ms");
        prior_sfd[n][v]=sfd[n]; tx_sn[n][v]++;
        seen_tx[n][id]=1; output_sfd[n][id]=sfd[n]; out_count[n]++;
        if(n==0) begin
            d=sfd[n]-source_sfd[id];
            if(d<dmin[v]) dmin[v]=d;
            if(d>dmax[v]) dmax[v]=d;
            offset=commit[id]-(base+time'(id)*SLOT_NS);
            if(offset>commit_offset_max) commit_offset_max=offset;
            tb_check(offset<59000, "profile: response committed before slot +59us");
            if(PHASE) tb_check(d>=50000 && d<=80000, "bounded profile echo in [50,80]us");
            payload_total+=l;
        end
        if(seen_tx[0][id] && seen_tx[1][id])
            $fdisplay(fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",v,r,l,source_sfd[id],app_last[id],commit[id],output_sfd[0][id],output_sfd[0][id]-source_sfd[id],base+time'(id)*SLOT_NS,output_sfd[1][id]);
        if(tb_errors) $fatal(1,"profile TX failed vl=%0d round=%0d",v,r);
    endtask
    always @(posedge clk) if(started) for(integer n=0;n<2;n++) begin
        if(tx_en[n]) begin
            if(!active[n]) begin
                tb_check(idle[n]>=12, "IFG >=12 cycles"); active[n]=1; pos[n]=0; crc[n]='1;
            end
            bytes[n][pos[n]]=txd[n*8 +: 8];
            if(pos[n]<7) tb_check(txd[n*8 +: 8]==8'h55, "preamble");
            else if(pos[n]==7) begin tb_check(txd[n*8 +: 8]==8'hd5, "SFD"); sfd[n]=$time; end
            else crc[n]=tb_crc32_byte(crc[n],txd[n*8 +: 8]);
            tb_check(!tx_er[n], "no TX error"); pos[n]++; idle[n]=0;
            if(pos[n]>1526) $fatal(1,"giant TX");
        end else begin
            if(active[n]) begin finish_frame(n); active[n]=0; end
            idle[n]++;
        end
    end
    for(genvar n=0;n<2;n++) begin : g_peak
        integer occupancy;
        always @(posedge clk) if(started) begin
            occupancy=(dut.g_net[n].u_link.u_rx_fifo.wp-dut.g_net[n].u_link.u_rx_fifo.rp)&4095;
            if(occupancy>rx_peak[n]) rx_peak[n]=occupancy;
        end
    end
    initial begin : main
        integer bg, v, sn, l;
        time due, jitter, max_jitter, max_delay;
        real mbps;
        max_jitter=0; max_delay=0;
        fd=$fopen(PHASE ? "phase_profile.csv" : "phase_control.csv","w");
        if(!fd) $fatal(1,"CSV open failed");
        $fdisplay(fd,"vl,round,payload_bytes,input_sfd_ns,app_last_ns,commit_ns,out_a_sfd_ns,echo_ns,slot_ns,out_b_sfd_ns");
        for(integer i=0;i<4096;i++) begin
            source_sn[i]=1;
            for(integer r=0;r<ROUNDS;r++) seen_rx[i][r]=0;
        end
        for(integer i=0;i<128;i++) begin
            dmin[i]='1; dmax[i]=0;
            for(integer n=0;n<2;n++) begin tx_sn[n][i]=1; prior_sfd[n][i]=0; end
        end
        repeat(40) @(negedge clk); reset=0; epoch=$time;
        base=epoch+PERIOD_NS; started=1; bg=128;
        // 计划完全由外部时间产生，不读取 DUT 的窗口/授权来推迟源数据。
        for(integer r=0;r<ROUNDS;r++) for(integer vl=0;vl<128;vl++) begin
            due=base+time'(r*128+vl)*SLOT_NS;
            until_time(due+time'(((r+vl)%5)*2000)); l=length_for(vl,r); sn=source_sn[vl]++;
            fork
                inject(0,vl,r,l,sn,0,r*128+vl);
                inject(1,vl,r,l,sn,(r+vl)%17,r*128+vl);
            join
            for(integer k=0;k<4 && bg<4096;k++) begin
                until_time(due+28000+time'(k)*4000); v=bg++; sn=source_sn[v]++;
                fork
                    inject(0,v,0,17,sn,0,-1);
                    inject(1,v,0,17,sn,16,-1);
                join
            end
        end
        until_time(base+time'(ROUNDS)*PERIOD_NS+200000);
        tb_check(rx_count==TARGETS+BACKGROUND && queued==TARGETS && committed==TARGETS, "complete logical RX and echo admission");
        tb_check(out_count[0]==TARGETS && out_count[1]==TARGETS, "complete A/B responses");
        for(integer v=0;v<4096;v++) tb_check(seen_rx[v][0], "all 4096 RX VL subscribed");
        for(integer v=0;v<128;v++) begin
            jitter=dmax[v]-dmin[v];
            if(jitter>max_jitter) max_jitter=jitter;
            if(dmax[v]>max_delay) max_delay=dmax[v];
            for(integer r=0;r<ROUNDS;r++) for(integer n=0;n<2;n++)
                tb_check(seen_tx[n][r*128+v], "no missing response sample");
            if(PHASE) tb_check(jitter<=50000 && dmax[v]<2000000, "same-VL jitter <=50us, latency <2ms");
        end
        tb_check(dut.u_system.u_core.cnt_rx_mac_ovf==0 &&
                 dut.u_system.u_core.cnt_rx_full_drop==0 &&
                 dut.u_system.u_core.cnt_tx_ovf_drop==0 &&
                 dut.u_system.u_core.cnt_rx_ic_drop==0, "no overflow or integrity drop");
        tb_check(dut.u_system.u_core.cnt_rx_rm_drop==TARGETS+BACKGROUND, "one redundant copy discarded per logical frame");
        mbps=real'(payload_total)*8000.0/real'(ROUNDS*PERIOD_NS);
        tb_check(mbps>=100.0, "unique application payload throughput >=100Mbps");
        $display("PHASE_RESULT enable=%0d TX_VL=128 RX_VL=4096 samples=%0d rx_unique=%0d payload_bytes=%0d interval_ns=%0d throughput_mbps=%0.6f max_echo_ns=%0d max_jitter_ns=%0d max_rx_ns=%0d max_commit_offset_ns=%0d rx_peak_a=%0d rx_peak_b=%0d",PHASE,TARGETS,rx_count,payload_total,ROUNDS*PERIOD_NS,mbps,max_delay,max_jitter,rx_max,commit_offset_max,rx_peak[0],rx_peak[1]);
        $fclose(fd); tb_finish("tb_afdx_phase_profile"); $finish;
    end
    initial begin #76000000; $fatal(1,"phase profile timeout"); end
endmodule
