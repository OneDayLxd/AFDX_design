/*
 * ================================================================
 * File Name    : tb_afdx_load_latency.sv
 * Module Name  : tb_afdx_load_latency
 *
 * Function:
 *   正式配置、连续 125MHz 的有界负载压力测试，不修改生产 RTL。
 *   分开计量 TX 入队、RX 应用交付和带 TB 应用回送的 SFD 路径。
 *   同 VL/同长度重复采样；统计完整性、丢失、背压和工程门限失败。
 *
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
`default_nettype none
module tb_afdx_load_latency;
    `include "tb_common.svh"
    reg clk=0, reset=1, started=0;
    bit rx_driver_busy=0, tx_driver_busy=0;
    always #4 clk=~clk;
    wire [1:0] tx_clk, tx_en, tx_er;
    wire [15:0] txd;
    reg [1:0] rx_dv=0;
    reg [15:0] rxd=0;
    reg [6:0] host_tx_vl=0;
    reg host_tx_valid=0, host_tx_last=0, host_rx_ready=1;
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
        .host_rx_ready               (host_rx_ready),
        .host_rx_len                 (host_rx_len),
        .host_rx_sn                  (host_rx_sn),
        .host_rx_net                 (host_rx_net),
        .host_rx_ts                  (host_rx_ts),
        .host_rx_src_ip              (),
        .host_rx_sport               (),
        .host_rx_dport               ()
    );

    localparam N_TARGET=16, MAX_BG_TX=512, N_ID=32;
    typedef struct {
        int vl;
        int length;
        int id;
        bit echo_job;
        time released;
    } job_t;
    mailbox #(job_t) tx_bg=new(), rx_bg=new(), requests=new(), replies=new();
    integer scenario=0, sample_fd, rx_fd, tx_fd, input_fd;
    string prefix;
    time epoch, last_activity=0;
    integer cycles=0, target_received=0, rx_received=0, bg_tx_total=0, bg_rx_total=0;
    integer tx_committed=0, tx_stall_cycles=0, rx_stall_cycles=0;
    integer wire_frames[0:1]='{0,0}, inject_sn[0:4095];
    integer wire_sn[0:1][0:127];
    integer target_length[0:N_TARGET-1];
    byte echo_buffer[0:N_TARGET-1][0:1470];
    time input_sfd[0:4095][0:N_ID-1], first_rx[0:4095][0:N_ID-1];
    bit rx_seen[0:4095][0:N_ID-1];
    time rx_min[0:4095], rx_max[0:4095];
    integer rx_samples[0:4095];
    time target_rx_done[0:N_TARGET-1], target_offer[0:N_TARGET-1];
    time target_accept[0:N_TARGET-1], target_commit[0:N_TARGET-1];
    time target_out[0:1][0:N_TARGET-1];
    bit target_seen[0:1][0:N_TARGET-1], target_logged[0:N_TARGET-1];
    time bg_offer[0:MAX_BG_TX-1], bg_accept[0:MAX_BG_TX-1], bg_commit[0:MAX_BG_TX-1];
    time bg_release[0:MAX_BG_TX-1];
    time bg_out[0:1][0:MAX_BG_TX-1];
    bit bg_seen[0:1][0:MAX_BG_TX-1], bg_logged[0:MAX_BG_TX-1];
    integer bg_vl[0:MAX_BG_TX-1], bg_length[0:MAX_BG_TX-1];
    time tx_min[0:127], tx_max[0:127];
    integer tx_samples[0:127];
    time echo_min=64'hffffffffffffffff, echo_max=0;
    time fixed_min[0:3], fixed_max[0:3];
    integer fixed_count[0:3]='{0,0,0,0};
    time tx_queue_max=0, rx_delivery_max=0, reply_wait_max=0;
    integer echo_samples=0, echo_over_2ms=0, tx_over_2ms=0, rx_over_2ms=0;

    // 资源水位只观察 DUT，不参与驱动或性能判定。
    integer rx_peak[0:1]='{0,0}, rx_full_cycles[0:1]='{0,0};
    integer parser_wait_cycles=0, tx_wire_wait_cycles=0;
    integer profile_fd;
    reg [7*128+1:0] profile_previous = 'x;
    wire [7*128+1:0] profile_mask = {
        dut.u_system.u_core.txq_nonempty,
        dut.u_system.u_core.u_sched.bag_open,
        dut.u_system.u_core.mac_a_bag_open,
        dut.u_system.u_core.mac_b_bag_open,
        dut.u_system.u_core.tx_egress_ready,
        dut.u_system.u_core.win_mask,
        dut.u_system.u_core.tx_wire_eligible,
        dut.u_system.u_core.fb_idle,
        dut.u_system.u_core.cfg_hold};
    // 只在状态改变时记录掩码，离线按每条消息的 VL/FIFO 次序积分，不驱动 DUT。
    always @(posedge clk) if(started) begin
        if(profile_mask !== profile_previous) begin
            $fdisplay(profile_fd,"M %0d %h",$time,profile_mask);
            profile_previous=profile_mask;
        end
        if(dut.u_system.u_core.u_txq.commit)
            $fdisplay(profile_fd,"Q %0d %0d %0d",$time,dut.u_system.u_core.u_txq.commit_vl,dut.u_system.u_core.u_txq.commit_len);
        if(dut.u_system.u_core.grant_ack)
            $fdisplay(profile_fd,"G %0d %0d",$time,dut.u_system.u_core.grant_vl);
        if(dut.u_system.u_core.u_txq.pop)
            $fdisplay(profile_fd,"P %0d %0d",$time,dut.u_system.u_core.txq_rd_vl);
        if(dut.u_system.u_core.fb_raw_valid && dut.u_system.u_core.fb_raw_ready) begin
            if(dut.u_system.u_core.u_fb.state==6 && dut.u_system.u_core.u_fb.hdr_idx==0)
                $fdisplay(profile_fd,"H %0d %0d",$time,dut.u_system.u_core.cfg_idx);
            if(dut.u_system.u_core.fb_raw_last)
                $fdisplay(profile_fd,"E %0d %0d",$time,dut.u_system.u_core.cfg_idx);
        end
        if(dut.u_system.u_core.u_mac_tx_a.start_frame)
            $fdisplay(profile_fd,"A %0d %0d",$time,dut.u_system.u_core.u_mac_tx_a.frame_vl);
        if(dut.u_system.u_core.u_mac_tx_b.start_frame)
            $fdisplay(profile_fd,"B %0d %0d",$time,dut.u_system.u_core.u_mac_tx_b.frame_vl);
        if(dut.u_system.u_core.mac_a_tx_valid && dut.u_system.u_core.mac_a_tx_slot && dut.u_system.u_core.mac_a_tx_last)
            $fdisplay(profile_fd,"a %0d %0d",$time,dut.u_system.u_core.u_mac_tx_a.frame_vl);
        if(dut.u_system.u_core.mac_b_tx_valid && dut.u_system.u_core.mac_b_tx_slot && dut.u_system.u_core.mac_b_tx_last)
            $fdisplay(profile_fd,"b %0d %0d",$time,dut.u_system.u_core.u_mac_tx_b.frame_vl);
    end
    for(genvar n=0;n<2;n++) begin : g_resource_monitor
        integer occupancy;
        always @(posedge clk) if(started) begin
            occupancy=(dut.g_net[n].u_link.u_rx_fifo.wp-
                       dut.g_net[n].u_link.u_rx_fifo.rp) &
                      ((2 << dut.g_net[n].u_link.u_rx_fifo.SLOT_LOG)-1);
            if(occupancy>rx_peak[n]) rx_peak[n]=occupancy;
            if(dut.g_net[n].u_link.u_rx_fifo.full) rx_full_cycles[n]++;
        end
    end
    always @(posedge clk) if(started) begin
        // 正式并行路径中 pf_* 观测描述符等待/复制，不再表示共享解析器字节反压。
        if(dut.u_system.u_core.pf_valid && !dut.u_system.u_core.pf_ready)
            parser_wait_cycles++;
        if(dut.u_system.u_core.u_mac_tx_a.state==9) tx_wire_wait_cycles++;
    end

    function automatic byte payload(input bit is_echo, input integer id, vl, length, pos);
        case(pos)
            0: return is_echo ? 8'he1 : 8'hb0;
            1: return byte'(id>>8);
            2: return byte'(id);
            3: return byte'(vl>>8);
            4: return byte'(vl);
            5: return byte'(length>>8);
            6: return byte'(length);
            default: return byte'(vl+id*17+pos*7);
        endcase
    endfunction
    function automatic [15:0] checksum(input [31:0] sum);
        reg [31:0] s;
        s=sum[15:0]+sum[31:16]; s=s[15:0]+s[31:16]; return ~s[15:0];
    endfunction
    task automatic until_time(input time due);
        if($time<due) #(due-$time);
    endtask
    // 输入源的 A/B 同一逻辑帧使用相同 SN/载荷，B 仅有确定性相位偏移。
    task automatic inject_frame(input integer net, input job_t j, input integer sn, skew);
        byte f[0:1517];
        reg [31:0] s,c;
        reg [15:0] v,iplen,uplen,ck;
        integer last_body;
        for(integer i=0;i<1518;i++) f[i]=0;
        v=16'h1000+j.vl; iplen=28+j.length; uplen=8+j.length;
        f[0]=3; f[4]=v>>8; f[5]=v;
        f[6]=2; f[9]=1; f[10]=1; f[11]=8'h21+net; f[12]=8'h08;
        f[14]=8'h45; f[16]=iplen>>8; f[17]=iplen; f[22]=1; f[23]=17;
        f[26]=10; f[27]=1; f[28]=1; f[29]=1;
        f[30]=8'he0; f[31]=8'he0; f[32]=v>>8; f[33]=v;
        s=0; for(integer i=14;i<34;i+=2) s+={f[i],f[i+1]};
        ck=checksum(s); f[24]=ck>>8; f[25]=ck;
        f[34]=16'(1000+j.vl)>>8; f[35]=1000+j.vl;
        f[36]=16'(2000+j.vl)>>8; f[37]=2000+j.vl;
        f[38]=uplen>>8; f[39]=uplen;
        for(integer i=0;i<j.length;i++) f[42+i]=payload(j.echo_job,j.id,j.vl,j.length,i);
        last_body=42+j.length; f[last_body]=sn;
        c=32'hffffffff; for(integer i=0;i<=last_body;i++) c=tb_crc32_byte(c,f[i]);
        for(integer i=0;i<4;i++) f[last_body+1+i]=(~c)>>(8*i);
        repeat(skew) @(negedge clk);
        for(integer i=0;i<8;i++) begin
            @(negedge clk); rx_dv[net]=1; rxd[8*net +: 8]=i==7 ? 8'hd5 : 8'h55;
        end
        @(posedge clk);
        if(net==0) begin
            input_sfd[j.vl][j.id]=$time;
            $fdisplay(input_fd,"%0d,%0d,%0d,%0d,%0d,%0d",scenario,j.vl,j.id,j.length,j.echo_job,$time);
        end
        for(integer i=0;i<last_body+5;i++) begin
            @(negedge clk); rx_dv[net]=1; rxd[8*net +: 8]=f[i];
        end
        @(negedge clk); rx_dv[net]=0; rxd[8*net +: 8]=0;
        repeat(12) @(negedge clk);
    endtask

    // 每一方向只有一个驱动者；回送优先，但不打断已开始的应用消息。
    initial begin : tx_driver
        job_t j;
        bit got_job;
        wait(started);
        forever begin
            @(negedge clk);
            got_job=replies.try_get(j);
            if(!got_job) got_job=tx_bg.try_get(j);
            if(got_job) begin
                tx_driver_busy=1;
                if(j.echo_job) target_offer[j.id]=$time;
                else bg_offer[j.id]=$time;
                for(integer p=0;p<j.length;p++) begin
                    host_tx_vl=j.vl; host_tx_valid=1; host_tx_last=p==j.length-1;
                    host_tx_data=j.echo_job ? echo_buffer[j.id][p] :
                        payload(0,j.id,j.vl,j.length,p);
                    @(posedge clk);
                    while(!host_tx_ready) @(posedge clk);
                    if(p==0) begin
                        if(j.echo_job) target_accept[j.id]=$time;
                        else bg_accept[j.id]=$time;
                    end
                    if(p==j.length-1) begin
                        if(j.echo_job) target_commit[j.id]=$time;
                        else bg_commit[j.id]=$time;
                        tx_committed++;
                    end
                    @(negedge clk);
                end
                host_tx_valid=0; host_tx_last=0;
                tx_driver_busy=0;
            end
        end
    end
    initial begin : rx_driver
        job_t j;
        integer sn;
        bit got_job;
        wait(started);
        forever begin
            @(negedge clk);
            got_job=requests.try_get(j);
            if(!got_job) got_job=rx_bg.try_get(j);
            if(got_job) begin
                rx_driver_busy=1;
                sn=inject_sn[j.vl]++;
                fork
                    inject_frame(0,j,sn,0);
                    inject_frame(1,j,sn,(j.id%3)*8);
                join
                rx_driver_busy=0;
            end
        end
    end
    always @(negedge clk) begin
        if(started) begin
            cycles++;
            // 仅应用接口有背压；网络 ready 从不参与握手。
            host_rx_ready=scenario!=3 || cycles%25000<18750;
        end
    end
    integer rx_pos=0;
    bit rx_held=0;
    reg [40:0] rx_held_fields;
    always @(posedge clk) if(started) begin : receive_check
        integer id,vl,length;
        time d;
        job_t j;
        if(|rx_dv) last_activity=$time;
        if(rx_held) begin
            tb_check(host_rx_valid && {host_rx_vl,host_rx_data,host_rx_last,host_rx_sn,host_rx_len,host_rx_net}===rx_held_fields,
                     "application RX stable under backpressure");
            if(tb_errors) $fatal(1,"RX changed before ready");
        end
        rx_held=host_rx_valid && !host_rx_ready;
        rx_held_fields={host_rx_vl,host_rx_data,host_rx_last,host_rx_sn,host_rx_len,host_rx_net};
        if(host_tx_valid && !host_tx_ready) tx_stall_cycles++;
        if(host_rx_valid && !host_rx_ready) rx_stall_cycles++;
        if(host_rx_valid && host_rx_ready) begin
            vl=host_rx_vl; id=int'(host_rx_sn)-1;
            if(id<0 || id>=N_ID) $fatal(1,"unexpected RX SN vl=%0d sn=%0d",vl,host_rx_sn);
            length=vl==1 ? target_length[id] : 17;
            tb_check(host_rx_len==length && host_rx_last==(rx_pos==length-1),"RX length/last");
            tb_check(host_rx_data===payload(vl==1,id,vl,length,rx_pos),"RX payload correlation");
            if(tb_errors) $fatal(1,"RX scoreboard vl=%0d id=%0d pos=%0d",vl,id,rx_pos);
            if(rx_pos==0) first_rx[vl][id]=$time;
            if(vl==1) echo_buffer[id][rx_pos]=host_rx_data;
            last_activity=$time;
            if(host_rx_last) begin
                tb_check(!rx_seen[vl][id] && input_sfd[vl][id]!=0,"unique RX delivery with input timestamp");
                rx_seen[vl][id]=1; rx_received++;
                if(rx_received%2048==0) $display("LOAD_RX_PROGRESS scenario=%0d received=%0d time_ns=%0d",scenario,rx_received,$time);
                d=$time-input_sfd[vl][id];
                if(d<rx_min[vl]) rx_min[vl]=d;
                if(d>rx_max[vl]) rx_max[vl]=d;
                rx_samples[vl]++;
                if(d>rx_delivery_max) rx_delivery_max=d;
                if(d>=2000000) rx_over_2ms++;
                $fdisplay(rx_fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                    scenario,vl,id,length,input_sfd[vl][id],first_rx[vl][id],$time,d);
                if(vl==1) begin
                    target_rx_done[id]=$time; target_received++;
                    j.vl=1; j.length=length; j.id=id; j.echo_job=1; j.released=$time;
                    replies.put(j);
                end
                rx_pos=0;
            end else rx_pos++;
        end
    end

    byte wire_bytes[0:1][0:1525];
    integer wire_pos[0:1]='{0,0}, idle[0:1]='{100,100};
    bit active[0:1]='{0,0};
    reg [31:0] wire_crc[0:1];
    time wire_sfd[0:1], previous_sfd[0:1][0:127];
    task automatic finish_wire(input integer net);
        integer vl,id,length,slot;
        bit is_echo;
        time first_out,d;
        vl=({wire_bytes[net][12],wire_bytes[net][13]})-16'h1000;
        id={wire_bytes[net][51],wire_bytes[net][52]};
        is_echo=wire_bytes[net][50]==8'he1;
        if(vl<0 || vl>127 || (is_echo && id>=N_TARGET) || (!is_echo && id>=bg_tx_total))
            $fatal(1,"bad TX correlation net=%0d vl=%0d id=%0d",net,vl,id);
        length=is_echo ? target_length[id] : bg_length[id];
        tb_check(wire_bytes[net][50]==(is_echo?8'he1:8'hb0),"TX class");
        tb_check(wire_pos[net]==length+55,"TX exact frame length");
        tb_check(wire_crc[net]===32'hdebb20e3,"TX independent CRC");
        tb_check(wire_bytes[net][wire_pos[net]-5]==wire_sn[net][vl],"TX per-VL SN");
        wire_sn[net][vl]=wire_sn[net][vl]==255 ? 1 : wire_sn[net][vl]+1;
        for(integer p=0;p<length;p++)
            tb_check(wire_bytes[net][50+p]===payload(is_echo,id,vl,length,p),"TX payload");
        if(previous_sfd[net][vl])
            tb_check(wire_sfd[net]-previous_sfd[net][vl]>=2000000,"wire BAG >=2ms");
        previous_sfd[net][vl]=wire_sfd[net];
        if(is_echo) begin
            tb_check(!target_seen[net][id],"no extra response per network");
            target_seen[net][id]=1; target_out[net][id]=wire_sfd[net];
            if(target_seen[1-net][id] && !target_logged[id]) begin
                target_logged[id]=1;
                first_out=target_out[0][id]<target_out[1][id] ? target_out[0][id] : target_out[1][id];
                d=first_out-input_sfd[1][id];
                echo_samples++; slot=id%4; fixed_count[slot]++;
                $display("LOAD_SAMPLE scenario=%0d id=%0d payload=%0d echo_ns=%0d",scenario,id,length,d);
                if(d<echo_min) echo_min=d;
                if(d>echo_max) echo_max=d;
                if(d<fixed_min[slot]) fixed_min[slot]=d;
                if(d>fixed_max[slot]) fixed_max[slot]=d;
                if(d>=2000000) echo_over_2ms++;
                if(target_offer[id]-target_rx_done[id]>reply_wait_max)
                    reply_wait_max=target_offer[id]-target_rx_done[id];
                if(first_out-target_accept[id]>tx_queue_max) tx_queue_max=first_out-target_accept[id];
                $fdisplay(sample_fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                    scenario,id,length,input_sfd[1][id],first_rx[1][id],target_rx_done[id],
                    target_offer[id],target_accept[id],target_commit[id],target_out[0][id],target_out[1][id],d);
                $fflush(sample_fd);
            end
        end else begin
            tb_check(bg_vl[id]==vl && !bg_seen[net][id],"background TX identity");
            bg_seen[net][id]=1; bg_out[net][id]=wire_sfd[net];
            if(bg_seen[1-net][id] && !bg_logged[id]) begin
                bg_logged[id]=1;
                first_out=bg_out[0][id]<bg_out[1][id] ? bg_out[0][id] : bg_out[1][id];
                d=first_out-bg_accept[id];
                if(d>tx_queue_max) tx_queue_max=d;
                if(d<tx_min[vl]) tx_min[vl]=d;
                if(d>tx_max[vl]) tx_max[vl]=d;
                tx_samples[vl]++;
                if(d>=2000000) tx_over_2ms++;
                $fdisplay(tx_fd,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",scenario,id,vl,length,
                    bg_release[id],bg_offer[id],bg_accept[id],bg_commit[id],bg_out[0][id],bg_out[1][id],d);
            end
        end
        if(tb_errors) $fatal(1,"TX checker net=%0d vl=%0d id=%0d",net,vl,id);
        wire_frames[net]++;
    endtask
    // 两个边界都按 clk 上升沿采样，避免混用半拍导致固定 4ns 偏移。
    always @(posedge clk) if(started) begin
        for(integer net=0;net<2;net++) begin
            if(tx_en[net]) begin
                if(!active[net]) begin
                    tb_check(idle[net]>=12,"GMII IFG");
                    active[net]=1; wire_pos[net]=0; wire_crc[net]=32'hffffffff;
                end
                wire_bytes[net][wire_pos[net]]=txd[net*8 +:8];
                if(wire_pos[net]<7) tb_check(txd[net*8 +:8]==8'h55,"preamble");
                else if(wire_pos[net]==7) begin
                    tb_check(txd[net*8 +:8]==8'hd5,"SFD"); wire_sfd[net]=$time;
                end else wire_crc[net]=tb_crc32_byte(wire_crc[net],txd[net*8 +:8]);
                tb_check(!tx_er[net],"no TX underflow");
                wire_pos[net]++; idle[net]=0; last_activity=$time;
                if(wire_pos[net]>1526) $fatal(1,"TX giant");
            end else begin
                if(active[net]) begin finish_wire(net); active[net]=0; end
                idle[net]++;
            end
        end
    end

    initial begin : main
        integer n_tx_rounds, n_rx_rounds;
        static integer sizes[0:3]='{17,465,977,1471};
        integer total_sent, missing_target, missing_tx, missing_rx, functional_errors;
        integer worst_rx_vl, worst_tx_vl;
        time worst_rx_jitter, worst_tx_jitter, fixed_jitter, drain_start;
        job_t j;
        if(!$value$plusargs("SCENARIO=%d",scenario)) scenario=0;
        if(scenario<0 || scenario>4) $fatal(1,"SCENARIO must be 0..4");
        prefix=$sformatf("load_%0d",scenario);
        sample_fd=$fopen({prefix,"_echo.csv"},"w"); rx_fd=$fopen({prefix,"_rx.csv"},"w");
        tx_fd=$fopen({prefix,"_tx.csv"},"w");
        input_fd=$fopen({prefix,"_input.csv"},"w");
        profile_fd=$fopen({prefix,"_tx_events.txt"},"w");
        if(!profile_fd) $fatal(1,"cannot open TX event profile");
        if(!sample_fd || !rx_fd || !tx_fd || !input_fd) $fatal(1,"cannot open sample CSV");
        $fdisplay(sample_fd,"scenario,id,payload_bytes,input_sfd_ns,rx_first_ns,rx_last_ns,tx_offer_ns,tx_accept_ns,tx_commit_ns,out_a_sfd_ns,out_b_sfd_ns,echo_ns");
        $fdisplay(rx_fd,"scenario,vl,id,payload_bytes,input_sfd_ns,app_first_ns,app_last_ns,rx_ns");
        $fdisplay(tx_fd,"scenario,id,vl,payload_bytes,release_ns,offer_ns,accept_ns,commit_ns,out_a_sfd_ns,out_b_sfd_ns,tx_ns");
        $fdisplay(input_fd,"scenario,vl,id,payload_bytes,is_echo,input_sfd_ns");
        for(integer v=0;v<4096;v++) begin
            inject_sn[v]=1; rx_min[v]=64'hffffffffffffffff; rx_max[v]=0; rx_samples[v]=0;
            for(integer i=0;i<N_ID;i++) begin input_sfd[v][i]=0; first_rx[v][i]=0; rx_seen[v][i]=0; end
        end
        for(integer v=0;v<128;v++) begin
            tx_min[v]=64'hffffffffffffffff; tx_max[v]=0; tx_samples[v]=0;
            for(integer n=0;n<2;n++) begin wire_sn[n][v]=1; previous_sfd[n][v]=0; end
        end
        for(integer i=0;i<MAX_BG_TX;i++) begin
            bg_seen[0][i]=0; bg_seen[1][i]=0; bg_logged[i]=0;
            bg_offer[i]=0; bg_accept[i]=0; bg_commit[i]=0;
        end
        for(integer i=0;i<N_TARGET;i++) begin
            target_length[i]=sizes[i%4]; target_seen[0][i]=0; target_seen[1][i]=0;
            target_logged[i]=0; target_offer[i]=0; target_accept[i]=0; target_commit[i]=0;
        end
        for(integer i=0;i<4;i++) begin fixed_min[i]=64'hffffffffffffffff; fixed_max[i]=0; end
        repeat(40) @(negedge clk); reset=0;
        wait(!dut.u_system.u_core.rxq_init_busy);
        repeat(200) @(negedge clk);
        epoch=$time; started=1;
        n_tx_rounds=scenario==0 ? 0 : ((scenario==1 || scenario==3) ? 3 : 4);
        n_rx_rounds=(scenario==1 || scenario==3) ? 4 : (scenario==2 ? 1 : 0);
        $display("LOAD_BEGIN scenario=%0d TX_VL=128 RX_VL=4096 target_samples=%0d epoch_ns=%0d",scenario,N_TARGET,epoch);
        fork
            begin : background_tx
                job_t b;
                if(scenario==4) begin
                    // 先把 VL1 的四条消息连续入队，明确构造同 VL 的 BAG 积压。
                    for(integer k=0;k<128;k++) for(integer r=0;r<4;r++) begin
                        b.vl=(k+1)%128; b.length=b.vl==0 ? 516 : 1471; b.id=bg_tx_total;
                        b.echo_job=0; b.released=$time;
                        bg_vl[b.id]=b.vl; bg_length[b.id]=b.length; bg_release[b.id]=b.released;
                        bg_tx_total++; tx_bg.put(b);
                    end
                end else for(integer r=0;r<n_tx_rounds;r++) begin
                    if(scenario==1 || scenario==3) until_time(epoch+time'(r)*12000000);
                    for(integer v=0;v<128;v++) begin
                        if(v==1 && scenario!=4) continue;
                        b.vl=v; b.length=v==0 ? 516 : 1471; b.id=bg_tx_total;
                        b.echo_job=0; b.released=$time;
                        bg_vl[b.id]=v; bg_length[b.id]=b.length; bg_release[b.id]=b.released;
                        bg_tx_total++; tx_bg.put(b);
                    end
                end
            end
            begin : background_rx
                job_t b;
                integer serial;
                serial=0;
                for(integer r=0;r<n_rx_rounds;r++) for(integer v=0;v<4096;v++) begin
                    if(v==1) continue;
                    if(scenario!=2) until_time(epoch+time'(serial)*2200);
                    b.vl=v; b.length=17; b.id=r; b.echo_job=0; b.released=$time;
                    bg_rx_total++; rx_bg.put(b); serial++;
                end
            end
            begin : target_source
                job_t t;
                time due;
                due=epoch+500000;
                for(integer i=0;i<N_TARGET;i++) begin
                    until_time(due);
                    t.vl=1; t.length=target_length[i]; t.id=i; t.echo_job=1; t.released=$time;
                    requests.put(t);
                    due+=time'(2200+(i%4)*37)*1000;
                end
            end
        join
        drain_start=$time;
        // 即使发生丢包，也必须有界结束并报告缺失，不能只统计幸存样本。
        while($time-drain_start<20000000) begin
            @(negedge clk);
            if(wire_frames[0]==bg_tx_total+target_received &&
               wire_frames[1]==bg_tx_total+target_received &&
               requests.num()==0 && rx_bg.num()==0 && replies.num()==0 && tx_bg.num()==0 &&
               !rx_driver_busy && !tx_driver_busy &&
               $time-last_activity>2100000) break;
        end
        missing_target=N_TARGET-target_received; missing_rx=bg_rx_total+N_TARGET-rx_received;
        missing_tx=0;
        for(integer i=0;i<bg_tx_total;i++) if(!bg_seen[0][i] || !bg_seen[1][i]) missing_tx++;
        worst_rx_jitter=0; worst_tx_jitter=0; worst_rx_vl=-1; worst_tx_vl=-1;
        for(integer v=0;v<4096;v++) if(rx_samples[v]>1 && rx_max[v]-rx_min[v]>worst_rx_jitter) begin
            worst_rx_jitter=rx_max[v]-rx_min[v]; worst_rx_vl=v;
        end
        for(integer v=0;v<128;v++) if(tx_samples[v]>1 && tx_max[v]-tx_min[v]>worst_tx_jitter) begin
            worst_tx_jitter=tx_max[v]-tx_min[v]; worst_tx_vl=v;
        end
        fixed_jitter=0;
        for(integer i=0;i<4;i++) begin
            $display("LOAD_FIXED payload=%0d n=%0d min_ns=%0d max_ns=%0d jitter_ns=%0d",
                sizes[i],fixed_count[i],fixed_min[i],fixed_max[i],fixed_max[i]-fixed_min[i]);
            if(fixed_count[i]>1 && fixed_max[i]-fixed_min[i]>fixed_jitter)
                fixed_jitter=fixed_max[i]-fixed_min[i];
        end
        functional_errors=tb_errors;
        $display("LOAD_RESULT scenario=%0d echo_n=%0d echo_min_ns=%0d echo_max_ns=%0d echo_jitter_ns=%0d fixed_jitter_ns=%0d rx_max_ns=%0d tx_accept_to_sfd_max_ns=%0d reply_app_wait_max_ns=%0d",
            scenario,echo_samples,echo_min,echo_max,echo_max-echo_min,fixed_jitter,rx_delivery_max,tx_queue_max,reply_wait_max);
        $display("LOAD_JITTER rx_same_vl_max_ns=%0d rx_vl=%0d tx_same_vl_max_ns=%0d tx_vl=%0d",
            worst_rx_jitter,worst_rx_vl,worst_tx_jitter,worst_tx_vl);
        $display("LOAD_COUNTS bg_tx=%0d bg_rx=%0d rx_delivered=%0d missing_target=%0d missing_rx=%0d missing_tx=%0d tx_stall_cycles=%0d rx_stall_cycles=%0d echo_over_2ms=%0d tx_over_2ms=%0d rx_over_2ms=%0d",
            bg_tx_total,bg_rx_total,rx_received,missing_target,missing_rx,missing_tx,tx_stall_cycles,rx_stall_cycles,
            echo_over_2ms,tx_over_2ms,rx_over_2ms);
        $display("LOAD_DROPS mac_ovf=%0d rx_full=%0d ic=%0d rm=%0d mac_err=%0d hdr=%0d csum=%0d",
            dut.u_system.cnt_rx_mac_ovf,dut.u_system.cnt_rx_full_drop,dut.u_system.cnt_rx_ic_drop,
            dut.u_system.cnt_rx_rm_drop,dut.u_system.cnt_rx_mac_error,dut.u_system.cnt_rx_hdr_err,dut.u_system.cnt_rx_csum_err);
        $display("LOAD_RESOURCE rx_peak_a=%0d rx_peak_b=%0d rx_full_cycles_a=%0d rx_full_cycles_b=%0d parser_wait_cycles=%0d tx_wire_wait_cycles=%0d",
            rx_peak[0],rx_peak[1],rx_full_cycles[0],rx_full_cycles[1],parser_wait_cycles,tx_wire_wait_cycles);
        $fclose(sample_fd); $fclose(rx_fd); $fclose(tx_fd); $fclose(input_fd);
        $fclose(profile_fd);
        tb_check(functional_errors==0 && missing_tx==0,"payload/ordering and all TX delivered on both networks");
        tb_check(missing_target==0 && echo_samples==N_TARGET,"all target samples, including adverse load");
        // 超载探测允许报告背景丢包，但不能把丢包样本当成时延 PASS。
        if(scenario!=2) begin
            tb_check(missing_rx==0,"no RX loss under specified non-overload workload");
            tb_check(dut.u_system.cnt_rx_mac_ovf==0 && dut.u_system.cnt_rx_full_drop==0 &&
                     dut.u_system.cnt_rx_ic_drop==0,"no overflow/integrity loss under non-overload workload");
        end
        tb_check(echo_over_2ms==0,"bounded echo path <2ms engineering guard");
        tb_check(echo_samples>1 && echo_max-echo_min<=50000,"same-VL variable-length jitter <=50us engineering guard");
        $display("LOAD_COMPLETE scenario=%0d functional_errors=%0d total_guard_errors=%0d",scenario,functional_errors,tb_errors);
        tb_finish("tb_afdx_load_latency"); $finish;
    end
    initial begin #100000000; $fatal(1,"load watchdog scenario=%0d tx=%0d rx=%0d",scenario,tx_committed,rx_received); end
endmodule
`default_nettype wire
