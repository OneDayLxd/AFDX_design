`timescale 1ns/1ps
module tb_afdx_rx_parallel #(
    parameter NATIVE_MAC = 1
);
    `include "tb_common.svh"
    reg clk = 0, rst = 1;
    always #4 clk = ~clk;
    reg [15:0] drive_data = 0;
    reg [1:0] drive_valid = 0, drive_last = 0;
    reg [63:0] drive_ts = 0;
    wire [15:0] in_data;
    wire [1:0] in_valid, in_last, drive_ready;
    wire [63:0] in_ts, mac_errors;
    reg [31:0] now_us = 100;
    wire [1:0] in_ready;
    genvar link;
    generate for(link=0;link<2;link++) begin : g_link
        if(NATIVE_MAC != 0) begin : g_native
            afdx_gmii_mac u_mac (
                .clk               (clk),
                .reset             (rst),
                .ts_us             (drive_ts[32*link+:32]),
                .tx_data           (8'd0),
                .tx_valid          (1'b0),
                .tx_last           (1'b0),
                .tx_ready          (),
                .tx_sof            (),
                .tx_done           (),
                .rx_data           (in_data[8*link+:8]),
                .rx_valid          (in_valid[link]),
                .rx_last           (in_last[link]),
                .rx_ts             (in_ts[32*link+:32]),
                .rx_ready          (in_ready[link]),
                .rx_good_count     (),
                .rx_error_count    (mac_errors[32*link+:32]),
                .rx_length_count   (),
                .rx_overflow_count (),
                .eth_rx_er         (1'b0),
                .eth_rx_dv         (drive_valid[link]),
                .eth_rxd           (drive_data[8*link+:8]),
                .eth_tx_er         (),
                .eth_tx_en         (),
                .eth_txd           ()
            );
            assign drive_ready[link] = 1'b1; // 物理 GMII 不接受上游反压。
        end else begin : g_body
            assign in_data[8*link+:8] = drive_data[8*link+:8];
            assign in_valid[link] = drive_valid[link];
            assign in_last[link] = drive_last[link];
            assign in_ts[32*link+:32] = drive_ts[32*link+:32];
            assign drive_ready[link] = in_ready[link];
            assign mac_errors[32*link+:32] = 32'd0;
        end
    end endgenerate
    wire [2:0] wr_vl, rd_vl, msg_vl;
    wire [7:0] wr_data, rd_data, msg_data;
    wire [104:0] wr_meta, rd_meta, msg_meta;
    wire wr_valid, wr_ready, wr_commit, wr_abort, wr_ovf, wr_commit_ok;
    wire rd_start, rd_pop, rd_info, rd_pop_ready, rd_valid, rd_last, rd_ready;
    wire [10:0] rd_len, msg_len;
    wire [7:0] nonempty;
    wire [15:0] ne_bits;
    wire ne_region, init_busy;
    wire msg_valid, msg_last;
    reg msg_ready = 1;
    wire accepted, full_drop, ic_drop, rm_drop;
    wire [1:0] hdr_err, len_err, unknown_vl, csum_err;
    reg fail_once = 0;
    afdx_rx_parallel #(
        .N_VL        (8),
        .VLW         (3),
        .MW          (105),
        .SKEW_MAX_US (50)
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .now_us          (now_us),
        .in_data         (in_data),
        .in_valid        (in_valid),
        .in_last         (in_last),
        .in_ts           (in_ts),
        .in_ready        (in_ready),
        .q_wr_vl         (wr_vl),
        .q_wr_valid      (wr_valid),
        .q_wr_data       (wr_data),
        .q_wr_ready      (wr_ready),
        .q_wr_commit     (wr_commit),
        .q_wr_meta       (wr_meta),
        .q_wr_abort      (wr_abort),
        .q_wr_ovf        (wr_ovf),
        .q_wr_commit_ok  (wr_commit_ok),
        .stat_ok         (accepted),
        .stat_hdr_err    (hdr_err),
        .stat_len_err    (len_err),
        .stat_vl_unknown (unknown_vl),
        .stat_csum_err   (csum_err),
        .stat_full_drop  (full_drop),
        .stat_ic_drop    (ic_drop),
        .stat_rm_drop    (rm_drop),
        .pending_valid   (),
        .pending_ready   ()
    );
    // 小消息池使长帧在块边界真实反压；fail_once 仅用于模拟一次提交失败。
    afdx_msg_queue #(
        .N_VL                 (8),
        .VLW                  (3),
        .DEPTH                (4),
        .DW                   (2),
        .SLOT_AW              (11),
        .MW                   (105),
        .POOL_AW              (0),
        .BLOCK_AW             (8),
        .MAX_MSGS             (8),
        .PIPE_ADMISSION_MATCH (1)
    ) u_queue (
        .clk            (clk),
        .rst            (rst),
        .wr_vl          (wr_vl),
        .wr_vl_same     (1'b1),
        .wr_valid       (wr_valid),
        .wr_data        (wr_data),
        .wr_ready       (wr_ready),
        .wr_commit      (wr_commit && !fail_once),
        .wr_meta        (wr_meta),
        .wr_abort       (wr_abort || (wr_commit && fail_once)),
        .wr_ovf         (wr_ovf),
        .rd_vl          (rd_vl),
        .rd_start       (rd_start),
        .rd_pop         (rd_pop),
        .rd_info_valid  (rd_info),
        .reclaim_busy   (),
        .rd_pop_ready   (rd_pop_ready),
        .rd_len         (rd_len),
        .rd_meta        (rd_meta),
        .rd_data        (rd_data),
        .rd_valid       (rd_valid),
        .rd_last        (rd_last),
        .rd_ready       (rd_ready),
        .nonempty       (nonempty),
        .pool_used      (),
        .pool_peak      (),
        .pool_msgs      (),
        .pool_msgs_peak (),
        .ne_group       (1'b0),
        .ne_bits        (ne_bits),
        .init_busy      (init_busy),
        .ne_regions     (ne_region),
        .wr_commit_ok   (wr_commit_ok)
    );
    afdx_rx_deliver #(
        .N_VL    (8),
        .VLW     (3),
        .SLOT_AW (11),
        .MW      (105)
    ) u_deliver (
        .clk             (clk),
        .rst             (rst),
        .nonempty        (nonempty),
        .ne_group        (),
        .ne_bits         (ne_bits),
        .q_init_busy     (init_busy),
        .q_rd_vl         (rd_vl),
        .q_rd_start      (rd_start),
        .q_rd_pop        (rd_pop),
        .q_rd_info_valid (rd_info),
        .q_rd_pop_ready  (rd_pop_ready),
        .q_rd_len        (rd_len),
        .q_rd_meta       (rd_meta),
        .q_rd_data       (rd_data),
        .q_rd_valid      (rd_valid),
        .q_rd_last       (rd_last),
        .q_rd_ready      (rd_ready),
        .msg_vl          (msg_vl),
        .msg_valid       (msg_valid),
        .msg_data        (msg_data),
        .msg_last        (msg_last),
        .msg_ready       (msg_ready),
        .msg_len         (msg_len),
        .msg_meta        (msg_meta),
        .stat_deliver    (),
        .ne_regions      (ne_region)
    );
    integer expected_len[64], expected_vl[64], expected_sn[64], expected_net[64];
    integer seen[64], written[64];
    integer receive_pos = 0, receive_id = 0, write_pos = 0, write_id = 0;
    integer n_accepted = 0, n_full = 0, n_ic = 0, n_rm = 0, n_csum = 0;
    integer n_hdr = 0, n_len = 0, n_unknown = 0;
    integer input_stalls[2] = '{0,0};
    integer copy_stalls = 0;
    function automatic byte pattern(input integer id, pos);
        return byte'(id ^ (pos*7));
    endfunction
    function automatic [15:0] checksum(input reg [31:0] sum);
        sum = (sum & 65535) + (sum >> 16);
        sum = (sum & 65535) + (sum >> 16);
        return ~sum[15:0];
    endfunction
    task automatic expect_message(input integer id, vl, sn, length, net = -1);
        expected_len[id] = length;
        expected_vl[id] = vl;
        expected_sn[id] = sn;
        expected_net[id] = net;
    endtask
    task automatic inject(input integer net, id, vl, sn, length, fault = 0);
        byte unsigned f[1514];
        reg [15:0] v, ck;
        reg [31:0] sum;
        reg [31:0] crc, fcs;
        integer last_byte, iplen, udplen;
        foreach(f[i]) f[i] = 0;
        v = (fault==3 ? 16'h2000 : 16'h1000) + vl;
        iplen = 28+length;
        udplen = 8+length;
        last_byte = 42+length < 59 ? 59 : 42+length;
        f[0]=3; f[4]=v>>8; f[5]=v;
        f[6]=2; f[11]=8'h20+net; f[12]=8'h08;
        f[14]=8'h45; f[16]=iplen>>8; f[17]=iplen; f[22]=1; f[23]=17;
        f[26]=10; f[27]=1; f[28]=1; f[29]=1;
        f[30]=8'he0; f[31]=8'he0; f[32]=v>>8; f[33]=v;
        if(fault==4) f[17]++;
        sum=0;
        for(integer i=14;i<34;i+=2) sum += {f[i],f[i+1]};
        ck=checksum(sum); f[24]=ck>>8; f[25]=ck;
        f[34]=8'h04; f[35]=8'hd2; f[36]=8'h07; f[37]=8'hd0;
        f[38]=udplen>>8; f[39]=udplen;
        for(integer i=0;i<length;i++) f[42+i]=pattern(id,i);
        sum=17+udplen;
        for(integer i=26;i<34;i+=2) sum += {f[i],f[i+1]};
        for(integer i=34;i<34+udplen;i+=2)
            sum += {f[i], i+1 < 34+udplen ? f[i+1] : 8'd0};
        ck=checksum(sum);
        if(ck==0) ck=16'hffff;
        f[40]=ck>>8; f[41]=ck;
        f[last_byte]=sn;
        if(fault==1) f[42] ^= 1; // 保留原 UDP 校验和，但下方仍按改后的帧生成正确 FCS。
        if(fault==2) f[0] = 0;
        if(fault==5) f[24] ^= 1;
        @(negedge clk);
        drive_ts[32*net+:32]=1000+id*2+net;
        if(NATIVE_MAC != 0) begin
            for(integer i=0;i<8;i++) begin
                drive_data[8*net+:8]=i==7 ? 8'hd5 : 8'h55;
                drive_valid[net]=1;
                @(negedge clk);
            end
        end
        crc=32'hffffffff;
        for(integer i=0;i<=last_byte;i++) begin
            drive_data[8*net+:8]=f[i]; drive_valid[net]=1; drive_last[net]=i==last_byte;
            crc=tb_crc32_byte(crc,f[i]);
            @(posedge clk);
            while(!drive_ready[net]) @(posedge clk);
            @(negedge clk);
        end
        if(NATIVE_MAC != 0) begin
            fcs=~crc;
            if(fault==6) fcs ^= 1;
            for(integer i=0;i<4;i++) begin
                drive_data[8*net+:8]=fcs>>(8*i);
                @(negedge clk);
            end
        end
        drive_valid[net]=0; drive_last[net]=0;
        repeat(12) @(negedge clk);
    endtask
    task automatic pair(input integer id, vl, sn, length);
        expect_message(id,vl,sn,length);
        fork
            inject(0,id,vl,sn,length);
            inject(1,id,vl,sn,length);
        join
    endtask
    task automatic drain;
        integer timeout;
        timeout=0;
        while((|nonempty || dut.pending_valid || dut.state!=0 || in_valid!=0 || drive_valid!=0 ||
               dut.g_frontend[0].u_frontend.filling || dut.g_frontend[1].u_frontend.filling ||
               dut.g_frontend[0].u_frontend.wp!=dut.g_frontend[0].u_frontend.rp ||
               dut.g_frontend[1].u_frontend.wp!=dut.g_frontend[1].u_frontend.rp) && timeout<50000) begin
            @(negedge clk); timeout++;
        end
        repeat(100) @(negedge clk);
        tb_check(timeout<50000,"all descriptors and payloads drain without deadlock");
    endtask
    task automatic check_messages;
        for(integer id=0;id<64;id++) begin
            if(expected_len[id]>0) begin
                tb_check(seen[id]==1,$sformatf("ID %0d delivered exactly once",id));
                tb_check(written[id]==expected_len[id]*(id==11 ? 2 : 1),
                    $sformatf("ID %0d no redundant payload written to shared pool",id));
            end
        end
    endtask
    always @(posedge clk) begin
        if(!rst) begin
            if(wr_commit && fail_once) fail_once <= 0;
            n_accepted += accepted; n_full += full_drop;
            n_ic += ic_drop; n_rm += rm_drop;
            n_csum += int'(csum_err[0]) + int'(csum_err[1]);
            n_hdr += int'(hdr_err[0]) + int'(hdr_err[1]);
            n_len += int'(len_err[0]) + int'(len_err[1]);
            n_unknown += int'(unknown_vl[0]) + int'(unknown_vl[1]);
            for(integer net=0;net<2;net++)
                if(in_valid[net] && !in_ready[net]) input_stalls[net]++;
            if(wr_valid && !wr_ready) copy_stalls++;
            tb_check(!wr_ovf,"copy never exceeds message length limit");
            if(wr_valid && wr_ready) begin
                if(write_pos==0) write_id=wr_data;
                if(write_id>=64) $fatal(1,"unexpected write ID");
                written[write_id]++;
                tb_check(wr_data===pattern(write_id,write_pos),"shared queue payload byte");
                write_pos++;
            end
            if(wr_commit || wr_abort) write_pos=0;
            if(msg_valid && msg_ready) begin
                if(receive_pos==0) begin
                    receive_id=msg_data;
                    if(receive_id>=64) $fatal(1,"unexpected receive ID");
                    tb_check(expected_len[receive_id]>0,"no invalid, reset or unexpected message");
                    tb_check(msg_len==expected_len[receive_id],"message length");
                    tb_check(msg_vl==expected_vl[receive_id],"VL metadata");
                    tb_check(msg_meta[7:0]==expected_sn[receive_id],"sequence metadata");
                    tb_check(expected_net[receive_id]<0 || msg_meta[8]==expected_net[receive_id],"winning network");
                    tb_check(msg_meta[104:73]==1000+receive_id*2+int'(msg_meta[8]),"MAC timestamp retained");
                    tb_check(msg_meta[72:41]==32'h0a010101 && msg_meta[40:25]==1234 && msg_meta[24:9]==2000,
                        "IP/UDP session metadata retained");
                end
                tb_check(msg_data===pattern(receive_id,receive_pos),"delivered payload byte");
                tb_check(msg_last===(receive_pos==expected_len[receive_id]-1),"last-byte alignment");
                receive_pos++;
                if(msg_last) begin seen[receive_id]++; receive_pos=0; end
            end
        end else begin write_pos=0; receive_pos=0; end
    end
    reg copy_held = 0;
    reg [7:0] held_data;
    reg [2:0] held_vl;
    reg [104:0] held_meta;
    always @(posedge clk) begin
        if(rst) copy_held <= 0;
        else begin
            if(copy_held) tb_check(wr_valid && wr_data===held_data && wr_vl===held_vl && wr_meta===held_meta,
                "stalled copy retains data and metadata until accepted");
            copy_held <= wr_valid && !wr_ready;
            held_data <= wr_data; held_vl <= wr_vl; held_meta <= wr_meta;
        end
    end
    genvar net;
    generate for(net=0;net<2;net++) begin : g_ownership
        always @(posedge clk) if(!rst) begin
            tb_check(((dut.g_frontend[net].u_frontend.wp-dut.g_frontend[net].u_frontend.rp)&7)<=4,
                "local slot ownership never overflows");
            if(dut.g_frontend[net].u_frontend.full) tb_check(!in_ready[net],"full slots backpressure only MAC FIFO");
        end
    end endgenerate
    initial begin : main
        foreach(expected_len[i]) begin
            expected_len[i]=0; expected_vl[i]=0; expected_sn[i]=0; expected_net[i]=-1;
            seen[i]=0; written[i]=0;
        end
        repeat(8) @(negedge clk); rst=0;
        repeat(50) @(negedge clk);
        pair(0,1,1,17);
        drain();
        expect_message(1,1,2,465,1);
        fork inject(0,1,1,2,465,1); begin repeat(30) @(negedge clk); inject(1,1,1,2,465); end join
        drain();
        expect_message(2,1,3,17,1);
        fork inject(1,2,1,3,17); begin repeat(30) @(negedge clk); inject(0,2,1,3,17); end join
        drain();
        pair(3,2,254,17); pair(4,2,255,17); pair(5,2,1,17); pair(6,2,2,17);
        drain();
        fork inject(0,7,2,9,17); inject(1,7,2,9,17); join
        drain();
        tb_check(n_ic==2 && written[7]==0,"IC rejection updates each network without copying payload");
        pair(8,2,10,17); drain();
        fork inject(0,9,3,1,17,1); inject(1,9,3,1,17,1); join
        drain();
        tb_check(n_csum==3 && written[9]==0,"simultaneous UDP errors counted twice, no shared writes");
        expect_message(10,3,1,17,1); inject(1,10,3,1,17); drain();
        for(integer fault=2;fault<=5;fault++) begin
            fork inject(0,22+fault,3,2,17,fault); inject(1,22+fault,3,2,17,fault); join
            drain();
        end
        tb_check(n_hdr==4 && n_len==2 && n_unknown==2,"both frontends retain header/IP/length/VL checks and exact counters");
        tb_check(mac_errors==0,"UDP/IP/header error frames carried valid Ethernet FCS");
        if(NATIVE_MAC != 0) begin
            fork inject(0,28,3,2,17,6); inject(1,28,3,2,17,6); join
            drain();
            tb_check(mac_errors[31:0]==1 && mac_errors[63:32]==1 && written[28]==0,
                "bad FCS copies rejected before either frontend");
        end
        @(negedge clk); fail_once=1;
        pair(11,4,1,465); drain();
        tb_check(n_full==1 && seen[11]==1,"failed commit leaves valid other-network copy eligible");

        // 用 2KiB 池制造中途块不足，再用满本地槽反压上游，最终全部恢复。
        @(negedge clk); msg_ready=0;
        expect_message(12,5,1,1471,0); inject(0,12,5,1,1471);
        expect_message(13,6,1,1471);
        fork inject(0,13,6,1,1471); inject(1,13,6,1,1471); join
        fork
            begin
                for(integer id=14;id<22;id++) begin
                    pair(id,7,id-13,17);
                end
            end
            begin
                repeat(5000) @(negedge clk);
                tb_check(wr_valid && !wr_ready,"payload copy can wait on an exhausted shared pool");
                tb_check(dut.g_frontend[0].u_frontend.full || dut.g_frontend[1].u_frontend.full,
                    "local slots fill without overwrite");
                now_us=1000; // 等待跨越 SkewMax 后提交，另一网的在途副本仍只能交付一次。
                msg_ready=1;
            end
        join
        drain();
        tb_check(input_stalls[0]>0 && input_stalls[1]>0 && copy_stalls>0,"both input FIFOs and copy saw backpressure");
        check_messages();
        tb_check(n_full==1,"ordinary backpressure does not drop frames");

        // 在第二条待交付事务中复位，旧描述符、暂存字节和 RM 预约均须清除。
        @(negedge clk); msg_ready=0;
        inject(0,30,0,1,1471);
        inject(1,31,3,2,1471);
        begin
            integer wait_copy;
            wait_copy=0;
            while(!(wr_valid && !wr_ready) && wait_copy<10000) begin
                @(negedge clk); wait_copy++;
            end
        end
        tb_check(wr_valid && !wr_ready,"reset exercised during an uncommitted copy");
        rst=1;
        repeat(8) @(negedge clk);
        rst=0;
        repeat(50) @(negedge clk);
        tb_check(nonempty==0 && !dut.pending_valid && dut.state==0,"reset releases all ownership and pending transactions");
        tb_check(seen[30]==0 && seen[31]==0,"reset frames were not delivered");
        foreach(expected_len[i]) begin expected_len[i]=0; seen[i]=0; written[i]=0; end
        msg_ready=1;
        // 同时覆盖严格 > SkewMax、32 位单调时间回绕及首次 B 网 PSN。
        now_us=32'hffffffd0;
        expect_message(32,0,1,17,0); inject(0,32,0,1,17);
        expect_message(34,1,1,17,0); inject(0,34,1,1,17);
        drain();
        now_us=2; inject(1,33,0,1,17); drain(); // age==50，必须丢弃。
        tb_check(seen[33]==0 && written[33]==0,"same SN at exactly SkewMax not copied or delivered");
        now_us=3;
        expect_message(35,1,1,17,1); inject(1,35,1,1,17); drain(); // age==51。
        check_messages();
        tb_check(n_full==1,"reset and SkewMax recovery do not report pool overflow");
        tb_finish("tb_afdx_rx_parallel");
        $finish;
    end
    initial begin
        #4_000_000;
        $fatal(1,"parallel RX test timeout");
    end
endmodule
