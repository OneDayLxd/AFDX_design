/*
 * ================================================================
 * File Name    : tb_afdx_es_post_synth.sv
 * Module Name  : tb_afdx_es_post_synth
 *
 * Function:
 *   使用相同外部激励对照完整生产 RTL 与综合网表，无层次引用或参数覆盖。
 *   检查应用收发、A/B 去重、单网故障、CRC/ER 隔离及 IPv4 重组。
 *   不以短功能用例代替容量、吞吐或时序验收。
 *
 * Sub Module:
 *   AFDX_es
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_es_post_synth;
    `include "tb_common.svh"
    reg         clk = 0;
    reg         rst = 1;
    reg [7:0]   tx_data = 0;
    reg         tx_valid = 0;
    reg         tx_last = 0;
    wire [7:0]  rx_data;
    wire        rx_valid, rx_last, tx_ready, app_drop;
    wire [2:0]  app_drop_reason;
    wire [31:0] app_drop_count;
    wire [15:0] wire_tx_data;
    wire [1:0]  wire_tx_valid, wire_tx_er;
    wire [15:0] wire_rx_data;
    wire [1:0]  wire_rx_valid, wire_rx_er;
    reg [15:0]  manual_data = 0;
    reg [1:0]   manual_valid = 0;
    reg         manual_rx = 0;
    // 0 双网；1 仅 A；2 仅 B；3 A 坏 CRC；4 双网坏 CRC；5 A RX_ER。
    integer     loop_mode = 0;
    integer     wire_pos[2] = '{0,0};
    integer     frame_count[2] = '{0,0};
    reg [31:0]  frame_crc[2];
    bit         frame_active[2] = '{0,0};
    integer     expected_len[128] = '{default:0};
    integer     expected_tag[128] = '{default:0};
    integer     expected_seen[128] = '{default:0};
    integer     deliveries = 0, app_pos = 0, app_vlid = 0;
    integer     fd, cycles = 0;
    string      capture;
    byte        udp[48];

    always #4 clk = ~clk;
    assign wire_rx_valid = manual_rx ? manual_valid :
        {wire_tx_valid[1] && loop_mode != 1, wire_tx_valid[0] && loop_mode != 2};
    assign wire_rx_data[7:0] = manual_rx ? manual_data[7:0] : wire_tx_data[7:0] ^
        ((wire_pos[0] == 30 && (loop_mode == 3 || loop_mode == 4)) ? 8'h01 : 8'h00);
    assign wire_rx_data[15:8] = manual_rx ? manual_data[15:8] : wire_tx_data[15:8] ^
        ((wire_pos[1] == 30 && loop_mode == 4) ? 8'h01 : 8'h00);
    assign wire_rx_er = manual_rx ? 2'b00 : {1'b0, wire_tx_valid[0] && loop_mode == 5};

    AFDX_es dut (
        .clk               (clk),
        .rst               (rst),
        .tx_data           (tx_data),
        .rx_data           (rx_data),
        .tx_valid          (tx_valid),
        .rx_valid          (rx_valid),
        .tx_ready          (tx_ready),
        .rx_ready          (1'b0),
        .tx_last           (tx_last),
        .rx_last           (rx_last),
        .app_drop          (app_drop),
        .app_drop_reason   (app_drop_reason),
        .app_drop_count    (app_drop_count),
        .a_gmii_tx_data    (wire_tx_data[7:0]),
        .a_gmii_tx_valid   (wire_tx_valid[0]),
        .a_gmii_tx_er      (wire_tx_er[0]),
        .a_gmii_rx_data    (wire_rx_data[7:0]),
        .a_gmii_rx_valid   (wire_rx_valid[0]),
        .a_gmii_rx_er      (wire_rx_er[0]),
        .b_gmii_tx_data    (wire_tx_data[15:8]),
        .b_gmii_tx_valid   (wire_tx_valid[1]),
        .b_gmii_tx_er      (wire_tx_er[1]),
        .b_gmii_rx_data    (wire_rx_data[15:8]),
        .b_gmii_rx_valid   (wire_rx_valid[1]),
        .b_gmii_rx_er      (wire_rx_er[1])
    );

    function automatic byte pattern(input integer tag, p);
        return byte'(tag*17 + p*7 + (p>>8));
    endfunction

    always @(posedge clk) begin
        cycles = cycles + 1;
        if (!rst) begin
            if (cycles % 2000 == 0) $display("POST progress cycle=%0d delivered=%0d", cycles, deliveries);
            if (tx_valid) $fdisplay(fd,"S %0d %02x %0d",cycles,tx_data,tx_last);
            if (app_drop) $fdisplay(fd,"E %0d %0d %0d",cycles,app_drop_reason,app_drop_count);
            for (int n=0; n<2; n++) begin
                if (wire_rx_valid[n]) $fdisplay(fd,"I %0d %0d %02x %0d",cycles,n,wire_rx_data[n*8+:8],wire_rx_er[n]);
                if (wire_tx_valid[n]) begin
                    tb_check(!$isunknown({wire_tx_data[n*8+:8],wire_tx_er[n]}),"known GMII output");
                    tb_check(!wire_tx_er[n],"no unexpected TX error");
                    if (!frame_active[n]) begin frame_active[n]=1; frame_crc[n]='1; end
                    if (wire_pos[n]<7) tb_check(wire_tx_data[n*8+:8]==8'h55,"preamble");
                    else if (wire_pos[n]==7) tb_check(wire_tx_data[n*8+:8]==8'hd5,"SFD");
                    else frame_crc[n]=tb_crc32_byte(frame_crc[n],wire_tx_data[n*8+:8]);
                    $fdisplay(fd,"T %0d %0d %02x",cycles,n,wire_tx_data[n*8+:8]);
                    wire_pos[n] <= wire_pos[n]+1;
                end else begin
                    if (frame_active[n]) begin
                        tb_check(wire_pos[n]>=72 && wire_pos[n]<=1526,"MAC frame length");
                        tb_check(frame_crc[n]==32'hdebb20e3,"Ethernet FCS");
                        frame_active[n]=0; frame_count[n]++;
                    end
                    wire_pos[n] <= 0;
                end
            end
            if (rx_valid) begin
                tb_check(!$isunknown({rx_data,rx_last}),"known application output");
                $fdisplay(fd,"R %0d %02x %0d",cycles,rx_data,rx_last);
                if (app_pos==0) app_vlid=int'(unsigned'(rx_data))<<8;
                else if (app_pos==1) app_vlid=app_vlid|int'(unsigned'(rx_data));
                else if (app_vlid!=16'h1000) begin
                    tb_check(app_vlid>=16'h1001 && app_vlid<16'h1080,"subscribed application VLID");
                    if (app_vlid>=16'h1001 && app_vlid<16'h1080) begin
                        tb_check(expected_len[app_vlid-16'h1000]>0,"expected unique message");
                        tb_check(rx_data===pattern(expected_tag[app_vlid-16'h1000],app_pos-2),"application payload");
                        tb_check(rx_last==(app_pos==expected_len[app_vlid-16'h1000]+1),"complete message last");
                    end
                end
                app_pos++;
                if (rx_last) begin
                    if (app_vlid!=16'h1000 && app_vlid>=16'h1001 && app_vlid<16'h1080) begin
                        tb_check(app_pos==expected_len[app_vlid-16'h1000]+2,"whole message size");
                        expected_seen[app_vlid-16'h1000]++;
                        tb_check(expected_seen[app_vlid-16'h1000]==1,"A/B delivered once");
                        deliveries++;
                    end
                    app_pos=0;
                end
            end
        end
    end

    task automatic source(input integer vlid,n,tag);
        $fdisplay(fd,"C %0d %04x %0d %0d %0d",cycles,vlid,n,tag,loop_mode);
        @(negedge clk); tx_valid=1; tx_data=byte'(vlid>>8); tx_last=0;
        @(negedge clk); tx_data=byte'(vlid);
        for(int p=0;p<n;p++) begin
            @(negedge clk); tx_valid=1; tx_data=pattern(tag,p); tx_last=p==n-1;
            if(p%97==11 && p!=n-1) begin
                @(negedge clk); tx_valid=0; tx_last=1;
                repeat(2) @(negedge clk);
            end
        end
        @(negedge clk); tx_valid=0; tx_last=0;
    endtask

    task automatic wait_delivery(input integer target);
        integer elapsed=0;
        while(deliveries<target && elapsed<12000) begin @(negedge clk); elapsed++; end
        tb_check(deliveries==target,"expected application delivery before timeout");
        if(deliveries!=target) $fatal(1,"application delivery timeout");
        repeat(80) @(negedge clk);
    endtask

    task automatic application_case(input integer vl,n,tag,mode);
        integer target=deliveries+1;
        loop_mode=mode; expected_len[vl]=n; expected_tag[vl]=tag;
        source(16'h1000+vl,n,tag); wait_delivery(target);
        $display("POST case VL=%0d length=%0d mode=%0d PASS",vl,n,mode);
    endtask

    task automatic fragment(input integer vl,off,n,mf,sn,tag,bad_udp);
        byte raw[];
        integer total,flags;
        int unsigned sum,crc;
        for(int p=0;p<48;p++) udp[p]=0;
        udp[0]=byte'((1000+vl)>>8); udp[1]=byte'(1000+vl);
        udp[2]=byte'((2000+vl)>>8); udp[3]=byte'(2000+vl); udp[5]=48;
        for(int p=0;p<40;p++) udp[p+8]=pattern(tag,p);
        sum='h0a01+'h0101+'he0e0+('h1000+vl)+17+48;
        for(int p=0;p<48;p++) sum+=p%2 ? int'(unsigned'(udp[p])) : int'(unsigned'(udp[p]))<<8;
        while(sum>>16) sum=(sum&65535)+(sum>>16);
        sum=(~sum)&65535; udp[6]=byte'(sum>>8); udp[7]=byte'(sum);
        if(bad_udp) udp[7]^=1;
        total=8+((34+n<59)?60:35+n)+4;
        raw=new[total];
        for(int p=0;p<7;p++) raw[p]=8'h55; raw[7]=8'hd5;
        raw[8]=3; raw[12]=8'h10; raw[13]=byte'(vl);
        raw[14]=2; raw[17]=1; raw[18]=1; raw[19]=8'h20;
        raw[20]=8'h08; raw[22]=8'h45; raw[25]=byte'(20+n); raw[27]=byte'(vl);
        flags=(mf?'h2000:0)|(off/8); raw[28]=byte'(flags>>8); raw[29]=byte'(flags);
        raw[30]=1; raw[31]=17; raw[34]=10; raw[35]=1; raw[36]=1; raw[37]=1;
        raw[38]=8'he0; raw[39]=8'he0; raw[40]=8'h10; raw[41]=byte'(vl);
        sum=0;
        for(int p=22;p<42;p++) sum+=p%2 ? int'(unsigned'(raw[p])) : int'(unsigned'(raw[p]))<<8;
        while(sum>>16) sum=(sum&65535)+(sum>>16);
        raw[32]=byte'((~sum)>>8); raw[33]=byte'(~sum);
        for(int p=0;p<n;p++) raw[42+p]=udp[off+p];
        raw[total-5]=byte'(sn); crc='1;
        for(int p=8;p<total-4;p++) crc=tb_crc32_byte(crc,raw[p]); crc=~crc;
        for(int p=0;p<4;p++) raw[total-4+p]=byte'(crc>>(8*p));
        for(int p=0;p<total;p++) begin @(negedge clk); manual_valid=3; manual_data={raw[p],raw[p]}; end
        @(negedge clk); manual_valid=0; manual_data=0;
        repeat(100) @(negedge clk);
    endtask

    initial begin
        integer saved,target,frames_before;
        if(!$value$plusargs("CAP=%s",capture)) capture="post_synth_capture.txt";
        fd=$fopen(capture,"w"); if(fd==0) $fatal(1,"capture open failed");
        repeat(32) @(negedge clk); rst=0;
        // 等待完整 4096 RX VL 状态初始化，不改写内部寄存器。
        repeat(9000) @(negedge clk);
        application_case(1,1,1,0);
        application_case(2,17,2,1);
        application_case(3,63,3,2);
        application_case(4,1471,4,3);
        application_case(5,31,5,5);
        saved=deliveries; frames_before=frame_count[0]+frame_count[1];
        loop_mode=4; source(16'h1006,33,6);
        repeat(3000) @(negedge clk);
        tb_check(frame_count[0]+frame_count[1]>=frames_before+2,"bad-copy test actually transmitted");
        tb_check(deliveries==saved,"both bad CRC copies rejected");
        manual_rx=1;
        target=deliveries+1; expected_len[10]=40; expected_tag[10]=10;
        fragment(10,0,32,1,1,10,0); fragment(10,32,16,0,2,10,0); wait_delivery(target);
        target=deliveries+1; expected_len[12]=40; expected_tag[12]=12;
        fragment(12,32,16,0,1,12,0); fragment(12,0,32,1,2,12,0); wait_delivery(target);
        saved=deliveries;
        fragment(11,0,32,1,1,11,1); fragment(11,32,16,0,2,11,1);
        repeat(1500) @(negedge clk);
        tb_check(deliveries==saved,"bad whole-datagram UDP checksum rejected");
        tb_check(app_drop_count==0,"all valid application messages accepted");
        source(16'hffff,17,13); source(16'h1007,8193,14);
        repeat(1000) @(negedge clk);
        tb_check(app_drop_count==2,"unknown VLID and overlength rejected atomically");
        tb_check(deliveries==7,"seven unique messages delivered");
        tb_check(expected_seen[6]==0 && expected_seen[7]==0 && expected_seen[11]==0,"no invalid message delivery");
        tb_check(app_pos==0,"no partial application message");
        $fdisplay(fd,"RESULT %0d %0d %0d %0d %0d %0d",tb_checks,tb_errors,frame_count[0],frame_count[1],deliveries,app_drop_count);
        $fclose(fd); tb_finish("tb_afdx_es_post_synth"); $finish;
    end
    initial begin #400000; $fatal(1,"production post-synthesis watchdog"); end
endmodule
