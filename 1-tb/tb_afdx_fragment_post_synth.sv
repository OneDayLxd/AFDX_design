/*
 * ================================================================
 * File Name    : tb_afdx_fragment_post_synth.sv
 * Module Name  : tb_afdx_fragment_post_synth
 * Function:
 *   同一激励对照 RTL 与综合网表：短消息双网发送、分片接收、去重、坏 FCS 隔离。
 *   功能测试以密集分片缩短仿真时间，不作为 BAG/吞吐/时序验收。
 * Sub Module:
 *   AFDX_fragment_smoke_top
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_fragment_post_synth;
    `include "tb_common.svh"
    reg clk=0,rst=1;
    always #4 clk=~clk;
    reg [7:0] itd=0;
    reg itv=0,itl=0;
    wire [7:0] ord;
    wire orv,orl;
    wire [15:0] txd;
    wire [1:0] txv;
    reg [15:0] rxd=0;
    reg [1:0] rxv=0;
    AFDX_fragment_smoke_top dut (
        .clk           (clk),
        .rst           (rst),
        .app_tx_data   (itd),
        .app_tx_valid  (itv),
        .app_tx_last   (itl),
        .app_rx_data   (ord),
        .app_rx_valid  (orv),
        .app_rx_last   (orl),
        .gmii_tx_data  (txd),
        .gmii_tx_valid (txv),
        .gmii_rx_data  (rxd),
        .gmii_rx_valid (rxv)
    );
    int fd,pos=0,delivered=0;
    int frame_pos[2]='{0,0},frames[2]='{0,0};
    bit active[2]='{0,0};
    reg [31:0] crc[2];
    string capture;
    byte udp[48],frame[72];
    function automatic byte pattern(int p);return byte'(p*7+23);endfunction
    always @(posedge clk)if(!rst)begin
        if(orv)begin
            tb_check(delivered==0,"only one valid unique UDP message delivered");
            tb_check(ord== (pos==0 ? 8'h10:pos==1 ? 8'h01:pattern(pos-2)),"reassembled payload");
            tb_check(orl==(pos==41),"last at whole-message end");
            $fdisplay(fd,"A %0d %02x %0d",$time,ord,orl);
            pos++;
            if(orl)begin delivered++;pos=0;end
        end
        for(int n=0;n<2;n++)begin
            if(txv[n])begin
                if(!active[n])begin active[n]=1;frame_pos[n]=0;crc[n]='1;end
                if(frame_pos[n]<7)tb_check(txd[n*8+:8]==8'h55,"TX preamble");
                else if(frame_pos[n]==7)tb_check(txd[n*8+:8]==8'hd5,"TX SFD");
                else crc[n]=tb_crc32_byte(crc[n],txd[n*8+:8]);
                $fdisplay(fd,"T %0d %0d %02x",$time,n,txd[n*8+:8]);
                frame_pos[n]++;
            end else if(active[n])begin
                tb_check(frame_pos[n]==72,"minimum TX frame");
                tb_check(crc[n]==32'hdebb20e3,"TX FCS");
                frames[n]++;active[n]=0;
            end
        end
    end
    task automatic send_fragment(int off,n,bit mf,int sn,bit corrupt);
        int iplen,flags;
        int unsigned s,c;
        for(int p=0;p<72;p++)frame[p]=0;
        for(int p=0;p<7;p++)frame[p]=8'h55;frame[7]=8'hd5;
        frame[8]=8'h03;frame[12]=8'h10;frame[13]=8'h01;
        frame[14]=8'h02;frame[17]=1;frame[18]=1;frame[19]=8'h20;
        frame[20]=8'h08;frame[22]=8'h45;
        iplen=20+n;frame[24]=byte'(iplen>>8);frame[25]=byte'(iplen);
        frame[27]=8'h72;flags=(mf?'h2000:0)|(off/8);
        frame[28]=byte'(flags>>8);frame[29]=byte'(flags);frame[30]=1;frame[31]=17;
        frame[34]=10;frame[35]=1;frame[36]=1;frame[37]=1;
        frame[38]=8'he0;frame[39]=8'he0;frame[40]=8'h10;frame[41]=8'h01;
        s=0;for(int p=22;p<42;p++)s+=p%2 ? int'(unsigned'(frame[p])) : int'(unsigned'(frame[p]))<<8;
        while(s>>16)s=(s&65535)+(s>>16);
        frame[32]=byte'((~s)>>8);frame[33]=byte'(~s);
        // 40 IP 数据字节对应 83 字节 MAC/FCS，故使用动态数组发送。
        begin
            byte raw[];
            int total;
            total=8+((14+iplen<59)?60:15+iplen)+4;
            raw=new[total];
            for(int p=0;p<42;p++)raw[p]=frame[p];
            for(int p=0;p<n;p++)raw[42+p]=udp[off+p];
            raw[total-5]=byte'(sn);
            c='1;for(int p=8;p<total-4;p++)c=tb_crc32_byte(c,raw[p]);c=~c;
            for(int p=0;p<4;p++)raw[total-4+p]=byte'(c>>(p*8));
            if(corrupt)raw[50]^=1;
            for(int p=0;p<total;p++)begin
                @(negedge clk);rxv=3;rxd={raw[p],raw[p]};
            end
            @(negedge clk);rxv=0;rxd=0;
            repeat(30)@(negedge clk);
        end
    endtask
    initial begin
        int unsigned s,cs;
        if(!$value$plusargs("CAP=%s",capture))capture="fragment_post_capture.txt";
        fd=$fopen(capture,"w");
        repeat(40)@(negedge clk);rst=0;
        repeat(400)@(negedge clk);
        @(negedge clk);itv=1;itd=8'h10;
        @(negedge clk);itd=2;
        for(int p=0;p<17;p++)begin @(negedge clk);itd=pattern(p);itl=p==16;end
        @(negedge clk);itv=0;itl=0;
        udp[0]=8'h03;udp[1]=8'he9;udp[2]=8'h07;udp[3]=8'hd1;
        udp[4]=0;udp[5]=48;udp[6]=0;udp[7]=0;
        for(int p=0;p<40;p++)udp[p+8]=pattern(p);
        s='h0a01+'h0101+'he0e0+'h1001+17+48;
        for(int p=0;p<48;p++)s+=p%2 ? int'(unsigned'(udp[p])) : int'(unsigned'(udp[p]))<<8;
        while(s>>16)s=(s&65535)+(s>>16);cs=(~s)&65535;
        udp[6]=byte'(cs>>8);udp[7]=byte'(cs);
        // 首片 32 字节 IP 数据、末片 16 字节；A/B 同时到达只交付一次。
        send_fragment(0,32,1,1,0);send_fragment(32,16,0,2,0);
        repeat(1000)@(negedge clk);
        // CRC 损坏的两个副本均不得交付；独立 IP ID 无关，MAC 先行拒绝。
        send_fragment(0,32,1,3,1);send_fragment(32,16,0,4,1);
        repeat(1000)@(negedge clk);
        tb_check(frames[0]==1 && frames[1]==1,"short application sent through both real MACs");
        tb_check(delivered==1,"reassembly and duplicate suppression in synthesized datapath");
        $fclose(fd);tb_finish("tb_afdx_fragment_post_synth");$finish;
    end
    initial begin #60000;$fatal(1,"post-synthesis watchdog");end
endmodule
