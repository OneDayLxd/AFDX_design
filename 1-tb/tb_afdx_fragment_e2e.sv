/*
 * ================================================================
 * File Name    : tb_afdx_fragment_e2e.sv
 * Module Name  : tb_afdx_fragment_e2e
 * Function:
 *   双端系统真实 MAC 分片收发，独立审计线上 IP 字段、FCS、UDP 校验及载荷。
 *   覆盖单帧边界、奇数长消息、8192 字节、仅 B 网可用和超长原子丢弃。
 * Sub Module:
 *   AFDX_top
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_fragment_e2e;
    `include "tb_common.svh"
    reg clk=0,rst=1;
    always #4 clk=~clk;
    reg [15:0] tx_data=0;
    reg [1:0] tx_valid=0,tx_last=0;
    wire [15:0] rx_data;
    wire [1:0] rx_valid,rx_last,ready,drop;
    wire [63:0] drops;
    int count[2]='{0,0},pos[2]='{0,0};
    int wanted_length[2],wanted_tag[2];
    time first[2];
    int wire_frames=0,wire_datagrams=0;
    AFDX_top #(
        .MGMT_EN             (0)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .tx_data             (tx_data),
        .tx_valid            (tx_valid),
        .tx_last             (tx_last),
        .tx_ready            (ready),
        .rx_data             (rx_data),
        .rx_valid            (rx_valid),
        .rx_last             (rx_last),
        .rx_ready            (2'b00),
        .app_drop            (drop),
        .app_drop_reason     (),
        .app_drop_count      (drops)
    );
    function automatic byte pattern(int tag,p);
        return byte'(tag*17+p*7+(p>>8));
    endfunction
    task automatic source(int src,n,tag);
        wanted_length[src]=n;wanted_tag[src]=tag;
        @(negedge clk);tx_valid[src]=1;tx_last[src]=0;tx_data[src*8+:8]=8'h10;
        first[src]=$time+4;
        @(negedge clk);tx_data[src*8+:8]=8'h01;
        for(int p=0;p<n;p++) begin
            @(negedge clk);tx_valid[src]=1;tx_last[src]=p==n-1;
            tx_data[src*8+:8]=pattern(tag,p);
            if(p%97==11 && p!=n-1)begin
                @(negedge clk);tx_valid[src]=0;tx_last[src]=1;
                repeat(2)@(negedge clk);
            end
        end
        @(negedge clk);tx_valid[src]=0;tx_last[src]=0;
    endtask
    always @(posedge clk) if(!rst) for(int dst=0;dst<2;dst++) if(rx_valid[dst]) begin
        automatic int src=1-dst;
        automatic byte b=rx_data[dst*8+:8];
        if(pos[dst]<2) tb_check(b== (pos[dst]==0 ? 8'h10:8'h01),"application VLID");
        else tb_check(b===pattern(wanted_tag[src],pos[dst]-2),"whole message matches application input");
        tb_check(rx_last[dst]==(pos[dst]==wanted_length[src]+1),"one last for the complete application message");
        pos[dst]++;
        if(rx_last[dst]) begin
            $display("fragment e2e src=%0d payload=%0d latency_us=%.3f",src,wanted_length[src],real'($time-first[src])/1000.0);
            count[dst]++;pos[dst]=0;
        end
    end
    byte capture[4][1526],udp[4][8200];
    int cp[4]='{default:0},up[4]='{default:0},ip_id[4],sn[4]='{default:0};
    bit active[4]='{default:0};
    task automatic audit(int lane,n);
        int iplen,flags,off,ipdata,ulen;
        int unsigned s,crc;
        int src=lane/2;
        tb_check(n>=72 && n<=1526,"MAC frame size");
        for(int p=0;p<7;p++)tb_check(capture[lane][p]==8'h55,"preamble");
        tb_check(capture[lane][7]==8'hd5,"SFD");
        crc='1;for(int p=8;p<n;p++)crc=tb_crc32_byte(crc,capture[lane][p]);
        tb_check(crc==32'hdebb20e3,"independent Ethernet FCS");
        iplen=(int'(unsigned'(capture[lane][24]))<<8)|int'(unsigned'(capture[lane][25]));
        flags=(int'(unsigned'(capture[lane][28]))<<8)|int'(unsigned'(capture[lane][29]));
        off=(flags&8191)*8;ipdata=iplen-20;
        tb_check(n==8+((14+iplen<59)?60:15+iplen)+4,"IP length, padding, SN and FCS boundaries");
        s=0;for(int p=22;p<42;p++)s+=p%2 ? int'(unsigned'(capture[lane][p])) : int'(unsigned'(capture[lane][p]))<<8;
        while(s>>16)s=(s&65535)+(s>>16);
        tb_check(s==65535,"independent IPv4 checksum");
        tb_check((flags&'hc000)==0,"reserved/DF clear");
        tb_check(!(flags&'h2000) || ipdata%8==0,"nonfinal fragment aligned to eight bytes");
        tb_check(off==up[lane],"contiguous fragment offsets on wire");
        if(off==0)ip_id[lane]=(int'(unsigned'(capture[lane][26]))<<8)|int'(unsigned'(capture[lane][27]));
        else tb_check(ip_id[lane]==((int'(unsigned'(capture[lane][26]))<<8)|int'(unsigned'(capture[lane][27]))),"same IP ID for all pieces");
        for(int p=0;p<ipdata;p++) if(off+p<8200)udp[lane][off+p]=capture[lane][42+p];
        up[lane]+=ipdata;
        sn[lane]=sn[lane]==255 ? 1:sn[lane]+1;
        tb_check(capture[lane][n-5]==8'(sn[lane]),"per-VL SN advances per fragment");
        wire_frames++;
        if(!(flags&'h2000))begin
            ulen=(int'(unsigned'(udp[lane][4]))<<8)|int'(unsigned'(udp[lane][5]));
            tb_check(up[lane]==ulen && ulen==wanted_length[src]+8,"whole UDP length independent of fragment lengths");
            s='h0a01+'h0101+'he0e0+'h1001+17+ulen;
            for(int p=0;p<ulen;p++)s+=p%2 ? int'(unsigned'(udp[lane][p])) : int'(unsigned'(udp[lane][p]))<<8;
            while(s>>16)s=(s&65535)+(s>>16);
            tb_check(s==65535,"independent whole-datagram UDP checksum");
            for(int p=8;p<ulen;p++)tb_check(udp[lane][p]===pattern(wanted_tag[src],p-8),"wire payload reconstructed independently");
            up[lane]=0;wire_datagrams++;
        end
    endtask
    always @(posedge clk) if(!rst) for(int lane=0;lane<4;lane++)begin
        if(dut.gmii_tx_valid[lane])begin
            active[lane]=1;
            if(cp[lane]<1526)capture[lane][cp[lane]]=dut.gmii_tx_data[lane*8+:8];
            cp[lane]++;
        end else if(active[lane])begin
            audit(lane,cp[lane]);active[lane]=0;cp[lane]=0;
        end
    end
    int lengths[7]='{1,1471,1472,2920,2921,8191,8192};
    initial begin
        repeat(12)@(negedge clk);rst=0;
        repeat(10000)@(negedge clk);
        for(int t=0;t<7;t++)begin
            // 全部 A 网接收失效时，长消息仍应由 B 网完整重组。
            if(t==6)begin force dut.gmii_rx_valid[0]=0;force dut.gmii_rx_valid[2]=0;end
            fork source(0,lengths[t],t+1);source(1,lengths[t],t+65);join
            wait(count[0]==t+1 && count[1]==t+1);
            repeat(300)@(negedge clk);
            if(t==6)begin release dut.gmii_rx_valid[0];release dut.gmii_rx_valid[2];end
            tb_check(drops==0,"valid long messages never discarded at input");
        end
        // 超过声明的重组上限应整条拒绝，不发布半条消息。
        fork source(0,8193,100);source(1,8193,101);join
        repeat(300000)@(negedge clk);
        tb_check(drops=={32'd1,32'd1},"overlength message rejected atomically");
        tb_check(count[0]==7 && count[1]==7,"one delivery per A/B datagram, no overlength output");
        tb_check(wire_datagrams==28,"four lanes times seven complete wire datagrams");
        tb_check(wire_frames==84,"expected number of physical fragments including A/B replicas");
        tb_finish("tb_afdx_fragment_e2e");$finish;
    end
    initial begin #80000000;$fatal(1,"fragment e2e timeout");end
endmodule
