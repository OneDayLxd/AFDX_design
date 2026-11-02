/*
 * ================================================================
 * File Name    : tb_afdx_fragment_rx.sv
 * Module Name  : tb_afdx_fragment_rx
 * Function:
 *   独立构造 IP 分片，检查解析元数据、原始 IP 数据及错误帧原子拒绝。
 * Sub Module:
 *   afdx_rx_parser
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_fragment_rx;
    `include "tb_common.svh"
    reg clk=0,rst=1,valid=0,last=0,lookup_hit=1;
    always #4 clk=~clk;
    reg [7:0] data=0;
    wire ready,wv,commit,abort,req;
    wire [7:0] wd;
    wire [136:0] meta;
    reg resp=0;
    always @(posedge clk)resp<=!rst&&req;
    int commits=0,aborts=0,wp=0;
    byte written[1500],frame[1600];
    afdx_rx_parser #(
        .FRAGMENT_ENABLE (1),
        .N_VL            (8),
        .VLW             (3),
        .MW              (137)
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .ts_us           (32'd99),
        .in_data         (data),
        .in_valid        (valid),
        .in_last         (last),
        .in_net          (1'b1),
        .in_ready        (ready),
        .lk_vlid         (),
        .lk_hit          (lookup_hit),
        .lk_idx          (3'd1),
        .q_wr_vl         (),
        .q_wr_valid      (wv),
        .q_wr_data       (wd),
        .q_wr_ready      (1'b1),
        .q_wr_commit     (commit),
        .q_wr_meta       (meta),
        .q_wr_abort      (abort),
        .ic_req          (req),
        .ic_vl           (),
        .ic_net          (),
        .ic_sn           (),
        .ic_resp         (resp),
        .ic_accept       (1'b1),
        .stat_ok         (),
        .stat_hdr_err    (),
        .stat_len_err    (),
        .stat_vl_unknown (),
        .stat_csum_err   (),
        .stat_full_drop  ()
    );
    always @(posedge clk)if(!rst)begin
        if(wv)begin written[wp]=wd;wp++;end
        if(commit)commits++;
        if(abort)aborts++;
    end
    task automatic check_frame(int flags,n,bit good,bit corrupt=0,int delta=0);
        int total,previous;
        int unsigned s;
        for(int p=0;p<1600;p++)frame[p]=0;
        frame[0]=3;frame[4]=8'h10;frame[5]=1;frame[6]=2;frame[11]=8'h40;
        frame[12]=8'h08;frame[14]=8'h45;frame[16]=byte'((n+20)>>8);frame[17]=byte'(n+20);
        frame[18]=8'h12;frame[19]=8'h34;frame[20]=byte'(flags>>8);frame[21]=byte'(flags);
        frame[22]=1;frame[23]=17;frame[26]=10;frame[27]=1;frame[28]=1;frame[29]=1;
        frame[30]=8'he0;frame[31]=8'he0;frame[32]=8'h10;frame[33]=1;
        s=0;for(int p=14;p<34;p++)s+=p%2?int'(unsigned'(frame[p])):int'(unsigned'(frame[p]))<<8;
        while(s>>16)s=(s&65535)+(s>>16);
        frame[24]=byte'((~s)>>8);frame[25]=byte'(~s);
        for(int p=0;p<n;p++)frame[34+p]=byte'(p*19+7);
        total=((34+n<59)?60:35+n)+delta;frame[total-1]=1;
        if(corrupt)frame[24]^=1;
        previous=commits;wp=0;
        for(int p=0;p<total;p++)begin
            @(negedge clk);valid=1;last=p==total-1;data=frame[p];
            do @(posedge clk);while(!ready);
        end
        @(negedge clk);valid=0;last=0;
        repeat(25)@(negedge clk);
        tb_check(commits-previous==(good?1:0),"fragment acceptance matches independent header/length rules");
        if(good)begin
            tb_check(wp==n,"only IP data, excluding SN and Ethernet padding");
            for(int p=0;p<n;p++)tb_check(written[p]===byte'(p*19+7),"raw IP bytes for reassembler");
            tb_check(meta[136:105]=={1'b1,flags[13],16'h1234,flags[12:0],1'b0},"ID, offset, MF descriptor");
            tb_check(meta[104:41]=={32'd99,32'h0a010101},"session metadata retained");
            tb_check(meta[8:0]==9'h101,"network and per-frame SN retained");
        end
    endtask
    initial begin
        repeat(5)@(negedge clk);rst=0;
        check_frame('h2000,16,1);
        check_frame(2,9,1);
        check_frame('h2000,1464,1);
        check_frame('ha000,16,0); // reserved flag
        check_frame('h6000,16,0); // DF plus MF
        check_frame('h4002,16,0); // DF plus offset
        check_frame('h2000,15,0); // nonfinal alignment
        check_frame(2,0,0);      // empty fragment
        check_frame('h2000,16,0,1);
        check_frame('h2000,16,0,0,1);
        lookup_hit=0;check_frame('h2000,16,0);lookup_hit=1;
        tb_check(commits==3 && aborts==8,"all invalid frames aborted, never committed");
        tb_finish("tb_afdx_fragment_rx");$finish;
    end
    initial begin #1000000;$fatal(1,"fragment parser watchdog");end
endmodule
