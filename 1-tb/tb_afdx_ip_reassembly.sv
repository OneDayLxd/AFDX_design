/*
 * ================================================================
 * File Name    : tb_afdx_ip_reassembly.sv
 * Module Name  : tb_afdx_ip_reassembly
 * Function:
 *   独立构造 UDP 数据报，检查乱序、交错、缺片、重叠、校验与容量边界。
 * Sub Module:
 *   afdx_ip_reassembly
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_ip_reassembly;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    always #4 clk=~clk;
    reg [11:0] ivl=1;
    reg [7:0] data=0;
    reg valid=0,last=0;
    reg [10:0] length=0;
    reg [95:0] meta={32'd123,32'h0a010101,16'd1001,16'd2001};
    reg [31:0] fragment=0;
    wire ready,ov,ol,drop;
    wire [11:0] ovl;
    wire [7:0] od;
    wire [15:0] olen;
    wire [95:0] ometa;
    wire [31:0] drops;
    reg ordy=1;
    int cycle=0,received=0,output_position=0;
    int expected_length[$], expected_tag[$];
    byte datagram[8200];
    afdx_ip_reassembly dut (
        .clk         (clk),
        .rst         (rst),
        .in_vl       (ivl),
        .in_data     (data),
        .in_valid    (valid),
        .in_last     (last),
        .in_len      (length),
        .in_meta     (meta),
        .in_fragment (fragment),
        .in_ready    (ready),
        .out_vl      (ovl),
        .out_data    (od),
        .out_valid   (ov),
        .out_last    (ol),
        .out_len     (olen),
        .out_meta    (ometa),
        .out_ready   (ordy),
        .drop_pulse  (drop),
        .drop_count  (drops)
    );
    function automatic byte pattern(int tag,pos);
        return byte'(tag*31+pos*7+(pos>>8));
    endfunction
    task automatic make_udp(int n,tag);
        int unsigned s,cs;
        datagram[0]=8'h03;datagram[1]=8'he9;
        datagram[2]=8'h07;datagram[3]=8'hd1;
        datagram[4]=byte'((n+8)>>8);datagram[5]=byte'(n+8);
        datagram[6]=0;datagram[7]=0;
        for(int p=0;p<n;p++) datagram[p+8]=pattern(tag,p);
        s='h0a01+'h0101+'he0e0+'h1001+17+n+8;
        for(int p=0;p<n+8;p++) s+=p%2 ? int'(unsigned'(datagram[p])) : int'(unsigned'(datagram[p]))<<8;
        while(s>>16) s=(s&65535)+(s>>16);
        cs=(~s)&65535;if(cs==0)cs=65535;
        datagram[6]=byte'(cs>>8);datagram[7]=byte'(cs);
    endtask
    task automatic send_piece(int id,off,n,bit mf,bit corrupt=0);
        @(negedge clk);
        fragment={1'b1,mf,16'(id),13'(off/8),1'b0};length=11'(n);
        for(int p=0;p<n;p++) begin
            data=datagram[off+p] ^ ((corrupt && p==n-1) ? 8'h01 : 0);
            valid=1;last=p==n-1;
            do @(posedge clk); while(!ready);
            @(negedge clk);
            if(p%37==9) begin
                valid=0;last=0;repeat(2)@(negedge clk);
            end
        end
        valid=0;last=0;
    endtask
    task automatic expect_message(int n,tag);
        expected_length.push_back(n);expected_tag.push_back(tag);
    endtask
    task automatic drain;
        repeat(20000) @(negedge clk);
        tb_check(expected_length.size()==0,"all expected datagrams delivered");
    endtask
    always @(negedge clk) begin
        cycle++;
        ordy=cycle%11!=2 && cycle%11!=3;
    end
    always @(posedge clk) if(!rst && ov && ordy) begin
        tb_check(expected_length.size()!=0,"no partial or invalid datagram escapes");
        if(expected_length.size()!=0) begin
            tb_check(od===pattern(expected_tag[0],output_position),"reassembled byte matches source");
            tb_check(olen==expected_length[0],"whole UDP payload length");
            tb_check(ovl==1,"VL retained");
            tb_check(ometa[63:0]=={32'h0a010101,16'd1001,16'd2001},"source and UDP ports retained");
            tb_check(ol==(output_position==expected_length[0]-1),"last only at datagram end");
            output_position++;
            if(ol) begin
                void'(expected_length.pop_front());void'(expected_tag.pop_front());
                output_position=0;received++;
            end
        end
    end
    initial begin
        repeat(5)@(negedge clk);rst=0;
        // 乱序：末片先到，再首片；包含奇数长度及 RAM 最大边界。
        make_udp(1472,1);expect_message(1472,1);
        send_piece(1,1464,16,0);send_piece(1,0,1464,1);drain();
        make_udp(8192,2);expect_message(8192,2);
        send_piece(2,7320,880,0);
        for(int off=5856;off>=0;off-=1464) send_piece(2,off,1464,1);
        drain();
        make_udp(2921,3);expect_message(2921,3);
        send_piece(3,0,1464,1);send_piece(3,1464,1464,1);send_piece(3,2928,1,0);drain();
        // 缺片不得交付；其他 IP ID 的完整消息仍可通过。
        make_udp(1472,4);send_piece(4,0,1464,1);
        make_udp(1472,5);expect_message(1472,5);
        send_piece(5,0,1464,1);send_piece(5,1464,16,0);drain();
        // 重叠整包拒绝，不能拼出重复/旧数据。
        make_udp(1472,6);send_piece(6,0,1464,1);send_piece(6,1456,24,0);drain();
        tb_check(drops==1,"overlap counted");
        // 整包 UDP 校验和错误、非末片未对齐、越界、UDP 长度不符。
        make_udp(1472,7);send_piece(7,0,1464,1);send_piece(7,1464,16,0,1);drain();
        make_udp(1472,8);send_piece(8,0,15,1);drain();
        make_udp(1472,9);send_piece(9,8192,16,0);drain();
        make_udp(1472,10);send_piece(10,0,1464,1);send_piece(10,1464,15,0);drain();
        tb_check(drops==5,"all invalid datagrams counted");
        // 有限上下文耗尽后可替换恢复；未完成消息没有定时超时。
        for(int id=20;id<27;id++)begin make_udp(1472,id);send_piece(id,0,1464,1);end
        make_udp(1472,30);expect_message(1472,30);
        send_piece(30,0,1464,1);send_piece(30,1464,16,0);drain();
        tb_check(drops>=9,"bounded context replacement observable");
        @(negedge clk);rst=1;repeat(5)@(negedge clk);rst=0;
        // 复位取消未完成重组，并允许 ID 复用。
        make_udp(1472,4);expect_message(1472,4);
        send_piece(4,0,1464,1);send_piece(4,1464,16,0);drain();
        tb_check(received==6,"six complete messages, no incomplete delivery");
        tb_finish("tb_afdx_ip_reassembly");$finish;
    end
    initial begin #20000000;$fatal(1,"reassembly timeout");end
endmodule
