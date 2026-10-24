/*
 * ================================================================
 * File Name    : tb_afdx_gmii_boundary.sv
 * Module Name  : tb_afdx_gmii_boundary
 * Function:
 *   按答疑第 96/97 行探测输入 SFD 到同帧输出 SFD 的验收边界。
 *   保留旧网络边界诊断，直接实例化 MAC 集成层，不用 TB 应用回送或强制内部信号。
 *   抓包供独立检查器匹配；不存在对应输出时必须失败，不报告零延迟。
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_gmii_boundary;
    `include "tb_common.svh"
    reg clk = 0, rst = 1;
    always #4 clk = ~clk;
    reg [15:0] rxd = 0;
    reg [1:0] rx_valid = 0;
    wire [15:0] txd;
    wire [1:0] tx_valid, tx_ready;
    // 普通业务使用 VL1..16；保留管理面，关闭独立演示发生器以排除无关输出。
    afdx_mac_system #(
        .APP_GEN_EN (0),
        .MGMT_EN    (1)
    ) dut (
        .clk        (clk),
        .reset      (rst),
        .tx_data    (txd[7:0]),
        .rx_data    (rxd[7:0]),
        .tx_valid   (tx_valid[0]),
        .rx_valid   (rx_valid[0]),
        .tx_ready   (tx_ready[0]),
        .rx_ready   (1'b0),
        .b_tx_data  (txd[15:8]),
        .b_rx_data  (rxd[15:8]),
        .b_tx_valid (tx_valid[1]),
        .b_rx_valid (rx_valid[1]),
        .b_tx_ready (tx_ready[1]),
        .b_rx_ready (1'b0),
        .rx_er          (1'b0),
        .b_rx_er        (1'b0),
        .tx_clk         (),
        .tx_er          (),
        .b_tx_clk       (),
        .b_tx_er        (),
        .host_tx_vl     ({`AFDX_TX_VLW{1'b0}}),
        .host_tx_valid  (1'b0),
        .host_tx_data   (8'd0),
        .host_tx_last   (1'b0),
        .host_tx_ready  (),
        .host_rx_vl     (),
        .host_rx_valid  (),
        .host_rx_data   (),
        .host_rx_last   (),
        .host_rx_ready  (1'b1),
        .host_rx_len    (),
        .host_rx_sn     (),
        .host_rx_net    (),
        .host_rx_ts     (),
        .host_rx_src_ip (),
        .host_rx_sport  (),
        .host_rx_dport  ()
    );
    integer scenario = 0, active_vls, cap, out_count = 0, business_out_count = 0;
    integer sizes[0:3] = '{17,465,977,1471};
    time epoch;
    string prefix;
    function automatic [15:0] checksum(input reg [31:0] sum);
        reg [31:0] folded;
        folded = sum[15:0] + sum[31:16];
        folded = folded[15:0] + folded[31:16];
        return ~folded[15:0];
    endfunction
    task automatic inject(input integer net, vl, round_id, length);
        byte unsigned frame[0:1517];
        reg [15:0] vlid, iplen, uplen, ck;
        reg [31:0] sum, crc;
        integer last_body;
        for (integer i=0; i<1518; i++) frame[i]=0;
        vlid = 16'h1000 + vl;
        iplen = 28 + length; uplen = 8 + length;
        frame[0]=3; frame[4]=vlid>>8; frame[5]=vlid;
        frame[6]=2; frame[9]=1; frame[10]=1; frame[11]=8'h21+net;
        frame[12]=8'h08; frame[14]=8'h45;
        frame[16]=iplen>>8; frame[17]=iplen;
        frame[18]=round_id>>8; frame[19]=round_id;
        frame[22]=1; frame[23]=17;
        frame[26]=10; frame[27]=1; frame[28]=1; frame[29]=1;
        frame[30]=8'he0; frame[31]=8'he0; frame[32]=vlid>>8; frame[33]=vlid;
        sum=0;
        for(integer i=14; i<34; i+=2) sum+={frame[i],frame[i+1]};
        ck=checksum(sum); frame[24]=ck>>8; frame[25]=ck;
        frame[34]=(1000+vl)>>8; frame[35]=1000+vl;
        frame[36]=(2000+vl)>>8; frame[37]=2000+vl;
        frame[38]=uplen>>8; frame[39]=uplen;
        for(integer i=0; i<length; i++) frame[42+i]=vl+round_id*17+i*7;
        last_body=42+length; frame[last_body]=round_id+1;
        crc=32'hffffffff;
        for(integer i=0; i<=last_body; i++) crc=tb_crc32_byte(crc,frame[i]);
        for(integer i=0; i<4; i++) frame[last_body+1+i]=(~crc)>>(i*8);
        for(integer i=0; i<8; i++) begin
            @(negedge clk);
            rx_valid[net]=1;
            rxd[net*8 +: 8]=i==7 ? 8'hd5 : 8'h55;
        end
        for(integer i=0; i<last_body+5; i++) begin
            @(negedge clk); rxd[net*8 +: 8]=frame[i];
        end
        @(negedge clk); rx_valid[net]=0; rxd[net*8 +: 8]=0;
        repeat(12) @(negedge clk);
    endtask
    // IN/OUT 使用同一个时钟沿和相同探测器；SFD 时间在看到 d5 的边沿锁存。
    genvar lane;
    generate for(lane=0; lane<4; lane=lane+1) begin : g_capture
        localparam NET = lane % 2;
        wire valid = lane<2 ? rx_valid[NET] : tx_valid[NET];
        wire [7:0] data = lane<2 ? rxd[NET*8 +: 8] : txd[NET*8 +: 8];
        bit in_frame = 0;
        integer pos = 0, idle = 100;
        time sfd;
        string body;
        reg [15:0] observed_vlid;
        always @(posedge clk) if(!rst) begin
            if(valid) begin
                if(!in_frame) begin
                    tb_check(idle>=12,"GMII IFG >=12 clocks");
                    in_frame=1; pos=0; body="";
                end
                if(pos<7) tb_check(data===8'h55,"preamble");
                else if(pos==7) begin
                    tb_check(data===8'hd5,"SFD"); sfd=$time;
                end else body={body,$sformatf("%02x",data)};
                if(pos==12) observed_vlid[15:8]=data;
                if(pos==13) observed_vlid[7:0]=data;
                pos++; idle=0;
            end else begin
                idle++;
                if(in_frame) begin
                    $fdisplay(cap,"%s %0d %0d %0d %s",lane<2?"IN":"OUT",NET,sfd,$time,body);
                    if(lane>=2) out_count++;
                    if(lane>=2 && observed_vlid>=16'h1001 && observed_vlid<16'h1001+active_vls)
                        business_out_count++;
                    in_frame=0;
                end
            end
        end
    end endgenerate
    initial begin
        void'($value$plusargs("SCENARIO=%d",scenario));
        if(scenario<0 || scenario>1) $fatal(1,"SCENARIO must be 0 or 1");
        active_vls = scenario==0 ? 1 : 16;
        prefix=$sformatf("gmii_boundary_%0d",scenario);
        cap=$fopen({prefix,".txt"},"w");
        if(!cap) $fatal(1,"cannot open capture");
        repeat(40) @(negedge clk); rst=0;
        // 等待状态表清扫；不访问或强制 DUT 内部 ready/data。
        repeat(12000) @(negedge clk);
        epoch=$time;
        fork
            begin
                for(integer r=0; r<8; r++) begin
                    if($time<epoch+r*2500000) #(epoch+r*2500000-$time);
                    for(integer v=1; v<=active_vls; v++) inject(0,v,r,sizes[(r+v-1)%4]);
                end
            end
            begin
                repeat(8) @(negedge clk);
                for(integer r=0; r<8; r++) begin
                    if($time<epoch+64+r*2500000) #(epoch+64+r*2500000-$time);
                    for(integer v=1; v<=active_vls; v++) inject(1,v,r,sizes[(r+v-1)%4]);
                end
            end
        join
        // 等待超过 2ms，并留足最大帧接收时间后才判缺失。
        repeat(252000) @(negedge clk);
        $fdisplay(cap,"END %0d %0d %0d %0d %0d %0d %0d",$time,
            dut.u_system.cnt_sink_msgs,
            dut.u_system.cnt_rx_rm_drop,
            dut.u_system.cnt_rx_hdr_err,
            dut.u_system.cnt_rx_csum_err,
            dut.u_system.cnt_rx_mac_error,
            dut.u_system.cnt_rx_full_drop);
        $fclose(cap);
        $display("BOUNDARY_COMPLETE scenario=%0d logical_inputs=%0d outputs=%0d",scenario,active_vls*8,out_count);
        tb_check(business_out_count==active_vls*8*2,"business GMII outputs exist; unrelated management frames do not count");
        tb_finish("tb_afdx_gmii_boundary");
        $finish;
    end
    initial begin
        #25000000;
        $fatal(1,"boundary watchdog");
    end
endmodule
