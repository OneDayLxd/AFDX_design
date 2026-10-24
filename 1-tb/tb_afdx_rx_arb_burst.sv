/*
 * ================================================================
 * File Name    : tb_afdx_rx_arb_burst.sv
 * Module Name  : tb_afdx_rx_arb_burst
 *
 * Function:
 *   125MHz、A/B 连续长短帧及输出长背压下的帧级公平性和数据稳定性。
 *   两路始终有帧时必须交替服务；不得交错帧、丢字节或破坏时间戳。
 *
 * Sub Module:
 *   afdx_rx_arb
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_rx_arb_burst;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    always #4 clk=~clk;
    localparam FRAMES=128;
    integer sent[0:1]='{0,0}, offset[0:1]='{0,0}, received[0:1]='{0,0};
    integer cycle=0, position=0, total=0, service_gap=0, peak_gap=0;
    reg out_ready=0;
    wire [1:0] ready;
    wire out_valid, out_last, out_net;
    wire [7:0] out_data;
    wire [31:0] out_ts;
    bit held=0, frame_active=0;
    bit frame_net;
    reg [41:0] held_value;
    function automatic integer length(input integer net, id);
        return ((id+net)%3==0) ? 1514 : ((id+net)%3==1) ? 60 : 257;
    endfunction
    function automatic byte pattern(input integer net, id, pos);
        return byte'(net*91+id*13+pos*7);
    endfunction
    afdx_rx_arb dut (
        .clk       (clk),
        .rst       (rst),
        .a_data    (pattern(0,sent[0],offset[0])),
        .a_valid   (sent[0]<FRAMES),
        .a_last    (offset[0]==length(0,sent[0])-1),
        .a_ready   (ready[0]),
        .b_data    (pattern(1,sent[1],offset[1])),
        .b_valid   (sent[1]<FRAMES),
        .b_last    (offset[1]==length(1,sent[1])-1),
        .b_ready   (ready[1]),
        .out_data  (out_data),
        .out_valid (out_valid),
        .out_last  (out_last),
        .out_net   (out_net),
        .a_ts      (32'(sent[0]*3)),
        .b_ts      (32'(sent[1]*3+1)),
        .out_ts    (out_ts),
        .out_ready (out_ready)
    );
    always @(negedge clk) begin
        cycle++;
        // 连续 50 拍背压和零星单拍停顿，包含帧尾交接期间。
        out_ready=!rst && cycle%200>=50 && cycle%11!=0;
    end
    always @(posedge clk) if(!rst) begin
        tb_check(ready!=2'b11,"only one source accepted per cycle");
        for(integer n=0;n<2;n++) if(ready[n] && sent[n]<FRAMES) begin
            if(offset[n]==length(n,sent[n])-1) begin
                sent[n] <= sent[n]+1;
                offset[n] <= 0;
            end else offset[n] <= offset[n]+1;
        end
        if(held) tb_check(out_valid && {out_ts,out_net,out_last,out_data}===held_value,
                         "all output fields stable while stalled");
        held=out_valid && !out_ready;
        held_value={out_ts,out_net,out_last,out_data};
        service_gap++;
        if(out_valid && out_ready) begin
            if(!frame_active) begin
                frame_net=out_net; frame_active=1;
                tb_check(out_net==(total%2),"round robin under continuous A/B demand");
            end
            tb_check(out_net==frame_net,"no interleaving within frame");
            tb_check(out_data===pattern(out_net,received[out_net],position),"network frame byte order");
            tb_check(out_ts==received[out_net]*3+out_net,"frame timestamp association");
            tb_check(out_last==(position==length(out_net,received[out_net])-1),"exact frame boundary");
            position++;
            if(out_last) begin
                received[out_net]++; total++; position=0; frame_active=0;
                if(service_gap>peak_gap) peak_gap=service_gap;
                // 最长 1514 字节，加 25%+零星背压和有限交接，不允许饿死。
                tb_check(service_gap<2500,"bounded frame service gap");
                service_gap=0;
            end
        end
    end
    initial begin
        repeat(12) @(negedge clk); rst=0;
        wait(total==2*FRAMES);
        repeat(5) @(negedge clk);
        tb_check(sent[0]==FRAMES && sent[1]==FRAMES && !out_valid,"both inputs completely drained");
        $display("ARB_BURST_RESOURCE frames=%0d peak_service_cycles=%0d",total,peak_gap);
        tb_finish("tb_afdx_rx_arb_burst"); $finish;
    end
    initial begin #5000000; $fatal(1,"RX arb burst watchdog"); end
endmodule
