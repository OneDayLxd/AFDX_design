/*
 * ================================================================
 * File Name    : tb_afdx_rx_frame_fifo.sv
 * Module Name  : tb_afdx_rx_frame_fifo
 *
 * Function:
 *   紧凑 RX 缓存的字节/描述符独立满载、坏帧回滚、双指针回绕、背压及复位。
 *
 * Sub Module:
 *   afdx_rx_frame_fifo
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_rx_frame_fifo;
    `include "tb_common.svh"
    reg clk=0, reset=1, blocked=1;
    always #4 clk=~clk;
    reg [7:0] in_data=0;
    reg in_valid=0, in_last=0, in_error=0;
    reg [31:0] in_ts=0;
    wire [7:0] out_data;
    wire out_valid, out_last, good, error_frame, length_error, overflow;
    wire [31:0] out_ts;
    integer cycles=0;
    wire out_ready=!blocked && cycles%7<4;
    afdx_rx_frame_fifo #(
        .BYTE_LOG (8),
        .SLOT_LOG (3),
        .MIN_LEN  (1),
        .MAX_LEN  (200)
    ) dut (
        .clk           (clk),
        .reset         (reset),
        .in_data       (in_data),
        .in_valid      (in_valid),
        .in_last       (in_last),
        .in_error      (in_error),
        .in_ts         (in_ts),
        .in_ready      (),
        .stat_good     (good),
        .stat_error    (error_frame),
        .stat_length   (length_error),
        .stat_overflow (overflow),
        .out_data      (out_data),
        .out_valid     (out_valid),
        .out_last      (out_last),
        .out_ts        (out_ts),
        .out_ready     (out_ready)
    );
    integer expected=0, received=0, position=0;
    integer lengths[0:255], seeds[0:255];
    integer n_good=0, n_error=0, n_length=0, n_overflow=0;
    bit held=0;
    reg [40:0] held_value;
    function automatic byte pattern(input integer seed, pos);
        return byte'(seed+pos*7);
    endfunction
    always @(posedge clk) begin
        cycles++;
        if(reset) held=0;
        else begin
            n_good+=good; n_error+=error_frame; n_length+=length_error; n_overflow+=overflow;
            tb_check(dut.used_bytes<=256 && dut.used_frames<=8,"both capacities bounded");
            if(held) tb_check(out_valid && {out_ts,out_last,out_data}===held_value,"stalled RAM output stable");
            held=out_valid && !out_ready; held_value={out_ts,out_last,out_data};
            if(out_valid && out_ready) begin
                tb_check(received<expected,"only committed frames published");
                tb_check(out_data===pattern(seeds[received],position),"byte pool order across wrap/rollback");
                tb_check(out_last==(position==lengths[received]-1),"descriptor length");
                tb_check(out_ts==seeds[received]*13,"first-byte timestamp");
                if(out_last) begin received++; position=0; end else position++;
            end
        end
    end
    task automatic send(input integer length, seed, input bit bad, deliver);
        if(deliver) begin lengths[expected]=length; seeds[expected]=seed; expected++; end
        for(integer k=0;k<length;k++) begin
            @(negedge clk); in_valid=1; in_data=pattern(seed,k); in_last=k==length-1;
            in_error=bad && in_last; in_ts=seed*13;
        end
        @(negedge clk); in_valid=0; in_last=0; in_error=0;
        repeat(5) @(negedge clk);
    endtask
    task automatic drain;
        blocked=0;
        wait(received==expected);
        repeat(8) @(negedge clk);
        tb_check(dut.used_bytes==0 && dut.used_frames==0 && !out_valid,"complete drain releases byte/descriptor space");
    endtask
    initial begin
        repeat(10) @(negedge clk); reset=0;
        send(100,1,0,1); send(100,2,0,1); send(100,3,0,0);
        tb_check(n_overflow==1 && dut.used_bytes==200,"mid-frame byte exhaustion rolls back all allocated bytes");
        send(50,4,1,0);
        tb_check(n_error==1 && dut.used_bytes==200,"CRC error rollback preserves earlier frames");
        drain();
        for(integer i=0;i<64;i++) begin send(1+(i*37)%199,10+i,0,1); drain(); end
        blocked=1;
        for(integer i=0;i<8;i++) send(1,100+i,0,1);
        send(1,120,0,0);
        tb_check(n_overflow==2 && dut.used_bytes==8,"descriptor full independent of byte capacity");
        drain();
        send(201,130,0,0);
        tb_check(n_length==1 && dut.used_bytes==0,"giant rollback without leaking byte space");
        send(60,131,1,0); tb_check(n_error==2,"bad frame never commits");
        send(200,140,0,1); drain();
        // 复位清指针，不清 RAM；清除未完成帧后仍能使用原空间。
        @(negedge clk); in_valid=1; in_last=0;
        repeat(20) @(negedge clk);
        reset=1; in_valid=0;
        repeat(10) @(negedge clk); reset=0;
        send(60,150,0,1); drain();
        tb_check(n_good==expected,"every expected frame committed exactly once");
        tb_finish("tb_afdx_rx_frame_fifo"); $finish;
    end
    initial begin #1000000; $fatal(1,"packed RX FIFO watchdog"); end
endmodule
