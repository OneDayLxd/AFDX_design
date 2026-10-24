/*
 * ================================================================
 * File Name    : tb_afdx_async_frame_fifo.sv
 * Module Name  : tb_afdx_async_frame_fifo
 * Function:
 *   异步整帧缓存的背压、错误回退、溢出、超长、指针回绕与复位自检。
 * Sub Module:
 *   afdx_async_frame_fifo
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_async_frame_fifo;
    `include "tb_common.svh"
    reg wc=0, rc=0, rst=1;
    always #4 wc=~wc;
    initial begin #1; forever #5 rc=~rc; end
    reg [7:0] data=0;
    reg valid=0, last=0, error_bit=0, ready=0;
    wire [7:0] dout;
    wire ov, ol, good, err, lenerr, overflow;
    wire [31:0] ots;
    integer ng=0, ne=0, nl=0, no=0, frames=0, pos=0;
    integer seeds[$], lens[$];
    afdx_async_frame_fifo #(
        .SLOT_LOG  (1),
        .DROP_FULL (1)
    ) dut (
        .wr_clk        (wc),
        .wr_rst        (rst),
        .in_data       (data),
        .in_valid      (valid),
        .in_last       (last),
        .in_error      (error_bit),
        .in_ts         (32'h12345678),
        .in_ready      (),
        .stat_good     (good),
        .stat_error    (err),
        .stat_length   (lenerr),
        .stat_overflow (overflow),
        .rd_clk        (rc),
        .rd_rst        (rst),
        .out_data      (dout),
        .out_valid     (ov),
        .out_last      (ol),
        .out_ts        (ots),
        .out_ready     (ready)
    );
    always @(posedge wc) if (!rst) begin
        if (good) ng++;
        if (err) ne++;
        if (lenerr) nl++;
        if (overflow) no++;
    end
    reg was_stalled=0;
    reg [40:0] held;
    always @(posedge rc) begin
        if (rst) begin was_stalled=0; pos=0; end
        else begin
            if (was_stalled) tb_check(ov && {ots,ol,dout}===held,"output stable under backpressure");
            was_stalled=ov && !ready;
            held={ots,ol,dout};
            if (ov && ready) begin
                if (lens.size()==0) $fatal(1,"unexpected committed frame");
                tb_check(dout===8'((seeds[0]+pos)&255),"payload order and data");
                tb_check(ots===32'h12345678,"timestamp survives CDC");
                tb_check(ol===(pos==lens[0]-1),"exact frame boundary");
                pos++;
                if (ol) begin void'(seeds.pop_front()); void'(lens.pop_front()); pos=0; frames++; end
            end
        end
    end
    task automatic push(input integer length, seed, bad_at, input bit expect_good);
        if (expect_good) begin seeds.push_back(seed); lens.push_back(length); end
        for (integer j=0;j<length;j++) begin
            @(negedge wc); valid=1; data=8'(seed+j); last=j==length-1; error_bit=j==bad_at;
        end
        @(negedge wc); valid=0; last=0; error_bit=0;
    endtask
    initial begin
        repeat(8) @(negedge wc); rst=0;
        push(60,1,-1,1); push(1514,2,-1,1);
        push(60,3,-1,0);
        repeat(4) @(negedge wc);
        tb_check(ng==2 && no==1,"full drops whole frame without overwriting committed data");
        ready=1;
        wait(frames==2); repeat(10) @(negedge wc);
        push(60,4,59,0); push(60,5,2,0); push(59,6,-1,0); push(1515,7,-1,0);
        repeat(10) @(negedge wc);
        tb_check(ne==2 && nl==2,"tail/early errors and length limits abort whole frames");
        for(integer k=0;k<20;k++) begin
            push(60+k,10+k,-1,1);
            wait(lens.size()==0);
            repeat(8) @(negedge wc);
        end
        ready=0; push(60,90,-1,1);
        repeat(10) @(negedge wc); rst=1;
        seeds.delete(); lens.delete();
        repeat(8) @(negedge wc); rst=0;
        repeat(10) @(negedge wc); ready=1;
        push(60,91,-1,1); wait(lens.size()==0);
        repeat(10) @(negedge wc);
        tb_check(!ov,"reset flushes outstanding frame");
        tb_finish("tb_afdx_async_frame_fifo"); $finish;
    end
    initial begin #500000; $fatal(1,"FIFO watchdog"); end
endmodule
