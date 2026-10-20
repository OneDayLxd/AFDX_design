`timescale 1ns/1ps
module tb_afdx_mac_rx_pipeline;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    always #5 clk=~clk;
    reg [7:0] data=0;
    reg valid=0, frame=0, ready=0, drain=0;
    wire [7:0] out_data;
    wire out_valid, out_last, available, good, bad, length_error, overflow;
    afdx_mac_rx #(.FIFO_AW(8)) dut (
        .clk(clk), .rst(rst), .rx_data(data), .rx_valid(valid), .rx_frame(frame),
        .out_data(out_data), .out_valid(out_valid), .out_last(out_last), .out_ready(ready),
        .frame_avail(available), .stat_frame_ok(good), .stat_fcs_err(bad),
        .stat_len_err(length_error), .stat_ovf(overflow)
    );
    integer received=0, good_count=0, bad_count=0, cycles=0, last_hold=0;
    reg stalled=0;
    reg [8:0] previous;
    always @(negedge clk) begin
        cycles=cycles+1;
        ready=drain && cycles%7<4;
        if(out_valid && out_last && last_hold<5) begin ready=0; last_hold=last_hold+1; end
    end
    always @(posedge clk) if(!rst) begin
        if(stalled) tb_check(out_valid && {out_last,out_data}==previous, "data and last stable under backpressure");
        stalled=out_valid && !ready; previous={out_last,out_data};
        if(good) good_count=good_count+1;
        if(bad) bad_count=bad_count+1;
        tb_check(!length_error && !overflow, "no length or FIFO overflow error");
        if(out_valid && ready) begin
            tb_check(out_data==((received/60*37+received%60)&255), "ordered payload, no FCS or corrupt frame bytes");
            tb_check(out_last==(received%60==59), "last marks exactly the final payload byte");
            received=received+1;
        end
    end
    task automatic send_frame(input integer seed, input bit corrupt);
        integer i;
        reg [31:0] crc, fcs;
        begin
            crc=32'hffffffff;
            for(i=0;i<60;i=i+1) begin
                @(negedge clk); frame=1; valid=1; data=seed+i;
                crc=tb_crc32_byte(crc,data);
            end
            fcs=~crc;
            if(corrupt) fcs=fcs^32'd1;
            for(i=0;i<4;i=i+1) begin @(negedge clk); data=fcs>>(i*8); end
            @(negedge clk); valid=0; frame=0;
            repeat(6) @(negedge clk);
        end
    endtask
    initial begin
        repeat(4) @(negedge clk); rst=0;
        send_frame(0,0); send_frame(37,0);
        tb_check(available && received==0, "committed frames wait while output is blocked");
        drain=1;
        send_frame(200,1); send_frame(74,0);
        wait(received==180);
        repeat(10) @(negedge clk);
        tb_check(!available && !out_valid, "frame count retires at external final-byte handshake");
        tb_check(good_count==3 && bad_count==1, "good and bad FCS counts");
        tb_check(last_hold==5, "last-byte backpressure exercised");
        tb_finish("tb_afdx_mac_rx_pipeline"); $finish;
    end
    initial begin #100000; $fatal(1,"MAC RX pipeline timeout"); end
endmodule
