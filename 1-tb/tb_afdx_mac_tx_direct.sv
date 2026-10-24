/*
 * ================================================================
 * File Name    : tb_afdx_mac_tx_direct.sv
 * Module Name  : tb_afdx_mac_tx_direct
 * Function:
 *   外部 MAC 无重复帧缓存分支：128 VL、随机输入/输出背压、
 *   延迟线路反馈、BAG、标签关联、逐字节顺序及中途复位。
 * Sub Module:
 *   afdx_mac_tx
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_mac_tx_direct;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    always #5 clk=~clk;
    reg [7:0] in_data=0;
    reg in_valid=0, in_last=0, in_tag=0;
    reg [6:0] in_vl=0;
    reg [19:0] in_bag=0;
    wire in_ready, tx_valid, tx_last, stat_sof, stat_tag, stat_frame;
    wire [7:0] tx_data;
    wire [6:0] stat_vl;
    wire [127:0] bag_open;
    reg tx_slot=1, wire_sof=0, wire_done=0;
    afdx_mac_tx #(
        .EXTERNAL_MAC (1),
        .N_VL         (128),
        .VLW          (7),
        .CLK_HZ       (100_000_000)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .in_data      (in_data),
        .in_valid     (in_valid),
        .in_last      (in_last),
        .in_vl        (in_vl),
        .in_bag_us    (in_bag),
        .in_tag       (in_tag),
        .in_ready     (in_ready),
        .tx_data      (tx_data),
        .tx_valid     (tx_valid),
        .tx_slot      (tx_slot),
        .tx_last      (tx_last),
        .tx_frame     (),
        .tx_busy      (),
        .stat_frame   (stat_frame),
        .stat_sof     (stat_sof),
        .stat_sof_vl  (stat_vl),
        .stat_sof_tag (stat_tag),
        .bag_open     (bag_open),
        .wire_sof     (wire_sof),
        .wire_done    (wire_done),
        .bag_wait     ()
    );
    integer offered=0, received=0, completed=0, pos=0, pending=0, feedback_id=0;
    integer expected_vl[0:511], expected_len[0:511], expected_bag[0:511];
    bit expected_tag[0:511];
    time previous_sof[0:127];
    time first_input_time, last_input_time, first_output_time;
    integer previous_bag[0:127];
    integer mode=0, cycles=0;
    bit held=0;
    reg [8:0] held_beat;
    function automatic [7:0] pattern(input integer id, offset);
        return 8'(id*13+offset*7);
    endfunction
    always @(negedge clk) begin
        cycles++;
        case(mode)
            1: tx_slot=$urandom_range(0,1);
            2: tx_slot=(cycles%8)==0;
            default: tx_slot=1;
        endcase
    end
    always @(posedge clk) begin
        if(rst) begin
            pending=0; wire_sof<=0; wire_done<=0; held=0;
        end else begin
            wire_sof<=0; wire_done<=0;
            if(held) begin
                tb_check(tx_valid && {tx_last,tx_data}===held_beat,"held TX beat stable under backpressure");
            end
            held=tx_valid && !tx_slot;
            held_beat={tx_last,tx_data};
            if(pending>0) begin
                pending--;
                if(pending==12) wire_sof<=1;
                if(pending==2) wire_done<=1;
            end
            if(tx_valid && tx_slot) begin
                if(received==0 && pos==0) first_output_time=$time;
                tb_check(received<=offered,"no unsolicited frame");
                tb_check(tx_data===pattern(received,pos),"ordered payload bytes");
                tb_check(tx_last===(pos==expected_len[received]-1),"last belongs to its accepted byte");
                pos++;
                if(tx_last) begin
                    tb_check(pending==0,"one frame in flight");
                    pending=20; feedback_id=received; received++; pos=0;
                end
            end
            if(wire_sof) begin
                integer vl;
                vl=expected_vl[feedback_id];
                if(previous_sof[vl]!=0)
                    tb_check($time-previous_sof[vl]>=previous_bag[vl]*1000,"BAG from delayed actual wire SOF");
                previous_sof[vl]=$time;
                previous_bag[vl]=expected_bag[feedback_id];
            end
            if(stat_sof) begin
                tb_check(stat_vl==expected_vl[feedback_id] && stat_tag==expected_tag[feedback_id],
                    "feedback VL/tag remain attached to the original frame");
            end
            if(stat_frame) completed++;
        end
    end
    task automatic push_frame(input integer vl, length, bag);
        integer id;
        id=offered;
        expected_vl[id]=vl; expected_len[id]=length; expected_bag[id]=bag; expected_tag[id]=id%2;
        for(integer j=0;j<length;j++) begin
            @(negedge clk);
            in_vl=vl; in_data=pattern(id,j); in_last=j==length-1; in_bag=bag;
            in_tag=id%2; in_valid=1;
            @(posedge clk);
            while(!in_ready) @(posedge clk);
            if(id==0 && j==0) first_input_time=$time;
            if(id==0 && j==length-1) last_input_time=$time;
            if(mode==1 && $urandom_range(0,4)==0) begin
                @(negedge clk); in_valid=0;
            end
        end
        @(negedge clk); in_valid=0; in_last=0;
        offered++;
    endtask
    task automatic wait_complete(input integer count);
        integer timeout=0;
        while(completed<count && timeout<1000000) begin @(negedge clk); timeout++; end
        tb_check(completed==count && received==count,"all frames completed once");
        if(completed!=count) $fatal(1,"stalled %0d/%0d",completed,count);
    endtask
    task automatic reset_all;
        @(negedge clk); rst=1; in_valid=0; in_last=0;
        repeat(8) @(negedge clk);
        offered=0; received=0; completed=0; pos=0;
        for(integer i=0;i<128;i++) begin previous_sof[i]=0; previous_bag[i]=0; end
        rst=0;
        repeat(40) @(negedge clk);
    endtask
    initial begin
        reset_all();
        for(integer pass=0;pass<3;pass++) begin
            mode=pass;
            for(integer vl=0;vl<128;vl++)
                push_frame(vl,(vl==0)?1514:(vl==1)?257:60+vl,20*(1<<(vl%3)));
        end
        wait_complete(384);
        tb_check(first_output_time<last_input_time,"external prefix forwarded before the complete frame is buffered");
        $display("DIRECT_METRIC first_output_latency_ns=%0d input_frame_time_ns=%0d",
            first_output_time-first_input_time,last_input_time-first_input_time);
        for(integer vl=0;vl<128;vl++) tb_check(previous_sof[vl]!=0,"all 128 VLs exercised");
        push_frame(37,60,80);
        push_frame(37,1514,80);
        wait_complete(386);
        // Abort an incomplete frame; reset must clear both its prefix and metadata.
        expected_vl[offered]=90; expected_len[offered]=1514;
        expected_bag[offered]=20; expected_tag[offered]=offered%2;
        for(integer j=0;j<10;j++) begin
            @(negedge clk); in_valid=1; in_last=0; in_vl=90; in_bag=20;
            in_tag=offered%2; in_data=pattern(offered,j);
            @(posedge clk); while(!in_ready) @(posedge clk);
        end
        reset_all();
        mode=0;
        push_frame(99,60,20);
        wait_complete(1);
        tb_finish("tb_afdx_mac_tx_direct"); $finish;
    end
    initial begin #50000000; $fatal(1,"direct TX watchdog"); end
endmodule
