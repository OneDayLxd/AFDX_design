/*
 * ================================================================
 * File Name    : tb_afdx_mac_tx_pipeline.sv
 * Module Name  : tb_afdx_mac_tx_pipeline
 *
 * Function:
 *   连续 125MHz、真实整帧 MAC 反馈下验证多帧在途、元数据回绕、BAG 及复位。
 *   混合短帧/长帧，检查线路载荷/FCS/IFG，不采用人工固定反馈延迟。
 *
 * Sub Module:
 *   afdx_mac_tx
 *   afdx_gmii_mac
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_mac_tx_pipeline;
    `include "tb_common.svh"
    reg clk=0, rst=1;
    always #4 clk=~clk;
    reg [7:0] in_data=0;
    reg in_valid=0, in_last=0, in_tag=0;
    reg [6:0] in_vl=0;
    reg [19:0] in_bag=0;
    wire in_ready, stream_valid, stream_last, stream_ready, sof, done;
    wire [7:0] stream_data, wire_data;
    wire wire_valid, wire_error, stat_sof, stat_tag, stat_frame, tx_busy;
    wire [6:0] stat_vl;
    wire [127:0] bag_open;
    afdx_mac_tx #(
        .EXTERNAL_MAC (1),
        .N_VL         (128),
        .VLW          (7),
        .CLK_HZ       (125_000_000)
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
        .tx_data      (stream_data),
        .tx_valid     (stream_valid),
        .tx_slot      (stream_ready),
        .tx_frame     (),
        .tx_busy      (tx_busy),
        .stat_frame   (stat_frame),
        .bag_open     (bag_open),
        .stat_sof     (stat_sof),
        .stat_sof_vl  (stat_vl),
        .stat_sof_tag (stat_tag),
        .tx_last      (stream_last),
        .wire_sof     (sof),
        .wire_done    (done),
        .bag_wait     ()
    );
    afdx_gmii_mac mac (
        .clk               (clk),
        .reset             (rst),
        .ts_us             (32'd0),
        .tx_data           (stream_data),
        .tx_valid          (stream_valid),
        .tx_last           (stream_last),
        .tx_ready          (stream_ready),
        .tx_sof            (sof),
        .tx_done           (done),
        .rx_data           (),
        .rx_valid          (),
        .rx_last           (),
        .rx_ts             (),
        .rx_ready          (1'b1),
        .rx_good_count     (),
        .rx_error_count    (),
        .rx_length_count   (),
        .rx_overflow_count (),
        .eth_rx_er         (1'b0),
        .eth_rx_dv         (1'b0),
        .eth_rxd           (8'd0),
        .eth_tx_er         (wire_error),
        .eth_tx_en         (wire_valid),
        .eth_txd           (wire_data)
    );
    integer offered=0, committed=0, reported=0, completed=0, wire_id=0;
    integer lengths[0:511], vls[0:511], bags[0:511];
    time last_sof[0:127];
    integer last_bag[0:127];
    integer pos=0, idle=32, peak_metadata=0;
    bit active=0, overlap_seen=0;
    reg [31:0] crc=32'hffffffff;
    function automatic byte pattern(input integer id, offset);
        return byte'(id*19+offset*7);
    endfunction
    always @(posedge clk) if(!rst) begin
        if(dut.meta_count>peak_metadata) peak_metadata=dut.meta_count;
        tb_check(dut.meta_count<=8,"metadata FIFO bounded");
        if(sof) tb_check(dut.meta_count!=0,"wire SOF has queued metadata");
        if(wire_valid) tb_check(tx_busy,"busy includes active wire transmission");
        if(stream_valid && stream_ready && stream_last) begin
            committed++;
            if(committed>completed+1) overlap_seen=1;
        end
        if(sof) begin
            integer v;
            v=vls[reported];
            if(last_sof[v]!=0) tb_check($time-last_sof[v]>=last_bag[v]*1000,"actual SOF BAG");
            last_sof[v]=$time; last_bag[v]=bags[reported];
        end
        if(stat_sof) begin
            tb_check(stat_vl==vls[reported] && stat_tag==(reported%2),"queued SOF VL/tag correlation");
            reported++;
        end
        if(stat_frame) completed++;
        if(wire_valid) begin
            if(!active) begin
                tb_check(idle>=12,"minimum IFG"); active=1; pos=0; crc=32'hffffffff;
            end
            tb_check(!wire_error,"no wire underflow");
            if(pos<7) tb_check(wire_data==8'h55,"preamble");
            else if(pos==7) tb_check(wire_data==8'hd5,"SFD");
            else begin
                crc=tb_crc32_byte(crc,wire_data);
                if(pos-8<lengths[wire_id])
                    tb_check(wire_data===pattern(wire_id,pos-8),"queued wire payload order");
                else if(pos-8<60) tb_check(wire_data==0,"short frame padding");
            end
            pos++; idle=0;
        end else begin
            if(active) begin
                tb_check(pos==8+((lengths[wire_id]<60)?60:lengths[wire_id])+4,"wire frame length");
                tb_check(crc==32'hdebb20e3,"wire FCS"); wire_id++; active=0;
            end
            idle++;
        end
    end
    task automatic push(input integer vl, length, bag);
        integer id;
        id=offered; vls[id]=vl; lengths[id]=length; bags[id]=bag;
        for(integer k=0;k<length;k++) begin
            @(negedge clk);
            in_valid=1; in_data=pattern(id,k); in_last=k==length-1;
            in_vl=vl; in_bag=bag; in_tag=id%2;
            @(posedge clk); while(!in_ready) @(posedge clk);
        end
        @(negedge clk); in_valid=0; in_last=0; offered++;
    endtask
    task automatic drain;
        wait(completed==offered);
        repeat(30) @(negedge clk);
        tb_check(reported==offered && wire_id==offered,"all queued wire events exactly once");
    endtask
    initial begin
        for(integer i=0;i<128;i++) begin last_sof[i]=0; last_bag[i]=0; end
        repeat(12) @(negedge clk); rst=0;
        for(integer i=0;i<256;i++)
            push(i%128,(i%4==0)?1514:(i%4==1)?1:(i%4==2)?60:257,20*(1<<(i%3)));
        drain();
        tb_check(overlap_seen && peak_metadata>1,"assembly overlaps previous wire transmission");
        // 下一次同 VL 的短 BAG 不得提前覆盖上一帧尚未反馈的长 BAG。
        push(37,1514,80); push(37,1,20); push(37,60,40); drain();
        // 同时复位流接口和 MAC，撤销在途元数据与未完成载荷。
        @(negedge clk); in_valid=1; in_vl=91; in_data=8'hcc; in_last=0;
        repeat(8) @(negedge clk);
        rst=1; in_valid=0;
        repeat(12) @(negedge clk);
        offered=0; committed=0; reported=0; completed=0; wire_id=0; active=0; idle=32;
        for(integer i=0;i<128;i++) begin last_sof[i]=0; last_bag[i]=0; end
        rst=0; push(99,60,20); drain();
        $display("PIPELINE_RESOURCE peak_metadata=%0d overlap_seen=%0d",peak_metadata,overlap_seen);
        tb_finish("tb_afdx_mac_tx_pipeline"); $finish;
    end
    initial begin #10000000; $fatal(1,"pipeline watchdog"); end
endmodule
