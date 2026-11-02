/*
 * ================================================================
 * File Name    : tb_afdx_tx_prepare.sv
 * Module Name  : tb_afdx_tx_prepare
 * Function:
 *   真实 MAC 链路下验证按帧长预备不缩短 BAG；交替长短帧、输入空洞及背压。
 * Sub Module:
 *   afdx_mac_tx
 *   afdx_gmii_mac
 * ================================================================
 */
`timescale 1ns/1ps
module tb_afdx_tx_prepare;
    `include "tb_common.svh"
    reg clk=0,rst=1,iv=0,il=0;
    reg [7:0] id=0;
    reg [10:0] length=60;
    wire ir,mv,ml,mr,sof,done,ethv;
    wire [7:0] md,ethd;
    always #4 clk=~clk;
    afdx_mac_tx #(
        .EARLY_PREPARE (1),
        .EXTERNAL_MAC  (1),
        .N_VL          (2),
        .VLW           (1),
        .CLK_HZ        (125000000)
    ) u_tx (
        .clk           (clk),
        .rst           (rst),
        .in_data       (id),
        .in_valid      (iv),
        .in_last       (il),
        .in_vl         (1'b0),
        .in_bag_us     (20'd1000),
        .in_tag        (1'b0),
        .in_frame_len  (length),
        .in_ready      (ir),
        .tx_data       (md),
        .tx_valid      (mv),
        .tx_slot       (mr),
        .tx_frame      (),
        .tx_busy       (),
        .stat_frame    (),
        .bag_open      (),
        .build_open    (),
        .stat_sof      (),
        .stat_sof_vl   (),
        .stat_sof_tag  (),
        .tx_last       (ml),
        .wire_sof      (sof),
        .wire_done     (done),
        .bag_wait      ()
    );
    afdx_gmii_mac u_mac (
        .clk               (clk),
        .reset             (rst),
        .ts_us             (32'd0),
        .tx_data           (md),
        .tx_valid          (mv),
        .tx_last           (ml),
        .tx_ready          (mr),
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
        .eth_tx_er         (),
        .eth_tx_en         (ethv),
        .eth_txd           (ethd)
    );
    int lengths[8]='{60,1514,61,1001,1470,65,512,60};
    int frames=0,p=0,launches=0;
    time prior_sof=0,launch[8];
    reg [31:0] crc;
    bit active=0;
    function automatic byte pattern(int n,j);return byte'(n*23+j*7);endfunction
    always @(posedge clk)if(!rst)begin
        if(u_tx.data_take && u_tx.first_byte)begin launch[launches]=$time;launches++;end
        if(sof)begin
            if(prior_sof)tb_check($time-prior_sof>=1000000,"actual GMII BAG never shortened across varying frame sizes");
            prior_sof=$time;
            $display("prepare frame=%0d N=%0d first_to_wire_cycles=%0d",frames,lengths[frames],($time-launch[frames])/8);
        end
        if(ethv)begin
            if(!active)begin active=1;p=0;crc='1;end
            if(p>=8)begin
                crc=tb_crc32_byte(crc,ethd);
                if(p<8+lengths[frames])tb_check(ethd===pattern(frames,p-8),"wire payload exact under prefetch");
            end
            p++;
        end else if(active)begin
            tb_check(p==lengths[frames]+12,"wire length unchanged");
            tb_check(crc==32'hdebb20e3,"wire CRC unchanged");
            frames++;active=0;
        end
    end
    initial begin
        repeat(40)@(negedge clk);rst=0;repeat(40)@(negedge clk);
        for(int n=0;n<8;n++)begin
            for(int j=0;j<lengths[n];j++)begin
                @(negedge clk);iv=1;il=j==lengths[n]-1;id=pattern(n,j);length=11'(lengths[n]);
                do @(posedge clk);while(!ir);
                if(n>=4 && j%31==9)begin @(negedge clk);iv=0;repeat(2)@(negedge clk);end
            end
            @(negedge clk);iv=0;il=0;
        end
        wait(frames==8);repeat(20)@(negedge clk);
        tb_check(launches==8,"one launch per frame");
        tb_finish("tb_afdx_tx_prepare");$finish;
    end
    initial begin #12000000;$fatal(1,"prepare watchdog");end
endmodule
