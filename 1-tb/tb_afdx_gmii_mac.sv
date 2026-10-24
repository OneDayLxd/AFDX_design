/*
 * ================================================================
 * File Name    : tb_afdx_gmii_mac.sv
 * Module Name  : tb_afdx_gmii_mac
 * Function:
 *   独立验证 MAC 适配器：逐字节载荷/填充/FCS/IFG、随机输入停顿、
 *   接收背压、CRC/错误指示/前导码/长短帧注错、缓存溢出及帧中复位。
 * Sub Module:
 *   afdx_gmii_mac
 * ================================================================
 */
`timescale 1ns/1ps
`default_nettype none
module tb_afdx_gmii_mac;
    `include "tb_common.svh"
    reg clk=0, reset=1;
    reg [31:0] timestamp_us=123, expected_timestamp=123;
    always #4 clk=~clk;
    reg [7:0] tx_data=0, inj_data=0;
    reg tx_valid=0, tx_last=0, inj_valid=0, inj_error=0;
    reg loopback=1, block_rx=0;
    wire tx_ready, tx_sof, tx_done, rx_valid, rx_last;
    wire [7:0] rx_data, eth_txd;
    wire [31:0] rx_ts, good, errors, lengths, overflows;
    wire eth_tx_en, eth_tx_er;
    integer cycle=0;
    always @(posedge clk) cycle<=cycle+1;
    wire rx_ready=!block_rx && cycle%8!=0;
    integer tx_len[0:127], tx_seed[0:127], rx_len[0:127], rx_seed[0:127], rx_original[0:127];
    integer ntx=0, nrx=0, tx_expected=0, rx_expected=0, rp=0, wp=0, idle=100;
    integer baseline, saved_errors, saved_lengths, saved_overflow;
    bit active=0, was_stalled=0;
    reg [7:0] held_data;
    reg held_last;
    reg [31:0] crc;
    afdx_gmii_mac #(
        .RX_SLOT_LOG (5),
        .RX_BYTE_LOG (12)
    ) dut (
        .clk               (clk),
        .reset             (reset),
        .ts_us             (timestamp_us),
        .tx_data           (tx_data),
        .tx_valid          (tx_valid),
        .tx_last           (tx_last),
        .tx_ready          (tx_ready),
        .tx_sof            (tx_sof),
        .tx_done           (tx_done),
        .rx_data           (rx_data),
        .rx_valid          (rx_valid),
        .rx_last           (rx_last),
        .rx_ts             (rx_ts),
        .rx_ready          (rx_ready),
        .rx_good_count     (good),
        .rx_error_count    (errors),
        .rx_length_count   (lengths),
        .rx_overflow_count (overflows),
        .eth_rx_er         (loopback ? 1'b0 : inj_error),
        .eth_rx_dv         (loopback ? eth_tx_en : inj_valid),
        .eth_rxd           (loopback ? eth_txd : inj_data),
        .eth_tx_er         (eth_tx_er),
        .eth_tx_en         (eth_tx_en),
        .eth_txd           (eth_txd)
    );
    function automatic byte pattern(input integer seed, pos);
        return byte'(seed+pos*7);
    endfunction
    task automatic expect_rx(input integer len, seed);
        rx_len[rx_expected]=len<60?60:len;
        rx_original[rx_expected]=len; rx_seed[rx_expected]=seed; rx_expected++;
    endtask
    task automatic send(input integer len, seed, input bit deliver=1);
        if(len<=1514) begin
            tx_len[tx_expected]=len; tx_seed[tx_expected]=seed; tx_expected++;
            if(deliver) expect_rx(len,seed);
        end
        for(integer j=0;j<len;j++) begin
            @(negedge clk);
            if(j%5==3) begin tx_valid=0; @(negedge clk); end
            tx_valid=1; tx_last=j==len-1; tx_data=pattern(seed,j);
            @(posedge clk); while(!tx_ready) @(posedge clk);
        end
        @(negedge clk); tx_valid=0; tx_last=0;
    endtask
    always @(negedge clk) begin
        if(reset) begin ntx=0; wp=0; active=0; idle=100; end
        else if(eth_tx_en) begin
            if(!active) begin
                active=1; wp=0; crc=32'hffffffff;
                tb_check(idle>=12,"GMII minimum IFG");
                tb_check(ntx<tx_expected,"no unexpected transmission");
            end
            if(wp<7) tb_check(eth_txd===8'h55,"seven-byte preamble");
            else if(wp==7) tb_check(eth_txd===8'hd5,"SFD");
            else begin
                crc=tb_crc32_byte(crc,eth_txd);
                if(wp<8+tx_len[ntx]) tb_check(eth_txd===pattern(tx_seed[ntx],wp-8),"TX payload identity");
                else if(wp<68) tb_check(eth_txd===8'd0,"short TX zero padding");
            end
            tb_check(!eth_tx_er,"no TX underflow"); wp++; idle=0;
        end else begin
            idle++;
            if(active) begin
                tb_check(wp==(tx_len[ntx]<60?60:tx_len[ntx])+12,"wire length includes FCS and preamble");
                tb_check(crc===32'hdebb20e3,"independent Ethernet FCS");
                active=0; ntx++;
            end
        end
    end
    always @(posedge clk) begin
        if(reset) begin nrx=0; rp=0; was_stalled=0; end
        else begin
            if(was_stalled) tb_check(rx_valid && rx_data===held_data && rx_last===held_last,"RX stable under backpressure");
            was_stalled=rx_valid && !rx_ready; held_data=rx_data; held_last=rx_last;
            if(rx_valid && rx_ready) begin
                tb_check(nrx<rx_expected,"only expected good frames delivered");
                if(rx_data!==(rp<rx_original[nrx]?pattern(rx_seed[nrx],rp):8'd0))
                    $fatal(1,"RX identity frame=%0d pos=%0d len=%0d got=%02h exp=%02h good=%0d err=%0d lenerr=%0d ovf=%0d",nrx,rp,rx_len[nrx],rx_data,pattern(rx_seed[nrx],rp),good,errors,lengths,overflows);
                tb_check(rx_data===(rp<rx_original[nrx]?pattern(rx_seed[nrx],rp):8'd0),"RX strips only FCS");
                tb_check(rx_last===(rp==rx_len[nrx]-1),"RX last boundary");
                tb_check(rx_ts==expected_timestamp,"SFD timestamp retained");
                rp++; if(rx_last) begin nrx++; rp=0; end
            end
        end
    end
    task automatic drain;
        integer timeout_cycles=0;
        while(nrx<rx_expected && timeout_cycles<100000) begin @(negedge clk); timeout_cycles++; end
        tb_check(nrx==rx_expected,"all accepted good frames delivered");
    endtask
    task automatic inject(input integer len, kind, gap_cycles=20, pre_bytes=7);
        reg [31:0] c;
        byte b;
        c=32'hffffffff;
        for(integer j=0;j<pre_bytes+1;j++) begin
            @(negedge clk); inj_valid=1; inj_data=j==pre_bytes?(kind==3?8'hd4:8'hd5):8'h55;
            inj_error=kind==5 && j==2;
            if(kind==4 && j==pre_bytes) timestamp_us=789;
        end
        for(integer j=0;j<len;j++) begin
            @(negedge clk); b=pattern(77,j); inj_data=b; inj_error=(kind==2 && j==20);
            if(kind==4) timestamp_us=790;
            c=tb_crc32_byte(c,b);
        end
        for(integer j=0;j<4;j++) begin
            @(negedge clk); inj_error=0; inj_data=byte'((~c)>>(8*j));
            if(kind==1 && j==0) inj_data=inj_data^1;
        end
        @(negedge clk); inj_valid=0; inj_error=0;
        repeat(gap_cycles) @(negedge clk);
    endtask
    initial begin
        repeat(8) @(negedge clk); reset=0;
        send(1,1); send(17,2); send(59,3); send(60,4); send(61,5);
        send(128,6); send(1514,7); drain();
        // 正确性阶段消费完再发送；超额输入的丢帧在下方单独精确检查。
        for(integer j=0;j<24;j++) begin send(60+(j*71)%600,20+j); drain(); end
        drain(); repeat(50) @(negedge clk);
        baseline=ntx; send(1515,88,0); repeat(2000) @(negedge clk);
        tb_check(ntx==baseline,"overlong TX discarded before wire");
        loopback=0; saved_errors=errors; saved_lengths=lengths; baseline=nrx;
        inject(60,1); inject(60,2); inject(60,3); inject(59,0); inject(1515,0);
        repeat(50) @(negedge clk);
        tb_check(nrx==baseline,"bad frames never reach application");
        tb_check(errors==saved_errors+3,"CRC/RX_ER/bad SFD error accounting");
        tb_check(lengths==saved_lengths+2,"runt and giant accounting");
        expect_rx(60,77); inject(60,0); drain();
        saved_errors=errors; baseline=nrx;
        // 内核接受五字节前导码，外层仍须执行端系统七字节策略。
        inject(60,0,12,5);
        inject(60,1,12);
        expect_rx(60,77); inject(60,0,12); drain();
        repeat(30) @(negedge clk);
        tb_check(nrx==baseline+1 && errors==saved_errors+2,"short preamble and adjacent CRC error discarded exactly once");
        saved_errors=errors; baseline=nrx;
        // 原始内核不报告前导码阶段 RX_ER，适配器必须补齐整帧丢弃。
        expect_rx(60,77); inject(60,5,12); inject(60,0,12); drain();
        tb_check(nrx==baseline+1 && errors==saved_errors+1,"preamble PHY error rejected and next frame recovered");
        // 极短坏帧可能仅有无效周期 LAST/USER，不能污染后续完整帧。
        saved_lengths=lengths; inject(0,0,12);
        expect_rx(60,77); inject(60,0,12); drain();
        tb_check(lengths==saved_lengths+1,"empty frame sidebands do not leak bytes");
        expected_timestamp=789;
        expect_rx(60,77); inject(60,4); drain();
        timestamp_us=123; expected_timestamp=123;
        loopback=1; block_rx=1; saved_overflow=overflows;
        // 使用 DUT 配置的实际深度，精确覆盖全部槽及额外两帧的整帧丢弃。
        for(integer j=0;j<(1<<dut.RX_SLOT_LOG)+2;j++)
            send(60,100+j,j<(1<<dut.RX_SLOT_LOG));
        wait(ntx==tx_expected); repeat(30) @(negedge clk);
        tb_check(overflows==saved_overflow+2,"RX full drops two complete frames");
        block_rx=0; drain();
        // 帧中复位必须丢弃未提交数据，且不清零整个 RAM 阵列。
        loopback=0;
        for(integer j=0;j<8;j++) begin @(negedge clk); inj_valid=1; inj_data=j==7?8'hd5:8'h55; end
        repeat(20) begin @(negedge clk); inj_data=8'h31; end
        reset=1; inj_valid=0; repeat(8) @(negedge clk); reset=0;
        tx_expected=0; rx_expected=0; loopback=1;
        repeat(40) @(negedge clk);
        tb_check(!rx_valid && !eth_tx_en && good==0 && errors==0,"reset clears partial frame and status");
        send(60,5); drain();
        tb_finish("tb_afdx_gmii_mac"); $finish;
    end
    initial begin #5000000; $fatal(1,"MAC unit watchdog"); end
endmodule
`default_nettype wire
