// 实际扩展配置端到端仿真：全部发送 VL 双网环回，逐一接收全部订阅 VL。
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module tb_afdx_scale;
    // 批处理统一检查完成标志，避免提前 $fatal 被误判为成功。
    integer tb_errors = 0;
    bit tb_done = 0;
    `include "tb_scale_common.svh"
    localparam NT = `AFDX_N_TX_VL, TW = `AFDX_TX_VLW;
    localparam NR = `AFDX_N_RX_VL, RW = `AFDX_RX_VLW;
    localparam ROUNDS = 4;
    reg clk = 0, rst = 1;
    always #4 clk = ~clk;
    reg [TW-1:0] tx_msg_vl = 0;
    reg tx_msg_valid = 0, tx_msg_last = 0;
    reg [7:0] tx_msg_data = 0;
    wire tx_msg_ready;
    wire [RW-1:0] rx_msg_vl;
    wire rx_msg_valid, rx_msg_last, rx_msg_net;
    wire [7:0] rx_msg_data, rx_msg_sn;
    wire [10:0] rx_msg_len;
    wire [7:0] a_data, b_data;
    wire a_valid, b_valid, a_frame, b_frame;
    reg [3:0] pace = 0;
    always @(posedge clk) pace <= pace == 7 ? 0 : pace+1;
    wire slot = pace == 0;
    reg loop_en = 1;
    reg [7:0] inject_data = 0;
    reg inject_valid = 0, inject_frame = 0;
    wire [31:0] tx_count, framed, rx_count, accepted, rm_drop, ic_drop, full_drop;
    wire [31:0] fcs_err, hdr_err, len_err, csum_err, mac_ovf, unknown, lmax_drop;
    wire [8:0] tx_used, rx_used, tx_peak, rx_peak, tx_msgs_peak, rx_msgs_peak;
    localparam TX_VLW_TIE = TW;
    afdx_es_core #(
        .CLK_HZ  (125_000_000),
        .N_TX_VL (NT),
        .TX_VLW  (TW),
        .N_RX_VL (NR),
        .RX_VLW  (RW)
    ) dut (
        .cnt_rx_mac_error          (),
        .mgmt_window               (),
        .mac_a_tx_last             (),
        .mac_a_wire_sof            (1'b0),
        .mac_a_wire_done           (1'b0),
        .mac_a_rx_last             (1'b0),
        .mac_a_rx_ready            (),
        .mac_a_rx_ts               (32'd0),
        .mac_a_rx_good_count       (32'd0),
        .mac_a_rx_error_count      (32'd0),
        .mac_a_rx_length_count     (32'd0),
        .mac_a_rx_overflow_count   (32'd0),
        .mac_b_tx_last             (),
        .mac_b_wire_sof            (1'b0),
        .mac_b_wire_done           (1'b0),
        .mac_b_rx_last             (1'b0),
        .mac_b_rx_ready            (),
        .mac_b_rx_ts               (32'd0),
        .mac_b_rx_good_count       (32'd0),
        .mac_b_rx_error_count      (32'd0),
        .mac_b_rx_length_count     (32'd0),
        .mac_b_rx_overflow_count   (32'd0),
        .clk(clk), .rst(rst), .tx_msg_vl(tx_msg_vl), .tx_msg_valid(tx_msg_valid), .tx_msg_data(tx_msg_data),
        .tx_msg_last(tx_msg_last), .tx_msg_ready(tx_msg_ready),
        .rx_msg_vl(rx_msg_vl), .rx_msg_valid(rx_msg_valid), .rx_msg_data(rx_msg_data),
        .rx_msg_last(rx_msg_last), .rx_msg_ready(1'b1), .rx_msg_len(rx_msg_len), .rx_msg_sn(rx_msg_sn), .rx_msg_net(rx_msg_net),
        .mac_a_tx_data(a_data), .mac_a_tx_valid(a_valid), .mac_a_tx_slot(slot), .mac_a_tx_frame(a_frame), .mac_a_tx_busy(),
        .mac_a_rx_data(loop_en ? a_data : inject_data), .mac_a_rx_valid(loop_en ? (a_valid && slot) : inject_valid),
        .mac_a_rx_frame(loop_en ? a_frame : inject_frame),
        .mac_b_tx_data(b_data), .mac_b_tx_valid(b_valid), .mac_b_tx_slot(slot), .mac_b_tx_frame(b_frame), .mac_b_tx_busy(),
        .mac_b_rx_data(b_data), .mac_b_rx_valid(loop_en && b_valid && slot), .mac_b_rx_frame(loop_en && b_frame),
        .rtc_load(1'b0), .rtc_load_val(32'd0), .rtc_us(),
        `include "tb_core_mgmt_tie.svh"
        .cnt_tx_msgs(tx_count), .cnt_tx_frames(framed), .cnt_tx_lmax_drop(lmax_drop),
        .cnt_rx_delivered(rx_count), .cnt_rx_accepted(accepted), .cnt_rx_rm_drop(rm_drop), .cnt_rx_ic_drop(ic_drop),
        .cnt_rx_full_drop(full_drop), .cnt_rx_fcs_err(fcs_err), .cnt_rx_hdr_err(hdr_err), .cnt_rx_len_err(len_err),
        .cnt_rx_csum_err(csum_err), .cnt_rx_mac_ovf(mac_ovf), .cnt_rx_vl_unknown(unknown),
        .tx_buffer_used(tx_used), .rx_buffer_used(rx_used), .tx_buffer_peak(tx_peak), .rx_buffer_peak(rx_peak),
        .tx_buffer_messages_peak(tx_msgs_peak), .rx_buffer_messages_peak(rx_msgs_peak));

    int phase = 0, delivered_tx = 0, delivered_rx = 0, pos = 0;
    int next_round[0:NT-1];
    realtime offered[0:NT-1][0:ROUNDS-1], max_latency = 0;
    int tx_stall_cycles = 0;
    always @(posedge clk) if (tx_msg_valid && !tx_msg_ready) tx_stall_cycles++;
    always @(posedge clk) if (rx_msg_valid) begin
        int v, expected_byte, s;
        v = rx_msg_vl;
        tb_check(rx_msg_len == 17, "scaled payload length");
        tb_check(rx_msg_last == (pos == 16), "scaled payload last");
        if (phase == 0) begin
            if (v >= NT) $fatal(1, "TX loopback delivered wrong VL index");
            s = next_round[v];
            expected_byte = pos == 0 ? v : (pos == 1 ? s : ((v + 17*s + pos) & 255));
            tb_check(rx_msg_data == expected_byte, "scaled TX loopback payload");
            tb_check(rx_msg_sn == s+1, "scaled TX sequence");
            if (rx_msg_last) begin
                realtime latency; latency = $realtime - offered[v][s];
                if (latency > max_latency) max_latency = latency;
                next_round[v]++; delivered_tx++;
            end
        end else begin
            v = delivered_rx;
            expected_byte = pos == 0 ? (v >> 8) : (pos == 1 ? (v & 255) : ((v + pos) & 255));
            tb_check(rx_msg_vl == v, "all RX VL indices delivered in injection order");
            tb_check(rx_msg_data == expected_byte, "scaled RX payload including high VL bits");
            tb_check(rx_msg_sn == (v < NT ? ROUNDS+1 : 1), "RX sequence state independent per VL");
            if (rx_msg_last) delivered_rx++;
        end
        pos = rx_msg_last ? 0 : pos+1;
    end

    byte unsigned frame_buf[0:1][0:1599];
    int frame_pos[0:1], wire_frames[0:1], next_sn[0:1][0:NT-1];
    bit on_wire[0:1];
    reg [31:0] crc[0:1];
    realtime start_t[0:1], last_t[0:1][0:NT-1];
    int bag_errors = 0;
    task automatic monitor(input int net, input [7:0] data, input bit valid, input bit envelope);
        if (valid) begin
            if (!on_wire[net]) begin on_wire[net] = 1; frame_pos[net] = 0; crc[net] = 32'hffffffff; start_t[net] = $realtime; end
            frame_buf[net][frame_pos[net]] = data;
            frame_pos[net]++; crc[net] = crc32_byte(crc[net], data);
        end else if (on_wire[net] && !envelope) begin
            int v; v = {frame_buf[net][4], frame_buf[net][5]} - 16'h1000;
            if (v < 0 || v >= NT) $fatal(1, "scaled TX VLID out of range");
            tb_check(frame_pos[net] == 64 && crc[net] == 32'hdebb20e3, "scaled TX frame length/FCS");
            tb_check(frame_buf[net][59] == next_sn[net][v]+1, "per-network per-VL sequence");
            if (next_sn[net][v] > 0 && start_t[net]-last_t[net][v] < 1_000_000) bag_errors++;
            last_t[net][v] = start_t[net]; next_sn[net][v]++; wire_frames[net]++; on_wire[net] = 0;
        end
    endtask
    always @(posedge clk) if (slot && !rst) begin
        monitor(0, a_data, a_valid, a_frame); monitor(1, b_data, b_valid, b_frame);
    end

    task automatic submit(input int v, input int round_no);
        for (int i = 0; i < 17; i++) begin
            @(negedge clk); tx_msg_vl = v; tx_msg_valid = 1; tx_msg_last = i == 16;
            tx_msg_data = i == 0 ? v : (i == 1 ? round_no : ((v+17*round_no+i) & 255));
            @(posedge clk);
            if (i == 0) offered[v][round_no] = $realtime;
            while (!tx_msg_ready) @(posedge clk);
        end
        @(negedge clk); tx_msg_valid = 0; tx_msg_last = 0;
    endtask
    task automatic inject(input int v, input bit is_unknown);
        int vlid; vlid = is_unknown ? 16'hffff : 16'h1000 + v;
        for (int i = 0; i < 17; i++) ref_pld[i] = i == 0 ? (v >> 8) : (i == 1 ? (v & 255) : ((v+i) & 255));
        ref_build_frame(vlid, v < NT ? ROUNDS+1 : 1, 1, 1, 1, 1, 1000, 2000, 17, 1);
        @(negedge clk); inject_frame = 1;
        for (int i = 0; i < ref_flen; i++) begin
            while (!slot) @(negedge clk);
            inject_data = ref_frm[i]; inject_valid = 1;
            @(negedge clk); inject_valid = 0;
        end
        inject_frame = 0;
        repeat (12*8) @(negedge clk);
    endtask
    initial begin
        for (int v = 0; v < NT; v++) begin next_round[v] = 0; for (int n = 0; n < 2; n++) begin next_sn[n][v] = 0; last_t[n][v] = 0; end end
        for (int n = 0; n < 2; n++) begin frame_pos[n] = 0; wire_frames[n] = 0; on_wire[n] = 0; end
        repeat (6) @(negedge clk); rst = 0;
        while (dut.rxq_init_busy) @(negedge clk);
        $display("---- SCALE: %0d TX VLs, %0d RX VLs ----", NT, NR);
        $fflush();
        for (int r = 0; r < ROUNDS; r++) for (int v = 0; v < NT; v++) submit(v, r);
        // 当前 Icarus 对 wait 直接引用二态 int 数组会崩溃；按时钟采样同一条件。
        while (delivered_tx != NT*ROUNDS || wire_frames[0] != NT*ROUNDS || wire_frames[1] != NT*ROUNDS)
            @(negedge clk);
        #20_000;
        tb_check(tx_count == NT*ROUNDS && framed == tx_count, "every scaled TX message framed");
        tb_check(rm_drop == NT*ROUNDS && accepted == NT*ROUNDS, "dual copies delivered once");
        tb_check(bag_errors == 0, "BAG at wire for every scaled TX VL");
        // Four messages per VL must respect three 1 ms BAG intervals.
        tb_check(max_latency < 4_000_000, "scaled four-message burst latency < 4ms");
        for (int v = 0; v < NT; v++) tb_check(next_round[v] == ROUNDS, "all TX VLs exercised");
        $display("---- SCALE: all TX VLs passed; injecting every RX subscription ----");
        $fflush();
        @(negedge clk); loop_en = 0; phase = 1;
        for (int v = 0; v < NR; v++) begin
            inject(v, 0);
            if ((v+1)%512 == 0) begin
                $display("---- SCALE: injected %0d/%0d RX subscriptions ----",v+1,NR);
                $fflush();
            end
        end
        while (delivered_rx != NR) @(negedge clk);
        inject(0, 1);
        #20_000;
        tb_check(unknown == 1 && delivered_rx == NR, "unknown VL rejected after highest RX index");
        tb_check(rx_count == NT*ROUNDS+NR && accepted == rx_count, "scaled receive conservation");
        tb_check(fcs_err == 0 && hdr_err == 0 && len_err == 0 && csum_err == 0 && mac_ovf == 0 && full_drop == 0 && ic_drop == 0 && lmax_drop == 0, "no errors at admitted load");
        tb_check(tx_used == 0 && rx_used == 0, "scaled pools drain without leaks");
        tb_check((1 << dut.u_txq.AW) == 65536 && (1 << dut.u_rxq.AW) == 65536,
                 "payload RAM stays 64KiB per direction");
        $display("METRIC scaled_tx_vls=%0d", NT);
        $display("METRIC scaled_rx_vls=%0d", NR);
        $display("METRIC scaled_delivered_messages=%0d", rx_count);
        $display("METRIC scaled_app_latency_max_us=%.3f", max_latency/1000.0);
        $display("METRIC scaled_tx_admission_stall_cycles=%0d", tx_stall_cycles);
        $display("METRIC scaled_tx_pool_peak_blocks=%0d", tx_peak);
        $display("METRIC scaled_rx_pool_peak_blocks=%0d", rx_peak);
        $display("METRIC scaled_tx_pool_peak_messages=%0d", tx_msgs_peak);
        $display("METRIC scaled_rx_pool_peak_messages=%0d", rx_msgs_peak);
        $display("METRIC scaled_bag_violations=%0d", bag_errors);
        tb_errors = tb_err_cnt;
        tb_done = (tb_err_cnt == 0);
        $display("tb_afdx_scale: %0d checks, %0d errors -> %s", tb_chk_cnt, tb_err_cnt, tb_err_cnt == 0 ? "PASS" : "FAIL");
        if (tb_err_cnt) $fatal(1, "scale regression failed");
        $finish;
    end
    initial begin
        #60_000_000;
        $fatal(1, "scale timeout: phase=%0d delivered_tx=%0d/%0d wire_a=%0d wire_b=%0d tx_count=%0d framed=%0d rx_count=%0d accepted=%0d rm_drop=%0d ic_drop=%0d full_drop=%0d tx_used=%0d rx_used=%0d fb_state=%0d fb_vl=%0d q_info=%b q_start=%b q_valid=%b grant=%b/%b tx_nonempty=%h pending_a=%h mac_a_state=%0d",
               phase, delivered_tx, NT*ROUNDS, wire_frames[0], wire_frames[1], tx_count,
               framed, rx_count, accepted, rm_drop, ic_drop, full_drop, tx_used, rx_used,
               dut.u_fb.state, dut.u_fb.vl_l, dut.txq_rd_info_valid, dut.txq_rd_start,
               dut.txq_rd_valid, dut.grant_valid, dut.grant_ack, dut.txq_nonempty,
               dut.tx_pending_a, dut.u_mac_tx_a.state);
    end
endmodule
