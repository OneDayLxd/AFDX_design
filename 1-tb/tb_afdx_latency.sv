// 历史单总线性能基线：专用低带宽配置的 afdx_system 经存储转发交换机模型自发自收，逐消息记录时间戳。
// 交换机模型：业务 VL 帧完整接收后按 FIFO 顺序以 100 Mbps（每 8 拍 1 字节、帧间 12 字节）送回 rx 口；
// 管理 VL（VLID MGMT_VLID）帧由管理站模型消费，不回环。
// 管理站（+MGMT_LOAD=1）：循环以 TFTP RRQ 读取配置对象并逐块 ACK（持续产生 516 字节 DATA 帧），
// 并每 SNMP_PERIOD_US 发送一次 14 个 VarBind 的 GET（响应约 508 字节）。
// 记录（+CSV=文件）：
//   R vl seq t_ns            应用首字节被接受（afdx_app_gen → 仲裁器）
//   S vl seq net t_ns len    业务帧在总线上的 SOF
//   M t_ns len dport         管理帧 SOF
//   D vl seq t_ns            接收应用最后一字节握手
// latency.csv 仅记录原始数据；本平台只检查数据面错误计数和交换机缓冲。
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module tb_afdx_latency;
    // 批处理统一检查完成标志，避免提前 $fatal 被误判为成功。
    integer tb_errors = 0;
    bit tb_done = 0;
    `include "tb_scale_common.svh"
    localparam [15:0] MGMT_VLID = 16'h1000;
    localparam RING = 1 << 20;

    reg clk = 1'b0;
    reg reset = 1'b1;
    always #5 clk = ~clk;

    wire [7:0] tx_data;
    wire tx_valid, tx_envelope;
    reg [7:0] rx_data = 8'h00;
    reg rx_valid = 1'b0, rx_envelope = 1'b0;

    afdx_system dut (
        .host_tx_vl                ('0),
        .host_tx_valid             (1'b0),
        .host_tx_data              ('0),
        .host_tx_last              (1'b0),
        .host_tx_ready             (),
        .host_rx_vl                (),
        .host_rx_valid             (),
        .host_rx_data              (),
        .host_rx_last              (),
        .host_rx_ready             (1'b0),
        .host_rx_len               (),
        .host_rx_sn                (),
        .host_rx_net               (),
        .host_rx_ts                (),
        .host_rx_src_ip            (),
        .host_rx_sport             (),
        .host_rx_dport             (),
        .ext_ts_us                 (),
        .ext_a_tx_data             (),
        .ext_a_tx_valid            (),
        .ext_a_tx_last             (),
        .ext_a_rx_ready            (),
        .ext_a_tx_ready            (1'b0),
        .ext_a_wire_sof            (1'b0),
        .ext_a_wire_done           (1'b0),
        .ext_a_rx_valid            (1'b0),
        .ext_a_rx_last             (1'b0),
        .ext_a_rx_data             (8'd0),
        .ext_a_rx_ts               (32'd0),
        .ext_a_rx_good_count       (32'd0),
        .ext_a_rx_error_count      (32'd0),
        .ext_a_rx_length_count     (32'd0),
        .ext_a_rx_overflow_count   (32'd0),
        .ext_b_tx_data             (),
        .ext_b_tx_valid            (),
        .ext_b_tx_last             (),
        .ext_b_rx_ready            (),
        .ext_b_tx_ready            (1'b0),
        .ext_b_wire_sof            (1'b0),
        .ext_b_wire_done           (1'b0),
        .ext_b_rx_valid            (1'b0),
        .ext_b_rx_last             (1'b0),
        .ext_b_rx_data             (8'd0),
        .ext_b_rx_ts               (32'd0),
        .ext_b_rx_good_count       (32'd0),
        .ext_b_rx_error_count      (32'd0),
        .ext_b_rx_length_count     (32'd0),
        .ext_b_rx_overflow_count   (32'd0),
        .clk(clk), .reset(reset), .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid), .tx_ready(tx_envelope), .rx_ready(rx_envelope)
    );

    integer fd;
    integer sim_ms, mgmt_load, snmp_period_us;
    reg [8*256-1:0] csv_name;

    // ---------------- 交换机模型：rx 口帧队列 ----------------
    byte unsigned ring [0:RING-1];
    int ring_wr = 0, ring_rd = 0, ring_used = 0, ring_peak = 0;
    int rq_len[$];
    task automatic push_frame_ref();                  // ref_frm[0..ref_flen-1]
        if (ring_used + ref_flen > RING) $fatal(1, "switch buffer overflow");
        for (int i = 0; i < ref_flen; i++) begin ring[ring_wr] = ref_frm[i]; ring_wr = (ring_wr + 1) % RING; end
        ring_used += ref_flen; if (ring_used > ring_peak) ring_peak = ring_used;
        rq_len.push_back(ref_flen);
    endtask

    int sp_len = 0, sp_pos = 0, sp_div = 0, sp_gap = 0;
    bit sp_active = 0;
    always @(posedge clk) begin
        rx_valid <= 1'b0;
        if (!sp_active) begin
            if (sp_gap > 0) sp_gap--;
            else if (rq_len.size() > 0) begin
                sp_len = rq_len.pop_front(); sp_active = 1; sp_pos = 0; sp_div = 0;
            end
        end
        if (sp_active) begin
            rx_envelope <= 1'b1;
            if (sp_div == 0 && sp_pos < sp_len) begin
                rx_data <= ring[ring_rd]; rx_valid <= 1'b1;
                ring_rd = (ring_rd + 1) % RING; ring_used--; sp_pos++;
            end
            sp_div = (sp_div == 7) ? 0 : sp_div + 1;
            if (sp_pos == sp_len && sp_div == 0) begin sp_active = 0; sp_gap = 12 * 8 - 1; end
        end else rx_envelope <= 1'b0;
    end

    // ---------------- 管理站 ----------------
    int inj_sn = 0;
    task automatic inject(input int sport, input int dport, input int plen);
        inj_sn = (inj_sn >= 255) ? 1 : inj_sn + 1;
        for (int iface = 1; iface <= 2; iface++) begin
            ref_build_frame(MGMT_VLID, inj_sn, iface, 1, 2, 1, sport, dport, plen, 1);
            push_frame_ref();
        end
    endtask
    int tftp_port = 20000, tftp_blocks = 0, tftp_files = 0, snmp_rsp = 0, mgmt_frames = 0;
    longint tftp_last_ns = 0;
    task automatic tftp_rrq();
        string name = "config/afdx-vl-config.bin";
        string mode = "octet";
        int n = 0;
        tftp_port = (tftp_port >= 20999) ? 20000 : tftp_port + 1;
        ref_pld[n++] = 0; ref_pld[n++] = 1;
        for (int i = 0; i < name.len(); i++) ref_pld[n++] = name[i];
        ref_pld[n++] = 0;
        for (int i = 0; i < mode.len(); i++) ref_pld[n++] = mode[i];
        ref_pld[n++] = 0;
        inject(tftp_port, 69, n);
        tftp_last_ns = $time;
    endtask
    task automatic tftp_ack(input int tid, input int block);
        ref_pld[0] = 0; ref_pld[1] = 4; ref_pld[2] = block >> 8; ref_pld[3] = block & 255;
        inject(tftp_port, tid, 4);
        tftp_last_ns = $time;
    endtask
    // BER 长度（<= 65535）
    function automatic int put_len(input int at, input int len);
        if (len < 128) begin ref_pld[at] = len; return 1; end
        if (len < 256) begin ref_pld[at] = 8'h81; ref_pld[at+1] = len; return 2; end
        ref_pld[at] = 8'h82; ref_pld[at+1] = len >> 8; ref_pld[at+2] = len & 255; return 3;
    endfunction
    function automatic int len_size(input int len);
        return len < 128 ? 1 : (len < 256 ? 2 : 3);
    endfunction
    int snmp_rid = 0;
    task automatic snmp_get(input int nvb);
        // GET sysDescr.0 × nvb
        byte unsigned vb [0:13] = '{8'h30, 8'h0c, 8'h06, 8'h08, 8'h2b, 8'h06, 8'h01, 8'h02, 8'h01, 8'h01,
                                   8'h01, 8'h00, 8'h05, 8'h00};
        int vbl = 14 * nvb;
        int pdu = 3 + 3 + 3 + 1 + len_size(vbl) + vbl;
        int msg = 3 + 8 + 1 + len_size(pdu) + pdu;
        int n = 0;
        snmp_rid = (snmp_rid + 1) & 127;
        ref_pld[n++] = 8'h30; n += put_len(n, msg);
        ref_pld[n++] = 8'h02; ref_pld[n++] = 1; ref_pld[n++] = 1;
        ref_pld[n++] = 8'h04; ref_pld[n++] = 6;
        ref_pld[n++] = "p"; ref_pld[n++] = "u"; ref_pld[n++] = "b"; ref_pld[n++] = "l"; ref_pld[n++] = "i"; ref_pld[n++] = "c";
        ref_pld[n++] = 8'ha0; n += put_len(n, pdu);
        ref_pld[n++] = 8'h02; ref_pld[n++] = 1; ref_pld[n++] = snmp_rid;
        ref_pld[n++] = 8'h02; ref_pld[n++] = 1; ref_pld[n++] = 0;
        ref_pld[n++] = 8'h02; ref_pld[n++] = 1; ref_pld[n++] = 0;
        ref_pld[n++] = 8'h30; n += put_len(n, vbl);
        for (int k = 0; k < nvb; k++) for (int i = 0; i < 14; i++) ref_pld[n++] = vb[i];
        inject(7000, 161, n);
    endtask

    // ---------------- 发送总线捕获 ----------------
    byte unsigned cap [0:1599];
    int cap_pos = 0;
    longint cap_sof = 0;
    bit capturing = 0;
    int bus_frames = 0, sof_records = 0;
    always @(posedge clk) begin
        if (reset || dut.rst) begin
            capturing = 0; cap_pos = 0;
        end else begin
            if (tx_valid) begin
                if (!capturing) begin capturing = 1; cap_pos = 0; cap_sof = $time; end
                if (cap_pos >= 1600) $fatal(1, "frame exceeds capture buffer");
                cap[cap_pos++] = tx_data;
            end
            if (capturing && !tx_envelope && !tx_valid) begin
                capturing = 0; bus_frames++;
                if ({cap[4], cap[5]} == MGMT_VLID) begin
                    int dport, sport;
                    dport = {cap[36], cap[37]};
                    sport = {cap[34], cap[35]};
                    mgmt_frames++;
                    $fdisplay(fd, "M %0d %0d %0d", cap_sof, cap_pos, dport);
                    // 同一帧的 A/B 两份只处理 A 份
                    if (cap[11][7:5] == 3'b001) begin
                        if (dport == tftp_port && cap[42] == 0 && cap[43] == 3) begin
                            int dlen;
                            dlen = {cap[38], cap[39]} - 8 - 4;
                            tftp_blocks++;
                            tftp_ack(sport, {cap[44], cap[45]});
                            if (dlen < 512) begin tftp_files++; tftp_rrq(); end
                        end else if (dport == tftp_port && cap[42] == 0 && cap[43] == 5) begin
                            tftp_rrq();
                        end else if (dport == 7000) snmp_rsp++;
                    end
                end else begin
                    // 业务帧：载荷字节 0 = VL 索引，1..2 = 消息序号
                    $fdisplay(fd, "S %0d %0d %0d %0d %0d", cap[42], {cap[43], cap[44]},
                              cap[11][7:5] == 3'b010, cap_sof, cap_pos);
                    sof_records++;
                    for (int i = 0; i < cap_pos; i++) ref_frm[i] = cap[i];
                    ref_flen = cap_pos;
                    push_frame_ref();
                end
            end
        end
    end

    // ---------------- 应用发送/交付监视 ----------------
    int g_idx = 0, g_vl = 0, g_seq = 0, releases = 0;
    longint g_t0 = 0;
    always @(posedge clk) if (!dut.rst && dut.gen_tx_valid && dut.gen_tx_ready) begin
        if (g_idx == 0) begin g_t0 = $time; g_vl = dut.gen_tx_vl; end
        if (g_idx == 1) g_seq = dut.gen_tx_data << 8;
        if (g_idx == 2) g_seq = g_seq | dut.gen_tx_data;
        if (dut.gen_tx_last) begin
            $fdisplay(fd, "R %0d %0d %0d", g_vl, g_seq, g_t0); releases++; g_idx = 0;
        end else g_idx++;
    end
    int d_idx = 0, d_seq = 0, deliveries = 0;
    always @(posedge clk) if (!dut.rst && dut.app_rx_valid && dut.app_rx_ready) begin
        if (d_idx == 1) d_seq = dut.app_rx_data << 8;
        if (d_idx == 2) d_seq = d_seq | dut.app_rx_data;
        if (dut.app_rx_last) begin
            if (dut.app_rx_vl != 0) begin $fdisplay(fd, "D %0d %0d %0d", dut.app_rx_vl, d_seq, $time); deliveries++; end
            d_idx = 0;
        end else d_idx++;
    end

    initial begin
        if (!$value$plusargs("CSV=%s", csv_name)) csv_name = "sim/latency.csv";
        if (!$value$plusargs("SIM_MS=%d", sim_ms)) sim_ms = 100;
        if (!$value$plusargs("MGMT_LOAD=%d", mgmt_load)) mgmt_load = 0;
        if (!$value$plusargs("SNMP_PERIOD_US=%d", snmp_period_us)) snmp_period_us = 5000;
        fd = $fopen(csv_name, "w");
        if (fd == 0) $fatal(1, "cannot open %0s", csv_name);
        repeat (8) @(negedge clk);
        reset = 1'b0;
        while (dut.rst) @(negedge clk);
        if (mgmt_load) begin
            #2_000_000;
            tftp_rrq();
            fork
                forever begin
                    snmp_get(14);
                    #(snmp_period_us * 1000);
                end
                forever begin
                    #10_000_000;
                    if ($time - tftp_last_ns > 20_000_000) tftp_rrq();     // 丢失重启
                end
            join_none
        end
        #(longint'(sim_ms) * 1_000_000 - (mgmt_load ? 2_000_000 : 0));
        // 停止观测后再运行 3 ms，让在途消息交付
        $fdisplay(fd, "E %0d", $time);
        #3_000_000;
        $fdisplay(fd, "C releases=%0d deliveries=%0d bus_frames=%0d mgmt_frames=%0d tftp_blocks=%0d tftp_files=%0d snmp_rsp=%0d switch_peak=%0d",
                  releases, deliveries, bus_frames, mgmt_frames, tftp_blocks, tftp_files, snmp_rsp, ring_peak);
        $fdisplay(fd, "C tx_lmax_drop=%0d tx_ovf_drop=%0d rx_fcs=%0d rx_hdr=%0d rx_len=%0d rx_csum=%0d rx_ic=%0d rx_full=%0d rx_unknown=%0d rx_mac_ovf=%0d",
                  dut.cnt_tx_lmax_drop, dut.cnt_tx_ovf_drop, dut.cnt_rx_fcs_err, dut.cnt_rx_hdr_err,
                  dut.cnt_rx_len_err, dut.cnt_rx_csum_err, dut.cnt_rx_ic_drop, dut.cnt_rx_full_drop,
                  dut.cnt_rx_vl_unknown, dut.cnt_rx_mac_ovf);
        $fclose(fd);
        tb_check(releases > 0 && deliveries > 0, "application traffic flows");
        tb_check(dut.cnt_rx_fcs_err == 0 && dut.cnt_rx_hdr_err == 0 && dut.cnt_rx_len_err == 0 &&
                 dut.cnt_rx_csum_err == 0 && dut.cnt_rx_ic_drop == 0 && dut.cnt_rx_full_drop == 0 &&
                 dut.cnt_rx_mac_ovf == 0, "no receive errors or drops");
        tb_check(dut.cnt_tx_ovf_drop == 0, "no transmit overflow drops");
        tb_check(!mgmt_load || (tftp_blocks > 0 && snmp_rsp > 0), "management load is served");
        tb_errors = tb_err_cnt;
        tb_done = (tb_err_cnt == 0);
        $display("tb_afdx_latency: %0d checks, %0d errors -> %s", tb_chk_cnt, tb_err_cnt, tb_err_cnt == 0 ? "PASS" : "FAIL");
        if (tb_err_cnt) $fatal(1, "latency measurement failed");
        $finish;
    end
endmodule
