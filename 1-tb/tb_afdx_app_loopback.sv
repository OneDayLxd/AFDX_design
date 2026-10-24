/*
 * ================================================================
 * File Name    : tb_afdx_app_loopback.sv
 * Module Name  : tb_afdx_app_loopback
 *
 * Function:
 *   应用发送 -> 双网存储转发模型 -> 应用接收；不进行应用回送。
 *   记录应用握手与四个 GMII 边界，供独立检查器分段计时。
 *   网络模型仅整帧缓存、FIFO 排队、固定延迟和确定性丢帧/坏 FCS。
 *
 * Sub Module:
 *   afdx_mac_system
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
`default_nettype none
module tb_afdx_app_loopback;
    `include "tb_common.svh"
    reg         clk = 0, reset = 1, started = 0;
    always #4 clk = ~clk;
    wire [15:0] txd;
    wire [1:0]  tx_en, tx_er;
    reg  [15:0] rxd = 0;
    reg  [1:0]  rx_dv = 0;
    reg  [6:0]  host_tx_vl = 0;
    reg  [7:0]  host_tx_data = 0;
    reg         host_tx_valid = 0, host_tx_last = 0, host_rx_ready = 1;
    wire        host_tx_ready, host_rx_valid, host_rx_last, host_rx_net;
    wire [11:0] host_rx_vl;
    wire [7:0]  host_rx_data, host_rx_sn;
    wire [10:0] host_rx_len;

    // 与生产顶层相同的双 MAC/AFDX 数据面，仅通过已有 host_* 接入 TB 应用。
    afdx_mac_system #(
        .APP_EXTERNAL (1),
        .APP_GEN_EN   (0),
        .MGMT_EN      (0)
    ) dut (
        .clk            (clk),
        .reset          (reset),
        .tx_data        (txd[7:0]),
        .rx_data        (rxd[7:0]),
        .tx_valid       (tx_en[0]),
        .rx_valid       (rx_dv[0]),
        .tx_ready       (),
        .rx_ready       (1'b0),
        .rx_er          (1'b0),
        .tx_clk         (),
        .tx_er          (tx_er[0]),
        .b_tx_data      (txd[15:8]),
        .b_rx_data      (rxd[15:8]),
        .b_tx_valid     (tx_en[1]),
        .b_rx_valid     (rx_dv[1]),
        .b_tx_ready     (),
        .b_rx_ready     (1'b0),
        .b_rx_er        (1'b0),
        .b_tx_clk       (),
        .b_tx_er        (tx_er[1]),
        .host_tx_vl     (host_tx_vl),
        .host_tx_valid  (host_tx_valid),
        .host_tx_data   (host_tx_data),
        .host_tx_last   (host_tx_last),
        .host_tx_ready  (host_tx_ready),
        .host_rx_vl     (host_rx_vl),
        .host_rx_valid  (host_rx_valid),
        .host_rx_data   (host_rx_data),
        .host_rx_last   (host_rx_last),
        .host_rx_ready  (host_rx_ready),
        .host_rx_len    (host_rx_len),
        .host_rx_sn     (host_rx_sn),
        .host_rx_net    (host_rx_net),
        .host_rx_ts     (),
        .host_rx_src_ip (),
        .host_rx_sport  (),
        .host_rx_dport  ()
    );

    typedef struct {
        integer id, size;
        time    eof;
        bit [1526*8-1:0] data;
    } packet_t;
    mailbox #(packet_t) net_a = new(), net_b = new();
    integer scenario = 0, fd, n_vl, rounds, expected;
    integer period_ns, extra_delay_ns, offered = 0, committed = 0, received = 0;
    integer exp_vl[0:2047], exp_len[0:2047], seen[0:2047];
    integer wire_count[0:3] = '{0,0,0,0};
    integer dropped = 0, corrupted = 0;
    bit     net_busy[0:1] = '{0,0};
    string  prefix;
    time    epoch;
    integer shared_bytes = 0, shared_commits = 0, shared_aborts = 0, dual_read_cycles = 0;
    always @(posedge clk) if (started) begin
        if (dut.u_system.u_core.rxq_wr_valid && dut.u_system.u_core.rxq_wr_ready) shared_bytes++;
        if (dut.u_system.u_core.rxq_commit_ok) shared_commits++;
        if (dut.u_system.u_core.rxq_wr_abort) shared_aborts++;
        if (dut.u_system.u_core.ma_valid && dut.u_system.u_core.ma_ready &&
            dut.u_system.u_core.mb_valid && dut.u_system.u_core.mb_ready) dual_read_cycles++;
    end

    function automatic byte payload(input integer id, vl, length, pos);
        case (pos)
            0: return byte'(id >> 24);
            1: return byte'(id >> 16);
            2: return byte'(id >> 8);
            3: return byte'(id);
            4: return byte'(vl >> 8);
            5: return byte'(vl);
            6: return byte'(length >> 8);
            7: return byte'(length);
            default: return byte'(id*17 + vl + pos*7);
        endcase
    endfunction

    // 捕获全部 GMII 字节；所有边界与应用握手均在同一上升沿计时。
    byte cap[0:3][0:1525];
    integer pos[0:3] = '{0,0,0,0};
    integer idle[0:3] = '{100,100,100,100};
    bit active[0:3] = '{0,0,0,0};
    time cap_sfd[0:3];
    always @(posedge clk) if (started) begin : wire_capture
        bit valid_byte;
        byte data_byte;
        packet_t packet;
        for (integer s = 0; s < 4; s++) begin
            valid_byte = s < 2 ? tx_en[s] : rx_dv[s-2];
            data_byte = s < 2 ? txd[s*8 +: 8] : rxd[(s-2)*8 +: 8];
            if (valid_byte) begin
                if (!active[s]) begin
                    tb_check(idle[s] >= 12, "GMII IFG >= 12 clocks");
                    active[s] = 1;
                    pos[s] = 0;
                end
                if (pos[s] >= 1526) $fatal(1, "overlong frame");
                cap[s][pos[s]] = data_byte;
                if (pos[s] < 7) tb_check(data_byte == 8'h55, "preamble");
                if (pos[s] == 7) begin
                    tb_check(data_byte == 8'hd5, "SFD");
                    cap_sfd[s] = $time;
                end
                if (s < 2) tb_check(!tx_er[s], "no TX error");
                pos[s]++;
                idle[s] = 0;
            end else begin
                idle[s]++;
                if (active[s]) begin
                    tb_check(pos[s] >= 72, "minimum wire frame length");
                    $fwrite(fd, "%s %0d %0d %0d ", s < 2 ? "W" : "I", s%2, cap_sfd[s], $time-8);
                    for (integer p = 0; p < pos[s]; p++) $fwrite(fd, "%02x", cap[s][p]);
                    $fwrite(fd, "\n");
                    wire_count[s]++;
                    if (s < 2) begin
                        packet.id = {cap[s][50],cap[s][51],cap[s][52],cap[s][53]};
                        packet.size = pos[s];
                        packet.eof = $time-8;
                        packet.data = '0;
                        for (integer p = 0; p < pos[s]; p++) packet.data[p*8 +: 8] = cap[s][p];
                        if (s == 0) net_a.put(packet);
                        else net_b.put(packet);
                    end
                    active[s] = 0;
                end
            end
        end
    end

    task automatic network(input integer net);
        packet_t packet;
        integer delay_ns, action;
        time due;
        forever begin
            if (net == 0) net_a.get(packet);
            else net_b.get(packet);
            net_busy[net] = 1;
            action = 0; // 0: 原样转发；1: 丢弃；2: 改一个载荷位但不更新 FCS。
            delay_ns = (net == 0 ? 256 : 512) + extra_delay_ns;
            if (scenario == 2) begin
                case (packet.id % 6)
                    1: if (net == 1) action = 1;
                    2: if (net == 0) action = 1;
                    3: if (net == 0) action = 2;
                    4: if (net == 1) action = 2;
                    5: delay_ns = net == 0 ? 1024 : 256;
                    default: ;
                endcase
            end
            $fdisplay(fd, "N %0d %0d %0d %0d", net, packet.id, action, delay_ns);
            if (action == 1) dropped++;
            else begin
                if (action == 2) begin
                    corrupted++;
                    packet.data[58*8] = ~packet.data[58*8];
                end
                // 整帧到齐才可发送；FIFO 与上一帧 IFG 还可能增加等待。
                due = packet.eof + delay_ns;
                @(negedge clk);
                while ($time < due) @(negedge clk);
                for (integer p = 0; p < packet.size; p++) begin
                    rx_dv[net] = 1;
                    rxd[net*8 +: 8] = packet.data[p*8 +: 8];
                    @(negedge clk);
                end
                rx_dv[net] = 0;
                rxd[net*8 +: 8] = 0;
                repeat (12) @(negedge clk);
            end
            net_busy[net] = 0;
        end
    endtask
    initial begin wait(started); network(0); end
    initial begin wait(started); network(1); end

    byte app_cap[0:1470];
    integer app_pos = 0;
    time app_first;
    always @(posedge clk) if (started && host_rx_valid && host_rx_ready) begin : app_receive
        integer id;
        if (app_pos == 0) app_first = $time;
        if (app_pos >= 1471) $fatal(1, "overlong application message");
        app_cap[app_pos] = host_rx_data;
        app_pos++;
        if (host_rx_last) begin
            id = {app_cap[0],app_cap[1],app_cap[2],app_cap[3]};
            if (id < 0 || id >= expected) $fatal(1, "unknown application ID %0d", id);
            tb_check(seen[id] == 0, "one logical delivery, no A/B duplicate");
            tb_check(host_rx_vl == exp_vl[id], "application VL identity");
            tb_check(app_pos == exp_len[id] && host_rx_len == exp_len[id], "application length");
            for (integer p = 0; p < app_pos; p++)
                tb_check(app_cap[p] === payload(id,exp_vl[id],exp_len[id],p), "application payload identity");
            seen[id]++;
            received++;
            $fwrite(fd, "D %0d %0d %0d %0d %0d %0d %0d ", id, host_rx_vl, host_rx_len,
                    host_rx_sn, host_rx_net, app_first, $time);
            for (integer p = 0; p < app_pos; p++) $fwrite(fd, "%02x", app_cap[p]);
            $fwrite(fd, "\n");
            app_pos = 0;
        end
    end
    integer rx_phase = 0;
    always @(negedge clk) if (started) begin
        rx_phase++;
        // 每 200us 停止应用接收 50us，网络侧仍连续收帧。
        host_rx_ready = scenario != 3 || rx_phase%25000 < 18750;
    end

    task automatic send_message(input integer id, vl, length);
        exp_vl[id] = vl;
        exp_len[id] = length;
        @(negedge clk);
        $fdisplay(fd, "O %0d %0d %0d %0d", id, vl, length, $time);
        offered++;
        for (integer p = 0; p < length; p++) begin
            host_tx_vl = vl;
            host_tx_valid = 1;
            host_tx_data = payload(id,vl,length,p);
            host_tx_last = p == length-1;
            @(posedge clk);
            while (!host_tx_ready) @(posedge clk);
            if (p == 0) $fdisplay(fd, "F %0d %0d", id, $time);
            if (host_tx_last) begin
                $fdisplay(fd, "C %0d %0d", id, $time);
                committed++;
            end
            @(negedge clk);
        end
        host_tx_valid = 0;
        host_tx_last = 0;
    endtask

    integer sizes[0:3] = '{17,465,977,1471};
    initial begin : main
        integer id, length, timeout_cycles;
        time due;
        if (!$value$plusargs("SCENARIO=%d", scenario)) scenario = 0;
        if (scenario < 0 || scenario > 5) $fatal(1, "unknown scenario");
        n_vl = (scenario == 1 || scenario == 5) ? 127 : 8;
        rounds = (scenario == 1 || scenario == 5) ? 4 : 8;
        period_ns = scenario == 5 ? 0 : 2500000;
        extra_delay_ns = scenario == 4 ? 5000 : 0;
        expected = n_vl*rounds;
        prefix = $sformatf("app_loopback_%0d", scenario);
        fd = $fopen({prefix,".txt"}, "w");
        if (!fd) $fatal(1, "cannot open capture");
        $fdisplay(fd, "CONFIG %0d %0d %0d %0d %0d", scenario,n_vl,rounds,period_ns,extra_delay_ns);
        for (integer i = 0; i < 2048; i++) begin exp_vl[i] = -1; exp_len[i] = 0; seen[i] = 0; end
        repeat (40) @(negedge clk);
        reset = 0;
        wait (!dut.u_system.u_core.rxq_init_busy);
        repeat (200) @(negedge clk);
        started = 1;
        epoch = $time;
        id = 0;
        for (integer r = 0; r < rounds; r++) begin
            due = epoch + r*period_ns;
            while ($time < due) @(negedge clk);
            for (integer v = 1; v <= n_vl; v++) begin
                length = sizes[(r+v-1)%4];
                send_message(id,v,length);
                id++;
            end
        end
        timeout_cycles = 0;
        while ((received < expected || wire_count[0] < expected || wire_count[1] < expected ||
                net_a.num() != 0 || net_b.num() != 0 || net_busy[0] || net_busy[1]) && timeout_cycles < 2000000) begin
            @(negedge clk);
            timeout_cycles++;
        end
        repeat (30000) @(negedge clk);
        tb_check(received == expected, "all offered messages delivered");
        tb_check(offered == expected && committed == expected, "all source messages committed");
        tb_check(wire_count[0] == expected && wire_count[1] == expected, "both TX networks complete");
        tb_check(wire_count[2]+wire_count[3] == 2*expected-dropped, "network drop accounting");
        tb_check(dut.u_system.cnt_rx_mac_error == corrupted, "expected corrupted copies rejected by MAC");
        tb_check(dut.u_system.cnt_rx_rm_drop == 2*expected-dropped-corrupted-received, "exact RM duplicate accounting");
        tb_check(dut.u_system.cnt_rx_hdr_err == 0 && dut.u_system.cnt_rx_csum_err == 0 &&
                 dut.u_system.cnt_rx_ic_drop == 0 && dut.u_system.cnt_rx_full_drop == 0 &&
                 dut.u_system.cnt_rx_mac_ovf == 0 && dut.u_system.cnt_tx_ovf_drop == 0 &&
                 dut.u_system.cnt_tx_lmax_drop == 0, "no unexpected errors or overflow");
        begin
            integer unique_bytes;
            unique_bytes = 0;
            for (integer i=0; i<expected; i++) unique_bytes += exp_len[i];
            tb_check(shared_bytes == unique_bytes, "shared pool stores each logical payload exactly once");
            tb_check(shared_commits == expected && shared_aborts == 0, "duplicates never commit/abort shared writes");
            tb_check(dual_read_cycles > 0, "A/B frontends actually consume bytes in parallel");
            $display("RX_PARALLEL_PROFILE bytes=%0d commits=%0d aborts=%0d dual_read_cycles=%0d",
                     shared_bytes,shared_commits,shared_aborts,dual_read_cycles);
        end
        $fdisplay(fd, "END %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                  $time,offered,committed,received,dropped,corrupted,dut.u_system.cnt_rx_rm_drop,
                  dut.u_system.cnt_rx_mac_error,dut.u_system.cnt_rx_hdr_err,dut.u_system.cnt_rx_csum_err,
                  dut.u_system.cnt_rx_ic_drop,tb_errors);
        $fclose(fd);
        $display("APP_LOOPBACK_COMPLETE scenario=%0d delivered=%0d functional_errors=%0d",scenario,received,tb_errors);
        tb_finish("tb_afdx_app_loopback");
        $finish;
    end
    initial begin #40000000; $fatal(1, "loopback watchdog"); end
endmodule
`default_nettype wire
