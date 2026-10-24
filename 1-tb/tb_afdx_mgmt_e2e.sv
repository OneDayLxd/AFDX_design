`timescale 1ns/1ps

// 管理面端到端回放驱动：按脚本把 UDP 载荷封装为完整 Ethernet/IPv4/UDP/AFDX 帧（管理 VL，
// 100 Mbps 字节节拍）注入 AFDX_top，捕获全部发送帧写入文件，由 scripts/mgmt_e2e.py 独立校验。
// 内置时间主站模型：master_ns = sim_ns * (1 + drift_ppb/1e9) + offset_ns；
// SYNC 的 t1 取注入首字节时刻，自动应答模式下收到 DELAY_REQ 后以帧尾时刻为 t4 回 DELAY_RESP。
// 脚本命令：
//   S sport dport n b0..      发送 UDP 载荷（dport=0：该客户端端口已学到的 TFTP 服务器 TID；
//                             dport=-p：客户端端口 p 学到的 TID，用于构造源端点变化）
//   W us                      等待
//   R n timeout_us            等待最近一次发送之后、发往该发送源端口的 n 帧   P tag  在捕获文件写标记
//   T drift_ppb offset_us     设置主站时钟            Y seq src q logp status   发送 SYNC
//   O seq src q logp          发送 ONEWAY             A 0|1       DELAY_REQ 自动应答
//   Q                         记录 DUT RTC 与主站时间  E           结束
//   D                         原样重发最近一次 SYNC/ONEWAY 载荷（新 AFDX 序列号）
module tb_afdx_mgmt_e2e;
    `include "tb_common.svh"

    reg clk = 1'b0;
    reg reset = 1'b1;
    always #5 clk = ~clk;

    wire [7:0] tx_data;
    wire tx_valid, tx_envelope;
    reg [7:0] rx_data = 0;
    reg rx_valid = 0;
    reg rx_envelope = 0;

    afdx_system #(
        .CLK_HZ(100_000_000), .BUS_BYTE_CYCLES(8), .APP_GEN_EN(0),
        .DUAL_NET_ON_BUS(0), .SKEW_MAX_US(100), .UDP_CSUM_EN(1),
        .TX_POOL_LOG(2), .RX_POOL_LOG(2), .BLOCK_AW(8), .TX_MAX_MSGS(16),
        .MGMT_EN(1), .SNMP_WRITE_COMM({"private", 72'd0}), .SNMP_WRITE_COMM_LEN(5'd7),
        .TFTP_TIMEOUT_US(20000), .SELFTEST_US(5000), .APP_AW(11)
    ) dut (
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
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_envelope), .rx_ready(rx_envelope)
    );

    localparam [15:0] RTC_PORT = 16'd3190;
    byte unsigned payload [0:1599];
    byte unsigned frame [0:1599];
    byte unsigned cap [0:1599];
    byte unsigned last_time [0:35];
    integer frame_len, cap_pos = 0, cap_count = 0, rsp_count = 0, fd_cap, fd_script;
    integer send_base = 0;                      // 最近一次发送后、发往 send_port 的帧数
    reg [15:0] send_port = 16'd0;
    integer cap_start_ns = 0;
    reg [31:0] cap_crc = 32'hffff_ffff;
    bit capturing = 0;
    reg [7:0] rx_sn = 8'd0;
    // 主站时钟
    real drift_ppb = 0.0, offset_us = 0.0;
    function automatic [63:0] master_us(input real t_ns);
        real m;
        begin
            m = t_ns * (1.0 + drift_ppb / 1.0e9) / 1000.0 + offset_us;
            master_us = $rtoi(m);
        end
    endfunction
    // DELAY_REQ 自动应答
    bit auto_resp = 0;
    bit resp_pending = 0;
    reg [31:0] resp_seq;
    reg [63:0] resp_t4;
    reg [15:0] resp_dport;
    // 客户端端口 -> 服务器 TID（从发往该端口的 DATA/ACK 学习，ERROR 不学习）
    reg [15:0] tid_port [0:63];
    reg [15:0] tid_val [0:63];
    integer tid_n = 0;
    function automatic [15:0] tid_lookup(input [15:0] port);
        integer k;
        begin
            tid_lookup = 16'd0;
            for (k = 0; k < tid_n; k = k + 1) if (tid_port[k] == port) tid_lookup = tid_val[k];
        end
    endfunction
    task automatic tid_learn(input [15:0] port, input [15:0] tid);
        integer k, hit;
        begin
            hit = -1;
            for (k = 0; k < tid_n; k = k + 1) if (tid_port[k] == port) hit = k;
            if (hit < 0 && tid_n < 64) begin hit = tid_n; tid_n = tid_n + 1; end
            if (hit >= 0) begin tid_port[hit] = port; tid_val[hit] = tid; end
        end
    endtask

    function automatic [15:0] fold(input [31:0] s);
        reg [31:0] v;
        begin v = s; v = v[15:0] + v[31:16]; v = v[15:0] + v[31:16]; fold = ~v[15:0]; end
    endfunction

    task automatic build_frame(input integer n, input [15:0] sport, input [15:0] dport);
        integer i, data_end;
        reg [15:0] ip_len, udp_len, ck;
        reg [31:0] sum, crc;
        begin
            for (i = 0; i < 1600; i = i + 1) frame[i] = 0;
            rx_sn = (rx_sn == 8'd255 || rx_sn == 8'd0) ? 8'd1 : rx_sn + 1'b1;
            frame[0]=8'h03; frame[4]=8'h10; frame[5]=8'h00;
            frame[6]=8'h02; frame[9]=8'h01; frame[10]=8'h01; frame[11]=8'h20;
            frame[12]=8'h08; frame[13]=8'h00;
            ip_len = 28 + n; udp_len = 8 + n;
            frame[14]=8'h45; frame[16]=ip_len >> 8; frame[17]=ip_len;
            frame[19]=rx_sn; frame[22]=1; frame[23]=17;
            frame[26]=8'h0a; frame[27]=1; frame[28]=1; frame[29]=1;
            frame[30]=8'he0; frame[31]=8'he0; frame[32]=8'h10; frame[33]=0;
            sum = 0;
            for (i = 14; i < 34; i = i + 2) sum = sum + {frame[i], frame[i+1]};
            ck = fold(sum); frame[24] = ck >> 8; frame[25] = ck;
            frame[34]=sport >> 8; frame[35]=sport; frame[36]=dport >> 8; frame[37]=dport;
            frame[38]=udp_len >> 8; frame[39]=udp_len;
            for (i = 0; i < n; i = i + 1) frame[42+i] = payload[i];
            sum = 0;
            for (i = 26; i < 34; i = i + 2) sum = sum + {frame[i], frame[i+1]};
            sum = sum + 16'd17 + udp_len;
            for (i = 34; i < 42 + n; i = i + 2)
                sum = sum + {frame[i], (i + 1 < 42 + n) ? frame[i+1] : 8'h00};
            ck = fold(sum); if (ck == 0) ck = 16'hffff;
            frame[40] = ck >> 8; frame[41] = ck;
            data_end = 42 + ((n < 17) ? 17 : n);
            frame[data_end] = rx_sn;
            crc = 32'hffff_ffff;
            for (i = 0; i <= data_end; i = i + 1) crc = tb_crc32_byte(crc, frame[i]);
            frame[data_end+1] = ~crc[7:0]; frame[data_end+2] = ~crc[15:8];
            frame[data_end+3] = ~crc[23:16]; frame[data_end+4] = ~crc[31:24];
            frame_len = data_end + 5;
        end
    endtask

    // 100 Mbps：每 8 个时钟一个字节，帧包络覆盖整帧
    task automatic inject_frame;
        integer i, k;
        begin
            @(negedge clk);
            rx_envelope = 1'b1;
            for (i = 0; i < frame_len; i = i + 1) begin
                rx_data = frame[i]; rx_valid = 1'b1;
                @(negedge clk); rx_valid = 1'b0;
                for (k = 0; k < 7; k = k + 1) @(negedge clk);
            end
            rx_envelope = 1'b0; rx_data = 0;
            for (k = 0; k < 12*8; k = k + 1) @(negedge clk);
        end
    endtask

    task automatic put64(input integer at, input [63:0] v);
        integer i;
        for (i = 0; i < 8; i = i + 1) payload[at+i] = v >> (56 - 8*i);
    endtask
    task automatic put32(input integer at, input [31:0] v);
        integer i;
        for (i = 0; i < 4; i = i + 1) payload[at+i] = v >> (24 - 8*i);
    endtask
    // 时间报文: t1 在注入首字节前一刻采样（SOF 与首个 negedge 相差不足一个字节时隙）
    task automatic send_time(input [7:0] typ, input [31:0] seq, input [15:0] src, input [7:0] q,
                             input [7:0] logp, input [7:0] status, input [63:0] ts, input [15:0] dport);
        integer i;
        begin
            for (i = 0; i < 36; i = i + 1) payload[i] = 0;
            put32(0, 32'h4154494D); payload[4] = 2; payload[5] = typ; payload[7] = 36;
            put32(8, seq); payload[12] = src >> 8; payload[13] = src; payload[14] = q; payload[15] = logp;
            put64(16, ts); payload[32] = status;
            if (typ != 8'd3) for (i = 0; i < 36; i = i + 1) last_time[i] = payload[i];
            build_frame(36, 16'd3190, dport);
            $fdisplay(fd_cap, "I %0d %0d %0d time type=%0d seq=%0d ts=%0d", $time, 3190, dport, typ, seq, ts);
            inject_frame();
        end
    endtask

    task automatic service;
        begin
            if (resp_pending) begin
                resp_pending = 0;
                send_time(8'd3, resp_seq, 16'h0a01, 8'd1, 8'd20, 8'd0, resp_t4, RTC_PORT);
            end
        end
    endtask
    task automatic wait_ns(input real ns);
        real stop;
        begin
            stop = $realtime + ns;
            while ($realtime < stop) begin service(); @(negedge clk); end
        end
    endtask

    // 捕获发送帧：完整帧（含 FCS）写入文件
    integer i_cap;
    always @(posedge clk) begin
        if (reset || dut.rst) begin
            capturing = 0; cap_pos = 0;
        end else begin
            if (tx_valid) begin
                if (!capturing) begin capturing = 1; cap_pos = 0; cap_crc = 32'hffff_ffff; cap_start_ns = $time; end
                cap[cap_pos] = tx_data; cap_pos = cap_pos + 1;
                cap_crc = tb_crc32_byte(cap_crc, tx_data);
            end
            if (capturing && !tx_envelope && !tx_valid) begin
                capturing = 0;
                cap_count = cap_count + 1;
                if ({cap[36], cap[37]} != 16'd162) rsp_count = rsp_count + 1;   // Trap 不计入响应数
                if ({cap[36], cap[37]} == send_port) send_base = send_base + 1;
                if (cap[34] == 8'hc0 && cap[35][7:4] == 4'h0 && cap[42] == 8'd0 && (cap[43] == 8'd3 || cap[43] == 8'd4))
                    tid_learn({cap[36], cap[37]}, {cap[34], cap[35]});
                tb_check(cap_crc == 32'hdebb20e3, "captured frame has a valid Ethernet FCS");
                $fwrite(fd_cap, "F %0d %0d %0d", cap_start_ns, $time, cap_pos);
                for (i_cap = 0; i_cap < cap_pos; i_cap = i_cap + 1) $fwrite(fd_cap, " %02x", cap[i_cap]);
                $fwrite(fd_cap, "\n");
                // DELAY_REQ：UDP 源端口 3190，类型 2
                if (auto_resp && cap_pos > 50 && {cap[34], cap[35]} == RTC_PORT && cap[47] == 8'd2) begin
                    resp_seq = {cap[50], cap[51], cap[52], cap[53]};
                    resp_t4 = master_us($realtime);
                    resp_pending = 1;
                end
            end
        end
    end

    reg [8*128-1:0] script_name, cap_name;
    integer rc, n, i, a0, a1, a2, a3, a4;
    reg [7:0] c;
    integer bv;
    initial begin
        if (!$value$plusargs("SCRIPT=%s", script_name)) script_name = "sim/mgmt_script.txt";
        fd_script = $fopen(script_name, "r");
        if (!$value$plusargs("CAP=%s", cap_name)) cap_name = "sim/mgmt_capture.txt";
        fd_cap = $fopen(cap_name, "w");
        if (fd_script == 0 || fd_cap == 0) $fatal(1, "cannot open script or capture file");
        repeat (8) @(negedge clk);
        reset = 1'b0;
        while (dut.rst) @(negedge clk);
        repeat (400) @(negedge clk);                     // 运行时配置 RAM 初始化
        forever begin
            rc = $fscanf(fd_script, " %c", c);
            if (rc != 1 || c == "E") begin
                wait_ns(50_000);
                $fdisplay(fd_cap, "END %0d", $time);
                $fclose(fd_cap);
                tb_check(cap_count > 0, "management scenario produced responses");
                tb_finish("tb_afdx_mgmt_e2e");
                $finish;
            end
            case (c)
                "S": begin
                    rc = $fscanf(fd_script, "%d %d %d", a0, a1, n);
                    for (i = 0; i < n; i = i + 1) begin rc = $fscanf(fd_script, "%h", bv); payload[i] = bv; end
                    if (a1 == 0) a1 = tid_lookup(a0);
                    else if (a1 < 0) a1 = tid_lookup(-a1);
                    build_frame(n, a0, a1);
                    send_base = 0; send_port = a0;
                    $fdisplay(fd_cap, "I %0d %0d %0d udp len=%0d", $time, a0, a1, n);
                    inject_frame();
                end
                "W": begin rc = $fscanf(fd_script, "%d", a0); wait_ns(a0 * 1000.0); end
                "R": begin
                    rc = $fscanf(fd_script, "%d %d", a0, a1);
                    wait_until(a0, a1);
                end
                "P": begin rc = $fscanf(fd_script, "%d", a0); $fdisplay(fd_cap, "P %0d %0d", a0, $time); end
                "T": begin rc = $fscanf(fd_script, "%d %d", a0, a1); drift_ppb = a0; offset_us = a1; end
                "Y": begin
                    rc = $fscanf(fd_script, "%d %d %d %d %d", a0, a1, a2, a3, a4);
                    send_base = 0; send_port = RTC_PORT;
                    send_time(8'd1, a0, a1, a2, a3, a4, master_us($realtime + 5.0), RTC_PORT);
                end
                "O": begin
                    rc = $fscanf(fd_script, "%d %d %d %d", a0, a1, a2, a3);
                    send_base = 0; send_port = RTC_PORT;
                    send_time(8'd4, a0, a1, a2, a3, 8'd0, master_us($realtime + 5.0), RTC_PORT);
                end
                "A": begin rc = $fscanf(fd_script, "%d", a0); auto_resp = a0 != 0; end
                "D": begin
                    for (i = 0; i < 36; i = i + 1) payload[i] = last_time[i];
                    build_frame(36, RTC_PORT, RTC_PORT);
                    send_base = 0; send_port = RTC_PORT;
                    $fdisplay(fd_cap, "I %0d %0d %0d time duplicate", $time, 3190, 3190);
                    inject_frame();
                end
                "Q": $fdisplay(fd_cap, "Q %0d %0d %0d %0d", $time, dut.rtc64, master_us($realtime), dut.rtc_state);
                default: $fatal(1, "bad script command %c", c);
            endcase
        end
    end
    task automatic wait_until(input integer target, input integer timeout_us);
        real stop;
        begin
            stop = $realtime + timeout_us * 1000.0;
            while (send_base < target && $realtime < stop) begin service(); @(negedge clk); end
            tb_check(send_base >= target, "expected response frames arrive before the timeout");
            if (send_base < target)
                $display("wait_until: %0d of %0d responses to port %0d at %0t", send_base, target, send_port, $time);
        end
    endtask

    initial begin
        #2_000_000_000;
        $fatal(1, "tb_afdx_mgmt_e2e timeout");
    end
endmodule
