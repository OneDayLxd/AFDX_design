`timescale 1ns/1ps
// 接收 VL 队列 + 交付模块联合测试：默认 4096 VL，可用 NE_VLS/NE_VLW 缩小回归；
// NONEMPTY_RAM=1 分组非空 RAM、
// 复位清零扫描、随机 VL 写入与交付，逐 VL 顺序及消息内容逐字节核对。
module tb_afdx_rx_ne_ram;
`ifndef NE_VLS
`define NE_VLS 4096
`endif
`ifndef NE_VLW
`define NE_VLW 12
`endif
`ifndef NE_MODE
`define NE_MODE 1
`endif
`ifndef N_MSGS
`define N_MSGS 300
`endif
    `include "tb_common.svh"
    localparam N_VL = `NE_VLS, VLW = `NE_VLW;
    localparam NE_GW = VLW > 4 ? VLW-4 : 1;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    reg [VLW-1:0] wr_vl = 0;
    reg wr_valid = 0, wr_commit = 0;
    reg [7:0] wr_data = 0;
    wire [8:0] wr_meta = wr_vl;
    wire wr_ready, wr_ovf;
    wire [VLW-1:0] q_rd_vl, msg_vl;
    wire q_rd_start, q_rd_pop, q_rd_ready, q_rd_info_valid, q_rd_pop_ready;
    wire [10:0] q_rd_len, msg_len;
    wire [8:0] q_rd_meta, msg_meta;
    wire [7:0] q_rd_data, msg_data;
    wire q_rd_valid, q_rd_last, msg_valid, msg_last, stat_deliver;
    wire [N_VL-1:0] nonempty;
    wire [NE_GW-1:0] ne_group;
    wire [15:0] ne_bits;
    wire init_busy;
    wire [(N_VL+255)/256-1:0] ne_regions;
    wire [8:0] pool_used, pool_peak, pool_msgs, pool_msgs_peak;
    reg msg_ready = 0;

    afdx_msg_queue #(.N_VL(N_VL), .VLW(VLW), .DEPTH(4), .DW(2), .SLOT_AW(11), .MW(9),
                     .POOL_AW(5), .BLOCK_AW(8), .NONEMPTY_RAM(`NE_MODE), .SUMMARY_ENABLE(1)) q (
        .clk(clk), .rst(rst), .wr_vl(wr_vl), .wr_valid(wr_valid), .wr_data(wr_data),
        .wr_ready(wr_ready), .wr_commit(wr_commit), .wr_meta(wr_meta),
        .wr_abort(1'b0), .wr_ovf(wr_ovf),
        .rd_vl(q_rd_vl), .rd_start(q_rd_start), .rd_pop(q_rd_pop),
        .rd_info_valid(q_rd_info_valid), .reclaim_busy(), .rd_pop_ready(q_rd_pop_ready),
        .rd_len(q_rd_len), .rd_meta(q_rd_meta), .rd_data(q_rd_data),
        .rd_valid(q_rd_valid), .rd_last(q_rd_last), .rd_ready(q_rd_ready),
        .nonempty(nonempty), .pool_used(pool_used), .pool_peak(pool_peak),
        .pool_msgs(pool_msgs), .pool_msgs_peak(pool_msgs_peak),
        .ne_group(ne_group), .ne_bits(ne_bits), .init_busy(init_busy), .ne_regions(ne_regions),
        .wr_commit_ok ());
    afdx_rx_deliver #(.N_VL(N_VL), .VLW(VLW), .SLOT_AW(11), .MW(9), .NONEMPTY_RAM(`NE_MODE), .SUMMARY_ENABLE(1)) d (
        .clk(clk), .rst(rst), .nonempty(nonempty),
        .ne_group(ne_group), .ne_bits(ne_bits), .q_init_busy(init_busy),
        .q_rd_vl(q_rd_vl), .q_rd_start(q_rd_start), .q_rd_pop(q_rd_pop),
        .q_rd_info_valid(q_rd_info_valid), .q_rd_pop_ready(q_rd_pop_ready), .q_rd_len(q_rd_len),
        .q_rd_meta(q_rd_meta), .q_rd_data(q_rd_data), .q_rd_valid(q_rd_valid), .q_rd_last(q_rd_last),
        .q_rd_ready(q_rd_ready), .msg_vl(msg_vl), .msg_valid(msg_valid), .msg_data(msg_data),
        .msg_last(msg_last), .msg_ready(msg_ready), .msg_len(msg_len), .msg_meta(msg_meta),
        .stat_deliver(stat_deliver), .ne_regions(ne_regions));

    // 消息内容 = {序号, 长度} 编码：首字节为该 VL 的发送序号，其余字节 = 序号+位置。
    integer sent_seq [0:N_VL-1];
    integer recv_seq [0:N_VL-1];
    integer sent = 0, received = 0, byte_pos = 0, errors_seen = 0;
    integer i, seed = 7;
    integer region_model [0:(N_VL+255)/256-1];
    integer region_index;
    always @(posedge clk) begin
        if (rst) begin
            for (region_index=0; region_index<(N_VL+255)/256; region_index=region_index+1)
                region_model[region_index] = 0;
        end else begin
            if (q.commit) region_model[q.commit_vl/256] = region_model[q.commit_vl/256] + 1;
            if (q.pop) region_model[q_rd_vl/256] = region_model[q_rd_vl/256] - 1;
        end
    end
    always @(negedge clk) begin
        if (!rst) begin
            for (region_index=0; region_index<(N_VL+255)/256; region_index=region_index+1)
                if (region_model[region_index] < 0 ||
                    ne_regions[region_index] !== (region_model[region_index] != 0))
                    $fatal(1, "region occupancy mismatch region=%0d count=%0d", region_index, region_model[region_index]);
        end
    end

    task automatic send_msg(input integer vl, input integer len);
        integer k, timeout;
        begin
            for (k = 0; k < len; k = k + 1) begin
                @(negedge clk);
                wr_vl = vl; wr_data = sent_seq[vl] + k; wr_valid = 1; wr_commit = (k == len-1);
                timeout = 0;
                #1;
                while (wr_ready !== 1'b1) begin
                    timeout = timeout + 1;
                    if (timeout > 20000)
                        $fatal(1, "write stalled on VL %0d: ready=%b sent=%0d received=%0d blocks=%0d msgs=%0d deliver_state=%0d", vl, wr_ready, sent, received, pool_used, pool_msgs, d.state);
                    @(negedge clk); #1;
                end
                @(posedge clk);
            end
            #1 wr_valid = 0; wr_commit = 0;
            sent_seq[vl] = sent_seq[vl] + 1;
            sent = sent + 1;
        end
    endtask

    always @(posedge clk) begin
        if (msg_valid && msg_ready) begin
            if (msg_data !== ((recv_seq[msg_vl] + byte_pos) & 8'hff)) errors_seen = errors_seen + 1;
            byte_pos = byte_pos + 1;
            if (msg_last) begin
                recv_seq[msg_vl] = recv_seq[msg_vl] + 1;
                received = received + 1;
                byte_pos = 0;
            end
        end
        msg_ready <= ($random(seed) & 3) != 0;
    end

    integer vl, t;
    initial begin
        for (i = 0; i < N_VL; i = i + 1) begin sent_seq[i] = 0; recv_seq[i] = 0; end
        repeat (5) @(negedge clk);
        rst = 0;
        @(posedge clk);
        if (`NE_MODE) tb_check(init_busy === 1'b1, "state RAM clear sweep starts after reset");
        if (`NE_MODE) tb_check(wr_ready === 1'b0, "writes blocked during clear sweep");
        t = 0;
        while (init_busy) begin @(posedge clk); t = t + 1; end
        if (`NE_MODE) tb_check(t >= N_VL && t <= N_VL+104, "clear sweep covers all VLs");
        // 边界 VL 与同组多 VL
        send_msg(0, 3); send_msg(N_VL-1, 5); send_msg(15, 2); send_msg(16, 2); send_msg(N_VL-1, 1);
        for (i = 0; i < `N_MSGS; i = i + 1) begin
            vl = $random(seed) & (N_VL-1);
            if (($random(seed) & 3) == 0) vl = vl & 12'h01f;   // 集中少数 VL，触发同 VL 多条排队
            send_msg(vl, 1 + (($random(seed) & 32'h7fff_ffff) % 40));
        end
        t = 0;
        while (received < sent && t < 200000) begin @(posedge clk); t = t + 1; end
        if (received != sent) $display("sent=%0d received=%0d deliver_state=%0d rd_vl=%0d info=%0d scan=%0d", sent, received, d.state, q_rd_vl, q_rd_info_valid, d.scan_group);
        tb_check(received == sent, "all messages delivered");
        tb_check(errors_seen == 0, "payload bytes and per-VL order correct");
        for (i = 0; i < N_VL; i = i + 1)
            if (recv_seq[i] != sent_seq[i]) errors_seen = errors_seen + 1;
        tb_check(errors_seen == 0, "per-VL message counts match");
        repeat (50) @(posedge clk);
        tb_check(pool_used == 0 && pool_msgs == 0, "pool drained");
        tb_finish("tb_afdx_rx_ne_ram");
        $finish;
    end
endmodule
