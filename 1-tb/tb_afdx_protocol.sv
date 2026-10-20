`timescale 1ns/1ps

module tb_afdx_protocol;
    `include "tb_common.svh"

    localparam CLK_HZ = 10_000_000;
    localparam N_VL = 4;
    localparam VLW = 2;
    localparam BAG_CYCLES = 200;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #50 clk = ~clk;

    reg [VLW-1:0] tx_msg_vl = 0;
    reg tx_msg_valid = 0;
    reg [7:0] tx_msg_data = 0;
    reg tx_msg_last = 0;
    wire tx_msg_ready;
    wire [VLW-1:0] rx_msg_vl;
    wire rx_msg_valid, rx_msg_last;
    wire [7:0] rx_msg_data, rx_msg_sn;
    wire rx_msg_net;
    wire [10:0] rx_msg_len;
    reg rx_msg_ready = 0;

    wire [7:0] mac_a_tx_data, mac_b_tx_data;
    wire mac_a_tx_valid, mac_b_tx_valid, mac_a_tx_frame, mac_b_tx_frame;
    wire [31:0] rtc_us, cnt_tx_msgs, cnt_tx_frames, cnt_tx_frames_a, cnt_tx_frames_b;
    wire [31:0] cnt_tx_lmax_drop, cnt_tx_ovf_drop;
    wire [31:0] cnt_rx_frames_a, cnt_rx_frames_b, cnt_rx_fcs_err, cnt_rx_mac_len_err, cnt_rx_mac_ovf;
    wire [31:0] cnt_rx_hdr_err, cnt_rx_len_err, cnt_rx_vl_unknown, cnt_rx_csum_err;
    wire [31:0] cnt_rx_full_drop, cnt_rx_ic_drop, cnt_rx_rm_drop, cnt_rx_accepted, cnt_rx_delivered;
    wire [5:0] tx_buffer_used, tx_buffer_peak, rx_buffer_used, rx_buffer_peak;
    wire [5:0] tx_buffer_messages, tx_buffer_messages_peak, rx_buffer_messages, rx_buffer_messages_peak;

    // Each physical network is looped back independently.  A received byte is
    // sampled on the same edge on which the corresponding MAC transmits it.
    localparam TX_VLW_TIE = VLW;
    afdx_es_core #(
        .CLK_HZ(CLK_HZ), .N_TX_VL(N_VL), .TX_VLW(VLW), .N_RX_VL(N_VL), .RX_VLW(VLW),
        .TX_DEPTH_LOG(3), .RX_DEPTH_LOG(3), .TX_POOL_LOG(2), .RX_POOL_LOG(2),
        .SLOT_AW(11), .BLOCK_AW(8), .TX_MAX_MSGS(16), .RX_MAX_MSGS(16),
        .SKEW_MAX_US(50), .UDP_CSUM_EN(1), .IC_EN(1), .RM_EN(1),
        .IFG(12), .MAC_FIFO_AW(11), .RX_GAP_CYCLES(4)
    ) dut (
        .clk(clk), .rst(rst),
        .tx_msg_vl(tx_msg_vl), .tx_msg_valid(tx_msg_valid), .tx_msg_data(tx_msg_data),
        .tx_msg_last(tx_msg_last), .tx_msg_ready(tx_msg_ready),
        .rx_msg_vl(rx_msg_vl), .rx_msg_valid(rx_msg_valid), .rx_msg_data(rx_msg_data),
        .rx_msg_last(rx_msg_last), .rx_msg_ready(rx_msg_ready), .rx_msg_len(rx_msg_len),
        .rx_msg_sn(rx_msg_sn), .rx_msg_net(rx_msg_net),
        .mac_a_tx_data(mac_a_tx_data), .mac_a_tx_valid(mac_a_tx_valid), .mac_a_tx_slot(1'b1),
        .mac_a_tx_frame(mac_a_tx_frame), .mac_a_tx_busy(),
        .mac_a_rx_data(mac_a_tx_data), .mac_a_rx_valid(mac_a_tx_valid), .mac_a_rx_frame(mac_a_tx_frame),
        .mac_b_tx_data(mac_b_tx_data), .mac_b_tx_valid(mac_b_tx_valid), .mac_b_tx_slot(1'b1),
        .mac_b_tx_frame(mac_b_tx_frame), .mac_b_tx_busy(),
        .mac_b_rx_data(mac_b_tx_data), .mac_b_rx_valid(mac_b_tx_valid), .mac_b_rx_frame(mac_b_tx_frame),
        .rtc_load(1'b0), .rtc_load_val(32'd0), .rtc_us(rtc_us), .tick_us(),
        `include "tb_core_mgmt_tie.svh"
        .cnt_tx_msgs(cnt_tx_msgs), .cnt_tx_frames(cnt_tx_frames),
        .cnt_tx_frames_a(cnt_tx_frames_a), .cnt_tx_frames_b(cnt_tx_frames_b),
        .cnt_tx_lmax_drop(cnt_tx_lmax_drop), .cnt_tx_ovf_drop(cnt_tx_ovf_drop),
        .cnt_rx_frames_a(cnt_rx_frames_a), .cnt_rx_frames_b(cnt_rx_frames_b),
        .cnt_rx_fcs_err(cnt_rx_fcs_err), .cnt_rx_mac_len_err(cnt_rx_mac_len_err),
        .cnt_rx_mac_ovf(cnt_rx_mac_ovf), .cnt_rx_hdr_err(cnt_rx_hdr_err),
        .cnt_rx_len_err(cnt_rx_len_err), .cnt_rx_vl_unknown(cnt_rx_vl_unknown),
        .cnt_rx_csum_err(cnt_rx_csum_err), .cnt_rx_full_drop(cnt_rx_full_drop),
        .cnt_rx_ic_drop(cnt_rx_ic_drop), .cnt_rx_rm_drop(cnt_rx_rm_drop),
        .cnt_rx_accepted(cnt_rx_accepted), .cnt_rx_delivered(cnt_rx_delivered),
        .tx_buffer_used(tx_buffer_used), .tx_buffer_peak(tx_buffer_peak),
        .rx_buffer_used(rx_buffer_used), .rx_buffer_peak(rx_buffer_peak),
        .tx_buffer_messages(tx_buffer_messages), .tx_buffer_messages_peak(tx_buffer_messages_peak),
        .rx_buffer_messages(rx_buffer_messages), .rx_buffer_messages_peak(rx_buffer_messages_peak)
    );

    integer cycle_count = 0;
    always @(posedge clk) cycle_count = cycle_count + 1;

    byte unsigned frame_bytes [0:1][0:1599];
    integer frame_pos [0:1];
    integer last_frame_len [0:1];
    integer frame_count [0:1];
    integer frame_start_cycle [0:1][0:299];
    reg [31:0] frame_crc [0:1];
    reg [31:0] last_frame_crc [0:1];
    bit frame_active [0:1];

    task automatic monitor_frame(input integer net, input [7:0] data,
                                 input logic valid, input logic envelope);
        begin
            if (rst) begin
                frame_pos[net] = 0;
                last_frame_len[net] = 0;
                frame_count[net] = 0;
                frame_crc[net] = 32'hffff_ffff;
                last_frame_crc[net] = 0;
                frame_active[net] = 0;
            end else begin
                if (valid) begin
                    if (!frame_active[net]) begin
                        frame_active[net] = 1;
                        frame_pos[net] = 0;
                        frame_crc[net] = 32'hffff_ffff;
                        frame_start_cycle[net][frame_count[net]] = cycle_count;
                    end
                    if (frame_pos[net] >= 1600) $fatal(1, "captured frame exceeds buffer");
                    frame_bytes[net][frame_pos[net]] = data;
                    frame_pos[net] = frame_pos[net] + 1;
                    frame_crc[net] = tb_crc32_byte(frame_crc[net], data);
                end
                if (frame_active[net] && !envelope && !valid) begin
                    last_frame_len[net] = frame_pos[net];
                    last_frame_crc[net] = frame_crc[net];
                    frame_count[net] = frame_count[net] + 1;
                    frame_active[net] = 0;
                end
            end
        end
    endtask

    always @(posedge clk) begin
        monitor_frame(0, mac_a_tx_data, mac_a_tx_valid, mac_a_tx_frame);
        monitor_frame(1, mac_b_tx_data, mac_b_tx_valid, mac_b_tx_frame);
    end

    function automatic [7:0] payload_byte(input integer seed, input integer position);
        payload_byte = (seed + position*7 + (position >> 3)) & 8'hff;
    endfunction

    task automatic reset_core;
        begin
            @(negedge clk);
            rst = 1'b1;
            tx_msg_valid = 1'b0;
            tx_msg_last = 1'b0;
            rx_msg_ready = 1'b0;
            repeat (6) @(negedge clk);
            rst = 1'b0;
            repeat (10) @(negedge clk);
        end
    endtask

    task automatic send_message(input integer vl, input integer length, input integer seed);
        integer position;
        integer timeout;
        begin
            for (position = 0; position < length; position = position + 1) begin
                @(negedge clk);
                tx_msg_vl = vl[VLW-1:0];
                tx_msg_data = payload_byte(seed, position);
                tx_msg_last = (position == length-1);
                tx_msg_valid = 1'b1;
                timeout = 0;
                #1;
                while (tx_msg_ready !== 1'b1 && timeout < 200) begin
                    @(negedge clk);
                    #1;
                    timeout = timeout + 1;
                end
                tb_check(timeout < 200, "TX application input makes progress");
                if (timeout >= 200) begin
                    $display("TX ready timeout: rst=%b ready=%b constants=%b/%b free_blocks=%b free_msgs=%b block_free=%b msg_free=%b group_valid=%b block_found=%b block_q=%b bcand=%b msg_found=%b mcand=%b admission=%b/%b count=%0d commit=%b cancel=%b",
                             rst, tx_msg_ready, dut.u_txq.BLOCKS_COUNT, dut.u_txq.MSGS_COUNT,
                             dut.u_txq.free_blocks, dut.u_txq.free_msgs,
                             dut.u_txq.block_free, dut.u_txq.msg_free, dut.u_txq.group_valid,
                             dut.u_txq.block_found, dut.u_txq.block_found_q,
                             dut.u_txq.block_candidate_valid, dut.u_txq.msg_found,
                             dut.u_txq.msg_candidate_valid, dut.u_txq.admission_vl,
                             dut.u_txq.admission_quota_ok, dut.u_txq.writer_count,
                             dut.u_txq.commit, dut.u_txq.cancel);
                    $display("allocator causes: txvalid=%b txbad=%b wrvalid=%b wrready=%b beat=%b active=%b count=%b need=%b abort=%b bad=%b over=%b allocb=%b allocm=%b delayed=%b/%b/%b msgdelayed=%b reclaim=%b release=%b pop=%b",
                             tx_msg_valid, dut.tx_bad, dut.u_txq.wr_valid, dut.u_txq.wr_ready,
                             dut.u_txq.beat, dut.u_txq.wr_active, dut.u_txq.wr_count,
                             dut.u_txq.need_block, dut.u_txq.wr_abort, dut.u_txq.wr_bad,
                             dut.u_txq.over, dut.u_txq.alloc_block, dut.u_txq.alloc_msg,
                             dut.u_txq.block_alloc_delayed, dut.u_txq.block_alloc_delayed2,
                             dut.u_txq.block_alloc_delayed3, dut.u_txq.msg_alloc_delayed,
                             dut.u_txq.reclaim_state, dut.u_txq.release_block, dut.u_txq.pop);
                    $display("input limit causes: vl=%b prev_lmax=%b cnt=%b dropping=%b limit=%b over=%b inbeat=%b msglast=%b",
                             tx_msg_vl, dut.tx_prev_lmax, dut.tx_in_cnt, dut.tx_dropping,
                             dut.put_limit, dut.put_over, dut.tx_in_beat, tx_msg_last);
                    $fatal(1, "TX application input stalled");
                end
                @(posedge clk);
            end
            @(negedge clk);
            tx_msg_valid = 1'b0;
            tx_msg_last = 1'b0;
        end
    endtask

    task automatic receive_message(input integer vl, input integer length,
                                   input integer seed, input [7:0] expected_sn);
        integer position;
        integer timeout;
        integer payload_errors;
        integer boundary_errors;
        begin
            payload_errors = 0;
            boundary_errors = 0;
            rx_msg_ready = 1'b0;
            for (position = 0; position < length; position = position + 1) begin
                timeout = 0;
                while (rx_msg_valid !== 1'b1 && timeout < 5000) begin
                    @(negedge clk);
                    timeout = timeout + 1;
                end
                tb_check(timeout < 5000, "RX application output makes progress");
                if (timeout >= 5000) begin
                    $display("RX timeout: tx_msgs=%0d tx_frames=%0d used=%0d nonempty=%b grant=%b/%0d fb_state=%0d a_ok=%0d b_ok=%0d accepted=%0d delivered=%0d fcs=%0d hdr=%0d len=%0d csum=%0d ic=%0d rm=%0d",
                             cnt_tx_msgs, cnt_tx_frames, tx_buffer_used, dut.txq_nonempty,
                             dut.grant_valid, dut.grant_vl, dut.u_fb.state,
                             cnt_rx_frames_a, cnt_rx_frames_b, cnt_rx_accepted,
                             cnt_rx_delivered, cnt_rx_fcs_err, cnt_rx_hdr_err, cnt_rx_len_err,
                             cnt_rx_csum_err, cnt_rx_ic_drop, cnt_rx_rm_drop);
                    $fatal(1, "RX application output stalled");
                end
                if (rx_msg_valid) begin
                    if (position == 0) begin
                        tb_check(rx_msg_vl == vl[VLW-1:0], "RX VL index matches");
                        tb_check(rx_msg_len == length, "RX message length metadata matches");
                        tb_check(rx_msg_sn == expected_sn, "RX sequence metadata matches");
                    end
                    if (rx_msg_data != payload_byte(seed, position)) payload_errors = payload_errors + 1;
                    if (rx_msg_last != (position == length-1)) boundary_errors = boundary_errors + 1;
                    rx_msg_ready = 1'b1;
                    @(posedge clk);
                    @(negedge clk);
                    rx_msg_ready = 1'b0;
                end
            end
            if (payload_errors != 0)
                $display("RX payload mismatch summary: vl=%0d len=%0d errors=%0d", vl, length, payload_errors);
            tb_check(payload_errors == 0, "RX payload matches application message exactly");
            tb_check(boundary_errors == 0, "RX last marker matches exact message boundary");
        end
    endtask

    task automatic wait_frames(input integer target_a, input integer target_b);
        integer timeout;
        begin
            timeout = 0;
            while ((frame_count[0] < target_a || frame_count[1] < target_b) && timeout < 100000) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            tb_check(timeout < 100000, "both enabled physical networks complete frames");
        end
    endtask

    task automatic verify_latest_frame(input integer net, input integer vl,
                                       input integer length, input integer seed,
                                       input [7:0] expected_sequence);
        integer position;
        integer expected_wire_length;
        integer payload_errors;
        reg [31:0] sum;
        begin
            expected_wire_length = ((length < 17) ? 17 : length) + 47;
            payload_errors = 0;
            $display("verify frame net=%0d vl=%0d expected_len=%0d actual_len=%0d crc=%08x",
                     net, vl, expected_wire_length, last_frame_len[net], last_frame_crc[net]);
            tb_check(last_frame_len[net] == expected_wire_length, "AFDX physical frame length is exact");
            tb_check(last_frame_len[net] <= 1518, "AFDX frame stays within 1518-byte limit");
            tb_check(last_frame_crc[net] == 32'hdebb20e3, "Ethernet FCS has independent CRC-32 residue");
            tb_check(frame_bytes[net][0] == 8'h03 && frame_bytes[net][1] == 0 &&
                     frame_bytes[net][2] == 0 && frame_bytes[net][3] == 0,
                     "AFDX multicast destination MAC prefix");
            tb_check({frame_bytes[net][4],frame_bytes[net][5]} == 16'h1000 + vl,
                     "destination MAC embeds VLID");
            tb_check(frame_bytes[net][6] == 8'h02 && frame_bytes[net][9] == 8'h01 &&
                     frame_bytes[net][10] == 8'h01,
                     "source MAC embeds network and equipment IDs");
            tb_check(frame_bytes[net][11][7:5] == net+1, "source MAC identifies redundant interface");
            tb_check({frame_bytes[net][12],frame_bytes[net][13]} == 16'h0800,
                     "EtherType is IPv4");
            tb_check(frame_bytes[net][14] == 8'h45 && frame_bytes[net][23] == 8'd17,
                     "IPv4 header is unfragmented UDP/IPv4");
            tb_check({frame_bytes[net][16],frame_bytes[net][17]} == length + 28,
                     "IPv4 total length matches UDP payload");
            tb_check(frame_bytes[net][30] == 8'he0 && frame_bytes[net][31] == 8'he0 &&
                     {frame_bytes[net][32],frame_bytes[net][33]} == 16'h1000 + vl,
                     "destination IPv4 embeds VLID");
            tb_check({frame_bytes[net][34],frame_bytes[net][35]} == 1000 + vl &&
                     {frame_bytes[net][36],frame_bytes[net][37]} == 2000 + vl,
                     "UDP ports match VL configuration");
            tb_check({frame_bytes[net][38],frame_bytes[net][39]} == length + 8,
                     "UDP length matches application payload");
            for (position = 0; position < length; position = position + 1)
                if (frame_bytes[net][42+position] != payload_byte(seed, position))
                    payload_errors = payload_errors + 1;
            if (payload_errors != 0)
                $display("wire payload mismatch summary: net=%0d vl=%0d len=%0d errors=%0d first=%02x expected=%02x",
                         net, vl, length, payload_errors, frame_bytes[net][42], payload_byte(seed,0));
            tb_check(payload_errors == 0, "wire payload matches submitted application bytes");
            for (position = length; position < 17; position = position + 1)
                tb_check(frame_bytes[net][42+position] == 0, "short AFDX payload is zero padded");
            tb_check(frame_bytes[net][expected_wire_length-5] == expected_sequence,
                     "AFDX sequence number is immediately before FCS");

            sum = 0;
            for (position = 14; position < 34; position = position + 2)
                sum = sum + {frame_bytes[net][position],frame_bytes[net][position+1]};
            sum = sum[15:0] + sum[31:16];
            sum = sum[15:0] + sum[31:16];
            tb_check(sum[15:0] == 16'hffff, "IPv4 header checksum is independently valid");

            sum = 0;
            for (position = 26; position < 34; position = position + 2)
                sum = sum + {frame_bytes[net][position],frame_bytes[net][position+1]};
            sum = sum + 16'd17 + (length+8);
            for (position = 34; position < 42+length; position = position + 2)
                sum = sum + {frame_bytes[net][position],
                             (position+1 < 42+length) ? frame_bytes[net][position+1] : 8'h00};
            sum = sum[15:0] + sum[31:16];
            sum = sum[15:0] + sum[31:16];
            tb_check(sum[15:0] == 16'hffff, "UDP checksum including pseudo-header is independently valid");
        end
    endtask

    // Standalone IC/RM instance allows precise bad-sequence and SkewMax cases.
    reg ic_req = 0;
    reg [0:0] ic_vl = 0;
    reg ic_net = 0;
    reg [7:0] ic_sn = 0;
    reg [31:0] ic_now = 0;
    wire ic_resp, ic_accept, ic_stat_drop, rm_stat_drop;
    afdx_rx_integrity #(.N_VL(2), .VLW(1), .SKEW_MAX_US(50)) u_integrity (
        .clk(clk), .rst(rst), .now_us(ic_now), .req(ic_req), .vl(ic_vl), .net(ic_net), .sn(ic_sn),
        .resp(ic_resp), .accept(ic_accept), .stat_ic_drop(ic_stat_drop), .stat_rm_drop(rm_stat_drop), .init_busy()
    );

    task automatic integrity_request(input integer net, input [7:0] sequence_value,
                                     input [31:0] now_value, input logic expected_accept,
                                     input logic expected_ic_drop, input logic expected_rm_drop);
        integer timeout;
        begin
            @(negedge clk);
            ic_net = net;
            ic_sn = sequence_value;
            ic_now = now_value;
            ic_req = 1'b1;
            @(negedge clk);
            ic_req = 1'b0;
            timeout = 0;
            while (!ic_resp && timeout < 30) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            tb_check(timeout < 30, "IC/RM request completes");
            tb_check(ic_accept == expected_accept, "IC/RM acceptance decision");
            tb_check(ic_stat_drop == expected_ic_drop, "IC drop classification");
            tb_check(rm_stat_drop == expected_rm_drop, "RM duplicate classification");
        end
    endtask

    // Standalone RTC/tick instance.
    reg timer_load = 0;
    reg [31:0] timer_load_value = 0;
    wire timer_tick;
    wire [31:0] timer_rtc, timer_monotonic;
    afdx_timer #(.CLK_HZ(CLK_HZ)) u_timer (
        .clk(clk), .rst(rst), .rtc_load(timer_load), .rtc_load_val(timer_load_value),
        .tick_us(timer_tick), .rtc_us(timer_rtc), .monotonic_us(timer_monotonic)
    );

    integer base_a, base_b;
    integer message_index;
    integer errors_before;
    integer delivery_start_cycle;
    initial begin
        reset_core();

        // One dual-network message exercises frame construction, MAC/FCS,
        // parser, UDP/IP checks, redundancy elimination and application delivery.
        base_a = frame_count[0]; base_b = frame_count[1];
        delivery_start_cycle = cycle_count;
        send_message(0, 17, 8'h21);
        receive_message(0, 17, 8'h21, 8'd1);
        wait_frames(base_a+1, base_b+1);
        verify_latest_frame(0, 0, 17, 8'h21, 8'd1);
        verify_latest_frame(1, 0, 17, 8'h21, 8'd1);
        message_index = 0;
        while (cnt_rx_rm_drop != 1 && message_index < 100) begin
            @(negedge clk); message_index = message_index + 1;
        end
        tb_check(message_index < 100, "redundant B copy reaches RM decision");
        tb_check(cnt_tx_msgs == 1 && cnt_tx_frames == 1 && cnt_tx_frames_a == 1 && cnt_tx_frames_b == 1,
                 "one application message creates one A and one B frame");
        tb_check(cnt_rx_accepted == 1 && cnt_rx_delivered == 1 && cnt_rx_rm_drop == 1,
                 "first redundant copy is delivered and duplicate is eliminated");
        tb_check(cnt_rx_fcs_err == 0 && cnt_rx_hdr_err == 0 && cnt_rx_len_err == 0 &&
                 cnt_rx_csum_err == 0 && cnt_rx_ic_drop == 0,
                 "valid AFDX traffic has no protocol errors");
        tb_check(cycle_count-delivery_start_cycle < 20_000, "end-to-end latency is below 2 ms");

        // Two queued messages on one VL must observe BAG at each physical output.
        reset_core();
        send_message(0, 17, 8'h31);
        send_message(0, 17, 8'h41);
        receive_message(0, 17, 8'h31, 8'd1);
        receive_message(0, 17, 8'h41, 8'd2);
        wait_frames(2, 2);
        tb_check(frame_start_cycle[0][1]-frame_start_cycle[0][0] >= BAG_CYCLES,
                 "network A enforces configured BAG between SOFs");
        tb_check(frame_start_cycle[1][1]-frame_start_cycle[1][0] >= BAG_CYCLES,
                 "network B enforces configured BAG between SOFs");
        tb_check(frame_start_cycle[0][1]-frame_start_cycle[0][0] < BAG_CYCLES+30,
                 "BAG scheduling jitter is bounded to a few clocks");

        // Maximum legal payload produces exactly the IEEE 802.3 maximum frame.
        reset_core();
        send_message(1, 1471, 8'h52);
        receive_message(1, 1471, 8'h52, 8'd1);
        wait_frames(1, 1);
        verify_latest_frame(0, 1, 1471, 8'h52, 8'd1);
        verify_latest_frame(1, 1, 1471, 8'h52, 8'd1);
        tb_check(last_frame_len[0] == 1518 && last_frame_len[1] == 1518,
                 "1471-byte payload maps to 1518-byte frames on both networks");

        // 1472 bytes are drained and rejected atomically before frame creation.
        reset_core();
        send_message(1, 1472, 8'h62);
        repeat (300) @(negedge clk);
        tb_check(cnt_tx_msgs == 1 && cnt_tx_lmax_drop == 1 && cnt_tx_frames == 0,
                 "payload above 1471 bytes is counted and never framed");
        tb_check(frame_count[0] == 0 && frame_count[1] == 0 && tx_buffer_used == 0,
                 "oversize rollback leaves no wire traffic or buffer leak");

        // Exercise every value in the required 1..255 sequence space and wrap.
        reset_core();
        for (message_index = 0; message_index < 256; message_index = message_index + 1) begin
            send_message(0, 1, message_index);
            receive_message(0, 1, message_index,
                            (message_index == 255) ? 8'd1 : message_index+1);
            wait_frames(message_index+1, message_index+1);
            if (message_index == 0 || message_index == 254 || message_index == 255)
                verify_latest_frame(0, 0, 1, message_index,
                                    (message_index == 255) ? 8'd1 : message_index+1);
        end
        message_index = 0;
        while ((cnt_rx_rm_drop < 256 || tx_buffer_used != 0 || rx_buffer_used != 0 ||
                tx_buffer_messages != 0 || rx_buffer_messages != 0) && message_index < 500) begin
            @(negedge clk);
            message_index = message_index + 1;
        end
        $display("sequence summary: tx=%0d accepted=%0d delivered=%0d rm=%0d ic=%0d tx_used=%0d rx_used=%0d tx_msgs=%0d rx_msgs=%0d settle=%0d",
                 cnt_tx_frames, cnt_rx_accepted, cnt_rx_delivered, cnt_rx_rm_drop, cnt_rx_ic_drop,
                 tx_buffer_used, rx_buffer_used, tx_buffer_messages, rx_buffer_messages, message_index);
        tb_check(cnt_tx_frames == 256 && cnt_rx_accepted == 256 && cnt_rx_delivered == 256,
                 "all 256 sequence-wrap messages are conserved");
        tb_check(cnt_rx_rm_drop == 256 && cnt_rx_ic_drop == 0,
                 "redundancy and integrity state remain correct across 255-to-1 wrap");
        tb_check(tx_buffer_used == 0 && rx_buffer_used == 0 &&
                 tx_buffer_messages == 0 && rx_buffer_messages == 0,
                 "shared message pools fully reclaim after sustained traffic");

        // Reset for isolated IC/RM decision tests.
        reset_core();
        integrity_request(0, 8'd1,   1, 1, 0, 0); // first copy
        integrity_request(1, 8'd1,   2, 0, 0, 1); // redundant copy
        integrity_request(0, 8'd1,   3, 0, 1, 0); // duplicate on same network
        integrity_request(0, 8'd3,   4, 1, 0, 0); // +2 is legal
        integrity_request(0, 8'd6,   5, 0, 1, 0); // +3 violates IC
        integrity_request(1, 8'd2, 100, 1, 0, 0); // RM resync after SkewMax

        // RTC has a 1 us tick, supports absolute load, while monotonic time is unaffected.
        reset_core();
        repeat (35) @(negedge clk);
        tb_check(timer_monotonic >= 3 && timer_monotonic <= 4, "timer emits one tick per microsecond");
        errors_before = timer_monotonic;
        @(negedge clk); timer_load_value = 32'h1234_5678; timer_load = 1'b1;
        @(negedge clk); timer_load = 1'b0;
        tb_check(timer_rtc == 32'h1234_5678, "RTC absolute load takes effect");
        repeat (12) @(negedge clk);
        tb_check(timer_rtc > 32'h1234_5678, "loaded RTC resumes microsecond counting");
        tb_check(timer_monotonic > errors_before, "protocol monotonic time is not reset by RTC load");

        tb_finish("tb_afdx_protocol");
        $finish;
    end

    initial begin
        #50_000_000;
        $fatal(1, "tb_afdx_protocol timeout");
    end
endmodule
