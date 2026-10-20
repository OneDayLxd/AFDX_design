`timescale 1ns/1ps

module tb_afdx_top #(
    parameter BYTE_CYCLES = 8,
    parameter DUAL_BUS = 1
);
    `include "tb_common.svh"

    reg clk = 1'b0;
    reg reset = 1'b1;
    always #5 clk = ~clk;

    wire [7:0] tx_data;
    wire tx_valid;
    wire tx_envelope;
    wire [7:0] rx_data = tx_data;
    wire rx_valid = tx_valid;
    wire rx_envelope = tx_envelope;

    AFDX_top #(
        .CLK_HZ(100_000_000), .BUS_BYTE_CYCLES(BYTE_CYCLES),
        .APP_GEN_EN(1), .DUAL_NET_ON_BUS(DUAL_BUS), .SKEW_MAX_US(100),
        .UDP_CSUM_EN(1), .TX_POOL_LOG(2), .RX_POOL_LOG(2),
        .BLOCK_AW(8), .TX_MAX_MSGS(16), .MGMT_EN(1)
    ) dut (
        .clk(clk), .reset(reset), .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_envelope), .rx_ready(rx_envelope)
    );

    integer cycle_count = 0;
    integer frame_count = 0;
    integer frame_position = 0;
    integer previous_valid_cycle = 0;
    integer spacing_errors = 0;
    integer envelope_errors = 0;
    integer network_a_frames = 0;
    integer network_b_frames = 0;
    integer min_frame_length = 2000;
    integer max_frame_length = 0;
    integer first_wire_cycle = 0;
    integer last_wire_cycle = 0;
    reg [31:0] frame_crc = 32'hffff_ffff;
    bit frame_active = 0;
    byte unsigned frame_capture [0:1599];

    always @(posedge clk) begin
        cycle_count = cycle_count + 1;
        if (reset || dut.rst) begin
            frame_count = 0;
            frame_position = 0;
            previous_valid_cycle = 0;
            spacing_errors = 0;
            envelope_errors = 0;
            network_a_frames = 0;
            network_b_frames = 0;
            min_frame_length = 2000;
            max_frame_length = 0;
            first_wire_cycle = 0;
            last_wire_cycle = 0;
            frame_crc = 32'hffff_ffff;
            frame_active = 0;
        end else begin
            if (tx_valid) begin
                if (!tx_envelope) envelope_errors = envelope_errors + 1;
                if (!frame_active) begin
                    frame_active = 1;
                    frame_position = 0;
                    frame_crc = 32'hffff_ffff;
                    previous_valid_cycle = cycle_count;
                    if (first_wire_cycle == 0) first_wire_cycle = cycle_count;
                end else begin
                    if (cycle_count-previous_valid_cycle != BYTE_CYCLES) spacing_errors = spacing_errors + 1;
                    previous_valid_cycle = cycle_count;
                end
                if (frame_position >= 1600) $fatal(1, "top-level frame exceeds capture buffer");
                frame_capture[frame_position] = tx_data;
                frame_position = frame_position + 1;
                frame_crc = tb_crc32_byte(frame_crc, tx_data);
            end
            if (frame_active && !tx_envelope && !tx_valid) begin
                tb_check(frame_position >= 64 && frame_position <= 1518,
                         "top-level frame obeys IEEE/AFDX length bounds");
                tb_check(frame_crc == 32'hdebb20e3, "top-level frame FCS is valid");
                tb_check(frame_capture[0] == 8'h03 && frame_capture[12] == 8'h08 &&
                         frame_capture[13] == 8'h00,
                         "top-level frame carries AFDX multicast IPv4 encapsulation");
                if (frame_capture[11][7:5] == 3'b001) network_a_frames = network_a_frames + 1;
                else if (frame_capture[11][7:5] == 3'b010) network_b_frames = network_b_frames + 1;
                else tb_check(1'b0, "top-level source MAC has a valid A/B interface marker");
                if (frame_position < min_frame_length) min_frame_length = frame_position;
                if (frame_position > max_frame_length) max_frame_length = frame_position;
                frame_count = frame_count + 1;
                last_wire_cycle = cycle_count;
                frame_active = 0;
            end
        end
    end

    integer timeout;
    integer observation_start;
    initial begin
        repeat (8) @(negedge clk);
        reset = 1'b0;
        while (dut.rst) @(negedge clk);
        observation_start = cycle_count;

        timeout = 0;
        while ((frame_count < 8 || dut.cnt_sink_msgs < 4) && timeout < 200_000) begin
            @(negedge clk);
            timeout = timeout + 1;
        end
        tb_check(timeout < 200_000, "top-level self-generated loopback traffic completes within 2 ms");
        tb_check(frame_count >= 8, "single MAC bus carries multiple A/B frames");
        tb_check(network_a_frames > 0 && (DUAL_BUS ? network_b_frames > 0 : network_b_frames == 0),
                 "bus carries the configured networks");
        tb_check(spacing_errors == 0, "successive valid bytes obey configured pacing");
        tb_check(envelope_errors == 0, "tx_valid is always inside the frame envelope");
        tb_check(min_frame_length >= 64 && max_frame_length <= 1518,
                 "observed top-level frame-length range is legal");
        tb_check(dut.cnt_rx_fcs_err == 0 && dut.cnt_rx_mac_len_err == 0 &&
                 dut.cnt_rx_hdr_err == 0 && dut.cnt_rx_len_err == 0 &&
                 dut.cnt_rx_csum_err == 0 && dut.cnt_rx_ic_drop == 0,
                 "top-level loopback has no MAC, header, length, checksum or IC errors");
        tb_check(dut.cnt_rx_delivered == dut.cnt_sink_msgs && dut.cnt_sink_msgs >= 4,
                 "all accepted top-level messages reach the application sink");
        tb_check(DUAL_BUS ? dut.cnt_rx_rm_drop > 0 : dut.cnt_rx_rm_drop == 0,
                 "redundancy elimination matches the configured bus mode");
        tb_check((last_wire_cycle-first_wire_cycle) < 200_000,
                 "observed traffic interval remains within the 2 ms requirement");

        $display("TOP_METRIC frames=%0d network_a=%0d network_b=%0d delivered=%0d rm_drops=%0d min_len=%0d max_len=%0d",
                 frame_count, network_a_frames, network_b_frames, dut.cnt_sink_msgs,
                 dut.cnt_rx_rm_drop, min_frame_length, max_frame_length);
        tb_finish("tb_afdx_top");
        $finish;
    end

    initial begin
        #3_000_000;
        $fatal(1, "tb_afdx_top timeout");
    end
endmodule
