`timescale 1ns/1ps
`include "afdx_snmp_mib.vh"

// Change both the OID and every source word each clock. This checks group
// boundaries, byte order and the sampling cycle of the five-clock interface.
module tb_afdx_snmp_values;
    `include "tb_common.svh"
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;
    reg [6:0] val_k = 0;
    reg [511:0] ld = 0;
    reg [863:0] rtc = 0;
    reg [703:0] ft = 0;
    wire [63:0] val_data;
    afdx_snmp_values dut (
        .clk(clk), .rst(rst), .val_k(val_k), .val_inst(16'd1), .val_data(val_data),
        .stats_clear(1'b0), .dp(800'd0), .uptime_ticks(32'd0), .mono_us(32'd0),
        .cfg_generation(32'd0), .cfg_active_bank(1'b0), .cfg_last_result(32'd0), .cfg_budget(32'd0),
        .cfg_m_rd_idx(), .cfg_m_rd_shadow(), .cfg_vlid(16'd0), .cfg_bag_us(20'd0),
        .cfg_lmax(11'd0), .cfg_net_en(2'd0), .cfg_rom_lmax(11'd0),
        .ld(ld), .rtc64(64'd0), .rtc(rtc), .ft(ft));

    reg [6:0] sampled_k;
    reg [63:0] expected [0:3];
    integer observed = 0;
    always @(posedge clk) begin
        sampled_k <= val_k;
        // Source buses are sampled one clock after the OID.
        if (sampled_k == `MIB_RTCTIME || sampled_k == `MIB_RTCSET)
            expected[0] <= 64'd0; // These objects use the separate rtc64 input.
        else if (sampled_k >= `MIB_LDSTATE && sampled_k <= `MIB_LDAPPCRC)
            expected[0] <= {32'd0, ld[(sampled_k-`MIB_LDSTATE)*32 +: 32]};
        else if (sampled_k >= `MIB_RTCTIMELOW && sampled_k <= `MIB_RTCLASTSYNCMONO)
            expected[0] <= {32'd0, rtc[(sampled_k-`MIB_RTCTIMELOW)*32 +: 32]};
        else if (sampled_k >= `MIB_TRAPMASK && sampled_k <= `MIB_TFTPERRORS)
            expected[0] <= {32'd0, ft[(sampled_k-`MIB_TRAPMASK)*32 +: 32]};
        else expected[0] <= 64'd0;
        expected[1] <= expected[0]; expected[2] <= expected[1]; expected[3] <= expected[2];
        if (rst) observed = 0;
        else begin
            observed = observed + 1;
            #1;
            if (observed > 5)
                tb_check(val_data === expected[3], "OID, source sample and byte lanes remain aligned");
        end
    end

    integer cycle, word_i;
    initial begin
        repeat (40) @(negedge clk);
        rst = 0;
        for (cycle = 0; cycle < 1024; cycle = cycle + 1) begin
            // Sweep all RTC words, then alternate unrelated groups/invalid OIDs.
            if (cycle < 54) val_k = `MIB_RTCTIMELOW + (cycle % 27);
            else case (cycle % 4)
                0: val_k = `MIB_RTCTIMELOW + $urandom_range(0, 26);
                1: val_k = `MIB_LDSTATE + $urandom_range(0, 15);
                2: val_k = `MIB_TRAPMASK + $urandom_range(0, 21);
                3: val_k = 7'd127;
            endcase
            for (word_i = 0; word_i < 27; word_i = word_i + 1)
                rtc[word_i*32 +: 32] = $urandom;
            for (word_i = 0; word_i < 16; word_i = word_i + 1)
                ld[word_i*32 +: 32] = $urandom;
            for (word_i = 0; word_i < 22; word_i = word_i + 1)
                ft[word_i*32 +: 32] = $urandom;
            @(negedge clk);
        end
        repeat (6) @(negedge clk);
        tb_finish("tb_afdx_snmp_values");
        $finish;
    end
endmodule
