`timescale 1ns/1ps
module tb_afdx_tx_cfg_pipeline;
    `include "tb_common.svh"
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1, we = 0, verify = 0;
    reg [2:0] idx = 0;
    reg [19:0] bag = 1000;
    reg [10:0] lmax = 1;
    reg [1:0] net = 1;
    wire busy, done, ok;
    wire [2:0] code, bad_idx;
    wire [31:0] budget;
    afdx_tx_cfg_ram #(.N_VL(8), .VLW(3)) dut (
        .clk(clk), .rst(rst), .g_idx(3'd0), .f_idx(3'd0), .builder_idle(1'b1),
        .m_rd_idx(3'd0), .m_rd_shadow(1'b1), .m_wr_en(we), .m_wr_idx(idx),
        .m_wr_bag_us(bag), .m_wr_lmax(lmax), .m_wr_net_en(net),
        .cmd_copy(1'b0), .cmd_verify(verify), .cmd_apply(1'b0), .cmd_rollback(1'b0),
        .busy(busy), .done(done), .verify_ok(ok), .verify_code(code),
        .verify_idx(bad_idx), .budget(budget)
    );
    integer round_id, i, expected, copies, k;
    task automatic run_verify;
        begin
            @(negedge clk); verify = 1;
            @(negedge clk); verify = 0;
            wait(done); @(negedge clk);
        end
    endtask
    initial begin
        repeat (4) @(negedge clk);
        rst = 0;
        wait(!busy);
        // Every scan changes BAG and copy count on adjacent entries. The
        // independent arithmetic model includes the last entry explicitly.
        for (round_id = 0; round_id < 16; round_id = round_id + 1) begin
            expected = 0;
            for (i = 0; i < 8; i = i + 1) begin
                @(negedge clk);
                idx = i; k = (i + round_id) % 8;
                bag = 1000 * (2 ** k);
                net = 1 + ((i + round_id) % 3);
                lmax = 1 + ((i + round_id) % 17);
                we = 1;
                copies = (net == 3) ? 2 : 1;
                expected = expected + 84 * (128000 / bag) * copies;
            end
            @(negedge clk); we = 0;
            repeat (2) @(negedge clk);
            run_verify();
            tb_check(ok && code == 0, "valid shadow configuration passes");
            tb_check(budget == expected, "all BAG/copy contributions including final VL are counted");
            run_verify();
            tb_check(ok && budget == expected, "back-to-back verification has no stale pipeline terms");
        end
        @(negedge clk); idx = 7; net = 0; we = 1;
        @(negedge clk); we = 0;
        repeat (2) @(negedge clk);
        run_verify();
        tb_check(!ok && code == 3 && bad_idx == 7, "last-entry error survives the extended pipeline");
        tb_finish("tb_afdx_tx_cfg_pipeline");
        $finish;
    end
    initial begin #1000000; $fatal(1, "configuration pipeline timeout"); end
endmodule
