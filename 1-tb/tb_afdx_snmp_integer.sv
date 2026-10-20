`timescale 1ns/1ps
// Check emitted BER, including sign boundaries and the ninth Counter64 byte.
// Only the encoder's public interfaces are used; the expected representation
// is obtained by stripping redundant sign bytes from a software-style value.
module tb_afdx_snmp_integer;
    `include "tb_common.svh"
    reg clk = 0, rst = 1, go = 0;
    always #5 clk = ~clk;
    reg [31:0] request_id = 0;
    reg [4:0] vb_n = 1;
    reg [7:0] value_tag = 8'h46;
    reg [63:0] value = 0;
    reg oid_values = 0;
    wire [3:0] descriptor_addr;
    // Alternate zero/nonzero descriptors in one response, including a value
    // whose only set bit is above the emitted low seven bits.
    wire [63:0] oid_value = descriptor_addr == 1 ? 64'h8000000000000000 :
                           descriptor_addr == 2 ? 64'd7 : 64'd0;
    wire [138:0] descriptor = {1'b1, 11'd0, 7'd1, 7'd0, 16'd0,
                               (oid_values ? 3'd3 : 3'd1), value_tag,
                               (oid_values ? oid_value : value), 11'd0, 11'd0};
    wire wr_en, done, too_big;
    wire [10:0] wr_addr, out_len;
    wire [7:0] wr_data;
    afdx_snmp_enc dut (
        .clk(clk), .rst(rst), .go(go), .pdu_tag(8'ha2), .request_id(request_id),
        .err_status(8'd0), .err_index(8'd0), .vb_n(vb_n),
        .comm_echo(1'b0), .comm_off(11'd0), .comm_len(6'd1), .comm_reg({8'h70, 120'd0}),
        .max_len(11'd516), .vbo_raddr(descriptor_addr), .vbo_rdata(descriptor), .rq_addr(), .rq_data(8'h2b),
        .tb_wr_en(wr_en), .tb_wr_addr(wr_addr), .tb_wr_data(wr_data),
        .done(done), .too_big(too_big), .out_len(out_len));
    reg [7:0] packet [0:515];
    integer writes = 0;
    always @(posedge clk) if (wr_en) begin
        tb_check(wr_addr == writes, "sequential response writes");
        packet[wr_addr] = wr_data;
        writes = writes + 1;
    end
    integer cursor;
    task check_integer(input [7:0] tag, input [71:0] number);
        integer n, b;
        reg [7:0] high_byte, next_byte;
        reg trimming;
        begin
            n = 9; trimming = 1;
            while (n > 1 && trimming) begin
                high_byte = number >> (8*(n-1));
                next_byte = number >> (8*(n-2));
                if ((high_byte == 0 && !next_byte[7]) ||
                    (high_byte == 255 && next_byte[7])) n = n - 1;
                else trimming = 0;
            end
            tb_check(packet[cursor] == tag, "integer tag");
            tb_check(packet[cursor+1] == n, $sformatf("canonical length for %h", number));
            cursor = cursor + 2;
            for (b = n-1; b >= 0; b = b-1) begin
                tb_check(packet[cursor] == ((number >> (8*b)) & 255), "integer content");
                cursor = cursor + 1;
            end
        end
    endtask
    task run_oid_values;
        integer cycles, v, n, i;
        reg [47:0] prefix;
        begin
            @(negedge clk);
            oid_values = 1; vb_n = 4; request_id = 0; writes = 0; go = 1;
            @(negedge clk); go = 0;
            cycles = 0;
            while (!done && cycles < 2000) begin @(negedge clk); cycles = cycles + 1; end
            if (!done) $fatal(1, "OID encoder timed out");
            tb_check(!too_big && out_len == writes, "OID response complete");
            tb_check(packet[0] == 8'h30 && packet[1] == writes-2, "OID message BER length");
            tb_check(packet[8] == 8'ha2 && packet[9] == writes-10, "OID PDU BER length");
            cursor = 10;
            check_integer(8'h02, 0); check_integer(8'h02, 0); check_integer(8'h02, 0);
            tb_check(packet[cursor] == 8'h30 && packet[cursor+1] == writes-cursor-2,
                     "OID VarBindList BER length");
            cursor = cursor + 2;
            prefix = 48'h2b0601038518;
            for (v = 0; v < 4; v = v+1) begin
                n = (v == 1 || v == 2) ? 8 : 6;
                tb_check(packet[cursor] == 8'h30 && packet[cursor+1] == n+5,
                         "OID VarBind length follows its own descriptor");
                cursor = cursor + 2;
                tb_check(packet[cursor] == 6 && packet[cursor+1] == 1 && packet[cursor+2] == 8'h2b,
                         "OID response echoes object name");
                cursor = cursor + 3;
                tb_check(packet[cursor] == 6 && packet[cursor+1] == n, "OID value length");
                cursor = cursor + 2;
                for (i = 0; i < 6; i = i+1) begin
                    tb_check(packet[cursor] == prefix[47-8*i -: 8], "OID value prefix");
                    cursor = cursor + 1;
                end
                if (n == 8) begin
                    tb_check(packet[cursor] == 7 && packet[cursor+1] == (v == 2 ? 7 : 0),
                             "nonzero OID value suffix");
                    cursor = cursor + 2;
                end
            end
            tb_check(cursor == writes, "no missing or extra OID bytes");
            oid_values = 0;
        end
    endtask
    task run_case(input [31:0] rid, input [7:0] tag, input [63:0] val, input bit empty);
        integer cycles;
        reg [71:0] extended;
        begin
            @(negedge clk);
            request_id = rid; value_tag = tag; value = val; vb_n = empty ? 0 : 1;
            writes = 0; go = 1;
            @(negedge clk); go = 0;
            cycles = 0;
            while (!done && cycles < 2000) begin @(negedge clk); cycles = cycles + 1; end
            if (!done) $fatal(1, "encoder timed out");
            tb_check(!too_big && out_len == writes, "complete bounded response");
            tb_check(packet[0] == 8'h30 && packet[1] == writes-2, "outer BER length");
            tb_check(packet[8] == 8'ha2 && packet[9] == writes-10, "PDU BER length");
            cursor = 10;
            check_integer(8'h02, {{40{rid[31]}}, rid});
            check_integer(8'h02, 72'd0);
            check_integer(8'h02, 72'd0);
            tb_check(packet[cursor] == 8'h30 && packet[cursor+1] == writes-cursor-2,
                     "VarBindList BER length");
            cursor = cursor + 2;
            if (!empty) begin
                tb_check(packet[cursor] == 8'h30 && packet[cursor+1] == writes-cursor-2,
                         "VarBind BER length");
                cursor = cursor + 2;
                tb_check(packet[cursor] == 6 && packet[cursor+1] == 1 && packet[cursor+2] == 8'h2b,
                         "echoed OID");
                cursor = cursor + 3;
                extended = tag == 8'h02 ? {{40{val[31]}}, val[31:0]} : {8'd0, val};
                check_integer(tag, extended);
            end
            tb_check(cursor == writes, "no extra bytes or stale length");
        end
    endtask
    integer b, k;
    reg [63:0] boundary, random_value;
    initial begin
        repeat (4) @(negedge clk); rst = 0;
        run_oid_values;
        run_case(0, 8'h46, 0, 0);
        run_case(32'hffffffff, 8'h02, 64'hffffffffffffffff, 0);
        for (b = 1; b <= 8; b = b+1) begin
            boundary = 64'd1 << (8*b-1);
            run_case(32'h7fffffff, 8'h46, boundary-1, 0);
            run_case(32'h80000000, 8'h46, boundary, 0);
            if (b <= 4) begin
                run_case(32'hffffff80, 8'h02, -boundary, 0);
                run_case(32'hffffff7f, 8'h02, -boundary-1, 0);
            end
        end
        run_case(128, 8'h46, 64'hffffffffffffffff, 0);
        run_case(127, 8'h02, 0, 1);
        run_case(128, 8'h02, 0, 1);
        run_case(32'h80000000, 8'h02, 0, 1);
        for (k = 0; k < 100; k = k+1) begin
            random_value = {$random, $random};
            run_case($random, k[0] ? 8'h02 : 8'h46, random_value, k % 7 == 0);
        end
        tb_finish("tb_afdx_snmp_integer");
        $finish;
    end
endmodule
