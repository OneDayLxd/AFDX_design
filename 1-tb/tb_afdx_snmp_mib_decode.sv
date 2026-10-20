`timescale 1ns/1ps
`include "afdx_snmp_mib.vh"
// Exercise the public lookup interface with synchronous request RAM. In
// particular, the final decoded arc must be committed before lookup starts.
module tb_afdx_snmp_mib_decode;
    `include "tb_common.svh"
    reg clk = 0, rst = 1, go = 0, mode = 0;
    always #5 clk = ~clk;
    reg [6:0] oid_len = 0;
    wire [10:0] rq_addr;
    reg [7:0] rq_data;
    reg [7:0] request_mem [0:127];
    always @(posedge clk) rq_data <= request_mem[rq_addr[6:0]];
    wire done, malformed, found;
    wire [1:0] exc;
    wire [6:0] res_k;
    wire [15:0] res_inst;
    afdx_snmp_mib dut (
        .clk(clk), .rst(rst), .go(go), .mode(mode), .oid_off(11'd0), .oid_len(oid_len),
        .rq_addr(rq_addr), .rq_data(rq_data), .done(done), .malformed(malformed),
        .found(found), .exc(exc), .res_k(res_k), .res_inst(res_inst));

    task lookup(input integer len, input bit next_mode, input bit bad,
                input bit hit, input [1:0] exception_code, input [6:0] key);
        integer cycles;
        begin
            @(negedge clk); oid_len = len; mode = next_mode; go = 1;
            @(negedge clk); go = 0;
            cycles = 0;
            while (!done && cycles < 20000) begin @(negedge clk); cycles = cycles + 1; end
            if (!done) $fatal(1, "MIB lookup timed out");
            tb_check(malformed == bad, "OID malformed status");
            if (!bad) begin
                tb_check(found == hit, "OID lookup hit status");
                if (hit) tb_check(res_k == key && res_inst == 0, "OID scalar lookup result");
                else tb_check(exc == exception_code, "OID lookup exception");
            end
        end
    endtask
    task sysname;
        begin
            request_mem[0] = 8'h2b; request_mem[1] = 6; request_mem[2] = 1;
            request_mem[3] = 2; request_mem[4] = 1; request_mem[5] = 1;
            request_mem[6] = 5; request_mem[7] = 0;
        end
    endtask
    integer i;
    initial begin
        for (i = 0; i < 128; i = i+1) request_mem[i] = 0;
        repeat (4) @(negedge clk); rst = 0;
        sysname;
        lookup(8, 0, 0, 1, 0, `MIB_SYSNAME);
        // First subidentifier splits at 40 and 80, including a one-byte OID.
        request_mem[0] = 39; lookup(1, 1, 0, 1, 0, `MIB_SYSDESCR);
        request_mem[0] = 40; lookup(1, 1, 0, 1, 0, `MIB_SYSDESCR);
        request_mem[0] = 80; lookup(1, 1, 0, 0, 3, 0);
        // 32-bit maximum subidentifier is legal; the next value overflows.
        request_mem[0] = 8'h8f;
        for (i = 1; i < 4; i = i+1) request_mem[i] = 8'hff;
        request_mem[4] = 8'h7f; lookup(5, 1, 0, 0, 3, 0);
        request_mem[0] = 8'h90;
        for (i = 1; i < 4; i = i+1) request_mem[i] = 8'h80;
        request_mem[4] = 0; lookup(5, 1, 1, 0, 0, 0);
        request_mem[0] = 8'h80; request_mem[1] = 1;
        lookup(2, 0, 1, 0, 0, 0); // Nonminimal subidentifier.
        request_mem[0] = 8'h81; lookup(1, 0, 1, 0, 0, 0); // Truncated.
        sysname;
        for (i = 8; i < 18; i = i+1) request_mem[i] = 0;
        lookup(15, 0, 0, 0, 2, 0); // Exactly 16 arcs.
        lookup(18, 0, 0, 0, 2, 0); // More than 16 must not overwrite old arcs.
        sysname;
        lookup(8, 0, 0, 1, 0, `MIB_SYSNAME); // Clean recovery after long/bad OIDs.
        tb_finish("tb_afdx_snmp_mib_decode");
        $finish;
    end
endmodule
