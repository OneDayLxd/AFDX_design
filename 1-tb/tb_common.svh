integer tb_checks = 0;
integer tb_errors = 0;

task automatic tb_check(input logic condition, input string message);
    begin
        tb_checks = tb_checks + 1;
        if (!condition) begin
            tb_errors = tb_errors + 1;
            $display("ERROR [%0t] %s", $time, message);
        end
    end
endtask

function automatic [31:0] tb_crc32_byte(input [31:0] crc_in, input [7:0] data);
    integer bit_index;
    reg [31:0] value;
    begin
        value = crc_in ^ {24'd0, data};
        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
            value = value[0] ? ((value >> 1) ^ 32'hEDB88320) : (value >> 1);
        tb_crc32_byte = value;
    end
endfunction

task automatic tb_finish(input string test_name);
    begin
        if (tb_errors == 0)
            $display("%s: %0d checks, 0 errors -> PASS", test_name, tb_checks);
        else begin
            $display("%s: %0d checks, %0d errors -> FAIL", test_name, tb_checks, tb_errors);
            $fatal(1, "%s failed", test_name);
        end
    end
endtask
