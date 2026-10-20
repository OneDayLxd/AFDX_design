/*
 * ================================================================
 * File Name    : afdx_math.vh
 * Module Name  : none
 *
 * Function:
 *   在模块内部包含；Verilog-2001 常量函数（不依赖 2005/SystemVerilog 的 $clog2）。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

function integer afdx_clog2;
    input integer value;
    integer remaining;
    begin
        remaining = value - 1;
        afdx_clog2 = 0;
        while (remaining > 0) begin
            remaining = remaining >> 1;
            afdx_clog2 = afdx_clog2 + 1;
        end
        if (afdx_clog2 == 0) afdx_clog2 = 1;
    end
endfunction
