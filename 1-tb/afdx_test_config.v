`timescale 1ns/1ps

// Compact deterministic configuration used by dynamic protocol tests.  The
// production 128/4096-entry ROM is exercised separately by tb_afdx_config.
module afdx_tx_vl_rom #(parameter VLW = 2) (
    input wire [VLW-1:0] idx,
    output wire [15:0] vlid,
    output wire [19:0] bag_us,
    output wire [10:0] lmax,
    output wire [15:0] udp_src,
    output wire [15:0] udp_dst,
    output wire [1:0] net_en,
    output wire [4:0] partition
);
    assign vlid = 16'h1000 + idx;
    assign bag_us = (idx == 2) ? 20'd40 : (idx == 3) ? 20'd80 : 20'd20;
    assign lmax = (idx == 0 || idx == 1) ? 11'd1471 : (idx == 2) ? 11'd32 : 11'd128;
    assign udp_src = 16'd1000 + idx;
    assign udp_dst = 16'd2000 + idx;
    assign net_en = (idx == 2) ? 2'b01 : (idx == 3) ? 2'b10 : 2'b11;
    assign partition = idx + 1'b1;
endmodule

module afdx_rx_vl_lookup #(parameter VLW = 2) (
    input wire [15:0] vlid,
    output wire hit,
    output wire [VLW-1:0] idx
);
    assign hit = (vlid >= 16'h1000) && (vlid <= 16'h1003);
    assign idx = vlid - 16'h1000;
endmodule

module afdx_gen_rom #(parameter VLW = 2) (
    input wire [VLW-1:0] idx,
    output wire [19:0] period_us,
    output wire [19:0] phase_us,
    output wire [10:0] len
);
    assign phase_us = 20'd0;
    assign period_us = 40 + {idx, 4'b0000};
    assign len = 17 + idx;
endmodule

// 与生成的 afdx_tx_vl_id_rom 接口相同：复用上面的完整 ROM
module afdx_tx_vl_id_rom #(parameter VLW = 2) (
    input wire [VLW-1:0] idx,
    output wire [15:0] vlid,
    output wire [10:0] lmax
);
    afdx_tx_vl_rom #(.VLW(VLW)) u_rom (.idx(idx), .vlid(vlid), .bag_us(), .lmax(lmax),
        .udp_src(), .udp_dst(), .net_en(), .partition());
endmodule
