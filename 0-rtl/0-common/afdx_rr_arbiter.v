/*
 * ================================================================
 * File Name    : afdx_rr_arbiter.v
 * Module Name  : afdx_rr_arbiter
 *                afdx_first16
 *                afdx_first16_nz
 *
 * Function:
 *   组合轮询选择：优先 start_index 及之后的请求，否则从 0 回绕。
 *   N<=16 时用移位掩码加按 4 位分组的最低置位查找，约三级 LUT4；
 *   更宽时使用两棵静态二叉优先树，深度随 log2(N) 增长。
 *
 * Sub Module:
 *   afdx_first16
 *   afdx_first16_nz
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_rr_arbiter #(
    parameter N = 8,
    parameter W = 3
) (
    input wire [N-1:0] request,
    input wire [W-1:0] start_index,
    output wire valid,
    output wire [W-1:0] index
);
    generate if (N <= 16) begin : g_small
        wire [15:0] padded = request;
        wire [15:0] after = padded & (16'hffff << start_index);
        wire after_any, request_any;
        wire [3:0] after_first, request_first;
        afdx_first16 u_after (
            .x     (after),
            .any   (after_any),
            .index (after_first)
        );
        afdx_first16 u_request (
            .x     (padded),
            .any   (request_any),
            .index (request_first)
        );
        wire [3:0] chosen = after_any ? after_first : (request_any ? request_first : 4'd0);
        wire [((W > 4) ? W : 4)-1:0] chosen_ext = chosen;
        assign valid = request_any;
        assign index = chosen_ext[W-1:0];
    end else begin : g_tree
        localparam PAD = 1 << W;
        wire any_v [1:2*PAD-1], after_v [1:2*PAD-1];
        wire [W-1:0] any_i [1:2*PAD-1], after_i [1:2*PAD-1];
        genvar g;
        for (g = PAD; g < 2*PAD; g = g + 1) begin : g_leaf
            localparam LEAF_NUMBER = g - PAD;
            localparam [W-1:0] LEAF = LEAF_NUMBER[W-1:0];
            if (g - PAD < N) begin : g_real
                assign any_v[g] = request[g-PAD];
                assign after_v[g] = request[g-PAD] && LEAF >= start_index;
            end else begin : g_pad
                assign any_v[g] = 1'b0;
                assign after_v[g] = 1'b0;
            end
            assign any_i[g] = LEAF;
            assign after_i[g] = LEAF;
        end
        for (g = 1; g < PAD; g = g + 1) begin : g_node
            assign any_v[g] = any_v[2*g] || any_v[2*g+1];
            assign after_v[g] = after_v[2*g] || after_v[2*g+1];
            assign any_i[g] = any_v[2*g] ? any_i[2*g] : any_i[2*g+1];
            assign after_i[g] = after_v[2*g] ? after_i[2*g] : after_i[2*g+1];
        end
        assign valid = any_v[1];
        assign index = after_v[1] ? after_i[1] : (any_v[1] ? any_i[1] : {W{1'b0}});
    end endgenerate
endmodule

// Lowest set bit of a 16-bit vector: per-nibble results, then nibble choice.
module afdx_first16 (
    input wire [15:0] x,
    output wire any,
    output wire [3:0] index
);
    wire [3:0] nonzero = {|x[15:12], |x[11:8], |x[7:4], |x[3:0]};
    afdx_first16_nz u_pick (
        .x       (x),
        .nonzero (nonzero),
        .any     (any),
        .index   (index)
    );
endmodule

// afdx_first16 with the per-nibble ORs supplied by the caller, typically
// registered alongside x so that they are not on the Q-to-pick path.
module afdx_first16_nz (
    input wire [15:0] x,
    input wire [3:0] nonzero,
    output wire any,
    output wire [3:0] index
);
    function [1:0] first4;
        input [2:0] nibble;
        first4 = nibble[0] ? 2'd0 : nibble[1] ? 2'd1 : nibble[2] ? 2'd2 : 2'd3;
    endfunction
    // Lane of the lowest set bit in a nonzero nibble (bit 3 is implied).
    wire [1:0] lane0 = first4(x[2:0]), lane1 = first4(x[6:4]);
    wire [1:0] lane2 = first4(x[10:8]), lane3 = first4(x[14:12]);
    assign any = |nonzero;
    assign index = nonzero[0] ? {2'd0, lane0} : nonzero[1] ? {2'd1, lane1} :
                   nonzero[2] ? {2'd2, lane2} : {2'd3, lane3};
endmodule
