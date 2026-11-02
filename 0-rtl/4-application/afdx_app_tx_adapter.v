/*
 * ================================================================
 * File Name    : afdx_app_tx_adapter.v
 * Module Name  : afdx_app_tx_adapter
 *
 * Function:
 *   应用消息格式为两字节大端 VLID、1..MAX_LEN 字节载荷，last 结束消息。
 *   VLID 并行比较后寄存，再经平衡仲裁树映射配置索引；载荷同步延迟。
 *   未知 VLID、截断首部及缓存异常均整条丢弃，不向核心发布部分消息。
 *
 * Sub Module:
 *   afdx_tx_vl_id_rom
 *   afdx_rr_arbiter
 *   afdx_app_ingress
 *   afdx_ip_fragment_tx
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_app_tx_adapter #(
    parameter FRAGMENT_ENABLE = 0,
    parameter VLW      = 7,
    parameter N_VL     = 128,
    parameter SLOT_LOG = 4,
    parameter MAX_LEN  = 1471
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [7:0]     in_data,
    input  wire           in_valid,
    input  wire           in_last,
    output wire [VLW-1:0] out_vl,
    output wire [7:0]     out_data,
    output wire           out_valid,
    output wire           out_last,
    output wire [79:0]    out_fragment,
    input  wire           out_ready,
    output wire           drop_pulse,
    output wire [2:0]     drop_reason,
    output wire [31:0]    drop_count
);
    localparam ID_HIGH = 0, ID_LOW = 1, PAYLOAD = 2;
    reg [1:0] state;
    reg [7:0] id_high;
    wire [N_VL-1:0] id_match;
    reg [N_VL-1:0] match_q;
    wire index_valid;
    wire [VLW-1:0] index;
    reg [VLW:0] index_q;
    genvar v;
    generate for (v = 0; v < N_VL; v = v + 1) begin : g_id
        localparam [VLW-1:0] CONFIG_INDEX = v;
        wire [15:0] configured_id;
        afdx_tx_vl_id_rom #(
            .VLW (VLW)
        ) u_id (
            .idx  (CONFIG_INDEX),
            .vlid (configured_id),
            .lmax ()
        );
        assign id_match[v] = {id_high, in_data} == configured_id;
    end endgenerate
    afdx_rr_arbiter #(
        .N (N_VL),
        .W (VLW)
    ) u_lookup (
        .request     (match_q),
        .start_index ({VLW{1'b0}}),
        .valid       (index_valid),
        .index       (index)
    );
    reg [7:0] data_q;
    reg valid_q, last_q, malformed_q;
    // 索引多留一位，未知 VLID/空载荷用越界索引复用入口的整条拒绝逻辑。
    wire [VLW:0] ingress_vl = malformed_q ? {1'b1, {VLW{1'b0}}} : index_q;
    wire [VLW:0] buffered_vl;
    wire [7:0] buffered_data;
    wire buffered_valid, buffered_last, buffered_ready;
    wire [15:0] buffered_len;
    wire [31:0] buffered_sum;
    wire [10:0] lmax;
    afdx_tx_vl_id_rom #(
        .VLW (VLW)
    ) u_lmax (
        .idx  (buffered_vl[VLW-1:0]),
        .vlid (),
        .lmax (lmax)
    );
    generate if (FRAGMENT_ENABLE != 0) begin : g_fragment
        afdx_ip_fragment_tx #(
            .VLW (VLW)
        ) u_fragment (
            .clk          (clk),
            .rst          (rst),
            .in_vl        (buffered_vl[VLW-1:0]),
            .in_data      (buffered_data),
            .in_valid     (buffered_valid),
            .in_last      (buffered_last),
            .in_len       (buffered_len),
            .in_sum       (buffered_sum),
            .in_lmax      (lmax),
            .in_ready     (buffered_ready),
            .out_vl       (out_vl),
            .out_data     (out_data),
            .out_valid    (out_valid),
            .out_last     (out_last),
            .out_fragment (out_fragment),
            .out_ready    (out_ready)
        );
    end else begin : g_no_fragment
        assign out_vl = buffered_vl[VLW-1:0];
        assign out_data = buffered_data;
        assign out_valid = buffered_valid;
        assign out_last = buffered_last;
        assign out_fragment = 0;
        assign buffered_ready = out_ready;
    end endgenerate
    always @(posedge clk) begin
        if (rst) begin
            state <= ID_HIGH;
            id_high <= 0;
            match_q <= 0;
            index_q <= {1'b1, {VLW{1'b0}}};
            data_q <= 0;
            valid_q <= 0;
            last_q <= 0;
            malformed_q <= 0;
        end else begin
            index_q <= index_valid ? {1'b0, index} : {1'b1, {VLW{1'b0}}};
            valid_q <= 0;
            last_q <= 0;
            malformed_q <= 0;
            if (in_valid) begin
                data_q <= in_data;
                case (state)
                    ID_HIGH: begin
                        id_high <= in_data;
                        state <= in_last ? ID_HIGH : ID_LOW;
                        if (in_last) begin
                            valid_q <= 1;
                            last_q <= 1;
                            malformed_q <= 1;
                        end
                    end
                    ID_LOW: begin
                        match_q <= id_match;
                        state <= in_last ? ID_HIGH : PAYLOAD;
                        if (in_last) begin
                            valid_q <= 1;
                            last_q <= 1;
                            malformed_q <= 1;
                        end
                    end
                    PAYLOAD: begin
                        valid_q <= 1;
                        last_q <= in_last;
                        if (in_last) state <= ID_HIGH;
                    end
                    default: state <= ID_HIGH;
                endcase
            end
        end
    end
    afdx_app_ingress #(
        .VLW      (VLW + 1),
        .N_VL     (N_VL),
        .SLOT_LOG (SLOT_LOG),
        .MAX_LEN  (MAX_LEN)
    ) u_buffer (
        .clk          (clk),
        .rst          (rst),
        .in_vl        (ingress_vl),
        .in_data      (data_q),
        .in_valid     (valid_q),
        .in_last      (last_q),
        .out_vl       (buffered_vl),
        .out_data     (buffered_data),
        .out_valid    (buffered_valid),
        .out_last     (buffered_last),
        .out_ready    (buffered_ready),
        .out_len      (buffered_len),
        .out_sum      (buffered_sum),
        .drop_pulse   (drop_pulse),
        .drop_reason  (drop_reason),
        .drop_count   (drop_count),
        .accept_count ()
    );
endmodule
