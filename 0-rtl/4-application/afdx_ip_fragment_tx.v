/*
 * ================================================================
 * File Name    : afdx_ip_fragment_tx.v
 * Module Name  : afdx_ip_fragment_tx
 *
 * Function:
 *   将完整应用消息切为可独立排队的 IP 分片载荷，首片预留 UDP 首部。
 *   非末片 IP 数据长度按八字节对齐；每片继续经过正常 VL/BAG 调度。
 *   80 位内部描述符保存分片标志、偏移、ID、UDP 长度及全消息载荷和。
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_ip_fragment_tx #(
    parameter VLW = 7
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [VLW-1:0] in_vl,
    input  wire [7:0]     in_data,
    input  wire           in_valid,
    input  wire           in_last,
    input  wire [15:0]    in_len,
    input  wire [31:0]    in_sum,
    input  wire [10:0]    in_lmax,
    output wire           in_ready,
    output wire [VLW-1:0] out_vl,
    output wire [7:0]     out_data,
    output wire           out_valid,
    output wire           out_last,
    output wire [79:0]    out_fragment,
    input  wire           out_ready
);
    localparam IDLE = 0, STREAM = 1, NEXT = 2;
    reg [1:0] state;
    reg fragmented;
    reg [15:0] remaining, udp_length, datagram_id, chunk_length, position;
    reg [15:0] aligned_bytes;
    reg [12:0] offset;
    reg [31:0] payload_sum;
    wire [15:0] aligned_lmax = {5'd0, in_lmax[10:3], 3'b000};
    assign out_vl = in_vl;
    assign out_data = in_data;
    assign out_valid = !rst && state == STREAM && in_valid;
    assign in_ready = !rst && state == STREAM && out_ready;
    assign out_last = out_valid && position + 1'b1 == chunk_length;
    // {fragmented, MF, offset[12:0], ID[15:0], UDP length[15:0], sum[31:0], reserved}
    assign out_fragment = fragmented ?
        {1'b1, (remaining > chunk_length), offset, datagram_id,
         udp_length, payload_sum, 1'b0} : 80'd0;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            fragmented <= 0;
            remaining <= 0;
            udp_length <= 0;
            datagram_id <= 0;
            chunk_length <= 0;
            position <= 0;
            aligned_bytes <= 0;
            offset <= 0;
            payload_sum <= 0;
        end else begin
            case (state)
                IDLE: if (in_valid) begin
                    fragmented <= in_len > in_lmax && aligned_lmax >= 16;
                    remaining <= in_len;
                    udp_length <= in_len + 16'd8;
                    payload_sum <= in_sum;
                    offset <= 0;
                    position <= 0;
                    aligned_bytes <= aligned_lmax;
                    // 极小 Lmax 不具备首片空间，交由核心原有 Lmax 检查整条拒绝。
                    chunk_length <= in_len > in_lmax && aligned_lmax >= 16 ?
                                    aligned_lmax - 16'd8 : in_len;
                    state <= STREAM;
                end
                STREAM: if (out_valid && out_ready) begin
                    if (out_last) begin
                        position <= 0;
                        remaining <= remaining - chunk_length;
                        if (remaining == chunk_length) begin
                            datagram_id <= datagram_id + 1'b1;
                            state <= IDLE;
                        end else begin
                            offset <= offset + (aligned_bytes >> 3);
                            state <= NEXT;
                        end
                    end else position <= position + 1'b1;
                end
                NEXT: begin
                    chunk_length <= remaining > aligned_bytes ? aligned_bytes : remaining;
                    state <= STREAM;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
