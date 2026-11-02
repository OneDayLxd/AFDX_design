/*
 * ================================================================
 * File Name    : afdx_app_rx_adapter.v
 * Module Name  : afdx_app_rx_adapter
 *
 * Function:
 *   接收配置索引还原实际 VLID，先输出大端 VLID，再输出有效载荷。
 *   两拍首部期间暂停内部消息流；外部为 valid-only，不使用 ready。
 *
 * Sub Module:
 *   afdx_rx_vl_id_rom
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_app_rx_adapter #(
    parameter VLW = 12
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [VLW-1:0] in_vl,
    input  wire [7:0]     in_data,
    input  wire           in_valid,
    input  wire           in_last,
    output wire           in_ready,
    output wire [7:0]     out_data,
    output wire           out_valid,
    output wire           out_last
);
    localparam ID_HIGH = 0, ID_LOW = 1, PAYLOAD = 2;
    reg [1:0] state;
    wire [15:0] vlid;
    afdx_rx_vl_id_rom #(
        .VLW  (VLW)
    ) u_id (
        .idx  (in_vl),
        .vlid (vlid)
    );
    assign out_valid = !rst && in_valid;
    assign out_data = state == ID_HIGH ? vlid[15:8] :
                      state == ID_LOW ? vlid[7:0] : in_data;
    assign out_last = state == PAYLOAD && out_valid && in_last;
    assign in_ready = !rst && state == PAYLOAD;
    always @(posedge clk) begin
        if (rst) state <= ID_HIGH;
        else if (in_valid) begin
            case (state)
                ID_HIGH: state <= ID_LOW;
                ID_LOW: state <= PAYLOAD;
                PAYLOAD: if (in_last) state <= ID_HIGH;
                default: state <= ID_HIGH;
            endcase
        end
    end
endmodule
