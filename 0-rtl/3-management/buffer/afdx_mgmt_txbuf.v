/*
 * ================================================================
 * File Name    : afdx_mgmt_txbuf.v
 * Module Name  : afdx_mgmt_txbuf
 *
 * Function:
 *   管理响应发送缓冲：引擎把一条 UDP 有效载荷写入 2 KB BRAM 后 post（长度、UDP 端点、
 *   SOF 时间戳标记），流式读出到应用/管理仲裁器。busy 从 post 持续到末字节被接受。
 *   只有一个缓冲：管理调度器在 busy 期间不启动会写缓冲的新作业。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_mgmt_txbuf #(
    parameter TX_VLW = 3,
    parameter MGMT_TX_VL = 0
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              wr_en,
    input  wire [10:0]       wr_addr,
    input  wire [7:0]        wr_data,
    input  wire              post,
    input  wire [10:0]       post_len,          // 1..1471
    input  wire [15:0]       post_sport,
    input  wire [15:0]       post_dport,
    input  wire              post_tag,
    output wire              busy,
    output wire [TX_VLW-1:0] tx_vl,
    output wire              tx_valid,
    output wire [7:0]        tx_data,
    output wire              tx_last,
    output wire [33:0]       tx_ep,
    input  wire              tx_ready,
    output reg  [31:0]       cnt_sent
);
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] mem [0:2047];
    reg [10:0] fetch_idx, len;
    reg active, rd_pending;
    reg [7:0] rd_q;
    reg [33:0] ep;
    // 三项输出缓冲：RAM 读出延迟一拍；预取只看“缓冲项 + 在途读”，不依赖同拍下游 ready，
    // 下游每拍接受时仍可每拍输出一字节。
    reg [8:0] out0, out1, out2;                  // {last, data}
    reg [1:0] count;
    wire take = count != 2'd0 && tx_ready;
    wire fetch = active && fetch_idx != len && ({1'b0, count} + {2'd0, rd_pending}) < 3'd3;
    always @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
        rd_q <= mem[fetch_idx];
    end
    // busy == active || count != 0，改为寄存器：post 接受时置位，末字节被接受时清零
    // （末字节在 out0 时已无在途读、count == 1）。post 的装载使能不再经过 count 译码。
    reg busy_q;
    assign busy = busy_q;
    assign tx_vl = MGMT_TX_VL[TX_VLW-1:0];
    assign tx_valid = count != 2'd0;
    assign {tx_last, tx_data} = out0;
    assign tx_ep = ep;
    reg pending_last;
    always @(posedge clk) begin
        if (rst) begin
            active <= 1'b0; rd_pending <= 1'b0; count <= 2'd0; fetch_idx <= 11'd0;
            len <= 11'd0; cnt_sent <= 32'd0; ep <= 34'd0;
            pending_last <= 1'b0; busy_q <= 1'b0;
        end else begin
            if (post && !busy_q) busy_q <= 1'b1;
            else if (take && tx_last) busy_q <= 1'b0;
            if (post && !busy) begin
                active <= 1'b1; fetch_idx <= 11'd0; len <= post_len;
                ep <= {post_tag, 1'b1, post_sport, post_dport};
            end
            rd_pending <= fetch;
            pending_last <= fetch && fetch_idx == len - 1'b1;
            if (fetch) begin
                fetch_idx <= fetch_idx + 1'b1;
                if (fetch_idx == len - 1'b1) active <= 1'b0;
            end
            // 出队与入队（上一拍发起的读）
            case ({rd_pending, take})
                2'b10: begin
                    if (count == 2'd0) out0 <= {pending_last, rd_q};
                    else if (count == 2'd1) out1 <= {pending_last, rd_q};
                    else out2 <= {pending_last, rd_q};
                    count <= count + 1'b1;
                end
                2'b01: begin out0 <= out1; out1 <= out2; count <= count - 1'b1; end
                2'b11: begin
                    out0 <= (count == 2'd1) ? {pending_last, rd_q} : out1;
                    if (count == 2'd2) out1 <= {pending_last, rd_q};
                    else out1 <= out2;
                    if (count == 2'd3) out2 <= {pending_last, rd_q};
                end
                default: ;
            endcase
            if (take && tx_last) cnt_sent <= cnt_sent + 1'b1;
        end
    end
endmodule
