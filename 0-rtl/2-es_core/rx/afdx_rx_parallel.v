/*
 * ================================================================
 * File Name    : afdx_rx_parallel.v
 * Module Name  : afdx_rx_parallel
 *
 * Function:
 *   A/B 并行解析和暂存，经共享 IC/RM 去重后仅复制有效副本到接收消息池。
 *   逐帧轮询描述符；消息池允许逐字节反压，成功提交后才确认 RM 交付状态。
 *   协议超时使用单调时钟；消息时间戳沿用 MAC 接收时间戳。
 *
 * Sub Module:
 *   afdx_rx_frontend (2)
 *   afdx_rx_integrity
 *
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_rx_parallel #(
    parameter FRAGMENT_ENABLE = 0,
    parameter N_VL        = 4096,
    parameter VLW         = 12,
    parameter MW          = 105,
    parameter NET_FROM_SA = 0,
    parameter UDP_CSUM_EN = 1,
    parameter IC_EN       = 1,
    parameter RM_EN       = 1,
    parameter SKEW_MAX_US = 10000
) (
    input  wire            clk,
    input  wire            rst,
    input  wire [31:0]     now_us,
    input  wire [15:0]     in_data,
    input  wire [1:0]      in_valid,
    input  wire [1:0]      in_last,
    input  wire [63:0]     in_ts,
    output wire [1:0]      in_ready,
    output reg  [VLW-1:0]  q_wr_vl,
    output wire            q_wr_valid,
    output wire [7:0]      q_wr_data,
    input  wire            q_wr_ready,
    output wire            q_wr_commit,
    output reg  [MW-1:0]   q_wr_meta,
    output wire            q_wr_abort,
    input  wire            q_wr_ovf,
    input  wire            q_wr_commit_ok,
    output wire            stat_ok,
    output wire [1:0]      stat_hdr_err,
    output wire [1:0]      stat_len_err,
    output wire [1:0]      stat_vl_unknown,
    output wire [1:0]      stat_csum_err,
    output wire            stat_full_drop,
    output wire            stat_ic_drop,
    output wire            stat_rm_drop,
    output wire            pending_valid,
    output wire            pending_ready
);
    wire [1:0] head_valid, pop, rd_start, rd_valid, rd_last, rd_ready;
    wire [2*VLW-1:0] head_vl;
    wire [2*MW-1:0] head_meta;
    wire [21:0] head_len;
    wire [15:0] rd_data;
    genvar n;
    generate for (n=0; n<2; n=n+1) begin : g_frontend
        afdx_rx_frontend #(
            .FRAGMENT_ENABLE (FRAGMENT_ENABLE),
            .N_VL            (N_VL),
            .VLW             (VLW),
            .MW              (MW),
            .NET_FROM_SA     (NET_FROM_SA),
            .UDP_CSUM_CHECK  (UDP_CSUM_EN)
        ) u_frontend (
            .clk             (clk),
            .rst             (rst),
            .in_data         (in_data[8*n+:8]),
            .in_valid        (in_valid[n]),
            .in_last         (in_last[n]),
            .in_net          (n != 0),
            .in_ts           (in_ts[32*n+:32]),
            .in_ready        (in_ready[n]),
            .head_valid      (head_valid[n]),
            .head_vl         (head_vl[VLW*n+:VLW]),
            .head_meta       (head_meta[MW*n+:MW]),
            .head_len        (head_len[11*n+:11]),
            .pop             (pop[n]),
            .rd_start        (rd_start[n]),
            .rd_data         (rd_data[8*n+:8]),
            .rd_valid        (rd_valid[n]),
            .rd_last         (rd_last[n]),
            .rd_ready        (rd_ready[n]),
            .stat_hdr_err    (stat_hdr_err[n]),
            .stat_len_err    (stat_len_err[n]),
            .stat_vl_unknown (stat_vl_unknown[n]),
            .stat_csum_err   (stat_csum_err[n])
        );
    end endgenerate

    localparam IDLE=0, REQUEST=1, WAIT_IC=2, START=3, COPY=4,
               COMMIT=5, ACK=6, ABORT=7, CANCEL=8, RELEASE=9;
    reg [3:0] state;
    reg selected, round_robin;
    wire choose = head_valid[round_robin] ? round_robin : !round_robin;
    wire ic_resp, ic_accept;
    // 一次只允许一个待提交裁决，避免同 VL 的另一网抢在提交前重复获准。
    afdx_rx_integrity #(
        .N_VL         (N_VL),
        .VLW          (VLW),
        .IC_EN        (IC_EN),
        .RM_EN        (RM_EN),
        .SKEW_MAX_US  (SKEW_MAX_US),
        .DEFER_COMMIT (1)
    ) u_ic (
        .clk           (clk),
        .rst           (rst),
        .now_us        (now_us),
        .req           (state == REQUEST),
        .vl            (q_wr_vl),
        .net           (q_wr_meta[8]),
        .sn            (q_wr_meta[7:0]),
        .resp          (ic_resp),
        .accept        (ic_accept),
        .stat_ic_drop  (stat_ic_drop),
        .stat_rm_drop  (stat_rm_drop),
        .init_busy     (),
        .delivery_done ((state == ACK && q_wr_commit_ok) || state == CANCEL),
        .delivery_ok   (state == ACK && q_wr_commit_ok)
    );
    assign pop           = state == RELEASE ? (selected ? 2'b10 : 2'b01) : 2'b00;
    assign rd_start      = state == START   ? (selected ? 2'b10 : 2'b01) : 2'b00;
    assign rd_ready      = state == COPY && q_wr_ready ? (selected ? 2'b10 : 2'b01) : 2'b00;
    assign q_wr_data     = selected ? rd_data[15:8] : rd_data[7:0];
    assign q_wr_valid    = state == COPY && rd_valid[selected];
    assign q_wr_commit   = state == COMMIT;
    assign q_wr_abort    = state == ABORT;
    assign stat_ok       = state == ACK && q_wr_commit_ok;
    assign stat_full_drop = state == CANCEL;
    assign pending_valid = |head_valid;
    assign pending_ready = state == IDLE;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            selected <= 0;
            round_robin <= 0;
            q_wr_vl <= 0;
            q_wr_meta <= 0;
        end else begin
            case (state)
                IDLE: if (|head_valid) begin
                    selected <= choose;
                    q_wr_vl <= choose ? head_vl[VLW+:VLW] : head_vl[0+:VLW];
                    q_wr_meta <= choose ? head_meta[MW+:MW] : head_meta[0+:MW];
                    state <= REQUEST;
                end
                REQUEST: state <= WAIT_IC;
                WAIT_IC: if (ic_resp) state <= ic_accept ? START : RELEASE;
                START: state <= COPY;
                COPY: begin
                    if (q_wr_ovf) state <= ABORT;
                    else if (rd_valid[selected] && q_wr_ready && rd_last[selected])
                        state <= COMMIT;
                end
                COMMIT: state <= ACK;
                ACK: state <= q_wr_commit_ok ? RELEASE : ABORT;
                ABORT: state <= CANCEL;
                CANCEL: state <= RELEASE;
                RELEASE: begin
                    round_robin <= !selected;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
