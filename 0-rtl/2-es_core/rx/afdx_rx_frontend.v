/*
 * ================================================================
 * File Name    : afdx_rx_frontend.v
 * Module Name  : afdx_rx_frontend
 *
 * Function:
 *   单网独立完成 AFDX/IP/UDP 校验，将完整有效载荷暂存在默认四个 2KiB 槽中。
 *   描述符先供共享 IC/RM 裁决；重复副本可直接释放，不占用共享消息池。
 *   槽满时仅反压 MAC 的已校验帧 FIFO，不反压外部 GMII。
 *
 * Sub Module:
 *   afdx_rx_vl_lookup
 *   afdx_rx_parser
 *
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_rx_frontend #(
    parameter N_VL           = 4096,
    parameter VLW            = 12,
    parameter MW             = 105,
    parameter NET_FROM_SA    = 0,
    parameter UDP_CSUM_CHECK = 1,
    parameter SLOT_LOG       = 2
) (
    input  wire            clk,
    input  wire            rst,
    input  wire [7:0]      in_data,
    input  wire            in_valid,
    input  wire            in_last,
    input  wire            in_net,
    input  wire [31:0]     in_ts,
    output wire            in_ready,
    output wire            head_valid,
    output wire [VLW-1:0]  head_vl,
    output wire [MW-1:0]   head_meta,
    output wire [10:0]     head_len,
    input  wire            pop,
    input  wire            rd_start,
    output reg  [7:0]      rd_data,
    output reg             rd_valid,
    output reg             rd_last,
    input  wire            rd_ready,
    output wire            stat_hdr_err,
    output wire            stat_len_err,
    output wire            stat_vl_unknown,
    output wire            stat_csum_err
);
    localparam SLOTS  = 1 << SLOT_LOG;
    localparam DESC_W = VLW + MW + 11;
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] payload [0:SLOTS*2048-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [DESC_W-1:0] descriptor [0:SLOTS-1];
    reg [SLOT_LOG:0] wp, rp;
    reg [10:0] write_count;
    reg filling;
    wire full = (wp[SLOT_LOG] != rp[SLOT_LOG]) &&
                (wp[SLOT_LOG-1:0] == rp[SLOT_LOG-1:0]);
    wire empty = wp == rp;
    wire [VLW-1:0] write_vl;
    wire [MW-1:0] write_meta;
    wire [7:0] write_data;
    wire write_valid, local_commit, local_abort, parser_ready;
    wire local_req;
    reg local_resp;
    // 提交/回滚拍不启动下一帧，避免槽指针更新与新帧预约重叠。
    wire permit = (filling || !full) && !local_commit && !local_abort;
    assign in_ready = parser_ready && permit;
    wire [15:0] lookup_vlid;
    wire lookup_hit;
    wire [VLW-1:0] lookup_idx;
    afdx_rx_vl_lookup #(
        .VLW  (VLW)
    ) u_lookup (
        .vlid (lookup_vlid),
        .hit  (lookup_hit),
        .idx  (lookup_idx)
    );
    afdx_rx_parser #(
        .N_VL           (N_VL),
        .VLW            (VLW),
        .MW             (MW),
        .NET_FROM_SA    (NET_FROM_SA),
        .UDP_CSUM_CHECK (UDP_CSUM_CHECK)
    ) u_parser (
        .clk             (clk),
        .rst             (rst),
        .ts_us           (in_ts),
        .in_data         (in_data),
        .in_valid        (in_valid && permit),
        .in_last         (in_last),
        .in_net          (in_net),
        .in_ready        (parser_ready),
        .lk_vlid         (lookup_vlid),
        .lk_hit          (lookup_hit),
        .lk_idx          (lookup_idx),
        .q_wr_vl         (write_vl),
        .q_wr_valid      (write_valid),
        .q_wr_data       (write_data),
        .q_wr_ready      (1'b1),
        .q_wr_commit     (local_commit),
        .q_wr_meta       (write_meta),
        .q_wr_abort      (local_abort),
        .ic_req          (local_req),
        .ic_vl           (),
        .ic_net          (),
        .ic_sn           (),
        .ic_resp         (local_resp),
        .ic_accept       (1'b1),
        .stat_ok         (),
        .stat_hdr_err    (stat_hdr_err),
        .stat_len_err    (stat_len_err),
        .stat_vl_unknown (stat_vl_unknown),
        .stat_csum_err   (stat_csum_err),
        .stat_full_drop  ()
    );
    // 此处仅确认本地校验结束；真正的 IC/RM 在两个前端之后统一执行。
    always @(posedge clk) begin
        if (rst) begin
            wp <= 0;
            rp <= 0;
            write_count <= 0;
            filling <= 0;
            local_resp <= 0;
        end else begin
            local_resp <= local_req;
            if (in_valid && in_ready) filling <= 1;
            if (write_valid) write_count <= write_count + 1'b1;
            if (local_commit) wp <= wp + 1'b1;
            if (local_commit || local_abort) begin
                write_count <= 0;
                filling <= 0;
            end
            if (pop) rp <= rp + 1'b1;
        end
        // 数据 RAM 不复位，只有已发布的描述符才允许读取。
        if (write_valid)
            payload[{wp[SLOT_LOG-1:0], write_count}] <= write_data;
        if (local_commit)
            descriptor[wp[SLOT_LOG-1:0]] <= {write_vl, write_meta, write_count};
    end
    reg [DESC_W-1:0] desc_q;
    reg [SLOT_LOG:0] desc_tag;
    reg desc_valid;
    always @(posedge clk) begin
        desc_q <= descriptor[rp[SLOT_LOG-1:0]];
        desc_tag <= rp;
        if (rst) desc_valid <= 0;
        else desc_valid <= !empty;
    end
    assign head_valid = desc_valid && !empty && desc_tag == rp;
    assign {head_vl, head_meta, head_len} = desc_q;

    reg reading;
    reg [10:0] read_address, remaining;
    wire load = reading && remaining != 0 && (!rd_valid || rd_ready);
    always @(posedge clk) begin
        if (load) rd_data <= payload[{rp[SLOT_LOG-1:0], read_address}];
        if (rst) begin
            reading <= 0;
            read_address <= 0;
            remaining <= 0;
            rd_valid <= 0;
            rd_last <= 0;
        end else begin
            if (rd_start) begin
                reading <= 1;
                read_address <= 0;
                remaining <= head_len;
            end
            if (!rd_valid || rd_ready) begin
                rd_valid <= load;
                rd_last <= load && remaining == 1;
            end
            if (load) begin
                read_address <= read_address + 1'b1;
                remaining <= remaining - 1'b1;
            end
            // pop 也可取消因消息池写入错误而中止的本地读取。
            if (pop) begin
                reading <= 0;
                rd_valid <= 0;
                remaining <= 0;
            end
        end
    end
endmodule
