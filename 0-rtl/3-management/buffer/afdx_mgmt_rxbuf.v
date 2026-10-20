/*
 * ================================================================
 * File Name    : afdx_mgmt_rxbuf.v
 * Module Name  : afdx_mgmt_rxbuf
 *
 * Function:
 *   管理请求接收缓冲：从接收交付流中取出管理 VL 上的报文，按解析后的 UDP 目的端口分类，
 *   连同会话元数据存入两个 2 KB 请求槽（BRAM，FIFO 顺序）。服务引擎通过同步读口
 *   （地址 -> 下一拍数据）访问队首请求，处理完毕后 release。
 *   时序：输入两级寄存（第 2 级为寄存的端口分类）；槽元数据为独立寄存器，队首输出再寄存一拍，
 *   读口数据由各引擎就近再寄存一拍后使用（引擎按“地址保持两拍”取字节）。
 *   分类：SNMP(161) / TFTP(69 与服务器 TID 区间) / RTC 同步端口；其它端口的报文不进入缓冲。
 *   两个槽都被占用时新请求被丢弃并计数（SNMP/TFTP/RTC 均由对端超时重传）。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_mgmt_rxbuf #(
    parameter VLW = 3,
    parameter MGMT_RX_VL = 0,
    parameter [15:0] TID_BASE = 16'hC000,  // TFTP 服务器 TID 区间 [TID_BASE, TID_BASE+16)
    parameter [15:0] TFTP_PORT = 16'd69
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            enable,
    input  wire [15:0]     snmp_port,
    input  wire [15:0]     rtc_port,
    // 接收交付流（元数据在整条消息期间保持）
    input  wire [VLW-1:0]  rx_vl,
    input  wire            rx_valid,
    input  wire [7:0]      rx_data,
    input  wire            rx_last,
    input  wire [31:0]     rx_src_ip,
    input  wire [15:0]     rx_sport,
    input  wire [15:0]     rx_dport,
    input  wire [31:0]     rx_ts,
    // 队首请求
    output wire            req_valid,
    output wire [1:0]      req_class,       // 1 SNMP, 2 TFTP, 3 RTC
    output wire [10:0]     req_len,
    output wire [31:0]     req_src_ip,
    output wire [15:0]     req_sport,
    output wire [15:0]     req_dport,
    output wire [31:0]     req_ts,
    input  wire            req_release,
    input  wire [10:0]     rd_addr,
    output reg  [7:0]      rd_data,
    // 统计
    output reg  [31:0]     cnt_accept,
    output reg  [31:0]     cnt_drop_full,
    output reg  [31:0]     cnt_drop_port,
    output reg  [31:0]     cnt_drop_long
);
    localparam [VLW-1:0] RX_VL = MGMT_RX_VL[VLW-1:0];
    localparam C_NONE = 2'd0, C_SNMP = 2'd1, C_TFTP = 2'd2, C_RTC = 2'd3;
    // 第 1 级寄存输入；第 2 级寄存端口分类结果，接收逻辑只看寄存的分类。
    reg [VLW-1:0] vl_q;
    reg [7:0] data_q, data_q2;
    reg valid_q, last_q, en_q, valid_q2, last_q2, mgmt_q2;
    reg [15:0] dport_q;
    reg [95:0] meta_q, meta_q2;
    reg [1:0] class_q2;
    always @(posedge clk) begin
        if (rst) begin valid_q <= 1'b0; valid_q2 <= 1'b0; end
        else begin valid_q <= rx_valid; valid_q2 <= valid_q; end
        vl_q <= rx_vl; data_q <= rx_data; last_q <= rx_last; en_q <= enable;
        dport_q <= rx_dport;
        meta_q <= {rx_src_ip, rx_sport, rx_dport, rx_ts};
        data_q2 <= data_q; last_q2 <= last_q; meta_q2 <= meta_q;
        mgmt_q2 <= en_q && vl_q == RX_VL;
        class_q2 <= !en_q || vl_q != RX_VL ? C_NONE :
                    (dport_q == snmp_port) ? C_SNMP :
                    (dport_q == TFTP_PORT || dport_q[15:4] == TID_BASE[15:4]) ? C_TFTP :
                    (dport_q == rtc_port) ? C_RTC : C_NONE;
    end

    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] mem [0:4095];
    reg [1:0] used;                              // 已提交的槽数
    reg head, tail;                              // 队首槽、写入槽
    // 两个槽的元数据用独立寄存器（不推断 RAM，避免读写旁路逻辑）
    reg [1:0] cls0, cls1;
    reg [10:0] len0, len1;
    reg [95:0] meta0, meta1;
    reg in_msg, accepting, too_long;
    reg [10:0] wr_idx;
    reg wr_at_limit;
    reg wr_en;
    reg [11:0] wr_addr;
    reg [7:0] wr_byte;
    // Payload/address are speculative; only wr_en authorizes a RAM write.
    always @(posedge clk) begin
        wr_byte <= data_q2;
        wr_addr <= {tail, in_msg ? wr_idx : 11'd0};
    end
    always @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_byte;
        rd_data <= mem[{head, rd_addr}];
    end
    // Only the unoccupied tail slot may be updated speculatively. On the
    // committing edge this captures the final length; tail then advances,
    // protecting that slot until release. No classification/last/limit
    // decision drives these wide register enables.
    wire [10:0] next_len = in_msg ? wr_idx + 11'd1 : 11'd1;
    always @(posedge clk) begin
        if (used != 2'd2 && !tail) len0 <= next_len;
        if (used != 2'd2 &&  tail) len1 <= next_len;
    end
    // 队首输出寄存一拍：释放当拍即撤销 req_valid，下一拍起给出新队首，有效位与数据同拍更新。
    wire release_head = req_release && used != 2'd0;
    reg req_valid_r;
    reg [1:0] req_class_r;
    reg [10:0] req_len_r;
    reg [95:0] req_meta_r;
    always @(posedge clk) begin
        if (rst) req_valid_r <= 1'b0;
        else req_valid_r <= used != 2'd0 && !release_head;
        req_class_r <= head ? cls1 : cls0;
        req_len_r <= head ? len1 : len0;
        req_meta_r <= head ? meta1 : meta0;
    end
    assign req_valid = req_valid_r;
    assign req_class = req_class_r;
    assign req_len = req_len_r;
    assign {req_src_ip, req_sport, req_dport, req_ts} = req_meta_r;
    wire commit_single = valid_q2 && !in_msg && last_q2 && class_q2 != C_NONE && used != 2'd2;
    wire commit_multi = valid_q2 && in_msg && last_q2 && accepting && !too_long && !wr_at_limit;

    // Queue occupancy/commit stays on the original edge. Statistics may
    // appear one clock later; consecutive commits still count individually.
    (* preserve *) reg accepted_event;
    always @(posedge clk) begin
        if (rst) begin accepted_event <= 1'b0; cnt_accept <= 32'd0; end
        else begin
            accepted_event <= commit_single || commit_multi;
            if (accepted_event) cnt_accept <= cnt_accept + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            used <= 2'd0; head <= 1'b0; tail <= 1'b0; in_msg <= 1'b0; accepting <= 1'b0;
            too_long <= 1'b0; wr_idx <= 11'd0; wr_at_limit <= 1'b0; wr_en <= 1'b0;
            cnt_drop_full <= 0; cnt_drop_port <= 0; cnt_drop_long <= 0;
        end else begin
            wr_en <= 1'b0;
            if (release_head) head <= ~head;
            // 提交与释放可能同拍发生
            case ({commit_single || commit_multi, release_head})
                2'b10: used <= used + 1'b1;
                2'b01: used <= used - 1'b1;
                default: ;
            endcase
            if (valid_q2) begin
                if (!in_msg) begin
                    // 首字节：分类并决定是否接收（释放同拍产生的空槽下一条消息再用）
                    in_msg <= !last_q2;
                    too_long <= 1'b0;
                    wr_idx <= 11'd1;
                    wr_at_limit <= 1'b0;
                    if (class_q2 == C_NONE) begin
                        accepting <= 1'b0;
                        if (mgmt_q2) cnt_drop_port <= cnt_drop_port + 1'b1;
                    end else if (used == 2'd2) begin
                        accepting <= 1'b0;
                        cnt_drop_full <= cnt_drop_full + 1'b1;
                    end else begin
                        accepting <= !last_q2;
                        wr_en <= 1'b1;
                        if (tail) begin meta1 <= meta_q2; cls1 <= class_q2; end
                        else begin meta0 <= meta_q2; cls0 <= class_q2; end
                        if (last_q2) begin
                            tail <= ~tail;
                        end
                    end
                end else begin
                    if (accepting) begin
                        if (wr_at_limit) too_long <= 1'b1;
                        else begin
                            wr_en <= 1'b1;
                            wr_idx <= wr_idx + 1'b1;
                            wr_at_limit <= wr_idx == 11'd1470;
                        end
                    end
                    if (last_q2) begin
                        in_msg <= 1'b0;
                        accepting <= 1'b0;
                        if (commit_multi) begin
                            tail <= ~tail;
                        end else if (accepting) begin
                            cnt_drop_long <= cnt_drop_long + 1'b1;
                        end
                    end
                end
            end
        end
    end
endmodule
