/*
 * ================================================================
 * File Name    : afdx_rx_parser.v
 * Module Name  : afdx_rx_parser
 *
 * Function:
 *   接收帧解析器（接收管理 / 帧过滤 / UDP-IP 协议处理）：
 *   逐字节解析 AFDX 帧，校验目的 MAC 前缀、以太网类型和 IPv4 首部；分片模式输出原始 IP 数据，
 *   目的 IP 与 VLID 一致性、IP/UDP 长度一致性、UDP 校验和；根据 VLID 查表得到接收队列号；
 *   分片单独检查 IP 长度与标记，整包 UDP 校验由后级重组处理。
 *   把 UDP 有效载荷或分片 IP 数据写入接收队列，帧尾取出序列号后交由完整性检查/冗余管理裁决，通过则提交，否则丢弃。
 *   MW > 9 时描述符附带 UDP 会话元数据：{接收时间戳, 源 IP, 源端口, 目的端口, 网络, 序列号}，
 *   时间戳在帧首字节进入解析器时采样（MAC 已完成 FCS 校验）。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_defs.vh"
module afdx_rx_parser #(
    parameter FRAGMENT_ENABLE = 0,
    parameter N_VL           = 8,
    parameter VLW            = 3,
    parameter MW             = 9,          // 基础 9、会话 105、分片会话 137 位
    parameter NET_FROM_SA    = 0,          // 1: 从源 MAC 接口号判定网络（单总线复用 A/B）
    parameter UDP_CSUM_CHECK = 1
) (
    input  wire            clk,
    input  wire            rst,
    input  wire [31:0]     ts_us,          // 接收时间戳时基
    // 输入帧流（不含 FCS）
    input  wire [7:0]      in_data,
    input  wire            in_valid,
    input  wire            in_last,
    input  wire            in_net,
    output wire            in_ready,
    // 接收 VL 查找表
    output wire [15:0]     lk_vlid,
    input  wire            lk_hit,
    input  wire [VLW-1:0]  lk_idx,
    // 接收队列写侧
    output wire [VLW-1:0]  q_wr_vl,
    output wire            q_wr_valid,
    output wire [7:0]      q_wr_data,
    input  wire            q_wr_ready,
    output reg             q_wr_commit,
    output wire [MW-1:0]   q_wr_meta,
    output reg             q_wr_abort,
    // 完整性检查 / 冗余管理请求
    output reg             ic_req,
    output wire [VLW-1:0]  ic_vl,
    output wire            ic_net,
    output wire [7:0]      ic_sn,
    input  wire            ic_resp,
    input  wire            ic_accept,
    // 统计脉冲
    output reg             stat_ok,          // 帧被接收并交付队列
    output reg             stat_hdr_err,
    output reg             stat_len_err,
    output reg             stat_vl_unknown,
    output reg             stat_csum_err,
    output reg             stat_full_drop
);
    localparam S_RX = 3'd0, S_DECIDE = 3'd1, S_WAIT = 3'd2, S_DONE = 3'd3, S_FOLD=3'd6, S_FOLDEND=3'd7;
    reg [2:0]  state;
    reg [13:0] bi;                                  // 帧内字节索引
    reg [15:0] vlid, ip_len, udp_len, udp_csum_f;
    reg [15:0] ip_id, ip_flags;
    wire fragmented = FRAGMENT_ENABLE != 0 && (ip_flags[13] || ip_flags[12:0] != 0);
    reg [7:0]  sn;
    reg        net_l;
    reg        hdr_err, len_err, vl_unk, full_drop;
    reg [10:0] hdr_flags;
    always @(posedge clk) begin
        if (rst) hdr_err <= 1'b0;
        else hdr_err <= |hdr_flags;
    end
    (* preserve, maxfan = 8 *) reg [VLW-1:0] idx_l;
    reg [31:0] ip_sum, udp_sum;

    wire beat = in_valid & in_ready;
    assign in_ready = (state == S_RX);
    assign lk_vlid  = {vlid[15:8], in_data};
    assign q_wr_vl  = idx_l;
    reg [15:0] pld_end; // computed when the UDP length arrives, before payload
    wire        in_pld = fragmented ? (bi >= 34 && {2'b00,bi} < 16'd14 + ip_len) :
                                        (bi >= 42 && {2'b00,bi} < pld_end);
    wire        pld_write = beat & in_pld & ~in_last & ~vl_unk & ~full_drop & ~hdr_err;
    // Payload bytes reach the queue one clock later from registers. A refused
    // byte is seen then: the frame is dropped and aborted as before, and at
    // most one following byte arrives together with that abort.
    reg        wr_pending;
    reg [7:0]  wr_byte;
    assign q_wr_valid = wr_pending;
    assign q_wr_data = wr_byte;
    always @(posedge clk) begin
        if (rst) wr_pending <= 1'b0;
        else wr_pending <= pld_write;
        wr_byte <= in_data;
    end
    reg [31:0] src_ip, rx_ts;
    reg [15:0] udp_sport, udp_dport;
    generate if (FRAGMENT_ENABLE != 0) begin : g_fragment_meta
        assign q_wr_meta = {fragmented,ip_flags[13],ip_id,ip_flags[12:0],1'b0,
                            rx_ts,src_ip,udp_sport,udp_dport,net_l,sn};
    end else if (MW > 9) begin : g_session_meta
        assign q_wr_meta = {rx_ts, src_ip, udp_sport, udp_dport, net_l, sn};
    end else begin : g_basic_meta
        assign q_wr_meta = {net_l, sn};
    end endgenerate
    assign ic_vl  = idx_l;
    assign ic_net = net_l;
    assign ic_sn  = sn;

    // 期望的最后字节索引：max(14 + ip_len + 1, 60) - 1
    // Both are registered from the length fields, which precede the last byte.
    reg [15:0] exp_last;
    reg        len_consistent;
    always @(posedge clk) begin
        exp_last <= ((16'd14 + ip_len) < 16'd59) ? 16'd59 : (16'd14 + ip_len);
        len_consistent <= fragmented ?
            (ip_len > 20 && ip_len <= 1499 && (!ip_flags[13] || ip_len[2:0] == 3'd4)) :
            (ip_len == udp_len + 16'd20 && udp_len >= 9 && udp_len <= 1479);
    end

    // 校验和折叠
    reg [16:0] ip_f1, udp_f1;
    reg [15:0] ip_f2, udp_f2;
    reg [31:0] udp_pseudo, udp_t;
    wire ip_csum_ok = ip_f2 == 16'hffff;
    wire udp_csum_ok = fragmented || (UDP_CSUM_CHECK == 0) || (udp_csum_f == 0) || (udp_f2 == 16'hffff);
    wire frame_reject = hdr_err || !ip_csum_ok || len_err || vl_unk || !udp_csum_ok || full_drop;
    wire [31:0] add_hi = {16'd0, in_data, 8'd0};
    wire [31:0] add_lo = {24'd0, in_data};
    // 伪首部长度在载荷前已稳定；折叠流水与收帧并行，不在 EOF 后再串行加两项。
    // SN 不参与 UDP 校验和，最后一个载荷在 SN 前一拍完成累加。
    always @(posedge clk) begin
        udp_pseudo <= {16'd0,udp_len}+32'd17;
        udp_t <= udp_sum+udp_pseudo;
        udp_f1 <= {1'b0,udp_t[15:0]}+{1'b0,udp_t[31:16]};
        udp_f2 <= udp_f1[15:0]+udp_f1[16];
        ip_f1 <= {1'b0,ip_sum[15:0]}+{1'b0,ip_sum[31:16]};
        ip_f2 <= ip_f1[15:0]+ip_f1[16];
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= S_RX;
            bi <= 14'd0;
            vlid <= 16'd0; ip_len <= 16'd0; udp_len <= 16'd0; udp_csum_f <= 16'd0;
            sn <= 8'd0; net_l <= 1'b0; ip_id <= 0; ip_flags <= 0;
            hdr_flags <= 0; len_err <= 1'b0; vl_unk <= 1'b0; full_drop <= 1'b0;
            idx_l <= {VLW{1'b0}};
            ip_sum <= 32'd0; udp_sum <= 32'd0; pld_end <= 0;
            q_wr_commit <= 1'b0; q_wr_abort <= 1'b0; ic_req <= 1'b0;
            stat_ok <= 1'b0; stat_hdr_err <= 1'b0; stat_len_err <= 1'b0;
            stat_vl_unknown <= 1'b0; stat_csum_err <= 1'b0; stat_full_drop <= 1'b0;
        end else begin
            q_wr_commit <= 1'b0; q_wr_abort <= 1'b0; ic_req <= 1'b0;
            stat_ok <= 1'b0; stat_hdr_err <= 1'b0; stat_len_err <= 1'b0;
            stat_vl_unknown <= 1'b0; stat_csum_err <= 1'b0; stat_full_drop <= 1'b0;
            if (wr_pending && !q_wr_ready && !full_drop) begin
                full_drop <= 1'b1;
                q_wr_abort <= 1'b1;
            end
            case (state)
                S_RX: begin
                    if (beat) begin
`ifdef AFDX_DEBUG2
                        $display("[%0t] byte bi=%0d d=%02x last=%b udp_sum=%08x", $time, bi, in_data, in_last, udp_sum);
`endif
                        bi <= bi + 1'b1;
                        // 小块池可能在报文中途耗尽：上面的检查在字节未被接收的下一拍回滚，
                        // 然后把整帧排空；禁止提交缺字节的载荷，也不推进 IC/RM 状态。
                        // Session fields have no reset and no effect on the checks.
                        case (bi)
                            14'd26: src_ip[31:24] <= in_data;
                            14'd27: src_ip[23:16] <= in_data;
                            14'd28: src_ip[15:8]  <= in_data;
                            14'd29: src_ip[7:0]   <= in_data;
                            14'd34: udp_sport[15:8] <= in_data;
                            14'd35: udp_sport[7:0]  <= in_data;
                            14'd36: udp_dport[15:8] <= in_data;
                            14'd37: udp_dport[7:0]  <= in_data;
                            default: ;
                        endcase
                        if (bi == 14'd0) rx_ts <= ts_us;
                        case (bi)
                            14'd0:  begin if (in_data != 8'h03) hdr_flags[0] <= 1'b1; net_l <= in_net; end
                            14'd1, 14'd2, 14'd3: if (in_data != 8'h00) hdr_flags[1] <= 1'b1;
                            14'd4:  vlid[15:8] <= in_data;
                            14'd5:  begin
                                        vlid[7:0] <= in_data;
                                        idx_l     <= lk_idx;
                                        vl_unk    <= ~lk_hit;

                                    end
                            14'd11: if (NET_FROM_SA != 0) net_l <= (in_data[7:5] == 3'b010);
                            14'd12: if (in_data != 8'h08) hdr_flags[2] <= 1'b1;
                            14'd13: if (in_data != 8'h00) hdr_flags[3] <= 1'b1;
                            14'd14: if (in_data != 8'h45) hdr_flags[4] <= 1'b1;
                            14'd16: ip_len[15:8] <= in_data;
                            14'd17: ip_len[7:0]  <= in_data;
                            14'd18: ip_id[15:8] <= in_data;
                            14'd19: ip_id[7:0] <= in_data;
                            14'd20: begin
                                ip_flags[15:8] <= in_data;
                                if (FRAGMENT_ENABLE != 0) begin
                                    if (in_data[7] || (in_data[6] && in_data[5:0] != 0)) hdr_flags[5] <= 1;
                                end else if (in_data[5:0] != 0) hdr_flags[5] <= 1;
                            end
                            14'd21: begin
                                ip_flags[7:0] <= in_data;
                                if ((FRAGMENT_ENABLE == 0 && in_data != 0) ||
                                    (FRAGMENT_ENABLE != 0 && ip_flags[14] && in_data != 0)) hdr_flags[6] <= 1;
                            end
                            14'd23: if (in_data != `AFDX_IP_PROTO_UDP) hdr_flags[7] <= 1'b1;
                            14'd30, 14'd31: if (in_data != 8'hE0) hdr_flags[8] <= 1'b1;
                            14'd32: if (in_data != vlid[15:8]) hdr_flags[9] <= 1'b1;
                            14'd33: if (in_data != vlid[7:0]) hdr_flags[10] <= 1'b1;
                            14'd38: udp_len[15:8] <= in_data;
                            14'd39: begin udp_len[7:0] <= in_data; pld_end <= 16'd34+{udp_len[15:8],in_data}; end
                            14'd40: udp_csum_f[15:8] <= in_data;
                            14'd41: udp_csum_f[7:0]  <= in_data;
                            default: ;
                        endcase
                        // IP 首部校验和：字节 14..33
                        if (bi >= 14'd14 && bi <= 14'd33)
                            ip_sum <= ip_sum + (bi[0] ? add_lo : add_hi);
                        // UDP 校验和：伪首部地址(26..33) + UDP 首部与数据(34..pld_end-1)
                        // 注意 udp_len 在 bi=40 起才有效，UDP 首部（34..41）固定计入
                        if ((bi >= 14'd26 && bi <= 14'd41) || (bi >= 14'd42 && {2'b00, bi} < pld_end && !in_last))
                            udp_sum <= udp_sum + (bi[0] ? add_lo : add_hi);
                        if (in_last) begin
                            sn      <= in_data;
                            len_err <= ({2'b00, bi} != exp_last) || !len_consistent;
                            state   <= S_FOLD;
                        end
                    end
                end
                S_FOLD: begin
                    state <= S_FOLDEND;
                end
                S_FOLDEND: begin
                    state <= S_DECIDE;
                end
                S_DECIDE: begin
`ifdef AFDX_DEBUG
                    $display("[%0t] parser: bi=%0d vlid=%04x ip_len=%0d udp_len=%0d hdr_err=%b ip_f2=%04x udp_f2=%04x udp_csum_f=%04x len_err=%b vl_unk=%b sn=%0d net=%b",
                             $time, bi, vlid, ip_len, udp_len, hdr_err, ip_f2, udp_f2, udp_csum_f, len_err, vl_unk, sn, net_l);
`endif
                    if (hdr_err || !ip_csum_ok) begin
                        q_wr_abort <= 1'b1; stat_hdr_err <= 1'b1; state <= S_DONE;
                    end else if (len_err) begin
                        q_wr_abort <= 1'b1; stat_len_err <= 1'b1; state <= S_DONE;
                    end else if (vl_unk) begin
                        q_wr_abort <= 1'b1; stat_vl_unknown <= 1'b1; state <= S_DONE;
                    end else if (!udp_csum_ok) begin
                        q_wr_abort <= 1'b1; stat_csum_err <= 1'b1; state <= S_DONE;
                    end else if (full_drop) begin
                        q_wr_abort <= 1'b1; stat_full_drop <= 1'b1; state <= S_DONE;
                    end else begin
                        ic_req <= 1'b1; state <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (ic_resp) begin
                        if (ic_accept) begin
                            q_wr_commit <= 1'b1; stat_ok <= 1'b1;
                        end else begin
                            q_wr_abort <= 1'b1;
                        end
                        state <= S_DONE;
                    end
                end
                S_DONE: begin
                    state <= S_RX;
                end
            endcase
            // 裁决返回时直接清理；下一帧首部可与队列 commit/abort 同拍处理。
            // idx_l/sn/会话元数据保持到下一帧对应字段到达，供队列采样上一帧。
            if ((state == S_DECIDE && frame_reject) || (state == S_WAIT && ic_resp)) begin
                    bi <= 14'd0;
                    hdr_flags <= 0; len_err <= 1'b0; vl_unk <= 1'b0; full_drop <= 1'b0;
                    ip_sum <= 32'd0; udp_sum <= 32'd0; pld_end <= 0;
                    state <= S_RX;
            end
        end
    end
endmodule
