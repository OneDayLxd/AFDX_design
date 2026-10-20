/*
 * ================================================================
 * File Name    : afdx_snmp_ber.v
 * Module Name  : afdx_snmp_ber
 *
 * Function:
 *   SNMPv2c BER 请求解析器（流式，外层到内层，逐层记录结束偏移）。
 *   Message ::= SEQUENCE { version INTEGER(1), community OCTET STRING, PDU }
 *   PDU      ::= [A0 GET | A1 GETNEXT | A3 SET] { request-id, error-status, error-index, VarBindList }
 *   规则：只接受定长编码；长格式最多 2 个长度字节且必须最短；子 TLV 不得越过父边界，
 *        Message 必须恰好占满 UDP 载荷；INTEGER 1..4 字节且最短；OID 1..127 字节；
 *        高位标签号（xx11111）拒绝。VarBind 最多 MAX_VB 条，超出只校验结构并置 vb_overflow。
 *   每条 VarBind 写出描述符 {oid_off, oid_len, val_tag, val_off, val_len}，OID 内容由 MIB 查找模块解码。
 *   读口：rq_addr 为组合地址（= pos），读出数据在本模块再寄存一拍；pos 保持两拍不变即表示当前字节可用。
 *   时序：TLV 头分三拍处理（求结束偏移 → 寄存边界/标签/长度比较 → 按上下文判定）；
 *        团体字用移位寄存器逐字节比较；内容末字节用预先求出的“结束偏移 - 1”判定。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_snmp_ber #(
    parameter MAX_VB = 16
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         go,
    input  wire [10:0]  req_len,
    output wire [10:0]  rq_addr,
    input  wire [7:0]   rq_data,
    input  wire [127:0] read_comm,          // 字节 0 在 [127:120]
    input  wire [4:0]   read_comm_len,      // 0 表示禁用
    input  wire [127:0] write_comm,
    input  wire [4:0]   write_comm_len,
    output reg          done,
    output reg          ok,
    output reg  [2:0]   err_code,           // 1 BER 2 版本 3 团体字 4 PDU 类型
    output reg          comm_write,         // 匹配写团体字（隐含读权限）
    output reg  [10:0]  comm_off,
    output reg  [5:0]   comm_len,
    output reg  [7:0]   pdu_type,
    output reg  [31:0]  request_id,
    output reg  [4:0]   vb_count,
    output reg          vb_overflow,
    output reg          vb_we,
    output reg  [3:0]   vb_widx,
    output reg  [47:0]  vb_wdata            // {oid_off[10:0], oid_len[6:0], val_tag[7:0], val_off[10:0], val_len[10:0]}
);
    localparam C_MSG = 4'd0, C_VER = 4'd1, C_COMM = 4'd2, C_PDU = 4'd3, C_RID = 4'd4, C_ES = 4'd5,
               C_EI = 4'd6, C_VBL = 4'd7, C_VB = 4'd8, C_OID = 4'd9, C_VAL = 4'd10;
    localparam P_IDLE = 4'd0, P_TAG = 4'd1, P_LEN = 4'd2, P_L1 = 4'd3, P_L2 = 4'd4, P_HDR = 4'd5,
               P_INT = 4'd6, P_COMM = 4'd7, P_INTDONE = 4'd8, P_END = 4'd9, P_FAIL = 4'd10,
               P_HDR2 = 4'd11, P_HDR3 = 4'd12;
    reg [3:0] ps, ctx;
    reg [11:0] pos, last_pos, bound;
    reg pos_stable;                       // 地址已保持两拍（last_pos == pos 的寄存值）
    reg started;
    assign rq_addr = pos[10:0];
    (* preserve *) reg [7:0] rq_q;              // 各引擎各自一份，就近布局
    always @(posedge clk) rq_q <= rq_data;
    wire byte_ok = started && last_pos == pos && pos_stable;
    wire [7:0] b = rq_q;
    reg [7:0] tag;
    reg [11:0] len, vend_m1, hdr_end;
    reg [1:0] len_bytes;
    reg [11:0] msg_end, pdu_end, vbl_end, vb_end;
    reg [10:0] oid_off_l;
    reg [6:0] oid_len_l;
    reg [31:0] ival;
    reg [2:0] ipos;
    reg [7:0] ifirst;
    reg [127:0] rc_sh, wc_sh;                   // 待比较的团体字（高字节先比）
    reg match_r, match_w, complete;
    wire int_nonmin = (ipos == 3'd1) && ((ifirst == 8'h00 && !b[7]) || (ifirst == 8'hff && b[7]));
    // P_HDR2 寄存的头部判定
    reg h_over, h_req, h_msg, h_pdu, h_vb, h_vbl, l_zero, l_le4, l_le16, l_le127, l_le255;
    reg t_30, t_02, t_04, t_06, t_pdu, t_a, m_r, m_w, vcnt_ok;

    always @(posedge clk) begin
        if (rst) begin
            ps <= P_IDLE; ctx <= C_MSG; pos <= 12'd0; last_pos <= 12'hfff; started <= 1'b0;
            done <= 1'b0; ok <= 1'b0; err_code <= 3'd0; vb_we <= 1'b0; vb_count <= 5'd0;
            vb_overflow <= 1'b0; comm_write <= 1'b0; pdu_type <= 8'd0; request_id <= 32'd0;
        end else begin
            last_pos <= pos; pos_stable <= last_pos == pos;
            done <= 1'b0;
            vb_we <= 1'b0;
            case (ps)
                P_IDLE: if (go) begin
                    pos <= 12'd0; last_pos <= 12'hfff; started <= 1'b1;
                    ctx <= C_MSG; bound <= {1'b0, req_len}; vb_count <= 5'd0; vb_overflow <= 1'b0;
                    ok <= 1'b0; err_code <= 3'd0; comm_write <= 1'b0; complete <= 1'b0;
                    ps <= P_TAG;
                end
                P_TAG: if (byte_ok) begin
                    if (pos >= bound || b[4:0] == 5'h1f) ps <= P_FAIL;
                    else begin tag <= b; pos <= pos + 1'b1; ps <= P_LEN; end
                end
                P_LEN: if (byte_ok) begin
                    if (pos >= bound) ps <= P_FAIL;
                    else begin
                        pos <= pos + 1'b1;
                        if (!b[7]) begin len <= {5'd0, b[6:0]}; ps <= P_HDR; end
                        else if (b == 8'h81 || b == 8'h82) begin len_bytes <= b[1:0]; ps <= P_L1; end
                        else ps <= P_FAIL;                         // 不定长或超过 2 个长度字节
                    end
                end
                P_L1: if (byte_ok) begin
                    if (pos >= bound) ps <= P_FAIL;
                    else begin
                        pos <= pos + 1'b1;
                        if (len_bytes == 2'd1) begin
                            if (!b[7]) ps <= P_FAIL;               // 非最短长格式
                            else begin len <= {4'd0, b}; ps <= P_HDR; end
                        end else begin
                            if (b == 8'd0 || b > 8'h07) ps <= P_FAIL;
                            else begin len <= {1'b0, b[2:0], 8'd0}; ps <= P_L2; end
                        end
                    end
                end
                P_L2: if (byte_ok) begin
                    if (pos >= bound) ps <= P_FAIL;
                    else begin pos <= pos + 1'b1; len <= {len[11:8], b}; ps <= P_HDR; end
                end
                P_HDR: begin                            // pos 已指向内容首字节
                    hdr_end <= pos + len;
                    ps <= P_HDR2;
                end
                P_HDR2: begin
                    vend_m1 <= hdr_end - 1'b1;
                    h_over <= hdr_end > bound;
                    h_req <= hdr_end == {1'b0, req_len};
                    h_msg <= hdr_end == msg_end;
                    h_pdu <= hdr_end == pdu_end;
                    h_vb <= hdr_end == vb_end;
                    h_vbl <= hdr_end == vbl_end;
                    l_zero <= len == 12'd0;
                    l_le4 <= len <= 12'd4;
                    l_le16 <= len <= 12'd16;
                    l_le127 <= len <= 12'd127;
                    l_le255 <= len <= 12'd255;
                    t_30 <= tag == 8'h30; t_02 <= tag == 8'h02; t_04 <= tag == 8'h04; t_06 <= tag == 8'h06;
                    t_pdu <= tag == 8'ha0 || tag == 8'ha1 || tag == 8'ha3;
                    t_a <= tag[7:4] == 4'ha;
                    m_r <= read_comm_len != 0 && len[4:0] == read_comm_len && !len[5];
                    m_w <= write_comm_len != 0 && len[4:0] == write_comm_len && !len[5];
                    vcnt_ok <= vb_count < MAX_VB;
                    ps <= P_HDR3;
                end
                P_HDR3: begin
                    if (h_over) ps <= P_FAIL;
                    else case (ctx)
                        C_MSG: if (t_30 && h_req) begin
                            msg_end <= hdr_end; ctx <= C_VER; ps <= P_TAG;
                        end else ps <= P_FAIL;
                        C_VER, C_RID, C_ES, C_EI:
                            if (t_02 && !l_zero && l_le4) begin
                                ipos <= 3'd0; ps <= P_INT;
                            end else ps <= P_FAIL;
                        C_COMM: if (t_04 && l_le16) begin
                            comm_off <= pos[10:0]; comm_len <= len[5:0];
                            rc_sh <= read_comm; wc_sh <= write_comm;
                            match_r <= m_r; match_w <= m_w;
                            if (l_zero) begin ctx <= C_PDU; ps <= P_TAG; end
                            else ps <= P_COMM;
                        end else if (t_04 && l_le255) begin
                            // 超长团体字不可能匹配：跳过内容继续校验结构
                            match_r <= 1'b0; match_w <= 1'b0; comm_len <= 6'd0; pos <= hdr_end;
                            ctx <= C_PDU; ps <= P_TAG;
                        end else ps <= P_FAIL;
                        C_PDU: if (t_pdu && h_msg) begin
                            pdu_type <= tag; pdu_end <= hdr_end; bound <= hdr_end; ctx <= C_RID; ps <= P_TAG;
                        end else if (t_a && h_msg) begin
                            err_code <= 3'd4; ps <= P_END;          // GETBULK/Inform/Trap/Response：不支持
                        end else ps <= P_FAIL;
                        C_VBL: if (t_30 && h_pdu) begin
                            vbl_end <= hdr_end; bound <= hdr_end;
                            if (l_zero) begin complete <= 1'b1; ps <= P_END; end
                            else begin ctx <= C_VB; ps <= P_TAG; end
                        end else ps <= P_FAIL;
                        C_VB: if (t_30) begin
                            vb_end <= hdr_end; bound <= hdr_end; ctx <= C_OID; ps <= P_TAG;
                        end else ps <= P_FAIL;
                        C_OID: if (t_06 && !l_zero && l_le127) begin
                            oid_off_l <= pos[10:0]; oid_len_l <= len[6:0]; pos <= hdr_end;
                            ctx <= C_VAL; ps <= P_TAG;
                        end else ps <= P_FAIL;
                        C_VAL: if (h_vb) begin
                            if (vcnt_ok) begin
                                vb_we <= 1'b1; vb_widx <= vb_count[3:0];
                                vb_wdata <= {oid_off_l, oid_len_l, tag, pos[10:0], len[10:0]};
                                vb_count <= vb_count + 1'b1;
                            end else vb_overflow <= 1'b1;
                            pos <= hdr_end; bound <= vbl_end;
                            if (h_vbl) begin complete <= 1'b1; ps <= P_END; end
                            else begin ctx <= C_VB; ps <= P_TAG; end
                        end else ps <= P_FAIL;
                        default: ps <= P_FAIL;
                    endcase
                end
                P_INT: if (byte_ok) begin
                    pos <= pos + 1'b1;
                    ipos <= ipos + 1'b1;
                    if (ipos == 3'd0) begin ival <= {{24{b[7]}}, b}; ifirst <= b; end
                    else ival <= {ival[23:0], b};
                    if (int_nonmin) ps <= P_FAIL;                  // INTEGER 非最短编码
                    else if (pos == vend_m1) ps <= P_INTDONE;
                end
                P_INTDONE: begin
                    ps <= P_TAG;
                    case (ctx)
                        C_VER: if (ival == 32'd1) ctx <= C_COMM;
                               else begin err_code <= 3'd2; ps <= P_END; end
                        C_RID: begin request_id <= ival; ctx <= C_ES; end
                        C_ES: ctx <= C_EI;
                        default: ctx <= C_VBL;
                    endcase
                end
                P_COMM: if (byte_ok) begin
                    if (b != rc_sh[127:120]) match_r <= 1'b0;
                    if (b != wc_sh[127:120]) match_w <= 1'b0;
                    rc_sh <= {rc_sh[119:0], 8'd0}; wc_sh <= {wc_sh[119:0], 8'd0};
                    pos <= pos + 1'b1;
                    if (pos == vend_m1) begin ctx <= C_PDU; ps <= P_TAG; end
                end
                P_END: begin
                    // err_code 已置位（版本/PDU）时不校验团体字
                    done <= 1'b1; started <= 1'b0; ps <= P_IDLE;
                    if (err_code != 3'd0) ok <= 1'b0;
                    else if (!complete) begin ok <= 1'b0; err_code <= 3'd1; end
                    else if (!match_r && !match_w) begin ok <= 1'b0; err_code <= 3'd3; end
                    else begin ok <= 1'b1; comm_write <= match_w; end
                end
                P_FAIL: begin
                    done <= 1'b1; ok <= 1'b0; err_code <= 3'd1; started <= 1'b0; ps <= P_IDLE;
                end
                default: ps <= P_IDLE;
            endcase
        end
    end
endmodule
