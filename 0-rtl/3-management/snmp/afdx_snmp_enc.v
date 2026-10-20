/*
 * ================================================================
 * File Name    : afdx_snmp_enc.v
 * Module Name  : afdx_snmp_enc
 *
 * Function:
 *   SNMPv2c 响应/Trap 编码器：先计算各层长度，再按正向顺序写出 BER，无固定偏移。
 *   第一遍：逐条 VarBind 求 OID 与值的内容长度，累加得到 VarBindList、PDU、Message 长度；
 *           总长超过 max_len 时改为 tooBig(1)、error-index 0、空 VarBindList 的最小响应。
 *   第二遍：写出 30 L {02 01 01, 04 L community, PDU L {02 L reqid, 02 01 es, 02 01 ei, 30 L {VarBind...}}}。
 *   VarBind 输出描述符（vbo_rdata，读延迟 1 拍）：
 *     [138] oid_echo  [137:127] oid_off  [126:120] oid_len  [119:113] k  [112:97] inst
 *     [96:94] vkind(0 异常 1 整数类 2 字符串 3 OID 值 4 回显请求值 5 8 字节 OCTET STRING)  [93:86] vtag
 *     [85:22] vval  [21:11] voff  [10:0] vlen
 *   生成的 OID 均以 1.3 开头（首字节 0x2B），各弧不超过 21 位。
 *   时序：MIB ROM 条目属性同步读出，弧值与其字节数再两级寄存（弧序号变化后等待 2 拍，条目变化后 3 拍）；
 *        总长/tooBig 判定、整数最短长度均先寄存。
 *
 * Sub Module:
 *   afdx_snmp_mibrom
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_snmp_enc (
    input  wire         clk,
    input  wire         rst,
    input  wire         go,
    input  wire [7:0]   pdu_tag,
    input  wire [31:0]  request_id,
    input  wire [7:0]   err_status,
    input  wire [7:0]   err_index,
    input  wire [4:0]   vb_n,
    input  wire         comm_echo,
    input  wire [10:0]  comm_off,
    input  wire [5:0]   comm_len,
    input  wire [127:0] comm_reg,           // comm_echo=0 时使用（Trap 团体字）
    input  wire [10:0]  max_len,
    output wire [3:0]   vbo_raddr,
    input  wire [138:0] vbo_rdata,
    output wire [10:0]  rq_addr,
    input  wire [7:0]   rq_data,
    output reg          tb_wr_en,
    output reg  [10:0]  tb_wr_addr,
    output reg  [7:0]   tb_wr_data,
    output reg          done,
    output reg  [10:0]  out_len,
    output reg          too_big
);
    localparam X_IDLE = 5'd0, X_P1_READ = 5'd1, X_P1_DEC = 5'd2, X_P1_OID = 5'd3, X_P1_VAL = 5'd4,
               X_P1_SUM = 5'd5, X_P1_ACC = 5'd6, X_TOT1 = 5'd7, X_TOT2 = 5'd8, X_TOT3 = 5'd9,
               X_E_VER = 5'd10, X_E_COMM = 5'd11, X_E_RID = 5'd12, X_E_ERR = 5'd13, X_E_VB = 5'd14,
               X_E_VBL = 5'd15, X_E_OID = 5'd16, X_E_GEN = 5'd17, X_E_VALH = 5'd18, X_E_VAL = 5'd19,
               X_H_TAG = 5'd20, X_H_LEN = 5'd21, X_CP = 5'd22, X_INT = 5'd23, X_DONE = 5'd24,
               X_E_PDU = 5'd25, X_E_OIDH = 5'd26, X_E_NEXT = 5'd27, X_WAIT = 5'd28, X_P1_W = 5'd29,
               X_TOT4 = 5'd30, X_RID_WAIT = 5'd31;
    reg [4:0] x, ret, ret_cp, ret_int;         // 头部、复制、整数输出之后的返回状态
    reg [3:0] vi;
    reg [4:0] n_vb;
    reg [7:0] es, ei;
    reg big_done;
    assign vbo_raddr = vi;
    // 当前 VarBind
    reg c_echo;
    reg [10:0] c_ooff, c_voff, c_vlen;
    reg [6:0] c_olen, c_k;
    reg [15:0] c_inst;
    reg [2:0] c_kind;
    reg [7:0] c_tag;
    reg [63:0] c_val;
    reg c_val_zero;
    // ld_c loads the descriptor in X_P1_DEC/X_E_VBL. Even the echo path
    // passes through X_P1_W before consuming this flag in X_P1_VAL.
    always @(posedge clk) c_val_zero <= (c_val == 64'd0);
    // 第一遍求得的逐条 OID/值内容长度：同步读 RAM，第二遍在 X_E_VB 读出、X_E_VBL 使用
    (* ramstyle = "M4K, no_rw_check" *) reg [17:0] olvl [0:15];
    reg [17:0] olvl_q;
    // MIB ROM：生成 OID 的弧
    reg [3:0] gj;
    wire [31:0] g_arc;
    wire [3:0] g_elen;
    wire g_table;
    afdx_snmp_mibrom u_rom (
        .clk      (clk),
        .k        (c_k),
        .j        (gj),
        .inst     (c_inst),
        .arc      (g_arc),
        .elen     (g_elen),
        .is_table (g_table),
        .vtype    (),
        .access   (),
        .check    ()
    );
    wire [3:0] g_n = g_elen + {3'd0, g_table};
    reg [31:0] g_arc_q, g_arc_q2;
    reg [3:0] g_n_q;
    reg [1:0] g_nb_q;                            // 与 g_arc_q2 对齐
    reg [1:0] g_wait;                            // gj/c_k 变化后等待 ROM 流水线
    always @(posedge clk) begin
        g_arc_q <= g_arc; g_n_q <= g_n;
        g_arc_q2 <= g_arc_q;
        g_nb_q <= (g_arc_q[31:7] == 25'd0) ? 2'd1 : (g_arc_q[31:14] == 18'd0) ? 2'd2 : 2'd3;
    end
    reg [1:0] gb;                                // 当前弧已输出字节数
    // 长度
    reg [10:0] olen, vlen, vbl_sum, vbc, pdu_c, msg_c, wptr;
    reg [71:0] iv;                               // 符号/零扩展后的整数
    wire [3:0] iv_len;
    genvar gl;
    // Adjacent byte/sign windows must each be all zero or all one. Their
    // one-bit overlap forces the same sign across the entire removed prefix.
    // Register these small comparisons before the length priority encoder.
    reg [8:1] sign_byte;
    wire [8:1] fits;
    generate for (gl = 1; gl <= 8; gl = gl + 1) begin : g_fit
        always @(posedge clk)
            sign_byte[gl] <= (&iv[8*gl+7:8*gl-1]) || !(|iv[8*gl+7:8*gl-1]);
        assign fits[gl] = &sign_byte[8:gl];
    end endgenerate
    assign iv_len = fits[1] ? 4'd1 : fits[2] ? 4'd2 : fits[3] ? 4'd3 : fits[4] ? 4'd4 :
                    fits[5] ? 4'd5 : fits[6] ? 4'd6 : fits[7] ? 4'd7 : fits[8] ? 4'd8 : 4'd9;
    wire [71:0] rid_ext = {{40{request_id[31]}}, request_id};
    reg [3:0] rid_len;
    // TLV 总长 = 1 + 长度字节数 + 内容
    wire [10:0] tlv_ol = {4'd0, olen[6:0]} + 11'd2;
    wire [10:0] tlv_vl = vlen + ((vlen < 11'd128) ? 11'd2 : (vlen < 11'd256) ? 11'd3 : 11'd4);
    wire [10:0] tlv_vbc = vbc + ((vbc < 11'd128) ? 11'd2 : (vbc < 11'd256) ? 11'd3 : 11'd4);
    wire [10:0] tlv_vbl = vbl_sum + ((vbl_sum < 11'd128) ? 11'd2 : (vbl_sum < 11'd256) ? 11'd3 : 11'd4);
    wire [10:0] tlv_pdu = pdu_c + ((pdu_c < 11'd128) ? 11'd2 : (pdu_c < 11'd256) ? 11'd3 : 11'd4);
    wire [11:0] tlv_msg = {1'b0, msg_c} + ((msg_c < 11'd128) ? 12'd2 : (msg_c < 11'd256) ? 12'd3 : 12'd4);
    reg [11:0] tlv_msg_q;
    reg msg_big;
    // 头部子序列：标签 + 1..3 个长度字节
    reg [7:0] h_tag;
    reg [10:0] h_len;
    reg [1:0] h_i;
    // 回显复制（请求 RAM，同步读）
    reg [11:0] cp_src, cp_last;
    reg cp_stable;                       // 地址已保持两拍（cp_last == cp_src 的寄存值）
    (* preserve *) reg [7:0] rq_q;               // 读出数据就近寄存（各引擎各一份）；地址保持两拍后取字节
    always @(posedge clk) rq_q <= rq_data;
    reg [10:0] cp_n;
    reg cp_started;
    assign rq_addr = cp_src[10:0];
    wire cp_ok = cp_started && cp_last == cp_src && cp_stable;
    reg cp_from_reg;
    reg [127:0] comm_sh;                         // Trap 团体字（高字节先出）
    // 整数/常量输出移位寄存器；按字节数左对齐的整数每拍预先移好（iv/vlen、rid_len 在使用前多拍已稳定）
    reg [71:0] sv, iv_sh, rid_sh;
    reg [3:0] sv_n, iv_sa, rid_sa;
    always @(posedge clk) begin
        iv_sa <= 4'd9 - vlen[3:0]; rid_sa <= 4'd9 - rid_len;
        iv_sh <= iv << {iv_sa, 3'b000};
        rid_sh <= rid_ext << {rid_sa, 3'b000};
    end
    // 固定串
    reg [4:0] s_i;
    reg [7:0] s_byte;
    localparam [159:0] SYS_DESCR = "AFDX End System FPGA";      // 20 字节
    localparam [55:0] SYS_NAME = "afdx-es";
    integer q;
    always @* begin                                 // 常量串查找表（避免变量减法与移位）
        s_byte = 8'd0;
        for (q = 0; q < 20; q = q + 1)
            if (s_i == q) s_byte = c_val[0] ? ((q < 7) ? SYS_NAME[55 - 8*(q % 7) -: 8] : 8'd0)
                                            : SYS_DESCR[159 - 8*q -: 8];
    end
    wire [10:0] str_len = c_val[0] ? 11'd7 : 11'd20;
    reg [2:0] k3;                                 // 常量字节计数

    // 当前 VarBind 描述符在 X_P1_DEC / X_E_VBL 装载；装载标志提前一拍由其唯一前驱状态寄存给出
    reg ld_c;
    always @(posedge clk) begin
        ld_c <= !rst && (x == X_P1_READ || x == X_E_VB);
        if (ld_c) {c_echo, c_ooff, c_olen, c_k, c_inst, c_kind, c_tag, c_val, c_voff, c_vlen} <= vbo_rdata;
    end
    always @(posedge clk) begin
        if (x == X_P1_SUM) olvl[vi] <= {olen[6:0], vlen};
        olvl_q <= olvl[vi];
    end

    task emit;
        input [7:0] v;
        begin
            tb_wr_en <= 1'b1; tb_wr_addr <= wptr; tb_wr_data <= v; wptr <= wptr + 1'b1;
        end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            x <= X_IDLE; done <= 1'b0; tb_wr_en <= 1'b0; too_big <= 1'b0; out_len <= 11'd0;
            cp_started <= 1'b0; cp_src <= 12'd0; cp_last <= 12'hfff; vi <= 4'd0;
        end else begin
            done <= 1'b0;
            tb_wr_en <= 1'b0;
            cp_last <= cp_src; cp_stable <= cp_last == cp_src;
            case (x)
                X_IDLE: if (go) begin
                    n_vb <= vb_n; es <= err_status; ei <= err_index; big_done <= 1'b0; too_big <= 1'b0;
                    vi <= 4'd0; vbl_sum <= 11'd0; iv <= rid_ext;
                    x <= (vb_n == 5'd0) ? X_WAIT : X_P1_READ;
                end
                X_WAIT: x <= X_RID_WAIT;                  // sign_byte after loading request_id
                X_RID_WAIT: begin rid_len <= iv_len; x <= X_TOT1; end
                X_P1_READ: x <= X_P1_DEC;                  // 描述符 RAM 读延迟
                X_P1_DEC: begin
                    iv <= (vbo_rdata[93:86] == 8'h02) ? {{40{vbo_rdata[53]}}, vbo_rdata[53:22]} : {8'd0, vbo_rdata[85:22]};
                    gj <= 4'd2; olen <= 11'd1; g_wait <= 2'd3;
                    x <= vbo_rdata[138] ? X_P1_W : X_P1_OID;
                    if (vbo_rdata[138]) olen <= {4'd0, vbo_rdata[126:120]};
                end
                X_P1_W: x <= X_P1_VAL;                    // sign_byte after loading the value
                X_P1_OID: begin
                    if (g_wait != 2'd0) g_wait <= g_wait - 1'b1;
                    else if (gj == g_n_q) x <= X_P1_VAL;
                    else begin olen <= olen + {9'd0, g_nb_q}; gj <= gj + 1'b1; g_wait <= 2'd2; end
                end
                X_P1_VAL: begin
                    case (c_kind)
                        3'd0: vlen <= 11'd0;
                        3'd1: vlen <= {7'd0, iv_len};
                        3'd2: vlen <= str_len;
                        3'd3: vlen <= c_val_zero ? 11'd6 : 11'd8;
                        3'd5: vlen <= 11'd8;
                        default: vlen <= c_vlen;
                    endcase
                    x <= X_P1_SUM;
                end
                X_P1_SUM: begin
                    vbc <= tlv_ol + tlv_vl;
                    x <= X_P1_ACC;
                end
                X_P1_ACC: begin
                    vbl_sum <= vbl_sum + tlv_vbc;
                    if ({1'b0, vi} == n_vb - 1'b1) begin iv <= rid_ext; x <= X_WAIT; end
                    else begin vi <= vi + 1'b1; x <= X_P1_READ; end
                end
                X_TOT1: begin pdu_c <= {7'd0, rid_len} + 11'd8 + tlv_vbl; x <= X_TOT2; end
                X_TOT2: begin msg_c <= 11'd3 + {5'd0, comm_len} + 11'd2 + tlv_pdu; x <= X_TOT3; end
                X_TOT3: begin
                    tlv_msg_q <= tlv_msg; msg_big <= tlv_msg > {1'b0, max_len};
                    x <= X_TOT4;
                end
                X_TOT4: begin
                    if (msg_big && !big_done) begin
                        big_done <= 1'b1; too_big <= 1'b1; es <= 8'd1; ei <= 8'd0; n_vb <= 5'd0;
                        vbl_sum <= 11'd0; x <= X_TOT1;
                    end else begin
                        out_len <= tlv_msg_q[10:0]; wptr <= 11'd0; k3 <= 3'd0;
                        h_tag <= 8'h30; h_len <= msg_c; ret <= X_E_VER; x <= X_H_TAG;
                    end
                end
                // ---------------- 头部 ----------------
                X_H_TAG: begin emit(h_tag); h_i <= 2'd0; x <= X_H_LEN; end
                X_H_LEN: begin
                    if (h_len < 11'd128) begin emit(h_len[7:0]); x <= ret; end
                    else if (h_len < 11'd256) begin
                        if (h_i == 2'd0) begin emit(8'h81); h_i <= 2'd1; end
                        else begin emit(h_len[7:0]); x <= ret; end
                    end else begin
                        if (h_i == 2'd0) begin emit(8'h82); h_i <= 2'd1; end
                        else if (h_i == 2'd1) begin emit({5'd0, h_len[10:8]}); h_i <= 2'd2; end
                        else begin emit(h_len[7:0]); x <= ret; end
                    end
                end
                X_E_VER: begin
                    // 02 01 01，然后团体字头部
                    k3 <= k3 + 1'b1;
                    if (k3 == 3'd0) emit(8'h02);
                    else if (k3 == 3'd1) emit(8'h01);
                    else begin
                        emit(8'h01); k3 <= 3'd0;
                        h_tag <= 8'h04; h_len <= {5'd0, comm_len}; ret <= X_E_COMM; x <= X_H_TAG;
                    end
                end
                X_E_COMM: begin
                    cp_src <= {1'b0, comm_off}; cp_n <= {5'd0, comm_len}; cp_from_reg <= !comm_echo;
                    comm_sh <= comm_reg; cp_started <= 1'b1; ret_cp <= X_E_PDU;
                    x <= (comm_len == 6'd0) ? X_E_PDU : X_CP;
                end
                X_E_PDU: begin
                    cp_started <= 1'b0;
                    h_tag <= pdu_tag; h_len <= pdu_c; ret <= X_E_RID; x <= X_H_TAG;
                end
                X_E_RID: begin
                    sv <= rid_sh; sv_n <= rid_len;
                    h_tag <= 8'h02; h_len <= {7'd0, rid_len}; ret <= X_INT; ret_int <= X_E_ERR; x <= X_H_TAG;
                    k3 <= 3'd0;
                end
                X_E_ERR: begin
                    k3 <= k3 + 1'b1;
                    case (k3)
                        3'd0, 3'd3: emit(8'h02);
                        3'd1, 3'd4: emit(8'h01);
                        3'd2: emit(es);
                        default: begin
                            emit(ei); k3 <= 3'd0;
                            h_tag <= 8'h30; h_len <= vbl_sum; vi <= 4'd0;
                            ret <= (n_vb == 5'd0) ? X_DONE : X_E_VB; x <= X_H_TAG;
                        end
                    endcase
                end
                // ---------------- VarBind ----------------
                X_E_VB: x <= X_E_VBL;                       // 描述符 RAM 读延迟
                X_E_VBL: begin
                    iv <= (vbo_rdata[93:86] == 8'h02) ? {{40{vbo_rdata[53]}}, vbo_rdata[53:22]} : {8'd0, vbo_rdata[85:22]};
                    olen <= {4'd0, olvl_q[17:11]}; vlen <= olvl_q[10:0];
                    x <= X_E_OIDH;
                end
                X_E_OIDH: begin
                    vbc <= tlv_ol + tlv_vl;
                    h_tag <= 8'h30; h_len <= tlv_ol + tlv_vl; ret <= X_E_OID; x <= X_H_TAG;
                end
                X_E_OID: begin
                    h_tag <= 8'h06; h_len <= olen; x <= X_H_TAG;
                    if (c_echo) begin
                        cp_src <= {1'b0, c_ooff}; cp_n <= {4'd0, c_olen}; cp_from_reg <= 1'b0; cp_started <= 1'b1;
                        ret <= X_CP; ret_cp <= X_E_VALH;
                    end else begin
                        gj <= 4'd1; gb <= 2'd0; g_wait <= 2'd0; ret <= X_E_GEN;
                    end
                end
                X_E_GEN: begin
                    // gj=1 输出首字节 0x2B（1.3），之后逐弧输出 base-128
                    if (gj == 4'd1) begin emit(8'h2b); gj <= 4'd2; g_wait <= 2'd2; end
                    else if (g_wait != 2'd0) g_wait <= g_wait - 1'b1;
                    else if (gj == g_n_q) x <= X_E_VALH;
                    else begin
                        if (g_nb_q == 2'd1) begin emit({1'b0, g_arc_q2[6:0]}); gj <= gj + 1'b1; g_wait <= 2'd2; end
                        else if (g_nb_q == 2'd2) begin
                            if (gb == 2'd0) begin emit({1'b1, g_arc_q2[13:7]}); gb <= 2'd1; end
                            else begin emit({1'b0, g_arc_q2[6:0]}); gb <= 2'd0; gj <= gj + 1'b1; g_wait <= 2'd2; end
                        end else begin
                            if (gb == 2'd0) begin emit({1'b1, g_arc_q2[20:14]}); gb <= 2'd1; end
                            else if (gb == 2'd1) begin emit({1'b1, g_arc_q2[13:7]}); gb <= 2'd2; end
                            else begin emit({1'b0, g_arc_q2[6:0]}); gb <= 2'd0; gj <= gj + 1'b1; g_wait <= 2'd2; end
                        end
                    end
                end
                X_E_VALH: begin
                    cp_started <= 1'b0;
                    h_tag <= (c_kind == 3'd2 || c_kind == 3'd5) ? 8'h04 : (c_kind == 3'd3) ? 8'h06 : c_tag;
                    h_len <= vlen; x <= X_H_TAG; k3 <= 3'd0; s_i <= 5'd0;
                    sv <= iv_sh; sv_n <= vlen[3:0];
                    case (c_kind)
                        3'd0: ret <= X_E_NEXT;
                        3'd1: begin ret <= X_INT; ret_int <= X_E_NEXT; end
                        3'd5: begin ret <= X_INT; ret_int <= X_E_NEXT; sv <= {c_val, 8'd0}; sv_n <= 4'd8; end
                        3'd4: begin
                            cp_src <= {1'b0, c_voff}; cp_n <= c_vlen; cp_from_reg <= 1'b0; cp_started <= 1'b1;
                            ret <= (c_vlen == 11'd0) ? X_E_NEXT : X_CP; ret_cp <= X_E_NEXT;
                        end
                        default: ret <= X_E_VAL;
                    endcase
                end
                X_E_VAL: begin
                    if (c_kind == 3'd2) begin
                        emit(s_byte); s_i <= s_i + 1'b1;
                        if ({6'd0, s_i} == str_len - 1'b1) x <= X_E_NEXT;
                    end else begin
                        // OID 值：1.3.6.1.3.664 [.7.t]
                        k3 <= k3 + 1'b1;
                        case (k3)
                            3'd0: emit(8'h2b); 3'd1: emit(8'h06); 3'd2: emit(8'h01); 3'd3: emit(8'h03);
                            3'd4: emit(8'h85);
                            3'd5: begin emit(8'h18); if (c_val_zero) x <= X_E_NEXT; end
                            3'd6: emit(8'h07);
                            default: begin emit({1'b0, c_val[6:0]}); x <= X_E_NEXT; end
                        endcase
                    end
                end
                X_E_NEXT: begin
                    cp_started <= 1'b0;
                    if ({1'b0, vi} == n_vb - 1'b1) x <= X_DONE;
                    else begin vi <= vi + 1'b1; x <= X_E_VB; end
                end
                // ---------------- 子序列 ----------------
                X_CP: begin
                    if (cp_from_reg) begin
                        emit(comm_sh[127:120]); comm_sh <= {comm_sh[119:0], 8'd0}; cp_n <= cp_n - 1'b1;
                        if (cp_n == 11'd1) x <= ret_cp;
                    end else if (cp_ok) begin
                        emit(rq_q); cp_src <= cp_src + 1'b1; cp_n <= cp_n - 1'b1;
                        if (cp_n == 11'd1) x <= ret_cp;
                    end
                end
                X_INT: begin
                    emit(sv[71:64]); sv <= {sv[63:0], 8'd0}; sv_n <= sv_n - 1'b1;
                    if (sv_n == 4'd1) x <= ret_int;
                end
                X_DONE: begin done <= 1'b1; x <= X_IDLE; end
                default: x <= X_IDLE;
            endcase
        end
    end
endmodule
