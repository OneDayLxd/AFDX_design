/*
 * ================================================================
 * File Name    : afdx_snmp_agent.v
 * Module Name  : afdx_snmp_agent
 *
 * Function:
 *   SNMPv2c 代理：解析 → 逐 VarBind 查找/取值（快照）→ SET 两阶段 → 编码 → 发送。
 *   - GET/GETNEXT：按输入顺序返回；GET 缺失返回 noSuchObject/noSuchInstance，GETNEXT 末尾 endOfMibView；
 *     值在编码前逐条采样到描述符 RAM，编码期间不再变化（64 位计数不会撕裂）。
 *   - SET：第一阶段检查全部 VarBind（写团体字、存在性、可写、类型、长度/编码、取值范围、对象间约束），
 *     任一失败即返回首个失败项的 error-status/error-index 且不写任何对象；全部通过后第二阶段依次写入。
 *     cfgCommand 必须单独成 PDU；提交校验失败返回 inconsistentValue，活动配置不变。
 *   - Trap：sysUpTime.0、snmpTrapOID.0、trapEventCount/First/Last，团体字为读团体字，PDU 0xA7。
 *   值接口：val_k/val_inst 寄存输出，val_data 在 VAL_WAIT 拍后有效（管理面寄存多路选择）。
 *
 * Sub Module:
 *   afdx_snmp_ber
 *   afdx_snmp_mib
 *   afdx_snmp_mibrom
 *   afdx_snmp_enc
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_snmp_mib.vh"
module afdx_snmp_agent #(
    parameter N_TABLE = 128,
    parameter VAL_WAIT = 6,
    parameter [10:0] MAX_PDU = 11'd1471          // 响应上限（= 管理 VL Lmax），超出返回 tooBig
) (
    input  wire         clk,
    input  wire         rst,
    // 请求作业 / Trap 作业
    input  wire         go_req,
    input  wire         go_trap,
    input  wire [2:0]   trap_type,
    output reg          done,
    input  wire [10:0]  req_len,
    input  wire [15:0]  req_sport,
    output wire [10:0]  rq_addr,
    input  wire [7:0]   rq_data,
    // 发送缓冲
    output wire         tb_wr_en,
    output wire [10:0]  tb_wr_addr,
    output wire [7:0]   tb_wr_data,
    output reg          tb_post,
    output reg  [10:0]  tb_len,
    output reg  [15:0]  tb_dport,
    // 配置
    input  wire [127:0] read_comm,
    input  wire [4:0]   read_comm_len,
    input  wire [127:0] write_comm,
    input  wire [4:0]   write_comm_len,
    input  wire [15:0]  trap_port,
    input  wire         cfg_busy,           // 配置/加载忙：相关写入返回 resourceUnavailable
    // MIB 取值 / 写入
    output reg  [6:0]   val_k,
    output reg  [15:0]  val_inst,
    input  wire [63:0]  val_data,
    output reg          wr_en,
    output reg  [6:0]   wr_k,
    output reg  [15:0]  wr_inst,
    output reg  [63:0]  wr_val,
    input  wire         wr_done,
    input  wire         wr_err,
    // 统计
    output reg  [31:0]  cnt_in, cnt_bad_comm, cnt_parse_err, cnt_out, cnt_trap
);
    // 本地复位树：每个子模块一份就近放置的复位寄存器（延迟相同），
    // 避免远处的全局复位副本经长布线驱动各子模块的使能。
    (* preserve, maxfan = 64 *) reg [3:0] rst_l;
    always @(posedge clk) rst_l <= {4{rst}};
    localparam A_IDLE = 5'd0, A_PARSE = 5'd1, A_VBRD = 5'd2, A_VBLD = 5'd3, A_LOOK = 5'd4, A_VAL = 5'd5,
               A_STORE = 5'd6, A_SDEC = 5'd7, A_SCHK = 5'd8, A_NEXT = 5'd9, A_P2RD = 5'd10, A_P2WR = 5'd11,
               A_P2WAIT = 5'd12, A_ENC = 5'd13, A_ENCW = 5'd14, A_POST = 5'd15, A_TRAP = 5'd16,
               A_TVAL = 5'd17, A_TSTORE = 5'd18, A_END = 5'd19, A_P2LD = 5'd20, A_SCHK0 = 5'd21,
               A_SCHK1 = 5'd22;
    localparam E_TOOBIG = 8'd1, E_NOACCESS = 8'd6, E_WRONGTYPE = 8'd7, E_WRONGLEN = 8'd8,
               E_WRONGENC = 8'd9, E_WRONGVAL = 8'd10, E_NOCREATE = 8'd11, E_INCONS = 8'd12,
               E_RESOURCE = 8'd13, E_NOTWRITABLE = 8'd17;
    reg [4:0] a;
    // One-hot owner avoids binary phase decode on the shared RAM address path.
    localparam PH_BER = 4'b0001, PH_MIB = 4'b0010, PH_SET = 4'b0100, PH_ENC = 4'b1000;
    (* preserve, maxfan = 16 *) reg [3:0] ph;
    reg is_trap;
    // ---------------- 子模块 ----------------
    wire ber_done, ber_ok, ber_cw, ber_ovf, ber_vb_we;
    wire [2:0] ber_err;
    wire [10:0] ber_addr, ber_comm_off;
    wire [5:0] ber_comm_len;
    wire [7:0] ber_pdu;
    wire [31:0] ber_rid;
    wire [4:0] ber_n;
    wire [3:0] ber_vb_idx;
    wire [47:0] ber_vb_data;
    reg ber_go;
    afdx_snmp_ber u_ber (
        .clk            (clk),
        .rst            (rst_l[1]),
        .go             (ber_go),
        .req_len        (req_len),
        .rq_addr        (ber_addr),
        .rq_data        (rq_data),
        .read_comm      (read_comm),
        .read_comm_len  (read_comm_len),
        .write_comm     (write_comm),
        .write_comm_len (write_comm_len),
        .done           (ber_done),
        .ok             (ber_ok),
        .err_code       (ber_err),
        .comm_write     (ber_cw),
        .comm_off       (ber_comm_off),
        .comm_len       (ber_comm_len),
        .pdu_type       (ber_pdu),
        .request_id     (ber_rid),
        .vb_count       (ber_n),
        .vb_overflow    (ber_ovf),
        .vb_we          (ber_vb_we),
        .vb_widx        (ber_vb_idx),
        .vb_wdata       (ber_vb_data)
    );
    reg mib_go, mib_mode;
    wire mib_done, mib_bad, mib_found;
    wire [1:0] mib_exc;
    wire [6:0] mib_k;
    wire [15:0] mib_inst;
    wire [10:0] mib_addr;
    reg [10:0] cur_oid_off, cur_voff, cur_vlen;
    reg [6:0] cur_oid_len;
    reg [7:0] cur_vtag;
    afdx_snmp_mib #(
        .N_TABLE (N_TABLE)
    ) u_mib (
        .clk       (clk),
        .rst       (rst_l[2]),
        .go        (mib_go),
        .mode      (mib_mode),
        .oid_off   (cur_oid_off),
        .oid_len   (cur_oid_len),
        .rq_addr   (mib_addr),
        .rq_data   (rq_data),
        .done      (mib_done),
        .malformed (mib_bad),
        .found     (mib_found),
        .exc       (mib_exc),
        .res_k     (mib_k),
        .res_inst  (mib_inst)
    );
    // 条目属性
    reg [6:0] ek;
    wire [7:0] e_type;
    wire [1:0] e_access;
    wire [3:0] e_check;
    // 属性 ROM 同步读出：ek 在 A_LOOK/A_TRAP 写入，最早在下一状态之后才使用
    afdx_snmp_mibrom u_attr (
        .clk      (clk),
        .k        (ek),
        .j        (4'd0),
        .inst     (16'd0),
        .arc      (),
        .elen     (),
        .is_table (),
        .vtype    (e_type),
        .access   (e_access),
        .check    (e_check)
    );
    // 描述符 RAM
    reg [47:0] vbi_mem [0:15];
    reg [47:0] vbi_q;
    reg [3:0] vi;
    always @(posedge clk) begin
        if (ber_vb_we) vbi_mem[ber_vb_idx] <= ber_vb_data;
        vbi_q <= vbi_mem[vi];
    end
    reg [138:0] vbo_mem [0:15];
    reg vbo_we;
    reg [3:0] vbo_wa;
    reg [138:0] vbo_wd;
    wire [3:0] enc_vbo_ra;
    reg [138:0] vbo_q;
    always @(posedge clk) begin
        if (vbo_we) vbo_mem[vbo_wa] <= vbo_wd;
        vbo_q <= vbo_mem[(a == A_ENCW) ? enc_vbo_ra : vi];
    end
    reg enc_go;
    reg [7:0] pdu_out, es, ei;
    reg [31:0] rid_out;
    reg [4:0] n_out;
    wire enc_done;
    wire [10:0] enc_addr, enc_len;
    afdx_snmp_enc u_enc (
        .clk        (clk),
        .rst        (rst_l[3]),
        .go         (enc_go),
        .pdu_tag    (pdu_out),
        .request_id (rid_out),
        .err_status (es),
        .err_index  (ei),
        .vb_n       (n_out),
        .comm_echo  (!is_trap),
        .comm_off   (ber_comm_off),
        .comm_len   (is_trap ? {1'b0, read_comm_len} : ber_comm_len),
        .comm_reg   (read_comm),
        .max_len    (MAX_PDU),
        .vbo_raddr  (enc_vbo_ra),
        .vbo_rdata  (vbo_q),
        .rq_addr    (enc_addr),
        .rq_data    (rq_data),
        .tb_wr_en   (tb_wr_en),
        .tb_wr_addr (tb_wr_addr),
        .tb_wr_data (tb_wr_data),
        .done       (enc_done),
        .out_len    (enc_len),
        .too_big    ()
    );
    // SET 值解码（同步读，数据就近再寄存一拍：地址保持两拍即数据可用）
    reg [11:0] sd_pos, sd_last, sd_end;
    reg sd_stable;                       // 地址已保持两拍（sd_last == sd_pos 的寄存值）
    (* preserve *) reg [7:0] sd_b;
    always @(posedge clk) sd_b <= rq_data;
    reg [63:0] sd_val;
    reg [3:0] sd_i;
    reg sd_bad_enc;
    reg [7:0] sd_first;
    wire sd_ok = ph[2] && sd_last == sd_pos && sd_stable;
    assign rq_addr = ({11{ph[0]}} & ber_addr) | ({11{ph[1]}} & mib_addr) |
                     ({11{ph[2]}} & sd_pos[10:0]) | ({11{ph[3]}} & enc_addr);
    // ---------------- 控制 ----------------
    reg [3:0] wait_cnt;
    reg [2:0] tv;                                   // Trap 变量序号
    reg [2:0] ttype;
    reg [31:0] trap_seq;
    reg set_pdu, next_pdu, has_cmd, has_vl;
    reg [7:0] fail_es;
    reg [3:0] fail_idx;
    wire [4:0] vb_total = ber_n;
    // 变量值种类
    wire [2:0] kind_of = (e_type == 8'h04) ? ((ek == `MIB_RTCSET) ? 3'd5 : 3'd2) :
                         (e_type == 8'h06) ? 3'd3 : 3'd1;
    wire [63:0] str_id = (ek == `MIB_SYSNAME) ? 64'd1 : 64'd0;
    // 写入取值范围：A_SCHK0 寄存各范围判定，A_SCHK1 按检查类别选择
    wire [63:0] v = sd_val;
    reg r_eq1, r_1_3, r_port, r_bag, r_lmax, r_nz, r_neg;
    reg range_ok;
    always @* begin
        case (e_check)
            4'd1: range_ok = r_eq1;
            4'd2, 4'd7: range_ok = r_1_3;
            4'd3: range_ok = r_port;
            4'd5: range_ok = r_bag;
            4'd6: range_ok = r_lmax;
            4'd9: range_ok = r_nz;
            default: range_ok = 1'b1;
        endcase
    end
    wire is_vl_col = (ek == `MIB_VLSHADOWBAGUS) || (ek == `MIB_VLSHADOWLMAX) || (ek == `MIB_VLSHADOWNETEN);
    reg cur_found;
    // 第一阶段检查结果（按 RFC 3416 的检查顺序）
    wire len_bad = (e_type == 8'h04 && cur_vlen != 11'd8) ||
                   (e_type == 8'h02 && (cur_vlen == 11'd0 || cur_vlen > 11'd4)) ||
                   (e_type == 8'h42 && (cur_vlen == 11'd0 || cur_vlen > 11'd5 ||
                                        (cur_vlen == 11'd5 && sd_first != 8'h00)));
    // 各项在 A_SCHK0 寄存，A_SCHK 只做优先级编码
    reg f_noacc, f_nocreate, f_nw, f_type, f_len, f_enc, f_val, f_res, f_incons, f_cmd, f_vl;
    wire [7:0] chk_es = f_noacc ? E_NOACCESS : f_nocreate ? E_NOCREATE : f_nw ? E_NOTWRITABLE :
                        f_type ? E_WRONGTYPE : f_len ? E_WRONGLEN : f_enc ? E_WRONGENC :
                        f_val ? E_WRONGVAL : f_res ? E_RESOURCE : f_incons ? E_INCONS : 8'd0;

    always @(posedge clk) begin
        if (rst_l[0]) begin
            a <= A_IDLE; ph <= PH_BER; done <= 1'b0; ber_go <= 1'b0; mib_go <= 1'b0; enc_go <= 1'b0;
            vbo_we <= 1'b0; tb_post <= 1'b0; wr_en <= 1'b0; trap_seq <= 32'd1; is_trap <= 1'b0;
            cnt_in <= 0; cnt_bad_comm <= 0; cnt_parse_err <= 0; cnt_out <= 0; cnt_trap <= 0;
            sd_last <= 12'hfff; sd_pos <= 12'd0; vi <= 4'd0;
        end else begin
            done <= 1'b0; ber_go <= 1'b0; mib_go <= 1'b0; enc_go <= 1'b0; vbo_we <= 1'b0;
            tb_post <= 1'b0; wr_en <= 1'b0;
            sd_last <= sd_pos; sd_stable <= sd_last == sd_pos;
            case (a)
                A_IDLE: begin
                    ph <= PH_BER; vi <= 4'd0;
                    if (go_req) begin
                        is_trap <= 1'b0; ber_go <= 1'b1; cnt_in <= cnt_in + 1'b1; a <= A_PARSE;
                    end else if (go_trap) begin
                        is_trap <= 1'b1; ttype <= trap_type; tv <= 3'd0; ph <= PH_ENC; a <= A_TRAP;
                    end
                end
                A_PARSE: if (ber_done) begin
                    if (!ber_ok) begin
                        if (ber_err == 3'd3) cnt_bad_comm <= cnt_bad_comm + 1'b1;
                        else cnt_parse_err <= cnt_parse_err + 1'b1;
                        a <= A_END;
                    end else begin
                        set_pdu <= ber_pdu == 8'ha3; next_pdu <= ber_pdu == 8'ha1;
                        has_cmd <= 1'b0; has_vl <= 1'b0; fail_es <= 8'd0; fail_idx <= 4'd0;
                        pdu_out <= 8'ha2; rid_out <= ber_rid; es <= 8'd0; ei <= 8'd0; n_out <= ber_n;
                        if (ber_ovf) begin es <= E_TOOBIG; n_out <= 5'd0; a <= A_ENC; end
                        else if (ber_n == 5'd0) a <= A_ENC;
                        else begin ph <= PH_MIB; vi <= 4'd0; a <= A_VBRD; end
                    end
                end
                A_VBRD: a <= A_VBLD;                        // vbi RAM 读延迟
                A_VBLD: begin
                    {cur_oid_off, cur_oid_len, cur_vtag, cur_voff, cur_vlen} <= vbi_q;
                    mib_mode <= next_pdu; mib_go <= 1'b1; a <= A_LOOK;
                end
                A_LOOK: if (mib_done) begin
                    ek <= mib_k;
                    if (mib_bad) begin cnt_parse_err <= cnt_parse_err + 1'b1; a <= A_END; end
                    else if (set_pdu) begin
                        // 每项都解码并回显；只有首个失败项决定 error-status/index
                        cur_found <= mib_found; val_k <= mib_k; val_inst <= mib_inst;
                        sd_pos <= {1'b0, cur_voff}; sd_last <= 12'hfff;
                        sd_end <= {1'b0, cur_voff} + {1'b0, cur_vlen};
                        sd_val <= 64'd0; sd_i <= 4'd0; sd_bad_enc <= 1'b0; ph <= PH_SET;
                        a <= A_SDEC;
                    end else if (mib_found) begin
                        val_k <= mib_k; val_inst <= mib_inst; wait_cnt <= VAL_WAIT[3:0]; a <= A_VAL;
                    end else begin
                        // 异常值：OID 回显
                        vbo_we <= 1'b1; vbo_wa <= vi;
                        vbo_wd <= {1'b1, cur_oid_off, cur_oid_len, 7'd0, 16'd0, 3'd0,
                                   (mib_exc == 2'd3) ? 8'h82 : (mib_exc == 2'd2) ? 8'h81 : 8'h80,
                                   64'd0, 11'd0, 11'd0};
                        a <= A_NEXT;
                    end
                end
                A_VAL: begin
                    if (wait_cnt != 4'd0) wait_cnt <= wait_cnt - 1'b1;
                    else a <= A_STORE;
                end
                A_STORE: begin
                    vbo_we <= 1'b1; vbo_wa <= vi;
                    vbo_wd <= {!next_pdu, cur_oid_off, cur_oid_len, val_k, val_inst, kind_of, e_type,
                               (kind_of == 3'd2) ? str_id : val_data, 11'd0, 11'd0};
                    a <= A_NEXT;
                end
                A_SDEC: begin
                    // 读取 SET 值字节（最多 8 字节），检查整数最短编码
                    if (sd_pos == sd_end || sd_i == 4'd9) a <= A_SCHK0;
                    else if (sd_ok) begin
                        sd_pos <= sd_pos + 1'b1; sd_i <= sd_i + 1'b1;
                        sd_val <= {sd_val[55:0], sd_b};
                        if (sd_i == 4'd0) begin
                            sd_first <= sd_b;
                            if (cur_vtag == 8'h02 && sd_b[7]) sd_val <= {56'hffffffffffffff, sd_b};
                        end
                        if (sd_i == 4'd1 && ((sd_first == 8'h00 && !sd_b[7]) ||
                                             (cur_vtag == 8'h02 && sd_first == 8'hff && sd_b[7])))
                            sd_bad_enc <= 1'b1;
                    end
                end
                A_SCHK0: begin
                    f_noacc <= !ber_cw;
                    f_nocreate <= !cur_found;
                    f_nw <= e_access != 2'd1;
                    f_type <= cur_vtag != e_type;
                    f_len <= len_bad;
                    f_enc <= e_type != 8'h04 && sd_bad_enc;
                    r_eq1 <= v == 64'd1;
                    r_1_3 <= v[63:2] == 62'd0 && v[1:0] != 2'd0;
                    r_port <= v[63:16] == 48'd0 && v[15:0] != 16'd0;
                    r_bag <= v[63:18] == 46'd0 && (v[17:0] == 18'd1000 || v[17:0] == 18'd2000 ||
                             v[17:0] == 18'd4000 || v[17:0] == 18'd8000 || v[17:0] == 18'd16000 ||
                             v[17:0] == 18'd32000 || v[17:0] == 18'd64000 || v[17:0] == 18'd128000);
                    r_lmax <= v[63:11] == 53'd0 && v[10:0] != 11'd0 && v[10:0] <= 11'd1471;
                    r_nz <= v != 64'd0;
                    r_neg <= e_type == 8'h02 && sd_val[63];
                    f_res <= (is_vl_col || ek == `MIB_CFGCOMMAND) && cfg_busy;
                    f_incons <= (ek == `MIB_CFGCOMMAND && (has_vl || vb_total != 5'd1)) || (is_vl_col && has_cmd);
                    f_cmd <= cur_found && ek == `MIB_CFGCOMMAND;
                    f_vl <= cur_found && is_vl_col;
                    a <= A_SCHK1;
                end
                A_SCHK1: begin f_val <= r_neg || !range_ok; a <= A_SCHK; end
                A_SCHK: begin
                    ph <= PH_MIB;
                    if (fail_es == 8'd0 && chk_es != 8'd0) begin fail_es <= chk_es; fail_idx <= vi; end
                    if (f_cmd) has_cmd <= 1'b1;
                    if (f_vl) has_vl <= 1'b1;
                    // 回显请求值，vval 暂存解码值供第二阶段写入
                    vbo_we <= 1'b1; vbo_wa <= vi;
                    vbo_wd <= {1'b1, cur_oid_off, cur_oid_len, ek, val_inst, 3'd4, cur_vtag, sd_val,
                               cur_voff, cur_vlen};
                    a <= A_NEXT;
                end
                A_NEXT: begin
                    if ({1'b0, vi} == vb_total - 1'b1) begin
                        vi <= 4'd0;
                        if (set_pdu && fail_es != 8'd0) begin
                            es <= fail_es; ei <= {4'd0, fail_idx} + 8'd1; a <= A_ENC;
                        end else if (set_pdu) a <= A_P2RD;
                        else a <= A_ENC;
                    end else begin vi <= vi + 1'b1; a <= A_VBRD; end
                end
                // ---------------- SET 第二阶段 ----------------
                A_P2RD: a <= A_P2LD;
                A_P2LD: begin
                    wr_k <= vbo_q[119:113]; wr_inst <= vbo_q[112:97]; wr_val <= vbo_q[85:22];
                    a <= A_P2WR;
                end
                A_P2WR: begin wr_en <= 1'b1; a <= A_P2WAIT; end
                A_P2WAIT: if (wr_done) begin
                    if (wr_err) begin es <= E_INCONS; ei <= {4'd0, vi} + 8'd1; a <= A_ENC; end
                    else if ({1'b0, vi} == vb_total - 1'b1) a <= A_ENC;
                    else begin vi <= vi + 1'b1; a <= A_P2RD; end
                end
                // ---------------- 编码与发送 ----------------
                A_ENC: begin ph <= PH_ENC; enc_go <= 1'b1; a <= A_ENCW; end
                A_ENCW: if (enc_done) begin
                    tb_len <= enc_len; tb_dport <= is_trap ? trap_port : req_sport; a <= A_POST;
                end
                A_POST: begin
                    tb_post <= 1'b1; cnt_out <= cnt_out + 1'b1;
                    if (is_trap) cnt_trap <= cnt_trap + 1'b1;
                    a <= A_END;
                end
                // ---------------- Trap ----------------
                A_TRAP: begin
                    case (tv)
                        3'd0: ek <= `MIB_SYSUPTIME;
                        3'd1: ek <= `MIB_SNMPTRAPOID;
                        3'd2: ek <= `MIB_TRAPEVENTCOUNT;
                        3'd3: ek <= `MIB_TRAPEVENTFIRST;
                        default: ek <= `MIB_TRAPEVENTLAST;
                    endcase
                    case (tv)
                        3'd0: val_k <= `MIB_SYSUPTIME;
                        3'd1: val_k <= `MIB_SNMPTRAPOID;
                        3'd2: val_k <= `MIB_TRAPEVENTCOUNT;
                        3'd3: val_k <= `MIB_TRAPEVENTFIRST;
                        default: val_k <= `MIB_TRAPEVENTLAST;
                    endcase
                    val_inst <= 16'd0; wait_cnt <= VAL_WAIT[3:0]; a <= A_TVAL;
                end
                A_TVAL: begin
                    if (wait_cnt != 4'd0) wait_cnt <= wait_cnt - 1'b1;
                    else a <= A_TSTORE;
                end
                A_TSTORE: begin
                    vbo_we <= 1'b1; vbo_wa <= {1'b0, tv};
                    vbo_wd <= {1'b0, 11'd0, 7'd0, ek, 16'd0, kind_of, e_type,
                               (tv == 3'd1) ? {61'd0, ttype} : val_data, 11'd0, 11'd0};
                    if (tv == 3'd4) begin
                        pdu_out <= 8'ha7; rid_out <= trap_seq; trap_seq <= trap_seq + 1'b1;
                        es <= 8'd0; ei <= 8'd0; n_out <= 5'd5; a <= A_ENC;
                    end else begin tv <= tv + 1'b1; a <= A_TRAP; end
                end
                A_END: begin done <= 1'b1; ph <= PH_BER; a <= A_IDLE; end
                default: a <= A_IDLE;
            endcase
        end
    end
endmodule
