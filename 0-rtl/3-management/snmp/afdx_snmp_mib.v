/*
 * ================================================================
 * File Name    : afdx_snmp_mib.v
 * Module Name  : afdx_snmp_mib
 *
 * Function:
 *   SNMP OID 解码与 MIB 查找。
 *   解码：base-128 子标识，拒绝非最短（0x80 开头）、超过 32 位、截断；首个子标识按 X/40 拆成两弧。
 *         最多保存 16 个弧，更长的 OID 只影响比较结果（本 MIB 条目最多 11 个弧）。
 *   查找：按条目字典序逐弧数值比较（每弧三拍：取数、寄存大小比较、判定），得到 LT/EQ/GT/PREFIX。
 *     条目属性 ROM 同步读出：换条目后先等一拍（M_WAITK）再取弧。
 *     GET/SET（mode=0）：标量 EQ 命中；表列 PREFIX 且恰好多一弧、实例 1..N_TABLE 命中；
 *                       对象存在但实例不存在返回 noSuchInstance，否则 noSuchObject。
 *     GETNEXT（mode=1）：返回字典序严格大于请求 OID 的第一个可见对象实例，末尾返回 endOfMibView。
 *   隐藏条目（仅用于 Trap）对 GET/GETNEXT 不可见。
 *
 * Sub Module:
 *   afdx_snmp_mibrom
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_snmp_mib.vh"
module afdx_snmp_mib #(
    parameter N_TABLE = 128
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        go,
    input  wire        mode,                // 0 精确 1 GETNEXT
    input  wire [10:0] oid_off,
    input  wire [6:0]  oid_len,
    output wire [10:0] rq_addr,
    input  wire [7:0]  rq_data,
    output reg         done,
    output reg         malformed,
    output reg         found,
    output reg  [1:0]  exc,                 // 1 noSuchObject 2 noSuchInstance 3 endOfMibView
    output reg  [6:0]  res_k,
    output reg  [15:0] res_inst
);
    localparam M_IDLE = 3'd0, M_DEC = 3'd1, M_FETCH = 3'd2, M_CMP = 3'd3, M_NEXT = 3'd4, M_END = 3'd5,
               M_CMP0 = 3'd6, M_WAITK = 3'd7;
    localparam O_LT = 2'd0, O_EQ = 2'd1, O_GT = 2'd2, O_PRE = 2'd3;
    reg [2:0] m;
    reg md;
    // ---------------- OID 解码 ----------------
    reg [11:0] pos, dec_last;               // dec_last：OID 末字节地址
    reg pos_w, pos_stable;                // pos_w：上一拍写过 pos；pos_stable：再前一拍也未写
    reg pos_at_last;                      // pos == dec_last 的寄存值（取字节时 pos 已保持两拍）
    reg started;
    assign rq_addr = pos[10:0];
    (* preserve *) reg [7:0] rq_q;          // 读出数据就近寄存（各引擎各一份）；地址保持两拍后取字节
    always @(posedge clk) rq_q <= rq_data;
    wire byte_ok = started && !pos_w && pos_stable;
    wire [7:0] b = rq_q;
    reg [31:0] a [0:15];
    reg [4:0] n;                            // 已保存弧数（最多 16）
    reg more;                               // 超过 16 个弧
    reg [31:0] acc;
    reg first_sub, sub_start;
    wire [31:0] sub_val = {acc[24:0], b[6:0]};
    // Completed arcs are committed one clock after decode. This separates
    // request-byte validation from the enables of the 16 x 32-bit array.
    // The first subidentifier produces two arcs; M_WAITK leaves a full clock
    // for the final pending write before M_FETCH can read the array.
    reg arc_valid, arc_first;
    reg [3:0] arc_index;
    reg [31:0] arc_value, arc_first0, arc_first1;
    always @(posedge clk) begin
        if (rst) arc_valid <= 1'b0;
        else arc_valid <= m == M_DEC && byte_ok && !b[7] &&
                          !(sub_start && b == 8'h80) && acc[31:25] == 7'd0 &&
                          (first_sub || n != 5'd16);
        arc_first <= first_sub;
        arc_index <= n[3:0];
        arc_value <= sub_val;
        arc_first0 <= (sub_val < 32'd40) ? 32'd0 : (sub_val < 32'd80) ? 32'd1 : 32'd2;
        arc_first1 <= (sub_val < 32'd40) ? sub_val : (sub_val < 32'd80) ? sub_val - 32'd40 : sub_val - 32'd80;
        if (!rst && arc_valid) begin
            if (arc_first) begin
                a[0] <= arc_first0;
                a[1] <= arc_first1;
            end else a[arc_index] <= arc_value;
        end
    end
    // ---------------- 条目比较 ----------------
    reg [6:0] k;
    reg [3:0] j;
    wire [31:0] e_arc;
    wire [3:0] elen;
    wire e_table;
    wire [1:0] e_access;
    afdx_snmp_mibrom u_rom (
        .clk      (clk),
        .k        (k),
        .j        (j),
        .inst     (16'd0),
        .arc      (e_arc),
        .elen     (elen),
        .is_table (e_table),
        .vtype    (),
        .access   (e_access),
        .check    ()
    );
    reg [31:0] a_j, e_j;
    reg at_elen, at_n, objm_seen, c_lt, c_gt, ar_hit, ar_more, n_inst;
    reg [1:0] outc;                         // 当前条目的比较结果
    reg [31:0] a_r;                         // 表列实例弧 a[elen]
    wire [4:0] n_eff = more ? 5'd17 : n;
    always @(posedge clk) begin
        if (rst) begin
            m <= M_IDLE; done <= 1'b0; started <= 1'b0; pos <= 12'd0; pos_w <= 1'b1;
            malformed <= 1'b0; found <= 1'b0; exc <= 2'd0; res_k <= 7'd0; res_inst <= 16'd0;
            k <= 7'd0; j <= 4'd0;
        end else begin
            pos_w <= 1'b0; pos_stable <= !pos_w; pos_at_last <= pos == dec_last;
            done <= 1'b0;
            case (m)
                M_IDLE: if (go) begin
                    md <= mode; pos <= {1'b0, oid_off}; pos_w <= 1'b1; started <= 1'b1;
                    dec_last <= {1'b0, oid_off} + {5'd0, oid_len} - 1'b1;
                    n <= 5'd0; more <= 1'b0; acc <= 32'd0; first_sub <= 1'b1; sub_start <= 1'b1;
                    malformed <= 1'b0; found <= 1'b0; exc <= 2'd0; objm_seen <= 1'b0;
                    m <= M_DEC;
                end
                M_DEC: if (byte_ok) begin
                    if ((sub_start && b == 8'h80) || acc[31:25] != 7'd0 || (pos_at_last && b[7])) begin
                        malformed <= 1'b1; m <= M_END;
                    end else begin
                        pos <= pos + 1'b1; pos_w <= 1'b1;
                        if (b[7]) begin acc <= sub_val; sub_start <= 1'b0; end
                        else begin
                            acc <= 32'd0; sub_start <= 1'b1; first_sub <= 1'b0;
                            if (first_sub) begin
                                n <= 5'd2;
                            end else if (n == 5'd16) more <= 1'b1;
                            else n <= n + 1'b1;
                        end
                        if (pos_at_last) begin
                            started <= 1'b0; k <= 7'd0; j <= 4'd0; m <= M_WAITK;
                        end
                    end
                end
                M_FETCH: begin
                    a_j <= a[j]; e_j <= e_arc;
                    at_elen <= j == elen;
                    at_n <= {1'b0, j} == n_eff;
                    if (j == elen) a_r <= ({1'b0, j} < n) ? a[j] : 32'd0;
                    // 标量：比较到最后一弧（.0）之前全部相等且请求至少这么长，即对象存在
                    if (!e_table && j == elen - 1'b1 && {1'b0, j} <= n_eff && e_access != 2'd2) objm_seen <= 1'b1;
                    m <= M_CMP0;
                end
                M_CMP0: begin
                    c_lt <= a_j < e_j; c_gt <= a_j > e_j;
                    ar_hit <= a_r != 32'd0 && a_r <= N_TABLE;
                    ar_more <= a_r < N_TABLE;
                    n_inst <= n_eff == {1'b0, elen} + 5'd1;
                    m <= M_CMP;
                end
                M_CMP: begin
                    // 得到本条目的比较结果后在 M_NEXT 判定
                    if (at_elen) begin outc <= (n_eff == {1'b0, elen}) ? O_EQ : O_PRE; m <= M_NEXT; end
                    else if (at_n || c_lt) begin outc <= O_LT; m <= M_NEXT; end
                    else if (c_gt) begin outc <= O_GT; m <= M_NEXT; end
                    else begin j <= j + 1'b1; m <= M_FETCH; end
                end
                M_WAITK: m <= M_FETCH;
                M_NEXT: begin
                    m <= M_WAITK; j <= 4'd0;
                    if (e_access == 2'd2) begin
                        // 隐藏条目不可见
                    end else if (!md) begin
                        if (!e_table) begin
                            if (outc == O_EQ) begin found <= 1'b1; res_k <= k; res_inst <= 16'd0; m <= M_END; end
                            else if (outc == O_LT) begin exc <= objm_seen ? 2'd2 : 2'd1; m <= M_END; end
                            else if (outc == O_PRE) objm_seen <= 1'b1;
                        end else begin
                            if (outc == O_PRE && n_inst && ar_hit) begin
                                found <= 1'b1; res_k <= k; res_inst <= a_r[15:0]; m <= M_END;
                            end else if (outc == O_PRE || outc == O_EQ) begin
                                exc <= 2'd2; m <= M_END;
                            end else if (outc == O_LT) begin exc <= objm_seen ? 2'd2 : 2'd1; m <= M_END; end
                        end
                    end else begin
                        if (!e_table) begin
                            if (outc == O_LT) begin found <= 1'b1; res_k <= k; res_inst <= 16'd0; m <= M_END; end
                        end else begin
                            if (outc == O_LT || outc == O_EQ) begin
                                found <= 1'b1; res_k <= k; res_inst <= 16'd1; m <= M_END;
                            end else if (outc == O_PRE && ar_more) begin
                                found <= 1'b1; res_k <= k; res_inst <= a_r[15:0] + 1'b1; m <= M_END;
                            end
                        end
                    end
                    k <= k + 1'b1;
                end
                M_END: begin done <= 1'b1; started <= 1'b0; m <= M_IDLE; end
                default: m <= M_IDLE;
            endcase
            // 条目扫描结束：GET 得到 noSuchObject/noSuchInstance，GETNEXT 得到 endOfMibView
            if (m == M_FETCH && k == `MIB_ENTRIES) begin
                exc <= md ? 2'd3 : (objm_seen ? 2'd2 : 2'd1);
                m <= M_END;
            end
        end
    end
endmodule
