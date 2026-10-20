/*
 * ================================================================
 * File Name    : afdx_tftp_server.v
 * Module Name  : afdx_tftp_server
 *
 * Function:
 *   TFTP 服务器（RFC 1350 基本报文：RRQ/WRQ/DATA/ACK/ERROR，octet 模式，512 字节块）。
 *   - 初始请求到 UDP 69；每个会话分配服务器 TID（TID_BASE 起的 16 个端口），五元组不符返回 ERROR 5；
 *   - 文件名严格白名单映射到对象（见 afdx_loader），拒绝未知名、方向不符；模式不区分大小写且只接受 octet；
 *     请求中的选项（RFC 2347）被忽略，不回 OACK；
 *   - 写会话：只接受期望块并写入一次，重复上一块只重发 ACK，旧块忽略，未来块/超长块 ERROR 4 终止；
 *     小于 512 字节的 DATA 结束传输，镜像校验通过才回最后的 ACK，否则回 ERROR；最后 ACK 之后
 *     停留一个超时周期以重发 ACK；
 *   - 读会话：DATA 块号从 1 开始，ACK 当前块才推进，重复/旧 ACK 忽略（避免 Sorcerer's Apprentice）；
 *     对象长度为 512 整数倍时追加 0 字节 DATA；
 *   - 超时（单调时间）后原样重传最后报文（按块号重新生成），重试 MAX_RETRIES 次后终止并放弃暂存写入。
 *   两个会话槽，最多一个暂存写入（配置/应用镜像）；控制对象 control/abort 可在暂存写入期间并行。
 *   时序：会话超时用微秒节拍递减计数（发送时装载）；会话报文的块号/操作码比较先在 T_CMP 寄存；
 *        文件名白名单字节按当前字符序号预取一拍；请求长度/端口与会话表的比较每拍预先寄存
 *        （请求元数据在作业期间不变）；请求读口数据就近寄存一拍，地址保持两拍后取字节。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_tftp_server #(
    parameter [15:0] TID_BASE = 16'hC000,
    parameter TIMEOUT_US = 1000000,
    parameter MAX_RETRIES = 5
) (
    input  wire         clk,
    input  wire         rst,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]  mono_us,            // 只用最低位检测微秒节拍
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire         go,                 // 请求作业
    input  wire         go_timer,           // 超时作业
    output reg          done,
    output wire         timer_due,
    input  wire [10:0]  req_len,
    input  wire [15:0]  req_sport,
    input  wire [15:0]  req_dport,
    output wire [10:0]  rq_addr,
    input  wire [7:0]   rq_data,
    output reg          tb_wr_en,
    output reg  [10:0]  tb_wr_addr,
    output reg  [7:0]   tb_wr_data,
    output reg          tb_post,
    output reg  [10:0]  tb_len,
    output reg  [15:0]  tb_sport,
    output reg  [15:0]  tb_dport,
    // 对象管理器
    output reg          ld_open,
    output reg  [2:0]   ld_obj,
    output reg          ld_write,
    input  wire         ld_open_ack,
    input  wire [4:0]   ld_open_err,
    input  wire [23:0]  ld_size,
    output reg          ld_wvalid,
    output reg          ld_wsel,
    output reg  [7:0]   ld_wbyte,
    input  wire [4:0]   ld_werr,
    output reg          ld_finish,
    input  wire         ld_fin_ack,
    input  wire [4:0]   ld_fin_err,
    output reg          ld_abort,
    output reg  [2:0]   ld_rd_obj,
    output reg  [23:0]  ld_rd_addr,
    input  wire [7:0]   ld_rd_data,
    output reg          ld_rd_close,
    input  wire         ld_abort_evt,
    output wire [15:0]  cur_block,
    output reg  [31:0]  cnt_retries,
    output reg  [31:0]  cnt_errors,
    output reg          ev_fault           // 重试耗尽 / 对端 ERROR / 控制记录错误；暂存写入的失败由对象管理器上报
);
    localparam [15:0] PORT69 = 16'd69;
    localparam O_CFG = 3'd1, O_APP = 3'd2, O_STATUS = 3'd3, O_FAULT = 3'd4, O_CTRL = 3'd5;
    localparam T_IDLE = 5'd0, T_OP = 5'd1, T_NAME = 5'd2, T_MODE = 5'd3, T_NEW = 5'd4, T_OPENW = 5'd5,
               T_SESS = 5'd6, T_RACK = 5'd7, T_WDATA = 5'd8, T_WCHK = 5'd9, T_FINW = 5'd10,
               T_ACK = 5'd11, T_ERR = 5'd12, T_DATA = 5'd13, T_DHDR = 5'd14, T_DSTREAM = 5'd15,
               T_POST = 5'd16, T_END = 5'd17, T_TIMER = 5'd18, T_CMP = 5'd19, T_TIMER0 = 5'd20;
    // 错误消息号
    localparam M_BUSY = 3'd0, M_NOFILE = 3'd1, M_ACCESS = 3'd2, M_ILLEGAL = 3'd3, M_TID = 3'd4,
               M_MODE = 3'd5, M_LOAD = 3'd6;
    reg [4:0] t;
    // ---------------- 会话 ----------------
    reg s_act [0:1], s_dir [0:1], s_final [0:1];
    reg [2:0] s_obj [0:1];
    reg [15:0] s_cport [0:1], s_tid [0:1], s_blk [0:1];
    reg [23:0] s_size [0:1];
    reg [3:0] s_retry [0:1];
    reg [31:0] s_left [0:1];                    // 距超时剩余微秒
    reg s_zero [0:1];                           // s_left == 0（寄存，装载时同拍清零）
    reg mono0_q;
    wire tick = mono_us[0] != mono0_q;
    reg cs;                                     // 当前会话
    reg [3:0] tid_ctr;
    wire due0 = s_act[0] && s_zero[0];
    wire due1 = s_act[1] && s_zero[1];
    assign timer_due = due0 || due1;
    assign cur_block = (s_act[1] && s_dir[1] && s_obj[1] != O_CTRL) ? s_blk[1] : s_blk[0];
    wire [3:0] new_tid = (s_act[~cs] && s_tid[~cs][3:0] == tid_ctr) ? tid_ctr + 1'b1 : tid_ctr;
    // ---------------- 请求读取 ----------------
    reg [11:0] pos, last_pos;
    reg pos_stable;                       // 地址已保持两拍（last_pos == pos 的寄存值）
    reg rd_on;
    assign rq_addr = pos[10:0];
    (* preserve *) reg [7:0] rq_q;              // 各引擎各自一份，就近布局
    always @(posedge clk) rq_q <= rq_data;
    wire byte_ok = rd_on && last_pos == pos && pos_stable;
    wire [7:0] b = rq_q;
    reg [15:0] op, blk;
    reg [1:0] opi;
    // 文件名白名单（26 字节，高位在前，NUL 填充）
    localparam [207:0] N1 = {"config/afdx-vl-config.bin", 8'd0};
    localparam [207:0] N2 = {"application/app-image.bin", 8'd0};
    localparam [207:0] N3 = {"status/loader-status.bin", 16'd0};
    localparam [207:0] N4 = {"diagnostic/fault-log.bin", 16'd0};
    localparam [207:0] N5 = {"control/abort", 104'd0};
    reg [4:0] cand;
    reg [5:0] ni;
    // 白名单字节按 ni 预取（ni 在取字节后变化，下一字节至少两拍后才可用）
    reg [7:0] n1b, n2b, n3b, n4b, n5b;
    reg ni_long, ni_13, ni_24, ni_25, ni_40;
    function [7:0] nbyte;                       // 常量名第 idx 字节（常量移位，综合为 5 输入查找表）
        input [207:0] nm;
        input [4:0] idx;
        reg [207:0] sh;
        begin
            sh = nm << {idx, 3'b000};
            nbyte = (idx > 5'd25) ? 8'd0 : sh[207:200];
        end
    endfunction
    // 字符序号相关的判定同样预取（ni 变化后至少三拍才取下一字节）
    always @(posedge clk) begin
        n1b <= nbyte(N1, ni[4:0]); n2b <= nbyte(N2, ni[4:0]); n3b <= nbyte(N3, ni[4:0]);
        n4b <= nbyte(N4, ni[4:0]); n5b <= nbyte(N5, ni[4:0]);
        ni_long <= ni > 6'd25;
        ni_13 <= ni == 6'd13; ni_24 <= ni == 6'd24; ni_25 <= ni == 6'd25; ni_40 <= ni == 6'd40;
    end
    // T_CMP 寄存的会话报文判定
    reg k_err, k_data, k_ack, k_long, k_len4, k_final, k_dir, k_ctrl, k_same, k_next, k_future;
    reg [2:0] name_obj;
    reg [2:0] mi;
    reg mode_ok;
    wire [7:0] lb = b | 8'h20;
    wire [7:0] octet_b = (mi == 3'd0) ? "o" : (mi == 3'd1) ? "c" : (mi == 3'd2) ? "t" : (mi == 3'd3) ? "e" : "t";
    // 每拍预先寄存的请求判定（请求元数据在作业期间保持；位置比较滞后一拍，
    // 但字节只在地址保持两拍后才被取用，判定总是对应当前位置）
    reg q_short, q_69, q_at_end, q_lt516, q_long, q_len4;
    reg hit0, hit1, cp0, cp1, dup0, dup1, q_badop, q_access;
    always @(posedge clk) begin
        q_short <= req_len < 11'd4 && !(req_dport == PORT69 && req_len >= 11'd2);
        q_69 <= req_dport == PORT69;
        q_at_end <= pos >= {1'b0, req_len};
        q_lt516 <= req_len < 11'd516;
        q_long <= req_len > 11'd516;
        q_len4 <= req_len == 11'd4;
        hit0 <= s_act[0] && s_tid[0] == req_dport;
        hit1 <= s_act[1] && s_tid[1] == req_dport;
        cp0 <= s_cport[0] == req_sport;
        cp1 <= s_cport[1] == req_sport;
        // 重复的初始请求：原会话尚未推进
        dup0 <= s_act[0] && s_cport[0] == req_sport && s_obj[0] == name_obj && s_dir[0] == (op == 16'd2) &&
                ((op == 16'd1) ? s_blk[0] == 16'd1 : s_blk[0] == 16'd0);
        dup1 <= s_act[1] && s_cport[1] == req_sport && s_obj[1] == name_obj && s_dir[1] == (op == 16'd2) &&
                ((op == 16'd1) ? s_blk[1] == 16'd1 : s_blk[1] == 16'd0);
        q_badop <= op != 16'd1 && op != 16'd2;
        q_access <= (op == 16'd1 && name_obj == O_CTRL) ||
                    (op == 16'd2 && (name_obj == O_STATUS || name_obj == O_FAULT));
    end
    // ---------------- 发送 ----------------
    reg [2:0] err_code, err_msg;
    // Error code is determined by the message class. Decode it every clock,
    // without the session-decision priority chain on a register enable.
    // T_ERR emits the code only at ei=3, after this decode has settled.
    reg err_full;
    always @(posedge clk) begin
        if (t == T_WCHK) err_full <= 1'b0;
        else if (t == T_FINW) err_full <= ld_fin_err == 5'd16;
        case (err_msg)
            M_NOFILE: err_code <= 3'd1;
            M_ACCESS: err_code <= 3'd2;
            M_ILLEGAL: err_code <= 3'd4;
            M_TID: err_code <= 3'd5;
            M_LOAD: err_code <= err_full ? 3'd3 : 3'd0;
            default: err_code <= 3'd0;
        endcase
    end
    reg [4:0] err_ld;
    reg [4:0] ei;
    reg [15:0] e_blk;
    reg free_after;
    reg [3:0] wait_n;
    reg [10:0] w_remain;
    reg [9:0] d_issue, d_got, d_len;
    reg [4:0] d_v;
    reg d_stream_done;
    // Clear outside the stream so the previous block (including an empty
    // tail block) cannot complete a new transfer with stale counters.
    always @(posedge clk) begin
        if (rst || t != T_DSTREAM) d_stream_done <= 1'b0;
        else d_stream_done <= (d_got == d_len && d_issue == d_len);
    end
    reg [10:0] data_packet_len;
    always @(posedge clk) data_packet_len <= {1'b0, d_len} + 11'd4;
    reg [23:0] d_base, d_rem;
    // 错误消息字符（ei = 4.. 为消息第 0.. 字节，末尾 NUL；直接按 ei 译码）
    reg [7:0] ld_hi, ld_lo;                     // 加载错误码的两位十六进制字符（err_ld 设置后至少四拍才用到）
    always @(posedge clk) begin
        ld_hi <= err_ld[4] ? "1" : "0";
        ld_lo <= (err_ld[3:0] < 4'd10) ? (8'h30 + {4'd0, err_ld[3:0]}) : (8'h37 + {4'd0, err_ld[3:0]});
    end
    reg [7:0] emsg;
    always @* begin
        emsg = 8'd0;
        case (err_msg)
            M_BUSY:    case (ei) 5'd4: emsg = "B"; 5'd5: emsg = "U"; 5'd6: emsg = "S"; 5'd7: emsg = "Y"; default: ; endcase
            M_NOFILE:  case (ei) 5'd4: emsg = "N"; 5'd5: emsg = "O"; 5'd6: emsg = "F"; 5'd7: emsg = "I";
                                 5'd8: emsg = "L"; 5'd9: emsg = "E"; default: ; endcase
            M_ACCESS:  case (ei) 5'd4: emsg = "A"; 5'd5: emsg = "C"; 5'd6: emsg = "C"; 5'd7: emsg = "E";
                                 5'd8: emsg = "S"; 5'd9: emsg = "S"; default: ; endcase
            M_ILLEGAL: case (ei) 5'd4: emsg = "I"; 5'd5: emsg = "L"; 5'd6: emsg = "L"; 5'd7: emsg = "E";
                                 5'd8: emsg = "G"; 5'd9: emsg = "A"; 5'd10: emsg = "L"; default: ; endcase
            M_TID:     case (ei) 5'd4: emsg = "T"; 5'd5: emsg = "I"; 5'd6: emsg = "D"; default: ; endcase
            M_MODE:    case (ei) 5'd4: emsg = "M"; 5'd5: emsg = "O"; 5'd6: emsg = "D"; 5'd7: emsg = "E"; default: ; endcase
            default:   case (ei) 5'd4: emsg = "L"; 5'd5: emsg = "O"; 5'd6: emsg = "A"; 5'd7: emsg = "D";
                                 5'd8: emsg = " "; 5'd9: emsg = ld_hi; 5'd10: emsg = ld_lo; default: ; endcase
        endcase
    end
    // 错误消息结束位置（末尾 NUL 所在的 ei）：4 + 消息长度
    wire [4:0] e_end = (err_msg == M_BUSY || err_msg == M_MODE) ? 5'd8 : (err_msg == M_TID) ? 5'd7 :
                       (err_msg == M_NOFILE || err_msg == M_ACCESS) ? 5'd10 : 5'd11;

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            t <= T_IDLE; done <= 1'b0; tb_wr_en <= 1'b0; tb_post <= 1'b0; rd_on <= 1'b0;
            pos <= 12'd0; last_pos <= 12'hfff; tid_ctr <= 4'd0; cs <= 1'b0;
            ld_open <= 1'b0; ld_wvalid <= 1'b0; ld_finish <= 1'b0; ld_abort <= 1'b0; ld_rd_close <= 1'b0;
            ld_wsel <= 1'b0; cnt_retries <= 0; cnt_errors <= 0; ev_fault <= 1'b0;
            for (i = 0; i < 2; i = i + 1) begin
                s_act[i] <= 1'b0; s_final[i] <= 1'b0; s_dir[i] <= 1'b0; s_retry[i] <= 4'd0; s_blk[i] <= 16'd0;
                s_tid[i] <= 16'd0; s_left[i] <= 32'd0; s_zero[i] <= 1'b1;
            end
        end else begin
            done <= 1'b0; tb_wr_en <= 1'b0; tb_post <= 1'b0; ld_open <= 1'b0; ld_wvalid <= 1'b0;
            ld_finish <= 1'b0; ld_abort <= 1'b0; ld_rd_close <= 1'b0; ev_fault <= 1'b0;
            last_pos <= pos; pos_stable <= last_pos == pos;
            mono0_q <= mono_us[0];
            for (i = 0; i < 2; i = i + 1) begin
                if (tick && !s_zero[i]) s_left[i] <= s_left[i] - 1'b1;
                s_zero[i] <= s_left[i] == 32'd0;
            end
            // 暂存写入被 control/abort 或 SNMP 终止：释放对应写会话
            if (ld_abort_evt)
                for (i = 0; i < 2; i = i + 1)
                    if (s_act[i] && s_dir[i] && s_obj[i] != O_CTRL) s_act[i] <= 1'b0;
            case (t)
                T_IDLE: begin
                    free_after <= 1'b0;
                    if (go) begin
                        pos <= 12'd0; last_pos <= 12'hfff; rd_on <= 1'b1; opi <= 2'd0;
                        ni <= 6'd0;                 // 使预取的 ni 判定在进入 T_NAME 前已更新
                        t <= T_OP;
                    end else if (go_timer) begin
                        cs <= due0 ? 1'b0 : 1'b1; t <= T_TIMER0;
                    end
                end
                // ---------------- 请求解析 ----------------
                T_OP: if (q_short) begin
                    rd_on <= 1'b0; t <= T_END;                       // 截断报文：丢弃
                end else if (byte_ok) begin
                    pos <= pos + 1'b1; opi <= opi + 1'b1;
                    case (opi)
                        2'd0: op[15:8] <= b;
                        2'd1: op[7:0] <= b;
                        2'd2: blk[15:8] <= b;
                        default: blk[7:0] <= b;
                    endcase
                    if (q_69 && opi == 2'd1) begin
                        cand <= 5'b11111; ni <= 6'd0; t <= T_NAME;
                    end else if (opi == 2'd3) begin rd_on <= 1'b0; t <= T_SESS; end
                end
                T_NAME: if (q_at_end || ni_40) begin
                    rd_on <= 1'b0; err_msg <= M_ILLEGAL; ei <= 5'd0; t <= T_ERR;
                end else if (byte_ok) begin
                    pos <= pos + 1'b1; ni <= ni + 1'b1;
                    if (b == 8'd0) begin
                        name_obj <= (cand[0] && ni_25) ? O_CFG : (cand[1] && ni_25) ? O_APP :
                                    (cand[2] && ni_24) ? O_STATUS : (cand[3] && ni_24) ? O_FAULT :
                                    (cand[4] && ni_13) ? O_CTRL : 3'd0;
                        mi <= 3'd0; mode_ok <= 1'b1; t <= T_MODE;
                    end else begin
                        if (ni_long || b != n1b) cand[0] <= 1'b0;
                        if (ni_long || b != n2b) cand[1] <= 1'b0;
                        if (ni_long || b != n3b) cand[2] <= 1'b0;
                        if (ni_long || b != n4b) cand[3] <= 1'b0;
                        if (ni_long || b != n5b) cand[4] <= 1'b0;
                    end
                end
                T_MODE: if (q_at_end) begin
                    rd_on <= 1'b0; err_msg <= M_ILLEGAL; ei <= 5'd0; t <= T_ERR;
                end else if (byte_ok) begin
                    pos <= pos + 1'b1; if (mi != 3'd7) mi <= mi + 1'b1;
                    if (b == 8'd0) begin
                        rd_on <= 1'b0; t <= T_NEW;
                        if (mi != 3'd5) mode_ok <= 1'b0;
                    end else if (mi >= 3'd5 || lb != octet_b) mode_ok <= 1'b0;
                end
                T_NEW: begin
                    ei <= 5'd0;
                    if (q_badop) begin err_msg <= M_ILLEGAL; t <= T_ERR; end
                    else if (!mode_ok) begin err_msg <= M_MODE; t <= T_ERR; end
                    else if (name_obj == 3'd0) begin err_msg <= M_NOFILE; t <= T_ERR; end
                    else if (q_access) begin err_msg <= M_ACCESS; t <= T_ERR; end
                    // 重复的初始请求：原会话尚未推进时重发首个报文
                    else if (dup0) begin cs <= 1'b0; t <= op[0] ? T_DATA : T_ACK; end
                    else if (dup1) begin cs <= 1'b1; t <= op[0] ? T_DATA : T_ACK; end
                    else if (s_act[0] && s_act[1]) begin err_msg <= M_BUSY; t <= T_ERR; end
                    else begin
                        cs <= s_act[0];
                        ld_open <= 1'b1; ld_obj <= name_obj; ld_write <= op == 16'd2; t <= T_OPENW;
                    end
                end
                T_OPENW: if (ld_open_ack) begin
                    ei <= 5'd0;
                    if (ld_open_err == 5'd19) begin err_msg <= M_NOFILE; t <= T_ERR; end
                    else if (ld_open_err != 5'd0) begin err_msg <= M_BUSY; t <= T_ERR; end
                    else begin
                        s_act[cs] <= 1'b1; s_dir[cs] <= ld_write; s_obj[cs] <= ld_obj; s_cport[cs] <= req_sport;
                        s_final[cs] <= 1'b0; s_retry[cs] <= 4'd0; s_size[cs] <= ld_size;
                        s_tid[cs] <= {TID_BASE[15:4], new_tid};
                        tid_ctr <= new_tid + 1'b1;
                        s_blk[cs] <= ld_write ? 16'd0 : 16'd1;
                        t <= ld_write ? T_ACK : T_DATA;
                    end
                end
                // ---------------- 会话报文 ----------------
                T_SESS: begin
                    cs <= !hit0; ei <= 5'd0;
                    if (!(hit0 || hit1) || !(hit0 ? cp0 : cp1)) begin
                        // 未知 TID 或源端点变化：只回 ERROR 5，不影响原会话
                        err_msg <= M_TID; t <= T_ERR;
                    end else t <= T_CMP;
                end
                T_CMP: begin
                    k_err <= op == 16'd5; k_data <= op == 16'd3; k_ack <= op == 16'd4;
                    k_long <= q_long; k_len4 <= q_len4;
                    k_final <= s_final[cs]; k_dir <= s_dir[cs]; k_ctrl <= s_obj[cs] == O_CTRL;
                    k_same <= blk == s_blk[cs];
                    k_next <= blk == s_blk[cs] + 1'b1;
                    k_future <= blk > s_blk[cs];
                    t <= T_RACK;
                end
                T_RACK: begin
                    ei <= 5'd0;
                    if (k_err) begin
                        // 对端 ERROR：终止会话，不回应（放弃暂存写入时由对象管理器产生事件）
                        s_act[cs] <= 1'b0;
                        if (k_dir && !k_ctrl && !k_final) ld_abort <= 1'b1;
                        else ev_fault <= 1'b1;
                        t <= T_END;
                    end else if (k_dir) begin
                        if (!k_data || k_long) begin
                            free_after <= 1'b1;
                            if (!k_ctrl && !k_final) ld_abort <= 1'b1;
                            err_msg <= M_ILLEGAL; t <= T_ERR;
                        end else if (k_final) t <= k_same ? T_ACK : T_END;
                        else if (k_next) begin
                            pos <= 12'd4; last_pos <= 12'hfff; rd_on <= 1'b1;
                            w_remain <= req_len - 11'd4; ld_wsel <= k_ctrl; wait_n <= 4'd6;
                            t <= k_len4 ? T_WCHK : T_WDATA;
                        end else if (k_same) t <= T_ACK;
                        else if (k_future) begin
                            free_after <= 1'b1;
                            if (!k_ctrl) ld_abort <= 1'b1;
                            err_msg <= M_ILLEGAL; t <= T_ERR;
                        end else t <= T_END;
                    end else begin
                        if (!k_ack) begin free_after <= 1'b1; err_msg <= M_ILLEGAL; t <= T_ERR; end
                        else if (k_same) begin
                            if (k_final) begin s_act[cs] <= 1'b0; ld_rd_close <= 1'b1; t <= T_END; end
                            else begin s_blk[cs] <= s_blk[cs] + 1'b1; s_retry[cs] <= 4'd0; t <= T_DATA; end
                        end else t <= T_END;
                    end
                end
                T_WDATA: if (byte_ok) begin
                    ld_wvalid <= 1'b1; ld_wbyte <= b; pos <= pos + 1'b1; w_remain <= w_remain - 1'b1;
                    if (w_remain == 11'd1) begin rd_on <= 1'b0; t <= T_WCHK; end
                end
                T_WCHK: begin
                    // 等待对象管理器完成逐字节检查（输入寄存一拍，清单头/配置项在字节后第 4 拍给出错误）
                    ei <= 5'd0;
                    if (wait_n != 4'd0) wait_n <= wait_n - 1'b1;
                    else begin
                        s_blk[cs] <= blk; s_retry[cs] <= 4'd0;
                        if (!ld_wsel && ld_werr != 5'd0) begin
                            err_ld <= ld_werr; free_after <= 1'b1; ld_abort <= 1'b1;
                            err_msg <= M_LOAD; t <= T_ERR;
                        end else if (q_lt516) begin ld_finish <= 1'b1; t <= T_FINW; end
                        else t <= T_ACK;
                    end
                end
                T_FINW: if (ld_fin_ack) begin
                    ei <= 5'd0;
                    if (ld_fin_err != 5'd0) begin
                        err_ld <= ld_fin_err; free_after <= 1'b1; ev_fault <= ld_wsel;
                        err_msg <= M_LOAD; t <= T_ERR;
                    end else begin s_final[cs] <= 1'b1; t <= T_ACK; end
                end
                // ---------------- 报文生成 ----------------
                T_ACK: begin
                    // 00 04 blk
                    if (ei == 5'd0) begin
                        s_left[cs] <= TIMEOUT_US; s_zero[cs] <= 1'b0; e_blk <= s_blk[cs];
                        tb_sport <= s_tid[cs]; tb_dport <= s_cport[cs]; tb_len <= 11'd4;
                    end
                    ei <= ei + 1'b1;
                    tb_wr_en <= 1'b1; tb_wr_addr <= {6'd0, ei};
                    tb_wr_data <= (ei == 5'd1) ? 8'd4 : (ei == 5'd2) ? e_blk[15:8] : (ei == 5'd3) ? e_blk[7:0] : 8'd0;
                    if (ei == 5'd3) t <= T_POST;
                end
                T_ERR: begin
                    // 00 05 00 code msg 00；源端口为请求的目的端口（69 或本会话 TID）
                    ei <= ei + 1'b1;
                    tb_wr_en <= 1'b1; tb_wr_addr <= {6'd0, ei};
                    tb_wr_data <= (ei == 5'd1) ? 8'd5 : (ei == 5'd3) ? {5'd0, err_code} : (ei < 5'd4) ? 8'd0 : emsg;
                    if (ei == e_end) begin
                        tb_len <= {6'd0, ei} + 11'd1; cnt_errors <= cnt_errors + 1'b1;
                        tb_sport <= req_dport; tb_dport <= req_sport;
                        if (free_after) s_act[cs] <= 1'b0;
                        t <= T_POST;
                    end
                end
                T_DATA: begin
                    s_left[cs] <= TIMEOUT_US; s_zero[cs] <= 1'b0; e_blk <= s_blk[cs];
                    d_base <= {s_blk[cs][14:0], 9'd0} - 24'd512;
                    tb_sport <= s_tid[cs]; tb_dport <= s_cport[cs];
                    ld_rd_obj <= s_obj[cs]; ei <= 5'd0;
                    t <= T_DHDR;
                end
                T_DHDR: begin
                    // 00 03 blk；同时确定本块长度（ei=0 求剩余字节，ei=1 定长度）
                    if (ei == 5'd0) d_rem <= s_size[cs] - d_base;
                    if (ei == 5'd1) begin
                        d_len <= (d_rem >= 24'd512) ? 10'd512 : d_rem[9:0];
                        s_final[cs] <= d_rem < 24'd512;
                    end
                    ei <= ei + 1'b1;
                    tb_wr_en <= 1'b1; tb_wr_addr <= {6'd0, ei};
                    tb_wr_data <= (ei == 5'd1) ? 8'd3 : (ei == 5'd2) ? e_blk[15:8] : (ei == 5'd3) ? e_blk[7:0] : 8'd0;
                    if (ei == 5'd3) begin d_issue <= 10'd0; d_got <= 10'd0; d_v <= 5'd0; t <= T_DSTREAM; end
                end
                T_DSTREAM: begin
                    // 读口 4 拍延迟：地址寄存后第 5 个时钟沿采样数据
                    ld_rd_addr <= d_base + {14'd0, d_issue};
                    d_v <= {d_v[3:0], d_issue != d_len};
                    if (d_issue != d_len) d_issue <= d_issue + 1'b1;
                    if (d_v[4]) begin
                        tb_wr_en <= 1'b1; tb_wr_addr <= {1'b0, d_got} + 11'd4; tb_wr_data <= ld_rd_data;
                        d_got <= d_got + 1'b1;
                    end
                    // Registered completion keeps the wide comparisons out of
                    // the FSM; the last RAM write has completed before posting.
                    if (d_stream_done) begin
                        tb_len <= data_packet_len; t <= T_POST;
                    end
                end
                T_POST: begin tb_post <= 1'b1; t <= T_END; end
                // ---------------- 超时 ----------------
                // 当前会话的判定先寄存一拍
                T_TIMER0: begin
                    k_dir <= s_dir[cs]; k_final <= s_final[cs]; k_ctrl <= s_obj[cs] == O_CTRL;
                    k_same <= s_retry[cs] == MAX_RETRIES;
                    t <= T_TIMER;
                end
                T_TIMER: begin
                    ei <= 5'd0;
                    if (k_dir && k_final) begin s_act[cs] <= 1'b0; t <= T_END; end
                    else if (k_same) begin
                        s_act[cs] <= 1'b0;
                        if (k_dir && !k_ctrl) ld_abort <= 1'b1;
                        else ev_fault <= 1'b1;
                        t <= T_END;
                    end else begin
                        s_retry[cs] <= s_retry[cs] + 1'b1; cnt_retries <= cnt_retries + 1'b1;
                        t <= k_dir ? T_ACK : T_DATA;
                    end
                end
                T_END: begin done <= 1'b1; rd_on <= 1'b0; t <= T_IDLE; end
                default: t <= T_IDLE;
            endcase
        end
    end
endmodule
