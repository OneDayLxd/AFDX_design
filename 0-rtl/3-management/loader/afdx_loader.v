/*
 * ================================================================
 * File Name    : afdx_loader.v
 * Module Name  : afdx_loader
 *
 * Function:
 *   ARINC 615A-4 有界子集的对象管理器（加载 / 卸载 / 状态 / 终止），由 TFTP 服务器驱动。
 *   对象：1 config/afdx-vl-config.bin  2 application/app-image.bin  3 status/loader-status.bin
 *         4 diagnostic/fault-log.bin    5 control/abort
 *   可加载对象的镜像格式（大端）：清单头 24 字节
 *     0 "AFLD" | 4 format_version=1 | 5 object_type | 6 header_length=24 | 8 generation
 *     12 payload_length | 16 payload_crc32 | 20 flags=0 | 24.. payload
 *   payload_crc32 为 CRC-32/ISO-HDLC（反射、初值 FFFFFFFF、结果取反，"123456789" -> CBF43926），只覆盖载荷。
 *   配置载荷：每项 8 字节 {vl u16, bag_code u8 (BAG=1000<<code us), net_en u8, lmax u16, 0 u16}，
 *     VL 严格递增（唯一），逐项写入影子银行；未出现的 VL 保持原活动值。generation 必须大于当前值。
 *   状态：IDLE→RECEIVING→VERIFYING→APPLY_PENDING→ACTIVE，失败 FAILED，终止 ABORTED；
 *     活动银行在新对象完整验证并切换前从不改写。配置切换后 SELFTEST_US 内必须有帧成功组装，
 *     否则回滚到上一银行（ldLastError=14）。
 *   写字节流分两路：wsel=0 暂存对象（配置/应用镜像），wsel=1 控制对象（"ABRT" 终止暂存写入）。
 *   读口：rd_addr 为对象内字节偏移，rd_data 固定 4 拍延迟（配置字段来自运行时配置 RAM 的管理读口）。
 *   时序：清单头各项检查先分别寄存再按优先级合成错误码；载荷用独立的序号/剩余计数与寄存的
 *   “载荷已满”标志；结束判定先寄存长度/CRC 比较结果（C_FINCHK）；读口第 2 级预选快照字节。
 *   写字节流先寄存一拍，“可接收暂存字节”标志每拍预先寄存；打开请求先锁存并译码（C_OPEN），
 *   快照装载只由寄存的脉冲驱动。
 *
 * Sub Module:
 *   afdx_crc32
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_loader #(
    parameter N_VL = 8,
    parameter VLW = 3,
    parameter APP_AW = 12,                  // 应用镜像每银行 2^APP_AW 字节（片上 BRAM，复位后不保持）
    parameter SELFTEST_US = 50000
) (
    input  wire            clk,
    input  wire            rst,
    input  wire [31:0]     mono_us,
    // TFTP 接口
    input  wire            open_req,
    input  wire [2:0]      open_obj,
    input  wire            open_write,
    output reg             open_ack,
    output reg  [4:0]      open_err,
    output reg  [23:0]     open_size,
    input  wire            wbyte_valid,
    input  wire            wsel,
    input  wire [7:0]      wbyte,
    output wire [4:0]      werr,            // 暂存写入已检出的错误（非零时 TFTP 终止会话）
    input  wire            wfinish,
    output reg             fin_ack,
    output reg  [4:0]      fin_err,
    input  wire            wabort,          // TFTP 放弃暂存写入（对端 ERROR、重试耗尽）
    input  wire [2:0]      rd_obj,
    input  wire [23:0]     rd_addr,
    output reg  [7:0]      rd_data,
    input  wire            rd_close,
    input  wire            snmp_abort,
    output reg             abort_evt,       // 暂存写入被终止（通知 TFTP 释放会话）
    output wire            staging,
    // 运行时配置 RAM
    output reg  [VLW-1:0]  cfg_rd_idx,
    input  wire [19:0]     cfg_bag_us,
    input  wire [10:0]     cfg_lmax,
    input  wire [1:0]      cfg_net_en,
    output reg             cfg_wr_en,
    output reg  [VLW-1:0]  cfg_wr_idx,
    output reg  [19:0]     cfg_wr_bag_us,
    output reg  [10:0]     cfg_wr_lmax,
    output reg  [1:0]      cfg_wr_net_en,
    output reg             cfg_copy, cfg_verify, cfg_apply, cfg_rollback,
    input  wire            cfg_busy,
    input  wire            bg_ok,           // 管理面空闲：允许后台自检回滚使用配置 RAM
    input  wire            cfg_done,
    input  wire            cfg_verify_ok,
    input  wire            tx_frame,        // 自检：组帧成功脉冲
    input  wire            ext_commit,      // SNMP cfgCommand 提交/回滚成功
    input  wire            ext_rollback,
    output reg  [31:0]     cfg_generation,
    // 应用镜像内部只读接口（活动银行）
    input  wire [APP_AW-1:0] app_rd_addr,
    output reg  [7:0]      app_rd_data,
    output reg             app_bank,
    output reg  [31:0]     app_length,
    output reg  [31:0]     app_version,
    output reg  [31:0]     app_crc,
    // 状态 / 统计
    input  wire [511:0]    fault_vec,
    input  wire [15:0]     cur_block,
    output reg  [2:0]      ld_state,
    output reg  [2:0]      ld_object,
    output reg  [31:0]     ld_bytes,
    output reg  [31:0]     ld_crc_expected,
    output reg  [31:0]     ld_crc_actual,
    output reg  [4:0]      ld_error,
    output reg  [15:0]     ld_fail_block,
    output reg  [31:0]     cnt_loads,
    output reg  [31:0]     cnt_unloads,
    output reg             ev_fault
);
    localparam L_IDLE = 3'd0, L_RECV = 3'd1, L_VERIFYING = 3'd2, L_APPLY = 3'd4,
               L_ACTIVE = 3'd5, L_FAILED = 3'd6, L_ABORTED = 3'd7;
    localparam O_CFG = 3'd1, O_APP = 3'd2, O_STATUS = 3'd3, O_FAULT = 3'd4, O_CTRL = 3'd5;
    localparam E_MAGIC = 5'd1, E_VERSION = 5'd2, E_TYPE = 5'd3, E_HDRLEN = 5'd4, E_LENGTH = 5'd5,
               E_CRC = 5'd6, E_ROLLBACK = 5'd7, E_VLIDX = 5'd8, E_ORDER = 5'd9, E_BAG = 5'd10,
               E_NET = 5'd11, E_LMAX = 5'd12, E_VERIFY = 5'd13, E_SELFTEST = 5'd14, E_ABORTED = 5'd15,
               E_TOOBIG = 5'd16, E_BUSY = 5'd18, E_NOTFOUND = 5'd19, E_CONTROL = 5'd20;
    localparam APP_BYTES = 1 << APP_AW;
    localparam integer CFG_IMG_I = 24 + 8 * N_VL;
    localparam [23:0] CFG_IMG = CFG_IMG_I[23:0];
    localparam [31:0] CFG_PLEN = 8 * N_VL;

    // ---------------- 暂存写入 ----------------
    reg [2:0] w_obj;                          // O_CFG / O_APP / 0
    reg [4:0] w_err;
    reg [4:0] h_cnt;                           // 已收清单头字节
    reg [31:0] pay_rem;                        // 剩余载荷字节
    reg [APP_AW-1:0] pay_idx;                  // 载荷内偏移（应用镜像写地址）
    reg pay_done;                              // pay_rem == 0
    reg [191:0] hdr;
    reg hdr_ok, hdr_chk, hdr_eval, ent_rdy;
    reg hc_magic, hc_ver, hc_type, hc_hlen, hc_big, hc_roll;
    reg ent_eval, ec_idx, ec_order, ec_bag, ec_net, ec_any;
    reg fin_len_ok, fin_crc_ok, fin_abrt;
    wire [31:0] h_magic = hdr[191:160];
    wire [7:0] h_ver = hdr[159:152], h_type = hdr[151:144];
    wire [15:0] h_hlen = hdr[143:128];
    wire [31:0] h_gen = hdr[127:96], h_plen = hdr[95:64], h_crc = hdr[63:32], h_flags = hdr[31:0];
    reg [31:0] crc;
    wire [31:0] crc_next;
    // 版本回退检查的参照值（暂存对象打开后不变）预先寄存
    reg [31:0] gen_ref;
    always @(posedge clk) gen_ref <= (w_obj == O_CFG) ? cfg_generation : app_version;
    // 写字节流寄存一拍（TFTP 字节间隔至少三拍）
    reg wv_q, ws_q;
    reg [7:0] wb_q;
    always @(posedge clk) begin
        wv_q <= !rst && wbyte_valid; ws_q <= wsel; wb_q <= wbyte;
    end
    afdx_crc32 u_crc (
        .crc_in  (crc),
        .data    (wb_q),
        .crc_out (crc_next)
    );
    reg [63:0] ent;
    reg [2:0] ent_i;
    reg [VLW-1:0] last_vl;
    reg first_ent;
    assign werr = w_err;
    assign staging = w_obj != 3'd0;
    // 暂存对象打开/关闭、错误置位与字节之间都相隔多拍，因此可用上一拍的判定
    (* maxfan = 48 *) reg take_ok;
    always @(posedge clk) take_ok <= staging && w_err == 5'd0;
    wire w_take = wv_q && !ws_q && take_ok;
    // 控制对象
    reg c_open;
    reg [31:0] c_word;
    reg [3:0] c_cnt;

    // 应用镜像双银行：写口写非活动银行；卸载读口与内部读口各一份存储
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] app_mem_t [0:2*APP_BYTES-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] app_mem_a [0:2*APP_BYTES-1];
    reg app_we;
    reg [APP_AW:0] app_wa, app_ra;
    reg [7:0] app_wd, app_rq, app_rq2;
    reg app_valid;
    always @(posedge clk) begin
        if (app_we) begin app_mem_t[app_wa] <= app_wd; app_mem_a[app_wa] <= app_wd; end
        app_rq <= app_mem_t[app_ra];
        app_rd_data <= app_mem_a[{app_bank, app_rd_addr}];
    end

    // ---------------- 读流水线（4 拍） ----------------
    reg [383:0] st_snap;
    reg [511:0] fl_snap;
    reg [191:0] rd_hdr;
    reg pre_run;
    reg [23:0] pre_addr;
    reg [3:0] pre_v;
    reg [31:0] rcrc;
    wire [31:0] rcrc_next;
    afdx_crc32 u_rcrc (
        .crc_in  (rcrc),
        .data    (rd_data),
        .crc_out (rcrc_next)
    );
    wire [23:0] eff_addr = pre_run ? pre_addr : rd_addr;
    wire [2:0] eff_obj = pre_run ? O_CFG : rd_obj;
    wire [23:0] eff_off = eff_addr - 24'd24;
    reg [2:0] r1_obj, r2_obj, r3_obj;
    reg [23:0] r1_addr, r2_addr;
    reg r3_hdr;                                // r3 地址位于清单头内
    reg [2:0] r3_fld;                          // 配置项内字节序号
    reg [7:0] r3_vl, hd_b, st_b, fl_b;         // 配置项 VL 号与第 2 级选出的字节
    reg [63:0] fl_g, st_g, hd_g;               // 第 1 级：各 8 字节组内按地址低 3 位选出的字节（组 0 在低位）
    // Decode before the snapshot mux; keep these selectors separate from
    // the identical low app-RAM address bits and place copies near each bank.
    (* preserve, maxfan = 32 *) reg [7:0] fl_lane, st_lane, hd_lane;
    (* preserve, maxfan = 16 *) reg [7:0] fl_group, st_group, hd_group;
    reg [2:0] bag_code;
    always @* begin
        case (cfg_bag_us)
            20'd2000: bag_code = 3'd1;   20'd4000: bag_code = 3'd2;  20'd8000: bag_code = 3'd3;
            20'd16000: bag_code = 3'd4;  20'd32000: bag_code = 3'd5; 20'd64000: bag_code = 3'd6;
            20'd128000: bag_code = 3'd7; default: bag_code = 3'd0;
        endcase
    end
    // One-hot byte select (lane 0 is the highest byte).
    function [7:0] pick8;
        input [63:0] g;
        input [7:0] sel;
        integer b;
        begin
            pick8 = 8'd0;
            for (b = 0; b < 8; b = b + 1)
                pick8 = pick8 | (g[63-8*b -: 8] & {8{sel[b]}});
        end
    endfunction
    always @(posedge clk) begin
        r1_obj <= eff_obj; r1_addr <= eff_addr;
        fl_lane <= 8'b1 << eff_addr[2:0];
        st_lane <= 8'b1 << eff_addr[2:0];
        hd_lane <= 8'b1 << eff_addr[2:0];
        cfg_rd_idx <= eff_off[VLW+2:3];
        app_ra <= {app_bank, eff_off[APP_AW-1:0]};
        r2_obj <= r1_obj; r2_addr <= r1_addr;
        // Group 0 is the low byte of *_g; pick8 selects from the high end.
        // Decode alongside the first mux stage, preserving the four clocks
        // of read latency and the header's original two-bit group index.
        fl_group <= 8'h80 >> r1_addr[5:3];
        st_group <= 8'h80 >> r1_addr[5:3];
        hd_group <= 8'h80 >> r1_addr[4:3];
        fl_g <= {pick8(fl_snap[63:0], fl_lane), pick8(fl_snap[127:64], fl_lane),
                 pick8(fl_snap[191:128], fl_lane), pick8(fl_snap[255:192], fl_lane),
                 pick8(fl_snap[319:256], fl_lane), pick8(fl_snap[383:320], fl_lane),
                 pick8(fl_snap[447:384], fl_lane), pick8(fl_snap[511:448], fl_lane)};
        st_g <= {16'd0, pick8(st_snap[63:0], st_lane), pick8(st_snap[127:64], st_lane),
                 pick8(st_snap[191:128], st_lane), pick8(st_snap[255:192], st_lane),
                 pick8(st_snap[319:256], st_lane), pick8(st_snap[383:320], st_lane)};
        hd_g <= {40'd0, pick8(rd_hdr[63:0], hd_lane), pick8(rd_hdr[127:64], hd_lane),
                 pick8(rd_hdr[191:128], hd_lane)};
        app_rq2 <= app_rq;
        r3_obj <= r2_obj;
        // 第 2 级：地址译码与快照/清单头字节预选（24 为 8 的倍数，配置项字节序号即地址低 3 位）
        r3_hdr <= r2_addr < 24'd24;
        r3_fld <= r2_addr[2:0];
        r3_vl <= r2_addr[10:3] - 8'd3;
        hd_b <= pick8(hd_g, hd_group);
        st_b <= pick8(st_g, st_group);
        fl_b <= pick8(fl_g, fl_group);
        case (r3_obj)
            O_CFG: begin
                if (r3_hdr) rd_data <= hd_b;
                else case (r3_fld)
                    3'd1: rd_data <= r3_vl;
                    3'd2: rd_data <= {5'd0, bag_code};
                    3'd3: rd_data <= {6'd0, cfg_net_en};
                    3'd4: rd_data <= {5'd0, cfg_lmax[10:8]};
                    3'd5: rd_data <= cfg_lmax[7:0];
                    default: rd_data <= 8'd0;
                endcase
            end
            O_APP: rd_data <= r3_hdr ? hd_b : app_rq2;
            O_STATUS: rd_data <= st_b;
            default: rd_data <= fl_b;
        endcase
    end

    // ---------------- 控制 ----------------
    localparam C_IDLE = 4'd0, C_COPY = 4'd1, C_PRE = 4'd2, C_PRE_END = 4'd3, C_FINISH = 4'd4,
               C_VERIFY = 4'd5, C_APPLY = 4'd6, C_FINCHK = 4'd7, C_OPEN = 4'd8;
    reg [3:0] c;
    reg self_active, self_seen, self_roll, self_tmo;
    reg [31:0] self_t0, self_el, prev_gen, rd_gen;
    // 配置 RAM 忙标志寄存一拍（管理作业串行，配置命令只由当前作业或本模块发出）
    reg cfg_busy_q;
    always @(posedge clk) cfg_busy_q <= cfg_busy;
    // 锁存的打开请求及其译码
    reg [2:0] o_obj;
    reg o_write, o_ctrl, o_cfg, o_app, o_status, o_neww;
    (* maxfan = 64 *) reg snap_st, snap_fl;
    always @(posedge clk) begin
        snap_st <= !rst && c == C_IDLE && open_req && !self_roll && !open_write && open_obj == O_STATUS;
        snap_fl <= !rst && c == C_IDLE && open_req && !self_roll && !open_write && open_obj == O_FAULT;
        if (snap_st)
            st_snap <= {32'h41465354, 5'd0, ld_state, 5'd0, ld_object, 3'd0, ld_error,
                        7'd0, app_bank, ld_fail_block, cur_block, ld_bytes, ld_crc_expected,
                        ld_crc_actual, cfg_generation, app_version, app_length, cnt_loads,
                        cnt_unloads, app_crc};
        if (snap_fl) fl_snap <= fault_vec;
    end
    always @(posedge clk) begin
        if (rst) begin
            c <= C_IDLE; open_ack <= 1'b0; fin_ack <= 1'b0; abort_evt <= 1'b0; ev_fault <= 1'b0;
            w_obj <= 3'd0; w_err <= 5'd0; h_cnt <= 5'd0; hdr_ok <= 1'b0; hdr_chk <= 1'b0; hdr_eval <= 1'b0;
            ent_rdy <= 1'b0; pay_rem <= 32'd0; pay_done <= 1'b1; pay_idx <= {APP_AW{1'b0}};
            app_we <= 1'b0; c_open <= 1'b0; pre_run <= 1'b0; pre_v <= 4'd0;
            cfg_wr_en <= 1'b0; cfg_copy <= 1'b0; cfg_verify <= 1'b0; cfg_apply <= 1'b0; cfg_rollback <= 1'b0;
            cfg_generation <= 32'd0; prev_gen <= 32'd0;
            app_bank <= 1'b0; app_valid <= 1'b0; app_length <= 32'd0; app_version <= 32'd0; app_crc <= 32'd0;
            ld_state <= L_IDLE; ld_object <= 3'd0; ld_bytes <= 32'd0; ld_crc_expected <= 32'd0;
            ld_crc_actual <= 32'd0; ld_error <= 5'd0; ld_fail_block <= 16'd0;
            cnt_loads <= 32'd0; cnt_unloads <= 32'd0;
            self_active <= 1'b0; self_seen <= 1'b0; self_roll <= 1'b0;
        end else begin
            open_ack <= 1'b0; fin_ack <= 1'b0; abort_evt <= 1'b0; ev_fault <= 1'b0; app_we <= 1'b0;
            cfg_wr_en <= 1'b0; cfg_copy <= 1'b0; cfg_verify <= 1'b0; cfg_apply <= 1'b0; cfg_rollback <= 1'b0;
            hdr_chk <= 1'b0; hdr_eval <= 1'b0; ent_rdy <= 1'b0; ent_eval <= 1'b0;
            if (rd_close) cnt_unloads <= cnt_unloads + 1'b1;
            if (ext_commit) begin prev_gen <= cfg_generation; cfg_generation <= cfg_generation + 1'b1; end
            if (ext_rollback) cfg_generation <= prev_gen;

            // ---- 暂存写入字节 ----
            if (w_take) begin
                ld_bytes <= ld_bytes + 1'b1;
                if (!hdr_ok) begin
                    hdr <= {hdr[183:0], wb_q};
                    h_cnt <= h_cnt + 1'b1;
                    if (h_cnt == 5'd23) begin hdr_ok <= 1'b1; hdr_chk <= 1'b1; end
                end else if (pay_done) w_err <= E_LENGTH;
                else begin
                    crc <= crc_next;
                    pay_idx <= pay_idx + 1'b1;
                    pay_rem <= pay_rem - 1'b1;
                    pay_done <= pay_rem == 32'd1;
                    if (w_obj == O_APP) begin
                        app_we <= 1'b1; app_wa <= {~app_bank, pay_idx}; app_wd <= wb_q;
                    end else begin
                        ent <= {ent[55:0], wb_q}; ent_i <= ent_i + 1'b1;
                        ent_rdy <= ent_i == 3'd7;
                    end
                end
            end
            // 清单头：第 24 字节后一拍寄存各项检查并装载载荷计数，再一拍按优先级给出错误码。
            // 其间最多再收一个载荷字节（TFTP 字节间隔至少两拍），出错时整个暂存对象都会作废。
            if (hdr_chk) begin
                ld_crc_expected <= h_crc;
                hc_magic <= h_magic != 32'h41464C44;
                hc_ver <= h_ver != 8'd1;
                hc_type <= h_type != {5'd0, w_obj};
                hc_hlen <= h_hlen != 16'd24 || h_flags != 32'd0;
                hc_big <= (w_obj == O_CFG && (h_plen > CFG_PLEN || h_plen[2:0] != 3'd0)) ||
                          (w_obj == O_APP && h_plen > APP_BYTES);
                hc_roll <= h_gen <= gen_ref;
                pay_rem <= h_plen; pay_done <= h_plen == 32'd0; pay_idx <= {APP_AW{1'b0}};
                hdr_eval <= 1'b1;
            end
            if (hdr_eval && w_err == 5'd0) begin
                if (hc_magic) w_err <= E_MAGIC;
                else if (hc_ver) w_err <= E_VERSION;
                else if (hc_type) w_err <= E_TYPE;
                else if (hc_hlen) w_err <= E_HDRLEN;
                else if (hc_big) w_err <= E_TOOBIG;
                else if (hc_roll) w_err <= E_ROLLBACK;
            end
            // 配置项：第 8 字节后一拍寄存各项检查，再一拍按优先级报错或写入影子银行
            // （下一项至少 16 拍后才完整，TFTP 在块尾 4 拍后才采样 werr）
            if (ent_rdy) begin
                ec_idx <= ent[63:48] >= N_VL;
                ec_order <= !first_ent && ent[48+VLW-1:48] <= last_vl;
                ec_bag <= ent[47:40] > 8'd7;
                ec_net <= ent[39:32] == 8'd0 || ent[39:32] > 8'd3;
                ec_any <= ent[63:48] >= N_VL || (!first_ent && ent[48+VLW-1:48] <= last_vl) ||
                          ent[47:40] > 8'd7 || ent[39:32] == 8'd0 || ent[39:32] > 8'd3 ||
                          ent[31:16] == 16'd0 || ent[31:16] > 16'd1471;
                ent_eval <= 1'b1;
            end
            if (ent_eval && w_err == 5'd0) begin
                // ec_any 为全部检查项之或；Lmax 越界是其余各项都通过时的唯一可能
                if (ec_any) w_err <= ec_idx ? E_VLIDX : ec_order ? E_ORDER : ec_bag ? E_BAG :
                                     ec_net ? E_NET : E_LMAX;
                else begin
                    cfg_wr_en <= 1'b1; cfg_wr_idx <= ent[48+VLW-1:48];
                    cfg_wr_bag_us <= 20'd1000 << ent[42:40]; cfg_wr_lmax <= ent[26:16];
                    cfg_wr_net_en <= ent[33:32];
                    last_vl <= ent[48+VLW-1:48]; first_ent <= 1'b0;
                end
            end
            // 逐字节检出错误：本次暂存立即失败并释放（随后 TFTP 的放弃请求不再改写状态/原因）
            if (w_err != 5'd0 && ld_state == L_RECV) begin
                ld_state <= L_FAILED; ld_error <= w_err; ld_fail_block <= cur_block; ev_fault <= 1'b1;
                w_obj <= 3'd0;
            end
            // ---- 控制对象字节 ----
            if (wv_q && ws_q && c_open) begin
                c_word <= {c_word[23:0], wb_q};
                if (c_cnt != 4'hf) c_cnt <= c_cnt + 1'b1;
            end
            // ---- 终止（SNMP ldAbort 或 TFTP 放弃）----
            if ((snmp_abort || wabort) && staging && (c == C_IDLE || c == C_OPEN || c == C_COPY)) begin
                w_obj <= 3'd0; ld_state <= L_ABORTED; ld_error <= E_ABORTED; ev_fault <= 1'b1;
                abort_evt <= snmp_abort;
            end
            // ---- 配置自检（后台）----
            if (tx_frame) self_seen <= 1'b1;
            self_el <= mono_us - self_t0;
            self_tmo <= self_el > SELFTEST_US;
            if (self_active && !self_roll) begin
                if (self_seen) begin self_active <= 1'b0; ld_state <= L_ACTIVE; end
                else if (self_tmo && c == C_IDLE && !cfg_busy_q && bg_ok) begin
                    cfg_rollback <= 1'b1; self_roll <= 1'b1;
                end
            end
            if (self_roll && cfg_done) begin
                self_roll <= 1'b0; self_active <= 1'b0; cfg_generation <= prev_gen;
                ld_state <= L_FAILED; ld_error <= E_SELFTEST; ev_fault <= 1'b1;
            end

            case (c)
                C_IDLE: if (open_req && !self_roll) begin
                    o_obj <= open_obj; o_write <= open_write;
                    o_ctrl <= open_obj == O_CTRL; o_cfg <= open_obj == O_CFG; o_app <= open_obj == O_APP;
                    o_status <= open_obj == O_STATUS;
                    // 可以开始新的暂存写入（配置 RAM 忙标志用本拍的实时值）
                    o_neww <= open_write && open_obj != O_CTRL && !staging && !self_active && !cfg_busy;
                    c <= C_OPEN;
                end else if (wfinish) c <= C_FINCHK;
                // 打开请求（快照由 snap_st/snap_fl 在本拍装载）
                C_OPEN: begin
                    open_size <= 24'd0; open_err <= 5'd0; c <= C_IDLE;
                    if (o_write) begin
                        if (o_ctrl) begin
                            c_open <= 1'b1; c_word <= 32'd0; c_cnt <= 4'd0; open_ack <= 1'b1;
                        end else if (!o_neww) begin
                            open_err <= E_BUSY; open_ack <= 1'b1;
                        end else begin
                            w_obj <= o_obj; w_err <= 5'd0; h_cnt <= 5'd0; hdr_ok <= 1'b0;
                            crc <= 32'hffffffff; ent_i <= 3'd0; first_ent <= 1'b1;
                            ld_state <= L_RECV; ld_object <= o_obj; ld_bytes <= 32'd0;
                            ld_error <= 5'd0; ld_crc_actual <= 32'd0; ld_fail_block <= 16'd0;
                            if (o_cfg) begin cfg_copy <= 1'b1; c <= C_COPY; end
                            else open_ack <= 1'b1;
                        end
                    end else if (o_cfg) begin
                        if (cfg_busy_q) begin open_err <= E_BUSY; open_ack <= 1'b1; end
                        else begin
                            rd_gen <= cfg_generation; pre_run <= 1'b1; pre_addr <= 24'd24;
                            pre_v <= 4'd0; rcrc <= 32'hffffffff; c <= C_PRE;
                        end
                    end else if (o_app) begin
                        open_err <= app_valid ? 5'd0 : E_NOTFOUND; open_ack <= 1'b1;
                        open_size <= app_length[23:0] + 24'd24;
                        rd_hdr <= {32'h41464C44, 8'd1, 8'd2, 16'd24, app_version, app_length, app_crc, 32'd0};
                    end else if (o_status) begin
                        open_ack <= 1'b1; open_size <= 24'd48;
                    end else if (o_obj == O_FAULT) begin
                        open_ack <= 1'b1; open_size <= 24'd64;
                    end else begin
                        open_err <= E_NOTFOUND; open_ack <= 1'b1;
                    end
                end
                C_COPY: if (cfg_done) begin open_ack <= 1'b1; c <= C_IDLE; end
                // 配置卸载：顺序读出全部配置项计算载荷 CRC（与读口同一 4 拍流水线）
                C_PRE: begin
                    if (pre_run) begin
                        pre_addr <= pre_addr + 1'b1;
                        if (pre_addr == CFG_IMG - 1'b1) pre_run <= 1'b0;
                    end
                    pre_v <= {pre_v[2:0], pre_run};
                    if (pre_v[3]) rcrc <= rcrc_next;
                    if (!pre_run && pre_v == 4'b1000) c <= C_PRE_END;
                end
                C_PRE_END: begin
                    rd_hdr <= {32'h41464C44, 8'd1, 8'd1, 16'd24, rd_gen, CFG_PLEN, ~rcrc, 32'd0};
                    open_ack <= 1'b1; open_size <= CFG_IMG; c <= C_IDLE;
                end
                C_FINCHK: begin
                    fin_len_ok <= hdr_ok && pay_done;
                    fin_crc_ok <= ~crc == h_crc;
                    fin_abrt <= c_word == 32'h41425254 && c_cnt == 4'd4;
                    c <= C_FINISH;
                end
                C_FINISH: begin
                    ld_crc_actual <= ~crc;
                    if (wsel) begin
                        // 控制记录恰为 "ABRT"：终止正在进行的暂存写入
                        c_open <= 1'b0; fin_ack <= 1'b1; c <= C_IDLE;
                        if (fin_abrt) begin
                            fin_err <= 5'd0;
                            if (staging) begin
                                w_obj <= 3'd0; ld_state <= L_ABORTED; ld_error <= E_ABORTED;
                                abort_evt <= 1'b1; ev_fault <= 1'b1;
                            end
                        end else fin_err <= E_CONTROL;
                    end else if (!staging) begin
                        fin_err <= E_ABORTED; fin_ack <= 1'b1; c <= C_IDLE;
                    end else if (w_err != 5'd0 || !fin_len_ok || !fin_crc_ok) begin
                        fin_err <= (w_err != 5'd0) ? w_err : fin_len_ok ? E_CRC : E_LENGTH;
                        ld_error <= (w_err != 5'd0) ? w_err : fin_len_ok ? E_CRC : E_LENGTH;
                        ld_state <= L_FAILED; ld_fail_block <= cur_block;
                        fin_ack <= 1'b1; w_obj <= 3'd0; ev_fault <= 1'b1; c <= C_IDLE;
                    end else if (w_obj == O_APP) begin
                        app_bank <= ~app_bank; app_valid <= 1'b1;
                        app_version <= h_gen; app_length <= h_plen; app_crc <= h_crc;
                        ld_state <= L_ACTIVE; cnt_loads <= cnt_loads + 1'b1;
                        fin_err <= 5'd0; fin_ack <= 1'b1; w_obj <= 3'd0; c <= C_IDLE;
                    end else begin
                        ld_state <= L_VERIFYING; cfg_verify <= 1'b1; c <= C_VERIFY;
                    end
                end
                C_VERIFY: if (cfg_done) begin
                    if (!cfg_verify_ok) begin
                        ld_state <= L_FAILED; ld_error <= E_VERIFY; fin_err <= E_VERIFY; fin_ack <= 1'b1;
                        w_obj <= 3'd0; ev_fault <= 1'b1; c <= C_IDLE;
                    end else begin
                        ld_state <= L_APPLY; cfg_apply <= 1'b1; c <= C_APPLY;
                    end
                end
                C_APPLY: if (cfg_done) begin
                    prev_gen <= cfg_generation; cfg_generation <= h_gen; cnt_loads <= cnt_loads + 1'b1;
                    fin_err <= 5'd0; fin_ack <= 1'b1; w_obj <= 3'd0;
                    self_active <= 1'b1; self_seen <= 1'b0; self_t0 <= mono_us; c <= C_IDLE;
                end
                default: c <= C_IDLE;
            endcase
        end
    end
endmodule
