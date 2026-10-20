/*
 * ================================================================
 * File Name    : afdx_rx_integrity.v
 * Module Name  : afdx_rx_integrity
 *
 * Function:
 *   接收完整性检查（Integrity Checking）+ 冗余管理（Redundancy Management）。
 *   完整性检查按 (VL, 网络) 维护前序序列号 PSN：接受 SN ∈ {PSN+1, PSN+2}（255 后回绕到 1）
 *     或 SN==0（发送端复位）；首帧无条件接受。PSN 无论是否转发都更新（与软件模型 IntegrityChecker 一致）。
 *   冗余管理按 VL 采用"先到有效者优先"：记录最近交付的 SN 及时间，
 *     若 SN 比已交付的更新（模 255 前向窗口 1..127）或为发送端复位帧（SN=0 且上一帧非 0）则交付；
 *     相同或更旧且在 SkewMax 时间窗内则视为冗余副本丢弃；超过 SkewMax 无帧后重新同步。
 *   状态表全部放在同步 RAM 中（含 valid 位），复位后由清扫计数器逐项清零，
 *     不使用逐 VL 触发器和大规模复位循环；清扫期间到达的请求保持等待。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_rx_integrity #(
    parameter N_VL        = 8,
    parameter VLW         = 3,
    parameter IC_EN       = 1,
    parameter RM_EN       = 1,
    parameter SKEW_MAX_US = 10000
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [31:0]    now_us,
    input  wire           req,
    input  wire [VLW-1:0] vl,
    input  wire           net,
    input  wire [7:0]     sn,
    output reg            resp,
    output reg            accept,
    output reg            stat_ic_drop,
    output reg            stat_rm_drop,
    output wire           init_busy       // 复位后状态表清扫中
);
    // {valid, sn} 打包存放，valid 位随 RAM 一起清扫，不需要独立触发器阵列。
    (* ramstyle = "M4K, no_rw_check" *) reg [8:0]  psn     [0:2*N_VL-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [8:0]  rm_sn   [0:N_VL-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [31:0] rm_time [0:N_VL-1];

    // 复位后的清扫：地址 0..2*N_VL-1 逐项写零；rm 表用低 VLW 位地址，被写两遍。
    reg [VLW:0] init_addr;
    reg         init_run;
    assign init_busy = init_run;
    always @(posedge clk) begin
        if (rst) begin
            init_run  <= 1'b1;
            init_addr <= {(VLW+1){1'b0}};
        end else if (init_run) begin
            init_addr <= init_addr + 1'b1;
            if (init_addr == {(VLW+1){1'b1}}) init_run <= 1'b0;
        end
    end

    // One request is outstanding at a time; the parser waits for resp.
    // Snapshot the protocol time at admission, so added latency cannot change
    // the SkewMax decision at its boundary or across the microsecond wrap.
    localparam IDLE=0, DIFF=1, AGE=2, CHECK=3, DECIDE=4, COMMIT=5, FETCH=6, HOLD=7;
    reg [2:0] state;
    reg [VLW-1:0] vl_l;
    reg net_l;
    reg [7:0] sn_l, p, lsn;
    reg pv, rv;
    reg [31:0] time_l, old_time;
    reg [16:0] age_lo;
    reg [15:0] age_hi_base, age_hi;
    reg [7:0] distance;
    reg ic_ok, rm_seq_ok, expired, decision;
    wire [VLW:0] k = {vl_l, net_l};
    wire [7:0] nx1 = p == 255 ? 8'd1 : p+8'd1;
    wire [7:0] nx2 = p == 255 ? 8'd2 : (p == 254 ? 8'd1 : p+8'd2);
    wire [7:0] delta = sn_l-lsn;
    reg [8:0] psn_q, rm_sn_q;
    reg [31:0] rm_time_q;
    // 读地址：IDLE 时跟随输入，其余状态使用已锁存的键（HOLD 状态重读清扫后的内容）。
    wire [VLW:0]   rd_key = (state == IDLE) ? {vl, net} : k;
    wire [VLW-1:0] rd_vl  = (state == IDLE) ? vl : vl_l;
    // Dedicated, unreset synchronous read ports allow block RAM inference.
    // IDLE captures the request; FETCH consumes the matching RAM outputs.
    // There is one outstanding request, so COMMIT cannot collide with FETCH.
    wire commit_write = (state == COMMIT) && !init_run;
    // 单写口：清扫期间写地址来自计数器，其余时间来自当前请求。
    wire            psn_we   = init_run || commit_write;
    wire [VLW:0]    psn_wa   = init_run ? init_addr : k;
    wire [8:0]      psn_wd   = init_run ? 9'd0 : {1'b1, sn_l};
    wire            rm_we    = init_run || (commit_write && decision);
    wire [VLW-1:0]  rm_wa    = init_run ? init_addr[VLW-1:0] : vl_l;
    wire [8:0]      rm_sn_wd = init_run ? 9'd0 : {1'b1, sn_l};
    wire [31:0]     rm_t_wd  = init_run ? 32'd0 : time_l;
    always @(posedge clk) begin
        psn_q     <= psn[rd_key];
        rm_sn_q   <= rm_sn[rd_vl];
        rm_time_q <= rm_time[rd_vl];
        if (psn_we) psn[psn_wa] <= psn_wd;
        if (rm_we) begin
            rm_sn[rm_wa]   <= rm_sn_wd;
            rm_time[rm_wa] <= rm_t_wd;
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE; resp <= 0; accept <= 0;
            stat_ic_drop <= 0; stat_rm_drop <= 0;
            vl_l <= 0; net_l <= 0; sn_l <= 0; time_l <= 0;
            p <= 0; lsn <= 0; pv <= 0; rv <= 0; old_time <= 0;
            age_lo <= 0; age_hi_base <= 0; age_hi <= 0; distance <= 0;
            ic_ok <= 0; rm_seq_ok <= 0; expired <= 0; decision <= 0;
        end else begin
            resp <= 0; accept <= 0; stat_ic_drop <= 0; stat_rm_drop <= 0;
            case (state)
                IDLE: if (req) begin
                    vl_l <= vl; net_l <= net; sn_l <= sn; time_l <= now_us;
                    state <= init_run ? HOLD : FETCH;
                end
                // 清扫尚未完成：保持请求，清扫结束后用锁存地址重新读表。
                HOLD: if (!init_run) state <= FETCH;
                FETCH: begin
                    p <= psn_q[7:0]; pv <= psn_q[8];
                    lsn <= rm_sn_q[7:0]; rv <= rm_sn_q[8];
                    old_time <= rm_time_q;
                    state <= DIFF;
                end
                DIFF: begin
                    age_lo <= {1'b0,time_l[15:0]} - {1'b0,old_time[15:0]};
                    age_hi_base <= time_l[31:16]-old_time[31:16];
                    distance <= sn_l >= lsn ? delta : delta-8'd1;
                    ic_ok <= IC_EN == 0 || !pv || (sn_l == 0 && p != 0) || sn_l == nx1 || sn_l == nx2;
                    state <= AGE;
                end
                AGE: begin
                    age_hi <= age_hi_base - {15'd0,age_lo[16]};
                    rm_seq_ok <= RM_EN == 0 || !rv || (sn_l == 0 && lsn != 0) ||
                                 (distance >= 1 && distance <= 127);
                    state <= CHECK;
                end
                CHECK: begin
                    expired <= {age_hi,age_lo[15:0]} > SKEW_MAX_US;
                    state <= DECIDE;
                end
                DECIDE: begin
                    decision <= ic_ok && (rm_seq_ok || expired);
                    state <= COMMIT;
                end
                COMMIT: begin
                    resp <= 1; accept <= decision;
                    stat_ic_drop <= !ic_ok;
                    stat_rm_drop <= ic_ok && !decision;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
