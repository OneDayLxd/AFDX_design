/*
 * ================================================================
 * File Name    : afdx_tx_cfg_ram.v
 * Module Name  : afdx_tx_cfg_ram
 *
 * Function:
 *   发送 VL 运行时配置：活动/影子双银行 RAM，保存每 VL 的 BAG、Lmax 与网络使能。
 *   - 复位后逐项把静态 ROM 拷入银行 0（init_busy 期间 hold 阻止帧组装器接受授权）；
 *   - 管理面只写影子银行：cmd_copy 先把活动银行复制到影子银行，m_wr_* 逐项修改；
 *   - cmd_verify 扫描影子银行：BAG ∈ {1,2,..,128} ms、1 <= Lmax <= ROM Lmax、网络使能非零，
 *     并按 (max(17,Lmax)+67) 字节 × (128 ms / BAG) × 上线副本数 累加 128 ms 总线字节预算；
 *   - cmd_apply / cmd_rollback 先拉高 hold，等帧组装器停在 S_IDLE 后一拍切换银行，
 *     因此一帧的 BAG/Lmax/网络使能与冗余信用始终来自同一银行。
 *   三个读口（授权、组帧、管理）各为一份同写的 1 读 1 写 RAM，读出两级寄存（延迟 2 拍）。
 *   VLID、UDP 端点、分区号仍由 ROM 提供，不属于运行时可改字段。
 *
 * Sub Module:
 *   afdx_tx_vl_rom
 *   afdx_tx_vl_id_rom
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_tx_cfg_ram #(
    parameter N_VL = 8,
    parameter VLW = 3,
    parameter SHARED_BUS = 1,               // 1: A/B 副本共用一条总线（AFDX_top 单总线复用）
    parameter [31:0] BUS_BUDGET = 32'd1600000   // 100 Mbps × 128 ms / 8
) (
    input  wire            clk,
    input  wire            rst,
    // 数据面读口
    input  wire [VLW-1:0]  g_idx,
    output reg  [19:0]     g_bag_us,
    output reg  [1:0]      g_net_en,
    input  wire [VLW-1:0]  f_idx,
    output reg  [19:0]     f_bag_us,
    output reg  [10:0]     f_lmax,
    output reg  [1:0]      f_net_en,
    output wire            hold,
    input  wire            builder_idle,
    // 管理读口：m_rd_shadow=1 读影子银行；ROM 字段同拍给出
    input  wire [VLW-1:0]  m_rd_idx,
    input  wire            m_rd_shadow,
    output reg  [19:0]     m_bag_us,
    output reg  [10:0]     m_lmax,
    output reg  [1:0]      m_net_en,
    output reg  [15:0]     m_vlid,
    output reg  [10:0]     m_rom_lmax,
    // 管理写口（影子银行），busy 期间忽略
    input  wire            m_wr_en,
    input  wire [VLW-1:0]  m_wr_idx,
    input  wire [19:0]     m_wr_bag_us,
    input  wire [10:0]     m_wr_lmax,
    input  wire [1:0]      m_wr_net_en,
    // 命令脉冲，仅在 !busy 时接受
    input  wire            cmd_copy,
    input  wire            cmd_verify,
    input  wire            cmd_apply,
    input  wire            cmd_rollback,
    output wire            busy,
    output wire            init_busy,
    output reg             active_bank,
    output reg             prev_valid,          // 可回滚到上一活动银行
    output reg             done,                // 命令完成脉冲
    output reg             verify_ok,
    output reg  [2:0]      verify_code,         // 0 通过 1 BAG 2 Lmax 3 网络 4 带宽
    output reg  [VLW-1:0]  verify_idx,          // 首个失败 VL
    output reg  [31:0]     budget               // 最近一次校验的 128 ms 总线字节数
);
    localparam DEPTH = 2 * N_VL;
    localparam EW = 33;
    localparam integer LAST_I = N_VL - 1;
    localparam [VLW-1:0] LAST = LAST_I[VLW-1:0];
    localparam M_IDLE = 3'd0, M_INIT = 3'd1, M_COPY = 3'd2, M_VERIFY = 3'd3,
               M_HOLD = 3'd4, M_SWITCH = 3'd5, M_DRAIN = 3'd6;
    reg [2:0] mode;
    reg switch_rollback;
    reg [VLW-1:0] sweep_idx;
    reg sweep_run;                              // 仍在发出扫描地址
    reg [1:0] pipe_valid;                       // 扫描读出流水线有效位
    reg [VLW-1:0] pipe_idx0, pipe_idx1;
    assign busy = mode != M_IDLE;
    assign init_busy = mode == M_INIT;
    reg hold_r;
    assign hold = hold_r;

    // ---------------- 存储体 ----------------
    (* ramstyle = "M4K, no_rw_check" *) reg [EW-1:0] mem_g [0:DEPTH-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [EW-1:0] mem_f [0:DEPTH-1];
    (* ramstyle = "M4K, no_rw_check" *) reg [EW-1:0] mem_m [0:DEPTH-1];
    reg wr_en;
    reg [VLW:0] wr_addr;
    reg [EW-1:0] wr_data;
    reg [EW-1:0] g_q, f_q, m_q;
    wire sweeping = (mode == M_COPY || mode == M_VERIFY) && sweep_run;
    wire [VLW:0] m_addr = sweeping ?
        {(mode == M_COPY) ? active_bank : ~active_bank, sweep_idx} :
        {m_rd_shadow ? ~active_bank : active_bank, m_rd_idx};
    always @(posedge clk) begin
        if (wr_en) begin
            mem_g[wr_addr] <= wr_data;
            mem_f[wr_addr] <= wr_data;
            mem_m[wr_addr] <= wr_data;
        end
        g_q <= mem_g[{active_bank, g_idx}];
        f_q <= mem_f[{active_bank, f_idx}];
        m_q <= mem_m[m_addr];
        {g_bag_us, g_net_en} <= {g_q[32:13], g_q[1:0]};
        {f_bag_us, f_lmax, f_net_en} <= f_q;
        {m_bag_us, m_lmax, m_net_en} <= m_q;
    end

    // ROM：扫描口（复位装载、校验时的 Lmax 上限）与管理读口（VLID、ROM Lmax）各一份，
    // 管理读口地址不经过扫描选择
    wire [19:0] rom_bag;
    wire [10:0] rom_lmax, rom_m_lmax;
    wire [1:0] rom_net;
    wire [15:0] rom_m_vlid;
    afdx_tx_vl_rom #(
        .VLW (VLW)
    ) u_rom (
        .idx       (sweep_idx),
        .vlid      (),
        .bag_us    (rom_bag),
        .lmax      (rom_lmax),
        .udp_src   (),
        .udp_dst   (),
        .net_en    (rom_net),
        .partition ()
    );
    afdx_tx_vl_id_rom #(
        .VLW (VLW)
    ) u_rom_m (
        .idx  (m_rd_idx),
        .vlid (rom_m_vlid),
        .lmax (rom_m_lmax)
    );
    reg [15:0] rom_vlid_q;
    reg [10:0] rom_lmax_q, rom_lmax_q2, rom_mlmax_q;
    always @(posedge clk) begin
        rom_vlid_q <= rom_m_vlid; m_vlid <= rom_vlid_q;
        rom_mlmax_q <= rom_m_lmax; m_rom_lmax <= rom_mlmax_q;
        rom_lmax_q <= rom_lmax; rom_lmax_q2 <= rom_lmax_q;
    end

    // ---------------- 校验流水线 ----------------
    // m_bag_us/m_lmax/m_net_en 与 rom_lmax_q2 对齐到同一扫描项。
    reg [2:0] v_k;
    reg v_bag_ok, v_lmax_ok, v_net_ok, v_valid, v_last;
    reg [VLW-1:0] v_idx;
    reg [11:0] v_frame;
    reg [1:0] v_copies;
    reg [19:0] bag_bytes;
    reg bag_double, bag_valid, bag_last;
    reg [19:0] s_bytes;
    reg s_valid, s_last;
    reg [31:0] acc;
    reg v_fail_seen;
    integer k;
    always @(posedge clk) begin
        v_bag_ok <= 1'b0; v_k <= 3'd0;
        for (k = 0; k < 8; k = k + 1)
            if (m_bag_us == (20'd1000 << k)) begin v_bag_ok <= 1'b1; v_k <= k[2:0]; end
        v_lmax_ok <= (m_lmax != 0) && (m_lmax <= rom_lmax_q2);
        v_net_ok <= m_net_en != 2'b00;
        v_frame <= ((m_lmax < 11'd17) ? 12'd17 : {1'b0, m_lmax}) + 12'd67;
        v_copies <= (SHARED_BUS != 0) ? ({1'b0, m_net_en[0]} + {1'b0, m_net_en[1]}) :
                    {1'b0, m_net_en[0] | m_net_en[1]};
        v_valid <= (mode == M_VERIFY) && pipe_valid[1];
        v_last <= pipe_idx1 == LAST;
        v_idx <= pipe_idx1;
        // 字节/128 ms = frame << (7-k)，副本数为 2 时再左移一位
        // Separate the variable shift from the copy-count mux.
        bag_bytes <= {8'd0, v_frame} << (3'd7 - v_k);
        bag_double <= v_copies == 2'd2;
        bag_valid <= v_valid && v_copies != 2'd0;
        bag_last <= v_valid && v_last;
        s_bytes <= bag_double ? {bag_bytes[18:0], 1'b0} : bag_bytes;
        s_valid <= bag_valid;
        s_last <= bag_last;
    end

    // ---------------- 控制 ----------------
    always @(posedge clk) begin
        if (rst) begin
            mode <= M_INIT; sweep_idx <= {VLW{1'b0}}; sweep_run <= 1'b1; pipe_valid <= 2'd0;
            active_bank <= 1'b0; prev_valid <= 1'b0; hold_r <= 1'b1; wr_en <= 1'b0;
            done <= 1'b0; verify_ok <= 1'b0; verify_code <= 3'd0; verify_idx <= {VLW{1'b0}};
            budget <= 32'd0; acc <= 32'd0; v_fail_seen <= 1'b0; switch_rollback <= 1'b0;
        end else begin
            wr_en <= 1'b0;
            done <= 1'b0;
            pipe_valid <= {pipe_valid[0], sweeping};
            pipe_idx0 <= sweep_idx; pipe_idx1 <= pipe_idx0;
            case (mode)
                M_INIT: begin
                    wr_en <= 1'b1;
                    wr_addr <= {1'b0, sweep_idx};
                    wr_data <= {rom_bag, rom_lmax, rom_net};
                    if (sweep_idx == LAST) begin
                        mode <= M_IDLE; hold_r <= 1'b0; sweep_run <= 1'b0;
                    end else sweep_idx <= sweep_idx + 1'b1;
                end
                M_IDLE: begin
                    sweep_idx <= {VLW{1'b0}};
                    if (cmd_copy) begin
                        mode <= M_COPY; sweep_run <= 1'b1; prev_valid <= 1'b0;
                    end else if (cmd_verify) begin
                        mode <= M_VERIFY; sweep_run <= 1'b1; acc <= 32'd0; v_fail_seen <= 1'b0;
                        verify_ok <= 1'b0; verify_code <= 3'd0; verify_idx <= {VLW{1'b0}};
                    end else if (cmd_apply || (cmd_rollback && prev_valid)) begin
                        mode <= M_HOLD; hold_r <= 1'b1; switch_rollback <= !cmd_apply;
                    end else if (cmd_rollback) begin
                        done <= 1'b1;
                    end else if (m_wr_en) begin
                        wr_en <= 1'b1;
                        wr_addr <= {~active_bank, m_wr_idx};
                        wr_data <= {m_wr_bag_us, m_wr_lmax, m_wr_net_en};
                        prev_valid <= 1'b0;
                    end
                end
                M_COPY: begin
                    if (sweep_run) begin
                        if (sweep_idx == LAST) sweep_run <= 1'b0;
                        else sweep_idx <= sweep_idx + 1'b1;
                    end
                    // m_q 是上一拍扫描地址的读出（RAM 寄存一级）
                    if (pipe_valid[0]) begin
                        wr_en <= 1'b1;
                        wr_addr <= {~active_bank, pipe_idx0};
                        wr_data <= m_q;
                        if (pipe_idx0 == LAST) begin mode <= M_IDLE; done <= 1'b1; end
                    end
                end
                M_VERIFY: begin
                    if (sweep_run) begin
                        if (sweep_idx == LAST) sweep_run <= 1'b0;
                        else sweep_idx <= sweep_idx + 1'b1;
                    end
                    if (v_valid && !v_fail_seen && !(v_bag_ok && v_lmax_ok && v_net_ok)) begin
                        v_fail_seen <= 1'b1;
                        verify_idx <= v_idx;
                        verify_code <= !v_bag_ok ? 3'd1 : !v_lmax_ok ? 3'd2 : 3'd3;
                    end
                    if (s_valid) acc <= acc + {12'd0, s_bytes};
                    if (s_last) begin
                        mode <= M_DRAIN;
                    end
                end
                M_DRAIN: begin
                    // 最后一项的累加在上一拍完成
                    budget <= acc;
                    verify_ok <= !v_fail_seen && acc <= BUS_BUDGET;
                    if (!v_fail_seen && acc > BUS_BUDGET) verify_code <= 3'd4;
                    mode <= M_IDLE; done <= 1'b1;
                end
                M_HOLD: if (builder_idle) mode <= M_SWITCH;
                M_SWITCH: begin
                    active_bank <= ~active_bank;
                    prev_valid <= !switch_rollback;
                    hold_r <= 1'b0;
                    mode <= M_IDLE; done <= 1'b1;
                end
                default: mode <= M_IDLE;
            endcase
        end
    end
endmodule
