/*
 * ================================================================
 * File Name    : afdx_snmp_values.v
 * Module Name  : afdx_snmp_values
 *
 * Function:
 *   SNMP 取值多路选择（5 级寄存）：val_k/val_inst 由代理寄存给出，val_data 5 拍后有效
 *   （代理在取值后等待 VAL_WAIT+1 拍再采样，留有余量）。
 *   各组总线按 MIB 条目顺序排列（字 0 = 组内第一个对象），见 afdx_mgmt_plane 的拼接：
 *     dp   : afdxStats R.2.1..25（计数类减去 statsClear 基线，缓冲类为瞬时值）
 *     ld   : loader R.4.1..16    rtc : R.5.2..28（R.5.1 为 64 位 rtc）    ft : R.6.1..22
 *   表列（afdxVlTable）来自运行时配置 RAM 的管理读口（cfg_m_rd_idx = 实例 - 1，列 6..8 读影子银行）。
 *   时序：第 1 级只把 val_k 译成各组的组内序号与组号（按字节道各一份，限制扇出），第 2 级各组内选择，
 *        （dp/rtc 组拆为 8 选 1 + 4 选 1 两拍，其他组第 2 级结果再寄存一拍对齐），
 *        第 3 级组选择并减去基线，第 4 级表列选择；配置 RAM 读口地址同样在第 1 级寄存。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_snmp_mib.vh"
module afdx_snmp_values #(
    parameter VLW = 7,
    parameter [31:0] FW_VERSION = 32'h00020000,
    parameter [31:0] BUILD_ID = 32'h20260925,
    parameter [31:0] ES_ID = 32'h00000101
) (
    input  wire          clk,
    input  wire          rst,
    input  wire [6:0]    val_k,
    input  wire [15:0]   val_inst,
    output reg  [63:0]   val_data,
    input  wire          stats_clear,
    input  wire [799:0]  dp,                // 25 x 32
    input  wire [31:0]   uptime_ticks,
    input  wire [31:0]   mono_us,
    input  wire [31:0]   cfg_generation,
    input  wire          cfg_active_bank,
    input  wire [31:0]   cfg_last_result,
    input  wire [31:0]   cfg_budget,
    output wire [VLW-1:0] cfg_m_rd_idx,
    output wire          cfg_m_rd_shadow,
    input  wire [15:0]   cfg_vlid,
    input  wire [19:0]   cfg_bag_us,
    input  wire [10:0]   cfg_lmax,
    input  wire [1:0]    cfg_net_en,
    input  wire [10:0]   cfg_rom_lmax,
    input  wire [511:0]  ld,                // 16 x 32
    input  wire [63:0]   rtc64,
    input  wire [863:0]  rtc,               // 27 x 32
    input  wire [703:0]  ft                 // 22 x 32
);
    // 统计清零基线（仅计数类：R.2.1..20 与 R.2.25）存于 RAM。复位后逐项写 0；statsClear 时借用
    // 第 1、2 级的 dp 选择通路逐项采样（25 拍，其间代理不会取值：清零发生在 SET 第二阶段，
    // 下一次取值属于之后的作业）。
    localparam [4:0] DP_LAST = 5'd24;
    (* ramstyle = "M4K, no_rw_check" *) reg [31:0] base_mem [0:31];
    reg sw_run, sw_init;
    reg [4:0] sw_i, sw_i1, sw_i2, sw_i3;
    reg sw_v1, sw_v2, sw_v3;
    always @(posedge clk) begin
        if (rst) begin sw_run <= 1'b1; sw_init <= 1'b1; sw_i <= 5'd0; end
        else if (stats_clear) begin sw_run <= 1'b1; sw_init <= 1'b0; sw_i <= 5'd0; end
        else if (sw_run) begin
            sw_i <= sw_i + 1'b1;
            if (sw_i == DP_LAST) sw_run <= 1'b0;
        end
        // 与第 1、2 级对齐：sw_i3 为本拍 g_dp 对应的项
        sw_v1 <= !rst && sw_run && !stats_clear; sw_i1 <= sw_i;
        sw_v2 <= !rst && sw_v1; sw_i2 <= sw_i1;
        sw_v3 <= !rst && sw_v2; sw_i3 <= sw_i2;
    end
    reg [31:0] base_q;
    (* preserve *) reg [4:0] a_base, a_base2;             // 基线 RAM 读地址（与字节道序号同值的独立寄存器）
    // 第 1 级：译码。组内序号与有效位按 8 位字节道各寄存一份（preserve，不被合并），
    // 每份只驱动本字节道的多路选择器。
    wire [6:0] i_dp = val_k - `MIB_TXMSGS;
    wire [6:0] i_ld = val_k - `MIB_LDSTATE;
    wire [6:0] i_rt = val_k - `MIB_RTCTIMELOW;
    wire [6:0] i_ft = val_k - `MIB_TRAPMASK;
    reg [2:0] grp;
    (* preserve *) reg [6:0] k1;                // 与字节道序号的低位同值，保留为独立寄存器
    reg [6:0] k1b, k2, k3;
    reg [VLW-1:0] rd_idx_q;
    reg rd_shadow_q;
    always @(posedge clk) begin
        k1 <= val_k;
        a_base <= sw_run ? sw_i : i_dp[4:0]; a_base2 <= a_base;
        grp <= (val_k == `MIB_RTCTIME || val_k == `MIB_RTCSET) ? 3'd6 :
               (val_k >= `MIB_TXMSGS && val_k <= `MIB_RXERRTOTAL) ? 3'd1 :
               (val_k >= `MIB_LDSTATE && val_k <= `MIB_LDAPPCRC) ? 3'd2 :
               (val_k >= `MIB_RTCTIMELOW && val_k <= `MIB_RTCLASTSYNCMONO) ? 3'd3 :
               (val_k >= `MIB_TRAPMASK && val_k <= `MIB_TFTPERRORS) ? 3'd4 :
               (val_k >= `MIB_VLID && val_k <= `MIB_VLROMLMAX) ? 3'd5 : 3'd0;
        rd_idx_q <= val_inst[VLW-1:0] - 1'b1;
        rd_shadow_q <= (val_k == `MIB_VLSHADOWBAGUS) || (val_k == `MIB_VLSHADOWLMAX) ||
                       (val_k == `MIB_VLSHADOWNETEN);
    end
    assign cfg_m_rd_idx = rd_idx_q;
    assign cfg_m_rd_shadow = rd_shadow_q;
    // 第 2 级：各组内选择（按字节道）
    wire [31:0] g_dp, g_ld, g_rt, g_ft;
    wire [1023:0] rtc_pad = {160'd0, rtc};
    genvar ln;
    generate for (ln = 0; ln < 4; ln = ln + 1) begin : g_lane
        (* preserve *) reg [4:0] a_dp, a_rt, a_ft;
        (* preserve *) reg [3:0] a_ld;
        (* preserve *) reg v_dp, v_ld, v_rt, v_ft;
        reg [7:0] q_dp, q_ld, q_ft, r_ld, r_rt, r_ft;
        reg [7:0] p_dp [0:3];                           // dp 第 2a 级：每 8 项一组的 8 选 1
        reg [7:0] p_rt [0:3];                           // RTC 同样利用已有两级，避免单拍 27 选 1
        reg [1:0] h_dp;
        reg [1:0] h_rt;
        reg w_dp, w_rt;
        integer gi;
        always @(posedge clk) begin
            a_dp <= sw_run ? sw_i : i_dp[4:0]; v_dp <= sw_run || i_dp < 7'd25;
            a_ld <= i_ld[3:0]; v_ld <= i_ld < 7'd16;
            a_rt <= i_rt[4:0]; v_rt <= i_rt < 7'd27;
            a_ft <= i_ft[4:0]; v_ft <= i_ft < 7'd22;
            for (gi = 0; gi < 3; gi = gi + 1)
                p_dp[gi] <= dp[(gi*8 + a_dp[2:0])*32 + ln*8 +: 8];
            p_dp[3] <= dp[24*32 + ln*8 +: 8];
            h_dp <= a_dp[4:3]; w_dp <= v_dp;
            for (gi = 0; gi < 4; gi = gi + 1)
                p_rt[gi] <= rtc_pad[(gi*8 + a_rt[2:0])*32 + ln*8 +: 8];
            h_rt <= a_rt[4:3]; w_rt <= v_rt;
            q_dp <= w_dp ? p_dp[h_dp] : 8'd0;              // 基线在第 3 级减去
            q_ld <= v_ld ? ld[a_ld*32 + ln*8 +: 8] : 8'd0;
            q_ft <= v_ft ? ft[a_ft*32 + ln*8 +: 8] : 8'd0;
            r_ld <= q_ld; r_rt <= w_rt ? p_rt[h_rt] : 8'd0; r_ft <= q_ft;
        end
        assign g_dp[ln*8 +: 8] = q_dp;
        assign g_ld[ln*8 +: 8] = r_ld;
        assign g_rt[ln*8 +: 8] = r_rt;
        assign g_ft[ln*8 +: 8] = r_ft;
    end endgenerate
    // 基线 RAM：读出与 g_dp 同拍给出；写口在清零/复位扫描时写入
    // 采样值（计数类）或 0（复位与非计数类）
    wire [31:0] g_base = base_q;
    always @(posedge clk) begin
        if (sw_v3) base_mem[sw_i3] <= (sw_init || !(sw_i3 < 5'd20 || sw_i3 == DP_LAST)) ? 32'd0 : g_dp;
        base_q <= base_mem[a_base2];
    end
    reg [31:0] g_misc;
    reg [2:0] grp1b, grp2;
    always @(posedge clk) begin
        k1b <= k1; grp1b <= grp;
        k2 <= k1b; grp2 <= grp1b;
        case (k1b)
            `MIB_SYSUPTIME:      g_misc <= uptime_ticks;
            `MIB_ESFWVERSION:    g_misc <= FW_VERSION;
            `MIB_ESBUILDID:      g_misc <= BUILD_ID;
            `MIB_ESIDENTITY:     g_misc <= ES_ID;
            `MIB_ESMONOTONICUS:  g_misc <= mono_us;
            `MIB_CFGGENERATION:  g_misc <= cfg_generation;
            `MIB_CFGACTIVEBANK:  g_misc <= {31'd0, cfg_active_bank};
            `MIB_CFGLASTRESULT:  g_misc <= cfg_last_result;
            `MIB_CFGBUDGET:      g_misc <= cfg_budget;
            default:             g_misc <= 32'd0;
        endcase
    end
    // 第 3 级：组选择；第 4 级：表列（配置 RAM 读出在第 1 级地址后两拍稳定）
    reg [63:0] s3;
    reg [2:0] grp3;
    always @(posedge clk) begin
        k3 <= k2; grp3 <= grp2;
        case (grp2)
            3'd1: s3 <= {32'd0, g_dp - g_base};
            3'd2: s3 <= {32'd0, g_ld};
            3'd3: s3 <= {32'd0, g_rt};
            3'd4: s3 <= {32'd0, g_ft};
            3'd6: s3 <= rtc64;
            default: s3 <= {32'd0, g_misc};
        endcase
        if (grp3 == 3'd5) begin
            case (k3)
                `MIB_VLID:                     val_data <= {48'd0, cfg_vlid};
                `MIB_VLBAGUS, `MIB_VLSHADOWBAGUS: val_data <= {44'd0, cfg_bag_us};
                `MIB_VLLMAX, `MIB_VLSHADOWLMAX:   val_data <= {53'd0, cfg_lmax};
                `MIB_VLNETEN, `MIB_VLSHADOWNETEN: val_data <= {62'd0, cfg_net_en};
                default:                       val_data <= {53'd0, cfg_rom_lmax};
            endcase
        end else val_data <= s3;
    end
endmodule
