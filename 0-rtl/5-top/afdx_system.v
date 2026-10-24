/*
 * ================================================================
 * File Name    : afdx_system.v
 * Module Name  : afdx_system
 *
 * Function:
 *   AFDX 应用/管理/协议系统：集成 VL 调度、组帧、接收和协议服务。
 *   MAC 总线信号约定（按模板方向）：
 *     tx_data/tx_valid : 发送字节及字节有效，字节速率按 BUS_BYTE_CYCLES 限制为 100 Mbps；
 *     tx_ready (输出)  : 发送帧包络，一帧（含 FCS）期间保持为高，MAC 据此识别帧起止；
 *     rx_data/rx_valid : 接收字节及字节有效；
 *     rx_ready (输入)  : 接收帧包络，MAC 在一帧期间保持为高（若 MAC 常高，则以字节空闲间隙判帧尾）。
 *   单条 MAC 总线上复用网络 A / 网络 B 两路冗余帧（发送侧按帧交替输出，接收侧按源 MAC 接口号区分网络），
 *   内部集成应用消息发生器与接收槽，无需额外主机接口即可自发自收。
 *   afdx_mac_system 集成本系统与两路 GMII MAC 适配器；AFDX_top 提供应用载荷入口。
 *   APP_EXTERNAL=1 时关闭演示消息发生器，通过 host_* 端口连接外部应用。
 *   EXTERNAL_MAC=1 时使用 ext_a/ext_b 无 FCS 帧流，关闭共享总线带宽校验与源地址网络判定。
 *
 * Sub Module:
 *   afdx_gen_rom
 *   afdx_app_gen
 *   afdx_app_sink
 *   afdx_mgmt_plane
 *   afdx_app_mux
 *   afdx_es_core
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module afdx_system #(
    parameter             APP_EXTERNAL         = 0,             // 1: 外部应用消息流
    parameter             EXTERNAL_MAC         = 0,             // 1: A/B 独立无 FCS 帧流
    parameter             CLK_HZ               = 100_000_000,   // 时钟频率
    parameter             BUS_BYTE_CYCLES      = 8,          // 每字节占用的时钟周期数（100 Mbps: CLK_HZ / 12.5MHz）
    parameter             APP_GEN_EN           = 1,             // 内部消息发生器使能
    parameter             DUAL_NET_ON_BUS      = 1,          // 1: A/B 两路帧都送上总线; 0: 仅网络 A
    parameter             SKEW_MAX_US          = 10000,         // 冗余管理 SkewMax
    parameter             UDP_CSUM_EN          = 1,
    parameter             TX_POOL_LOG          = 5,
    parameter             RX_POOL_LOG          = 5,
    parameter             BLOCK_AW             = 8,
    parameter             TX_MAX_MSGS          = 128,
    parameter             TX_SOF_FEEDBACK      = 1,
    parameter             TX_PHASE_ENABLE      = 0,
    parameter             MGMT_EN              = 1,                  // SNMP/TFTP-615A/RTC 管理面
    parameter [127:0]     SNMP_WRITE_COMM      = 128'd0, // 写团体字（缺省未配置，SET 返回 noAccess）
    parameter [4:0]       SNMP_WRITE_COMM_LEN  = 5'd0,
    parameter             TFTP_TIMEOUT_US      = 1000000,
    parameter             SELFTEST_US          = 50000,
    parameter             APP_AW               = 12,                  // 应用镜像每银行 2^APP_AW 字节
    parameter [7:0]       NET_ID               = 8'd1,
    parameter [7:0]       EQ_ID                = 8'd1
) (
    input   wire                         clk,  // 时钟信号
    input   wire                         reset,  // 复位信号
    output  wire [7:0]                   tx_data,  // 发送数据
    input   wire [7:0]                   rx_data,  // 接收数据
    output  wire                         tx_valid,  // 发送数据有效指示
    input   wire                         rx_valid,  // 接收数据有效指示
    output  wire                         tx_ready,  // 发送数据准备好指示
    input   wire                         rx_ready,  // 接收数据准备好指示
    output  wire [7:0]                   ext_a_tx_data,
    output  wire                         ext_a_tx_valid,
    output  wire                         ext_a_tx_last,
    input   wire                         ext_a_tx_ready,
    input   wire                         ext_a_wire_sof,
    input   wire                         ext_a_wire_done,
    input   wire [7:0]                   ext_a_rx_data,
    input   wire                         ext_a_rx_valid,
    input   wire                         ext_a_rx_last,
    output  wire                         ext_a_rx_ready,
    input   wire [31:0]                  ext_a_rx_ts,
    input   wire [31:0]                  ext_a_rx_good_count,
    input   wire [31:0]                  ext_a_rx_error_count,
    input   wire [31:0]                  ext_a_rx_length_count,
    input   wire [31:0]                  ext_a_rx_overflow_count,
    output  wire [7:0]                   ext_b_tx_data,
    output  wire                         ext_b_tx_valid,
    output  wire                         ext_b_tx_last,
    input   wire                         ext_b_tx_ready,
    input   wire                         ext_b_wire_sof,
    input   wire                         ext_b_wire_done,
    input   wire [7:0]                   ext_b_rx_data,
    input   wire                         ext_b_rx_valid,
    input   wire                         ext_b_rx_last,
    output  wire                         ext_b_rx_ready,
    input   wire [31:0]                  ext_b_rx_ts,
    input   wire [31:0]                  ext_b_rx_good_count,
    input   wire [31:0]                  ext_b_rx_error_count,
    input   wire [31:0]                  ext_b_rx_length_count,
    input   wire [31:0]                  ext_b_rx_overflow_count,
    output  wire [31:0]                  ext_ts_us,
    input   wire [`AFDX_TX_VLW-1:0]      host_tx_vl,
    input   wire                         host_tx_valid,
    input   wire [7:0]                   host_tx_data,
    input   wire                         host_tx_last,
    output  wire                         host_tx_ready,
    output  wire [`AFDX_RX_VLW-1:0]      host_rx_vl,
    output  wire                         host_rx_valid,
    output  wire [7:0]                   host_rx_data,
    output  wire                         host_rx_last,
    input   wire                         host_rx_ready,
    output  wire [10:0]                  host_rx_len,
    output  wire [7:0]                   host_rx_sn,
    output  wire                         host_rx_net,
    output  wire [31:0]                  host_rx_ts,
    output  wire [31:0]                  host_rx_src_ip,
    output  wire [15:0]                  host_rx_sport,
    output  wire [15:0]                  host_rx_dport
);
    `include "afdx_math.vh"
    localparam N_TX_VL = `AFDX_N_TX_VL;
    localparam TX_VLW  = `AFDX_TX_VLW;
    localparam N_RX_VL = `AFDX_N_RX_VL;
    localparam RX_VLW  = `AFDX_RX_VLW;
    // 管理 VL Lmax 与管理发送窗口来自配置生成器（未生成时：Lmax 1471、不开窗）
`ifdef AFDX_MGMT_LMAX
    localparam MGMT_LMAX = `AFDX_MGMT_LMAX;
`else
    localparam MGMT_LMAX = 1471;
`endif
`ifdef AFDX_MGMT_WIN_PERIOD_US
    localparam MGMT_WIN_VL = `AFDX_MGMT_WIN_VL;
    localparam MGMT_WIN_PERIOD_US = `AFDX_MGMT_WIN_PERIOD_US;
    localparam MGMT_WIN_START_US = `AFDX_MGMT_WIN_START_US;
    localparam MGMT_WIN_OPEN_US = `AFDX_MGMT_WIN_OPEN_US;
`else
    localparam MGMT_WIN_VL = 0;
    localparam MGMT_WIN_PERIOD_US = 0;
    localparam MGMT_WIN_START_US = 0;
    localparam MGMT_WIN_OPEN_US = 0;
`endif

    // 复位同步
    (* maxfan = 32, preserve *) reg [1:0] rst_sync;
    always @(posedge clk) rst_sync <= {rst_sync[0], reset};
    wire rst = rst_sync[1];
    // 顶层各模块的本地复位（与核心、管理面内部的本地复位同为同步器后一拍）：
    // [0] 顶层寄存器  [1] 消息发生器  [2] 接收槽  [3] 应用复用器
    (* preserve, maxfan = 64 *) reg [3:0] rst_t;
    always @(posedge clk) rst_t <= {4{rst}};

    // MAC 总线接口寄存：输入引脚直接进触发器、输出引脚直接由触发器驱动，
    // 引脚与内部逻辑之间没有组合路径（各增加一拍固定延迟）
    reg [7:0] rx_data_q;
    reg rx_valid_q, rx_ready_q;
    always @(posedge clk) begin
        rx_data_q <= rx_data;
        rx_valid_q <= rx_valid;
        rx_ready_q <= rx_ready;
    end

    // 核心应用接口、普通应用发生器及管理响应接口
    wire [TX_VLW-1:0] app_tx_vl;  wire app_tx_valid, app_tx_last, app_tx_ready; wire [7:0] app_tx_data;
    wire [TX_VLW-1:0] gen_tx_vl;  wire gen_tx_valid, gen_tx_last, gen_tx_ready; wire [7:0] gen_tx_data;
    wire [TX_VLW-1:0] mgmt_tx_vl; wire mgmt_tx_valid, mgmt_tx_last, mgmt_tx_ready; wire [7:0] mgmt_tx_data;
    wire [33:0] mgmt_tx_ep, app_tx_ep;
    wire [RX_VLW-1:0] app_rx_vl;  wire app_rx_valid, app_rx_last, app_rx_ready; wire [7:0] app_rx_data;
    wire [10:0] app_rx_len; wire [7:0] app_rx_sn; wire app_rx_net;
    wire [31:0] app_rx_src_ip, app_rx_ts; wire [15:0] app_rx_sport, app_rx_dport;
    // MAC A/B
    wire [7:0] a_tx_data, b_tx_data; wire a_tx_valid, b_tx_valid, a_tx_slot, b_tx_slot, a_tx_busy, b_tx_busy;
    wire a_tx_frame, b_tx_frame;
    wire tick_us, sof_tag_a, sof_tag_b, mgmt_window;
    wire [31:0] mono_us, core_rtc_us;
    wire [63:0] rtc64;
    assign ext_ts_us = rtc64[31:0];
    assign ext_a_tx_data = a_tx_data;
    assign ext_b_tx_data = b_tx_data;
    assign ext_a_tx_valid = a_tx_valid;
    assign ext_b_tx_valid = b_tx_valid;
    wire [31:0] cnt_rx_mac_error;

    // 内部消息发生器（使用核心的微秒节拍）
    wire [TX_VLW-1:0] gen_scan_idx, gen_len_idx; wire [19:0] gen_period_us, gen_phase_us; wire [10:0] gen_len;
    wire [31:0] cnt_gen_msgs;
    afdx_gen_rom #(
        .VLW (TX_VLW)
    ) u_gen_rom_p (
        .idx       (gen_scan_idx),
        .period_us (gen_period_us),
        .phase_us  (gen_phase_us),
        .len       ()
    );
    afdx_gen_rom #(
        .VLW (TX_VLW)
    ) u_gen_rom_l (
        .idx       (gen_len_idx),
        .period_us (),
        .phase_us  (),
        .len       (gen_len)
    );
    afdx_app_gen #(
        .N_VL (N_TX_VL),
        .VLW  (TX_VLW)
    ) u_gen (
        .clk           (clk),
        .rst           (rst_t[1]),
        .enable        ((APP_GEN_EN != 0) && (APP_EXTERNAL == 0)),
        .tick_us       (tick_us),
        .scan_idx      (gen_scan_idx),
        .rom_period_us (gen_period_us),
        .rom_phase_us  (gen_phase_us),
        .len_idx       (gen_len_idx),
        .rom_len       (gen_len),
        .msg_vl        (gen_tx_vl),
        .msg_valid     (gen_tx_valid),
        .msg_data      (gen_tx_data),
        .msg_last      (gen_tx_last),
        .msg_ready     (gen_tx_ready),
        .cnt_msgs      (cnt_gen_msgs)
    );

    wire [31:0] cnt_sink_msgs, cnt_sink_bytes; wire sink_ready;
    afdx_app_sink #(
        .VLW (RX_VLW)
    ) u_sink (
        .clk       (clk),
        .rst       (rst_t[2]),
        .msg_vl    (app_rx_vl),
        .msg_valid (app_rx_valid && (APP_EXTERNAL == 0)),
        .msg_data  (app_rx_data),
        .msg_last  (app_rx_last),
        .msg_ready (sink_ready),
        .cnt_msgs  (cnt_sink_msgs),
        .cnt_bytes (cnt_sink_bytes)
    );

    // 统计计数器（SNMP afdxStats）
    wire [31:0] cnt_tx_msgs, cnt_tx_frames, cnt_tx_frames_a, cnt_tx_frames_b, cnt_tx_lmax_drop, cnt_tx_ovf_drop;
    wire [31:0] cnt_rx_frames_a, cnt_rx_frames_b, cnt_rx_fcs_err, cnt_rx_mac_len_err, cnt_rx_mac_ovf;
    wire [31:0] cnt_rx_hdr_err, cnt_rx_len_err, cnt_rx_vl_unknown, cnt_rx_csum_err, cnt_rx_full_drop;
    wire [31:0] cnt_rx_ic_drop, cnt_rx_rm_drop, cnt_rx_accepted, cnt_rx_delivered;
    wire [TX_POOL_LOG+11-BLOCK_AW:0] tx_buffer_used, tx_buffer_peak, tx_buffer_messages, tx_buffer_messages_peak;
    wire [RX_POOL_LOG+11-BLOCK_AW:0] rx_buffer_used, rx_buffer_peak, rx_buffer_messages, rx_buffer_messages_peak;
    // Balanced pipeline: each snapshot sums the same cycle's nine counters.
    // Statistics are four cycles old; dataplane counters themselves are exact.
    reg [31:0] err_pair0, err_pair1, err_pair2, err_pair3, err_last0;
    reg [31:0] err_quad0, err_quad1, err_last1, err_oct, err_last2, err_total;
    always @(posedge clk) begin
        if (rst_t[0]) begin
            err_pair0<=0; err_pair1<=0; err_pair2<=0; err_pair3<=0; err_last0<=0;
            err_quad0<=0; err_quad1<=0; err_last1<=0; err_oct<=0; err_last2<=0; err_total<=0;
        end else begin
            err_pair0 <= cnt_rx_fcs_err + cnt_rx_mac_len_err + cnt_rx_mac_error;
            err_pair1 <= cnt_rx_mac_ovf + cnt_rx_hdr_err;
            err_pair2 <= cnt_rx_len_err + cnt_rx_vl_unknown;
            err_pair3 <= cnt_rx_csum_err + cnt_rx_full_drop;
            err_last0 <= cnt_rx_ic_drop;
            err_quad0 <= err_pair0+err_pair1; err_quad1 <= err_pair2+err_pair3;
            err_last1 <= err_last0;
            err_oct <= err_quad0+err_quad1; err_last2 <= err_last1;
            err_total <= err_oct+err_last2;
        end
    end
    // 管理面读取的统计全部先寄存一拍，不进入数据面计数器的时序路径
    reg [799:0] dp_stats;
    reg [31:0] tx_frames_q;
    reg tx_frame_pulse;
    always @(posedge clk) begin
        dp_stats <= {err_total, 32'd0 | rx_buffer_peak, 32'd0 | rx_buffer_used, 32'd0 | tx_buffer_peak,
                     32'd0 | tx_buffer_used, cnt_rx_delivered, cnt_rx_accepted, cnt_rx_rm_drop, cnt_rx_ic_drop,
                     cnt_rx_full_drop, cnt_rx_csum_err, cnt_rx_vl_unknown, cnt_rx_len_err, cnt_rx_hdr_err,
                     cnt_rx_mac_ovf, cnt_rx_mac_len_err, cnt_rx_fcs_err, cnt_rx_frames_b, cnt_rx_frames_a,
                     cnt_tx_ovf_drop, cnt_tx_lmax_drop, cnt_tx_frames_b, cnt_tx_frames_a, cnt_tx_frames, cnt_tx_msgs};
        tx_frames_q <= cnt_tx_frames;
        tx_frame_pulse <= !rst_t[0] && tx_frames_q != cnt_tx_frames;
    end

    // 管理面通过管理 VL 上的 UDP 报文收发（SNMP 161/162、TFTP 69、RTC 同步）。
    wire [TX_VLW-1:0] cfg_m_rd_idx, cfg_m_wr_idx, cfg_verify_idx;
    wire cfg_m_rd_shadow, cfg_m_wr_en, cfg_cmd_copy, cfg_cmd_verify, cfg_cmd_apply, cfg_cmd_rollback;
    wire cfg_busy, cfg_active_bank, cfg_prev_valid, cfg_done, cfg_verify_ok;
    wire [19:0] cfg_m_bag_us, cfg_m_wr_bag_us;
    wire [10:0] cfg_m_lmax, cfg_m_rom_lmax, cfg_m_wr_lmax;
    wire [1:0] cfg_m_net_en, cfg_m_wr_net_en;
    wire [15:0] cfg_m_vlid;
    wire [2:0] cfg_verify_code, rtc_state;
    wire [31:0] cfg_budget, app_length, app_version;
    wire [7:0] app_rd_data;
    wire app_bank;
    afdx_mgmt_plane #(
        .VLW             (RX_VLW),
        .TX_VLW          (TX_VLW),
        .N_TX_VL         (N_TX_VL),
        .MGMT_RX_VL      (0),
        .MGMT_TX_VL      (0),
        .WRITE_COMM      (SNMP_WRITE_COMM),
        .WRITE_COMM_LEN  (SNMP_WRITE_COMM_LEN),
        .TFTP_TIMEOUT_US (TFTP_TIMEOUT_US),
        .APP_AW          (APP_AW),
        .SELFTEST_US     (SELFTEST_US),
        .NET_ID          (NET_ID),
        .EQ_ID           (EQ_ID),
        .SNMP_MAX_PDU    (MGMT_LMAX)
    ) u_mgmt (
        .clk              (clk),
        .rst              (rst),
        .enable           (MGMT_EN != 0),
        .rx_vl            (app_rx_vl),
        // 管理旁路仅采样已交付字节，外部应用背压时不得重复写入。
        .rx_valid         (app_rx_valid && app_rx_ready),
        .rx_data          (app_rx_data),
        .rx_last          (app_rx_last),
        .rx_src_ip        (app_rx_src_ip),
        .rx_sport         (app_rx_sport),
        .rx_dport         (app_rx_dport),
        .rx_ts            (app_rx_ts),
        .tx_vl            (mgmt_tx_vl),
        .tx_valid         (mgmt_tx_valid),
        .tx_data          (mgmt_tx_data),
        .tx_last          (mgmt_tx_last),
        .tx_ep            (mgmt_tx_ep),
        .tx_ready         (mgmt_tx_ready),
        .tick_us          (tick_us),
        .mono_us          (mono_us),
        .rtc64            (rtc64),
        .sof_tag          (sof_tag_a | sof_tag_b),
        .dp_stats         (dp_stats),
        .tx_frame         (tx_frame_pulse),
        .cfg_m_rd_idx     (cfg_m_rd_idx),
        .cfg_m_rd_shadow  (cfg_m_rd_shadow),
        .cfg_m_bag_us     (cfg_m_bag_us),
        .cfg_m_lmax       (cfg_m_lmax),
        .cfg_m_net_en     (cfg_m_net_en),
        .cfg_m_vlid       (cfg_m_vlid),
        .cfg_m_rom_lmax   (cfg_m_rom_lmax),
        .cfg_m_wr_en      (cfg_m_wr_en),
        .cfg_m_wr_idx     (cfg_m_wr_idx),
        .cfg_m_wr_bag_us  (cfg_m_wr_bag_us),
        .cfg_m_wr_lmax    (cfg_m_wr_lmax),
        .cfg_m_wr_net_en  (cfg_m_wr_net_en),
        .cfg_cmd_copy     (cfg_cmd_copy),
        .cfg_cmd_verify   (cfg_cmd_verify),
        .cfg_cmd_apply    (cfg_cmd_apply),
        .cfg_cmd_rollback (cfg_cmd_rollback),
        .cfg_busy         (cfg_busy),
        .cfg_active_bank  (cfg_active_bank),
        .cfg_prev_valid   (cfg_prev_valid),
        .cfg_done         (cfg_done),
        .cfg_verify_ok    (cfg_verify_ok),
        .cfg_verify_code  (cfg_verify_code),
        .cfg_budget       (cfg_budget),
        .app_rd_addr      ({APP_AW{1'b0}}),
        .app_rd_data      (app_rd_data),
        .app_bank         (app_bank),
        .app_length       (app_length),
        .app_version      (app_version),
        .rtc_state        (rtc_state)
    );
    // 管理面不对接收交付施加反压（请求缓冲满时丢弃并计数）
    assign app_rx_ready = (APP_EXTERNAL != 0) ? host_rx_ready : sink_ready;
    assign host_rx_vl = app_rx_vl;
    assign host_rx_valid = app_rx_valid && (APP_EXTERNAL != 0);
    assign host_rx_data = app_rx_data;
    assign host_rx_last = app_rx_last;
    assign host_rx_len = app_rx_len;
    assign host_rx_sn = app_rx_sn;
    assign host_rx_net = app_rx_net;
    assign host_rx_ts = app_rx_ts;
    assign host_rx_src_ip = app_rx_src_ip;
    assign host_rx_sport = app_rx_sport;
    assign host_rx_dport = app_rx_dport;
    wire source_ready;
    assign host_tx_ready = (APP_EXTERNAL != 0) && source_ready;
    assign gen_tx_ready = (APP_EXTERNAL == 0) && source_ready;

    afdx_app_mux #(
        .VLW (TX_VLW)
    ) u_app_mux (
        .clk        (clk),
        .rst        (rst_t[3]),
        .app_vl     ((APP_EXTERNAL != 0) ? host_tx_vl : gen_tx_vl),
        .app_valid  ((APP_EXTERNAL != 0) ? host_tx_valid : gen_tx_valid),
        .app_data   ((APP_EXTERNAL != 0) ? host_tx_data : gen_tx_data),
        .app_last   ((APP_EXTERNAL != 0) ? host_tx_last : gen_tx_last),
        .app_ready  (source_ready),
        .mgmt_vl    (mgmt_tx_vl),
        .mgmt_valid (mgmt_tx_valid),
        .mgmt_data  (mgmt_tx_data),
        .mgmt_last  (mgmt_tx_last),
        .mgmt_ep    (mgmt_tx_ep),
        .mgmt_allow (mgmt_window),
        .mgmt_ready (mgmt_tx_ready),
        .out_vl     (app_tx_vl),
        .out_valid  (app_tx_valid),
        .out_data   (app_tx_data),
        .out_last   (app_tx_last),
        .out_ep     (app_tx_ep),
        .out_ready  (app_tx_ready)
    );

    // 单总线复用：接收侧两路都由总线馈入（A 口），网络归属由源 MAC 接口号判定
    afdx_es_core #(
        .EXTERNAL_MAC              (EXTERNAL_MAC),
        .CLK_HZ                    (CLK_HZ),
        .N_TX_VL                   (N_TX_VL),
        .TX_VLW                    (TX_VLW),
        .N_RX_VL                   (N_RX_VL),
        .RX_VLW                    (RX_VLW),
        .NET_ID                    (NET_ID),
        .EQ_ID                     (EQ_ID),
        .SKEW_MAX_US               (SKEW_MAX_US),
        .UDP_CSUM_EN               (UDP_CSUM_EN),
        .NET_FROM_SA               (EXTERNAL_MAC == 0),
        .RX_GAP_CYCLES             (4 * BUS_BYTE_CYCLES),
        .TX_POOL_LOG               (TX_POOL_LOG),
        .RX_POOL_LOG               (RX_POOL_LOG),
        .BLOCK_AW                  (BLOCK_AW),
        .TX_MAX_MSGS               (TX_MAX_MSGS),
        .TX_SOF_FEEDBACK           (TX_SOF_FEEDBACK),
        .TX_PHASE_ENABLE           (TX_PHASE_ENABLE),
        .CFG_SHARED_BUS            ((EXTERNAL_MAC != 0) ? 0 : DUAL_NET_ON_BUS),
        .MGMT_WIN_VL               (MGMT_WIN_VL),
        .MGMT_WIN_PERIOD_US        (MGMT_WIN_PERIOD_US),
        .MGMT_WIN_START_US         (MGMT_WIN_START_US),
        .MGMT_WIN_OPEN_US          (MGMT_WIN_OPEN_US)
    ) u_core (
        .clk                       (clk),
        .rst                       (rst),
        .tx_msg_vl                 (app_tx_vl),
        .tx_msg_valid              (app_tx_valid),
        .tx_msg_data               (app_tx_data),
        .tx_msg_last               (app_tx_last),
        .tx_msg_ready              (app_tx_ready),
        .tx_msg_ep                 (app_tx_ep),
        .rx_msg_vl                 (app_rx_vl),
        .rx_msg_valid              (app_rx_valid),
        .rx_msg_data               (app_rx_data),
        .rx_msg_last               (app_rx_last),
        .rx_msg_ready              (app_rx_ready),
        .rx_msg_len                (app_rx_len),
        .rx_msg_sn                 (app_rx_sn),
        .rx_msg_net                (app_rx_net),
        .rx_msg_src_ip             (app_rx_src_ip),
        .rx_msg_sport              (app_rx_sport),
        .rx_msg_dport              (app_rx_dport),
        .rx_msg_ts                 (app_rx_ts),
        .ts_us                     (rtc64[31:0]),
        .tx_sof_tag_a              (sof_tag_a),
        .tx_sof_tag_b              (sof_tag_b),
        .cfg_m_rd_idx              (cfg_m_rd_idx),
        .cfg_m_rd_shadow           (cfg_m_rd_shadow),
        .cfg_m_bag_us              (cfg_m_bag_us),
        .cfg_m_lmax                (cfg_m_lmax),
        .cfg_m_net_en              (cfg_m_net_en),
        .cfg_m_vlid                (cfg_m_vlid),
        .cfg_m_rom_lmax            (cfg_m_rom_lmax),
        .cfg_m_wr_en               (cfg_m_wr_en),
        .cfg_m_wr_idx              (cfg_m_wr_idx),
        .cfg_m_wr_bag_us           (cfg_m_wr_bag_us),
        .cfg_m_wr_lmax             (cfg_m_wr_lmax),
        .cfg_m_wr_net_en           (cfg_m_wr_net_en),
        .cfg_cmd_copy              (cfg_cmd_copy),
        .cfg_cmd_verify            (cfg_cmd_verify),
        .cfg_cmd_apply             (cfg_cmd_apply),
        .cfg_cmd_rollback          (cfg_cmd_rollback),
        .cfg_busy                  (cfg_busy),
        .cfg_active_bank           (cfg_active_bank),
        .cfg_prev_valid            (cfg_prev_valid),
        .cfg_done                  (cfg_done),
        .cfg_verify_ok             (cfg_verify_ok),
        .cfg_verify_code           (cfg_verify_code),
        .cfg_verify_idx            (cfg_verify_idx),
        .cfg_budget                (cfg_budget),
        .cnt_rx_mac_error          (cnt_rx_mac_error),
        .mac_a_tx_last             (ext_a_tx_last),
        .mac_a_wire_sof            (ext_a_wire_sof),
        .mac_a_wire_done           (ext_a_wire_done),
        .mac_a_rx_last             (ext_a_rx_last),
        .mac_a_rx_ready            (ext_a_rx_ready),
        .mac_a_rx_ts               (ext_a_rx_ts),
        .mac_a_rx_good_count       (ext_a_rx_good_count),
        .mac_a_rx_error_count      (ext_a_rx_error_count),
        .mac_a_rx_length_count     (ext_a_rx_length_count),
        .mac_a_rx_overflow_count   (ext_a_rx_overflow_count),
        .mac_b_tx_last             (ext_b_tx_last),
        .mac_b_wire_sof            (ext_b_wire_sof),
        .mac_b_wire_done           (ext_b_wire_done),
        .mac_b_rx_last             (ext_b_rx_last),
        .mac_b_rx_ready            (ext_b_rx_ready),
        .mac_b_rx_ts               (ext_b_rx_ts),
        .mac_b_rx_good_count       (ext_b_rx_good_count),
        .mac_b_rx_error_count      (ext_b_rx_error_count),
        .mac_b_rx_length_count     (ext_b_rx_length_count),
        .mac_b_rx_overflow_count   (ext_b_rx_overflow_count),
        .mac_a_tx_data             (a_tx_data),
        .mac_a_tx_valid            (a_tx_valid),
        .mac_a_tx_slot             (a_tx_slot),
        .mac_a_tx_frame            (a_tx_frame),
        .mac_a_tx_busy             (a_tx_busy),
        .mac_a_rx_data             ((EXTERNAL_MAC != 0) ? ext_a_rx_data : rx_data_q),
        .mac_a_rx_valid            ((EXTERNAL_MAC != 0) ? ext_a_rx_valid : rx_valid_q),
        .mac_a_rx_frame            (rx_ready_q),
        .mac_b_tx_data             (b_tx_data),
        .mac_b_tx_valid            (b_tx_valid),
        .mac_b_tx_slot             (b_tx_slot),
        .mac_b_tx_frame            (b_tx_frame),
        .mac_b_tx_busy             (b_tx_busy),
        .mac_b_rx_data             ((EXTERNAL_MAC != 0) ? ext_b_rx_data : 8'h00),
        .mac_b_rx_valid            ((EXTERNAL_MAC != 0) ? ext_b_rx_valid : 1'b0),
        .mac_b_rx_frame            (1'b0),
        .rtc_load                  (1'b0),
        .rtc_load_val              (32'd0),
        .rtc_us                    (core_rtc_us),
        .tick_us                   (tick_us),
        .mono_us                   (mono_us),
        .mgmt_window               (mgmt_window),
        .cnt_tx_msgs               (cnt_tx_msgs),
        .cnt_tx_frames             (cnt_tx_frames),
        .cnt_tx_frames_a           (cnt_tx_frames_a),
        .cnt_tx_frames_b           (cnt_tx_frames_b),
        .cnt_tx_lmax_drop          (cnt_tx_lmax_drop),
        .cnt_tx_ovf_drop           (cnt_tx_ovf_drop),
        .cnt_rx_frames_a           (cnt_rx_frames_a),
        .cnt_rx_frames_b           (cnt_rx_frames_b),
        .cnt_rx_fcs_err            (cnt_rx_fcs_err),
        .cnt_rx_mac_len_err        (cnt_rx_mac_len_err),
        .cnt_rx_mac_ovf            (cnt_rx_mac_ovf),
        .cnt_rx_hdr_err            (cnt_rx_hdr_err),
        .cnt_rx_len_err            (cnt_rx_len_err),
        .cnt_rx_vl_unknown         (cnt_rx_vl_unknown),
        .cnt_rx_csum_err           (cnt_rx_csum_err),
        .cnt_rx_full_drop          (cnt_rx_full_drop),
        .cnt_rx_ic_drop            (cnt_rx_ic_drop),
        .cnt_rx_rm_drop            (cnt_rx_rm_drop),
        .cnt_rx_accepted           (cnt_rx_accepted),
        .cnt_rx_delivered          (cnt_rx_delivered),
        .tx_buffer_used            (tx_buffer_used),
        .tx_buffer_peak            (tx_buffer_peak),
        .rx_buffer_used            (rx_buffer_used),
        .rx_buffer_peak            (rx_buffer_peak),
        .tx_buffer_messages        (tx_buffer_messages),
        .tx_buffer_messages_peak   (tx_buffer_messages_peak),
        .rx_buffer_messages        (rx_buffer_messages),
        .rx_buffer_messages_peak   (rx_buffer_messages_peak)
    );

    // 100 Mbps 字节时隙发生器：每 BUS_BYTE_CYCLES 个周期允许送出一个字节
    localparam PW = afdx_clog2(BUS_BYTE_CYCLES);
    reg [PW-1:0] pace_cnt;
    // Predict count==0 for the next cycle. All three copies keep exactly the
    // original phase, including reset and BUS_BYTE_CYCLES=1. Preserve local
    // sources for bus sampling and the two MACs instead of a shared decode.
    (* preserve, dont_merge *) reg bus_slot, bus_slot_a, bus_slot_b;
    always @(posedge clk) begin
        if (rst_t[0]) begin
            pace_cnt <= {PW{1'b0}};
            bus_slot <= 1'b1; bus_slot_a <= 1'b1; bus_slot_b <= 1'b1;
        end else begin
            pace_cnt <= (pace_cnt == BUS_BYTE_CYCLES - 1) ? {PW{1'b0}} : pace_cnt + 1'b1;
            bus_slot <= (pace_cnt == BUS_BYTE_CYCLES - 1);
            bus_slot_a <= (pace_cnt == BUS_BYTE_CYCLES - 1);
            bus_slot_b <= (pace_cnt == BUS_BYTE_CYCLES - 1);
        end
    end

    // 发送总线复用：以帧为单位在 A/B 两路 MAC 发送之间切换（B 路仅在 DUAL_NET_ON_BUS=1 时上总线）。
    // MAC 从选定帧（离开 S_IDLE，至多比 BAG 截止时刻早几拍）起即占用总线，使首字节能在截止时刻
    // 所在的字节时隙发出；若等到 tx_valid 再占用，每帧会晚一个时隙，周期与 BAG 相等的 VL 会逐周期累积。
    reg  bus_lock, bus_sel, bus_rr;
    wire b_on_bus = (DUAL_NET_ON_BUS != 0);
    always @(posedge clk) begin
        if (rst_t[0]) begin
            bus_lock <= 1'b0; bus_sel <= 1'b0; bus_rr <= 1'b0;
        end else if (!bus_lock) begin
            if (a_tx_busy && (!b_tx_busy || !b_on_bus || !bus_rr)) begin bus_lock <= 1'b1; bus_sel <= 1'b0; end
            else if (b_tx_busy && b_on_bus) begin bus_lock <= 1'b1; bus_sel <= 1'b1; end
        end else if (bus_sel ? !b_tx_busy : !a_tx_busy) begin
            bus_lock <= 1'b0;               // 该路完成发送（含帧间隔）后释放总线
            bus_rr <= ~bus_sel;
        end
    end
    // tx_valid 仅在字节时隙脉冲时为高一个周期（MAC 每时隙采样一个字节）
    reg [7:0] tx_data_q;
    reg tx_valid_q, tx_ready_q;
    always @(posedge clk) begin
        tx_data_q  <= bus_sel ? b_tx_data : a_tx_data;
        tx_valid_q <= bus_lock && bus_slot && (bus_sel ? b_tx_valid : a_tx_valid);
        tx_ready_q <= bus_lock && (bus_sel ? b_tx_frame : a_tx_frame);
    end
    assign tx_data  = tx_data_q;
    assign tx_valid = tx_valid_q;
    assign tx_ready = tx_ready_q;
    assign a_tx_slot = (EXTERNAL_MAC != 0) ? ext_a_tx_ready : (bus_lock & ~bus_sel & bus_slot_a);
    assign b_tx_slot = (EXTERNAL_MAC != 0) ? ext_b_tx_ready : (b_on_bus ? (bus_lock & bus_sel & bus_slot_b) : 1'b1);   // 未上总线时 B 路直接吞掉
endmodule
