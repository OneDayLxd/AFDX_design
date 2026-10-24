/*
 * ================================================================
 * File Name    : afdx_es_core.v
 * Module Name  : afdx_es_core
 *
 * Function:
 *   AFDX 端系统核心：发送路径（VL 队列 -> BAG 调度 -> 帧组装 -> 冗余复制 -> 双网 MAC 发送）
 *                   接收路径（双网 MAC -> 并行校验/暂存 -> 共享完整性/冗余裁决 -> VL 接收队列 -> 交付）
 *   EXTERNAL_MAC=0 保留历史字节总线的串行接收路径；正式双 GMII MAC 使用并行接收路径。
 *   对外提供：应用消息收发接口、网络 A/B 两路 8 位 MAC 总线、统计计数器。
 *   管理扩展：发送消息可覆盖 UDP 端点并请求 SOF 时间戳；接收消息附带源 IP/端口与接收时间戳；
 *             发送 VL 的 BAG/Lmax/网络使能来自运行时双银行配置 RAM（afdx_tx_cfg_ram）。
 *
 * Sub Module:
 *   afdx_timer
 *   afdx_tx_vl_rom
 *   afdx_msg_queue
 *   afdx_tx_cfg_ram
 *   afdx_bag_scheduler
 *   afdx_frame_builder
 *   afdx_redundancy_ctrl
 *   afdx_mac_tx
 *   afdx_mac_rx
 *   afdx_rx_arb
 *   afdx_rx_vl_lookup
 *   afdx_rx_parser
 *   afdx_rx_integrity
 *   afdx_rx_parallel
 *   afdx_rx_deliver
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_es_core #(
    parameter CLK_HZ       = 100_000_000,
    parameter EXTERNAL_MAC = 0,            // 1: 无 FCS 帧接口，连接核心外的 MAC
    parameter N_TX_VL      = 8,
    parameter TX_VLW       = 3,
    parameter N_RX_VL      = 8,
    parameter RX_VLW       = 3,
    parameter TX_DEPTH_LOG = 2,                // 每发送 VL 槽位数 = 2^TX_DEPTH_LOG
    parameter RX_DEPTH_LOG = 2,
    parameter TX_POOL_LOG  = 5,                // 共享发送槽总数 = 2^TX_POOL_LOG
    parameter RX_POOL_LOG  = 5,                // 共享接收槽总数，与接收 VL 数量独立
    parameter SLOT_AW      = 11,               // 槽位大小 2^SLOT_AW 字节
    parameter BLOCK_AW     = (SLOT_AW < 8) ? SLOT_AW : 8,
    parameter TX_MAX_MSGS  = 128,              // 描述符信用限制；字节容量仍各 64 KiB
    parameter RX_MAX_MSGS  = 1 << (RX_POOL_LOG + SLOT_AW - BLOCK_AW),
    parameter [7:0] NET_ID = 8'd1,
    parameter [7:0] EQ_ID  = 8'd1,
    parameter SKEW_MAX_US  = 10000,
    parameter UDP_CSUM_EN  = 1,
    parameter IC_EN        = 1,
    parameter RM_EN        = 1,
    parameter NET_FROM_SA  = 0,
    parameter IFG          = 12,
    parameter MAC_FIFO_AW  = 12,
    parameter RX_GAP_CYCLES = 32,
    parameter TX_SOF_FEEDBACK = 1,           // 每 VL 至多预组装一份尚未开始出线的消息
    parameter TX_PHASE_ENABLE = 0,           // 有相位约束的低抖动业务；0 保留原轮询策略
    parameter RX_SESSION_META = 1,           // 1: 接收描述符保存 UDP 会话元数据
    parameter CFG_SHARED_BUS = 1,            // 运行时配置带宽校验：A/B 副本共用一条总线
    // 管理窗口：入口仅在 [START_US, START_US+OPEN_US) 开始管理消息；
    // 已排队的管理 VL 每窗口预约一次发送，预约可等待 BAG，但不累积。
    parameter MGMT_WIN_VL = 0,
    parameter [19:0] MGMT_WIN_PERIOD_US = 20'd0,
    parameter [19:0] MGMT_WIN_START_US = 20'd0,
    parameter [19:0] MGMT_WIN_OPEN_US = 20'd0
) (
    input  wire               clk,
    input  wire               rst,
    // 应用发送接口（消息 = UDP 有效载荷）
    input  wire [TX_VLW-1:0]  tx_msg_vl,
    input  wire               tx_msg_valid,
    input  wire [7:0]         tx_msg_data,
    input  wire               tx_msg_last,
    output wire               tx_msg_ready,
    input  wire [33:0]        tx_msg_ep,           // {SOF 时间戳标记, 端点覆盖, UDP 源端口, UDP 目的端口}，末字节采样
    // 应用接收接口
    output wire [RX_VLW-1:0]  rx_msg_vl,
    output wire               rx_msg_valid,
    output wire [7:0]         rx_msg_data,
    output wire               rx_msg_last,
    input  wire               rx_msg_ready,
    output wire [SLOT_AW-1:0] rx_msg_len,
    output wire [7:0]         rx_msg_sn,
    output wire               rx_msg_net,
    output wire [31:0]        rx_msg_src_ip,
    output wire [15:0]        rx_msg_sport,
    output wire [15:0]        rx_msg_dport,
    output wire [31:0]        rx_msg_ts,           // 默认取解析起点；外部 MAC 模式使用随帧传入的时间戳
    input  wire [31:0]        ts_us,               // 接收时间戳时基（RTC 低 32 位）
    output wire               tx_sof_tag_a,        // 带标记帧在网络 A/B 的 SOF 脉冲
    output wire               tx_sof_tag_b,
    // 运行时发送配置（管理面）
    input  wire [TX_VLW-1:0]  cfg_m_rd_idx,
    input  wire               cfg_m_rd_shadow,
    output wire [19:0]        cfg_m_bag_us,
    output wire [10:0]        cfg_m_lmax,
    output wire [1:0]         cfg_m_net_en,
    output wire [15:0]        cfg_m_vlid,
    output wire [10:0]        cfg_m_rom_lmax,
    input  wire               cfg_m_wr_en,
    input  wire [TX_VLW-1:0]  cfg_m_wr_idx,
    input  wire [19:0]        cfg_m_wr_bag_us,
    input  wire [10:0]        cfg_m_wr_lmax,
    input  wire [1:0]         cfg_m_wr_net_en,
    input  wire               cfg_cmd_copy,
    input  wire               cfg_cmd_verify,
    input  wire               cfg_cmd_apply,
    input  wire               cfg_cmd_rollback,
    output wire               cfg_busy,
    output wire               cfg_active_bank,
    output wire               cfg_prev_valid,
    output wire               cfg_done,
    output wire               cfg_verify_ok,
    output wire [2:0]         cfg_verify_code,
    output wire [TX_VLW-1:0]  cfg_verify_idx,
    output wire [31:0]        cfg_budget,
    // 网络 A MAC 总线
    output wire [7:0]         mac_a_tx_data,
    output wire               mac_a_tx_valid,
    input  wire               mac_a_tx_slot,        // 字节时隙使能 / ready
    output wire               mac_a_tx_frame,       // 帧包络
    output wire               mac_a_tx_busy,
    input  wire [7:0]         mac_a_rx_data,
    input  wire               mac_a_rx_valid,
    input  wire               mac_a_rx_frame,       // 帧包络
    // 网络 B MAC 总线
    output wire [7:0]         mac_b_tx_data,
    output wire               mac_b_tx_valid,
    input  wire               mac_b_tx_slot,        // 字节时隙使能 / ready
    output wire               mac_b_tx_frame,       // 帧包络
    output wire               mac_b_tx_busy,
    input  wire [7:0]         mac_b_rx_data,
    input  wire               mac_b_rx_valid,
    input  wire               mac_b_rx_frame,       // 帧包络
    output wire               mac_a_tx_last,
    input  wire               mac_a_wire_sof,
    input  wire               mac_a_wire_done,
    input  wire               mac_a_rx_last,
    output wire               mac_a_rx_ready,
    input  wire [31:0]         mac_a_rx_ts,
    input  wire [31:0]         mac_a_rx_good_count,
    input  wire [31:0]         mac_a_rx_error_count,
    input  wire [31:0]         mac_a_rx_length_count,
    input  wire [31:0]         mac_a_rx_overflow_count,
    output wire               mac_b_tx_last,
    input  wire               mac_b_wire_sof,
    input  wire               mac_b_wire_done,
    input  wire               mac_b_rx_last,
    output wire               mac_b_rx_ready,
    input  wire [31:0]         mac_b_rx_ts,
    input  wire [31:0]         mac_b_rx_good_count,
    input  wire [31:0]         mac_b_rx_error_count,
    input  wire [31:0]         mac_b_rx_length_count,
    input  wire [31:0]         mac_b_rx_overflow_count,
    output wire [31:0]        cnt_rx_mac_error,      // 外部 MAC 路径综合帧错误，不能仅解释为 FCS
    // RTC
    input  wire               rtc_load,
    input  wire [31:0]        rtc_load_val,
    output wire [31:0]        rtc_us,
    output wire               tick_us,
    output wire [31:0]        mono_us,             // 单调微秒计数（不受 RTC 校时影响）
    output wire               mgmt_window,         // 管理发送窗口开启（未配置窗口时恒为 1）
    // 统计计数器（网络管理）
    output reg  [31:0]        cnt_tx_msgs,           // 应用提交的消息数
    output reg  [31:0]        cnt_tx_frames,         // 组装的帧数
    output reg  [31:0]        cnt_tx_frames_a,
    output reg  [31:0]        cnt_tx_frames_b,
    output reg  [31:0]        cnt_tx_lmax_drop,
    output reg  [31:0]        cnt_tx_ovf_drop,       // 消息超过槽位被丢弃
    output reg  [31:0]        cnt_rx_frames_a,       // MAC 层正确接收（FCS 正确）
    output reg  [31:0]        cnt_rx_frames_b,
    output reg  [31:0]        cnt_rx_fcs_err,
    output reg  [31:0]        cnt_rx_mac_len_err,
    output reg  [31:0]        cnt_rx_mac_ovf,
    output reg  [31:0]        cnt_rx_hdr_err,
    output reg  [31:0]        cnt_rx_len_err,
    output reg  [31:0]        cnt_rx_vl_unknown,
    output reg  [31:0]        cnt_rx_csum_err,
    output reg  [31:0]        cnt_rx_full_drop,
    output reg  [31:0]        cnt_rx_ic_drop,
    output reg  [31:0]        cnt_rx_rm_drop,
    output reg  [31:0]        cnt_rx_accepted,       // 通过全部检查进入接收队列
    output reg  [31:0]        cnt_rx_delivered,      // 交付给应用
    output wire [TX_POOL_LOG+SLOT_AW-BLOCK_AW:0] tx_buffer_used,
    output wire [TX_POOL_LOG+SLOT_AW-BLOCK_AW:0] tx_buffer_peak,
    output wire [RX_POOL_LOG+SLOT_AW-BLOCK_AW:0] rx_buffer_used,
    output wire [RX_POOL_LOG+SLOT_AW-BLOCK_AW:0] rx_buffer_peak,
    output wire [TX_POOL_LOG+SLOT_AW-BLOCK_AW:0] tx_buffer_messages,
    output wire [TX_POOL_LOG+SLOT_AW-BLOCK_AW:0] tx_buffer_messages_peak,
    output wire [RX_POOL_LOG+SLOT_AW-BLOCK_AW:0] rx_buffer_messages,
    output wire [RX_POOL_LOG+SLOT_AW-BLOCK_AW:0] rx_buffer_messages_peak
);
    localparam MW = (RX_SESSION_META != 0) ? 105 : 9;
    localparam TMW = 66;                        // 发送描述符元数据 {ep[33:0], UDP 载荷和}

    // 接收交付在模块内部实现同深度复位副本；其余子模块在此寄存一拍。
    // 本地复位：输入复位再寄存一拍，每个子模块一位独立寄存器（preserve 不合并），负载限于该模块的
    // 布局区域；maxfan 使 eLinx 只在模块内部复制，不再由全局复位的任意分组副本远距离驱动
    (* preserve, maxfan = 64 *) reg [14:0] rst_l;
    always @(posedge clk) rst_l <= {15{rst}};
    wire rst_c = rst_l[0];

    // ---------------- 时基 ----------------
    wire [31:0] monotonic_us;
    assign mono_us = monotonic_us;
    afdx_timer #(
        .CLK_HZ       (CLK_HZ)
    ) u_timer (
        .clk          (clk),
        .rst          (rst_l[1]),
        .rtc_load     (rtc_load),
        .rtc_load_val (rtc_load_val),
        .tick_us      (tick_us),
        .rtc_us       (rtc_us),
        .monotonic_us (monotonic_us)
    );

    // ---------------- 发送队列 ----------------
    wire [N_TX_VL-1:0] txq_nonempty;
    wire               txq_ovf;
    wire [TX_VLW-1:0]  txq_rd_vl;
    wire               txq_rd_start, txq_rd_pop, txq_rd_valid, txq_rd_last, txq_rd_ready;
    wire txq_rd_info_valid, txq_rd_pop_ready;
    wire [SLOT_AW-1:0] txq_rd_len;
    wire [7:0]         txq_rd_data;
    wire [TMW-1:0]     txq_rd_meta;
    // 入口 Lmax 检查：应用提交消息时逐字节计数，最后一拍若长度超过该 VL 的 Lmax（或 1471）则整条丢弃
    // Two-entry input slice as a circular buffer. Every per-byte decision
    // (Lmax/slot limit, VL rejection, drop state, UDP payload sum) is made as
    // a byte enters (tx_put) and stored with it; the bytes leave in the same
    // order, so the result equals deciding them on the way out. An entry is
    // loaded only on tx_put, and taking the head (which depends on the
    // queue's wr_ready) only flips the read pointer and the valid flags.
    localparam TIW = 34 + TX_VLW + 1 + 8 + 1 + 1 + 32;
    reg [TIW-1:0] tx_e0, tx_e1;
    reg tx_e0_same, tx_e1_same;                 // 该项与前一写入项的 VL 相同
    reg tx_wsel, tx_rsel;
    reg tx_hsel;                                // 输出选择 = 有效时 tx_rsel，空时最近取走的一项
    reg tx_head_valid, tx_spare_valid;
    // 缓冲空时输出最近取走的一项（与原保持寄存器相同），队列在空闲期按该 VL 预取准入状态
    wire [TIW-1:0] tx_head = tx_hsel ? tx_e1 : tx_e0;
    wire [TX_VLW-1:0] txh_vl;
    wire txh_last, txh_bad, txh_abort;
    wire [7:0] txh_data;
    wire [33:0] txh_ep;
    wire [31:0] txh_sum;
    assign {txh_ep, txh_vl, txh_last, txh_data, txh_bad, txh_abort, txh_sum} = tx_head;
    assign tx_msg_ready = !tx_spare_valid;
    wire tx_put = tx_msg_valid && tx_msg_ready;
    wire tx_take;

    // 入口判定（按写入顺序）：tx_in_cnt 为本字节在消息中的序号；上一字节的计数与 Lmax 寄存后
    // 在本字节写入时比较，Lmax ROM 输出不经加法比较链。
    wire [10:0] entry_lmax;
    wire entry_reject = (entry_lmax == 0) || (tx_msg_vl >= N_TX_VL);
    afdx_tx_vl_rom #(
        .VLW          (TX_VLW)
    ) u_txrom_in (
        .idx       (tx_msg_vl),
        .vlid      (),
        .bag_us    (),
        .lmax      (entry_lmax),
        .udp_src   (),
        .udp_dst   (),
        .net_en    (),
        .partition ()
    );
    reg  [10:0] tx_in_cnt;
    reg  [10:0] tx_prev_lmax;
    reg         tx_prev_last;                   // 上一字节为消息末字节（复位后为 1）
    reg         tx_dropping;
    reg  [31:0] tx_payload_sum;
    reg  [TX_VLW-1:0] tx_prev_vl;
    // 上一字节序号 = Lmax-1、1470 或槽位上限-2 时本字节超限（原判定 cnt == Lmax-1 等价于 cnt+1 == Lmax）
    wire put_limit = !tx_prev_last && ((tx_in_cnt == tx_prev_lmax) || (tx_in_cnt == 11'd1471) ||
                                       (tx_in_cnt == ((1 << SLOT_AW) - 1)));
    wire put_over  = put_limit || entry_reject;
    wire put_bad   = tx_dropping || put_over;
    wire put_abort = put_over && !tx_dropping;  // 首个越限字节：通知队列回滚
    // 丢弃中的消息不会提交，其载荷和无需保持，故累加不受 put_bad 控制
    wire [31:0] put_sum = tx_payload_sum +
                          (tx_in_cnt[0] ? {24'd0, tx_msg_data} : {16'd0, tx_msg_data, 8'd0});
    wire put_same = tx_msg_vl == tx_prev_vl;
    wire [TIW-1:0] tx_entry = {tx_msg_ep, tx_msg_vl, tx_msg_last, tx_msg_data, put_bad, put_abort, put_sum};
    always @(posedge clk) begin
        if (tx_put) begin
            if (tx_wsel) begin tx_e1 <= tx_entry; tx_e1_same <= put_same; end
            else begin tx_e0 <= tx_entry; tx_e0_same <= put_same; end
            tx_prev_vl <= tx_msg_vl;
            tx_prev_lmax <= entry_lmax;
        end
    end
    always @(posedge clk) begin
        if (rst_c) begin
            tx_in_cnt <= 0; tx_prev_last <= 1'b1; tx_dropping <= 0; tx_payload_sum <= 0;
        end else if (tx_put) begin
            tx_prev_last <= tx_msg_last;
            if (tx_msg_last) begin
                tx_in_cnt <= 0; tx_dropping <= 0; tx_payload_sum <= 0;
            end else begin
                tx_in_cnt <= tx_in_cnt + 11'd1;
                if (put_bad) tx_dropping <= 1'b1;
                if (UDP_CSUM_EN != 0) tx_payload_sum <= put_sum;
            end
        end
    end
    // txh_vl_same: head VL equals its value one clock ago (EXT_VL_SAME of the
    // queue). A new head is the entry written after the old one, so its
    // stored same flag (or put_same when it arrives while taking) applies.
    reg txh_vl_same;
    always @(posedge clk) begin
        txh_vl_same <= 1'b1;
        if (rst_c) begin
            tx_head_valid <= 0; tx_spare_valid <= 0; txh_vl_same <= 0; tx_wsel <= 0; tx_rsel <= 0;
            tx_hsel <= 1'b1;
        end else begin
            if (tx_put) tx_wsel <= ~tx_wsel;
            if (tx_take) begin
                tx_rsel <= ~tx_rsel;
                tx_hsel <= (tx_spare_valid || tx_put) ? ~tx_rsel : tx_rsel;
                if (tx_spare_valid) begin
                    tx_spare_valid <= 0; txh_vl_same <= tx_rsel ? tx_e0_same : tx_e1_same;
                end else if (tx_put) txh_vl_same <= put_same;
                else tx_head_valid <= 0;
            end else if (tx_put) begin
                if (!tx_head_valid) begin tx_head_valid <= 1; txh_vl_same <= 0; tx_hsel <= tx_rsel; end
                else tx_spare_valid <= 1;
            end
        end
    end

    wire               txq_wr_ready;
    wire               txq_commit_ok;
    reg                txq_commit_is_mgmt;
    // 与队列内部提交 VL 同拍采样，只在真实提交成功时使用。
    always @(posedge clk) txq_commit_is_mgmt <= txh_vl == MGMT_WIN_VL;
    wire               tx_bad = txh_bad;
    // 首次超限立即回滚，继续接收并丢弃到 last，避免槽位溢出后永远拉低 ready。
    assign tx_take = tx_head_valid && (tx_bad || txq_wr_ready);
    wire               tx_in_beat  = tx_take;
    wire tx_msg_abort  = tx_in_beat & txh_last & tx_bad;
    // abort is stored only on a bad byte, which tx_take always accepts.
    wire txq_abort    = tx_head_valid & txh_abort;
    wire [31:0] tx_sum_next = txh_sum;

    afdx_msg_queue #(
        .N_VL         (N_TX_VL),
        .VLW          (TX_VLW),
        .DEPTH            (1 << TX_DEPTH_LOG),
        .DW               (TX_DEPTH_LOG),
        .SLOT_AW          (SLOT_AW),
        .MW               (TMW),
        .POOL_AW          (TX_POOL_LOG),
        .BLOCK_AW         (BLOCK_AW),
        .RD_INFO_UNTAGGED (1),
        .EXT_VL_SAME      (1),
        .MAX_MSGS         ((TX_MAX_MSGS < (1 << (TX_POOL_LOG+SLOT_AW-BLOCK_AW))) ? TX_MAX_MSGS : (1 << (TX_POOL_LOG+SLOT_AW-BLOCK_AW)))
    ) u_txq (
        .clk            (clk),
        .rst            (rst_l[2]),
        .wr_vl          (txh_vl),
        .wr_vl_same     (txh_vl_same),
        .wr_valid       (tx_head_valid & ~tx_bad),
        .wr_data        (txh_data),
        .wr_ready       (txq_wr_ready),
        .wr_commit      (tx_head_valid && txh_last && !tx_bad),
        .wr_meta        ({txh_ep, tx_sum_next}),
        .wr_abort       (txq_abort),
        .wr_ovf         (txq_ovf),
        .wr_commit_ok   (txq_commit_ok),
        .rd_vl          (txq_rd_vl),
        .rd_start       (txq_rd_start),
        .rd_pop         (txq_rd_pop),
        .rd_info_valid  (txq_rd_info_valid),
        .rd_pop_ready   (txq_rd_pop_ready),
        .reclaim_busy   (),
        .rd_len         (txq_rd_len),
        .rd_meta        (txq_rd_meta),
        .rd_data        (txq_rd_data),
        .rd_valid       (txq_rd_valid),
        .rd_last        (txq_rd_last),
        .rd_ready       (txq_rd_ready),
        .nonempty       (txq_nonempty),
        .pool_used      (tx_buffer_used),
        .pool_peak      (tx_buffer_peak),
        .pool_msgs      (tx_buffer_messages),
        .pool_msgs_peak (tx_buffer_messages_peak),
        .ne_group       ({(TX_VLW>4 ? TX_VLW-4 : 1){1'b0}}),
        .ne_bits        (),
        .init_busy      (),
        .ne_regions     ()
    );

    // ---------------- VL 配置 ROM ----------------
    wire [TX_VLW-1:0] cfg_idx;
    wire [15:0] cfg_vlid, cfg_udp_src, cfg_udp_dst;
    wire [19:0] cfg_bag_us;
    wire [10:0] cfg_lmax;
    wire [1:0]  cfg_net_en;
    wire [4:0]  cfg_partition;
    wire        grant_valid, grant_ack;
    wire [TX_VLW-1:0] grant_vl;
    wire [1:0] grant_net_en;
    wire [N_TX_VL-1:0] mac_a_bag_open, mac_b_bag_open;
    wire mac_a_sof, mac_b_sof, mac_a_bag_wait, mac_b_bag_wait;
    wire [TX_VLW-1:0] mac_a_sof_vl, mac_b_sof_vl;
    wire stat_lmax_drop;
    reg [N_TX_VL-1:0] tx_pending_a, tx_pending_b;
    wire [N_TX_VL-1:0] tx_egress_ready = ~tx_pending_a & ~tx_pending_b;
    // 外部 MAC 仅留一份整帧 CDC 存储；BAG 未开时不提前组帧，避免阻塞其它 VL。
    wire [N_TX_VL-1:0] tx_wire_eligible = (EXTERNAL_MAC != 0) ?
        (mac_a_bag_open & mac_b_bag_open) : {N_TX_VL{1'b1}};
    // 从授权到实际 SOF 保留逐网络信用，防止在首帧尚未出线时再次授权同一 VL。
    // 单网 VL 在另一网从不占信用。旧模式允许提前组帧后在各出口排队；
    // 外部模式按 tx_wire_eligible 筛选，BAG 打开后才组帧，以省去出口帧副本。
    // Credit events are applied one clock later from registered group/lane
    // one-hot decodes, so no VL register drives 2x128 update decoders. The
    // scheduler re-arbitrates only several clocks after an ack.
    localparam TX_GROUPS = (N_TX_VL + 15) / 16;
    reg [TX_GROUPS-1:0] ev_grant_group, ev_drop_group, ev_sof_a_group, ev_sof_b_group;
    reg [15:0] ev_grant_lane, ev_drop_lane, ev_sof_a_lane, ev_sof_b_lane;
    reg [1:0] ev_grant_net;
    genvar tv;
    generate for (tv = 0; tv < TX_GROUPS; tv = tv + 1) begin : g_credit_group
        always @(posedge clk) begin
            ev_grant_group[tv] <= !rst_c && grant_ack && grant_vl / 16 == tv;
            ev_drop_group[tv] <= !rst_c && stat_lmax_drop && cfg_idx / 16 == tv;
            ev_sof_a_group[tv] <= !rst_c && mac_a_sof && mac_a_sof_vl / 16 == tv;
            ev_sof_b_group[tv] <= !rst_c && mac_b_sof && mac_b_sof_vl / 16 == tv;
        end
    end endgenerate
    always @(posedge clk) begin
        ev_grant_lane <= 16'd1 << (grant_vl % 16);
        ev_drop_lane <= 16'd1 << (cfg_idx % 16);
        ev_sof_a_lane <= 16'd1 << (mac_a_sof_vl % 16);
        ev_sof_b_lane <= 16'd1 << (mac_b_sof_vl % 16);
        ev_grant_net <= grant_net_en;
    end
    generate for (tv = 0; tv < N_TX_VL; tv = tv + 1) begin : g_credit
        wire drop_hit = ev_drop_group[tv/16] && ev_drop_lane[tv%16];
        wire grant_hit = ev_grant_group[tv/16] && ev_grant_lane[tv%16];
        always @(posedge clk) begin
            if (rst_c) begin tx_pending_a[tv] <= 1'b0; tx_pending_b[tv] <= 1'b0; end
            else begin
                if (ev_sof_a_group[tv/16] && ev_sof_a_lane[tv%16]) tx_pending_a[tv] <= 1'b0;
                if (ev_sof_b_group[tv/16] && ev_sof_b_lane[tv%16]) tx_pending_b[tv] <= 1'b0;
                if (drop_hit) begin tx_pending_a[tv] <= 1'b0; tx_pending_b[tv] <= 1'b0; end
                if (grant_hit) begin
                    tx_pending_a[tv] <= ev_grant_net[0];
                    tx_pending_b[tv] <= ev_grant_net[1];
                end
            end
        end
    end endgenerate
    // 静态字段（VLID/端点/分区）来自 ROM；BAG/Lmax/网络使能来自运行时配置 RAM。
    // 授权读口按 grant_vl、组帧读口按 cfg_idx 寻址，读出延迟 2 拍：帧组装器在授权后
    // 多等一拍再确认，并在 S_LOAD 至少两拍后锁存，读出均已稳定。
    wire [19:0] rom_bag_us;
    wire fb_idle, cfg_hold;
    afdx_tx_vl_rom #(
        .VLW          (TX_VLW)
    ) u_txrom (
        .idx       (cfg_idx),
        .vlid      (cfg_vlid),
        .bag_us    (),
        .lmax      (),
        .udp_src   (cfg_udp_src),
        .udp_dst   (cfg_udp_dst),
        .net_en    (),
        .partition (cfg_partition)
    );
    afdx_tx_cfg_ram #(
        .N_VL         (N_TX_VL),
        .VLW          (TX_VLW),
        .SHARED_BUS (CFG_SHARED_BUS),
        .BUS_BUDGET ((EXTERNAL_MAC != 0) ? 32'd16000000 : 32'd1600000)
    ) u_txcfg (
        .clk          (clk),
        .rst          (rst_l[3]),
        .g_idx        (grant_vl),
        .g_bag_us     (cfg_bag_us),
        .g_net_en     (grant_net_en),
        .f_idx        (cfg_idx),
        .f_bag_us     (rom_bag_us),
        .f_lmax       (cfg_lmax),
        .f_net_en     (cfg_net_en),
        .hold         (cfg_hold),
        .builder_idle (fb_idle),
        .m_rd_idx     (cfg_m_rd_idx),
        .m_rd_shadow  (cfg_m_rd_shadow),
        .m_bag_us     (cfg_m_bag_us),
        .m_lmax       (cfg_m_lmax),
        .m_net_en     (cfg_m_net_en),
        .m_vlid       (cfg_m_vlid),
        .m_rom_lmax   (cfg_m_rom_lmax),
        .m_wr_en      (cfg_m_wr_en),
        .m_wr_idx     (cfg_m_wr_idx),
        .m_wr_bag_us  (cfg_m_wr_bag_us),
        .m_wr_lmax    (cfg_m_wr_lmax),
        .m_wr_net_en  (cfg_m_wr_net_en),
        .cmd_copy     (cfg_cmd_copy),
        .cmd_verify   (cfg_cmd_verify),
        .cmd_apply    (cfg_cmd_apply),
        .cmd_rollback (cfg_cmd_rollback),
        .busy         (cfg_busy),
        .init_busy    (),
        .active_bank  (cfg_active_bank),
        .prev_valid   (cfg_prev_valid),
        .done         (cfg_done),
        .verify_ok    (cfg_verify_ok),
        .verify_code  (cfg_verify_code),
        .verify_idx   (cfg_verify_idx),
        .budget       (cfg_budget)
    );

    // ---------------- BAG 调度 ----------------
    // The frame builder is still in S_POP when this registered pulse arrives,
    // so an unaccepted grant is withdrawn before it can be acknowledged.
    reg txq_popped;
    always @(posedge clk) begin
        if (rst_c) txq_popped <= 1'b0;
        else txq_popped <= txq_rd_pop;
    end
    // ---------------- 管理发送窗口 ----------------
    // 与发生器的微秒计数同源同起点：窗口位置 = 复位后微秒数 mod 周期。
    wire [N_TX_VL-1:0] win_mask;
    localparam [19:0] WIN_LAST = MGMT_WIN_PERIOD_US - 20'd1;
    reg [19:0] win_pos;
    wire win_enabled = MGMT_WIN_PERIOD_US != 20'd0;
    wire [19:0] win_next = (win_pos == WIN_LAST) ? 20'd0 : win_pos + 20'd1;
    // Decode the window directly from the modulo-period position.  Besides
    // avoiding a redundant state bit, fixed-width arithmetic keeps older
    // Verific-based synthesis front ends from building an invalid state table.
    wire [19:0] win_offset = (win_pos >= MGMT_WIN_START_US) ?
                             win_pos - MGMT_WIN_START_US :
                             MGMT_WIN_PERIOD_US - MGMT_WIN_START_US + win_pos;
    // 开窗判定寄存一拍；入口仍使用原窗口，出口只保留一份待服务预约。
    reg win_open;
    always @(posedge clk) begin
        if (rst_c) begin win_pos <= 20'd0; win_open <= !win_enabled; end
        else begin
            if (tick_us && win_enabled) win_pos <= win_next;
            win_open <= !win_enabled || (win_offset < MGMT_WIN_OPEN_US);
        end
    end
    assign mgmt_window = win_open;
    reg win_open_d, win_reserved, win_credit;
    wire win_grant = grant_ack && grant_vl == MGMT_WIN_VL;
    reg [TX_POOL_LOG+SLOT_AW-BLOCK_AW:0] win_waiting;
    wire win_enqueue = txq_commit_ok && txq_commit_is_mgmt;
    always @(posedge clk) begin
        if (rst_c) win_waiting <= 0;
        else case ({win_enqueue, win_grant})
            2'b10: win_waiting <= win_waiting + 1'b1;
            2'b01: win_waiting <= win_waiting - 1'b1;
            default: begin end
        endcase
    end
    always @(posedge clk) begin
        if (rst_c) begin
            win_open_d <= 1'b0;
            win_reserved <= 1'b0;
            win_credit <= 1'b0;
        end else begin
            win_open_d <= win_open;
            if (win_open && !win_open_d) win_reserved <= 1'b0;
            // 已授权但尚未出队的帧不能重复预约；BAG 未开时保留未授权消息的资格。
            if (win_open && (!win_reserved || !win_open_d) && win_waiting != 0) begin
                win_reserved <= 1'b1;
                win_credit <= 1'b1;
            end
            // 授权优先于同拍开窗/预约，避免同一窗口重复授权。
            if (win_grant) begin
                win_reserved <= 1'b1;
                win_credit <= 1'b0;
            end
        end
    end
    // 关闭窗口机制、全时开窗或时隙模式保持直通，避免常开窗口只发一次。
    assign win_mask = (!win_enabled || MGMT_WIN_OPEN_US >= MGMT_WIN_PERIOD_US ||
                       win_credit || TX_PHASE_ENABLE != 0) ? {N_TX_VL{1'b1}} :
                      ~({{(N_TX_VL-1){1'b0}}, 1'b1} << MGMT_WIN_VL);

    afdx_bag_scheduler #(
        .N_VL                (N_TX_VL),
        .VLW                 (TX_VLW),
        .PHASE_ENABLE        (TX_PHASE_ENABLE),
        .PHASE_SLOT_CYCLES   (64 * (CLK_HZ / 1_000_000)),
        .PHASE_OPEN_CYCLE    (60 * (CLK_HZ / 1_000_000)),
        .PHASE_WINDOW_CYCLES (CLK_HZ / 1_000_000)
    ) u_sched (
        .clk          (clk),
        .rst          (rst_l[4]),
        .tick_us      (tick_us),
        .nonempty     (txq_nonempty & win_mask & tx_wire_eligible & ((TX_SOF_FEEDBACK != 0) ? tx_egress_ready : {N_TX_VL{1'b1}})),
        .queue_pop    (txq_popped),
        .grant_bag_us (cfg_bag_us),
        .grant_valid  (grant_valid),
        .grant_vl     (grant_vl),
        .grant_ack    (grant_ack),
        .bag_open     ()
    );

    // ---------------- 帧组装 ----------------
    wire [7:0] fb_data; wire fb_valid, fb_last, fb_ready; wire [1:0] fb_net_en;
    wire [7:0] fb_raw_data;
    wire fb_raw_valid, fb_raw_last, fb_raw_ready;
    wire [1:0] fb_raw_net;
    // Two-entry elastic FIFO breaks the MAC-ready -> payload-RAM path.
    // Carry per-frame scheduling metadata with every byte across the buffer.
    localparam FBW = 32 + TX_VLW;
    reg [FBW-1:0] fb_fifo_0, fb_fifo_1;
    reg fb_wp, fb_rp;
    reg [1:0] fb_count;
    wire [TX_VLW-1:0] fb_vl;
    wire [19:0] fb_bag;
    wire fb_tag, fb_raw_tag;
    wire fb_push = fb_raw_valid && fb_raw_ready;
    wire fb_take = fb_valid && fb_ready;
    assign fb_raw_ready = fb_count != 2;
    assign fb_valid = fb_count != 0;
    assign {fb_tag,fb_vl,fb_bag,fb_net_en,fb_last,fb_data} = fb_rp ? fb_fifo_1 : fb_fifo_0;
    always @(posedge clk) begin
        if (rst_c) begin fb_wp <= 0; fb_rp <= 0; fb_count <= 0; end
        else begin
            case ({fb_push,fb_take})
                2'b10: fb_count <= fb_count+1'b1;
                2'b01: fb_count <= fb_count-1'b1;
                default: ;
            endcase
            if (fb_push) begin
                if (fb_wp) fb_fifo_1 <= {fb_raw_tag,cfg_idx,rom_bag_us,fb_raw_net,fb_raw_last,fb_raw_data};
                else fb_fifo_0 <= {fb_raw_tag,cfg_idx,rom_bag_us,fb_raw_net,fb_raw_last,fb_raw_data};
                fb_wp <= ~fb_wp;
            end
            if (fb_take) fb_rp <= ~fb_rp;
        end
    end
    wire stat_tx_frame;
    afdx_frame_builder #(
        .N_VL         (N_TX_VL),
        .VLW          (TX_VLW),
        .SLOT_AW     (SLOT_AW),
        .UDP_CSUM_EN (UDP_CSUM_EN),
        .NET_ID      (NET_ID),
        .EQ_ID       (EQ_ID)
    ) u_fb (
        .clk              (clk),
        .rst              (rst_l[5]),
        .grant_valid      (grant_valid),
        .grant_vl         (grant_vl),
        .grant_ack        (grant_ack),
        .cfg_idx          (cfg_idx),
        .cfg_vlid         (cfg_vlid),
        .cfg_lmax         (cfg_lmax),
        .cfg_udp_src      (cfg_udp_src),
        .cfg_udp_dst      (cfg_udp_dst),
        .cfg_net_en       (cfg_net_en),
        .cfg_partition    (cfg_partition),
        .q_rd_vl          (txq_rd_vl),
        .q_rd_start       (txq_rd_start),
        .q_rd_pop         (txq_rd_pop),
        .q_rd_info_valid  (txq_rd_info_valid),
        .q_rd_pop_ready   (txq_rd_pop_ready),
        .q_rd_len         (txq_rd_len),
        .q_rd_payload_sum (txq_rd_meta[31:0]),
        .q_rd_ep          (txq_rd_meta[65:32]),
        .cfg_hold         (cfg_hold),
        .q_rd_data        (txq_rd_data),
        .q_rd_valid       (txq_rd_valid),
        .q_rd_last        (txq_rd_last),
        .q_rd_ready       (txq_rd_ready),
        .f_data           (fb_raw_data),
        .f_valid          (fb_raw_valid),
        .f_last           (fb_raw_last),
        .f_net_en         (fb_raw_net),
        .f_tag            (fb_raw_tag),
        .idle             (fb_idle),
        .f_ready          (fb_raw_ready),
        .stat_frame       (stat_tx_frame),
        .stat_lmax_drop   (stat_lmax_drop)
    );

    // ---------------- 冗余复制 + MAC 发送 ----------------
    wire [7:0] ra_data, rb_data; wire ra_valid, ra_last, ra_ready, rb_valid, rb_last, rb_ready;
    wire stat_tx_a, stat_tx_b;
    afdx_redundancy_ctrl u_rc (
        .clk       (clk),
        .rst       (rst_l[6]),
        .in_data   (fb_data),
        .in_valid  (fb_valid),
        .in_last   (fb_last),
        .in_net_en (fb_net_en),
        .in_ready  (fb_ready),
        .a_data    (ra_data),
        .a_valid   (ra_valid),
        .a_last    (ra_last),
        .a_ready   (ra_ready),
        .b_data    (rb_data),
        .b_valid   (rb_valid),
        .b_last    (rb_last),
        .b_ready   (rb_ready)
    );
    afdx_mac_tx #(
        .FIFO_AW      (MAC_FIFO_AW),
        .IFG          (IFG),
        .N_VL         (N_TX_VL),
        .VLW          (TX_VLW),
        .CLK_HZ       (CLK_HZ),
        .EXTERNAL_MAC (EXTERNAL_MAC)
    ) u_mac_tx_a (
        .clk          (clk),
        .rst          (rst_l[7]),
        .in_data      (ra_data),
        .in_valid     (ra_valid),
        .in_last      (ra_last),
        .in_ready     (ra_ready),
        .in_vl        (fb_vl),
        .in_bag_us    (fb_bag),
        .in_tag       (fb_tag),
        .tx_last      (mac_a_tx_last),
        .wire_sof     (mac_a_wire_sof),
        .wire_done    (mac_a_wire_done),
        .tx_data      (mac_a_tx_data),
        .tx_valid     (mac_a_tx_valid),
        .tx_slot      (mac_a_tx_slot),
        .tx_frame     (mac_a_tx_frame),
        .tx_busy      (mac_a_tx_busy),
        .stat_frame   (stat_tx_a),
        .bag_open     (mac_a_bag_open),
        .stat_sof     (mac_a_sof),
        .stat_sof_vl  (mac_a_sof_vl),
        .stat_sof_tag (tx_sof_tag_a),
        .bag_wait     (mac_a_bag_wait)
    );
    afdx_mac_tx #(
        .FIFO_AW      (MAC_FIFO_AW),
        .IFG          (IFG),
        .N_VL         (N_TX_VL),
        .VLW          (TX_VLW),
        .CLK_HZ       (CLK_HZ),
        .EXTERNAL_MAC (EXTERNAL_MAC)
    ) u_mac_tx_b (
        .clk          (clk),
        .rst          (rst_l[8]),
        .in_data      (rb_data),
        .in_valid     (rb_valid),
        .in_last      (rb_last),
        .in_ready     (rb_ready),
        .in_vl        (fb_vl),
        .in_bag_us    (fb_bag),
        .in_tag       (fb_tag),
        .tx_last      (mac_b_tx_last),
        .wire_sof     (mac_b_wire_sof),
        .wire_done    (mac_b_wire_done),
        .tx_data      (mac_b_tx_data),
        .tx_valid     (mac_b_tx_valid),
        .tx_slot      (mac_b_tx_slot),
        .tx_frame     (mac_b_tx_frame),
        .tx_busy      (mac_b_tx_busy),
        .stat_frame   (stat_tx_b),
        .bag_open     (mac_b_bag_open),
        .stat_sof     (mac_b_sof),
        .stat_sof_vl  (mac_b_sof_vl),
        .stat_sof_tag (tx_sof_tag_b),
        .bag_wait     (mac_b_bag_wait)
    );

    // ---------------- MAC 接收 + 仲裁 ----------------
    wire [7:0] ma_data, mb_data; wire ma_valid, ma_last, ma_ready, mb_valid, mb_last, mb_ready;
    wire ma_ok, ma_fcs, ma_len, ma_ovf, mb_ok, mb_fcs, mb_len, mb_ovf;
    assign cnt_rx_mac_error = EXTERNAL_MAC != 0 ? mac_a_rx_error_count + mac_b_rx_error_count : 32'd0;
    generate if (EXTERNAL_MAC != 0) begin : g_external_rx
        assign ma_data = mac_a_rx_data;
        assign mb_data = mac_b_rx_data;
        assign ma_valid = mac_a_rx_valid;
        assign mb_valid = mac_b_rx_valid;
        assign ma_last = mac_a_rx_last;
        assign mb_last = mac_b_rx_last;
        assign {ma_ok, mb_ok, ma_fcs, mb_fcs, ma_len, mb_len, ma_ovf, mb_ovf} = 8'd0;
        assign mac_a_rx_ready = ma_ready;
        assign mac_b_rx_ready = mb_ready;
    end else begin : g_legacy_rx
        assign mac_a_rx_ready = 1'b0;
        assign mac_b_rx_ready = 1'b0;
    afdx_mac_rx #(
        .FIFO_AW      (MAC_FIFO_AW),
        .GAP_CYCLES (RX_GAP_CYCLES)
    ) u_mac_rx_a (
        .clk           (clk),
        .rst           (rst_l[9]),
        .rx_data       (mac_a_rx_data),
        .rx_valid      (mac_a_rx_valid),
        .rx_frame      (mac_a_rx_frame),
        .out_data      (ma_data),
        .out_valid     (ma_valid),
        .out_last      (ma_last),
        .out_ready     (ma_ready),
        .frame_avail   (),
        .stat_frame_ok (ma_ok),
        .stat_fcs_err  (ma_fcs),
        .stat_len_err  (ma_len),
        .stat_ovf      (ma_ovf)
    );
    afdx_mac_rx #(
        .FIFO_AW      (MAC_FIFO_AW),
        .GAP_CYCLES (RX_GAP_CYCLES)
    ) u_mac_rx_b (
        .clk           (clk),
        .rst           (rst_l[10]),
        .rx_data       (mac_b_rx_data),
        .rx_valid      (mac_b_rx_valid),
        .rx_frame      (mac_b_rx_frame),
        .out_data      (mb_data),
        .out_valid     (mb_valid),
        .out_last      (mb_last),
        .out_ready     (mb_ready),
        .frame_avail   (),
        .stat_frame_ok (mb_ok),
        .stat_fcs_err  (mb_fcs),
        .stat_len_err  (mb_len),
        .stat_ovf      (mb_ovf)
    );
    end endgenerate
    wire pf_valid, pf_ready;
    wire [RX_VLW-1:0] rxq_wr_vl; wire rxq_wr_valid, rxq_wr_ready, rxq_wr_commit, rxq_wr_abort, rxq_ovf;
    wire [7:0] rxq_wr_data; wire [MW-1:0] rxq_wr_meta; wire [N_RX_VL-1:0] rxq_nonempty;
    wire st_ok, st_full, st_ic, st_rm;
    wire [1:0] rx_hdr_events, rx_len_events, rx_unknown_events, rx_csum_events;
    // 先压缩两路事件为 0/1/2，只让一个 32 位加法器进入统计寄存器反馈路径。
    wire [1:0] rx_hdr_count = {1'b0, rx_hdr_events[0]} + {1'b0, rx_hdr_events[1]};
    wire [1:0] rx_len_count = {1'b0, rx_len_events[0]} + {1'b0, rx_len_events[1]};
    wire [1:0] rx_unknown_count = {1'b0, rx_unknown_events[0]} + {1'b0, rx_unknown_events[1]};
    wire [1:0] rx_csum_count = {1'b0, rx_csum_events[0]} + {1'b0, rx_csum_events[1]};
    wire rxq_commit_ok;
    generate if (EXTERNAL_MAC != 0) begin : g_parallel_rx_path
        afdx_rx_parallel #(
            .N_VL        (N_RX_VL),
            .VLW         (RX_VLW),
            .MW          (MW),
            .NET_FROM_SA (NET_FROM_SA),
            .UDP_CSUM_EN (UDP_CSUM_EN),
            .IC_EN       (IC_EN),
            .RM_EN       (RM_EN),
            .SKEW_MAX_US (SKEW_MAX_US)
        ) u_rx_parallel (
            .clk             (clk),
            .rst             (rst_l[12]),
            .now_us          (monotonic_us),
            .in_data         ({mb_data, ma_data}),
            .in_valid        ({mb_valid, ma_valid}),
            .in_last         ({mb_last, ma_last}),
            .in_ts           ({mac_b_rx_ts, mac_a_rx_ts}),
            .in_ready        ({mb_ready, ma_ready}),
            .q_wr_vl         (rxq_wr_vl),
            .q_wr_valid      (rxq_wr_valid),
            .q_wr_data       (rxq_wr_data),
            .q_wr_ready      (rxq_wr_ready),
            .q_wr_commit     (rxq_wr_commit),
            .q_wr_meta       (rxq_wr_meta),
            .q_wr_abort      (rxq_wr_abort),
            .q_wr_ovf        (rxq_ovf),
            .q_wr_commit_ok  (rxq_commit_ok),
            .stat_ok         (st_ok),
            .stat_hdr_err    (rx_hdr_events),
            .stat_len_err    (rx_len_events),
            .stat_vl_unknown (rx_unknown_events),
            .stat_csum_err   (rx_csum_events),
            .stat_full_drop  (st_full),
            .stat_ic_drop    (st_ic),
            .stat_rm_drop    (st_rm),
            .pending_valid   (pf_valid),
            .pending_ready   (pf_ready)
        );
    end else begin : g_serial_rx_path
        // 仅旧字节总线测试路径使用，生产并行接收不生成这些连线。
        wire [7:0] pf_data;
        wire pf_last, pf_net;
        wire ic_req, ic_resp, ic_accept, ic_net;
        wire [RX_VLW-1:0] ic_vl;
        wire [7:0] ic_sn;
        wire st_hdr, st_len, st_unk, st_csum;
        afdx_rx_arb u_arb (
            .a_ts            (mac_a_rx_ts),
            .b_ts            (mac_b_rx_ts),
            .out_ts          (),
            .clk             (clk),
            .rst             (rst_l[11]),
            .a_data          (ma_data),
            .a_valid         (ma_valid),
            .a_last          (ma_last),
            .a_ready         (ma_ready),
            .b_data          (mb_data),
            .b_valid         (mb_valid),
            .b_last          (mb_last),
            .b_ready         (mb_ready),
            .out_data        (pf_data),
            .out_valid       (pf_valid),
            .out_last        (pf_last),
            .out_net         (pf_net),
            .out_ready       (pf_ready)
        );

        // ---------------- 帧解析 + 完整性/冗余 + 接收队列 ----------------
        wire [15:0] lk_vlid; wire lk_hit; wire [RX_VLW-1:0] lk_idx;
        afdx_rx_vl_lookup #(
            .VLW             (RX_VLW)
        ) u_rxlk (
            .vlid            (lk_vlid),
            .hit             (lk_hit),
            .idx             (lk_idx)
        );

        afdx_rx_parser #(
            .N_VL            (N_RX_VL),
            .VLW             (RX_VLW),
            .MW              (MW),
            .NET_FROM_SA     (NET_FROM_SA),
            .UDP_CSUM_CHECK  (UDP_CSUM_EN)
        ) u_parser (
            .clk             (clk),
            .rst             (rst_l[12]),
            .ts_us           (ts_us),
            .in_data         (pf_data),
            .in_valid        (pf_valid),
            .in_last         (pf_last),
            .in_net          (pf_net),
            .in_ready        (pf_ready),
            .lk_vlid         (lk_vlid),
            .lk_hit          (lk_hit),
            .lk_idx          (lk_idx),
            .q_wr_vl         (rxq_wr_vl),
            .q_wr_valid      (rxq_wr_valid),
            .q_wr_data       (rxq_wr_data),
            .q_wr_ready      (rxq_wr_ready),
            .q_wr_commit     (rxq_wr_commit),
            .q_wr_meta       (rxq_wr_meta),
            .q_wr_abort      (rxq_wr_abort),
            .ic_req          (ic_req),
            .ic_vl           (ic_vl),
            .ic_net          (ic_net),
            .ic_sn           (ic_sn),
            .ic_resp         (ic_resp),
            .ic_accept       (ic_accept),
            .stat_ok         (st_ok),
            .stat_hdr_err    (st_hdr),
            .stat_len_err    (st_len),
            .stat_vl_unknown (st_unk),
            .stat_csum_err   (st_csum),
            .stat_full_drop  (st_full)
        );
        afdx_rx_integrity #(
            .N_VL            (N_RX_VL),
            .VLW             (RX_VLW),
            .IC_EN           (IC_EN),
            .RM_EN           (RM_EN),
            .SKEW_MAX_US     (SKEW_MAX_US)
        ) u_ic (
            .clk             (clk),
            .rst             (rst_l[13]),
            .now_us          (monotonic_us),
            .req             (ic_req),
            .vl              (ic_vl),
            .net             (ic_net),
            .sn              (ic_sn),
            .resp            (ic_resp),
            .accept          (ic_accept),
            .stat_ic_drop    (st_ic),
            .stat_rm_drop    (st_rm),
            .init_busy       (),
            .delivery_done   (1'b0),
            .delivery_ok     (1'b0)
        );
        assign rx_hdr_events = {1'b0, st_hdr};
        assign rx_len_events = {1'b0, st_len};
        assign rx_unknown_events = {1'b0, st_unk};
        assign rx_csum_events = {1'b0, st_csum};
    end endgenerate

    wire [RX_VLW-1:0] rxq_rd_vl; wire rxq_rd_start, rxq_rd_pop, rxq_rd_valid, rxq_rd_last, rxq_rd_ready;
    wire rxq_rd_info_valid, rxq_rd_pop_ready;
    wire [SLOT_AW-1:0] rxq_rd_len; wire [MW-1:0] rxq_rd_meta; wire [7:0] rxq_rd_data;
    wire [(RX_VLW>4 ? RX_VLW-4 : 1)-1:0] rxq_ne_group; wire [15:0] rxq_ne_bits; wire rxq_init_busy;
    wire [(N_RX_VL+255)/256-1:0] rxq_ne_regions;
    afdx_msg_queue #(
        .N_VL         (N_RX_VL),
        .VLW          (RX_VLW),
        .DEPTH                (1 << RX_DEPTH_LOG),
        .DW                   (RX_DEPTH_LOG),
        .SLOT_AW              (SLOT_AW),
        .MW                   (MW),
        .POOL_AW              (RX_POOL_LOG),
        .BLOCK_AW             (BLOCK_AW),
        .MAX_MSGS             (RX_MAX_MSGS),
        .PIPE_ADMISSION_MATCH (1),
        .NONEMPTY_RAM         (RX_VLW > 4 ? 1 : 0),
        .SUMMARY_ENABLE       (1)
    ) u_rxq (
        .clk            (clk),
        .rst            (rst_l[14]),
        .wr_vl          (rxq_wr_vl),
        .wr_vl_same     (1'b1),
        .wr_valid       (rxq_wr_valid),
        .wr_data        (rxq_wr_data),
        .wr_ready       (rxq_wr_ready),
        .wr_commit      (rxq_wr_commit),
        .wr_meta        (rxq_wr_meta),
        .wr_abort       (rxq_wr_abort),
        .wr_ovf         (rxq_ovf),
        .wr_commit_ok   (rxq_commit_ok),
        .rd_vl          (rxq_rd_vl),
        .rd_start       (rxq_rd_start),
        .rd_pop         (rxq_rd_pop),
        .rd_info_valid  (rxq_rd_info_valid),
        .rd_pop_ready   (rxq_rd_pop_ready),
        .reclaim_busy   (),
        .rd_len         (rxq_rd_len),
        .rd_meta        (rxq_rd_meta),
        .rd_data        (rxq_rd_data),
        .rd_valid       (rxq_rd_valid),
        .rd_last        (rxq_rd_last),
        .rd_ready       (rxq_rd_ready),
        .nonempty       (rxq_nonempty),
        .pool_used      (rx_buffer_used),
        .pool_peak      (rx_buffer_peak),
        .pool_msgs      (rx_buffer_messages),
        .pool_msgs_peak (rx_buffer_messages_peak),
        .ne_group       (rxq_ne_group),
        .ne_bits        (rxq_ne_bits),
        .init_busy      (rxq_init_busy),
        .ne_regions     (rxq_ne_regions)
    );

    wire [MW-1:0] rx_msg_meta; wire st_deliver;
    afdx_rx_deliver #(
        .N_VL         (N_RX_VL),
        .VLW          (RX_VLW),
        .SLOT_AW        (SLOT_AW),
        .MW             (MW),
        .NONEMPTY_RAM   (RX_VLW > 4 ? 1 : 0),
        .REGISTER_RESET (1),
        .SUMMARY_ENABLE (1)
    ) u_deliver (
        .clk             (clk),
        .rst             (rst),
        .nonempty        (rxq_nonempty),
        .ne_group        (rxq_ne_group),
        .ne_bits         (rxq_ne_bits),
        .q_init_busy     (rxq_init_busy),
        .q_rd_vl         (rxq_rd_vl),
        .q_rd_start      (rxq_rd_start),
        .q_rd_pop        (rxq_rd_pop),
        .q_rd_info_valid (rxq_rd_info_valid),
        .q_rd_pop_ready  (rxq_rd_pop_ready),
        .q_rd_len        (rxq_rd_len),
        .q_rd_meta       (rxq_rd_meta),
        .q_rd_data       (rxq_rd_data),
        .q_rd_valid      (rxq_rd_valid),
        .q_rd_last       (rxq_rd_last),
        .q_rd_ready      (rxq_rd_ready),
        .msg_vl          (rx_msg_vl),
        .msg_valid       (rx_msg_valid),
        .msg_data        (rx_msg_data),
        .msg_last        (rx_msg_last),
        .msg_ready       (rx_msg_ready),
        .msg_len         (rx_msg_len),
        .msg_meta        (rx_msg_meta),
        .stat_deliver    (st_deliver),
        .ne_regions      (rxq_ne_regions)
    );
    assign rx_msg_net = rx_msg_meta[8];
    assign rx_msg_sn  = rx_msg_meta[7:0];
    generate if (RX_SESSION_META != 0) begin : g_rx_session
        assign {rx_msg_ts, rx_msg_src_ip, rx_msg_sport, rx_msg_dport} = rx_msg_meta[MW-1:9];
    end else begin : g_rx_no_session
        assign {rx_msg_ts, rx_msg_src_ip, rx_msg_sport, rx_msg_dport} = 96'd0;
    end endgenerate

    // ---------------- 统计计数 ----------------
    reg tx_message_done;
    // Merge the two drop sources before driving the 32 counter enables.
    // Count simultaneous assertions once, as at the original interface.
    reg rx_full_drop_event;
    always @(posedge clk) begin
        if (rst_c) tx_message_done <= 0;
        else tx_message_done <= tx_in_beat && txh_last;
        rx_full_drop_event <= !rst_c && (st_full || (EXTERNAL_MAC == 0 && rxq_ovf));
    end
    always @(posedge clk) begin
        if (rst_c) begin
            cnt_tx_msgs <= 0; cnt_tx_frames <= 0; cnt_tx_frames_a <= 0; cnt_tx_frames_b <= 0;
            cnt_tx_lmax_drop <= 0; cnt_tx_ovf_drop <= 0;
            cnt_rx_frames_a <= 0; cnt_rx_frames_b <= 0; cnt_rx_fcs_err <= 0; cnt_rx_mac_len_err <= 0;
            cnt_rx_mac_ovf <= 0; cnt_rx_hdr_err <= 0; cnt_rx_len_err <= 0; cnt_rx_vl_unknown <= 0;
            cnt_rx_csum_err <= 0; cnt_rx_full_drop <= 0; cnt_rx_ic_drop <= 0; cnt_rx_rm_drop <= 0;
            cnt_rx_accepted <= 0; cnt_rx_delivered <= 0;
        end else begin
            if (tx_message_done) cnt_tx_msgs <= cnt_tx_msgs + 1;
            if (stat_tx_frame)   cnt_tx_frames    <= cnt_tx_frames + 1;
            if (stat_tx_a)       cnt_tx_frames_a  <= cnt_tx_frames_a + 1;
            if (stat_tx_b)       cnt_tx_frames_b  <= cnt_tx_frames_b + 1;
            if (stat_lmax_drop | tx_msg_abort) cnt_tx_lmax_drop <= cnt_tx_lmax_drop + 1;
            if (txq_ovf)         cnt_tx_ovf_drop  <= cnt_tx_ovf_drop + 1;
            if (EXTERNAL_MAC != 0) begin
                cnt_rx_frames_a <= mac_a_rx_good_count;
                cnt_rx_frames_b <= mac_b_rx_good_count;
                cnt_rx_mac_len_err <= mac_a_rx_length_count + mac_b_rx_length_count;
                cnt_rx_mac_ovf <= mac_a_rx_overflow_count + mac_b_rx_overflow_count;
            end else if (ma_ok)  cnt_rx_frames_a <= cnt_rx_frames_a + 1;
            if (mb_ok)           cnt_rx_frames_b  <= cnt_rx_frames_b + 1;
            cnt_rx_fcs_err <= cnt_rx_fcs_err + {31'd0, ma_fcs} + {31'd0, mb_fcs};
            if (EXTERNAL_MAC == 0) cnt_rx_mac_len_err <= cnt_rx_mac_len_err + {31'd0, ma_len} + {31'd0, mb_len};
            if (EXTERNAL_MAC == 0) cnt_rx_mac_ovf <= cnt_rx_mac_ovf + {31'd0, ma_ovf} + {31'd0, mb_ovf};
            if (|rx_hdr_events) cnt_rx_hdr_err <= cnt_rx_hdr_err + {30'd0, rx_hdr_count};
            if (|rx_len_events) cnt_rx_len_err <= cnt_rx_len_err + {30'd0, rx_len_count};
            if (|rx_unknown_events) cnt_rx_vl_unknown <= cnt_rx_vl_unknown + {30'd0, rx_unknown_count};
            if (|rx_csum_events) cnt_rx_csum_err <= cnt_rx_csum_err + {30'd0, rx_csum_count};
            if (rx_full_drop_event) cnt_rx_full_drop <= cnt_rx_full_drop + 1;
            if (st_ic)           cnt_rx_ic_drop   <= cnt_rx_ic_drop + 1;
            if (st_rm)           cnt_rx_rm_drop   <= cnt_rx_rm_drop + 1;
            if (st_ok)           cnt_rx_accepted  <= cnt_rx_accepted + 1;
            if (st_deliver)      cnt_rx_delivered <= cnt_rx_delivered + 1;
        end
    end
endmodule
