/*
 * ================================================================
 * File Name    : afdx_mgmt_plane.v
 * Module Name  : afdx_mgmt_plane
 *
 * Function:
 *   管理面顶层：请求缓冲 → 按 UDP 端口分发的服务引擎（SNMP / TFTP+615A / RTC 同步）→ 响应缓冲。
 *   作业串行执行，一次只有一个引擎占用请求读口和响应写口：
 *     优先级 RTC 请求 > SNMP 请求 > TFTP 请求 > TFTP 超时 > Trap；响应缓冲忙时不启动新作业，
 *     配置 RAM 忙时不启动 SNMP 作业（避免表读取与校验扫描冲突）。
 *   管理响应经 afdx_app_mux 以低于实时业务的优先级进入管理 VL；UDP 端点按请求源端口覆盖，
 *   RTC 的 DELAY_REQ 带 SOF 时间戳标记。SNMP 端口修改在本次响应发出后生效（影子值）。
 *
 * Sub Module:
 *   afdx_mgmt_rxbuf
 *   afdx_mgmt_txbuf
 *   afdx_snmp_agent
 *   afdx_tftp_server
 *   afdx_loader
 *   afdx_rtc64
 *   afdx_rtc_sync
 *   afdx_mgmt_events
 *   afdx_snmp_values
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_snmp_mib.vh"
module afdx_mgmt_plane #(
    parameter VLW = 3,
    parameter TX_VLW = 3,
    parameter N_TX_VL = 8,
    parameter MGMT_RX_VL = 0,
    parameter MGMT_TX_VL = 0,
    parameter [15:0] TID_BASE = 16'hC000,
    parameter [15:0] RTC_PORT = 16'd3190,
    parameter [127:0] READ_COMM = {"public", 80'd0},
    parameter [4:0] READ_COMM_LEN = 5'd6,
    parameter [127:0] WRITE_COMM = 128'd0,
    parameter [4:0] WRITE_COMM_LEN = 5'd0,  // 0：写团体字未配置，SET 一律 noAccess
    parameter TFTP_TIMEOUT_US = 1000000,
    parameter TFTP_RETRIES = 5,
    parameter APP_AW = 12,
    parameter SELFTEST_US = 50000,
    parameter [7:0] NET_ID = 8'd1,
    parameter [7:0] EQ_ID = 8'd1,
    parameter [31:0] BUF_HIGH = 32'd224,     // 缓冲高水位（块）
    parameter [10:0] SNMP_MAX_PDU = 11'd1471 // SNMP 响应上限，不超过管理 VL 的 Lmax
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              enable,
    input  wire [VLW-1:0]    rx_vl,
    input  wire              rx_valid,
    input  wire [7:0]        rx_data,
    input  wire              rx_last,
    input  wire [31:0]       rx_src_ip,
    input  wire [15:0]       rx_sport,
    input  wire [15:0]       rx_dport,
    input  wire [31:0]       rx_ts,
    output wire [TX_VLW-1:0] tx_vl,
    output wire              tx_valid,
    output wire [7:0]        tx_data,
    output wire              tx_last,
    output wire [33:0]       tx_ep,
    input  wire              tx_ready,
    input  wire              tick_us,
    input  wire [31:0]       mono_us,
    output wire [63:0]       rtc64,
    input  wire              sof_tag,
    input  wire [799:0]      dp_stats,          // afdxStats R.2.1..25
    input  wire              tx_frame,
    // 运行时发送配置
    output wire [TX_VLW-1:0] cfg_m_rd_idx,
    output wire              cfg_m_rd_shadow,
    input  wire [19:0]       cfg_m_bag_us,
    input  wire [10:0]       cfg_m_lmax,
    input  wire [1:0]        cfg_m_net_en,
    input  wire [15:0]       cfg_m_vlid,
    input  wire [10:0]       cfg_m_rom_lmax,
    output wire              cfg_m_wr_en,
    output wire [TX_VLW-1:0] cfg_m_wr_idx,
    output wire [19:0]       cfg_m_wr_bag_us,
    output wire [10:0]       cfg_m_wr_lmax,
    output wire [1:0]        cfg_m_wr_net_en,
    output wire              cfg_cmd_copy, cfg_cmd_verify, cfg_cmd_apply, cfg_cmd_rollback,
    input  wire              cfg_busy,
    input  wire              cfg_active_bank,
    input  wire              cfg_prev_valid,
    input  wire              cfg_done,
    input  wire              cfg_verify_ok,
    input  wire [2:0]        cfg_verify_code,
    input  wire [31:0]       cfg_budget,
    // 应用镜像内部只读接口
    input  wire [APP_AW-1:0] app_rd_addr,
    output wire [7:0]        app_rd_data,
    output wire              app_bank,
    output wire [31:0]       app_length,
    output wire [31:0]       app_version,
    output wire [2:0]        rtc_state
);
    // 本地复位树：每个子模块一份就近放置的复位寄存器（延迟相同），
    // 避免远处的全局复位副本经长布线驱动各子模块的使能。
    (* preserve, maxfan = 64 *) reg [9:0] rst_l;
    always @(posedge clk) rst_l <= {10{rst}};
    // ---------------- 可写配置 ----------------
    reg [15:0] snmp_port, snmp_port_new, trap_port, rtc_port;
    reg [31:0] trap_mask, trap_interval_ms, err_threshold, link_timeout_ms;
    reg [31:0] rtc_step_th, rtc_holdover, rtc_unsync, rtc_oneway;
    reg [31:0] cfg_last_result;
    reg snmp_port_pend, shadow_ready, stats_clear, snmp_abort, rtc_manual;
    reg [63:0] rtc_manual_val;
    reg ext_commit, ext_rollback, ev_cfg;
    wire ld_cfg_event;

    // ---------------- 请求 / 响应缓冲 ----------------
    wire req_valid;
    wire [1:0] req_class;
    wire [10:0] req_len;
    wire [31:0] req_ts;
    wire [15:0] req_sport, req_dport;
    reg req_release;
    reg [10:0] rq_addr;
    wire [7:0] rq_data;
    wire [31:0] cnt_rx_accept, cnt_rx_full, cnt_rx_port;
    afdx_mgmt_rxbuf #(
        .VLW        (VLW),
        .MGMT_RX_VL (MGMT_RX_VL),
        .TID_BASE   (TID_BASE)
    ) u_rxbuf (
        .clk           (clk),
        .rst           (rst_l[1]),
        .enable        (enable),
        .snmp_port     (snmp_port),
        .rtc_port      (rtc_port),
        .rx_vl         (rx_vl),
        .rx_valid      (rx_valid),
        .rx_data       (rx_data),
        .rx_last       (rx_last),
        .rx_src_ip     (rx_src_ip),
        .rx_sport      (rx_sport),
        .rx_dport      (rx_dport),
        .rx_ts         (rx_ts),
        .req_valid     (req_valid),
        .req_class     (req_class),
        .req_len       (req_len),
        .req_src_ip    (),
        .req_sport     (req_sport),
        .req_dport     (req_dport),
        .req_ts        (req_ts),
        .req_release   (req_release),
        .rd_addr       (rq_addr),
        .rd_data       (rq_data),
        .cnt_accept    (cnt_rx_accept),
        .cnt_drop_full (cnt_rx_full),
        .cnt_drop_port (cnt_rx_port),
        .cnt_drop_long ()
    );
    reg tb_we, tb_post, tb_tag;
    reg [10:0] tb_wa, tb_len;
    reg [7:0] tb_wd;
    reg [15:0] tb_sport, tb_dport;
    wire tb_busy;
    afdx_mgmt_txbuf #(
        .TX_VLW     (TX_VLW),
        .MGMT_TX_VL (MGMT_TX_VL)
    ) u_txbuf (
        .clk        (clk),
        .rst        (rst_l[2]),
        .wr_en      (tb_we),
        .wr_addr    (tb_wa),
        .wr_data    (tb_wd),
        .post       (tb_post),
        .post_len   (tb_len),
        .post_sport (tb_sport),
        .post_dport (tb_dport),
        .post_tag   (tb_tag),
        .busy       (tb_busy),
        .tx_vl      (tx_vl),
        .tx_valid   (tx_valid),
        .tx_data    (tx_data),
        .tx_last    (tx_last),
        .tx_ep      (tx_ep),
        .tx_ready   (tx_ready),
        .cnt_sent   ()
    );

    // ---------------- 作业调度 ----------------
    localparam W_NONE = 3'd0, W_SNMP = 3'd1, W_TFTP = 3'd2, W_RTC = 3'd3, W_TRAP = 3'd4, W_TIMER = 3'd5;
    reg [2:0] owner;
    // 读写口选择用与 owner 同步更新的独热寄存器（own_* 全 0 时选 RTC 口）
    (* maxfan = 32 *) reg own_snmp, own_tftp, own_rtc;
    reg go_snmp, go_trap, go_tftp, go_timer, go_rtc, ev_take;
    wire snmp_done, tftp_done, rtc_done, tftp_due, ev_pending;
    wire [2:0] ev_pick;
    wire job_done = (owner == W_SNMP || owner == W_TRAP) ? snmp_done :
                    (owner == W_TFTP || owner == W_TIMER) ? tftp_done : (owner == W_RTC) ? rtc_done : 1'b0;
    always @(posedge clk) begin
        if (rst_l[0]) begin
            owner <= W_NONE; go_snmp <= 1'b0; go_trap <= 1'b0; go_tftp <= 1'b0; go_timer <= 1'b0;
            go_rtc <= 1'b0; req_release <= 1'b0; ev_take <= 1'b0;
            own_snmp <= 1'b0; own_tftp <= 1'b0; own_rtc <= 1'b0;
        end else begin
            go_snmp <= 1'b0; go_trap <= 1'b0; go_tftp <= 1'b0; go_timer <= 1'b0; go_rtc <= 1'b0;
            req_release <= 1'b0; ev_take <= 1'b0;
            if (owner == W_NONE) begin
                // 空闲时各引擎都不会 post（post 总在 done 之前），只需检查发送缓冲忙
                if (enable && !tb_busy && !req_release) begin
                    if (req_valid && req_class == 2'd3) begin owner <= W_RTC; own_rtc <= 1'b1; go_rtc <= 1'b1; end
                    else if (req_valid && req_class == 2'd1 && !cfg_busy) begin
                        owner <= W_SNMP; own_snmp <= 1'b1; go_snmp <= 1'b1;
                    end else if (req_valid && req_class == 2'd2) begin
                        owner <= W_TFTP; own_tftp <= 1'b1; go_tftp <= 1'b1;
                    end else if (tftp_due) begin owner <= W_TIMER; own_tftp <= 1'b1; go_timer <= 1'b1; end
                    else if (ev_pending && ev_pick != 3'd0 && !cfg_busy) begin
                        owner <= W_TRAP; own_snmp <= 1'b1; go_trap <= 1'b1; ev_take <= 1'b1;
                    end
                end
            end else if (job_done) begin
                if (owner == W_SNMP || owner == W_TFTP || owner == W_RTC) req_release <= 1'b1;
                owner <= W_NONE; own_snmp <= 1'b0; own_tftp <= 1'b0; own_rtc <= 1'b0;
            end
        end
    end

    // ---------------- SNMP ----------------
    wire [10:0] snmp_addr, snmp_tb_wa, snmp_tb_len;
    wire snmp_tb_we, snmp_tb_post;
    wire [7:0] snmp_tb_wd;
    wire [15:0] snmp_tb_dport;
    wire [6:0] val_k, wr_k;
    wire [15:0] val_inst, wr_inst;
    wire [63:0] val_data, wr_val;
    wire wr_en;
    reg wr_done, wr_err;
    wire [31:0] cnt_snmp_in, cnt_snmp_badc, cnt_snmp_perr, cnt_snmp_out, cnt_trap;
    wire [2:0] tv_type;
    wire [31:0] tv_count, tv_first, tv_last, cnt_trap_drop;
    wire ld_staging;
    afdx_snmp_agent #(
        .N_TABLE (N_TX_VL),
        .MAX_PDU (SNMP_MAX_PDU)
    ) u_snmp (
        .clk            (clk),
        .rst            (rst_l[3]),
        .go_req         (go_snmp),
        .go_trap        (go_trap),
        .trap_type      (ev_pick),
        .done           (snmp_done),
        .req_len        (req_len),
        .req_sport      (req_sport),
        .rq_addr        (snmp_addr),
        .rq_data        (rq_data),
        .tb_wr_en       (snmp_tb_we),
        .tb_wr_addr     (snmp_tb_wa),
        .tb_wr_data     (snmp_tb_wd),
        .tb_post        (snmp_tb_post),
        .tb_len         (snmp_tb_len),
        .tb_dport       (snmp_tb_dport),
        .read_comm      (READ_COMM),
        .read_comm_len  (READ_COMM_LEN),
        .write_comm     (WRITE_COMM),
        .write_comm_len (WRITE_COMM_LEN),
        .trap_port      (trap_port),
        .cfg_busy       (cfg_busy || ld_staging),
        .val_k          (val_k),
        .val_inst       (val_inst),
        .val_data       (val_data),
        .wr_en          (wr_en),
        .wr_k           (wr_k),
        .wr_inst        (wr_inst),
        .wr_val         (wr_val),
        .wr_done        (wr_done),
        .wr_err         (wr_err),
        .cnt_in         (cnt_snmp_in),
        .cnt_bad_comm   (cnt_snmp_badc),
        .cnt_parse_err  (cnt_snmp_perr),
        .cnt_out        (cnt_snmp_out),
        .cnt_trap       (cnt_trap)
    );

    // ---------------- TFTP + 对象管理器 ----------------
    wire [10:0] tftp_addr, tftp_tb_wa, tftp_tb_len;
    wire tftp_tb_we, tftp_tb_post;
    wire [7:0] tftp_tb_wd;
    wire [15:0] tftp_tb_sport, tftp_tb_dport, cur_block;
    wire ld_open, ld_write, ld_open_ack, ld_wvalid, ld_wsel, ld_finish, ld_fin_ack, ld_abort, ld_rd_close;
    wire ld_abort_evt, ld_ev, tftp_ev;
    wire [2:0] ld_obj, ld_rd_obj, ld_state, ld_object;
    wire [4:0] ld_open_err, ld_werr, ld_fin_err, ld_error;
    wire [23:0] ld_size, ld_rd_addr;
    wire [7:0] ld_wbyte, ld_rd_data;
    wire [31:0] cnt_tftp_retry, cnt_tftp_err, cfg_generation, app_crc;
    wire [31:0] ld_bytes, ld_crc_e, ld_crc_a, cnt_loads, cnt_unloads;
    wire [15:0] ld_fail_block;
    afdx_tftp_server #(
        .TID_BASE    (TID_BASE),
        .TIMEOUT_US  (TFTP_TIMEOUT_US),
        .MAX_RETRIES (TFTP_RETRIES)
    ) u_tftp (
        .clk          (clk),
        .rst          (rst_l[4]),
        .mono_us      (mono_us),
        .go           (go_tftp),
        .go_timer     (go_timer),
        .done         (tftp_done),
        .timer_due    (tftp_due),
        .req_len      (req_len),
        .req_sport    (req_sport),
        .req_dport    (req_dport),
        .rq_addr      (tftp_addr),
        .rq_data      (rq_data),
        .tb_wr_en     (tftp_tb_we),
        .tb_wr_addr   (tftp_tb_wa),
        .tb_wr_data   (tftp_tb_wd),
        .tb_post      (tftp_tb_post),
        .tb_len       (tftp_tb_len),
        .tb_sport     (tftp_tb_sport),
        .tb_dport     (tftp_tb_dport),
        .ld_open      (ld_open),
        .ld_obj       (ld_obj),
        .ld_write     (ld_write),
        .ld_open_ack  (ld_open_ack),
        .ld_open_err  (ld_open_err),
        .ld_size      (ld_size),
        .ld_wvalid    (ld_wvalid),
        .ld_wsel      (ld_wsel),
        .ld_wbyte     (ld_wbyte),
        .ld_werr      (ld_werr),
        .ld_finish    (ld_finish),
        .ld_fin_ack   (ld_fin_ack),
        .ld_fin_err   (ld_fin_err),
        .ld_abort     (ld_abort),
        .ld_rd_obj    (ld_rd_obj),
        .ld_rd_addr   (ld_rd_addr),
        .ld_rd_data   (ld_rd_data),
        .ld_rd_close  (ld_rd_close),
        .ld_abort_evt (ld_abort_evt),
        .cur_block    (cur_block),
        .cnt_retries  (cnt_tftp_retry),
        .cnt_errors   (cnt_tftp_err),
        .ev_fault     (tftp_ev)
    );
    wire [TX_VLW-1:0] ld_cfg_idx, ld_cfg_wr_idx, wh_idx;
    wire ld_cfg_wr, ld_copy, ld_verify, ld_apply, ld_roll;
    wire [19:0] ld_cfg_bag;
    wire [10:0] ld_cfg_lmax;
    wire [1:0] ld_cfg_net;
    wire [511:0] fault_vec;
    afdx_loader #(
        .N_VL        (N_TX_VL),
        .VLW         (TX_VLW),
        .APP_AW      (APP_AW),
        .SELFTEST_US (SELFTEST_US)
    ) u_loader (
        .clk             (clk),
        .rst             (rst_l[5]),
        .mono_us         (mono_us),
        .open_req        (ld_open),
        .open_obj        (ld_obj),
        .open_write      (ld_write),
        .open_ack        (ld_open_ack),
        .open_err        (ld_open_err),
        .open_size       (ld_size),
        .wbyte_valid     (ld_wvalid),
        .wsel            (ld_wsel),
        .wbyte           (ld_wbyte),
        .werr            (ld_werr),
        .wfinish         (ld_finish),
        .fin_ack         (ld_fin_ack),
        .fin_err         (ld_fin_err),
        .wabort          (ld_abort),
        .rd_obj          (ld_rd_obj),
        .rd_addr         (ld_rd_addr),
        .rd_data         (ld_rd_data),
        .rd_close        (ld_rd_close),
        .snmp_abort      (snmp_abort),
        .abort_evt       (ld_abort_evt),
        .staging         (ld_staging),
        .cfg_rd_idx      (ld_cfg_idx),
        .cfg_bag_us      (cfg_m_bag_us),
        .cfg_lmax        (cfg_m_lmax),
        .cfg_net_en      (cfg_m_net_en),
        .cfg_wr_en       (ld_cfg_wr),
        .cfg_wr_idx      (ld_cfg_wr_idx),
        .cfg_wr_bag_us   (ld_cfg_bag),
        .cfg_wr_lmax     (ld_cfg_lmax),
        .cfg_wr_net_en   (ld_cfg_net),
        .cfg_copy        (ld_copy),
        .cfg_verify      (ld_verify),
        .cfg_apply       (ld_apply),
        .cfg_rollback    (ld_roll),
        .cfg_busy        (cfg_busy),
        .bg_ok           (owner == W_NONE && !go_snmp),
        .cfg_done        (cfg_done),
        .cfg_verify_ok   (cfg_verify_ok),
        .tx_frame        (tx_frame),
        .ext_commit      (ext_commit),
        .ext_rollback    (ext_rollback),
        .cfg_generation  (cfg_generation),
        .app_rd_addr     (app_rd_addr),
        .app_rd_data     (app_rd_data),
        .app_bank        (app_bank),
        .app_length      (app_length),
        .app_version     (app_version),
        .app_crc         (app_crc),
        .fault_vec       (fault_vec),
        .cur_block       (cur_block),
        .ld_state        (ld_state),
        .ld_object       (ld_object),
        .ld_bytes        (ld_bytes),
        .ld_crc_expected (ld_crc_e),
        .ld_crc_actual   (ld_crc_a),
        .ld_error        (ld_error),
        .ld_fail_block   (ld_fail_block),
        .cnt_loads       (cnt_loads),
        .cnt_unloads     (cnt_unloads),
        .ev_fault        (ld_ev)
    );

    // ---------------- RTC ----------------
    wire [10:0] rtc_addr;
    wire rtc_tb_we, rtc_tb_post, rtc_slew_load, rtc_step, rtc_set, rtc_ev;
    wire [10:0] rtc_tb_wa, rtc_tb_len;
    wire [7:0] rtc_tb_wd, rtc_err;
    wire [15:0] rtc_tb_dport, rtc_src;
    wire signed [31:0] rtc_freq, rtc_slew_val, rtc_last_off;
    wire signed [63:0] rtc_step_delta;
    wire [63:0] rtc_set_val;
    wire [31:0] r_sync, r_step, r_slew, r_switch, r_lost, r_dup, r_ooo, r_bad, r_tmo, r_rej;
    wire [31:0] r_seq, r_mono, r_maxoff, r_delay;
    wire [7:0] r_quality;
    afdx_rtc64 u_rtc64 (
        .clk            (clk),
        .rst            (rst_l[6]),
        .tick_us        (tick_us),
        .freq_corr      (rtc_freq),
        .slew_load      (rtc_slew_load),
        .slew_value     (rtc_slew_val),
        .step           (rtc_step),
        .step_delta     (rtc_step_delta),
        .set            (rtc_set),
        .set_value      (rtc_set_val),
        .rtc_us         (rtc64),
        .slew_remaining (),
        .adjust_busy    ()
    );
    afdx_rtc_sync #(
        .LOCAL_ID ({NET_ID, EQ_ID})
    ) u_rtc_sync (
        .clk             (clk),
        .rst             (rst_l[7]),
        .mono_us         (mono_us),
        .rtc_us          (rtc64),
        .go              (go_rtc),
        .done            (rtc_done),
        .req_len         (req_len),
        .req_ts          (req_ts),
        .req_sport       (req_sport),
        .rq_addr         (rtc_addr),
        .rq_data         (rq_data),
        .tb_wr_en        (rtc_tb_we),
        .tb_wr_addr      (rtc_tb_wa),
        .tb_wr_data      (rtc_tb_wd),
        .tb_post         (rtc_tb_post),
        .tb_len          (rtc_tb_len),
        .tb_dport        (rtc_tb_dport),
        .sof_tag         (sof_tag),
        .freq_corr       (rtc_freq),
        .slew_load       (rtc_slew_load),
        .slew_value      (rtc_slew_val),
        .step            (rtc_step),
        .step_delta      (rtc_step_delta),
        .set             (rtc_set),
        .set_value       (rtc_set_val),
        .step_threshold  (rtc_step_th),
        .holdover_us     (rtc_holdover),
        .unsync_us       (rtc_unsync),
        .oneway_delay_us (rtc_oneway),
        .manual_set      (rtc_manual),
        .manual_value    (rtc_manual_val),
        .state           (rtc_state),
        .cnt_sync        (r_sync),
        .cnt_step        (r_step),
        .cnt_slew        (r_slew),
        .cnt_switch      (r_switch),
        .cnt_lost        (r_lost),
        .cnt_dup         (r_dup),
        .cnt_ooo         (r_ooo),
        .cnt_bad         (r_bad),
        .cnt_timeout     (r_tmo),
        .cnt_reject      (r_rej),
        .last_seq        (r_seq),
        .last_sync_mono  (r_mono),
        .last_offset     (rtc_last_off),
        .max_abs_offset  (r_maxoff),
        .path_delay      (r_delay),
        .source_id       (rtc_src),
        .source_quality  (r_quality),
        .last_error      (rtc_err),
        .ev_fault        (rtc_ev)
    );

    // ---------------- 读写口多路选择 ----------------
    always @* begin
        if (own_snmp) begin
            rq_addr = snmp_addr; tb_we = snmp_tb_we; tb_wa = snmp_tb_wa; tb_wd = snmp_tb_wd;
            tb_post = snmp_tb_post; tb_len = snmp_tb_len; tb_sport = snmp_port; tb_dport = snmp_tb_dport;
            tb_tag = 1'b0;
        end else if (own_tftp) begin
            rq_addr = tftp_addr; tb_we = tftp_tb_we; tb_wa = tftp_tb_wa; tb_wd = tftp_tb_wd;
            tb_post = tftp_tb_post; tb_len = tftp_tb_len; tb_sport = tftp_tb_sport; tb_dport = tftp_tb_dport;
            tb_tag = 1'b0;
        end else begin
            rq_addr = rtc_addr; tb_we = rtc_tb_we; tb_wa = rtc_tb_wa; tb_wd = rtc_tb_wd;
            tb_post = rtc_tb_post && own_rtc; tb_len = rtc_tb_len; tb_sport = rtc_port;
            tb_dport = rtc_tb_dport; tb_tag = 1'b1;
        end
    end

    // ---------------- SNMP 第二阶段写入 ----------------
    localparam H_IDLE = 3'd0, H_COPY = 3'd1, H_READ = 3'd2, H_WRITE = 3'd3, H_CMD = 3'd4, H_APPLY = 3'd5,
               H_DONE = 3'd6;
    reg [2:0] h;
    reg [2:0] h_wait;
    localparam H_EXEC = 3'd7;
    reg h_active, h_copy, h_verify, h_apply, h_roll, h_wr;
    reg [TX_VLW-1:0] h_idx;
    reg [19:0] h_bag;
    reg [10:0] h_lmax;
    reg [1:0] h_net;
    reg [6:0] h_k;
    reg [63:0] h_v;
    // h_v is captured in H_IDLE, before H_EXEC publishes rtc_manual. The
    // receiver only consumes this bus with that pulse; no wide enable needed.
    always @(posedge clk) rtc_manual_val <= h_v;
    // Same reset depth as rst_l[0], but a separate local load for this bank.
    (* preserve, maxfan = 32 *) reg threshold_reset;
    always @(posedge clk) threshold_reset <= rst;
    always @(posedge clk) begin
        if (threshold_reset) err_threshold <= 32'd16;
        else if (h == H_EXEC && h_k == `MIB_ERRTHRESHOLD)
            err_threshold <= h_v[31:0];
    end
    wire ld_self_fail;
    always @(posedge clk) begin
        if (rst_l[0]) begin
            h <= H_IDLE; h_active <= 1'b0; wr_done <= 1'b0; wr_err <= 1'b0;
            h_copy <= 1'b0; h_verify <= 1'b0; h_apply <= 1'b0; h_roll <= 1'b0; h_wr <= 1'b0;
            snmp_port <= 16'd161; snmp_port_new <= 16'd161; snmp_port_pend <= 1'b0; trap_port <= 16'd162;
            rtc_port <= RTC_PORT; trap_mask <= 32'd0; trap_interval_ms <= 32'd1000;
            link_timeout_ms <= 32'd100; rtc_step_th <= 32'd1000; rtc_holdover <= 32'd3000000;
            rtc_unsync <= 32'd30000000; rtc_oneway <= 32'd0; cfg_last_result <= 32'd0;
            shadow_ready <= 1'b0; stats_clear <= 1'b0; snmp_abort <= 1'b0; rtc_manual <= 1'b0;
            ext_commit <= 1'b0; ext_rollback <= 1'b0; ev_cfg <= 1'b0;
        end else begin
            wr_done <= 1'b0; h_copy <= 1'b0; h_verify <= 1'b0; h_apply <= 1'b0; h_roll <= 1'b0; h_wr <= 1'b0;
            stats_clear <= 1'b0; snmp_abort <= 1'b0; rtc_manual <= 1'b0;
            ext_commit <= 1'b0; ext_rollback <= 1'b0; ev_cfg <= ld_cfg_event;
            if (snmp_done && snmp_port_pend) begin snmp_port <= snmp_port_new; snmp_port_pend <= 1'b0; end
            if (ld_self_fail) cfg_last_result <= 32'd6;
            if (ld_apply || ld_copy) shadow_ready <= 1'b0;     // TFTP 加载改写了影子银行
            case (h)
                H_IDLE: if (wr_en) begin
                    h_k <= wr_k; h_v <= wr_val; h_idx <= wr_inst[TX_VLW-1:0] - 1'b1; wr_err <= 1'b0;
                    h <= H_EXEC;
                end
                H_EXEC: begin
                    case (h_k)
                        `MIB_STATSCLEAR: begin stats_clear <= 1'b1; h <= H_DONE; end
                        `MIB_LDABORT: begin snmp_abort <= 1'b1; h <= H_DONE; end
                        `MIB_RTCSTEPTHRESHOLD: begin rtc_step_th <= h_v[31:0]; h <= H_DONE; end
                        `MIB_RTCHOLDOVERUS: begin rtc_holdover <= h_v[31:0]; h <= H_DONE; end
                        `MIB_RTCUNSYNCUS: begin rtc_unsync <= h_v[31:0]; h <= H_DONE; end
                        `MIB_RTCONEWAYDELAY: begin rtc_oneway <= h_v[31:0]; h <= H_DONE; end
                        `MIB_RTCSET: begin rtc_manual <= 1'b1; h <= H_DONE; end
                        `MIB_TRAPMASK: begin trap_mask <= h_v[31:0]; h <= H_DONE; end
                        `MIB_TRAPINTERVALMS: begin trap_interval_ms <= h_v[31:0]; h <= H_DONE; end
                        `MIB_TRAPDSTPORT: begin trap_port <= h_v[15:0]; h <= H_DONE; end
                        `MIB_SNMPPORT: begin snmp_port_new <= h_v[15:0]; snmp_port_pend <= 1'b1; h <= H_DONE; end
                        `MIB_ERRTHRESHOLD: h <= H_DONE;
                        `MIB_LINKTIMEOUTMS: begin link_timeout_ms <= h_v[31:0]; h <= H_DONE; end
                        `MIB_RTCPORT: begin rtc_port <= h_v[15:0]; h <= H_DONE; end
                        `MIB_VLSHADOWBAGUS, `MIB_VLSHADOWLMAX, `MIB_VLSHADOWNETEN: begin
                            h_active <= 1'b1;
                            if (!shadow_ready) begin h_copy <= 1'b1; h <= H_COPY; end
                            else begin h_wait <= 3'd4; h <= H_READ; end
                        end
                        `MIB_CFGCOMMAND: begin
                            h_active <= 1'b1;
                            if (h_v[1:0] == 2'd1) begin h_verify <= 1'b1; h <= H_CMD; end
                            else if (h_v[1:0] == 2'd2) begin
                                if (cfg_prev_valid) begin h_roll <= 1'b1; h <= H_APPLY; end
                                else begin wr_err <= 1'b1; h <= H_DONE; end
                            end else begin h_copy <= 1'b1; h <= H_COPY; end
                        end
                        default: h <= H_DONE;
                    endcase
                end
                H_COPY: if (cfg_done) begin
                    shadow_ready <= 1'b1;
                    if (h_k == `MIB_CFGCOMMAND) h <= H_DONE;
                    else begin h_wait <= 3'd4; h <= H_READ; end
                end
                H_READ: begin
                    // 读出影子项（管理读口 2 拍）后只改写一个字段
                    if (h_wait != 3'd0) h_wait <= h_wait - 1'b1;
                    else begin
                        h_bag <= (h_k == `MIB_VLSHADOWBAGUS) ? h_v[19:0] : cfg_m_bag_us;
                        h_lmax <= (h_k == `MIB_VLSHADOWLMAX) ? h_v[10:0] : cfg_m_lmax;
                        h_net <= (h_k == `MIB_VLSHADOWNETEN) ? h_v[1:0] : cfg_m_net_en;
                        h <= H_WRITE;
                    end
                end
                H_WRITE: begin h_wr <= 1'b1; h <= H_DONE; end
                H_CMD: if (cfg_done) begin
                    if (!cfg_verify_ok) begin
                        cfg_last_result <= {29'd0, cfg_verify_code}; wr_err <= 1'b1; h <= H_DONE;
                    end else begin h_apply <= 1'b1; h <= H_APPLY; end
                end
                H_APPLY: if (cfg_done) begin
                    if (h_v[1:0] == 2'd1) begin ext_commit <= 1'b1; cfg_last_result <= 32'd0; end
                    else begin ext_rollback <= 1'b1; cfg_last_result <= 32'd5; end
                    shadow_ready <= 1'b0; ev_cfg <= 1'b1; h <= H_DONE;
                end
                H_DONE: begin wr_done <= 1'b1; h_active <= 1'b0; h <= H_IDLE; end
                default: h <= H_IDLE;
            endcase
        end
    end
    // 加载器应用/自检回滚产生配置事件并记录结果
    reg [2:0] ld_state_q;
    always @(posedge clk) ld_state_q <= ld_state;
    assign ld_self_fail = ld_state == 3'd6 && ld_state_q != 3'd6 && ld_error == 5'd14;
    assign ld_cfg_event = ld_apply || ld_self_fail;
    wire tftp_owner = own_tftp;
    wire [TX_VLW-1:0] val_idx;
    wire val_shadow;
    assign cfg_m_rd_idx = h_active ? h_idx : tftp_owner ? ld_cfg_idx : val_idx;
    // 无待提交修改时影子列显示活动值（提交/回滚后影子银行存放的是上一配置）
    assign cfg_m_rd_shadow = h_active ? 1'b1 : tftp_owner ? 1'b0 : (val_shadow && shadow_ready);
    assign cfg_m_wr_en = h_wr || ld_cfg_wr;
    assign cfg_m_wr_idx = h_wr ? h_idx : ld_cfg_wr_idx;
    assign cfg_m_wr_bag_us = h_wr ? h_bag : ld_cfg_bag;
    assign cfg_m_wr_lmax = h_wr ? h_lmax : ld_cfg_lmax;
    assign cfg_m_wr_net_en = h_wr ? h_net : ld_cfg_net;
    assign cfg_cmd_copy = h_copy || ld_copy;
    assign cfg_cmd_verify = h_verify || ld_verify;
    assign cfg_cmd_apply = h_apply || ld_apply;
    assign cfg_cmd_rollback = h_roll || ld_roll;

    // ---------------- 事件 / Trap ----------------
    // 事件源的比较与求和先寄存一拍
    wire [31:0] err_total = dp_stats[24*32 +: 32];
    reg buf_high;
    reg [31:0] drop_total;
    always @(posedge clk) begin
        buf_high <= dp_stats[20*32 +: 32] > BUF_HIGH || dp_stats[22*32 +: 32] > BUF_HIGH;
        drop_total <= dp_stats[5*32 +: 32] + dp_stats[15*32 +: 32];
    end
    afdx_mgmt_events u_events (
        .clk             (clk),
        .rst             (rst_l[8]),
        .mono_us         (mono_us),
        .rx_frames_a     (dp_stats[6*32 +: 32]),
        .rx_frames_b     (dp_stats[7*32 +: 32]),
        .err_total       (err_total),
        .buf_high        (buf_high),
        .drop_total      (drop_total),
        .ev_loader       (ld_ev || tftp_ev),
        .ev_rtc          (rtc_ev),
        .ev_cfg          (ev_cfg),
        .mask            (trap_mask),
        .interval_ms     (trap_interval_ms),
        .err_threshold   (err_threshold),
        .link_timeout_ms (link_timeout_ms),
        .pending         (ev_pending),
        .pick            (ev_pick),
        .take            (ev_take),
        .tv_type         (tv_type),
        .tv_count        (tv_count),
        .tv_first        (tv_first),
        .tv_last         (tv_last),
        .cnt_dropped     (cnt_trap_drop),
        .link_up         ()
    );

    // ---------------- MIB 取值 ----------------
    reg [31:0] uptime, up_us, sync_age;
    always @(posedge clk) sync_age <= mono_us - r_mono;     // rtcSyncAge，寄存后进入取值多路选择
    always @(posedge clk) begin
        if (rst_l[0]) begin uptime <= 32'd0; up_us <= 32'd0; end
        else if (tick_us) begin
            if (up_us == 32'd9999) begin up_us <= 32'd0; uptime <= uptime + 1'b1; end
            else up_us <= up_us + 1'b1;
        end
    end
    wire [511:0] ld_bus = {app_crc, cnt_unloads, cnt_loads, 32'd0, app_version, app_length, {31'd0, app_bank},
                           cfg_generation, {16'd0, ld_fail_block}, {27'd0, ld_error}, ld_crc_a, ld_crc_e,
                           {16'd0, cur_block}, ld_bytes, {29'd0, ld_object}, {29'd0, ld_state}};
    wire [863:0] rtc_bus = {r_mono, rtc64[31:0], rtc_oneway, rtc_unsync, rtc_holdover, rtc_step_th,
                            {24'd0, rtc_err}, {24'd0, r_quality}, {16'd0, rtc_src}, r_seq, r_rej, r_tmo,
                            r_bad, r_ooo, r_dup, r_lost, r_switch, r_slew, r_step, r_sync, sync_age,
                            rtc_freq, r_delay, r_maxoff, rtc_last_off, {29'd0, rtc_state}, rtc64[31:0]};
    wire [703:0] ft_bus = {cnt_tftp_err, cnt_tftp_retry, {16'd0, rtc_port}, link_timeout_ms, err_threshold,
                           cnt_snmp_out, cnt_snmp_perr, cnt_snmp_badc, cnt_snmp_in, cnt_rx_port, cnt_rx_full,
                           cnt_rx_accept, tv_last, tv_first, tv_count, {29'd0, tv_type}, cnt_trap_drop, cnt_trap,
                           {16'd0, snmp_port}, {16'd0, trap_port}, trap_interval_ms, trap_mask};
    afdx_snmp_values #(
        .VLW   (TX_VLW),
        .ES_ID ({16'd0, NET_ID, EQ_ID})
    ) u_values (
        .clk             (clk),
        .rst             (rst_l[9]),
        .val_k           (val_k),
        .val_inst        (val_inst),
        .val_data        (val_data),
        .stats_clear     (stats_clear),
        .dp              (dp_stats),
        .uptime_ticks    (uptime),
        .mono_us         (mono_us),
        .cfg_generation  (cfg_generation),
        .cfg_active_bank (cfg_active_bank),
        .cfg_last_result (cfg_last_result),
        .cfg_budget      (cfg_budget),
        .cfg_m_rd_idx    (val_idx),
        .cfg_m_rd_shadow (val_shadow),
        .cfg_vlid        (cfg_m_vlid),
        .cfg_bag_us      (cfg_m_bag_us),
        .cfg_lmax        (cfg_m_lmax),
        .cfg_net_en      (cfg_m_net_en),
        .cfg_rom_lmax    (cfg_m_rom_lmax),
        .ld              (ld_bus),
        .rtc64           (rtc64),
        .rtc             (rtc_bus),
        .ft              (ft_bus)
    );
    // 故障日志对象（64 字节）
    assign fault_vec = {32'h4146464C, mono_us, dp_stats[8*32 +: 32], dp_stats[9*32 +: 32], dp_stats[11*32 +: 32],
                        dp_stats[12*32 +: 32], dp_stats[14*32 +: 32], dp_stats[15*32 +: 32], dp_stats[16*32 +: 32],
                        dp_stats[17*32 +: 32], dp_stats[4*32 +: 32], cnt_trap_drop, r_bad, r_tmo, cnt_tftp_err,
                        {27'd0, ld_error}};
endmodule
