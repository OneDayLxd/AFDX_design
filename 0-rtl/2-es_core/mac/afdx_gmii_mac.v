/*
 * ================================================================
 * File Name    : afdx_gmii_mac.v
 * Module Name  : afdx_gmii_mac
 *
 * Function:
 *   125MHz 单时钟 MAC 适配：完整 TX 缓存、ready 限定发送使能。
 *   RX 保留末字节直到错误裁决，完整好帧才发布；支持下游反压。
 *   线路 SFD 时间戳、实际 TX 首尾反馈及长短帧/错误/溢出统计。
 *
 * Sub Module:
 *   gmii_mac_minimal
 *   afdx_async_frame_fifo
 *   afdx_rx_frame_fifo
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_gmii_mac #(
    parameter SLOT_LOG    = 2,
    parameter RX_SLOT_LOG = 11,
    parameter RX_BYTE_LOG = 17
) (
    input  wire        clk,
    input  wire        reset,
    input  wire [31:0] ts_us,
    input  wire [7:0]  tx_data,
    input  wire        tx_valid,
    input  wire        tx_last,
    output wire        tx_ready,
    output wire        tx_sof,
    output wire        tx_done,
    output wire [7:0]  rx_data,
    output wire        rx_valid,
    output wire        rx_last,
    output wire [31:0] rx_ts,
    input  wire        rx_ready,
    output reg  [31:0] rx_good_count,
    output reg  [31:0] rx_error_count,
    output reg  [31:0] rx_length_count,
    output reg  [31:0] rx_overflow_count,
    input  wire        eth_rx_er,
    input  wire        eth_rx_dv,
    input  wire [7:0]  eth_rxd,
    output wire        eth_tx_er,
    output wire        eth_tx_en,
    output wire [7:0]  eth_txd
);
    wire [7:0] tq_data, mac_rx_data;
    wire tq_valid, tq_last, mac_tx_ready;
    wire mac_rx_valid, mac_rx_last, mac_rx_error;
    reg [3:0] startup;
    reg wire_active;
    always @(posedge clk) begin
        if (reset) begin startup<=0; wire_active<=0; end
        else begin startup<={startup[2:0],1'b1}; wire_active<=eth_tx_en; end
    end
    wire tx_take = tq_valid && mac_tx_ready && startup[3] && !reset;
    assign tx_sof = eth_tx_en && !wire_active && !reset;
    assign tx_done = !eth_tx_en && wire_active && !reset;
    // 原始内核不支持反压时保持 valid；FIFO 仅在 ready 拍推进。
    afdx_async_frame_fifo #(
        .SLOT_LOG (SLOT_LOG),
        .MIN_LEN  (1)
    ) u_tx_fifo (
        .wr_clk        (clk),
        .wr_rst        (reset),
        .in_data       (tx_data),
        .in_valid      (tx_valid),
        .in_last       (tx_last),
        .in_error      (1'b0),
        .in_ts         (32'd0),
        .in_ready      (tx_ready),
        .stat_good     (),
        .stat_error    (),
        .stat_length   (),
        .stat_overflow (),
        .rd_clk        (clk),
        .rd_rst        (reset),
        .out_data      (tq_data),
        .out_valid     (tq_valid),
        .out_last      (tq_last),
        .out_ts        (),
        .out_ready     (mac_tx_ready && startup[3] && !reset)
    );
    gmii_mac_minimal u_mac (
        .rst_n              (~reset),
        .gtx_clk            (clk),
        .eth_rx_clk         (clk),
        .tx_axis_mac_tdata  (tq_data),
        .tx_axis_mac_tvalid (tx_take),
        .tx_axis_mac_tlast  (tx_take && tq_last),
        .tx_axis_mac_tready (mac_tx_ready),
        .tx_mac_aclk        (),
        .rx_mac_aclk        (),
        .eth_tx_clk         (),
        .eth_txd            (eth_txd),
        .eth_tx_en          (eth_tx_en),
        .eth_tx_er          (eth_tx_er),
        .eth_rxd            (eth_rxd),
        .eth_rx_dv          (eth_rx_dv),
        .eth_rx_er          (eth_rx_er),
        .rx_axis_mac_tdata  (mac_rx_data),
        .rx_axis_mac_tvalid (mac_rx_valid),
        .rx_axis_mac_tlast  (mac_rx_last),
        .rx_axis_mac_tuser  (mac_rx_error),
        .tx_frame_done      (),
        .tx_frame_error     (),
        .rx_good_frame      (),
        .rx_bad_frame       (),
        .rx_bad_fcs         ()
    );

    // 外层保留端系统的帧策略；原始内核会接受部分短前导码及超长帧。
    reg wire_rx_active, wire_sfd, wire_pre_bad, wire_phy_bad;
    reg [2:0] pre_count;
    reg [12:0] wire_count;
    reg [31:0] sfd_ts;
    reg ended_pre_bad, ended_length_bad, ended_phy_bad;
    reg pre_error, length_error;
    always @(posedge clk) begin
        if (reset) begin
            wire_rx_active<=0; wire_sfd<=0; wire_pre_bad<=0; wire_phy_bad<=0;
            pre_count<=0; wire_count<=0; sfd_ts<=0;
            ended_pre_bad<=0; ended_length_bad<=0; ended_phy_bad<=0; pre_error<=0; length_error<=0;
        end else begin
            wire_rx_active<=eth_rx_dv; pre_error<=0; length_error<=0;
            if (eth_rx_dv) begin
                // 内核忽略部分前导码 RX_ER；外层覆盖整个线路帧。
                wire_phy_bad<=eth_rx_er || (wire_rx_active && wire_phy_bad);
                if (!wire_rx_active) begin
                    wire_sfd<=0; wire_pre_bad<=eth_rxd!=8'h55;
                    pre_count<=eth_rxd==8'h55 ? 3'd1 : 3'd0; wire_count<=0;
                end else if (wire_sfd) begin
                    if (wire_count!=8191) wire_count<=wire_count+1'b1;
                end else if (!wire_pre_bad) begin
                    if (eth_rxd==8'h55) begin
                        if (pre_count!=7) pre_count<=pre_count+1'b1;
                    end else if (eth_rxd==8'hd5 && pre_count==7) begin
                        wire_sfd<=1; sfd_ts<=ts_us;
                    end else wire_pre_bad<=1;
                end
            end else if (wire_rx_active) begin
                ended_phy_bad<=wire_phy_bad;
                ended_pre_bad<=wire_pre_bad || !wire_sfd;
                ended_length_bad<=wire_sfd && (wire_count<64 || wire_count>1518);
                pre_error<=wire_pre_bad || !wire_sfd;
                length_error<=wire_sfd && (wire_count<64 || wire_count>1518);
            end
        end
    end

    reg held_valid, finish_pending, frame_bad;
    reg [7:0] held_data, rq_data;
    reg [31:0] frame_ts;
    reg rq_valid, rq_last, rq_error, stream_error;
    // LAST/USER 可能在 valid=0 时出现。末字节延后一拍，裁决前不提交描述符。
    always @(posedge clk) begin
        if (reset) begin
            held_valid<=0; held_data<=0; finish_pending<=0; frame_bad<=0;
            frame_ts<=0; rq_data<=0; rq_valid<=0; rq_last<=0; rq_error<=0; stream_error<=0;
        end else begin
            rq_valid<=0; rq_last<=0; rq_error<=0; stream_error<=0;
            if (finish_pending) begin
                rq_valid<=held_valid; rq_data<=held_data; rq_last<=1;
                rq_error<=frame_bad || ended_phy_bad || ended_pre_bad || ended_length_bad;
                stream_error<=(frame_bad || ended_phy_bad) && !ended_pre_bad && !ended_length_bad;
                held_valid<=0; finish_pending<=0; frame_bad<=0;
            end else begin
                if (mac_rx_valid) begin
                    if (!held_valid) frame_ts<=sfd_ts;
                    if (held_valid) begin rq_valid<=1; rq_data<=held_data; end
                    held_data<=mac_rx_data; held_valid<=1;
                    frame_bad<=frame_bad || mac_rx_error;
                end
                if (mac_rx_last) begin
                    finish_pending<=1;
                    frame_bad<=frame_bad || mac_rx_error;
                end
            end
        end
    end
    wire good, overflow;
    afdx_rx_frame_fifo #(
        .SLOT_LOG (RX_SLOT_LOG),
        .BYTE_LOG (RX_BYTE_LOG)
    ) u_rx_fifo (
        .clk           (clk),
        .reset         (reset),
        .in_data       (rq_data),
        .in_valid      (rq_valid),
        .in_last       (rq_last),
        .in_error      (rq_error),
        .in_ts         (frame_ts),
        .in_ready      (),
        .stat_good     (good),
        .stat_error    (),
        .stat_length   (),
        .stat_overflow (overflow),
        .out_data      (rx_data),
        .out_valid     (rx_valid),
        .out_last      (rx_last),
        .out_ts        (rx_ts),
        .out_ready     (rx_ready)
    );
    always @(posedge clk) begin
        if (reset) begin
            rx_good_count<=0; rx_error_count<=0; rx_length_count<=0; rx_overflow_count<=0;
        end else begin
            if (good) rx_good_count<=rx_good_count+1'b1;
            // stream_error 只计正常长度且无前导码错误的帧，避免重复计数。
            if (pre_error || stream_error) rx_error_count<=rx_error_count+1'b1;
            if (length_error) rx_length_count<=rx_length_count+1'b1;
            if (overflow) rx_overflow_count<=rx_overflow_count+1'b1;
        end
    end
endmodule
