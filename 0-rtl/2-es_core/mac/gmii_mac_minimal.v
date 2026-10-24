/*
 * ================================================================
 * File Name    : gmii_mac_minimal.v
 * Module Name  : gmii_mac_minimal
 *
 * Function:
 *   固定千兆全双工 GMII MAC，保留锁定参考的收发流水线和缓存行为。
 *   TX 由外层适配 ready 限定 valid；RX 无反压，末尾错误由外层整帧回滚。
 *   原始逻辑来源：gmii_mac/rtl/gmii_mac_minimal.v，SHA256:
 *   2aa9d71da3830c20d65c70415fe3ffca21aa232a6c19ac5b571b5cbb38256bf2
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module gmii_mac_minimal #(
    parameter MAX_FRAME_BYTES = 2048
) (
    input  wire       rst_n,
    input  wire       gtx_clk,
    input  wire       eth_rx_clk,
    input  wire [7:0] tx_axis_mac_tdata,
    input  wire       tx_axis_mac_tvalid,
    input  wire       tx_axis_mac_tlast,
    output wire       tx_axis_mac_tready,
    output wire       tx_mac_aclk,
    output wire       rx_mac_aclk,
    output wire       eth_tx_clk,
    output wire [7:0] eth_txd,
    output wire       eth_tx_en,
    output wire       eth_tx_er,
    input  wire [7:0] eth_rxd,
    input  wire       eth_rx_dv,
    input  wire       eth_rx_er,
    output wire [7:0] rx_axis_mac_tdata,
    output wire       rx_axis_mac_tvalid,
    output wire       rx_axis_mac_tlast,
    output wire       rx_axis_mac_tuser,
    output reg        tx_frame_done,
    output wire       tx_frame_error,
    output reg        rx_good_frame,
    output reg        rx_bad_frame,
    output reg        rx_bad_fcs
);
// Register power-up values mirror the reference. Runtime resets stay in their original domains.
reg tx_write_valid_d;
reg tx_ready_d;
reg rx_reset_meta;
reg rx_run;
reg rx_dv_d;
reg rx_dv_fall;
reg [47:0] rx_preamble_window;
reg rx_state;
reg rx_frame_started;
reg rx_frame_valid;
reg [8:0] rx_valid_pipe;
reg [4:0] rx_done_pipe;
reg [7:0] rx_input_data_d;
reg [7:0] rx_front_data;
reg [7:0] rx_crc_data;
reg [7:0] rx_data_d;
reg [4:0] rx_crc_done_pipe;
reg [31:0] rx_crc;
reg rx_crc_ok;
reg [31:0] rx_fcs_window;
reg [11:0] rx_byte_count;
reg [12:0] rx_length_high;
reg [2:0] rx_length_low;
reg rx_crc_checked;
reg rx_frame_valid_d;
reg rx_phy_error_d;
reg rx_phy_error_latched;
reg rx_error_flag;
reg tx_reset_meta;
reg tx_run;
reg tx_last_d;
reg tx_write_bank;
reg tx_release_d1;
reg tx_release_d2;
reg tx_read_bank;
reg tx_bank1_pending;
reg [7:0] tx_ifg_count;
reg [11:0] tx_byte_count;
reg [10:0] tx_write_address;
reg [11:0] tx_bank1_length;
reg [11:0] tx_bank0_length;
reg [11:0] tx_wire_length;
reg tx_frame_enable;
reg tx_release;
reg [15:0] rx_pause_info;
reg rx_pause_en;
reg [3:0] rx_pause_cnt;
reg [15:0] rx_pause_time;
reg [1:0] pause_cdc_write_address;
reg [1:0] pause_cdc_read_address;
reg tx_rx_mac_pause_en_d2;
reg tx_pause_in_req_pos;
reg [15:0] tx_pause_in_wait_time;
reg [21:0] tx_pause_in_wait_cnt;
reg tx_pause_in_waiting;
reg tx_send_done;
reg tx_release_pulse;
reg tx_bank0_pending;
reg tx_gmii_enable;
reg [2:0] tx_crc_valid_pipe;
reg [11:0] tx_frame_length;
reg [7:0] tx_write_data_d;
reg tx_read_enable;
reg [10:0] tx_read_address;
reg [31:0] tx_crc;
reg [31:0] tx_fcs_shift;
reg [7:0] tx_payload_d;
reg [7:0] tx_frame_data;
reg [7:0] tx_gmii_data;
reg [1:0] tx_state;
localparam TX_IDLE=2'd0, TX_WAIT_FRAME=2'd1, TX_SEND=2'd2, TX_IFG=2'd3;
localparam RX_SEARCH=1'b0, RX_FRAME=1'b1;
initial begin
    tx_write_valid_d = 0;
    tx_ready_d = 0;
    rx_reset_meta = 0;
    rx_run = 0;
    rx_dv_d = 0;
    rx_dv_fall = 0;
    rx_preamble_window = 0;
    rx_state = 0;
    rx_frame_started = 0;
    rx_frame_valid = 0;
    rx_valid_pipe = 0;
    rx_done_pipe = 0;
    rx_input_data_d = 0;
    rx_front_data = 0;
    rx_crc_data = 0;
    rx_data_d = 0;
    rx_crc_done_pipe = 0;
    rx_crc = 0;
    rx_crc_ok = 0;
    rx_fcs_window = 0;
    rx_byte_count = 0;
    rx_length_high = 0;
    rx_length_low = 0;
    rx_crc_checked = 0;
    rx_frame_valid_d = 0;
    rx_phy_error_d = 0;
    rx_phy_error_latched = 0;
    rx_error_flag = 0;
    tx_reset_meta = 0;
    tx_run = 0;
    tx_last_d = 0;
    tx_write_bank = 0;
    tx_release_d1 = 0;
    tx_release_d2 = 0;
    tx_read_bank = 0;
    tx_bank1_pending = 0;
    tx_ifg_count = 0;
    tx_byte_count = 0;
    tx_write_address = 0;
    tx_bank1_length = 0;
    tx_bank0_length = 0;
    tx_wire_length = 0;
    tx_frame_enable = 0;
    tx_release = 0;
    rx_pause_info = 0;
    rx_pause_en = 0;
    rx_pause_cnt = 0;
    rx_pause_time = 0;
    pause_cdc_write_address = 0;
    pause_cdc_read_address = 0;
    tx_rx_mac_pause_en_d2 = 0;
    tx_pause_in_req_pos = 0;
    tx_pause_in_wait_time = 0;
    tx_pause_in_wait_cnt = 0;
    tx_pause_in_waiting = 0;
    tx_send_done = 0;
    tx_release_pulse = 0;
    tx_bank0_pending = 0;
    tx_gmii_enable = 0;
    tx_crc_valid_pipe = 0;
    tx_frame_length = 0;
    tx_write_data_d = 0;
    tx_read_enable = 0;
    tx_read_address = 0;
    tx_crc = 0;
    tx_fcs_shift = 0;
    tx_payload_d = 0;
    tx_frame_data = 0;
    tx_gmii_data = 0;
    tx_state = TX_IDLE;
    tx_frame_done=0;rx_good_frame=0;rx_bad_frame=0;rx_bad_fcs=0;
end
wire tx_busy = (tx_state != TX_IDLE);
wire tx_wait_frame = (tx_state == TX_WAIT_FRAME);
wire tx_sending = (tx_state == TX_SEND);
wire tx_in_ifg = (tx_state == TX_IFG);
wire control_pause_time_d1_rtl_0_auto_generated_cntr1_modulus_trigger;
wire rx_preamble_match;
wire rx_front_idle;
wire rx_Add3_0;
wire rx_Add3_10;
wire rx_Add3_12;
wire rx_Add3_12COUT1_76;
wire rx_Add3_15;
wire rx_Add3_17;
wire rx_Add3_17COUT1_74;
wire rx_Add3_20;
wire rx_Add3_25;
wire rx_Add3_27;
wire rx_Add3_30;
wire rx_Add3_35;
wire rx_Add3_45;
wire rx_Add3_50;
wire rx_Add3_52;
wire rx_Add3_5;
wire rx_Equal13_0;
wire rx_Equal13_1;
wire rx_Equal13_2;
wire rx_Equal13_3;
wire rx_Equal13_5;
wire rx_Equal13_6;
wire rx_Equal13_7;
wire rx_Equal7_0;
wire rx_Equal7_2;
wire rx_Equal9_0;
wire rx_always13_0;
wire rx_always13_1;
wire rx_always13_3;
wire rx_always16_1;
wire rx_always16_2;
wire rx_always16_3;
wire rx_frame_length_10__1;
wire rx_mac_tuser_0;
wire rx_mac_tuser_1;
wire rx_mac_tuser_2;
wire rx_output_valid;
wire rx_crc_complete;
wire tx_buf_always0_0;
wire tx_buf_always1_0;
wire tx_buf_comb_0;
wire tx_bank_pending;
wire tx_buf_wr_ram_addr_7__0;
wire tx_Add3_0;
wire tx_Add3_10;
wire tx_Add3_12;
wire tx_Add3_15;
wire tx_Add3_17;
wire tx_Add3_20;
wire tx_Add3_22;
wire tx_Add3_25;
wire tx_Add3_27;
wire tx_Add3_30;
wire tx_Add3_32;
wire tx_Add3_35;
wire tx_Add3_40;
wire tx_Add3_42COUT1_108;
wire tx_Add3_45;
wire tx_Add3_47COUT1_110;
wire tx_Add3_50;
wire tx_Add3_52COUT1_112;
wire tx_Add3_55;
wire tx_Add3_57;
wire tx_Add3_5;
wire tx_Add3_60;
wire tx_Add3_62;
wire tx_Add3_65;
wire tx_Add3_7;
wire tx_Add3_70;
wire tx_Add5_0;
wire tx_Add5_10;
wire tx_Add5_15;
wire tx_Add5_20;
wire tx_Add5_25;
wire tx_Add5_30;
wire tx_Add5_35;
wire tx_Add5_40;
wire tx_Add5_5;
wire tx_Equal4_4;
wire tx_LessThan2_5;
wire tx_LessThan3_2;
wire tx_LessThan6_0;
wire tx_LessThan7_0;
wire tx_LessThan8_2;
wire tx_frame_available;
wire tx_always15_0;
wire tx_always15_1;
wire tx_always15_2;
wire tx_always15_3;
wire tx_always15_4;
wire tx_always15_5;
wire tx_always15_6;
wire tx_always19_0;
wire tx_not_sending;
wire tx_always20_2;
wire tx_crc_payload;
wire tx_mac_txd_17;
wire tx_mac_txd_3__20;
wire tx_mac_txd_3__21;
wire tx_mac_txd_7;
wire tx_ifg_incomplete;
wire tx_pause_in_wait_time_3__10;
wire tx_pause_in_wait_time_3__12;
wire tx_pause_in_wait_time_3__1;
wire tx_pause_in_wait_time_3__2;
wire tx_pause_in_wait_time_3__3;
wire tx_pause_in_wait_time_3__4;
wire tx_pause_in_wait_time_3__5;
wire tx_pause_in_wait_time_3__6;
wire tx_pause_in_wait_time_3__7;
wire tx_pause_in_wait_time_3__8;
wire tx_pause_in_wait_time_3__9;
wire tx_crc_reset_n;
wire tx_crc_lfsr_q_1__4;
wire tx_fcs_load;
wire tx_pause_timer_run;
wire tx_pause_timer_start;
wire [7:0] tx_preamble_byte;
wire [7:0] tx_sfd_byte;
wire [7:0] tx_payload_byte;
wire [7:0] tx_fcs_byte;
wire [7:0] tx_ram_data;
reg [7:0] rx_delay_q;
reg [16:0] pause_cdc_q;
// Combinational decode and interface signals.
assign control_pause_time_d1_rtl_0_auto_generated_cntr1_modulus_trigger = (pause_cdc_write_address[1] >= 1'd1);
assign rx_preamble_match = (rx_preamble_window == 48'h5555555555d5);
assign rx_front_idle = (~rx_run | ~rx_state);
assign rx_Add3_0 = (rx_byte_count[1] ? rx_byte_count[3] : ((((rx_byte_count[2] < 1'd1)) & ~rx_byte_count[3]) |
    (rx_byte_count[3] & ~((rx_byte_count[2] < 1'd1)))));
assign rx_Add3_10 = (rx_byte_count[1] ? ((rx_Add3_17COUT1_74 & ~rx_byte_count[5]) | (rx_byte_count[5] &
    ~rx_Add3_17COUT1_74)) : ((rx_Add3_17 & ~rx_byte_count[5]) | (rx_byte_count[5] & ~rx_Add3_17)));
assign rx_Add3_12 = (rx_byte_count[5:2] >= 4'd5);
assign rx_Add3_12COUT1_76 = (rx_byte_count[5:4] != 2'h0);
assign rx_Add3_15 = (rx_byte_count[4:1] >= 4'd1 && rx_byte_count[4:1] <= 4'd8);
assign rx_Add3_17 = (rx_byte_count[4:2] < 3'd5);
assign rx_Add3_17COUT1_74 = (rx_byte_count[4] < 1'd1);
assign rx_Add3_20 = ((rx_Add3_27 & ~rx_byte_count[7]) | (rx_byte_count[7] & ~rx_Add3_27));
assign rx_Add3_25 = (rx_byte_count[1] ? ((rx_Add3_12COUT1_76 & rx_byte_count[6]) | (~rx_Add3_12COUT1_76 &
    ~rx_byte_count[6])) : ((rx_Add3_12 & rx_byte_count[6]) | (~rx_Add3_12 & ~rx_byte_count[6])));
assign rx_Add3_27 = (rx_byte_count[6:1] < 6'd9);
assign rx_Add3_30 = (rx_Add3_27 ? ((((rx_byte_count[8:7] == 2'h0)) & ~rx_byte_count[9]) | (rx_byte_count[9] &
    ~((rx_byte_count[8:7] == 2'h0)))) : rx_byte_count[9]);
assign rx_Add3_35 = (rx_Add3_27 ? ((rx_byte_count[7] & rx_byte_count[8]) | (~rx_byte_count[7] & ~rx_byte_count[8]))
    : rx_byte_count[8]);
assign rx_Add3_45 = (rx_Add3_27 ? ((((rx_byte_count[9:7] != 3'h0)) & rx_byte_count[10]) | (~((rx_byte_count[9:7] !=
    3'h0)) & ~rx_byte_count[10])) : rx_byte_count[10]);
assign rx_Add3_50 = (rx_Add3_27 ? ((((rx_byte_count[10:7] == 4'h0)) & ~rx_byte_count[11]) | (rx_byte_count[11] &
    ~((rx_byte_count[10:7] == 4'h0)))) : rx_byte_count[11]);
assign rx_Add3_52 = (rx_byte_count[11:1] >= 11'd9);
assign rx_Add3_5 = ((rx_byte_count[1] & rx_byte_count[2]) | (~rx_byte_count[1] & ~rx_byte_count[2]));
assign rx_Equal13_0 = ((rx_length_low[0] & rx_length_low[1] & rx_byte_count[0] & ~rx_byte_count[1]) |
    (rx_length_low[0] & rx_byte_count[0] & rx_byte_count[1] & ~rx_length_low[1]) | (rx_length_low[1] &
    ~rx_length_low[0] & ~rx_byte_count[0] & ~rx_byte_count[1]) | (rx_byte_count[1] & ~rx_length_low[0] &
    ~rx_length_low[1] & ~rx_byte_count[0]));
assign rx_Equal13_1 = ((rx_Add3_0 & rx_Add3_5 & rx_length_high[0] & rx_length_low[2]) | (rx_Add3_0 &
    rx_length_high[0] & ~rx_Add3_5 & ~rx_length_low[2]) | (rx_Add3_5 & rx_length_low[2] & ~rx_Add3_0 &
    ~rx_length_high[0]) | (~rx_Add3_0 & ~rx_Add3_5 & ~rx_length_high[0] & ~rx_length_low[2]));
assign rx_Equal13_2 = ((rx_Add3_10 & rx_Add3_15 & rx_length_high[1] & rx_length_high[2]) | (rx_Add3_10 &
    rx_length_high[2] & ~rx_Add3_15 & ~rx_length_high[1]) | (rx_Add3_15 & rx_length_high[1] & ~rx_Add3_10 &
    ~rx_length_high[2]) | (~rx_Add3_10 & ~rx_Add3_15 & ~rx_length_high[1] & ~rx_length_high[2]));
assign rx_Equal13_3 = ((rx_Add3_20 & rx_Add3_25 & rx_length_high[3] & rx_length_high[4]) | (rx_Add3_20 &
    rx_length_high[4] & ~rx_Add3_25 & ~rx_length_high[3]) | (rx_Add3_25 & rx_length_high[3] & ~rx_Add3_20 &
    ~rx_length_high[4]) | (~rx_Add3_20 & ~rx_Add3_25 & ~rx_length_high[3] & ~rx_length_high[4]));
assign rx_Equal13_5 = ((rx_Add3_30 & rx_Add3_35 & rx_length_high[5] & rx_length_high[6]) | (rx_Add3_30 &
    rx_length_high[6] & ~rx_Add3_35 & ~rx_length_high[5]) | (rx_Add3_35 & rx_length_high[5] & ~rx_Add3_30 &
    ~rx_length_high[6]) | (~rx_Add3_30 & ~rx_Add3_35 & ~rx_length_high[5] & ~rx_length_high[6]));
assign rx_Equal13_6 = ((rx_Add3_45 & rx_Add3_52 & rx_length_high[7] & ~rx_Add3_50) | (rx_Add3_52 & ~rx_Add3_45 &
    ~rx_Add3_50 & ~rx_length_high[7]));
assign rx_Equal13_7 = (((rx_Equal13_0 & rx_Equal13_1 & rx_Equal13_2 & rx_Equal13_3)) & rx_Equal13_5 & rx_Equal13_6);
assign rx_Equal7_0 = (rx_byte_count[4] & ~rx_byte_count[0] & ~rx_byte_count[2] & ~rx_byte_count[3]);
assign rx_Equal7_2 = (rx_byte_count[11:5] == 7'h0);
assign rx_Equal9_0 = (rx_pause_cnt != 4'hf);
assign rx_always13_0 = ({rx_length_low[2:1], rx_length_high[6:1]} == 8'h0);
assign rx_always13_1 = ((rx_length_high[8] & ~rx_length_high[12]) | (rx_length_high[12] & rx_length_low[0]) |
    (rx_length_high[0] & ~rx_length_high[8]) | (~rx_length_high[8] & ~rx_length_low[0]) | (~rx_length_high[0] &
    ~rx_length_low[0]));
assign rx_always13_3 = (((rx_always13_1 | ~rx_crc_checked | ~rx_crc_ok)) | rx_length_high[7] | ~rx_always13_0 |
    ~rx_mac_tuser_1);
assign rx_always16_1 = (((~rx_length_high[4] & ~rx_length_high[5] & ~rx_length_low[1] & ~rx_length_low[2])) &
    ~rx_length_high[1] & ~rx_length_high[2] & ~rx_length_high[3]);
assign rx_always16_2 = ((rx_always16_1 & ~rx_length_high[0] & ~rx_length_low[0]) | ~rx_length_high[6]);
assign rx_always16_3 = ((rx_always16_2 & rx_length_high[7]) | (rx_length_high[0] & ~rx_length_high[7]) |
    (~rx_always13_0 & ~rx_length_high[7]));
assign rx_frame_length_10__1 = ((rx_Equal7_0 & rx_Equal7_2 & ~rx_byte_count[1]) | ~rx_run);
assign rx_mac_tuser_0 = ((rx_crc_checked & ~rx_crc_ok) | rx_phy_error_latched);
assign rx_mac_tuser_1 = (rx_length_high[11:9] == 3'h0);
assign rx_mac_tuser_2 = (rx_always16_3 & rx_mac_tuser_1 & ~rx_length_high[8] & ~rx_length_high[12]);
assign rx_output_valid = (rx_valid_pipe[4] & rx_valid_pipe[8]);
assign rx_crc_complete = (rx_crc_done_pipe[0] | rx_crc_done_pipe[1] | rx_crc_done_pipe[2] | rx_crc_done_pipe[3]);
assign tx_buf_always0_0 = (tx_write_bank | ~tx_axis_mac_tlast);
assign tx_buf_always1_0 = (~tx_axis_mac_tlast | ~tx_write_bank);
assign tx_buf_comb_0 = (tx_ready_d & tx_write_valid_d);
assign tx_bank_pending = (tx_bank0_pending | tx_bank1_pending);
assign tx_buf_wr_ram_addr_7__0 = (tx_run & ~tx_last_d);
assign tx_Add3_0 = ((tx_pause_in_wait_time[0] & tx_pause_in_wait_time[1]) | (~tx_pause_in_wait_time[0] &
    ~tx_pause_in_wait_time[1]));
assign tx_Add3_10 = ((tx_Add3_7 & tx_pause_in_wait_time[3]) | (~tx_Add3_7 & ~tx_pause_in_wait_time[3]));
assign tx_Add3_12 = (tx_pause_in_wait_time[3] < 1'd1);
assign tx_Add3_15 = (tx_Add3_7 ? tx_pause_in_wait_time[4] : ((tx_Add3_12 & ~tx_pause_in_wait_time[4]) |
    (tx_pause_in_wait_time[4] & ~tx_Add3_12)));
assign tx_Add3_17 = (tx_pause_in_wait_time[4:3] != 2'h0);
assign tx_Add3_20 = (tx_Add3_7 ? tx_pause_in_wait_time[5] : ((tx_Add3_17 & tx_pause_in_wait_time[5]) | (~tx_Add3_17
    & ~tx_pause_in_wait_time[5])));
assign tx_Add3_22 = (tx_pause_in_wait_time[5:3] == 3'h0);
assign tx_Add3_25 = (tx_Add3_7 ? tx_pause_in_wait_time[6] : ((tx_Add3_22 & ~tx_pause_in_wait_time[6]) |
    (tx_pause_in_wait_time[6] & ~tx_Add3_22)));
assign tx_Add3_27 = (tx_pause_in_wait_time[6:3] != 4'h0);
assign tx_Add3_30 = (tx_Add3_7 ? tx_pause_in_wait_time[7] : ((tx_Add3_27 & tx_pause_in_wait_time[7]) | (~tx_Add3_27
    & ~tx_pause_in_wait_time[7])));
assign tx_Add3_32 = (tx_pause_in_wait_time[7:0] == 8'h0);
assign tx_Add3_35 = ((tx_Add3_32 & ~tx_pause_in_wait_time[8]) | (tx_pause_in_wait_time[8] & ~tx_Add3_32));
assign tx_Add3_40 = (tx_Add3_32 ? ((tx_pause_in_wait_time[8] & tx_pause_in_wait_time[9]) |
    (~tx_pause_in_wait_time[8] & ~tx_pause_in_wait_time[9])) : tx_pause_in_wait_time[9]);
assign tx_Add3_42COUT1_108 = (tx_pause_in_wait_time[9:8] == 2'h0);
assign tx_Add3_45 = (tx_Add3_32 ? ((tx_Add3_42COUT1_108 & ~tx_pause_in_wait_time[10]) | (tx_pause_in_wait_time[10]
    & ~tx_Add3_42COUT1_108)) : tx_pause_in_wait_time[10]);
assign tx_Add3_47COUT1_110 = (tx_pause_in_wait_time[10:8] != 3'h0);
assign tx_Add3_50 = (tx_Add3_32 ? ((tx_Add3_47COUT1_110 & tx_pause_in_wait_time[11]) | (~tx_Add3_47COUT1_110 &
    ~tx_pause_in_wait_time[11])) : tx_pause_in_wait_time[11]);
assign tx_Add3_52COUT1_112 = (tx_pause_in_wait_time[11:8] == 4'h0);
assign tx_Add3_55 = (tx_Add3_32 ? ((tx_Add3_52COUT1_112 & ~tx_pause_in_wait_time[12]) | (tx_pause_in_wait_time[12]
    & ~tx_Add3_52COUT1_112)) : tx_pause_in_wait_time[12]);
assign tx_Add3_57 = (tx_pause_in_wait_time[12:0] != 13'h0);
assign tx_Add3_5 = ((((tx_pause_in_wait_time[1:0] == 2'h0)) & ~tx_pause_in_wait_time[2]) |
    (tx_pause_in_wait_time[2] & ~((tx_pause_in_wait_time[1:0] == 2'h0))));
assign tx_Add3_60 = ((tx_Add3_57 & tx_pause_in_wait_time[13]) | (~tx_Add3_57 & ~tx_pause_in_wait_time[13]));
assign tx_Add3_62 = (tx_pause_in_wait_time[13] < 1'd1);
assign tx_Add3_65 = (tx_Add3_57 ? tx_pause_in_wait_time[14] : ((tx_Add3_62 & ~tx_pause_in_wait_time[14]) |
    (tx_pause_in_wait_time[14] & ~tx_Add3_62)));
assign tx_Add3_7 = (tx_pause_in_wait_time[2:0] != 3'h0);
assign tx_Add3_70 = (tx_Add3_57 ? tx_pause_in_wait_time[15] : (((tx_pause_in_wait_time[14:13] != 2'h0)) ^
    tx_pause_in_wait_time[15] ^ 1'b1));
assign tx_Add5_0 = (tx_wire_length[4:2] >= 3'd1 && tx_wire_length[4:2] <= 3'd4);
assign tx_Add5_10 = (tx_wire_length[6:2] >= 5'd13 && tx_wire_length[6:2] <= 5'd28);
assign tx_Add5_15 = (tx_wire_length[5:2] >= 4'd5 && tx_wire_length[5:2] <= 4'd12);
assign tx_Add5_20 = (tx_wire_length[8:2] >= 7'd61 && tx_wire_length[8:2] <= 7'd124);
assign tx_Add5_25 = (tx_wire_length[7:2] >= 6'd29 && tx_wire_length[7:2] <= 6'd60);
assign tx_Add5_30 = (tx_wire_length[10:2] >= 9'd253 && tx_wire_length[10:2] <= 9'd508);
assign tx_Add5_35 = (tx_wire_length[9:2] >= 8'd125 && tx_wire_length[9:2] <= 8'd252);
assign tx_Add5_40 = (tx_wire_length[11:2] >= 10'd509 && tx_wire_length[11:2] <= 10'd1020);
assign tx_Add5_5 = ((tx_wire_length[2] & tx_wire_length[3]) | (~tx_wire_length[2] & ~tx_wire_length[3]));
assign tx_Equal4_4 = (tx_pause_in_wait_time == 16'h0);
assign tx_LessThan2_5 = (tx_pause_in_wait_cnt != 22'h0);
assign tx_LessThan3_2 = (tx_byte_count[11:4] == 8'h0);
assign tx_LessThan6_0 = (tx_byte_count < (tx_wire_length + 12'd10));
assign tx_LessThan7_0 = (tx_byte_count > (tx_wire_length + 12'd13));
assign tx_LessThan8_2 = (tx_byte_count[11:1] != 11'h0);
assign tx_frame_available = ((tx_bank0_pending & ~tx_pause_in_waiting) | (tx_bank1_pending & ~tx_pause_in_waiting));
assign tx_always15_0 = ((tx_wire_length[1] & tx_wire_length[2] & tx_byte_count[1] & ~tx_byte_count[2]) |
    (tx_wire_length[1] & tx_byte_count[1] & tx_byte_count[2] & ~tx_wire_length[2]) | (tx_wire_length[2] &
    ~tx_wire_length[1] & ~tx_byte_count[1] & ~tx_byte_count[2]) | (tx_byte_count[2] & ~tx_wire_length[1] &
    ~tx_wire_length[2] & ~tx_byte_count[1]));
assign tx_always15_1 = ((tx_always15_0 & tx_wire_length[0] & tx_byte_count[0] & ~tx_not_sending) | (tx_always15_0 &
    ~tx_not_sending & ~tx_wire_length[0] & ~tx_byte_count[0]));
assign tx_always15_2 = ((tx_Add5_0 & tx_Add5_5 & tx_byte_count[3] & tx_byte_count[4]) | (tx_Add5_0 &
    tx_byte_count[4] & ~tx_Add5_5 & ~tx_byte_count[3]) | (tx_Add5_5 & tx_byte_count[3] & ~tx_Add5_0 &
    ~tx_byte_count[4]) | (~tx_Add5_0 & ~tx_Add5_5 & ~tx_byte_count[3] & ~tx_byte_count[4]));
assign tx_always15_3 = ((tx_Add5_10 & tx_Add5_15 & tx_byte_count[5] & tx_byte_count[6]) | (tx_Add5_10 &
    tx_byte_count[6] & ~tx_Add5_15 & ~tx_byte_count[5]) | (tx_Add5_15 & tx_byte_count[5] & ~tx_Add5_10 &
    ~tx_byte_count[6]) | (~tx_Add5_10 & ~tx_Add5_15 & ~tx_byte_count[5] & ~tx_byte_count[6]));
assign tx_always15_4 = ((tx_Add5_20 & tx_Add5_25 & tx_byte_count[7] & tx_byte_count[8]) | (tx_Add5_20 &
    tx_byte_count[8] & ~tx_Add5_25 & ~tx_byte_count[7]) | (tx_Add5_25 & tx_byte_count[7] & ~tx_Add5_20 &
    ~tx_byte_count[8]) | (~tx_Add5_20 & ~tx_Add5_25 & ~tx_byte_count[7] & ~tx_byte_count[8]));
assign tx_always15_5 = ((tx_Add5_30 & tx_Add5_35 & tx_byte_count[10] & tx_byte_count[9]) | (tx_Add5_30 &
    tx_byte_count[10] & ~tx_Add5_35 & ~tx_byte_count[9]) | (tx_Add5_35 & tx_byte_count[9] & ~tx_Add5_30 &
    ~tx_byte_count[10]) | (~tx_Add5_30 & ~tx_Add5_35 & ~tx_byte_count[10] & ~tx_byte_count[9]));
assign tx_always15_6 = ((tx_Add5_40 & tx_always15_4 & tx_always15_5 & tx_byte_count[11]) | (tx_always15_4 &
    tx_always15_5 & ~tx_Add5_40 & ~tx_byte_count[11]));
assign tx_always19_0 = (tx_LessThan6_0 | tx_LessThan7_0);
assign tx_not_sending = (tx_sending < 1'd1);
assign tx_always20_2 = (tx_byte_count[3:0] >= 4'd9);
assign tx_crc_payload = (((tx_byte_count < (tx_frame_length + 12'd11))) & tx_sending);
assign tx_mac_txd_17 = (tx_byte_count == 12'h9);
assign tx_mac_txd_3__20 = ((tx_byte_count[1] & tx_byte_count[3]) | (tx_byte_count[2] & tx_byte_count[3]) |
    (~tx_byte_count[1] & ~tx_byte_count[2] & ~tx_byte_count[3]) | ~tx_LessThan3_2);
assign tx_mac_txd_3__21 = (tx_byte_count[11:1] >= 11'd5);
assign tx_mac_txd_7 = ((tx_mac_txd_3__21 & ~tx_LessThan6_0 & ~tx_LessThan7_0) | (tx_mac_txd_3__21 & ~tx_not_sending));
assign tx_ifg_incomplete = (tx_ifg_count != 8'hc);
assign tx_pause_in_wait_time_3__10 = (tx_pause_in_wait_time_3__6 & tx_pause_in_wait_time_3__7 &
    tx_pause_in_wait_time_3__8 & tx_pause_in_wait_time_3__9);
assign tx_pause_in_wait_time_3__12 = (tx_pause_in_wait_cnt[0] & tx_pause_in_wait_cnt[1] &
    tx_pause_in_wait_time_3__10 & ((tx_pause_in_wait_cnt[5:2] == 4'hf)));
assign tx_pause_in_wait_time_3__1 = ((tx_Add3_0 & tx_pause_in_wait_cnt[6] & tx_pause_in_wait_cnt[7] &
    ~tx_pause_in_wait_time[0]) | (tx_Add3_0 & tx_pause_in_wait_cnt[7] & tx_pause_in_wait_time[0] &
    ~tx_pause_in_wait_cnt[6]) | (tx_pause_in_wait_cnt[6] & ~tx_Add3_0 & ~tx_pause_in_wait_cnt[7] &
    ~tx_pause_in_wait_time[0]) | (tx_pause_in_wait_time[0] & ~tx_Add3_0 & ~tx_pause_in_wait_cnt[6] &
    ~tx_pause_in_wait_cnt[7]));
assign tx_pause_in_wait_time_3__2 = ((tx_Add3_10 & tx_Add3_5 & tx_pause_in_wait_cnt[8] & tx_pause_in_wait_cnt[9]) |
    (tx_Add3_10 & tx_pause_in_wait_cnt[9] & ~tx_Add3_5 & ~tx_pause_in_wait_cnt[8]) | (tx_Add3_5 &
    tx_pause_in_wait_cnt[8] & ~tx_Add3_10 & ~tx_pause_in_wait_cnt[9]) | (~tx_Add3_10 & ~tx_Add3_5 &
    ~tx_pause_in_wait_cnt[8] & ~tx_pause_in_wait_cnt[9]));
assign tx_pause_in_wait_time_3__3 = ((tx_Add3_15 & tx_Add3_20 & tx_pause_in_wait_cnt[10] &
    tx_pause_in_wait_cnt[11]) | (tx_Add3_15 & tx_pause_in_wait_cnt[10] & ~tx_Add3_20 & ~tx_pause_in_wait_cnt[11]) |
    (tx_Add3_20 & tx_pause_in_wait_cnt[11] & ~tx_Add3_15 & ~tx_pause_in_wait_cnt[10]) | (~tx_Add3_15 & ~tx_Add3_20
    & ~tx_pause_in_wait_cnt[10] & ~tx_pause_in_wait_cnt[11]));
assign tx_pause_in_wait_time_3__4 = ((tx_Add3_25 & tx_Add3_30 & tx_pause_in_wait_cnt[12] &
    tx_pause_in_wait_cnt[13]) | (tx_Add3_25 & tx_pause_in_wait_cnt[12] & ~tx_Add3_30 & ~tx_pause_in_wait_cnt[13]) |
    (tx_Add3_30 & tx_pause_in_wait_cnt[13] & ~tx_Add3_25 & ~tx_pause_in_wait_cnt[12]) | (~tx_Add3_25 & ~tx_Add3_30
    & ~tx_pause_in_wait_cnt[12] & ~tx_pause_in_wait_cnt[13]));
assign tx_pause_in_wait_time_3__5 = (tx_pause_in_wait_time_3__1 & tx_pause_in_wait_time_3__2 &
    tx_pause_in_wait_time_3__3 & tx_pause_in_wait_time_3__4);
assign tx_pause_in_wait_time_3__6 = ((tx_Add3_35 & tx_Add3_40 & tx_pause_in_wait_cnt[14] &
    tx_pause_in_wait_cnt[15]) | (tx_Add3_35 & tx_pause_in_wait_cnt[14] & ~tx_Add3_40 & ~tx_pause_in_wait_cnt[15]) |
    (tx_Add3_40 & tx_pause_in_wait_cnt[15] & ~tx_Add3_35 & ~tx_pause_in_wait_cnt[14]) | (~tx_Add3_35 & ~tx_Add3_40
    & ~tx_pause_in_wait_cnt[14] & ~tx_pause_in_wait_cnt[15]));
assign tx_pause_in_wait_time_3__7 = ((tx_Add3_45 & tx_Add3_50 & tx_pause_in_wait_cnt[16] &
    tx_pause_in_wait_cnt[17]) | (tx_Add3_45 & tx_pause_in_wait_cnt[16] & ~tx_Add3_50 & ~tx_pause_in_wait_cnt[17]) |
    (tx_Add3_50 & tx_pause_in_wait_cnt[17] & ~tx_Add3_45 & ~tx_pause_in_wait_cnt[16]) | (~tx_Add3_45 & ~tx_Add3_50
    & ~tx_pause_in_wait_cnt[16] & ~tx_pause_in_wait_cnt[17]));
assign tx_pause_in_wait_time_3__8 = ((tx_Add3_55 & tx_Add3_60 & tx_pause_in_wait_cnt[18] &
    tx_pause_in_wait_cnt[19]) | (tx_Add3_55 & tx_pause_in_wait_cnt[18] & ~tx_Add3_60 & ~tx_pause_in_wait_cnt[19]) |
    (tx_Add3_60 & tx_pause_in_wait_cnt[19] & ~tx_Add3_55 & ~tx_pause_in_wait_cnt[18]) | (~tx_Add3_55 & ~tx_Add3_60
    & ~tx_pause_in_wait_cnt[18] & ~tx_pause_in_wait_cnt[19]));
assign tx_pause_in_wait_time_3__9 = ((tx_Add3_65 & tx_Add3_70 & tx_pause_in_wait_cnt[20] &
    tx_pause_in_wait_cnt[21]) | (tx_Add3_65 & tx_pause_in_wait_cnt[20] & ~tx_Add3_70 & ~tx_pause_in_wait_cnt[21]) |
    (tx_Add3_70 & tx_pause_in_wait_cnt[21] & ~tx_Add3_65 & ~tx_pause_in_wait_cnt[20]) | (~tx_Add3_65 & ~tx_Add3_70
    & ~tx_pause_in_wait_cnt[20] & ~tx_pause_in_wait_cnt[21]));
assign tx_crc_reset_n = ((tx_run & tx_crc_valid_pipe[1]) | (tx_run & ~tx_crc_valid_pipe[2]));
assign tx_crc_lfsr_q_1__4 = ((((tx_byte_count < (tx_wire_length + 12'd9))) & tx_always20_2) | (((tx_byte_count <
    (tx_wire_length + 12'd9))) & ~tx_LessThan3_2) | ~tx_crc_reset_n);
assign tx_axis_mac_tready = ((tx_buf_wr_ram_addr_7__0 & ~tx_bank0_pending) | (tx_buf_wr_ram_addr_7__0 &
    ~tx_bank1_pending));
assign tx_mac_aclk = gtx_clk;
assign rx_mac_aclk = eth_rx_clk;
assign eth_tx_clk = ~gtx_clk;
assign eth_txd = tx_gmii_data;
assign eth_tx_en = tx_gmii_enable;
assign eth_tx_er = 1'b0;
assign rx_axis_mac_tdata = (rx_output_valid ? rx_delay_q : 8'd0);
assign rx_axis_mac_tvalid = (rx_valid_pipe[4] & rx_valid_pipe[8]);
assign rx_axis_mac_tlast = rx_done_pipe[4];
assign rx_axis_mac_tuser = rx_error_flag;
// TX four-state controller. The reference stores IDLE with inverted polarity.
wire tx_state_reset = ~tx_run;
always @(posedge gtx_clk or posedge tx_state_reset) begin
    if (tx_state_reset) tx_state <= TX_IDLE;
    else case (tx_state)
        TX_IDLE:       tx_state <= TX_WAIT_FRAME;
        TX_WAIT_FRAME: if (tx_frame_available) tx_state <= TX_SEND;
        TX_SEND:       if (tx_send_done) tx_state <= TX_IFG;
        TX_IFG:        if (!tx_ifg_incomplete) tx_state <= TX_IDLE;
        default:       tx_state <= TX_IDLE;
    endcase
end
// RX search/receive state: latch the detected preamble until RX_DV falls.
always @(posedge eth_rx_clk) begin
    if (!rx_run) rx_state <= RX_SEARCH;
    else case (rx_state)
        RX_SEARCH: if (rx_preamble_match && !rx_dv_fall) rx_state <= RX_FRAME;
        RX_FRAME:  if (rx_dv_fall) rx_state <= RX_SEARCH;
    endcase
end
// ---- Clock-domain reset pipelines ----
// rx_reset_meta
always @(posedge eth_rx_clk) begin
    rx_reset_meta <= rst_n;
end
// rx_run
always @(posedge eth_rx_clk) begin
    rx_run <= (rx_reset_meta & rst_n);
end
// tx_reset_meta
always @(posedge gtx_clk) begin
    tx_reset_meta <= rst_n;
end
// tx_run
always @(posedge gtx_clk) begin
    tx_run <= (tx_reset_meta & rst_n);
end
// ---- TX ping-pong buffer and frame descriptors ----
// tx_write_valid_d
always @(posedge gtx_clk) begin
    tx_write_valid_d <= (tx_run & tx_axis_mac_tvalid & ~tx_last_d);
end
// tx_ready_d
always @(posedge gtx_clk) begin
    tx_ready_d <= ((tx_buf_wr_ram_addr_7__0 & ~tx_bank0_pending) | (tx_buf_wr_ram_addr_7__0 & ~tx_bank1_pending));
end
// tx_last_d
always @(posedge gtx_clk) begin
    tx_last_d <= (tx_run & tx_axis_mac_tlast);
end
// tx_write_bank
always @(posedge gtx_clk) begin
    tx_write_bank <= ((tx_last_d ^ tx_write_bank) & tx_run);
end
// tx_release_d1
always @(posedge gtx_clk) begin
    tx_release_d1 <= tx_release;
end
// tx_release_d2
always @(posedge gtx_clk) begin
    tx_release_d2 <= tx_release_d1;
end
// tx_read_bank
always @(posedge gtx_clk) begin
    tx_read_bank <= (tx_run & ((tx_read_bank & tx_release_d2) | (tx_read_bank & ~tx_release_d1) | (tx_release_d1 &
        ~tx_read_bank & ~tx_release_d2)));
end
// tx_bank1_pending
always @(posedge gtx_clk) begin
    tx_bank1_pending <= (tx_run & ((tx_bank1_pending & ~tx_read_bank) | (tx_bank1_pending & ~tx_release_pulse) |
        (~tx_buf_always1_0 & ~tx_read_bank) | (~tx_buf_always1_0 & ~tx_release_pulse)));
end
// tx_write_address
always @(posedge gtx_clk) begin
    if (((tx_axis_mac_tvalid | tx_last_d | ~tx_run))) begin
        tx_write_address <= ((tx_run & ~tx_last_d) ? (tx_write_address + 1'b1) : 11'd0);
    end
end
// tx_bank1_length
always @(posedge gtx_clk) begin
    if ((((tx_axis_mac_tlast & tx_write_bank) | ~tx_run))) begin
        tx_bank1_length <= tx_run ? ({1'b0, tx_write_address} + 12'd1) : 12'd0;
    end
end
// tx_bank0_length
always @(posedge gtx_clk) begin
    if ((((tx_axis_mac_tlast & ~tx_write_bank) | ~tx_run))) begin
        tx_bank0_length <= tx_run ? ({1'b0, tx_write_address} + 12'd1) : 12'd0;
    end
end
// tx_release
always @(posedge gtx_clk) begin
    tx_release <= tx_in_ifg;
end
// tx_release_pulse
always @(posedge gtx_clk) begin
    tx_release_pulse <= (tx_release & ~tx_release_d1);
end
// tx_bank0_pending
always @(posedge gtx_clk) begin
    tx_bank0_pending <= (tx_run & ((tx_read_bank & tx_bank0_pending) | (tx_read_bank & ~tx_buf_always0_0) |
        (tx_bank0_pending & ~tx_release_pulse) | (~tx_buf_always0_0 & ~tx_release_pulse)));
end
// tx_write_data_d
always @(posedge gtx_clk) begin
    tx_write_data_d <= ((tx_run & tx_axis_mac_tvalid & ~tx_last_d) ? tx_axis_mac_tdata : 8'd0);
end
// tx_read_address
always @(posedge gtx_clk) begin
    tx_read_address <= (tx_read_enable ? (tx_read_address + 1'b1) : 11'd0);
end
// ---- TX counters, CRC and GMII data pipeline ----
// tx_ifg_count
always @(posedge gtx_clk) begin
    tx_ifg_count <= (~((tx_in_ifg < 1'd1)) ? (tx_ifg_count + 1'b1) : 8'd0);
end
// tx_byte_count
always @(posedge gtx_clk) begin
    tx_byte_count <= (~tx_not_sending ? (tx_byte_count + 1'b1) : 12'd0);
end
// tx_wire_length
always @(posedge gtx_clk) begin
    if (((tx_bank0_pending | tx_bank1_pending | ~tx_run))) begin
        tx_wire_length <= !tx_run ? 12'd0 : ((!tx_bank_pending || (tx_read_bank ? tx_bank1_length :
            tx_bank0_length) < 12'd60) ? 12'd60 : (tx_read_bank ? tx_bank1_length : tx_bank0_length));
    end
end
// tx_frame_enable
always @(posedge gtx_clk) begin
    tx_frame_enable <= ((tx_LessThan8_2 & ~tx_always19_0) | (tx_LessThan8_2 & ~tx_not_sending) | (tx_LessThan8_2 &
        ~tx_mac_txd_3__20));
end
// tx_send_done
always @(posedge gtx_clk) begin
    tx_send_done <= (tx_always15_1 & tx_always15_2 & tx_always15_3 & tx_always15_6);
end
// tx_gmii_enable
always @(posedge gtx_clk) begin
    tx_gmii_enable <= tx_frame_enable;
end
// tx_crc_valid_pipe
always @(posedge gtx_clk) begin
    tx_crc_valid_pipe <= {tx_crc_valid_pipe[1:0], tx_frame_enable};
end
// tx_frame_length
always @(posedge gtx_clk) begin
    tx_frame_length <= tx_bank_pending ? (tx_read_bank ? tx_bank1_length : tx_bank0_length) : 12'd0;
end
// tx_read_enable
always @(posedge gtx_clk) begin
    tx_read_enable <= (((tx_byte_count < (tx_wire_length + 12'd6))) & ((tx_byte_count < (tx_frame_length + 12'd6)))
        & ~((tx_byte_count[11:1] < 11'd3)) & ~tx_not_sending);
end
// tx_crc
always @(posedge gtx_clk) begin
    if (tx_crc_lfsr_q_1__4) begin
        tx_crc <= (~tx_crc_reset_n ? 32'hffffffff : crc32_byte(tx_crc, (tx_crc_payload ? tx_ram_data : 8'd0)));
    end
end
// tx_fcs_shift
always @(posedge gtx_clk) begin
    tx_fcs_shift <= tx_fcs_load ? reflect_octets(~tx_crc) : {tx_fcs_shift[23:0], 8'd0};
end
// tx_payload_d
always @(posedge gtx_clk) begin
    tx_payload_d <= tx_ram_data;
end
// tx_frame_data
always @(posedge gtx_clk) begin
    tx_frame_data <= tx_preamble_byte | tx_sfd_byte | tx_payload_byte | tx_fcs_byte;
end
// tx_gmii_data
always @(posedge gtx_clk) begin
    tx_gmii_data <= tx_frame_data;
end
// ---- RX preamble capture and front-end byte pipeline ----
// rx_dv_d
always @(posedge eth_rx_clk) begin
    rx_dv_d <= (rx_run & eth_rx_dv);
end
// rx_dv_fall
always @(posedge eth_rx_clk) begin
    rx_dv_fall <= (rx_run & rx_dv_d & ~eth_rx_dv);
end
// rx_preamble_window
always @(posedge eth_rx_clk) begin
    if (((eth_rx_dv | ~rx_run))) begin
        rx_preamble_window <= (rx_run ? {rx_preamble_window[39:0], eth_rxd} : 48'd0);
    end
end
// rx_frame_started
always @(posedge eth_rx_clk) begin
    rx_frame_started <= (rx_run & rx_state);
end
// rx_frame_valid
always @(posedge eth_rx_clk) begin
    rx_frame_valid <= (rx_dv_d & ~rx_front_idle);
end
// rx_input_data_d
always @(posedge eth_rx_clk) begin
    rx_input_data_d <= eth_rxd;
end
// rx_front_data
always @(posedge eth_rx_clk) begin
    rx_front_data <= (~rx_front_idle ? rx_input_data_d : 8'd0);
end
// rx_crc_data
always @(posedge eth_rx_clk) begin
    rx_crc_data <= (rx_frame_started ? rx_front_data : 8'd0);
end
// rx_data_d
always @(posedge eth_rx_clk) begin
    rx_data_d <= rx_crc_data;
end
// ---- RX CRC, frame policy and output pipelines ----
// rx_valid_pipe
always @(posedge eth_rx_clk) begin
    rx_valid_pipe <= {rx_valid_pipe[7:0], (rx_frame_valid & rx_frame_started)};
end
// rx_done_pipe
always @(posedge eth_rx_clk) begin
    rx_done_pipe <= {rx_done_pipe[3:0], (rx_run & rx_dv_fall)};
end
// rx_crc_done_pipe
always @(posedge eth_rx_clk) begin
    rx_crc_done_pipe <= (rx_run ? {rx_crc_done_pipe[3:0], rx_done_pipe[0]} : 5'd0);
end
// rx_crc
always @(posedge eth_rx_clk) begin
    if (((rx_valid_pipe[0] | rx_crc_done_pipe[4] | ~rx_run))) begin
        rx_crc <= (~((rx_run & rx_valid_pipe[0])) ? 32'hffffffff : crc32_byte(rx_crc, rx_crc_data));
    end
end
// rx_crc_ok
always @(posedge eth_rx_clk) begin
    rx_crc_ok <= (rx_crc_complete && rx_crc == 32'hc704dd7b);
end
// rx_fcs_window
always @(posedge eth_rx_clk) begin
    if (rx_frame_valid) begin
        rx_fcs_window <= {rx_fcs_window[23:0], rx_front_data};
    end
end
// rx_byte_count
always @(posedge eth_rx_clk) begin
    if ((((rx_frame_valid & rx_frame_started) | rx_done_pipe[4] | ~rx_run))) begin
        rx_byte_count <= (~((rx_done_pipe[4] | ~rx_run)) ? (rx_byte_count + 1'b1) : 12'd0);
    end
end
// rx_length_high
always @(posedge eth_rx_clk) begin
    if (rx_frame_length_10__1) begin
        rx_length_high <= (rx_run ? rx_fcs_window[31:19] : 13'd0);
    end
end
// rx_length_low
always @(posedge eth_rx_clk) begin
    if (rx_frame_length_10__1) begin
        rx_length_low <= (rx_run ? rx_fcs_window[18:16] : 3'd0);
    end
end
// rx_crc_checked
always @(posedge eth_rx_clk) begin
    rx_crc_checked <= rx_crc_complete;
end
// rx_frame_valid_d
always @(posedge eth_rx_clk) begin
    rx_frame_valid_d <= (rx_run & rx_frame_valid);
end
// rx_phy_error_d
always @(posedge eth_rx_clk) begin
    rx_phy_error_d <= (rx_run & eth_rx_er);
end
// rx_phy_error_latched
always @(posedge eth_rx_clk) begin
    rx_phy_error_latched <= (rx_run & ((rx_phy_error_d & rx_frame_valid_d) | (rx_phy_error_d & ~rx_frame_valid) |
        (rx_frame_valid_d & rx_phy_error_latched) | (rx_phy_error_latched & ~rx_frame_valid)));
end
// rx_error_flag
always @(posedge eth_rx_clk) begin
    rx_error_flag <= ((rx_done_pipe[3] & rx_mac_tuser_0) | (rx_done_pipe[3] & rx_mac_tuser_2 & ~rx_Equal13_7));
end
// ---- Reference PAUSE receive/timer compatibility ----
// rx_pause_info
always @(posedge eth_rx_clk) begin
    if ((((rx_Equal7_0 & rx_Equal7_2 & rx_byte_count[1]) | ~rx_run))) begin
        rx_pause_info <= (rx_run ? rx_fcs_window[15:0] : 16'd0);
    end
end
// rx_pause_en
always @(posedge eth_rx_clk) begin
    rx_pause_en <= (rx_run & ((rx_Equal9_0 & rx_pause_en) | (rx_Equal9_0 & ~rx_always13_3)));
end
// rx_pause_cnt
always @(posedge eth_rx_clk) begin
    rx_pause_cnt <= (rx_pause_en ? (rx_pause_cnt + 1'b1) : 4'd0);
end
// rx_pause_time
always @(posedge eth_rx_clk) begin
    if (((~rx_run | ~rx_Equal9_0 | ~rx_always13_3))) begin
        rx_pause_time <= ((rx_run & rx_Equal9_0) ? rx_pause_info : 16'd0);
    end
end
// pause_cdc_write_address
always @(posedge gtx_clk) begin
    pause_cdc_write_address <= (~control_pause_time_d1_rtl_0_auto_generated_cntr1_modulus_trigger ?
        (pause_cdc_write_address + 1'b1) : 2'd0);
end
// pause_cdc_read_address
always @(posedge gtx_clk) begin
    pause_cdc_read_address <= {(pause_cdc_write_address[1] ^ 1'b1), pause_cdc_write_address[0]};
end
// tx_rx_mac_pause_en_d2
always @(posedge gtx_clk) begin
    tx_rx_mac_pause_en_d2 <= pause_cdc_q[1];
end
// tx_pause_in_req_pos
always @(posedge gtx_clk) begin
    tx_pause_in_req_pos <= (pause_cdc_q[1] & ~tx_rx_mac_pause_en_d2);
end
// tx_pause_in_wait_time
always @(posedge gtx_clk) begin
    if ((((pause_cdc_q[1] & ~tx_pause_in_wait_time_3__12) | (pause_cdc_q[1] & ~tx_pause_in_wait_time_3__5) |
        ~tx_run))) begin
        tx_pause_in_wait_time <= (tx_run ? {pause_cdc_q[16:6], pause_cdc_q[0], pause_cdc_q[2], pause_cdc_q[3],
            pause_cdc_q[4], pause_cdc_q[5]} : 16'd0);
    end
end
// tx_pause_in_wait_cnt
always @(posedge gtx_clk) begin
    tx_pause_in_wait_cnt <= (tx_pause_timer_run ? tx_pause_in_wait_cnt + 22'd1 : 22'd0) | {21'd0, tx_pause_timer_start};
end
// tx_pause_in_waiting
always @(posedge gtx_clk) begin
    tx_pause_in_waiting <= tx_LessThan2_5;
end
assign tx_fcs_load = (tx_LessThan6_0 | tx_LessThan7_0);
assign tx_pause_timer_run = ((tx_LessThan2_5 & ~tx_Equal4_4 & ~((tx_pause_in_wait_cnt == {tx_pause_in_wait_time,
    1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0})) & ~tx_pause_in_req_pos));
assign tx_pause_timer_start = (tx_pause_in_req_pos & ~tx_Equal4_4);
assign tx_preamble_byte = (tx_LessThan8_2 & ~tx_mac_txd_3__20) ? 8'h55 : 8'd0;
assign tx_sfd_byte = (tx_mac_txd_17 & (~tx_mac_txd_3__21 | ~(((tx_LessThan6_0 & ~tx_sending) | (tx_LessThan7_0 &
    ~tx_sending))))) ? 8'h80 : 8'd0;
assign tx_payload_byte = (tx_always19_0 & tx_mac_txd_7) ? tx_payload_d : 8'd0;
assign tx_fcs_byte = (tx_mac_txd_7 & ~tx_always19_0) ? tx_fcs_shift[31:24] : 8'd0;
function [31:0] reflect_octets;
    input [31:0] data; integer octet, bit_index;
    begin
        for(octet=0;octet<4;octet=octet+1)
            for(bit_index=0;bit_index<8;bit_index=bit_index+1)
                reflect_octets[octet*8+bit_index] = data[octet*8+7-bit_index];
    end
endfunction
function [31:0] crc32_byte;
    input [31:0] state; input [7:0] data;
    reg [31:0] crc; integer bit_index;
    begin
        crc = state;
        for (bit_index=0; bit_index<8; bit_index=bit_index+1)
            crc = (crc[31] ^ data[bit_index]) ? ((crc << 1) ^ 32'h04c11db7) : (crc << 1);
        crc32_byte = crc;
    end
endfunction
// Byte-wide ping-pong TX RAM, preserving the registered read address.
reg [7:0] tx_frame_ram [0:4095];
reg [11:0] tx_ram_read_address_d;
assign tx_ram_data = tx_frame_ram[tx_ram_read_address_d];
integer tx_ram_index;
initial for(tx_ram_index=0;tx_ram_index<4096;tx_ram_index=tx_ram_index+1) tx_frame_ram[tx_ram_index]=0;
always @(posedge gtx_clk) begin
    if (tx_buf_comb_0) tx_frame_ram[{tx_write_bank, tx_write_address}] <= tx_write_data_d;
    tx_ram_read_address_d <= {tx_read_bank, tx_read_address};
end
// RX byte pipeline: five history bytes plus the reference old-read/output stages.
reg [7:0] rx_history [0:4];
reg [7:0] rx_delay_old;
integer rx_history_index;
initial for(rx_history_index=0;rx_history_index<5;rx_history_index=rx_history_index+1) rx_history[rx_history_index]=0;
always @(posedge eth_rx_clk) begin
    rx_history[0] <= rx_data_d;
    for(rx_history_index=1;rx_history_index<5;rx_history_index=rx_history_index+1)
        rx_history[rx_history_index] <= rx_history[rx_history_index-1];
    rx_delay_old <= rx_history[4];
    rx_delay_q <= rx_delay_old;
end
// pause_cdc_ram: original registered-address/read-data staging.
reg [16:0] pause_cdc_ram [0:3];
integer pause_cdc_ram_i;
initial for(pause_cdc_ram_i=0;pause_cdc_ram_i<4;pause_cdc_ram_i=pause_cdc_ram_i+1) pause_cdc_ram[pause_cdc_ram_i]=0;
always @(posedge gtx_clk) pause_cdc_ram[pause_cdc_write_address] <= {rx_pause_time[15:5], rx_pause_time[0],
    rx_pause_time[1], rx_pause_time[2], rx_pause_time[3], rx_pause_en, rx_pause_time[4]};
reg [1:0] pause_cdc_ram_address_d;
always @(posedge gtx_clk) begin
    pause_cdc_ram_address_d <= {((pause_cdc_read_address[1] < 1'd1)), pause_cdc_read_address[0]};
    pause_cdc_q <= pause_cdc_ram[pause_cdc_ram_address_d];
end
// Additional diagnostics have no equivalent ports on the original reference.
reg tx_en_d=0;
reg crc_bad_at_user=0;
assign tx_frame_error=1'b0;
always @(posedge gtx_clk) begin
 if (!rst_n) begin tx_en_d<=0;tx_frame_done<=0;end
 else begin tx_en_d<=eth_tx_en;tx_frame_done<=tx_en_d && !eth_tx_en;end
end
always @(posedge eth_rx_clk) begin
 if (!rst_n) begin
  crc_bad_at_user<=0;rx_good_frame<=0;rx_bad_frame<=0;rx_bad_fcs<=0;
 end else begin
  crc_bad_at_user<=!rx_crc_ok;
  rx_good_frame<=rx_axis_mac_tvalid && rx_axis_mac_tlast && !rx_axis_mac_tuser;
  rx_bad_frame<=rx_axis_mac_tlast && rx_axis_mac_tuser;
  rx_bad_fcs<=rx_axis_mac_tvalid && rx_axis_mac_tlast && crc_bad_at_user;
 end
end
endmodule
