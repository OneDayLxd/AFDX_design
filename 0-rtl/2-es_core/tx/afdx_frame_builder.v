/*
 * ================================================================
 * File Name    : afdx_frame_builder.v
 * Module Name  : afdx_frame_builder
 *
 * Function:
 *   帧组装器：把 VL 发送队列中的消息封装为 AFDX 帧（不含 FCS，FCS 由各网络的 MAC 发送模块附加）。
 *   帧结构：MAC 头(14) + IPv4 头(20) + UDP 头(8) + 有效载荷(1..1471) + 填充(补至 17) + 序列号(1)
 *     目的 MAC = 03:00:00:00:VV:VV（VLID）  源 MAC = 02:00:00:NN:EE:(iii)00000（网络号/设备号/接口号）
 *     源 IP   = 10.NN.EE.(000ppppp)（分区号）    目的 IP = 224.224.VV.VV
 *   UDP 载荷和在应用写入队列时计算并随消息保存，组帧只读取一次载荷。
 *   IP 分片仅首片带 UDP 首部，各片独立序列号；UDP 校验和覆盖完整消息。
 *   描述符可携带 UDP 端点覆盖（管理响应回到请求源端口）和 SOF 时间戳标记，二者在 S_LOAD 锁存。
 *   授权先寄存一拍（S_GRANT）再确认，使运行时配置 RAM 的两级寄存读出在确认前稳定。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
`include "afdx_defs.vh"
module afdx_frame_builder #(
    parameter FRAGMENT_ENABLE = 0,
    parameter N_VL        = 8,
    parameter VLW         = 3,
    parameter SLOT_AW     = 11,
    parameter UDP_CSUM_EN = 1,                  // 1: 计算 UDP 校验和；0: 填 0（不校验）
    parameter [7:0] NET_ID = 8'd1,              // 网络号 n
    parameter [7:0] EQ_ID  = 8'd1               // 设备号 e
) (
    input  wire               clk,
    input  wire               rst,
    // 调度器授权
    input  wire               grant_valid,
    input  wire [VLW-1:0]     grant_vl,
    output reg                grant_ack,
    // 当前 VL 配置（索引 = cfg_idx）
    output wire [VLW-1:0]     cfg_idx,
    input  wire [15:0]        cfg_vlid,
    input  wire [10:0]        cfg_lmax,
    input  wire [15:0]        cfg_udp_src,
    input  wire [15:0]        cfg_udp_dst,
    input  wire [1:0]         cfg_net_en,
    input  wire [4:0]         cfg_partition,
    // 发送队列读侧
    output wire [VLW-1:0]     q_rd_vl,
    output wire               q_rd_start,
    output wire               q_rd_pop,
    input  wire               q_rd_info_valid,
    input  wire               q_rd_pop_ready,
    input  wire [SLOT_AW-1:0] q_rd_len,
    input  wire [31:0]        q_rd_payload_sum,
    input  wire [79:0]        q_rd_fragment,
    input  wire [33:0]        q_rd_ep,            // {tag, ep_en, udp_src, udp_dst}
    input  wire               cfg_hold,           // 配置初始化/切换期间不接受新授权
    input  wire [7:0]         q_rd_data,
    input  wire               q_rd_valid,
    input  wire               q_rd_last,
    output reg                q_rd_ready,
    // 输出帧流（不含 FCS）
    output reg  [7:0]         f_data,
    output reg                f_valid,
    output reg                f_last,
    output reg  [1:0]         f_net_en,           // 该帧要发送到的网络（bit0=A, bit1=B）
    output reg                f_tag,              // 该帧的 SOF 需要时间戳
    output wire [10:0]        f_length,
    output wire               idle,               // S_IDLE；cfg_hold 期间保持
    input  wire               f_ready,
    // 统计脉冲
    output reg                stat_frame,         // 成功组装一帧
    output reg                stat_lmax_drop      // 消息长度超过 Lmax（或为 0）被丢弃
);
    localparam S_IDLE    = 4'd0,
               S_LOAD    = 4'd1,
               S_CHECK   = 4'd2,
               S_CSUM    = 4'd5,
               S_HDR     = 4'd6,
               S_PLD     = 4'd7,
               S_PLDRUN  = 4'd8,
               S_PAD     = 4'd9,
               S_SN      = 4'd10,
               S_POP     = 4'd11,
               S_RELEASE = 4'd12,
               S_SUM1=5'd13, S_SUM2=5'd14, S_SUM3=5'd15, S_FOLD1=5'd16, S_FOLD2=5'd17, S_FINAL=5'd18, S_HDR_INIT=5'd19,
               S_GRANT=5'd20;

    reg [4:0]         state;
    assign idle = (state == S_IDLE);

    // Descriptor validity may change after a concurrent enqueue. The queue
    // must sample rd_start on the same edge on which validity is observed.
    assign q_rd_start = (state == S_PLD) && q_rd_info_valid;
    assign q_rd_pop = (state == S_RELEASE) && q_rd_pop_ready;
    // 当前 VL. Separate copies serve the queue read port, the configuration
    // ROM/credit release, and the sequence-number table, so no single
    // register fans out to all three.
    (* preserve *) reg [VLW-1:0]     vl_l;
    (* preserve *) reg [VLW-1:0]     cfg_vl, seq_vl;
    reg [10:0]        len;                      // 有效载荷长度
    reg [15:0]        ip_len, udp_len;
    assign f_length = ip_len < 45 ? 11'd60 : ip_len[10:0] + 11'd15;
    reg [4:0]         pad_cnt;                  // 填充字节数
    reg [5:0]         hdr_idx;                  // 头部字节索引 0..41
    reg [15:0]        ip_csum, udp_csum;
    reg [31:0]        sum;                      // UDP 校验和累加器
    // The SN table has a synchronous read and no reset so it can map to RAM.
    // A resettable valid bit supplies the required first SN of 1 without a
    // wide reset/write mux on every table byte.
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] sn_mem [0:N_VL-1];
    reg [7:0] sn_read_q, current_sn;
    reg sn_valid_l;
    reg [N_VL-1:0] sn_valid, sn_selected;
    wire sn_selected_valid = |(sn_valid & sn_selected);
    wire sn_advance = (state == S_SN) && f_ready;
    always @(posedge clk) begin
        sn_read_q <= sn_mem[seq_vl];
        if (sn_advance)
            sn_mem[seq_vl] <= (current_sn == 8'd255) ? 8'd1 : current_sn + 1'b1;
    end
    genvar sn_vl;
    generate for (sn_vl = 0; sn_vl < N_VL; sn_vl = sn_vl + 1) begin : g_sn_valid
        localparam [VLW-1:0] INDEX = sn_vl[VLW-1:0];
        always @(posedge clk) begin
            if (rst) begin
                sn_valid[sn_vl] <= 1'b0;
                sn_selected[sn_vl] <= 1'b0;
            end else begin
                if (state == S_LOAD) sn_selected[sn_vl] <= seq_vl == INDEX;
                if (state == S_CSUM && sn_selected[sn_vl]) sn_valid[sn_vl] <= 1'b1;
            end
        end
    end endgenerate
    reg [15:0]        ip_id;
    reg [79:0]        fragment;
    wire fragmented = FRAGMENT_ENABLE != 0 && fragment[79];
    wire first_fragment = fragment[77:65] == 0;
    wire [15:0] ip_ident = fragmented ? fragment[64:49] : ip_id;
    wire [15:0] ip_flags = fragmented ? {2'b00,fragment[78],fragment[77:65]} : 16'd0;
    assign cfg_idx = cfg_vl;
    assign q_rd_vl = vl_l;
    // Checksum pipeline. Its inputs are held from S_CHECK to S_FINAL, so every
    // stage runs each clock without state-bit enables; S_CSUM..S_FOLD2 only
    // wait while the sums advance one stage per clock.
    reg [31:0] udp_a1, udp_b1, udp_c1, udp_d1, udp_a2, udp_c2, udp_a3, udp_a4;
    reg [19:0] ip_a1, ip_b1, ip_a2, ip_b2, ip_a3, ip_b3, ip_total;
    reg [16:0] ip_fold, udp_fold;
    reg [15:0] ip_fold2, udp_fold2;

    (* preserve *) reg [15:0] frame_vlid, frame_udp_src, frame_udp_dst;
    reg [10:0] frame_lmax;
    reg [1:0] frame_net_en;
    reg [4:0] frame_partition;
    // 头部字节生成
    wire [5:0] hdr_fetch_idx = (state == S_HDR && f_ready) ? hdr_idx+1'b1 : hdr_idx;
    reg [7:0] hdr_bank [0:5];
    wire [7:0] hdr_byte = hdr_bank[hdr_idx[5:3]];
    always @(posedge clk) begin
        case (hdr_fetch_idx[2:0])
            3'd0: hdr_bank[0] <= 8'h03;
            3'd1: hdr_bank[0] <= 8'h00;
            3'd2: hdr_bank[0] <= 8'h00;
            3'd3: hdr_bank[0] <= 8'h00;
            3'd4: hdr_bank[0] <= frame_vlid[15:8];
            3'd5: hdr_bank[0] <= frame_vlid[7:0];
            3'd6: hdr_bank[0] <= 8'h02;
            3'd7: hdr_bank[0] <= 8'h00;
        endcase
        case (hdr_fetch_idx[2:0])
            3'd0: hdr_bank[1] <= 8'h00;
            3'd1: hdr_bank[1] <= NET_ID;
            3'd2: hdr_bank[1] <= EQ_ID;
            3'd3: hdr_bank[1] <= 8'h20;
            3'd4: hdr_bank[1] <= 8'h08;
            3'd5: hdr_bank[1] <= 8'h00;
            3'd6: hdr_bank[1] <= 8'h45;
            3'd7: hdr_bank[1] <= 8'h00;
        endcase
        case (hdr_fetch_idx[2:0])
            3'd0: hdr_bank[2] <= ip_len[15:8];
            3'd1: hdr_bank[2] <= ip_len[7:0];
            3'd2: hdr_bank[2] <= ip_ident[15:8];
            3'd3: hdr_bank[2] <= ip_ident[7:0];
            3'd4: hdr_bank[2] <= ip_flags[15:8];
            3'd5: hdr_bank[2] <= ip_flags[7:0];
            3'd6: hdr_bank[2] <= 8'h01;
            3'd7: hdr_bank[2] <= `AFDX_IP_PROTO_UDP;
        endcase
        case (hdr_fetch_idx[2:0])
            3'd0: hdr_bank[3] <= ip_csum[15:8];
            3'd1: hdr_bank[3] <= ip_csum[7:0];
            3'd2: hdr_bank[3] <= 8'h0A;
            3'd3: hdr_bank[3] <= NET_ID;
            3'd4: hdr_bank[3] <= EQ_ID;
            3'd5: hdr_bank[3] <= {3'b000, frame_partition};
            3'd6: hdr_bank[3] <= 8'hE0;
            3'd7: hdr_bank[3] <= 8'hE0;
        endcase
        case (hdr_fetch_idx[2:0])
            3'd0: hdr_bank[4] <= frame_vlid[15:8];
            3'd1: hdr_bank[4] <= frame_vlid[7:0];
            3'd2: hdr_bank[4] <= frame_udp_src[15:8];
            3'd3: hdr_bank[4] <= frame_udp_src[7:0];
            3'd4: hdr_bank[4] <= frame_udp_dst[15:8];
            3'd5: hdr_bank[4] <= frame_udp_dst[7:0];
            3'd6: hdr_bank[4] <= udp_len[15:8];
            3'd7: hdr_bank[4] <= udp_len[7:0];
        endcase
        case (hdr_fetch_idx[2:0])
            3'd0: hdr_bank[5] <= udp_csum[15:8];
            3'd1: hdr_bank[5] <= udp_csum[7:0];
            3'd2: hdr_bank[5] <= 8'd0;
            3'd3: hdr_bank[5] <= 8'd0;
            3'd4: hdr_bank[5] <= 8'd0;
            3'd5: hdr_bank[5] <= 8'd0;
            3'd6: hdr_bank[5] <= 8'd0;
            3'd7: hdr_bank[5] <= 8'd0;
        endcase
    end

    always @(posedge clk) begin
        ip_a1 <= 20'h4611 + ip_len + ip_flags;
        ip_b1 <= {8'h0A,NET_ID} + {EQ_ID,3'b000,frame_partition};
        udp_a1 <= sum + 32'h0011;
        udp_b1 <= {8'h0A,NET_ID} + {EQ_ID,3'b000,frame_partition};
        udp_c1 <= 32'hE0E0 + frame_vlid;
        udp_d1 <= frame_udp_src + {16'd0,frame_udp_dst};
        ip_a2 <= ip_a1 + ip_ident;
        ip_b2 <= ip_b1 + 20'hE0E0;
        udp_a2 <= udp_a1 + udp_b1;
        udp_c2 <= udp_c1 + udp_d1;
        ip_a3 <= ip_a2 + frame_vlid;
        ip_b3 <= ip_b2;
        udp_a3 <= udp_a2 + udp_c2;
        ip_total <= ip_a3 + ip_b3;
        udp_a4 <= udp_a3 + {15'd0,udp_len,1'b0};
        ip_fold <= {1'b0,ip_total[15:0]} + {13'd0,ip_total[19:16]};
        udp_fold <= {1'b0,udp_a4[15:0]} + {1'b0,udp_a4[31:16]};
        ip_fold2 <= ip_fold[15:0]+ip_fold[16];
        udp_fold2 <= udp_fold[15:0]+udp_fold[16];
    end

    // 输出流复用
    always @* begin
        f_data     = 8'h00;
        f_valid    = 1'b0;
        f_last     = 1'b0;
        q_rd_ready = 1'b0;
        case (state)
            S_HDR: begin
                f_data  = hdr_byte;
                f_valid = 1'b1;
            end
            S_PLDRUN: begin
                f_data     = q_rd_data;
                f_valid    = q_rd_valid;
                q_rd_ready = f_ready;
            end
            S_PAD: begin
                f_data  = 8'h00;
                f_valid = 1'b1;
            end
            S_SN: begin
                f_data  = current_sn;
                f_valid = 1'b1;
                f_last  = 1'b1;
            end
            default: ;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            grant_ack      <= 1'b0;
            stat_frame     <= 1'b0;
            stat_lmax_drop <= 1'b0;
            vl_l           <= {VLW{1'b0}};
            cfg_vl         <= {VLW{1'b0}};
            seq_vl         <= {VLW{1'b0}};
            len            <= 11'd0;
            ip_len         <= 16'd0;
            udp_len        <= 16'd0;
            pad_cnt        <= 5'd0;
            hdr_idx        <= 6'd0;
            ip_csum        <= 16'd0;
            udp_csum       <= 16'd0;
            sum            <= 32'd0;
            ip_id          <= 16'd0;
            fragment       <= 80'd0;
            f_net_en       <= 2'b00;
            f_tag          <= 1'b0;
            current_sn     <= 8'd1;
            sn_valid_l     <= 1'b0;
        end else begin
            grant_ack      <= 1'b0;
            stat_frame     <= 1'b0;
            stat_lmax_drop <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (grant_valid && !cfg_hold) begin
                        vl_l      <= grant_vl;
                        cfg_vl    <= grant_vl;
                        seq_vl    <= grant_vl;
                        state     <= S_GRANT;
                    end
                end
                S_GRANT: begin
                    grant_ack <= 1'b1;
                    state     <= S_LOAD;
                end
                // The queue's info valid omits the VL tag; it may still describe
                // the previous VL on the first S_LOAD clock (grant_ack high).
                S_LOAD: if (q_rd_info_valid && !grant_ack) begin         // 等待配置 ROM / 队列长度稳定
                    len   <= q_rd_len[10:0];
                    fragment <= FRAGMENT_ENABLE != 0 ? q_rd_fragment : 80'd0;
                    frame_vlid <= cfg_vlid;
                    frame_udp_src <= q_rd_ep[32] ? q_rd_ep[31:16] : cfg_udp_src;
                    frame_udp_dst <= q_rd_ep[32] ? q_rd_ep[15:0] : cfg_udp_dst;
                    f_tag <= q_rd_ep[33];
                    frame_lmax <= cfg_lmax; frame_net_en <= cfg_net_en; frame_partition <= cfg_partition;
                    state <= S_CHECK;
                end
                S_CHECK: begin
                    // Only the transition depends on the length comparison;
                    // a dropped frame never uses these fields.
                    sn_valid_l <= sn_selected_valid;
                    ip_len   <= (fragmented && !first_fragment ? 16'd20 : 16'd28) + len;
                    udp_len  <= fragmented ? fragment[48:33] : 16'd8 + len;
                    pad_cnt  <= fragmented && !first_fragment ?
                                (len < 25 ? 5'd25 - len[4:0] : 5'd0) :
                                (len < `AFDX_MIN_PAYLOAD ? 5'd17 - len[4:0] : 5'd0);
                    f_net_en <= frame_net_en;
                    sum      <= fragmented ? fragment[32:1] : q_rd_payload_sum;
                    if (len == 0 || len > frame_lmax || len > `AFDX_MAX_PAYLOAD) begin
                        stat_lmax_drop <= 1'b1;
                        state          <= S_RELEASE;
                    end else begin
                        state    <= S_CSUM;
                    end
                end
                S_CSUM: begin
                    current_sn <= sn_valid_l ? sn_read_q : 8'd1;
                    state <= S_SUM1;
                end
                S_SUM1: state <= S_SUM2;
                S_SUM2: state <= S_SUM3;
                S_SUM3: state <= S_FOLD1;
                S_FOLD1: state <= S_FOLD2;
                S_FOLD2: state <= S_FINAL;
                S_FINAL: begin
                    ip_csum <= ~ip_fold2;
                    udp_csum <= UDP_CSUM_EN == 0 ? 16'd0 : (udp_fold2==16'hffff ? 16'hffff : ~udp_fold2);
                    hdr_idx <= 0;
                    state <= S_HDR_INIT;
                end
                S_HDR_INIT: state <= S_HDR;
                S_HDR: begin
                    if (f_ready) begin
                        hdr_idx <= hdr_idx + 1'b1;
                        if (hdr_idx == (fragmented && !first_fragment ? 6'd33 : 6'd41)) state <= S_PLD;
                    end
                end
                S_PLD: if (q_rd_info_valid) state <= S_PLDRUN;
                S_PLDRUN: begin
                    if (q_rd_valid && f_ready && q_rd_last)
                        state <= (pad_cnt != 0) ? S_PAD : S_SN;
                end
                S_PAD: begin
                    if (f_ready) begin
                        pad_cnt <= pad_cnt - 1'b1;
                        if (pad_cnt == 5'd1) state <= S_SN;
                    end
                end
                S_SN: begin
                    if (f_ready) begin
                        // 题目要求：首帧为 1，之后 1..255 循环。
                        ip_id        <= ip_id + 1'b1;
                        stat_frame   <= 1'b1;
                        state        <= S_RELEASE;
                    end
                end
                S_RELEASE: if (q_rd_pop_ready) state <= S_POP;
                S_POP: state <= S_IDLE;            // 等待出队生效
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
