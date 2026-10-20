/*
 * ================================================================
 * File Name    : afdx_bag_scheduler.v
 * Module Name  : afdx_bag_scheduler
 *
 * Function:
 *   VL 发送调度器（流量整形）：为每个发送 VL 维护一个 BAG 定时器，
 *   队列非空且 BAG 定时器到期的 VL 为可发送 VL；多个可发送 VL 之间采用轮询仲裁，
 *   授权在接受前保持稳定；实际总线出口的 BAG 由各 MAC 再次约束。
 *   参考软件模型 RegulatorLogic：首帧立即发送并启动 BAG 计时，计时期间到达的消息排队等待。
 *
 * Sub Module:
 *   afdx_first16
 *   afdx_rr_arbiter
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_bag_scheduler #(
    parameter N_VL = 8,
    parameter VLW  = 3,
    parameter BAGW = 20                        // BAG 计数位宽（微秒，最大 128000）
) (
    input  wire            clk,
    input  wire            rst,
    input  wire            tick_us,            // 微秒脉冲
    input  wire [N_VL-1:0] nonempty,           // 各 VL 发送队列非空
    input  wire            queue_pop,          // 发送队列出队后一拍（非空位再过一拍清零）
    input  wire [BAGW-1:0] grant_bag_us,       // 被授权 VL 的 BAG（来自配置 ROM，索引 = grant_vl）
    output reg             grant_valid,        // 存在可发送 VL
    output reg  [VLW-1:0]  grant_vl,           // 被选中的 VL
    input  wire            grant_ack,          // 帧组装器接受授权（脉冲）
    output reg [N_VL-1:0] bag_open            // 各 VL 的 BAG 窗口已打开（调试/统计）
);
    `include "afdx_math.vh"
    reg [BAGW-1:0] bag_cnt [0:N_VL-1];
    reg [VLW-1:0]  rr_ptr;                     // 轮询指针

    localparam GROUPS = (N_VL + 15) / 16;
    localparam GW = afdx_clog2(GROUPS);
    // A local microsecond tick per 16 counters (two uniform register stages),
    // and a one-hot copy of grant_vl made when the grant is issued.
    (* preserve *) reg [GROUPS-1:0] tick_group, tick_count_group;
    (* preserve, maxfan = 32 *) reg [BAGW-1:0] load_bag;
    reg load_zero;
    reg [GROUPS-1:0] grant_group_oh;
    reg [15:0] grant_lane_oh;
    always @(posedge clk) begin
        if (rst) begin tick_group <= {GROUPS{1'b0}}; tick_count_group <= {GROUPS{1'b0}}; end
        else begin tick_group <= {GROUPS{tick_us}}; tick_count_group <= tick_group; end
        load_bag <= grant_bag_us;
        load_zero <= grant_bag_us == 0;
    end
    genvar g;
    generate
        for (g = 0; g < N_VL; g = g + 1) begin : g_open
            // Register the decoded ack next to its counter. Shift the tick
            // and BAG value equally so a coincident ack/tick retains priority.
            reg load;
            always @(posedge clk)
                load <= !rst && grant_ack && grant_group_oh[g/16] && grant_lane_oh[g%16];
            // Predict the counter's next zero state, preserving the exact
            // eligibility cycle without a wide comparator before arbitration.
            // bag_open masks an unused counter, so the counter needs no reset;
            // this removes reset from every BAG count bit's D/enable path.
            always @(posedge clk) begin
                if (load) bag_cnt[g] <= load_bag;
                else if (tick_count_group[g/16] && !bag_open[g])
                    bag_cnt[g] <= bag_cnt[g] - 1'b1;
            end
            always @(posedge clk) begin
                if (rst) bag_open[g] <= 1'b1;
                else if (load) begin
                    // 授权被接受：重启该 VL 的 BAG 计时
                    bag_open[g] <= load_zero;
                end else if (tick_count_group[g/16] && !bag_open[g] && bag_cnt[g] == 1)
                    bag_open[g] <= 1'b1;
            end
        end
    endgenerate

    wire [N_VL-1:0] elig = nonempty & bag_open;

    // Registered round robin: M rr_ptr masks, A1 eligibility, S1 first
    // eligible VL in each 16-VL group, S2 group choice, S3 VL number. A grant
    // uses only candidates computed after the last ack or queue pop, when
    // eligibility can fall and rr_ptr moves.
    // ack -> local load (1) + masks (1) + A1..S3 (4).
    // The extra clock also covers the one-clock delayed bag_open update.
    localparam HOLD = 6;
    localparam HW = afdx_clog2(HOLD + 1);
    wire [GW-1:0] rr_group;
    wire [3:0] rr_lane;
    generate if (VLW > 4) begin : g_wide_rr
        assign rr_group = rr_ptr[VLW-1:4];
        assign rr_lane = rr_ptr[3:0];
    end else begin : g_narrow_rr
        assign rr_group = {GW{1'b0}};
        if (VLW == 4) begin : g_four_lane
            assign rr_lane = rr_ptr;
        end else begin : g_short_lane
            assign rr_lane = {{(4-VLW){1'b0}}, rr_ptr};
        end
    end endgenerate
    reg [GW-1:0] rr_group_m;
    reg [GROUPS-1:0] start_group_m;
    reg [15:0] upper_mask_m;
    always @(posedge clk) begin
        rr_group_m <= rr_group;
        upper_mask_m <= 16'hffff << rr_lane;
    end
    reg [GROUPS-1:0] group_any_q, group_after_q;
    reg [3:0] group_pick_q [0:GROUPS-1];
    genvar grp, lane;
    generate for (grp = 0; grp < GROUPS; grp = grp + 1) begin : g_group
        localparam [GW-1:0] GROUP_ID = grp[GW-1:0];
        wire [15:0] bits;
        reg [15:0] bits_q, after_bits_q;
        wire [15:0] after_mask = start_group_m[grp] ? upper_mask_m : 16'hffff;
        wire any_found, after_found;
        wire [3:0] any_first, after_first;
        for (lane = 0; lane < 16; lane = lane + 1) begin : g_lane
            if (grp*16 + lane < N_VL) begin : g_real
                assign bits[lane] = elig[grp*16 + lane];
            end else begin : g_pad
                assign bits[lane] = 1'b0;
            end
        end
        always @(posedge clk) begin
            start_group_m[grp] <= rr_group == GROUP_ID;
            bits_q <= bits;
            after_bits_q <= bits & after_mask;
        end
        afdx_first16 u_any (
            .x     (bits_q),
            .any   (any_found),
            .index (any_first)
        );
        afdx_first16 u_after (
            .x     (after_bits_q),
            .any   (after_found),
            .index (after_first)
        );
        always @(posedge clk) begin
            if (rst) begin group_any_q[grp] <= 1'b0; group_after_q[grp] <= 1'b0; end
            else begin
                group_any_q[grp] <= any_found;
                group_after_q[grp] <= after_found;
            end
            group_pick_q[grp] <= after_found ? after_first : any_first;
        end
    end endgenerate

    // Only the start group needs lanes at or after rr_lane.
    wire [GROUPS-1:0] group_request = group_any_q & (~start_group_m | group_after_q);
    wire group_found;
    wire [GW-1:0] group_choice;
    afdx_rr_arbiter #(
        .N (GROUPS),
        .W (GW)
    ) u_group_rr (
        .request     (group_request),
        .start_index (rr_group_m),
        .valid       (group_found),
        .index       (group_choice)
    );
    reg any_s2, found_s3;
    reg [GW-1:0] group_s2;
    reg [3:0] lane_s2 [0:GROUPS-1];
    reg [VLW-1:0] pick_s3;
    always @(posedge clk) begin
        if (rst) any_s2 <= 1'b0;
        else any_s2 <= |group_any_q;
        // Requests only below rr_lane in the start group use its wrapped pick.
        group_s2 <= group_found ? group_choice : rr_group_m;
    end
    generate for (grp = 0; grp < GROUPS; grp = grp + 1) begin : g_lane_pipe
        always @(posedge clk) lane_s2[grp] <= group_pick_q[grp];
    end endgenerate
    wire [GW+3:0] selected_full = {group_s2, lane_s2[group_s2]};
    wire [VLW-1:0] selected_vl;
    generate if (VLW <= GW+4) begin : g_pick_narrow
        assign selected_vl = selected_full[VLW-1:0];
    end else begin : g_pick_wide
        assign selected_vl = {{(VLW-GW-4){1'b0}}, selected_full};
    end endgenerate
    always @(posedge clk) begin
        if (rst) found_s3 <= 1'b0;
        else found_s3 <= any_s2;
        pick_s3 <= selected_vl;
    end

    reg [HW-1:0] hold;
    generate for (grp = 0; grp < GROUPS; grp = grp + 1) begin : g_grant_group
        always @(posedge clk)
            if (!rst && !grant_ack && !queue_pop && !grant_valid && hold == 0)
                grant_group_oh[grp] <= pick_s3 / 16 == grp;
    end endgenerate
    always @(posedge clk) begin
        if (rst) begin
            grant_valid <= 1'b0;
            grant_vl    <= {VLW{1'b0}};
            rr_ptr      <= {VLW{1'b0}};
            hold        <= {HW{1'b0}};
        end else begin
            if (hold != 0) hold <= hold - 1'b1;
            if (grant_ack) begin
                // 授权被接受：轮询指针后移
                rr_ptr            <= (grant_vl == N_VL - 1) ? {VLW{1'b0}} : grant_vl + 1'b1;
                grant_valid       <= 1'b0;
                hold              <= HOLD[HW-1:0];
            end else if (queue_pop) begin
                // The frame builder has just released a message and cannot
                // accept a grant on this clock. Re-arbitrate afterwards.
                grant_valid       <= 1'b0;
                hold              <= HOLD[HW-1:0];
            end else if (!grant_valid && hold == 0) begin
                grant_valid <= found_s3;
                grant_vl    <= pick_s3;
                grant_lane_oh <= 16'd1 << (pick_s3 % 16);
            end
        end
    end
endmodule
