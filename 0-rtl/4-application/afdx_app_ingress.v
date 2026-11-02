/*
 * ================================================================
 * File Name    : afdx_app_ingress.v
 * Module Name  : afdx_app_ingress
 *
 * Function:
 *   valid-only 输入转为 valid/ready 消息流；完整消息才提交。
 *   满、超长、非法/变化的 VL 整条丢弃并计数；last 是唯一消息边界。
 *   同步读写 RAM 不复位；每 VL 运行时 Lmax 仍由核心检查。
 *
 * Sub Module:
 *   none
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_app_ingress #(
    parameter VLW      = 7,
    parameter N_VL     = 128,
    parameter SLOT_LOG = 4,
    parameter MAX_LEN  = 1471
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [VLW-1:0] in_vl,
    input  wire [7:0]     in_data,
    input  wire           in_valid,
    input  wire           in_last,
    output reg  [VLW-1:0] out_vl,
    output reg  [7:0]     out_data,
    output reg            out_valid,
    output reg            out_last,
    input  wire           out_ready,
    output reg            drop_pulse,
    output reg  [2:0]     drop_reason,
    output reg  [31:0]    drop_count,
    output reg  [31:0]    accept_count,
    output reg  [15:0]    out_len,
    output reg  [31:0]    out_sum
);
    localparam SLOTS = 1 << SLOT_LOG;
    localparam BYTE_AW = (MAX_LEN <= 2048) ? 11 : $clog2(MAX_LEN);
    localparam LEN_W = BYTE_AW + 1;
    localparam R_IDLE = 0, R_STREAM = 1, R_LAST = 2;
    // 描述符独立于字节池，长消息按实际长度占用环形 RAM，保留原有总容量。
    localparam POOL_AW = SLOT_LOG + 11;
    localparam POOL_BYTES = 1 << POOL_AW;
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] data_mem [0:POOL_BYTES-1];
    reg active;
    reg [POOL_AW:0] byte_wp, byte_rp, message_start;
    reg [POOL_AW:0] start_mem [0:SLOTS-1];
    reg [POOL_AW:0] read_start;
    wire [POOL_AW:0] write_start = active ? message_start : byte_wp;
    wire byte_full = byte_wp - byte_rp == POOL_BYTES;
    reg [LEN_W-1:0] len_mem [0:SLOTS-1];
    reg [31:0] sum_mem [0:SLOTS-1];
    reg [VLW-1:0] vl_mem [0:SLOTS-1];
    reg [SLOT_LOG:0] wr_seq, rd_seq;

    reg [2:0] faults;
    reg [LEN_W-1:0] wr_pos;
    reg [31:0] wr_sum;
    reg [VLW-1:0] msg_vl;
    reg [1:0] rd_state;
    reg [LEN_W-1:0] rd_pos, rd_len;
    wire [POOL_AW-1:0] read_index = read_start[POOL_AW-1:0] + rd_pos;
    wire full = wr_seq == {~rd_seq[SLOT_LOG], rd_seq[SLOT_LOG-1:0]};
    wire [LEN_W-1:0] pos = active ? wr_pos : {LEN_W{1'b0}};
    wire [31:0] next_sum = (active ? wr_sum : 32'd0) +
        (pos[0] ? {24'd0, in_data} : {16'd0, in_data, 8'd0});
    // bit0: 无空槽；bit1: 超长；bit2: 非法索引或中途更换 VL。
    wire [2:0] beat_faults = (active ? faults : 3'd0) |
        {((in_vl >= N_VL) || (active && in_vl != msg_vl)),
         (pos >= MAX_LEN), ((!active && full) || byte_full)};
    always @(posedge clk) begin
        if (!rst && in_valid && beat_faults == 0)
            data_mem[byte_wp[POOL_AW-1:0]] <= in_data;
        if (!rst && in_valid && in_last && beat_faults == 0) begin
            start_mem[wr_seq[SLOT_LOG-1:0]] <= write_start;
            len_mem[wr_seq[SLOT_LOG-1:0]] <= pos + 1'b1;
            sum_mem[wr_seq[SLOT_LOG-1:0]] <= next_sum;
            vl_mem[wr_seq[SLOT_LOG-1:0]] <= active ? msg_vl : in_vl;
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            wr_seq <= 0;
            byte_wp <= 0;
            message_start <= 0;
            active <= 0;
            faults <= 0;
            wr_pos <= 0;
            wr_sum <= 0;
            msg_vl <= 0;
            drop_pulse <= 0;
            drop_reason <= 0;
            drop_count <= 0;
            accept_count <= 0;
        end else begin
            drop_pulse <= 0;
            if (in_valid) begin
                wr_sum <= next_sum;
                if (!active) message_start <= byte_wp;
                if (beat_faults == 0) byte_wp <= byte_wp + 1'b1;
                if (!active) msg_vl <= in_vl;
                if (in_last) begin
                    active <= 0;
                    faults <= 0;
                    wr_pos <= 0;
                    if (beat_faults != 0) begin
                        byte_wp <= write_start;
                        drop_pulse <= 1;
                        drop_reason <= beat_faults;
                        drop_count <= drop_count + 1'b1;
                    end else begin
                        wr_seq <= wr_seq + 1'b1;
                        accept_count <= accept_count + 1'b1;
                    end
                end else begin
                    active <= 1;
                    faults <= beat_faults;
                    // 饱和以防未结束的超长消息回卷覆盖旧数据。
                    if (pos < MAX_LEN) wr_pos <= pos + 11'd1;
                end
            end
        end
    end
    // RAM 输出寄存器不复位；valid 复位后无效数据不可见。
    always @(posedge clk) begin
        if (rd_state == R_STREAM && (!out_valid || out_ready))
            out_data <= data_mem[read_index];
    end
    // 同步读 RAM，背压时保持输出；末字节被取走才释放槽。
    always @(posedge clk) begin
        if (rst) begin
            rd_seq <= 0;
            byte_rp <= 0;
            read_start <= 0;
            rd_state <= R_IDLE;
            rd_pos <= 0;
            rd_len <= 0;
            out_vl <= 0;

            out_valid <= 0;
            out_last <= 0;
            out_len <= 0;
            out_sum <= 0;
        end else begin
            if (out_valid && out_ready) out_valid <= 0;
            case (rd_state)
                R_IDLE: if (rd_seq != wr_seq) begin
                    read_start <= start_mem[rd_seq[SLOT_LOG-1:0]];
                    out_vl <= vl_mem[rd_seq[SLOT_LOG-1:0]];
                    rd_len <= len_mem[rd_seq[SLOT_LOG-1:0]];
                    out_len <= len_mem[rd_seq[SLOT_LOG-1:0]];
                    out_sum <= sum_mem[rd_seq[SLOT_LOG-1:0]];
                    rd_pos <= 0;
                    rd_state <= R_STREAM;
                end
                R_STREAM: if (!out_valid || out_ready) begin

                    out_valid <= 1;
                    out_last <= rd_pos == rd_len - 1'b1;
                    if (rd_pos == rd_len - 1'b1) rd_state <= R_LAST;
                    else rd_pos <= rd_pos + 1'b1;
                end
                R_LAST: if (out_valid && out_ready) begin
                    rd_seq <= rd_seq + 1'b1;
                    byte_rp <= read_start + rd_len;
                    rd_state <= R_IDLE;
                end
                default: rd_state <= R_IDLE;
            endcase
        end
    end
endmodule
