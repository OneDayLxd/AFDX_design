/*
 * ================================================================
 * File Name    : afdx_ip_reassembly.v
 * Module Name  : afdx_ip_reassembly
 *
 * Function:
 *   在完整性检查与 A/B 去重后重组 IPv4/UDP 分片，再原子交付应用消息。
 *   支持乱序分片；以 VL、源 IP、IP ID 区分上下文，拒绝重叠和越界。
 *   重组完成才校验 UDP 长度和整包校验和；未分片消息直接透传。
 *   上下文满时替换轮询选中的未完成消息，不采用重组定时超时。
 *
 * Sub Module:
 *   afdx_rx_vl_id_rom
 * ================================================================
 */
`timescale 1ns/1ps
module afdx_ip_reassembly #(
    parameter VLW         = 12,
    parameter MAX_LEN     = 8192,
    parameter CONTEXT_LOG = 2
) (
    input  wire           clk,
    input  wire           rst,
    input  wire [VLW-1:0] in_vl,
    input  wire [7:0]     in_data,
    input  wire           in_valid,
    input  wire           in_last,
    input  wire [10:0]    in_len,
    input  wire [95:0]    in_meta,
    input  wire [31:0]    in_fragment,
    output wire           in_ready,
    output wire [VLW-1:0] out_vl,
    output wire [7:0]     out_data,
    output wire           out_valid,
    output wire           out_last,
    output wire [15:0]    out_len,
    output wire [95:0]    out_meta,
    input  wire           out_ready,
    output reg            drop_pulse,
    output reg  [31:0]    drop_count
);
    localparam CONTEXTS = 1 << CONTEXT_LOG;
    localparam AW = $clog2(MAX_LEN);
    localparam BLOCKS = (MAX_LEN + 15) / 8;
    localparam IDLE=0, PASS=1, ALLOC=2, RECEIVE=3, CHECK=4,
               FOLD=5, DECIDE=6, OUTPUT_DATA=7, SUM1=8, SUM2=9;
    reg [3:0] state;
    (* ramstyle = "M4K, no_rw_check" *) reg [7:0] memory [0:CONTEXTS*(1<<AW)-1];
    reg [CONTEXTS-1:0] used;
    reg [VLW-1:0] vl [0:CONTEXTS-1];
    reg [31:0] source_ip [0:CONTEXTS-1];
    reg [15:0] ident [0:CONTEXTS-1];
    reg [BLOCKS-1:0] coverage [0:CONTEXTS-1];
    reg [16:0] received [0:CONTEXTS-1], end_max [0:CONTEXTS-1];
    reg [16:0] final_length [0:CONTEXTS-1];
    reg [31:0] sum [0:CONTEXTS-1];
    reg [CONTEXTS-1:0] header_seen, final_seen;
    reg [15:0] udp_length [0:CONTEXTS-1], udp_checksum [0:CONTEXTS-1];
    reg [95:0] session [0:CONTEXTS-1];
    reg [CONTEXT_LOG-1:0] context_index, victim;
    reg [VLW-1:0] frame_vl;
    reg [95:0] frame_meta;
    reg [15:0] frame_id;
    reg [16:0] frame_offset, frame_end;
    reg [10:0] frame_length, position;
    reg more, bad;
    integer c;
    reg found, free_found;
    reg [CONTEXT_LOG-1:0] selected, free_index;
    always @* begin
        found = 0;
        free_found = 0;
        selected = victim;
        free_index = victim;
        for (c=0; c<CONTEXTS; c=c+1) begin
            if (!used[c] && !free_found) begin
                free_found = 1;
                free_index = c[CONTEXT_LOG-1:0];
            end
            if (used[c] && vl[c] == frame_vl && source_ip[c] == frame_meta[63:32] &&
                ident[c] == frame_id) begin
                found = 1;
                selected = c[CONTEXT_LOG-1:0];
            end
        end
        if (!found && free_found) selected = free_index;
    end
    wire [BLOCKS-1:0] coverage_l = coverage[context_index];
    wire [95:0] session_l = session[context_index];
    wire [15:0] udp_length_l = udp_length[context_index];
    wire [15:0] udp_checksum_l = udp_checksum[context_index];
    wire [31:0] source_ip_l = source_ip[context_index];
    wire normal = state == PASS || (state == IDLE && !in_fragment[31]);
    wire [16:0] address = frame_offset + position;
    wire address_ok = address < MAX_LEN + 8;
    wire [AW-1:0] payload_address = address - 17'd8;
    wire overlap = address_ok && position[2:0] == 0 && coverage_l[address >> 3];
    assign in_ready = !rst && (normal ? out_ready : state == RECEIVE);
    reg [7:0] read_data;
    reg read_valid;
    reg [AW-1:0] read_address;
    reg [15:0] read_remaining;
    wire load = state == OUTPUT_DATA && read_remaining != 0 && (!read_valid || out_ready);
    assign out_vl = normal ? in_vl : vl[context_index];
    assign out_data = normal ? in_data : read_data;
    assign out_valid = !rst && (normal ? in_valid : state == OUTPUT_DATA && read_valid);
    assign out_last = normal ? in_last : read_valid && read_remaining == 0;
    assign out_len = normal ? {5'd0,in_len} : udp_length[context_index] - 16'd8;
    assign out_meta = normal ? in_meta : session[context_index];
    wire [15:0] vlid;
    afdx_rx_vl_id_rom #(
        .VLW  (VLW)
    ) u_vlid (
        .idx  (vl[context_index]),
        .vlid (vlid)
    );
    reg [31:0] total_sum, sum_a, sum_b, sum_c;
    reg [16:0] folded;
    wire [15:0] checksum_sum = folded[15:0] + folded[16];
    // 数据 RAM 无复位，只有覆盖完整且校验通过的上下文可启动读出。
    always @(posedge clk) begin
        if (load) read_data <= memory[{context_index,read_address}];
        if (state == RECEIVE && in_valid && !bad && address_ok && !overlap && address >= 8)
            memory[{context_index,payload_address}] <= in_data;
        if (rst) begin
            state <= IDLE;
            used <= 0;
            header_seen <= 0;
            final_seen <= 0;
            victim <= 0;
            context_index <= 0;
            read_valid <= 0;
            read_remaining <= 0;
            read_address <= 0;
            drop_pulse <= 0;
            drop_count <= 0;
            bad <= 0;
            position <= 0;
        end else begin
            drop_pulse <= 0;
            case (state)
                IDLE: if (in_valid) begin
                    if (!in_fragment[31]) begin
                        if (!(in_ready && in_last)) state <= PASS;
                    end else begin
                        frame_vl <= in_vl;
                        frame_meta <= in_meta;
                        frame_id <= in_fragment[29:14];
                        frame_offset <= {1'b0,in_fragment[13:1],3'b000};
                        frame_end <= {1'b0,in_fragment[13:1],3'b000} + in_len;
                        frame_length <= in_len;
                        more <= in_fragment[30];
                        position <= 0;
                        state <= ALLOC;
                    end
                end
                PASS: if (in_valid && in_ready && in_last) state <= IDLE;
                ALLOC: begin
                    context_index <= selected;
                    bad <= frame_length == 0 || frame_end > MAX_LEN + 8 ||
                           (more && frame_length[2:0] != 0);
                    if (!found) begin
                        if (used[selected]) begin
                            drop_pulse <= 1;
                            drop_count <= drop_count + 1'b1;
                        end
                        used[selected] <= 1;
                        vl[selected] <= frame_vl;
                        source_ip[selected] <= frame_meta[63:32];
                        ident[selected] <= frame_id;
                        coverage[selected] <= 0;
                        received[selected] <= 0;
                        end_max[selected] <= 0;
                        final_length[selected] <= 0;
                        sum[selected] <= 0;
                        header_seen[selected] <= 0;
                        final_seen[selected] <= 0;
                        udp_length[selected] <= 0;
                        udp_checksum[selected] <= 0;
                        session[selected] <= frame_meta;
                        victim <= selected + 1'b1;
                    end
                    state <= RECEIVE;
                end
                RECEIVE: if (in_valid) begin
                    position <= position + 1'b1;
                    if (!address_ok || overlap) bad <= 1;
                    if (!bad && address_ok && !overlap) begin
                        coverage[context_index] <= coverage_l | ({{(BLOCKS-1){1'b0}},1'b1} << (address >> 3));
                        sum[context_index] <= sum[context_index] +
                            (address[0] ? {24'd0,in_data} : {16'd0,in_data,8'd0});
                        case (address)
                            0: session[context_index] <= {session_l[95:32],in_data,session_l[23:0]};
                            1: session[context_index] <= {session_l[95:24],in_data,session_l[15:0]};
                            2: session[context_index] <= {session_l[95:16],in_data,session_l[7:0]};
                            3: session[context_index] <= {session_l[95:8],in_data};
                            4: udp_length[context_index] <= {in_data,udp_length_l[7:0]};
                            5: udp_length[context_index] <= {udp_length_l[15:8],in_data};
                            6: udp_checksum[context_index] <= {in_data,udp_checksum_l[7:0]};
                            7: udp_checksum[context_index] <= {udp_checksum_l[15:8],in_data};
                            default: ;
                        endcase
                    end
                    if (in_last) begin
                        if (position + 1'b1 != frame_length || !address_ok || overlap || bad) begin
                            used[context_index] <= 0;
                            drop_pulse <= 1;
                            drop_count <= drop_count + 1'b1;
                            state <= IDLE;
                        end else begin
                            received[context_index] <= received[context_index] + frame_length;
                            if (frame_end > end_max[context_index]) end_max[context_index] <= frame_end;
                            if (frame_offset == 0) begin
                                header_seen[context_index] <= 1;
                                session[context_index] <= {frame_meta[95:32],session_l[31:0]};
                            end
                            if (!more) begin
                                if (final_seen[context_index] && final_length[context_index] != frame_end)
                                    bad <= 1;
                                final_seen[context_index] <= 1;
                                final_length[context_index] <= frame_end;
                            end
                            state <= CHECK;
                        end
                    end
                end
                CHECK: begin
                    if (bad || (header_seen[context_index] &&
                        (udp_length[context_index] < 9 || udp_length[context_index] > MAX_LEN + 8 ||
                         end_max[context_index] > udp_length[context_index])) ||
                        (final_seen[context_index] && end_max[context_index] > final_length[context_index])) begin
                        used[context_index] <= 0;
                        drop_pulse <= 1;
                        drop_count <= drop_count + 1'b1;
                        state <= IDLE;
                    end else if (header_seen[context_index] && final_seen[context_index] &&
                                 received[context_index] == final_length[context_index]) begin
                        sum_a <= sum[context_index];
                        sum_b <= {16'd0,source_ip_l[31:16]} +
                                 {16'd0,source_ip_l[15:0]};
                        sum_c <= 32'hE0F1 + {16'd0,vlid} + {16'd0,udp_length[context_index]};
                        state <= SUM1;
                    end else state <= IDLE;
                end
                SUM1: begin
                    total_sum <= sum_a + sum_b;
                    state <= SUM2;
                end
                SUM2: begin
                    total_sum <= total_sum + sum_c;
                    state <= FOLD;
                end
                FOLD: begin
                    folded <= {1'b0,total_sum[15:0]} + {1'b0,total_sum[31:16]};
                    state <= DECIDE;
                end
                DECIDE: begin
                    if (udp_length[context_index] != final_length[context_index] ||
                        (udp_checksum[context_index] != 0 && checksum_sum != 16'hffff)) begin
                        used[context_index] <= 0;
                        drop_pulse <= 1;
                        drop_count <= drop_count + 1'b1;
                        state <= IDLE;
                    end else begin
                        read_address <= 0;
                        read_remaining <= udp_length[context_index] - 16'd8;
                        read_valid <= 0;
                        state <= OUTPUT_DATA;
                    end
                end
                OUTPUT_DATA: begin
                    if (!read_valid || out_ready) read_valid <= load;
                    if (load) begin
                        read_address <= read_address + 1'b1;
                        read_remaining <= read_remaining - 1'b1;
                    end
                    if (out_valid && out_ready && out_last) begin
                        used[context_index] <= 0;
                        read_valid <= 0;
                        state <= IDLE;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
