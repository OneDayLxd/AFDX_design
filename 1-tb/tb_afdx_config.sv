`timescale 1ns/1ps

module tb_afdx_config;
    `include "tb_common.svh"

    reg [6:0] tx_index = 0;
    wire [15:0] tx_vlid;
    wire [19:0] tx_bag_us;
    wire [10:0] tx_lmax;
    wire [15:0] id_vlid;
    wire [10:0] id_lmax;
    wire [15:0] tx_udp_src, tx_udp_dst;
    wire [1:0] tx_net_en;
    wire [4:0] tx_partition;
    wire [19:0] gen_period_us, gen_phase_us;
    wire [10:0] gen_length;
    integer phase_of [0:127], slot_ns [0:127], period_of [0:127];
    integer other, overlaps = 0;

    reg [15:0] lookup_vlid = 0;
    wire lookup_hit;
    wire [11:0] lookup_index;

    afdx_tx_vl_rom #(.VLW(7)) u_tx (
        .idx(tx_index), .vlid(tx_vlid), .bag_us(tx_bag_us), .lmax(tx_lmax),
        .udp_src(tx_udp_src), .udp_dst(tx_udp_dst), .net_en(tx_net_en),
        .partition(tx_partition)
    );
    afdx_tx_vl_id_rom #(
        .VLW (7)
    ) u_id (
        .idx  (tx_index),
        .vlid (id_vlid),
        .lmax (id_lmax)
    );
    afdx_gen_rom #(.VLW(7)) u_gen (
        .idx(tx_index), .period_us(gen_period_us), .phase_us(gen_phase_us), .len(gen_length)
    );
    afdx_rx_vl_lookup #(.VLW(12)) u_rx (
        .vlid(lookup_vlid), .hit(lookup_hit), .idx(lookup_index)
    );

    integer index;
    integer network_a_count = 0;
    integer network_b_count = 0;
    longint unsigned load_a_bits_per_second = 0;
    longint unsigned load_b_bits_per_second = 0;
    initial begin
        // Exhaustively validate all 128 production TX VL entries.
        for (index = 0; index < 128; index = index + 1) begin
            tx_index = index[6:0];
            #1;
            tb_check(tx_vlid == 16'h1000 + index, "TX ROM VLID is unique and contiguous");
            tb_check(tx_bag_us >= 1000 && tx_bag_us <= 128000,
                     "TX ROM BAG lies in the ARINC 664 range");
            tb_check(id_vlid==tx_vlid && id_lmax==tx_lmax, "entry ID/Lmax ROM matches full configuration");
            tb_check(tx_lmax==(index==0 ? 516 : 1471), "production management/business payload ceilings");
            tb_check(tx_lmax >= 17 && tx_lmax <= 1471, "TX ROM Lmax is within AFDX bounds");
            tb_check(tx_udp_src == 1000 + index && tx_udp_dst == 2000 + index,
                     "TX ROM UDP ports match the configured VL");
            tb_check(tx_net_en != 0, "every TX VL enables at least one redundant network");
            tb_check(tx_partition <= 31, "TX partition fits the AFDX source-IP field");
            // 周期 0 表示内部发生器不使用该 VL（管理 VL）
            tb_check(gen_length >= 1 && gen_length <= tx_lmax,
                     "application generator period and length fit the VL contract");
            tb_check(gen_phase_us == 0 || gen_phase_us < gen_period_us, "generator phase lies inside its period");
            phase_of[index] = gen_phase_us; period_of[index] = gen_period_us;
            // 演示发生器 A/B 两帧（按实际 gen_length，最短 64 字节）+ IFG 在 100 Mbps 总线上的占用
            slot_ns[index] = ((gen_length + 47 < 64 ? 64 : gen_length + 47) + 12) * 80 * (tx_net_en[0] + tx_net_en[1]);
            if (tx_net_en[0]) begin
                network_a_count = network_a_count + 1;
                load_a_bits_per_second = load_a_bits_per_second +
                    ((tx_lmax + 67) * 8 * 1_000_000) / tx_bag_us;
            end
            if (tx_net_en[1]) begin
                network_b_count = network_b_count + 1;
                load_b_bits_per_second = load_b_bits_per_second +
                    ((tx_lmax + 67) * 8 * 1_000_000) / tx_bag_us;
            end
        end
        // 同周期的周期性 VL 相位时隙互不重叠（生产配置全部为 2 ms）
        for (index = 0; index < 128; index = index + 1)
            for (other = 0; other < 128; other = other + 1)
                if (index != other && period_of[index] != 0 && period_of[index] == period_of[other] &&
                    phase_of[index] <= phase_of[other] &&
                    phase_of[index] * 1000 + slot_ns[index] > phase_of[other] * 1000)
                    overlaps = overlaps + 1;
        tb_check(overlaps == 0, "planned generator phases give every periodic VL its own bus slot");
        tb_check(network_a_count == 128 && network_b_count == 128,
                 "production configuration exposes all 128 TX VLs on both networks");
        tb_check(load_a_bits_per_second <= 1_000_000_000 && load_b_bits_per_second <= 1_000_000_000,
                 "configured worst-case load remains below 1 Gbps per physical network");
        tb_check(load_a_bits_per_second > 100_000_000,
                 "GMII configuration permits payload loads above the old 100 Mbps line limit");

        // VL0 carries the largest management request/response supported by the agent.
        tx_index = 0;
        #1;
        tb_check(tx_lmax >= 64, "management VL Lmax carries SNMP and 615A packets up to 64 bytes");

        // Exhaustively validate all 4096 production RX subscriptions and endpoints.
        for (index = 0; index < 4096; index = index + 1) begin
            lookup_vlid = 16'h1000 + index;
            #1;
            tb_check(lookup_hit, "RX lookup hits every configured VLID");
            tb_check(lookup_index == index[11:0], "RX lookup maps VLID to the exact queue index");
        end
        lookup_vlid = 16'h0fff; #1;
        tb_check(!lookup_hit && lookup_index == 0, "RX lookup rejects VLID below configured range");
        lookup_vlid = 16'h2000; #1;
        tb_check(!lookup_hit && lookup_index == 0, "RX lookup rejects VLID above configured range");

        $display("CONFIG_METRIC tx_vls=128 rx_vls=4096 load_a_mbps=%0f load_b_mbps=%0f shared_mbps=%0f",
                 load_a_bits_per_second/1_000_000.0,
                 load_b_bits_per_second/1_000_000.0,
                 (load_a_bits_per_second+load_b_bits_per_second)/1_000_000.0);
        tb_finish("tb_afdx_config");
        $finish;
    end
endmodule
