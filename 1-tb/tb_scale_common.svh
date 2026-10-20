// 测试平台公共例程：CRC-32、参考帧构造（独立于 RTL 的软件参考实现）、结果统计
`ifndef TB_COMMON_SVH
`define TB_COMMON_SVH

int tb_err_cnt = 0;
int tb_chk_cnt = 0;

task automatic tb_check(input bit cond, input string msg);
    tb_chk_cnt++;
    if (!cond) begin
        tb_err_cnt++;
        $display("[%0t] FAIL: %s", $time, msg);
        if (tb_err_cnt == 20) $fatal(1, "stopping after 20 failed checks");
    end
endtask

function automatic [31:0] crc32_byte(input [31:0] crc, input [7:0] d);
    reg [31:0] c;
    c = crc ^ {24'd0, d};
    for (int i = 0; i < 8; i++) c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
    return c;
endfunction

// 参考帧缓冲（构造待发送到 DUT 接收口的帧）
byte unsigned ref_pld [0:2047];
byte unsigned ref_frm [0:2047];
int           ref_flen;

function automatic [15:0] csum_fold(input [31:0] s);
    reg [31:0] t;
    t = s;
    t = t[15:0] + t[31:16];
    t = t[15:0] + t[31:16];
    return ~t[15:0];
endfunction

// 构造一帧 AFDX 帧（含 FCS）到 ref_frm；有效载荷取 ref_pld[0..plen-1]
// iface: 1=网络 A, 2=网络 B；udp_csum_mode: 0=填 0, 1=正确
task automatic ref_build_frame(input int vlid, input int sn, input int iface, input int net_id, input int eq_id,
                               input int partition, input int udp_src, input int udp_dst, input int plen,
                               input int udp_csum_mode);
    int ip_len, udp_len, pad, idx;
    reg [31:0] s;
    reg [31:0] crc;
    ip_len  = 28 + plen;
    udp_len = 8 + plen;
    pad     = (plen < 17) ? 17 - plen : 0;
    ref_frm[0] = 8'h03; ref_frm[1] = 0; ref_frm[2] = 0; ref_frm[3] = 0;
    ref_frm[4] = vlid[15:8]; ref_frm[5] = vlid[7:0];
    ref_frm[6] = 8'h02; ref_frm[7] = 0; ref_frm[8] = 0; ref_frm[9] = net_id[7:0]; ref_frm[10] = eq_id[7:0];
    ref_frm[11] = {iface[2:0], 5'b00000};
    ref_frm[12] = 8'h08; ref_frm[13] = 8'h00;
    ref_frm[14] = 8'h45; ref_frm[15] = 8'h00; ref_frm[16] = ip_len[15:8]; ref_frm[17] = ip_len[7:0];
    ref_frm[18] = 0; ref_frm[19] = 0; ref_frm[20] = 0; ref_frm[21] = 0; ref_frm[22] = 8'h01; ref_frm[23] = 8'd17;
    ref_frm[24] = 0; ref_frm[25] = 0;
    ref_frm[26] = 8'h0A; ref_frm[27] = net_id[7:0]; ref_frm[28] = eq_id[7:0]; ref_frm[29] = partition[7:0];
    ref_frm[30] = 8'hE0; ref_frm[31] = 8'hE0; ref_frm[32] = vlid[15:8]; ref_frm[33] = vlid[7:0];
    // IP 首部校验和
    s = 0;
    for (int i = 14; i < 34; i += 2) s += {ref_frm[i], ref_frm[i+1]};
    {ref_frm[24], ref_frm[25]} = csum_fold(s);
    ref_frm[34] = udp_src[15:8]; ref_frm[35] = udp_src[7:0]; ref_frm[36] = udp_dst[15:8]; ref_frm[37] = udp_dst[7:0];
    ref_frm[38] = udp_len[15:8]; ref_frm[39] = udp_len[7:0]; ref_frm[40] = 0; ref_frm[41] = 0;
    for (int i = 0; i < plen; i++) ref_frm[42 + i] = ref_pld[i];
    if (udp_csum_mode) begin
        s = 0;
        for (int i = 26; i < 34; i += 2) s += {ref_frm[i], ref_frm[i+1]};
        s += 16'h0011 + udp_len;
        for (int i = 34; i < 42 + plen; i += 2)
            s += {ref_frm[i], (i + 1 < 42 + plen) ? ref_frm[i+1] : 8'h00};
        {ref_frm[40], ref_frm[41]} = (csum_fold(s) == 16'h0000) ? 16'hFFFF : csum_fold(s);
    end
    idx = 42 + plen;
    for (int i = 0; i < pad; i++) ref_frm[idx++] = 8'h00;
    ref_frm[idx++] = sn[7:0];
    crc = 32'hFFFFFFFF;
    for (int i = 0; i < idx; i++) crc = crc32_byte(crc, ref_frm[i]);
    crc = ~crc;
    ref_frm[idx++] = crc[7:0]; ref_frm[idx++] = crc[15:8]; ref_frm[idx++] = crc[23:16]; ref_frm[idx++] = crc[31:24];
    ref_flen = idx;
endtask

`endif
