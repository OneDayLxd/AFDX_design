/*
 * ================================================================
 * File Name    : tb_afdx_two_endpoints.sv
 * Module Name  : tb_afdx_two_endpoints
 * Function:
 *   两个生产端系统通过独立 A/B 存储转发模型双向通信，无应用回送。
 *   应用端仅使用 valid；抓取全部应用消息及 GMII 帧供独立检查。
 *   0:变长多 VL；1:127 业务 VL 吞吐；2:冗余故障；3:额外链路延迟。
 * Sub Module:
 *   AFDX_top (two instances)
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module tb_afdx_two_endpoints;
    `include "tb_common.svh"
    reg clk=0, rst=1, started=0;
    always #4 clk=~clk;
    reg [7:0] app_in_data[2]='{0,0};
    reg [6:0] app_in_vl[2]='{0,0};
    reg [1:0] app_in_valid=0, app_in_last=0;
    wire [7:0] app_out_data[2];
    wire [11:0] app_out_vl[2];
    wire [1:0] app_out_valid, app_out_last, app_ready, app_drop;
    wire [2:0] app_reason[2];
    wire [31:0] app_drops[2];
    wire [31:0] txd;
    wire [3:0] txv, txer;
    reg [31:0] rxd=0;
    reg [3:0] rxv=0;
    int scenario=0, nvl, rounds, period_ns=2500000, extra_ns, fd, timing_fd;
    // rx_ready 交替变化，输出载荷必须完全不受其影响。
    reg [1:0] ignored_ready=0;
    always @(negedge clk) ignored_ready<=ignored_ready+1'b1;
    genvar e;
    generate for(e=0;e<2;e++) begin : g_endpoint
        AFDX_top #(
            .MGMT_EN (0)
        ) dut (
            .clk             (clk),
            .rst             (rst),
            .tx_data         (app_out_data[e]),
            .rx_data         (app_in_data[e]),
            .tx_valid        (app_out_valid[e]),
            .rx_valid        (app_in_valid[e]),
            .tx_ready        (app_ready[e]),
            .rx_ready        (ignored_ready[e]),
            .rx_vl           (app_in_vl[e]),
            .rx_last         (app_in_last[e]),
            .tx_vl           (app_out_vl[e]),
            .tx_last         (app_out_last[e]),
            .app_drop        (app_drop[e]),
            .app_drop_reason (app_reason[e]),
            .app_drop_count  (app_drops[e]),
            .a_gmii_tx_data  (txd[(e*2)*8 +: 8]),
            .a_gmii_tx_valid (txv[e*2]),
            .a_gmii_tx_er    (txer[e*2]),
            .a_gmii_rx_data  (rxd[(e*2)*8 +: 8]),
            .a_gmii_rx_valid (rxv[e*2]),
            .a_gmii_rx_er    (1'b0),
            .b_gmii_tx_data  (txd[(e*2+1)*8 +: 8]),
            .b_gmii_tx_valid (txv[e*2+1]),
            .b_gmii_tx_er    (txer[e*2+1]),
            .b_gmii_rx_data  (rxd[(e*2+1)*8 +: 8]),
            .b_gmii_rx_valid (rxv[e*2+1]),
            .b_gmii_rx_er    (1'b0)
        );
        // 仅观测握手，不驱动 DUT；消息 ID 来自载荷头四字节。
        `include "two_endpoints_timing.svh"
    end endgenerate
    typedef struct {
        int id, size;
        time eof;
        bit [1526*8-1:0] data;
    } packet_t;
    mailbox #(packet_t) queues[4];
    string capture;
    time epoch;
    int expected_len[2][1024], expected_vl[2][1024], seen[2][1024];
    int received[2]='{0,0};
    bit source_done[2]='{0,0};
    function automatic byte pattern(input int id, vl, len, pos);
        case(pos)
            0: return byte'(id>>24);
            1: return byte'(id>>16);
            2: return byte'(id>>8);
            3: return byte'(id);
            4: return byte'(vl>>8);
            5: return byte'(vl);
            6: return byte'(len>>8);
            7: return byte'(len);
            default: return byte'(id*17+vl+pos*7);
        endcase
    endfunction
    function automatic int size_at(input int round_id);
        if(scenario==1) return 1471;
        case(round_id%4)
            0: return 17;
            1: return 465;
            2: return 977;
            default: return 1471;
        endcase
    endfunction
    task automatic source(input int src);
        int id, len;
        time first;
        for(int round_id=0;round_id<rounds;round_id++) begin
            while($time<epoch+round_id*period_ns) @(negedge clk);
            for(int v=1;v<=nvl;v++) begin
                id=round_id*nvl+v-1; len=size_at(round_id);
                expected_len[src][id]=len; expected_vl[src][id]=v;
                for(int p=0;p<len;p++) begin
                    @(negedge clk);
                    app_in_valid[src]=1; app_in_last[src]=p==len-1;
                    app_in_vl[src]=v; app_in_data[src]=pattern(id,v,len,p);
                    if(p==0) first=$time+4;
                    if(scenario!=1 && p%31==13 && p!=len-1) begin
                        @(negedge clk); app_in_valid[src]=0; app_in_last[src]=1;
                        repeat(2) @(negedge clk);
                    end
                end
                @(negedge clk);
                $fdisplay(fd,"O %0d %0d %0d %0d %0d %0d",src,id,v,len,first,$time-4);
                app_in_valid[src]=0; app_in_last[src]=0;
            end
        end
        source_done[src]=1;
    endtask
    // 抓取四路 TX 和四路 RX，方向由 src/lane 显式记录。
    byte cap[8][1526];
    int pos[8]='{default:0}, idle[8]='{default:100};
    bit active[8]='{default:0};
    time sfd[8];
    always @(posedge clk) if(started && !rst) begin : wire_monitor
        byte b;
        bit v;
        packet_t packet;
        for(int lane=0;lane<8;lane++) begin
            v=lane<4 ? txv[lane] : rxv[lane-4];
            b=lane<4 ? txd[lane*8 +:8] : rxd[(lane-4)*8 +:8];
            if(v) begin
                if(!active[lane]) begin
                    tb_check(idle[lane]>=12,"wire IFG >= 12 clocks");
                    active[lane]=1; pos[lane]=0;
                end
                if(pos[lane]>=1526) $fatal(1,"wire frame too long");
                cap[lane][pos[lane]]=b;
                if(pos[lane]<7) tb_check(b===8'h55,"wire preamble");
                if(pos[lane]==7) begin tb_check(b===8'hd5,"wire SFD"); sfd[lane]=$time; end
                if(lane<4) tb_check(!txer[lane],"no TX error");
                pos[lane]++; idle[lane]=0;
            end else begin
                idle[lane]++;
                if(active[lane]) begin
                    $fwrite(fd,"%s %0d %0d %0d %0d ",lane<4?"W":"I",(lane%4)/2,lane%2,sfd[lane],$time-8);
                    for(int p=0;p<pos[lane];p++) $fwrite(fd,"%02x",cap[lane][p]);
                    $fwrite(fd,"\n");
                    if(lane<4) begin
                        packet.id={cap[lane][50],cap[lane][51],cap[lane][52],cap[lane][53]};
                        packet.size=pos[lane]; packet.eof=$time-8; packet.data='0;
                        for(int p=0;p<pos[lane];p++) packet.data[p*8 +:8]=cap[lane][p];
                        queues[lane].put(packet);
                    end
                    active[lane]=0;
                end
            end
        end
    end
    genvar l;
    generate for(l=0;l<4;l++) begin : g_network
        localparam SRC=l/2, NET=l%2, DEST=(1-SRC)*2+NET;
        initial begin : model
            packet_t packet;
            int action, delay_ns;
            time due;
            wait(started);
            forever begin
                queues[l].get(packet);
                action=0; delay_ns=(NET==0?256:512)+extra_ns;
                if(scenario==2) case(packet.id%6)
                    1: if(NET==1) action=1;
                    2: if(NET==0) action=1;
                    3: if(NET==0) action=2;
                    4: if(NET==1) action=2;
                    5: delay_ns=NET==0?1024:256;
                    default: ;
                endcase
                $fdisplay(fd,"N %0d %0d %0d %0d %0d",SRC,NET,packet.id,action,delay_ns);
                if(action!=1) begin
                    if(action==2) packet.data[58*8]=~packet.data[58*8];
                    due=packet.eof+delay_ns;
                    @(negedge clk);
                    while($time<due) @(negedge clk);
                    for(int p=0;p<packet.size;p++) begin
                        rxd[DEST*8 +:8]=packet.data[p*8 +:8]; rxv[DEST]=1;
                        @(negedge clk);
                    end
                    rxv[DEST]=0; rxd[DEST*8 +:8]=0;
                    repeat(12) @(negedge clk);
                end
            end
        end
    end endgenerate
    byte app_cap[2][1471];
    int app_pos[2]='{0,0};
    time app_first[2];
    always @(posedge clk) if(started && !rst) begin : app_monitor
        int src, id, len, vl;
        for(int dst=0;dst<2;dst++) begin
            tb_check(app_ready[dst]===1'b1,"template ready fixed high");
            tb_check(!app_drop[dst],"no ingress rejection in allowed load");
            if(app_out_valid[dst]) begin
                if(app_pos[dst]==0) app_first[dst]=$time;
                if(app_pos[dst]>=1471) $fatal(1,"application overlength");
                app_cap[dst][app_pos[dst]]=app_out_data[dst];
                if(app_out_last[dst]) begin
                    src=1-dst; len=app_pos[dst]+1;
                    id={app_cap[dst][0],app_cap[dst][1],app_cap[dst][2],app_cap[dst][3]};
                    if(id>=nvl*rounds) $fatal(1,"unexpected application ID");
                    vl=expected_vl[src][id];
                    tb_check(seen[src][id]==0,"A/B delivered only once");
                    tb_check(len==expected_len[src][id],"application length");
                    tb_check(app_out_vl[dst]==vl,"application VL");
                    for(int p=0;p<len;p++) tb_check(app_cap[dst][p]===pattern(id,vl,len,p),"application payload");
                    seen[src][id]++; received[dst]++;
                    $fwrite(fd,"D %0d %0d %0d %0d %0d %0d ",dst,id,vl,len,app_first[dst],$time);
                    for(int p=0;p<len;p++) $fwrite(fd,"%02x",app_cap[dst][p]);
                    $fwrite(fd,"\n"); app_pos[dst]=0;
                end else app_pos[dst]++;
            end
        end
    end
    initial begin
        void'($value$plusargs("SCENARIO=%d",scenario));
        if(scenario<0 || scenario>3) $fatal(1,"invalid scenario");
        nvl=scenario==1?127:8; rounds=scenario==2?8:4; extra_ns=scenario==3?5000:0;
        capture=$sformatf("two_endpoints_%0d.txt",scenario);
        fd=$fopen(capture,"w"); if(!fd) $fatal(1,"capture open failed");
        timing_fd=$fopen($sformatf("two_endpoints_%0d_stages.txt",scenario),"w");
        if(!timing_fd) $fatal(1,"timing capture open failed");
        $fdisplay(fd,"CONFIG %0d %0d %0d %0d %0d",scenario,nvl,rounds,period_ns,extra_ns);
        for(int q=0;q<4;q++) queues[q]=new();
        repeat(24) @(negedge clk); rst=0;
        // 复位释放后立即输入半条消息，随后再次复位，检查不会泄漏。
        app_in_valid=3; app_in_last=0;
        app_in_vl[0]=1; app_in_vl[1]=1;
        repeat(5) @(negedge clk); app_in_valid=0;
        rst=1; repeat(24) @(negedge clk); rst=0;
        repeat(32) @(negedge clk);
        tb_check(app_out_valid==0 && txv==0,"partial message flushed by reset");
        epoch=$time; started=1;
        fork source(0); source(1); join
        wait(received[0]==nvl*rounds && received[1]==nvl*rounds);
        repeat(5000) @(negedge clk);
        for(int d=0;d<2;d++) begin
            tb_check(app_drops[d]==0,"no ingress drops");
            tb_check(received[d]==nvl*rounds,"exact message delivery count");
            for(int id=0;id<nvl*rounds;id++) tb_check(seen[d][id]==1,"every offered message delivered once");
        end
        $fdisplay(fd,"END %0d %0d %0d",received[0],received[1],tb_errors);
        $fclose(fd);
        $fclose(timing_fd);
        tb_finish("tb_afdx_two_endpoints"); $finish;
    end
    initial begin #25000000; $fatal(1,"two endpoint watchdog counts=%0d/%0d",received[0],received[1]); end
endmodule
