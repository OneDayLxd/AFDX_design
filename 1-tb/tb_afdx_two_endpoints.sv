/*
 * ================================================================
 * File Name    : tb_afdx_two_endpoints.sv
 * Module Name  : tb_afdx_two_endpoints
 * Function:
 *   两个生产端系统通过A/B GMII 直连双向通信，无应用回送。
 *   应用端仅使用 valid；抓取全部应用消息及 GMII 帧供独立检查。
 *   0:变长多 VL；1:127 业务 VL 吞吐；2:冗余故障；3:TB 延迟对照。
 *   4:每种长度重复三次；5:20 VL、100 Mbps 持续应用载荷输入。
 *   6:127 业务 VL 均按 2ms BAG、1471 字节上限输入，覆盖配置满负载。
 * Sub Module:
 *   AFDX_top
 * ================================================================
 */
`timescale 1ns/1ps
`include "afdx_cfg_params.vh"
module tb_afdx_two_endpoints;
    `include "tb_common.svh"
    reg clk=0, rst=1, started=0;
    always #4 clk=~clk;
    reg [7:0] app_in_data[2]='{0,0};
    reg [1:0] app_in_valid=0, app_in_last=0;
    wire [7:0] app_out_data[2];
    wire [1:0] app_out_valid, app_out_last, app_ready, app_drop;
    wire [2:0] app_reason[2];
    wire [31:0] app_drops[2];
    wire [31:0] txd;
    wire [3:0] txv, txer;
    wire [31:0] rxd;
    wire [3:0] rxv;
    int scenario=0, nvl, rounds, period_ns=2500000, extra_ns, fd, timing_fd;
    // rx_ready 交替变化，输出载荷必须完全不受其影响。
    reg [1:0] ignored_ready=0;
    always @(negedge clk) ignored_ready<=ignored_ready+1'b1;
    AFDX_top #(
        .MGMT_EN            (0)
    ) dut (
        .clk                (clk),
        .rst                (rst),
        .tx_data            ({app_in_data[1],app_in_data[0]}),
        .tx_valid           (app_in_valid),
        .tx_last            (app_in_last),
        .tx_ready           (app_ready),
        .rx_data            ({app_out_data[1],app_out_data[0]}),
        .rx_valid           (app_out_valid),
        .rx_last            (app_out_last),
        .rx_ready           (ignored_ready),
        .app_drop           (app_drop),
        .app_drop_reason    ({app_reason[1],app_reason[0]}),
        .app_drop_count     ({app_drops[1],app_drops[0]})
    );
    // 普通业务只驱动应用端口；下方故障/延迟场景单独覆盖接收连线。
    assign txd=dut.gmii_tx_data;
    assign txv=dut.gmii_tx_valid;
    assign txer=dut.gmii_tx_er;
    assign rxd=dut.gmii_rx_data;
    assign rxv=dut.gmii_rx_valid;
    genvar e;
    generate for(e=0;e<2;e++) begin : g_observe
        `include "two_endpoints_timing.svh"
        task automatic check_rate_counters;
            tb_check(dut.g_endpoint[e].u_es.u_mac_system.rx_overflow_count==64'd0,"100Mbps A/B MAC queues have no overflow");
            tb_check(dut.g_endpoint[e].u_es.u_mac_system.u_system.cnt_tx_ovf_drop==0,"100Mbps TX queue has no overflow");
            tb_check(dut.g_endpoint[e].u_es.u_mac_system.u_system.cnt_rx_full_drop==0,"100Mbps RX message pool has no overflow");
        endtask
    end endgenerate
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
        if(scenario==1 || scenario==5 || scenario==6) return 1471;
        case((scenario==4 ? round_id/3 : round_id)%4)
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
            if(scenario!=5) while($time<epoch+round_id*period_ns) @(negedge clk);
            for(int v=1;v<=nvl;v++) begin
                id=round_id*nvl+v-1; len=size_at(round_id);
                expected_len[src][id]=len; expected_vl[src][id]=v;
                // 100 Mbps 持续载荷输入：每个载荷字节占 80 ns，VLID 使用字节间空隙。
                if(scenario==5) while($time<epoch+id*1471*80) @(negedge clk);
                @(negedge clk); app_in_valid[src]=1; app_in_last[src]=0;
                app_in_data[src]=8'h10; first=$time+4;
                @(negedge clk); app_in_data[src]=byte'(v);
                for(int p=0;p<len;p++) begin
                    @(negedge clk);
                    app_in_valid[src]=1; app_in_last[src]=p==len-1;
                    app_in_data[src]=pattern(id,v,len,p);
                    if(scenario==5 && p!=len-1) begin
                        @(negedge clk); app_in_valid[src]=0; app_in_last[src]=0;
                        repeat(8) @(negedge clk);
                    end else if(scenario!=1 && scenario!=5 && scenario!=6 && p%31==13 && p!=len-1) begin
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
                    active[lane]=0;
                end
            end
        end
    end
    // 故障仅由 TB 覆盖接收连线；普通场景不覆盖 DUT 的直接连接。
    wire [6:0] frame_vl[4];
    genvar l;
    generate for(l=0;l<4;l++) begin : g_fault_observe
        if(l%2==0) assign frame_vl[l]=dut.g_endpoint[l/2].u_es.u_mac_system.u_system.u_core.u_mac_tx_a.meta_vl;
        else assign frame_vl[l]=dut.g_endpoint[l/2].u_es.u_mac_system.u_system.u_core.u_mac_tx_b.meta_vl;
    end endgenerate
    int lane_round[4][128];
    int fault_pos[4]='{default:0}, fault_action[4]='{default:0};
    bit fault_active[4]='{default:0};
    reg [31:0] inject_data=0;
    reg [3:0] inject_valid=0;
    reg [31:0] delayed_data=0;
    reg [3:0] delayed_valid=0;
    // 非阻塞传输延迟保留每个字节/有效边沿，不能用会滤掉短帧的惯性延迟。
    always @(inject_data) delayed_data <= #5000 inject_data;
    always @(inject_valid) delayed_valid <= #5000 inject_valid;
    // 在 GMII 下个采样沿前决定整帧丢弃；线上 ID 与此观察值由检查器独立交叉核对。
    always @(negedge clk) begin : faults
        int id,vl;
        if(rst) begin
            inject_data=0;inject_valid=0;
            for(int lane=0;lane<4;lane++)begin
                fault_active[lane]=0;fault_pos[lane]=0;fault_action[lane]=0;
                for(int v=0;v<128;v++)lane_round[lane][v]=0;
            end
        end else for(int lane=0;lane<4;lane++)begin
            if(txv[lane] && !fault_active[lane])begin
                vl=frame_vl[lane];
                id=lane_round[lane][vl]*nvl+vl-1;
                lane_round[lane][vl]++;
                fault_active[lane]=1;fault_pos[lane]=0;fault_action[lane]=0;
                if(scenario==2)case(id%6)
                    1: if(lane%2==1)fault_action[lane]=1;
                    2: if(lane%2==0)fault_action[lane]=1;
                    3: if(lane%2==0)fault_action[lane]=2;
                    4: if(lane%2==1)fault_action[lane]=2;
                    default: ;
                endcase
                if(started)$fdisplay(fd,"N %0d %0d %0d %0d %0d",lane/2,lane%2,id,fault_action[lane],extra_ns);
            end
            inject_valid[lane^2]=txv[lane] && fault_action[lane]!=1;
            inject_data[(lane^2)*8+:8]=txd[lane*8+:8] ^
                ((fault_action[lane]==2 && fault_pos[lane]==58) ? 8'h01:8'h00);
            if(txv[lane])fault_pos[lane]++;
            else begin fault_active[lane]=0;fault_pos[lane]=0;end
        end
    end
    // 从实际输入端口抓包，独立核对 VLID、载荷和输入吞吐。
    byte input_cap[2][1473];
    int input_pos[2]='{0,0};
    time input_first[2], input_payload_first[2];
    always @(posedge clk) if(started && !rst) begin : input_monitor
        int id, vlid;
        for(int src=0;src<2;src++) if(app_in_valid[src]) begin
            if(input_pos[src]==0) input_first[src]=$time;
            if(input_pos[src]==2) input_payload_first[src]=$time;
            if(input_pos[src]>=1473) $fatal(1,"input overlength");
            input_cap[src][input_pos[src]]=app_in_data[src];
            if(app_in_last[src]) begin
                id={input_cap[src][2],input_cap[src][3],input_cap[src][4],input_cap[src][5]};
                vlid={input_cap[src][0],input_cap[src][1]};
                $fwrite(fd,"A %0d %0d %0d %0d %0d %0d ",src,id,vlid,input_first[src],input_payload_first[src],$time);
                for(int p=0;p<=input_pos[src];p++) $fwrite(fd,"%02x",input_cap[src][p]);
                $fwrite(fd,"\n"); input_pos[src]=0;
            end else input_pos[src]++;
        end
    end
    byte app_cap[2][1473];
    int app_pos[2]='{0,0};
    time app_first[2];
    always @(posedge clk) if(started && !rst) begin : app_monitor
        int src, id, len, vl;
        for(int dst=0;dst<2;dst++) begin
            tb_check(app_ready[dst]===1'b1,"template ready fixed high");
            tb_check(!app_drop[dst],"no ingress rejection in allowed load");
            if(app_out_valid[dst]) begin
                if(app_pos[dst]==0) app_first[dst]=$time;
                if(app_pos[dst]>=1473) $fatal(1,"application overlength");
                app_cap[dst][app_pos[dst]]=app_out_data[dst];
                if(app_out_last[dst]) begin
                    src=1-dst; len=app_pos[dst]-1;
                    id={app_cap[dst][2],app_cap[dst][3],app_cap[dst][4],app_cap[dst][5]};
                    if(id>=nvl*rounds) $fatal(1,"unexpected application ID");
                    vl=expected_vl[src][id];
                    tb_check(seen[src][id]==0,"A/B delivered only once");
                    tb_check(len==expected_len[src][id],"application length");
                    tb_check({app_cap[dst][0],app_cap[dst][1]}==16'h1000+vl,"actual application VLID");
                    for(int p=0;p<len;p++) tb_check(app_cap[dst][p+2]===pattern(id,vl,len,p),"application payload");
                    seen[src][id]++; received[dst]++;
                    $fwrite(fd,"D %0d %0d %0d %0d %0d %0d ",dst,id,vl,len,app_first[dst],$time);
                    for(int p=0;p<len+2;p++) $fwrite(fd,"%02x",app_cap[dst][p]);
                    $fwrite(fd,"\n"); app_pos[dst]=0;
                end else app_pos[dst]++;
            end
        end
    end
    initial begin
        void'($value$plusargs("SCENARIO=%d",scenario));
        if(scenario<0 || scenario>6) $fatal(1,"invalid scenario");
        if(scenario==2)begin
            force dut.gmii_rx_data=inject_data;
            force dut.gmii_rx_valid=inject_valid;
        end
        if(scenario==3)begin
            force dut.gmii_rx_data=delayed_data;
            force dut.gmii_rx_valid=delayed_valid;
        end
        nvl=(scenario==1 || scenario==6)?127:scenario==5?20:8;
        rounds=(scenario==2 || scenario==6)?8:scenario==4?12:4;
        extra_ns=scenario==3?5000:0;
        if(scenario==5) period_ns=1471*80;
        if(scenario==6) period_ns=2000000;
        capture=$sformatf("two_endpoints_%0d.txt",scenario);
        fd=$fopen(capture,"w"); if(!fd) $fatal(1,"capture open failed");
        timing_fd=$fopen($sformatf("two_endpoints_%0d_stages.txt",scenario),"w");
        if(!timing_fd) $fatal(1,"timing capture open failed");
        $fdisplay(fd,"CONFIG %0d %0d %0d %0d %0d",scenario,nvl,rounds,period_ns,extra_ns);
        repeat(24) @(negedge clk); rst=0;
        // 复位释放后立即输入半条消息，随后再次复位，检查不会泄漏。
        app_in_valid=3; app_in_last=0;
        app_in_data[0]=8'h10; app_in_data[1]=8'h10;
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
        if(scenario==5 || scenario==6) begin
            g_observe[0].check_rate_counters();
            g_observe[1].check_rate_counters();
        end
        $fdisplay(fd,"END %0d %0d %0d",received[0],received[1],tb_errors);
        $fclose(fd);
        $fclose(timing_fd);
        tb_finish("tb_afdx_two_endpoints"); $finish;
    end
    initial begin #40000000; $fatal(1,"two endpoint watchdog counts=%0d/%0d",received[0],received[1]); end
endmodule
