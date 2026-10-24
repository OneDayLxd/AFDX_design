/*
 * ================================================================
 * File Name    : tb_afdx_post_synth.sv
 * Module Name  : tb_afdx_post_synth
 *
 * Function:
 *   仅通过生产顶层端口验证综合网表，不覆盖参数、不访问 RTL 内部层次。
 *   检查 A/B 发送、帧校验、SNMP 往返、冗余去重及坏 FCS 隔离。
 * Sub Module:
 *   AFDX_top
 * ================================================================
 */
`timescale 1ns/1ps
`default_nettype none
module tb_afdx_post_synth;
    `include "tb_common.svh"
    reg         clk = 0;
    reg         rst = 1;
    reg [15:0]  rxd = 0;
    reg [1:0]   rxv = 0;
    reg [1:0]   ready = 0;
    wire [15:0] txd;
    wire [1:0]  txv;
    wire [1:0]  tx_ready;
    always #4 clk = ~clk;

    AFDX_top dut (
        .clk         (clk),
        .rst         (rst),
        .tx_data     (txd[7:0]),
        .rx_data     (rxd[7:0]),
        .tx_valid    (txv[0]),
        .rx_valid    (rxv[0]),
        .tx_ready    (tx_ready[0]),
        .rx_ready    (ready[0]),
        .b_tx_data   (txd[15:8]),
        .b_rx_data   (rxd[15:8]),
        .b_tx_valid  (txv[1]),
        .b_rx_valid  (rxv[1]),
        .b_tx_ready  (tx_ready[1]),
        .b_rx_ready  (ready[1])
    );

    byte unsigned frame [0:1599];
    byte unsigned payload [0:255];
    byte unsigned cap [0:1][0:1599];
    integer pos [0:1] = '{0, 0};
    integer idle [0:1] = '{100, 100};
    integer frames [0:1] = '{0, 0};
    integer apps [0:1] = '{0, 0};
    integer responses [0:1][0:2];
    bit active [0:1] = '{0, 0};
    reg [31:0] crc [0:1];
    integer frame_len;
    integer fd, net, i;
    reg [1023:0] cap_name;

    function automatic [15:0] checksum(input reg [31:0] sum_in);
        reg [31:0] sum;
        begin
            sum = (sum_in & 32'hffff) + (sum_in >> 16);
            sum = (sum & 32'hffff) + (sum >> 16);
            checksum = ~sum[15:0];
        end
    endfunction

    // 三个标量依次为 rxAccepted、rxRmDrop、rxErrTotal；独立编码请求。
    // 外部 MAC 的坏 FCS 计入综合 MAC 错误，因此从错误合计读取。
    task automatic build_request(input integer id, input [7:0] sn);
        integer j, q, n, oid_col;
        reg [31:0] sum, c;
        reg [15:0] ck;
        reg [151:0] prefix;
        begin
            prefix = 152'h300002010104067075626c6963a00002021200;
            for (j=0; j<19; j++) payload[j] = prefix[(18-j)*8 +: 8];
            payload[18]=id; payload[19]=2; payload[20]=1; payload[21]=0;
            payload[22]=2; payload[23]=1; payload[24]=0;
            payload[25]=8'h30;
            q=27;
            for(j=0;j<3;j++) begin
                oid_col = (j==0) ? 19 : (j==1) ? 18 : 25;
                payload[q]=8'h30; payload[q+1]=13;
                payload[q+2]=6; payload[q+3]=9;
                payload[q+4]=8'h2b; payload[q+5]=6; payload[q+6]=1;
                payload[q+7]=3; payload[q+8]=8'h85; payload[q+9]=8'h18;
                payload[q+10]=2; payload[q+11]=oid_col; payload[q+12]=0;
                payload[q+13]=5; payload[q+14]=0;
                q=q+15;
            end
            n=q;
            payload[1]=n-2; payload[14]=n-15; payload[26]=n-27;
            for(j=0;j<1600;j++) frame[j]=0;
            frame[0]=3; frame[4]=8'h10; frame[5]=0;
            frame[6]=2; frame[9]=1; frame[10]=1; frame[11]=8'h20;
            frame[12]=8'h08; frame[13]=0;
            frame[14]=8'h45; frame[16]=(28+n)>>8; frame[17]=28+n;
            frame[19]=sn; frame[22]=1; frame[23]=17;
            frame[26]=10; frame[27]=1; frame[28]=1; frame[29]=1;
            frame[30]=224; frame[31]=224; frame[32]=16; frame[33]=0;
            sum=0;
            for(j=14;j<34;j=j+2) sum=sum+{frame[j],frame[j+1]};
            ck=checksum(sum); frame[24]=ck[15:8]; frame[25]=ck[7:0];
            frame[34]=8'hc3; frame[35]=id; frame[36]=0; frame[37]=161;
            frame[38]=(8+n)>>8; frame[39]=8+n;
            for(j=0;j<n;j++) frame[42+j]=payload[j];
            sum=17+8+n;
            for(j=26;j<34;j=j+2) sum=sum+{frame[j],frame[j+1]};
            for(j=34;j<42+n;j=j+2)
                sum=sum+{frame[j],(j+1<42+n) ? frame[j+1] : 8'h00};
            ck=checksum(sum); if(ck==0) ck=16'hffff;
            frame[40]=ck[15:8]; frame[41]=ck[7:0];
            frame[42+n]=sn;
            c=32'hffffffff;
            for(j=0;j<=42+n;j++) c=tb_crc32_byte(c,frame[j]);
            for(j=0;j<4;j++) frame[43+n+j]=~(c>>(8*j));
            frame_len=47+n;
        end
    endtask

    task automatic inject(input [1:0] mask, input bit bad_a);
        integer j;
        reg [7:0] d;
        begin
            $display("INJECT id=%0d mask=%b bad_a=%0d at %0t",payload[18],mask,bad_a,$time);
            for(j=0;j<frame_len+8;j++) begin
                @(negedge clk);
                d=(j<7) ? 8'h55 : (j==7) ? 8'hd5 : frame[j-8];
                rxv=mask;
                rxd={d, (bad_a && j==frame_len+7) ? (d^8'h01) : d};
            end
            @(negedge clk); rxv=0; rxd=0;
            repeat(16) @(negedge clk);
        end
    endtask

    task automatic inspect_frame(input integer link);
        integer j, n, id, q, vlen, k, expected;
        reg [31:0] sum, value;
        begin
            n=pos[link];
            tb_check(n>=72 && n<=1526,"complete GMII frame length");
            tb_check(crc[link]===32'hdebb20e3,"Ethernet FCS residue");
            sum=0;
            for(j=22;j<42;j=j+2) sum=sum+{cap[link][j],cap[link][j+1]};
            tb_check(checksum(sum)==0,"IPv4 header checksum");
            $fwrite(fd,"F %0t %0d %0d",$time,link,n);
            for(j=0;j<n;j++) $fwrite(fd," %02x",cap[link][j]);
            $fwrite(fd,"\n");
            frames[link]++;
            if(cap[link][44]==8'hc3 && cap[link][45]>=1 && cap[link][45]<=2) begin
                id=cap[link][45]; responses[link][id]++;
                // GMII 前导码占 8 字节，UDP 负载从偏移 50 开始。
                tb_check(cap[link][63]==8'ha2,"SNMP GetResponse PDU");
                tb_check(cap[link][67]==8'h12 && cap[link][68]==id,"SNMP request ID preserved");
                tb_check(cap[link][71]==0 && cap[link][74]==0,"SNMP success status/index");
                q=77;
                for(j=0;j<3;j++) begin
                    tb_check(cap[link][q]==8'h30 && cap[link][q+2]==6 &&
                             cap[link][q+3]==9,"SNMP response varbind OID shape");
                    tb_check(cap[link][q+4]==8'h2b && cap[link][q+5]==6 &&
                             cap[link][q+6]==1 && cap[link][q+7]==3 &&
                             cap[link][q+8]==8'h85 && cap[link][q+9]==8'h18 &&
                             cap[link][q+10]==2 && cap[link][q+11]==((j==0)?19:(j==1)?18:25) &&
                             cap[link][q+12]==0,"SNMP response matches requested OID");
                    tb_check(cap[link][q+13]==8'h41,"SNMP counter32 type");
                    vlen=cap[link][q+14]; value=0;
                    tb_check(vlen>=1 && vlen<=5,"SNMP counter32 length");
                    for(k=0;k<vlen;k++) value=(value<<8)|cap[link][q+15+k];
                    expected=(j==0) ? id : (j==1) ? 1 : id-1;
                    tb_check(value==expected,$sformatf("request %0d counter %0d = %0d (expected %0d)",id,j,value,expected));
                    q=q+2+cap[link][q+1];
                end
                $display("RESPONSE link=%0d id=%0d at %0t",link,id,$time);
            end else if(cap[link][12]==8'h10 && cap[link][13]!=0) apps[link]++;
        end
    endtask

    always @(posedge clk) begin
        if(!rst) begin
            tb_check(tx_ready===2'b11,"template tx_ready tied high");
            tb_check(!$isunknown(txv),"TX valid is known");
            for(net=0;net<2;net++) begin
                if(txv[net]===1'b1) begin
                    tb_check(!$isunknown(txd[net*8 +: 8]),"TX byte is known");
                    if(!active[net]) begin
                        tb_check(idle[net]>=12,"GMII IFG >=12 byte times");
                        active[net]=1; pos[net]=0; crc[net]=32'hffffffff;
                    end
                    if(pos[net]>=1600) $fatal(1,"unterminated GMII frame");
                    cap[net][pos[net]]=txd[net*8 +: 8];
                    if(pos[net]<7) tb_check(txd[net*8 +: 8]===8'h55,"GMII preamble");
                    else if(pos[net]==7) tb_check(txd[net*8 +: 8]===8'hd5,"GMII SFD");
                    else crc[net]=tb_crc32_byte(crc[net],txd[net*8 +: 8]);
                    pos[net]++; idle[net]=0;
                end else begin
                    if(active[net]) begin inspect_frame(net); active[net]=0; end
                    idle[net]++;
                end
            end
        end
    end

    task automatic wait_response(input integer id);
        integer cycles;
        begin
            cycles=0;
            // 复位通知 Trap 先占用 VL0，后续请求仍须遵守生产 BAG=2ms。
            while((responses[0][id]==0 || responses[1][id]==0) && cycles<600000) begin
                @(negedge clk); cycles++;
                if(cycles%50000==0) $display("WAIT id=%0d time=%0t frames=%0d/%0d",id,$time,frames[0],frames[1]);
            end
            if(responses[0][id]==0 || responses[1][id]==0)
                $fatal(1,"no SNMP response id=%0d at production GMII ports",id);
        end
    endtask

    initial begin
        if(!$value$plusargs("CAP=%s",cap_name)) cap_name="post_synth_capture.txt";
        fd=$fopen(cap_name,"w"); if(fd==0) $fatal(1,"cannot open capture");
        for(i=0;i<3;i++) begin responses[0][i]=0; responses[1][i]=0; end
        repeat(40) @(negedge clk); rst=0;
        // 给 4096 VL 状态表的复位扫描留足时间，不强制内部寄存器。
        repeat(12500) @(negedge clk);
        ready=2'b01;
        build_request(1,1); inject(2'b11,0);
        wait_response(1);
        // A 网坏 FCS，B 网有效；两路外部 ready 交换后仍应正常处理。
        ready=2'b10;
        build_request(2,2); inject(2'b11,1);
        wait_response(2);
        repeat(1000) @(negedge clk);
        tb_check(apps[0]>0 && apps[1]>0,"production application generator sends on both networks");
        for(i=0;i<2;i++) begin
            tb_check(responses[i][1]==1 && responses[i][2]==1,"one response per request per network");
            tb_check(!active[i],"last frame completed");
        end
        $fclose(fd);
        $display("SUMMARY frames A/B=%0d/%0d apps=%0d/%0d",frames[0],frames[1],apps[0],apps[1]);
        tb_finish("tb_afdx_post_synth"); $finish;
    end
    initial begin #10_000_000; $fatal(1,"post-synthesis watchdog"); end
endmodule
`default_nettype wire
