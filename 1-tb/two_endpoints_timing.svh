// Included inside each endpoint generate scope. Times use pre-NBA handshakes.
`define TIMING_CORE dut.u_mac_system.u_system.u_core
int timing_pos[3]='{default:0};
int timing_id[3]='{default:0};
time timing_first[3];
int timing_grants[128]='{default:0};
always @(posedge clk) if(started && !rst) begin : stage_monitor
    bit beat[3];
    byte data_byte[3];
    int id, vl;
    beat[0]=dut.host_tx_valid && dut.host_tx_ready && dut.startup[4];
    beat[1]=`TIMING_CORE.tx_in_beat && !`TIMING_CORE.tx_bad;
    beat[2]=`TIMING_CORE.rxq_wr_valid && `TIMING_CORE.rxq_wr_ready;
    data_byte[0]=dut.host_tx_data;
    data_byte[1]=`TIMING_CORE.txh_data;
    data_byte[2]=`TIMING_CORE.rxq_wr_data;
    for(int k=0;k<3;k++) if(beat[k]) begin
        if(timing_pos[k]==0) begin timing_first[k]=$time; timing_id[k]=0; end
        if(timing_pos[k]<4) timing_id[k]=(timing_id[k]<<8)|int'(unsigned'(data_byte[k]));
        timing_pos[k]++;
    end
    if(beat[0] && dut.host_tx_last) begin
        $fdisplay(timing_fd,"H %0d %0d %0d %0d %0d -1 %0d",e,timing_id[0],dut.host_tx_vl,timing_first[0],$time,timing_pos[0]);
        timing_pos[0]=0;
    end
    if(beat[1] && `TIMING_CORE.txh_last) begin
        $fdisplay(timing_fd,"Q %0d %0d %0d %0d %0d -1 %0d",e,timing_id[1],`TIMING_CORE.txh_vl,timing_first[1],$time,timing_pos[1]);
        timing_pos[1]=0;
    end
    if(`TIMING_CORE.grant_ack) begin
        vl=`TIMING_CORE.grant_vl;
        // 本 TB 同 VL 保序，ID = round*nvl + vl-1；分析器再与 Q/W 载荷交叉验证。
        id=timing_grants[vl]*nvl+vl-1;
        timing_grants[vl]++;
        $fdisplay(timing_fd,"G %0d %0d %0d %0d %0d -1 0",e,id,vl,$time,$time);
    end
    if(`TIMING_CORE.rxq_commit_ok) begin
        $fdisplay(timing_fd,"R %0d %0d %0d %0d %0d %0d %0d",e,timing_id[2],`TIMING_CORE.rxq_wr_vl,timing_first[2],$time,`TIMING_CORE.rxq_wr_meta[8],timing_pos[2]);
        timing_pos[2]=0;
    end
end
`undef TIMING_CORE
