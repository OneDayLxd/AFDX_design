/*
 * ================================================================
 * File Name    : afdx_snmp_mibrom.v
 * Module Name  : afdx_snmp_mibrom
 *
 * Function:
 *   自动生成文件：由 scripts/gen_snmp_mib.py 生成，请勿手工修改。
 *   SNMP MIB ROM（按 OID 字典序）。arc(k, j) 给出条目 k 的第 j 个弧；表列条目在 j == elen 处给出实例号。
 *   elen：标量为完整 OID 弧数（含 .0）；表列为列前缀弧数（不含实例）。
 *   条目属性 e 按 k 寄存（k 变化后下一拍有效，逻辑实现以便就近布局）；elen 直接存于表中；
 *   arc 由寄存的 e 与 j、inst 组合给出：前缀弧、后缀弧和结束判定并行求出后再选择。
 *
 * Sub Module:
 *   none
 *
 * ================================================================
 */

`timescale 1ns/1ps
module afdx_snmp_mibrom (
    input  wire        clk,
    input  wire [6:0]  k,
    input  wire [3:0]  j,
    input  wire [15:0] inst,
    output reg  [31:0] arc,
    output wire [3:0]  elen,
    output wire        is_table,
    output wire [7:0]  vtype,
    output wire [1:0]  access,
    output wire [3:0]  check
);
    // {base[1:0], nsuf[2:0], s0..s3 (4x5 位), table, vtype[7:0], access[1:0], check[3:0], elen[3:0]}
    (* romstyle = "logic" *) reg [43:0] e;
    always @(posedge clk) begin
        case (k)
            7'd0: e <= {2'd0, 3'd2, 5'd1, 5'd0, 5'd0, 5'd0, 1'b0, 8'h04, 2'd0, 4'd0, 4'd9}; // sysDescr
            7'd1: e <= {2'd0, 3'd2, 5'd2, 5'd0, 5'd0, 5'd0, 1'b0, 8'h06, 2'd0, 4'd0, 4'd9}; // sysObjectID
            7'd2: e <= {2'd0, 3'd2, 5'd3, 5'd0, 5'd0, 5'd0, 1'b0, 8'h43, 2'd0, 4'd0, 4'd9}; // sysUpTime
            7'd3: e <= {2'd0, 3'd2, 5'd5, 5'd0, 5'd0, 5'd0, 1'b0, 8'h04, 2'd0, 4'd0, 4'd9}; // sysName
            7'd4: e <= {2'd1, 3'd3, 5'd1, 5'd1, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // esFwVersion
            7'd5: e <= {2'd1, 3'd3, 5'd1, 5'd2, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // esBuildId
            7'd6: e <= {2'd1, 3'd3, 5'd1, 5'd3, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // esIdentity
            7'd7: e <= {2'd1, 3'd3, 5'd1, 5'd4, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // esMonotonicUs
            7'd8: e <= {2'd1, 3'd3, 5'd2, 5'd1, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // txMsgs
            7'd9: e <= {2'd1, 3'd3, 5'd2, 5'd2, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // txFrames
            7'd10: e <= {2'd1, 3'd3, 5'd2, 5'd3, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // txFramesA
            7'd11: e <= {2'd1, 3'd3, 5'd2, 5'd4, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // txFramesB
            7'd12: e <= {2'd1, 3'd3, 5'd2, 5'd5, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // txLmaxDrop
            7'd13: e <= {2'd1, 3'd3, 5'd2, 5'd6, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // txOvfDrop
            7'd14: e <= {2'd1, 3'd3, 5'd2, 5'd7, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxFramesA
            7'd15: e <= {2'd1, 3'd3, 5'd2, 5'd8, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxFramesB
            7'd16: e <= {2'd1, 3'd3, 5'd2, 5'd9, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxFcsErr
            7'd17: e <= {2'd1, 3'd3, 5'd2, 5'd10, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxMacLenErr
            7'd18: e <= {2'd1, 3'd3, 5'd2, 5'd11, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxMacOvf
            7'd19: e <= {2'd1, 3'd3, 5'd2, 5'd12, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxHdrErr
            7'd20: e <= {2'd1, 3'd3, 5'd2, 5'd13, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxLenErr
            7'd21: e <= {2'd1, 3'd3, 5'd2, 5'd14, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxVlUnknown
            7'd22: e <= {2'd1, 3'd3, 5'd2, 5'd15, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxCsumErr
            7'd23: e <= {2'd1, 3'd3, 5'd2, 5'd16, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxFullDrop
            7'd24: e <= {2'd1, 3'd3, 5'd2, 5'd17, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxIcDrop
            7'd25: e <= {2'd1, 3'd3, 5'd2, 5'd18, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxRmDrop
            7'd26: e <= {2'd1, 3'd3, 5'd2, 5'd19, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxAccepted
            7'd27: e <= {2'd1, 3'd3, 5'd2, 5'd20, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxDelivered
            7'd28: e <= {2'd1, 3'd3, 5'd2, 5'd21, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // txBufUsed
            7'd29: e <= {2'd1, 3'd3, 5'd2, 5'd22, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // txBufPeak
            7'd30: e <= {2'd1, 3'd3, 5'd2, 5'd23, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rxBufUsed
            7'd31: e <= {2'd1, 3'd3, 5'd2, 5'd24, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rxBufPeak
            7'd32: e <= {2'd1, 3'd3, 5'd2, 5'd25, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rxErrTotal
            7'd33: e <= {2'd1, 3'd3, 5'd2, 5'd30, 5'd0, 5'd0, 1'b0, 8'h02, 2'd1, 4'd1, 4'd9}; // statsClear
            7'd34: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd2, 1'b1, 8'h42, 2'd0, 4'd0, 4'd10}; // vlId
            7'd35: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd3, 1'b1, 8'h42, 2'd0, 4'd0, 4'd10}; // vlBagUs
            7'd36: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd4, 1'b1, 8'h42, 2'd0, 4'd0, 4'd10}; // vlLmax
            7'd37: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd5, 1'b1, 8'h02, 2'd0, 4'd0, 4'd10}; // vlNetEn
            7'd38: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd6, 1'b1, 8'h42, 2'd1, 4'd5, 4'd10}; // vlShadowBagUs
            7'd39: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd7, 1'b1, 8'h42, 2'd1, 4'd6, 4'd10}; // vlShadowLmax
            7'd40: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd8, 1'b1, 8'h02, 2'd1, 4'd7, 4'd10}; // vlShadowNetEn
            7'd41: e <= {2'd1, 3'd4, 5'd3, 5'd1, 5'd1, 5'd9, 1'b1, 8'h42, 2'd0, 4'd0, 4'd10}; // vlRomLmax
            7'd42: e <= {2'd1, 3'd3, 5'd3, 5'd2, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // cfgGeneration
            7'd43: e <= {2'd1, 3'd3, 5'd3, 5'd3, 5'd0, 5'd0, 1'b0, 8'h02, 2'd1, 4'd2, 4'd9}; // cfgCommand
            7'd44: e <= {2'd1, 3'd3, 5'd3, 5'd4, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // cfgActiveBank
            7'd45: e <= {2'd1, 3'd3, 5'd3, 5'd5, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // cfgLastResult
            7'd46: e <= {2'd1, 3'd3, 5'd3, 5'd6, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // cfgBudget
            7'd47: e <= {2'd1, 3'd3, 5'd4, 5'd1, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // ldState
            7'd48: e <= {2'd1, 3'd3, 5'd4, 5'd2, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // ldObject
            7'd49: e <= {2'd1, 3'd3, 5'd4, 5'd3, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldBytes
            7'd50: e <= {2'd1, 3'd3, 5'd4, 5'd4, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldBlock
            7'd51: e <= {2'd1, 3'd3, 5'd4, 5'd5, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldCrcExpected
            7'd52: e <= {2'd1, 3'd3, 5'd4, 5'd6, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldCrcActual
            7'd53: e <= {2'd1, 3'd3, 5'd4, 5'd7, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // ldLastError
            7'd54: e <= {2'd1, 3'd3, 5'd4, 5'd8, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldFailBlock
            7'd55: e <= {2'd1, 3'd3, 5'd4, 5'd9, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldCfgVersion
            7'd56: e <= {2'd1, 3'd3, 5'd4, 5'd10, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // ldAppBank
            7'd57: e <= {2'd1, 3'd3, 5'd4, 5'd11, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldAppLength
            7'd58: e <= {2'd1, 3'd3, 5'd4, 5'd12, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldAppVersion
            7'd59: e <= {2'd1, 3'd3, 5'd4, 5'd13, 5'd0, 5'd0, 1'b0, 8'h02, 2'd1, 4'd1, 4'd9}; // ldAbort
            7'd60: e <= {2'd1, 3'd3, 5'd4, 5'd14, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // ldLoads
            7'd61: e <= {2'd1, 3'd3, 5'd4, 5'd15, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // ldUnloads
            7'd62: e <= {2'd1, 3'd3, 5'd4, 5'd16, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // ldAppCrc
            7'd63: e <= {2'd1, 3'd3, 5'd5, 5'd1, 5'd0, 5'd0, 1'b0, 8'h46, 2'd0, 4'd0, 4'd9}; // rtcTime
            7'd64: e <= {2'd1, 3'd3, 5'd5, 5'd2, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcTimeLow
            7'd65: e <= {2'd1, 3'd3, 5'd5, 5'd3, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // rtcState
            7'd66: e <= {2'd1, 3'd3, 5'd5, 5'd4, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // rtcLastOffset
            7'd67: e <= {2'd1, 3'd3, 5'd5, 5'd5, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcMaxAbsOffset
            7'd68: e <= {2'd1, 3'd3, 5'd5, 5'd6, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcPathDelay
            7'd69: e <= {2'd1, 3'd3, 5'd5, 5'd7, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // rtcFreqCorr
            7'd70: e <= {2'd1, 3'd3, 5'd5, 5'd8, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcSyncAge
            7'd71: e <= {2'd1, 3'd3, 5'd5, 5'd9, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcSyncs
            7'd72: e <= {2'd1, 3'd3, 5'd5, 5'd10, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcSteps
            7'd73: e <= {2'd1, 3'd3, 5'd5, 5'd11, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcSlews
            7'd74: e <= {2'd1, 3'd3, 5'd5, 5'd12, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcSourceSwitches
            7'd75: e <= {2'd1, 3'd3, 5'd5, 5'd13, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcLost
            7'd76: e <= {2'd1, 3'd3, 5'd5, 5'd14, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcDuplicate
            7'd77: e <= {2'd1, 3'd3, 5'd5, 5'd15, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcOutOfOrder
            7'd78: e <= {2'd1, 3'd3, 5'd5, 5'd16, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcBadFormat
            7'd79: e <= {2'd1, 3'd3, 5'd5, 5'd17, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcTimeouts
            7'd80: e <= {2'd1, 3'd3, 5'd5, 5'd18, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // rtcRejected
            7'd81: e <= {2'd1, 3'd3, 5'd5, 5'd19, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcLastSeq
            7'd82: e <= {2'd1, 3'd3, 5'd5, 5'd20, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcSourceId
            7'd83: e <= {2'd1, 3'd3, 5'd5, 5'd21, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcSourceQuality
            7'd84: e <= {2'd1, 3'd3, 5'd5, 5'd22, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // rtcLastError
            7'd85: e <= {2'd1, 3'd3, 5'd5, 5'd23, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd9, 4'd9}; // rtcStepThreshold
            7'd86: e <= {2'd1, 3'd3, 5'd5, 5'd24, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd9, 4'd9}; // rtcHoldoverUs
            7'd87: e <= {2'd1, 3'd3, 5'd5, 5'd25, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd9, 4'd9}; // rtcUnsyncUs
            7'd88: e <= {2'd1, 3'd3, 5'd5, 5'd26, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd4, 4'd9}; // rtcOnewayDelay
            7'd89: e <= {2'd1, 3'd3, 5'd5, 5'd27, 5'd0, 5'd0, 1'b0, 8'h04, 2'd1, 4'd8, 4'd9}; // rtcSet
            7'd90: e <= {2'd1, 3'd3, 5'd5, 5'd28, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // rtcLastSyncMono
            7'd91: e <= {2'd1, 3'd3, 5'd6, 5'd1, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd4, 4'd9}; // trapMask
            7'd92: e <= {2'd1, 3'd3, 5'd6, 5'd2, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd9, 4'd9}; // trapIntervalMs
            7'd93: e <= {2'd1, 3'd3, 5'd6, 5'd3, 5'd0, 5'd0, 1'b0, 8'h02, 2'd1, 4'd3, 4'd9}; // trapDstPort
            7'd94: e <= {2'd1, 3'd3, 5'd6, 5'd4, 5'd0, 5'd0, 1'b0, 8'h02, 2'd1, 4'd3, 4'd9}; // snmpPort
            7'd95: e <= {2'd1, 3'd3, 5'd6, 5'd5, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // trapSent
            7'd96: e <= {2'd1, 3'd3, 5'd6, 5'd6, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // trapDropped
            7'd97: e <= {2'd1, 3'd3, 5'd6, 5'd7, 5'd0, 5'd0, 1'b0, 8'h02, 2'd0, 4'd0, 4'd9}; // trapEventType
            7'd98: e <= {2'd1, 3'd3, 5'd6, 5'd8, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // trapEventCount
            7'd99: e <= {2'd1, 3'd3, 5'd6, 5'd9, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // trapEventFirst
            7'd100: e <= {2'd1, 3'd3, 5'd6, 5'd10, 5'd0, 5'd0, 1'b0, 8'h42, 2'd0, 4'd0, 4'd9}; // trapEventLast
            7'd101: e <= {2'd1, 3'd3, 5'd6, 5'd11, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // mgmtRxAccepted
            7'd102: e <= {2'd1, 3'd3, 5'd6, 5'd12, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // mgmtRxDropFull
            7'd103: e <= {2'd1, 3'd3, 5'd6, 5'd13, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // mgmtRxDropPort
            7'd104: e <= {2'd1, 3'd3, 5'd6, 5'd14, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // snmpInPkts
            7'd105: e <= {2'd1, 3'd3, 5'd6, 5'd15, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // snmpInBadCommunity
            7'd106: e <= {2'd1, 3'd3, 5'd6, 5'd16, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // snmpInParseErrs
            7'd107: e <= {2'd1, 3'd3, 5'd6, 5'd17, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // snmpOutPkts
            7'd108: e <= {2'd1, 3'd3, 5'd6, 5'd18, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd9, 4'd9}; // errThreshold
            7'd109: e <= {2'd1, 3'd3, 5'd6, 5'd19, 5'd0, 5'd0, 1'b0, 8'h42, 2'd1, 4'd9, 4'd9}; // linkTimeoutMs
            7'd110: e <= {2'd1, 3'd3, 5'd6, 5'd20, 5'd0, 5'd0, 1'b0, 8'h02, 2'd1, 4'd3, 4'd9}; // rtcPort
            7'd111: e <= {2'd1, 3'd3, 5'd6, 5'd21, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // tftpRetries
            7'd112: e <= {2'd1, 3'd3, 5'd6, 5'd22, 5'd0, 5'd0, 1'b0, 8'h41, 2'd0, 4'd0, 4'd9}; // tftpErrors
            7'd113: e <= {2'd2, 3'd1, 5'd0, 5'd0, 5'd0, 5'd0, 1'b0, 8'h06, 2'd2, 4'd0, 4'd11}; // snmpTrapOID
            default: e <= {2'd3, 42'd0};
        endcase
    end
    wire [1:0] base = e[43:42];
    assign elen = e[3:0];
    assign is_table = e[18];
    assign vtype = e[17:10];
    assign access = e[9:8];
    assign check = e[7:4];
    // 前缀长度 7/6/10（base 0/1/2）：前缀内判定、前缀弧与后缀序号只依赖 j 与 base
    reg in_pre;
    reg [1:0] js;
    reg [9:0] pre_arc;
    always @* begin
        case (base)
            2'd0: begin in_pre = j < 4'd7; js = j[1:0] - 2'd3; end
            2'd1: begin in_pre = j < 4'd6; js = j[1:0] - 2'd2; end
            default: begin in_pre = j < 4'd10; js = j[1:0] - 2'd2; end
        endcase
        case (j)
            4'd0: pre_arc = 10'd1;
            4'd1: pre_arc = 10'd3;
            4'd2: pre_arc = 10'd6;
            4'd3: pre_arc = 10'd1;
            4'd4: pre_arc = (base == 2'd0) ? 10'd2 : (base == 2'd1) ? 10'd3 : 10'd6;
            4'd5: pre_arc = (base == 2'd0) ? 10'd1 : (base == 2'd1) ? 10'd664 : 10'd3;
            4'd8: pre_arc = 10'd4;
            default: pre_arc = 10'd1;
        endcase
    end
    wire [4:0] suf = (js == 2'd0) ? e[38:34] : (js == 2'd1) ? e[33:29] : (js == 2'd2) ? e[28:24] : e[23:19];
    always @* begin
        if (is_table && j == elen) arc = {16'd0, inst};
        else if (j >= elen) arc = 32'd0;
        else if (in_pre) arc = {22'd0, pre_arc};
        else arc = {27'd0, suf};
    end
endmodule
