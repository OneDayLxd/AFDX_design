# 默认显示 testbench 信号和 DUT 端口；内部信号按需在此追加。
quietly WaveActivateNextPane {} 0
delete wave *
add wave -group Testbench sim:/$TB_TOP/*
if {[llength [find instances sim:/$TB_TOP/dut]]} {
    add wave -group DUT -ports sim:/$TB_TOP/dut/*
}
configure wave -namecolwidth 260
configure wave -valuecolwidth 120
configure wave -timelineunits ns
