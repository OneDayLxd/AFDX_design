# 编译文件清单，加载指定 testbench，运行并保留波形。
onerror {if {[info exists BATCH_MODE] && $BATCH_MODE} {quit -force -code 1} else {abort}}
if {![info exists BATCH_MODE]} {set BATCH_MODE 0}
if {![info exists TB_TOP] || ![regexp {^tb_afdx_[a-z0-9_]+$} $TB_TOP]} {
    echo "ERROR: use run_ModelSim tb_afdx_top.sv"
    if {$BATCH_MODE} {quit -force -code 1}
    return
}

quit -sim
# 由启动器先切换到 2-sim，所有生成文件都留在此目录。
if {![file isfile flist.f] || ![file isfile ../1-tb/$TB_TOP.sv]} {
    echo "ERROR: start ModelSim in 2-sim and select an existing testbench."
    if {$BATCH_MODE} {quit -force -code 1}
    return
}

# 各份配置 ROM 定义了同名模块，只能选择其中一份，不能一起编译。
set cfg_header ../0-rtl/1-config/afdx_cfg_params.vh
set cfg_rom ../0-rtl/1-config/afdx_vl_config.v
switch -- $TB_TOP {
    tb_afdx_top - tb_afdx_protocol {
        set cfg_header ../1-tb/afdx_cfg_params.vh
        set cfg_rom ../1-tb/afdx_test_config.v
    }
    tb_afdx_mgmt_e2e {
        set cfg_header ../1-tb/afdx_cfg_params.vh
        set cfg_rom ../1-tb/afdx_mgmt_test_config.v
    }
    tb_afdx_scale {
        set cfg_header ../1-tb/scale/afdx_cfg_params.vh
        set cfg_rom ../1-tb/scale/afdx_vl_config.v
    }
}

# 仅两个文件驱动的 testbench 需要默认文件参数，不调用 Python 分析。
if {![info exists SIM_ARGS]} {set SIM_ARGS {}}
set default_args {}
if {$TB_TOP eq "tb_afdx_mgmt_e2e"} {
    set default_args {+SCRIPT=../1-tb/stimuli/smoke.txt +CAP=capture.txt}
} elseif {$TB_TOP eq "tb_afdx_latency"} {
    set default_args {+CSV=latency.csv +SIM_MS=20 +MGMT_LOAD=0}
}
# 手工指定的同名 plusarg 覆盖默认值。
set sim_options [dict create]
foreach arg [concat $default_args $SIM_ARGS] {
    dict set sim_options [lindex [split $arg =] 0] $arg
}

if {[catch {
    if {![file isfile modelsim.ini]} {vmap -c}
    if {![file isdirectory work]} {vlib work}
    vmap work [file normalize work]
    # -mfcu 使最先加载的参数头在整个编译中生效，防止误用同名头文件。
    vlog -sv -mfcu -timescale 1ns/1ps -work work $cfg_header -f flist.f $cfg_rom ../1-tb/$TB_TOP.sv
    vsim -onfinish stop -voptargs=+acc -wlf vsim.wlf work.$TB_TOP {*}[dict values $sim_options]
    if {!$BATCH_MODE} {
        log -r /*
        view wave
        do wave.do
    }
    onbreak {}
    run -all
} sim_error]} {
    echo "ERROR: $sim_error"
    if {$BATCH_MODE} {quit -force -code 1}
    return
}

if {$BATCH_MODE} {
    quit -force -code 0
} else {
    configure wave -timelineunits ns
    wave zoom full
}
