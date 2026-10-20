@echo off
setlocal
cd /d "%~dp0"

if "%~1"=="" (
    echo Usage: .\run_ModelSim tb_afdx_top.sv
    exit /b 1
)
for %%I in ("%~1") do set "tb_top=%%~nI"
if not exist "..\1-tb\%tb_top%.sv" (
    echo ERROR: Testbench not found: %tb_top%.sv
    exit /b 1
)

set "PATH=D:\Softwares\ModelSim_SE_2020.4\win64;%PATH%"
if not defined MGLS_LICENSE_FILE if not defined LM_LICENSE_FILE set "MGLS_LICENSE_FILE=D:\Softwares\ModelSim_SE_2020.4\LICENSE.TXT"
set "modelsim_do=%~dp0ModelSim.do"
set "modelsim_do=%modelsim_do:\=/%"
start "" /b vsim -l "%~dp0sim.log" -do "set TB_TOP {%tb_top%}; do {%modelsim_do%}" > "%~dp0modelsim_console.log" 2>&1
set "vsim_rc=%ERRORLEVEL%"
endlocal & exit /b %vsim_rc%
