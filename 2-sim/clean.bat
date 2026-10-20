@echo off
setlocal
cd /d "%~dp0"

rem Close ModelSim first. Only remove generated files in this script's directory.
for %%F in (*.log *.wlf transcript capture.txt latency.csv) do (
    if exist "%%~fF" (
        echo Removing %%~fF
        del /q "%%~fF"
    )
)
endlocal
