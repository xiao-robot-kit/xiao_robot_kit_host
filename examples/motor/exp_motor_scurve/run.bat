@echo off
rem ===== exp_motor_scurve Host entry: EXPLICIT mode only (never auto-starts QEMU) =====
rem User ruling 2026-11-05: the data source must be chosen explicitly; starting QEMU
rem with no argument counts as a defect.
rem This experiment has no standalone host viewer (scope_motor has no scurve scenario:
rem demo mode only reads 4-slot 64B frames, this firmware emits 2-slot 32B 0x05
rem frames), so the assertion runs through the Embedded-side run_qemu.bat probe
rem (probe_sim.py --expect-type 0x05; exit code is the verdict). This entry only
rem forwards the mode explicitly.
rem
rem Usage:
rem   run.bat qemu [port] [seconds]  = QEMU chain (Embedded run_qemu.bat)
rem   run.bat                        = prints usage and exits (never starts QEMU)
rem   run.bat sim                    = explains where the synthetic source lives
rem
rem Real board: flash per the Embedded README and verify with a serial host tool;
rem this entry does not forward --serial (the probe is QEMU-only).
rem (This file stays ASCII-only for cmd codepage safety.)
setlocal
if "%~1"=="" goto :usage
if /i "%~1"=="qemu" goto :qemu
if /i "%~1"=="sim" goto :sim
goto :usage

:qemu
cd /d "%~dp0..\..\..\..\FirmwareProject\examples\motor\exp_motor_scurve"
call run_qemu.bat %2 %3
set "RC=%ERRORLEVEL%"
goto :done

:sim
echo [exp_motor_scurve] sim: the synthetic source lives on the Embedded side.
echo                     Use -DEXP_SIM=1 per the Embedded README (probe is QEMU-only).
goto :done

:usage
echo usage: run.bat qemu [port] [seconds]
echo        run.bat   (prints this message; never auto-starts QEMU)
echo        QEMU chain: FirmwareProject\examples\motor\exp_motor_scurve\run_qemu.bat

:done
endlocal & exit /b %RC%
