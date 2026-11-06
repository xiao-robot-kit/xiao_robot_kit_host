@echo off
rem ===== exp3 Euler-angle example launcher (Python stdlib only, no third-party deps) =====
rem The data source has exactly two explicit modes; nothing starts on its own:
rem   run.bat                      idle: serve the page only, no data source
rem                                (pick "Start sim" or "Connect real" in the page)
rem   run.bat sim                  sim: local synthetic stream (firmware preset
rem                                sim.exp3.sway: pitch 20deg / 4s / 100Hz)
rem   run.bat real                 real: connect --tcp default 127.0.0.1:5558
rem                                (QEMU UART0 bridge; start run_qemu.bat first)
rem   run.bat real 127.0.0.1:5999  real: connect the given host:port
rem Extra args pass through: --http-port, --lang zh or en, --sim-amp-deg, ...
rem The old form --source sim or tcp still works as an alias of --mode sim or real.
rem Console stays ASCII on purpose: Chinese text here breaks cmd parsing under CP936.
setlocal
cd /d "%~dp0"

rem First token = mode alias; the rest is collected into TAIL (SHIFT does not touch %*)
set "FIRST=%~1"
set "SECOND="
set "TAIL="
:collect
shift
if "%~1"=="" goto :dispatch
if not defined SECOND set "SECOND=%~1"
set "TAIL=%TAIL% "%~1""
goto :collect

:dispatch
if "%FIRST%"=="" goto :idle
if /i "%FIRST%"=="sim" goto :sim
if /i "%FIRST%"=="real" goto :real
rem First token is a flag (--mode / --source / --tcp / --http-port ...): pass everything through
python euler_view.py %*
goto :done

:idle
python euler_view.py %TAIL%
goto :done

:sim
python euler_view.py --mode sim %TAIL%
goto :done

:real
if "%SECOND%"=="" (
  python euler_view.py --mode real
  goto :done
)
if "%SECOND:~0,2%"=="--" (
  rem flags only, no address: keep the default --tcp address
  python euler_view.py --mode real %TAIL%
) else (
  rem first TAIL token is host:port, the rest keeps passing through
  python euler_view.py --mode real --tcp %TAIL%
)
goto :done

:done
endlocal
