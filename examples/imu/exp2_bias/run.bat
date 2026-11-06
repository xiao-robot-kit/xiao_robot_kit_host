@echo off
rem ===== exp2_bias entry (server architecture): one-click local server =====
rem Host = ONE local server program (backend\server.py) + the page stored in THIS
rem example folder. This bat guarantees a single instance:
rem   * server not running -> start it (foreground console = server; close to stop)
rem   * server already up  -> kill it and start a fresh one (no port/device contention)
rem Page: http://127.0.0.1:8760/exp2/  (opened in your browser automatically)
rem Data source mode stays EXPLICIT: nothing starts until you press
rem [Sim | Real] -> Start/Connect; default is Real (no auto-sim, no auto-QEMU).
rem (ASCII-only on purpose: Chinese text in .bat breaks cmd parsing under CP936.)
setlocal
cd /d "%~dp0..\..\.."
echo [exp2] ensuring single local server instance on 127.0.0.1:8760 ...
python backend\server.py --ensure --page exp2 --port 8760
endlocal
