@echo off
rem ===== motor_nested entry (server architecture): one-click local server =====
rem Page: examples/motor/exp_motor_nested/index.html  ->  http://127.0.0.1:8760/motor_nested/
rem Host = ONE local server program + the page stored in THIS example folder.
rem This bat guarantees a single server instance on 127.0.0.1:8760:
rem   * server not running  -> start it (foreground console = server; close to stop)
rem   * server already up   -> kill it and start a fresh one (no port/device contention)
rem   * a browser window is opened automatically by the server after boot
rem Data source mode stays EXPLICIT: the page starts nothing until you press
rem [Sim | Real] -> Start/Connect; default is Real (no auto-sim, no auto-QEMU).
rem (ASCII-only on purpose: Chinese text in .bat breaks cmd parsing under CP936;
rem  Chinese notes live in README.md.)
setlocal
cd /d "%~dp0\..\..\.."
echo [motor_nested] ensuring single local server instance on 127.0.0.1:8760 ...
python backend\server.py --ensure --page motor_nested --port 8760
endlocal
