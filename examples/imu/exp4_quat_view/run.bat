@echo off
rem ===== exp4_quat_view entry (server architecture): one-click local server =====
rem Host = ONE local server program + the page stored in THIS example folder.
rem This bat guarantees a single server instance:
rem   * server not running  -> start it (foreground console = server; close to stop)
rem   * server already up   -> kill it and start a fresh one (no port/device contention)
rem The page is served at http://127.0.0.1:8760/exp4/ and opened in the browser.
rem Data source mode stays EXPLICIT: the page starts nothing until you press
rem [Sim | Real] -> Start/Connect; default is Real (no auto-sim, no auto-QEMU).
rem (ASCII-only on purpose: Chinese text in .bat breaks cmd parsing under CP936.)
setlocal
cd /d "%~dp0..\..\.."
echo [exp4] ensuring single local server instance on 127.0.0.1:8760 ...
python backend\server.py --ensure --page exp4 --port 8760
endlocal
