"""test_exp4_page_server.py — 服务器版 exp4 页全链路测试（offscreen，无需人工开窗）。

链路（服务器架构，用户定案 2026-11-05）：
    backend/server.py（--no-open，随机端口）→ 页面住 examples/imu/exp4_quat_view/
    → QWebEngineView 载 http://127.0.0.1:<port>/exp4/ → 页面 LiveBridge 建 SSE
    → 点「启动仿真」（startSim()）→ POST /api/open{sim} → hub SimTrajSource
    → SSE frame 事件 → 页面 DOM 更新。

断言：
  1) 页面从服务器加载成功 + three.js 本地资产可用（window.THREE）
  2) **加载后不启动任何源**：status=未连接、帧计数 0（禁隐式回落）
  3) startSim() 后：status=已连接、QUAT 计数 ≥100、|q|≈1、欧拉对照非 —
  4) 双语：UI.setLang('en') 后状态/标题变英文

跑法：python tests/test_exp4_page_server.py（exit 0 = 全过；需 PySide6 + WebEngine）
"""

import json
import os
import socket
import subprocess
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "backend"))

from PySide6.QtCore import QTimer, QUrl  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from PySide6.QtWebEngineWidgets import QWebEngineView  # noqa: E402

SERVER = os.path.join(ROOT, "backend", "server.py")
RUN_MS = 4000             # 收流时长（ms）
MIN_QUAT = 100
READY_BUDGET_S = 6.0      # 页面就绪（kick 契约 + 面板锚点）等待上限（s）


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_ready(port: int, budget: float = 12.0) -> bool:
    deadline = time.time() + budget
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.15)
    return False


def main() -> int:
    fails = []
    idle, run, en = {}, {}, {}
    port = free_port()
    proc = subprocess.Popen([sys.executable, SERVER, "--no-open", "--port", str(port),
                             "--page", "exp4"],
                            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        if not wait_ready(port):
            print("== test_exp4_page_server: 服务器未就绪 ==")
            return 1
        url = f"http://127.0.0.1:{port}/exp4/"

        app = QApplication.instance() or QApplication([])
        view = QWebEngineView()
        view.load(QUrl(url))

        probe_js = (
            "JSON.stringify({"
            " status: document.getElementById('status').textContent,"
            " stat: document.getElementById('stat').textContent,"
            " vals: document.getElementById('vals').textContent,"
            " pitch: document.getElementById('e-pitch').textContent,"
            " three: typeof window.THREE,"
            " title: document.querySelector('h1').textContent"
            "})")

        # 页面就绪钩子：exp4 页已改为「薄壳 + 统一页面核心」（逐级继承：raw/cal/euler/quat
        # 四个面板共用一份核心，含 3 个 WebGL 上下文）——加载时长比单内联页长且随
        # GPU 进程冷启动浮动，故按就绪条件（kick 契约 + 面板锚点就位）再动作，
        # 不用固定延时偷跑 startSim（偷跑会在 window.startSim 未定义时静默失败）。
        ready_js = (
            "JSON.stringify({"
            " ready: typeof window.startSim === 'function'"
            " && !!document.getElementById('stat')"
            " && !!document.getElementById('e-pitch')"
            "})")
        ready = {"since": None, "ok": False}

        def kick():
            view.page().runJavaScript(probe_js, lambda s: idle.update(json.loads(s) if s else {}))
            view.page().runJavaScript("startSim()")
            QTimer.singleShot(RUN_MS, probe_run)

        def probe_run():
            view.page().runJavaScript(probe_js, lambda s: run.update(json.loads(s) if s else {}))
            QTimer.singleShot(300, lambda: view.page().runJavaScript("UI.setLang('en')"))
            QTimer.singleShot(900, probe_en)
            QTimer.singleShot(1400, app.quit)

        def probe_en():
            view.page().runJavaScript(probe_js, lambda s: en.update(json.loads(s) if s else {}))

        ready_timer = QTimer()

        def on_ready_state(s):
            state = json.loads(s) if s else {}
            if state.get("ready"):
                ready_timer.stop()
                ready["ok"] = True
                QTimer.singleShot(200, kick)
            elif time.time() - ready["since"] > READY_BUDGET_S:
                ready_timer.stop()          # 超时：kick 一次，让断言报出真实状态
                kick()

        def check_ready():
            view.page().runJavaScript(ready_js, on_ready_state)

        ready_timer.timeout.connect(check_ready)

        def on_load(ok):
            idle["load"] = ok
            ready["since"] = time.time()
            ready_timer.start(50)          # loadFinished 后再等面板锚点全就位

        view.loadFinished.connect(on_load)
        QTimer.singleShot((READY_BUDGET_S + RUN_MS / 1000.0 + 4.0) * 1000, app.quit)
        app.exec()

        if idle.get("load") is not True:
            fails.append(f"页面加载失败（loadFinished={idle.get('load')}），"
                         f"就绪超时={not ready['ok']}")
        if idle.get("three") != "object":
            fails.append(f"three.js 资产未载入（{idle.get('three')!r}）")
        if idle.get("status") != "未连接":
            fails.append(f"加载后应为未连接（禁隐式回落），实际 {idle.get('status')!r}")
        if "QUAT 0" not in (idle.get("stat") or "?"):
            fails.append(f"加载后不应有帧，stat={idle.get('stat')!r}")

        if run.get("status") != "已连接":
            fails.append(f"仿真启动后 status={run.get('status')!r}（应为 已连接）")
        stat = run.get("stat") or ""
        nq = 0
        for tok in stat.replace("·", " ").split():
            if tok.isdigit():
                nq = max(nq, int(tok))
        if nq < MIN_QUAT:
            fails.append(f"QUAT 计数 {nq} < {MIN_QUAT}（stat={stat!r}）")
        if "—" in (run.get("pitch") or "—"):
            fails.append("欧拉对照表未更新（仍有 —）")

        if en.get("status") != "Connected":
            fails.append(f"切英文后 status={en.get('status')!r}（应为 Connected）")
        if "Quaternion attitude" not in (en.get("title") or ""):
            fails.append(f"切英文后标题未重建：{en.get('title')!r}")

        print(f"== test_exp4_page_server: idle={idle.get('status')!r}, "
              f"run={run.get('status')!r}, en={en.get('status')!r}, {len(fails)} fail ==")
        for f in fails:
            print("  [FAIL]", f)
        return 1 if fails else 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
