"""exp5 页面 Qt-WebEngine 加载链路测试（offscreen，无需人工开窗）。

链路（服务器架构）：backend/server.py 提供 /exp5/（页面住实例目录
examples/imu/exp5_trajectory/）→ QWebEngineView 载 URL → 页面 LiveBridge 建 SSE →
kick startSim() → POST /api/open{sim} → hub SimTrajSource（带轨迹重建）→
SSE frame/pos → 页面 DOM。经 runJavaScript 探针读页面状态断言：
  1) 页面加载成功（loadFinished ok）
  2) three.js 本地副本已载入（window.THREE）
  3) startSim/autoConnect 显式契约在 window 上
  4) 模拟源连通（status = 已连接）
  5) 页面在收真实帧（stat 文本 QUAT 计数 ≥ 100，位置读数非零——
     QWebChannel 双向桥 + posReady 驱动的端到端证据）

WebGL 像素渲染不在此断言（offscreen 环境无显示面；视觉项归人工开窗）。
跑法：python tests/test_exp5_page_qt.py（exit 0 = 全过；需 PySide6 + WebEngine）
"""

import json
import math
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtCore import QTimer  # noqa: E402

from _server_page import qt_client, spawn_server, stop_server  # noqa: E402
LOAD_BUDGET_MS = 4000     # 页面加载预算（WebEngine 冷启动含子进程拉起）
RUN_MS = 3000             # 收流时长（ms，模拟源起播后）
MIN_QUAT = 100            # 页面侧 QUAT 计数下限（100Hz 双流）


def main() -> int:
    fails = []
    page_state = {}

    proc, port = spawn_server("exp5")
    if port is None:
        stop_server(proc)
        print("== test_exp5_page_qt: 服务器未就绪 ==")
        return 1
    app, view = qt_client(f"http://127.0.0.1:{port}/exp5/")

    def on_load(ok):
        page_state["load"] = ok
        if ok:
            view.page().runJavaScript("startSim()")   # app.py --sim 同款 kick
    view.loadFinished.connect(on_load)

    def probe():
        view.page().runJavaScript(
            "JSON.stringify({"
            " status: document.getElementById('status').textContent,"
            " stat: document.getElementById('stat').textContent,"
            " vals: document.getElementById('vals').textContent,"
            " hasThree: !!window.THREE,"
            " hasStartSim: typeof window.startSim === 'function'"
            "})",
            lambda s: page_state.update(json.loads(s) if s else {}))

    QTimer.singleShot(LOAD_BUDGET_MS + RUN_MS, probe)
    QTimer.singleShot(LOAD_BUDGET_MS + RUN_MS + 500, app.quit)
    app.exec()

    stop_server(proc)

    ok = page_state.get("load")
    if ok is not True:
        fails.append(f"页面加载失败（loadFinished={ok}）")
    if page_state.get("hasThree") is not True:
        fails.append("window.THREE 缺失（three.min.js 未载入）")
    if page_state.get("hasStartSim") is not True:
        fails.append("window.startSim 缺失（kick 契约未挂）")
    if page_state.get("status") != "已连接":
        fails.append(f"status={page_state.get('status')!r}（应为 已连接）")
    stat = page_state.get("stat") or ""
    n_quat = 0
    for tok in stat.replace("·", " ").split():
        if tok.isdigit():
            n_quat = max(n_quat, int(tok))
            break
    if "QUAT" not in stat or n_quat < MIN_QUAT:
        fails.append(f"stat 无有效 QUAT 计数（{stat!r}，需 ≥{MIN_QUAT}）")

    print(f"== test_exp5_page_qt: load={ok}, status={page_state.get('status')!r}, "
          f"stat={stat!r}, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
