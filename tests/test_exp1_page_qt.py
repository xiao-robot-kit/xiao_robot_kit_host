"""exp1 裸数据波形页 Qt-WebEngine 加载链路 + 模式开关测试（offscreen，无需人工开窗）。

链路（服务器架构）：backend/server.py 提供 /exp1/（页面住实例目录
examples/imu/exp1_waveform/）→ QWebEngineView 载 URL → 页面 LiveBridge 建 SSE；
页面自己**不**启源，loadFinished 后：
  1) 先探一次「未启动态」：status=未连接、帧计数 0、模式按钮 2 个
     （禁隐式回落：页面加载绝不自动开数据源）
  2) 再 kick startSim()（app.py --mode sim 同款）→ SimImuSource 六轴摆 →
     frameReady → 波形缓存 + 数值面板
  3) 断言：status=已连接、帧计数 ≥ 300（约 3 s@100Hz）、六轴读数非 —
  4) 语言开关：runJavaScript("UI.setLang('en')") 后按钮/状态文案变英文，
     标题 data-i18n 也被重建（中英双语可切且即时生效）
  5) 抖动缓冲/相位契约不在本文件断言（rAF 在 offscreen 不跑）——
     由 tests/exp1_wave_harness.js 直接驱动泵体（__xrkDebug.pump）回放抖动到达模式

跑法：python tests/test_exp1_page_qt.py（exit 0 = 全过；需 PySide6 + WebEngine）
"""

import json
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtCore import QTimer  # noqa: E402

from _server_page import qt_client, spawn_server, stop_server  # noqa: E402
LOAD_BUDGET_MS = 4000     # 页面加载预算（WebEngine 冷启动含子进程拉起）
RUN_MS = 4000             # 收流时长（ms；100Hz → ≥300 帧）
MIN_FRAMES = 300


def main() -> int:
    fails = []
    idle_state, run_state, en_state = {}, {}, {}

    proc, port = spawn_server("exp1")
    if port is None:
        stop_server(proc)
        print("== test_exp1_page_qt: 服务器未就绪 ==")
        return 1
    app, view = qt_client(f"http://127.0.0.1:{port}/exp1/")

    def probe_idle():
        """加载完成 + 1.5s 后的「未启动态」探测（禁隐式回落证据）。"""
        view.page().runJavaScript(
            "JSON.stringify({"
            " status: document.getElementById('status').textContent,"
            " stat: document.getElementById('stat').textContent,"
            " modes: document.querySelectorAll('#modebar .ui-seg').length,"
            " btn: document.getElementById('btn').textContent,"
            " title: document.querySelector('h1').textContent"
            "})",
            lambda s: idle_state.update(json.loads(s) if s else {}))

    def on_load(ok):
        idle_state["load"] = ok
        if ok:
            QTimer.singleShot(1500, probe_idle)
            QTimer.singleShot(2500, lambda: view.page().runJavaScript("startSim()"))

    view.loadFinished.connect(on_load)

    def probe_run():
        view.page().runJavaScript(
            "JSON.stringify({"
            " status: document.getElementById('status').textContent,"
            " stat: document.getElementById('stat').textContent,"
            " vals: document.getElementById('vals').textContent"
            "})",
            lambda s: run_state.update(json.loads(s) if s else {}))

    def probe_en():
        view.page().runJavaScript(
            "JSON.stringify({"
            " status: document.getElementById('status').textContent,"
            " btn: document.getElementById('btn').textContent,"
            " title: document.querySelector('h1').textContent,"
            " modeSim: document.querySelector('#modebar .ui-seg').textContent"
            "})",
            lambda s: en_state.update(json.loads(s) if s else {}))

    QTimer.singleShot(LOAD_BUDGET_MS + 2500 + RUN_MS, probe_run)
    QTimer.singleShot(LOAD_BUDGET_MS + 2500 + RUN_MS + 300,
                      lambda: view.page().runJavaScript("UI.setLang('en')"))
    QTimer.singleShot(LOAD_BUDGET_MS + 2500 + RUN_MS + 900, probe_en)
    QTimer.singleShot(LOAD_BUDGET_MS + 2500 + RUN_MS + 1400, app.quit)
    app.exec()
    stop_server(proc)

    def add(msg):
        fails.append(msg)

    if idle_state.get("load") is not True:
        add(f"页面加载失败（loadFinished={idle_state.get('load')}）")
    if idle_state.get("status") != "未连接":
        add(f"加载后应为未连接（禁隐式回落），实际 {idle_state.get('status')!r}")
    if "帧 0" not in (idle_state.get("stat") or "?"):
        add(f"加载后不应有帧（禁自动仿真），stat={idle_state.get('stat')!r}")
    if idle_state.get("modes") != 2:
        add(f"模式开关应有 2 档，实际 {idle_state.get('modes')!r}")
    if idle_state.get("btn") != "连接":
        add(f"真实模式（缺省）按钮应为 连接，实际 {idle_state.get('btn')!r}")

    if run_state.get("status") != "已连接":
        add(f"仿真启动后 status={run_state.get('status')!r}（应为 已连接）")
    stat = run_state.get("stat") or ""
    n = 0
    for tok in stat.replace("·", " ").split():
        if tok.isdigit():
            n = max(n, int(tok))
    if n < MIN_FRAMES:
        add(f"帧计数 {n} < {MIN_FRAMES}（stat={stat!r}）")
    if "—" in (run_state.get("vals") or "—"):
        add("数值面板未更新（仍有 —）")
    # 抖动缓冲/相位契约由 tests/exp1_wave_harness.js 断言（rAF 在 offscreen 不跑，
    # 页面内 rAF 泵在此环境不会推进——视觉平滑属人工项）

    if en_state.get("status") != "Connected":
        add(f"切英文后 status={en_state.get('status')!r}（应为 Connected）")
    if en_state.get("modeSim") != "Sim":
        add(f"切英文后模式按钮={en_state.get('modeSim')!r}（应为 Sim）")
    if "Raw IMU stream" not in (en_state.get("title") or ""):
        add(f"切英文后标题未重建：{en_state.get('title')!r}")

    print(f"== test_exp1_page_qt: idle={idle_state.get('status')!r}, "
          f"run={run_state.get('status')!r}, "
          f"en={en_state.get('status')!r}, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
