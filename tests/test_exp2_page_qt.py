"""exp2 调零温补页 Qt-WebEngine 加载链路测试（offscreen，无需人工开窗）。

链路（服务器架构）：backend/server.py 提供 /exp2/（页面住实例目录
examples/imu/exp2_bias/）→ QWebEngineView 载 http://127.0.0.1:<port>/exp2/ →
页面 LiveBridge 建 SSE → kick startSim() → POST /api/open{sim} → hub SimCalSource
（镜像固件开机时序：TEXT 横幅 → 5 s 标定窗 → 0x02 流）→ SSE frame → 页面 DOM。
经 runJavaScript 探针读页面状态断言：
  1) 页面加载成功（loadFinished ok）
  2) 模拟源连通（status = 已连接）
  3) 开机标定时序可见（固件日志含 OK sim / OK calib 500 frames / OK stream）
  4) 页面在收 0x02 帧（stat 的 IMU_CAL 计数 ≥ 100——QWebChannel 双向桥 +
     bridge 0x02 透传分支的端到端证据）
  5) 数值面板活：温度读数非 —、温补后 deb 列非 —、零偏统计窗非 0

WebGL/像素渲染不在此页断言（纯 DOM 页面）；布局视觉归人工开窗。
跑法：python tests/test_exp2_page_qt.py（exit 0 = 全过；需 PySide6 + WebEngine）
"""

import json
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtCore import QTimer  # noqa: E402

from _server_page import qt_client, spawn_server, stop_server  # noqa: E402
LOAD_BUDGET_MS = 4000     # 页面加载预算（WebEngine 冷启动含子进程拉起）
RUN_MS = 9000             # 收流时长（ms）：5 s 标定窗 + ≥4 s 0x02 流
MIN_CAL = 100             # 页面侧 IMU_CAL 计数下限（100Hz，4s 流 ≥ 400 留余量）


def main() -> int:
    fails = []
    page_state = {}

    proc, port = spawn_server("exp2")
    if port is None:
        stop_server(proc)
        print("== test_exp2_page_qt: 服务器未就绪 ==")
        return 1
    app, view = qt_client(f"http://127.0.0.1:{port}/exp2/")

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
            " temp: document.getElementById('temp').textContent,"
            " dgx: document.getElementById('d-gx').textContent,"
            " bgx: document.getElementById('b-gx').textContent,"
            " title: document.getElementById('stat-title').textContent,"
            " log: document.getElementById('log').textContent"
            "})",
            lambda s: page_state.update(json.loads(s) if s else {}))

    QTimer.singleShot(LOAD_BUDGET_MS + RUN_MS, probe)
    QTimer.singleShot(LOAD_BUDGET_MS + RUN_MS + 500, app.quit)
    app.exec()

    stop_server(proc)

    if page_state.get("load") is not True:
        fails.append(f"页面加载失败（loadFinished={page_state.get('load')}）")
    if page_state.get("status") != "已连接":
        fails.append(f"status={page_state.get('status')!r}（应为 已连接）")
    log = page_state.get("log") or ""
    for want in ("OK sim:", "OK calib 500 frames:", "OK stream:"):
        if want not in log:
            fails.append(f"固件日志缺 {want!r}（开机标定时序断裂）")
    stat = page_state.get("stat") or ""
    n_cal = 0
    for tok in stat.replace("·", " ").split():
        if tok.isdigit():
            n_cal = max(n_cal, int(tok))
            break
    if "IMU_CAL" not in stat or n_cal < MIN_CAL:
        fails.append(f"stat 无有效 IMU_CAL 计数（{stat!r}，需 ≥{MIN_CAL}）")
    if page_state.get("temp") in (None, "", "—") or "°C" not in (page_state.get("temp") or ""):
        fails.append(f"温度读数未更新（temp={page_state.get('temp')!r}）")
    if page_state.get("dgx") in (None, "", "—"):
        fails.append(f"deb 列未更新（d-gx={page_state.get('dgx')!r}）")
    if page_state.get("bgx") in (None, "", "—"):
        fails.append(f"bias 列未更新（b-gx={page_state.get('bgx')!r}）")
    if "窗 0/300" in (page_state.get("title") or ""):
        fails.append(f"统计窗未累计（title={page_state.get('title')!r}）")

    print(f"== test_exp2_page_qt: load={page_state.get('load')}, "
          f"status={page_state.get('status')!r}, stat={stat!r}, "
          f"temp={(page_state.get('temp') or '')[:24]!r}, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
