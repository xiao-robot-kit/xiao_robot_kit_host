"""_server_page.py — 服务器版页面测试的公共骨架（被 test_expN_page_*.py 复用）。

服务器架构（用户定案 2026-11-05）：页面由 backend/server.py 在
http://127.0.0.1:<port>/<page>/ 提供，数据面走 SSE；测试就按真实部署形态跑：
    起服务器（随机端口，--no-open）→ offscreen QWebEngineView 载 URL → 探针断言。
"""

import os
import socket
import subprocess
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SERVER = os.path.join(ROOT, "backend", "server.py")


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


def spawn_server(page: str):
    """起一个 --no-open 服务器（随机端口）；返回 (proc, port)；就绪失败返回 (proc, None)。"""
    port = free_port()
    proc = subprocess.Popen([sys.executable, SERVER, "--no-open", "--port", str(port),
                             "--page", page],
                            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if not wait_ready(port):
        return proc, None
    return proc, port


def stop_server(proc) -> None:
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def qt_client(url: str):
    """offscreen QWebEngineView 载 URL；返回 (app, view)（调用方自己 app.exec()）。"""
    from PySide6.QtCore import QUrl
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWebEngineWidgets import QWebEngineView

    app = QApplication.instance() or QApplication([])
    view = QWebEngineView()
    view.load(QUrl(url))
    return app, view
