"""app.py — 上位机**桌面客户端**（可选；产品本体是 backend/server.py 本地服务器）。

架构（用户定案 2026-11-05）：
    上位机 = 本地服务器程序（backend/server.py：静态页 + 数据源中枢 hub.py + SSE）
           + 各实验实例目录里的交互页面（examples/imu/<实验>/index.html）
           + 一键入口 run.bat（没起就起；已起就杀掉重启——单实例，避免抢占）。
本文件只做「可选的桌面窗客户端」：拉起/重启单实例服务器（子进程，与本窗口同
控制台）→ webview 载 http://127.0.0.1:<port>/<page>/ → 按命令行显式传
模式/语言/串口。服务器**随本窗口存活**（关窗/关控制台即停，与 run.bat 同语义）；
要「服务器常驻 + 浏览器访问」请直接跑实例目录里的 run.bat。页面本身不依赖 Qt。

页面与端口：
    exp1 裸数据波形 / exp2 调零温补 / exp4 四元数姿态 / exp5 位移轨迹
    store_demo pstore 断电续跑演示（MOD_STORE 命令：GET / SET / STATUS）
        → 同一服务器（SERVER_PORT=8760）的 /exp1/ … /exp5/、/store_demo/
    exp3 欧拉角 → 独立页自带的 euler_view.py 页服务（JSON 轮询，历史原因保留）

模式（只许显式，禁隐式回落）：
  --mode real（缺省）  真实设备：什么都不自动起；给了位置参数串口名则加载完成后
                      投递给页面（等价于用户点「连接」——仍是一次显式指定）。
  --mode sim           仿真：本机合成源（不碰设备、不碰 QEMU），加载后 kick startSim()。
  --sim 为 --mode sim 的旧别名。无 --mode 无串口 = 真实模式空跑。
语言：--lang zh|en 指定初始语言（页内「EN/中文」按钮随时切，选择存 localStorage）。

跑法：
    python app.py                       # exp1（真实模式，页面内选串口连接）
    python app.py --page exp4 --mode sim
    python app.py --page exp5 COM57
    python app.py --page exp3           # 独立页（自带页服务）
依赖：PySide6（桌面客户端可选依赖）、pyserial——见 README 依赖声明段。
"""

import os
import socket
import subprocess
import sys
import time

MODES = ("sim", "real")
LANGS = ("zh", "en")
DEFAULT_MODE = "real"     # 缺省真实模式：不点不启动（禁「没插设备就自动仿真」）

_ROOT = os.path.dirname(os.path.abspath(__file__))
_EXAMPLES = os.path.join(_ROOT, "examples")

# 服务器版页面（页面住实例目录，数据面走 SSE）
SERVER_PAGES = ("exp1", "exp2", "exp4", "exp5", "store_demo")
SERVER_PORT = 8760
SERVER_SCRIPT = os.path.join(_ROOT, "backend", "server.py")
SERVER_READY_TIMEOUT_S = 12.0

# exp3 独立页（自带 http.server，历史原因保留独立进程）
EXP3_PAGE = "exp3"
EXP3_DIR = os.path.join(_EXAMPLES, "imu", "exp3_euler_view")
EXP3_PORT = 8765
EXP3_READY_TIMEOUT_S = 10.0

DEFAULT_PAGE = "exp1"
KNOWN_PAGES = {p: None for p in SERVER_PAGES}
KNOWN_PAGES[EXP3_PAGE] = None


def _pick_arg(argv, flag: str, default: str) -> str:
    """取 `--flag value` / `--flag=value` 的值（缺省 default）。"""
    args = argv[1:]
    for i, a in enumerate(args):
        if a == flag and i + 1 < len(args):
            return args[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def _pick_page(argv) -> str:
    """从命令行取 --page 名（缺省 exp1）；未知名 SystemExit(2)（Qt 起前拦截）。"""
    page = _pick_arg(argv, "--page", DEFAULT_PAGE)
    if page not in KNOWN_PAGES:
        known = " / ".join(sorted(KNOWN_PAGES))
        raise SystemExit(f"[app] 未知页面 {page!r}（已知：{known}）")
    return page


def _pick_mode(argv) -> str:
    """--mode sim|real（缺省 real）；--sim 为旧别名（仅在未显式给 --mode 时生效）。"""
    explicit = any(a == "--mode" or a.startswith("--mode=") for a in argv[1:])
    mode = _pick_arg(argv, "--mode", DEFAULT_MODE)
    if "--sim" in argv[1:] and not explicit:
        mode = "sim"
    if mode not in MODES:
        raise SystemExit(f"[app] 未知模式 {mode!r}（已知：{' / '.join(MODES)}）")
    return mode


def _pick_lang(argv) -> str:
    """--lang zh|en（缺省 zh：页面 localStorage 有历史选择时以页面为准）。"""
    lang = _pick_arg(argv, "--lang", "zh")
    if lang not in LANGS:
        raise SystemExit(f"[app] 未知语言 {lang!r}（已知：{' / '.join(LANGS)}）")
    return lang


def _serial_arg(argv):
    """取位置参数串口名（--page/--mode/--lang 的值不是；无则 None）。"""
    skip_next = False
    for a in argv[1:]:
        if skip_next:
            skip_next = False
            continue
        if a in ("--page", "--mode", "--lang"):
            skip_next = True
            continue
        if a.startswith("--"):
            continue
        return a
    return None


def _wait_port(port: int, timeout_s: float) -> bool:
    """轮询 127.0.0.1:port 直到可连（页服务就绪探针）。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def _kick_page(view, mode: str, lang: str, serial_arg) -> None:
    """加载完成后的显式 kick：语言先落，再按模式/串口投递（缺省不投递）。"""

    def _kick(ok):
        if not ok:
            return
        if lang != "zh":
            view.page().runJavaScript(f"UI && UI.setLang({lang!r})")
        if mode == "sim":
            view.page().runJavaScript("startSim()")
        elif serial_arg:
            view.page().runJavaScript(f"autoConnect({serial_arg!r})")

    view.loadFinished.connect(_kick)


def _webview_client(url: str, title: str, mode: str, lang: str, serial_arg) -> int:
    """起一个 webview 客户端窗口（PySide6 延迟导入：无 Qt 环境不影响服务器本体）。"""
    from PySide6.QtCore import QUrl
    from PySide6.QtWidgets import QApplication, QMainWindow
    from PySide6.QtWebEngineWidgets import QWebEngineView

    app = QApplication.instance() or QApplication(sys.argv)
    win = QMainWindow()
    win.setWindowTitle(title)
    win.resize(1100, 720)
    view = QWebEngineView()
    view.load(QUrl(url))
    win.setCentralWidget(view)
    win.show()
    _kick_page(view, mode, lang, serial_arg)
    return app.exec()


def _run_server_page(page: str, mode: str, lang: str, serial_arg) -> int:
    """拉起/重启单实例服务器（--ensure）→ webview 载 /<page>/。

    服务器是本进程的子进程（同一控制台）：窗口关闭 / 控制台结束即一起停——
    与 run.bat 的「控制台就是服务器」语义一致，不留孤儿进程占端口。
    """
    proc = subprocess.Popen([sys.executable, SERVER_SCRIPT, "--ensure",
                             "--page", page, "--no-open",
                             "--port", str(SERVER_PORT)])
    if not _wait_port(SERVER_PORT, SERVER_READY_TIMEOUT_S):
        print(f"[app] 服务器 {SERVER_PORT} 就绪超时（pid={proc.pid} 仍在跑，查看其控制台）",
              flush=True)
        return 1
    url = f"http://127.0.0.1:{SERVER_PORT}/{page}/"
    print(f"[app] 服务器页 {url}（服务器随本窗口存活；关窗即停）", flush=True)
    return _webview_client(url, f"XIAO Robot Kit — {page}（服务器页 · "
                                + ("仿真" if mode == "sim" else "真实") + "模式）",
                           mode, lang, serial_arg)


def _run_exp3_page(mode: str, lang: str) -> int:
    """exp3 路由：子进程拉起 euler_view.py（自带页服务）→ webview 载其 URL。"""
    script = os.path.join(EXP3_DIR, "euler_view.py")
    proc = subprocess.Popen([sys.executable, script, "--mode", mode,
                             "--lang", lang, "--http-port", str(EXP3_PORT)])
    done = [False]

    def _cleanup():
        if done[0]:
            return
        done[0] = True
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()

    if not _wait_port(EXP3_PORT, EXP3_READY_TIMEOUT_S):
        _cleanup()
        raise SystemExit(
            f"[app] exp3 页服务 127.0.0.1:{EXP3_PORT} 就绪超时"
            f"（{EXP3_READY_TIMEOUT_S}s，子进程已回收）")

    from PySide6.QtWidgets import QApplication  # noqa: F401  （使用方自检依赖）

    rc = _webview_client(f"http://127.0.0.1:{EXP3_PORT}/",
                         "XIAO Robot Kit — exp3（独立页 euler_view.py · "
                         + ("仿真" if mode == "sim" else "真实") + "模式）",
                         mode, lang, None)
    _cleanup()
    return rc


def main() -> int:
    page = _pick_page(sys.argv)
    mode = _pick_mode(sys.argv)
    lang = _pick_lang(sys.argv)
    serial_arg = _serial_arg(sys.argv)

    if page == EXP3_PAGE:
        if serial_arg:
            raise SystemExit(
                f"[app] exp3 为独立 http 页（仿真/真实模式），不接受串口参数 "
                f"{serial_arg!r}——真实模式请用 "
                "examples/imu/exp3_euler_view/run.bat --mode real")
        return _run_exp3_page(mode, lang)

    return _run_server_page(page, mode, lang, serial_arg)


if __name__ == "__main__":
    sys.exit(main())
