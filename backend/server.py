"""server.py — 上位机本地服务器（stdlib only：http.server + SSE）。

架构（用户定案 2026-11-05）：
    上位机 = **一个本地服务器程序** + **各实验实例目录里的交互页面**。
    - 页面住在自己的实验目录（examples/imu/exp4_quat_view/index.html …）；
    - 服务器负责：静态页/共享资产、数据源中枢（hub.py）、帧流推送（SSE）、
      下行命令（POST）、串口枚举；
    - run.bat 一键：**没起就起，已起就杀掉重启**（单实例，避免端口/设备被抢占）。

接口（页面只认这套，不依赖 Qt）：
    GET  /                          → 重定向到 --page（缺省 exp4）
    GET  /<page>/ , /<page>/<file>  → 该实验目录里的页面与本地资源（IMU 与电机两套注册表）
    GET  /assets/<file>             → 共享资产（ui-shell.js / live.js / vendor/…）
    GET  /api/state                 → IMU：{"mode","source","connected","target","counters",...}
                                      电机：{"kind","scenario","source","target",
                                             "connected","frames","stats","counters"}
    GET  /api/ports                 → [{"device","desc"}, ...]（pyserial 缺失则为空）
    GET  /api/stream                → SSE：IMU event=frame|text|stat|pos|port|state
                                      电机 event=mframe|text|verdict|port|state
    POST /api/open   IMU  {"mode":"sim"} / {"mode":"real","target":"tcp:127.0.0.1:5558"}
                          / {"mode":"real","port":"COM5","baud":115200}（旧形，归一成
                          serial:COM5@115200）
                     电机 {"mode":"sim"} / {"mode":"real","target":"tcp:127.0.0.1:5560"}
    POST /api/close
    POST /api/imu/cmd               → IMU 命令通道（CMD 0x7D → ACK 0x7E）：
                                      {"func":"calibrate|get_bias|get_status|temp_calib|"
                                               "set_flags|get_drift|get_tc_state",
                                       "args":{...}}
                                      → {"func","status","payload":[...]}；失败（未连接/
                                      等回执超时）→ 409 {"error":"…"}（报文头里明报原因）；
                                      MCU 自己回的非 0 status 原样透传（200）
    POST /api/calib                 → 旧入口兼容（内部即 func=calibrate）：sim 回
                                      {"code":"calibSimulated","payload":bias6,…}，real 回
                                      {"code":"calibSent",…}（等 ACK 后才返回）
    POST /api/verify                → 电机页断言（{"ok","pass","items","summary"}）
    POST /api/command {"line":"SCEN 1\n"} → 电机页下行（仅 real 档透传）
    POST /api/target? 不支持（目标只走 open 参数，避免隐藏状态）

页面注册表（两张）：PAGES = IMU 页（FrameHub）；MOTOR_PAGES = 电机页（MotorHub，
见 backend/motor_hub.py）。--page 决定本进程挂哪一个中枢，端点按中枢类型分派。

单实例纪律（run.bat 调用）：
    python backend/server.py --ensure --page exp4
      · .runtime/server.json 记录 pid/port；进程活着 → terminate（Windows TerminateProcess）
        并等端口释放（≤3s）；然后本进程前台起服务（控制台即服务器，关窗即停）。
      · 端口被**别的**进程占用且无法识别 → 明确报错 exit 3，不静默改端口。

跑法：
    python backend/server.py --page exp4             # 缺省端口 8760，自动开浏览器
    python backend/server.py --ensure --page exp4    # 一键（run.bat 用这个）
    python backend/server.py --no-open --port 8770   # 自测/CI 用
"""

import argparse
import json
import os
import queue
import signal
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import unquote, urlparse

_ROOT = Path(__file__).resolve().parent.parent          # Host_Project/
sys.path.insert(0, str(_ROOT / "backend"))

from hub import FrameHub, FUNC_IMU                         # noqa: E402
from motor_hub import MotorHub                            # noqa: E402
from store_hub import StoreHub                            # noqa: E402
from trajectory import TRAJ_PRESETS                       # noqa: E402

DEFAULT_PORT = 8760
RUNTIME_DIR = _ROOT / ".runtime"
RUNTIME_FILE = RUNTIME_DIR / "server.json"
ASSETS_DIR = _ROOT / "web" / "assets"
EXAMPLES = _ROOT / "examples" / "imu"
EXAMPLES_MOTOR = _ROOT / "examples" / "motor"

# 页面注册表：page → (目录, 仿真源选型, 是否挂轨迹重建)
PAGES = {
    "exp1": (EXAMPLES / "exp1_waveform", "imu", False),
    "exp2": (EXAMPLES / "exp2_bias", "cal", False),
    "exp3": (EXAMPLES / "exp3_euler_view", "imu", False),   # exp3 自带页面，数据面待迁移
    "exp4": (EXAMPLES / "exp4_quat_view", "traj", False),
    "exp5": (EXAMPLES / "exp5_trajectory", "traj", True),
}
DEFAULT_PAGE = "exp4"

# 电机页注册表：page → (实例目录名（examples/motor/ 下）, 数据面类型, 场景名, scen_id)# 数据面类型决定 sim 源与断言口径：motor = 0x05 demo / 0x06 场景；pidscan = 0x05 单槽 + TEXT 表。
# (实例目录名, 数据面类型, 场景名, scen_id, fixed)
#   fixed=True：该页对应**单实例工程固件**（exp_motor_pwm 等五类）——开机即按自己的
#   场景出 0x05，不支持「SCEN <n>」切换，真实档不做握手（否则白等 15s 误报失败）；
#   仿真档不受影响（合成源同号幂等下发 SCEN）。motor_scope/pidscan 同理 fixed=True
#   （前者是 demo 0x05，后者固件自己轮转）。
MOTOR_PAGES = {
    "motor_pwm":      ("exp_motor_pwm",      "motor",    "pwm",      1, True),
    "motor_servo":    ("exp_motor_servo",    "motor",    "servo",    2, True),
    "motor_enc":      ("exp_motor_enc",      "motor",    "enc",      3, True),
    "motor_nested":   ("exp_motor_nested",   "motor",    "nested",   4, True),
    "motor_dc":       ("exp_motor_dc",       "motor",    "dc",       5, True),
    "motor_scope":    ("motor_scope",        "motor",    None,   None, True),  # demo 0x05 4 槽
    "motor_pidscan":  ("exp_motor_pidscan",  "pidscan",  None,   None, True),
}

# store_demo 页注册表（独立模块中枢 MOD_STORE=0x20；目录在 examples/ 下直接子级）：
#   页 → (目录名, data 面类型). store 页不发数据帧流，只有 TEXT + ACK（命令回执），
#   中枢 = backend/store_hub.py 的 StoreHub（与 IMU FrameHub / 电机 MotorHub 并列）。
STORE_PAGES = {
    "store_demo": "store_demo",
}
EXAMPLES_STORE = _ROOT / "examples"


# ────────────────────────── 单实例（ensure / 释放） ──────────────────────────


def _read_runtime() -> dict:
    try:
        return json.loads(RUNTIME_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    if os.name == "nt":
        # Windows：OpenProcess + GetExitCodeProcess（STILL_ACTIVE=259）。
        # 不用 tasklist 文本解析——中文系统输出是 GBK，utf-8 解码会直接抛异常。
        import ctypes
        handle = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)  # QUERY_LIMITED
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            ctypes.windll.kernel32.GetExitCodeProcess(handle, ctypes.byref(code))
            return code.value == 259
        finally:
            ctypes.windll.kernel32.CloseHandle(handle)
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _port_free(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
            return True
        except OSError:
            return False


def ensure_restart(port: int, host: str) -> None:
    """已唤醒 → 杀掉重启（单实例，避免端口/设备抢占）；没唤醒 → 直接起。"""
    info = _read_runtime()
    pid = int(info.get("pid", 0) or 0)
    old_port = int(info.get("port", 0) or 0)
    if pid and _pid_alive(pid):
        print(f"[server] 发现已在运行的服务器 pid={pid} port={old_port} → 杀掉重启（单实例）",
              flush=True)
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass
        # 等端口释放（旧进程可能是我方，也可能是同端口的遗留）
        deadline = time.time() + 3.0
        target = old_port or port
        while time.time() < deadline:
            if _port_free(target, host) and not _pid_alive(pid):
                break
            time.sleep(0.1)
        else:
            print(f"[server] 警告：旧实例 pid={pid} 未在 3s 内退出", flush=True)
    elif pid:
        print(f"[server] 运行记录里的 pid={pid} 已不存在（上次异常退出）→ 清理后启动",
              flush=True)

    if not _port_free(port, host):
        raise SystemExit(
            f"[server] 端口 {host}:{port} 被其它进程占用且无法识别（不是本服务器）——"
            "请先释放该端口，或换 --port。为避免抢资源，本服务器不会静默换端口。")

    RUNTIME_DIR.mkdir(exist_ok=True)
    RUNTIME_FILE.write_text(json.dumps(
        {"pid": os.getpid(), "port": port, "started": time.time()},
        ensure_ascii=False), encoding="utf-8")


# ────────────────────────── HTTP + SSE ──────────────────────────


class HubBroadcaster:
    """hub 回调 → 每个 SSE 客户端一条队列（慢客户端丢帧而不是阻塞采集线程）。"""

    MAX_QUEUE = 2000

    def __init__(self, hub: FrameHub) -> None:
        self._hub = hub
        self._clients = set()
        self._lock = threading.Lock()
        hub.add_listener(self._on_event)

    def register(self) -> "queue.Queue":
        q: queue.Queue = queue.Queue(maxsize=self.MAX_QUEUE)
        with self._lock:
            self._clients.add(q)
        return q

    def unregister(self, q) -> None:
        with self._lock:
            self._clients.discard(q)

    def _on_event(self, kind: str, payload) -> None:
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait((kind, payload))
            except queue.Full:
                pass

    def broadcast_state(self) -> None:
        self._on_event("state", self._hub.state())


class Handler(BaseHTTPRequestHandler):
    server_version = "XRKHost/1.0"
    hub = None                      # main() 注入：FrameHub（IMU 页）或 MotorHub（电机页）
    bus: HubBroadcaster = None      # 两者都提供 add_listener/state()，广播器与中枢无关
    default_page: str = DEFAULT_PAGE

    # ── 工具 ──
    def _send_json(self, obj, code: int = 200) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, content_type: str) -> None:
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception:
            return {}

    # ── 路由 ──
    def do_GET(self) -> None:                       # noqa: N802
        path = unquote(urlparse(self.path).path)
        if path == "/":
            self.send_response(302)
            self.send_header("Location", f"/{self.default_page}/")
            self.end_headers()
            return
        if path == "/api/state":
            self._send_json(self.hub.state())
            return
        if path == "/api/ports":
            self._send_json({"ports": list_serial_ports()})
            return
        if path == "/api/stream":
            self._stream()
            return
        if path.startswith("/assets/"):
            rel = path[len("/assets/"):]
            target = (ASSETS_DIR / rel).resolve()
            if not str(target).startswith(str(ASSETS_DIR.resolve())) or not target.is_file():
                self.send_error(404)
                return
            self._send_file(target, guess_type(target))
            return
        # /<page>/... → 该实验目录
        parts = [p for p in path.split("/") if p]
        if not parts:
            self.send_error(404)
            return
        page = parts[0]
        page_dir = page_directory(page)
        if page_dir is None:
            self.send_error(404, f"unknown page {page!r}")
            return
        base = page_dir.resolve()
        rel = "/".join(parts[1:]) or "index.html"
        if rel.endswith("/"):
            rel += "index.html"
        target = (base / rel).resolve()
        if not str(target).startswith(str(base)) or not target.is_file():
            self.send_error(404)
            return
        self._send_file(target, guess_type(target))

    def do_POST(self) -> None:                      # noqa: N802
        path = unquote(urlparse(self.path).path)
        body = self._read_json()
        try:
            if path == "/api/open":
                if isinstance(self.hub, MotorHub):
                    mode = body.get("mode")
                    if mode == "sim":
                        state = self.hub.open_sim()
                    elif mode == "real":
                        target = str(body.get("target") or "").strip()
                        if not target:
                            self._send_json({"error": "real 模式需要 target"
                                             "（tcp:host:port 或 serial:COMx@baud）"}, 400)
                            return
                        state = self.hub.open_real(target)
                    else:
                        self._send_json({"error": f"unknown mode {mode!r}"}, 400)
                        return
                    self.bus.broadcast_state()
                    self._send_json(state)
                    return
                mode = body.get("mode")
                if mode == "sim":
                    state = self.hub.open_sim()
                elif mode == "real":
                    target = _imu_real_target(body)
                    if not target:
                        self._send_json({"error": "real 模式需要 target"
                                         "（tcp:host:port 或 serial:COMx@baud）"}, 400)
                        return
                    state = self.hub.open_real(target)
                else:
                    self._send_json({"error": f"unknown mode {mode!r}"}, 400)
                    return
                self.bus.broadcast_state()
                self._send_json(state)
                return
            if path == "/api/close":
                state = self.hub.close()
                self.bus.broadcast_state()
                self._send_json(state)
                return
            if path == "/api/imu/cmd":
                if not isinstance(self.hub, FrameHub):      # 电机页无 IMU 命令通道
                    self.send_error(404)
                    return
                func = str(body.get("func") or "").strip()
                args = body.get("args") or {}
                if not func or func not in FUNC_IMU:
                    self._send_json({"error": f"func 须为 {sorted(FUNC_IMU)} 之一，得 {func!r}"},
                                    400)
                    return
                if not isinstance(args, dict):
                    self._send_json({"error": "args 须为对象"}, 400)
                    return
                self._send_json(self.hub.imu_cmd(func, args))
                return
            if path == "/api/calib":
                if not isinstance(self.hub, FrameHub):      # 电机页无调零下行（走 /api/command）
                    self.send_error(404)
                    return
                # 旧入口兼容：内部就是 func=calibrate（有 ACK 语义，等回执最多 20s）
                result = self.hub.imu_cmd("calibrate")
                code = "calibSimulated" if self.hub.state().get("mode") == "sim" else "calibSent"
                self._send_json({"code": code, "cmd": "CAL", **result})
                return
            if path == "/api/store/cmd":
                if not isinstance(self.hub, StoreHub):  # 非 store 页无此命令通道
                    self.send_error(404)
                    return
                func = str(body.get("func") or "").strip()
                args = body.get("args") or {}
                if not func:
                    self._send_json({"error": "store cmd 需要 func 字段"}, 400)
                    return
                if not isinstance(args, dict):
                    self._send_json({"error": "args 须为对象"}, 400)
                    return
                try:
                    self._send_json(self.hub.store_cmd(func, args))
                except Exception as exc:
                    # 未连接 / 未知 func / 参数非法 → 409 明报失败（不静默忽略）
                    self._send_json({"error": str(exc)}, 409)
                return
            if path == "/api/verify":
                if not isinstance(self.hub, MotorHub):
                    self.send_error(404)
                    return
                self._send_json(self.hub.verify())
                return
            if path == "/api/command":
                if not isinstance(self.hub, MotorHub):
                    self.send_error(404)
                    return
                self._send_json(self.hub.send_line(str(body.get("line") or "")))
                return
        except Exception as exc:
            self._send_json({"error": str(exc)}, 409)
            return
        self.send_error(404)

    # ── SSE ──
    def _stream(self) -> None:
        q = self.bus.register()
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            self._write_event("state", self.hub.state())
            while True:
                try:
                    kind, payload = q.get(timeout=15.0)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")     # 心跳（防代理/浏览器超时断流）
                    self.wfile.flush()
                    continue
                self._write_event(kind, _jsonable(kind, payload))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.bus.unregister(q)

    def _write_event(self, kind: str, payload) -> None:
        data = json.dumps(payload, ensure_ascii=False)
        self.wfile.write(f"event: {kind}\ndata: {data}\n\n".encode("utf-8"))
        self.wfile.flush()

    def log_message(self, fmt: str, *args) -> None:  # 静默访问日志（控制台只留业务打印）
        pass


def _jsonable(kind: str, payload):
    """hub 回调 payload → JSON（tuple 转 list，stat 三计数转对象）。

    电机中枢（MotorHub）只发 JSON 就绪的 payload（text/verdict 为 dict），故此处只给
    mframe 的元组做形状映射；IMU 帧纹理保持原样（frame/stat/pos/port 分支不变）。
    """
    if kind == "mframe":
        typ, seq, scen, timers, recs = payload
        return {"type": typ, "seq": seq, "scen": scen,
                "timers": [list(t) for t in timers] if timers is not None else None,
                "recs": [{"slot": s, "kind": k, "sp": sp, "duty": du, "actual": ac,
                          "phase": ph, "fault": fa}
                         for (s, k, sp, du, ac, ph, fa) in recs]}
    if kind == "frame":
        typ, seq, vals = payload
        return {"type": typ, "seq": seq, "vals": list(vals)}
    if kind == "stat":
        imu, text, drop = payload
        return {"imu": imu, "text": text, "drop": drop}
    if kind == "pos":
        return {"x": payload[0], "y": payload[1], "z": payload[2]}
    if kind == "port":
        return {"connected": bool(payload)}
    return payload


def _imu_real_target(body: dict) -> str:
    """／api/open 的 real 请求体 → 统一目标串（tcp:<host>:<port> / serial:<COMx>[@baud]）。

    两种写法都认（页面新旧两条路径都能直接用）：
      {"target": "tcp:127.0.0.1:5558"}                 —— 原样（已带 scheme）
      {"port": "COM5", "baud": 115200}                —— 归一成 serial:COM5@115200
      {"port": "tcp:127.0.0.1:5558"}                   —— port 字段里放完整目标串也认
    什么都没给 → 返回 ""（调用方回 400）。
    """
    target = str(body.get("target") or "").strip()
    if target:
        return target
    port = str(body.get("port") or "").strip()
    if not port:
        return ""
    if port.startswith(("tcp:", "serial:")):
        return port
    return f"serial:{port}@{int(body.get('baud') or 115200)}"


def page_directory(page: str) -> Optional[Path]:
    """页面名 → 实例目录（IMU/电机/store 三套注册表统一入口）；未注册页面返回 None。"""
    entry = PAGES.get(page)
    if entry is not None:
        return entry[0]
    motor = MOTOR_PAGES.get(page)
    if motor is not None:
        return EXAMPLES_MOTOR / motor[0]
    store = STORE_PAGES.get(page)
    if store is not None:
        return EXAMPLES_STORE / store
    return None


def guess_type(path: Path) -> str:
    ext = path.suffix.lower()
    return {
        ".html": "text/html; charset=utf-8",
        ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".json": "application/json; charset=utf-8",
        ".png": "image/png",
        ".jpg": "image/jpeg",
        ".svg": "image/svg+xml",
        ".ico": "image/x-icon",
    }.get(ext, "application/octet-stream")


def list_serial_ports() -> list:
    """串口枚举（pyserial 可选）：缺失/异常都返回空列表，由页面如实提示。"""
    try:
        from serial.tools import list_ports
        return [{"device": p.device, "desc": p.description or ""}
                for p in list_ports.comports()]
    except Exception:
        return []


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="上位机本地服务器（页面 + 数据源中枢 + SSE）")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--page", default=DEFAULT_PAGE,
                    choices=sorted(list(PAGES) + list(MOTOR_PAGES) + list(STORE_PAGES)))
    ap.add_argument("--ensure", action="store_true",
                    help="单实例：已在运行则杀掉重启（run.bat 用）")
    ap.add_argument("--no-open", action="store_true", help="不自动打开浏览器")
    args = ap.parse_args(argv)

    page_dir = page_directory(args.page)
    if page_dir is None:
        raise SystemExit(f"[server] 未知页面 {args.page!r}")

    if args.ensure:
        ensure_restart(args.port, args.host)

    if not (page_dir / "index.html").is_file():
        print(f"[server] 警告：页面尚未迁移到实例目录：{page_dir / 'index.html'}", flush=True)

    if args.page in STORE_PAGES:
        hub = StoreHub()
    elif args.page in MOTOR_PAGES:
        _dirname, data_kind, scenario, scen_id, fixed = MOTOR_PAGES[args.page]
        hub = MotorHub(kind=data_kind, scenario=scenario, scen_id=scen_id,
                       fixed=fixed)
    else:
        _, sim_kind, with_traj = PAGES[args.page]
        hub = FrameHub(sim_kind=sim_kind,
                       traj_cfg=TRAJ_PRESETS["exp5_sway"] if with_traj else None)
    bus = HubBroadcaster(hub)
    Handler.hub, Handler.bus, Handler.default_page = hub, bus, args.page

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True
    url = f"http://{args.host}:{args.port}/{args.page}/"
    print(f"[server] 上位机服务 {url}（数据源：显式模式，缺省不启动；Ctrl+C 退出）", flush=True)
    print(f"[server] 页面目录：{page_dir}", flush=True)

    if not args.ensure:
        RUNTIME_DIR.mkdir(exist_ok=True)
        RUNTIME_FILE.write_text(json.dumps(
            {"pid": os.getpid(), "port": args.port, "started": time.time()},
            ensure_ascii=False), encoding="utf-8")

    if not args.no_open:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("[server] Ctrl+C，退出", flush=True)
    finally:
        hub.close()
        httpd.server_close()
        info = _read_runtime()
        if info.get("pid") == os.getpid():
            try:
                RUNTIME_FILE.unlink()
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
