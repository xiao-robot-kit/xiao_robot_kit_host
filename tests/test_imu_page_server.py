"""test_imu_page_server.py — IMU 统一页（逐级继承）服务器链路测试（headless，无 Qt/无浏览器）。

背景（用户定案 2026-11-06）：四个 IMU 实验页 = 薄壳声明面板集合 + 统一页面核心
（web/assets/imu-page.js）。页面能显示什么，取决于仿真档能否产出对应帧型；本测试
就按真实部署形态（本地服务器 + SSE）验证这条数据面契约：

  1) 路由：/exp1/ /exp2/ /exp4/ /exp5/ 全 200，且每页薄壳声明的面板集合逐级包含
     （exp1=[raw] ⊂ exp2=[raw,cal] ⊂ exp4=+euler,+quat ⊂ exp5=+traj）；
     /assets/imu-page.js 可服务（页面核心是共享资产）
  2) 显式模式：/api/state 缺省 mode=none（页面加载不启动任何源）
  3) 仿真档帧型覆盖：open sim → SSE 同时收到 0x01 raw / 0x03 euler / 0x04 quat
     （0x03 由 hub 从同一条 QUAT 流按 ZYX 解算派生，与固件 quat_to_euler 同式），
     exp5 还有 pos（轨迹重建）事件；0x01 带虚拟传感器的固有零偏（物理模型，非主机补偿）
  4) **IMU 命令通道（本轮核心验收）**：POST /api/imu/cmd {"func","args"}：
     a) get_bias（未标定）= 全 0
     b) calibrate → status=0 且 payload=bias6（= 虚拟传感器零偏）；get_bias 回读一致
     c) get_status → [flags, bias_valid, tc_valid]；temp_calib(秒) → k6
     d) set_flags(flags=1) → get_status 的 tc_valid=1、TEMPCOMP_EN 位=1
     e) get_drift / get_tc_state 与 temp_calib 的 k6 同源
     f) 未知 func / 缺 func → 400；未连接 → 409；标定后流上有 3 帧 0x02 + TEXT start/done
     g) **补偿在 MCU 内**：调零前 0x01 陀螺整周期均值 ≈ 传感器零偏；调零后 ≈ 0
        （虚拟 MCU 按固件语义把补偿量应用到输出；主机只转发，不做算法）
  5) 旧入口兼容：/api/calib 内部 = func=calibrate（回执也带 payload=bias6）
  6) close → 回到 none；real 目标不可达 → 409 且 state.source 仍为 none（不回落 sim）

统计口径（「整周期均值」）：仿真源是严格周期信号（exp5 = traj_sway，周期
period_s·rate_hz 个采样），连续帧段上取整数倍周期的尾部窗口 → 摆动分量求和
恰为 0，均值就只剩零偏。故必须选 seq 连续（mod 256）的帧段，丢帧即断段。

跑法：python tests/test_imu_page_server.py（exit 0 = 全过）
"""

import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SERVER = os.path.join(ROOT, "backend", "server.py")

sys.path.insert(0, os.path.join(ROOT, "backend"))
from frame import SOF, FrameDecoder, crc8  # noqa: E402
from sim_source import SIM_CAL_PRESETS, SIM_TRAJ_PRESETS  # noqa: E402

PAGES = {
    "exp1": ["raw"],
    "exp2": ["raw", "cal"],
    "exp4": ["raw", "cal", "euler", "quat"],
    "exp5": ["raw", "cal", "euler", "quat", "traj"],
}
SERVER_PAGE = "exp5"          # 仿真源选型：traj（QUAT+IMU 双流 + 轨迹重建）最全
CALIB_FRAMES = 3              # 契约：每次标定完成固件连发 3 帧 0x02

# 虚拟传感器零偏 / 温漂系数 / 仿真周期都从单源契约取（不手写数字）
SENSOR_BIAS = SIM_CAL_PRESETS["exp2_static"].bias          # (ax..az g, gx..gz dps)
SENSOR_GYRO = tuple(SENSOR_BIAS[3:6])
SENSOR_K = (tuple(SIM_CAL_PRESETS["exp2_static"].accel_drift)
            + tuple(SIM_CAL_PRESETS["exp2_static"].gyro_drift))   # 采样序 6 元（每 °C）
SWAY = SIM_TRAJ_PRESETS["traj_sway"]
PERIOD_FRAMES = round(SWAY.period_s * SWAY.rate_hz)         # 整周期采样数（= 160）
GYRO_TOL = 0.3                # 陀螺均值容差（dps）
MEASURE_FRAMES = 2 * PERIOD_FRAMES + 40   # 量测窗下限（≥2 周期 + 边界余量）

fails = []

# ───────────────────────── 假 MCU（real 链路回环用） ─────────────────────────

def _frame(typ: int, seq: int, payload: bytes) -> bytes:
    """按本仓 proto 编一帧（SOF|LEN|TYPE|SEQ|payload|CRC8）：测试侧独立实现，不复用 hub。"""
    head = bytes((SOF, len(payload), typ, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


class FakeMcu:
    """监听 tcp 端口的假 MCU：收 CMD（0x7D）按 [MOD][FUNC][params] 解，回 ACK（0x7E）。

    用途：验 host 侧**真实链路**协议层（sim 档走虚拟 MCU，不覆盖 encode_cmd/
    ACK 匹配/超时）。ack=False 时可模拟「MCU 不回 ACK」。
    """

    def __init__(self) -> None:
        self._srv = socket.socket()
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self.cmds = []                       # [(func, params, mod)]
        self.bias = SIM_CAL_PRESETS["exp2_static"].bias
        self._ack = True
        self._conn = None
        self._lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "FakeMcu":
        self.thread.start()
        return self

    def ack_stop(self) -> None:
        """此后收到 CMD 不再回 ACK（模拟固件无响应）。"""
        self._ack = False

    def send(self, data: bytes) -> None:
        """主动往流上推字节（如 0x01 帧，验原值透传）。"""
        with self._lock:
            if self._conn is not None:
                self._conn.sendall(data)

    def _run(self) -> None:
        try:
            conn, _ = self._srv.accept()
        except OSError:
            return
        with self._lock:
            self._conn = conn
        dec = FrameDecoder()
        try:
            while True:
                chunk = conn.recv(512)
                if not chunk:
                    break
                for typ, seq, payload in dec.feed(chunk):
                    if typ != 0x7D or len(payload) < 2:
                        continue
                    mod, func, params = payload[0], payload[1], payload[2:]
                    self.cmds.append((func, params, mod))
                    if not self._ack:
                        continue
                    data = b""
                    if func in (0x01, 0x02, 0x03, 0x0D):      # calibrate/get_bias/temp_calib/get_drift
                        data = struct.pack("<6f", *self.bias)
                    conn.sendall(_frame(0x7E, seq, bytes((mod, func, 0)) + data))
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def close(self) -> None:
        try:
            self._srv.close()
        except OSError:
            pass


def check(name, ok, detail=""):
    if not ok:
        fails.append(f"{name}{('：' + detail) if detail else ''}")


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


def longest_run(frames: list) -> list:
    """取 seq（mod 256）连续的最长帧段：段内相邻帧序号步进 1 ⇒ 仿真时基连续。"""
    best, cur, prev = [], [], None
    for f in frames:
        if prev is not None and f["seq"] == (prev + 1) & 0xFF:
            cur.append(f)
        else:
            cur = [f]
        if len(cur) > len(best):
            best = list(cur)
        prev = f["seq"]
    return best


def gyro_period_mean(frames: list):
    """陀螺三轴整周期均值（dps）：返回 (mean|None, 连续帧段长度)。

    摆动分量在整数倍周期上求和恰为 0 → 均值只剩零偏。连续段长度不足一个周期
    则返回 None（调用方断言失败，不得静默当作 0）。
    """
    run = longest_run(frames)
    k = (len(run) // PERIOD_FRAMES) * PERIOD_FRAMES
    if k < PERIOD_FRAMES:
        return None, len(run)
    tail = run[-k:]
    return [sum(f["vals"][3 + i] for f in tail) / k for i in range(3)], len(run)


def http(url: str, timeout: float = 3.0, method: str = "GET", body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.read()


class StreamReader:
    """后台线程读 SSE（事件名 + JSON 载荷），主线程做 POST/断言。"""

    def __init__(self, port: int) -> None:
        self.port = port
        self.events = []            # [(event, payload)]
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.resp = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "StreamReader":
        self.thread.start()
        return self

    def _run(self) -> None:
        try:
            self.resp = urllib.request.urlopen(
                f"http://127.0.0.1:{self.port}/api/stream", timeout=20)
            name = None
            while not self.stop.is_set():
                line = self.resp.readline()
                if not line:
                    break
                text = line.decode("utf-8", "replace").strip()
                if text.startswith("event: "):
                    name = text[7:]
                elif text.startswith("data: ") and name:
                    try:
                        payload = json.loads(text[6:])
                    except ValueError:
                        payload = None
                    with self.lock:
                        self.events.append((name, payload))
                    name = None
        except Exception:
            pass                        # 主线程 close()/服务器退出都会走到这里

    def clear(self) -> None:
        """丢弃已收事件（测「标定之后」的数据面时用：避免把标定前的样本混进均值）。"""
        with self.lock:
            self.events.clear()

    def snapshot(self) -> list:
        with self.lock:
            return list(self.events)

    def frames(self, typ=None) -> list:
        out = [(n, p) for n, p in self.snapshot()
               if n == "frame" and isinstance(p, dict)]
        return [p for _n, p in out if typ is None or p.get("type") == typ]

    def texts(self) -> list:
        return [p for n, p in self.snapshot() if n == "text" and isinstance(p, str)]

    def poses(self) -> list:
        return [p for n, p in self.snapshot() if n == "pos" and isinstance(p, dict)]

    def wait_for(self, cond, budget: float, label: str) -> bool:
        deadline = time.time() + budget
        while time.time() < deadline:
            if cond(self):
                return True
            time.sleep(0.02)
        return False

    def close(self) -> None:
        self.stop.set()
        try:
            if self.resp is not None:
                self.resp.close()
        except Exception:
            pass


def main() -> int:
    port = free_port()
    proc = subprocess.Popen([sys.executable, SERVER, "--no-open", "--port", str(port),
                             "--page", SERVER_PAGE],
                            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    stream = None
    meas = {}                  # 实测证据：调零前后的陀螺整周期均值（dps）
    try:
        if not wait_ready(port):
            print("== test_imu_page_server: 服务器未就绪 ==")
            return 1
        base = f"http://127.0.0.1:{port}"

        # 1) 页面路由 + 薄壳面板声明（逐级包含）
        for page, want in PAGES.items():
            code, body = http(base + f"/{page}/")
            text = body.decode("utf-8", "replace")
            check(f"{page} 页面 200", code == 200, f"code={code}")
            check(f"{page} 薄壳声明 IMU_PAGE", "window.IMU_PAGE" in text)
            check(f"{page} 薄壳 id 自洽", f'id: "{page}"' in text)
            for panel in want:
                check(f"{page} 声明面板 {panel}", f'"{panel}"' in text,
                      text[:120].replace("\n", " "))
            check(f"{page} 载入统一核心", "/assets/imu-page.js" in text)
        code, body = http(base + "/assets/imu-page.js")
        check("页面核心资产可服务", code == 200 and b"IMU_PAGE" in body, f"code={code}")

        # 2) 显式模式：缺省不启动
        code, body = http(base + "/api/state")
        state = json.loads(body)
        check("缺省 none/未连接",
              state.get("mode") == "none" and state.get("connected") is False, str(state))

        # 3) 仿真档帧型覆盖：raw / euler / quat（+ pos）
        stream = StreamReader(port).start()
        code, body = http(base + "/api/open", method="POST", body={"mode": "sim"})
        opened = json.loads(body)
        check("open sim 生效", opened.get("mode") == "sim", str(opened))
        ok = stream.wait_for(
            lambda s: (s.frames(1) and s.frames(3) and s.frames(4) and s.poses()),
            budget=4.0, label="raw/euler/quat/pos")
        check("SSE 收到 raw/euler/quat/pos", ok,
              f"raw={len(stream.frames(1))} euler={len(stream.frames(3))} "
              f"quat={len(stream.frames(4))} pos={len(stream.poses())}")

        # 3b) 标定前：仿真档流上带**虚拟传感器固有零偏**（物理模型；gz 是恒定分量）
        ok = stream.wait_for(lambda s: len(s.frames(1)) >= MEASURE_FRAMES, budget=6.0,
                             label="0x01 pre-cal")
        pre_mean, pre_run = gyro_period_mean(stream.frames(1))
        meas["pre"] = pre_mean
        check("标定前 0x01 gz ≈ 传感器零偏 2.5 dps",
              pre_mean is not None and abs(pre_mean[2] - SENSOR_GYRO[2]) <= GYRO_TOL,
              f"pre={pre_mean} want gz≈{SENSOR_GYRO[2]} (run={pre_run}/{PERIOD_FRAMES})")

        raw = stream.frames(1)
        euler = stream.frames(3)
        quat = stream.frames(4)
        if raw:
            check("0x01 六轴载荷", len(raw[0]["vals"]) == 6, str(raw[0]["vals"]))
        if euler:
            vals = euler[0]["vals"]
            check("0x03 三轴载荷（弧度）",
                  len(vals) == 3 and all(abs(v) <= math.pi + 1e-3 for v in vals), str(vals))
            # 派生通道与固件同式：拿同 seq 的 QUAT 反算 ZYX 对照（欧拉由四元数解算）
            by_seq = {f["seq"]: f["vals"] for f in reversed(quat)}
            ref = None
            for f in euler:
                if f["seq"] in by_seq:
                    ref = by_seq[f["seq"]]
                    got = f["vals"]
                    break
            if ref is not None:
                qw, qx, qy, qz = ref
                roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
                sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
                pitch = math.asin(sinp)
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                check("0x03 由 0x04 按 ZYX 解算（同固件式）",
                      all(abs(a - b) < 1e-5 for a, b in zip([roll, pitch, yaw], got)),
                      f"quat={ref} euler={got}")
        if quat:
            vals = quat[0]["vals"]
            norm = math.sqrt(sum(v * v for v in vals))
            check("0x04 四分量载荷（|q|≈1）", len(vals) == 4 and abs(norm - 1.0) < 0.05,
                  f"vals={vals} |q|={norm:.4f}")

        # 4) IMU 命令通道（用户定案：算法全在 MCU，上位机只发命令 + 读回执）
        #    4a) 命令通道语义：calibrate 回 bias6、get_bias 回读一致、temp_calib 回 k6、
        #        set_flags 后 get_status 的 tc_valid=1、get_drift/get_tc_state 同源
        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "get_bias"})
        r = json.loads(body)
        check("未标定时 get_bias 回全 0", code == 200 and r.get("status") == 0
              and r.get("payload") == [0.0] * 6, str(r))

        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "calibrate"})
        r_cal = json.loads(body)
        check("calibrate 返回 status=0", code == 200 and r_cal.get("status") == 0, str(r_cal))
        check("calibrate 返回 bias6（非零）", len(r_cal.get("payload") or []) == 6
              and all(abs(v) > 1e-6 for v in r_cal["payload"]), str(r_cal.get("payload")))
        check("calibrate 的 bias6 = 虚拟传感器零偏",
              r_cal.get("payload") is not None and
              all(abs(a - b) <= 1e-6 for a, b in zip(r_cal["payload"], SENSOR_BIAS)),
              f"{r_cal.get('payload')} vs {SENSOR_BIAS}")

        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "get_bias"})
        r_get = json.loads(body)
        check("get_bias 回读与 calibrate 一致",
              r_get.get("status") == 0 and
              all(abs(a - b) <= 1e-6 for a, b in zip(r_get.get("payload") or [], r_cal["payload"])),
              str(r_get.get("payload")))

        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "get_status"})
        st1 = json.loads(body)
        check("get_status：flags=0 / bias_valid=1 / tc_valid=0",
              st1.get("payload") == [0, 1, 0], str(st1.get("payload")))

        code, body = http(base + "/api/imu/cmd", method="POST",
                          body={"func": "temp_calib", "args": {"seconds": 5}})
        r_tc = json.loads(body)
        check("temp_calib 返回 status=0 + k6", r_tc.get("status") == 0
              and len(r_tc.get("payload") or []) == 6, str(r_tc))
        check("temp_calib 的 k6 = 预设温漂系数",
              r_tc.get("payload") is not None and
              all(abs(a - b) <= 1e-6 for a, b in zip(r_tc["payload"], SENSOR_K)),
              f"{r_tc.get('payload')} vs {SENSOR_K}")

        code, body = http(base + "/api/imu/cmd", method="POST",
                          body={"func": "set_flags", "args": {"flags": 1}})
        sf = json.loads(body)
        check("set_flags 返回生效 flags=1", sf.get("status") == 0
              and sf.get("payload") == [1], str(sf.get("payload")))

        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "get_status"})
        st2 = json.loads(body)
        check("set_flags 后 get_status 的 tc_valid=1",
              (st2.get("payload") or [])[2] == 1, str(st2.get("payload")))
        check("set_flags 后 TEMPCOMP_EN 位=1", (st2.get("payload") or [0])[0] & 1 == 1,
              str(st2.get("payload")))

        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "get_drift"})
        r_dr = json.loads(body)
        check("get_drift 与 temp_calib 的 k6 一致",
              r_dr.get("status") == 0 and
              all(abs(a - b) <= 1e-6 for a, b in zip(r_dr.get("payload") or [], SENSOR_K)),
              str(r_dr.get("payload")))

        code, body = http(base + "/api/imu/cmd", method="POST", body={"func": "get_tc_state"})
        r_tcs = json.loads(body)
        check("get_tc_state = [valid, enabled, k6]",
              (r_tcs.get("payload") or [])[:2] == [1, 1] and
              len(r_tcs.get("payload") or []) == 8, str(r_tcs.get("payload")))

        try:
            http(base + "/api/imu/cmd", method="POST", body={"func": "nope"})
            check("未知 func → 400", False, "未报错")
        except urllib.error.HTTPError as exc:
            check("未知 func → 400", exc.code == 400, f"code={exc.code}")
        try:
            http(base + "/api/imu/cmd", method="POST", body={})
            check("缺 func → 400", False, "未报错")
        except urllib.error.HTTPError as exc:
            check("缺 func → 400", exc.code == 400, f"code={exc.code}")

        # 4b) 标定的流面回执：TEXT start/done + 3 帧 0x02（固件同形）
        ok = stream.wait_for(lambda s: len(s.frames(2)) >= CALIB_FRAMES, budget=3.0,
                             label="0x02")
        check("calibrate 后流上有 3 帧 0x02", ok, f"n={len(stream.frames(2))}")
        cal = stream.frames(2)
        if cal:
            vals = cal[-1]["vals"]
            check("0x02 十九值载荷（raw|deb|bias|temp）", len(vals) == 19, str(len(vals)))
            check("0x02 温度位合理", 0.0 < vals[18] < 60.0, str(vals[18]))
            check("0x02 补偿关系 deb = raw − bias",
                  all(abs(vals[i] - vals[6 + i] - vals[12 + i]) < 1e-5 for i in range(6)),
                  str(vals))
            check("0x02 bias 块 = MCU 回的零偏",
                  all(abs(vals[12 + i] - SENSOR_BIAS[i]) < 1e-5 for i in range(6)),
                  str(vals[12:18]))
        ok = stream.wait_for(
            lambda s: (any("OK calib start" in t for t in s.texts())
                       and any("OK calib done bias" in t for t in s.texts())),
            budget=3.0, label="TEXT")
        check("calibrate 回执 TEXT start + done bias …", ok, str(stream.texts()[-3:]))
        check("TEXT 回执带窗长 N=500",
              any("OK calib start N=500" == t.strip() for t in stream.texts()),
              str(stream.texts()[-3:]))

        # 4c) 旧入口兼容：/api/calib 内部 = func=calibrate（回执也带回 payload=bias6）
        code, body = http(base + "/api/calib", method="POST", body={})
        legacy = json.loads(body)
        check("旧 /api/calib 仍可用（code=calibSimulated）",
              legacy.get("code") == "calibSimulated", str(legacy))
        check("旧 /api/calib 回执带 func/status/payload",
              legacy.get("func") == "calibrate" and legacy.get("status") == 0
              and len(legacy.get("payload") or []) == 6, str(legacy.get("payload")))

        # 4d) **补偿在 MCU 内（虚拟 MCU 同语义）**：调零后 0x01 陀螺整周期均值 ≈ 0
        #     （调零前 ≈ 传感器零偏 的断言在 3b；这里先 clear，只统计**标定之后**的帧，
        #      否则均值里会混进标定前的带偏样本 → 假失败）
        stream.clear()
        ok = stream.wait_for(lambda s: len(s.frames(1)) >= MEASURE_FRAMES,
                             budget=6.0, label="0x01 post-cal")
        check("采到足量 0x01", ok, f"n={len(stream.frames(1))}")
        mean, run = gyro_period_mean(stream.frames(1))
        meas["mean"] = mean
        check("标定后 0x01 陀螺均值 ≈ 0（MCU 补偿已作用到输出）",
              mean is not None and
              all(abs(a) <= GYRO_TOL for a in mean),
              f"mean={mean} want≈(0,0,0) (run={run}/{PERIOD_FRAMES})")

        # 5) close → 回到 none；real 目标不可达 → 409 且 state.source 仍为 none（不回落 sim）
        code, body = http(base + "/api/close", method="POST", body={})
        state = json.loads(body)
        check("close 回到 none",
              state.get("mode") == "none" and state.get("connected") is False, str(state))

        dead = f"tcp:127.0.0.1:{free_port()}"        # 刚释放的端口：无人监听
        try:
            http(base + "/api/open", method="POST",
                 body={"mode": "real", "target": dead})
            check("real 目标不可达 → 409", False, "未报错（应为 409）")
        except urllib.error.HTTPError as exc:
            check("real 目标不可达 → 409", exc.code == 409, f"code={exc.code}")
            check("409 带明报错误文案", b"error" in exc.read())
        code, body = http(base + "/api/state")
        state = json.loads(body)
        check("real 失败后不回落 sim",
              state.get("mode") == "none" and state.get("source") == "none"
              and state.get("connected") is False, str(state))
        try:
            http(base + "/api/open", method="POST", body={"mode": "real", "target": "bogus"})
            check("未知目标串 → 409", False, "未报错（应为 409）")
        except urllib.error.HTTPError as exc:
            check("未知目标串 → 409", exc.code == 409, f"code={exc.code}")

        # 6) 未连接 → 命令通道 409（页面据此明报失败，不静默）
        try:
            http(base + "/api/imu/cmd", method="POST", body={"func": "get_bias"})
            check("未连接下发命令 → 409", False, "未报错（应为 409）")
        except urllib.error.HTTPError as exc:
            check("未连接下发命令 → 409", exc.code == 409, f"code={exc.code}")
        # 7) real 链路协议层：CMD 帧下行 → ACK 帧解析（假 MCU 回环，验 host 侧编解码）
        #    同时验证 **0x01 原值透传**：主机绝不对数据流做减偏/改写。
        mcu = FakeMcu().start()
        try:
            code, body = http(base + "/api/open", method="POST",
                              body={"mode": "real",
                                    "target": f"tcp:127.0.0.1:{mcu.port}"})
            st = json.loads(body)
            check("real tcp 目标 open 成功", code == 200 and st.get("source") == "tcp",
                  str(st))
            code, body = http(base + "/api/imu/cmd", method="POST",
                              body={"func": "get_bias"}, timeout=12)
            r = json.loads(body)
            check("real get_bias → ACK 解析出 bias6",
                  r.get("status") == 0 and
                  all(abs(a - b) < 1e-6 for a, b in zip(r.get("payload") or [], mcu.bias)),
                  str(r))
            check("下行 CMD 帧载荷 = [MOD=1][FUNC=0x02]（无参数）",
                  mcu.cmds and mcu.cmds[-1] == (0x02, b"", 0x01), str(mcu.cmds[-1:]))

            code, body = http(base + "/api/imu/cmd", method="POST",
                              body={"func": "temp_calib", "args": {"seconds": 7}}, timeout=12)
            r = json.loads(body)
            check("real temp_calib 参数 = 秒数 7（params u8）",
                  mcu.cmds[-1][0] == 0x03 and mcu.cmds[-1][1] == bytes((7,)),
                  str(mcu.cmds[-1]))

            mcu.send(_frame(0x01, 9, struct.pack("<6f", 0.1, 0.2, 0.3, 4.0, 5.0, 6.0)))
            ok = stream.wait_for(
                lambda s: any(abs(f["vals"][3] - 4.0) < 1e-6 and abs(f["vals"][0] - 0.1) < 1e-6
                              for f in s.frames(1)),
                budget=3.0, label="0x01 透传")
            check("real 0x01 原值透传（主机不减偏/不改写）", ok,
                  str([f["vals"] for f in stream.frames(1)[-1:]]))

            mcu.ack_stop()
            t0 = time.time()
            try:
                http(base + "/api/imu/cmd", method="POST", body={"func": "get_status"},
                     timeout=15)
                check("MCU 不回 ACK → 409（明报超时）", False, "未报错")
            except urllib.error.HTTPError as exc:
                check("MCU 不回 ACK → 409（明报超时）", exc.code == 409,
                      f"code={exc.code} 用时{time.time() - t0:.1f}s")
                check("超时错误文案含「回执超时」", "超时" in exc.read().decode("utf-8"),
                      "error 文案缺关键词")
        finally:
            mcu.close()
            http(base + "/api/close", method="POST", body={})

    finally:
        if stream is not None:
            stream.close()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    if fails:
        for f in fails:
            print("  [FAIL]", f)
        print(f"== test_imu_page_server: {len(fails)} fail ==")
        return 1
    print("== test_imu_page_server: 四页路由/逐级面板/仿真帧型覆盖(raw·euler·quat·pos)/"
          "IMU 命令通道(calibrate·get_bias·get_status·temp_calib·set_flags·get_drift·"
          "get_tc_state)/旧 /api/calib 兼容/主机不减偏/real 409/real 链路 CMD·ACK 回环,"
          " 0 fail ==")
    print(f"   [data] 传感器零偏 gx/gy/gz={SENSOR_GYRO} dps · 温漂 k6={SENSOR_K} · "
          f"周期 {PERIOD_FRAMES} 帧 · 标定后 0x01 陀螺整周期均值={meas.get('mean')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
