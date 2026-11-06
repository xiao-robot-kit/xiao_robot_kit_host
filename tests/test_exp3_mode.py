#!/usr/bin/env python
"""exp3 独立页「仿真/真实」显式模式 + zh/en 双语的回归测试（stdlib，无第三方）。

链路：examples/imu/exp3_euler_view/{euler_view.py,index.html}
断言：
  1) 模式解析：不带 --mode/--source = none（空闲，禁止隐式回落）；--source 是旧别名
     （sim→sim，tcp→real）；两者冲突报错；--tcp 地址解析（缺端口取 5558）
  1b) 目标串解析：`tcp:<host>:<port>` / `serial:<COMx>@<baud>`（baud 可省 → 缺省 115200）；
     其它写法一律 ValueError（非法目标 → HTTP 400 且档位不变）
  2) 页面契约（静态扫描 index.html）：持久化键 xrk.mode / xrk.lang / xrk.target 且
     缺省 real / zh；词典 zh/en 成对（每个键至少出现两次）；只在用户点击路径 POST /api/mode；
     真实模式目标输入框 + 刷新串口按钮（只填 datalist，不覆盖输入值）
  3) HTTP 行为（真起子进程）：
     a) 无模式起服务 = idle：mode=none、connected=False、euler_frames=0、表显无 last_deg
     b) --mode sim 起服务：/api/euler 两次轮询 euler_frames 增长
     c) GET /api/mode?value=none → mode none 且停流（last_deg 清空）；再 ?value=sim 恢复增长
     d) POST /api/mode（JSON body）同样生效；非法值 → 400 且模式不变
     e) --mode real --serial <不存在的口>：不崩、未连接、不回落仿真；/api/euler.target 报串口目标
     f) /api/mode?value=real&target=serial:... 切目标；不带 target 时沿用上一次目标；
        非法目标 → 400 且目标与档位不变
     g) GET /api/ports：{"ports": [{"device", "desc"}]}（pyserial 缺失时空数组 + error）
  4) 零点校准（MCU 内部标定，主机只发命令 + 读数据）：
     a) 编解码：CMD 帧（0x7D = MOD|FUNC|params）字段序；ACK（0x7E = MOD|FUNC|STATUS|data）
        按功能码解析各数据块（GET_BIAS 24B / GET_STATUS 20B / GET_TC_STATE 28B / GET_DRIFT 24B）
     b) 虚拟 MCU 直测：GET_* 同步回执；SET_FLAGS 只改开关；CALIBRATE 异步（采集窗过后才 ACK+3×0x02）
     c) 空闲 / real 未连接：GET|POST /api/imu/cmd → {"code":"notConnected"}（不启源、不改渲染面）
     d) sim 档三个控件全链路：读取(GET_BIAS+GET_STATUS) / 一键调零(CALIBRATE 异步) /
        温漂(TEMP_CALIB→tc_valid、SET_FLAGS 开关、GET_TC_STATE、GET_DRIFT)，面板走 /api/euler.imu
     e) real 档假 MCU 端到端：CMD 字节上链路 → ACK → 渲染面；0x02 帧 deb 原样透传（主机不二次去偏）
  5) 页面新增契约：三个控件按钮 + 只读面板（0x02 raw·deb·bias + GET_BIAS 行 + 状态行）+
     Δ=raw−deb + TEXT 日志 OK calib 高亮 + 命令文案 zh/en 成对 + 初始化不发命令
跑法：python tests/test_exp3_mode.py（exit 0 = 全过）
"""

import importlib.util
import json
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_DIR = ROOT / "examples" / "imu" / "exp3_euler_view"
SCRIPT = EXAMPLE_DIR / "euler_view.py"
PAGE = EXAMPLE_DIR / "index.html"
DEFAULT_TCP_PORT = 5558                       # 与服务端 DEFAULT_TCP_PORT 同值（页面/命令行缺省）

fails = []


def check(name, ok, detail=""):
    if not ok:
        fails.append(f"{name}{'：' + str(detail) if detail else ''}")


def load_module():
    spec = importlib.util.spec_from_file_location("exp3_euler_view", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def free_port():
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def get_json(url, method="GET", body=None, timeout=5.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, json.loads(resp.read().decode("utf-8"))


def wait_port(port, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.05)
    return False


class Server:
    """起 euler_view.py 子进程（幂等回收；控制台输出留档便于失败排查）。"""

    def __init__(self, *args):
        self.port = free_port()
        self.cmd = [sys.executable, str(SCRIPT), "--http-port", str(self.port)] + list(args)
        self.proc = None
        self.log_path = ROOT / f".exp3_mode_test_{self.port}.log"

    def __enter__(self):
        self.log = open(self.log_path, "w", encoding="utf-8")
        self.proc = subprocess.Popen(self.cmd, stdout=self.log, stderr=subprocess.STDOUT)
        if not wait_port(self.port):
            raise RuntimeError(f"页服务未就绪：{' '.join(self.cmd)}")
        return self

    def __exit__(self, *exc):
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.log.close()
        self.console = self.log_path.read_text(encoding="utf-8", errors="replace")
        self.log_path.unlink(missing_ok=True)
        return False

    def url(self, path):
        return f"http://127.0.0.1:{self.port}{path}"


def approx(got, want, tol=1e-4):
    """六轴/多值浮点对比（缺值直接 False，不看真假值）。"""
    return (got is not None and len(got) == len(want)
            and all(abs(a - b) <= tol for a, b in zip(got, want)))


def make_frame(mod, typ, seq, payload):
    """造一帧（与服务端同 CRC 口径）：SOF | LEN | TYPE | SEQ | payload | CRC8。"""
    head = bytes((mod.SOF, len(payload), typ, seq & 0xFF))
    return head + payload + bytes((mod.crc8(head[1:] + payload),))


# 假 MCU 回包内容：0x02 帧的 raw|deb|bias|temp 十九值（块序 ax..az g, gx..gz dps）
CAL_RAW = (0.02, -0.03, 0.015, 1.0, -0.5, 2.5)
CAL_DEB = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)          # MCU 已去偏（主机不得二次减）
CAL_BIAS = (0.02, -0.03, -0.985, 1.0, -0.5, 2.5)  # 固件口径：加速度轴已扣重力
CAL_TEMP = 26.5
CAL_DONE_TEXT = ("OK calib done bias ax=20 ay=-30 az=15 mg "
                 "gx=1000 gy=-500 gz=2500 mdps temp=26500 mC")


class FakeBridge:
    """假 MCU（TCP UART0 桥替身）：解主机 CMD 帧（0x7D），按 (MOD,FUNC) 回 ACK（0x7E）。

    CALIBRATE 按固件语义**异步**：同一个 ACK 包后面紧跟 3 帧 0x02（raw|deb|bias|temp）。
    GET_STATUS / GET_BIAS / GET_TC_STATE / GET_DRIFT 回对应数据块；未知功能码回 0xFF。
    """

    def __init__(self, mod, bias=(1.0, -0.5, 2.5, 0.02, -0.03, 0.015),
                 flags=0x000F, status_extra=None):
        self._mod = mod
        self._bias = bias                       # imu_bias_t 口径：gyro 在前
        self._flags = flags
        self._status_extra = status_extra or {}
        self.received = []                      # [(mod_id, func_id, params_bytes)]
        self._stop = threading.Event()
        self._srv = socket.socket()
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self._srv.settimeout(0.2)
        self.port = self._srv.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        self._thread.join(3.0)
        return False

    def _serve(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except (socket.timeout, OSError):
                continue
            conn.settimeout(0.2)
            dec = self._mod.FrameDecoder()
            try:
                while not self._stop.is_set():
                    try:
                        chunk = conn.recv(256)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    for typ, _seq, payload in dec.feed(chunk):
                        if typ != self._mod.TYPE_CMD or len(payload) < 2:
                            continue
                        mid, func = payload[0], payload[1]
                        self.received.append((mid, func, payload[2:]))
                        conn.sendall(self._reply(mid, func))
            finally:
                try:
                    conn.close()
                except OSError:
                    pass

    def _frame(self, typ, payload):
        return make_frame(self._mod, typ, 0, payload)

    def _ack(self, mid, func, data=b"", status=0):
        return self._frame(self._mod.TYPE_ACK, bytes((mid, func, status)) + data)

    def _reply(self, mid, func):
        M = self._mod
        bias = self._bias
        if func == M.FUNC_IMU["GET_BIAS"]:
            return self._ack(mid, func, struct.pack("<6f", *bias))
        if func == M.FUNC_IMU["GET_STATUS"]:
            data = bytes((0, 0x07, 20, 0, 0, 0)) + struct.pack("<H", self._flags)
            data += bytes((1, 1, 20, 5)) + bytes(8)
            return self._ack(mid, func, data)
        if func == M.FUNC_IMU["GET_TC_STATE"]:
            data = bytes((1, 1 if self._flags & 1 else 0))
            data += struct.pack("<3f", 27.0, 25.0, 26.0) + struct.pack("<3f", 0.05, 0.05, 0.05)
            data += struct.pack("<H", 3)
            return self._ack(mid, func, data)
        if func == M.FUNC_IMU["GET_DRIFT"]:
            return self._ack(mid, func, struct.pack("<6f", 0.05, 0.05, 0.05,
                                                   0.0005, 0.0005, 0.0005))
        if func == M.FUNC_IMU["CALIBRATE"]:
            cal = struct.pack("<19f", *(CAL_RAW + CAL_DEB + CAL_BIAS + (CAL_TEMP,)))
            return (self._ack(mid, func)
                    + b"".join(self._frame(M.TYPE_IMU_CAL, cal) for _ in range(3)))
        if func == M.FUNC_IMU["SET_FLAGS"]:
            return self._ack(mid, func)
        return self._ack(mid, func, status=0xFF)


def test_resolve_mode(mod):
    cases = [
        ((None, None), "none"),          # 什么都不给 = 空闲（禁止隐式回落）
        (("sim", None), "sim"),
        (("real", None), "real"),
        ((None, "sim"), "sim"),          # --source 旧别名
        ((None, "tcp"), "real"),
        (("sim", "sim"), "sim"),         # 同义并存不算冲突
        (("real", "tcp"), "real"),
    ]
    for args, want in cases:
        got = mod.resolve_mode(*args)
        check(f"模式解析 {args}", got == want, f"got={got} want={want}")
    try:
        mod.resolve_mode("sim", "tcp")
        check("模式冲突报错", False, "未抛 ValueError")
    except ValueError as e:
        check("模式冲突报错", "--source" in str(e), str(e))


def test_split_addr(mod):
    check("地址解析 完整", mod.split_tcp_addr("127.0.0.1:5999") == ("127.0.0.1", 5999))
    check("地址解析 缺端口", mod.split_tcp_addr("myhost") == ("myhost", mod.DEFAULT_TCP_PORT))
    check("地址解析 缺 host", mod.split_tcp_addr(":6000") == ("127.0.0.1", 6000))
    try:
        mod.split_tcp_addr("127.0.0.1:99999")
        check("地址解析 端口越界", False, "未抛 ValueError")
    except ValueError:
        check("地址解析 端口越界", True)


def test_target_parse(mod):
    cases = [
        ("tcp:127.0.0.1:5558", "tcp:127.0.0.1:5558", "tcp"),
        ("tcp:127.0.0.1:5999", "tcp:127.0.0.1:5999", "tcp"),
        ("tcp::6000", "tcp:127.0.0.1:6000", "tcp"),     # 缺 host → 回环
        ("tcp:myhost", f"tcp:myhost:{mod.DEFAULT_TCP_PORT}", "tcp"),
        ("serial:COM5@115200", "serial:COM5@115200", "serial"),
        ("serial:COM5", "serial:COM5@115200", "serial"),              # baud 省 → 缺省
        ("serial:COM5@9600", "serial:COM5@9600", "serial"),
        ("  serial:COM5  ", "serial:COM5@115200", "serial"),          # 容忍两端空白
    ]
    for text, want, scheme in cases:
        tgt = mod.parse_target(text)
        check(f"目标解析 {text!r}", tgt.text == want and tgt.scheme == scheme,
              f"got={tgt.text}/{tgt.scheme} want={want}/{scheme}")
    check("目标解析 自定义缺省波特率",
          mod.parse_target("serial:COM7", 57600).text == "serial:COM7@57600")
    for bad in ("", "   ", "udp:1:2", "serial:", "serial:COM5@fast", "tcp:127.0.0.1:99999"):
        try:
            mod.parse_target(bad)
            check(f"非法目标报错 {bad!r}", False, "未抛 ValueError")
        except ValueError:
            check(f"非法目标报错 {bad!r}", True)


def test_page_contract():
    html = PAGE.read_text(encoding="utf-8")
    blocks = re.findall(r"<script>([\s\S]*?)</script>", html)
    check("页面内联脚本单块", len(blocks) == 1, f"blocks={len(blocks)}")
    js = blocks[0] if blocks else ""
    check("页面零外链脚本", "<script src=" not in html)
    check("持久化键 xrk.mode", 'LS_MODE = "xrk.mode"' in js)
    check("持久化键 xrk.lang", 'LS_LANG = "xrk.lang"' in js)
    check("持久化键 xrk.target", 'LS_TARGET = "xrk.target"' in js)
    check("缺省模式 real", re.search(r'readPref\(LS_MODE, MODES, "real"\)', js) is not None)
    check("缺省语言 zh", re.search(r'readPref\(LS_LANG, LANGS, "zh"\)', js) is not None)
    check("只轮询 100ms", "setInterval(tick, 100)" in js)
    # 词典成对：这些键在 zh/en 各出现一次（缺一侧即漏词）
    for key in ("title", "doc.title", "mode.sim", "mode.real", "btn.startSim", "btn.connect",
                "btn.disconnect", "btn.lang", "status.idle", "status.connected",
                "status.disconnected", "meta.idle", "note.body", "foot",
                "target.label", "target.ph", "target.title", "btn.ports", "btn.portsTip",
                "ports.found", "ports.error", "meta.target"):
        n = js.count('"' + key + '"')
        check(f"词典键成对 {key}", n >= 2, f"count={n}")
    # 真实模式目标输入：id/联想/按钮齐备，POST 带 target，串口探测只读不覆盖输入值
    check("目标输入框 id=target", 'id="target"' in html and "list=\"ports\"" in html)
    check("串口联想 datalist", 'id="ports"' in html and "datalist" in html)
    check("刷新串口按钮", 'id="btn-ports"' in html and 'fetch("/api/ports")' in js)
    check("目标输入只在真实模式显示", 'byId("realbar").hidden = pref.mode !== "real"' in js)
    check("POST 带 target 参数", 'url += "&target=" + encodeURIComponent(tgt)' in js
          and 'store.setItem(LS_TARGET, tgt)' in js)
    check("空目标按 placeholder 默认", 'return v || el.placeholder' in js)
    ports_fn = js.split("function refreshPorts", 1)[-1].split("function postMode", 1)[0]
    check("刷新串口不改输入值", 'byId("target").value' not in ports_fn, ports_fn[:120])
    # 禁自动启动：POST /api/mode 只出现在 postMode() 里，且 postMode 只在按钮回调里被调用
    check("存在 POST /api/mode", 'method: "POST"' in js)
    hits = js.count("postMode(")
    check("POST 只经单入口", "function postMode(" in js and hits == 2,
          f"定义+调用应共 2 处，实际 {hits}")
    check("入口挂在按钮点击", 'byId("btn-run").onclick = onRunClick' in js)
    # 初始化路径不得发请求（注释里的 /api/mode 字样不算）：只允许 applyI18n/render*/tick
    tail = js.rsplit("/* 初始化", 1)[-1]
    tail_code = re.sub(r"/\*[\s\S]*?\*/", "", tail)
    check("初始化不发 /api/mode", "postMode(" not in tail_code
          and '"/api/mode' not in tail_code, tail_code.strip()[:120])
    # ── 零点校准：三个控件（读取 / 一键调零 / 温漂补偿）+ 只读面板 + TEXT 高亮 ──
    check("三个控件按钮", all(f'id="{i}"' in html for i in
                          ("btn-read", "btn-zero", "btn-tc", "btn-tc-en")))
    check("控件入口挂点击", all(f'onclick = {fn}' in js for fn in
                          ("onReadClick", "onZeroClick", "onTempCalibClick", "onTempCompToggle")))
    check("命令只走 POST /api/imu/cmd", 'fetch("/api/imu/cmd", { method: "POST"' in js)
    check("命令按 func/args 组包", 'body = { func: func }' in js and 'body.args = args' in js)
    check("三个控件语义齐备", all(f'"{f}"' in js for f in
                          ("GET_BIAS", "GET_STATUS", "CALIBRATE", "TEMP_CALIB", "SET_FLAGS",
                           "GET_TC_STATE", "GET_DRIFT")))
    check("调零是异步等 ACK（校准中文案）", "cmd.calibrating" in js and "CALIBRATE" in js)
    check("TEMPCOMP_EN 位翻转", "TEMPCOMP_EN = 0x0001" in js and "res.data.flags ^ TEMPCOMP_EN" in js)
    check("繁忙时四个按钮置灰",
          '["btn-read", "btn-zero", "btn-tc", "btn-tc-en"].forEach' in js
          and 'disabled = busy ? true : false' in js)
    for key in ("cmd.running", "cmd.calibrating", "cmd.ok", "cmd.notConnected", "cmd.timeout",
                "cmd.ackError", "cmd.badArgs", "tc.sec"):
        n = js.count('"' + key + '"')
        check(f"命令文案词典键 {key}", n >= 2, f"count={n}")
    check("前端超时兵底（不永久置灰）", "CAL_TIMEOUT_MS" in js and "TC_MARGIN_MS" in js
          and "setTimeout(function" in js)
    check("零点校准面板", 'id="tbl-cal"' in html and 'id="tbl-state"' in html
          and 'id="v-temp"' in html and 'id="calib-hint"' in html)
    for axis in ("ax", "ay", "az", "gx", "gy", "gz"):
        for col in ("raw", "deb", "fbias", "ack", "dlt"):
            check(f"校准单元格 c-{col}-{axis}", f'id="c-{col}-{axis}"' in html)
    for fid in ("s-bias-valid", "s-tc-valid", "s-enabled", "s-tref", "s-kg", "s-applied"):
        check(f"状态单元格 {fid}", f'id="{fid}"' in html)
    check("面板 Δ = raw − deb", "raw[a.i] - deb[a.i]" in js and 'data-i18n="calib.row.delta"' in html)
    check("GET_BIAS 回执按列名取（gyro 在前）", "function ackBiasAt" in js and "bias[axis.k]" in js)
    check("TEXT 日志 + OK calib 高亮", 'id="textlog"' in html and 'indexOf("OK calib") === 0' in js
          and "txt-cal" in js)
    for key in ("btn.read", "btn.readTip", "btn.zero", "btn.zeroTip", "btn.tc", "btn.tcTip",
                "btn.tcEn", "btn.tcEnTip", "calib.title", "calib.sub", "calib.th.row",
                "calib.row.raw", "calib.row.deb", "calib.row.fbias", "calib.row.ack_bias",
                "calib.row.delta", "calib.temp", "calib.f.biasValid", "calib.f.tcValid",
                "calib.f.enabled", "calib.f.tempRef", "calib.f.kg", "calib.f.applied",
                "calib.on", "calib.off", "calib.hint.idle", "calib.hint.empty",
                "calib.hint.sim", "calib.hint.real", "textlog.title", "textlog.empty"):
        n = js.count('"' + key + '"')
        check(f"校准词典键成对 {key}", n >= 2, f"count={n}")
    tail_cal = re.sub(r"/\*[\s\S]*?\*/", "", js.rsplit("/* 初始化", 1)[-1])
    check("初始化不自动发命令", all(fn + "()" not in tail_cal for fn in
                              ("onReadClick", "onZeroClick", "onTempCalibClick",
                               "onTempCompToggle"))
          and '"/api/imu/cmd' not in tail_cal, tail_cal.strip()[:120])
    check("页面不做主机侧去偏", "session bias" not in js and "sim_bias" not in js)


def test_idle_server():
    with Server() as srv:
        _, snap = get_json(srv.url("/api/euler"))
        check("idle mode=none", snap["mode"] == "none", snap["mode"])
        check("idle connected=false", snap["connected"] is False)
        check("idle 无帧", snap["euler_frames"] == 0 and snap["last_deg"] is None)
        # 空闲起服务绝不能自己冒出数据（等 0.5s 再确认）
        time.sleep(0.5)
        _, snap2 = get_json(srv.url("/api/euler"))
        check("idle 不自启", snap2["euler_frames"] == 0 and snap2["mode"] == "none",
              f"frames={snap2['euler_frames']} mode={snap2['mode']}")


def test_sim_and_switch():
    with Server("--mode", "sim", "--lang", "en") as srv:
        _, a = get_json(srv.url("/api/euler"))
        time.sleep(0.4)
        _, b = get_json(srv.url("/api/euler"))
        check("sim 模式回报", b["mode"] == "sim" and b["connected"] is True,
              f"{b['mode']}/{b['connected']}")
        check("sim 帧增长", b["euler_frames"] > a["euler_frames"] > 0,
              f"{a['euler_frames']} -> {b['euler_frames']}")
        check("旧字段保留", {"source", "target", "connected", "frames", "euler_frames", "seq_jumps",
                            "bad_resync", "last_deg", "last_rad", "history", "texts"} <= set(b))
        check("sim 目标仍报配置目标", b["target"] == f"tcp:127.0.0.1:{DEFAULT_TCP_PORT}",
              b["target"])

        status, m = get_json(srv.url("/api/mode?value=none"))
        check("GET 切 none 回执", status == 200 and m == {"mode": "none", "connected": False}, m)
        _, off = get_json(srv.url("/api/euler"))
        check("none 停流", off["mode"] == "none" and off["connected"] is False,
              f"{off['mode']}/{off['connected']}")
        check("none 清易变显示态", off["last_deg"] is None and off["history"] == [],
              f"last={off['last_deg']} hist={len(off['history'])}")
        frozen = off["euler_frames"]
        time.sleep(0.4)
        _, still = get_json(srv.url("/api/euler"))
        check("none 不再增长", still["euler_frames"] == frozen,
              f"{frozen} -> {still['euler_frames']}")

        _, back = get_json(srv.url("/api/mode?value=sim"))
        check("GET 切回 sim", back["mode"] == "sim", back)
        time.sleep(0.4)
        _, again = get_json(srv.url("/api/euler"))
        check("切回后继续收帧", again["euler_frames"] > frozen,
              f"{frozen} -> {again['euler_frames']}")

        status, posted = get_json(srv.url("/api/mode"), method="POST", body={"mode": "none"})
        check("POST JSON 切 none", status == 200 and posted["mode"] == "none", posted)
        try:
            get_json(srv.url("/api/mode?value=bogus"), method="POST")
            check("非法值 400", False, "未抛 HTTPError")
        except urllib.error.HTTPError as e:
            payload = json.loads(e.read().decode("utf-8"))
            check("非法值 400", e.code == 400 and "error" in payload, f"{e.code} {payload}")
        _, after_bad = get_json(srv.url("/api/euler"))
        check("非法值不改模式", after_bad["mode"] == "none", after_bad["mode"])
    check("控制台如实打印切档", "data source switch" in srv.console,
          srv.console.strip().splitlines()[-1:])


def test_real_unready():
    with Server("--mode", "real", "--tcp", "127.0.0.1:5999") as srv:
        time.sleep(0.6)
        _, snap = get_json(srv.url("/api/euler"))
        check("real 未就绪不崩", snap["mode"] == "real" and snap["connected"] is False,
              f"{snap['mode']}/{snap['connected']}")
        check("real 不回落仿真", snap["euler_frames"] == 0, snap["euler_frames"])
        check("real 报 tcp 目标", snap["target"] == "tcp:127.0.0.1:5999", snap["target"])
    check("real 打印未就绪", "127.0.0.1:5999" in srv.console and "未就绪" in srv.console,
          srv.console.strip().splitlines()[:2])


def test_real_serial_target():
    """--serial 不存在的口：不崩、未连接、不回落仿真；目标串贯穿控制台 + /api/euler。"""
    with Server("--mode", "real", "--serial", "__no_such_port__") as srv:
        time.sleep(0.8)
        _, snap = get_json(srv.url("/api/euler"))
        check("serial 未就绪不崩", snap["mode"] == "real" and snap["connected"] is False,
              f"{snap['mode']}/{snap['connected']}")
        check("serial 不回落仿真", snap["euler_frames"] == 0, snap["euler_frames"])
        check("serial 报串口目标", snap["target"] == "serial:__no_such_port__@115200",
              snap["target"])
        # 带目标切 real：目标替换生效
        get_json(srv.url("/api/mode?value=real&target=serial:COM99%409600"), method="POST")
        _, switched = get_json(srv.url("/api/euler"))
        check("切目标生效", switched["target"] == "serial:COM99@9600", switched["target"])
        # 不带目标切回：沿用上一次目标
        get_json(srv.url("/api/mode?value=none"))
        get_json(srv.url("/api/mode?value=real"), method="POST")
        _, reused = get_json(srv.url("/api/euler"))
        check("无目标沿用上次", reused["target"] == "serial:COM99@9600", reused["target"])
        # 非法目标 → 400 且档位/目标不变
        try:
            get_json(srv.url("/api/mode?value=real&target=bogus"), method="POST")
            check("非法目标 400", False, "未抛 HTTPError")
        except urllib.error.HTTPError as e:
            payload = json.loads(e.read().decode("utf-8"))
            check("非法目标 400", e.code == 400 and "error" in payload, f"{e.code} {payload}")
        _, after_bad = get_json(srv.url("/api/euler"))
        check("非法目标不改目标", after_bad["target"] == "serial:COM99@9600"
              and after_bad["mode"] == "real", f"{after_bad['target']}/{after_bad['mode']}")
    lines = srv.console.strip().splitlines()
    check("serial 控制台打印目标", any("serial:__no_such_port__@115200" in ln for ln in lines),
          lines[:3])
    fail_lines = [ln for ln in lines if "pyserial" in ln
                  or (("打开串口" in ln or "cannot open serial port" in ln)
                      and "__no_such_port__" in ln)]
    check("serial 失败一行明确", bool(fail_lines), lines[:4])


def test_ports_api():
    """GET /api/ports：{"ports": [{"device", "desc"}]}；pyserial 缺失 = 空数组 + error。"""
    with Server() as srv:
        status, payload = get_json(srv.url("/api/ports"))
        check("ports 200", status == 200, status)
        check("ports 是列表", isinstance(payload.get("ports"), list), payload)
        if "error" in payload:
            check("ports 缺包时空数组", payload["ports"] == [] and payload["error"], payload)
        for item in payload.get("ports", []):
            check("ports 项含 device/desc", isinstance(item, dict) and "device" in item
                  and "desc" in item, item)
            break
        _, snap = get_json(srv.url("/api/euler"))
        check("ports 不改变数据源档位", snap["mode"] == "none", snap["mode"])


def floateq(v, tol=1e-4):
    """float32 往返容差比对（f32 存一遍再读出来不保证逐位相等）。"""
    class _Eq:
        def __eq__(self, other):
            return abs(float(other) - v) <= tol

    return _Eq()


def test_cmd_codec_unit(mod):
    """命令/回执编解码 + 解析：CMD 0x7D 字段序、ACK 0x7E 三头 + 各功能码数据块。"""
    frame = mod.encode_cmd_frame(7, mod.MOD_IMU, mod.FUNC_IMU["TEMP_CALIB"], b"\x1e")
    frames = mod.FrameDecoder().feed(frame)
    check("CMD 帧单帧解出", len(frames) == 1, frames)
    typ, seq, payload = frames[0] if frames else (None, None, b"")
    check("CMD 帧类型 0x7D", typ == mod.TYPE_CMD, typ)
    check("CMD 帧序号/字段序", seq == 7 and payload == bytes((mod.MOD_IMU, 0x03, 30)),
          f"{seq} {payload!r}")

    bias = (1.0, -0.5, 2.5, 0.02, -0.03, 0.015)          # imu_bias_t 口径：陀螺在前
    data = mod.parse_ack_data(mod.FUNC_IMU["GET_BIAS"], struct.pack("<6f", *bias))
    check("GET_BIAS 解析（gyro 在前）", data["bias"]["gx"] == 1.0 and data["bias"]["gz"] == 2.5
          and data["bias"]["az"] == floateq(0.015), data)
    status = (bytes((1, 0x07, 20, 0, 0, 0)) + struct.pack("<H", 0x000F)
              + bytes((1, 0, 20, 5)) + bytes(8))       # 20B：尾部保留字节也得在
    st = mod.parse_ack_data(mod.FUNC_IMU["GET_STATUS"], status)
    check("GET_STATUS 解析 offsets", st["rate_hz"] == 20 and st["flags"] == 0x000F
          and st["enabled"] == 1 and st["bias_valid"] == 1 and st["tc_valid"] == 0
          and st["kp"] == 2.0 and st["ki"] == 0.05, st)
    tc = (bytes((1, 0)) + struct.pack("<3f", 31.5, 25.0, 30.0)
          + struct.pack("<3f", 0.05, 0.05, 0.05) + struct.pack("<H", 7))
    tcs = mod.parse_ack_data(mod.FUNC_IMU["GET_TC_STATE"], tc)
    check("GET_TC_STATE 解析 offsets", tcs["tc_valid"] == 1 and tcs["enabled"] == 0
          and tcs["t_now"] == floateq(31.5) and tcs["t_ref"] == floateq(25.0)
          and tcs["apply_count"] == 7 and len(tcs["k_g"]) == 3, tcs)
    drift = mod.parse_ack_data(mod.FUNC_IMU["GET_DRIFT"],
                               struct.pack("<6f", 0.05, 0.05, 0.05, 0.0005, 0.0005, 0.0005))
    check("GET_DRIFT 解析（gyro/accel 分块）", drift["drift"]["gyro"][0] == floateq(0.05)
          and drift["drift"]["accel"][0] == floateq(0.0005), drift)
    check("未知功能码解析返空", mod.parse_ack_data(0x55, b"\x00" * 8) == {})
    check("短数据块不硬解", mod.parse_ack_data(mod.FUNC_IMU["GET_BIAS"], b"\x00" * 8) == {})

    # ACK 入账：SharedState 的只读渲染面 + 等回执机制（真采集线程与虚拟 MCU 共用此路）
    st_obj = mod.SharedState()
    st_obj.set_mode("real")
    ack = bytes((mod.MOD_IMU, mod.FUNC_IMU["GET_BIAS"], 0)) + struct.pack("<6f", *bias)
    st_obj.on_frame(mod.TYPE_ACK, 1, ack)
    snap = st_obj.snapshot()
    check("ACK 帧计数", snap["ack_frames"] == 1, snap["ack_frames"])
    check("ACK → imu.bias 渲染面", snap["imu"]["bias"] is not None
          and snap["imu"]["bias"]["gz"] == 2.5, snap["imu"].get("bias"))
    check("ACK → last_ack 摘要", snap["imu"]["last_ack"] == {"func": "GET_BIAS", "status": 0,
                                                             "name": "ok"},
          snap["imu"].get("last_ack"))
    base = st_obj.ack_count()
    check("wait_ack 不把旧回执当新回执",
          st_obj.wait_ack(mod.FUNC_IMU["GET_BIAS"], 0.2, base) is None)
    st_obj.on_frame(mod.TYPE_ACK, 2, ack)
    res = st_obj.wait_ack(mod.FUNC_IMU["GET_BIAS"], 0.2, base)
    check("wait_ack 拿到新回执", res is not None and res[0] == 0 and len(res[1]) == 24, res)


def test_virtual_mcu_unit(mod):
    """虚拟 MCU 直测：同步应答 / SET_FLAGS 只改开关 / CALIBRATE 异步在采集窗后才回。"""
    mcu = mod.VirtualMcu(collect_s=0.5, temp_start_c=25.0, temp_ramp_c_per_s=2.0)
    got = dict(mcu.submit(mod.FUNC_IMU["GET_BIAS"], b"", 0.0))
    ack = got[mod.TYPE_ACK]
    check("虚拟 MCU ACK 头 [MOD][FUNC][0]", ack[0:3] == bytes((mod.MOD_IMU, 0x02, 0)), ack[:3])
    check("虚拟 MCU 开机未标定 → 零偏全零", len(ack) == 3 + 24
          and ack[3:27] == bytes(24), ack[3:27])

    check("CALIBRATE 立刻不返 ACK", mcu.submit(mod.FUNC_IMU["CALIBRATE"], b"", 0.0) == [])
    check("采集窗内仍无回执", mcu.poll(0.4) == [])
    types = [typ for typ, _ in mcu.poll(0.6)]
    check("CALIBRATE 采集完回 ACK + 3 帧 0x02",
          types == [mod.TYPE_ACK] + [mod.TYPE_IMU_CAL] * mod.SIM_CALIB_FRAMES, types)
    ack = dict(mcu.submit(mod.FUNC_IMU["GET_BIAS"], b"", 0.7))[mod.TYPE_ACK]
    check("标定后零偏 = 预设注入零偏（重力已扣）",
          ack[3:15] == struct.pack("<3f", *mod.SIM_MCU_BIAS[0:3]), ack[3:15])
    check("标定后 bias_valid=1", mcu.bias_valid == 1, mcu.bias_valid)

    flags = mcu.sys_flags
    mcu.submit(mod.FUNC_IMU["SET_FLAGS"], struct.pack("<H", flags & ~mod.SYSF_TEMPCOMP_EN), 1.0)
    check("SET_FLAGS 关 TEMPCOMP_EN", not (mcu.sys_flags & mod.SYSF_TEMPCOMP_EN), mcu.sys_flags)
    check("SET_FLAGS 不动 bias_valid", mcu.bias_valid == 1, mcu.bias_valid)
    tc = dict(mcu.submit(mod.FUNC_IMU["GET_TC_STATE"], b"", 1.0))[mod.TYPE_ACK]
    check("GET_TC_STATE enabled 跟随开关位", tc[4] == 0, tc[:6])
    check("未知功能码回 0xFF", dict(mcu.submit(0x55, b"", 1.0))[mod.TYPE_ACK][2] == 0xFF)


def test_cmd_idle():
    """未启动数据源：GET/POST /api/imu/cmd 都回 notConnected（HTTP 200，不启源）。"""
    with Server() as srv:
        status, res = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_BIAS"})
        check("空闲命令回执 notConnected", status == 200 and res.get("code") == "notConnected",
              f"{status} {res}")
        status, res = get_json(srv.url("/api/imu/cmd?func=CALIBRATE"))
        check("GET 手测同一路径", status == 200 and res.get("code") == "notConnected", res)
        _, snap = get_json(srv.url("/api/euler"))
        check("空闲命令不启源", snap["mode"] == "none" and snap["euler_frames"] == 0,
              f"{snap['mode']}/{snap['euler_frames']}")


def test_cmd_sim(mod):
    """sim 档：虚拟 MCU 应答三个控件（读取 / 调零 / 温漂），面板走 /api/euler.imu。"""
    with Server("--mode", "sim", "--sim-calib-s", "0.4", "--sim-temp-ramp-c", "2.0") as srv:
        time.sleep(0.3)
        # ① 读取当前值：GET_BIAS + GET_STATUS
        status, bias = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_BIAS"})
        check("sim GET_BIAS ok", status == 200 and bias.get("code") == "ok", bias)
        check("sim 调零前零偏全零（未标定）",
              approx([bias["data"]["bias"][k] for k in ("gx", "gy", "gz", "ax", "ay", "az")],
                     (0.0,) * 6, 1e-6), bias.get("data"))
        status, st = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_STATUS"})
        check("sim GET_STATUS ok（未标定 bias_valid=0）",
              st.get("code") == "ok" and st["data"]["bias_valid"] == 0, st)
        check("sim GET_STATUS flags 带 TEMPCOMP_EN", st["data"]["flags"] & 1 == 1,
              st["data"]["flags"])

        # ② 一键调零：异步 ACK（采集窗 0.4s）+ 3 帧 0x02
        t0 = time.monotonic()
        status, zero = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "CALIBRATE"})
        elapsed = time.monotonic() - t0
        check("sim CALIBRATE ok", zero.get("code") == "ok", zero)
        check("sim CALIBRATE 等到了异步回执", elapsed >= 0.3, f"{elapsed:.2f}s")
        _, snap = get_json(srv.url("/api/euler"))
        check("sim 标定后 3 帧 0x02", snap["cal_frames"] >= mod.SIM_CALIB_FRAMES, snap["cal_frames"])
        check("sim 0x02 raw/deb/bias 齐全", snap["cal"] is not None
              and len(snap["cal"]["raw"]) == 6 and len(snap["cal"]["deb"]) == 6
              and len(snap["cal"]["bias"]) == 6, snap.get("cal"))
        check("sim 0x02 deb 是 MCU 已去偏值（az≈+1g）",
              abs(snap["cal"]["deb"][2] - 1.0) < 1e-3, snap["cal"]["deb"])
        status, bias2 = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_BIAS"})
        check("sim 调零后零偏 = 预设注入零偏（重力已扣）",
              approx([bias2["data"]["bias"][k] for k in ("gx", "gy", "gz")],
                     (1.0, -0.5, 2.5), 1e-4), bias2.get("data"))
        check("sim 面板 bias 来自 ACK", abs(snap["imu"]["bias"]["gx"]) < 1e-6
              or abs(snap["imu"]["bias"]["gx"] - 1.0) < 1e-4, snap["imu"].get("bias"))
        get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_STATUS"})
        _, snap2 = get_json(srv.url("/api/euler"))
        check("sim 标定后 bias_valid=1", snap2["imu"]["bias_valid"] == 1, snap2["imu"])

        # ③ 温漂补偿：TEMP_CALIB(2s) → 温差 2°C/s×2s = 4°C ≥ 3°C → tc_valid=1
        status, tc = get_json(srv.url("/api/imu/cmd"), method="POST",
                              body={"func": "TEMP_CALIB", "args": {"seconds": 2}})
        check("sim TEMP_CALIB ok", tc.get("code") == "ok", tc)
        status, tcs = get_json(srv.url("/api/imu/cmd"), method="POST",
                               body={"func": "GET_TC_STATE"})
        check("sim 温漂标定后 tc_valid=1", tcs["data"]["tc_valid"] == 1, tcs.get("data"))
        check("sim tc 状态带 k_g", len(tcs["data"]["k_g"]) == 3 and tcs["data"]["k_g"][0] > 0,
              tcs.get("data"))
        status, drift = get_json(srv.url("/api/imu/cmd"), method="POST",
                                 body={"func": "GET_DRIFT"})
        check("sim GET_DRIFT 回 6 系数", len(drift["data"]["drift"]["gyro"]) == 3
              and len(drift["data"]["drift"]["accel"]) == 3, drift.get("data"))

        # ④ 温漂开关：SET_FLAGS 只翻 TEMPCOMP_EN（不动 tc_valid）
        flags = st["data"]["flags"]
        if not flags & 1:
            flags |= 1
        status, sf = get_json(srv.url("/api/imu/cmd"), method="POST",
                              body={"func": "SET_FLAGS", "flags": flags & ~1})
        check("sim SET_FLAGS ok", sf.get("code") == "ok", sf)
        _, tcs2 = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_TC_STATE"})
        check("sim 开关关掉 → enabled=0 但 tc_valid 仍是 1",
              tcs2["data"]["enabled"] == 0 and tcs2["data"]["tc_valid"] == 1, tcs2.get("data"))

        # ⑤ 非法功能码 / 参数：主机侧拦下，不惊动 MCU
        status, bad = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "NOPE"})
        check("未知功能码回 badFunc", status == 200 and bad.get("code") == "badFunc", bad)
        status, bad2 = get_json(srv.url("/api/imu/cmd"), method="POST",
                                body={"func": "TEMP_CALIB", "args": {"seconds": 999}})
        check("参数越界回 badArgs", bad2.get("code") == "badArgs", bad2)


def test_cmd_real_unready():
    """real 档未连接：命令回 notConnected（不假装下发）。"""
    with Server("--mode", "real", "--tcp", "127.0.0.1:5999") as srv:
        time.sleep(0.6)
        status, res = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "CALIBRATE"})
        check("real 未连接命令 notConnected", status == 200 and res.get("code") == "notConnected",
              f"{status} {res}")
        _, snap = get_json(srv.url("/api/euler"))
        check("real 未连接不改渲染面", snap["imu"].get("bias") is None
              and snap["ack_frames"] == 0, snap["imu"])


def test_cmd_real_fake_mcu(mod):
    """假桥（真 MCU 替身）：CMD 帧字节上链路 → ACK 回执 → 面板；0x02 帧只读展示。"""
    mcu_bias = (1.0, -0.5, 2.5, 0.02, -0.03, 0.015)
    with FakeBridge(mod, mcu_bias) as dev, Server("--mode", "real",
                                                  "--tcp", f"127.0.0.1:{dev.port}") as srv:
        deadline, snap = time.monotonic() + 5.0, None
        while time.monotonic() < deadline:
            _, snap = get_json(srv.url("/api/euler"))
            if snap["connected"]:
                break
            time.sleep(0.05)
        check("假 MCU 链路就绪", snap is not None and snap["connected"] is True, snap)

        status, bias = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_BIAS"})
        check("real GET_BIAS ok", bias.get("code") == "ok", bias)
        check("real 回执零偏一致",
              approx([bias["data"]["bias"][k] for k in ("gx", "gy", "gz")],
                     mcu_bias[:3], 1e-4), bias.get("data"))
        check("下行 CMD 帧字段序", dev.received[0][0:2] == (mod.MOD_IMU, mod.FUNC_IMU["GET_BIAS"]),
              dev.received)

        status, zero = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "CALIBRATE"})
        check("real CALIBRATE 等到异步 ACK", zero.get("code") == "ok", zero)
        time.sleep(0.2)
        _, done = get_json(srv.url("/api/euler"))
        check("real 0x02 三帧进面板", done["cal_frames"] >= 3, done["cal_frames"])
        check("real deb 取固件块（主机不二次减偏）", approx(done["cal"]["deb"], CAL_DEB, 1e-4),
              done["cal"]["deb"])
        status, st = get_json(srv.url("/api/imu/cmd"), method="POST", body={"func": "GET_STATUS"})
        check("real GET_STATUS ok", st.get("code") == "ok", st)
        _, after = get_json(srv.url("/api/euler"))
        check("real 面板 bias_valid 来自 GET_STATUS", after["imu"]["bias_valid"] == 1,
              after["imu"].get("bias_valid"))


def main() -> int:
    if not SCRIPT.is_file() or not PAGE.is_file():
        print(f"== test_exp3_mode: 缺文件 {SCRIPT} / {PAGE} ==")
        return 1
    mod = load_module()
    test_resolve_mode(mod)
    test_split_addr(mod)
    test_target_parse(mod)
    test_page_contract()
    test_idle_server()
    test_sim_and_switch()
    test_real_unready()
    test_real_serial_target()
    test_ports_api()
    test_cmd_codec_unit(mod)
    test_virtual_mcu_unit(mod)
    test_cmd_idle()
    test_cmd_sim(mod)
    test_cmd_real_unready()
    test_cmd_real_fake_mcu(mod)
    print(f"== test_exp3_mode: 模式解析/目标串/页面契约/HTTP 显式切换/real 未就绪/串口目标/端口枚举/"
          f"MCU 校准命令（CMD·ACK 编解码、虚拟 MCU、三个控件链路、real 未连接）, "
          f"{len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    os.chdir(ROOT)
    raise SystemExit(main())
