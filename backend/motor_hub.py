"""motor_hub.py — 电机实验数据源中枢（Qt-free，服务器版上位机的电机数据通路）。

职责：按页面注册选一档数据源（sim/real，单源）→ 用 scope_motor.FrameDecoder 解码
0x05 MOTOR / 0x06 SCEN / 0x7F TEXT → 按槽累积 Trace + 链路统计 → 广播订阅者
（服务器 SSE 侧只转发，不另建解码）。与 backend/hub.py（IMU 中枢）同构：订阅者回调
广播、先停后起、close 幂等、异常不拖垮采集线程；差别只在数据类型与断言口径——
断言一律走 scope_motor / scope_pidscan 的既有函数，阈值单源（本模块不复制任何阈值）。

事件契约（订阅者回调 fn(kind, payload)；HTTP/SSE 的 JSON 化在 server.py 的 _jsonable）：
    ("mframe", (type, seq, scen, timers, recs))
        recs = [(slot, kind, sp, duty, actual, phase, fault), ...]（16B 记录逐条解出）
        scen/timers 仅 0x06 场景帧有值；0x05（demo / pidscan）→ None
    ("text", {"line": "..."})   0x7F TEXT 行（逐行透传，含启动/汇总横幅）
    ("verdict", {...})          verify() 返回体；采集线程每 2s 或每 500 帧重算并广播
    ("port", bool)              连接状态变化（true=已连上，false=停止/异常收口）

断言口径（verify；items[].key 为稳定英文键，页面自行翻译，detail 供人读）：
    demo 页（motor_scope）  → scope_motor.check_traces（A~D，阈值 CHECKS）
    场景页（pwm/servo/enc/nested/dc）
                            → scope_motor.check_staircase / check_servo_points /
                              check_enc / check_nested / check_dc + SCEN_CHECKS 链路项
    pidscan 页              → scope_pidscan.group_metrics / parse_fw_line（TOL_PCT 单源）

采集窗口与展示流：
    每槽 Trace 只保留最近 TRACE_WINDOW 个样本（长跑不吃内存，断言跑在窗口内；
    summary.frames 是「自 open 以来的累计数据帧数」）。mframe 事件按
    MFRAME_MIN_INTERVAL_S 抽稀到 ≤50Hz —— 仿真档 1000fps 的全量事件会把 SSE
    队列灌满、把 verdict 挤掉；断言仍用全量采集，抽稀只影响绘图点密度。

模式纪律（与 hub.py 同）：open_sim / open_real 只由页面显式请求触发，open 前先停旧源
（单一源）；real 连接失败或场景激活握手失败 → 抛 RuntimeError（服务器回 409），
**任何情况都不自动回落 sim**。
"""

import struct
import threading
import time
from typing import Callable, List, Optional

import scope_motor as sm
import scope_pidscan as ps
import sim_motor
import sim_pidscan

KIND_MOTOR = "motor"          # 0x05 demo（motor_scope）/ 0x06 场景页
KIND_PIDSCAN = "pidscan"      # 0x05 单槽（组号）+ 0x7F TEXT 汇总表

TRACE_WINDOW = 30000          # 每槽保留的最近样本数（≈60s@500Hz 实机 / 30s@1000fps 仿真）
TRIM_SLACK = 4000             # 批量裁剪余量（摊薄 del 成本，避免每帧搬列表）
MFRAME_MIN_INTERVAL_S = 0.02  # mframe 展示流上限 50Hz（断言走全量采集）
VERDICT_INTERVAL_S = 2.0      # verdict 定时重算周期（s）
VERDICT_FRAMES = 500          # verdict 帧数触发（先到者为准）

ACTIVATE_APP_TIMEOUT_S = 20.0  # 等首帧 app 帧（ROM 启动期注入命令会卡死 boot——实测）
HANDSHAKE_TIMEOUT_S = 15.0     # SCEN 重发窗（scope_motor.run_scenario 口径）
SCEN_RESEND_S = 0.5            # SCEN 重发间隔（s）
SCEN_DRAIN_S = 0.6             # 激活后排干在途重复命令（重发即重启，瞬态不进采集）
FIXED_FALLBACK_S = 2.5         # 单实例固件软回退窗（持续 0x05 且无 0x06 → 固定场景采集）

PIDSCAN_SIM_FPS = 2000.0       # pidscan 仿真档墙钟压缩比（源按帧计数确定性推进）
PIDSCAN_SIM_ROUNDS = 200       # 仿真预生成轮数（一轮 27s 实验时长，够一局长跑）
PIDSCAN_FRAMES_PER_READ = 128  # 单次假串口读的帧数（分片粒度）
PIDSCAN_PAYLOAD_LEN = 16       # 单电机 0x05 记录长（scope_pidscan 口径）

# 场景 → 断言键（页面按 key 取词典；新增场景必须同时登记，缺一即 KeyError）
SCEN_ASSERTION_KEY = {
    "pwm": "pwm.staircase",
    "servo": "servo.calibration",
    "enc": "enc.settling",
    "nested": "nested.isolation",
    "dc": "dc.direction",
}


def _item(key: str, ok: bool, detail: str = "") -> dict:
    """verify 的一条断言项（key = 稳定英文键，detail = 数值证据）。"""
    return {"key": key, "ok": bool(ok), "detail": detail}


class _PidscanSimLink:
    """pidscan 仿真链路：把帧计数源按固定帧率推出（SimLink 的同职责替代品）。

    SimPidscanSource 只提供 read(max_frames)/close（没有 next_due_s），故本适配层
    负责「何时到期」：绝对截止调度（每批 frames_per_read 帧推进一格，线程被挂起
    后不追补），与 scope_motor.SimLink 对 SimMotorSource 的做法同构。recv 语义同
    SocketLink：bytes = 数据，b"" = 流尽；不建 socket、不碰串口。
    """

    def __init__(self, src: sim_pidscan.SimPidscanSource, fps: float,
                 frames_per_read: int) -> None:
        self._src = src
        self._fps = float(fps)
        self._batch = int(frames_per_read)
        self._deadline = time.monotonic()

    def recv(self, n: int) -> bytes:
        now = time.monotonic()
        wait = self._deadline - now
        if wait > 0:
            time.sleep(wait)
            now = time.monotonic()
        if self._deadline < now - 0.25:      # 线程被挂起后不追补（与 hub 的 sim 调度同）
            self._deadline = now
        chunk = self._src.read(self._batch)
        self._deadline += self._batch / self._fps
        return chunk

    def sendall(self, data: bytes) -> None:
        """合成源没有设备侧下行（页面命令在 sim 档回 noDownlink，见 MotorHub.send_line）。"""
        return None

    def settimeout(self, t: float) -> None:
        return None

    def close(self) -> None:
        self._src.close()


def _open_link(target: str):
    """target 文本 → 已连通的链路对象；失败抛 RuntimeError（服务器回 409）。

    形如 tcp:127.0.0.1:5560（QEMU 串口 TCP 桥，wait 模式：连接即开机）或
    serial:COM5@115200。连接预算沿用 scope_motor.CONNECT_TIMEOUT_S（≤5s 明报失败，
    不干等），读超时沿用 SOCKET/SERIAL_TIMEOUT_S（session 层按超时循环判截止）。
    """
    text = (target or "").strip()
    if text.startswith("tcp:"):
        host, _, port = text[4:].rpartition(":")
        host = host or "127.0.0.1"
        try:
            port_n = int(port)
        except ValueError:
            raise RuntimeError("tcp 目标格式应为 tcp:<host>:<port>，得 %r" % text)
        link = sm.connect_scope(host, port_n)
        if link is None:
            raise RuntimeError("连接 %s:%d 失败（%.1fs 内无可连的串口桥）"
                               % (host, port_n, sm.CONNECT_TIMEOUT_S))
        return link
    if text.startswith("serial:"):
        spec = text[len("serial:"):]
        port, _, baud = spec.partition("@")
        port = port.strip()
        if not port:
            raise RuntimeError("serial 目标格式应为 serial:<COMx>[@baud]，得 %r" % text)
        try:
            import serial
        except ImportError:
            raise RuntimeError("serial 目标需要 pyserial（pip install pyserial）")
        try:
            ser = serial.Serial(port, int(baud or sm.DEFAULT_BAUD), timeout=sm.SERIAL_TIMEOUT_S)
        except Exception as exc:                 # SerialException 及驱动层异常
            raise RuntimeError("串口 %s 打不开（%s）" % (port, exc))
        return sm.SerialLink(ser)
    raise RuntimeError("target 须为 tcp:<host>:<port> 或 serial:<COMx>[@baud]，得 %r" % text)


class MotorHub:
    """单源中枢：同一时刻最多一路电机数据源；订阅者按注册顺序收到回调。

    参数（来自 server.py 的页面注册表 MOTOR_PAGES）：
      kind     — "motor"（0x05 demo / 0x06 场景）或 "pidscan"（0x05 单槽 + TEXT 表）
      scenario — 场景名（scope_motor.SCEN 的键：pwm/servo/enc/nested/dc）；
                 None = demo（motor_scope）或 pidscan 页
      scen_id  — 场景号（与 SCEN[scenario]["id"] 必须一致，None 时按场景表取）
      fixed    — True = 真实档**不做 SCEN 握手**：单实例工程固件（exp_motor_pwm 等
                 五类）开机即按自己的场景出 0x05，不支持「SCEN <n>」切换；握手只会
                 白等 15s 后误报失败。断言仍按 scenario 的 check_* 口径走。
                 仿真档不受影响（合成源本来就用 SCEN 同号幂等下发）。
    """

    def __init__(self, kind: str = KIND_MOTOR, scenario: Optional[str] = None,
                 scen_id: Optional[int] = None, fixed: bool = False) -> None:
        if kind not in (KIND_MOTOR, KIND_PIDSCAN):
            raise ValueError(f"kind={kind!r} 须为 motor/pidscan")
        if kind == KIND_PIDSCAN:
            if scenario is not None:
                raise ValueError("pidscan 页不接受 scenario（数据面是 0x05 单槽 + TEXT 表）")
            self._scen_id = None
            self._slots: tuple = ()
        elif scenario is None:
            self._scen_id = None
            self._slots = tuple(range(sm.MOTOR_SLOTS))        # demo：0x05 四槽
        else:
            info = sm.SCEN.get(scenario)
            if info is None:
                raise ValueError(f"未知场景 {scenario!r}（可选 {sorted(sm.SCEN)}）")
            if scen_id is not None and int(scen_id) != info["id"]:
                raise ValueError(f"场景 {scenario} 的 scen_id 应为 {info['id']}，得 {scen_id}")
            self._scen_id = info["id"]
            self._slots = tuple(range(len(info["slots"])))
        self._kind = kind
        self._scenario = scenario
        self._fixed = bool(fixed)      # 真实档固定场景固件：SCEN 无响应时软回退
        self._fixed_run = False        # 本次运行是否走了软回退（诊断用）

        self._lock = threading.Lock()
        self._listeners: List[Callable[[str, object], None]] = []
        self._data_lock = threading.Lock()

        self._stop: Optional[threading.Event] = None
        self._thread: Optional[threading.Thread] = None
        self._link = None
        self._source = "none"         # none / sim / real
        self._connected = False
        self._target = ""
        self._last_mframe = 0.0
        self._bad_crc_seen = 0        # 已并入 stats 的 FrameDecoder.bad_crc 水位
        self._reset_data()

    # ── 订阅 ──
    def add_listener(self, fn: Callable[[str, object], None]) -> None:
        with self._lock:
            self._listeners.append(fn)

    def remove_listener(self, fn: Callable[[str, object], None]) -> None:
        with self._lock:
            try:
                self._listeners.remove(fn)
            except ValueError:
                pass

    def _emit(self, kind: str, payload) -> None:
        with self._lock:
            listeners = list(self._listeners)
        for fn in listeners:
            try:
                fn(kind, payload)
            except Exception:      # 单个订阅者异常不拖垮数据通路
                pass

    # ── 状态 ──
    def state(self) -> dict:
        with self._data_lock:
            stats = dict(self._stats)
            counters = dict(self._counters)
        return {
            "kind": self._kind,
            "scenario": self._scenario,
            "source": self._source,
            "target": self._target,
            "connected": self._connected,
            "frames": counters["motor"] + counters["scen"],
            "stats": stats,
            "counters": counters,
        }

    # ── 启停（先停后起：单一源，不抢资源）──
    def open_sim(self) -> dict:
        """仿真档：按页面注册建源（电机场景/演示 → SimMotorSource；pidscan → SimPidscanSource）。

        不碰网络、不碰串口；场景页直接在合成源上下行 "SCEN <id>"（同号幂等，无需握手）。
        """
        self._halt()
        if self._kind == KIND_PIDSCAN:
            link = _PidscanSimLink(
                sim_pidscan.SimPidscanSource(rounds=PIDSCAN_SIM_ROUNDS),
                PIDSCAN_SIM_FPS, PIDSCAN_FRAMES_PER_READ)
        else:
            src = sim_motor.SimMotorSource(sm.SCEN, sm.SLOT_KIND, sm.SG90, sm.sim_cfg())
            if self._scen_id is not None:
                src.command("SCEN %d" % self._scen_id)
            link = sm.SimLink(src)
        self._start_loop("sim", link, "sim")
        return self.state()

    def open_real(self, target: str) -> dict:
        """真实档：连 tcp/serial 目标；场景页连上后做 SCEN 激活握手。

        连接/握手失败抛 RuntimeError（服务器回 409），**不回落 sim**。
        握手在调用线程内完成（真机/QEMU 需要几百 ms 到几秒），随后才起采集线程——
        握手窗口内的帧按 scope_motor 口径只用于确认激活，不进采集窗口。
        """
        self._halt()
        link = _open_link(target)
        if self._scen_id is not None:
            try:
                self._handshake(link)
            except RuntimeError:
                self._close_quietly(link)
                raise
            except Exception as exc:             # 链路层异常同样按握手失败收口
                self._close_quietly(link)
                raise RuntimeError("激活握手失败：%s" % exc)
        self._start_loop("real", link, target.strip())
        return self.state()

    def close(self) -> dict:
        """停源（幂等）；累积结果保留供页面显示最后一屏。"""
        self._halt()
        return self.state()

    def send_line(self, line: str) -> dict:
        """host→device 明文行（SCEN 等）：仅 real 档有下行通道。"""
        if self._source != "real" or self._link is None:
            return {"code": "noDownlink"}
        try:
            self._link.sendall(line.encode("ascii"))
        except Exception as exc:
            return {"code": "notConnected", "detail": str(exc)}
        return {"code": "sent", "line": line.strip()}

    # ── 断言 ──
    def verify(self) -> dict:
        """对当前采集窗口做断言（POST /api/verify 与定时广播共用同一条路径）。"""
        if self._kind == KIND_PIDSCAN:
            with self._data_lock:
                rounds = [{g: list(rows) for g, rows in rnd.items()} for rnd in self._rounds]
                texts = [list(lines) for lines in self._text_rounds]
                stats = dict(self._stats)
                counters = dict(self._counters)
                round_done = self._round_done
            return self._verify_pidscan(rounds, texts, stats, counters, round_done)
        with self._data_lock:
            traces = [self._copy_trace(tr) for tr in self._traces]
            stats = dict(self._stats)
            counters = dict(self._counters)
            timers = list(self._base_timers) if self._base_timers else None
        if self._scen_id is None:
            return self._verify_demo(traces, stats, counters)
        return self._verify_scenario(traces, stats, counters, timers)

    def _verify_demo(self, traces, stats, counters) -> dict:
        ch = sm.CHECKS
        n = len(traces[0].sp) if traces else 0
        gap_pct = 100.0 * stats["seq_gaps"] / n if n else 100.0
        items = [
            _item("link.frames", n >= ch["min_frames"],
                  "frames=%d need=%d" % (n, ch["min_frames"])),
            _item("link.bad_crc", stats["bad_crc"] <= ch["max_bad_crc"],
                  "bad_crc=%d max=%d" % (stats["bad_crc"], ch["max_bad_crc"])),
            _item("link.seq_gaps", gap_pct <= ch["max_seq_gap_pct"],
                  "gaps=%.2f%% max=%.1f%%" % (gap_pct, ch["max_seq_gap_pct"])),
        ]
        fails = sm.check_traces(traces, stats) if n else []
        items.append(_item("scurve.shape", not fails,
                           "；".join(fails) if fails else "check_traces 全过"))
        return self._verdict(items, counters, stats)

    def _verify_scenario(self, traces, stats, counters, timers) -> dict:
        ch = sm.SCEN_CHECKS
        info = sm.SCEN[self._scenario]
        n = len(traces[0].sp) if traces else 0
        gap_pct = 100.0 * stats["seq_gaps"] / n if n else 100.0
        faults = sum(1 for tr in traces for f in tr.fa if f)
        items = [
            _item("link.frames", n >= ch["min_frames"],
                  "frames=%d need=%d" % (n, ch["min_frames"])),
            _item("link.bad_crc", stats["bad_crc"] <= ch["max_bad_crc"],
                  "bad_crc=%d max=%d" % (stats["bad_crc"], ch["max_bad_crc"])),
            _item("link.seq_gaps", gap_pct <= ch["max_seq_gap_pct"],
                  "gaps=%.2f%% max=%.1f%%" % (gap_pct, ch["max_seq_gap_pct"])),
            _item("link.fault", faults == 0, "fault_frames=%d" % faults),
            _item("link.layout", stats["layout_bad"] == 0 and stats["scen_bad"] == 0,
                  "layout_bad=%d scen_bad=%d" % (stats["layout_bad"], stats["scen_bad"])),
            _item("link.timers",
                  True if self._fixed else (timers is not None
                                            and sorted(timers) == sorted(info["timers"])),
                  "n/a（0x05 单实例固件不带定时器快照）" if self._fixed
                  else "got=%s exp=%s" % (timers, info["timers"])),
        ]
        key = SCEN_ASSERTION_KEY[self._scenario]
        if n == 0:
            items.append(_item(key, False, "frames=0（尚未收到本场景 0x06 帧）"))
        else:
            fails = self._scenario_fails(traces, timers)
            items.append(_item(key, not fails, "；".join(fails) if fails else "全断言通过"))
        return self._verdict(items, counters, stats)

    def _scenario_fails(self, traces, timers) -> list:
        """按场景调 scope_motor 的断言函数（口径与 scope_motor.run_scenario 一致）。"""
        info = sm.SCEN[self._scenario]
        if self._scenario == "pwm":
            fails, _ = sm.check_staircase(traces[0], info["slots"][0]["steps"], "pwm")
        elif self._scenario == "servo":
            freq = timers[0][2] if timers else 50
            fails, _ = sm.check_servo_points(traces[0], freq, "servo",
                                            info["slots"][0]["steps"])
        elif self._scenario == "enc":
            fails, _, _ = sm.check_enc(traces[0], self._frame_dt_s())
        elif self._scenario == "nested":
            freq50 = 50
            for timer in (timers or []):
                if timer[0] == 1:
                    freq50 = timer[2]
            fails, _, _, _ = sm.check_nested(traces[0], traces[1], traces[2], freq50)
        else:                                   # dc
            fails, _, _ = sm.check_dc(traces[0], "dc")
        return fails

    def _verify_pidscan(self, rounds, texts, stats, counters, round_done) -> dict:
        complete = [i for i, lines in enumerate(texts)
                    if any(ln.startswith("PIDSCAN round done") for ln in lines)]
        idx = complete[-1] if complete else len(rounds) - 1
        traces = rounds[idx] if rounds else {g: [] for g in range(ps.N_GROUPS)}
        lines = texts[idx] if texts else []
        items = []
        host = {}
        for g in range(ps.N_GROUPS):
            settle, ov, ss, ok = ps.group_metrics(traces.get(g, []))
            host[g] = (settle, ov, ss, bool(ok))
            items.append(_item("pidscan.g%d" % g, bool(ok),
                               "st=%.3fs ov=%.1f%% se=%.2f%%" % (settle, ov, ss)))
        items.append(_item("pidscan.round", bool(complete), "round_done=%d" % round_done))
        fw, stars = {}, []
        for line in lines:
            parsed = ps.parse_fw_line(line)
            if parsed is None:
                continue
            g, _kp, _ki, settle, ov, se, star = parsed
            fw[g] = (settle, ov, se)
            if star:
                stars.append(g)
        items.append(_item("pidscan.table", len(fw) == ps.N_GROUPS and len(stars) == 1,
                           "rows=%d best_marks=%s" % (len(fw), stars)))
        best = self._host_best(host)
        items.append(_item("pidscan.best", best >= 0 and best in stars,
                           "host_best=%d firmware_best=%s" % (best, stars)))
        return self._verdict(items, counters, stats)

    @staticmethod
    def _host_best(host: dict) -> int:
        """host 最优组（scope_pidscan 同式：settle + ov/100 + se/100 最小，只比收敛组）。"""
        ok_groups = [g for g, metrics in host.items() if metrics[3]]
        if not ok_groups:
            return -1
        return min(ok_groups,
                   key=lambda g: host[g][0] + host[g][1] / 100.0 + host[g][2] / 100.0)

    @staticmethod
    def _verdict(items, counters, stats) -> dict:
        summary = {"frames": counters["motor"] + counters["scen"],
                   "bad_crc": stats["bad_crc"],
                   "seq_gaps": stats["seq_gaps"],
                   "text": counters["text"]}
        return {"ok": True, "pass": all(i["ok"] for i in items),
                "items": items, "summary": summary}

    # ── 内部：生命周期 ──
    def _start_loop(self, source: str, link, target: str) -> None:
        self._reset_data()
        self._source, self._link, self._target = source, link, target
        self._stop = threading.Event()
        self._connected = True
        self._last_mframe = 0.0
        self._emit("port", True)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _halt(self) -> None:
        stop = self._stop
        if stop is not None:
            stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(1.5)
        self._stop = None
        self._thread = None
        link = self._link
        self._link = None
        if link is not None:
            self._close_quietly(link)
        self._source = "none"
        self._target = ""
        if self._connected:
            self._connected = False
            self._emit("port", False)

    @staticmethod
    def _close_quietly(link) -> None:
        try:
            link.close()
        except Exception:
            pass

    def _reset_data(self) -> None:
        """清空采集累积（open 时调用；stats/counters 归零，traces 重建）。"""
        with self._data_lock:
            self._stats = {"bad_crc": 0, "seq_gaps": 0, "text": 0, "layout_bad": 0,
                           "scen_bad": 0, "hdr_mismatch": 0, "demo05": 0}
            self._counters = {"motor": 0, "scen": 0, "text": 0}
            self._traces = [sm.Trace(i) for i in self._slots]
            self._prev_seq = None
            self._base_hdr = None
            self._base_timers = None
            self._t_first = time.monotonic()
            self._bad_crc_seen = 0
            self._rounds = [self._blank_round()]      # pidscan：最近 ≤2 轮
            self._text_rounds = [[]]
            self._last_g = -1
            self._round_done = 0

    @staticmethod
    def _blank_round() -> dict:
        """pidscan 一轮的 0x05 桶（组号 → 按序 (sp, duty, actual, phase)）。"""
        return {g: [] for g in range(ps.N_GROUPS)}

    # ── 内部：采集线程 ──
    def _loop(self) -> None:
        stop = self._stop
        dec = sm.FrameDecoder()
        last_verdict = time.monotonic()
        since_verdict = 0
        try:
            while not stop.is_set():
                frames, eof = sm.read_frames(self._link, dec)
                for typ, seq, payload in frames:
                    try:
                        self._dispatch(typ, seq, payload)
                    except Exception:
                        # 单帧异常按坏帧计，不中断采集线程（订阅者异常见 _emit）
                        with self._data_lock:
                            self._stats["layout_bad"] += 1
                    since_verdict += 1
                if dec.bad_crc != self._bad_crc_seen:
                    with self._data_lock:
                        self._stats["bad_crc"] += dec.bad_crc - self._bad_crc_seen
                    self._bad_crc_seen = dec.bad_crc
                now = time.monotonic()
                if since_verdict >= VERDICT_FRAMES or now - last_verdict >= VERDICT_INTERVAL_S:
                    try:
                        self._emit("verdict", self.verify())
                    except Exception as exc:      # 断言异常不能拖垮采集
                        self._emit("text", {"line": "[hub] verdict 计算失败：%s" % exc})
                    last_verdict, since_verdict = now, 0
                if eof:
                    self._emit("text", {"line": "[hub] 数据源已流尽（重新启动可继续）"})
                    break
        except Exception as exc:                  # 串口拔出/占用/对端被杀
            self._emit("text", {"line": "[hub] 数据源异常：%s" % exc})
        finally:
            self._connected = False
            self._emit("port", False)

    def _dispatch(self, typ: int, seq: int, payload: bytes) -> None:
        if typ == sm.TYPE_TEXT:
            line = payload.decode("ascii", "replace")
            with self._data_lock:
                self._stats["text"] += 1
                self._counters["text"] += 1
                self._prev_seq = None            # TEXT 与遥测混在同一线序：不参与缺口判定
                if self._kind == KIND_PIDSCAN:
                    self._on_pidscan_text(line)
            self._emit("text", {"line": line})
            return
        if self._kind == KIND_PIDSCAN:
            self._dispatch_pidscan(typ, seq, payload)
        elif self._scen_id is None:
            self._dispatch_demo(typ, seq, payload)
        else:
            self._dispatch_scen(typ, seq, payload)

    def _dispatch_demo(self, typ: int, seq: int, payload: bytes) -> None:
        """demo 档 0x05 四槽（layout 错位按坏帧计，与 scope_motor.run_demo 同口径）。"""
        if typ != sm.TYPE_MOTOR:
            return
        if len(payload) != sm.MOTOR_PAYLOAD_LEN:
            self._with_stats("layout_bad")
            return
        recs = []
        with self._data_lock:
            self._count_seq(seq)
            self._counters["motor"] += 1
            for r in range(sm.MOTOR_SLOTS):
                slot, kind, sp, du, ac, ph, fa = sm.MOTOR_REC.unpack_from(
                    payload, r * sm.MOTOR_REC.size)
                if slot != r or kind != sm.SLOT_KIND[r]:
                    self._stats["layout_bad"] += 1
                    self._stats["bad_crc"] += 1
                    continue
                self._traces[r].add(sp, du, ac, ph, fa)
                self._trim(self._traces[r])
                recs.append((slot, kind, sp, du, ac, ph, fa))
        self._emit_mframe(typ, seq, None, None, recs)

    def _dispatch_scen(self, typ: int, seq: int, payload: bytes) -> None:
        """场景档 0x06（header/layout/定时器快照校验与 scope_motor.run_scenario 同口径）。"""
        if typ == sm.TYPE_MOTOR:
            # 单实例工程固件（fixed 页）：0x05 本身就是本场景数据（16B × n 槽），
            # 按场景口径累积进轨迹；长度不符/非本场景 → 按残余 demo 帧只计数。
            if self._fixed and len(payload) == sm.MOTOR_REC.size * len(self._traces):
                self._accumulate_fixed(seq, payload)
            else:
                self._with_stats("demo05")
            return
        if typ != sm.TYPE_SCEN:
            return
        parsed = sm.parse_scen_payload(payload)
        if parsed is None:
            self._with_stats("layout_bad")
            return
        scen, timers, n_rec, rec_off = parsed
        if scen != self._scen_id or n_rec != len(self._traces):
            self._with_stats("scen_bad")
            return
        hdr = bytes(payload[:rec_off])
        recs = []
        with self._data_lock:
            if self._base_hdr is None:
                self._base_hdr = hdr
                self._base_timers = timers
            elif hdr != self._base_hdr:
                self._stats["hdr_mismatch"] += 1
            self._count_seq(seq)
            self._counters["scen"] += 1
            slots = sm.SCEN[self._scenario]["slots"]
            for r in range(n_rec):
                slot, kind, sp, du, ac, ph, fa = sm.MOTOR_REC.unpack_from(
                    payload, rec_off + r * sm.MOTOR_REC.size)
                if slot != r or kind != slots[r]["kind"]:
                    self._stats["layout_bad"] += 1
                    continue
                self._traces[r].add(sp, du, ac, ph, fa)
                self._trim(self._traces[r])
                recs.append((slot, kind, sp, du, ac, ph, fa))
        self._emit_mframe(typ, seq, scen, timers, recs)

    def _accumulate_fixed(self, seq: int, payload: bytes) -> None:
        """fixed 固件（单实例工程）的 0x05 场景记录累积 + 广播。

        0x05 不带定时器快照（那是 0x06 专属），故 mframe 的 scen/timers 置 None；
        记录布局/槽序/kind 校验与 0x06 分支同口径，断言走 scenario 的 check_*。
        """
        slots = sm.SCEN[self._scenario]["slots"]
        recs = []
        with self._data_lock:
            self._count_seq(seq)
            self._counters["scen"] += 1
            for r in range(len(self._traces)):
                slot, kind, sp, du, ac, ph, fa = sm.MOTOR_REC.unpack_from(
                    payload, r * sm.MOTOR_REC.size)
                if slot != r or kind != slots[r]["kind"]:
                    self._stats["layout_bad"] += 1
                    continue
                self._traces[r].add(sp, du, ac, ph, fa)
                self._trim(self._traces[r])
                recs.append((slot, kind, sp, du, ac, ph, fa))
        self._emit_mframe(sm.TYPE_MOTOR, seq, None, None, recs)

    def _dispatch_pidscan(self, typ: int, seq: int, payload: bytes) -> None:
        """pidscan 档 0x05 单槽：slot = 扫描组号，组号回卷 = 新一轮（不跨轮拼桶）。"""
        if typ != sm.TYPE_MOTOR:
            return
        if len(payload) != PIDSCAN_PAYLOAD_LEN:
            self._with_stats("layout_bad")
            return
        slot, kind = payload[0], payload[1]
        sp, du, ac = struct.unpack("<3f", payload[2:14])
        ph, fa = payload[14], payload[15]
        with self._data_lock:
            self._count_seq(seq)
            self._counters["motor"] += 1
            if slot >= ps.N_GROUPS:
                self._stats["layout_bad"] += 1
                return                        # 组号越界 = 非本实验帧
            if slot < self._last_g:
                self._next_round()            # 组号回卷 → 新一轮
            self._last_g = slot
            if fa == 0:                       # 故障帧不进轨迹（与 Collector 同口径）
                self._rounds[-1][slot].append((sp, du, ac, ph))
        self._emit_mframe(typ, seq, None, None, [(slot, kind, sp, du, ac, ph, fa)])

    def _on_pidscan_text(self, line: str) -> None:
        """TEXT 归轮 + round done 收轮（scope_pidscan.Collector 同序，调用方持锁）。"""
        self._text_rounds[-1].append(line)
        if line.startswith("PIDSCAN round done"):
            self._round_done += 1
            self._next_round()

    def _next_round(self) -> None:
        """开新轮（0x05 桶与 TEXT 桶同开）；只保留最近 2 轮（长跑不吃内存）。"""
        self._rounds.append(self._blank_round())
        self._text_rounds.append([])
        self._last_g = -1
        while len(self._rounds) > 2:
            self._rounds.pop(0)
            self._text_rounds.pop(0)

    def _with_stats(self, key: str) -> None:
        with self._data_lock:
            self._stats[key] += 1

    def _count_seq(self, seq: int) -> None:
        """数据帧 seq 缺口计数（TEXT 不参与：固件 TEXT 可能与遥测不同计数器）。"""
        prev = self._prev_seq
        if prev is not None and (seq - prev - 1) % 256 != 0:
            self._stats["seq_gaps"] += 1
        self._prev_seq = seq

    @staticmethod
    def _trim(trace) -> None:
        """窗口裁剪：超出 TRACE_WINDOW 就整批丢最旧样本（摊薄 del 成本）。"""
        over = len(trace.sp) - TRACE_WINDOW
        if over < TRIM_SLACK:
            return
        for series in (trace.sp, trace.du, trace.ac, trace.ph, trace.fa):
            del series[:over]

    def _emit_mframe(self, typ: int, seq: int, scen, timers, recs) -> None:
        """展示流抽稀（≤1/MFRAME_MIN_INTERVAL_S）：断言走全量采集，事件只服务绘图。"""
        now = time.monotonic()
        if now - self._last_mframe < MFRAME_MIN_INTERVAL_S:
            return
        self._last_mframe = now
        self._emit("mframe", (typ, seq, scen, timers, recs))

    def _frame_dt_s(self) -> float:
        """每帧时间步长（enc 的 rise 统计用）：sim 取源声明步长，real 用墙钟均值。"""
        dt = getattr(self._link, "frame_dt_s", None)
        if dt:
            return float(dt)
        with self._data_lock:
            n = self._counters["motor"] + self._counters["scen"]
        wall = time.monotonic() - self._t_first
        return wall / n if n else 0.0

    @staticmethod
    def _copy_trace(trace):
        """Trace 快照（列表浅拷贝）：断言在锁外跑，采集线程继续追加。"""
        snap = sm.Trace(trace.slot)
        snap.sp = list(trace.sp)
        snap.du = list(trace.du)
        snap.ac = list(trace.ac)
        snap.ph = list(trace.ph)
        snap.fa = list(trace.fa)
        return snap

    # ── 内部：激活握手（scope_motor.run_scenario 口径）──
    def _handshake(self, link) -> None:
        """场景激活：等 app 帧 → 0.5s 重发 "SCEN <id>" 直到 0x06 scen_id 匹配。

        ROM 启动期注入命令会卡死 boot（实测），故先等首帧 app 帧再注入；握手窗口内
        收到的 TEXT 行照常广播给页面（用户能看到启动横幅）。失败抛 RuntimeError。
        """
        link.settimeout(sm.SOCKET_TIMEOUT_S)
        dec = sm.FrameDecoder()
        app_types = (sm.TYPE_MOTOR, sm.TYPE_SCEN)
        start = time.monotonic()
        booted = False
        while not booted and time.monotonic() - start < ACTIVATE_APP_TIMEOUT_S:
            frames, eof = sm.read_frames(link, dec)
            if eof:
                break
            for typ, _seq, payload in frames:
                if typ == sm.TYPE_TEXT:
                    self._emit("text", {"line": payload.decode("ascii", "replace")})
                elif typ in app_types:
                    booted = True
        if not booted:
            raise RuntimeError("%.0fs 内未见任何 app 帧（固件未启动或链路不通）"
                               % ACTIVATE_APP_TIMEOUT_S)

        start = time.monotonic()
        last_send = 0.0
        activated = False
        first_motor = None            # 首次收到 0x05 的时刻（软回退判据）
        while not activated and time.monotonic() - start < HANDSHAKE_TIMEOUT_S:
            now = time.monotonic()
            if now - last_send >= SCEN_RESEND_S:
                link.sendall(b"SCEN %d\n" % self._scen_id)
                last_send = now
            frames, eof = sm.read_frames(link, dec)
            if eof:
                break
            for typ, _seq, payload in frames:
                if typ == sm.TYPE_TEXT:
                    self._emit("text", {"line": payload.decode("ascii", "replace")})
                elif typ == sm.TYPE_SCEN:
                    parsed = sm.parse_scen_payload(payload)
                    if parsed is not None and parsed[0] == self._scen_id:
                        activated = True
                elif typ == sm.TYPE_MOTOR and first_motor is None:
                    first_motor = time.monotonic()
            # 软回退（fixed 页 = 单实例工程固件）：该族固件开机即按自己的场景出
            # 0x05、**不支持**「SCEN <n>」切换；持续收到 0x05 且 2.5s 内等不到
            # 0x06 就按固定场景直接采集（不白等 15s 误报失败，也不回落 sim）。
            if (not activated and self._fixed and first_motor is not None
                    and time.monotonic() - first_motor >= FIXED_FALLBACK_S):
                self._fixed_run = True
                self._emit("text", {"line": "[hub] %s 固件未响应 SCEN 切换（单实例工程），"
                                            "按固定场景采集" % self._scenario})
                return
        if not activated:
            raise RuntimeError("%.0fs 内未收到 scen_id=%d 的 0x06 帧（命令注入/切换失败）"
                               % (HANDSHAKE_TIMEOUT_S, self._scen_id))

        drain_until = time.monotonic() + SCEN_DRAIN_S      # 重发即场景重启：排干瞬态
        while time.monotonic() < drain_until:
            _frames, eof = sm.read_frames(link, dec)
            if eof:
                break
