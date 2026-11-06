#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""store_hub.py — store_demo 页数据源中枢（MOD_STORE=0x20 命令通道）。

与 FrameHub（IMU 模块）并列的第二个模块中枢：store_demo 固件（Embedded
examples/store_demo）只发 TEXT（上电 boot 行）+ 回 ACK（命令回执），**不发数据
帧流**——所以本中枢没有帧解码/轨迹重建，职责收敛为：
    · 模式（sim/real）+ 链路读循环（TEXT → 广播；ACK → 命令回执匹配）
    · store_cmd()：MOD_STORE 的 GET / SET / STATUS 命令（RESET 语义 = SET boot=0，
      由页面层表达；ERASE 介质级清空因 pstore 缺 API 未实现，回 UNKNOWN_FUNC）

帧/命令编码全部复用 hub.py 的通用件（encode_cmd 已内建 mod 参数 / decode_ack /
_TcpLink / _SerialLink / _open_link / FrameDecoder）——MOD_STORE 表的唯一对照点在本
文件头部常量区（与固件 proto/frame.h 的 PROTO_STORE_FUNC_* 对应）。

契约（GET ACK data 22B，小端）：
    boot u32@0 | gain f32@4 | flags u8@8 | note u8[8]@9..16 | load_state u8@17
    | frame_len u16@18..19 | dirty u8@20 | capacity_kb u8@21
契约（STATUS ACK data 6B）：
    load_state u8@0 | frame_len u16@1..2 | dirty u8@3 | entries u8@4 | capacity_kb u8@5
SET params = [count u8][(klen u8 key dlen u8 data)×count]，键白名单 boot/gain/flags/note。
"""
from typing import Callable, Dict, List, Optional, Tuple

import struct
import threading

from frame import FrameDecoder, TYPE_ACK, TYPE_TEXT   # noqa: F401  （TYPE_* 直接复用）
from hub import (CMD_TYPE, _TcpLink, _SerialLink, _open_link,
                 encode_cmd, decode_ack)

# ── MOD_STORE 命令表（唯一对照点；固件单源 = proto/frame.h PROTO_STORE_FUNC_*）──
MOD_STORE = 0x20
STORE_FUNC: Dict[str, int] = {
    "get": 0x01,       # ACK data 22B
    "set": 0x02,       # ACK data 空（落盘在固件侧完成）
    "status": 0x04,    # ACK data 6B
    # 0x03 ERASE 协议码存在但固件回 UNKNOWN_FUNC（pstore 缺 pstore_media_erase，
    # MEM-4 vendor 只读消费不可在消费仓补）——本档不注册，"恢复出厂"= SET boot=0。
}
STORE_KEYS = ("boot", "gain", "flags", "note")      # SET 键白名单
ACK_TIMEOUT_S = 5.0
ST_OK = 0
# 键 → 编码器（value 校验 + 字节）；note 定长 8
_BOOT_NONE = b""


def _enc_boot(v) -> bytes:
    n = int(v)
    if not 0 <= n <= 0xFFFFFFFF:
        raise ValueError(f"boot 须在 0~2^32-1，得 {v}")
    return struct.pack("<I", n)


def _enc_gain(v) -> bytes:
    return struct.pack("<f", float(v))


def _enc_flags(v) -> bytes:
    n = int(v)
    if not 0 <= n <= 0xFF:
        raise ValueError(f"flags 须在 0~255，得 {v}")
    return bytes((n,))


def _enc_note(v) -> bytes:
    if isinstance(v, str):
        raw = v.encode("utf-8", "replace")
    elif isinstance(v, (bytes, bytearray)):
        raw = bytes(v)
    else:
        raise ValueError("note 须为字符串或字节序列")
    if len(raw) > 8:
        raise ValueError(f"note 最长 8 字节（定长，无 NUL 约定），得 {len(raw)}")
    return raw.ljust(8, b"\x00")


_ENC = {"boot": _enc_boot, "gain": _enc_gain, "flags": _enc_flags, "note": _enc_note}
GET_BYTES = 22
STATUS_BYTES = 6


def _store_params(func_name: str, args: dict) -> bytes:
    """页面/API 参数 → SET params 字节（GET/STATUS 无参数返回 b''）。"""
    if func_name != "set":
        return b""
    kv = args.get("kv")
    if not isinstance(kv, dict) or not kv:
        raise ValueError("set 需要 kv 对象（如 {\"boot\": 0}）")
    if len(kv) > 1:
        raise ValueError("本版 set 一次只写一个键（上层按需组多键）")
    key, val = next(iter(kv.items()))
    if key not in _ENC:
        raise ValueError(f"set 键须为 {STORE_KEYS} 之一，得 {key!r}")
    body = _ENC[key](val)
    return bytes((1, len(key.encode("ascii")),)) + key.encode("ascii") + \
        bytes((len(body),)) + body


def _store_payload_json(func_name: str, data: bytes) -> dict:
    """ACK data → 页面渲染对象（布局见模块头契约）。"""
    if func_name == "get":
        if len(data) < GET_BYTES:
            raise ValueError(f"GET data {len(data)}B < {GET_BYTES}B")
        boot, = struct.unpack("<I", data[0:4])
        gain, = struct.unpack("<f", data[4:8])
        frame_len, = struct.unpack("<H", data[18:20])
        return {
            "boot": boot, "gain": round(gain, 6),
            "flags": data[8], "note": list(data[9:17]),
            "load_state": data[17], "frame_len": frame_len,
            "dirty": data[20], "capacity_kb": data[21],
        }
    if func_name == "status":
        if len(data) < STATUS_BYTES:
            raise ValueError(f"STATUS data {len(data)}B < {STATUS_BYTES}B")
        frame_len, = struct.unpack("<H", data[1:3])
        return {"load_state": data[0], "frame_len": frame_len,
                "dirty": data[3], "entries": data[4], "capacity_kb": data[5]}
    return {}


class _SimStore:
    """仿真档的**虚拟 store**（无板时命令通道也能跑；语义对齐固件 store_demo）。

    虚拟介质 = 内存镜像（open_sim 里做「上电 boot」：有镜像 → load_state=1 恢复；
    无 → load_state=0 出厂空白；boot_count 一律 +1 尾拍落盘——与固件
    store_demo_boot 同构）。断电续跑在仿真档的表达 = 再次 open_sim（模拟重新上电）。
    """

    def __init__(self) -> None:
        self._boot = 0               # 镜像条目（= 上次断电时 boot 值）
        self._gain = 0.0
        self._flags = 0
        self._note = bytes(8)
        self._load_state = 0
        self._dirty = 0
        self._capacity_kb = 4        # 固件 APP_STORE_CAPACITY 4096B
        self._mirror = False         # 是否有存档（模拟 flash 非空白）

    def boot_power_on(self, emit) -> None:
        """模拟上电：load（0=fresh / 1=restored）→ boot_count+1 → 尾拍落盘。"""
        if self._mirror:
            self._load_state = 1     # 有镜像 → restored
        else:
            self._load_state = 0     # 出厂空白 → fresh
        self._boot = (self._boot + 1) & 0xFFFFFFFF
        self._mirror = True
        self._dirty = 0
        emit("text", f"OK store boot: count={self._boot} "
                     f"load={self._load_state} len=70 kb={self._capacity_kb}")

    def handle(self, func_name: str, args: dict, emit) -> Tuple[int, bytes]:
        if func_name == "get":
            data = struct.pack("<If", self._boot, self._gain)
            data += bytes((self._flags,)) + self._note
            data += bytes((self._load_state,)) + struct.pack("<H", 70)
            data += bytes((self._dirty, self._capacity_kb))
            return ST_OK, data
        if func_name == "status":
            data = bytes((self._load_state,)) + struct.pack("<H", 70)
            data += bytes((self._dirty, 4, self._capacity_kb))
            return ST_OK, data
        if func_name == "set":
            kv = args.get("kv") or {}
            key, val = next(iter(kv.items()))
            if key == "boot":
                self._boot = int(val) & 0xFFFFFFFF
            elif key == "gain":
                self._gain = float(val)
            elif key == "flags":
                self._flags = int(val) & 0xFF
            elif key == "note":
                self._note = _enc_note(val)
            else:
                raise ValueError(f"未知键 {key!r}")
            self._dirty = 1
            emit("text", f"OK store set {key}")
            return ST_OK, b""
        raise ValueError(f"未知 func {func_name!r}")


class StoreHub:
    """store_demo 页中枢：同一时刻最多一路数据源；订阅者回调 text/port/state。

    与 FrameHub 的差异：无帧流/无轨迹重建；命令走 store_cmd()（MOD_STORE）。
    模式纪律同 FrameHub：open_sim/open_real 由页面显式触发，绝不自动启动；
    real 连不上抛 RuntimeError（服务器回 409），**任何情况不回落 sim**。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._listeners: List[Callable[[str, object], None]] = []
        self._stop = None
        self._thread = None
        self._link = None
        self._mcu: Optional[_SimStore] = None
        self._mode = "none"
        self._transport = "none"
        self._connected = False
        self._target = ""
        self._counters = {"text": 0}
        # 命令通道：一次在飞（_cmd_lock 串行化），ACK 按 FUNC 匹配
        self._cmd_lock = threading.Lock()
        self._cmd_seq = 0
        self._ack_wait: Dict[int, Tuple[threading.Event, dict]] = {}

    # ── 订阅（HubBroadcaster 契约：add_listener/state）──
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
            except Exception:
                pass

    def state(self) -> dict:
        return {"mode": self._mode, "source": self._transport,
                "connected": self._connected, "target": self._target,
                "counters": dict(self._counters)}

    # ── 启停（先停后起：单一源）──
    def open_sim(self) -> dict:
        self._halt()
        mcu = _SimStore()
        mcu.boot_power_on(self._emit)          # 模拟上电（boot 尾拍）
        self._mcu = mcu
        self._mode, self._transport = "sim", "sim"
        self._target = "sim"
        self._counters = {"text": 0}
        self._connected = True
        return self.state()

    def open_real(self, target: str) -> dict:
        self._halt()
        link = _open_link(target)              # 失败抛 RuntimeError → 409
        text = target.strip()
        self._link = link
        self._target = text
        self._transport = "tcp" if text.startswith("tcp:") else "serial"
        self._mode = "real"
        self._counters = {"text": 0}
        self._connected = True
        self._emit("port", True)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self.state()

    def close(self) -> dict:
        self._halt()
        return self.state()

    def _halt(self) -> None:
        stop = self._stop
        if stop is not None:
            stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(1.5)
        self._stop, self._thread = None, None
        if self._link is not None:
            self._link.close()
        self._link = None
        self._mcu = None
        self._mode, self._transport, self._target = "none", "none", ""
        for ev, _holder in list(self._ack_wait.values()):
            ev.set()
        self._ack_wait.clear()
        if self._connected:
            self._connected = False
            self._emit("port", False)

    # ── 命令通道（MOD_STORE：CMD → ACK）──
    def store_cmd(self, func_name: str, args: Optional[dict] = None) -> dict:
        kwargs = dict(args or {})
        func_name = str(func_name or "").strip().lower()
        if func_name not in STORE_FUNC:
            raise ValueError(f"未知 func {func_name!r}（可选 {sorted(STORE_FUNC)}）")
        params = _store_params(func_name, kwargs)
        func = STORE_FUNC[func_name]
        if self._link is not None:
            status, data = self._cmd_via_link(func, params)
        elif self._mode == "sim" and self._mcu is not None:
            with self._cmd_lock:
                status, data = self._mcu.handle(func_name, kwargs, self._emit)
        else:
            raise RuntimeError("未连接（先在仿真或真实模式启动数据源，再下发命令）")
        payload = _store_payload_json(func_name, data) if status == ST_OK else {}
        return {"func": func_name, "status": int(status), "payload": payload}

    def _cmd_via_link(self, func: int, params: bytes):
        with self._cmd_lock:
            waiter = (threading.Event(), {})
            self._ack_wait[func] = waiter
            ev, holder = waiter
            try:
                self._cmd_seq = (self._cmd_seq + 1) & 0xFF
                frame = encode_cmd(func, params, self._cmd_seq, mod=MOD_STORE)
                self._link.write(frame)
                if not ev.wait(ACK_TIMEOUT_S):
                    raise RuntimeError(
                        f"等 MCU 回执超时（{ACK_TIMEOUT_S:.0f}s，func={func_name_tag(func)}）")
                return holder["result"]
            finally:
                self._ack_wait.pop(func, None)

    def _on_ack(self, payload: bytes) -> None:
        try:
            _mod, func, status, data = decode_ack(payload)
        except ValueError as exc:
            self._emit("text", f"[store] 丢弃非法 ACK：{exc}")
            return
        if _mod != MOD_STORE:           # 非 MOD_STORE 回执不匹配命令（诊断帧）
            self._emit("text", f"[store] 收到非 MOD_STORE 回执 mod=0x{_mod:02X}，忽略")
            return
        waiter = self._ack_wait.get(func)
        if waiter is None:
            return
        ev, holder = waiter
        holder["result"] = (status, data)
        ev.set()

    def _loop(self) -> None:
        stop = self._stop
        dec = FrameDecoder()
        try:
            while not stop.is_set():
                chunk = self._link.read(512)
                if not chunk:
                    continue
                for typ, _seq, payload in dec.feed(chunk):
                    if typ == TYPE_TEXT:
                        self._counters["text"] += 1
                        self._emit("text", payload.decode("ascii", "replace"))
                    elif typ == TYPE_ACK:
                        self._on_ack(payload)
        except Exception as exc:
            self._emit("text", f"[store] 数据源异常：{exc}")
        finally:
            self._connected = False
            self._emit("port", False)


def func_name_tag(func: int) -> str:
    for name, code in STORE_FUNC.items():
        if code == func:
            return name
    return f"0x{func:02X}"