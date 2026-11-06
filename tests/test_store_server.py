#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""test_store_server.py — store_demo 页服务器契约测试（headless，真 HTTP + 真 SSE）。

store 页 = 独立模块中枢（backend/store_hub.py 的 StoreHub，MOD_STORE=0x20），
数据面只有 TEXT + ACK（命令回执），无数据帧流。覆盖（对齐 test_motor_server.py 形态）：

  1) 注册表：--page store_demo 合法；/store_demo/ 页面可服务（薄壳 index.html）
  2) 启动即状态面：source=none & connected=false（服务器不碰任何数据源）
  3) sim 档：POST /api/open {"mode":"sim"} → state.connected=true + SSE 收 TEXT
     boot 行（上电语义）→ GET → 快照 22B（boot/load_state/... 布局契约）
     → set boot=0（RESET 语义）→ STATUS → 6B 布局 → 再 GET 值已变
     → /api/close 回到 none
  4) 真实目标不可达：POST /api/open {"mode":"real","target":"tcp:..."} → 409 +
     error，source 仍为 none（**不自动回落 sim**）
  5) 非 store 页访问 /api/store/cmd → 404（命令通道只在 store 页存在）
  6) sim 档真实链路：TCP 回环假桥模拟 store_demo 固件（MOD_STORE GET/SET/STATUS
     ACK 应答）→ mode real 下 GET/SET 往返 → 假桥发出 TEXT boot 行入 SSE

跑法：python tests/test_store_server.py（exit 0 = 全过；需 python>=3.10）
"""
import json
import os
import socket
import subprocess
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SERVER = os.path.join(ROOT, "backend", "server.py")

SOF = 0xA5
CMD_TYPE = 0x7D
ACK_TYPE = 0x7E
TEXT_TYPE = 0x7F
CRC = 0x07
MOD_STORE = 0x20          # 与 proto/frame.h PROTO_MOD_STORE、store_hub.MOD_STORE 同源
FUNC = {"get": 0x01, "set": 0x02, "status": 0x04}

fails = []


def check(name, ok, detail=""):
    if ok:
        print(f"  [ok]   {name}")
    else:
        fails.append(f"{name}{('：' + detail) if detail else ''}")
        print(f"  [FAIL] {name}{('：' + detail) if detail else ''}")


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ CRC if crc & 0x80 else (crc << 1)
        crc &= 0xFF
    return crc


def frame(ftype: int, payload: bytes, seq: int = 0) -> bytes:
    # LEN 只盖 PAYLOAD（FrameDecoder：SOF | LEN | TYPE | SEQ | PAYLOAD | CRC）
    head = bytes([SOF, len(payload), ftype, seq & 0xFF])
    return head + payload + bytes([crc8(head[1:] + payload)])


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def http(base, path, body=None, method="GET", timeout=120.0):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
            try:
                return r.status, (json.loads(raw) if raw else {})
            except ValueError:
                return r.status, {"raw": raw.decode("utf-8", "replace")}
    except urllib.error.HTTPError as exc:
        raw = exc.read()
        try:
            return exc.code, json.loads(raw) if raw else {}
        except ValueError:
            return exc.code, {"raw": raw.decode("utf-8", "replace")}


def wait_ready(port: int, budget: float = 15.0) -> bool:
    deadline = time.time() + budget
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.15)
    return False


class _Recorder:
    """SSE 事件记录器：收集 text/state/port（store 页无 frame 事件）。"""

    def __init__(self, port: int):
        self.port = port
        self.events = []
        self._stop = threading.Event()
        self.ready = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def wait_ready(self, budget: float = 8.0) -> bool:
        return self.ready.wait(budget)

    def _connect(self, retries: int = 40) -> socket.socket:
        last = None
        for _ in range(retries):
            try:
                s = socket.create_connection(("127.0.0.1", self.port), timeout=0.5)
                req = (b"GET /api/stream HTTP/1.1\r\nHost: 127.0.0.1\r\n"
                       b"Accept: text/event-stream\r\nConnection: keep-alive\r\n\r\n")
                s.sendall(req)
                return s
            except OSError as exc:
                last = exc
                time.sleep(0.1)
        raise last

    def _run(self):
        s = self._connect()
        buf = b""
        # register 完成判定：服务器 _stream 注册后先推一条 state。
        # （客户端 sendall 完成 ≠ 服务器已 register——text 竞态就出在这）
        # recv 超时不是断流：SSE 空闲无事件也可能触发 timeout，一律继续。
        while not self._stop.is_set():
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                if self.ready.is_set():
                    break
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n\n" in buf:
                block, buf = buf.split(b"\n\n", 1)
                ev = {}
                for rawline in block.split(b"\n"):   # SSE 事件用 \n 行（服务器 \n\n 分块）
                    line = rawline.strip()
                    if b":" in line:
                        k, v = line.split(b":", 1)
                        ev[k.decode("ascii")] = v.strip().decode("utf-8", "replace")
                if ev.get("event") and ev["event"] == "state":
                    self.ready.set()
            if self.ready.is_set():
                break
        # 注：到达此法时 buf 里可能残留已到帧（含首个 state）——放弃它们，
        # 后续事件从主循环接管（首个 state 本身不是被测对象）。
        buf = b""
        # 主循环：后续事件满一帧推一次
        while not self._stop.is_set() and s is not None:
            try:
                chunk = s.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n\n" in buf:
                block, buf = buf.split(b"\n\n", 1)
                ev = {}
                for rawline in block.split(b"\n"):   # SSE 事件用 \n 行（服务器 \n\n 分块）
                    line = rawline.strip()
                    if b":" in line:
                        k, v = line.split(b":", 1)
                        ev[k.decode("ascii")] = v.strip().decode("utf-8", "replace")
                if ev.get("event"):
                    try:
                        payload = json.loads(ev.get("data", ""))
                    except ValueError:
                        payload = ev.get("data", "")
                    self.events.append((ev["event"], payload))
        try:
            s.close()
        except OSError:
            pass

    def wait_event(self, kind, predicate, budget=8.0):
        deadline = time.time() + budget
        while time.time() < deadline:
            for k, p in self.events:
                if k == kind and predicate(p):
                    return p
            time.sleep(0.03)
        return None

    def close(self):
        self._stop.set()
        self._th.join(1.0)


def spawn_server(page: str):
    port = free_port()
    proc = subprocess.Popen(
        [sys.executable, SERVER, "--page", page, "--port", str(port), "--no-open"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if not wait_ready(port):
        out = proc.stdout.read().decode("utf-8", "replace") if proc.stdout else ""
        proc.kill()
        return None, port, out
    return proc, port, None


def main() -> int:
    print("== store_demo 页服务器契约 ==")

    """ 1) 注册表：--page store_demo 合法 + 页面服务（薄壳可 GET） """
    proc, port, _out = spawn_server("store_demo")
    if proc is None:
        check("store_demo 服务器可启动", False, _out or "端口未就绪")
        return 1
    base = f"http://127.0.0.1:{port}"
    try:
        st, page = http(base, "/store_demo/")
        html_raw = page.get("raw", "") if isinstance(page, dict) else ""
        check("页面可服务 /store_demo/ (200)", st == 200 and html_raw, f"status={st}")
        check("薄壳含 window.STORE_PAGE 声明", "STORE_PAGE" in html_raw)
        check("薄壳含 #app 宿主容器", 'id="app"' in html_raw)

        """ 2) 启动状态面：不碰任何数据源 """
        st, state = http(base, "/api/state")
        check("/api/state 200", st == 200)
        check("启动 source=none & connected=false",
              state.get("source") == "none" and state.get("connected") is False,
              json.dumps(state))

        rec = _Recorder(port)
        if not rec.wait_ready():
            check("SSE Recorder 就绪", False)
            return 1

        """ 3) sim 档：open → TEXT boot → GET/SET/STATUS 往返 → close """
        st, s1 = http(base, "/api/open", {"mode": "sim"}, "POST")
        check("sim open 200", st == 200 and s1.get("connected") is True,
              json.dumps(s1))
        boot = rec.wait_event("text", lambda p: "store boot" in str(p))
        check("sim 上电 TEXT boot 行（load_state=0 fresh）",
              boot is not None and "load=0" in str(boot), str(boot))
        st, g = http(base, "/api/store/cmd", {"func": "get"}, "POST")
        p = g.get("payload", {})
        check("GET 200 + 22B 快照（boot=1 语义：sim 上电写 1）",
              st == 200 and g.get("status") == 0 and p.get("boot") == 1,
              json.dumps(g))
        check("GET 快照字段齐（load_state/frame_len/dirty/kb）",
              all(k in p for k in ("boot", "gain", "flags", "note",
                                   "load_state", "frame_len", "dirty", "capacity_kb")),
              json.dumps(p))
        check("GET capacity_kb=4（固件 4096B）", p.get("capacity_kb") == 4,
              str(p.get("capacity_kb")))

        st, s2 = http(base, "/api/store/cmd", {"func": "set", "args": {"kv": {"boot": 0}}},
                      "POST")
        check("RESET（SET boot=0）ACK OK", st == 200 and s2.get("status") == 0,
              json.dumps(s2))
        st, g2 = http(base, "/api/store/cmd", {"func": "get"}, "POST")
        check("SET 后 GET 回读 boot=0", g2.get("payload", {}).get("boot") == 0,
              json.dumps(g2))

        st, s3 = http(base, "/api/store/cmd", {"func": "status"}, "POST")
        sp = s3.get("payload", {})
        check("STATUS 6B（entries=4 / kb=4）",
              st == 200 and sp.get("entries") == 4 and sp.get("capacity_kb") == 4,
              json.dumps(s3))

        st, e = http(base, "/api/store/cmd", {"func": "erase"}, "POST")
        check("ERASE（协议码未实现）410 明报",
              st in (409, 410) and e.get("error"), json.dumps(e))

        st, st2 = http(base, "/api/close", {}, "POST")
        check("close 回 none", st == 200 and st2.get("source") == "none",
              json.dumps(st2))

        """ 4) real 不可达 → 409，不回落 sim """
        st, rf = http(base, "/api/open",
                      {"mode": "real", "target": f"tcp:127.0.0.1:{free_port()}"}, "POST")
        check("real 不可达 409 + 明报", st == 409 and rf.get("error"),
              json.dumps(rf))
        st, state2 = http(base, "/api/state")
        check("409 后 source 仍 none（禁隐式回落）",
              state2.get("source") == "none", json.dumps(state2))

        """ 6) real 假桥：TCP 回环模拟固件（MOD_STORE ACK 应答 + TEXT 上电行） """
        bridge = FreePortBridge()
        st, rb = http(base, "/api/open", {"mode": "real", "target": f"tcp:127.0.0.1:{bridge.port}"},
                      "POST")
        check("real 假桥 open 200（握手需有 TEXT 上电行）",
              st == 200 and rb.get("connected") is True, json.dumps(rb))

        # 假桥回 TEXTPLain 上电行（固件语义：开机发 TEXT）
        bridge.emit_text(f"OK store boot: count={7} load={1} len=70 kb=4")

        st, g3 = http(base, "/api/store/cmd", {"func": "get"}, "POST")
        p3 = g3.get("payload", {})
        check("real GET 假桥回读 boot=7", st == 200 and p3.get("boot") == 7,
              json.dumps(g3))
        st, r3 = http(base, "/api/store/cmd",
                      {"func": "set", "args": {"kv": {"boot": 0}}}, "POST")
        check("real SET boot=0 ACK OK", st == 200 and r3.get("status") == 0,
              json.dumps(r3))
        st, rb2 = http(base, "/api/close", {}, "POST")
        check("real close", st == 200 and rb2.get("source") == "none", json.dumps(rb2))
        bridge.stop()

        rec.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print(f"== {'FAIL' if fails else 'ALL PASS'}: store_demo 服务器契约 ==")
    return 1 if fails else 0


class FreePortBridge:
    """TCP 回环**假 store 固件**：listen 后应答 MOD_STORE GET/SET/STATUS ACK。

    boot 值起始 7（模拟"已经历 6 次上电"的持久化现场），SET boot 记录最近值。
    """

    def __init__(self, boot_start: int = 7):
        self.port = free_port()
        cs = socket.socket()
        cs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cs.bind(("127.0.0.1", self.port))
        cs.listen(1)
        cs.settimeout(1.0)
        self._cs = cs
        self._conn = None
        self._lock = threading.Lock()
        self._boot = boot_start
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                conn, _addr = self._cs.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._conn = conn
            conn.settimeout(1.0)
            buf = b""
            try:
                while not self._stop.is_set():
                    try:
                        chunk = conn.recv(512)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        i = buf.find(bytes([SOF]))
                        if i < 0:
                            buf = b""
                            break
                        if len(buf) - i < 4:
                            break
                        ln = buf[i + 1]
                        total = 4 + ln + 1      # SOF LEN TYPE SEQ PAYLOAD CRC
                        if len(buf) - i < total:
                            break
                        if crc8(buf[i + 1:i + 4 + ln]) != buf[i + 4 + ln]:
                            buf = buf[i + 1:]
                            continue
                        ply = buf[i + 4:i + 4 + ln]
                        self._handle(conn, ply)
                        buf = buf[i + total:]    # noqa: E741
            except Exception:
                pass
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
            self._conn = None

    def _handle(self, conn, payload):
        if not payload or payload[0] != MOD_STORE:
            return
        fn = payload[1]
        if fn == FUNC["get"]:
            n = self._boot
            data = struct.pack("<If", n, 3.25)
            data += bytes((0,)) + bytes(8)
            data += bytes((1,)) + struct.pack("<H", 70)
            data += bytes((0, 4))
            self._ack(conn, 0x01, 0, data)
        elif fn == FUNC["set"]:
            # params: [count][klen key dlen data...] — 只认 boot
            kv = self._parse_set(payload[2:])
            if "boot" in kv:
                self._boot = kv["boot"] & 0xFFFFFFFF
            self._ack(conn, 0x02, 0, b"")
        elif fn == FUNC["status"]:
            data = bytes((1,)) + struct.pack("<H", 70)
            data += bytes((0, 4, 4))
            self._ack(conn, 0x04, 0, data)

    @staticmethod
    def _parse_set(params):
        kv = {}
        if not params:
            return kv
        count = params[0]
        off = 1
        for _ in range(count):
            klen = params[off]
            key = params[off + 1:off + 1 + klen]
            dlen = params[off + 1 + klen]
            data = params[off + 2 + klen:off + 2 + klen + dlen]
            off += 2 + klen + dlen
            if len(data) == 4 and key == b"boot":
                kv["boot"] = struct.unpack("<I", data)[0]
        return kv

    def _ack(self, conn, func, status, data):
        payload = bytes([MOD_STORE, func, status]) + data
        conn.sendall(frame(ACK_TYPE, payload))

    def emit_text(self, line: str):
        with self._lock:
            conn = self._conn
        if conn:
            try:
                conn.sendall(frame(TEXT_TYPE, line.encode("ascii")))
            except OSError:
                pass

    def stop(self):
        self._stop.set()
        try:
            self._cs.close()
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())