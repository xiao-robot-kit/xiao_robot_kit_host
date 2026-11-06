#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""test_motor_server.py — 电机实验页服务器契约测试（headless，真 HTTP + 真 SSE）。

覆盖（用户定案架构：本地服务器 + 实例目录页面；电机数据面 = backend/motor_hub.py）：
  1) 状态面：/api/state 出 kind/scenario/source/target/connected/frames/stats/counters；
     缺省 source=none & connected=false（服务器启动不碰任何数据源）
  2) 场景页 motor_pwm（0x06 SCEN 1）：open{sim} → SSE 收 mframe（type/scen/timers/recs
     形状）→ POST /api/verify pass=true（占空阶梯断言，阈值走 scope_motor.SCEN_CHECKS）
     → /api/close 回到 none
  3) 场景页 motor_dc（0x06 SCEN 5）：open{sim} → verify pass=true（H 桥方向对称断言）
  4) pidscan 页 motor_pidscan：open{sim} → SSE 收 mframe + text（TEXT 行）
     → verify pass=true（9 组收敛 + 固件/host 双表，口径走 scope_pidscan）
  4b) 其余场景页（servo/enc/nested）：in-process 驱动 MotorHub → verify pass
     （验「场景 → 断言函数 → 稳定键」映射，pwm/dc 已在 HTTP 链路上验过）
  5) 真实目标不可达：POST /api/open{real,target=tcp:…} → 409 + error，
     source 仍为 none（**任何情况都不自动回落 sim**）
  5b) 真实目标可达（TCP 回环假桥）：real 档握手（0.5s 重发 SCEN 直到 0x06 匹配）
     → verify pass=true → /api/command 下行透传
  6) sim 档下行命令：POST /api/command → {"code":"noDownlink"}

跑法：python tests/test_motor_server.py（exit 0 = 全过）
"""

import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SERVER = os.path.join(ROOT, "backend", "server.py")

fails = []


def check(name, ok, detail=""):
    if ok:
        print(f"  [ok]   {name}")
    else:
        fails.append(f"{name}{('：' + detail) if detail else ''}")
        print(f"  [FAIL] {name}{('：' + detail) if detail else ''}")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def http(base, path, body=None, method="GET", timeout=120.0):
    """发一次请求 → (status, json)；HTTP 错误也返回状态码 + 响应体（409 要可断言）。"""
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
            return r.status, (json.loads(raw) if raw else {})
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


def read_sse(port: int, until: dict, budget: float = 20.0):
    """读 SSE 直到每个事件名都达到 until 里的条数（或超时）。

    返回 (counts, samples)：counts = {事件名: 条数}，samples = {事件名: 首个 data}。
    """
    counts, samples = {}, {}
    opener = urllib.request.build_opener()
    deadline = time.time() + budget
    try:
        with opener.open(f"http://127.0.0.1:{port}/api/stream", timeout=budget) as r:
            ev = None
            while time.time() < deadline:
                line = r.readline()
                if not line:
                    break
                text = line.decode("utf-8", "replace").strip()
                if text.startswith("event: "):
                    ev = text[7:]
                    continue
                if not text.startswith("data: ") or ev is None:
                    continue
                try:
                    data = json.loads(text[6:])
                except ValueError:
                    data = None
                counts[ev] = counts.get(ev, 0) + 1
                samples.setdefault(ev, data)
                if all(counts.get(k, 0) >= v for k, v in until.items()):
                    break
    except (socket.timeout, TimeoutError, OSError):
        pass
    return counts, samples


def wait_verify(base: str, budget: float = 30.0, interval: float = 0.4) -> dict:
    """轮询 POST /api/verify 到 pass=true（或预算用尽，返回最后一次返回体）。"""
    deadline = time.time() + budget
    last = {}
    while time.time() < deadline:
        code, last = http(base, "/api/verify", body={}, method="POST")
        if code == 200 and last.get("pass"):
            return last
        time.sleep(interval)
    return last


class FakeBridge:
    """假 QEMU 串口 TCP 桥（测试对端）：收 "SCEN <n>" 行 → 用 sim_motor 合成帧回吐。

    只出现在测试里：把真实 tcp 链路（SocketLink）+ 场景激活握手（0.5s 重发直到
    0x06 scen_id 匹配）跑通，不碰硬件。缺省先吐 0x05 demo 帧（模拟固件启动后的
    遥测），收到 SCEN 命令才切 0x06——与 scope_motor.run_scenario 假设同序。
    """

    def __init__(self):
        sys.path.insert(0, os.path.join(ROOT, "backend"))
        import scope_motor
        import sim_motor
        self._src = sim_motor.SimMotorSource(scope_motor.SCEN, scope_motor.SLOT_KIND,
                                            scope_motor.SG90, sim_motor.SimMotorCfg())
        self._srv = socket.socket()
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    @property
    def target(self) -> str:
        return f"tcp:127.0.0.1:{self.port}"

    def _serve(self) -> None:
        try:
            conn, _ = self._srv.accept()
        except OSError:
            return
        conn.settimeout(0.02)
        start = time.monotonic()
        rx = b""
        try:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(256)
                    if not chunk:
                        break
                    rx += chunk
                    while b"\n" in rx:
                        line, _, rx = rx.partition(b"\n")
                        self._src.command(line.decode("ascii", "replace"))
                except (socket.timeout, TimeoutError):
                    pass
                data = self._src.poll(time.monotonic() - start)
                if data:
                    conn.sendall(data)
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def stop(self) -> None:
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        self._thread.join(1.0)


class Server:
    """一个页面实例的服务器进程（--no-open，独占端口）。"""

    def __init__(self, page: str):
        self.page = page
        self.port = free_port()
        self.proc = subprocess.Popen(
            [sys.executable, SERVER, "--no-open", "--port", str(self.port), "--page", page],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.ready = wait_ready(self.port)

    @property
    def base(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def stop(self) -> None:
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def booted(page: str) -> Server:
    srv = Server(page)
    check(f"{page} 服务器就绪", srv.ready, f"port={srv.port}")
    return srv


def case_pwm() -> None:
    srv = booted("motor_pwm")
    try:
        if not srv.ready:
            return
        code, st = http(srv.base, "/api/state")
        check("pwm 状态面字段", code == 200 and st.get("kind") == "motor"
              and st.get("scenario") == "pwm"
              and {"source", "target", "connected", "frames", "stats", "counters"} <= set(st),
              json.dumps(st, ensure_ascii=False))
        check("pwm 缺省不启动", st.get("source") == "none" and st.get("connected") is False,
              json.dumps(st, ensure_ascii=False))

        code, ports = http(srv.base, "/api/ports")
        check("pwm 串口枚举形状", code == 200 and isinstance(ports.get("ports"), list), str(ports))

        code, st = http(srv.base, "/api/open", {"mode": "sim"}, "POST")
        check("pwm open sim 生效", code == 200 and st.get("source") == "sim"
              and st.get("connected") is True, f"code={code} {st}")

        counts, samples = read_sse(srv.port, {"mframe": 10, "state": 1}, budget=15.0)
        mf = samples.get("mframe") or {}
        check("pwm SSE 收 mframe", counts.get("mframe", 0) >= 10, f"counts={counts}")
        check("pwm mframe 是 0x06 场景帧", mf.get("type") == 6 and mf.get("scen") == 1,
              str(mf)[:200])
        timers = mf.get("timers") or []
        check("pwm mframe timers 为列表", timers == [[0, 4, 20000]], str(timers))
        recs = mf.get("recs") or []
        check("pwm mframe recs 单槽六字段", len(recs) == 1 and set(recs[0]) >= {
            "slot", "kind", "sp", "duty", "actual", "phase", "fault"}, str(recs)[:200])

        ver = wait_verify(srv.base)
        keys = [i.get("key") for i in ver.get("items", [])]
        check("pwm verify pass", ver.get("pass") is True, json.dumps(ver, ensure_ascii=False)[:300])
        check("pwm verify 键稳定", "pwm.staircase" in keys and "link.frames" in keys, str(keys))
        summ = ver.get("summary") or {}
        check("pwm summary 字段", {"frames", "bad_crc", "seq_gaps", "text"} <= set(summ)
              and summ.get("frames", 0) >= 2000, str(summ))

        code, cmd = http(srv.base, "/api/command", {"line": "SCEN 1\n"}, "POST")
        check("pwm sim 无下行", code == 200 and cmd.get("code") == "noDownlink", str(cmd))

        code, st = http(srv.base, "/api/close", {"mode": "none"}, "POST")
        check("pwm close 回 none", st.get("source") == "none" and st.get("connected") is False,
              str(st))
    finally:
        srv.stop()


def case_dc() -> None:
    srv = booted("motor_dc")
    try:
        if not srv.ready:
            return
        code, st = http(srv.base, "/api/open", {"mode": "sim"}, "POST")
        check("dc open sim 生效", code == 200 and st.get("source") == "sim", f"code={code} {st}")
        counts, samples = read_sse(srv.port, {"mframe": 10}, budget=15.0)
        mf = samples.get("mframe") or {}
        check("dc mframe 场景号=5", mf.get("type") == 6 and mf.get("scen") == 5, str(mf)[:200])
        check("dc SSE 收 mframe", counts.get("mframe", 0) >= 10, f"counts={counts}")

        ver = wait_verify(srv.base)
        keys = [i.get("key") for i in ver.get("items", [])]
        check("dc verify pass", ver.get("pass") is True, json.dumps(ver, ensure_ascii=False)[:300])
        check("dc verify 键稳定", "dc.direction" in keys, str(keys))

        # 真实目标不可达：409 + error，且**不回落 sim**
        dead = free_port()
        code, err = http(srv.base, "/api/open",
                         {"mode": "real", "target": f"tcp:127.0.0.1:{dead}"}, "POST")
        check("real 不可达 → 409", code == 409 and bool(err.get("error")),
              f"code={code} {err}")
        code, st = http(srv.base, "/api/state")
        check("real 失败不回落 sim", st.get("source") == "none" and st.get("connected") is False,
              json.dumps(st, ensure_ascii=False))

        code, err = http(srv.base, "/api/open", {"mode": "real", "target": "bogus"}, "POST")
        check("real 目标格式错 → 409", code == 409 and bool(err.get("error")),
              f"code={code} {err}")
        check("close 幂等", http(srv.base, "/api/close", {}, "POST")[0] == 200)
    finally:
        srv.stop()


def case_real_tcp() -> None:
    """real 档真链路：真 TCP + 场景激活握手（SCEN 0.5s 重发→0x06 匹配）。"""
    bridge = FakeBridge()
    srv = booted("motor_pwm")
    try:
        if not srv.ready:
            return
        code, st = http(srv.base, "/api/open",
                        {"mode": "real", "target": bridge.target}, "POST", timeout=60.0)
        check("real tcp 握手成功", code == 200 and st.get("source") == "real"
              and st.get("connected") is True and bridge.target in st.get("target", ""),
              f"code={code} {st}")
        ver = wait_verify(srv.base)
        keys = [i.get("key") for i in ver.get("items", [])]
        check("real tcp verify pass", ver.get("pass") is True,
              json.dumps(ver, ensure_ascii=False)[:300])
        check("real tcp 场景断言键", "pwm.staircase" in keys, str(keys))
        code, cmd = http(srv.base, "/api/command", {"line": "SCEN 1\n"}, "POST")
        check("real 档下行透传", code == 200 and cmd.get("code") == "sent", str(cmd))
    finally:
        srv.stop()
        bridge.stop()


def case_scenarios_inproc() -> None:
    """剩余场景页的断言映射（pwm/dc 已在 HTTP 链路上验过；这里补 servo/enc/nested）。

    in-process 直接驱动 MotorHub（不经服务器）：只验「页面场景 → 断言函数 → 稳定键」
    映射，不必为每个场景再起一个服务器进程。
    """
    sys.path.insert(0, os.path.join(ROOT, "backend"))
    import motor_hub
    import scope_motor
    for scen in ("servo", "enc", "nested"):
        hub = motor_hub.MotorHub(kind="motor", scenario=scen,
                                 scen_id=scope_motor.SCEN[scen]["id"])
        try:
            hub.open_sim()
            deadline = time.time() + 20.0
            verdict = {}
            while time.time() < deadline:
                time.sleep(0.4)
                verdict = hub.verify()
                if verdict.get("pass"):
                    break
            key = motor_hub.SCEN_ASSERTION_KEY[scen]
            keys = [i.get("key") for i in verdict.get("items", [])]
            check(f"{scen} 场景 verify pass + 键 {key}",
                  verdict.get("pass") is True and key in keys,
                  json.dumps([i for i in verdict.get("items", []) if not i.get("ok")],
                             ensure_ascii=False)[:300])
        finally:
            hub.close()


def case_pidscan() -> None:
    srv = booted("motor_pidscan")
    try:
        if not srv.ready:
            return
        code, st = http(srv.base, "/api/state")
        check("pidscan 状态面 kind", st.get("kind") == "pidscan" and st.get("scenario") is None,
              json.dumps(st, ensure_ascii=False))

        code, st = http(srv.base, "/api/open", {"mode": "sim"}, "POST")
        check("pidscan open sim 生效", code == 200 and st.get("source") == "sim",
              f"code={code} {st}")
        counts, samples = read_sse(srv.port, {"mframe": 5, "text": 1}, budget=20.0)
        check("pidscan SSE 收 mframe + text", counts.get("mframe", 0) >= 5
              and counts.get("text", 0) >= 1, f"counts={counts}")
        txt = samples.get("text") or {}
        check("pidscan text 事件带 line", isinstance(txt.get("line"), str) and txt["line"],
              str(txt)[:200])

        ver = wait_verify(srv.base)
        keys = [i.get("key") for i in ver.get("items", [])]
        check("pidscan verify pass", ver.get("pass") is True,
              json.dumps(ver, ensure_ascii=False)[:300])
        check("pidscan 九组 + 双表键", all(f"pidscan.g{g}" in keys for g in range(9))
              and {"pidscan.round", "pidscan.table", "pidscan.best"} <= set(keys), str(keys))
        summ = ver.get("summary") or {}
        check("pidscan summary 帧数", summ.get("frames", 0) >= 9000 and summ.get("text", 0) >= 12,
              str(summ))
    finally:
        srv.stop()


def main() -> int:
    case_pwm()
    case_dc()
    case_real_tcp()
    case_scenarios_inproc()
    case_pidscan()
    if fails:
        print(f"\n== test_motor_server: {len(fails)} fail ==")
        return 1
    print("\n== test_motor_server: 状态面/场景页 sim+SSE+verify/real tcp 握手/五场景断言/"
          "pidscan/409 不回落, 0 fail ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())
