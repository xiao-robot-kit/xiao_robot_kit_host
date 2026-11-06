#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""test_server.py — 服务器版上位机（backend/server.py + hub.py）契约测试（headless）。

覆盖（用户定案架构：本地服务器 + 实例目录页面 + 单实例重启）：
  1) 路由：`/` → 302 到 --page；`/<page>/` 出页面；`/assets/...` 出共享资产；
     未知页面 / 路径穿越（../backend/server.py）一律 404
  2) 显式模式：/api/state 缺省 mode=none & connected=false（不自动启任何源）
  3) 数据面：POST /api/open{sim} → SSE /api/stream 收到 ≥20 个 frame 事件；
     POST /api/close → 回到 none；帧内容与协议同规格（QUAT w 分量 ≈±1）
  4) 单实例：已运行时 `--ensure` 必须杀掉旧进程并在同端口重新起服务
     （避免端口/设备被抢占——run.bat 的一键语义）

跑法：python tests/test_server.py（exit 0 = 全过）
"""

import json
import os
import socket
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SERVER = os.path.join(ROOT, "backend", "server.py")
RUNTIME = os.path.join(ROOT, ".runtime", "server.json")

fails = []


def check(name, ok, detail=""):
    if not ok:
        fails.append(f"{name}{('：' + detail) if detail else ''}")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def http(url, timeout=3.0, method="GET", body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.read(), dict(r.headers)


def wait_ready(port: float, budget=10.0) -> bool:
    deadline = time.time() + budget
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.15)
    return False


def pid_alive(pid: int) -> bool:
    """存活判定：Windows 用 GetExitCodeProcess==STILL_ACTIVE（OpenProcess 成功不等于活着）。"""
    if os.name == "nt":
        import ctypes
        h = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)
        if not h:
            return False
        try:
            code = ctypes.c_ulong()
            ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
            return code.value == 259          # STILL_ACTIVE
        finally:
            ctypes.windll.kernel32.CloseHandle(h)
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *a, **kw):
        return None


def read_sse(port: int, want: int = 20, budget: float = 3.0) -> list:
    """读 SSE 直到收到 want 个 frame 事件或超时；返回事件名列表。"""
    opener = urllib.request.build_opener()
    events = []
    deadline = time.time() + budget
    with opener.open(f"http://127.0.0.1:{port}/api/stream", timeout=budget) as r:
        ev = None
        while time.time() < deadline and events.count("frame") < want:
            line = r.readline()
            if not line:
                break
            text = line.decode("utf-8", "replace").strip()
            if text.startswith("event: "):
                ev = text[7:]
                events.append(ev)
            elif text.startswith("data: ") and ev == "frame":
                payload = json.loads(text[6:])
                global _frame_sample
                _frame_sample = payload
    return events


_frame_sample = {}


def main() -> int:
    port = free_port()
    env = dict(os.environ)
    proc = subprocess.Popen([sys.executable, SERVER, "--no-open", "--port", str(port),
                             "--page", "exp4"],
                            cwd=ROOT, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        check("服务器就绪", wait_ready(port), f"port={port}")
        if fails:
            print("== test_server: 服务未起，终止 ==")
            return 1

        base = f"http://127.0.0.1:{port}"
        # 1) 路由
        opener = urllib.request.build_opener(NoRedirect)
        try:
            opener.open(base + "/", timeout=3)
            check("根路径 302", False, "未重定向")
        except urllib.error.HTTPError as e:
            check("根路径 302", e.code in (301, 302) and "/exp4/" in (e.headers.get("Location") or ""),
                  f"code={e.code} loc={e.headers.get('Location')}")
        code, body, _ = http(base + "/exp4/")
        check("页面可服务", code == 200 and b"exp4" in body, f"code={code}")
        code, body, hdr = http(base + "/assets/ui-shell.js")
        check("共享资产可服务", code == 200 and b"UI" in body, f"code={code}")
        for bad in ("/nope/", "/exp4/../../backend/server.py", "/assets/../../backend/server.py"):
            try:
                code, _, _ = http(base + bad)
                check(f"拒绝 {bad}", code == 404, f"code={code}")
            except urllib.error.HTTPError as e:
                check(f"拒绝 {bad}", e.code == 404, f"code={e.code}")

        # 2) 显式模式（不自动启动）
        code, body, _ = http(base + "/api/state")
        st = json.loads(body)
        check("缺省 none/未连接", st["mode"] == "none" and st["connected"] is False, str(st))

        # 3) 数据面：open sim → SSE 收帧
        code, body, _ = http(base + "/api/open", body={"mode": "sim"}, method="POST")
        st = json.loads(body)
        check("open sim 生效", st["mode"] == "sim" and st["connected"] is True, str(st))
        events = read_sse(port, want=20, budget=3.0)
        check("SSE 收帧", events.count("frame") >= 20, f"events={events[:8]}… 共{len(events)}")
        vals = _frame_sample.get("vals") or []
        check("帧内容与协议同规格", len(vals) == 4 and abs(abs(vals[0]) - 1.0) < 0.1,
              f"sample={_frame_sample}")

        code, body, _ = http(base + "/api/close", body={}, method="POST")
        st = json.loads(body)
        check("close 回到 none", st["mode"] == "none" and st["connected"] is False, str(st))

        # 4) 单实例：--ensure 杀掉旧进程并重启
        old_pid = json.load(open(RUNTIME, encoding="utf-8"))["pid"]
        proc2 = subprocess.Popen([sys.executable, SERVER, "--ensure", "--no-open",
                                  "--port", str(port), "--page", "exp4"],
                                 cwd=ROOT, env=env,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            check("重启后端口就绪", wait_ready(port, 12.0), f"port={port}")
            time.sleep(0.5)
            new_pid = json.load(open(RUNTIME, encoding="utf-8"))["pid"]
            check("旧进程被终止", not pid_alive(old_pid), f"old_pid={old_pid}")
            check("新进程接管", new_pid != old_pid and pid_alive(new_pid),
                  f"old={old_pid} new={new_pid}")
            code, body, _ = http(base + "/exp4/")
            check("重启后页面可服务", code == 200 and b"exp4" in body, f"code={code}")
            code, body, _ = http(base + "/api/state")
            check("重启后状态干净", json.loads(body)["mode"] == "none", body[:80].decode())
        finally:
            proc2.terminate()
            try:
                proc2.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc2.kill()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    if fails:
        for f in fails:
            print("  [FAIL]", f)
        print(f"== test_server: {len(fails)} fail ==")
        return 1
    print("== test_server: 路由/显式模式/SSE 数据面/单实例重启, 0 fail ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())
