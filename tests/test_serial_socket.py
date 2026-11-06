"""SerialSocketSource socket 层一致性测试（真实 TCP 回环，无 Qt/真串口依赖，headless 可跑）。

链路：本地 socket 服务器线程发合成帧字节（编码复用 sim_source.encode_imu_sample，
      常量单源自 frame.py）→ SerialSocketSource（pyserial socket:// URL 真连接）
      → 帧断言（数量 / 类型 / seq 连续 / payload 字节往返 / 重同步）。
口径：四场景——
  1) EOF 停止（on_eof="stop"）：N 帧发完即断流，生成器应产出恰好 N 帧后返回，
     流头注入 3 字节非 SOF 噪声考验重同步；
  2) 迟到服务器（连接退避重试）：客户端先起（连预选端口，必然被拒）、服务器
     0.4s 后才监听同端口，退避重连后仍应收满 N 帧（连接阶段失败重试路径）；
  3) 流断重连（默认 on_eof="reconnect"）：两轮各 N1/N2 帧、轮间断流关连接，
     应续收满 N1+N2 帧且 seq 跨轮连续（读阶段流断重连路径）；
  4) 无帧收口（max_idle_s）：连接活着但只发噪声字节并保持连接不关，
     生成器应在 max_idle_s 到点附近返回（有界收流，杜绝无帧流挂死）。
台架约定（规避 pyserial socket 后端两坑，见 backend/serial_socket.py 模块注）：
  - 服务器 accept 后延迟 POST_ACCEPT_DELAY_S 再发：open() 的
    reset_input_buffer 会排空 open 窗口内已到数据，必须让客户端 open 完成后再发；
  - 分片 13B/5ms 发送：考验解码器跨 read 分片缓冲（read_size=1 时即跨字节流）。
跑法：python tests/test_serial_socket.py（exit 0 = 全过）
"""

import os
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from frame import TYPE_IMU_SAMPLE, decode_sample  # noqa: E402
from serial_socket import SerialSocketSource  # noqa: E402
from sim_source import encode_imu_sample  # noqa: E402

CHUNK_B = 13             # 服务器分片发送粒度（< 帧长 29B）
CHUNK_DELAY_S = 0.005
NOISE_HEAD = b"\x00\x11\x22"          # 流头噪声（不含 SOF）：重同步考验
POST_ACCEPT_DELAY_S = 0.08            # accept 后延迟发送：躲开 open() 排空窗口
LATE_SERVER_DELAY_S = 0.4             # 场景 2：服务器迟到时长（s）


def make_frames(n: int, seq0: int) -> bytes:
    """合成 n 帧（seq 从 seq0 起连续）：vals=(k/1000, -k/1000, 1, 0, 0, 0) g/dps。"""
    out = bytearray()
    for k in range(n):
        vals = ((seq0 + k) / 1000.0, -(seq0 + k) / 1000.0, 1.0, 0.0, 0.0, 0.0)
        out += encode_imu_sample(seq0 + k, vals)
    return bytes(out)


def expected_payload(seq: int) -> bytes:
    """第 seq 帧的期望 payload（与 make_frames 的 vals 同式，字节级往返口径）。"""
    return struct.pack("<6f", seq / 1000.0, -seq / 1000.0, 1.0, 0.0, 0.0, 0.0)


def free_port() -> int:
    """预选一个空闲端口（bind 后即关；仅供「先拒后连」时序用）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FeedServer(threading.Thread):
    """一次性回环服务器：rounds 为 (data, hold_s) 元组列表，逐轮
    accept → 延迟 → 分片发 → （按 hold_s 保持连接不关）→ 关连接。

    port=None 时自选临时端口；指定 port 时绑定之（服务器迟到场景用）；
    hold_s>0 发完后保持连接静默 hold_s 秒（无帧 idle 场景）。
    """

    def __init__(self, rounds, port=None):
        super().__init__(daemon=True)
        self._rounds = rounds
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", port if port is not None else 0))
        self._srv.listen(4)
        self.port = self._srv.getsockname()[1]
        self.served = threading.Event()      # 全部 round 发完（含关连接）置位

    def run(self):
        try:
            for data, hold_s in self._rounds:
                conn, _ = self._srv.accept()
                time.sleep(POST_ACCEPT_DELAY_S)   # 等客户端 open()（含排空）完成
                try:
                    for i in range(0, len(data), CHUNK_B):
                        conn.sendall(data[i:i + CHUNK_B])
                        time.sleep(CHUNK_DELAY_S)
                    if hold_s > 0:
                        time.sleep(hold_s)   # 连接活着但无数据（idle 场景）
                finally:
                    conn.shutdown(socket.SHUT_WR)   # → 客户端读到 EOF
                    conn.close()
        finally:
            self._srv.close()
            self.served.set()


def collect(url: str, want: int, deadline_s: float, **src_kwargs):
    """起一个受看门狗约束的收流线程，返回 (frames, timed_out)。"""
    out = []

    def worker():
        deadline = time.time() + deadline_s
        for f in SerialSocketSource(url, **src_kwargs).frames():
            out.append(f)
            if len(out) >= want or time.time() > deadline:
                return

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(deadline_s + 2.0)
    return out, t.is_alive()


def scenario_eof_stop(n: int):
    """场景 1：噪声头 + N 帧 + 断流 → on_eof="stop" 应恰好收满 N 帧后返回。"""
    srv = FeedServer([(NOISE_HEAD + make_frames(n, seq0=1), 0)])
    srv.start()
    frames, hung = collect(f"socket://127.0.0.1:{srv.port}", n, deadline_s=5.0,
                           read_timeout=0.2, reconnect_initial_s=0.1, on_eof="stop")
    srv.served.wait(5.0)
    return frames, hung


def scenario_late_server(n: int, port: int):
    """场景 2：客户端先起（连无监听端口，必然被拒）→ 退避重试 → 服务器 0.4s 后上线。"""
    holder = {"port": port}

    def worker():
        deadline = time.time() + 5.0
        out = []
        for f in SerialSocketSource(f"socket://127.0.0.1:{holder['port']}",
                                    read_timeout=0.2, reconnect_initial_s=0.15,
                                    reconnect_max_s=0.6, on_eof="stop").frames():
            out.append(f)
            if len(out) >= n or time.time() > deadline:
                break
        holder["frames"] = out

    t = threading.Thread(target=worker, daemon=True)
    t.start()                                  # 客户端先起：此刻端口无监听
    time.sleep(LATE_SERVER_DELAY_S)
    srv = FeedServer([(make_frames(n, seq0=1), 0)], port=holder["port"])   # 服务器迟到
    srv.start()
    t.join(7.0)
    srv.served.wait(5.0)
    return holder.get("frames", []), t.is_alive()


def scenario_reconnect_after_eof(n_a: int, n_b: int):
    """场景 3：两轮流、轮间断流 → 默认 on_eof="reconnect" 应跨轮续收满。"""
    want = n_a + n_b
    srv = FeedServer([(make_frames(n_a, seq0=1), 0), (make_frames(n_b, seq0=n_a + 1), 0)])
    srv.start()
    frames, hung = collect(f"socket://127.0.0.1:{srv.port}", want, deadline_s=6.0,
                           read_timeout=0.2, reconnect_initial_s=0.1,
                           reconnect_max_s=0.5)
    srv.served.wait(5.0)
    return frames, hung, want


def scenario_idle_return(hold_s: float = 1.5, max_idle_s: float = 0.4):
    """场景 4：连接活着但只发噪声并保持 hold_s 不关 → max_idle_s 到点应返回。

    返回 (frames, elapsed_s, hung)；elapsed = 生成器从启动到返回的墙钟时长。
    """
    srv = FeedServer([(NOISE_HEAD, hold_s)])
    holder = {"port": srv.port}

    def worker():
        t0 = time.time()
        n = 0
        for _f in SerialSocketSource(f"socket://127.0.0.1:{holder['port']}",
                                     read_timeout=0.2, on_eof="stop",
                                     max_idle_s=max_idle_s).frames():
            n += 1
        holder["frames"] = n
        holder["elapsed"] = time.time() - t0

    srv.start()
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(4.0)
    srv.served.wait(5.0)
    return holder.get("frames", -1), holder.get("elapsed"), t.is_alive()


def main() -> int:
    fails = []

    # 场景 1：EOF 停止 + 流头噪声重同步
    n1 = 12
    frames, hung = scenario_eof_stop(n1)
    if hung:
        fails.append(f"S1 看门狗超时（收 {len(frames)} 帧）")
    if len(frames) != n1:
        fails.append(f"S1 frames={len(frames)} != {n1}（噪声重同步或 EOF 停止失效）")
    else:
        if set(t for t, _, _ in frames) != {TYPE_IMU_SAMPLE}:
            fails.append("S1 帧类型不全为 IMU_SAMPLE")
        if [s for _, s, _ in frames] != list(range(1, n1 + 1)):
            fails.append("S1 seq 不连续（1..N）")
        bad = [s for _, s, p in frames if p != expected_payload(s)]
        if bad:
            fails.append(f"S1 payload 字节往返不一致 seq={bad[:3]}")
        else:
            v = decode_sample(frames[0][2])
            if v[2] != 1.0 or abs(v[0] - 1 / 1000.0) > 1e-9:
                fails.append(f"S1 decode_sample 抽查不符 {v[:3]!r}")

    # 场景 2：迟到服务器（连接阶段失败 → 退避重试）
    n2 = 10
    frames2, hung2 = scenario_late_server(n2, port=free_port())
    if hung2:
        fails.append(f"S2 看门狗超时（收 {len(frames2)} 帧）")
    if len(frames2) != n2:
        fails.append(f"S2 frames={len(frames2)} != {n2}（迟到服务器退避重连失效）")
    elif [s for _, s, _ in frames2] != list(range(1, n2 + 1)):
        fails.append("S2 seq 不连续（1..N）")

    # 场景 3：流断重连（读阶段 EOF → 默认 on_eof="reconnect" 续收）
    frames3, hung3, want3 = scenario_reconnect_after_eof(8, 7)
    if hung3:
        fails.append(f"S3 看门狗超时（收 {len(frames3)} 帧）")
    if len(frames3) != want3:
        fails.append(f"S3 frames={len(frames3)} != {want3}（轮间断流后续收失效）")
    elif [s for _, s, _ in frames3] != list(range(1, want3 + 1)):
        fails.append("S3 seq 跨轮不连续")

    # 场景 4：无帧收口（max_idle_s；连接活着只发噪声并保持 1.5s 不关）
    frames4, elapsed4, hung4 = scenario_idle_return()
    if hung4:
        fails.append("S4 看门狗超时（无帧流未收口）")
    if frames4 != 0:
        fails.append(f"S4 frames={frames4} != 0（噪声不应产帧）")
    if elapsed4 is not None:
        if elapsed4 < 0.35 or elapsed4 > 1.2:
            fails.append(f"S4 elapsed={elapsed4:.2f}s 越界（期望 ≈max_idle 0.4s，"
                         f"且早于服务器关连接 1.6s）")

    print(f"== test_serial_socket: S1={len(frames)}/{n1}, S2={len(frames2)}/{n2}, "
          f"S3={len(frames3)}/{want3}, S4={frames4}/0 @{elapsed4 and round(elapsed4, 2)}s, "
          f"{len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
