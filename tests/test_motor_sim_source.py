"""电机合成帧源 + 三档数据源/双语文案一致性测试（纯标准库，headless 可跑）。

链路：sim_motor.SimMotorSource（假串口 poll/command）→ scope_motor.FrameDecoder
      → scope_motor 既有断言语义（check_traces / check_staircase …），
      外加一条真实 TCP 回环跑通 qemu 链路（SocketLink）的整链自证。

口径：
  1) demo 档：12s 仿真时长合成流 → check_traces 零 FAIL（既有阈值不放宽）；
  2) 场景档：五个场景各切一次 → 0x06 头/定时器/seq 不变量 + 各场景断言零 FAIL；
  3) 确定性：同一 poll 时刻序列两次产出字节逐字节相同（帧计数推进，不用墙钟/随机）；
  4) 下行命令：SCEN 同号重发幂等、未知场景号忽略、非法行忽略；
  5) 文案表：TEXT 键 zh/en 成对（text_table_errors 空）+ print 行无中文字面量
     （面向用户的打印必须走 T()）+ 连接预算 ≤ 5s；
  6) TCP 回环：sim 帧从真 socket 流过 → scope_motor.run_demo 全断言 PASS（qemu 路径）。

跑法：python tests/test_motor_sim_source.py（exit 0 = 全过）
"""

import contextlib
import io
import os
import re
import socket
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

import scope_motor as sm                                      # noqa: E402
import sim_motor                                              # noqa: E402

DEMO_SIM_S = 12.0        # demo 合成时长（仿真秒；500Hz 步长 → 6000 帧）
DEMO_MIN_FRAMES = 5000   # 12s@500Hz 理论 6000，下限留余量
CAPTURE_S = {            # 各场景采集窗（仿真秒）：≥ 该场景完整循环 + 余量
    "pwm": 16.0,         # 10s 循环
    "servo": 20.0,       # 16s 循环
    "enc": 16.0,         # 16s 循环
    "nested": 12.0,      # 10s 循环
    "dc": 8.0,           # 4s 循环
}
DRAIN_S = 1.2            # 激活后排干时长（仿真秒；镜像真机 0.6s 排干 × 2× 压缩）
HANDSHAKE_S = 0.4        # 激活前 demo 帧窗口（仿真秒）
CJK = re.compile(r"[\u4e00-\u9fff]")


def collect_demo(seconds=DEMO_SIM_S, source=None):
    """合成 demo 流 → (traces, stats, n_frames)。"""
    src = source if source is not None else sim_motor.SimMotorSource(
        sm.SCEN, sm.SLOT_KIND, sm.SG90)
    dec = sm.FrameDecoder()
    traces = [sm.Trace(i) for i in range(sm.MOTOR_SLOTS)]
    stats = {"bad_crc": 0, "seq_gaps": 0, "text": 0}
    prev = None
    n = 0
    for ftype, seq, payload in dec.feed(src.poll(seconds)):
        if ftype != sm.TYPE_MOTOR or len(payload) != sm.MOTOR_PAYLOAD_LEN:
            continue
        if prev is not None and (seq - prev - 1) % 256 != 0:
            stats["seq_gaps"] += 1
        prev = seq
        n += 1
        for r in range(sm.MOTOR_SLOTS):
            slot, kind, sp, du, ac, ph, fa = sm.MOTOR_REC.unpack_from(
                payload, r * sm.MOTOR_REC.size)
            if slot != r or kind != sm.SLOT_KIND[r]:
                stats["bad_crc"] += 1
                continue
            traces[r].add(sp, du, ac, ph, fa)
    stats["bad_crc"] += dec.bad_crc
    return traces, stats, n


def collect_scen(name):
    """切场景 → 排干 → 采集 → (traces, stats, base_timers, n_frames)。"""
    info = sm.SCEN[name]
    src = sim_motor.SimMotorSource(sm.SCEN, sm.SLOT_KIND, sm.SG90)
    dec = sm.FrameDecoder()
    dec.feed(src.poll(HANDSHAKE_S))
    src.command("SCEN %d\n" % info["id"])
    t = HANDSHAKE_S + DRAIN_S
    dec.feed(src.poll(t))                       # 排干：重启瞬态丢弃
    traces = [sm.Trace(i) for i in range(len(info["slots"]))]
    stats = {"bad_crc": 0, "seq_gaps": 0, "text": 0, "layout_bad": 0,
             "scen_bad": 0, "hdr_mismatch": 0, "demo05": 0}
    prev = None
    base_hdr = None
    timers = None
    n = 0
    for ftype, seq, payload in dec.feed(src.poll(t + CAPTURE_S[name])):
        if ftype == sm.TYPE_MOTOR:
            stats["demo05"] += 1
            continue
        if ftype != sm.TYPE_SCEN:
            continue
        parsed = sm.parse_scen_payload(payload)
        if parsed is None:
            stats["layout_bad"] += 1
            continue
        scen_id, tim, n_rec, rec_off = parsed
        if scen_id != info["id"] or n_rec != len(info["slots"]):
            stats["scen_bad"] += 1
            continue
        hdr = bytes(payload[:rec_off])
        if base_hdr is None:
            base_hdr, timers = hdr, tim
        elif hdr != base_hdr:
            stats["hdr_mismatch"] += 1
        if prev is not None and (seq - prev - 1) % 256 != 0:
            stats["seq_gaps"] += 1
        prev = seq
        n += 1
        for r in range(n_rec):
            slot, kind, sp, du, ac, ph, fa = sm.MOTOR_REC.unpack_from(
                payload, rec_off + 16 * r)
            if slot != r or kind != info["slots"][r]["kind"]:
                stats["layout_bad"] += 1
                continue
            traces[r].add(sp, du, ac, ph, fa)
    stats["bad_crc"] += dec.bad_crc
    return traces, stats, timers, n


def scen_fails(name, traces, stats, timers, n):
    """场景断言集合（镜像 scope_motor.run_scenario 的断言调用，阈值同源）。"""
    info = sm.SCEN[name]
    fails = []
    if n < sm.SCEN_CHECKS["min_frames"]:
        fails.append("frames %d < %d" % (n, sm.SCEN_CHECKS["min_frames"]))
    for key in ("bad_crc", "seq_gaps", "layout_bad", "scen_bad", "demo05", "hdr_mismatch"):
        if stats[key]:
            fails.append("%s=%d" % (key, stats[key]))
    if timers is None or sorted(timers) != sorted(info["timers"]):
        fails.append("timers %r != %r" % (timers, info["timers"]))
    if name == "pwm":
        fails += sm.check_staircase(traces[0], info["slots"][0]["steps"], "pwm")[0]
    elif name == "servo":
        fails += sm.check_servo_points(traces[0], timers[0][2], "servo",
                                       info["slots"][0]["steps"])[0]
    elif name == "enc":
        fails += sm.check_enc(traces[0], sim_motor.SimMotorCfg().dt_s)[0]
    elif name == "nested":
        fails += sm.check_nested(traces[0], traces[1], traces[2], timers[1][2])[0]
    elif name == "dc":
        fails += sm.check_dc(traces[0], "dc")[0]
    return fails


def serve_once(payload: bytes, ready, done):
    """一次性 TCP 回环服务器：accept 后发 payload 并干净关闭。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    ready.append(srv.getsockname()[1])
    conn, _ = srv.accept()
    try:
        conn.sendall(payload)
        done.wait(3.0)          # 让客户端收完再关（关闭即 EOF，收口路径照常）
    finally:
        conn.close()
        srv.close()


def case_demo(fails):
    traces, stats, n = collect_demo()
    if n < DEMO_MIN_FRAMES:
        fails.append(f"demo frames {n} < {DEMO_MIN_FRAMES}")
    demo_fails = sm.check_traces(traces, stats)
    for f in demo_fails:
        fails.append("demo: " + f)
    print(f"  demo: frames={n} bad_crc={stats['bad_crc']} gaps={stats['seq_gaps']} "
          f"assert_fails={len(demo_fails)}")


def case_scenarios(fails):
    for name in sorted(CAPTURE_S):
        traces, stats, timers, n = collect_scen(name)
        got = scen_fails(name, traces, stats, timers, n)
        for f in got:
            fails.append("%s: %s" % (name, f))
        print(f"  scenario {name}: frames={n} assert_fails={len(got)}")


def case_determinism(fails):
    sched = [0.0, 0.25, 0.5, 1.0, 2.5]
    outs = []
    for _ in range(2):
        src = sim_motor.SimMotorSource(sm.SCEN, sm.SLOT_KIND, sm.SG90)
        buf = bytearray()
        for t in sched:
            buf += src.poll(t)
        src.command("SCEN 5\n")
        for t in (0.3, 0.9, 1.7):
            buf += src.poll(sched[-1] + t)
        outs.append(bytes(buf))
    if outs[0] != outs[1]:
        fails.append(f"determinism: {len(outs[0])}B != {len(outs[1])}B")


def case_commands(fails):
    src = sim_motor.SimMotorSource(sm.SCEN, sm.SLOT_KIND, sm.SG90)
    if src.mode is not None:
        fails.append(f"initial mode {src.mode!r} != demo")
    if not src.command("SCEN 3\n"):
        fails.append("SCEN 3 rejected")
    if src.mode != 3:
        fails.append(f"mode {src.mode!r} != 3")
    before = src.poll(0.2)
    if not src.command("SCEN 3\n"):
        fails.append("idempotent resend rejected")
    after = src.poll(0.4)
    first = sm.FrameDecoder()
    frames = first.feed(before + after)
    if len(frames) != 400:
        fails.append(f"idempotent resend: {len(frames)} frames != 400")
    elif [f[1] for f in frames] != [(k + 1) % 256 for k in range(400)]:
        fails.append("idempotent resend: seq reset (scenario restarted)")
    if src.command("SCEN 9\n") or src.mode != 3:
        fails.append("unknown scenario id accepted")
    if src.command("HELLO\n") or src.command("SCEN\n"):
        fails.append("malformed line accepted")


def case_text_table(fails):
    for e in sm.text_table_errors():
        fails.append("text table: " + e)
    keys = sorted(sm.TEXT)
    zh_only = [k for k in keys if not sm.TEXT[k][0].strip()]
    en_only = [k for k in keys if not sm.TEXT[k][1].strip()]
    if zh_only or en_only:
        fails.append(f"text keys missing a language: zh={zh_only} en={en_only}")
    # 面向用户的打印必须走 T()：print 行不得出现中文字面量（文案集中在 TEXT 表）
    path = os.path.join(os.path.dirname(__file__), "..", "backend", "scope_motor.py")
    hits = [f"{i}: {line.strip()}" for i, line in
            enumerate(open(path, encoding="utf-8").read().splitlines(), 1)
            if "print(" in line and CJK.search(line)]
    for h in hits:
        fails.append("hardcoded CJK print: " + h)
    if sm.CONNECT_TIMEOUT_S + sm.CONNECT_ATTEMPT_S > 5.0:
        fails.append(f"connect budget {sm.CONNECT_TIMEOUT_S}s exceeds 5s")
    print(f"  text table: {len(keys)} keys paired, print lines CJK-free")


def case_tcp_loopback(fails):
    """sim 帧经真 socket（qemu 链路）→ run_demo 全断言 PASS。"""
    src = sim_motor.SimMotorSource(sm.SCEN, sm.SLOT_KIND, sm.SG90)
    payload = src.poll(3.0)          # 3s 仿真时长 = 1500 帧
    ready = []
    done = threading.Event()
    th = threading.Thread(target=serve_once, args=(payload, ready, done), daemon=True)
    th.start()
    while not ready:
        th.join(0.01)
    args = sm.argparse.Namespace(source="qemu", lang="zh", host="127.0.0.1",
                                 port=ready[0], serial=None, baud=sm.DEFAULT_BAUD,
                                 seconds=2.0, render_every=0, quiet=True,
                                 no_plot=True, scenario=None)
    link = sm.connect_scope(args.host, args.port)
    if link is None:
        fails.append("tcp loopback: connect failed")
        done.set()
        return
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            rc = sm.run_demo(args, link)
    finally:
        done.set()
    if rc != 0:
        fails.append(f"tcp loopback: run_demo rc={rc}")
    print(f"  tcp loopback: port={ready[0]} rc={rc}")


def main() -> int:
    fails = []
    case_text_table(fails)
    case_commands(fails)
    case_determinism(fails)
    case_demo(fails)
    case_scenarios(fails)
    case_tcp_loopback(fails)
    print(f"== test_motor_sim_source: {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
