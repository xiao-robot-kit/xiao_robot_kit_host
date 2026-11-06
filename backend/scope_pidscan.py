#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scope_pidscan.py — PID 参数网格扫描实验的调参对比表查看器（纯标准库零依赖）。

意图：解码 exp_motor_pidscan 固件的 0x05 遥测帧与 0x7F TEXT 汇总帧，在终端
呈现**调参对比表**（调参实验的核心查看面）：固件侧 TEXT 表 + host 独立重算
表双表对照，最优组 '*' 标注。

数据源三档显式（--source；缺省 sim——「没接设备却自动连 QEMU」不合理，故不自动连）：
    sim   本机合成帧源（backend/sim_pidscan.py，按帧计数推进、零外部依赖）——
          其 TEXT 表由同一份合成轨迹按固件口径算出，故 sim 档双表验的是「解码 +
          重算口径一致」；真独立实现对照由 qemu / real 档提供；
    qemu  TCP 串口桥（QEMU -serial tcp:host:port,server，wait 模式——QEMU 先起、
          本工具连上即开机收流；连不上 ≤5s 明报 FAIL 并 exit 2）；
    real  pyserial 物理串口（--serial COMx --baud 115200，解码路径与 qemu 档同一套）。

面向用户打印行中英成对（--lang zh|en，缺省 zh；表格列名保留符号）。

帧规格（单源 = 固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md））：

    SOF(0xA5) LEN TYPE SEQ PAYLOAD(LEN) CRC8（覆盖 LEN..PAYLOAD, poly 0x07）

    TYPE=0x05 MOTOR：本工程 payload = 1×16B 记录（单电机轮转）：
        u8 slot（= 扫描组号 0..8）| u8 kind | f32 setpoint | f32 duty
        | f32 actual | u8 phase（组内步号：0=低段 30rpm，1=高段 90rpm）
        | u8 fault
    TYPE=0x7F TEXT：ASCII 行（对比表逐行：表头 + 9 组行 + BEST + round done）。

host 独立断言（与固件指标同口径、独立算法——防同源盲区）：
    A. 链路健康：0x05 帧数下限、坏 CRC 重同步上限、9 个组号全覆盖；
    B. 每组收敛：高段末半段均值 = 90±3%（任务卡：终值 ±3%）；
    C. 对比表完整：TEXT 组行 ≥9、最优标注恰 1 行、round done 行出现；
    D. 双表对照：host 独立算的三指标与固件 TEXT 组行数值一致（容差内）。

用法：
    python scope_pidscan.py [--source sim|qemu|real] [--lang zh|en] [--seconds 75]
                            [--host 127.0.0.1] [--port 5565]
                            [--serial COM5] [--baud 115200] [--quiet]
    exit 0=PASS；1=断言 FAIL；2=连接/选源失败。
"""

import argparse
import socket
import struct
import sys
import time

from sim_pidscan import ROUND_S, SimPidscanSource, rounds_for_seconds

SOF = 0xA5
PAYLOAD_MAX = 96
TYPE_MOTOR = 0x05
TYPE_TEXT = 0x7F

N_GROUPS = 9          # 网格组数（与固件 EXP_MOTOR_PIDSCAN_GROUPS 对齐）
STEP_LO = 30.0        # 低段目标 rpm（独立期望，不读固件表）
STEP_HI = 90.0        # 高段目标 rpm
TOL_PCT = 3.0         # 段收敛判据 ±3%（任务卡）
PH_LO, PH_HI = 0, 1   # 组内步号（0=低段，1=高段）

CONNECT_TIMEOUT_S = 4.5     # qemu 档连接总超时：端到端（含解释器启动）必须 ≤5s 出 FAIL
READ_CHUNK = 4096
DRAIN_S = 1.0               # round done 后的排干窗（收全在途帧即收工）


def crc8(data: bytes) -> int:
    """CRC-8，多项式 0x07，初值 0x00（与 C 侧 proto_crc8 / probe_sim.py 同结果）。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameDecoder:
    """流式解码器：喂任意分片，逐帧产出 (type, seq, payload)。

    容错与 probe_sim.py 同口径：CRC 错或 LEN 超限 → 丢到下一候选 SOF。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_resync = 0

    def feed(self, chunk: bytes):
        self._buf.extend(chunk)
        frames = []
        while True:
            frame = self._try_pop()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _try_pop(self):
        buf = self._buf
        while True:
            i = buf.find(bytes([SOF]))
            if i < 0:
                buf.clear()
                return None
            if i:
                del buf[:i]
            if len(buf) < 4:
                return None
            length = buf[1]
            if length > PAYLOAD_MAX:
                self.bad_resync += 1
                del buf[:1]
                continue
            total = 4 + length + 1
            if len(buf) < total:
                return None
            if crc8(bytes(buf[1:4 + length])) != buf[4 + length]:
                self.bad_resync += 1
                del buf[:1]
                continue
            frame = (buf[2], buf[3], bytes(buf[4:4 + length]))
            del buf[:total]
            return frame


class Collector:
    """0x05 轨迹（按轮 × 组号分桶）+ TEXT 行（分轮）+ 链路统计。

    固件无限循环重扫（一轮 = 组 0..8 递增），同组跨轮的轨迹不能拼桶
    （实测暴露：拼接后 settle 变成 N 轮时长、稳态均值跨轮偏移）——
    组号回卷（g < 上一帧组号）即新一轮，开新桶。
    """

    def __init__(self) -> None:
        self.rounds = []            # 每轮：{g: [(sp, duty, actual, phase)]}
        self.text_rounds = []       # 每轮：[line]
        self.n_motor = 0
        self.n_text = 0
        self.bad_resync = 0
        self.round_done = 0         # round done 行出现次数（= 完整轮数）
        self._last_g = -1
        self._start_round()

    def _start_round(self) -> None:
        """开新轮（0x05 桶与 TEXT 桶同开——固件序：END → header → g0 帧）。"""
        self.rounds.append({g: [] for g in range(N_GROUPS)})
        self.text_rounds.append([])
        self._last_g = -1

    @property
    def traces(self):
        """最后一个完整轮（含 round done 的文本轮）的轨迹桶。

        0x05 桶与 TEXT 桶在 _start_round 同步开轮（rounds[i] ↔
        text_rounds[i] 同号）——完整文本轮的同号轨迹轮即该轮 0x05 全量。
        末尾的部分轮（采集在其中途收工）如实跳过，不拼桶。
        """
        for i in range(len(self.text_rounds) - 1, -1, -1):
            if any(ln.startswith("PIDSCAN round done")
                   for ln in self.text_rounds[i]):
                if i < len(self.rounds):
                    return self.rounds[i]
        return self.rounds[-1] if self.rounds else {}

    def on_frame(self, typ, seq, payload):
        if typ == TYPE_MOTOR and len(payload) == 16:
            g = payload[0]
            sp, duty, actual = struct.unpack("<3f", payload[2:14])
            phase = payload[14]
            fault = payload[15]
            if g < N_GROUPS:
                if g < self._last_g:        # 组号回卷 → 新一轮（兜底分轮信号）
                    self._start_round()
                self._last_g = g
                if fault == 0:
                    self.rounds[-1][g].append((sp, duty, actual, phase))
            self.n_motor += 1
        elif typ == TYPE_TEXT:
            self.n_text += 1
            line = payload.decode("ascii", "replace")
            self.text_rounds[-1].append(line)   # round done 行先归当轮
            if line.startswith("PIDSCAN round done"):
                self.round_done += 1
                self._start_round()             # 然后开新轮（与 0x05 回卷互斥）
            print("[text] " + line)


def group_metrics(trace):
    """host 独立指标（与固件 scen.c 同口径；trace = 该组按序的全部拍）。

    返回 (settle_s, overshoot_pct, ss_err_pct, ok)；ok=0 = 不收敛/数据不足。
    """
    hi_part = [a for (sp, duty, a, ph) in trace if ph == PH_HI and sp == STEP_HI]
    if len(hi_part) < 64:
        return (-1.0, -1.0, -1.0, 0)
    band = STEP_HI * TOL_PCT / 100.0
    peak = max(hi_part)
    last_out = -1
    for t, a in enumerate(hi_part):
        if a < STEP_HI - band or a > STEP_HI + band:
            last_out = t
    tail = hi_part[len(hi_part) // 2:]
    mean = sum(tail) / len(tail)
    ss = abs(mean - STEP_HI) / STEP_HI * 100.0
    ov = max(0.0, (peak - STEP_HI) / (STEP_HI - STEP_LO) * 100.0)
    n = len(hi_part)
    if last_out >= 0 and last_out < n - 1 and ss <= TOL_PCT:
        settle = (last_out + 1) * 0.002    # 2ms 拍（POLL_US 单源）
        return (settle, ov, ss, 1)
    return (-1.0, ov, ss, 0)


def parse_fw_line(line):
    """固件组行 → (g, kp, ki, settle, ov, se)；不可解返回 None。

    行格式（固件 exp_motor_pidscan_scen.c 单源契约）：
    "G<k> kp<kp> ki<ki> st<st>s ov<ov>% se<se>%[ *| !]"
    """
    try:
        if not line.startswith("G"):
            return None
        parts = line.split()
        g = int(parts[0][1:])
        kp = float(parts[1][2:])
        ki = float(parts[2][2:])
        settle = float(parts[3][2:-1])
        ov = float(parts[4][2:-1])
        se = float(parts[5][2:-1])
        star = line.rstrip().endswith("*")
        return (g, kp, ki, settle, ov, se, star)
    except (ValueError, IndexError):
        return None


# ─────────────────────── 中英成对文案（--lang） ───────────────────────

#
# 所有面向用户打印行都走这张表（表格列名 / 固件 TEXT 原文保留符号，不进表）；
# 新增打印行必须同时给 zh/en 两条，缺一条即 KeyError（不允许单语静默漏出）。
STRINGS = {
    "zh": {
        "mode_sim": "模式：sim — 本机合成帧源（零外部依赖；虚拟 %d 轮 × 27s，按帧计数推进）",
        "mode_qemu": "模式：qemu — TCP 串口桥 %s:%d（需 Embedded run_qemu.bat 已在跑）",
        "mode_real": "模式：real — 物理串口 %s @ %d bps",
        "capture_sim": "采集：虚拟窗 %.0fs = %d 轮（设计一轮 27s；sim 不等墙钟）...",
        "capture_tcp": "已连接 %s:%d — 采集 %.0fs（设计一轮 27s；QEMU 实测拍率 ~231Hz ≈ 60s/轮）...",
        "capture_serial": "已打开 %s — 采集 %.0fs（设计一轮 27s；实机 500Hz ≈ 27s/轮）...",
        "progress": "  ... %d 帧（0x05 %d，TEXT %d，完整轮 %d）",
        "captured": "采集完成：0x05 %d 帧，TEXT %d 行，坏重同步 %d 次，完整轮 %d",
        "host_title": "\n=== host 独立重算对比表（最后一轮 0x05 轨迹；"
                      "settle=到带时间，ov=超调，se=稳态误差）===",
        "host_best": "host 最优组：G%d（score = settle + ov/100 + se/100 最小）",
        "fw_title": "\n=== 固件 TEXT 对比表（0x7F 汇总帧，最后一轮）===",
        "fail_title": "\n=== FAIL x%d ===",
        "pass": "\nPASS exp_motor_pidscan 调参对比表全断言（%d 帧；9 组收敛 ±3%%；"
                "固件/host 双表一致；最优 G%d）",
        "link_frames": "链路：0x05 帧数 %d < 9000（一轮 13500）",
        "link_resync": "链路：坏重同步 %d > 50",
        "cover": "组覆盖：最后一轮仅 %d/9 组有轨迹",
        "converge": "收敛：G%d 未收敛（host: st=%.3f se=%.2f%%）",
        "no_round_done": "对比表：未见 round done 行（采集窗不足一轮？）",
        "fw_line_bad": "对比表：固件组行不可解：%s",
        "fw_rows": "对比表：固件组行 %d < 9",
        "fw_star": "对比表：最优 '*' 标注数 %d != 1",
        "fw_star_round": "对比表：第 %d 轮 '*' 数 %d != 1",
        "fw_star_rounds": "对比表：多轮最优组不一致：%s",
        "dual": "双表对照：G%d 固件(st=%.3f ov=%.1f se=%.2f) vs host(st=%.3f ov=%.1f se=%.2f)",
        "dual_best": "双表对照：host 最优 G%d 与固件 '*' 标注 %s 不一致",
        "connect_fail": "连接 %s:%d 失败（%.1fs 内未连上；QEMU 串口桥未起？先跑 Embedded run_qemu.bat）",
        "serial_required": "real 档必须给 --serial COMx（或 run.bat COM5）",
        "serial_missing": "real 档需要 pyserial（pip install pyserial）",
        "serial_open": "打开串口 %s 失败：%s",
    },
    "en": {
        "mode_sim": "mode: sim -- local synthetic frame source (no external deps; "
                    "%d virtual rounds x 27s, frame-count driven)",
        "mode_qemu": "mode: qemu -- TCP serial bridge %s:%d "
                     "(Embedded run_qemu.bat must already be running)",
        "mode_real": "mode: real -- physical serial port %s @ %d bps",
        "capture_sim": "capturing: virtual window %.0fs = %d rounds "
                       "(one round = 27s; sim does not wait on wall clock)...",
        "capture_tcp": "connected %s:%d -- capturing %.0fs (one round = 27s; QEMU tick "
                       "rate ~231Hz => ~60s/round)...",
        "capture_serial": "opened %s -- capturing %.0fs "
                          "(one round = 27s; hardware 500Hz => ~27s/round)...",
        "progress": "  ... %d frames (0x05 %d, TEXT %d, complete rounds %d)",
        "captured": "captured: 0x05 %d frames, TEXT %d lines, bad resyncs %d, complete rounds %d",
        "host_title": "\n=== host independent recompute (last round 0x05 trace; "
                      "settle=time-to-band, ov=overshoot, se=steady-state error) ===",
        "host_best": "host best group: G%d (min score = settle + ov/100 + se/100)",
        "fw_title": "\n=== firmware TEXT comparison table (0x7F frames, last round) ===",
        "fail_title": "\n=== FAIL x%d ===",
        "pass": "\nPASS exp_motor_pidscan comparison table: all assertions "
                "(%d frames; 9 groups settled within +-3%%; firmware/host tables agree; "
                "best G%d)",
        "link_frames": "link: 0x05 frames %d < 9000 (one round = 13500)",
        "link_resync": "link: bad resyncs %d > 50",
        "cover": "coverage: last round has only %d/9 groups in trace",
        "converge": "converge: G%d did not settle (host: st=%.3f se=%.2f%%)",
        "no_round_done": "table: no round-done line (capture window shorter than one round?)",
        "fw_line_bad": "table: unparsable firmware group line: %s",
        "fw_rows": "table: firmware group rows %d < 9",
        "fw_star": "table: best '*' marks %d != 1",
        "fw_star_round": "table: round %d has %d '*' marks != 1",
        "fw_star_rounds": "table: best group differs across rounds: %s",
        "dual": "dual-table: G%d firmware(st=%.3f ov=%.1f se=%.2f) vs "
                "host(st=%.3f ov=%.1f se=%.2f)",
        "dual_best": "dual-table: host best G%d != firmware '*' marks %s",
        "connect_fail": "cannot connect to %s:%d (not connected within %.1fs; is the QEMU "
                        "serial bridge up? run Embedded run_qemu.bat)",
        "serial_required": "real source needs --serial COMx (or run.bat COM5)",
        "serial_missing": "real source needs pyserial (pip install pyserial)",
        "serial_open": "cannot open serial port %s: %s",
    },
}


class Msgs:
    """按 --lang 取中英成对文案，并按 % 参数套值。"""

    def __init__(self, lang: str):
        self._table = STRINGS[lang]

    def __call__(self, key: str, *args) -> str:
        """key → 该语言的格式化文本（args 顺序与模板占位一致）。"""
        text = self._table[key]
        return text % args if args else text


def assertion_fails(col, msg):
    """A/B/C/D 四组断言（与数据源无关：三档走同一套判据，阈值不许档位特调）。

    返回 (fails, best_host)；fails 为空 = 全过。
    """
    fails = []

    # ---- 断言 A：链路健康 ----
    if col.n_motor < 9000:
        fails.append(msg("link_frames", col.n_motor))
    if col.bad_resync > 50:
        fails.append(msg("link_resync", col.bad_resync))
    covered = [g for g in range(N_GROUPS) if len(col.traces[g]) >= 64]
    if len(covered) < N_GROUPS:
        fails.append(msg("cover", len(covered)))

    # ---- host 独立指标（B）+ 双表对照（D）：取最后一轮（多轮重扫不拼桶）----
    print(msg("host_title"))
    host = {}
    for g in range(N_GROUPS):
        settle, ov, ss, ok = group_metrics(col.traces[g])
        host[g] = (settle, ov, ss, ok)
        mark = "" if ok else " !"
        print("G%d st%.3fs ov%.1f%% se%.2f%%%s" % (g, settle, ov, ss, mark))
        if not ok:
            fails.append(msg("converge", g, settle, ss))
    ok_groups = [g for g in range(N_GROUPS) if host[g][3]]
    best_host = -1
    if ok_groups:
        best_host = min(ok_groups,
                        key=lambda g: host[g][0] + host[g][1] / 100.0 + host[g][2] / 100.0)
    print(msg("host_best", best_host))

    # ---- 固件 TEXT 对比表（C + D）：每轮独立断言（多轮 '*' 各恰 1 且同组）----
    print(msg("fw_title"))
    if not col.round_done:
        fails.append(msg("no_round_done"))
    # 取最后一个完整文本轮（含 round done 行的那轮）
    complete = [rl for rl in col.text_rounds
                if any(ln.startswith("PIDSCAN round done") for ln in rl)]
    if not complete:
        complete = [col.text_rounds[-1]] if col.text_rounds else [[]]
    last = complete[-1]
    for ln in last:
        print("  " + ln)
    fw = {}
    n_star = 0
    for ln in last:
        if not ln.startswith("G"):
            continue
        r = parse_fw_line(ln)
        if r is None:
            fails.append(msg("fw_line_bad", ln))
            continue
        g, kp, ki, settle, ov, se, star = r
        fw[g] = (settle, ov, se)
        if star:
            n_star += 1
    if len(fw) < N_GROUPS:
        fails.append(msg("fw_rows", len(fw)))
    if n_star != 1:
        fails.append(msg("fw_star", n_star))
    # 每轮星唯一且同组（多轮一致性）
    for i, rl in enumerate(complete):
        stars = [ln for ln in rl if ln.startswith("G") and ln.rstrip().endswith("*")]
        if len(stars) != 1:
            fails.append(msg("fw_star_round", i + 1, len(stars)))
    star_groups = set()
    for rl in complete:
        for ln in rl:
            if ln.startswith("G") and ln.rstrip().endswith("*"):
                star_groups.add(parse_fw_line(ln)[0])
    if len(star_groups) > 1:
        fails.append(msg("fw_star_rounds", sorted(star_groups)))
    for g, (settle, ov, se) in fw.items():
        if g not in host or not host[g][3]:
            continue
        if (abs(settle - host[g][0]) > 0.006 or abs(ov - host[g][1]) > 0.6
                or abs(se - host[g][2]) > 0.3):
            fails.append(msg("dual", g, settle, ov, se,
                             host[g][0], host[g][1], host[g][2]))
    if best_host >= 0 and best_host not in star_groups:
        fails.append(msg("dual_best", best_host, sorted(star_groups)))
    return fails, best_host


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="exp_motor_pidscan 调参对比表查看器（0x05 轨迹 + TEXT 汇总双表）")
    ap.add_argument("--source", choices=("sim", "qemu", "real"), default="sim",
                    help="数据源：sim=本机合成源（缺省，零外部依赖）/ qemu=TCP 串口桥 "
                         "/ real=物理串口")
    ap.add_argument("--lang", choices=("zh", "en"), default="zh",
                    help="面向用户打印语言（缺省 zh；表格列名保留符号）")
    ap.add_argument("--host", default="127.0.0.1", help="qemu 档 TCP 主机")
    ap.add_argument("--port", type=int, default=5565, help="qemu 档 TCP 端口")
    ap.add_argument("--serial", default=None, help="real 档串口号（如 COM5）")
    ap.add_argument("--baud", type=int, default=115200, help="real 档波特率")
    ap.add_argument("--seconds", type=float, default=75.0,
                    help="采集窗 s（设计一轮 %.0fs；sim 档=虚拟窗（按 %.0fs/轮换算轮数），"
                         "qemu 档实测 ~60s/轮，默认 75 收满一轮+裕量）" % (ROUND_S, ROUND_S))
    ap.add_argument("--quiet", action="store_true", help="抑制逐帧计数打印")
    args = ap.parse_args(argv)
    msg = Msgs(args.lang)

    # ---- 选源（三档显式；sim 缺省，绝不自动连 QEMU）----
    try:
        src = open_source(args, msg)
    except SourceError as exc:
        print("FAIL " + msg(exc.key, *exc.fmt_args))
        return 2

    col = collect(src, args, msg)
    fails, best_host = assertion_fails(col, msg)

    # ---- 结论 ----
    if fails:
        print(msg("fail_title", len(fails)))
        for f in fails:
            print("  " + f)
        return 1
    print(msg("pass", col.n_motor, best_host))
    return 0


# ─────────────────────── 数据源三档（sim / qemu / real） ───────────────────────


class SourceError(Exception):
    """选源/连接失败（携带文案 key + 格式参数，由 main 按 --lang 打 FAIL 并 exit 2）。"""

    def __init__(self, key: str, *fmt_args):
        super().__init__(key)
        self.key = key
        self.fmt_args = fmt_args


class TcpSource:
    """qemu 档：TCP 串口桥（QEMU -serial tcp:host:port,server，wait 模式）。

    read() 三态：bytes = 新数据；None = 本轮无数据（超时）；b"" = 对端关闭。
    """

    def __init__(self, host: str, port: int, connect_timeout_s: float = CONNECT_TIMEOUT_S):
        self._host = host
        self._port = port
        self._timeout_s = connect_timeout_s
        self._sock = None

    def open(self) -> None:
        """阻塞连接（≤ connect_timeout_s）；连不上抛 SourceError（带实际耗时）。"""
        t0 = time.time()
        deadline = t0 + self._timeout_s
        while True:
            remain = deadline - time.time()
            if remain <= 0:
                raise SourceError("connect_fail", self._host, self._port, time.time() - t0)
            try:
                self._sock = socket.create_connection(
                    (self._host, self._port), timeout=min(2.0, remain))
                self._sock.settimeout(0.5)
                return
            except OSError:
                time.sleep(min(0.2, max(0.0, deadline - time.time())))

    def read(self):
        """读一段字节（三态语义见类注释）。"""
        try:
            return self._sock.recv(READ_CHUNK)
        except socket.timeout:
            return None

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()


class SerialSource:
    """real 档：物理串口（pyserial；缺包或打不开抛 SourceError → exit 2）。

    波特率/串口号来自 --baud/--serial（115200 8N1，与固件 console 口一致）。
    """

    def __init__(self, port: str, baud: int):
        self._port = port
        self._baud = baud
        self._ser = None

    def open(self) -> None:
        try:
            import serial   # 延迟导入：qemu/sim 档不需要 pyserial
        except ImportError:
            raise SourceError("serial_missing")
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.5)
        except (OSError, ValueError) as exc:
            raise SourceError("serial_open", self._port, exc)

    def read(self):
        """读一段字节：None = 本轮无数据（超时），b"" = 口已失效。"""
        try:
            n = self._ser.in_waiting
            data = self._ser.read(n or 1)
        except (OSError, ValueError):
            return b""
        return data or None

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()


def open_source(args, msg):
    """按 --source 选并打开数据源，返回已就绪的源对象（同时打印模式/采集行）。

    失败抛 SourceError（sim 档不会失败——本机合成无外部依赖）。
    """
    if args.source == "sim":
        rounds = rounds_for_seconds(args.seconds)
        print(msg("mode_sim", rounds))
        print(msg("capture_sim", args.seconds, rounds))
        return SimPidscanSource(rounds=rounds)
    if args.source == "qemu":
        print(msg("mode_qemu", args.host, args.port))
        src = TcpSource(args.host, args.port)
        src.open()
        print(msg("capture_tcp", args.host, args.port, args.seconds))
        return src
    if not args.serial:
        raise SourceError("serial_required")
    print(msg("mode_real", args.serial, args.baud))
    src = SerialSource(args.serial, args.baud)
    src.open()
    print(msg("capture_serial", args.serial, args.seconds))
    return src


def collect(src, args, msg):
    """采集一轮（round done 后 1s 排干即收工，不硬等满窗），返回 Collector。"""
    dec = FrameDecoder()
    col = Collector()
    t_end = time.time() + args.seconds
    drain_until = None
    while time.time() < t_end:
        chunk = src.read()
        if chunk is None:                  # 无新数据（超时）
            if drain_until is not None and time.time() > drain_until:
                break
            continue
        if not chunk:                      # 对端关闭 / sim 流尽
            break
        for typ, seq, payload in dec.feed(chunk):
            col.on_frame(typ, seq, payload)
        if col.round_done and drain_until is None:
            drain_until = time.time() + DRAIN_S
        if not args.quiet and col.n_motor % 2000 == 0 and col.n_motor:
            print(msg("progress", col.n_motor + col.n_text, col.n_motor,
                      col.n_text, col.round_done))
    src.close()
    col.bad_resync = dec.bad_resync
    print(msg("captured", col.n_motor, col.n_text, col.bad_resync, col.round_done))
    return col


if __name__ == "__main__":
    sys.exit(main())
