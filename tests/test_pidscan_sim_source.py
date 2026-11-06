#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sim_pidscan 合成帧源契约测试（纯 Python 零依赖，headless 可跑）。

覆盖（backend/sim_pidscan.py = scope_pidscan.py --source sim 的数据源）：
  1) 轮结构：一轮 13500 帧 0x05 + 12 行 TEXT（表头 + 9 组行 + BEST + round done），
     坏重同步 0、seq 无缺口（TEXT/0x05 共用计数器）；
  2) 组结构：每组 1500 帧 —— 前 500 拍 phase=0/setpoint=30，后 1000 拍
     phase=1/setpoint=90，slot=组号、fault=0；高段首拍与低段终值连续（≈30）；
  3) TEXT 对比表：9 组行可解、'*' 恰 1 行、TEXT 三指标与 host 后置重算一致
     （容差与 scope_pidscan 断言 D 同口径）、最优组与 host 最优同组；
  4) 确定性：同参数两次生成逐字节相同（按帧计数推进，无墙钟/随机）；
  5) rounds_for_seconds：--seconds 虚拟窗 → 轮数（至少 1 轮）。

跑法：python tests/test_pidscan_sim_source.py（exit 0 = 全过）
"""

import contextlib
import io
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from scope_pidscan import (Collector, FrameDecoder, group_metrics,  # noqa: E402
                           parse_fw_line)
from sim_pidscan import (GROUP_TICKS, GRID, HOLD_HI, HOLD_LO, N_GROUPS,  # noqa: E402
                         PH_HI, PH_LO, ROUND_S, STEP_HI, STEP_LO,
                         SimPidscanSource, rounds_for_seconds)

N_FRAMES_ROUND = N_GROUPS * GROUP_TICKS      # 13500
N_TEXT_ROUND = 1 + N_GROUPS + 1 + 1          # 表头 + 9 组行 + BEST + round done
TOL_SETTLE_S = 0.006                         # 与 scope_pidscan 断言 D 同容差
TOL_OV_PCT = 0.6
TOL_SE_PCT = 0.3


def drain(src):
    """把源的整条流喂进解码器，返回 (解码帧列表, 坏重同步数, 原始字节)。"""
    dec = FrameDecoder()
    frames = []
    raw = bytearray()
    while True:
        chunk = src.read()
        if not chunk:
            break
        raw += chunk
        frames.extend(dec.feed(chunk))
    return frames, dec.bad_resync, bytes(raw)


def main() -> int:
    fails = []

    # ---- 5) 虚拟窗 → 轮数 ----
    for seconds, expect in ((0.5, 1), (8.0, 1), (ROUND_S, 1), (ROUND_S + 0.1, 2), (75.0, 3)):
        got = rounds_for_seconds(seconds)
        if got != expect:
            fails.append(f"rounds_for_seconds({seconds}) = {got} != {expect}")

    # ---- 1) 轮结构 ----
    frames, bad, raw = drain(SimPidscanSource(rounds=1))
    motors = [f for f in frames if f[0] == 0x05]
    texts = [f[2].decode("ascii") for f in frames if f[0] == 0x7F]
    if len(motors) != N_FRAMES_ROUND:
        fails.append(f"0x05 帧数 {len(motors)} != {N_FRAMES_ROUND}")
    if len(texts) != N_TEXT_ROUND:
        fails.append(f"TEXT 行数 {len(texts)} != {N_TEXT_ROUND}")
    if bad != 0:
        fails.append(f"坏重同步 {bad} != 0")
    last_seq = -1
    for typ, seq, _payload in frames:
        if last_seq >= 0 and ((seq - (last_seq + 1)) & 0xFF) != 0:
            fails.append(f"seq 缺口：{(last_seq, seq)}（type=0x{typ:02x}）")
            break
        last_seq = seq
    if not texts or texts[0] != "PIDSCAN 3x3: g kp ki settle_s ov% se%":
        fails.append(f"表头行异常：{texts[0] if texts else '<无>'}")
    if not texts or not texts[-1].startswith("PIDSCAN round done"):
        fails.append(f"末行不是 round done：{texts[-1] if texts else '<无>'}")

    # ---- 2) 组结构（逐组 1500 拍：低段 500 + 高段 1000）----
    payloads = [p for _t, _s, p in motors]
    for g in range(N_GROUPS):
        rows = payloads[g * GROUP_TICKS:(g + 1) * GROUP_TICKS]
        if any(p[0] != g for p in rows):
            fails.append(f"G{g} slot 域 != 组号")
        if any(p[1] != 2 for p in rows):
            fails.append(f"G{g} kind 域 != 2（ENC_CLOSED）")
        if any(p[15] != 0 for p in rows):
            fails.append(f"G{g} fault 域 != 0")
        lo = rows[:HOLD_LO]
        hi = rows[HOLD_LO:]
        if any(p[14] != PH_LO for p in lo):
            fails.append(f"G{g} 低段 phase 域 != 0")
        if any(p[14] != PH_HI for p in hi):
            fails.append(f"G{g} 高段 phase 域 != 1")
        sp_lo = {round(struct.unpack("<f", p[2:6])[0], 6) for p in lo}
        sp_hi = {round(struct.unpack("<f", p[2:6])[0], 6) for p in hi}
        if sp_lo != {STEP_LO}:
            fails.append(f"G{g} 低段 setpoint {sorted(sp_lo)} != {{{STEP_LO}}}")
        if sp_hi != {STEP_HI}:
            fails.append(f"G{g} 高段 setpoint {sorted(sp_hi)} != {{{STEP_HI}}}")
        first_hi = struct.unpack("<f", hi[0][10:14])[0]
        last_lo = struct.unpack("<f", lo[-1][10:14])[0]
        if abs(first_hi - STEP_LO) > 0.5 or abs(last_lo - STEP_LO) > 0.5:
            fails.append(f"G{g} 段间不连续：低段末 {last_lo:.2f} → 高段首 {first_hi:.2f}")

    # ---- 3) TEXT 对比表 ↔ host 后置重算（断言 D 同口径）----
    col = Collector()
    with contextlib.redirect_stdout(io.StringIO()):   # Collector 逐行打 [text]，测试不掺噪
        for typ, seq, payload in frames:
            col.on_frame(typ, seq, payload)
    if col.round_done != 1:
        fails.append(f"round done 计数 {col.round_done} != 1")
    last = [rl for rl in col.text_rounds
            if any(ln.startswith("PIDSCAN round done") for ln in rl)][-1]
    fw = {}
    n_star = 0
    for ln in last:
        if not ln.startswith("G"):
            continue
        parsed = parse_fw_line(ln)
        if parsed is None:
            fails.append(f"组行不可解：{ln}")
            continue
        g, kp, ki, settle, ov, se, star = parsed
        if (kp, ki) != GRID[g]:
            fails.append(f"G{g} 网格值 {(kp, ki)} != {GRID[g]}")
        fw[g] = (settle, ov, se)
        n_star += 1 if star else 0
    if len(fw) != N_GROUPS:
        fails.append(f"固件组行 {len(fw)} != {N_GROUPS}")
    if n_star != 1:
        fails.append(f"最优 '*' 标注数 {n_star} != 1")
    host = {}
    for g in range(N_GROUPS):
        host[g] = group_metrics(col.traces[g])
        if not host[g][3]:
            fails.append(f"G{g} host 重算未收敛：{host[g]}")
            continue
        if g in fw:
            fs, fo, fse = fw[g]
            hs, ho, hse, _ok = host[g]
            if (abs(fs - hs) > TOL_SETTLE_S or abs(fo - ho) > TOL_OV_PCT
                    or abs(fse - hse) > TOL_SE_PCT):
                fails.append(f"双表不一致 G{g}：固件{(fs, fo, fse)} vs host{(hs, ho, hse)}")
    ok_groups = [g for g in range(N_GROUPS) if host[g][3]]
    best_host = min(ok_groups, key=lambda g: host[g][0] + host[g][1] / 100 + host[g][2] / 100)
    stars = [ln for ln in last if ln.startswith("G") and ln.rstrip().endswith("*")]
    if len(stars) == 1 and parse_fw_line(stars[0])[0] != best_host:
        fails.append(f"最优组不一致：固件 {parse_fw_line(stars[0])[0]} vs host {best_host}")

    # ---- 4) 确定性（两次生成逐字节相同时刻同序）----
    _f2, _b2, raw2 = drain(SimPidscanSource(rounds=1))
    if raw != raw2:
        fails.append("同参数两次生成字节流不同（非确定性）")

    print(f"== test_pidscan_sim_source: 轮结构/组结构/TEXT双表/确定性/"
          f"轮数换算, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
