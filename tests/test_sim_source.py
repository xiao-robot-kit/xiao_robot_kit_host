"""SimImuSource 合成流一致性测试（纯 Python，无 Qt/串口依赖，headless 可跑）。

链路：SimImuSource（sway10_2s 预设，默认 100Hz）按 20ms 轮询分片合成 3s 字节流
      → FrameDecoder 解码 → 断言帧数 / 丢帧 / 帧类型 / roll 摆幅峰峰值。
口径：帧数 ≥ 250（3s@100Hz 理论 300）；丢帧 = 0（seq 连续，含 0..255 回绕口径）；
      roll 峰峰值 = 2×设定摆幅（±10%）；帧类型全为 IMU_SAMPLE。
      roll 重建公式 = atan2(ay, hypot(ax, az))（由合成运动模型反解，见 sim_source 模块注）。

跑法：python tests/test_sim_source.py（exit 0 = 全过）
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from frame import FrameDecoder, decode_sample, TYPE_IMU_SAMPLE  # noqa: E402
from sim_source import SIM_PRESETS, SimImuSource  # noqa: E402

DURATION_S = 3.0     # 合成时长（s）
POLL_S = 0.02        # 轮询分片（s；模拟 _SimWorker 20ms 节奏）
MIN_FRAMES = 250     # 3s@100Hz 理论 300，下限留余量
PP_TOL = 0.10        # 峰峰值容差（比例，对 2×摆幅）
STILL_PROBE_S = 0.5  # 静止预设探测时长（s）


def collect(cfg, seconds: float, poll_s: float):
    """合成→解码一轮，返回 (frames, drop)。

    入参：cfg=SimImuCfg；seconds/poll_s 单位秒。
    出参：frames=[(type, seq, payload), ...]；drop=seq 回绕口径丢帧数。
    """
    src = SimImuSource(cfg)
    dec = FrameDecoder()
    frames = []
    last_seq = -1
    drop = 0
    n_poll = int(round(seconds / poll_s))
    for k in range(1, n_poll + 1):
        for f in dec.feed(src.poll(k * poll_s)):
            frames.append(f)
    for _typ, seq, _payload in frames:
        if last_seq >= 0:
            drop += (seq - (last_seq + 1)) & 0xFF
        last_seq = seq
    return frames, drop


def main() -> int:
    fails = []
    cfg = SIM_PRESETS["sway10_2s"]
    frames, drop = collect(cfg, DURATION_S, POLL_S)

    # 1) 帧数
    if len(frames) < MIN_FRAMES:
        fails.append(f"frames={len(frames)} < {MIN_FRAMES}")

    # 2) 丢帧 = 0（seq 连续，含 0..255 回绕）
    if drop != 0:
        fails.append(f"drop={drop} != 0")

    # 3) 帧类型全为 IMU_SAMPLE
    types = set(t for t, _, _ in frames)
    if types != {TYPE_IMU_SAMPLE}:
        fails.append(f"frame types {types!r} != {{IMU_SAMPLE}}")

    # 4) roll 摆幅峰峰值 vs 设定（±10%）
    rolls = [math.degrees(math.atan2(v[1], math.hypot(v[0], v[2])))
             for v in (decode_sample(p) for _, _, p in frames)]
    pp_deg = max(rolls) - min(rolls)
    want_pp = 2.0 * cfg.roll_amp_deg
    if abs(pp_deg - want_pp) > PP_TOL * want_pp:
        fails.append(f"roll pp {pp_deg:.3f}° != {want_pp}°（±{PP_TOL * 100:.0f}%）")

    # 5) 静止预设自证：ax/ay≈0、az≈+1g（静止重力模型口径）
    frames2, _ = collect(SIM_PRESETS["still"], STILL_PROBE_S, POLL_S)
    v = decode_sample(frames2[-1][2])
    if abs(v[0]) > 1e-6 or abs(v[1]) > 1e-6 or abs(v[2] - 1.0) > 1e-6:
        fails.append(f"still preset ax/ay/az = {v[:3]!r}")

    print(f"== test_sim_source: frames={len(frames)}, drop={drop}, "
          f"roll_pp={pp_deg:.3f}° (set {want_pp}°), still az={v[2]:.3f}g, "
          f"{len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
