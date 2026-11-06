"""exp2 调零温补链路测试（纯 Python，无 Qt/串口依赖，headless 可跑）。

覆盖（exp2_bias 页的 backend 侧一致性）：
  1) decode_cal / encode_cal 契约：19 值往返逐分量相等；长度错 ValueError；
     整帧经 FrameDecoder 解码类型/序号正确
  2) SimCalSource 合成流：开机时序（TEXT 横幅 → 标定窗静默 → 0x02 流）、
     帧数、seq 连续（TEXT+CAL 共用计数器）、raw 块 = 真值+注入零偏+温漂、
     deb 块 ≈ 真值（残差 = 阶梯滞后 ≤ step_c·k）、bias 块 = 线性温补模型、
     temp 块 = 起始温度 + 斜率·t
  3) 标定窗镜像：窗内不出 0x02 帧（5 s@100Hz 后才起流）；窗满发
     OK calib / OK stream 横幅

跑法：python tests/test_sim_cal_source.py（exit 0 = 全过）
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from frame import (FrameDecoder, decode_cal, encode_cal,  # noqa: E402
                   TYPE_IMU_CAL, TYPE_TEXT)
from sim_source import SIM_CAL_PRESETS, SimCalSource  # noqa: E402

DURATION_S = 12.0      # 合成时长（s）：5 s 标定窗 + 7 s 流
POLL_S = 0.02          # 轮询分片（s；模拟 _SimWorker 20ms 节奏）
TOL = 1e-6             # 浮点比较容差（合成值同机生成，纯舍入级）


def collect(seconds: float, poll_s: float):
    """合成→解码一轮，返回 (frames, drop)。

    frames=[(type, seq, payload), ...]；drop = seq 回绕口径丢帧数
    （TEXT 与 CAL 共用计数器——镜像固件 exp2_main.c 单一 s_seq）。
    """
    src = SimCalSource(SIM_CAL_PRESETS["exp2_static"])
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
    cfg = SIM_CAL_PRESETS["exp2_static"]

    # 1) 编解码契约
    vals = tuple(round(0.125 * i, 3) for i in range(19))
    frame = encode_cal(7, vals)
    dec = FrameDecoder()
    got = dec.feed(frame)
    if len(got) != 1:
        fails.append(f"encode_cal→decode 帧数 {len(got)} != 1")
    else:
        typ, seq, payload = got[0]
        if typ != TYPE_IMU_CAL or seq != 7:
            fails.append(f"帧头 {(typ, seq)} != (0x02, 7)")
        if decode_cal(payload) != vals:
            fails.append(f"19 值往返不等：{decode_cal(payload)[:4]}…")
    for bad in (b"", b"\x00" * 75, b"\x00" * 77):
        try:
            decode_cal(bad)
            fails.append(f"decode_cal 长度 {len(bad)} 未抛 ValueError")
        except ValueError:
            pass

    # 2/3) 合成流
    frames, drop = collect(DURATION_S, POLL_S)
    texts = [(s, p.decode("ascii")) for t, s, p in frames if t == TYPE_TEXT]
    cals = [(s, decode_cal(p)) for t, s, p in frames if t == TYPE_IMU_CAL]

    if drop != 0:
        fails.append(f"seq 丢帧 {drop} != 0（TEXT+CAL 共用计数器应连续）")
    if len(texts) != 3:
        fails.append(f"TEXT 横幅 {len(texts)} 条 != 3：{[t[1] for t in texts]}")
    elif not (texts[0][1].startswith("OK sim: static Z_UP")
              and texts[1][1].startswith("OK calib 500 frames: bias g[")
              and texts[2][1] == "OK stream: TYPE_IMU_CAL 0x02 76B"):
        fails.append(f"横幅内容/次序错：{[t[1] for t in texts]}")

    exp_cal = int((DURATION_S - cfg.calib_frames / cfg.rate_hz) * cfg.rate_hz)
    if abs(len(cals) - exp_cal) > 2:      # 轮询分片舍入容差
        fails.append(f"0x02 帧数 {len(cals)} != ~{exp_cal}")

    # 标定窗内静默：第一条 CAL 帧之前只有 3 条 TEXT（横幅在窗满那拍才发）
    first_cal_idx = next(i for i, (t, _s, _p) in enumerate(frames)
                         if t == TYPE_IMU_CAL)
    pre_types = [t for t, _s, _p in frames[:first_cal_idx]]
    if pre_types != [TYPE_TEXT, TYPE_TEXT, TYPE_TEXT]:
        fails.append(f"标定窗内非静默（前缀帧型 {pre_types}）")

    # 逐帧块一致性（首帧细查 + 末帧细查）
    if cals:
        t0 = (cfg.calib_frames + 1) / cfg.rate_hz   # 首条 0x02 的拍时刻
        temp0 = cfg.temp_start_c + cfg.temp_ramp_c_per_s * t0
        _s, v = cals[0]
        dt = temp0 - cfg.temp_start_c
        raw_exp = [cfg.bias[0] + cfg.accel_drift[0] * dt,
                   cfg.bias[1] + cfg.accel_drift[1] * dt,
                   1.0 + cfg.bias[2] + cfg.accel_drift[2] * dt,
                   cfg.bias[3] + cfg.gyro_drift[0] * dt,
                   cfg.bias[4] + cfg.gyro_drift[1] * dt,
                   cfg.bias[5] + cfg.gyro_drift[2] * dt]
        for i in range(6):
            if abs(v[i] - raw_exp[i]) > TOL:
                fails.append(f"首帧 raw[{i}] {v[i]} != {raw_exp[i]}（注入模型）")
        if abs(v[18] - temp0) > TOL:
            fails.append(f"首帧 temp {v[18]} != {temp0}")

        # deb = raw − bias_now：残差 ≤ 阶梯滞后（step_c·k）+ 舍入。
        # 真值 az = +1g：bias_ref 的重力轴已减 1g（镜像 imu_calib.c），补偿不吞重力
        step_lag = cfg.step_c * max(cfg.gyro_drift + cfg.accel_drift)
        truth = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        for i in range(6):
            if abs(v[6 + i] - truth[i]) > step_lag + TOL:
                fails.append(f"首帧 deb[{i}] {v[6 + i]} 偏真值超阶梯滞后"
                             f"（> {step_lag}）")
        # bias 块（payload 采样序 ax..gz，gx = v[15]）首拍必算：
        # bias_now = bias_ref + k·(T−T_ref)，线性斜坡下望远镜到
        # 注入值 + k·(T − temp_start)（T_ref 与 bias_ref 的窗内偏移相消）
        bias_gx_exp = cfg.bias[3] + cfg.gyro_drift[0] * (temp0 - cfg.temp_start_c)
        if abs(v[15] - bias_gx_exp) > 1e-3:
            fails.append(f"首帧 bias gx {v[15]} != 线性模型 {bias_gx_exp:.6f}")

        # 末帧：温升斜率走满 + deb 仍贴真值
        _s, v = cals[-1]
        t_last = (cfg.calib_frames + len(cals)) / cfg.rate_hz
        temp_last = cfg.temp_start_c + cfg.temp_ramp_c_per_s * t_last
        if abs(v[18] - temp_last) > 2 * cfg.rate_hz * POLL_S * \
                cfg.temp_ramp_c_per_s + TOL:
            fails.append(f"末帧 temp {v[18]} != {temp_last}")
        for i in range(6):
            if abs(v[6 + i] - truth[i]) > step_lag + TOL:
                fails.append(f"末帧 deb[{i}] {v[6 + i]} 偏真值超阶梯滞后")

    print(f"== test_sim_cal_source: 编解码/时序/帧数/raw注入/deb贴真值/"
          f"bias线性/temp斜率/seq连续, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
