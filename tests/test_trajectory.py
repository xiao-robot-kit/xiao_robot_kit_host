"""TrajectoryReconstructor 位移轨迹重建测试（exp5，纯 Python 无 Qt，headless 可跑）。

链路口径（任务卡 T-112 exp5）：
  1) 合成已知运动（静止→已知恒加速 1s→静止）：恒定倾斜姿态（roll 10°/pitch -5°，
     姿态恒定但非单位阵——四元数旋转 + 去重力必须真实做功，倾斜下重力漏进
     水平轴的误差可达 sinθ·g 量级）；断言位移量级与方向（±15%）。
     运动读法：恒加速 1s 后速度 0.5 m/s，「→静止」由 ZUPT 兜住（静止确认窗
     内速度硬清零）——确认窗延迟 v·Δ 是 ZUPT 机制的固有代价，容差覆盖。
  2) 合成正弦摆 QUAT 流：SimTrajSource 字节流 → FrameDecoder → (QUAT,IMU)
     配对 → 重建器，全链路含 0x04 帧编解码；断言峰值位移 ±15% + 方向纯净
     （y/z 分量）+ 周期回零有界（漂移不累积）。
  3) 静止段漂移有界：静止 5s + 注入 body 系常值零偏（5/-3/4 mg），断言
     漂移 < 阈值 且零偏被学习吸收。
  4) POSIX 参数校验：非法配置抛 ValueError。

跑法：python tests/test_trajectory.py（exit 0 = 全过）
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from frame import (FrameDecoder, TYPE_IMU_SAMPLE, TYPE_QUAT,  # noqa: E402
                   decode_quat, decode_sample, encode_quat)
from sim_source import SIM_TRAJ_PRESETS, SimTrajSource  # noqa: E402
from trajectory import (TrajCfg, TrajectoryReconstructor,  # noqa: E402
                        TRAJ_PRESETS, quat_to_matrix)

RATE_HZ = 100.0            # 合成流帧率（Hz）
DT_S = 1.0 / RATE_HZ       # 采样周期（s）
TOL = 0.15                 # 位移量级容差（比例，±15%）
GY = 9.80665               # 重力（m/s²，与 trajectory/sim 两默认对齐）
PULSE_A = 0.5              # 恒加速度（m/s²）
PULSE_T = 1.0              # 恒加速时长（s）
PULSE_WANT = 0.5 * PULSE_A * PULSE_T * PULSE_T   # 理想位移 ½aT² = 0.25 m
CROSS_TOL = 1e-6           # 非主轴分量绝对容差（m，方向纯净断言）
DRIFT_LIMIT_M = 0.005      # 静止段漂移上限（m）
BIAS_INJ = (0.005, -0.003, 0.004)   # 注入 body 系零偏（g）
BIAS_TOL = 0.0005          # 学习值与注入值偏差上限（g）
STILL_S = 5.0              # 静止段时长（s）
SWAY_PERIODS = 2           # 摆动整周期数
F32_EPS = 1e-6             # float32 编解码往返容差（分量级）


# —— 测试侧独立数基（与 trajectory/sim 实现解耦，交叉验证约定一致性）——

def rot_x(r):
    """绕 x 轴旋转矩阵（rad 入参）。"""
    c, s = math.cos(r), math.sin(r)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(p):
    """绕 y 轴旋转矩阵（rad 入参）。"""
    c, s = math.cos(p), math.sin(p)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def mat_mul(a, b):
    """3×3 矩阵乘。"""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def euler_quat(roll, pitch):
    """R = Rx(roll)·Ry(pitch) 对应四元数（Hamilton 积 q_x⊗q_y，w 在前）。

    测试侧自行推导（半角公式直乘），与 sim_source._model 的化简式互为独立实现。
    """
    c2r, s2r = math.cos(roll / 2.0), math.sin(roll / 2.0)
    c2p, s2p = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return (c2r * c2p, s2r * c2p, c2r * s2p, s2r * s2p)


def body_force(R, a_world):
    """世界系线加速度 → 机体比力（g）：f_b = R^T·(a_world + [0,0,g])/g。"""
    gw = (0.0, 0.0, GY)
    return [sum(R[j][i] * (a_world[j] + gw[j]) for j in range(3)) / GY
            for i in range(3)]


def run_pulse(rec):
    """恒定倾斜下的 静止0.5s→+a 1s→静止1s 合成流，返回末帧位置。"""
    roll, pitch = math.radians(10.0), math.radians(-5.0)
    R = mat_mul(rot_x(roll), rot_y(pitch))
    q = euler_quat(roll, pitch)
    n = int(2.5 * RATE_HZ)
    p = (0.0, 0.0, 0.0)
    for k in range(n):
        t = k * DT_S
        a_world = (PULSE_A if 0.5 <= t <= 1.5 else 0.0, 0.0, 0.0)
        p = rec.feed(*q, *body_force(R, a_world))
    return p


def run_sway_stream(rec):
    """SimTrajSource 摆动 QUAT+IMU 双流 → 解码 → 配对 → 重建。

    返回 (n_quat, n_imu, 帧型集合, quat 归一化最大偏差, 位置序列)。
    配对口径：QUAT 帧先到先存，IMU 帧到达时取最新四元数（与 bridge
    _on_traj_frame 相同的 hold-latest 口径）。
    """
    cfg = SIM_TRAJ_PRESETS["traj_sway"]
    src = SimTrajSource(cfg)
    dec = FrameDecoder()
    quat = None
    n_q = n_i = 0
    types = set()
    qnorm_err = 0.0
    n_tick = int(cfg.period_s * SWAY_PERIODS * cfg.rate_hz)
    for k in range(1, n_tick + 1):
        for typ, _seq, payload in dec.feed(src.poll(k / cfg.rate_hz)):
            types.add(typ)
            if typ == TYPE_QUAT:
                quat = decode_quat(payload)
                n_q += 1
                qnorm_err = max(qnorm_err, abs(
                    math.sqrt(sum(c * c for c in quat)) - 1.0))
            elif typ == TYPE_IMU_SAMPLE:
                ax, ay, az, *_ = decode_sample(payload)
                rec.feed(*quat, ax, ay, az)
                n_i += 1
    return n_q, n_i, types, qnorm_err, rec.positions()


def run_still(rec):
    """恒定倾斜 + 注入零偏的静止流（5s），返回 (末位置, 学习零偏)。"""
    roll, pitch = math.radians(10.0), math.radians(-5.0)
    R = mat_mul(rot_x(roll), rot_y(pitch))
    q = euler_quat(roll, pitch)
    n = int(STILL_S * RATE_HZ)
    p = (0.0, 0.0, 0.0)
    for _ in range(n):
        fb = [c + b for c, b in zip(body_force(R, (0.0, 0.0, 0.0)), BIAS_INJ)]
        p = rec.feed(*q, *fb)
    return p, rec.bias()


def main() -> int:
    fails = []

    # 1) 静止→恒加速 1s→静止：位移量级 ±15% + 方向（y/z 纯净）
    rec = TrajectoryReconstructor(TrajCfg(dt_s=DT_S))
    px, py, pz = run_pulse(rec)
    if not (PULSE_WANT * (1 - TOL) <= abs(px) <= PULSE_WANT * (1 + TOL)):
        fails.append(f"pulse |x|={abs(px):.4f} 超出 {PULSE_WANT}±{TOL * 100:.0f}%"
                     f"（{PULSE_WANT * (1 - TOL):.4f}..{PULSE_WANT * (1 + TOL):.4f}）")
    if px <= 0:
        fails.append(f"pulse 方向反了：x={px:.4f} 应为 +x（世界系 +a 方向）")
    if abs(py) > CROSS_TOL or abs(pz) > CROSS_TOL:
        fails.append(f"pulse 非主轴分量 y={py:.2e} z={pz:.2e} > {CROSS_TOL}")

    # 2) 正弦摆 QUAT 流（SimTrajSource 字节流全链路）
    cfg = SIM_TRAJ_PRESETS["traj_sway"]
    rec2 = TrajectoryReconstructor(TrajCfg(dt_s=DT_S))
    n_q, n_i, types, qnorm_err, ps = run_sway_stream(rec2)
    want_peak = 2.0 * cfg.amp_m          # 相对起点峰值（cos 相位：0→-2A→0）
    peak_x = max(abs(p[0]) for p in ps)
    if not (want_peak * (1 - TOL) <= peak_x <= want_peak * (1 + TOL)):
        fails.append(f"sway peak|x|={peak_x:.4f} 超出 {want_peak}±{TOL * 100:.0f}%")
    peak_y = max(abs(p[1]) for p in ps)
    peak_z = max(abs(p[2]) for p in ps)
    if peak_y > CROSS_TOL or peak_z > CROSS_TOL:
        fails.append(f"sway 方向不纯：|y|max={peak_y:.2e} |z|max={peak_z:.2e} > {CROSS_TOL}")
    final = ps[-1]
    if abs(final[0]) > TOL * want_peak:
        fails.append(f"sway 周期回零漂移 |x_final|={abs(final[0]):.4f} > {TOL * want_peak:.4f}")
    expect_ticks = int(cfg.period_s * SWAY_PERIODS * cfg.rate_hz)
    if n_q != expect_ticks or n_i != expect_ticks:
        fails.append(f"帧数 quat={n_q} imu={n_i} != {expect_ticks}（双流应 1:1）")
    if types != {TYPE_QUAT, TYPE_IMU_SAMPLE}:
        fails.append(f"帧型集合 {types!r} 应为 {{QUAT, IMU_SAMPLE}}")
    if qnorm_err > F32_EPS:
        fails.append(f"|q| 偏离 1 达 {qnorm_err:.2e} > {F32_EPS}")

    # 2b) 四元数约定交叉验证：sim 输出的 q ↔ 测试侧独立 euler_quat/矩阵
    roll, pitch = math.radians(cfg.roll_amp_deg), math.radians(cfg.pitch_amp_deg)
    q_ind = euler_quat(roll, 0.0)     # 同 roll、pitch=0 处（sim 内 pitch 正交相不可直取）
    R_ind = mat_mul(rot_x(roll), rot_y(0.0))
    R_rec = quat_to_matrix(*q_ind)
    m_err = max(abs(R_rec[i][j] - R_ind[i][j]) for i in range(3) for j in range(3))
    if m_err > 1e-12:
        fails.append(f"quat_to_matrix 与独立矩阵推导偏差 {m_err:.2e} > 1e-12")

    # 2c) QUAT 帧编解码往返（float32 精度内复原）
    qv = (0.7071068, 0.0, 0.7071068, 0.0)
    back = decode_quat(encode_quat(0xAB, *qv)[4:20])
    if max(abs(a - b) for a, b in zip(qv, back)) > F32_EPS:
        fails.append(f"quat 编解码往返偏差超 {F32_EPS}: {back!r}")

    # 3) 静止段漂移有界 + 零偏学习
    rec3 = TrajectoryReconstructor(TrajCfg(dt_s=DT_S))
    p_still, bias_learned = run_still(rec3)
    drift = math.sqrt(sum(c * c for c in p_still))
    if drift > DRIFT_LIMIT_M:
        fails.append(f"静止段漂移 {drift:.4f} m > {DRIFT_LIMIT_M} m")
    for got, want in zip(bias_learned, BIAS_INJ):
        if abs(got - want) > BIAS_TOL:
            fails.append(f"零偏学习 {bias_learned!r} 偏离注入值 {BIAS_INJ!r} 超 {BIAS_TOL}")

    # 4) POSIX 参数校验（非法配置必须抛 ValueError，不许静默）
    bad_cfgs = [
        ("dt_s<=0", dict(dt_s=0.0)),
        ("gravity<=0", dict(gravity_mps2=-1.0)),
        ("hp_cutoff<=0", dict(hp_cutoff_hz=0.0)),
        ("lp_cutoff<=0", dict(lp_cutoff_hz=-1.0)),
        ("still_threshold<=0", dict(still_threshold_g=0.0)),
        ("still_window<1", dict(still_window_frames=0)),
        ("still_damp=0", dict(still_damp_per_s=0.0)),
        ("move_damp>1", dict(move_damp_per_s=1.5)),
        ("bias_rate<=0", dict(bias_learn_rate_per_s=0.0)),
        ("max_speed<=0", dict(max_speed_mps=0.0)),
        ("pos_limit<=0", dict(pos_limit_m=-2.0)),
        ("positions_maxlen<1", dict(positions_maxlen=0)),
    ]
    for name, kw in bad_cfgs:
        try:
            TrajCfg(**kw)
        except ValueError:
            continue
        fails.append(f"TrajCfg({name}) 未抛 ValueError")
    try:
        TrajectoryReconstructor(cfg="not-a-cfg")
        fails.append("非 TrajCfg 入参未抛 ValueError")
    except ValueError:
        pass
    try:
        rec.feed(float("nan"), 0, 0, 1, 0, 0, 1)
        fails.append("NaN 输入未抛 ValueError")
    except ValueError:
        pass
    try:
        quat_to_matrix(0.0, 0.0, 0.0, 0.0)
        fails.append("零四元数未抛 ValueError")
    except ValueError:
        pass

    print(f"== test_trajectory: pulse x={px:+.4f}m (want {PULSE_WANT}±15%), "
          f"sway peak={peak_x:.4f}m (want {want_peak}±15%), "
          f"still drift={drift * 1000:.2f}mm (<{DRIFT_LIMIT_M * 1000:.0f}mm), "
          f"frames {n_q}/{n_i}, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
