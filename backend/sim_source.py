"""模拟 IMU 帧源（app.py --sim 无串口演示模式的数据源；纯 Python 零依赖，headless 可测）。

自研合成器：静止重力 + 可编程正弦摇摆（roll/pitch 幅值与周期走 SimImuCfg 参数表，
预设注册 SIM_PRESETS，参数零写死）；产出与固件 proto/src/frame.c 同规格的
IMU_SAMPLE 字节流（帧编码常量复用 frame.py 单源），消费端走 FrameDecoder
——与真实串口完全相同的解码路径。

单位约定（入参/出参显式标注）：角度 deg、角速度 dps、加速度 g、时间 s、帧率 Hz。
"""

import math
import struct
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from frame import (SOF, TYPE_IMU_CAL, TYPE_IMU_SAMPLE, TYPE_QUAT, TYPE_TEXT,  # noqa: F401
                   crc8, encode_cal, encode_quat, encode_text)

PAYLOAD_IMU_B = 24   # IMU_SAMPLE payload 字节数 = 6×float32 LE（decode_sample 的 24B 契约）
TIME_EPS_S = 1e-9    # 帧到期判定容忍（s）：浮点/时钟抖动
GRAVITY_MPS2 = 9.80665   # 重力常量（物理常量非调参项；与 trajectory.TrajCfg
                         # 默认 gravity_mps2 对齐——两侧不一致会在去重力处引入系统漂移）


@dataclass(frozen=True)
class SimImuCfg:
    """SimImuSource 参数表（参照 CoreLib/components/imu 的 imu_cfg.h 预设表模式）。

    单位：rate_hz = Hz（帧率，>0）；*_amp_deg = deg（摆幅峰，0 = 该轴静止，>0）；
          *_period_s = s（摇摆周期，>0）。
    方向：正 roll 绕 +x、正 pitch 绕 +y（与下方加速度模型的旋转推导一致）。
    """

    rate_hz: float = 100.0
    roll_amp_deg: float = 10.0
    roll_period_s: float = 2.0
    pitch_amp_deg: float = 10.0
    pitch_period_s: float = 2.0

    def __post_init__(self):
        if self.rate_hz <= 0:
            raise ValueError(f"rate_hz={self.rate_hz} 必须 > 0（Hz）")
        if self.roll_period_s <= 0 or self.pitch_period_s <= 0:
            raise ValueError(f"period {self.roll_period_s}/{self.pitch_period_s} 必须 > 0（s）")


# 预设注册表：按名取用，调用点禁止手写数字参数（参数零写死纪律）
SIM_PRESETS = {
    "sway10_2s": SimImuCfg(),                                   # 默认：roll/pitch ±10°、周期 2s
    "still": SimImuCfg(roll_amp_deg=0.0, pitch_amp_deg=0.0),    # 纯静止（仅重力，az≈+1g）
}


def encode_imu_sample(seq: int, vals: Sequence[float]) -> bytes:
    """六轴样本 → IMU_SAMPLE 整帧字节（与固件同规格）。

    入参：seq 帧序号（自动按 0..255 回绕）；vals=(ax,ay,az,gx,gy,gz)，前 3 单位 g、
          后 3 单位 dps（与 decode_sample 的 payload 契约一致）。
    出参：29B 帧 = SOF | LEN=24 | TYPE | SEQ | payload(24B) | CRC8（覆盖 LEN..PAYLOAD）。
    """
    payload = struct.pack("<6f", *vals)
    head = bytes((SOF, PAYLOAD_IMU_B, TYPE_IMU_SAMPLE, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


class SimImuSource:
    """六轴合成帧源（假串口：poll(now_s) → 新到期帧的字节流）。

    运动模型（时间基由 poll 的入参驱动，与墙钟/轮询节奏解耦）：
        roll(t)  = A_r·sin(w_r·t)，pitch(t) = A_p·sin(w_p·t)     [deg]
        （w = 2π/周期，rad/s）
    加速度（g）= 静止重力 [0,0,1] 经 R^T 投影到体轴系（R = Rx(roll)·Ry(pitch)）：
        ax = -sin(pitch)·cos(roll)，ay = sin(roll)，az = cos(pitch)·cos(roll)
        （az ≈ +1g 与 exp1 静止板验收口径一致）
    陀螺（dps）= 姿态角微分：
        gx = A_r·w_r·cos(w_r·t)，gy = A_p·w_p·cos(w_p·t)，gz = 0
    帧序号：从 1 起连续自增（&0xFF 回绕），供 seq 丢帧检测口径复用。
    """

    def __init__(self, cfg: Optional[SimImuCfg] = None):
        """入参 cfg = None 时取预设 SIM_PRESETS["sway10_2s"]。"""
        self._cfg = cfg if cfg is not None else SIM_PRESETS["sway10_2s"]
        self._n_emit = 0    # 已产出帧数（序号源）

    def poll(self, now_s: float) -> bytes:
        """假串口读接口：产出 (上一帧到期, now_s] 内到期的帧字节。

        入参 now_s：流内时间（s），须单调不减（回退容忍 = 不产帧）。
        出参：0..N 帧拼接字节（每帧 29B），直接喂 FrameDecoder.feed。
        到期规则：第 n 帧到期时刻 = n/rate_hz；帧率与调用节奏解耦（慢轮询补帧、
        快轮询空读）。
        """
        out = bytearray()
        rate = self._cfg.rate_hz
        while (self._n_emit + 1) / rate <= now_s + TIME_EPS_S:
            self._n_emit += 1
            out += encode_imu_sample(self._n_emit, self._model(self._n_emit / rate))
        return bytes(out)

    def _model(self, t_s: float) -> Tuple[float, ...]:
        """t_s（s）时刻的六轴值 (ax,ay,az,gx,gy,gz)，前 3 单位 g、后 3 单位 dps；私有。"""
        cfg = self._cfg
        w_roll = 2.0 * math.pi / cfg.roll_period_s      # rad/s
        w_pitch = 2.0 * math.pi / cfg.pitch_period_s    # rad/s
        roll_deg = cfg.roll_amp_deg * math.sin(w_roll * t_s)
        pitch_deg = cfg.pitch_amp_deg * math.sin(w_pitch * t_s)
        roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)
        ax = -math.sin(pitch) * math.cos(roll)          # g
        ay = math.sin(roll)                             # g
        az = math.cos(pitch) * math.cos(roll)           # g
        gx = cfg.roll_amp_deg * w_roll * math.cos(w_roll * t_s)      # dps（deg/s）
        gy = cfg.pitch_amp_deg * w_pitch * math.cos(w_pitch * t_s)   # dps
        return (ax, ay, az, gx, gy, 0.0)


# ───────────────────────── exp5：摆动位移轨迹合成源（QUAT + IMU 双流） ─────────────────────────


@dataclass(frozen=True)
class SimTrajCfg:
    """SimTrajSource 参数表（exp5 位移轨迹演示/测试的合成运动模型）。

    单位：rate_hz=Hz（帧率，>0）；amp_m=m（世界 +x 摆动位移幅，>0）；
          period_s=s（摆动周期，>0）；*_amp_deg=deg（姿态随动摆幅，>=0）。
    运动模型（世界系，Z 上；**cos 相位起步**——从极值点出发，v(0)=0，
    与位移推算的"起点静止"假设物理自洽；sin 相位起步会在 v(0)=Aω 处
    起播，推算速度整体差一个常量、位移线性漂走）：
        p(t)  = [A·cos(wt), 0, 0]            位移（真值，供测试断言）
        a(t)  = [-A·w²·cos(wt), 0, 0]        线加速度（真值）
        roll(t)  = θr·sin(wt)                 姿态随动（与位移正交相）
        pitch(t) = θp·cos(wt)                 姿态随动（与位移同相）
    每拍产出两帧（同一 t 的姿态与比力，QUAT 先发保证配对消费方先收到姿态）：
        QUAT(0x04)：q = Rx(roll)⊗Ry(pitch) 对应四元数（w 在前）
        IMU(0x01)：f_b = R^T·(a + [0,0,g])/g（机体比力，g 单位）+ 姿态角微分（dps）
    """

    rate_hz: float = 100.0
    amp_m: float = 0.2
    period_s: float = 2.0
    roll_amp_deg: float = 5.0
    pitch_amp_deg: float = 3.0

    def __post_init__(self):
        if self.rate_hz <= 0:
            raise ValueError(f"rate_hz={self.rate_hz} 必须 > 0（Hz）")
        if self.period_s <= 0:
            raise ValueError(f"period_s={self.period_s} 必须 > 0（s）")
        if self.amp_m <= 0:
            raise ValueError(f"amp_m={self.amp_m} 必须 > 0（m）")
        if self.roll_amp_deg < 0 or self.pitch_amp_deg < 0:
            raise ValueError(
                f"姿态摆幅须 >= 0（deg）：roll={self.roll_amp_deg} pitch={self.pitch_amp_deg}")


SIM_TRAJ_PRESETS = {
    # exp5 页面 --sim 默认：±0.25m、周期 1.6s。幅值/周期比的选取约束：
    # 过零附近 |a|<still阈 的连续帧数（≈2·arcsin(thr·g/(A·ω²))/ω·rate）须
    # 显著小于重建器确认窗（默认 6 帧）——否则摆动过零被误判静止、ZUPT
    # 截断速度造成位移棘轮。本预设实测 2~3 帧 < 6，留 2 倍余量。
    "traj_sway": SimTrajCfg(amp_m=0.25, period_s=1.6),
}


class SimTrajSource:
    """摆动轨迹合成帧源（exp5 --sim：QUAT + IMU 双流，假串口 poll 语义与
    SimImuSource 一致，直接喂 FrameDecoder）。

    运动模型见 SimTrajCfg 注；帧序号：QUAT 与 IMU 各自独立连续自增（各自
    &0xFF 回绕）——消费端按帧型分别做 seq 连续性检测（与 bridge 的
    frameReady(type,...) 口径一致）。
    """

    def __init__(self, cfg: Optional[SimTrajCfg] = None):
        """入参 cfg = None 时取预设 SIM_TRAJ_PRESETS["traj_sway"]。"""
        self._cfg = cfg if cfg is not None else SIM_TRAJ_PRESETS["traj_sway"]
        self._n_emit = 0      # 已产出拍数（两帧共用节拍，序号各自独立）
        self._seq_quat = 0
        self._seq_imu = 0

    def poll(self, now_s: float) -> bytes:
        """假串口读接口：产出 (上一拍到期, now_s] 内到期的拍字节流。

        每拍 = QUAT 帧(21B) + IMU 帧(29B) 共 50B；到期规则与 SimImuSource.poll
        一致（第 n 拍到期时刻 = n/rate_hz，与调用节奏解耦）。
        """
        out = bytearray()
        rate = self._cfg.rate_hz
        while (self._n_emit + 1) / rate <= now_s + TIME_EPS_S:
            self._n_emit += 1
            t = self._n_emit / rate
            w, x, y, z, ax, ay, az, gx, gy = self._model(t)
            self._seq_quat += 1
            out += encode_quat(self._seq_quat, w, x, y, z)
            self._seq_imu += 1
            out += encode_imu_sample(self._seq_imu, (ax, ay, az, gx, gy, 0.0))
        return bytes(out)

    def _model(self, t_s: float) -> Tuple[float, ...]:
        """t_s（s）时刻的一拍模型值，私有。

        出参 (qw,qx,qy,qz, ax,ay,az, gx,gy)：q 无量纲（|q|=1）、加速度 g、
        角速度 dps。推导见 SimTrajCfg 类注（R = Rx(roll)·Ry(pitch)）。
        """
        cfg = self._cfg
        w_m = 2.0 * math.pi / cfg.period_s              # rad/s
        acc = -cfg.amp_m * w_m * w_m * math.cos(w_m * t_s)   # 世界 +x 线加速度（m/s²）
        roll = math.radians(cfg.roll_amp_deg) * math.sin(w_m * t_s)
        pitch = math.radians(cfg.pitch_amp_deg) * math.cos(w_m * t_s)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        # 四元数：q = q_x(roll) ⊗ q_y(pitch)（半角公式直乘化简）
        c2r, s2r = math.cos(roll / 2.0), math.sin(roll / 2.0)
        c2p, s2p = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        qw, qx, qy, qz = c2r * c2p, s2r * c2p, c2r * s2p, s2r * s2p
        # 机体比力（g）= R^T·([acc,0,0] + [0,0,g]) / g；R^T·[0,0,1] = (-sp·cr, sr, cp·cr)
        a_g = acc / GRAVITY_MPS2                         # 世界线加速度（g 单位）
        ax = cp * a_g - sp * cr
        ay = sr
        az = sp * a_g + cp * cr
        # 姿态角微分（dps）：roll'=θr·w·cos(wt)，pitch'=-θp·w·sin(wt)
        gx = math.degrees(math.radians(cfg.roll_amp_deg) * w_m * math.cos(w_m * t_s))
        gy = math.degrees(-math.radians(cfg.pitch_amp_deg) * w_m * math.sin(w_m * t_s))
        return (qw, qx, qy, qz, ax, ay, az, gx, gy)


# ───────────────────────── exp2：调零温补合成源（0x02 IMU_CAL 流） ─────────────────────────


def _milli(v: float) -> int:
    """float → 毫单位整数（截断取整；镜像固件 exp2_main.c f_to_milli，仅展示）。"""
    return int(v * 1000.0)


@dataclass(frozen=True)
class SimCalCfg:
    """SimCalSource 参数表（exp2 调零温补页 --sim 演示的合成管线）。

    管线**镜像固件 exp2_main.c 的可观测行为**（不是重写算法，是复刻输出）：
        raw(t) = 真值 + 注入零偏 + k·(T(t) − temp_start)      [imu_sim 注入模型]
        标定窗（前 calib_frames 帧）均值 → bias_ref（**重力轴自动判 +
        减 sign·1g**，镜像 imu_calib.c：bias = measured − expected，故补偿后
        deb az ≈ +1g 而非 0）、T_ref = 窗均温
        bias_now = bias_ref + k·(T − T_ref)（step_c 阶梯重算，首拍必算）
        deb = raw − bias_now ≈ 真值（残差 = k·阶梯滞后 ≤ step_c·k）
    真值 = 静止 Z_UP：acc (0,0,1) g、gyro (0,0,0) dps。

    默认值镜像三张预设表（参数零写死——调用点走 SIM_CAL_PRESETS）：
      - sim.exp2.static（Embedded testing/imu_sim/imu_sim.c 预设）：
        rate 100Hz、T 25°C + 0.1°C/s 斜坡、零偏 gx=+1.0 gy=−0.5 gz=+2.5 dps、
        ax=+0.02 ay=−0.03 az=+0.015 g、温漂 0.05 dps/°C 与 0.0005 g/°C；
      - cal.exp2.bias（CoreLib calibration/imu_calib.c）：标定窗 500 帧；
      - cal.exp2.drift（temp_comp.c）：阶梯阈值 0.5°C。
    """

    rate_hz: float = 100.0
    temp_start_c: float = 25.0
    temp_ramp_c_per_s: float = 0.1
    calib_frames: int = 500
    step_c: float = 0.5
    bias: Tuple[float, ...] = (0.02, -0.03, 0.015, 1.0, -0.5, 2.5)  # 采样序 ax..az g, gx..gz dps
    gyro_drift: Tuple[float, ...] = (0.05, 0.05, 0.05)      # dps/°C（对应轴 gx,gy,gz）
    accel_drift: Tuple[float, ...] = (0.0005, 0.0005, 0.0005)  # g/°C（对应轴 ax,ay,az）

    def __post_init__(self):
        if self.rate_hz <= 0:
            raise ValueError(f"rate_hz={self.rate_hz} 必须 > 0（Hz）")
        if self.calib_frames <= 0:
            raise ValueError(f"calib_frames={self.calib_frames} 必须 > 0")
        if self.step_c < 0:
            raise ValueError(f"step_c={self.step_c} 须 >= 0（°C；0 = 每拍重算）")
        if len(self.bias) != 6:
            raise ValueError(f"bias 须 6 元（采样序 ax..gz），得 {len(self.bias)}")
        if len(self.gyro_drift) != 3 or len(self.accel_drift) != 3:
            raise ValueError("gyro_drift/accel_drift 须各 3 元")


SIM_CAL_PRESETS = {
    # exp2 --sim 默认：镜像 sim.exp2.static 静止注入（值全同 SimCalCfg 默认，
    # 显式列出便于对照固件预设表；改值只动此处）
    "exp2_static": SimCalCfg(),
}


class SimCalSource:
    """调零温补合成帧源（exp2 --sim：0x02 IMU_CAL 流，假串口 poll 语义与
    SimImuSource 一致，直接喂 FrameDecoder）。

    开机时序镜像固件：先 TEXT 横幅（OK sim: …）→ 标定窗（calib_frames 帧静默
    采集，无输出字节）→ TEXT（OK calib … / OK stream …）→ 0x02 帧流。
    帧序号：TEXT 与 CAL 共用一个计数器连续自增（&0xFF 回绕，镜像固件
    exp2_main.c 单一 s_seq 口径）。
    """

    def __init__(self, cfg: Optional[SimCalCfg] = None):
        """入参 cfg = None 时取预设 SIM_CAL_PRESETS["exp2_static"]。"""
        self._cfg = cfg if cfg is not None else SIM_CAL_PRESETS["exp2_static"]
        self._n_tick = 0          # 已推进拍数（标定窗与流共用时间基）
        self._seq = 0             # 已发出帧数（TEXT+CAL 共用序号源）
        self._bias_ref = None     # 标定窗零偏均值（采样序 6 元）
        self._t_ref = None        # 标定窗均温（°C）
        self._bias_now = None     # 当前生效零偏（阶梯保持）
        self._t_applied = None    # 上次阶梯重算温度（°C）
        self._cal_sum = (0.0,) * 6   # 标定窗零偏累加器
        self._cal_tsum = 0.0         # 标定窗温度累加器（°C）

    def poll(self, now_s: float) -> bytes:
        """假串口读接口：产出 (上一拍到期, now_s] 内到期的帧字节流。

        到期规则与 SimImuSource.poll 一致（第 n 拍到期时刻 = n/rate_hz）；
        标定窗内只累计不出帧（镜像固件：窗内无 0x02 输出），窗满那拍先发
        两条 TEXT 再开始流送。
        """
        out = bytearray()
        cfg = self._cfg
        rate = cfg.rate_hz
        while (self._n_tick + 1) / rate <= now_s + TIME_EPS_S:
            self._n_tick += 1
            t = self._n_tick / rate
            temp = cfg.temp_start_c + cfg.temp_ramp_c_per_s * t
            raw = self._raw_at(t)
            if self._n_tick == 1:      # 开机横幅（镜像 source_init 的 send_text）
                out += self._send_text(self._banner_sim())
            if self._bias_ref is None:
                # 标定窗：累计零偏与温度（未满窗不出帧）
                self._cal_sum = tuple(s + r for s, r in zip(self._cal_sum, raw))
                self._cal_tsum += temp
                if self._n_tick >= cfg.calib_frames:
                    n = cfg.calib_frames
                    means = tuple(s / n for s in self._cal_sum)
                    # 重力轴判定 + 减重力（镜像固件 imu_calib.c：|窗口均值| 最大
                    # 的加速度轴减 sign·1g——bias = measured − expected，其余轴
                    # 不减；静止 Z_UP 下 az 轴命中，补偿后 deb az ≈ +1g）
                    grav_axis = 0
                    if abs(means[1]) > abs(means[0]):
                        grav_axis = 1
                    if abs(means[2]) > abs(means[grav_axis]):
                        grav_axis = 2
                    sign = 1.0 if means[grav_axis] > 0.0 else -1.0
                    bias_ref = list(means)
                    bias_ref[grav_axis] -= sign   # 1 g = 1（单位 g，重力常量）
                    self._bias_ref = tuple(bias_ref)
                    self._t_ref = self._cal_tsum / n
                    out += self._send_text(self._banner_calib())
                    out += self._send_text("OK stream: TYPE_IMU_CAL 0x02 76B")
                continue
            bias_now = self._bias_at(temp)
            deb = tuple(r - b for r, b in zip(raw, bias_now))
            self._seq += 1
            out += encode_cal(self._seq, tuple(raw) + tuple(deb) +
                              tuple(bias_now) + (temp,))
        return bytes(out)

    # —— 私有：模型与横幅 ——

    def _raw_at(self, t_s: float) -> Tuple[float, ...]:
        """t_s 拍的原始六轴（真值 + 注入零偏 + 温漂；采样序 ax..gz）。

        dt = T(t) − temp_ref（预设 temp_ref_c = temp_start_c = 25°C，故 = ramp·t）；
        真值 = 静止 Z_UP：acc (0,0,1) g、gyro (0,0,0) dps。
        """
        cfg = self._cfg
        dt = cfg.temp_ramp_c_per_s * t_s
        ad, gd = cfg.accel_drift, cfg.gyro_drift
        b = cfg.bias
        return (b[0] + ad[0] * dt, b[1] + ad[1] * dt, 1.0 + b[2] + ad[2] * dt,
                b[3] + gd[0] * dt, b[4] + gd[1] * dt, b[5] + gd[2] * dt)

    def _bias_at(self, temp: float) -> Tuple[float, ...]:
        """温补零偏 bias_now = bias_ref + k·(T − T_ref)（step_c 阶梯重算）。"""
        cfg = self._cfg
        if self._bias_now is None or abs(temp - self._t_applied) >= cfg.step_c:
            ad, gd = cfg.accel_drift, cfg.gyro_drift
            dt = temp - self._t_ref
            self._bias_now = tuple(br + k * d for br, k, d in
                                   zip(self._bias_ref, ad + gd, (dt,) * 6))
            self._t_applied = temp
        return self._bias_now

    def _send_text(self, text: str) -> bytes:
        self._seq += 1
        return encode_text(self._seq, text)

    def _banner_sim(self) -> str:
        cfg = self._cfg
        return (f"OK sim: static Z_UP {cfg.rate_hz:g}Hz temp {_milli(cfg.temp_start_c)}mC "
                f"ramp {_milli(cfg.temp_ramp_c_per_s):+d}mC/s "
                f"bias_g {_milli(cfg.bias[3]):+d}mdps")

    def _banner_calib(self) -> str:
        cfg = self._cfg
        br, tr = self._bias_ref, self._t_ref
        return (f"OK calib {cfg.calib_frames} frames: "
                f"bias g[{_milli(br[3])},{_milli(br[4])},{_milli(br[5])}]mdps "
                f"a[{_milli(br[0])},{_milli(br[1])},{_milli(br[2])}]mg "
                f"t_ref {_milli(tr)}mC")
