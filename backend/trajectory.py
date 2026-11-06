"""位移轨迹重建（exp5）：四元数姿态 + 机体比力 → 世界系位置序列。

链路（惯性航位推算 DR，自研重写——算法形态参照旧仓 MotionTracker 思路 +
docs/research/IMU-DISPLACEMENT-LIMITS.md 的位移极限研究，实现零拷贝）：

    f_b(g, 机体比力) ──[去 body 系零偏]──> R(q)·f_b ──[×g, 去世界系重力 -z·g]
    ──> a_lin ──[高通去缓变零偏]──[低通抗抖(可选)]──> a_f
    ──> v（梯形积分 + 静止强阻尼 + ZUPT 硬清零 + 限幅）──> p（梯形积分 + 软限位）

输入契约：feed(w,x,y,z, ax,ay,az) 一帧一喂——姿态四元数 (w,x,y,z) 来自
QUAT 帧（0x04，16B 契约见 frame.py decode_quat）；机体比力 (ax,ay,az) 单位 g
来自 IMU_SAMPLE 帧（0x01，decode_sample 前三分量）。采样周期走构造配置
（TrajCfg.dt_s），不逐帧传时间戳——帧率由固件节拍保证，漂移抑制窗口以帧计。

漂移抑制策略（依据 IMU-DISPLACEMENT-LIMITS.md §三，策略自定）：
  1) **ZUPT 零速修正**（研究文档表 ZUPT 行：静止相硬清零速度）——静止检测
     用**去重力后、滤波前**的原始幅值（旧仓实测：滤波后再判会把运动起止
     抹平成"静止"，形状失真 21%→368%）；确认窗口内先强阻尼（停手即停的
     观感）、确认后硬清零（吸收姿态收敛期重力补偿残差持续注入）。
  2) **静止门控零偏学习**（研究文档 §三"业界把零偏作为状态在线估计"的
     降级实现）：静止确认后按残差迭代 body 系零偏——**必须在 body 系学/减**
     （旧仓实测：世界系减零偏，yaw 慢漂会把零偏投影转成圆轨迹）。
  3) **一阶高通去缓变零偏**（研究文档 §三 EMA 高通行）：截止取极低
     （默认 0.02Hz，时间常数 ~8s）——研究文档实测较高截止会吃掉低频真实
     运动（-30%），故只作长时基带抑制，短时漂移交给 ZUPT。
  4) 误差物理上限（研究文档 §一：MEMS 二重积分 60s 漂百米级）不可调参
     消除——本类输出定位为"运动示意"（方向/形状/量级），非米制计量。

POSIX 风格：构造参数非法抛 ValueError；无全局态；单实例非线程安全
（调用方——bridge——须在单线程内串行 feed）。
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class TrajCfg:
    """TrajectoryReconstructor 参数表（参照 SimImuCfg 模式：默认值即推荐档，
    调用点按名取预设，禁止手写数字参数）。

    单位：dt_s=s（采样周期，>0）；gravity_mps2=m/s²（重力常量，>0）；
    *_hz=Hz（截止频率，高通>0；低通 None=关）；still_threshold_g=g（静止
    判据幅值阈，作用于去重力后的世界系加速度，>0）；*_frames=帧（>0）；
    *_per_s=每秒保留率（(0,1]）；bias_learn_rate_per_s=1/s（零偏学习速率，
    >0；收敛时间 ~1/速率）；max_speed_mps=m/s（None=不限幅）；
    pos_limit_m=m（软限位半径，None=不限）；positions_maxlen=位置序列
    容量上限（None=无限增长）。
    """

    dt_s: float = 0.01
    gravity_mps2: float = 9.80665
    hp_cutoff_hz: float = 0.02
    lp_cutoff_hz: Optional[float] = None
    still_threshold_g: float = 0.02
    still_window_frames: int = 6
    still_damp_per_s: float = 0.05
    move_damp_per_s: float = 1.0
    bias_learn_rate_per_s: float = 3.0
    max_speed_mps: Optional[float] = None
    pos_limit_m: Optional[float] = None
    positions_maxlen: Optional[int] = None

    def __post_init__(self):
        if self.dt_s <= 0:
            raise ValueError(f"dt_s={self.dt_s} 必须 > 0（s）")
        if self.gravity_mps2 <= 0:
            raise ValueError(f"gravity_mps2={self.gravity_mps2} 必须 > 0（m/s²）")
        if self.hp_cutoff_hz <= 0:
            raise ValueError(f"hp_cutoff_hz={self.hp_cutoff_hz} 必须 > 0（Hz）")
        if self.lp_cutoff_hz is not None and self.lp_cutoff_hz <= 0:
            raise ValueError(f"lp_cutoff_hz={self.lp_cutoff_hz} 须为 > 0（Hz）或 None")
        if self.still_threshold_g <= 0:
            raise ValueError(f"still_threshold_g={self.still_threshold_g} 必须 > 0（g）")
        if self.still_window_frames < 1:
            raise ValueError(f"still_window_frames={self.still_window_frames} 必须 >= 1")
        for name in ("still_damp_per_s", "move_damp_per_s"):
            v = getattr(self, name)
            if not (0 < v <= 1):
                raise ValueError(f"{name}={v} 须在 (0,1]（每秒保留率）")
        if self.bias_learn_rate_per_s <= 0:
            raise ValueError(
                f"bias_learn_rate_per_s={self.bias_learn_rate_per_s} 必须 > 0（1/s）")
        if self.max_speed_mps is not None and self.max_speed_mps <= 0:
            raise ValueError(f"max_speed_mps={self.max_speed_mps} 须为 > 0 或 None")
        if self.pos_limit_m is not None and self.pos_limit_m <= 0:
            raise ValueError(f"pos_limit_m={self.pos_limit_m} 须为 > 0 或 None")
        if self.positions_maxlen is not None and self.positions_maxlen < 1:
            raise ValueError(f"positions_maxlen={self.positions_maxlen} 须 >= 1 或 None")


# 预设注册表：调用点按名取用（参数零写死纪律，与 SIM_PRESETS 同模式）
TRAJ_PRESETS = {
    # exp5 演示/页面默认：100Hz 节拍档（摆幅/周期由 SimTrajSource 预设定义，
    # 此处只锁算法窗口/滤波档；摆源过零低幅帧数须 < still_window_frames，
    # 见 sim_source.SIM_TRAJ_PRESETS 注）
    "exp5_sway": TrajCfg(),
}


def quat_to_matrix(w: float, x: float, y: float, z: float) -> List[List[float]]:
    """四元数 (w,x,y,z) → 3×3 旋转矩阵 R（body→world，Hamilton 约定）。

    入参未归一化时自动归一化（零四元数拒绝——姿态无定义）；出参嵌套列表
    [行][列]，与旧仓 attitude.js quatToR 公式形态一致（实现自研）。
    R 语义：v_world = R · v_body；R 第三行 = body 系 z 轴的世界向，亦即
    body 系重力单位向量（静止时比力 = R^T·[0,0,1]）。
    """
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError(f"|q|={n} 过小（零四元数无姿态定义）")
    qw, qx, qy, qz = w / n, x / n, y / n, z / n
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ]


class TrajectoryReconstructor:
    """惯性位移重建器：逐帧喂 (q, f_b) 输出世界系位置（m，X/Y 水平、Z 上）。

    用法（POSIX 风格）：
        rec = TrajectoryReconstructor(TRAJ_PRESETS["exp5_sway"])
        px, py, pz = rec.feed(w, x, y, z, ax, ay, az)   # 一帧一喂
        hist = rec.positions()                          # 位置序列（拷贝）
    坐标约定：世界系 Z 上（与去重力口径 -z·g 一致）；输入比力单位 g
    （0x01 帧契约），内部按 cfg.gravity_mps2 转 SI；输出单位 m。
    """

    def __init__(self, cfg: Optional[TrajCfg] = None):
        """入参 cfg = None 时取预设 TRAJ_PRESETS["exp5_sway"]。"""
        self._cfg = cfg if cfg is not None else TRAJ_PRESETS["exp5_sway"]
        if not isinstance(self._cfg, TrajCfg):
            raise ValueError(f"cfg 须为 TrajCfg 实例，得到 {type(cfg).__name__}")
        self.reset()

    # —— 对外接口 ——

    def feed(self, w: float, x: float, y: float, z: float,
             ax: float, ay: float, az: float) -> Tuple[float, float, float]:
        """喂一帧：姿态四元数 (w,x,y,z) + 机体比力 (ax,ay,az)（单位 g）。

        返回当前世界系位置 (px, py, pz)（m）。数值非法（NaN/Inf）抛
        ValueError——脏数据宁可拒帧不积分（NaN 会毒化全部后续状态）。
        """
        for v in (w, x, y, z, ax, ay, az):
            if not math.isfinite(v):
                raise ValueError(f"输入含非有限值：q=({w},{x},{y},{z}) a=({ax},{ay},{az})")
        cfg = self._cfg
        dt = cfg.dt_s
        g = cfg.gravity_mps2

        # 1) 姿态：四元数 → R（body→world）
        R = quat_to_matrix(w, x, y, z)

        # 2) body 系去零偏（零偏是传感器固有 → body 系常量；世界系减会被
        #    yaw 漂移转成圆轨迹，见模块注漂移抑制策略第 2 条）
        bx, by, bz = self._bias
        fx, fy, fz = ax - bx, ay - by, az - bz

        # 3) 转世界系（g → m/s²）
        axw = (R[0][0] * fx + R[0][1] * fy + R[0][2] * fz) * g
        ayw = (R[1][0] * fx + R[1][1] * fy + R[1][2] * fz) * g
        azw = (R[2][0] * fx + R[2][1] * fy + R[2][2] * fz) * g

        # 4) 去重力：世界系恒定 -z·g（加速度计静止读 +1g 反作用 → 减 [0,0,+g]）
        lx, ly, lz = axw, ayw, azw - g

        # 5) 静止检测：去重力后、滤波前的原始幅值（滤波会抹平运动起止，
        #    见模块注策略 1）。阻尼/清零只认**确认静止**（连续满窗）——
        #    瞬时低幅窗口（摆动过零附近）若也施强阻尼，速度波形被周期性
        #    打断，位移棘轮式累积（实测 0.2m 摆幅被放大到 1.18m）。
        mag_g = math.sqrt(lx * lx + ly * ly + lz * lz) / g
        if mag_g < cfg.still_threshold_g:
            self._still_run += 1
        else:
            self._still_run = 0
        still_confirmed = self._still_run >= cfg.still_window_frames

        # 6) 高通（一阶，去缓变零偏）。状态**零初值**：若用首样本初始化，
        #    运动中起播（a(0) 可达满幅）会把 a(0) 以时间常数 τ 注入成
        #    a(0)·τ 量级伪速度（实测 0.2m 摆幅被推到 13.5m）——零初值下
        #    瞬态仅为稳态跟踪幅度（LP 增益×信号），可忽略。
        tau_hp = 1.0 / (2.0 * math.pi * cfg.hp_cutoff_hz)
        k_hp = min(1.0, dt / tau_hp)
        if self._hp is None:
            self._hp = [0.0, 0.0, 0.0]
        hpx = lx - (self._hp[0] + k_hp * (lx - self._hp[0]))
        hpy = ly - (self._hp[1] + k_hp * (ly - self._hp[1]))
        hpz = lz - (self._hp[2] + k_hp * (lz - self._hp[2]))
        # 状态推进（显式两步，避免读改写顺序歧义）
        self._hp[0] += k_hp * (lx - self._hp[0])
        self._hp[1] += k_hp * (ly - self._hp[1])
        self._hp[2] += k_hp * (lz - self._hp[2])

        # 7) 低通（可选，抗抖；默认关——合成流干净，实机抖动由预设开启）
        if cfg.lp_cutoff_hz is not None:
            tau_lp = 1.0 / (2.0 * math.pi * cfg.lp_cutoff_hz)
            k_lp = min(1.0, dt / tau_lp)
            if self._lp is None:
                self._lp = [hpx, hpy, hpz]
            afx = self._lp[0] + k_lp * (hpx - self._lp[0])
            afy = self._lp[1] + k_lp * (hpy - self._lp[1])
            afz = self._lp[2] + k_lp * (hpz - self._lp[2])
            self._lp = [afx, afy, afz]
        else:
            afx, afy, afz = hpx, hpy, hpz

        # 8) 速度：**对加速度也用梯形**（0.5·(a_prev+a)·dt，研究文档 §三
        #    中点积分条目：矩形法则对周期信号有 ~A·ω·dt/2 常值速度偏置，
        #    4s 累到峰值的 10% 量级）+ 阻尼（确认静止才强阻尼；运动相
        #    弱/无阻尼保真实位移）+ ZUPT 硬清零（确认后）
        vox, voy, voz = self._vel
        apx, apy, apz = self._a_prev if self._a_prev is not None else (afx, afy, afz)
        damp = cfg.still_damp_per_s if still_confirmed else cfg.move_damp_per_s
        damp_dt = damp ** dt
        vnx = (vox + 0.5 * (apx + afx) * dt) * damp_dt
        vny = (voy + 0.5 * (apy + afy) * dt) * damp_dt
        vnz = (voz + 0.5 * (apz + afz) * dt) * damp_dt
        self._a_prev = [afx, afy, afz]

        # 速度限幅：安全阀（摆动/噪声积分爆走封顶；旧仓实测手写上限 ~1.5m/s）
        if cfg.max_speed_mps is not None:
            vmag = math.sqrt(vnx * vnx + vny * vny + vnz * vnz)
            if vmag > cfg.max_speed_mps:
                s = cfg.max_speed_mps / vmag
                vnx, vny, vnz = vnx * s, vny * s, vnz * s

        # 9) 位置：梯形（中点）积分——研究文档 §三：前向欧拉对短脉冲系统
        #    性高估，中点误差低一个量级（参考 #9）
        self._pos[0] += 0.5 * (vox + vnx) * dt
        self._pos[1] += 0.5 * (voy + vny) * dt
        self._pos[2] += 0.5 * (voz + vnz) * dt

        # ZUPT 硬清零（位置已按阻尼后速度积分完尾段，此后冻结）
        if still_confirmed:
            vnx = vny = vnz = 0.0

        # 10) 静止门控零偏学习：静止时 body 系比力应 = R^T·[0,0,1]（= R 的
        #     第三行），残差即零偏；按剩余残差迭代（不减已学部分会无限过冲）
        if still_confirmed:
            kg = min(1.0, cfg.bias_learn_rate_per_s * dt)
            ex = ax - bx - R[2][0]      # 期望重力 body 系分量 = R 第三行
            ey = ay - by - R[2][1]
            ez = az - bz - R[2][2]
            self._bias[0] += kg * ex
            self._bias[1] += kg * ey
            self._bias[2] += kg * ez

        # 11) 位置软限位（超限等比缩回 + 速度清零，防漂出视野）
        if cfg.pos_limit_m is not None:
            pd = math.sqrt(self._pos[0] ** 2 + self._pos[1] ** 2 + self._pos[2] ** 2)
            if pd > cfg.pos_limit_m:
                s = cfg.pos_limit_m / pd
                self._pos = [c * s for c in self._pos]
                vnx = vny = vnz = 0.0

        self._vel = [vnx, vny, vnz]
        self._positions.append(tuple(self._pos))
        if (self._cfg.positions_maxlen is not None
                and len(self._positions) > self._cfg.positions_maxlen):
            del self._positions[:len(self._positions) - self._cfg.positions_maxlen]
        return (self._pos[0], self._pos[1], self._pos[2])

    def positions(self) -> List[Tuple[float, float, float]]:
        """位置序列拷贝 [(x,y,z), ...]（m；下标 0 = 首帧）。"""
        return list(self._positions)

    def velocity(self) -> Tuple[float, float, float]:
        """当前速度 (vx,vy,vz)（m/s）。"""
        return (self._vel[0], self._vel[1], self._vel[2])

    def position(self) -> Tuple[float, float, float]:
        """当前位置 (x,y,z)（m）。"""
        return (self._pos[0], self._pos[1], self._pos[2])

    def bias(self) -> Tuple[float, float, float]:
        """已学习零偏 (bx,by,bz)（g，body 系；诊断用）。"""
        return (self._bias[0], self._bias[1], self._bias[2])

    def reset(self) -> None:
        """清零全部积分/滤波/零偏状态（重连续跑用）。"""
        self._pos: List[float] = [0.0, 0.0, 0.0]
        self._vel: List[float] = [0.0, 0.0, 0.0]
        self._bias: List[float] = [0.0, 0.0, 0.0]
        self._hp: Optional[List[float]] = None
        self._lp: Optional[List[float]] = None
        self._a_prev: Optional[List[float]] = None
        self._still_run = 0
        self._positions: List[Tuple[float, float, float]] = []
