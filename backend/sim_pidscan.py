#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本机合成 PID 扫描帧源（backend/scope_pidscan.py --source sim 的数据源；纯标准库零依赖）。

意图：既没接 QEMU、也没接真机时，用本机确定性合成流跑通「解码 → 双表 → 断言」
整条链路。按**帧计数**推进（不用墙钟、不用随机——同一 rounds 跑两遍逐字节相同），
产出字节流与 exp_motor_pidscan 固件（main/exp_motor_pidscan_main.c）同序同规格：

    表头 TEXT（"PIDSCAN 3x3: g kp ki settle_s ov% se%"）
    → 9 组 × [500 拍低段 30rpm + 1000 拍高段 90rpm] 的 0x05 MOTOR 单电机 16B 记录
    → 9 组行 TEXT（最优组行尾 " *"）→ BEST 行 → "PIDSCAN round done 9 groups"

闭环模型不是拟合出来的：plant（160 rpm/duty、τ=20ms、2ms 拍）的闭环特征方程
0.02s²+(1+160kp)s+160ki=0 给出 ζ=(1+160kp)/(2√(3.2ki))、ωn=√(8000ki)，高段响应取
二阶欠阻尼阶跃解析解；低段一阶爬升 30·(1-e^(-t/0.02))，进高段时值连续（≈30，无跳变）。

组三指标按固件口径**流式累积**（peak / 末次出带拍号 / 高段末半段均值，同 scen.c
group_end 的算式），TEXT 组行由该累积态算出；host 侧是收流后按轨迹**后置重算**
（scope_pidscan.group_metrics）——两条实现路径不同，sim 档双表断言验的是「解码 +
重算口径一致」；QEMU / 真机档的独立性来自真实现，sim 档不替代。

单位：rpm；拍 = TICK_S = 0.002s（Embedded scen.h EXP_MOTOR_PIDSCAN_POLL_US 单源）。
"""

import math
import struct
from dataclasses import dataclass

from frame import SOF, TYPE_TEXT, crc8   # 帧编码常量单源（SOF/CRC/TEXT 类型）

TYPE_MOTOR = 0x05        # 契约镜像 proto/frame.h PROTO_TYPE_MOTOR（scope_pidscan.TYPE_MOTOR 同值）
KIND_ENC_CLOSED = 2      # motor_kind_t（proto/frame.h 偏移 1：0=PWM_OPEN 1=SERVO 2=ENC_CLOSED 3=MOTOR_DC）

STEP_LO = 30.0           # 低段目标 rpm
STEP_HI = 90.0           # 高段目标 rpm
HOLD_LO = 500            # 低段前导拍数
HOLD_HI = 1000           # 高段统计窗拍数
TICK_S = 0.002           # 拍宽 s（2ms 单源）
SS_TOL_PCT = 3.0         # 到带/稳态判据 ±3%（scen.h EXP_MOTOR_PIDSCAN_SS_TOL_PCT）
PH_LO, PH_HI = 0, 1      # 组内步号（相位字段域）

PLANT_RPM_PER_DUTY = 160.0   # plant 增益 rpm/duty（scen.c s_plant_rpm_per_duty）
PLANT_TAU_S = 0.02           # plant 时间常数 s（scen.c s_plant_tau_s）

N_GROUPS = 9                       # 网格组数（scen.h EXP_MOTOR_PIDSCAN_GROUPS）
GROUP_TICKS = HOLD_LO + HOLD_HI    # 每组拍数 = 1500
ROUND_S = N_GROUPS * GROUP_TICKS * TICK_S   # 一轮设计时长 = 27.0s

READ_FRAMES = 512   # 单次 read() 最多产出的帧数（分片粒度，与采集侧缓冲无耦合）

# 3×3 网格（scen.c s_grid 单源：g = kp_idx*3 + ki_idx，行= kp 升序，列= ki 升序）
GRID = (
    (0.006, 1.00), (0.006, 1.50), (0.006, 2.00),
    (0.010, 1.00), (0.010, 1.50), (0.010, 2.00),
    (0.015, 1.00), (0.015, 1.50), (0.015, 2.00),
)


def rounds_for_seconds(seconds: float) -> int:
    """--seconds 虚拟采集窗 → 轮数（27s/轮）。

    至少 1 轮：不满一轮的流出不了完整对比表（round done 未到 = 断言无意义）。
    """
    return max(1, int(math.ceil(seconds / ROUND_S)))


def f32(x: float) -> float:
    """double → float32 往返（帧里存的是 f32，指标必须用同一批 f32 值算）。"""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def group_dynamics(g: int):
    """组 g 的二阶闭环参数 (zeta, wn)。

    特征方程 0.02s² + (1+160kp)s + 160ki = 0（scen.c 网格注）；网格全体
    ζ ∈ [0.39, 0.68] 欠阻尼，故 sqrt(1-ζ²) 恒有效。
    """
    kp, ki = GRID[g]
    zeta = (1.0 + PLANT_RPM_PER_DUTY * kp) / (
        2.0 * math.sqrt(PLANT_RPM_PER_DUTY * PLANT_TAU_S * ki))
    wn = math.sqrt(PLANT_RPM_PER_DUTY * ki / PLANT_TAU_S)
    return zeta, wn


def hi_actual_rpm(k: int, zeta: float, wn: float) -> float:
    """高段第 k 拍（0 起）的 actual（rpm）：二阶欠阻尼阶跃解析解。

    起点 = STEP_LO（= 低段终值，无跳变），终值 = STEP_HI；
    超调峰 = STEP_HI + (STEP_HI-STEP_LO)·e^(-ζπ/√(1-ζ²))——与固件超调分母同口径。
    """
    t = k * TICK_S
    root = math.sqrt(1.0 - zeta * zeta)
    wd = wn * root
    decay = math.exp(-zeta * wn * t)
    return STEP_HI - (STEP_HI - STEP_LO) * decay * (
        math.cos(wd * t) + zeta / root * math.sin(wd * t))


def lo_actual_rpm(k: int) -> float:
    """低段第 k 拍（0 起）的 actual（rpm）：一阶爬升 30·(1-e^(-t/τ))。"""
    return STEP_LO * (1.0 - math.exp(-(k * TICK_S) / PLANT_TAU_S))


def encode_motor(seq: int, slot: int, setpoint: float, duty: float,
                 actual: float, phase: int, fault: int = 0) -> bytes:
    """单电机 16B 记录 → 0x05 整帧字节（布局单源 proto/frame.h PROTO_TYPE_MOTOR）。

    入参 seq 帧序号（&0xFF 回绕）；setpoint/actual 单位 rpm，duty 单位 0~1；
          phase 组内步号（0 低段 / 1 高段）；fault 位域（0 = 正常）。
    出参 21B 帧 = SOF | LEN=16 | TYPE=0x05 | SEQ | payload(16B) | CRC8。
    """
    payload = (bytes((slot & 0xFF, KIND_ENC_CLOSED))
               + struct.pack("<3f", setpoint, duty, actual)
               + bytes((phase & 0xFF, fault & 0xFF)))
    head = bytes((SOF, len(payload), TYPE_MOTOR, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


@dataclass(frozen=True)
class GroupResult:
    """单组三指标（固件 exp_pid_result_t 的对应物；settle_s = -1 表示不收敛）。"""

    settle_s: float
    overshoot_pct: float
    ss_err_pct: float
    ok: bool

    @property
    def score(self) -> float:
        """最优组排序键（固件同式：settle + ov/100 + se/100，越小越好）。"""
        return self.settle_s + self.overshoot_pct / 100.0 + self.ss_err_pct / 100.0


class SimPidscanSource:
    """合成帧源（假串口：read() → 新帧字节流，流尽返回 b""）。

    每实例产出 rounds 轮完整扫描；帧序号 TEXT 与 0x05 共用自增计数器
    （镜像固件单一 s_seq）。组指标在产出该组帧的过程中流式累积（同固件
    tick/group_end 时序），全部组收齐后按固件格式吐出对比表 TEXT 行。
    """

    def __init__(self, rounds: int = 1):
        """入参 rounds = 产出轮数（≥1；rounds_for_seconds 由采集窗换算）。"""
        if rounds < 1:
            raise ValueError("rounds=%d 必须 >= 1（一轮 = 9 组 × 1500 拍）" % rounds)
        self._rounds = rounds
        self._seq = 0
        self._results = {}          # g → GroupResult（_group 边产出边写）
        self._frames = self._generate()

    def read(self, max_frames: int = READ_FRAMES) -> bytes:
        """假串口读接口：按帧计数推进，产出接下来 max_frames 帧的拼接字节。

        出参：0 ≤ len ≤ max_frames 帧字节；b"" = 流已尽（调用方据此收工）。
        """
        out = bytearray()
        for _ in range(max_frames):
            frame = next(self._frames, None)
            if frame is None:
                break
            out += frame
        return bytes(out)

    def close(self) -> None:
        """假串口无资源需释放（与 TCP/串口源同接口，采集侧不必分支）。"""
        self._frames = iter(())

    def _generate(self):
        """逐轮产出整帧字节：表头 → 9 组 0x05 → 9 组行 → BEST → round done。"""
        for _ in range(self._rounds):
            yield self._text("PIDSCAN 3x3: g kp ki settle_s ov% se%")
            for g in range(N_GROUPS):
                yield from self._group(g)
            for g in range(N_GROUPS):
                yield self._text(self._row(g))
            best = self._best()
            if best >= 0:
                yield self._text("BEST G%d score%.3fs (st+ov%%/100+se%%/100 min)"
                                 % (best, self._results[best].score))
            else:
                yield self._text("BEST none (no group converged)")
            yield self._text("PIDSCAN round done 9 groups")

    def _group(self, g: int):
        """产出组 g 的 1500 帧 0x05，并把该组三指标写入 self._results[g]。

        统计口径 = scen.c：只在高段累积 peak / 末次出带拍号 / 末半段均值。
        """
        zeta, wn = group_dynamics(g)
        band = STEP_HI * SS_TOL_PCT / 100.0
        peak = 0.0
        last_out = -1
        tail_sum = 0.0
        tail_cnt = 0
        for k in range(HOLD_LO):
            yield self._motor(g, STEP_LO, lo_actual_rpm(k), PH_LO)
        for k in range(HOLD_HI):
            actual = f32(hi_actual_rpm(k, zeta, wn))
            yield self._motor(g, STEP_HI, actual, PH_HI)
            if actual > peak:
                peak = actual
            if actual < STEP_HI - band or actual > STEP_HI + band:
                last_out = k
            if k >= HOLD_HI // 2:
                tail_sum += actual
                tail_cnt += 1
        ss = abs(tail_sum / tail_cnt - STEP_HI) / STEP_HI * 100.0
        ov = max(0.0, (peak - STEP_HI) / (STEP_HI - STEP_LO) * 100.0)
        ok = last_out >= 0 and last_out < HOLD_HI - 1 and ss <= SS_TOL_PCT
        self._results[g] = GroupResult(
            settle_s=(last_out + 1) * TICK_S if ok else -1.0,
            overshoot_pct=ov, ss_err_pct=ss, ok=ok)

    def _motor(self, g: int, setpoint: float, actual: float, phase: int) -> bytes:
        """组 g 的一帧 0x05（duty 取速比前馈 sp/160，= 该拍转速对应的稳态占空）。"""
        self._seq += 1
        return encode_motor(self._seq, g, setpoint,
                            setpoint / PLANT_RPM_PER_DUTY, actual, phase)

    def _text(self, line: str) -> bytes:
        """一行 TEXT → 0x7F 整帧（格式与固件 send_text 同规格）。"""
        self._seq += 1
        payload = line.encode("ascii", "replace")
        head = bytes((SOF, len(payload), TYPE_TEXT, self._seq & 0xFF))
        return head + payload + bytes((crc8(head[1:] + payload),))

    def _best(self) -> int:
        """最优组号（严格小于取组号小，-1 = 无组收敛；固件 best_group 同式）。"""
        best = -1
        for g in range(N_GROUPS):
            if not self._results[g].ok:
                continue
            if best < 0 or self._results[g].score < self._results[best].score:
                best = g
        return best

    def _row(self, g: int) -> str:
        """固件组行文本（scen.c summary_line 同格式；最优组 " *"、不收敛 " !"）。"""
        r = self._results[g]
        kp, ki = GRID[g]
        if not r.ok:
            return "G%d kp%g ki%g stN/A ov%.1f%% se%.2f%% !" % (
                g, kp, ki, r.overshoot_pct, r.ss_err_pct)
        mark = " *" if g == self._best() else ""
        return "G%d kp%g ki%g st%.3fs ov%.1f%% se%.2f%%%s" % (
            g, kp, ki, r.settle_s, r.overshoot_pct, r.ss_err_pct, mark)
