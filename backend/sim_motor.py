"""电机帧合成源（scope_motor.py --source sim 的缺省数据源；纯标准库零依赖）。

意图：把「示波器渲染 + 数值断言」链路从外部依赖（QEMU TCP 桥 / 真实串口）解耦——
本模块按帧计数合成与固件同规格的 0x05 MOTOR（4 槽 64B）与 0x06 SCEN 帧
（记录布局与解码契约见 backend/scope_motor.py 模块 docstring，CRC 复用
backend/frame.py 单源），仿真档因此零外部依赖即可跑通 demo 与五个场景
（pwm/servo/enc/nested/dc）的全部既有断言。

时间基（确定性：不用墙钟取值、不用随机数）：
  - 每帧推进 dt_s = 1/500 s（= 固件 POLL_US 2000us 采样口径）；
  - 流帧率 frame_rate_hz = 1000 帧/墙钟秒 → 2× 仿真时间压缩。仿真档不复刻
    实时（无 plant 时限约束），压缩让常规采集长度内能完整覆盖最长 16s 场景
    循环；帧值与步进都是「帧序号」的确定性函数，同一采集长度可复现。

契约表注入（单源纪律）：场景步表/定时器/槽 kind（scope_motor.SCEN，镜像
固件 motor_scen_table.c）与 SG90 标定（scope_motor.SG90，镜像 motor.servo.sg90）
由调用方传入——本模块只提供物理模型（S 形腿 + dwell 停驻 / 一阶 plant 滞后 /
脉宽换算），不复制这些表。
"""

import math
import struct
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

from frame import crc8

SOF = 0xA5
TYPE_MOTOR = 0x05
TYPE_SCEN = 0x06
MOTOR_SLOTS = 4
MOTOR_REC = struct.Struct("<BBfffBB")   # 16B：slot,kind,setpoint,duty,actual,phase,fault
SCEN_TIMER_REC = struct.Struct("<BBI")  # 6B：tim_idx,channels,freq_hz（0x06 快照）
PHASE_DONE = 7                          # scurve_phase_t 终态（停驻段）
TIME_EPS_S = 1e-9                       # 帧到期判定容忍（s）
SECONDS_PER_US = 1e-6

# 各场景每步保持时长（s，按槽序）：镜像固件 scen 表 hold_ticks × POLL_US 2000us
#   pwm 1000 拍=2s｜servo 1000 拍=2s｜enc 2000 拍=4s｜nested A/B 1000 拍=2s、C 1250 拍=2.5s
#   ｜dc 1000 拍=2s
SCEN_HOLD_S = {
    1: (2.0,),            # pwm：5 档 × 2s = 10s 一循环
    2: (2.0,),            # servo：8 点 × 2s = 16s 一循环
    3: (4.0,),            # enc：4 段 × 4s = 16s 一循环
    4: (2.0, 2.0, 2.5),   # nested：A(ch0)/B(ch1) 2s，C(独立 50Hz) 2.5s——均 10s 一循环
    5: (2.0,),            # dc：+0.5/−0.5 各 2s = 4s 一循环
}


@dataclass(frozen=True)
class SimMotorCfg:
    """SimMotorSource 参数表（物理模型与演示节奏；改模型只改这里）。

    单位：frame_rate_hz=帧/墙钟秒（>0）；dt_s=s（每帧仿真步长，>0）；
          dwell_s=s（演示停驻段时长，>0）；leg_s=s（演示单腿 S 形时长，>0）；
          plant_rpm_per_duty=rpm/duty（>0，ENC 合成 plant 增益）；
          plant_tau_s=s（>0，一阶滞后）；demo_servo_hz=Hz（演示舵机定时器频率）。
    """

    frame_rate_hz: float = 1000.0
    dt_s: float = 0.002
    dwell_s: float = 1.0     # 镜像 MOTOR_DEMO_DWELL_TICKS 500 拍 × 2ms
    leg_s: float = 0.6       # 首腿 ≤0.8s（500 拍自检窗口可见完整 S 形）
    plant_rpm_per_duty: float = 160.0   # 镜像 motor.enc.n20 / scen 表 plant 增益
    plant_tau_s: float = 0.02           # 同上
    demo_servo_hz: float = 50.0         # 镜像机 pwm.servo.50（槽1/槽3 挂 50Hz 定时器）

    def __post_init__(self):
        if self.frame_rate_hz <= 0:
            raise ValueError(f"frame_rate_hz={self.frame_rate_hz} 必须 > 0（帧/墙钟秒）")
        for name in ("dt_s", "dwell_s", "leg_s", "plant_rpm_per_duty", "plant_tau_s",
                     "demo_servo_hz"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name}={getattr(self, name)} 必须 > 0")


@dataclass(frozen=True)
class SimDemoSlot:
    """演示模式单槽运动设计（域与断言域同名；targets = 交替往返目标，镜像固件
    motor_demo_table.c 的 target[2]）。"""

    domain: str                        # duty / deg / rpm / sduty
    targets: Tuple[float, float]


# 槽序与 scope_motor.SLOT_NAME 一致；targets 镜像固件 demo 表：
# 槽0 PWM duty 0.8/0.2｜槽1 SERVO 135/45°｜槽2 ENC 120/0 rpm｜槽3 DC ±0.5
DEMO_SLOTS = (
    SimDemoSlot("duty", (0.8, 0.2)),
    SimDemoSlot("deg", (135.0, 45.0)),
    SimDemoSlot("rpm", (120.0, 0.0)),
    SimDemoSlot("sduty", (0.5, -0.5)),
)


def encode_motor(seq: int, recs: Sequence[bytes]) -> bytes:
    """0x05 MOTOR 整帧字节（recs = 4 条 16B 记录，槽序 0..3）。"""
    payload = b"".join(recs)
    head = bytes((SOF, len(payload), TYPE_MOTOR, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


def encode_scen(seq: int, scen_id: int, timers: Sequence[Tuple[int, int, int]],
                recs: Sequence[bytes]) -> bytes:
    """0x06 SCEN 整帧字节：场景头（id,n_tim,timers,n_rec）+ 记录（16B 同构）。"""
    payload = bytearray((scen_id, len(timers)))
    for idx, ch, freq in timers:
        payload += SCEN_TIMER_REC.pack(idx, ch, freq)
    payload.append(len(recs))
    payload += b"".join(recs)
    head = bytes((SOF, len(payload), TYPE_SCEN, seq & 0xFF))
    return head + bytes(payload) + bytes((crc8(head[1:] + bytes(payload)),))


class SimMotorSource:
    """电机合成帧源（假串口：poll(now_s) 产出到期帧字节；command() 收下行命令）。

    帧序号：0x05/0x06 共用单一计数器连续自增（&0xFF 回绕，镜像固件 s_seq），
    供消费端丢帧检测复用。模式：缺省 demo（0x05 四槽）；收到「SCEN <n>」后切
    到该场景（0x06）并从步 0 重新计时；同号重发幂等（不重启——握手重发不应把
    已激活场景重置回步 0）。

    演示模型（每槽独立节拍，共用一个时刻）：腿 leg_s 秒走 smoothstep（单调 S
    形，端点导数为 0）从 targets[k%2] 到 targets[(k+1)%2]，随后 dwell_s 秒停驻
    （phase=7），再交替回程。ENC 域（rpm）actual = 一阶滞后跟踪 setpoint
    （τ=plant_tau_s），duty = clamp(actual/plant_rpm_per_duty, 0, 1)。
    """

    def __init__(self, scen: Dict[str, dict], slot_kinds: Sequence[int],
                 sg90: Dict[str, float], cfg: Optional[SimMotorCfg] = None):
        """入参：scen = 场景表（scope_motor.SCEN 形状：id/slots/timers/steps）；
        slot_kinds = 演示槽 kind 表（scope_motor.SLOT_KIND）；sg90 = 舵机标定
        （scope_motor.SG90：pmin_us/pmax_us/amin/amax）；cfg = None 取默认参数表。
        """
        self._cfg = cfg if cfg is not None else SimMotorCfg()
        self._scen = scen
        self._slot_kinds = tuple(slot_kinds)
        self._sg90 = sg90
        if len(DEMO_SLOTS) != len(self._slot_kinds):
            raise ValueError(
                f"演示槽数不配：DEMO_SLOTS={len(DEMO_SLOTS)} slot_kinds={len(self._slot_kinds)}")
        self._by_id = {info["id"]: (name, info) for name, info in scen.items()}
        for scen_id, holds in SCEN_HOLD_S.items():
            if scen_id not in self._by_id:
                raise ValueError(f"场景表缺 id={scen_id}（SCEN_HOLD_S 与场景表不配）")
            if len(holds) != len(self._by_id[scen_id][1]["slots"]):
                raise ValueError(f"场景 id={scen_id} 槽数与 SCEN_HOLD_S 保持表不配")
        self.frame_dt_s = self._cfg.dt_s    # 每帧仿真时长（消费端 dt 显示/rise 统计用）
        self._n_emit = 0                    # 已到期帧数（流节拍源）
        self._seq = 0                       # 帧序号（0x05/0x06 共用）
        self._mode = None                   # None = demo；int = 场景号
        self._t = 0.0                       # 当前模式内的仿真时刻（s）
        self._lag = [0.0] * len(DEMO_SLOTS)  # rpm 域槽的一阶滞后状态

    @property
    def frame_rate_hz(self) -> float:
        """流帧率（帧/墙钟秒）——假串口适配器据此限速。"""
        return self._cfg.frame_rate_hz

    @property
    def mode(self):
        """当前模式：None = demo；int = 场景号。"""
        return self._mode

    def next_due_s(self) -> float:
        """下一帧到期的流内时刻（s），供假串口适配器计算等待量。"""
        return (self._n_emit + 1) / self._cfg.frame_rate_hz

    def command(self, text: str) -> bool:
        """下行命令（固件行协议）："SCEN <n>" 切场景。返回是否被接受。

        幂等：切到当前场景的同号命令不重启（握手会重发同一条）。
        """
        parts = text.strip().split()
        if len(parts) < 2 or parts[0].upper() != "SCEN" or not parts[1].isdigit():
            return False
        scen_id = int(parts[1])
        if scen_id not in self._by_id:
            return False                      # 未知场景号：与固件一致地忽略
        if self._mode == scen_id:
            return True
        self._mode = scen_id
        self._t = 0.0                         # 场景从步 0 重新计时
        self._lag = [0.0] * len(self._lag)
        return True

    def poll(self, now_s: float) -> bytes:
        """假串口读接口：产出 (上一帧到期, now_s] 内到期的帧字节。

        入参 now_s：流内时间（s），单调不减（回退 = 不产帧）。出参：0..N 帧拼接
        字节，直接喂 FrameDecoder.feed——与真实串口完全相同的解码路径。
        """
        out = bytearray()
        rate = self._cfg.frame_rate_hz
        while (self._n_emit + 1) / rate <= now_s + TIME_EPS_S:
            self._n_emit += 1
            self._emit(out)
        return bytes(out)

    # ── 私有：逐帧产出 ──

    def _emit(self, out: bytearray) -> None:
        self._seq += 1
        if self._mode is None:
            out += encode_motor(self._seq, self._demo_recs(self._t))
        else:
            _, info = self._by_id[self._mode]
            out += encode_scen(self._seq, self._mode, info["timers"],
                                self._scen_recs(info, self._t))
        self._t += self._cfg.dt_s

    def _demo_recs(self, t_s: float) -> list:
        cfg = self._cfg
        recs = []
        for slot, design in enumerate(DEMO_SLOTS):
            sp, phase = demo_setpoint(t_s, design.targets, cfg.leg_s, cfg.dwell_s)
            du, ac = self._apply(slot, design.domain, sp, cfg.demo_servo_hz)
            recs.append(MOTOR_REC.pack(slot, self._slot_kinds[slot], sp, du, ac, phase, 0))
        return recs

    def _scen_recs(self, info: dict, t_s: float) -> list:
        holds = SCEN_HOLD_S[info["id"]]
        servo_hz = min(freq for _, _, freq in info["timers"])   # 慢定时器 = 舵机通道
        recs = []
        for r, slot in enumerate(info["slots"]):
            steps = slot["steps"]                 # 步表按槽（镜像固件每槽 s_steps_*）
            idx, _ = step_index(t_s, len(steps), holds[r])
            sp = steps[idx]
            du, ac = self._apply(r, slot["domain"], sp, servo_hz)
            recs.append(MOTOR_REC.pack(r, slot["kind"], sp, du, ac, idx, 0))
        return recs

    def _apply(self, slot: int, domain: str, sp: float, servo_hz: float):
        """域模型：(setpoint) → (duty 回读, actual)。"""
        cfg = self._cfg
        if domain in ("duty", "sduty"):
            return sp, sp                     # 开环直通（PWM 存储回读 / H 桥带符号回读）
        if domain == "deg":
            return servo_duty(sp, servo_hz, self._sg90), sp
        # rpm（ENC 闭环）：一阶 plant 滞后 + duty 按 plant 增益反解并夹在 [0,1]
        alpha = 1.0 - math.exp(-cfg.dt_s / cfg.plant_tau_s)
        self._lag[slot] += (sp - self._lag[slot]) * alpha
        ac = self._lag[slot]
        du = max(0.0, min(1.0, ac / cfg.plant_rpm_per_duty))
        return du, ac


def demo_setpoint(t_s: float, targets: Sequence[float], leg_s: float, dwell_s: float):
    """演示时刻 t_s 的 (setpoint, phase)：腿 = smoothstep 单调 S 形，停驻 = phase 7。"""
    cycle = leg_s + dwell_s
    k = int(t_s / cycle)
    u = t_s - k * cycle
    a = targets[k % 2]
    b = targets[(k + 1) % 2]
    if u < leg_s:
        p = u / leg_s
        return a + (b - a) * (p * p * (3.0 - 2.0 * p)), min(6, int(p * 7.0))
    return b, PHASE_DONE


def step_index(t_s: float, n_steps: int, hold_s: float):
    """循环步进时刻 t_s → (步下标, 步内进度 u∈[0,1))；hold_s = 每步保持时长（s）。"""
    k = int(t_s / hold_s)
    return k % n_steps, t_s / hold_s - k


def servo_duty(angle_deg: float, freq_hz: float, sg90: Dict[str, float]) -> float:
    """舵机角度 → 占空回读 duty = 脉宽/周期（SG90 线性标定，镜像固件预设）。"""
    pmin, pmax = sg90["pmin_us"], sg90["pmax_us"]
    amin, amax = sg90["amin"], sg90["amax"]
    us = pmin + (angle_deg - amin) * (pmax - pmin) / (amax - amin)
    return us * SECONDS_PER_US * freq_hz
