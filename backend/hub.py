"""hub.py — 实验数据源中枢（Qt-free，服务器版上位机的唯一数据通路）。

职责：模式（sim/real）+ 帧解码 + 广播给订阅者（服务器进程内，HTTP/SSE 侧只管转发）。
与旧 backend/bridge.py（PySide6 QWebChannel 桌面壳用）的区别：**不依赖 Qt**，
所以可以跑在纯 stdlib 的本地服务器里；解码/仿真源/轨迹重建仍走同一批单源模块
（frame.py / sim_source.py / trajectory.py），不复制协议逻辑。

契约（订阅者回调，kind 语义与旧 bridge 信号一一对应）：
    ("frame", (type, seq, vals))    解码值（IMU_SAMPLE=6 值 / IMU_CAL=19 值 /
                                    EULER=3 值弧度 / QUAT=4 值）
    ("text",  str)                  TEXT 帧
    ("stat",  (imu, text, drop))    计数（imu 只计 0x01；drop 按 IMU seq 回绕差）
    ("pos",   (x, y, z))            轨迹重建位置（仅 traj 页）
    ("port",  bool)                 连接状态变化

模式纪律（用户定案 2026-11-05）：open_sim / open_real 只能由页面显式请求触发，
本模块**不自动启动任何源**；open_* 前若已有源，先停旧的（先停后起，单一源）。
real 档目标与 motor_hub 同形（tcp:<host>:<port> / serial:<COMx>@<baud>）：连不上就抛
RuntimeError（服务器回 409），**任何情况都不回落 sim**。

调零 / 温补的归属（用户定案 2026-11-07，覆盖早前的「主机去偏」设计）：
**算法全部在 MCU 内部**，本模块只做「发命令 + 读数据」：
  · 下行：CMD 帧（TYPE=0x7D，payload = [MOD_ID][FUNC_ID][params…]），MOD_IMU=1；
    FUNC 码与参考原型 参考原型 MagicRobot 一致（0x01 CALIBRATE / 0x02 GET_BIAS /
    0x03 TEMP_CALIB / 0x09 SET_FLAGS / 0x0A GET_STATUS / 0x0D GET_DRIFT /
    0x10 GET_TC_STATE），编码单点 = encode_cmd()；
  · 上行：ACK 帧（TYPE=0x7E，payload = [MOD_ID][FUNC_ID][STATUS][data…]），
    解码单点 = decode_ack()；
  · **主机绝不减偏、不拟合**：0x01/0x02 帧原值透传给页面（去偏是固件内部的事，
    且由 SYSF_* 功能位决定），页面看到的零偏就是 MCU 自己算的零偏。
仿真档提供 **虚拟 MCU**（_SimMcu）：按同一套 CMD/ACK 语义应答（calibrate 后零偏非零、
 temp_calib 回 k、set_flags 切 tc_valid），并像固件一样推送 TEXT + 3 帧 0x02；
 仿真源自带的固有零偏（SIM_SENSOR_BIAS）是「虚拟传感器的物理模型」，主机不解不补。
仿真档帧型覆盖（页面逐级继承的前提）：0x03 由同一条 QUAT 仿真流按 ZYX 解算派生
（与固件 quat_to_euler 同式，不另起仿真器）。
"""

import math
import socket
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from frame import (FrameDecoder, PAYLOAD_MAX, SOF, crc8, decode_cal, decode_euler,
                   decode_quat, decode_sample, TYPE_ACK, TYPE_EULER, TYPE_IMU_CAL,
                   TYPE_IMU_SAMPLE, TYPE_QUAT, TYPE_TEXT)
from sim_source import (SIM_CAL_PRESETS, SIM_PRESETS, SIM_TRAJ_PRESETS,
                        SimCalCfg, SimCalSource, SimImuSource, SimTrajSource)
from trajectory import TrajCfg, TrajectoryReconstructor

SIM_CALIB_FRAMES = 3          # 每次标定完成固件连发 3 帧 0x02（契约）
SIM_CALIB_TRUTH = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)   # 静止 Z_UP 真值（g / dps）
# 仿真源（虚拟传感器）的固有零偏：采样序 ax..az g, gx..gz dps；与固件 sim.exp2.static
# 预设同源。**这是被模拟的物理偏移，不是主机的补偿量**——上位机不解不补，
# 虚拟 MCU 的 calibrate 就是把它量出来回报（页面看到的零偏即此值）。
SIM_SENSOR_BIAS = SIM_CAL_PRESETS["exp2_static"].bias
CONNECT_TIMEOUT_S = 5.0       # real 目标连接预算（明报失败，不干等）
READ_TIMEOUT_S = 0.2          # 读超时（s）：到点返回已有字节（POSIX 读语义）
DEFAULT_BAUD = 115200

# ── 下行命令通道（IMU 模块）：CMD/ACK 帧与 payload 布局的唯一对照点 ──
# 帧规格 = 本仓 proto（SOF 0xA5 | LEN | TYPE | SEQ | payload | CRC8，见 frame.py）：
#   CMD（上位机→固件） TYPE=0x7D, payload = [MOD_ID][FUNC_ID][params…]
#   ACK（固件→上位机） TYPE=0x7E, payload = [MOD_ID][FUNC_ID][STATUS][data…]
# STATUS=0 = 成功；非 0 由固件定义（本端只透传给页面）。
# 各 func 的 params / data 布局（全部小端；浮点=f32；六轴一律用采样序
# ax,ay,az（g）, gx,gy,gz（dps），与 0x02 帧零偏块同序）：
#   calibrate     params ()            data 6×f32 新零偏
#   get_bias      params ()            data 6×f32 当前零偏
#   temp_calib    params (秒 u8)        data 6×f32 温漂系数 k（每 °C）
#   set_flags     params (flags u16 LE) data u16 flags（生效值）
#   get_status    params ()            data u16 flags, u8 bias_valid, u8 tc_valid
#   get_drift     params ()            data 6×f32 温漂系数 k
#   get_tc_state  params ()            data u8 tc_valid, u8 enabled, 6×f32 k
# 固件侧（Embedded lane）必须按同一张表实现，否则「上位机发得出、MCU 不认」。
CMD_TYPE = 0x7D
MOD_IMU = 0x01
FUNC_IMU: Dict[str, int] = {
    "calibrate": 0x01,
    "get_bias": 0x02,
    "temp_calib": 0x03,
    "set_flags": 0x09,
    "get_status": 0x0A,
    "get_drift": 0x0D,
    "get_tc_state": 0x10,
}
AXES6_ORDER = "ax,ay,az,gx,gy,gz"      # 六轴 payload 序（文档/断言用）
SYSF_TEMPCOMP_EN = 0x0001               # SYS_FLAGS bit0：温度补偿生效（原型同义）
ST_OK = 0                                # ACK STATUS：成功
ACK_TIMEOUT_S = 5.0                      # 一般命令等 ACK 预算（s）
CALIB_TIMEOUT_S = 20.0                   # CALIBRATE 异步：固件采 500 样本（≈5s）后才回
TC_TIMEOUT_MARGIN_S = 10.0               # TEMP_CALIB：请求秒数 + 余量
CALIB_SAMPLES = 500                      # 固件标定窗（进度/回执文案口径）


def quat_to_euler_zyx(qw: float, qx: float, qy: float, qz: float):
    """四元数 (w,x,y,z) → ZYX 欧拉 (roll,pitch,yaw)，单位弧度。

    与固件 CoreLib quat.c quat_to_euler 同式：pitch 的 |sinp| >= 1 处钉 ±π/2
    （C 侧 copysignf 同义钉法），roll/yaw 用 atan2 全值域。仿真档 0x03 帧由本函数
    从 QUAT 流派生——与真实固件的两条姿态表示通道同源，不是第二套姿态模型。
    """
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = (math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0
             else math.asin(sinp))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return (roll, pitch, yaw)


def _milli(v: float) -> int:
    """float → 毫单位整数（TEXT 回执里的 mg/mdps/mC，镜像 sim_source._milli）。"""
    return int(round(v * 1000.0))


class _TcpLink:
    """tcp:<host>:<port> 链路（QEMU UART0 串口 TCP 桥）：read 超时返回 b""（POSIX 语义）。

    只做 read/write/close 三件事——hub 采集线程与 send_line 的下行都走这三个方法，
    传输面（TCP / 串口）对上层无差别。
    """

    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock

    def read(self, n: int) -> bytes:
        try:
            return self._sock.recv(n)
        except socket.timeout:
            return b""

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class _SerialLink:
    """serial:<COMx>@<baud> 链路（真板）：语义与 _TcpLink 同，底层是 pyserial。"""

    def __init__(self, ser) -> None:
        self._ser = ser

    def read(self, n: int) -> bytes:
        return self._ser.read(n)

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


def _open_link(target: str):
    """目标串 → 已连通的链路；失败抛 RuntimeError（服务器回 409）。

    只认两种形态（与 motor_hub._open_link 同形，页面直接用同一串）：
      tcp:<host>:<port>           QEMU UART0 串口桥 / 串口服转 TCP
      serial:<COMx>[@<baud>]      物理串口（省 @baud 时用 DEFAULT_BAUD）
    连接预算 CONNECT_TIMEOUT_S，读超时 READ_TIMEOUT_S；不认别的形态也不做任何回落。
    """
    text = (target or "").strip()
    if text.startswith("tcp:"):
        host, _, port = text[len("tcp:"):].rpartition(":")
        host = host or "127.0.0.1"
        try:
            port_n = int(port)
        except ValueError:
            raise RuntimeError(f"tcp 目标格式应为 tcp:<host>:<port>，得 {text!r}")
        try:
            sock = socket.create_connection((host, port_n), timeout=CONNECT_TIMEOUT_S)
        except OSError as exc:
            raise RuntimeError(f"连接 {host}:{port_n} 失败（{exc}）")
        sock.settimeout(READ_TIMEOUT_S)
        return _TcpLink(sock)
    if text.startswith("serial:"):
        spec = text[len("serial:"):]
        port, _, baud = spec.partition("@")
        port = port.strip()
        if not port:
            raise RuntimeError(f"serial 目标格式应为 serial:<COMx>[@baud]，得 {text!r}")
        try:
            import serial                       # pyserial（可选依赖，懒导入）
        except Exception as exc:                # pragma: no cover - 环境相关
            raise RuntimeError(f"serial 目标需要 pyserial（{exc}）")
        try:
            ser = serial.Serial(port, int(baud or DEFAULT_BAUD), timeout=READ_TIMEOUT_S)
        except Exception as exc:                # SerialException 及驱动层异常
            raise RuntimeError(f"串口 {port} 打不开（{exc}）")
        return _SerialLink(ser)
    raise RuntimeError(f"real 目标须为 tcp:<host>:<port> 或 serial:<COMx>@<baud>，得 {text!r}")


# ────────────────────────── 下行命令编解码（唯一单点） ──────────────────────────

def encode_cmd(func: int, params: bytes = b"", seq: int = 0, mod: int = MOD_IMU) -> bytes:
    """命令帧编码（host->device 唯一编码点）：SOF|LEN|0x7D|SEQ|[MOD][FUNC][params]|CRC8。

    入参：func = FUNC_IMU 里的功能码；params = 该 func 的参数字节（无参数传 b""）；
          seq = 帧序号（0..255 自增回绕，仅用于丢帧诊断）。
    出参：完整帧字节（直接写链路）。
    """
    body = bytes((mod, func)) + bytes(params)
    if len(body) > PAYLOAD_MAX:
        raise ValueError(f"CMD payload {len(body)}B 超过协议上限 {PAYLOAD_MAX}B")
    head = bytes((SOF, len(body), CMD_TYPE, seq & 0xFF))
    return head + body + bytes((crc8(head[1:] + body),))


def decode_ack(payload: bytes) -> Tuple[int, int, int, bytes]:
    """ACK payload → (mod, func, status, data)；长度不足抛 ValueError。

    契约：[MOD_ID][FUNC_ID][STATUS][data…]（与参考原型 参考原型 MagicRobot 同构）。
    """
    if len(payload) < 3:
        raise ValueError(f"ACK payload {len(payload)}B < 3B（应含 MOD/FUNC/STATUS）")
    return payload[0], payload[1], payload[2], bytes(payload[3:])


def _f32_list(data: bytes, n: int, what: str) -> List[float]:
    """data 按 n 个 float32 LE 解出（长度不符即报错：宁可不爽也不默默截断）。"""
    need = 4 * n
    if len(data) < need:
        raise ValueError(f"{what} 数据 {len(data)}B < {need}B（{n}×f32）")
    return list(struct.unpack("<" + "f" * n, data[:need]))


def _u16(data: bytes, off: int = 0) -> int:
    if len(data) < off + 2:
        raise ValueError(f"u16 字段越界（data {len(data)}B，off={off}）")
    return data[off] | (data[off + 1] << 8)


def _cmd_params(func_name: str, args: dict) -> bytes:
    """页面/API 参数 → CMD params 字节（布局见模块常量区四张表）。"""
    if func_name == "temp_calib":
        seconds = int(args.get("seconds", 30))
        if not 1 <= seconds <= 255:
            raise ValueError(f"temp_calib 秒数须在 1~255，得 {seconds}")
        return bytes((seconds,))
    if func_name == "set_flags":
        flags = int(args.get("flags", SYSF_TEMPCOMP_EN)) & 0xFFFF
        return bytes((flags & 0xFF, (flags >> 8) & 0xFF))
    return b""


def _cmd_payload_json(func_name: str, data: bytes) -> List[float]:
    """ACK data → JSON 数组（页面直接渲染；六轴序 = ax,ay,az,gx,gy,gz）。"""
    if func_name in ("calibrate", "get_bias", "temp_calib", "get_drift"):
        return _f32_list(data, 6, func_name)
    if func_name == "set_flags":
        return [_u16(data, 0)]
    if func_name == "get_status":
        if len(data) < 4:
            raise ValueError(f"get_status 数据 {len(data)}B < 4B（flags/bias_valid/tc_valid）")
        return [_u16(data, 0), data[2], data[3]]
    if func_name == "get_tc_state":
        if len(data) < 2:
            raise ValueError(f"get_tc_state 数据 {len(data)}B < 2B（valid/enabled）")
        return [data[0], data[1]] + _f32_list(data[2:], 6, func_name)
    raise ValueError(f"未知 func {func_name!r}")


def _cmd_timeout_s(func_name: str, args: dict) -> float:
    """等 ACK 的预算：CALIBRATE 异步采集（≈5s）、TEMP_CALIB 跟请求秒数走。"""
    if func_name == "calibrate":
        return CALIB_TIMEOUT_S
    if func_name == "temp_calib":
        return float(int(args.get("seconds", 30))) + TC_TIMEOUT_MARGIN_S
    return ACK_TIMEOUT_S


class _SimMcu:
    """仿真档的**虚拟 MCU**（无板时命令通道也能跑；语义与固件对齐，算法不在主机）。

    状态机（与固件的 bias_valid / tc_valid 两维口径一致）：
        bias = 0, bias_valid = 0      出厂未标定
        calibrate → 量到虚拟传感器的固有零偏（SIM_SENSOR_BIAS）→ bias_valid = 1
        temp_calib(秒) → 记下预设温漂系数 k（accel_drift+gyro_drift，采样序）
                            → tc_valid = 1（已标定；是否生效看 SYSF_TEMPCOMP_EN）
        set_flags → 写 flags（TEMPCOMP_EN 置/清）；不动 valid（开关≠标定）

    calibrate 成功后像固件一样往流上推送：TEXT `OK calib start N=…` → 3 帧 0x02
    （raw=真值+零偏 / deb=raw−零偏 / bias=本次零偏 / temp）→ TEXT `OK calib done bias …`。
    **注意**：仿真的 calibrate 立即应答（不真等 5 s 采集）——语义相同、时间尺度不同。
    """

    def __init__(self) -> None:
        cfg: SimCalCfg = SIM_CAL_PRESETS["exp2_static"]
        self._cfg = cfg
        self._bias = (0.0,) * 6                    # 未标定时零偏为 0（页面如实显示）
        self._k = tuple(cfg.accel_drift) + tuple(cfg.gyro_drift)   # 采样序 6 元（每 °C）
        self._flags = 0
        self._bias_valid = False
        self._tc_valid = False
        self._seq = 0                              # 合成的 TEXT/0x02 帧序号源

    # —— 命令处理（与固件 CMD/ACK 同一语义）——
    def handle(self, func_name: str, args: dict, emit) -> Tuple[int, bytes]:
        if func_name == "calibrate":
            return self._calibrate(emit)
        if func_name == "get_bias":
            return ST_OK, _pack_f32(self._bias)
        if func_name == "temp_calib":
            self._tc_valid = True
            return ST_OK, _pack_f32(self._k)
        if func_name == "set_flags":
            self._flags = _cmd_flags(args)
            return ST_OK, bytes((self._flags & 0xFF, (self._flags >> 8) & 0xFF))
        if func_name == "get_status":
            return ST_OK, bytes((self._flags & 0xFF, (self._flags >> 8) & 0xFF,
                                 1 if self._bias_valid else 0, 1 if self._tc_valid else 0))
        if func_name == "get_drift":
            return ST_OK, _pack_f32(self._k)
        if func_name == "get_tc_state":
            return ST_OK, bytes((1 if self._tc_valid else 0,
                                 1 if self._flags & SYSF_TEMPCOMP_EN else 0)) + _pack_f32(self._k)
        raise ValueError(f"未知 func {func_name!r}")

    def bias(self) -> tuple:
        """当前生效零偏（六轴，采样序 ax,ay,az,gx,gy,gz）——虚拟 MCU 的补偿量。

        供仿真档出帧时扣除（MCU 内部行为）；未标定时为全 0（如实：未补偿）。
        """
        return tuple(self._bias)

    def _calibrate(self, emit) -> Tuple[int, bytes]:
        cfg = self._cfg
        bias = tuple(SIM_SENSOR_BIAS)
        self._bias = bias
        self._bias_valid = True
        emit("text", f"OK calib start N={cfg.calib_frames}")
        raw = tuple(t + b for t, b in zip(SIM_CALIB_TRUTH, bias))
        deb = tuple(r - b for r, b in zip(raw, bias))
        vals = raw + deb + bias + (cfg.temp_start_c,)
        for _ in range(SIM_CALIB_FRAMES):
            self._seq = (self._seq + 1) & 0xFF
            emit("frame", (TYPE_IMU_CAL, self._seq, list(vals)))
        emit("text", (
            "OK calib done bias "
            f"ax={_milli(bias[0])} ay={_milli(bias[1])} az={_milli(bias[2])} mg "
            f"gx={_milli(bias[3])} gy={_milli(bias[4])} gz={_milli(bias[5])} mdps "
            f"temp={_milli(cfg.temp_start_c)} mC"))
        return ST_OK, _pack_f32(bias)


def _pack_f32(vals) -> bytes:
    """6 元（或一般数列）→ float32 LE 字节（ACK data 段）。"""
    return struct.pack("<" + "f" * len(vals), *vals)


def _cmd_flags(args: dict) -> int:
    """set_flags 参数 → flags 值（与 _cmd_params 同一规则，仿真侧复用）。"""
    return int(args.get("flags", SYSF_TEMPCOMP_EN)) & 0xFFFF


class FrameHub:
    """单源中枢：同一时刻最多一路数据源；订阅者按注册顺序收到回调。

    参数：
      sim_kind  — 仿真源选型："imu"（exp1）/ "cal"（exp2）/ "traj"（exp4/exp5）
      traj_cfg  — 非 None 时在 hub 线程内跑 TrajectoryReconstructor（exp5），
                  发出 ("pos", (x,y,z))；QUAT+IMU 双流配对喂入
      poll_ms   — 仿真源轮询周期（ms，缺省 10ms@100Hz；绝对截止调度，不漂移）
    """

    def __init__(self, sim_kind: str = "imu",
                 traj_cfg: Optional[TrajCfg] = None, poll_ms: float = 10.0) -> None:
        if sim_kind not in ("imu", "cal", "traj"):
            raise ValueError(f"sim_kind={sim_kind!r} 须为 imu/cal/traj")
        self._sim_kind = sim_kind
        self._traj_cfg = traj_cfg
        self._poll_ms = poll_ms
        self._lock = threading.Lock()
        self._listeners: List[Callable[[str, object], None]] = []
        self._stop: Optional[threading.Event] = None
        self._thread: Optional[threading.Thread] = None
        self._source = None          # sim 源对象（有 poll(now_s) 的鸭子类型）
        self._link = None            # real 档链路（_TcpLink / _SerialLink）
        self._mode = "none"          # none / sim / real
        self._transport = "none"     # none / sim / tcp / serial（state.source）
        self._connected = False
        self._target = ""            # real 档目标串（tcp:… / serial:…）/ sim 档 "sim"
        self._counters = {"imu": 0, "text": 0, "drop": 0}
        self._last_seq = -1
        self._last_quat: Optional[List[float]] = None
        self._sim_seq = 0            # 仿真档派生 0x03 帧的序号源
        self._mcu: Optional[_SimMcu] = None    # 仿真档的虚拟 MCU（real 档为 None）
        # 命令通道：一次一条在飞（_cmd_lock 串行化），ACK 按 FUNC 匹配
        self._cmd_lock = threading.Lock()
        self._cmd_seq = 0
        self._ack_wait: Dict[int, Tuple[threading.Event, dict]] = {}

    # ── 订阅 ──
    def add_listener(self, fn: Callable[[str, object], None]) -> None:
        with self._lock:
            self._listeners.append(fn)

    def remove_listener(self, fn: Callable[[str, object], None]) -> None:
        with self._lock:
            try:
                self._listeners.remove(fn)
            except ValueError:
                pass

    def _emit(self, kind: str, payload) -> None:
        with self._lock:
            listeners = list(self._listeners)
        for fn in listeners:
            try:
                fn(kind, payload)
            except Exception:      # 单个订阅者异常不拖垮数据通路
                pass

    # ── 状态 ──
    def state(self) -> dict:
        return {
            "mode": self._mode,
            "source": self._transport,
            "connected": self._connected,
            "target": self._target,
            "sim_kind": self._sim_kind,
            "traj": self._traj_cfg is not None,
            "counters": dict(self._counters),
        }

    # ── 启停（先停后起：单一源，不抢资源）──
    def open_sim(self) -> dict:
        self._halt()
        if self._sim_kind == "cal":
            src = SimCalSource(SIM_CAL_PRESETS["exp2_static"])
        elif self._sim_kind == "traj":
            src = SimTrajSource(SIM_TRAJ_PRESETS["traj_sway"])
        else:
            src = SimImuSource(SIM_PRESETS["sway10_2s"])
        self._mcu = _SimMcu()          # 虚拟 MCU：命令通道在无板时可用
        self._start_loop(mode="sim", source=src, target="sim", transport="sim")
        return self.state()

    def open_real(self, target: str) -> dict:
        """真实链路：target 为 tcp:<host>:<port> 或 serial:<COMx>@<baud>。

        先连接（失败立即抛 RuntimeError → 服务器回 409，状态留在 none，**不回落 sim**），
        连上后才起读线程。real 档不做主机去偏（固件 CAL 内部已去偏）。
        """
        self._halt()
        link = _open_link(target)
        text = target.strip()
        self._start_loop(mode="real", source=None, target=text,
                         transport=("tcp" if text.startswith("tcp:") else "serial"),
                         link=link)
        return self.state()

    def close(self) -> dict:
        self._halt()
        return self.state()

    # ── 命令通道（CMD 帧 → ACK 帧）──
    def imu_cmd(self, func_name: str, args: Optional[dict] = None) -> dict:
        """下发一条 IMU 模块命令并等 ACK；返回 {"func","status","payload"}。

        real 档：写 CMD 帧（encode_cmd）→ 等固件 ACK（按 FUNC 匹配，带超时）；
        sim 档：由 _SimMcu 按同一语义应答（虚拟 MCU，不碰链路）；
        未连接 / 超时 / 帧格式不符 → 抛 RuntimeError（服务器回 409，页面显示明确失败）。
        主机不做任何减偏/拟合——这里只有命令与读数。
        """
        args = dict(args or {})
        func_name = str(func_name or "").strip().lower()   # 大小写不敏感（统一口径）
        if func_name not in FUNC_IMU:
            raise ValueError(f"未知 func {func_name!r}（可选 {sorted(FUNC_IMU)}）")
        params = _cmd_params(func_name, args)
        timeout = _cmd_timeout_s(func_name, args)
        if self._link is not None:
            status, data = self._cmd_via_link(FUNC_IMU[func_name], params, timeout)
        elif self._mode == "sim" and self._mcu is not None:
            with self._cmd_lock:                    # 与真档同一条串行口径
                status, data = self._mcu.handle(func_name, args, self._emit)
        else:
            raise RuntimeError("未连接（先在仿真或真实模式启动数据源，再下发命令）")
        payload = _cmd_payload_json(func_name, data) if status == ST_OK else []
        return {"func": func_name, "status": int(status), "payload": payload}

    def _cmd_via_link(self, func: int, params: bytes, timeout: float):
        """写 CMD 帧 → 等 ACK（按 FUNC 匹配）：返回 (status, data)。"""
        with self._cmd_lock:
            waiter = (threading.Event(), {})
            self._ack_wait[func] = waiter
            ev, holder = waiter
            try:
                self._cmd_seq = (self._cmd_seq + 1) & 0xFF
                self._link.write(encode_cmd(func, params, self._cmd_seq))
                if not ev.wait(timeout):
                    raise RuntimeError(
                        f"等 MCU 回执超时（{timeout:.0f}s，CMD func=0x{func:02X}）")
                return holder["result"]
            finally:
                self._ack_wait.pop(func, None)

    def _on_ack(self, payload: bytes) -> None:
        """ACK 帧分发：交给在等该 FUNC 的调用方（不在等就当作诊断帧忽略）。"""
        try:
            _mod, func, status, data = decode_ack(payload)
        except ValueError as exc:
            self._emit("text", f"[hub] 丢弃非法 ACK：{exc}")
            return
        waiter = self._ack_wait.get(func)
        if waiter is None:
            return
        ev, holder = waiter
        holder["result"] = (status, data)
        ev.set()

    # ── 内部 ──
    def _start_loop(self, mode: str, source, target: str, transport: str,
                    link=None) -> None:
        self._mode, self._source, self._link, self._target = mode, source, link, target
        self._transport = transport
        self._counters = {"imu": 0, "text": 0, "drop": 0}
        self._last_seq = -1
        self._last_quat = None
        self._sim_seq = 0
        self._stop = threading.Event()
        self._connected = True
        self._emit("port", True)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _halt(self) -> None:
        stop = self._stop
        if stop is not None:
            stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(1.5)
        self._stop = None
        self._thread = None
        if self._link is not None:
            self._link.close()
        self._link = None
        self._source = None
        self._mcu = None
        self._mode = "none"
        self._transport = "none"
        self._target = ""
        for ev, _holder in list(self._ack_wait.values()):     # 收口在等 ACK 的命令
            ev.set()
        self._ack_wait.clear()
        if self._connected:
            self._connected = False
            self._emit("port", False)

    def _loop(self) -> None:
        stop = self._stop
        dec = FrameDecoder()
        traj = TrajectoryReconstructor(self._traj_cfg) if self._traj_cfg else None
        t0 = time.monotonic()
        next_deadline = t0
        try:
            while not stop.is_set():
                if self._source is not None:            # 仿真档：绝对截止调度
                    now = time.monotonic()
                    wait = next_deadline - now
                    if wait > 0 and stop.wait(min(wait, 0.05)):
                        break
                    next_deadline += self._poll_ms / 1000.0
                    now = time.monotonic()
                    if next_deadline < now - 0.25:       # 线程被挂起后不追补
                        next_deadline = now
                    chunk = self._source.poll(now - t0)
                else:                                   # real 档：链路读（自带 timeout）
                    chunk = self._link.read(512)
                if not chunk:
                    continue
                for typ, seq, payload in dec.feed(chunk):
                    self._dispatch(typ, seq, payload, traj)
        except Exception as exc:                        # 串口拔出/占用等
            self._emit("text", f"[hub] 数据源异常：{exc}")
        finally:
            self._connected = False
            self._emit("port", False)

    def _dispatch(self, typ: int, seq: int, payload: bytes, traj) -> None:
        if typ == TYPE_IMU_SAMPLE:
            self._counters["imu"] += 1
            if self._last_seq >= 0:
                self._counters["drop"] += (seq - (self._last_seq + 1)) & 0xFF
            self._last_seq = seq
            vals = list(decode_sample(payload))
            if self._source is not None:
                # 仿真档 = 虚拟传感器 + 虚拟 MCU 的物理模型（**主机不做算法，只转发**）：
                #   ① 合成源出理想信号 → 叠加虚拟传感器的固有零偏（物理量）；
                #   ② 减去「虚拟 MCU 当前生效的零偏」= MCU 内部的补偿结果
                #      （未标定时 mcu.bias=0 → 流上带零偏/轨迹漂移；
                #        一键调零后 mcu.bias=固有零偏 → 流被补偿干净）。
                # 这一段是**模拟设备**的行为，不是上位机算法（META-011）。
                vals = [v + b for v, b in zip(vals, SIM_SENSOR_BIAS)]
                mb = self._mcu.bias() if self._mcu is not None else (0.0,) * 6
                vals = [v - c for v, c in zip(vals, mb)]
            self._emit("frame", (typ, seq, vals))
            if traj is not None and self._last_quat is not None:
                w, x, y, z = self._last_quat
                px, py, pz = traj.feed(w, x, y, z, vals[0], vals[1], vals[2])
                self._emit("pos", (px, py, pz))
        elif typ == TYPE_IMU_CAL:
            self._emit("frame", (typ, seq, list(decode_cal(payload))))
        elif typ == TYPE_EULER:
            self._emit("frame", (typ, seq, list(decode_euler(payload))))
        elif typ == TYPE_QUAT:
            qv = list(decode_quat(payload))
            self._last_quat = qv
            self._emit("frame", (typ, seq, qv))
            if self._source is not None:
                # 仿真档没有 0x03 字节流：由同一条 QUAT 流按 ZYX 解算派生（同固件式），
                # 页面在无板时也能看到欧拉通道；real 档一律以固件 0x03 帧为准。
                self._emit("frame", (TYPE_EULER, seq, list(quat_to_euler_zyx(*qv))))
        elif typ == TYPE_TEXT:
            self._counters["text"] += 1
            self._emit("text", payload.decode("ascii", "replace"))
        elif typ == TYPE_ACK:
            self._on_ack(payload)
        if self._counters["imu"] and self._counters["imu"] % 20 == 0:
            self._emit("stat", (self._counters["imu"], self._counters["text"],
                                self._counters["drop"]))
