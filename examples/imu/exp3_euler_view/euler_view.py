#!/usr/bin/env python
# euler_view.py — 实验三欧拉角 host 页服务（stdlib 零第三方依赖，自研）
#
# 定位：exp3_euler 固件 0x03 欧拉帧的上位机演示页——数值表 + 三轴条形 canvas，
#       不走 app.py/PySide6/QWebChannel 主壳（那是 exp1/exp5 波形/轨迹页的
#       依赖链），本页 sim/tcp 挡零第三方：http.server + socket + struct 即全量；
#       只有 real 挡的 `serial:` 目标懒导入 pyserial（缺包不崩、不回落仿真）。
# 数据源只有两种**显式**模式（用户定案 2026-11-05：禁止隐式回落）：
#   sim   仿真：本机合成流：镜像固件合成源预设 sim.exp3.sway（绕 Y 纯摆，
#         幅值/周期/采样率可调，默认 20°/4s/100Hz）——先编码成 0x03
#         协议帧字节流、再走与 real 挡同一个解码器（字节级闭环，
#         不是直接喂数值，保证演示的就是协议链路本身）。
#   real  真实：只连用户显式指定的**一个目标串**——`serial:<COMx>@<baud>`（物理串口，
#         pyserial）或 `tcp:<host>:<port>`（设备串口转 TCP / QEMU UART0 桥，缺省
#         127.0.0.1:5558）——收真实固件发出的 0x03 帧；**不换目标、不回落仿真**。
#   none  空闲（缺省）：不带 --mode/--source 时只提供页面，**不启动任何数据源**，
#         等页面点「启动仿真 / 连接真实」经 /api/mode 显式切换。
# 页面：本目录 index.html（/api/euler 100ms 轮询 JSON + /api/mode 显式切换 + /api/imu/cmd 命令，
#       zh/en 双语词典内联在页面里）。
#
# 零点校准 **全部在 MCU 内部**（采集 / 拟合 / 去偏都在固件，上位机只发命令 + 读数据；
# 参考原型 参考原型 MagicRobot embedding modules/imu/app_imu.c）：
#   POST /api/imu/cmd {"func":"...", "args":{...}} → 上行 CMD 帧（0x7D），等 MCU ACK（0x7E）：
#     GET_BIAS(0x02)   → 读当前零偏 24B（gx,gy,gz,ax,ay,az，镜像 imu_bias_t）
#     GET_STATUS(0x0A) → 读 20B 状态（流/掩码/Hz/寄存器/flags/bias_valid/tc_valid/kp/ki/odr）
#     CALIBRATE(0x01)  → 一键调零（**异步**：固件采集完毕才回 ACK，期间页面显示「校准中…」）
#     TEMP_CALIB(0x03) → 温漂标定（参数=秒，异步；采集期需温差 ≥ 3°C）
#     SET_FLAGS(0x09)  → 开关位（bits: TEMPCOMP_EN=bit0；只控制「是否应用」，不改 tc_valid）
#     GET_TC_STATE(0x10) / GET_DRIFT(0x0D) → 温补状态快照 / 温漂系数
#     sim 档无设备：由内置**虚拟 MCU**（VirtualMcu，见类注释）按同一帧语义应答，
#     不是字符串命令，也不在主机侧做任何去偏。
#   未启动数据源/未连接 → {"code":"notConnected"}；ACK 超时 → {"code":"timeout"}。
#   0x02 IMU_CAL（76B = 19×float32：raw6 | deb6 | bias6 | temp）仅供**只读展示**：
#   deb 已由 MCU 去偏，主机绝不二次减——快照 `cal` 字段如实透传最近一帧。
#
# 帧规格单源 = 固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md）（INT-2 契约）：
#   SOF=0xA5 | LEN(1B) | TYPE(1B) | SEQ(1B) | PAYLOAD(LEN B) | CRC8(1B)
#   TYPE 0x02 IMU_CAL：payload = raw6|deb6|bias6|temp = 19×float32 LE = 76B
#                       （块序 ax,ay,az g｜gx,gy,gz dps；deb 已由固件去偏）
#   TYPE 0x03 EULER：payload = roll,pitch,yaw 3×float32 LE = 12B，弧度，ZYX。
#   TYPE 0x7D CMD  ：payload = MOD_ID | FUNC_ID | params…（本页只用 MOD_IMU=0x01；
#                    0x7D 在 frame.h 尚未登记——本页先按定案契约实现，登记后此处对齐）
#   TYPE 0x7E ACK  ：payload = MOD_ID | FUNC_ID | STATUS | data…（STATUS: 0=OK 1=参数错 2=执行失败 0xFF=未知功能码）
#   TYPE 0x7F TEXT ：ASCII 行（开机横幅 / 固件日志）
#
# 用法示例：
#   python euler_view.py                                # 空闲：只开页面，模式待页面点选
#   python euler_view.py --mode sim                     # 仿真合成演示（虚拟 MCU 应答命令）
#   python euler_view.py --mode real --tcp 127.0.0.1:5558   # 真实：连 QEMU UART0 桥
#   python euler_view.py --mode real --serial COM5      # 真实：连物理串口（波特率 --baud 115200）
#   python euler_view.py --lang en                      # 控制台用英文（缺省 zh）
#   curl -X POST -d '{"func":"GET_BIAS"}' http://127.0.0.1:8765/api/imu/cmd   # 读零偏（手测）
#   curl -X POST -d '{"func":"CALIBRATE"}' http://127.0.0.1:8765/api/imu/cmd   # 一键调零（等 ACK）

import argparse
import json
import math
import queue
import socket
import struct
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

# ---- 帧协议常量（契约镜像，非可调参数；单源见 proto/frame.h） ----
SOF = 0xA5
PAYLOAD_MAX = 96
TYPE_IMU_CAL = 0x02
TYPE_EULER = 0x03
TYPE_CMD = 0x7D                   # host→MCU 命令帧 payload=[MOD][FUNC][params…]
TYPE_ACK = 0x7E                   # MCU→host 应答帧 payload=[MOD][FUNC][STATUS][data…]
TYPE_TEXT = 0x7F
EULER_FLOATS = 3
EULER_PAYLOAD_B = EULER_FLOATS * 4
CAL_FLOATS = 19
CAL_PAYLOAD_B = CAL_FLOATS * 4    # 0x02 payload = raw6|deb6|bias6|temp（proto/frame.h TYPE_IMU_CAL）
ACK_HEAD_B = 3                    # [MOD_ID][FUNC_ID][STATUS]
ACK_OK = 0x00

# MOD / FUNC 码表（镜像原型 参考原型 MagicRobot embedded/src/serial.h + modules/imu/README.md）
MOD_IMU = 0x01
FUNC_IMU = {
    "CALIBRATE": 0x01,       # 异步：采集 500 样本后回 ACK（期间无回执）
    "GET_BIAS": 0x02,        # ACK 附 24B 零偏（gx,gy,gz,ax,ay,az）
    "TEMP_CALIB": 0x03,      # 参数=秒（异步，采集期需变温）
    "SET_FLAGS": 0x09,       # 参数=flags u16 小端
    "GET_STATUS": 0x0A,      # ACK 附 20B 状态快照
    "GET_DRIFT": 0x0D,       # ACK 附 24B 温漂（gyro[3]+accel[3]）
    "GET_TC_STATE": 0x10,    # ACK 附 28B 温补状态快照
}
FUNC_IMU_NAME = {v: k for k, v in FUNC_IMU.items()}
BIAS_BYTES = 24                  # imu_bias_t = 6×float（gx,gy,gz,ax,ay,az——**陀螺在前**）
STATUS_BYTES = 20                # GET_STATUS 数据块（偏移定义见 _parse_ack）
TC_STATE_BYTES = 28              # GET_TC_STATE 数据块
DRIFT_BYTES = 24                 # GET_DRIFT 数据块（gyro[3] + accel[3]）
SYSF_TEMPCOMP_EN = 0x0001        # SYS_FLAGS bit0：温补开关（≠ tc_valid：这是「是否应用」）
STATUS_NAMES = {0x00: "ok", 0x01: "badArgs", 0x02: "execFailed", 0xFF: "unknownFunc"}

CMD_TIMEOUT_S = 3.0              # 同步命令（GET_*/SET_FLAGS）等 ACK 上限（s）
CALIBRATE_TIMEOUT_S = 20.0       # CALIBRATE 异步：500 样本@100Hz ≈ 5s，留余量
TEMP_CALIB_MARGIN_S = 15.0       # TEMP_CALIB 等待 = 秒参数 + 采集收尾余量（s）

# sim 档虚拟 MCU 的镜像预设（镜像 sim.exp2.static + cal.exp2.bias）。
# 注意两条通道服务两个教学点：虚拟 MCU 的六轴通道是**静止 Z_UP**+注入零偏
#（调零的前提就是传感器不动），而 0x03 欧拉通道仍是 sway 摆动（姿态演示）。
SIM_MCU_TRUTH = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)      # 静止 Z_UP：ax..az g，gx..gz dps
SIM_MCU_BIAS = (1.0, -0.5, 2.5, 0.02, -0.03, 0.015)  # **陀螺在前**（imu_bias_t 口径，dps / g）
SIM_MCU_FLAGS = SYSF_TEMPCOMP_EN | 0x0002 | 0x0004 | 0x0008   # 原型 DEF_SYS_FLAGS
SIM_MCU_KP = 2.0                 # Mahony kp（ACK 里以 int8 kp×10 传）
SIM_MCU_KI = 0.05                # Mahony ki（ACK 里以 int8 ki×100 传）
SIM_MCU_DRIFT = (0.05, 0.05, 0.05, 0.0005, 0.0005, 0.0005)    # 温漂 k（gyro dps/°C，accel g/°C）
TEMP_CALIB_MIN_SPAN_C = 3.0      # 温漂标定的最小温差（°C，镜像 temp_calib.h）
SIM_CALIB_FRAMES = 3             # 标定完成后连发 3 帧 0x02（固件契约）

CONNECT_RETRY_S = 0.5      # real 挡连接退避（s）
READ_TIMEOUT_S = 0.5       # 单次 recv 超时（s，POSIX 读语义：到点返回已有字节）
HIST_KEEP = 300            # 历史环窗样本数（页面峰值回看用；sim 100Hz=3s）
MONITOR_PERIOD_S = 5.0     # 控制台统计打印周期（s）
DEFAULT_TCP_PORT = 5558    # --tcp 只给 host 时的端口（Embedded run_qemu.bat 的 -serial tcp 槽）
DEFAULT_BAUD = 115200      # serial 目标缺省波特率（固件 UART0 console 口 115200 8N1）
TARGET_SCHEMES = ("tcp", "serial")   # real 挡目标串前缀全集（解析即校验，无第三种）

# 数据源模式全集：none=空闲（不启动任何数据源）/ sim=仿真 / real=真实
MODES = ("none", "sim", "real")
# --source 旧写法 → 模式（兼容别名，不新增语义）
SOURCE_ALIAS = {"sim": "sim", "tcp": "real"}
# /api/euler 的 source 字段 = 当前**传输面**（none / sim / tcp / serial），
# mode 字段 = 模式词汇（none / sim / real）。real 挡先落 "tcp"，带上串口目标后
# 由 set_target 改成 "serial"——字段如实反映实际链路，不做「real 恒等于 tcp」的谎报。
STATE_SOURCE = {"none": "none", "sim": "sim", "real": "tcp"}

# 控制台文案（--lang 控制打印语言；键在 zh/en 两侧必须成对）
CONSOLE = {
    "zh": {
        "banner": "[euler_view] mode={mode} http={url}（Ctrl+C 退出）",
        "idle": "[euler_view] 空闲：未启动任何数据源——页面点「启动仿真 / 连接真实」再启动",
        "real_target": "[euler_view] 真实模式：目标 {target}，正在连接，未就绪",
        "real_connected": "[euler_view] 真实模式：已连接 {target}",
        "real_lost": "[euler_view] 真实模式：{target} 断连，{retry}s 后重试",
        "serial_missing": ("[euler_view] 串口不可用：需要 pyserial（pip install pyserial）"
                           "——保持未连接，不切仿真"),
        "serial_open": ("[euler_view] 打开串口 {device} 失败：{err}"
                        "——保持未连接，{retry}s 后重试"),
        "ports_missing": "pyserial 缺失（pip install pyserial），无法枚举串口",
        "mode_switch": "[euler_view] 数据源切换：{old} → {new}",
        "halt_timeout": "[euler_view] 旧 {old} 采集线程 {join}s 内未退出（继续切换）",
        "monitor": ("[euler_view] mode={mode} 连接={connected} 帧={frames}（0x03={euler}）"
                    " seq跳变={seq_jumps} 坏帧={bad_resync} 最新={last}"),
        "monitor_empty": "（尚无 0x03 帧）",
        "quit": "[euler_view] 收到 Ctrl+C，退出",
    },
    "en": {
        "banner": "[euler_view] mode={mode} http={url} (Ctrl+C to quit)",
        "idle": "[euler_view] idle: no data source started — pick Start sim / Connect real in the page",
        "real_target": "[euler_view] real mode: target {target}, connecting, not ready yet",
        "real_connected": "[euler_view] real mode: connected to {target}",
        "real_lost": "[euler_view] real mode: {target} dropped, retrying in {retry}s",
        "serial_missing": ("[euler_view] serial unavailable: pyserial required "
                           "(pip install pyserial) -- staying disconnected, no sim fallback"),
        "serial_open": ("[euler_view] cannot open serial port {device}: {err}"
                        " -- staying disconnected, retrying in {retry}s"),
        "ports_missing": "pyserial missing (pip install pyserial), cannot enumerate ports",
        "mode_switch": "[euler_view] data source switch: {old} -> {new}",
        "halt_timeout": "[euler_view] previous {old} thread still alive after {join}s (switching anyway)",
        "monitor": ("[euler_view] mode={mode} connected={connected} frames={frames} (0x03={euler})"
                    " seq_jumps={seq_jumps} bad_resync={bad_resync} last={last}"),
        "monitor_empty": "(no 0x03 frame yet)",
        "quit": "[euler_view] Ctrl+C received, exiting",
    },
}


def console_msg(lang: str, key: str, **vars) -> str:
    """控制台文案取词（缺键直接报 KeyError——漏词比静默英文更可见）。"""
    return CONSOLE[lang][key].format(**vars)


def make_logger(lang: str):
    """把 (key, **vars) 打成一行控制台输出；线程里调用同样安全（单行 print）。"""
    def log(key: str, **vars) -> None:
        print(console_msg(lang, key, **vars), flush=True)
    return log


def crc8(data: bytes) -> int:
    """CRC-8，多项式 0x07，初值 0x00（与固件 proto_crc8 / Host frame.py 同结果）。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameDecoder:
    """流式解码器：喂任意分片字节流，逐帧产出 (type, seq, payload)。

    容错与 Host backend/frame.py 同口径：CRC 错或 LEN 超限 → 丢弃到下一候选 SOF。
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


def parse_ack_data(func: int, data: bytes) -> dict:
    """ACK 数据块 → dict（布局镜像原型 app_imu.c 各分支；长度不足就返 {}
    ——不拿旧布局硬解，宁可少显也不谎报）。

    键名与 `GET_BIAS`/`GET_STATUS`/`GET_TC_STATE`/`GET_DRIFT` 的语义一致，
    `/api/imu/cmd` 回执的 `data` 与 `/api/euler.imu` 同一份字典（单份解析）。
    """
    if func == FUNC_IMU["GET_BIAS"] and len(data) >= BIAS_BYTES:
        gx, gy, gz, ax, ay, az = struct.unpack("<6f", data[:BIAS_BYTES])
        return {"bias": {"gx": gx, "gy": gy, "gz": gz, "ax": ax, "ay": ay, "az": az}}
    if func == FUNC_IMU["GET_STATUS"] and len(data) >= STATUS_BYTES:
        flags = data[6] | (data[7] << 8)
        return {
            "stream_active": data[0], "stream_mask": data[1], "rate_hz": data[2],
            "ctrl1_xl": data[3], "ctrl2_g": data[4], "i2c_400k": data[5],
            "flags": flags, "enabled": 1 if flags & SYSF_TEMPCOMP_EN else 0,
            "bias_valid": data[8], "tc_valid": data[9],
            "kp": struct.unpack("<b", data[10:11])[0] / 10.0,
            "ki": struct.unpack("<b", data[11:12])[0] / 100.0,
            "accel_odr": data[12], "gyro_odr": data[13],
        }
    if func == FUNC_IMU["GET_TC_STATE"] and len(data) >= TC_STATE_BYTES:
        t_now, t_ref, last_applied = struct.unpack("<3f", data[2:14])
        return {
            "tc_valid": data[0], "enabled": data[1],
            "t_now": t_now, "t_ref": t_ref, "last_applied": last_applied,
            "k_g": list(struct.unpack("<3f", data[14:26])),
            "apply_count": data[26] | (data[27] << 8),
        }
    if func == FUNC_IMU["GET_DRIFT"] and len(data) >= DRIFT_BYTES:
        return {"drift": {"gyro": list(struct.unpack("<3f", data[0:12])),
                          "accel": list(struct.unpack("<3f", data[12:24]))}}
    return {}


class SharedState:
    """跨线程共享的页面状态（锁保护；采集线程写、HTTP 线程读快照）。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ack_cond = threading.Condition(self._lock)   # ACK 到达时唤醒等回执的命令线程
        self.mode = "none"
        self.source = STATE_SOURCE["none"]
        self.target = ""             # real 挡当前目标串（tcp:host:port / serial:COMx@baud）
        self.connected = False
        self.frames = 0
        self.euler_frames = 0
        self.seq_jumps = 0
        self.bad_resync = 0
        self.last_deg = None          # (roll, pitch, yaw) 度
        self.last_rad = None
        self.history = deque(maxlen=HIST_KEEP)   # (t_s, roll, pitch, yaw) 度
        self.texts = deque(maxlen=8)
        self._last_seq = None
        # —— 零点校准（MCU 内部标定）：只读展示 + ACK 回执 ——
        self.cal_frames = 0           # 收到的 0x02 IMU_CAL 帧数
        self.cal = None               # 最近一次 0x02：{raw[6], deb[6], bias[6], temp}
        self.ack_frames = 0           # 收到的 0x7E ACK 帧数（也是 wait_ack 的单调基线）
        self.acks = {}                # (mod, func) -> (ack_seq, status, data_bytes)
        self.imu = {                  # MCU 回执渲染面（页面只读展示；值全部来自 ACK/0x02）
            "bias": None,             # {"gx","gy","gz","ax","ay","az"}
            "bias_valid": None,       # GET_STATUS bit：零偏已标定
            "tc_valid": None,         # 温漂已标定（≠ 已启用，META-008）
            "enabled": None,          # SYSF_TEMPCOMP_EN：温补开关
            "flags": None,            # SYS_FLAGS u16
            "stream_active": None, "stream_mask": None, "rate_hz": None,
            "drift": None,            # {"gyro": [3], "accel": [3]}
            "t_now": None, "t_ref": None, "last_applied": None,
            "k_g": None, "apply_count": None,
            "last_ack": None,         # {"func", "status", "name"}
        }

    def on_frame(self, typ: int, seq: int, payload: bytes) -> None:
        with self._lock:
            self.frames += 1
            if self._last_seq is not None and (seq - self._last_seq) & 0xFF != 1:
                self.seq_jumps += 1
            self._last_seq = seq
            if typ == TYPE_TEXT:
                self.texts.append(payload.decode("ascii", "replace"))
            elif typ == TYPE_EULER and len(payload) >= EULER_PAYLOAD_B:
                roll, pitch, yaw = struct.unpack("<3f", payload[:EULER_PAYLOAD_B])
                self.euler_frames += 1
                self.last_rad = (roll, pitch, yaw)
                self.last_deg = tuple(math.degrees(v) for v in (roll, pitch, yaw))
                self.history.append((time.time(),) + self.last_deg)
            elif typ == TYPE_IMU_CAL and len(payload) >= CAL_PAYLOAD_B:
                vals = struct.unpack("<19f", payload[:CAL_PAYLOAD_B])
                self.cal_frames += 1
                self.cal = {"raw": list(vals[0:6]), "deb": list(vals[6:12]),
                            "bias": list(vals[12:18]), "temp": vals[18]}
            elif typ == TYPE_ACK and len(payload) >= ACK_HEAD_B:
                self._on_ack_locked(payload)

    def _on_ack_locked(self, payload: bytes) -> None:
        """ACK 入账：回填渲染面 + 唤醒等回执的线程（等的人自己按 (mod,func) 匹配）。"""
        mod, func, status = payload[0], payload[1], payload[2]
        data = bytes(payload[ACK_HEAD_B:])
        self.ack_frames += 1
        self.acks[(mod, func)] = (self.ack_frames, status, data)
        self.imu["last_ack"] = {"func": FUNC_IMU_NAME.get(func, f"0x{func:02X}"),
                                "status": status,
                                "name": STATUS_NAMES.get(status, f"0x{status:02X}")}
        if status == ACK_OK:
            self._apply_ack_data_locked(func, data)
        self._ack_cond.notify_all()

    def _apply_ack_data_locked(self, func: int, data: bytes) -> None:
        """ACK 数据块 → 渲染面（键名与 /api/imu/cmd 回执 data 同义，只一份解析）。"""
        parsed = parse_ack_data(func, data)
        if parsed:
            self.imu.update(parsed)

    def ack_count(self) -> int:
        """ACK 帧计数（发出命令前取基线，防止把上一次回执当成这次的应答）。"""
        with self._lock:
            return self.ack_frames

    def wait_ack(self, func: int, timeout_s: float, after: int):
        """等 (MOD_IMU, func) 的新 ACK（seq > after）：返 (status, data)；超时 None。"""
        deadline = time.time() + timeout_s
        with self._lock:
            while True:
                rec = self.acks.get((MOD_IMU, func))
                if rec is not None and rec[0] > after:
                    return rec[1], rec[2]
                remain = deadline - time.time()
                if remain <= 0:
                    return None
                self._ack_cond.wait(remain)

    def on_bytes(self, n: int, bad_resync: int) -> None:
        with self._lock:
            self.bad_resync = bad_resync

    def set_connected(self, flag: bool) -> None:
        with self._lock:
            self.connected = flag

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self.mode = mode
            self.source = STATE_SOURCE[mode]

    def set_target(self, target: str) -> None:
        """记录当前目标串（页面回读 /api/euler.target 显示与下次不带目标切换时复用）。

        source 字段同步成真实传输面：`serial:` 目标 → "serial"，`tcp:` → "tcp"
        （旧实现 real 恒报 "tcp"，串口目标时是谎报）。
        """
        with self._lock:
            self.target = target
            if self.mode == "real" and target:
                scheme = target.split(":", 1)[0]
                if scheme in TARGET_SCHEMES:
                    self.source = scheme

    def clear_stream(self) -> None:
        """停掉/切换数据源时清「易变显示态」：新源出帧前表显 —（累计计数保留，如实反映本进程收帧总量）。"""
        with self._lock:
            self.connected = False
            self.last_deg = None
            self.last_rad = None
            self.history.clear()
            self.texts.clear()
            self._last_seq = None
            self.cal = None
            for key in self.imu:
                self.imu[key] = None       # MCU 回执渲染面随数据源一起清（新源自己会再填）

    def snapshot(self) -> dict:
        with self._lock:
            cal = dict(self.cal) if self.cal else None
            imu = dict(self.imu)
            if imu.get("drift"):
                imu["drift"] = {"gyro": list(imu["drift"]["gyro"]),
                                "accel": list(imu["drift"]["accel"])}
            if imu.get("k_g"):
                imu["k_g"] = list(imu["k_g"])
            return {
                "mode": self.mode,
                "source": self.source,
                "target": self.target,
                "connected": self.connected,
                "frames": self.frames,
                "euler_frames": self.euler_frames,
                "seq_jumps": self.seq_jumps,
                "bad_resync": self.bad_resync,
                "last_deg": list(self.last_deg) if self.last_deg else None,
                "last_rad": list(self.last_rad) if self.last_rad else None,
                "history": [list(h) for h in self.history],
                "texts": list(self.texts),
                "cal": cal,              # 最近一帧 0x02（raw|deb|bias|temp，只读展示）
                "cal_frames": self.cal_frames,
                "ack_frames": self.ack_frames,
                "imu": imu,              # MCU 回执渲染面（GET_BIAS/STATUS/TC_STATE/DRIFT）
            }


class VirtualMcu:
    """sim 挡虚拟 MCU（MOD_IMU）：与固件**同一帧语义**应答，不发明任何字符串命令。

    状态 = 原型 g_ctx 的 IMU 子集（bias / bias_valid / tc_valid / sys_flags / drift /
    采样温度），行为逐条镜像 `app_imu.c` + `app_stream.c`：
      - GET_BIAS / GET_STATUS / GET_TC_STATE / GET_DRIFT：立即回 ACK（带数据块）；
      - SET_FLAGS：改开关位（只改「是否应用」，不伪造 tc_valid）；
      - CALIBRATE：**异步**，采集 collect_s 秒后才回 ACK，并送 3 帧 0x02；
        零偏 = 窗内均值 − 重力轴 sign·1g（bias = 测量 − 期望，故 deb az ≈ +1 g）；
      - TEMP_CALIB：异步，采集 seconds 秒后拟合温漂；温差 < TEMP_CALIB_MIN_SPAN_C
        → 拟合被拒（tc_valid=0），否则 tc_valid=1 + 写入预设 k。
    采集与拟合全在虚拟 MCU 内部，主机只发命令、读回执——与真实固件一致。
    """

    def __init__(self, collect_s: float, temp_start_c: float, temp_ramp_c_per_s: float) -> None:
        self._collect_s = collect_s       # 500 样本 @100Hz = 5s（镜像 cal.exp2.bias 窗口）
        self._temp_start_c = temp_start_c
        self._temp_ramp = temp_ramp_c_per_s
        self.bias = [0.0] * 6          # 开机未标定：零偏全零（等价 pstore 为空）+ bias_valid=0
        self.bias_valid = 0
        self.sys_flags = SIM_MCU_FLAGS
        self.drift = [0.0] * 6            # 未做温漂标定 → k = 0
        self.tc_valid = 0
        self.t_ref = 0.0
        self.last_applied = 0.0
        self.apply_count = 0
        self._pending = []                # [(due_s, job)] 异步回执队列（时间轴 = 采集线程拍点）

    def submit(self, func: int, params: bytes, now_s: float):
        """命令到达：返立刻要发的 [(typ, payload)]；异步命令只入队（到点由 poll 吐）。"""
        if func == FUNC_IMU["CALIBRATE"]:
            self._pending.append((now_s + self._collect_s, ("calib",)))
            return []
        if func == FUNC_IMU["TEMP_CALIB"]:
            seconds = float(params[0]) if params else 30.0
            self._pending.append((now_s + seconds, ("tempcalib", seconds)))
            return []
        if func == FUNC_IMU["GET_BIAS"]:
            return [(TYPE_ACK, self._ack(func, struct.pack("<6f", *self.bias)))]
        if func == FUNC_IMU["GET_STATUS"]:
            data = bytes((0, 0x07, 20, 0, 0, 0))                  # 流/掩码/Hz/ctrl1/ctrl2/i2c
            data += struct.pack("<H", self.sys_flags)
            data += bytes((self.bias_valid, self.tc_valid,
                           int(SIM_MCU_KP * 10) & 0xFF, int(SIM_MCU_KI * 100) & 0xFF))
            return [(TYPE_ACK, self._ack(func, data + bytes(STATUS_BYTES - len(data))))]
        if func == FUNC_IMU["GET_TC_STATE"]:
            data = bytes((self.tc_valid, 1 if self.sys_flags & SYSF_TEMPCOMP_EN else 0))
            data += struct.pack("<3f", self._temp_at(now_s), self.t_ref, self.last_applied)
            data += struct.pack("<3f", *self.drift[0:3])
            data += struct.pack("<H", self.apply_count)
            return [(TYPE_ACK, self._ack(func, data))]
        if func == FUNC_IMU["GET_DRIFT"]:
            return [(TYPE_ACK, self._ack(func, struct.pack("<6f", *self.drift)))]
        if func == FUNC_IMU["SET_FLAGS"]:
            if len(params) < 2:
                return [(TYPE_ACK, self._ack(func, b"", status=0x01))]
            self.sys_flags = params[0] | (params[1] << 8)
            return [(TYPE_ACK, self._ack(func, b""))]
        return [(TYPE_ACK, self._ack(func, b"", status=0xFF))]    # 未知功能码

    def poll(self, now_s: float):
        """取到期帧：异步命令的 ACK（CALIBRATE 另加 3 帧 0x02）。"""
        out = []
        while self._pending and self._pending[0][0] <= now_s:
            job = self._pending.pop(0)[1]
            if job[0] == "calib":
                self._finish_calibrate()
                out.append((TYPE_ACK, self._ack(FUNC_IMU["CALIBRATE"], b"")))
                out.extend((TYPE_IMU_CAL, self._cal_payload()) for _ in range(SIM_CALIB_FRAMES))
            else:
                self._finish_temp_calib(job[1], now_s)
                out.append((TYPE_ACK, self._ack(FUNC_IMU["TEMP_CALIB"], b"")))
        return out

    # —— 内部：标定与回执 ——

    def _temp_at(self, now_s: float) -> float:
        return self._temp_start_c + self._temp_ramp * now_s

    def _finish_calibrate(self) -> None:
        """零偏 = 窗内均值 − 重力轴 sign·1g（镜像 imu_calib.c：bias = 测量 − 期望）。

        虚拟 MCU 的 raw = 静止真值 + 注入零偏，故算出的 bias 恰好 = SIM_MCU_BIAS
        （重力已扣），而 deb = raw − bias 回到真值（az ≈ +1 g）——与真实固件同口径。
        温补开关不动：同固件（无温漂系数就不开 tc_valid）。
        """
        self.bias = list(SIM_MCU_BIAS)
        self.bias_valid = 1
        drift_ok = any(v != 0.0 for v in self.drift[0:3])
        self.tc_valid = 1 if drift_ok else 0
        self.t_ref = self._temp_at(0.0)

    def _finish_temp_calib(self, seconds: float, now_s: float) -> None:
        """温差够 → 拟合有效（预设 k + tc_valid=1）；不够 → 拒绝（k 清零 + tc_valid=0）。

        温差 = 采集窗内的温升 = ramp × 秒数（窗长，不是绝对时长）。
        """
        span = self._temp_ramp * seconds
        if span >= TEMP_CALIB_MIN_SPAN_C:
            self.drift = list(SIM_MCU_DRIFT)
            self.tc_valid = 1
            self.t_ref = self._temp_at(now_s)
        else:
            self.drift = [0.0] * 6
            self.tc_valid = 0

    def _ack(self, func: int, data: bytes, status: int = ACK_OK) -> bytes:
        return bytes((MOD_IMU, func, status)) + data

    def _cal_payload(self) -> bytes:
        """一帧 0x02 的 19 值：raw6 | deb6 | bias6 | temp（块序 ax,ay,az,gx,gy,gz）。"""
        bias_frame = (self.bias[3], self.bias[4], self.bias[5],
                      self.bias[0], self.bias[1], self.bias[2])
        raw = tuple(t + b for t, b in zip(SIM_MCU_TRUTH, bias_frame))
        deb = tuple(r - b for r, b in zip(raw, bias_frame))
        return struct.pack("<19f", *(raw + deb + bias_frame + (self.t_ref,)))


def encode_frame(seq: int, typ: int, payload: bytes) -> bytes:
    """任意类型整帧（sim 出字节流 / 命令帧均走此口，与固件 proto_frame_encode 同规格）。"""
    head = bytes((SOF, len(payload), typ, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


def encode_cmd_frame(seq: int, mod: int, func: int, params: bytes = b"") -> bytes:
    """命令帧 0x7D：payload = MOD_ID | FUNC_ID | params…（real 挡下行用）。"""
    return encode_frame(seq, TYPE_CMD, bytes((mod, func)) + params)


def sim_source_loop(state: SharedState, amp_deg: float, period_s: float,
                    rate_hz: float, mcu: VirtualMcu, cmd_q: queue.Queue,
                    stop: threading.Event) -> None:
    """sim 挡：合成欧拉流 + 虚拟 MCU 应答，全部先编码成协议帧再走同一个解码器。

    cmd_q：HTTP 线程投进来的 (func, params) 命令——虚拟 MCU 在**采集线程的拍点**上
    消费，故命令→回执的时间轴与真实固件一致（异步命令在采集结束时才回 ACK）。
    """
    state.set_connected(True)
    seq = 0
    dec = FrameDecoder()
    t0 = time.monotonic()          # 时间基用单调时钟：异步回执的「秒」必须与墙钟一致
    period_sleep = 1.0 / rate_hz
    while not stop.is_set():
        t = time.monotonic() - t0
        pitch = amp_deg * math.sin(2.0 * math.pi * t / period_s)
        frames = [(TYPE_EULER, struct.pack("<3f", 0.0, math.radians(pitch), 0.0))]
        while True:
            try:
                func, params = cmd_q.get_nowait()
            except queue.Empty:
                break
            frames.extend(mcu.submit(func, params, t))
        frames.extend(mcu.poll(t))
        for typ, payload in frames:
            for dtyp, dseq, dpayload in dec.feed(encode_frame(seq, typ, payload)):
                state.on_frame(dtyp, dseq, dpayload)
            seq = (seq + 1) & 0xFF
        stop.wait(period_sleep)
    state.on_bytes(0, dec.bad_resync)


class OutLink:
    """real 挡当前链路的**写口**（调零下行 `CAL\\n` 用）。

    采集线程连上时挂载 write 回调、断开时摘除；HTTP 线程经 send() 写。
    未挂载 = 链路未就绪 → 调零回 notConnected（不排队、不缓存、不等重连）。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._write = None

    def attach(self, write) -> None:
        with self._lock:
            self._write = write

    def detach(self) -> None:
        with self._lock:
            self._write = None

    def send(self, data: bytes) -> bool:
        with self._lock:
            write = self._write
        if write is None:
            return False
        try:
            write(data)
        except OSError:                  # 链路已断（socket/serial 均抛 OSError 系）
            return False
        return True


def tcp_source_loop(state: SharedState, host: str, port: int, stop: threading.Event,
                    log, link: OutLink) -> None:
    """real 挡 tcp 目标：只连给定 host:port 收真实 0x03/0x02/TEXT 帧流（EOF/断连重连同一地址）。

    不换目标、不回落仿真——连不上就一直「未就绪」，控制台只在状态变化时说话。
    连上就先挂 write 口再置 connected：页面看到 connected=True 时 CAL 一定能发出去。
    """
    addr = f"tcp:{host}:{port}"
    while not stop.is_set():
        try:
            sock = socket.create_connection((host, port), timeout=CONNECT_RETRY_S)
        except OSError:
            state.set_connected(False)
            stop.wait(CONNECT_RETRY_S)
            continue
        link.attach(sock.sendall)
        state.set_connected(True)
        log("real_connected", target=addr)
        sock.settimeout(READ_TIMEOUT_S)
        dec = FrameDecoder()
        try:
            while not stop.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    state.on_bytes(0, dec.bad_resync)
                    continue
                if not chunk:            # EOF：对端关闭 → 回连接阶段重试
                    break
                for typ, seq, payload in dec.feed(chunk):
                    state.on_frame(typ, seq, payload)
                state.on_bytes(len(chunk), dec.bad_resync)
        except OSError:
            pass                         # 流断：回外层重连
        finally:
            link.detach()
            try:
                sock.close()
            except OSError:
                pass
        state.set_connected(False)
        log("real_lost", target=addr, retry=CONNECT_RETRY_S)
        stop.wait(CONNECT_RETRY_S)


def serial_source_loop(state: SharedState, device: str, baud: int, stop: threading.Event,
                       log, link: OutLink) -> None:
    """real 挡 serial 目标：开物理串口收真实帧流（与 tcp 挡同一条 FrameDecoder 路径）。

    pyserial **懒导入**（tcp/sim 挡不拉这个依赖）：缺包 → 保持未连接 + 控制台一行，
    线程正常结束（不崩、不回落仿真）；打不开/拔线 → 每 0.5s 退避重连同一设备，
    同一错误只喊一次（不刷屏）。串口 opened 后才挂 write 口（调零下行 CAL 用）。
    """
    try:
        import serial                    # pyserial（BSD-3，已在 Host 依赖声明内）
    except ImportError:
        state.set_connected(False)
        log("serial_missing", device=device)
        return
    addr = f"serial:{device}@{baud}"
    last_err = None
    while not stop.is_set():
        try:
            ser = serial.Serial(device, baud, timeout=READ_TIMEOUT_S)
        except (serial.SerialException, OSError, ValueError) as exc:
            state.set_connected(False)
            if str(exc) != last_err:
                log("serial_open", device=device, err=exc, retry=CONNECT_RETRY_S)
                last_err = str(exc)
            stop.wait(CONNECT_RETRY_S)
            continue
        last_err = None
        link.attach(ser.write)
        state.set_connected(True)
        log("real_connected", target=addr)
        dec = FrameDecoder()
        try:
            while not stop.is_set():
                chunk = ser.read(4096)   # timeout=READ_TIMEOUT_S：到点返回已有字节（POSIX 语义）
                if not chunk:
                    state.on_bytes(0, dec.bad_resync)
                    continue
                for typ, seq, payload in dec.feed(chunk):
                    state.on_frame(typ, seq, payload)
                state.on_bytes(len(chunk), dec.bad_resync)
        except (serial.SerialException, OSError):
            pass                         # 流断（拔线/关串口）：回外层重连
        finally:
            link.detach()
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        state.set_connected(False)
        if not stop.is_set():
            log("real_lost", target=addr, retry=CONNECT_RETRY_S)
            stop.wait(CONNECT_RETRY_S)


class SourceManager:
    """显式数据源开关（none / sim / real）：同一时刻最多一个采集线程。

    切换顺序固定「先停旧线程（Event + join<1s）再起新线程」——sim↔real 之间不存在
    两个线程同时写 state 的窗口，也不存在隐式回落（没有设备就一直是未连接，绝不自动仿真）。
    """

    JOIN_TIMEOUT_S = 1.0

    def __init__(self, state: SharedState, sim_cfg, target, default_baud: int, log) -> None:
        """sim_cfg = (amp_deg, period_s, rate_hz, calib_s, temp_start_c, temp_ramp_c_per_s)。"""
        self._state = state
        (self._sim_amp_deg, self._sim_period_s, self._sim_rate_hz,
         self._sim_calib_s, self._sim_temp_c, self._sim_temp_ramp) = sim_cfg
        self._target = target
        self._default_baud = default_baud
        self._log = log
        self._lock = threading.Lock()
        self._stop = None
        self._thread = None
        self._mode = "none"
        self._link = OutLink()           # real 挡链路写口（命令下行 CMD 帧用，采集线程挂/摘）
        self._cmd_q = None               # sim 挡命令队列（虚拟 MCU 在采集线程拍点上消费）
        self._cmd_seq = 0                # CMD 帧序号（只影响 seq 字段，回绕即回绕）
        state.set_target(target.text)     # 页面连上就能看到服务端当前目标（即使还没切到 real）

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def target(self):
        return self._target

    def set_mode(self, mode: str, target: str = "") -> dict:
        """切到 mode（可带目标串）：先停旧线程再起新线程；返回切换后的 /api/euler 快照。

        目标只对 real 挡生效：带 target 则解析并替换当前目标，不带则沿用上一次的目标
        （缺省来自 --serial / --tcp）；目标串非法 → ValueError（HTTP 层回 400，档位不变）。
        """
        if mode not in MODES:
            raise ValueError(f"非法模式 {mode!r}（只认 {'|'.join(MODES)}）")
        new_target = (parse_target(target, self._default_baud)
                      if mode == "real" and target else None)
        with self._lock:
            old = self._mode
            self._halt_locked(old)
            self._mode = mode
            if new_target is not None:
                self._target = new_target
            self._state.set_mode(mode)
            self._state.set_target(self._target.text)
            self._state.clear_stream()    # 切档先清易变显示态：新源出帧前表显—（不留上一档的残值）
            if mode != "none":
                self._stop = threading.Event()
                if mode == "sim":
                    loop = sim_source_loop
                    self._cmd_q = queue.Queue()
                    mcu = VirtualMcu(self._sim_calib_s, self._sim_temp_c, self._sim_temp_ramp)
                    loop_args = (self._state, self._sim_amp_deg, self._sim_period_s,
                                 self._sim_rate_hz, mcu, self._cmd_q, self._stop)
                else:
                    self._cmd_q = None
                    tgt = self._target
                    self._log("real_target", target=tgt.text)
                    if tgt.scheme == "tcp":
                        loop = tcp_source_loop
                        loop_args = (self._state, tgt.host, tgt.port, self._stop, self._log,
                                     self._link)
                    else:
                        loop = serial_source_loop
                        loop_args = (self._state, tgt.device, tgt.baud, self._stop, self._log,
                                     self._link)
                self._thread = threading.Thread(target=loop, args=loop_args, daemon=True)
                self._thread.start()
            if old != mode:
                self._log("mode_switch", old=old, new=mode)
            return self._state.snapshot()

    def stop(self) -> None:
        self.set_mode("none")

    def cmd(self, func_name: str, args=None) -> dict:
        """MOD_IMU 命令（POST /api/imu/cmd）：real 下行 CMD 帧 + 等 ACK；sim 交虚拟 MCU 应答。

        一律 HTTP 200 + code（页面按 code 翻译）：
          ok            — ACK status=0，data = 解析后的数据块（GET_* 才非空）
          notConnected  — 未启动数据源 / real 链路未就绪（写不进去）
          timeout       — 命令已发出但 ACK 超时（CALIBRATE 采集还没完 = 20s 内无回执）
          ackError      — MCU 回 status≠0（参数错/执行失败/未知功能码）
          badFunc/badArgs — 功能码或参数不认（参数错在主机侧就拦下，不必惊动 MCU）

        **主机不做任何去偏/拟合**：零偏与温漂都在 MCU 里算，主机只要回执。
        """
        args = args or {}
        func_name = str(func_name or "").strip().upper()   # 大小写不敏感（与 /api/imu/cmd 统一）
        func = FUNC_IMU.get(func_name)
        if func is None:
            return {"code": "badFunc", "func": func_name,
                    "known": sorted(FUNC_IMU)}
        try:
            params = self._cmd_params(func_name, args)
        except (KeyError, TypeError, ValueError):
            return {"code": "badArgs", "func": func_name}
        timeout_s = self._cmd_timeout(func_name, params)
        with self._lock:
            mode = self._mode
            cmd_q = self._cmd_q
        if mode == "none":
            return {"code": "notConnected", "func": func_name}
        after = self._state.ack_count()       # 基线：只认发命令之后到的 ACK
        if mode == "sim":
            if cmd_q is None:
                return {"code": "notConnected", "func": func_name}
            cmd_q.put((func, params))         # 虚拟 MCU 在采集线程拍点上消费
        else:
            frame = encode_cmd_frame(self._cmd_seq, MOD_IMU, func, params)
            self._cmd_seq = (self._cmd_seq + 1) & 0xFF
            if not self._link.send(frame):
                return {"code": "notConnected", "func": func_name}
        res = self._state.wait_ack(func, timeout_s, after)
        if res is None:
            return {"code": "timeout", "func": func_name, "waitS": round(timeout_s, 1)}
        status, data = res
        if status != ACK_OK:
            return {"code": "ackError", "func": func_name, "status": status,
                    "statusName": STATUS_NAMES.get(status, f"0x{status:02X}")}
        return {"code": "ok", "func": func_name, "data": parse_ack_data(func, data),
                "source": mode}

    @staticmethod
    def _cmd_params(func_name: str, args: dict) -> bytes:
        """参数区编码（镜像原型 app_imu.c：TEMP_CALIB 1B 秒；SET_FLAGS 2B flags 小端）。"""
        if func_name == "TEMP_CALIB":
            seconds = int(args.get("seconds", 30))
            if not 1 <= seconds <= 255:
                raise ValueError(f"秒数越界：{seconds}（1..255）")
            return bytes((seconds,))
        if func_name == "SET_FLAGS":
            flags = int(args["flags"])
            if not 0 <= flags <= 0xFFFF:
                raise ValueError(f"flags 越界：{flags}")
            return struct.pack("<H", flags)
        return b""

    @staticmethod
    def _cmd_timeout(func_name: str, params: bytes) -> float:
        """等 ACK 上限：异步命令按采集时长放宽（同步 GET/SET 走 CMD_TIMEOUT_S）。"""
        if func_name == "CALIBRATE":
            return CALIBRATE_TIMEOUT_S
        if func_name == "TEMP_CALIB":
            return (params[0] if params else 30) + TEMP_CALIB_MARGIN_S
        return CMD_TIMEOUT_S

    def _halt_locked(self, old: str) -> None:
        if self._stop is not None:
            self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(self.JOIN_TIMEOUT_S)
            if thread.is_alive():
                self._log("halt_timeout", old=old, join=self.JOIN_TIMEOUT_S)
        self._stop = None
        self._thread = None
        self._cmd_q = None


class PageHandler(BaseHTTPRequestHandler):
    """静态页 + JSON API：GET / · GET /api/euler · GET /api/ports · GET|POST /api/mode ·
    GET|POST /api/imu/cmd，其余 404。"""

    state: SharedState = None            # main() 注入类属性
    manager: SourceManager = None
    index_path: Path = None
    lang: str = "zh"

    def do_GET(self) -> None:            # noqa: N802（http.server 约定名）
        path, _, query = self.path.partition("?")
        if path == "/":
            self._send_index()
        elif path == "/api/euler":
            self._send_json(self.state.snapshot())
        elif path == "/api/ports":
            self._send_json(detect_ports(self.lang))
        elif path == "/api/mode":
            self._switch_mode(*self._mode_args(query, b""))
        elif path == "/api/imu/cmd":   # GET 也认（?func=GET_BIAS）：curl 手测不必组 JSON
            self._send_json(self.manager.cmd(*self._cmd_args(b"", query)))
        else:
            self._send_not_found()

    def do_POST(self) -> None:           # noqa: N802（http.server 约定名）
        path, _, query = self.path.partition("?")
        if path not in ("/api/mode", "/api/imu/cmd"):
            self._send_not_found()
            return
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else b""   # 请求体读了才能保持连接可复用
        if path == "/api/imu/cmd":
            self._send_json(self.manager.cmd(*self._cmd_args(body, query)))
            return
        self._switch_mode(*self._mode_args(query, body))

    @staticmethod
    def _cmd_args(body: bytes, query: str = ""):
        """取 (func_name, args)：query（func=…&seconds=…）优先，其次 JSON body。

        兼容 `{"func":"TEMP_CALIB","args":{"seconds":30}}` 与扁平
        `{"func":"SET_FLAGS","flags":1}` 两种写法（后者省一层花括号）。
        """
        q = parse_qs(query)
        func = q.get("func", [""])[0].strip()
        args = {k: v[0] for k, v in q.items() if k not in ("func", "mode")}
        text = body.decode("utf-8", "replace").strip()
        if text:
            try:
                data = json.loads(text)
            except ValueError:
                if not func:
                    func = text          # 裸文本 body = 功能码
                data = None
            if isinstance(data, dict):
                func = str(data.get("func") or func).strip()
                if isinstance(data.get("args"), dict):
                    args.update(data["args"])
                args.update({k: v for k, v in data.items() if k not in ("func", "args")})
            elif isinstance(data, str) and not func:
                func = data.strip()
        return func, args

    @staticmethod
    def _mode_args(query: str, body: bytes):
        """取 (value, target)：优先级 query → JSON body → 裸文本 body。"""
        q = parse_qs(query)
        target = q.get("target", [""])[0].strip()
        for key in ("value", "mode"):
            if key in q:
                return q[key][0].strip(), target
        text = body.decode("utf-8", "replace").strip()
        if not text:
            return "", target
        try:
            data = json.loads(text)
        except ValueError:
            return text, target
        if isinstance(data, dict):
            target = str(data.get("target") or target).strip()
            for key in ("value", "mode"):
                if key in data:
                    return str(data[key]).strip(), target
            return "", target
        return str(data).strip(), target

    def _switch_mode(self, value: str, target: str) -> None:
        if value not in MODES:
            self._send_json({"error": f"value 只认 {'|'.join(MODES)}",
                             "mode": self.manager.mode}, status=400)
            return
        try:
            snap = self.manager.set_mode(value, target)
        except ValueError as e:          # 目标串非法：档位不变，页面拿 error 提示
            self._send_json({"error": str(e), "mode": self.manager.mode}, status=400)
            return
        self._send_json({"mode": snap["mode"], "connected": snap["connected"]})

    def _send_index(self) -> None:
        body = self.index_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")              # 10Hz 轮询快照绝不缓存
        self.send_header("Access-Control-Allow-Origin", "null")   # file:// 打开亦可用
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_not_found(self) -> None:
        self.send_response(404)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def log_message(self, fmt: str, *args) -> None:
        pass                             # 静默访问日志（控制台只留统计打印）


def resolve_mode(mode, source) -> str:
    """--mode 优先，--source 作兼容别名（sim→sim，tcp→real）；两者冲突则报错。

    都不给 = "none"（空闲）——缺省绝不启动任何数据源（禁止隐式回落）。
    """
    alias = SOURCE_ALIAS.get(source) if source else None
    if mode and alias and mode != alias:
        raise ValueError(f"--mode {mode} 与 --source {source} 冲突：--source 是旧别名，只许给一个")
    return mode or alias or "none"


def split_tcp_addr(text: str):
    """'host:port' → (host, port)；只给 host 时端口取 DEFAULT_TCP_PORT。"""
    host, _, port_s = text.partition(":")
    host = host or "127.0.0.1"
    if not port_s:
        return host, DEFAULT_TCP_PORT
    port = int(port_s)
    if not 0 < port < 65536:
        raise ValueError(f"端口越界：{port}")
    return host, port


class Target:
    """real 挡目标（统一目标串）：`tcp:<host>:<port>` 或 `serial:<COMx>@<baud>`。

    两种目标共用同一条解码路径（tcp_source_loop / serial_source_loop 都过 FrameDecoder），
    差异只在字节从哪来；`text` 是控制台 / `/api/euler` / 页面输入框看到的同一份写法。
    """

    __slots__ = ("scheme", "host", "port", "device", "baud")

    def __init__(self, scheme: str, host=None, port=None, device=None, baud=None) -> None:
        self.scheme = scheme
        self.host, self.port = host, port
        self.device, self.baud = device, baud

    @property
    def text(self) -> str:
        if self.scheme == "tcp":
            return f"tcp:{self.host}:{self.port}"
        return f"serial:{self.device}@{self.baud}"

    def __str__(self) -> str:
        return self.text

    def __repr__(self) -> str:
        return f"Target({self.text})"

    def __eq__(self, other) -> bool:
        return isinstance(other, Target) and other.text == self.text

    def __hash__(self) -> int:
        return hash(self.text)


def parse_target(text: str, default_baud: int = DEFAULT_BAUD) -> Target:
    """目标串 → Target（解析即校验；只认两种前缀，其余 ValueError）。

    `serial:COM5` 省 `@baud` → 用 default_baud（= --baud，缺省 115200）。
    """
    s = (text or "").strip()
    if not s:
        raise ValueError("目标为空：只认 tcp:<host>:<port> 或 serial:<COMx>@<baud>")
    scheme, _, rest = s.partition(":")
    scheme = scheme.strip().lower()
    if scheme == "tcp":
        host, port = split_tcp_addr(rest)
        return Target("tcp", host=host, port=port)
    if scheme == "serial":
        device, _, baud_s = rest.partition("@")
        device = device.strip()
        if not device:
            raise ValueError(f"串口目标缺设备名：{s!r}（例 serial:COM5@115200）")
        try:
            baud = default_baud if not baud_s.strip() else int(baud_s)
        except ValueError:
            raise ValueError(f"波特率不是整数：{baud_s.strip()!r}（例 serial:COM5@115200）") from None
        if not 0 < baud < 10000000:
            raise ValueError(f"波特率越界：{baud}")
        return Target("serial", device=device, baud=baud)
    raise ValueError(f"目标串 {s!r} 不认：只认 {' / '.join(TARGET_SCHEMES)}（tcp:<host>:<port>"
                     " 或 serial:<COMx>@<baud>）")


def detect_ports(lang: str) -> dict:
    """枚举本机串口（GET /api/ports）：返回 {"ports": [{"device", "desc"}]}。

    pyserial 懒导入：缺失 → 空数组 + error 一行（页面据此提示，不崩）。
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return {"ports": [], "error": console_msg(lang, "ports_missing")}
    return {"ports": [{"device": p.device, "desc": p.description or ""}
                      for p in list_ports.comports()]}


def main() -> int:
    ap = argparse.ArgumentParser(
        description="exp3 欧拉角 host 页（0x03 帧解码 → 数值表 + 三轴条形 canvas，零第三方）")
    ap.add_argument("--mode", choices=("sim", "real"), default=None,
                    help="数据源模式：sim=本机合成；real=连 --serial / --tcp 指定的目标；"
                         "不带 = 空闲（等页面点选）")
    ap.add_argument("--source", choices=("sim", "tcp"), default=None,
                    help="兼容别名（旧写法）：sim→--mode sim，tcp→--mode real")
    ap.add_argument("--serial", default=None, metavar="COMx",
                    help="real 挡串口号（如 COM5）：目标串 serial:<COMx>@<baud>，优先于 --tcp")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help=f"real 挡串口波特率（缺省 {DEFAULT_BAUD}，8N1）")
    ap.add_argument("--tcp", default=f"127.0.0.1:{DEFAULT_TCP_PORT}",
                    help="real 挡 TCP 目标 host:port（对应 run_qemu.bat 的 -serial tcp 槽）")
    ap.add_argument("--http-port", type=int, default=8765,
                    help="页面 HTTP 端口（浏览器开 http://127.0.0.1:<port>）")
    ap.add_argument("--lang", choices=("zh", "en"), default="zh",
                    help="控制台打印语言（页面语言另有内联词典，缺省 zh）")
    # sim 挡参数默认值 = 固件合成源预设 sim.exp3.sway 的镜像（testing/imu_sim）
    ap.add_argument("--sim-amp-deg", type=float, default=20.0,
                    help="sim 摆幅（deg，默认 20 = sim.exp3.sway sway_amp_deg）")
    ap.add_argument("--sim-period-s", type=float, default=4.0,
                    help="sim 摆动周期（s，默认 4 = sim.exp3.sway sway_period_s）")
    ap.add_argument("--sim-rate-hz", type=float, default=100.0,
                    help="sim 采样率（Hz，默认 100 = sim.exp3.sway sample_rate_hz）")
    ap.add_argument("--sim-calib-s", type=float, default=5.0,
                    help="虚拟 MCU 调零采集窗（s，默认 5 = 500 样本@100Hz，镜像 cal.exp2.bias）")
    ap.add_argument("--sim-temp-start-c", type=float, default=25.0,
                    help="虚拟 MCU 起始温度（°C，默认 25 = sim.exp2.static temp_start_c）")
    ap.add_argument("--sim-temp-ramp-c", type=float, default=0.1,
                    help="虚拟 MCU 温度斜坡（°C/s，默认 0.1 = sim.exp2.static；温漂标定需温差 ≥3°C，"
                         "无真机加热时把这调大让演示等得起）")
    args = ap.parse_args()

    try:
        mode = resolve_mode(args.mode, args.source)
        # 目标串只有一个构造口（parse_target）：--serial 优先于 --tcp；二者都不给 = --tcp 缺省
        target = parse_target(f"serial:{args.serial}@{args.baud}" if args.serial
                              else f"tcp:{args.tcp}")
    except ValueError as e:
        ap.error(str(e))

    log = make_logger(args.lang)
    state = SharedState()
    state.set_mode("none")
    manager = SourceManager(
        state,
        (args.sim_amp_deg, args.sim_period_s, args.sim_rate_hz,
         args.sim_calib_s, args.sim_temp_start_c, args.sim_temp_ramp_c),
        target,
        args.baud,
        log)

    here = Path(__file__).resolve().parent
    PageHandler.state = state
    PageHandler.manager = manager
    PageHandler.index_path = here / "index.html"
    PageHandler.lang = args.lang
    srv = ThreadingHTTPServer(("127.0.0.1", args.http_port), PageHandler)
    srv.timeout = 1.0                   # 空闲时 handle_request 1s 一跳（监控打印/可 Ctrl+C）

    url = f"http://127.0.0.1:{args.http_port}/"
    log("banner", mode=mode, url=url)
    if mode == "none":
        log("idle")
    manager.set_mode(mode)

    try:
        next_mon = time.time() + MONITOR_PERIOD_S
        while True:
            srv.handle_request()          # 单请求步进，穿插周期统计打印
            now = time.time()
            if now >= next_mon:
                snap = state.snapshot()
                last = snap["last_deg"]
                last_txt = ("[" + ", ".join(f"{v:+8.3f}" for v in last) + "] deg"
                            ) if last else console_msg(args.lang, "monitor_empty")
                log("monitor", mode=snap["mode"], connected=snap["connected"],
                    frames=snap["frames"], euler=snap["euler_frames"],
                    seq_jumps=snap["seq_jumps"], bad_resync=snap["bad_resync"],
                    last=last_txt)
                next_mon = now + MONITOR_PERIOD_S
    except KeyboardInterrupt:
        log("quit")
    finally:
        manager.stop()                    # Event + join<1s 收口采集线程
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
