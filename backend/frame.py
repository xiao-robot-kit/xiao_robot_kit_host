"""proto frame 解码（Python 侧）——与固件 proto/src/frame.c 同规格。

规格单源：帧格式与 CRC8 参数以 CoreLib 契约文档为准（INT-2），
两侧实现必须通过同一组测试向量（见 tests/test_frame.py）。
"""

SOF = 0xA5
TYPE_IMU_SAMPLE = 0x01
TYPE_IMU_CAL = 0x02   # 契约镜像：proto/frame.h PROTO_TYPE_IMU_CAL（exp2 起发送）
                      # payload = 19×float32 LE = 76B，四块（零偏块用采样序
                      # ax,ay,az,gx,gy,gz）：原始六轴 | 补偿后六轴 | 当前零偏 |
                      # 芯片温度（°C）——块序与单位见 frame.h TYPE_IMU_CAL 注释
TYPE_EULER = 0x03   # 契约镜像：proto/frame.h PROTO_TYPE_EULER（exp3 起发送）
                    # payload = roll,pitch,yaw 三个 float32 LE = 12B，单位弧度，
                    # ZYX 约定（与 mahony_get_angles 输出同源）；
                    # 教学通道——pitch ±90° 万向锁（3D 主通道用 0x04）
TYPE_QUAT = 0x04   # 契约镜像：proto/frame.h PROTO_TYPE_QUAT（exp4 起发送）
                   # payload = qw,qx,qy,qz 四个 float32 LE = 16B，w 在前
TYPE_TEXT = 0x7F
TYPE_ACK = 0x7E
PAYLOAD_MAX = 96   # 契约单源 proto/frame.h PROTO_PAYLOAD_MAX（exp2 起 0x02 帧 76B）；
                   # 旧值 64 为 exp1 期遗留——T-112 exp_proto 注入回归实测 76B 帧
                   # 被整帧静默丢弃后对齐修正（scope_motor.py/probe_sim.py 同值）
PAYLOAD_EULER_B = 12   # EULER payload 字节数 = 3×float32 LE
PAYLOAD_QUAT_B = 16   # QUAT payload 字节数 = 4×float32 LE
PAYLOAD_CAL_B = 76   # IMU_CAL payload 字节数 = 19×float32 LE（exp2 调零温补帧）


def crc8(data: bytes) -> int:
    """CRC-8，多项式 0x07，初值 0x00（与 C 侧 proto_crc8 同结果）。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameDecoder:
    """流式解码器：喂任意分片的字节流，逐帧产出 (type, seq, payload)。

    帧格式：SOF | LEN | TYPE | SEQ | PAYLOAD(LEN) | CRC8(覆盖 LEN..PAYLOAD)
    容错：CRC 错或 LEN 超限即重新找 SOF（丢弃至下一候选头）。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes):
        """喂入一段字节流，产出完整帧列表 [(type, seq, payload), ...]。"""
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
            # 1) 找 SOF
            i = buf.find(bytes([SOF]))
            if i < 0:
                buf.clear()
                return None
            if i:
                del buf[:i]
            # 2) 头够吗（SOF LEN TYPE SEQ）
            if len(buf) < 4:
                return None
            length = buf[1]
            if length > PAYLOAD_MAX:      # 假 SOF，跳过这一字节再找
                del buf[:1]
                continue
            # 3) 整帧够吗
            total = 4 + length + 1
            if len(buf) < total:
                return None
            # 4) CRC
            if crc8(bytes(buf[1:4 + length])) != buf[4 + length]:
                del buf[:1]               # 坏帧：从下一字节重同步
                continue
            frame = (buf[2], buf[3], bytes(buf[4:4 + length]))
            del buf[:total]
            return frame


def decode_sample(payload: bytes):
    """IMU_SAMPLE 帧 payload（24B = 6×float LE）→ (ax,ay,az,gx,gy,gz)。"""
    import struct
    if len(payload) != 24:
        raise ValueError(f"imu sample payload {len(payload)}B != 24B")
    return struct.unpack("<6f", payload)


def decode_cal(payload: bytes):
    """IMU_CAL 帧 payload（76B = 19×float LE）→ 19 元组，块序与协议一致。

    布局契约 = proto/frame.h PROTO_TYPE_IMU_CAL 注释（镜像）：
      [0:6]   原始六轴   ax,ay,az（g），gx,gy,gz（dps）   未去偏
      [6:12]  补偿后六轴 ax,ay,az（g），gx,gy,gz（dps）   已减当前零偏
      [12:18] 当前零偏   ax,ay,az（g），gx,gy,gz（dps）   温补后生效值
      [18]    芯片温度 temp（°C）
    消费方（exp2_bias 页）自行做窗口统计；本端保真解码不做任何数值修正。
    """
    import struct
    if len(payload) != PAYLOAD_CAL_B:
        raise ValueError(f"imu cal payload {len(payload)}B != {PAYLOAD_CAL_B}B")
    return struct.unpack("<19f", payload)


def decode_euler(payload: bytes):
    """EULER 帧 payload（12B = 3×float LE）→ (roll, pitch, yaw)，单位弧度（ZYX）。

    分量序与单位契约见 proto/frame.h PROTO_TYPE_EULER 注释（与固件
    mahony_get_angles 输出同源）；消费方自行换算 deg，本端不做角度归一
    （保真解码——pitch 理论域 ±π/2，roll/yaw ±π，越界即上游异常，如实透传）。
    """
    import struct
    if len(payload) != PAYLOAD_EULER_B:
        raise ValueError(f"euler payload {len(payload)}B != {PAYLOAD_EULER_B}B")
    return struct.unpack("<3f", payload)


def decode_quat(payload: bytes):
    """QUAT 帧 payload（16B = 4×float LE，w 在前）→ (qw,qx,qy,qz)。

    分量序与固件 mahony_get_quat 输出一致（proto/frame.h TYPE_QUAT 注释）；
    |q|≈1 由发送端保持，本端不归一化（保真解码，姿态消费方自行容错）。
    """
    import struct
    if len(payload) != PAYLOAD_QUAT_B:
        raise ValueError(f"quat payload {len(payload)}B != {PAYLOAD_QUAT_B}B")
    return struct.unpack("<4f", payload)


def encode_quat(seq: int, w: float, x: float, y: float, z: float) -> bytes:
    """四元数样本 → QUAT 整帧字节（与固件同规格；sim_source 合成流用）。

    入参：seq 帧序号（自动按 0..255 回绕）；(w,x,y,z) 四元数分量（分量序=协议序）。
    出参：21B 帧 = SOF | LEN=16 | TYPE | SEQ | payload(16B) | CRC8（覆盖 LEN..PAYLOAD）。
    """
    import struct
    payload = struct.pack("<4f", w, x, y, z)
    head = bytes((SOF, PAYLOAD_QUAT_B, TYPE_QUAT, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


def encode_cal(seq: int, vals) -> bytes:
    """IMU_CAL 样本 → 0x02 整帧字节（与固件同规格；sim_source 合成流用）。

    入参：seq 帧序号（自动按 0..255 回绕）；vals = 19 个数（块序=协议序，
          见 decode_cal 布局注释：raw6 | deb6 | bias6 | temp）。
    出参：81B 帧 = SOF | LEN=76 | TYPE | SEQ | payload(76B) | CRC8（覆盖 LEN..PAYLOAD）。
    """
    import struct
    payload = struct.pack("<19f", *vals)
    head = bytes((SOF, PAYLOAD_CAL_B, TYPE_IMU_CAL, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))


def encode_text(seq: int, text: str) -> bytes:
    """ASCII 文本 → 0x7F 整帧字节（与固件 send_text 同规格；sim 源开机横幅用）。

    入参：seq 帧序号（自动按 0..255 回绕）；text ASCII（超 PAYLOAD_MAX 截断，
          与固件 strlen+len 上限口径一致——调用方自行保证不超）。
    """
    payload = text.encode("ascii", "replace")[:PAYLOAD_MAX]
    head = bytes((SOF, len(payload), TYPE_TEXT, seq & 0xFF))
    return head + payload + bytes((crc8(head[1:] + payload),))
