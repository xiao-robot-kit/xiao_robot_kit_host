"""串口帧流读取（pyserial 封装，POSIX 语义）。

依赖声明：pyserial（MIT）——本项目唯一第三方 Python 库，登记于 README 依赖声明段。
"""

import time
from typing import Iterator, Tuple

import serial  # pyserial, MIT

from frame import FrameDecoder, decode_sample


class SerialFrameSource:
    """打开串口并持续产出解码后的帧 (type, seq, payload)。"""

    def __init__(self, port: str, baud: int = 115200):
        self._port = port
        self._baud = baud

    def frames(self) -> Iterator[Tuple[int, int, bytes]]:
        """阻塞生成器；串口异常抛 serial.SerialException。"""
        with serial.Serial(self._port, self._baud, timeout=1) as ser:
            dec = FrameDecoder()
            while True:
                chunk = ser.read(256)
                if chunk:
                    for f in dec.feed(chunk):
                        yield f


def run_exp1_probe(port: str, seconds: float = 5.0) -> dict:
    """实验一链路验收（headless，无 UI）：收帧统计。

    返回 {"imu": n, "text": n, "bad_crc": n, "first_text": str|None}
    验收口径：imu > 0 且 az 绝对值在 0.8~1.2g（静止板）。
    """
    stats = {"imu": 0, "text": 0, "bad_crc": 0, "first_text": None}
    t0 = time.time()
    src = SerialFrameSource(port)
    az_last = None
    for typ, seq, payload in src.frames():
        if typ == 1:  # TYPE_IMU_SAMPLE
            stats["imu"] += 1
            ax, ay, az, *_ = decode_sample(payload)
            az_last = az
        elif typ == 0x7F:  # TYPE_TEXT
            stats["text"] += 1
            if stats["first_text"] is None:
                stats["first_text"] = payload.decode("ascii", "replace")
        if time.time() - t0 > seconds:
            break
    stats["az_last"] = az_last
    return stats
