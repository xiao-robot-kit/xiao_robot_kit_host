"""QEMU 串口→TCP socket 帧流读取（pyserial socket:// URL，POSIX 语义）。

依赖声明：pyserial（MIT）——本项目唯一第三方 Python 库，登记于 README 依赖声明段。

与 serial_port.SerialFrameSource 同构（frames() 生成器产出 (type, seq, payload)），
差异仅在传输端：本端是 pyserial URL（如 socket://127.0.0.1:5555），即 QEMU
`-serial tcp:...,server` 的 host 侧直连客户端。接线范式与 esp-idf 6.1 自身的
qemu 集成一致（QEMU_PORT_SERIAL=5555 + pyserial socket://localhost:5555，
见 esp-idf tools/idf_py_actions/qemu_ext.py）。

pyserial socket 后端契约（3.5，serial/urlhandler/protocol_socket.py）：
- 连接失败（对端未监听）与流断（EOF/复位）都抛 serial.SerialException——
  前者发生在 open、后者发生在 read（'socket disconnected'）；本类按「阶段」
  区分二者：连接阶段失败 = 对端未就绪 → 按退避重试；读阶段异常 = 流断 →
  按 on_eof 处置（重连或返回）。
- read 的 POSIX 语义：timeout 到点返回已收字节（可能为空 b''）。
- 串口参数（baud 等）被该后端静默忽略——故本类无 baud 形参（与
  SerialFrameSource 的显式差异，勿照搬）。
- URL 必须经 serial.serial_for_url() 打开（serial.Serial 只认本机设备名）。
两个实测坑（3.5，本机 Windows 复现）：
- read(N>1) 撞上 EOF 时抛异常并丢弃本次调用内已收的半截字节（内部
  bytearray 不返还）→ 唯 read(1) 构造上零丢失（EOF 时局部缓冲必为空），
  故本类 read_size 默认 1；
- open() 末尾 reset_input_buffer 会排空 open 窗口内已到数据 → 对端在
  open 完成前发送的字节丢失。QEMU `-serial tcp:...,server`（无 nowait）
  等连接成功才开机，天然规避此坑（实测 ROM 横幅完整）。
"""

import time
from typing import Iterator, Optional, Tuple

import serial  # pyserial, MIT

from frame import FrameDecoder, TYPE_IMU_SAMPLE, TYPE_TEXT, decode_sample


class SerialSocketSource:
    """连接 pyserial socket:// URL 并持续产出解码后的帧 (type, seq, payload)。

    入参（单位：秒=s、字节=B）：
        url                 pyserial URL，如 "socket://127.0.0.1:5555"
        read_timeout        单次 read 超时（s；到点返回已有字节，POSIX 读语义）
        read_size           单次 read 上限（B）。默认 1 且建议保持 1：pyserial
                            socket 后端 read(N>1) 在 EOF 时丢弃本次已收半截
                            字节（见模块注「两个实测坑」），唯 read(1) 零丢失
        reconnect_initial_s 重连退避初值（s；指数翻倍，封顶 reconnect_max_s；
                            连接成功即复位回初值）
        reconnect_max_s     重连退避上限（s）
        on_eof              流断（EOF/连接复位）处置："reconnect"（默认，扛对端
                            重启，如 QEMU 重跑）或 "stop"（生成器就地返回，
                            有限长合成流/测试用）
        max_idle_s          无帧上限（s；None=无限）。连接/读两阶段统一口径：
                            持续这么久没有产出任何完整帧（含对端消失反复重连、
                            或连接活着但只有非帧字节），生成器返回。有界收流
                            （如 run_socket_probe）必设；QEMU 长跑监视不设。
    """

    ON_EOF_RECONNECT = "reconnect"
    ON_EOF_STOP = "stop"

    def __init__(
        self,
        url: str,
        read_timeout: float = 1.0,
        read_size: int = 1,
        reconnect_initial_s: float = 0.5,
        reconnect_max_s: float = 5.0,
        on_eof: str = ON_EOF_RECONNECT,
        max_idle_s: Optional[float] = None,
    ):
        if on_eof not in (self.ON_EOF_RECONNECT, self.ON_EOF_STOP):
            raise ValueError(f"on_eof={on_eof!r} 须为 'reconnect' 或 'stop'")
        if read_timeout <= 0 or read_size <= 0:
            raise ValueError(f"read_timeout/read_size 须为正（{read_timeout}/{read_size}）")
        if reconnect_initial_s <= 0 or reconnect_max_s < reconnect_initial_s:
            raise ValueError(
                f"退避参数须满足 0 < initial({reconnect_initial_s}) <= max({reconnect_max_s})")
        if max_idle_s is not None and max_idle_s <= 0:
            raise ValueError(f"max_idle_s={max_idle_s} 须为 > 0 或 None")
        self._url = url
        self._read_timeout = read_timeout
        self._read_size = read_size
        self._reconnect_initial_s = reconnect_initial_s
        self._reconnect_max_s = reconnect_max_s
        self._on_eof = on_eof
        self._max_idle_s = max_idle_s

    def frames(self) -> Iterator[Tuple[int, int, bytes]]:
        """阻塞生成器：产出 (type, seq, payload)。

        连接阶段失败（对端未就绪）→ 指数退避后重试（无限，直到对端出现）；
        读阶段异常（流断）→ on_eof="stop" 时生成器返回，"reconnect" 时退避重连；
        max_idle_s 置 None 外的值时：任何阶段超过该时长无帧产出 → 生成器返回；
        消费者 break/.close 触发 GeneratorExit → 经 finally 关闭底层连接。
        """
        backoff = self._reconnect_initial_s
        t_last_frame = time.time()
        while True:
            # 连接阶段：对端（QEMU socket server）未就绪 → 退避重试
            try:
                ser = serial.serial_for_url(self._url, timeout=self._read_timeout)
            except (serial.SerialException, OSError):
                if self._idle_out(t_last_frame):
                    return
                time.sleep(backoff)
                backoff = self._backoff_next(backoff)
                continue
            backoff = self._reconnect_initial_s   # 连上即复位退避
            dec = FrameDecoder()
            try:
                # 读阶段：流断（EOF/复位）在 pyserial socket 后端表现为 SerialException
                while True:
                    chunk = ser.read(self._read_size)
                    if chunk:
                        for f in dec.feed(chunk):
                            t_last_frame = time.time()
                            yield f
                    if self._idle_out(t_last_frame):
                        return
            except (serial.SerialException, OSError):
                if self._on_eof == self.ON_EOF_STOP:
                    return
            finally:
                ser.close()
            # 流断重连路径：退避后回到连接阶段
            time.sleep(backoff)
            backoff = self._backoff_next(backoff)

    def _idle_out(self, t_last_frame: float) -> bool:
        """无帧超时判定（未设 max_idle_s 时恒 False）；私有。"""
        if self._max_idle_s is None:
            return False
        return time.time() - t_last_frame > self._max_idle_s

    def _backoff_next(self, backoff: float) -> float:
        """下一档退避值（s）：翻倍封顶；私有。"""
        return min(backoff * 2.0, self._reconnect_max_s)


def run_socket_probe(url: str, seconds: float = 5.0, **src_kwargs) -> dict:
    """实验一 socket 链路验收（headless，无 UI）：收帧统计。

    返回口径与 serial_port.run_exp1_probe 一致：
    {"imu": n, "text": n, "bad_crc": 0, "first_text": str|None, "az_last": float|None}
    （bad_crc 恒 0：FrameDecoder 对坏帧静默重同步，与 serial_port 同口径；
    az_last = 最后一块 imu 帧的 az，单位 g。）
    src_kwargs 透传 SerialSocketSource 形参（如 on_eof / 退避参数）。

    有界性（与串口版的关键差异）：默认注入 max_idle_s=seconds——连续 seconds
    无帧（连接活着但只有非帧字节、或对端消失反复重连）时 frames() 自动收口
    返回，不会挂死；逐帧时间预算（>seconds 即停）仍保留。显式传 max_idle_s
    可覆盖该默认（含 None=无限）。
    """
    stats = {"imu": 0, "text": 0, "bad_crc": 0, "first_text": None}
    t0 = time.time()
    src_kwargs.setdefault("max_idle_s", seconds)
    src = SerialSocketSource(url, **src_kwargs)
    az_last = None
    for typ, seq, payload in src.frames():
        if typ == TYPE_IMU_SAMPLE:
            stats["imu"] += 1
            ax, ay, az, *_ = decode_sample(payload)
            az_last = az
        elif typ == TYPE_TEXT:
            stats["text"] += 1
            if stats["first_text"] is None:
                stats["first_text"] = payload.decode("ascii", "replace")
        if time.time() - t0 > seconds:
            break
    stats["az_last"] = az_last
    return stats
