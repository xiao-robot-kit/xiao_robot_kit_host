"""QWebChannel 桥：串口/模拟帧流 → 网页（PySide6 壳的 backend 侧）。

依赖声明：PySide6（LGPL/Qt 官方，预研仓已认可架子沿用）；pyserial（BSD-3）。
"""

import time
from typing import List, Optional

import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal, Slot, QThread

from frame import (FrameDecoder, decode_cal, decode_quat, decode_sample,  # noqa: E402
                   TYPE_IMU_CAL, TYPE_IMU_SAMPLE, TYPE_QUAT, TYPE_TEXT)
from sim_source import (SIM_CAL_PRESETS, SIM_PRESETS, SIM_TRAJ_PRESETS,  # noqa: E402
                        SimCalSource, SimImuCfg, SimImuSource, SimTrajSource)
from trajectory import TrajCfg, TrajectoryReconstructor  # noqa: E402

# host→device 调零命令（明文行协议，与 exp_motor 的「SCEN <n>」命令同款：
# scope_motor 场景握手即裸 ASCII 行下发）。当前 exp2 固件开机自标定、
# 不解析下行——本命令演示下发链路；固件侧接 RX 解析后即生效（回执走 0x7E）。
CAL_CMD_LINE = "CAL\n"


class _FrameWorker(QThread):
    """帧流后台线程骨架（真实串口 / --sim 模拟源共用）：字节 chunk → 解码 → 信号。

    信号协议（页面桥契约，签名勿改——串口与 sim 走同一条路径）：
      frameReady(type, seq, vals)   解码值：IMU_SAMPLE=6 值（前 3 g、后 3 dps）；
                                    IMU_CAL=19 值（raw6 | deb6 | bias6 | temp，
                                    exp2 调零温补页消费）；QUAT=4 值
                                    (qw,qx,qy,qz)（exp4 起固件发送；
                                    exp1 页面配 exp1 固件只收 0x01，不受影响）
      textReady(str)                TEXT 帧文本
      statReady(imu, text, drop)    计数统计（imu 只计 0x01；drop 按 IMU seq
                                    回绕差估计——QUAT/IMU_CAL 帧独立 seq，不掺入）
      posReady(x, y, z)             轨迹模式（traj_cfg 传入时）：本线程内重建的
                                    世界系位置（m）；裸数据模式不发
      errorOccurred(str)            源异常（串口拔出/占用等）

    轨迹重建**在 worker 线程内**完成（QUAT 帧存最新姿态，IMU 帧到达时配对
    喂 TrajectoryReconstructor——hold-latest 四元数口径，满足其单线程串行
    feed 约束）。不放在桥（WebChannel 注册对象）侧的原因（PySide6 6.12
    实测）：把注册对象的普通 Python 方法 connect 到 worker 信号后，该对象
    一切 Python→JS 信号转发即失效（连空转槽也复现）——重建留在 worker、
    桥只做 signal→signal 中继（与 frameReady/textReady 同款已验证路径）。
    """

    frameReady = Signal(int, int, list)
    textReady = Signal(str)
    statReady = Signal(int, int, int)
    posReady = Signal(float, float, float)
    errorOccurred = Signal(str)

    def __init__(self, traj_cfg: Optional[TrajCfg] = None) -> None:
        super().__init__()
        self._stop = False
        self._traj = TrajectoryReconstructor(traj_cfg) if traj_cfg else None
        self._quat: Optional[List[float]] = None   # 最新四元数（配对用）

    def run(self):  # noqa: D102
        dec = FrameDecoder()
        n_imu = n_text = last_seq = -1
        drop = 0
        try:
            self._open()
            while not self._stop:
                chunk = self._read_chunk()
                if not chunk:
                    continue
                for typ, seq, payload in dec.feed(chunk):
                    if typ == TYPE_IMU_SAMPLE:
                        n_imu += 1
                        if last_seq >= 0:
                            drop += (seq - (last_seq + 1)) & 0xFF
                        last_seq = seq
                        vals = list(decode_sample(payload))
                        self.frameReady.emit(typ, seq, vals)
                        if self._traj is not None and self._quat is not None:
                            w, x, y, z = self._quat
                            px, py, pz = self._traj.feed(
                                w, x, y, z, vals[0], vals[1], vals[2])
                            self.posReady.emit(px, py, pz)
                    elif typ == TYPE_IMU_CAL:
                        # exp2 调零温补帧：19 值整块透传（统计在页面侧做）
                        self.frameReady.emit(typ, seq, list(decode_cal(payload)))
                    elif typ == TYPE_QUAT:
                        qv = list(decode_quat(payload))
                        self.frameReady.emit(typ, seq, qv)
                        if self._traj is not None:
                            self._quat = qv
                    elif typ == TYPE_TEXT:
                        n_text += 1
                        self.textReady.emit(payload.decode("ascii", "replace"))
                    if n_imu % 20 == 0 or n_text:
                        self.statReady.emit(n_imu, n_text, drop)
        except Exception as e:  # 串口拔出/占用等
            self.errorOccurred.emit(str(e))
        finally:
            self._close()

    def stop(self):
        self._stop = True

    # —— 子类钩子 ——
    def _open(self) -> None:
        """打开数据源；失败抛异常（经 errorOccurred 上报）。"""

    def _read_chunk(self) -> bytes:
        """读一段字节（可阻塞带超时）；空 bytes 表示暂无数据。"""
        raise NotImplementedError

    def _close(self) -> None:
        """释放数据源（幂等：未打开时 no-op）。"""


class _SerialWorker(_FrameWorker):
    """真实串口源（pyserial；读超时 1s，块上限 512B）。"""

    def __init__(self, port: str, baud: int,
                 traj_cfg: Optional[TrajCfg] = None):
        super().__init__(traj_cfg)
        self._port, self._baud = port, baud
        self._ser = None

    def _open(self):
        self._ser = serial.Serial(self._port, self._baud, timeout=1)

    def _read_chunk(self) -> bytes:
        return self._ser.read(512)

    def send_line(self, line: str) -> None:
        """host→device 明文行下发（下行命令链路，如 exp2 的 CAL——
        与 exp_motor 的 SCEN 命令同款裸 ASCII 行协议）。"""
        if self._ser is not None:
            self._ser.write(line.encode("ascii"))

    def _close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None


class _SimWorker(_FrameWorker):
    """模拟源线程（--mode sim 仿真）：假串口源 → 同一解码/信号路径。

    入参：source = 显式源对象（SimImuSource 或 SimTrajSource，鸭子类型：
          poll(now_s)→bytes）；None 时按 cfg 建六轴摆源。poll_ms = 轮询周期
          （ms；缺省 10ms@100Hz → 1 帧/次，把批量化抖动压到最小）。
          traj_cfg 透传 _FrameWorker（轨迹模式）。
    节拍：**绝对截止时刻**递增（next_deadline += poll_s），不用「睡多久」
          累加——msleep 的实际睡眠长度在 Windows 上按系统 tick 量化，
          累加睡会慢漂移（旧实现实测 49 次/s 而目标 50，批大小 1~4 帧乱跳）。
    """

    def __init__(self, cfg: Optional[SimImuCfg] = None, poll_ms: float = 10.0,
                 source=None, traj_cfg: Optional[TrajCfg] = None):
        super().__init__(traj_cfg)
        self._cfg = cfg if cfg is not None else SIM_PRESETS["sway10_2s"]
        self._poll_ms = poll_ms
        self._source = source
        self._src: Optional[SimImuSource] = None
        self._t0 = 0.0
        self._next_deadline = 0.0

    def _open(self):
        self._src = self._source if self._source is not None \
            else SimImuSource(self._cfg)
        self._t0 = time.monotonic()
        self._next_deadline = self._t0

    def _read_chunk(self) -> bytes:
        now = time.monotonic()
        wait_s = self._next_deadline - now
        if wait_s > 0:
            self.msleep(max(1, int(wait_s * 1000)))     # 睡到截止时刻（非忙等）
        self._next_deadline += self._poll_ms / 1000.0
        # 落后于截止时刻时对齐到现在，避免追赶式一秒狂轮（线程被挂起后回补）
        now = time.monotonic()
        if self._next_deadline < now - 0.25:
            self._next_deadline = now
        return self._src.poll(now - self._t0)

    def _close(self):
        self._src = None


class FrameBridge(QObject):
    """网页可调用的桥对象（new QWebChannel(qt.webChannelTransport).objects.bridge）。

    exp5 轨迹模式：traj_cfg 传入时 worker 线程内跑 TrajectoryReconstructor
    （单源算法，测试见 tests/test_trajectory.py），QUAT+IMU 帧配对喂入、
    重建位置经 posReady 推给页面（中继见 _FrameWorker 注）。不传 traj_cfg
    = 裸数据模式（exp1 原口径，posReady 不发）。重连即重建
    worker = 重建器状态清零（轨迹重新起笔）。
    sim_mode（--sim 演示的合成源选型，app.py 按页注入）：
      "imu"（缺省）SimImuSource 六轴摆——exp1 波形页；
      "traj"      SimTrajSource 摆动 QUAT+IMU 双流——exp4 姿态页
                  （traj_cfg=None，只看姿态不做轨迹重建）与 exp5 轨迹页
                  （traj_cfg 同时传入）；
      "cal"       SimCalSource 静止注入零偏+温漂 0x02 流——exp2 调零温补页。
    注意：当前 exp4 固件只发 0x04 无加速度——真实串口模式下无 0x01 帧则
    不重建（页面仍显示姿态），完整轨迹演示走 --sim（SimTrajSource 双流）。
    """

    SIM_IMU = "imu"      # --sim 源选型：SimImuSource（exp1）
    SIM_TRAJ = "traj"    # --sim 源选型：SimTrajSource（exp4/exp5）
    SIM_CAL = "cal"      # --sim 源选型：SimCalSource（exp2）

    frameReady = Signal(int, int, list)
    textReady = Signal(str)
    statReady = Signal(int, int, int)
    portChanged = Signal(bool)      # 连接状态（真实串口或 --sim 模拟源）
    posReady = Signal(float, float, float)   # exp5：重建世界系位置 (x,y,z)（m）

    def __init__(self, traj_cfg: Optional[TrajCfg] = None,
                 sim_mode: str = SIM_IMU):
        super().__init__()
        if sim_mode not in (self.SIM_IMU, self.SIM_TRAJ, self.SIM_CAL):
            raise ValueError(f"sim_mode={sim_mode!r} 须为 imu/traj/cal")
        self._worker: Optional[_FrameWorker] = None
        self._traj_cfg = traj_cfg
        self._sim_mode = sim_mode

    @Slot(result=list)
    def listPorts(self) -> List[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    @Slot(str, int)
    def openPort(self, port: str, baud: int):
        if self._worker:
            return
        self._start_worker(_SerialWorker(port, baud, traj_cfg=self._traj_cfg))

    @Slot()
    def openSim(self):
        """仿真源：显式启动（--mode sim / 页面选「仿真」后点启动）。

        源选型见类注 sim_mode（imu/traj/cal）；traj 模式在 exp5（traj_cfg
        非 None）时附带轨迹重建。
        """
        if self._worker:
            return
        if self._sim_mode == self.SIM_CAL:
            src = SimCalSource(SIM_CAL_PRESETS["exp2_static"])
            self._start_worker(_SimWorker(source=src))
        elif self._sim_mode == self.SIM_TRAJ or self._traj_cfg is not None:
            src = SimTrajSource(SIM_TRAJ_PRESETS["traj_sway"])
            self._start_worker(_SimWorker(source=src, traj_cfg=self._traj_cfg))
        else:
            self._start_worker(_SimWorker())

    @Slot(result=dict)
    def sendCalib(self) -> dict:
        """调零命令下发（exp2 页「调零」按钮）：明文行 CAL\\n 经串口下发。

        回执**键值化**（code + detail）而非中文字串：文案由页面按当前语言
        选择（web/assets/ui-shell.js 的 msg.* 词典），后端不再拼界面文案。
        下行链路演示（与 exp_motor 的 SCEN 命令同款协议）。当前 exp2 固件
        开机自标定、不解析下行命令——本调用真实串口连接时写出字节并回执
        状态；仿真源无下行通道，如实回执。
        """
        w = self._worker
        if isinstance(w, _SerialWorker):
            w.send_line(CAL_CMD_LINE)
            return {"code": "calibSent", "cmd": CAL_CMD_LINE.strip()}
        if isinstance(w, _SimWorker):
            return {"code": "calibSimNoDownlink"}
        return {"code": "notConnected"}

    @Slot()
    def closePort(self):
        if self._worker:
            self._worker.stop()
            self._worker.wait(1500)
            self._worker = None
            self.portChanged.emit(False)

    def _start_worker(self, w: _FrameWorker) -> None:
        """统一接线：worker 信号 → 桥信号（串口/sim 共用，语义与旧 openPort 一致）。"""
        self._worker = w
        w.frameReady.connect(self.frameReady)
        w.textReady.connect(self.textReady)
        w.statReady.connect(self.statReady)
        w.posReady.connect(self.posReady)
        w.errorOccurred.connect(self.textReady)
        w.errorOccurred.connect(lambda _: self.portChanged.emit(False))
        w.finished.connect(lambda: self.portChanged.emit(False))
        w.start()
        self.portChanged.emit(True)
