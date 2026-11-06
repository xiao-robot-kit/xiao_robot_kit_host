"""exp5 桥全链 headless 测试（PySide6 offscreen，无 WebEngine/无显示器）。

链路：FrameBridge(traj_cfg) → openSim() → _SimWorker(SimTrajSource) QThread
      → FrameDecoder → frameReady(QUAT/IMU) → _on_traj_frame 配对
      → TrajectoryReconstructor → posReady —— 页面消费的同一条信号路径。

断言：
  1) posReady 事件数 ≥ 200（2.6s 真实墙钟 @100Hz 双流）
  2) frameReady 的 QUAT 帧（type=4）数 ≥ 200（姿态链同在跑）
  3) 重建位移峰值 ≈ 2A（摆动相对起点峰峰值，±15%）——与
     tests/test_trajectory.py 的算法级断言在「桥+线程」环境下复证
  4) y/z 分量 < 1e-3（方向纯净，跨线程浮点不引入串扰）
  5) closePort 后 portChanged(False) 到达（生命周期收口）

跑法：python tests/test_exp5_bridge.py（exit 0 = 全过；需 PySide6）
"""

import math
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # 无显示器环境

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from PySide6.QtWidgets import QApplication  # noqa: E402

from bridge import FrameBridge  # noqa: E402
from sim_source import SIM_TRAJ_PRESETS  # noqa: E402
from trajectory import TRAJ_PRESETS  # noqa: E402

RUN_MS = 2600            # 真实墙钟收流时长（ms）
MIN_EVENTS = 200         # posReady/QUAT 事件下限（2.6s@100Hz=260，留裕量）
TOL = 0.15               # 峰值位移容差（±15%）


def main() -> int:
    fails = []
    app = QApplication.instance() or QApplication([])
    cfg = SIM_TRAJ_PRESETS["traj_sway"]
    want_peak = 2.0 * cfg.amp_m          # 相对起点峰峰（cos 相位起播）

    poses, quats, port_events = [], [], []
    bridge = FrameBridge(TRAJ_PRESETS["exp5_sway"])
    bridge.posReady.connect(lambda x, y, z: poses.append((x, y, z)))
    bridge.frameReady.connect(
        lambda typ, seq, vals: quats.append(typ) if typ == 4 else None)
    bridge.portChanged.connect(lambda on: port_events.append(on))

    bridge.openSim()
    from PySide6.QtCore import QTimer
    QTimer.singleShot(RUN_MS, app.quit)
    app.exec()                          # 真实事件循环（信号跨线程队列派发）
    bridge.closePort()
    app.processEvents()                 # 收口信号的排队派发

    # 1) 事件量
    if len(poses) < MIN_EVENTS:
        fails.append(f"posReady 事件 {len(poses)} < {MIN_EVENTS}")
    if len(quats) < MIN_EVENTS:
        fails.append(f"QUAT 帧事件 {len(quats)} < {MIN_EVENTS}")

    # 2) 位移峰值与方向（算法级断言的桥环境复证）
    if poses:
        peak_x = max(abs(p[0]) for p in poses)
        peak_y = max(abs(p[1]) for p in poses)
        peak_z = max(abs(p[2]) for p in poses)
        if not (want_peak * (1 - TOL) <= peak_x <= want_peak * (1 + TOL)):
            fails.append(f"桥环境峰值 |x|={peak_x:.4f} 超出 "
                         f"{want_peak}±{TOL * 100:.0f}%")
        if peak_y > 1e-3 or peak_z > 1e-3:
            fails.append(f"桥环境方向不纯 |y|={peak_y:.2e} |z|={peak_z:.2e}")
        print(f"  posReady={len(poses)}  quat={len(quats)}  "
              f"peak_x={peak_x:.4f}m  |y|max={peak_y:.2e}  |z|max={peak_z:.2e}")
    else:
        fails.append("posReady 零事件（链路未通）")

    # 3) 生命周期收口
    if True not in port_events:
        fails.append("openSim 未发 portChanged(True)")
    if False not in port_events:
        fails.append("closePort 未发 portChanged(False)")

    print(f"== test_exp5_bridge: {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
