#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""tests/run_all.py — 上位机测试总入口（headless，一键跑全套）。

分组：
  A. Python 纯逻辑/协议/桥/路由测试（不需要 Qt 窗口）
  B. node 页面 harness（DOM/桥桩，不需要浏览器）
  C. Qt-WebEngine 页面链路测试（offscreen，需 PySide6 + WebEngine；--no-qt 可跳过）

跑法：
  python tests/run_all.py            # A+B+C
  python tests/run_all.py --no-qt    # A+B（CI/无 WebEngine 环境）

退出码：0 = 全过；1 = 有失败（逐项打印）。
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PY_TESTS = [
    "test_frame.py",            # 帧协议编解码（与固件同源向量）
    "test_sim_source.py",       # 仿真 IMU 源（帧规格/时序）
    "test_sim_cal_source.py",   # 仿真调零温补源（开机标定时序）
    "test_trajectory.py",       # 轨迹重建（ZUPT/高通/零偏）
    "test_app_pages.py",        # 路由表 + --mode/--lang 解析 + exp3 拒绝路径
    "test_proto_fuzz.py",       # 协议鲁棒性（半帧/坏 CRC/假 SOF/seq 回绕）
    "test_serial_socket.py",    # socket 源（迟到服务器/断流重连/idle 收口）
    "test_exp5_bridge.py",      # 桥 + 仿真源线程生命周期
    "test_exp3_mode.py",        # exp3 显式模式（HTTP 切换/real 未就绪）
    "test_imu_page_server.py",  # IMU 统一页（逐级继承）：四页路由/仿真帧型覆盖/sim 调零
    "test_server.py",           # 服务器版：路由/显式模式/SSE 数据面/单实例重启
    "test_motor_server.py",     # 电机页服务器版：MotorHub/场景+pidscan sim/real 握手/409
    "test_store_server.py",      # store_demo 页服务器版：StoreHub（MOD_STORE GET/SET/STATUS）/TEXT/sim/real 假桥
    "test_pidscan_sim_source.py",   # pidscan 仿真源契约
    "test_motor_sim_source.py",     # motor 仿真源契约（帧规格/场景断言）
]

NODE_TESTS = [
    "ui_shell_harness.js",      # 模式/语言/词典键对齐/禁自动启动
    "imu_page_harness.js",      # IMU 统一页核心：薄壳面板逐级包含/0x02·0x03·0x04 面板/双语
    "exp1_wave_harness.js",     # 波形泵（抖动缓冲/匀速消费/相位）
    "exp2_page_harness.js",     # exp2 页面逻辑
    "exp4_page_harness.js",     # exp4 页面逻辑
    "exp5_page_harness.js",     # exp5 页面逻辑
    "store_page_harness.js",     # store_demo 页：三按钮 GET/RESET/STATUS + TEXT 日志 + 显式模式
]

QT_TESTS = [
    "test_exp1_page_qt.py",     # exp1：未启动态 + 仿真链路 + 双语
    "test_exp2_page_qt.py",     # exp2：0x02 流 + 温补面板
    "test_exp4_page_server.py", # exp4（服务器页）：QUAT 流 + 欧拉对照 + 双语
    "test_exp5_page_qt.py",     # exp5：双流 + 轨迹
]


def run(cmd, extra_env=None):
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    if extra_env:
        env.update(extra_env)
    proc = subprocess.run(cmd, cwd=ROOT, env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return proc.returncode, proc.stdout.strip().splitlines()


def main(argv) -> int:
    no_qt = "--no-qt" in argv[1:]
    groups = [("A·python", [([sys.executable, os.path.join(HERE, t)], None) for t in PY_TESTS]),
              ("B·node", [(["node", os.path.join(HERE, t)], None) for t in NODE_TESTS])]
    if not no_qt:
        groups.append(("C·qt-webengine",
                       [([sys.executable, os.path.join(HERE, t)], {"QT_QPA_PLATFORM": "offscreen"})
                        for t in QT_TESTS]))

    failures = []
    for name, items in groups:
        print(f"\n=== {name} ===")
        for cmd, env in items:
            label = os.path.basename(cmd[-1])
            code, out = run(cmd, env)
            tail = out[-1] if out else ""
            print(f"  [{'PASS' if code == 0 else 'FAIL'}] {label}  {tail[:110]}")
            if code != 0:
                failures.append((label, out))

    print()
    if failures:
        for label, out in failures:
            print(f"--- {label} 失败输出（末 15 行） ---")
            for line in out[-15:]:
                print("   ", line)
        print(f"== run_all: {len(failures)} 组失败 ==")
        return 1
    print("== run_all: 全过 ==")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
