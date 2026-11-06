<!-- SPDX-License-Identifier: MIT -->
# XIAO Robot Kit Host — 可视化调试上位机

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

面向 **XIAO Robot Kit** 示例的**调试上位机**：已支持的实例都可以在这里
**可视化交互展示**——波形、姿态、电机、参数持久化，sim 仿真 / 真机两档。
避免「盲开发」：先在这里看到数据在动，再回固件调。

## 架构（本地服务器 + 页面）

```
（一个 local server 进程）              （各示例页面）
backend/server.py ──静态页+SSE──▶ examples/<实验>/index.html
       │                    浏览器打开 http://127.0.0.1:8760/<页面>/
       ├─ hub.py     数据源中枢：sim/real + 帧解码 + 广播
       ├─ frame.py   帧协议解码（与固件 frame.h 契约一致）
       ├─ sim_*.py   仿真数据源（无设备也能看到数据在动）
       └─ store_hub.py  pstore 参数持久化演示中枢（GET/RESET/ERASE）
```

- **数据源模式显式**（禁隐式回落）：页面有 [Sim | Real] 开关，默认 Real——
  不插设备不自动仿真、不自动连 QEMU。
- **单实例**：`run.bat` 发现已有服务器就杀掉重启，避免端口/设备抢占
  （`.runtime/server.json` 记 pid/port）。
- **中英双语**：页内 EN/中文 随时切，语言选择存 localStorage。

## 快速开始（无设备 2 分钟看到波形）

```bash
# 依赖：Python 3.10+、pyserial
pip install pyserial

# 方案 A：直接起服务器，浏览器开 http://127.0.0.1:8760/exp1/
python backend/server.py --page exp1 --port 8760

# 方案 B：一键入口（自动开浏览器；单实例）
#   Windows: examples\imu\exp1_waveform\run.bat
```

页面里点 **[Sim] → Start→Connect**：仿真数据流立刻滚动（合成帧与固件同规格，
走与串口同一 FrameDecoder→hub 信号路径）。**没有设备也能先看交互 UI**。

## 支持的可视化实例（页面注册表 `PAGES`）

| 页面 | 看什么 | 仿真档 | 真机 |
| --- | --- | --- | --- |
| exp1 裸数据波形 | 六轴 IMU 原始波形（g/dps） | SimImuSource | 串口 real |
| exp2 调零温补 | 零偏标定 + 温补曲线（GET_BIAS/CALIBRATE/TEMP_CALIB） | SimCalSource | 串口 real |
| exp3 欧拉角 | roll/pitch/yaw（独立页：JSON 轮询） | sim 合成 | QEMU real 帧 |
| exp4 四元数姿态 | 3D 姿态球（\|q\|≡1） | SimTrajSource | 串口 real |
| exp5 位移轨迹 | 匀速分段轨迹重建 | SimTrajSource | 串口 real |
| store_demo | pstore 断电续跑（GET/RESET/ERASE，boot_count 回读） | SimStore 虚拟介质 | flash 分区 real |
| motor_scope | 电机遥测（duty/speed） | SimMotor | QEMU real |
| motor_pidscan | PID 参数扫描 | SimPidscan | QEMU real |

> 真机档 = 插上 XIAO ESP32S3 串口 → 页面里选串口连接；QEMU 档 =
> 固件在 QEMU 里跑、串口桥到本机端口（配套说明见各页面 README 的
> `FirmwareProject/` 占位路径——固件工程请自行 prepare/移植）。

## 页面怎么来的、怎么加新页面

- 每个页面 = `examples/<实验>/index.html` +（可选）`run.bat`，页面自建 DOM
  （薄壳 `#app`），数据面走 SSE（`/api/stream`）。
- 新实验页面：照 `examples/imu/exp1_waveform/` 抄一份 index.html，在
  `backend/server.py` 的 `PAGES` 注册表登记，跑 `tests/run_all.py` 验证。

## 测试（自证：全量本地跑）

```bash
python tests/run_all.py        # 27 组：frame/hub/server/页面/仿真源/协议模糊
node  tests/store_page_harness.js # store 页交互
```

用户界面无设备也能验证：`tests/*_harness.js`（JSDOM/无头浏览器）直接驱动
页面逻辑断言。

## 目录布局

```
xiao_robot_kit_host/
├── backend/         server.py + hub.py + frame.py + 各仿真源 + store_hub.py
├── web/assets/      ui-shell.js（统一壳：模式/双语）+ 各页面脚本
├── examples/        各实验页面（index.html + run.bat + README）
├── tests/           27 组测试（Python + Node 双端）
└── app.py           可选桌面客户端（PySide6 webview 壳；产品本体是 server.py）
```

## 与其它仓的关系

- 固件侧帧协议见 [xiao_robot_kit_qemu docs/protocol-spec.md](https://github.com/xiao-robot-kit/xiao_robot_kit_qemu/blob/main/docs/protocol-spec.md)
- 固件/库：[xiao_robot_kit_idf](https://github.com/xiao-robot-kit/xiao_robot_kit_idf) / [xiao_robot_kit_arduino](https://github.com/xiao-robot-kit/xiao_robot_kit_arduino)
- 使用教学：[xiao_robot_kit_docs](https://github.com/xiao-robot-kit/xiao_robot_kit_docs)
- 无实物调试：[xiao_robot_kit_qemu](https://github.com/xiao-robot-kit/xiao_robot_kit_qemu)

## License

[MIT](LICENSE) © XIAO Robot Kit Contributors