# exp1_waveform — IMU 六轴波形示例

> 运行入口示例：进本文件夹双击 `run.bat`（或命令行执行），自动启动上位机。

## 用法

| 命令 | 效果 |
| --- | --- |
| `run.bat` | **真实模式**（缺省）：不自动启动任何数据源，窗口内选串口点「连接」；无设备时窗口空转，不会自己造数据 |
| `run.bat sim` | 仿真模式（显式）：`python app.py --page exp1 --mode sim`——合成六轴正弦摆动 100Hz，Three.js 波形+帧计数 |
| `run.bat COM57` | 真串口：连接板载固件（exp1_raw 烧录后），真 I2C 数据流 |

## 前置

- Python 3.12+ 与 `PySide6`、`pyserial`（`pip install PySide6 pyserial`）
- 无板演示零额外依赖；真串口需 Embedded exp1 固件已烧录

## 入口链路

`run.bat` → 项目根 `app.py`（webview 壳）→ `backend/sim_source.py`（合成流）或 `backend/serial_port.py`（真串口）→ `backend/frame.py` 解码 → QWebChannel 桥 → `web/exp1/index.html` Three.js 波形页。
