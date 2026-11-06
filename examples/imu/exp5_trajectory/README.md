# exp5_trajectory — 3D 轨迹示例

> 四元数→位移轨迹重建（去重力+二重积分+漂移抑制）+ Three.js 3D 轨迹页（尾部渐隐）。

## 用法

| 命令 | 效果 |
| --- | --- |
| `run.bat` | **真实模式**（缺省）：不自动启动任何数据源，窗口内选串口点「连接」 |
| `run.bat sim` | 仿真模式（显式）：`python app.py --page exp5 --mode sim`（合成摆动 QUAT+IMU 双流→轨迹重建→轨迹页） |
| `run.bat COM57` | 真串口：exp4_quat 固件四元数流驱动轨迹（QEMU 版经 socket 亦可） |

## 前置

同 exp1_waveform（PySide6/pyserial）；轨迹算法为 `backend/trajectory.py`（320 帧单测全绿，静止漂移 0.18mm）。

## 入口链路

`run.bat` → `app.py --page exp5` → QUAT 帧（0x04）→ `backend/trajectory.py` 位置序列 → `web/exp5/index.html` Three.js 渐隐轨迹。
