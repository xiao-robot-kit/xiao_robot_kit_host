# exp2_bias — 调零温补示例页

> 实验二 host 页（对应固件 `exp2_calib`）：0x02 IMU_CAL 帧的零偏统计 + 温补前后对比。
> 入口：进本文件夹双击 `run.bat`（或命令行执行）；主壳统一路由 `python app.py --page exp2`。

## 用法

| 命令 | 效果 |
| --- | --- |
| `run.bat` | **真实模式**（缺省）：不自动启动任何数据源，窗口内选串口点「连接」 |
| `run.bat sim` | 仿真模式（显式）：`python app.py --page exp2 --mode sim`——SimCalSource 镜像固件合成源预设 `sim.exp2.static`（静止 Z_UP + 注入零偏 gx=+1.0/gy=−0.5/gz=+2.5 dps、ax=+0.02/ay=−0.03/az=+0.015 g + 0.1°C/s 温升），**开机时序同固件**：先 TEXT 横幅 → 5 s 标定窗（500 帧，`cal.exp2.bias`）→ 0x02 帧流 |
| `run.bat COM57` | 真串口：连接 exp2_calib 固件（0x02 帧 76B：raw6 \| deb6 \| bias6 \| temp，块序见 `proto/frame.h` TYPE_IMU_CAL） |
| 主壳 | `python app.py --page exp2 [--mode sim \| --sim \| COMx] [--lang zh\|en]`（PAGES 注册表路由，缺省页 exp1；`--sim` 为 `--mode sim` 旧别名） |

## 页面内容

- **最新帧 · 温补前后对比**：raw（补偿前）/ deb（补偿后）/ bias（当前生效零偏，
  温补后）三块分列 + 芯片温度；参考列 = 静止 Z_UP 口径（az≈+1g、陀螺≈0）
- **零偏统计**（滑窗 300 帧 = 100Hz 下 3 s）：raw 与 deb 的窗口均值 ± 样本标准差
  ——raw 均值随温度爬走（零偏+温漂可见），deb 均值贴住参考值（补偿生效）
- **调零按钮**：① 经串口向固件发 `CAL\n` 明文行（与 exp_motor 的 `SCEN <n>`
  命令同款下行协议，链路演示）；② 重开统计窗。
  **现状（如实）**：当前 exp2 固件**开机自标定**（日志 `OK calib 500 frames…`）、
  **不解析下行命令**——按钮演示命令下发链路；固件接 RX 解析后即生效（回执 0x7E）。
  仿真模式无下行通道，回执会如实说明
- **固件日志**（TEXT 帧）：开机标定时序（`OK sim` → `OK calib` → `OK stream`）

## headless 可测部分

页面数值换算 `makeWinStats`（滑窗均值/σ/min/max）与 0x02 入页/调零按钮/断开清窗
逻辑——`tests/exp2_page_harness.js`（node，无浏览器）逐项断言，内联脚本副本
`tests/exp2_page_inline.js` 与页面逐字同步（同步守卫）。GUI 视觉项归人工开窗。

## 前置

- Python 3.12+ 与 `PySide6`、`pyserial`（同 exp1_waveform）
- 无板演示零额外依赖；真串口需 Embedded exp2_calib 固件已烧录
  （QEMU 联调变体编译须带双宏 `-DEXP_SIM=1 -DEXP_QEMU_UART=1`，见 GOV-ERR-004）

## 入口链路

`run.bat` → 项目根 `app.py --page exp2` → `backend/bridge.py`（0x02 帧解码透传
frameReady）→ `web/exp2/index.html` 调零温补页；仿真档（`--mode sim`）走 `backend/sim_source.py`
SimCalSource（一致性由 `tests/test_sim_cal_source.py` 保证）。
