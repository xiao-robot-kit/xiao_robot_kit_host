# exp4_quat_view — 四元数 3D 姿态示例页

> 实验四 host 页（对应固件 `exp4_quat`）：0x04 QUAT 帧驱动 Three.js 3D 立方体姿态 +
> 四元数四分量数值 + 欧拉对照（exp3↔exp4 教学呼应）。与 exp3_euler_view 名字对仗。
> 入口：进本文件夹双击 `run.bat`（或命令行执行）；主壳统一路由 `python app.py --page exp4`。

## 用法

| 命令 | 效果 |
| --- | --- |
| `run.bat` | **真实模式**（缺省）：不自动启动任何数据源，窗口内选串口点「连接」 |
| `run.bat sim` | 仿真模式（显式）：`python app.py --page exp4 --mode sim`——SimTrajSource 摆动 QUAT 流（±5° 姿态随动摆动）驱动立方体 |
| `run.bat COM57` | 真串口：连接 exp4_quat 固件（0x04 四元数流：qw,qx,qy,qz 4×float32 LE，w 在前，\|q\|≈1） |
| 主壳 | `python app.py --page exp4 [--mode sim \| --sim \| COMx] [--lang zh\|en]`（PAGES 注册表路由，缺省页 exp1） |

## 页面内容

- **3D 立方体**：面色区分 ±轴 + 棱线 + 体轴三线（X红 Y绿 Z蓝）随 QUAT 帧旋转；
  世界网格/世界轴不动（姿态 vs 世界对照）；拖拽旋转 / 滚轮缩放 / 双击复位
  （自研轨道控制，零第三方）——**\|q\|=1 主通道**：四元数无奇异，直接驱动渲染
- **四元数四分量**：qw/qx/qy/qz 帧原值 + \|q\| 范数（发送端保持 ≈1 的健康度读数）
- **欧拉对照**：本页从四元数**单源解算** ZYX 欧拉（公式与固件 `quat.c`
  `quat_to_euler` 同式，含 ±90° 万向锁钉法）；pitch ≥ 85° 时锁指示亮起——
  与 exp3 页（直读 0x03 帧）对照着看：同一姿态、两条表示通道
- **固件日志**（TEXT 帧）

## headless 可测部分

数值换算 `quatToEulerZyx`/`quatNorm`（与固件 quat.c 同式的镜像契约）、QUAT 帧→
立方四元数参数序、万向锁 91° 姿态退化（ZYX 反解显示 180°/89°/180°——欧拉丢
自由度的数值证据）、锁指示——`tests/exp4_page_harness.js`（node，无浏览器）
逐项断言，内联副本 `tests/exp4_page_inline.js` 与页面逐字同步（同步守卫）。
WebGL 像素渲染与轨道控制手感归人工开窗。

## 前置

- Python 3.12+ 与 `PySide6`、`pyserial`（同 exp1_waveform）；Three.js 用本地自有副本
  `web/assets/vendor/three.min.js`（r128，MIT，离线无 CDN）
- 无板演示零额外依赖；真串口需 Embedded exp4_quat 固件已烧录
  （QEMU 联调变体编译须带双宏 `-DEXP_SIM=1 -DEXP_QEMU_UART=1`，见 GOV-ERR-004）

## 入口链路

`run.bat` → 项目根 `app.py --page exp4` → `backend/bridge.py`（QUAT 帧经
frameReady 透传，仿真档源选型 traj）→ `web/exp4/index.html` 3D 姿态页。
