# exp3_euler_view — 欧拉角姿态示例页

> 实验三 host 页入口：进本文件夹双击 `run.bat`（或命令行执行）。
> **零第三方依赖**：Python 标准库（http.server / socket / struct）+ 自研 canvas，
> 不走 PySide6/QWebChannel 主壳（那是 exp1/exp2/exp4/exp5 页的依赖链）；
> 主壳侧另有统一路由：`python app.py --page exp3`（子进程拉起本页 + webview 载入）。
> 唯一例外：真实模式的**串口目标**（`serial:...`）需 `pyserial`（懒导入，缺包不崩）——
> `sim` 与 `tcp` 目标仍是纯标准库。
> 本目录原名 `examples/euler_angles`（补齐波产物，平级于 `imu/`），已归位改名
> `examples/imu/exp3_euler_view`——与固件 `exp3_euler` 名字对仗。

## 数据源：两种显式模式（无隐式回落）

用户定案（2026-11-05）：数据源只有两种**显式**模式，**禁止隐式回落**——没插设备
不会自动仿真、启动时也不会默认连 QEMU。无参启动 = 空闲（idle），只开页面，
模式由页面上的按钮显式选择。

| 模式 | 数据从哪来 | 怎么开 |
| --- | --- | --- |
| **idle**（无参） | 不启动任何数据源 | `run.bat`（只开页面，等页面点「启动仿真 / 连接真实」） |
| **sim** 仿真 | 本机合成流：镜像固件合成源预设 `sim.exp3.sway`（pitch 绕 Y 纯摆 20°/4s/100Hz）。合成值**先编码成 0x03 协议帧字节流、再走与 real 挡同一个解码器**——演示的就是协议链路本身，不是直接喂数值 | `run.bat sim`（= `--mode sim`） |
| **real** 真实 | 只连用户指定的**一个目标串**：`serial:<COMx>@<baud>`（物理串口）或 `tcp:<host>:<port>`（设备串口转 TCP / 外部桥）；连不上就一直「未就绪」，**不换目标、不回落仿真** | `run.bat real 127.0.0.1:5558`（tcp 目标）/ `run.bat real --serial COM5`（串口目标）/ 页面「连接真实」+ 目标输入框 |

### 真实模式目标（目标串）

真实模式不接受隐式目标：目标只有一种写法——**目标串**，命令行参数与页面输入框都归到它。

| 目标串 | 含义 | 缺省 |
| --- | --- | --- |
| `serial:<COMx>@<baud>` | 物理串口（pyserial，8N1） | `@baud` 可省，省则用 `--baud`（缺省 115200） |
| `tcp:<host>:<port>` | TCP 字节流（设备串口转 TCP 桥 / QEMU UART0 桥） | 只给 host 时端口 5558；host 省略 = `127.0.0.1` |

- 命令行优先级：`--serial COM5` **>** `--tcp host:port`；二者都不给 → 目标仍是
  `tcp:127.0.0.1:5558`（旧行为不变）。
- 目标串非法（拼错前缀 / 端口越界 / 波特率不是整数）→ 命令行 `ap.error`，HTTP 回 400 且档位不变。
- 两种目标共用同一条 `FrameDecoder` 解码路径，差别只在字节从哪来。
- **与 QEMU 桥的关系**：QEMU 不是第三种模式，它只是 `tcp:` 目标的一种来源——
  `run_qemu.bat` 把固件 UART0 挂成 `-serial tcp:127.0.0.1:5558`，所以本页连
  `tcp:127.0.0.1:5558` 就等于「连 QEMU 里的固件」。真板插在 COMx 上用 `serial:COMx@115200`，
  不经过 QEMU。

## 用法

| 命令 | 效果 |
| --- | --- |
| `run.bat` | 空闲：只开页面 `http://127.0.0.1:8765/`，页面上点「启动仿真」或「连接真实」才启动数据源 |
| `run.bat sim` | 仿真模式（本机合成流，镜像 `sim.exp3.sway`），也可调 `--sim-amp-deg` / `--sim-period-s` / `--sim-rate-hz` |
| `run.bat real` | 真实模式，连 `--tcp` 缺省地址 `127.0.0.1:5558`（先起 `FirmwareProject/examples/imu/exp3_euler/run_qemu.bat`，其 30 s 收帧窗口内本页显示真实固件发出的 0x03 帧） |
| `run.bat real 127.0.0.1:5999` | 真实模式，连指定 TCP 地址（设备串口转 TCP 的桥 / 外部 UART 桥） |
| `run.bat real --serial COM5` | 真实模式，连真板串口 `COM5`（115200 8N1，串口目标优先于 `--tcp`） |
| `python euler_view.py --mode real --serial COM5 --baud 9600` | 同上，显式指定波特率（`--baud` 缺省 115200，串口目标省略 `@baud` 时也用它） |
| `run.bat sim --sim-amp-deg 45` | 改合成摆幅（角度域教学：逼近 ±90° 看条形图锁边界） |
| `run.bat sim --sim-calib-s 0.5 --sim-temp-ramp-c 2` | 缩短虚拟 MCU 的调零采集窗、加大温升速率（无板演示温漂补偿：2 s 就够 3 °C 温差） |
| `run.bat --http-port 8790 --lang en` | 其它参数原样透传：改页面端口；控制台打印语言 zh/en（缺省 zh；**页面语言另有开关**，与此参数无关） |

启动后浏览器访问 `http://127.0.0.1:8765/`（`--http-port` 可改）；Ctrl+C 退出。
旧写法 `--source sim|tcp` 仍作兼容别名（`--source sim` = `--mode sim`，`--source tcp` = `--mode real`）。

## 页面：模式开关 + 目标输入 + 中英双语

- **模式分段开关（仿真 | 真实）**：只选择档位，**不启动任何数据源**；必须点
  「启动仿真 / 连接真实」（运行中点「停止仿真 / 断开」）才真正切换。
- **真实模式目标输入框**（仅真实模式显示）：值就是目标串（`serial:COM5@115200` /
  `tcp:127.0.0.1:5558`），点「连接真实」时随 `POST /api/mode?value=real&target=...` 一起发出；
  输入为空时按 placeholder 默认值（`serial:COM5@115200`）。页面加载时若本地无持久目标，
  输入框回填服务端当前目标（`/api/euler.target`）。
- **「刷新串口」按钮**：`GET /api/ports` 探测本机串口，把每条填进 `<datalist>` 供输入框联想
  （选项值 = `serial:<设备>@115200`，标签显示设备名 + 描述）；**不覆盖已输入内容**。
  pyserial 缺失时 `/api/ports` 回空数组 + `error`，页面提示后照旧可手输目标串。
- **语言按钮（EN / 中文）**：切换即时生效（所有静态文案、表格、教学说明、状态行
  走页面内联 zh/en 词典重建，无第三方 i18n 库）。
- **持久化键与 `web/assets/ui-shell.js` 同键同语义**：`localStorage["xrk.mode"]` =
  `sim|real`（缺省 `real`）、`localStorage["xrk.lang"]` = `zh|en`（缺省 `zh`）、
  `localStorage["xrk.target"]` = 目标串（缺省空，空则回填/用 placeholder 默认）。
  页面加载**只按持久值高亮模式 + 回填目标**，不发任何启动请求。
- **状态行**：未启动 / 已连接 / 未连接——由 `/api/euler` 回报的服务端真实状态驱动；
  未启动时表显 `—`。

## 零点校准（MCU 内部标定 · 三个控件 · 只读展示）

**架构定案：校准算法全部在 MCU 内部，上位机只发命令 + 读数据**（参考原型
`参考原型 MagicRobot` 的 `modules/imu/app_imu.c`）。采集、均值/拟合、去偏都在固件里，
主机**不做任何减偏**——0x02 帧的 `deb` 块是 MCU 算好的，重减就是双重补偿。

页面头部一排三个控件（zh/en 成对）：

| 控件 | 命令 | 语义 |
| --- | --- | --- |
| **读取当前值** | `GET_BIAS(0x02)` + `GET_STATUS(0x0A)` | 读 MCU 当前零偏（24B）与状态快照（20B：流/掩码/Hz/flags/bias_valid/tc_valid/kp/ki/odr） |
| **一键调零** | `CALIBRATE(0x01)` | **异步**：固件自己采 500 样本算零偏，采完才回 ACK；期间按钮置灰 + 状态行「校准中…」，ACK 后自动重读零偏与状态 |
| **温漂补偿** | `TEMP_CALIB(0x03, 秒)` | 异步温漂标定（采集期需温差 ≥ 3 °C）；完成后自动 `GET_TC_STATE(0x10)` + `GET_DRIFT(0x0D)` |
| **温漂开关** | `GET_STATUS` → `SET_FLAGS(0x09)` → `GET_TC_STATE` | 先读回完整 flags，只翻转 `TEMPCOMP_EN`(bit0) 再整字写回（不伪造 `tc_valid`） |

请求/回执链路：

```
页面按钮 → POST /api/imu/cmd {"func":"CALIBRATE","args":{…}}
        → real：CMD 帧 0x7D [MOD_IMU][FUNC][params…] 写当前 tcp/serial 链路 → 等 ACK 0x7E [MOD][FUNC][STATUS][data…]
        → sim ：同一 func/params 投进命令队列，由内置**虚拟 MCU** 在采集线程拍点上应答（同一 ACK 路径、同一解析）
        → 回执 {"code":"ok|notConnected|timeout|ackError|badFunc|badArgs", …}，HTTP 200
页面状态行按 code 翻译（执行中 / 校准中 / 回执 ok / 未连接 / 等 ACK 超时 / status≠0 / 参数不认）
```

- **sim 的虚拟 MCU 在哪**：`euler_view.py` 的 `VirtualMcu` 类（`--mode sim` 时由
  `SourceManager.set_mode` 建实例，交给 `sim_source_loop`）。它持有固件 `g_ctx` 的 IMU 子集
  （bias / bias_valid / tc_valid / sys_flags / drift / 采样温度），按 `app_imu.c` 逐条语义应答：
  GET_* 立即回数据块；`CALIBRATE` 等 `--sim-calib-s`（默认 5 s = 500 样本@100 Hz）后回 ACK 并发
  3 帧 0x02；`TEMP_CALIB` 等秒参数后按温差判定 `tc_valid`；`SET_FLAGS` 只改开关位。
  命令与回执**都走真实的帧编解码**（先编码成字节、再喂同一个 `FrameDecoder`），不是函数直调。
- **温漂演示旋钮**：`--sim-temp-ramp-c`（默认 0.1 °C/s，镜像 `sim.exp2.static`）。真实固件的温漂
  标定要求采集期温度变化 ≥ 3 °C，没有热风枪时把这调大（如 2.0 = 2 秒就 ≥ 3 °C）即可演示
  `tc_valid=1`；`--sim-temp-start-c` 改起始温度。
- **面板（只读）**：「零点校准（MCU 内部 · 只读展示）」＝六轴（ax/ay/az g，gx/gy/gz dps）× 行
  `raw` / `deb`（MCU 已减）/ `0x02 bias 块` / `GET_BIAS 回执` / `Δ = raw − deb` + 温度行；
  下方状态行显示 `bias_valid` / `tc_valid` / `温补开关` / `t_now/t_ref` / `k_g` / `apply_count`。
  未取到回执时全部显示 `—` 并给出提示行（点「读取当前值」）。
- **开关 ≠ 标定**（META-008）：`tc_valid` 只表示「温漂已标定」，`TEMPCOMP_EN` 只表示「是否应用」，
  页面分两列显示；`GET_STATUS`/`GET_TC_STATE` 各自带这两个位。
- **TEXT 日志**：页面底部展示最近 8 条 `0x7F` 文本帧，`OK calib …` 开头的行绿色加粗。
- **加载语义不变**：开页只按持久值高亮模式 + 回填目标，**不自动启动数据源、也不自动发命令**。

## 页面内容

- **数值表**：roll/pitch/yaw 三轴的 deg + rad + 本次开页峰值 |max|
- **三轴条形 canvas**：中心零点、各轴独立显示域（roll/yaw ±180°，pitch ±90°），
  pitch 条两端的**红虚线 = ±90° 万向锁边界**——全部自研绘制，无任何 JS 库

## HTTP 接口（页面与脚本共用）

| 接口 | 说明 |
| --- | --- |
| `GET /` | 本页 `index.html` |
| `GET /api/euler` | 状态快照 JSON：`mode`（`none\|sim\|real`）、`source`（旧词汇 `none\|sim\|tcp`，兼容保留）、`target`（当前目标串，如 `serial:COM5@115200`）、`connected`、`frames`、`euler_frames`、`seq_jumps`、`bad_resync`、`last_deg`、`last_rad`、`history`、`texts`、`cal`（最近一帧 0x02：`raw`/`deb`/`bias` 各 6 值 + `temp`，无则 `null`）、`cal_frames`（0x02 帧计数）、`ack_frames`（ACK 帧计数）、`imu`（MCU 回执渲染面：`bias`/`bias_valid`/`tc_valid`/`enabled`/`flags`/`drift`/`t_now`/`t_ref`/`k_g`/`apply_count`/`last_ack`）；页面 100 ms 轮询 |
| `GET/POST /api/mode?value=sim\|real\|none[&target=<目标串>]` | 显式切换数据源，回执 `{"mode":..., "connected":...}`；切 `real` 时可带目标串（不带则沿用上一次目标，目标串非法 → 400 且档位不变）；先停旧线程（Event + join<1s）再起新线程；非法 `value` → 400 |
| `GET /api/ports` | 枚举本机串口：`{"ports": [{"device": "COM5", "desc": "USB-SERIAL CH340"}]}`；pyserial 缺失 → `{"ports": [], "error": "..."}` |
| `GET/POST /api/imu/cmd` | MOD_IMU 命令（GET 手测：`?func=GET_BIAS`；POST：`{"func":"…","args":{…}}`）：real → CMD 帧 `0x7D` 下行 + 等 ACK `0x7E`；sim → 内置虚拟 MCU 应答。回执 `{"code":"ok","func":"…","data":{…}}` / `notConnected` / `timeout` / `ackError`（status≠0）/ `badFunc` / `badArgs`，一律 HTTP 200；`data` 键与 `/api/euler.imu` 同义（GET_BIAS→`bias`、GET_STATUS→`flags`/`bias_valid`/`tc_valid`/…、GET_TC_STATE→`t_now`/`t_ref`/`k_g`/`apply_count`、GET_DRIFT→`drift`） |

## 教学对照：pitch ±90° 万向节死锁（实验 3 ↔ 实验 4 的对照点）

本页三角度来自固件 `mahony_get_angles` 的 **ZYX 约定**分解（人读直观）。但当
pitch 逼近 ±90°（红虚线）时，ZYX 分解到达**万向节死锁（gimbal lock）**：roll
与 yaw 不再是两个独立自由度，只剩 (yaw−roll) 一个组合可分辨——三参数表示在此
丢一个自由度。host 侧 gcc 自检（`固件侧 testing/exp3_sim_main.c`
B 组）给出数值证据：pitch=+90° 时 (roll,yaw)=(0.10,0.60) 与 (−0.40,0.10) 两个
不同姿态映射到**同一个四元数**（分量差 ~1e-8）。

实验四（`exp4_quat`）走 `0x04` 四元数帧——4 参数表示**无奇异**，是 3D/渲染
主通道；本实验 `0x03` 是教学对照通道。两实验跑同一个 Mahony 滤波器、同一组
预设参数（`att.exp3.mahony` = `att.exp4.mahony`），唯一变量就是姿态表示法。

## 数据契约

- 帧型 `0x03`（`PROTO_TYPE_EULER`）：payload = roll,pitch,yaw 3×float32 LE = 12B，
  单位**弧度**，ZYX 约定——规格单源：`固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md）`
  与 `公开协议 xiao_robot_kit_qemu docs/protocol-spec.md` 类型表
- 帧型 `0x02`（`PROTO_TYPE_IMU_CAL`）：payload = `raw6 | deb6 | bias6 | temp` = 19×float32 LE = 76B
  （块序 ax,ay,az g ｜ gx,gy,gz dps；`deb` 已由固件去偏）——本页只**保真透传**，不重算
- 帧型 `0x7F`（`PROTO_TYPE_TEXT`）：ASCII 行；标定回执固定为
  `OK calib start N=500` / `OK calib progress k/500` / `OK calib done bias ax=… temp=…`，
  完成后连发 3 帧 `0x02`，标定窗（5 s）内不出 `0x03`
- 下行命令：CMD 帧 `0x7D`，payload = `MOD_ID | FUNC_ID | params…`（本页只用 `MOD_IMU=0x01`）；
  回执 ACK 帧 `0x7E`，payload = `MOD_ID | FUNC_ID | STATUS | data…`（`STATUS`: 0=OK / 1=参数错 /
  2=执行失败 / 0xFF=未知功能码）。FUNC 码与原型 `modules/imu/app_imu.c` 一致：
  `0x01` CALIBRATE / `0x02` GET_BIAS / `0x03` TEMP_CALIB(秒) / `0x09` SET_FLAGS(flags u16 LE) /
  `0x0A` GET_STATUS / `0x0D` GET_DRIFT / `0x10` GET_TC_STATE；零偏块口径 = `imu_bias_t`
  （**gx,gy,gz,ax,ay,az**，与 0x02 帧的 ax..gz 块序不同，页面按列名映射）。
  注：`0x7D` 在 `固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md）` 尚未登记（现只有 `0x7E` ACK），
  本页先按定案契约实现，登记后此处对齐（INT-2）。
- 帧格式：`SOF 0xA5 | LEN | TYPE | SEQ | PAYLOAD | CRC8(0x07, 覆盖 LEN..PAYLOAD)`

## 前置

- Python 3.8+（`sim` 与 `tcp` 目标标准库即可，无 pip 依赖）
- `serial:` 目标需要 `pyserial`（`pip install pyserial`）：懒导入——缺包时控制台打一行
  「串口不可用：需要 pyserial」并保持未连接，**不崩、不回落仿真**
- tcp 目标需要目标先就绪（QEMU 侧先起 `run_qemu.bat`，端口默认 5558；或串口转 TCP 桥）；
  serial 目标需要真板插在 `COMx` 上且串口未被占用（115200 8N1）
- 回归测试：`python tests/test_exp3_mode.py`（模式解析 / 目标串解析 / 页面契约 / HTTP 显式切换 / real 未就绪 / 串口目标 / 端口枚举 / MCU 校准命令：CMD·ACK 编解码、虚拟 MCU、三个控件链路、假 MCU 端到端）
