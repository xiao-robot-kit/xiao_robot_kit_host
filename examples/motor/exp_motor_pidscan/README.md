# exp_motor_pidscan — PID 参数网格扫描实验（Host 上位机）

> 0x05 遥测帧（slot=扫描组号）+ 0x7F TEXT 汇总帧：同一台 N20 带编码器电机
> （合成 plant 160 rpm/duty、τ=20ms）轮转 9 组速度环 PI 参数
> （kp×ki 3×3 网格：kp {0.006, 0.010, 0.015} × ki {1.0, 1.5, 2.0}，底版
> `motor.enc.n20` 整定 0.015/1.0 为「快稳」角点）——每组 30→90 rpm 阶跃
> （低段 1s 前导 + 高段 2s 统计窗），记录收敛时间/超调%/稳态误差%，
> 27s 一轮、无限循环重扫。
> **调参对比表的查看方式** = 本入口 `run.bat`：终端双表呈现——固件 TEXT
> 汇总表（轮末 12 行）+ host 独立重算表（0x05 轨迹自算三指标），最优组
> `'*'` 标注，两表数值断言一致（`backend/scope_pidscan.py`，纯标准库）。

## 对应矩阵（一实验一上位机，用户定案结构）

| 侧 | 位置（同名工程） | 入口 |
| --- | --- | --- |
| 固件（Embedded） | `FirmwareProject/examples/motor/exp_motor_pidscan/` | `run_qemu.bat 5565 35`（起 QEMU：串口桥 tcp:5565 wait 模式） |
| 上位机（Host） | 本文件夹 `Host_Project/examples/motor/exp_motor_pidscan/` | `run.bat`（缺省 sim 零依赖双表断言；`run.bat qemu` 连 QEMU；`run.bat COM5` 连真机） |

同构五兄弟 + 本实验：`exp_motor_pwm` / `exp_motor_dc` / `exp_motor_servo` /
`exp_motor_enc` / `exp_motor_nested`（Host ↔ Embedded 同名工程，端口
5560~5564 一一配对）；pidscan 为第六实验，端口 5565。

## 用法（数据源三档显式：--source sim|qemu|real）

> 不再「没接设备却自动连 QEMU」：缺省 sim（本机合成帧源，零外部依赖、
> 不乱连任何人）；要真数据必须显式点名 qemu 或 real。

| 命令 | 等价于 | 说明 |
| --- | --- | --- |
| `run.bat` | `--source sim --seconds 75` | 本机合成源：按帧计数推进（不等墙钟），虚拟 27s/轮；不连 QEMU、不碰串口 |
| `run.bat qemu` | `--source qemu --port 5565 --seconds 75` | TCP 串口桥（QEMU wait 模式）；连不上 ≤5s 明报 `FAIL 连接…` 并 exit 2 |
| `run.bat qemu 5565` | `--source qemu --port 5565 --seconds 75` | 同上，端口可指定 |
| `run.bat COM5` | `--source real --serial COM5 --baud 115200 --seconds 75` | 真串口（115200 8N1，pyserial）；解码路径与 qemu 档同一套 |

命令行参数面见 `python backend/scope_pidscan.py --help`
（`--source/--lang/--host/--port/--serial/--baud/--seconds/--quiet`）。

### 输出语言（--lang zh|en，缺省 zh）

面向用户的打印行（模式/连接/断言小标题/PASS-FAIL 汇总/错误原因）中英成对；
表格列名与固件 TEXT 原文保留符号。要英文输出（`run.bat` 不传语言，直接调工具）：

```
python backend\scope_pidscan.py --source sim --lang en
python backend\scope_pidscan.py --source qemu --port 5565 --lang en
```

### 各档适用场景

- **sim（缺省）**：没起 QEMU、手边没板子时验上位机侧链路（解码 → 双表 → 断言）
  与回归；合成流按帧计数确定生成（同参数两遍逐字节相同）。注意：sim 的 TEXT 表
  由同一份合成轨迹按固件口径算出，**验的是解码/重算口径一致**，不替代 QEMU/真机
  的独立实现对照。
- **qemu**：须先起 QEMU 对端——Embedded 侧同名工程 `run_qemu.bat 5565 35`
  （或另开窗口同时跑；wait 模式，本工具连上即开机收流）。
- **real**：真板串口（USB-TTL，COM5 之类），固件上电即 500Hz 连续吐流；
  需 `pip install pyserial`，缺包/打不开串口均 exit 2。

## 调参对比表的查看方式（本实验的核心输出面）

1. **固件 TEXT 汇总表**（0x7F 帧逐行，终端 `[text]` 前缀实时打印）：轮首
   表头 + 9 组行（`G<k> kp<v> ki<v> st<v>s ov<v>% se<v>%`，最优组行尾
   `*`）+ `BEST G<k> score…` + `PIDSCAN round done`；
2. **host 独立重算表**（收满后统一打印）：从 0x05 轨迹按组号分桶，高段
   统计窗独立重算三指标（与固件同口径、独立算法——防同源盲区），并打印
   host 侧最优组；
3. **双表断言**：两组数值逐一对照（容差内）、`'*'` 标注恰 1 行且与 host
   最优一致。

## 验收判据（scope_pidscan.py 断言集；exit 0=PASS）

- **每组收敛**：高段末半段均值 = 90±3%（任务卡：终值 ±3%）
- **链路健康**：0x05 帧 ≥9000（一轮 13500）、坏重同步 ≤50、9 组号全覆盖
- **对比表完整**：固件组行 ≥9、`'*'` 标注恰 1、round done 行出现
- **双表一致**：固件 TEXT 三指标 vs host 独立重算逐组对照

## 前置

- Python 3.x（纯标准库零依赖；sim / qemu 档不需要任何第三方包）
- qemu 档：QEMU 已装（`D:\Espressif\tools\qemu-xtensa`）；固件镜像由 Embedded
  同名工程 `run_qemu.bat` 现场构建（build-qemu 目录，双宏
  `-DEXP_SIM=1 -DEXP_QEMU_UART=1`，GOV-ERR-004 铁律）
- real 档：`pip install pyserial` + 真板串口（115200 8N1）

## 入口链路

`run.bat` →（无参 = sim / `qemu [port]` = qemu / `COM5` = real）
`backend/scope_pidscan.py --source <档位>` → 选源（sim 本机合成 / TCP wait 握手 /
串口打开）→ 0x05/TEXT 帧流采集（round done 后 1s 排干即收工）→ 双表打印 +
断言（exit 0=PASS / 1=断言 FAIL / 2=选源或连接失败）。
