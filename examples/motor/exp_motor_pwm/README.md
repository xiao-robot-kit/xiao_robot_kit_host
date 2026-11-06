# exp_motor_pwm — PWM 占空阶梯实验（Host 上位机）

> 0x06 场景帧 SCEN 1（pwm）：M0（`motor.pwm.generic`，20kHz 定时器）占空阶梯
> 0/25/50/75/100%、每档 2s（10s 一循环）；duty = sim PWM 合成存储回读 = 物理输出面。
> 纯标准库 ASCII 示波器 + 阶梯形状数值断言（`backend/scope_motor.py` 场景模式）。

## 对应矩阵（一实验一上位机，用户定案结构）

| 侧 | 位置（同名工程） | 入口 |
| --- | --- | --- |
| 固件（Embedded） | `FirmwareProject/examples/motor/exp_motor_pwm/` | `run_qemu.bat`（起 QEMU：串口桥 tcp:5560 wait 模式） |
| 上位机（Host） | 本文件夹 `Host_Project/examples/motor/exp_motor_pwm/` | `run.bat`（示波器 + 阶梯断言） |

同构五兄弟：`exp_motor_pwm` / `exp_motor_dc` / `exp_motor_servo` / `exp_motor_enc` / `exp_motor_nested`
（Host 五文件夹 ↔ Embedded 五同名工程，端口 5560~5564 一一配对）。

## 数据源（`--source`，显式三档；缺省 `sim` = 零外部依赖）

| 档 | 本文件夹命令 | 说明 |
| --- | --- | --- |
| `sim` | `run.bat` | 本机合成帧源 `backend/sim_motor.py`：不读 socket、不碰串口，帧按**帧计数**确定性推进（2× 仿真时间压缩：30s 采集 ≈ 60s 场景时长），无设备也能跑完整断言 |
| `qemu` | `run.bat qemu [port]` | 连 QEMU esp32s3 串口 TCP 桥（wait 模式：连接即开机；缺省端口 5560） |
| `real` | `run.bat COM5` | 连真实串口（pyserial，115200 8N1；`COMx` 单参数即本档） |

- **不再默认连 QEMU**：无参 = `sim`，连接失败场景不会干等；`qemu`/`real` 连不上时
  **≤5s 内明确 FAIL 并 exit 2**（旧版静默重试 15s 已取消）。
- 输出语言：`--lang {zh,en}`（缺省 `zh`）；所有面向用户的打印行走中英文案表（成对，缺一即 bug）。
- 三档共用同一解码与断言路径——换源不换判据。

## 用法

```
run.bat                    = cd Host_Project 根 → --source sim --scenario pwm --seconds 30
run.bat qemu 5560          = --source qemu --scenario pwm --port 5560 --seconds 30
run.bat COM5               = --source real --scenario pwm --serial COM5 --baud 115200 --seconds 30
```

`qemu` 档须先起 QEMU 对端（wait 模式：示波器连接触发开机）——Embedded 侧同名工程
`run_qemu.bat`，或另开窗口同时跑。命令行参数面见 `python backend/scope_motor.py --help`
（`--source/--lang/--host/--port/--serial/--baud/--seconds/--scenario/--render-every/--quiet/--no-plot`；
`--scenario` 取值 `{dc,enc,nested,pwm,servo}`，pwm 场景缺省时长 26s，本入口统一 30s——
10s 一循环采满 3 循环；`sim` 档 2× 压缩后 30s 覆盖约 60s 场景时长）。

## 验收判据（`--scenario pwm` 断言集；exit 0=PASS，三档同一套）

- **阶梯跟随**：各档 duty 均值 = 档位值 ±0.02（±2% 满量程）
- **档内平稳**：各档 duty 波动（max−min）≤ 0.01
- **五档全覆盖**：0/25/50/75/100% 平台均出现（±0.01 判同）+ 完整升序阶梯序列可见
- **duty 界内**：全程 [0,1]（±0.002 容差）
- **链路健康**：≥2000 帧、坏 CRC ≤50、SEQ 丢帧 ≤1%、故障帧零容忍、
  切换后无残余 0x05 demo 帧、定时器快照 = 20kHz 全帧一致

## 前置

- Python 3.x（纯标准库零依赖）——**`sim` 档只需这一项**
- `qemu` 档：QEMU 已装（`D:\Espressif\tools\qemu-xtensa`）+ 电机镜像
  `FirmwareProject/testing/qemu/motor_img/exp_motor_flash.bin` + `exp_motor_efuse.bin`
  （Embedded 侧构建产出，见 Embedded 同名工程说明）
- `real` 档：`pip install pyserial` + 已烧录同场景固件的真机（缺失 pyserial 即 exit 2）

## 入口链路

`run.bat` → `backend/scope_motor.py --source sim --scenario pwm --seconds 30`
→ **sim 档**本机合成帧直接解码；**qemu/real 档**激活握手（先见 app 帧再注入「SCEN 1」——
ROM 启动期注入会卡死 boot，实测）后收 0x06 帧流
→ 阶梯断言（exit 0=PASS / 1=FAIL / 2=连接失败或参数缺失）。
