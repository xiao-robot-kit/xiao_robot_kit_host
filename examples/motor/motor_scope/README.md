# motor_scope — 电机示波器示例

> 电机遥测（0x05 帧，四槽四类齐：PWM_OPEN/SERVO/ENC_CLOSED/DC）ASCII 示波器 +
> S 梯度形状数值断言；场景模式（0x06 帧）五场景断言（纯标准库零依赖）。

## 数据源（`--source`，显式三档；缺省 `sim` = 零外部依赖）

| 档 | 本文件夹命令 | 说明 |
| --- | --- | --- |
| `sim` | `run.bat` | 本机合成帧源 `backend/sim_motor.py`：不读 socket、不碰串口，帧按**帧计数**确定性推进（2× 仿真时间压缩：30s 采集 ≈ 60s 场景时长），四槽 S 梯度 + dwell 停驻按帧推进 |
| `qemu` | `run.bat qemu [port]` | 连 QEMU esp32s3 串口 TCP 桥（wait 模式：连接即开机；缺省端口 5558） |
| `real` | `run.bat COM5` | 连真实串口（pyserial，115200 8N1；`COMx` 单参数即本档） |

- **不再默认连 QEMU**：无参 = `sim`；`qemu`/`real` 连不上时 **≤5s 内明确 FAIL 并 exit 2**
  （旧版静默重试 15s 已取消）；`real` 缺 pyserial 同样 exit 2。
- 输出语言：`--lang {zh,en}`（缺省 `zh`）；所有面向用户的打印行走中英文案表（成对，缺一即 bug）。
- 三档共用同一解码与断言路径——换源不换判据。

## 用法

**无参（本机合成帧，零外部依赖）**：

```
run.bat            = cd Host_Project 根 → --source sim --seconds 30（demo 模式：0x05 四槽 + S 梯度断言）
```

**QEMU 对端**：在 `FirmwareProject/examples/motor/exp_motor_qemu/` 下执行 `run_qemu.bat`
（起 QEMU → 示波器 30s → 自动杀），本文件夹 `run.bat qemu` 亦可（须先起对端或另开窗口）。

```
run.bat qemu              = 连 QEMU 5558 收 0x05 帧
run.bat qemu 5565         = 连 QEMU 5565（自定义端口）
run.bat COM5              = 连真实串口（115200）
```

**场景模式（0x06，含 DC 正反转）**：

```
python backend/scope_motor.py --source sim --scenario dc --seconds 10
python backend/scope_motor.py --source qemu --scenario dc --port 5565 --seconds 26
```

（`--scenario {pwm,servo,enc,nested,dc}`；`qemu` 档须先起 QEMU 对端——一键走
`FirmwareProject/testing/qemu/run_exp_motor_dc.sh`；DC 断言 = 方向位翻转 + duty 对称 +
逐帧方向同号，duty 显示为方向桥带符号回读，滚动行带 `dir+FWD/dir-REV` 方向位。
`sim` 档 2× 时间压缩：`--seconds 5` 已覆盖 10s 场景时长。）

命令行参数面见 `python backend/scope_motor.py --help`
（`--source/--lang/--host/--port/--serial/--baud/--seconds/--scenario/--render-every/--quiet/--no-plot`）。

## 验收判据（exit 0=PASS，三档同一套）

- **demo 断言（0x05）**：帧数下限 / 坏 CRC / SEQ 丢帧 / 故障位零容忍；开环槽每腿 setpoint
  单调 + S 形无回摆 + 停驻段稳定；闭环槽 duty 界内 + 停驻段与末段收敛；每槽 ≥2 腿 ≥2 停驻段
- **场景断言（0x06）**：pwm 阶梯 ±2% 跟随、servo 五点脉宽 ±25us/回读 ±0.5°、enc 段收敛 ±3%、
  nested 通道隔离 + 20kHz/50Hz 并存、dc 方向位逐帧一致 + duty 对称（详见各实验 README）

## 前置

- Python 3.x（仅标准库）——**`sim` 档只需这一项**
- `qemu` 档：QEMU 已装（`D:\Espressif\tools\qemu-xtensa`）+ exp_motor_qemu 固件已编译
  （Embedded 侧 run_qemu.bat 自动处理）
- `real` 档：`pip install pyserial` + 真机

## 入口链路

`run.bat` → `backend/scope_motor.py --source sim`（本机合成帧）或 `--source qemu`（socket 5558）
或 `--source real --serial COMx` → ASCII 波形 + 滚动数值 + 断言（exit 0=PASS / 1=FAIL /
2=连接失败或参数缺失）。
