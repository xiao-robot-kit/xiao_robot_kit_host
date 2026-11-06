# exp_motor_scurve — S 梯度轨迹实验（Host 上位机）

> 双槽（PWM_OPEN + ENC_CLOSED）七相位 S 梯度目标位置扫描：曲线速度输出前馈
> 为电机设定，腿走完停驻收敛后 rewind 循环；0x05 帧逐拍出
> setpoint（速度目标）/duty（占空回读）/actual（速度实测）/phase（七相位，
> 7=DONE 停驻）。形状断言：加速段 duty 单调升、匀速段平稳（波动<阈值）、
> 减速段单调降、到目标收敛（±2%）。

## 对应矩阵（一实验一上位机，用户定案结构）

| 侧 | 位置（同名工程） | 入口 |
| --- | --- | --- |
| 固件（Embedded） | `FirmwareProject/examples/motor/exp_motor_scurve/` | `run_qemu.bat`（双宏编译 → 起 QEMU：串口桥 tcp:5576 → probe 0x05 断言 → 必杀） |
| 上位机（Host） | 本文件夹 `Host_Project/examples/motor/exp_motor_scurve/` | `run.bat qemu [port]`（显式 QEMU 档：透传 Embedded 侧全链验收，端口 5576 配对；**无参只打印用法，不自动起 QEMU**） |

同构六兄弟：`exp_motor_pwm` / `exp_motor_dc` / `exp_motor_servo` /
`exp_motor_enc` / `exp_motor_nested` / 本工程（Host 六文件夹 ↔ Embedded
六同名工程；五兄弟走 scope_motor 场景模式 5560~5564，本工程走 probe 0x05
——见下）。

## 用法

```
run.bat qemu [port] [seconds] （显式 QEMU 档；缺省 5576 / 16s；无参只打印用法）
  = cd FirmwareProject\examples\motor\exp_motor_scurve → run_qemu.bat
    （激活 IDF → build-qemu 双宏编译 → 起 QEMU esp32s3 → probe 收 0x05 帧
    断言 → 必杀 QEMU；全链一键，无需手动起对端）
```

## 为什么不走 scope_motor（如实注明，任务卡口径）

`backend/scope_motor.py` **无 scurve 场景**：`--scenario` 取值仅
`{pwm,servo,enc,nested,dc}`（五兄弟对应 exp_motor_qemu 场景帧 0x06）；其
demo 模式（0x05）按 4 槽 64B payload 解码（`MOTOR_PAYLOAD_LEN = 4×16B`），
而本实验固件出 2 槽 32B 记录——demo 断言路径不适用。故本实验的无板断言走
**QEMU probe**：`FirmwareProject/testing/qemu/probe_sim.py --expect-type 0x05`
（退出码 0 = 收到 0x05 帧流；由 `run.bat` 一键执行）。若后续需要示波器化
观测面，须扩展 scope_motor 的 scurve 场景（decode 2×16B + 相位分段形状断言）
——登记为可选扩展，不在本实验交付面。

## 验收判据（分两级，如实）

- **无板链路（本入口，run.bat）**：QEMU 起 + 0x05 帧流收帧（probe 退出码 0
  = PASS；含 CRC/SEQ 健康统计）。
- **形状与收敛断言（host gcc 自检，Embedded 侧）**：
  `bash FirmwareProject/examples/motor/exp_motor_scurve/testing/check_sim.sh`
  ——每完整腿加速段 duty 单调升（ENC 槽 50ms 平滑信号）/匀速段平稳
  （PWM 匀速占空 0.6000..0.6000；ENC 平滑波动 <0.05、速度均值 72.0 rpm）/
  减速段单调降/到目标收敛 ±2%（实测双槽 0.00%：PWM 积分位置 2.0000 /
  ENC 编码器位置 5.0000）+ 0x05 帧 37B 字节级回读。

## 前置

- ESP-IDF v6.1（run_qemu.bat 自动激活）；QEMU 已装
  （`D:\Espressif\tools\qemu-xtensa`）；Python 3.x（probe 纯标准库）
- 电机库 CoreLib `components/motor`（源仓 dev 检出，interim 同
  exp_motor_qemu）

## 入口链路

`run.bat` → Embedded 侧 `run_qemu.bat` → `idf.py -B build-qemu
-DEXP_SIM=1 -DEXP_QEMU_UART=1 build`（GOV-ERR-004 双宏铁律）→ QEMU esp32s3
（-serial tcp:5576，第 1 槽 = UART0 帧出口）→ `run_exp_sim.sh` →
`probe_sim.py --expect-type 0x05`（16s 收帧统计，退出码即判据）→ taskkill
必杀 QEMU。
