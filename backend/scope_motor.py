#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scope_motor.py — 电机 QEMU 演示的模拟示波器（T-102e/T-112/T-113，纯标准库零依赖）。

意图：从**显式声明的数据源**（--source sim/qemu/real，缺省 sim = 本机合成帧、
零外部依赖）取 0x05/0x06 电机遥测帧，解码后终端 ASCII 波形 + 滚动数值表渲染
duty/速度轨迹，并对轨迹形状做数值断言。三档源共用同一解码/断言路径：
    sim    本机合成帧源（backend/sim_motor.py，帧计数确定性推进，不读
           socket、不碰串口；2× 仿真时间压缩）；
    qemu   QEMU esp32s3 串口 TCP 桥（`-serial tcp:127.0.0.1:5558,server`，
           wait 模式——本工具连上才开机，帧流零丢失）；
    real   真实串口（pyserial，--serial COMx --baud 115200）。

语言：--lang {zh,en}（缺省 zh）——所有面向用户的打印行走下方 TEXT 文案表
（键值 zh/en 成对，缺一即 bug，见 text_table_errors）。

帧规格（单源 = 固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md），PAYLOAD_MAX=96；
注意 backend/frame.py 的 64 是 exp1 期旧值，本工具自带解码不依赖它——
零第三方依赖 = 只用 Python 标准库）：

    SOF(0xA5) LEN TYPE SEQ PAYLOAD(LEN) CRC8(覆盖 LEN..PAYLOAD, poly 0x07)

    TYPE=0x05 MOTOR（demo，T-102e 3 槽 → T-113 扩 4 槽四类齐）:
        payload = 4 x 16B 记录（小端）：
        u8 slot | u8 kind | f32 setpoint | f32 duty | f32 actual | u8 phase | u8 fault
    slot 0/1/2/3 = PWM_OPEN(duty 域) / SERVO(deg 域) / ENC_CLOSED(rpm 域) /
    DC(带符号 duty 域 −1..+1，符号 = H 桥方向位——3→4 槽扩展，记录布局不变)；
    phase = scurve_phase_t（0..6 运动，7=DONE 停驻）。

    TYPE=0x06 SCEN（T-112 四专项 + T-113 dc 场景帧）: payload = 场景头 + 定时器快照 + 记录：
        u8 scen_id | u8 n_tim | n_tim x {u8 idx, u8 channels, u32 freq_hz}
        | u8 n_rec | n_rec x 16B（记录布局与 0x05 逐字节同构）
    scen_id 1..5 = pwm/servo/enc/nested/dc（槽表单源 = 固件 motor_scen_table.c）；
    duty = sim PWM 合成存储回读（物理输出面；DC 槽为方向桥带符号回读，
    符号 = 方向位）；phase = 步进下标。

demo 断言（0x05，任务卡：加速段占空单调升、匀速段稳定、末段收敛）——按
phase 字段分段逐腿验证：
    A. 链路健康：帧数下限、坏 CRC 上限、SEQ 缺帧率上限、故障位零容忍；
    B. 开环槽（0/1/3）：每条运动腿 setpoint 单调（升腿不减/降腿不增），
       占空/角度呈 S 形无回摆；每个停驻段（phase=7 连续段）setpoint 稳定；
       （槽3 DC 为带符号 duty 域：duty==setpoint 同判，正反转腿交替可见）
    C. 闭环槽（2）：duty 全程界内 [0,1]；停驻段 actual 收敛于 setpoint
       （PI 残差容差 8%）；末段收敛 = 最后一个停驻段满足同一判据；
    D. 循环性：每槽 ≥2 条运动腿 + ≥2 个停驻段（retarget 交替腿在工作）。

场景断言（--scenario，0x06，T-112 四专项 + T-113 DC 任务卡口径；期望值独立于固件表）：
    激活握手：等首帧 app 帧出现（ROM 启动期注入命令会卡死 boot——实测），
    0.5s 重发「SCEN <n>」直到 0x06 scen_id 匹配，排干在途重复命令后采集。
    pwm    占空阶梯 0/25/50/75/100%：各档 duty 均值跟随 ±0.02（±2%），
           档内平稳，五档全覆盖 + 完整升序阶梯可见；
    servo  五点标定 0/45/90/135/180°：各点脉宽 = 线性插值 ±25us（SG90
           500~2500us/0~180° 镜像），回读角度一致 ±0.5°，五点全覆盖；
    enc    30→90 rpm 阶跃：各段末半段均值收敛 ±3%；升沿 rise(10%→90%) /
           峰值 / 超调% 如实记录（无硬阈值）；30 与 90 平台均出现；
    nested 同定时器 A(ch0)/B(ch1) + 独立定时器 C(50Hz 舵机)：A 阶梯跟随，
           B 全程恒 0.5（通道隔离——A 步进期间 B 不动，duty 为存储回读），
           C 舵机映射（频率并存可观测面），定时器快照恰为 20kHz+50Hz 且
           全帧一致；
    dc     H 桥正反转切换 +50%↔−50%（各 2s 循环）：逐帧方向位与步进一致
           （sign(duty)==sign(setpoint)）；正/反段各 ≥2 次（方向位翻转循环）；
           duty 对称 = 正反段 |duty| 均值都 ≈0.5（±2%）且互差 ≤2%；
           |duty| ≤ 1 界内。

连接失败（qemu/real）：≤5s 内判 FAIL + exit 2（不再干等 15s）；real 档 pyserial
缺失同样 exit 2。sim 档不建任何连接，连接路径零外部依赖。

用法：
    python scope_motor.py [--source {sim,qemu,real}] [--lang {zh,en}]
                          [--host 127.0.0.1] [--port 5558]
                          [--serial COMx] [--baud 115200]
                          [--seconds 30] [--render-every 125] [--quiet]
                          [--no-plot] [--scenario {dc,enc,nested,pwm,servo}]
    （--scenario 时 --seconds 缺省取各场景推荐时长：pwm/nested/dc 26s，
     servo/enc 34s；不带 --scenario = demo 模式，4 槽四类齐。sim 档 2× 仿真
     时间压缩：--seconds 10 即 20s 场景时长，覆盖最长 16s 场景循环）
退出码：0 = 全部断言通过；1 = 有 FAIL（明细打印）；2 = 连接失败/参数缺失。
"""

import argparse
import socket
import string
import struct
import sys
import time

import sim_motor

# ── 帧契约常量（与 proto/frame.h 同源；见模块 docstring） ──
SOF = 0xA5
TYPE_MOTOR = 0x05
TYPE_SCEN = 0x06
TYPE_TEXT = 0x7F
PAYLOAD_MAX = 96
MOTOR_SLOTS = 4
MOTOR_REC = struct.Struct("<BBfffBB")   # 16B：slot,kind,setpoint,duty,actual,phase,fault
MOTOR_PAYLOAD_LEN = MOTOR_SLOTS * MOTOR_REC.size   # 64B（3→4 槽扩展，T-113）
SCEN_TIMER_REC = struct.Struct("<BBI")  # 6B：tim_idx,channels,freq_hz（0x06 快照）

# 槽身份（与固件 motor_demo_table.c 契约一致；kind 值域 = mtr/motor_types.h）
SLOT_NAME = ("PWM_OPEN", "SERVO", "ENC_CLOSED", "DC")
SLOT_KIND = (0, 2, 3, 1)
PHASE_DONE = 7

# ── demo 断言阈值（表驱动，改阈值只动这里；元组按槽序索引 0..3） ──
CHECKS = {
    "min_frames": 500,        # 30s @ 500Hz 应 ~15000 帧；500 = 桥降级仍可判的下限
    "max_bad_crc": 50,        # ROM 横幅噪声重同步余量
    "max_seq_gap_pct": 1.0,   # 丢帧率上限（%）
    "max_fault_frames_pct": 0.1,
    "min_legs": 2,            # 每槽运动腿数下限（retarget 交替在工作）
    "min_plates": 2,          # 每槽停驻段数下限
    "min_plate_len": 100,     # 停驻段纳入判定的最小样本数（0.2s）
    # 单调容差按槽域：slot0 duty / slot1 deg / slot2 rpm（单调判不覆盖）/ slot3 带符号 duty
    "mono_tol": (1e-4, 0.05, 8.0, 1e-4),
    "plate_sp_dev_pct": 0.5,  # 停驻段 setpoint 偏差上限（% of |中位数|，加绝对底）
    # 绝对底按槽域：duty / deg / rpm / 带符号 duty
    "plate_sp_dev_abs": (2e-3, 0.2, 0.2, 2e-3),
    "s2_duty_max": 1.0,
    "s2_track_pct": 8.0,      # 闭环停驻段 |actual-setpoint| 上限（% of setpoint，PI 残差）
    "s2_track_abs": 2.0,      # rpm 绝对底（0 目标停驻段：纯相对容差会塌缩到 0）
}

# ── 场景断言阈值（T-112 任务卡口径；表驱动） ──
SCEN_CHECKS = {
    "min_frames": 2000,       # 26~34s @ ~494fps ≈ 13k~17k 帧；2000 = 降级下限
    "max_bad_crc": 50,
    "max_seq_gap_pct": 1.0,
    "min_plate_len": 300,     # 台阶纳入判定的最小样本数（~0.6s）
    "duty_follow_tol": 0.02,  # 阶梯档跟随容差（±2% 占空满量程，任务卡）
    "duty_flat_tol": 0.01,    # 档内平稳（max-min）
    "servo_us_tol": 25.0,     # 脉宽标定容差（us；线性插值 ±容差）
    "servo_rb_deg_tol": 0.5,  # 回读角度一致容差（deg）
    "enc_settle_pct": 3.0,    # 闭环段收敛（末半段均值 ±3%，任务卡）
}

# SG90 标定 oracle（镜像固件预设 motor.servo.sg90：500~2500us / 0~180°/50Hz）
SG90 = {"pmin_us": 500.0, "pmax_us": 2500.0, "amin": 0.0, "amax": 180.0}

# ── 场景表（镜像固件 motor_scen_table.c：id/槽/步进/定时器；单源在固件侧） ──
# 步表按槽存放（每槽一套步进序列，镜像固件每槽 s_steps_*；同场景各槽可不同步长/值）。
SCEN = {
    "pwm": {
        "id": 1, "secs": 26.0,
        "slots": [{"name": "M0", "domain": "duty", "full": 1.0, "kind": 0,
                   "steps": [0.0, 0.25, 0.5, 0.75, 1.0]}],
        "timers": [(0, 4, 20000)],
    },
    "servo": {
        "id": 2, "secs": 34.0,
        "slots": [{"name": "S0", "domain": "deg", "full": 180.0, "kind": 2,
                   "steps": [0.0, 45.0, 90.0, 135.0, 180.0, 135.0, 90.0, 45.0]}],
        "timers": [(0, 4, 50)],
    },
    "enc": {
        "id": 3, "secs": 34.0,
        "slots": [{"name": "E0", "domain": "rpm", "full": 160.0, "kind": 3,
                   "steps": [30.0, 90.0, 30.0, 90.0]}],
        "timers": [(0, 4, 20000)],
    },
    "nested": {
        "id": 4, "secs": 26.0,
        "slots": [
            {"name": "A", "domain": "duty", "full": 1.0, "kind": 0,
             "steps": [0.0, 0.25, 0.5, 0.75, 1.0]},
            {"name": "B", "domain": "duty", "full": 1.0, "kind": 0,
             "steps": [0.5]},                    # 恒 0.5（通道隔离的「静方」）
            {"name": "C", "domain": "deg", "full": 180.0, "kind": 2,
             "steps": [0.0, 90.0, 180.0, 90.0]},  # 慢角度摆动（独立 50Hz 定时器）
        ],
        "timers": [(0, 4, 20000), (1, 4, 50)],
    },
    "dc": {
        "id": 5, "secs": 26.0,
        "slots": [{"name": "D0", "domain": "sduty", "full": 1.0, "lo": -1.0,
                   "kind": 1, "steps": [0.5, -0.5]}],
        "timers": [(0, 4, 20000)],
    },
}

# 槽域缩放（终段 ASCII 波形纵轴：满量程 + 下限——与演示表目标同量级；
# slot3 DC 为带符号 duty 域 −1..+1，下限 −1 才不把反转腿裁到零线）
PLOT_FULLSCALE = (1.0, 180.0, 160.0, 1.0)
PLOT_LO = (0.0, 0.0, 0.0, -1.0)

# ── 连接预算与串口缺省（三档源共用） ──
CONNECT_TIMEOUT_S = 3.5     # qemu/real 连接预算：到点即 FAIL + exit 2（任务卡 ≤ 5s）
CONNECT_ATTEMPT_S = 0.5     # 单次 TCP 连接超时（s）：Windows 上被拒端口会吃满超时，必须短
CONNECT_RETRY_S = 0.1       # 连接重试间隔（s）
SOCKET_TIMEOUT_S = 0.5      # 建流后单次读超时（s）
SERIAL_TIMEOUT_S = 0.2      # 串口读超时（s）：短超时保证采集窗与握手窗的截止时刻准时
DEFAULT_BAUD = 115200

# ── 语言与文案表（--lang 切档；zh/en 必须成对——缺一即 bug，见 text_table_errors） ──
LANG = "zh"

TEXT = {
    # banner / 模式 / 连接
    "banner.title": ("电机示波器 scope_motor — 数据源 {source}｜语言 {lang}",
                     "motor scope — source {source} | lang {lang}"),
    "mode.sim": ("数据源 sim：本机合成帧源（不读 socket、不碰串口；帧计数推进，{speed:g}× 仿真时间压缩）",
                 "source sim: built-in synthetic frames (no socket, no serial; frame-count driven, {speed:g}x sim-time compression)"),
    "mode.qemu": ("数据源 qemu：QEMU 串口 TCP 桥 {host}:{port}（wait 模式：连接即开机）",
                  "source qemu: QEMU serial TCP bridge {host}:{port} (wait mode: connect boots the target)"),
    "mode.real": ("数据源 real：真实串口 {port} @ {baud} baud（pyserial）",
                  "source real: serial port {port} @ {baud} baud (pyserial)"),
    "conn.ok": ("已连接：{desc}", "connected: {desc}"),
    "fail.conn_qemu": ("FAIL connect：{host}:{port} 在 {timeout:.1f}s 内无可连的 QEMU 串口桥（先起 Embedded 侧 run_qemu.bat，或改 --source sim/real）",
                       "FAIL connect: no QEMU serial bridge at {host}:{port} within {timeout:.1f}s (start the Embedded run_qemu.bat first, or use --source sim/real)"),
    "fail.conn_pyserial": ("FAIL connect：--source real 需要 pyserial（pip install pyserial）",
                           "FAIL connect: --source real requires pyserial (pip install pyserial)"),
    "fail.conn_serial": ("FAIL connect：串口 {port} 打不开（{err}）",
                         "FAIL connect: cannot open serial port {port} ({err})"),
    "fail.need_serial": ("FAIL connect：--source real 须指定 --serial COMx（例：--serial COM5）",
                         "FAIL connect: --source real needs --serial COMx (e.g. --serial COM5)"),
    # 模式说明
    "mode.demo": ("模式 demo：0x05 四槽四类（PWM_OPEN/SERVO/ENC_CLOSED/DC）— 采集 {secs:.0f}s",
                  "mode demo: 0x05 four slots (PWM_OPEN/SERVO/ENC_CLOSED/DC) — capturing {secs:.0f}s"),
    "mode.scenario": ("模式场景 {scen}：0x06 SCEN {id}，槽 {slots} — 采集 {secs:.0f}s",
                      "mode scenario {scen}: 0x06 SCEN {id}, slots {slots} — capturing {secs:.0f}s"),
    # 激活握手
    "activate.wait": ("等待 app 帧（ROM 启动期注入命令会卡死 boot——实测）…",
                      "waiting for app frames (commands injected during ROM boot hang the target — measured)…"),
    "fail.activate_app": ("FAIL activate：{timeout:.0f}s 内未见任何 app 帧（固件未启动或桥不通）",
                          "FAIL activate: no app frame within {timeout:.0f}s (firmware not running or link down)"),
    "fail.activate_scen": ("FAIL activate：{timeout:.0f}s 内未收到 scen_id={id} 的 0x06 帧（命令注入/切换失败）",
                           "FAIL activate: no 0x06 frame with scen_id={id} within {timeout:.0f}s (command injection/switch failed)"),
    "activate.ok": ("已激活：场景 {scen}（id {id}）— 0x06 帧流确认",
                    "activated: scenario {scen} (id {id}) — 0x06 stream confirmed"),
    "text.frame": ("TEXT  {msg}", "TEXT  {msg}"),
    # 采集统计 / 汇总
    "cap.demo": ("采集完成：{n} 帧（{wall:.1f}s，{fps:.1f} fps，dt={dt:.4f}s）， bad_crc={bad}，丢帧={gaps}，text={text}",
                 "capture done: {n} frames ({wall:.1f}s, {fps:.1f} fps, dt={dt:.4f}s), bad_crc={bad}, seq_gaps={gaps}, text={text}"),
    "cap.scen": ("采集完成：{n} 个场景帧（{wall:.1f}s，{fps:.1f} fps，dt={dt:.4f}s）， bad_crc={bad}，丢帧={gaps}，layout_bad={lb}，scen_bad={sb}，hdr_mismatch={hm}，demo05={d5}，text={text}",
                 "capture done: {n} scenario frames ({wall:.1f}s, {fps:.1f} fps, dt={dt:.4f}s), bad_crc={bad}, seq_gaps={gaps}, layout_bad={lb}, scen_bad={sb}, hdr_mismatch={hm}, demo05={d5}, text={text}"),
    "fail.head.demo": ("=== 断言 FAIL × {n} ===", "=== assertions FAIL x{n} ==="),
    "fail.head.scen": ("=== scenario {scen} 断言 FAIL × {n} ===",
                       "=== scenario {scen} assertions FAIL x{n} ==="),
    "pass.demo": ("PASS S 梯度形状全断言：加速段单调升 / 停驻段稳定 / 末段收敛 / 交替腿循环（{n} 帧）",
                  "PASS S-curve shape assertions: monotonic acceleration legs / stable dwell plates / final convergence / alternating legs ({n} frames)"),
    "pass.scenario": ("PASS scenario {scen} 全断言（{n} 帧，四专项+DC 任务卡口径）",
                      "PASS scenario {scen}: all assertions ({n} frames, task-card thresholds)"),
    "pass.link": ("PASS link：{n} 帧，bad_crc={bad}，gaps={gap:.2f}%，零故障，scen_id/槽数/kind 全符",
                  "PASS link: {n} frames, bad_crc={bad}, gaps={gap:.2f}%, zero faults, scen_id/slots/kind all match"),
    # 数值表表头
    "tbl.stair": ("      阶梯表（档, duty 均值, 偏差, 档内波动, 样本数）：",
                  "      staircase table (step, duty mean, err, spread, samples):"),
    "tbl.cal": ("      标定表（角度°, 实测脉宽 us, 期望脉宽 us, 偏差 us, 回读角度°）：",
                "      calibration table (angle deg, measured us, expected us, err us, readback deg):"),
    "tbl.conv": ("      收敛表（目标 rpm, 末半段均值, 误差%）：",
                 "      settling table (target rpm, second-half mean, err %):"),
    "tbl.rise": ("      阶跃表（事件, rise 10%→90%, 峰值 rpm, 超调%）——如实记录，不设硬阈值：",
                 "      step table (event, rise 10%->90%, peak rpm, overshoot%) — recorded as-is, no hard threshold:"),
    "tbl.iso": ("      B 全程统计: duty 均值 {mean:.4f}, 波动 {spread:.4f}（A 步进期间，通道隔离判据）",
                "      B whole-capture stats: duty mean {mean:.4f}, spread {spread:.4f} (channel isolation criterion during A stepping)"),
    "tbl.sym": ("      方向对称统计: 正转段 |duty| 均值 {pos:.4f}, 反转段 |duty| 均值 {neg:.4f}, 错向帧 {bad}（方向位 = sign(duty)，带符号桥回读）",
                "      direction symmetry: |duty| mean {pos:.4f} forward, {neg:.4f} reverse, wrong-sign frames {bad} (direction bit = sign(duty), signed bridge readback)"),
    "tbl.dcseg": ("      方向段表（步进, duty 均值, 偏差, 段内波动, 样本数）：",
                  "      direction segment table (step, duty mean, err, spread, samples):"),
    # 各场景 PASS 行
    "pass.pwm": ("PASS pwm: 占空阶梯逐档精确跟随（±2%）+ 五档全覆盖 + 升序阶梯完整",
                 "PASS pwm: every duty step tracks (±2%), all five steps covered, ascending ladder complete"),
    "pass.servo": ("PASS servo: 五点脉宽-角度线性映射（±{tol:.0f}us）+ 回读角度一致 + 五点全覆盖",
                   "PASS servo: five-point linear pulse-angle mapping (±{tol:.0f}us), readback angles match, all five points covered"),
    "pass.enc": ("PASS enc: 各段收敛 ±3% + {n} 个 30→90 升沿已记录（rise/峰值/超调见上表）",
                 "PASS enc: every segment settles within ±3%, {n} rising 30->90 edges recorded (rise/peak/overshoot in the table above)"),
    "pass.nested": ("PASS nested: 通道隔离（改 A 时 B 不动）+ 频率独立（20kHz/50Hz 并存）",
                    "PASS nested: channel isolation (B untouched while A steps) + independent frequencies (20kHz and 50Hz coexist)"),
    "pass.dc": ("PASS dc: 方向位翻转（正/反段各 {npos}/{nneg} 段）+ duty 对称（±0.5 幅值）+ 逐帧方向一致（零错向帧）",
                "PASS dc: direction bit toggles ({npos}/{nneg} forward/reverse segments) + symmetric duty (±0.5 magnitude) + per-frame direction agreement (zero wrong-sign frames)"),
    # 方向词 / 绘图页脚
    "dir.rise": ("升", "rising"),
    "dir.fall": ("降", "falling"),
    "plot.footer": (" {lo:.2f}+{bar}  t=[0..{t:.1f}s] {name}（{n} 点）",
                    " {lo:.2f}+{bar}  t=[0..{t:.1f}s] {name} ({n} samples)"),
    # demo 断言 FAIL
    "fail.a_frames": ("A frames: {n} < {need}（桥降级或无帧）",
                      "A frames: {n} < {need} (degraded bridge or no frames)"),
    "fail.a_bad_crc": ("A bad_crc: {got} > {max}", "A bad_crc: {got} > {max}"),
    "fail.a_gaps": ("A seq_gaps: {got:.2f}% > {max:.1f}%", "A seq_gaps: {got:.2f}% > {max:.1f}%"),
    "fail.a_fault": ("A fault frames: {n}", "A fault frames: {n}"),
    "fail.d_legs": ("D s{slot} legs: {got} < {need}（未见交替腿）",
                    "D s{slot} legs: {got} < {need} (no alternating legs)"),
    "fail.d_plates": ("D s{slot} plates: {got} < {need}（未见停驻段）",
                      "D s{slot} plates: {got} < {need} (no dwell plates)"),
    "fail.b_mono": ("B s{slot} leg[{a}:{b}] {dir} 单调违例 {bad} 处（tol {tol:g}）",
                    "B s{slot} leg[{a}:{b}] {bad} monotonic violations on {dir} leg (tol {tol:g})"),
    "fail.b_plate_dev": ("B s{slot} plate[{a}:{b}] setpoint 漂移 {dev:g} > {lim:g}",
                         "B s{slot} plate[{a}:{b}] setpoint drift {dev:g} > {lim:g}"),
    "fail.b_openloop": ("B s{slot} plate[{a}:{b}] duty != setpoint（开环直通破坏）",
                        "B s{slot} plate[{a}:{b}] duty != setpoint (open-loop pass-through broken)"),
    "fail.c_track": ("C s{slot} plate[{a}:{b}] actual 偏离 setpoint {err:g} > {lim:g}（rpm）",
                     "C s{slot} plate[{a}:{b}] actual deviates from setpoint {err:g} > {lim:g} (rpm)"),
    "fail.c_duty_range": ("C s2 duty 越界 [0,1]：[{lo:g}, {hi:g}]",
                          "C s2 duty out of [0,1]: [{lo:g}, {hi:g}]"),
    "fail.c_last_conv": ("C s2 末段未收敛：最后一个停驻段 actual 偏差 {err:g} rpm > {lim:g}",
                         "C s2 final segment not settled: last dwell plate actual deviation {err:g} rpm > {lim:g}"),
    # 场景链路 FAIL
    "fail.link_frames": ("link frames: {n} < {need}（桥降级或无帧）",
                         "link frames: {n} < {need} (degraded bridge or no frames)"),
    "fail.link_bad_crc": ("link bad_crc: {got} > {max}", "link bad_crc: {got} > {max}"),
    "fail.link_gaps": ("link seq_gaps: {got:.2f}% > {max:.1f}%",
                        "link seq_gaps: {got:.2f}% > {max:.1f}%"),
    "fail.link_fault": ("link fault frames: {n}（零容忍）",
                        "link fault frames: {n} (zero tolerance)"),
    "fail.link_layout": ("link 布局/场景号错帧: layout_bad={lb} scen_bad={sb}",
                         "link layout/scenario mismatch frames: layout_bad={lb} scen_bad={sb}"),
    "fail.link_demo05": ("link 切换后仍见 0x05 demo 帧: {n}",
                         "link 0x05 demo frames still seen after switch: {n}"),
    "fail.link_hdr": ("link 定时器快照帧间漂移: {n}",
                      "link timer snapshot drift between frames: {n}"),
    "fail.timers": ("timers 快照 {got} != 期望 {exp}",
                    "timers snapshot {got} != expected {exp}"),
    # 阶梯 / 舵机 / 闭环 / 嵌套 / DC FAIL
    "fail.sc_duty_range": ("{label} duty 越界 [0,1]：[{lo:g}, {hi:g}]",
                           "{label} duty out of [0,1]: [{lo:g}, {hi:g}]"),
    "fail.sc_follow": ("{label} 档 {v:.2f}: duty 均值 {m:.4f} 偏差 {err:.4f} > {tol:.2f}（±2% 跟随破坏）",
                       "{label} step {v:.2f}: duty mean {m:.4f} off by {err:.4f} > {tol:.2f} (±2% tracking broken)"),
    "fail.sc_flat": ("{label} 档 {v:.2f}: 档内 duty 波动 {spread:.4f} > {tol:.2f}（非平稳台阶）",
                     "{label} step {v:.2f}: in-step duty spread {spread:.4f} > {tol:.2f} (not a flat plateau)"),
    "fail.sc_missing": ("{label} 未见档位 {v}（步进覆盖不全）",
                        "{label} step {v} not seen (incomplete step coverage)"),
    "fail.sc_ladder": ("{label} 未见完整升序阶梯 {exp}",
                       "{label} no complete ascending ladder {exp}"),
    "fail.sv_us": ("{label} 角 {ang}°: 脉宽实测 {meas:.1f}us 期望 {exp:.1f}us 偏差 {err:.1f} > {tol:.0f}us",
                   "{label} angle {ang} deg: measured pulse {meas:.1f}us vs expected {exp:.1f}us, off by {err:.1f} > {tol:.0f}us"),
    "fail.sv_rb": ("{label} 角 {ang}°: 回读均值 {ac:.2f}° 偏差 > {tol:.1f}°",
                   "{label} angle {ang} deg: readback mean {ac:.2f} deg deviates > {tol:.1f} deg"),
    "fail.sv_missing": ("{label} 未见标定点 {ang}°",
                        "{label} calibration point {ang} deg not seen"),
    "fail.enc_settle": ("enc 段 {v:.0f} rpm: 末半段均值 {m:.2f} 误差 {err:.2f}% > {tol:.0f}%（收敛破坏）",
                        "enc segment {v:.0f} rpm: second-half mean {m:.2f}, err {err:.2f}% > {tol:.0f}% (settling broken)"),
    "fail.enc_no_rise": ("enc 未见 30→90 升沿事件（步进未见或帧太少）",
                         "enc no 30->90 rising edge event (step missing or too few frames)"),
    "fail.enc_no_30": ("enc 未见 30 rpm 平台", "enc no 30 rpm plateau"),
    "fail.enc_no_90": ("enc 未见 90 rpm 平台", "enc no 90 rpm plateau"),
    "fail.nested_b_mean": ("nested B: 全程 duty 均值 {m:.4f} 偏离 0.5 > {tol:.2f}（隔离破坏）",
                           "nested B: whole-capture duty mean {m:.4f} off 0.5 by > {tol:.2f} (isolation broken)"),
    "fail.nested_b_spread": ("nested B: duty 全程波动 {spread:.4f} > {tol:.2f}（A 步进期间 B 被扰动——通道隔离破坏）",
                             "nested B: whole-capture duty spread {spread:.4f} > {tol:.2f} (B disturbed while A steps — channel isolation broken)"),
    "fail.nested_b_sp": ("nested B: setpoint 漂移 [{lo:.4f}, {hi:.4f}]（应恒 0.5）",
                         "nested B: setpoint drift [{lo:.4f}, {hi:.4f}] (should stay 0.5)"),
    "fail.dc_sign": ("{label} 方向位: {n} 帧与步进方向相反（sign(duty) != sign(sp)）",
                     "{label} direction bit: {n} frames opposite to the step (sign(duty) != sign(sp))"),
    "fail.dc_range": ("{label} duty 越界 [-1,1]：[{lo:g}, {hi:g}]",
                      "{label} duty out of [-1,1]: [{lo:g}, {hi:g}]"),
    "fail.dc_follow": ("{label} 段 {v:+.2f}: duty 均值 {m:.4f} 偏差 {err:.4f} > {tol:.2f}（跟随破坏）",
                       "{label} segment {v:+.2f}: duty mean {m:.4f} off by {err:.4f} > {tol:.2f} (tracking broken)"),
    "fail.dc_flat": ("{label} 段 {v:+.2f}: 段内 duty 波动 {spread:.4f} > {tol:.2f}（非平稳）",
                     "{label} segment {v:+.2f}: in-segment duty spread {spread:.4f} > {tol:.2f} (not flat)"),
    "fail.dc_few_pos": ("{label} 正转段仅 {n} 段（< 2，方向位翻转循环不足）",
                        "{label} only {n} forward segments (< 2, direction toggle cycle insufficient)"),
    "fail.dc_few_neg": ("{label} 反转段仅 {n} 段（< 2，方向位翻转循环不足）",
                        "{label} only {n} reverse segments (< 2, direction toggle cycle insufficient)"),
    "fail.dc_asym_pos": ("{label} 正转段 |duty| 均值 {m:.4f} 偏离 0.5 > {tol:.2f}（不对称）",
                         "{label} forward |duty| mean {m:.4f} off 0.5 by > {tol:.2f} (asymmetric)"),
    "fail.dc_asym_neg": ("{label} 反转段 |duty| 均值 {m:.4f} 偏离 0.5 > {tol:.2f}（不对称）",
                         "{label} reverse |duty| mean {m:.4f} off 0.5 by > {tol:.2f} (asymmetric)"),
    "fail.dc_sym": ("{label} 正/反转 |duty| 均值互差 {d:.4f} > {tol:.2f}（duty 对称破坏）",
                    "{label} forward/reverse |duty| means differ {d:.4f} > {tol:.2f} (duty symmetry broken)"),
}


def text_table_errors():
    """TEXT 表自检：返回问题行列表（空 = 合格）。

    检查三件：每键值对恰 zh/en 两条非空字符串；两侧 `{...}` 占位符集合一致
    （否则一侧 .format 会 KeyError）；键名非空。
    """
    errs = []
    for key, pair in TEXT.items():
        if len(pair) != 2:
            errs.append("%s: 需 zh/en 两条，得 %d" % (key, len(pair)))
            continue
        zh, en = pair
        if not isinstance(zh, str) or not zh:
            errs.append("%s: zh 缺失或空" % key)
        if not isinstance(en, str) or not en:
            errs.append("%s: en 缺失或空" % key)
        if _placeholders(zh) != _placeholders(en):
            errs.append("%s: 占位符不成对 %s vs %s"
                        % (key, sorted(_placeholders(zh)), sorted(_placeholders(en))))
    return errs


def _placeholders(text: str):
    """取格式化占位符集合（{name} / {name:spec} 取 name；{{ }} 转义不计）。"""
    out = set()
    for _, field, spec, _ in string.Formatter().parse(text):
        if field is not None:
            out.add(field.split(".")[0].split("[")[0])
    return out


def T(key: str, **kw) -> str:
    """按当前 LANG 取文案（zh/en 成对，见 TEXT）并按需格式化。"""
    zh, en = TEXT[key]
    template = zh if LANG == "zh" else en
    return template.format(**kw) if kw else template



def crc8(data: bytes) -> int:
    """CRC-8（poly 0x07，初值 0x00）——与固件 proto_crc8 / frame.py 同结果。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class FrameDecoder:
    """流式帧解码器：喂任意分片字节流，逐帧产出 (type, seq, payload)。

    容错与 frame.py 同语义：坏 CRC / LEN 超限 → 从下一字节重找 SOF
    （ROM 启动横幅与帧流同端口，靠此重同步）。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_crc = 0

    def feed(self, chunk: bytes):
        frames = []
        self._buf.extend(chunk)
        while True:
            frame = self._pop_one()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _pop_one(self):
        buf = self._buf
        while True:
            i = buf.find(bytes([SOF]))
            if i < 0:
                del buf[:]
                return None
            if i:
                del buf[:i]
            if len(buf) < 4:
                return None
            length = buf[1]
            if length > PAYLOAD_MAX:
                del buf[:1]
                continue
            total = 4 + length + 1
            if len(buf) < total:
                return None
            if crc8(bytes(buf[1:4 + length])) != buf[4 + length]:
                self.bad_crc += 1
                del buf[:1]
                continue
            frame = (buf[2], buf[3], bytes(buf[4:4 + length]))
            del buf[:total]
            return frame


class Trace:
    """单槽轨迹：setpoint/duty/actual/phase/fault 逐帧序列（等间隔采样）。"""

    def __init__(self, slot: int) -> None:
        self.slot = slot
        self.sp = []
        self.du = []
        self.ac = []
        self.ph = []
        self.fa = []

    def add(self, sp, du, ac, ph, fa):
        self.sp.append(sp)
        self.du.append(du)
        self.ac.append(ac)
        self.ph.append(ph)
        self.fa.append(fa)

    def runs(self):
        """按 phase==DONE 切段：产出 (kind, start, end)——kind='plate'（停驻）或
        'leg'（运动腿）。段边界含头不含尾，与列表切片同语义。"""
        out = []
        n = len(self.ph)
        i = 0
        while i < n:
            j = i
            is_done = self.ph[i] == PHASE_DONE
            while j < n and (self.ph[j] == PHASE_DONE) == is_done:
                j += 1
            out.append(("plate" if is_done else "leg", i, j))
            i = j
        return out


def fmt_val(v, domain):
    if domain in ("duty", "sduty"):
        return "%.4f" % v
    if domain == "deg":
        return "%8.2f" % v
    return "%8.2f" % v   # rpm


def render_row(tr: Trace, ranges, meta=None):
    """滚动数值表一行 + 内联自量程 ASCII 条（示波器滚动视图）。

    meta=None 走 demo 槽身份（SLOT_NAME/固定域）；场景模式传入
    {"name","domain"} 走场景槽身份。带符号 duty 域（sduty，DC 槽）追加
    方向位显示 dir+FWD/dir-REV（符号 = H 桥方向位）。
    """
    if meta is None:
        name = SLOT_NAME[tr.slot]
        dom = ("duty", "deg", "rpm", "sduty")[tr.slot]
    else:
        name = meta["name"]
        dom = meta["domain"]
    i = len(tr.sp) - 1
    lo, hi = ranges[tr.slot]
    span = (hi - lo) or 1.0
    frac = max(0.0, min(1.0, (tr.sp[i] - lo) / span))
    bar = "#" * int(frac * 24) + "." * (24 - int(frac * 24))
    row = ("s%d %-9s sp %s [%s] du %.4f ac %s ph%d f%d"
           % (tr.slot, name, fmt_val(tr.sp[i], dom), bar,
              tr.du[i], fmt_val(tr.ac[i], dom), tr.ph[i], tr.fa[i]))
    if dom == "sduty":
        row += " dir%s" % ("+FWD" if tr.du[i] >= 0.0 else "-REV")
    return row


def plot_ascii(tr: Trace, width=72, height=10, name=None, full=None, dt=0.002,
               lo=0.0):
    """终段 ASCII 波形：全捕获轨迹下采样叠加——'#'=setpoint，'*'=actual。

    纵轴 = [lo, full] 槽域（demo：PLOT_FULLSCALE/PLOT_LO；场景：传入该槽
    full/lo）；横轴 = 采样序（dt = 实测帧周期）。开环槽两条线重合
    （actual==施加值）；闭环槽可见追踪滞后与收敛；带符号域（DC）负值
    走 lo 下限不再裁零。
    """
    if name is None:
        name = SLOT_NAME[tr.slot]
    if full is None:
        full = PLOT_FULLSCALE[tr.slot]
        lo = PLOT_LO[tr.slot]
    span = (full - lo) or 1.0
    rows = [[" "] * width for _ in range(height)]
    for series, ch in ((tr.sp, "#"), (tr.ac, "*")):
        if not series:
            continue
        for col in range(width):
            lo_i = col * len(series) // width
            hi_i = (col + 1) * len(series) // width or len(series)
            v = min(series[lo_i:hi_i]) if ch == "#" else max(series[lo_i:hi_i])
            frac = (max(lo, min(full, v)) - lo) / span
            r = int((1.0 - frac) * (height - 1))
            rows[r][col] = ch if rows[r][col] == " " else "+"
    lines = ["    +%s" % ("-" * width)]
    for k, row in enumerate(rows):
        tag = "%.2f|" % (full - span * (height - 1 - k) / (height - 1))
        lines.append(tag + "".join(row))
    lines.append(T("plot.footer", lo=lo, bar="-" * width, t=len(tr.sp) * dt,
                    name=name, n=len(tr.sp)))
    return "\n".join(lines)


def check_traces(traces, stats):
    """S 梯度形状断言（A~D，见模块 docstring）。返回 FAIL 行列表。"""
    fails = []
    n_frames = len(traces[0].sp) if traces else 0

    # A. 链路健康
    if n_frames < CHECKS["min_frames"]:
        fails.append(T("fail.a_frames", n=n_frames, need=CHECKS["min_frames"]))
    if stats["bad_crc"] > CHECKS["max_bad_crc"]:
        fails.append(T("fail.a_bad_crc", got=stats["bad_crc"], max=CHECKS["max_bad_crc"]))
    gap_pct = 100.0 * stats["seq_gaps"] / n_frames if n_frames else 100.0
    if gap_pct > CHECKS["max_seq_gap_pct"]:
        fails.append(T("fail.a_gaps", got=gap_pct, max=CHECKS["max_seq_gap_pct"]))
    fault_frames = sum(1 for tr in traces for f in tr.fa if f)
    if 100.0 * fault_frames / max(1, len(traces) * n_frames) > CHECKS["max_fault_frames_pct"]:
        fails.append(T("fail.a_fault", n=fault_frames))

    for tr in traces:
        s = tr.slot
        legs = [r for r in tr.runs() if r[0] == "leg"]
        plates = [r for r in tr.runs() if r[0] == "plate" and r[2] - r[1] >= CHECKS["min_plate_len"]]

        # D. 循环性（retarget 交替腿）
        if len(legs) < CHECKS["min_legs"]:
            fails.append(T("fail.d_legs", slot=s, got=len(legs), need=CHECKS["min_legs"]))
        if len(plates) < CHECKS["min_plates"]:
            fails.append(T("fail.d_plates", slot=s, got=len(plates), need=CHECKS["min_plates"]))

        # B. 运动腿单调（开环槽 0/1/3：duty/deg/带符号 duty 域；容差表按槽索引）
        if s in (0, 1, 3):
            tol = CHECKS["mono_tol"][s]
            for _, a, b in legs:
                seg = tr.sp[a:b]
                if b - a < 2:
                    continue
                rising = seg[-1] >= seg[0]
                bad = 0
                for k in range(1, len(seg)):
                    if (rising and seg[k] < seg[k - 1] - tol) or \
                       (not rising and seg[k] > seg[k - 1] + tol):
                        bad += 1
                if bad:
                    fails.append(T("fail.b_mono", slot=s, a=a, b=b, bad=bad, tol=tol,
                                    dir=T("dir.rise") if rising else T("dir.fall")))

        # B/C. 停驻段稳定 + 收敛
        sp_abs = CHECKS["plate_sp_dev_abs"][s]
        for _, a, b in plates:
            seg = tr.sp[a:b]
            med = sorted(seg)[len(seg) // 2]
            dev = max(abs(v - med) for v in seg)
            lim = max(CHECKS["plate_sp_dev_pct"] / 100.0 * abs(med), sp_abs)
            if dev > lim:
                fails.append(T("fail.b_plate_dev", slot=s, a=a, b=b, dev=dev, lim=lim))
            # 开环直通恒等：slot0 duty 域 / slot3 带符号 duty 域（DC 桥回读）
            if s in (0, 3) and max(abs(tr.du[k] - tr.sp[k]) for k in range(a, b)) > 1e-3:
                fails.append(T("fail.b_openloop", slot=s, a=a, b=b))
            if s == 2:
                err = abs(sum(tr.ac[a:b]) / (b - a) - med)
                lim2 = max(CHECKS["s2_track_pct"] / 100.0 * abs(med),
                           CHECKS["s2_track_abs"])
                if err > lim2:
                    fails.append(T("fail.c_track", slot=s, a=a, b=b, err=err, lim=lim2))

        # C. 闭环槽 duty 界 + 末段收敛
        if s == 2:
            if max(tr.du, default=0.0) > CHECKS["s2_duty_max"] + 1e-6 or min(tr.du, default=0.0) < -1e-6:
                fails.append(T("fail.c_duty_range", lo=min(tr.du), hi=max(tr.du)))
            if plates:
                _, a, b = plates[-1]
                med = sorted(tr.sp[a:b])[(b - a) // 2]
                err = abs(sum(tr.ac[a:b]) / (b - a) - med)
                lim2 = max(CHECKS["s2_track_pct"] / 100.0 * abs(med),
                           CHECKS["s2_track_abs"])
                if err > lim2:
                    fails.append(T("fail.c_last_conv", err=err, lim=lim2))
    return fails


# ═════════════ 场景模式（T-112 四专项） ═════════════

def parse_scen_payload(payload):
    """0x06 payload → (scen_id, timers, n_rec, rec_off)；None = 布局不合法。"""
    if len(payload) < 4:
        return None
    scen = payload[0]
    n_tim = payload[1]
    off = 2 + 6 * n_tim
    if off + 1 > len(payload):
        return None
    timers = []
    for t in range(n_tim):
        idx, ch, freq = SCEN_TIMER_REC.unpack_from(payload, 2 + 6 * t)
        timers.append((idx, ch, freq))
    n_rec = payload[off]
    off += 1
    if len(payload) != off + 16 * n_rec:
        return None
    return scen, timers, n_rec, off


def plateaus(sp, min_len):
    """按 setpoint 恒值切台阶：产出 (start, end, value)；短段丢弃。"""
    out = []
    i = 0
    n = len(sp)
    while i < n:
        j = i
        while j < n and abs(sp[j] - sp[i]) <= 1e-6:
            j += 1
        if j - i >= min_len:
            out.append((i, j, sp[i]))
        i = j
    return out


def check_staircase(tr, exp_steps, label):
    """占空阶梯断言：各档 duty 均值跟随 ±2%、档内平稳、全覆盖 + 升序可见。"""
    fails = []
    plates = plateaus(tr.sp, SCEN_CHECKS["min_plate_len"])
    if min(tr.du) < -0.002 or max(tr.du) > 1.002:
        fails.append(T("fail.sc_duty_range", label=label, lo=min(tr.du), hi=max(tr.du)))
    rows = []
    for a, b, v in plates:
        seg = tr.du[a:b]
        m = sum(seg) / len(seg)
        err = abs(m - v)
        spread = max(seg) - min(seg)
        rows.append((v, m, err, spread, b - a))
        if err > SCEN_CHECKS["duty_follow_tol"]:
            fails.append(T("fail.sc_follow", label=label, v=v, m=m, err=err,
                           tol=SCEN_CHECKS["duty_follow_tol"]))
        if spread > SCEN_CHECKS["duty_flat_tol"]:
            fails.append(T("fail.sc_flat", label=label, v=v, spread=spread,
                           tol=SCEN_CHECKS["duty_flat_tol"]))
    seen = [v for _, _, v in plates]
    for e in exp_steps:
        if not any(abs(v - e) < 0.01 for v in seen):
            fails.append(T("fail.sc_missing", label=label, v=e))
    seq = [round(v, 2) for _, _, v in plates]
    exp = [round(e, 2) for e in exp_steps]
    k = len(exp)
    if not any(seq[i:i + k] == exp for i in range(len(seq) - k + 1)):
        fails.append(T("fail.sc_ladder", label=label, exp=exp))
    return fails, rows


def check_servo_points(tr, freq, label, exp_angles):
    """舵机标定断言：各点脉宽 = 线性插值 ±容差；回读角度一致；点覆盖。"""
    fails = []
    plates = plateaus(tr.sp, SCEN_CHECKS["min_plate_len"])
    pmin, pmax = SG90["pmin_us"], SG90["pmax_us"]
    amin, amax = SG90["amin"], SG90["amax"]
    cal = []
    for a, b, v in plates:
        ang = v
        us_exp = pmin + (ang - amin) * (pmax - pmin) / (amax - amin)
        seg = tr.du[a:b]
        duty_m = sum(seg) / len(seg)
        us_meas = duty_m * 1000000.0 / freq
        ac_seg = tr.ac[a:b]
        ac_m = sum(ac_seg) / len(ac_seg)
        err = abs(us_meas - us_exp)
        cal.append((ang, us_meas, us_exp, err, ac_m))
        if err > SCEN_CHECKS["servo_us_tol"]:
            fails.append(T("fail.sv_us", label=label, ang=ang, meas=us_meas, exp=us_exp,
                           err=err, tol=SCEN_CHECKS["servo_us_tol"]))
        if abs(ac_m - ang) > SCEN_CHECKS["servo_rb_deg_tol"]:
            fails.append(T("fail.sv_rb", label=label, ang=ang, ac=ac_m,
                           tol=SCEN_CHECKS["servo_rb_deg_tol"]))
    seen = [v for _, _, v in plates]
    for e in exp_angles:
        if not any(abs(v - e) < 0.5 for v in seen):
            fails.append(T("fail.sv_missing", label=label, ang=e))
    return fails, cal


def check_enc(tr, dt_frame):
    """闭环阶跃断言：各段末半段均值收敛 ±3%；升沿 rise/峰值/超调如实记录。"""
    fails = []
    plates = plateaus(tr.sp, SCEN_CHECKS["min_plate_len"])
    conv = []
    for a, b, v in plates:
        s0 = a + (b - a) // 2
        seg = tr.ac[s0:b]
        m = sum(seg) / len(seg)
        err_pct = abs(m - v) / abs(v) * 100.0
        conv.append((v, m, err_pct))
        if err_pct > SCEN_CHECKS["enc_settle_pct"]:
            fails.append(T("fail.enc_settle", v=v, m=m, err=err_pct,
                           tol=SCEN_CHECKS["enc_settle_pct"]))
    events = []
    for k in range(len(plates) - 1):
        a1, b1, v1 = plates[k + 1]
        v0 = plates[k][2]
        if v1 <= v0 + 1.0:
            continue
        lo = v0 + 0.1 * (v1 - v0)
        hi = v0 + 0.9 * (v1 - v0)
        t10 = -1
        t90 = -1
        for i in range(a1, b1):
            if t10 < 0 and tr.ac[i] >= lo:
                t10 = i
            if t10 >= 0 and tr.ac[i] >= hi:
                t90 = i
                break
        rise = (t90 - t10) * dt_frame if (t10 >= 0 and t90 >= 0) else -1.0
        peak = max(tr.ac[a1:b1])
        ov = (peak - v1) / (v1 - v0) * 100.0
        events.append((v0, v1, rise, peak, ov))
    if not events:
        fails.append(T("fail.enc_no_rise"))
    targets = set(round(v, 1) for _, _, v in plates)
    if 30.0 not in targets:
        fails.append(T("fail.enc_no_30"))
    if 90.0 not in targets:
        fails.append(T("fail.enc_no_90"))
    return fails, conv, events


def check_nested(tr_a, tr_b, tr_c, freq50):
    """嵌套断言：B 全程恒 0.5（通道隔离）+ A 阶梯 + C 舵机映射。"""
    fails = []
    b_mean = sum(tr_b.du) / len(tr_b.du)
    b_spread = max(tr_b.du) - min(tr_b.du)
    if abs(b_mean - 0.5) > SCEN_CHECKS["duty_follow_tol"]:
        fails.append(T("fail.nested_b_mean", m=b_mean, tol=SCEN_CHECKS["duty_follow_tol"]))
    if b_spread > SCEN_CHECKS["duty_flat_tol"]:
        fails.append(T("fail.nested_b_spread", spread=b_spread,
                       tol=SCEN_CHECKS["duty_flat_tol"]))
    if max(tr_b.sp) - min(tr_b.sp) > 1e-3:
        fails.append(T("fail.nested_b_sp", lo=min(tr_b.sp), hi=max(tr_b.sp)))
    fails_a, rows_a = check_staircase(tr_a, SCEN["nested"]["slots"][0]["steps"], "nested A")
    fails += fails_a
    fails_c, cal_c = check_servo_points(
        tr_c, freq50, "nested C", SCEN["nested"]["slots"][2]["steps"])
    fails += fails_c
    return fails, rows_a, cal_c, (b_mean, b_spread)


def check_dc(tr, label="dc"):
    """DC H 桥断言：方向位翻转 + duty 对称 + 逐帧方向一致 + 界内。

    - 方向位 = sign(带符号桥回读 duty)；步进 ±0.5 无过渡零点 → 逐帧
      sign(duty) == sign(setpoint) 可判（错向帧计数零容忍）。
    - duty 对称：正段/反段 |duty| 均值各自 ≈ 0.5（±duty_follow_tol）且
      两者互差 ≤ duty_follow_tol（正反转输出幅值对称）。
    - 方向位翻转循环：正/反段各 ≥2（4s 周期、26s 采集应各 ~6）。
    """
    fails = []
    tol = SCEN_CHECKS["duty_follow_tol"]
    # 逐帧方向位一致 + |duty| 界内（带符号域 [−1,1]）
    bad_sign = sum(1 for s_, d_ in zip(tr.sp, tr.du) if s_ * d_ <= 0.0)
    if bad_sign:
        fails.append(T("fail.dc_sign", label=label, n=bad_sign))
    if min(tr.du) < -1.0 - 1e-6 or max(tr.du) > 1.0 + 1e-6:
        fails.append(T("fail.dc_range", label=label, lo=min(tr.du), hi=max(tr.du)))
    # 台阶段：跟随 + 平稳（带符号值直接与步进值比）
    plates = plateaus(tr.sp, SCEN_CHECKS["min_plate_len"])
    rows = []
    for a, b, v in plates:
        seg = tr.du[a:b]
        m = sum(seg) / len(seg)
        err = abs(m - v)
        spread = max(seg) - min(seg)
        rows.append((v, m, err, spread, b - a))
        if err > tol:
            fails.append(T("fail.dc_follow", label=label, v=v, m=m, err=err, tol=tol))
        if spread > SCEN_CHECKS["duty_flat_tol"]:
            fails.append(T("fail.dc_flat", label=label, v=v, spread=spread,
                           tol=SCEN_CHECKS["duty_flat_tol"]))
    pos = [r for r in rows if r[0] > 0.0]
    neg = [r for r in rows if r[0] < 0.0]
    if len(pos) < 2:
        fails.append(T("fail.dc_few_pos", label=label, n=len(pos)))
    if len(neg) < 2:
        fails.append(T("fail.dc_few_neg", label=label, n=len(neg)))
    # duty 对称：正/反段幅值各自到 0.5 ±tol，且互差 ≤ tol
    pos_mag = sum(r[1] for r in pos) / len(pos) if pos else 0.0
    neg_mag = -sum(r[1] for r in neg) / len(neg) if neg else 0.0
    if pos and abs(pos_mag - 0.5) > tol:
        fails.append(T("fail.dc_asym_pos", label=label, m=pos_mag, tol=tol))
    if neg and abs(neg_mag - 0.5) > tol:
        fails.append(T("fail.dc_asym_neg", label=label, m=neg_mag, tol=tol))
    if pos and neg and abs(pos_mag - neg_mag) > tol:
        fails.append(T("fail.dc_sym", label=label, d=abs(pos_mag - neg_mag), tol=tol))
    return fails, rows, (pos_mag, neg_mag, bad_sign)


# ── 数据源链路（三档源统一到一个 socket 形状的读/写面） ──

class SourceError(Exception):
    """数据源建立失败：带文案键与格式参数，由 main 用 T() 本地化输出后 exit 2。"""

    def __init__(self, key: str, **kw):
        super().__init__(key)
        self.key = key
        self.kw = kw


class SocketLink:
    """TCP 链路（qemu 档）：recv 超时招 socket.timeout，与旧路径语义相同。"""

    frame_dt_s = None      # 真实桥：帧周期由采集结果实测算

    def __init__(self, sock):
        self._sock = sock

    def recv(self, n):
        return self._sock.recv(n)

    def sendall(self, data):
        self._sock.sendall(data)

    def settimeout(self, t):
        self._sock.settimeout(t)

    def close(self):
        self._sock.close()


class SerialLink:
    """pyserial 链路（real 档）：读超时抬 socket.timeout，与 SocketLink 同语义
    （采集/握手循环的超时与截止处理零分岔）；flush 保证 SCEN 命令即时下发。"""

    frame_dt_s = None

    def __init__(self, ser):
        self._ser = ser

    def recv(self, n):
        data = self._ser.read(n)
        if not data:
            raise socket.timeout()      # 等价于 socket 读超时：上层继续循环判截止
        return data

    def sendall(self, data):
        self._ser.write(data)
        self._ser.flush()

    def settimeout(self, t):
        self._ser.timeout = t

    def close(self):
        self._ser.close()


class SimLink:
    """合成帧源链路（sim 档）：把 sim_motor 的定帧率帧流按墙钟节拍“推”给上层。

    帧值与步进由帧序号确定性决定（见 sim_motor 模块注），本层只负责“何时到期”：
    recv 先消费待发缓冲，空了就算到期帧（src.poll），仍空就睡到下一帧到期。
    不建任何 socket、不碰串口，因此缺省档零外部依赖。
    """

    def __init__(self, src: sim_motor.SimMotorSource):
        self._src = src
        self._pending = bytearray()
        self._t0 = time.monotonic()
        self.frame_dt_s = src.frame_dt_s    # 每帧仿真时长（dt 显示 / rise 统计用）

    def recv(self, n):
        while True:
            if self._pending:
                chunk = bytes(self._pending[:n])
                del self._pending[:n]
                return chunk
            now = time.monotonic() - self._t0
            self._pending += self._src.poll(now)
            if self._pending:
                continue
            wait = self._src.next_due_s() - now
            time.sleep(min(max(wait, 0.0002), 0.05))

    def sendall(self, data):
        self._src.command(data.decode("ascii", "replace"))

    def settimeout(self, t):
        pass      # 合成源无读超时概念（帧率自己控）

    def close(self):
        pass


def connect_scope(host, port, timeout_s=None):
    """连 QEMU 串口桥（wait 模式：连接即开机）。返回 SocketLink 或 None。

    timeout_s = None 取 CONNECT_TIMEOUT_S：到点未连通即返回 None（不干等）。
    单次尝试超时 = CONNECT_ATTEMPT_S（Windows 被拒端口会吃满单次超时）。
    """
    budget = CONNECT_TIMEOUT_S if timeout_s is None else timeout_s
    deadline = time.monotonic() + budget
    while True:
        try:
            return SocketLink(socket.create_connection((host, port),
                                                      timeout=CONNECT_ATTEMPT_S))
        except OSError:
            if time.monotonic() >= deadline:
                return None
            time.sleep(CONNECT_RETRY_S)


def sim_cfg():
    """sim 档参数表（describe_source 与 open_source 共用同一份，避免口径分叉）。"""
    return sim_motor.SimMotorCfg()


def open_source(args):
    """按 --source 建链路：sim（默认，零外部依赖）/ qemu（TCP 桥）/ real（串口）。

    失败一律抛 SourceError（带文案键），由 main 本地化打印后 exit 2。
    """
    if args.source == "sim":
        return SimLink(sim_motor.SimMotorSource(SCEN, SLOT_KIND, SG90, sim_cfg()))
    if args.source == "qemu":
        link = connect_scope(args.host, args.port)
        if link is None:
            raise SourceError("fail.conn_qemu", host=args.host, port=args.port,
                              timeout=CONNECT_TIMEOUT_S)
        return link
    # real：pyserial 真串口（缺失/打不开均明确报错 exit 2）
    if not args.serial:
        raise SourceError("fail.need_serial")
    try:
        import serial
    except ImportError:
        raise SourceError("fail.conn_pyserial")
    try:
        return SerialLink(serial.Serial(args.serial, args.baud, timeout=SERIAL_TIMEOUT_S))
    except Exception as exc:            # serial.SerialException 及驱动层异常
        raise SourceError("fail.conn_serial", port=args.serial, err=exc)


def describe_source(args) -> str:
    """当前数据源的一句说明（banner 后的第二行）。"""
    if args.source == "sim":
        cfg = sim_cfg()
        return T("mode.sim", speed=cfg.frame_rate_hz * cfg.dt_s)
    if args.source == "qemu":
        return T("mode.qemu", host=args.host, port=args.port)
    return T("mode.real", port=args.serial or "-", baud=args.baud)


def source_endpoint(args) -> str:
    """连接成功的端点标识（已连接行的 desc）。"""
    if args.source == "sim":
        return "sim"
    if args.source == "qemu":
        return "%s:%d" % (args.host, args.port)
    return "%s @ %d" % (args.serial or "-", args.baud)


def read_frames(link, dec):
    """读一次链路并解码：返回 (frames, eof)；超时 = 空帧列表（非 EOF）。"""
    try:
        chunk = link.recv(4096)
    except (socket.timeout, TimeoutError):
        return [], False
    except OSError:
        return [], True                  # 对端被杀/串口拔掉：按已收数据收口
    if not chunk:
        return [], True
    return dec.feed(chunk), False


def run_scenario(args, link):
    """场景模式：激活握手（SCEN 命令注入）→ 采集 → 四专项断言集。"""
    info = SCEN[args.scenario]
    secs = args.seconds if args.seconds is not None else info["secs"]
    link.settimeout(SOCKET_TIMEOUT_S)
    print(T("mode.scenario", scen=args.scenario, id=info["id"], secs=secs,
            slots=",".join(s["name"] for s in info["slots"])))

    dec = FrameDecoder()

    # ── 激活握手第一步：等 app 帧出现（ROM 启动期注入命令会卡死 boot——实测） ──
    t0 = time.monotonic()
    booted = False
    while not booted and time.monotonic() - t0 < 20.0:
        frames, eof = read_frames(link, dec)
        if eof:
            break
        for ftype, seq, payload in frames:
            if ftype == TYPE_TEXT:
                print(T("text.frame", msg=payload.decode("ascii", "replace").strip()))
            if ftype in (TYPE_MOTOR, TYPE_SCEN):
                booted = True
    if not booted:
        print(T("fail.activate_app", timeout=20.0))
        link.close()
        return 1

    # ── 激活握手第二步：0.5s 重发 SCEN <n>，直到 0x06 帧 scen_id 匹配 ──
    t1 = time.monotonic()
    last_send = 0.0
    activated = False
    while not activated and time.monotonic() - t1 < 15.0:
        now = time.monotonic()
        if now - last_send >= 0.5:
            link.sendall(b"SCEN %d\n" % info["id"])
            last_send = now
        frames, eof = read_frames(link, dec)
        if eof:
            break
        for ftype, seq, payload in frames:
            if ftype == TYPE_TEXT:
                print(T("text.frame", msg=payload.decode("ascii", "replace").strip()))
            elif ftype == TYPE_SCEN:
                parsed = parse_scen_payload(payload)
                if parsed is not None and parsed[0] == info["id"]:
                    activated = True
    if not activated:
        print(T("fail.activate_scen", timeout=15.0, id=info["id"]))
        link.close()
        return 1
    print(T("activate.ok", scen=args.scenario, id=info["id"]))

    # ── 排干在途重复命令（重发即场景重启）：0.6s 后场景稳定在步 0 附近 ──
    drain_until = time.monotonic() + 0.6
    while time.monotonic() < drain_until:
        frames, eof = read_frames(link, dec)
        if eof:
            break
        # 丢弃：重启瞬态不进采集

    # ── 采集 ──
    n_slots = len(info["slots"])
    traces = [Trace(i) for i in range(n_slots)]
    ranges = [[float("inf"), float("-inf")] for _ in range(n_slots)]
    stats = {"bad_crc": 0, "seq_gaps": 0, "text": 0, "layout_bad": 0,
             "scen_bad": 0, "hdr_mismatch": 0, "demo05": 0}
    prev_seq = None
    base_hdr = None
    base_timers = None
    n_frames = 0
    t2 = time.monotonic()
    eof = False

    while not eof and time.monotonic() - t2 < secs:
        frames, eof = read_frames(link, dec)
        for ftype, seq, payload in frames:
            if ftype == TYPE_TEXT:
                stats["text"] += 1
                print(T("text.frame", msg=payload.decode("ascii", "replace").strip()))
                continue
            if ftype == TYPE_MOTOR:
                stats["demo05"] += 1   # 切换后残余 demo 帧（不应出现，仅计数）
                continue
            if ftype != TYPE_SCEN:
                continue
            parsed = parse_scen_payload(payload)
            if parsed is None:
                stats["layout_bad"] += 1
                continue
            scen, timers, n_rec, rec_off = parsed
            if scen != info["id"] or n_rec != n_slots:
                stats["scen_bad"] += 1
                continue
            hdr = bytes(payload[:rec_off])
            if base_hdr is None:
                base_hdr = hdr
                base_timers = timers
            elif hdr != base_hdr:
                stats["hdr_mismatch"] += 1
            if prev_seq is not None and (seq - prev_seq - 1) % 256 != 0:
                stats["seq_gaps"] += 1
            prev_seq = seq
            n_frames += 1
            for r in range(n_rec):
                slot, kind, sp, du, ac, ph, fa = MOTOR_REC.unpack_from(payload, rec_off + 16 * r)
                if slot != r or kind != info["slots"][r]["kind"]:
                    stats["layout_bad"] += 1   # 布局错位按坏帧计（防静默错解）
                    continue
                traces[r].add(sp, du, ac, ph, fa)
                lo, hi = ranges[r]
                ranges[r] = [min(lo, sp, ac, du), max(hi, sp, ac, du)]
            if (not args.quiet and args.render_every > 0
                    and n_frames % args.render_every == 0):
                metas = info["slots"]
                print("[%6.2fs f=%-6d] %s" % (time.monotonic() - t2, n_frames,
                                              " | ".join(render_row(t, ranges, metas[t.slot])
                                                         for t in traces)))
    link.close()

    stats["bad_crc"] += dec.bad_crc
    wall = time.monotonic() - t2
    # 仿真档 dt = 源声明的每帧仿真时长（墙钟与仿真时间不同比，用墙钟均值会错标 rise）
    dt_frame = link.frame_dt_s if link.frame_dt_s else wall / max(1, n_frames)
    print(T("cap.scen", n=n_frames, wall=wall, fps=n_frames / wall if wall > 0 else 0.0,
            dt=dt_frame, bad=stats["bad_crc"], gaps=stats["seq_gaps"],
            lb=stats["layout_bad"], sb=stats["scen_bad"],
            hm=stats["hdr_mismatch"], d5=stats["demo05"], text=stats["text"]))

    if not args.no_plot:
        for tr in traces:
            meta = info["slots"][tr.slot]
            print(plot_ascii(tr, name="%s(%s)" % (meta["name"], args.scenario),
                             full=meta["full"], dt=dt_frame,
                             lo=meta.get("lo", 0.0)))

    # ── 断言 ──
    fails = []
    if n_frames < SCEN_CHECKS["min_frames"]:
        fails.append(T("fail.link_frames", n=n_frames, need=SCEN_CHECKS["min_frames"]))
    if stats["bad_crc"] > SCEN_CHECKS["max_bad_crc"]:
        fails.append(T("fail.link_bad_crc", got=stats["bad_crc"],
                       max=SCEN_CHECKS["max_bad_crc"]))
    gap_pct = 100.0 * stats["seq_gaps"] / n_frames if n_frames else 100.0
    if gap_pct > SCEN_CHECKS["max_seq_gap_pct"]:
        fails.append(T("fail.link_gaps", got=gap_pct, max=SCEN_CHECKS["max_seq_gap_pct"]))
    fault_frames = sum(1 for tr in traces for f in tr.fa if f)
    if fault_frames:
        fails.append(T("fail.link_fault", n=fault_frames))
    if stats["layout_bad"] or stats["scen_bad"]:
        fails.append(T("fail.link_layout", lb=stats["layout_bad"], sb=stats["scen_bad"]))
    if stats["demo05"]:
        fails.append(T("fail.link_demo05", n=stats["demo05"]))
    if stats["hdr_mismatch"]:
        fails.append(T("fail.link_hdr", n=stats["hdr_mismatch"]))
    if base_timers is None or sorted(base_timers) != sorted(info["timers"]):
        fails.append(T("fail.timers", got=base_timers, exp=info["timers"]))
    print(T("pass.link", n=n_frames, bad=stats["bad_crc"], gap=gap_pct))

    if args.scenario == "pwm":
        f2, rows = check_staircase(traces[0], info["slots"][0]["steps"], "pwm")
        fails += f2
        print(T("tbl.stair"))
        for v, m, err, spread, n in rows:
            print("      %.2f, %.4f, %.4f, %.4f, %d" % (v, m, err, spread, n))
        if not f2:
            print(T("pass.pwm"))
    elif args.scenario == "servo":
        freq = base_timers[0][2] if base_timers else 50
        f2, cal = check_servo_points(traces[0], freq, "servo",
                                     info["slots"][0]["steps"])
        fails += f2
        print(T("tbl.cal"))
        for ang, us_m, us_e, err, ac_m in cal:
            print("      %.1f, %.1f, %.1f, %.1f, %.2f" % (ang, us_m, us_e, err, ac_m))
        if not f2:
            print(T("pass.servo", tol=SCEN_CHECKS["servo_us_tol"]))
    elif args.scenario == "enc":
        f2, conv, events = check_enc(traces[0], dt_frame)
        fails += f2
        print(T("tbl.conv"))
        for v, m, e in conv:
            print("      %.0f, %.2f, %.2f" % (v, m, e))
        print(T("tbl.rise"))
        for k, (v0, v1, rise, peak, ov) in enumerate(events):
            print("      #%d %.0f→%.0f, %.4fs, %.2f, %.2f" % (k, v0, v1, rise, peak, ov))
        if not f2:
            print(T("pass.enc", n=len(events)))
    elif args.scenario == "nested":
        freq50 = 50
        for t in (base_timers or []):
            if t[0] == 1:
                freq50 = t[2]
        f2, rows_a, cal_c, bstat = check_nested(traces[0], traces[1], traces[2], freq50)
        fails += f2
        print(T("tbl.iso", mean=bstat[0], spread=bstat[1]))
        print(T("tbl.stair"))
        for v, m, err, spread, n in rows_a:
            print("      %.2f, %.4f, %.4f, %.4f, %d" % (v, m, err, spread, n))
        print(T("tbl.cal"))
        for ang, us_m, us_e, err, ac_m in cal_c:
            print("      %.1f, %.1f, %.1f, %.1f, %.2f" % (ang, us_m, us_e, err, ac_m))
        if not f2:
            print(T("pass.nested"))
    elif args.scenario == "dc":
        f2, rows, dcstat = check_dc(traces[0], "dc")
        fails += f2
        print(T("tbl.sym", pos=dcstat[0], neg=dcstat[1], bad=dcstat[2]))
        print(T("tbl.dcseg"))
        for v, m, err, spread, n in rows:
            print("      %+.2f, %+.4f, %.4f, %.4f, %d" % (v, m, err, spread, n))
        if not f2:
            print(T("pass.dc", npos=len([r for r in rows if r[0] > 0]),
                    nneg=len([r for r in rows if r[0] < 0])))

    if fails:
        print("\n" + T("fail.head.scen", scen=args.scenario, n=len(fails)))
        for f in fails:
            print("FAIL " + f)
        return 1
    print("\n" + T("pass.scenario", scen=args.scenario, n=n_frames))
    return 0


def run_demo(args, link):
    """demo 模式：采 0x05 四槽遥测，做 S 梯度形状断言（A~D）。"""
    secs = args.seconds if args.seconds is not None else 30.0
    link.settimeout(SOCKET_TIMEOUT_S)
    print(T("mode.demo", secs=secs))

    dec = FrameDecoder()
    traces = [Trace(i) for i in range(MOTOR_SLOTS)]
    ranges = [[float("inf"), float("-inf")] for _ in range(MOTOR_SLOTS)]
    stats = {"bad_crc": 0, "seq_gaps": 0, "text": 0}
    prev_seq = None
    n_motor = 0
    t0 = time.monotonic()
    eof = False

    while not eof and time.monotonic() - t0 < secs:
        frames, eof = read_frames(link, dec)
        for ftype, seq, payload in frames:
            if ftype == TYPE_TEXT:
                stats["text"] += 1
                print(T("text.frame", msg=payload.decode("ascii", "replace").strip()))
                continue
            if ftype != TYPE_MOTOR or len(payload) != MOTOR_PAYLOAD_LEN:
                continue
            if prev_seq is not None and (seq - prev_seq - 1) % 256 != 0:
                stats["seq_gaps"] += 1
            prev_seq = seq
            n_motor += 1
            for r in range(MOTOR_SLOTS):
                slot, kind, sp, du, ac, ph, fa = MOTOR_REC.unpack_from(payload, r * MOTOR_REC.size)
                if slot != r or kind != SLOT_KIND[r]:
                    stats["bad_crc"] += 1   # 布局错位按坏帧计（防静默错解）
                    continue
                traces[r].add(sp, du, ac, ph, fa)
                lo, hi = ranges[r]
                ranges[r] = [min(lo, sp, ac), max(hi, sp, ac)]
            if (not args.quiet and args.render_every > 0
                    and n_motor % args.render_every == 0):
                print("[%6.2fs f=%-6d] %s" % (time.monotonic() - t0, n_motor,
                                              " | ".join(render_row(t, ranges) for t in traces)))
    link.close()

    stats["bad_crc"] += dec.bad_crc
    n_frames = len(traces[0].sp)
    wall = time.monotonic() - t0
    dt_frame = link.frame_dt_s if link.frame_dt_s else wall / max(1, n_frames)
    print(T("cap.demo", n=n_frames, wall=wall, fps=n_frames / wall if wall > 0 else 0.0,
            dt=dt_frame, bad=stats["bad_crc"], gaps=stats["seq_gaps"], text=stats["text"]))

    if not args.no_plot:
        for tr in traces:
            print(plot_ascii(tr, dt=dt_frame))

    fails = check_traces(traces, stats)
    if fails:
        print("\n" + T("fail.head.demo", n=len(fails)))
        for f in fails:
            print("FAIL " + f)
        return 1
    print("\n" + T("pass.demo", n=n_frames))
    return 0


def main() -> int:
    global LANG
    ap = argparse.ArgumentParser(
        description="电机示波器（sim 合成帧 / qemu TCP 桥 / real 串口；纯标准库）")
    ap.add_argument("--source", choices=("sim", "qemu", "real"), default="sim",
                    help="数据源（缺省 sim = 本机合成帧，零外部依赖；qemu = TCP 桥；real = 真串口）")
    ap.add_argument("--lang", choices=("zh", "en"), default="zh", help="输出语言（缺省 zh）")
    ap.add_argument("--host", default="127.0.0.1", help="qemu 档主机（缺省 127.0.0.1）")
    ap.add_argument("--port", type=int, default=5558, help="qemu 档 TCP 端口（缺省 5558）")
    ap.add_argument("--serial", default=None, help="real 档串口名（如 COM5）")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="real 档波特率（缺省 115200）")
    ap.add_argument("--seconds", type=float, default=None,
                    help="采集时长（demo 缺省 30s；场景缺省取各场景推荐时长）")
    ap.add_argument("--render-every", type=int, default=125)
    ap.add_argument("--quiet", action="store_true", help="抑制滚动渲染（断言与波形仍输出）")
    ap.add_argument("--no-plot", action="store_true", help="跳过终段 ASCII 波形")
    ap.add_argument("--scenario", choices=sorted(SCEN.keys()), default=None,
                    help="T-112 四专项场景（dc/enc/nested/pwm/servo）；缺省 = demo 模式")
    args = ap.parse_args()
    LANG = args.lang

    errs = text_table_errors()
    if errs:                       # 文案表自检：zh/en 成对是硬约束（缺一即 bug）
        for e in errs:
            print("FAIL text table: " + e)
        return 2

    print(T("banner.title", source=args.source, lang=args.lang))
    print(describe_source(args))
    try:
        link = open_source(args)
    except SourceError as exc:
        print(T(exc.key, **exc.kw))
        return 2
    print(T("conn.ok", desc=source_endpoint(args)))

    if args.scenario is not None:
        return run_scenario(args, link)
    return run_demo(args, link)


if __name__ == "__main__":
    sys.exit(main())
