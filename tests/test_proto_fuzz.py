# -*- coding: utf-8 -*-
"""proto 协议注入回归（exp_proto / T-112 倒数第二项；纯自研，零第三方依赖）。

被测件：backend/frame.py 的 FrameDecoder（流式解码）+ crc8。
契约单源：固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md） ——
    SOF(0xA5) | LEN | TYPE | SEQ | PAYLOAD(LEN B) | CRC8（poly 0x07 初值 0x00，
    覆盖 LEN..PAYLOAD 不含 SOF）；PROTO_PAYLOAD_MAX=96；帧类型 0x01/0x02/0x04/
    0x05/0x06/0x7E/0x7F（0x06 为 frame.h 在册类型，一并覆盖）。

注入用例：
  1) 半帧注入：canonical 帧组逐帧在【每一个】字节位截断——
     1a 分片补齐：前缀/后缀分两次喂 → 该帧恰解出一次 + 后续帧零丢失；
     1b 尾巴丢失：余段不再发送、直接续传后续帧 + 冲刷帧 → 后续帧零丢失（流式恢复）。
  2) 坏 CRC / 坏 LEN / 假 SOF 噪声：注入后跟合法帧 → 重同步成功且合法帧全收。
  3) 乱序与重复 seq：0xFF→0x00 回绕边界 + 重复帧 + 真丢帧 + seq 回跳 →
     参考丢帧统计口径不误报（差分式镜像 scope_motor.py 生产用的 (seq-prev)%256
     回绕式，加显式分类）。
  4) 帧类型全集：按 frame.h 各类型注释布局造帧 → TYPE/SEQ 透传 + payload
     长度守恒（0x05 payload 64B = 4×16B 记录 → 整帧 69B；0x02 payload 76B
     = 19×f32；0x06 = 3+6·n_tim+16·n_rec）。
  5) 压力：100,000 帧背靠背连续流 + 随机 1% 比特错误（每坏帧恰 1 比特）→
     好帧全收、坏帧全弃（零静默污染：产出与发送帧逐字节身份比对，含顺序）。
     附 20k 帧把 LEN 字节也纳入翻转的补充通过性检查（断言取碰撞下的守恒面）。
  6) 回归基线：进程内重跑 test_frame.main()（C 侧固定向量，防规格漂移）。

确定性论证（本测试的断言为何是精确断言而非概率断言）：
  - canonical 帧的 payload 字节取 0x00..0x1F、SEQ 避开 0xA5：帧内不含 SOF
    字节值；frame.py 对坏帧只 del buf[:1] 再找 SOF（后续帧头仍在缓冲区内，
    不被吞掉），故重同步扫描必直达下一真帧头；
  - 单比特错误落在 CRC 覆盖域（TYPE/SEQ/PAYLOAD/CRC 字节）时，CRC-8(poly
    0x07) 对单比特差错全检出（poly 常数项非零 ⇒ 不整除任何 x^k）⇒ 坏帧必
    被弃——代数性质，不是 255/256 概率；
  - 用例 5 主压力的随机翻转位避开 LEN 字节（帧内索引 1）：LEN 被破坏后 CRC
    校验面对任意垃圾跨度，8 位 CRC 存在不可消除的 1/256 假接受概率——协议
    层极限而非解码器缺陷（任何 8 位 CRC 帧协议都有，与解码器写法无关）；
    坏 LEN 注入由用例 2 以确定性向量覆盖。排除 LEN 后「零污染 + 好帧全收」
    为可证断言；LEN 纳入翻转的碰撞面由 5b 子用例以守恒断言覆盖。
  - 用例 1b 每个截断位会发生一次「跨帧垃圾跨度 vs 后续帧字节」的 CRC 评估，
    同样有 1/256 假接受概率；1b 因此按协议语义对每个截断位精确二分断言
    （常规位=后续帧零丢失；假接受位=幽灵帧 + 尾部恢复，期望值由测试侧独立
    推演），与种子无关。5b 子用例的 LEN 翻转同属该极限面，断言取守恒口径
    （回收率下限 + 零静默污染）。

跑法：python tests/test_proto_fuzz.py（或 cd backend && python -m tests.test_proto_fuzz）
退出码：0 = 全部通过；1 = 有 FAIL（明细打印）。
"""

import os
import random
import struct
import sys
import time
from collections import Counter

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "backend"))
sys.path.insert(0, _HERE)

import frame as frame_mod  # noqa: E402  被测模块（常量漂移守卫用）
from frame import FrameDecoder, crc8, decode_quat, decode_sample  # noqa: E402
import test_frame  # noqa: E402  回归基线（同目录；import 时自挂 backend 路径）

# ── 契约常量（单源 = 固件侧 proto/frame.h（公开协议见 xiao_robot_kit_qemu docs/protocol-spec.md））──
SOF = 0xA5
TYPE_IMU_SAMPLE = 0x01   # payload 24B = 6×f32（ax..gz，g/dps）
TYPE_IMU_CAL = 0x02      # payload 76B = 19×f32（原始/补偿后/零偏各 6 轴 + 温度）
TYPE_QUAT = 0x04         # payload 16B = 4×f32，w 在前
TYPE_MOTOR = 0x05        # payload 64B = 4×16B 记录（3→4 槽扩 DC，T-113）→ 整帧 69B
TYPE_SCEN = 0x06         # payload = 3 + 6·n_tim + 16·n_rec ≤ 63B
TYPE_ACK = 0x7E          # payload 1B 状态码
TYPE_TEXT = 0x7F         # payload ASCII，可为空（frame.c 允许 len==0）
CONTRACT_PAYLOAD_MAX = 96  # frame.h PROTO_PAYLOAD_MAX（解码器侧必须同值）

MOTOR_REC = struct.Struct("<BBfffBB")   # 16B/记录：slot,kind,setpoint,duty,actual,phase,fault
SCEN_TIMER = struct.Struct("<BBI")      # 6B/快照：tim_idx,channels,freq_hz

# canonical 帧组（用例 1/2/3/5 的载体）：payload 取受限字母表 0x00..0x1F，
# 使帧内不出现 SOF 字节值（重同步扫描必直达下一真帧头，见模块 docstring）
CANON = (
    (TYPE_IMU_SAMPLE, 24),
    (TYPE_IMU_CAL, 76),
    (TYPE_QUAT, 16),
    (TYPE_MOTOR, 64),
    (TYPE_SCEN, 63),
    (TYPE_ACK, 1),
    (TYPE_TEXT, 9),
    (TYPE_TEXT, 0),   # LEN=0 最小帧（5B）：截断扫描也覆盖
)
# SEQ 取值避开 0xA5（0xFF→0x00 回绕仍在环上）
SEQ_CYCLE = [s for s in range(256) if s != SOF]

RNG_SEED = 20261106      # 向量种子（1b 已碰撞感知、与种子无关；5b 的 LEN 翻转面
                         # 若换种子后见 1~2 帧污染，属 1/256 协议层极限，见 docstring）

STRESS_FRAMES = 100_000  # 用例 5 主压力规模（任务卡：10 万帧）
STRESS_RATE = 0.01       # 随机 1% 比特错误（每坏帧恰 1 比特）
STRESS_CHUNK = 4096      # 分片喂入粒度（串口读缓冲量级）
STRESS2_FRAMES = 20_000  # 5b 补充子用例规模（LEN 字节也纳入翻转）


# ── 测试侧工具 ────────────────────────────────────────────────────────────

def _make_crc8_table():
    """CRC-8(0x07) 查表（与 frame.crc8 位级实现等价；启动时锚定校验）。"""
    tbl = []
    for v in range(256):
        c = v
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        tbl.append(c)
    return tbl


_CRC8_TABLE = _make_crc8_table()


def crc8_fast(data):
    """查表 CRC-8（纯提速用；等价性由 main() 开头的锚定校验保证）。"""
    tbl = _CRC8_TABLE
    c = 0
    for b in data:
        c = tbl[c ^ b]
    return c


def encode_frame(ftype, seq, payload):
    """与固件 proto_frame_encode 同规格的测试侧编码器（帧字节 → 解码对照）。"""
    if len(payload) > CONTRACT_PAYLOAD_MAX:
        raise ValueError("payload %dB > 契约上限 %dB" % (len(payload), CONTRACT_PAYLOAD_MAX))
    head = bytes((SOF, len(payload), ftype, seq & 0xFF))
    return head + payload + bytes((crc8_fast(head[1:] + payload),))


def canon_frames():
    """canonical 帧组：[(type, seq, payload, 帧字节)]，内容由 RNG_SEED 确定。"""
    out = []
    for i, (ftype, plen) in enumerate(CANON):
        rnd = random.Random(RNG_SEED * 1000003 + i * 1009 + ftype)
        payload = bytes([rnd.randrange(0x20) for _ in range(plen)])
        seq = SEQ_CYCLE[i]
        out.append((ftype, seq, payload, encode_frame(ftype, seq, payload)))
    return out


def f32(x):
    """float64 字面量 → f32 量化值（payload 契约即 f32；回读比对须同量化）。"""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def first_diff(got, want):
    """诊断：产出/期望序列的第一处差异（截断化描述，避免刷屏）。"""
    for j in range(min(len(got), len(want))):
        if got[j] != want[j]:
            g, w = got[j], want[j]
            return ("idx %d: got(t=%02X,s=%02X,p=%dB) want(t=%02X,s=%02X,p=%dB)"
                    % (j, g[0], g[1], len(g[2]), w[0], w[1], len(w[2])))
    if len(got) != len(want):
        return "len %d != %d" % (len(got), len(want))
    return "equal"


# ── 用例 1：半帧注入 ─────────────────────────────────────────────────────

def sub1a(fails):
    """1a 分片补齐：每帧在每个字节位切成两段分喂 → 恰解一次 + 后续零丢失。"""
    frames = canon_frames()
    n = 0
    for i, (ftype, seq, payload, fb) in enumerate(frames):
        rest = frames[i + 1:]
        rest_bytes = b"".join(f[3] for f in rest)
        want = [(ftype, seq, payload)] + [(f[0], f[1], f[2]) for f in rest]
        for k in range(1, len(fb)):
            n += 1
            dec = FrameDecoder()
            got = dec.feed(fb[:k])
            if got:
                fails.append("1a [t=%02X #%d k=%d] 半帧前缀不应产出，得 %r"
                             % (ftype, i, k, got[:2]))
                continue
            got = dec.feed(fb[k:] + rest_bytes)
            if got != want:
                fails.append("1a [t=%02X #%d k=%d] 分片补齐后 %d 帧 != 期望 %d（%s）"
                             % (ftype, i, k, len(got), len(want), first_diff(got, want)))
    return n


def sub1b(fails):
    """1b 尾巴丢失：截断余段不再发送、续传后续帧 + 冲刷帧 → 后续帧零丢失。

    冲刷帧（0x02 81B + 0x7E 6B）保证挂起候选必被评估而不留尾巴：挂起候选
    在流式语义下会暂时挡住其后帧（字节有序，不能跳帧定夺），补足字节后即
    释放——这正是「流式恢复」的考察点。被截断帧本身允许丢失（余段未到）。

    断言按协议语义对每个 (帧, 截断位) 精确二分（与种子无关）：
      - 常规位（跨帧垃圾跨度的 CRC 评估不通过，255/256）：截断帧被弃、
        后续帧 + 冲刷帧全收零丢失；
      - CRC 假接受位（8 位 CRC 协议层极限，每处 1/256）：解码器按其声明的
        重同步语义（frame.py：坏帧只丢 1 字节重找 SOF）产出——幽灵帧吞掉
        假帧跨度后从下一真帧头恢复。期望值由本测试按同一协议语义独立推演
        （非复用被测代码），逐帧比对。
    """
    frames = canon_frames()
    flush = [frames[1], frames[5]]
    n = 0
    n_phantom = 0
    for i, (ftype, seq, payload, fb) in enumerate(frames):
        total = len(fb)
        prefix = frames[:i]
        prefix_bytes = b"".join(f[3] for f in prefix)
        prefix_tuples = [(f[0], f[1], f[2]) for f in prefix]
        S = frames[i + 1:] + flush            # 续传流：后续帧 + 冲刷帧（背靠背）
        S_bytes = b"".join(f[3] for f in S)
        S_tuples = [(f[0], f[1], f[2]) for f in S]
        starts = []                            # S 内各帧起始偏移
        off = 0
        for f in S:
            starts.append(off)
            off += len(f[3])
        for k in range(1, total):
            n += 1
            buf = fb[:k] + S_bytes             # 第二次喂入后解码器缓冲的镜像
            if k == 1 or crc8_fast(buf[1:total - 1]) != buf[total - 1]:
                want = prefix_tuples + S_tuples
            else:
                # CRC 假接受：幽灵帧 = (TYPE, SEQ, 假跨度 payload)，吞 buf[:total]
                n_phantom += 1
                phantom = (buf[2], buf[3], buf[4:total - 1])
                j = 0
                while starts[j] < total - k:   # 吞掉区后的第一个真帧头
                    j += 1
                want = prefix_tuples + [phantom] + S_tuples[j:]
            dec = FrameDecoder()
            got = dec.feed(prefix_bytes + fb[:k]) + dec.feed(S_bytes)
            if got != want:
                fails.append("1b [t=%02X #%d k=%d] 尾巴丢失续传 %d 帧 != 期望 %d"
                             "（%s）"
                             % (ftype, i, k, len(got), len(want), first_diff(got, want)))
    print("  [sub1b] %d 组（其中 CRC 假接受位 %d 个——协议层极限路径已实覆盖）"
          % (n, n_phantom))
    return n


# ── 用例 2：坏 CRC / 坏 LEN / 假 SOF 噪声 → 重同步 + 合法帧全收 ─────────

def sub2(fails):
    frames = canon_frames()
    victim = frames[0]                       # 0x01 帧（29B，真实 LEN=24）
    vt = (victim[0], victim[1], victim[2])
    follow = [frames[2], frames[5], frames[6]]  # QUAT / ACK / TEXT 三类合法跟随帧
    fbF = b"".join(f[3] for f in follow)
    wF = [(f[0], f[1], f[2]) for f in follow]
    cases = []   # (名, 流字节, 期望产出)

    # 2.1 坏 CRC——两变体：CRC 字节翻 1 位 / payload 首字节翻 1 位（都必被弃）
    for i, (ftype, seq, payload, fb) in enumerate(frames):
        bad_crc = fb[:-1] + bytes((fb[-1] ^ 0x01,))
        cases.append(("badcrc-crcbyte t=%02X" % ftype,
                      bad_crc + fb + fbF, [(ftype, seq, payload)] + wF))
        if len(payload) >= 1:
            bad_pay = fb[:4] + bytes((fb[4] ^ 0x40,)) + fb[5:]
            cases.append(("badcrc-payload t=%02X" % ftype,
                          bad_pay + fb + fbF, [(ftype, seq, payload)] + wF))

    # 2.2 坏 LEN（受害帧 0x01，真实 LEN=24）
    def with_len(new_len):
        return bytes((SOF, new_len)) + victim[3][2:]

    cases.append(("badlen=0xFF(超限)", with_len(0xFF) + victim[3] + fbF, [vt] + wF))
    cases.append(("badlen=0xA5(超限)", with_len(0xA5) + victim[3] + fbF, [vt] + wF))
    cases.append(("badlen=8(偏小)", with_len(8) + victim[3] + fbF, [vt] + wF))
    cases.append(("badlen=0(压零)", with_len(0) + victim[3] + fbF, [vt] + wF))
    # 坏 LEN=96（≤上限的假大值）：候选需 101B 才评估——补一帧把缓冲推过去，
    # 断言跨帧等待后仍重同步成功且后续帧（含补足帧）全收
    cases.append(("badlen=96(假大值跨帧等待)",
                  with_len(96) + victim[3] + fbF + victim[3], [vt] + wF + [vt]))

    # 2.3 假 SOF 噪声
    cases.append(("fakesof-前导噪声", b"\x00\x11\x22\xA5\xFF\x33" + victim[3] + fbF,
                  [vt] + wF))
    cases.append(("fakesof-假整帧坏CRC",
                  b"\xA5\x0A\x7F\x01" + b"\x11" * 10 + b"\x00" + victim[3] + fbF,
                  [vt] + wF))
    cases.append(("fakesof-贴真帧头", b"\xA5" + victim[3] + fbF, [vt] + wF))
    cases.append(("fakesof-洪泛50次", b"\xA5\xFF" * 50 + victim[3] + fbF, [vt] + wF))
    cases.append(("fakesof-混合噪声", b"\xDE\xAD\xA5\x70\xBE\xEF" + victim[3] + fbF,
                  [vt] + wF))

    for name, stream, want in cases:
        dec = FrameDecoder()
        got = dec.feed(stream)
        if got != want:
            fails.append("2 [%s] 产出 %d 帧 != 期望 %d（%s）"
                         % (name, len(got), len(want), first_diff(got, want)))

    # 2.4 跨块假 SOF 头（半截假头挂起 + 下一块真帧）：两段喂入
    dec = FrameDecoder()
    got = dec.feed(b"\x55\xAA\xA5\x04") + dec.feed(victim[3] + fbF)
    if got != [vt] + wF:
        fails.append("2 [fakesof-跨块半截头] 产出 %d 帧 != 期望 %d（%s）"
                     % (len(got), len([vt] + wF), first_diff(got, [vt] + wF)))

    # 2.5 坏 LEN=96 挂起态：字节不足时解码器应等待（零产出、零异常、不丢缓冲）
    dec = FrameDecoder()
    got = dec.feed(with_len(96) + frames[5][3])   # 29+6=35B < 101B：候选挂起
    if got != []:
        fails.append("2 [badlen=96 挂起态] 字节不足不应产出，得 %r" % (got[:2],))
    got = dec.feed(b"")
    if got != []:
        fails.append("2 [badlen=96 挂起态] 空喂不应产出，得 %r" % (got[:2],))

    return len(cases) + 2


# ── 用例 3：乱序与重复 seq → 统计口径不误报 ──────────────────────────────

def seq_drop_stats(seqs):
    """参考丢帧统计口径：镜像 scope_motor.py 生产差分 (seq-prev)%256（回绕式），
    加显式分类——回绕=连续、重复=非丢帧、大回跳=乱序单计且回跳后的首个前跳
    不折算丢帧（跨的是已计区间）。返回 (drops, dups, reorders)。"""
    drops = dups = reorders = 0
    prev = None
    skip_next_drop = False
    for s in seqs:
        if prev is None:
            prev = s
            continue
        d = (s - prev) % 256
        if d == 0:
            dups += 1
        elif d == 1:
            pass                      # 连续（含 0xFF→0x00 回绕：d 恰为 1）
        elif d <= 128:
            if skip_next_drop:
                skip_next_drop = False
            else:
                drops += d - 1        # 正向跳号 = 真丢帧
        else:
            reorders += 1             # 大回跳：乱序单计，不误报 ~250 丢帧
            skip_next_drop = True
        prev = s
    return drops, dups, reorders


def sub3(fails):
    rnd = random.Random(RNG_SEED ^ 0xC0FFEE)
    vectors = [
        # (名, seq 序列, 期望 (drops, dups, reorders))
        ("回绕链 0xFD→0x02", [0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02], (0, 0, 0)),
        ("重复帧", [5, 5, 6], (0, 1, 0)),
        ("真丢帧(敏感性)", [5, 8], (2, 0, 0)),
        ("seq 回跳(乱序)", [10, 3, 11], (0, 0, 1)),
        ("回绕+重复", [0xFE, 0xFE, 0xFF, 0x00], (0, 1, 0)),
    ]
    n = 0
    for name, seqs, want_stats in vectors:
        n += 1
        by_seq = {}
        for s in dict.fromkeys(seqs):          # 去重保序：重复 seq 复用同一帧字节
            payload = bytes([rnd.randrange(0x20) for _ in range(9)])
            by_seq[s] = (payload, encode_frame(TYPE_TEXT, s, payload))
        stream = b"".join(by_seq[s][1] for s in seqs)
        want = [(TYPE_TEXT, s, by_seq[s][0]) for s in seqs]
        got = FrameDecoder().feed(stream)
        if got != want:
            fails.append("3 [%s] 帧保真 %d != %d（重复帧不得被去重；%s）"
                         % (name, len(got), len(want), first_diff(got, want)))
            continue
        stats = seq_drop_stats([f[1] for f in got])
        if stats != want_stats:
            fails.append("3 [%s] 统计口径 (drops,dups,reorders)=%s != 期望 %s"
                         % (name, stats, want_stats))
    return n


# ── 用例 4：帧类型全集（类型透传 + payload 长度守恒 + 布局回读） ─────────

def sub4(fails):
    imu_vals = (1.0, -0.5, 0.25, 2.0, -3.5, 100.0)
    cal_vals = (0.987, -0.012, 0.441, -8.3, 12.7, -0.9,
                0.975, -0.010, 0.438, -7.9, 12.1, -0.8,
                0.012, -0.002, 0.003, -0.4, 0.6, -0.1, 36.6)
    quat = (1.0, 0.0, 0.0, 0.0)
    motor_rows = ((0, 0, 0.50, 0.48, 0.48, 3, 0),
                  (1, 2, 90.0, 0.075, 90.0, 7, 0),
                  (2, 3, 120.0, 0.62, 118.5, 5, 0),
                  (3, 1, -0.50, -0.50, -0.50, 7, 0))   # DC 槽：带符号 duty 域
    motor_payload = b"".join(MOTOR_REC.pack(*r) for r in motor_rows)
    # 回读 oracle：浮点值按 payload 契约（f32）量化——pack 侧同一量化，回读
    # 必须逐位一致；payload 字节本身已由上面的帧保真断言逐字节比对
    cal_q = tuple(f32(v) for v in cal_vals)
    motor_q = tuple((r[0], r[1], f32(r[2]), f32(r[3]), f32(r[4]), r[5], r[6])
                    for r in motor_rows)
    # 0x06 场景帧造帧取前三槽（MOTOR_SCEN_MAX_SLOTS=3 单源；nested 满槽形）
    scen_motor_payload = b"".join(MOTOR_REC.pack(*r) for r in motor_rows[:3])
    scen_payload = (bytes((4, 2)) + SCEN_TIMER.pack(0, 4, 20000)
                    + SCEN_TIMER.pack(1, 4, 50) + bytes((3,)) + scen_motor_payload)
    spec = [
        # (type, seq, payload, 契约 payload 长度, 说明)
        (TYPE_IMU_SAMPLE, 0x42, struct.pack("<6f", *imu_vals), 24, "0x01 6×f32"),
        (TYPE_IMU_CAL, 0x07, struct.pack("<19f", *cal_vals), 76, "0x02 19×f32=76B"),
        (TYPE_QUAT, 0x10, struct.pack("<4f", *quat), 16, "0x04 4×f32 w 前置"),
        (TYPE_MOTOR, 0xA7, motor_payload, 64, "0x05 4×16B → 整帧 69B"),
        (TYPE_SCEN, 0x2C, scen_payload, 63, "0x06 3+6·2+16·3=63B"),
        (TYPE_ACK, 0x01, b"\x00", 1, "0x7E 1B 状态码"),
        (TYPE_TEXT, 0x7F, b"cal done!", 9, "0x7F ASCII"),
        (TYPE_TEXT, 0x80, b"", 0, "0x7F LEN=0（frame.c 允许）"),
    ]
    n = 0

    # 常量漂移守卫：解码器 PAYLOAD_MAX 必须与 frame.h 契约一致
    n += 1
    if frame_mod.PAYLOAD_MAX != CONTRACT_PAYLOAD_MAX:
        fails.append("4 [契约] frame.PAYLOAD_MAX=%d != frame.h PROTO_PAYLOAD_MAX=%d"
                     "（0x02 帧 76B 会被整帧丢弃）"
                     % (frame_mod.PAYLOAD_MAX, CONTRACT_PAYLOAD_MAX))

    # 长度守恒 + 整帧长度（0x05 整帧恰 69B = 4 头 + 64B + 1 CRC）
    for ftype, seq, payload, clen, note in spec:
        n += 1
        fb = encode_frame(ftype, seq, payload)
        if len(payload) != clen or len(fb) != 5 + clen:
            fails.append("4 [%s] 长度不守恒：payload %dB（契约 %dB）/ 整帧 %dB（契约 %dB）"
                         % (note, len(payload), clen, len(fb), 5 + clen))
        if ftype == TYPE_MOTOR and len(fb) != 69:
            fails.append("4 [0x05] 整帧 %dB != 69B（frame.h 0x05：4×16B 记录 + 5 头尾）"
                         % len(fb))

    # 两种喂法（粘包单块 / 逐字节）产出必须一致且逐帧全收
    stream = b"".join(encode_frame(t, s, p) for t, s, p, _, _ in spec)
    want = [(t, s, p) for t, s, p, _, _ in spec]
    n += 1
    got_a = FrameDecoder().feed(stream)
    if got_a != want:
        fails.append("4 [粘包单块] %d 帧 != %d（%s）"
                     % (len(got_a), len(want), first_diff(got_a, want)))
    n += 1
    dec = FrameDecoder()
    got_b = []
    for b in stream:
        got_b += dec.feed(bytes((b,)))
    if got_b != want:
        fails.append("4 [逐字节] %d 帧 != %d（%s）"
                     % (len(got_b), len(want), first_diff(got_b, want)))

    # 布局回读：各类型 payload 按契约解包应与造帧值（经 f32 量化）逐一相等
    n += 1
    if got_a == want:
        by_type = {(t, s): p for t, s, p, _, _ in spec}
        if decode_sample(by_type[(TYPE_IMU_SAMPLE, 0x42)]) != imu_vals:
            fails.append("4 [0x01] decode_sample 回读 != 造帧六轴")
        if struct.unpack("<19f", by_type[(TYPE_IMU_CAL, 0x07)]) != cal_q:
            fails.append("4 [0x02] 19×f32 回读 != 造帧值（f32 量化）")
        if decode_quat(by_type[(TYPE_QUAT, 0x10)]) != quat:
            fails.append("4 [0x04] decode_quat 回读 != 造帧四元数")
        rows = [MOTOR_REC.unpack_from(by_type[(TYPE_MOTOR, 0xA7)], 16 * r)
                for r in range(4)]
        if tuple(rows) != motor_q:
            fails.append("4 [0x05] 4×16B 记录回读 != 造帧记录（f32 量化）")
        sp = by_type[(TYPE_SCEN, 0x2C)]
        if (sp[0] != 4 or sp[1] != 2 or sp[14] != 3 or len(sp) != 63
                or SCEN_TIMER.unpack_from(sp, 2) != (0, 4, 20000)
                or SCEN_TIMER.unpack_from(sp, 8) != (1, 4, 50)):
            fails.append("4 [0x06] 场景头/定时器快照/记录数回读不一致")
        if by_type[(TYPE_ACK, 0x01)] != b"\x00":
            fails.append("4 [0x7E] ACK 状态码回读 != b\\x00")
        if by_type[(TYPE_TEXT, 0x7F)] != b"cal done!" or by_type[(TYPE_TEXT, 0x80)] != b"":
            fails.append("4 [0x7F] TEXT 回读不一致（含 LEN=0 空帧）")
    return n


# ── 用例 5：压力（10 万帧 + 随机 1% 比特错误） ───────────────────────────

def _build_stress(n_frames, seed, rate, flip_len_byte):
    """合成 n_frames 帧背靠背流 + 单比特注入。返回 (流, 发送帧表, 坏帧序号表)。

    每个被选中帧恰翻 1 个随机比特；主用例不翻 LEN 字节（索引 1，理由见模块
    docstring），5b 补充用例放开。
    """
    rnd = random.Random(seed)
    stream = bytearray()
    sent = []
    corrupted = []
    for i in range(n_frames):
        ftype, plen = CANON[i % len(CANON)]
        seq = SEQ_CYCLE[i % len(SEQ_CYCLE)]
        payload = bytes([rnd.randrange(0x20) for _ in range(plen)])
        fb = encode_frame(ftype, seq, payload)
        if rnd.random() < rate:
            j = rnd.randrange(len(fb))
            while j == 1 and not flip_len_byte:
                j = rnd.randrange(len(fb))
            ba = bytearray(fb)
            ba[j] ^= 1 << rnd.randrange(8)
            fb = bytes(ba)
            corrupted.append(i)
        stream += fb
        sent.append((ftype, seq, payload))
    return bytes(stream), sent, corrupted


def _feed_chunked(stream, chunk):
    dec = FrameDecoder()
    out = []
    for off in range(0, len(stream), chunk):
        out += dec.feed(stream[off:off + chunk])
    return out, dec


def sub5(fails):
    t0 = time.monotonic()
    stream, sent, corrupted = _build_stress(STRESS_FRAMES, RNG_SEED ^ 0x5EED,
                                            STRESS_RATE, flip_len_byte=False)
    bad = set(corrupted)
    expected = [sent[i] for i in range(STRESS_FRAMES) if i not in bad]
    emissions, _ = _feed_chunked(stream, STRESS_CHUNK)
    if not 500 <= len(corrupted) <= 1500:
        fails.append("5 [注入率] 坏帧 %d/100000 偏离 1%% 太远（生成器异常？）"
                     % len(corrupted))
    if emissions != expected:
        extra = Counter(emissions) - Counter(expected)     # 产出多于期望 → 污染面
        missing = Counter(expected) - Counter(emissions)   # 期望多于产出 → 丢帧面
        fails.append("5 [10 万帧] 产出 %d != 期望 %d：污染 %d 帧 / 丢失 %d 帧 / 首差 %s"
                     % (len(emissions), len(expected), sum(extra.values()),
                        sum(missing.values()), first_diff(emissions, expected)))
    print("  [sub5] %d 帧（坏 %d，%.2f%%），分片 %dB × %d 喂入，产出 %d/%d 期望，%.1fs"
          % (STRESS_FRAMES, len(corrupted), 100.0 * len(corrupted) / STRESS_FRAMES,
             STRESS_CHUNK, (len(stream) + STRESS_CHUNK - 1) // STRESS_CHUNK,
             len(emissions), len(expected), time.monotonic() - t0))

    # 5b 补充：LEN 字节也纳入翻转（20k 帧）。断言取碰撞下的守恒面：
    #   ① 好帧回收率 ≥ 99.5%；② 零静默污染（产出 ⊆ 发送好帧多重集）。
    #   （LEN 破坏后存在 1/256 的 CRC 假接受协议层极限，见模块 docstring。）
    t1 = time.monotonic()
    stream2, sent2, corrupted2 = _build_stress(STRESS2_FRAMES, RNG_SEED ^ 0x5EED2,
                                               STRESS_RATE, flip_len_byte=True)
    bad2 = set(corrupted2)
    good2 = [sent2[i] for i in range(STRESS2_FRAMES) if i not in bad2]
    emissions2, _ = _feed_chunked(stream2, STRESS_CHUNK)
    matched = sum((Counter(emissions2) & Counter(good2)).values())
    pollution = sum((Counter(emissions2) - Counter(good2)).values())
    if pollution:
        fails.append("5b [LEN 翻转] 静默污染 %d 帧（CRC 假接受——若为 1~2 帧属协议层"
                     "极限，见 docstring；>2 帧需排查）" % pollution)
    recovery = 100.0 * matched / max(1, len(good2))
    if recovery < 99.5:
        fails.append("5b [LEN 翻转] 好帧回收率 %.2f%% < 99.5%%" % recovery)
    print("  [sub5b] %d 帧（坏 %d，LEN 纳入翻转），回收 %.2f%%，污染 %d，%.1fs"
          % (STRESS2_FRAMES, len(corrupted2), recovery, pollution,
             time.monotonic() - t1))
    return 2


# ── 用例 6：回归基线（进程内重跑 C 侧固定向量） ──────────────────────────

def sub6(fails):
    rc = test_frame.main()
    if rc != 0:
        fails.append("6 [基线] test_frame.main() 退出码 %d（C 侧固定向量回归破坏）" % rc)
    return 1


def main() -> int:
    fails = []
    t0 = time.monotonic()

    # 锚定校验：查表 CRC 与被测位级 crc8 同结果（C 侧向量 + canonical 帧 + 随机样）
    rnd = random.Random(RNG_SEED ^ 0xAA)
    anchor = (b"\x00", bytes([1, 2, 3]), b"\xff" * 3,
              test_frame.V_EMPTY, test_frame.V_HI, test_frame.V_SAMPLE)
    anchor += tuple(f[3] for f in canon_frames())
    anchor += tuple(bytes([rnd.randrange(256) for _ in range(64)]) for _ in range(64))
    for a in anchor:
        if crc8_fast(a) != crc8(a):
            fails.append("0 [锚定] 查表 CRC != frame.crc8（测试侧编码器不可信）")
            break

    n1a = sub1a(fails)
    n1b = sub1b(fails)
    n2 = sub2(fails)
    n3 = sub3(fails)
    n4 = sub4(fails)
    n5 = sub5(fails)
    n6 = sub6(fails)

    print("== test_proto_fuzz: 锚定1 / 1a %d / 1b %d / 2 %d / 3 %d / 4 %d / 5 %d"
          " / 6 %d 组, %.1fs, %d fail =="
          % (n1a, n1b, n2, n3, n4, n5, n6, time.monotonic() - t0, len(fails)))
    for f in fails[:30]:
        print("  [FAIL]", f)
    if len(fails) > 30:
        print("  ... 其余 %d 条省略（共 %d fail）" % (len(fails) - 30, len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
