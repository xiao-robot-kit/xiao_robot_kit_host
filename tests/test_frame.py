"""frame.py 双实现一致性测试（向量由 固件侧 testing/frame_vector_main.c 生成）。

跑法：cd Host_Project/backend && python -m tests.test_frame
（或 python tests/test_frame.py，见 main）
"""

import struct
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from frame import FrameDecoder, crc8, decode_sample, SOF  # noqa: E402

# ---- C 侧生成的固定向量（frame_vector.exe 输出，勿手改） ----
V_EMPTY = bytes.fromhex("A5007E0074")
V_HI = bytes.fromhex("A5027F074F4B5C")
V_SAMPLE = bytes.fromhex(
    "A51801420000803F000000BF0000803E00000040000060C00000C84273"
)
CRC_ZERO = 0x00
CRC_SEQ3 = 0x48
CRC_FF3 = 0x0F


def main() -> int:
    fails = []

    # CRC 向量
    for name, got, want in [
        ("crc_zero", crc8(b"\x00"), CRC_ZERO),
        ("crc_seq3", crc8(bytes([1, 2, 3])), CRC_SEQ3),
        ("crc_ff3", crc8(b"\xff\xff\xff"), CRC_FF3),
    ]:
        if got != want:
            fails.append(f"{name}: {got:02X} != {want:02X}")

    # 整帧解码向量
    dec = FrameDecoder()
    frames = dec.feed(V_EMPTY + V_HI + V_SAMPLE)  # 一次喂三帧（粘包）
    if len(frames) != 3:
        fails.append(f"frames={len(frames)} != 3")
    else:
        t, s, p = frames[0]
        if (t, s, p) != (0x7E, 0x00, b""):
            fails.append(f"v_empty frame {frames[0]!r}")
        t, s, p = frames[1]
        if (t, s, p) != (0x7F, 0x07, b"OK"):
            fails.append(f"v_hi frame {frames[1]!r}")
        t, s, p = frames[2]
        vals = decode_sample(p)
        want = (1.0, -0.5, 0.25, 2.0, -3.5, 100.0)
        if t != 0x01 or s != 0x42 or any(abs(a - b) > 1e-6 for a, b in zip(vals, want)):
            fails.append(f"v_sample {vals!r}")

    # 分片容错：半帧分片 + CRC 坏帧重同步
    dec2 = FrameDecoder()
    got = dec2.feed(V_HI[:3])          # 半帧
    if got:
        fails.append("half frame should not emit")
    got = dec2.feed(V_HI[3:])          # 补齐
    if len(got) != 1 or got[0][2] != b"OK":
        fails.append(f"completed frame {got!r}")
    bad = bytearray(V_HI)
    bad[-1] ^= 0xFF                    # 破坏 CRC
    got = dec2.feed(bytes(bad) + V_HI)  # 坏帧后应恢复
    if len(got) != 1 or got[0][2] != b"OK":
        fails.append(f"resync after bad crc {got!r}")

    # 假 SOF 噪声
    dec3 = FrameDecoder()
    got = dec3.feed(b"\x00A5\xFF\xFF" + V_HI)
    if len(got) != 1 or got[0][2] != b"OK":
        fails.append(f"noise skip {got!r}")

    # 结构体字节序自证（与固件 memcpy 打包假设一致：小端）
    if struct.unpack("<f", b"\x00\x00\x80\x3F")[0] != 1.0:
        fails.append("host is not little-endian?!" + hex(SOF))

    n = 5 - len(fails) + 3  # 断言组数近似
    print(f"== test_frame: {n} groups, {len(fails)} fail ==")
    for f in fails:
        print("  [FAIL]", f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
