#!/usr/bin/env python3
"""RC2 上机自检 —— 量报告率、看四轴和 24 位按键。

比 rc2_sticks.py 少一个用途：这个只回答"现在通不通、多少 Hz"，
适合烧板子之前跑一遍，和板子上的日志对照。

    python3 ~/esp32_now/tools/rc2_check.py            # 3 秒
    python3 ~/esp32_now/tools/rc2_check.py -s 10      # 10 秒
"""
import argparse
import struct
import sys
import time

try:
    import hid
except ImportError:
    sys.exit("需要 hidapi：python3 -m pip install hidapi")

VID, PID = 0x2CA3, 0x1020
REPORT_LEN = 13


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--seconds", type=float, default=3.0)
    a = ap.parse_args()

    devs = list(hid.enumerate(VID, PID))
    if not devs:
        sys.exit("没找到 RC2：确认遥控器已开机、Type-C 插在 Mac 上、线是数据线")
    for d in devs:
        print(f"  iface={d.get('interface_number')} usage_page={d.get('usage_page')} "
              f"usage={d.get('usage')}")

    dev = hid.device()
    dev.open(VID, PID)
    dev.set_nonblocking(1)

    t0 = time.time()
    stamps = []
    last = None
    b_seen = set()
    axes = [0, 0, 0, 0]

    while time.time() - t0 < a.seconds:
        raw = dev.read(64, 20)
        if not raw:
            continue
        now = time.time()
        stamps.append(now)
        data = bytes(raw)
        if len(data) >= REPORT_LEN:
            axes = [struct.unpack_from("<h", data, off)[0] for off in (3, 5, 7, 9)]
            b = int.from_bytes(data[0:3], "little")
            last = b
            for i in range(24):
                if b >> i & 1:
                    b_seen.add(i)

    dev.close()

    if len(stamps) < 2:
        sys.exit("收到 0 帧：设备在，但没出报告")

    span = stamps[-1] - stamps[0]
    gaps = [stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)]
    gaps.sort()
    hz = (len(stamps) - 1) / span
    print(f"\n报告数 {len(stamps)}  /  {span:.3f} s  ->  {hz:.1f} Hz")
    print(f"间隔 中位 {gaps[len(gaps)//2]*1000:.2f} ms  "
          f"P95 {gaps[int(len(gaps)*0.95)]*1000:.2f} ms  "
          f"最大 {gaps[-1]*1000:.2f} ms")
    print(f"四轴 X={axes[0]:+5d} Y={axes[1]:+5d} Z={axes[2]:+5d} Rz={axes[3]:+5d}")
    print(f"按键 0x{last:06x}  常亮/按住的位: {sorted(b_seen) if b_seen else '无'}")


if __name__ == "__main__":
    main()
