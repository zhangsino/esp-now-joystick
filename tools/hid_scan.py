#!/usr/bin/env python3
"""列出/监视任意 HID 设备 —— 用来判断一个新遥控器（如 DJI RC-N1）能不能走这套链路。

判断标准只有一条：它插上电脑后，是不是枚举成 Usage Page 1 (Generic Desktop)、
Usage 4/5/8（Joystick/GamePad/Multi-axis）的设备。是 → ESP32 那边改 VID/PID 和解析就能读；
不是 → 得先搞清它在哪个接口上吐数据，工作量大得多。

用法：
    python3 ~/esp32_now/tools/hid_scan.py                    # 列出所有 HID 设备
    python3 ~/esp32_now/tools/hid_scan.py --gamepads         # 只列手柄类
    python3 ~/esp32_now/tools/hid_scan.py --watch 0x2CA3 0x1020 -s 5   # 读某个设备 5 秒
    python3 ~/esp32_now/tools/hid_scan.py --watch-auto -s 5  # 自动挑第一个手柄类读 5 秒
"""
import argparse
import sys
import time

try:
    import hid
except ImportError:
    sys.exit("需要 hidapi：python3 -m pip install hidapi")

USAGE_PAGE_GENERIC = 0x01
GAMEPAD_USAGES = {0x04: "Joystick", 0x05: "GamePad", 0x08: "Multi-axis"}


def list_devices(gamepads_only=False):
    devs = list(hid.enumerate())
    rows = []
    for d in devs:
        vid, pid = d.get("vendor_id"), d.get("product_id")
        up, us = d.get("usage_page"), d.get("usage")
        is_gp = us in GAMEPAD_USAGES and up == USAGE_PAGE_GENERIC
        if gamepads_only and not is_gp:
            continue
        rows.append((is_gp, d))
    if not rows:
        print("没有设备" if not gamepads_only else "没有手柄类设备（usage page 1 / usage 4,5,8）")
        return rows

    for is_gp, d in rows:
        flag = "  <== 手柄类" if is_gp else ""
        print(f"VID 0x{d.get('vendor_id', 0):04X}  PID 0x{d.get('product_id', 0):04X}  "
              f"iface={d.get('interface_number')}  usage_page=0x{d.get('usage_page', 0):02X} "
              f"usage=0x{d.get('usage', 0):02X}({GAMEPAD_USAGES.get(d.get('usage'), '-')})"
              f"{flag}")
        print(f"    厂商: {d.get('manufacturer_string')!r}  产品: {d.get('product_string')!r}")
        print(f"    序列号: {d.get('serial_number')!r}")
    return rows


def watch(vid, pid, seconds):
    dev = hid.device()
    try:
        dev.open(vid, pid)
    except Exception as exc:
        sys.exit(f"打不开 {vid:#06x}:{pid:#06x}：{exc}")
    dev.set_nonblocking(1)
    print(f"读 {vid:#06x}:{pid:#06x}，{seconds} 秒（Ctrl-C 退出）")
    t0 = time.time()
    n = 0
    prev = None
    lens = {}
    while time.time() - t0 < seconds:
        data = dev.read(64, 20)
        if not data:
            continue
        b = bytes(data)
        n += 1
        lens[len(b)] = lens.get(len(b), 0) + 1
        if prev is None or b != prev:
            print(f"[{time.time()-t0:6.2f}s] len={len(b):2d}  {b.hex(' ')}")
            prev = b
    dev.close()
    span = time.time() - t0
    print(f"\n收到 {n} 帧 / {span:.2f} s -> {n/span:.1f} Hz   报告长度分布: {lens}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gamepads", action="store_true")
    ap.add_argument("--watch", nargs=2, metavar=("VID", "PID"))
    ap.add_argument("--watch-auto", action="store_true")
    ap.add_argument("-s", "--seconds", type=float, default=5.0)
    a = ap.parse_args()

    if a.watch:
        watch(int(a.watch[0], 0), int(a.watch[1], 0), a.seconds)
        return
    if a.watch_auto:
        rows = [(is_gp, d) for is_gp, d in list_devices() if is_gp]
        if not rows:
            sys.exit("没找到手柄类设备")
        d = rows[0][1]
        watch(d["vendor_id"], d["product_id"], a.seconds)
        return
    list_devices(a.gamepads)


if __name__ == "__main__":
    main()
