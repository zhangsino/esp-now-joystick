#!/usr/bin/env python3
"""在 Mac 上按 receiver/main/main.c 的坐标和字体重画一遍屏幕，用来先看效果再烧录。

用法：
    python3 tools/preview_ui.py            # 出一张 /tmp/ui_preview.png
    python3 tools/preview_ui.py --x 660 --y -330 --z -200 --rz 400 --btn 0x50
"""
import argparse
import os
import re

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FONT_C = os.path.join(HERE, "..", "common", "font6x8.c")


def load_font():
    glyphs = []
    for line in open(FONT_C, encoding="utf-8"):
        m = re.match(r"\s*\{((?:0x[0-9a-f]{2},?){6})\}", line)
        if m:
            glyphs.append([int(v, 16) for v in m.group(1).split(",") if v])
    assert len(glyphs) == 95, len(glyphs)
    return glyphs


FONT = load_font()
W, H = 128, 64


class Canvas:
    def __init__(self):
        self.px = [[0] * W for _ in range(H)]

    def pixel(self, x, y):
        if 0 <= x < W and 0 <= y < H:
            self.px[y][x] = 1

    def hline(self, x, y, w):
        for i in range(w):
            self.pixel(x + i, y)

    def vline(self, x, y, h):
        for i in range(h):
            self.pixel(x, y + i)

    def circle(self, cx, cy, r):
        x, y, err = r, 0, 1 - r
        while x >= y:
            for dx, dy in ((x, y), (y, x), (-y, x), (-x, y),
                           (-x, -y), (-y, -x), (y, -x), (x, -y)):
                self.pixel(cx + dx, cy + dy)
            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1

    def disc(self, cx, cy, r):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self.pixel(cx + dx, cy + dy)

    def text(self, x, y, s):
        for ch in s:
            c = ord(ch)
            if c < 32 or c > 126:
                c = 32
            g = FONT[c - 32]
            for col in range(6):
                for row in range(8):
                    if g[col] >> row & 1:
                        self.pixel(x + col, y + row)
            x += 6

    def text_center(self, y, s):
        self.text((W - len(s) * 6) // 2, y, s)

    def image(self):
        img = Image.new("1", (W, H), 0)
        p = img.load()
        for y in range(H):
            for x in range(W):
                if self.px[y][x]:
                    p[x, y] = 1
        return img


FULL = 660
R, LX, RX, CY = 15, 32, 96, 25


def render(x, y, z, rz, btn, dial=0, pct=100, rssi=8, lost=0, linked=True,
           message=None):
    c = Canvas()
    if message:
        c.text_center(20, message[0])
        c.text_center(34, message[1])
        return c.image()
    c.text(0, 0, "%3d%%%+5ddB" % (pct, rssi) if linked else "NO LINK")
    c.text(70, 0, "D%+4d" % dial)
    if linked:
        c.text(104, 0, "L%d" % min(lost, 999))
    if linked:
        for cx, ax, ay in ((LX, rz, z), (RX, x, y)):
            c.circle(cx, CY, R)
            c.hline(cx - 3, CY, 7)
            c.vline(cx, CY - 3, 7)
            dx = max(-(R - 2), min(R - 2, int(ax * (R - 2) / FULL)))
            dy = max(-(R - 2), min(R - 2, int(ay * (R - 2) / FULL)))
            c.disc(cx + dx, CY - dy, 2)
        for i in range(24):
            col, row = i % 12, i // 12
            px_, py_ = 8 + col * 10, 47 if row == 0 else 58
            if btn >> i & 1:
                c.disc(px_, py_, 3)
            else:
                c.pixel(px_, py_)
    return c.image()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=int, default=330)
    ap.add_argument("--dial", type=int, default=12)
    ap.add_argument("--y", type=int, default=-260)
    ap.add_argument("--z", type=int, default=-200)
    ap.add_argument("--rz", type=int, default=180)
    ap.add_argument("--btn", type=lambda s: int(s, 0), default=0x50)
    ap.add_argument("--out", default="/tmp/ui_preview.png")
    a = ap.parse_args()

    a_img = render(a.x, a.y, a.z, a.rz, a.btn, dial=a.dial)
    b_img = render(0, 0, 0, 0, 0, dial=0)
    c_img = render(0, 0, 0, 0, 0, linked=False)

    scale = 4
    gap = 12
    out = Image.new("1", (W * scale * 3 + gap * 2, H * scale + 24), 0)
    for i, img in enumerate((a_img, b_img, c_img)):
        out.paste(img.resize((W * scale, H * scale), Image.NEAREST), (i * (W * scale + gap), 24))
    out.save(a.out)
    print("written", a.out)


if __name__ == "__main__":
    main()
