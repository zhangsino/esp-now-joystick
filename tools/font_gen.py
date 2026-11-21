# 生成 6x8 点阵字体（ASCII 32..126）—— 用 macOS Monaco 9 渲染后二值化
from PIL import Image, ImageDraw, ImageFont
FONT = ImageFont.truetype("/System/Library/Fonts/Monaco.ttf", 9)
CW, CH = 6, 8
first, last = 32, 126
glyphs = []
for code in range(first, last+1):
    img = Image.new("L", (CW, CH), 0)
    d = ImageDraw.Draw(img)
    d.text((0, -1), chr(code), fill=255, font=FONT)
    b = img.point(lambda v: 255 if v > 110 else 0)
    cols = []
    for x in range(CW):
        col = 0
        for y in range(CH):
            if b.getpixel((x, y)):
                col |= (1 << y)
        cols.append(col)
    glyphs.append(cols)

with open("font6x8.c", "w") as f:
    f.write("/* 自动生成：tools/font_gen.py（macOS Monaco 9 二值化），ASCII 32..126 */\n")
    f.write("#include \"font6x8.h\"\n\n")
    f.write("const uint8_t font6x8[%d][6] = {\n" % (last-first+1))
    for i, g in enumerate(glyphs):
        f.write("    {0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x}, /* %3d %s */\n" %
                (g[0],g[1],g[2],g[3],g[4],g[5], first+i,
                 repr(chr(first+i)).strip("'").replace("/*","").replace("*/","")))
    f.write("};\n")

# 预览图：全部字符 + 一行示例
allc = "".join(chr(c) for c in range(first, last+1))
demo = "NOW 50Hz R-45  X+660 Y-660 Z+100 RZ-002  B 0x000050  bit 7  LINK LOST"
cols = max(len(allc), len(demo))
img = Image.new("L", (cols*CW, 20), 0)
fb = img.load()
def put(x, y, s):
    for i, ch in enumerate(s):
        g = glyphs[ord(ch)-first]
        for cx in range(CW):
            for cy in range(CH):
                if g[cx] >> cy & 1:
                    fb[x+i*CW+cx, y+cy] = 255
put(0, 0, allc)
put(0, 10, demo)
img.resize((img.width*4, img.height*4), Image.NEAREST).save("/tmp/font_preview.png")
print("ok", len(glyphs))
