from PIL import Image, ImageDraw, ImageFont
chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz+-.:%[]abcdB"
fonts = []
def add(name, path, size):
    try:
        fonts.append((name, ImageFont.truetype(path, size)))
    except Exception as e:
        print("skip", name, e)
add("AileronDef8", None, 8)
add("Menlo9", "/System/Library/Fonts/Menlo.ttc", 9)
add("Courier9", "/System/Library/Fonts/Supplemental/Courier New.ttf", 9)
add("AndaleMono10", "/System/Library/Fonts/Supplemental/Andale Mono.ttf", 10)
add("Monaco9", "/System/Library/Fonts/Monaco.ttf", 9)

cw, ch = 6, 8
rows = []
for name, f in fonts:
    img = Image.new("L", (len(chars)*cw, ch), 0)
    d = ImageDraw.Draw(img)
    for i, c in enumerate(chars):
        d.text((i*cw, 0), c, fill=255, font=f)
    rows.append((name, img))
out = Image.new("L", (len(chars)*cw, ch*len(rows)*4), 0)
for i,(name,img) in enumerate(rows):
    b = img.point(lambda v: 255 if v>110 else 0)
    out.paste(b, (0, i*ch*4))
    out.paste(b.resize((len(chars)*cw*4, ch*4), Image.NEAREST), (0, i*ch*4))
out = out.resize((out.width*3, out.height*3), Image.NEAREST)
out.save("/tmp/font_try.png")
print("\n".join(n for n,_ in rows))
