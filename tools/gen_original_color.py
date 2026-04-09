#!/usr/bin/env python3
"""
gen_original_color.py  --  Render the tb_enc_top synthetic test pattern as a PNG.

Reproduces the exact pixel formulas from tb_enc_top.vhd (the fallback pattern
used when YUV_FILE=""), converts YUV 4:2:0 -> RGB, and saves original_color.png
in the current directory.

Usage:
  python gen_original_color.py [width] [height]
  Defaults: 64x48 (tb_enc_top defaults)
"""

import sys
from PIL import Image

W = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
H = int(sys.argv[2]) if len(sys.argv) > 2 else 768
cW = W // 2
cH = H // 2

# ---- Y plane: diagonal ramp (same as tb_enc_top.vhd) -------------------------
Y = []
for row in range(H):
    for col in range(W):
        i = row * W + col
        Y.append((row * 4 + col * 2) % 256)

# ---- Cb plane: horizontal ramp 100..227 --------------------------------------
Cb = []
for row in range(cH):
    for col in range(cW):
        Cb.append(100 + col * 127 // (cW - 1))

# ---- Cr plane: diagonal ramp 80..200 -----------------------------------------
Cr = []
for row in range(cH):
    for col in range(cW):
        Cr.append(80 + (row * 3 + col * 5) % 121)

# ---- YUV 4:2:0 -> RGB (BT.601 full-range) ------------------------------------
img = Image.new('RGB', (W, H))
pixels = img.load()

def clamp(v):
    return max(0, min(255, int(v)))

for py in range(H):
    for px in range(W):
        y  = Y[py * W + px]
        cb = Cb[(py // 2) * cW + (px // 2)]
        cr = Cr[(py // 2) * cW + (px // 2)]

        cb -= 128
        cr -= 128

        r = clamp(y + 1.402  * cr)
        g = clamp(y - 0.344136 * cb - 0.714136 * cr)
        b = clamp(y + 1.772  * cb)

        pixels[px, py] = (r, g, b)

out = 'original_color.png'
img.save(out)
print(f"Saved {out}  ({W}x{H})")
