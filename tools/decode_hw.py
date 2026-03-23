#!/usr/bin/env python3
"""
decode_hw.py  --  Decoder for the DRN1 hardware encoder bitstream (bs_out.bin)

Bitstream format (produced by enc_top.vhd)
------------------------------------------
  Per frame:
    [1 bit] frame_type  (0=I-frame, 1=P-frame)
    Per 8x8 luma block in raster order (left-to-right, strip-major):
      I-frame block:
        ue(count) + count * se(coeff)   -- residual in zigzag order
      P-frame block:
        skip(1 bit)
        if skip == 0:
          se(mv_dx) + se(mv_dy)         -- motion vector, half-pixel units
          ue(count) + count * se(coeff) -- residual in zigzag order

  Chroma (Cb/Cr): not encoded; decoder writes 128 for both planes.

Usage:
  python decode_hw.py <bs_out.bin> <output.yuv> <width> <height> <qp> [<gop_size>]

Example:
  python decode_hw.py bs_out.bin out.yuv 64 48 28 30

Play back with:
  ffplay -f rawvideo -pixel_format yuv420p -video_size 64x48 -framerate 1 out.yuv
"""

import sys
import struct

# ---------------------------------------------------------------------------
# Fixed-point constants (must match dct.c / dct_hw.c)
# ---------------------------------------------------------------------------
IDCT_CONST_BITS = 13
IDCT_PASS1_BITS = 1

FIX_0_298631336 = 2446
FIX_0_390180644 = 3196
FIX_0_541196100 = 4433
FIX_0_765366865 = 6270
FIX_0_899976223 = 7373
FIX_1_175875602 = 9633
FIX_1_501321110 = 12299
FIX_1_847759065 = 15137
FIX_1_961570560 = 16069
FIX_2_053119869 = 16819
FIX_2_562915447 = 20995
FIX_3_072711026 = 25172

def _idescale(x, n):
    return (x + (1 << (n - 1))) >> n

def _idct_col(d, c):
    """In-place IDCT column pass on column c of 8x8 array d."""
    z2 = d[2][c];  z3 = d[6][c]
    z1 = (z2 + z3) * FIX_0_541196100
    tmp2 = z1 + z3 * (-FIX_1_847759065)
    tmp3 = z1 + z2 * FIX_0_765366865

    tmp0 = (d[0][c] + d[4][c]) << IDCT_CONST_BITS
    tmp1 = (d[0][c] - d[4][c]) << IDCT_CONST_BITS

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2

    z1 = d[7][c] + d[1][c]
    z2 = d[5][c] + d[3][c]
    z3 = d[7][c] + d[3][c]
    z4 = d[5][c] + d[1][c]
    z5 = (z3 + z4) * FIX_1_175875602

    z1 *= -FIX_0_899976223
    z2 *= -FIX_2_562915447
    z3 *= -FIX_1_961570560
    z4 *= -FIX_0_390180644
    z3 += z5;  z4 += z5

    r0 = d[7][c] * FIX_0_298631336
    r1 = d[5][c] * FIX_2_053119869
    r2 = d[3][c] * FIX_3_072711026
    r3 = d[1][c] * FIX_1_501321110

    r0 += z1 + z3
    r1 += z2 + z4
    r2 += z2 + z3
    r3 += z1 + z4

    sh = IDCT_CONST_BITS + IDCT_PASS1_BITS + 3
    d[0][c] = _idescale(tmp10 + r3, sh)
    d[7][c] = _idescale(tmp10 - r3, sh)
    d[1][c] = _idescale(tmp11 + r2, sh)
    d[6][c] = _idescale(tmp11 - r2, sh)
    d[2][c] = _idescale(tmp12 + r1, sh)
    d[5][c] = _idescale(tmp12 - r1, sh)
    d[3][c] = _idescale(tmp13 + r0, sh)
    d[4][c] = _idescale(tmp13 - r0, sh)

def _idct_row(d, r):
    """In-place IDCT row pass on row r of 8x8 array d."""
    row = d[r]
    z2 = row[2];  z3 = row[6]
    z1 = (z2 + z3) * FIX_0_541196100
    tmp2 = z1 + z3 * (-FIX_1_847759065)
    tmp3 = z1 + z2 * FIX_0_765366865

    tmp0 = (row[0] + row[4]) << IDCT_CONST_BITS
    tmp1 = (row[0] - row[4]) << IDCT_CONST_BITS

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2

    z1 = row[7] + row[1]
    z2 = row[5] + row[3]
    z3 = row[7] + row[3]
    z4 = row[5] + row[1]
    z5 = (z3 + z4) * FIX_1_175875602

    z1 *= -FIX_0_899976223
    z2 *= -FIX_2_562915447
    z3 *= -FIX_1_961570560
    z4 *= -FIX_0_390180644
    z3 += z5;  z4 += z5

    r0 = row[7] * FIX_0_298631336
    r1 = row[5] * FIX_2_053119869
    r2 = row[3] * FIX_3_072711026
    r3 = row[1] * FIX_1_501321110

    r0 += z1 + z3
    r1 += z2 + z4
    r2 += z2 + z3
    r3 += z1 + z4

    sh = IDCT_CONST_BITS - IDCT_PASS1_BITS + 3
    row[0] = _idescale(tmp10 + r3, sh)
    row[7] = _idescale(tmp10 - r3, sh)
    row[1] = _idescale(tmp11 + r2, sh)
    row[6] = _idescale(tmp11 - r2, sh)
    row[2] = _idescale(tmp12 + r1, sh)
    row[5] = _idescale(tmp12 - r1, sh)
    row[3] = _idescale(tmp13 + r0, sh)
    row[4] = _idescale(tmp13 - r0, sh)

def idct8(dqcoeff):
    """8x8 inverse DCT. Input/output: list-of-lists [8][8] of int32."""
    d = [list(row) for row in dqcoeff]
    for c in range(8):
        _idct_col(d, c)
    for r in range(8):
        _idct_row(d, r)
    return d

# ---------------------------------------------------------------------------
# Zigzag table (matches common.h ZIGZAG_8x8)
# ---------------------------------------------------------------------------
ZIGZAG_8x8 = [
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
]

# ---------------------------------------------------------------------------
# Quantisation step (matches quant.c / quant_enc.vhd)
# ---------------------------------------------------------------------------
QP_STEP_BASE = [10, 11, 13, 14, 16, 18]

def quant_step(qp):
    qp = max(1, min(51, qp))
    return QP_STEP_BASE[(qp - 1) % 6] << ((qp - 1) // 6)

# ---------------------------------------------------------------------------
# Exp-Golomb bit reader
# ---------------------------------------------------------------------------
class BitstreamReader:
    def __init__(self, data: bytes):
        self.data = data
        self.byte_pos = 0
        self.bit_buf  = 0
        self.bit_count = 0

    def read_bit(self) -> int:
        if self.bit_count == 0:
            if self.byte_pos >= len(self.data):
                return 0
            self.bit_buf   = self.data[self.byte_pos]
            self.byte_pos += 1
            self.bit_count = 8
        bit = (self.bit_buf >> 7) & 1
        self.bit_buf   = (self.bit_buf << 1) & 0xFF
        self.bit_count -= 1
        return bit

    def read_bits(self, n: int) -> int:
        v = 0
        for _ in range(n):
            v = (v << 1) | self.read_bit()
        return v

    def read_ue(self) -> int:
        """Unsigned exp-Golomb."""
        leading = 0
        while self.read_bit() == 0:
            leading += 1
            if leading > 20:
                return 0
        suffix = 0
        for _ in range(leading):
            suffix = (suffix << 1) | self.read_bit()
        return (1 << leading) - 1 + suffix

    def read_se(self) -> int:
        """Signed exp-Golomb."""
        ue = self.read_ue()
        if ue == 0:
            return 0
        if ue & 1:
            return (ue + 1) >> 1
        else:
            return -(ue >> 1)

# ---------------------------------------------------------------------------
# Half-pixel bilinear interpolation (matches halfpel_mc.vhd / predict.c)
# ---------------------------------------------------------------------------
def bilerp_pixel(ref_frame, W, H, rx, ry, xhalf, yhalf):
    """
    Bilinear half-pixel interpolation for a single pixel at integer ref (rx,ry)
    with sub-pixel flags xhalf, yhalf.
    ref_frame: flat list of W*H pixels.
    """
    def get(x, y):
        x = max(0, min(W - 1, x))
        y = max(0, min(H - 1, y))
        return ref_frame[y * W + x]

    p00 = get(rx,     ry)
    p01 = get(rx + 1, ry)      if xhalf else 0
    p10 = get(rx,     ry + 1)  if yhalf else 0
    p11 = get(rx + 1, ry + 1)  if (xhalf and yhalf) else 0

    if not xhalf and not yhalf:
        return p00
    elif xhalf and not yhalf:
        return (p00 + p01 + 1) >> 1
    elif not xhalf and yhalf:
        return (p00 + p10 + 1) >> 1
    else:
        return (p00 + p01 + p10 + p11 + 2) >> 2

def mc_block(ref_frame, W, H, blk_x, blk_y, mv_dx, mv_dy):
    """
    Produce 8x8 block of MC prediction pixels.
    mv_dx/mv_dy are in half-pixel units (same as VHDL me_engine output).
    Returns list-of-lists [8][8] of uint8.
    """
    int_dx = mv_dx >> 1  if mv_dx >= 0 else -((-mv_dx + 1) >> 1)
    int_dy = mv_dy >> 1  if mv_dy >= 0 else -((-mv_dy + 1) >> 1)
    # Python arithmetic right-shift handles sign correctly:
    int_dx = mv_dx >> 1
    int_dy = mv_dy >> 1
    xhalf  = bool(mv_dx & 1)
    yhalf  = bool(mv_dy & 1)
    ref_x  = blk_x + int_dx
    ref_y  = blk_y + int_dy
    pred = []
    for r in range(8):
        row = []
        for c in range(8):
            p = bilerp_pixel(ref_frame, W, H, ref_x + c, ref_y + r, xhalf, yhalf)
            row.append(p)
        pred.append(row)
    return pred

# ---------------------------------------------------------------------------
# Decode one 8x8 luma block residual (with DC DPCM)
# ---------------------------------------------------------------------------
def decode_residual(bs: BitstreamReader, step: int, prev_dc: int = 0):
    """
    Read ue(count) + count*se(coeff), dequantise, IDCT.
    First SE coeff is dc_diff (DPCM); actual DC = dc_diff + prev_dc.
    Returns (residuals 8x8, new_prev_dc).
    """
    count = bs.read_ue()
    qcoeff_flat = [0] * 64
    new_prev_dc = 0
    if count > 0:
        dc_diff = bs.read_se()
        dc_quant = dc_diff + prev_dc
        new_prev_dc = dc_quant
        qcoeff_flat[0] = dc_quant
        for i in range(1, count):
            qcoeff_flat[i] = bs.read_se()

    qcoeff = [[0]*8 for _ in range(8)]
    for z, nat in enumerate(ZIGZAG_8x8):
        r = nat >> 3
        c = nat & 7
        qcoeff[r][c] = qcoeff_flat[z]

    dqcoeff = [[qcoeff[r][c] * step for c in range(8)] for r in range(8)]
    return idct8(dqcoeff), new_prev_dc

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) < 6:
        print("Usage: decode_hw.py <bs_out.bin> <output.yuv> <width> <height> <qp> [<gop_size>]")
        print("Example: decode_hw.py bs_out.bin out.yuv 64 48 28 30")
        sys.exit(1)

    bs_path  = sys.argv[1]
    yuv_path = sys.argv[2]
    W        = int(sys.argv[3])
    H        = int(sys.argv[4])
    QP       = int(sys.argv[5])
    gop_size = int(sys.argv[6]) if len(sys.argv) > 6 else 1

    step = quant_step(QP)
    blk_cols = W // 8
    blk_rows = H // 8
    n_blocks = blk_cols * blk_rows

    print(f"Decoding {bs_path}  {W}x{H}  QP={QP}  step={step}  GOP={gop_size}")

    with open(bs_path, 'rb') as f:
        raw = f.read()
    print(f"Bitstream: {len(raw)} bytes")

    bs = BitstreamReader(raw)

    # Reference frame (flat list, initialised to 128)
    ref_frame = [128] * (W * H)

    output_frames = []
    frame_idx = 0

    # Decode at most gop_size frames (stop early if bitstream exhausted)
    max_frames = gop_size

    for frame_idx in range(max_frames):
        # ---- Frame header ----
        ftype = bs.read_bit()   # 0=I, 1=P
        is_p  = bool(ftype)
        ftype_str = "P" if is_p else "I"
        print(f"  Frame {frame_idx}: {ftype_str}-frame")

        Y  = [[128]*W for _ in range(H)]

        # Intra predictor state (I-frame): per-pixel row-7 and col-7
        # above_store[bx] = 8 pixels of reconstructed row-7 from block above
        above_store  = [[128]*8 for _ in range(blk_cols)]
        left_col_pix = [128]*8   # 8 pixels of reconstructed col-7 from left block
        prev_dc      = 0         # DC DPCM accumulator (reset each frame)

        MODE_DC, MODE_HORIZ, MODE_VERT = 0, 1, 2

        for blk_idx in range(n_blocks):
            bx = blk_idx % blk_cols
            by = blk_idx // blk_cols
            px = bx * 8
            py = by * 8

            if is_p:
                # ---- P-frame block ----
                skip = bs.read_bit()
                if skip:
                    # Copy reference block unchanged
                    for r in range(8):
                        for c in range(8):
                            rx = max(0, min(W - 1, px + c))
                            ry = max(0, min(H - 1, py + r))
                            Y[py + r][px + c] = ref_frame[ry * W + rx]
                else:
                    mv_dx = bs.read_se()   # half-pixel units
                    mv_dy = bs.read_se()
                    pred  = mc_block(ref_frame, W, H, px, py, mv_dx, mv_dy)
                    resid, _ = decode_residual(bs, step, 0)
                    for r in range(8):
                        for c in range(8):
                            Y[py + r][px + c] = max(0, min(255,
                                resid[r][c] + pred[r][c]))
            else:
                # ---- I-frame block ----
                if bx == 0:
                    left_col_pix = [128]*8

                # Read 2-bit intra mode: 0=DC, 1=HORIZ, 2=VERT
                mode = bs.read_bits(2)

                resid, prev_dc = decode_residual(bs, step, prev_dc)

                for r in range(8):
                    for c in range(8):
                        if mode == MODE_VERT:
                            pred_pix = above_store[bx][c]
                        elif mode == MODE_HORIZ:
                            pred_pix = left_col_pix[r]
                        else:
                            pred_pix = 128
                        Y[py + r][px + c] = max(0, min(255,
                            resid[r][c] + pred_pix))

                # Update per-pixel predictors from reconstructed block
                above_store[bx] = [Y[py + 7][px + c] for c in range(8)]
                left_col_pix    = [Y[py + r][px + 7] for r in range(8)]

        # Update reference frame with reconstructed luma
        for r in range(H):
            for c in range(W):
                ref_frame[r * W + c] = Y[r][c]

        output_frames.append(Y)

        # Stop if we've consumed the entire bitstream
        if bs.byte_pos >= len(raw) and bs.bit_count == 0:
            break

    # Write YUV420p planar (all frames)
    with open(yuv_path, 'wb') as f:
        for Y in output_frames:
            # Y plane
            for row in Y:
                f.write(bytes(row))
            # Cb/Cr planes (128 = grey, encoder is luma-only)
            chroma_row = bytes([128] * (W // 2))
            for _ in range(H // 2):
                f.write(chroma_row)
            for _ in range(H // 2):
                f.write(chroma_row)

    n_frames  = len(output_frames)
    yuv_bytes = n_frames * (W * H + 2 * (W // 2) * (H // 2))
    print(f"Decoded {n_frames} frame(s), written {yuv_bytes} bytes -> {yuv_path}")
    print(f"Play:  ffplay -f rawvideo -pixel_format yuv420p "
          f"-video_size {W}x{H} -framerate 1 {yuv_path}")

if __name__ == '__main__':
    main()
