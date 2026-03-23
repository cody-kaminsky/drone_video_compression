#!/usr/bin/env python3
"""
diag_hw.py  --  Diagnostic: trace VHDL encoder output vs software reference

Usage:
  python diag_hw.py <bs_out.bin> <width> <height> <qp>
  python diag_hw.py bs_out.bin 64 48 28
"""
import sys

# ---------------------------------------------------------------------------
# Fixed-point constants (match enc_pkg.vhd)
# ---------------------------------------------------------------------------
CONST_BITS     = 13
PASS1_BITS     = 2
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

QP_STEP_BASE = [10, 11, 13, 14, 16, 18]

def quant_step(qp):
    qp = max(1, min(51, qp))
    return QP_STEP_BASE[(qp - 1) % 6] << ((qp - 1) // 6)

# ---------------------------------------------------------------------------
# Software FDCT (matches dct8_fwd.vhd with exact VHDL arithmetic)
# ---------------------------------------------------------------------------
def descale_fwd(x, n):
    """VHDL descale: add bias then arithmetic right shift."""
    bias = 1 << (n - 1)
    return (x + bias) >> n

def mul32_fwd(a, b):
    """VHDL mul32: truncate a to 25-bit signed, multiply by b (18-bit)."""
    # Truncate a to 25 signed bits
    a = (a + (1<<24)) % (1<<25) - (1<<24)
    return a * b  # No truncation of result needed here (Python handles big ints)

def fdct_row(d):
    """Row pass of 8-point forward DCT."""
    tmp0 = d[0] + d[7];  tmp7 = d[0] - d[7]
    tmp1 = d[1] + d[6];  tmp6 = d[1] - d[6]
    tmp2 = d[2] + d[5];  tmp5 = d[2] - d[5]
    tmp3 = d[3] + d[4];  tmp4 = d[3] - d[4]

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2

    r = [0]*8
    r[0] = (tmp10 + tmp11) << PASS1_BITS
    r[4] = (tmp10 - tmp11) << PASS1_BITS

    z1   = mul32_fwd(tmp12 + tmp13, FIX_0_541196100)
    r[2] = descale_fwd(z1 + mul32_fwd(tmp13,  FIX_0_765366865), CONST_BITS - PASS1_BITS)
    r[6] = descale_fwd(z1 + mul32_fwd(tmp12, -FIX_1_847759065), CONST_BITS - PASS1_BITS)

    z1 = tmp4 + tmp7;  z2 = tmp5 + tmp6
    z3 = tmp4 + tmp6;  z4 = tmp5 + tmp7
    z5 = mul32_fwd(z3 + z4,  FIX_1_175875602)
    z1 = mul32_fwd(z1, -FIX_0_899976223)
    z2 = mul32_fwd(z2, -FIX_2_562915447)
    z3 = mul32_fwd(z3, -FIX_1_961570560)
    z4 = mul32_fwd(z4, -FIX_0_390180644)
    z3 += z5;  z4 += z5

    r[7] = descale_fwd(mul32_fwd(tmp4, FIX_0_298631336) + z1 + z3, CONST_BITS - PASS1_BITS)
    r[5] = descale_fwd(mul32_fwd(tmp5, FIX_2_053119869) + z2 + z4, CONST_BITS - PASS1_BITS)
    r[3] = descale_fwd(mul32_fwd(tmp6, FIX_3_072711026) + z2 + z3, CONST_BITS - PASS1_BITS)
    r[1] = descale_fwd(mul32_fwd(tmp7, FIX_1_501321110) + z1 + z4, CONST_BITS - PASS1_BITS)
    return r

def fdct_col(b, c):
    """Column pass of 8-point forward DCT (modifies column c of block b in-place)."""
    tmp0 = b[0][c] + b[7][c];  tmp7 = b[0][c] - b[7][c]
    tmp1 = b[1][c] + b[6][c];  tmp6 = b[1][c] - b[6][c]
    tmp2 = b[2][c] + b[5][c];  tmp5 = b[2][c] - b[5][c]
    tmp3 = b[3][c] + b[4][c];  tmp4 = b[3][c] - b[4][c]

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2

    b[0][c] = descale_fwd(tmp10 + tmp11, PASS1_BITS)
    b[4][c] = descale_fwd(tmp10 - tmp11, PASS1_BITS)

    z1      = mul32_fwd(tmp12 + tmp13, FIX_0_541196100)
    b[2][c] = descale_fwd(z1 + mul32_fwd(tmp13,  FIX_0_765366865), CONST_BITS + PASS1_BITS)
    b[6][c] = descale_fwd(z1 + mul32_fwd(tmp12, -FIX_1_847759065), CONST_BITS + PASS1_BITS)

    z1 = tmp4 + tmp7;  z2 = tmp5 + tmp6
    z3 = tmp4 + tmp6;  z4 = tmp5 + tmp7
    z5 = mul32_fwd(z3 + z4,  FIX_1_175875602)
    z1 = mul32_fwd(z1, -FIX_0_899976223)
    z2 = mul32_fwd(z2, -FIX_2_562915447)
    z3 = mul32_fwd(z3, -FIX_1_961570560)
    z4 = mul32_fwd(z4, -FIX_0_390180644)
    z3 += z5;  z4 += z5

    b[7][c] = descale_fwd(mul32_fwd(tmp4, FIX_0_298631336) + z1 + z3, CONST_BITS + PASS1_BITS)
    b[5][c] = descale_fwd(mul32_fwd(tmp5, FIX_2_053119869) + z2 + z4, CONST_BITS + PASS1_BITS)
    b[3][c] = descale_fwd(mul32_fwd(tmp6, FIX_3_072711026) + z2 + z3, CONST_BITS + PASS1_BITS)
    b[1][c] = descale_fwd(mul32_fwd(tmp7, FIX_1_501321110) + z1 + z4, CONST_BITS + PASS1_BITS)

def fdct8(block):
    """8x8 forward DCT. block: 8x8 list-of-lists. Returns 8x8 DCT coefficients."""
    b = [list(row) for row in block]
    # Row pass
    for r in range(8):
        b[r] = fdct_row(b[r])
    # Column pass
    for c in range(8):
        fdct_col(b, c)
    return b

# ---------------------------------------------------------------------------
# Software quantizer (matches quant_enc.vhd)
# ---------------------------------------------------------------------------
def quantize_block(dct_block, step, is_intra):
    """Quantize 8x8 DCT block. Returns 8x8 quantized coefficients."""
    recip = 65536 // step
    dz_intra = (step * 3) // 8
    dz_inter = step // 4  # not used here
    dz = dz_intra if is_intra else dz_inter

    q = [[0]*8 for _ in range(8)]
    for r in range(8):
        for c in range(8):
            v = dct_block[r][c]
            av = abs(v)
            if av < dz:
                q[r][c] = 0
            else:
                qv = (av * recip) >> 16
                q[r][c] = qv if v >= 0 else -qv
    return q

# ---------------------------------------------------------------------------
# Software IDCT (matches decode_hw.py / dct8_inv.vhd)
# ---------------------------------------------------------------------------
def idescale(x, n):
    return (x + (1 << (n - 1))) >> n

def idct_col(d, c):
    z2 = d[2][c];  z3 = d[6][c]
    z1 = (z2 + z3) * FIX_0_541196100
    tmp2 = z1 + z3 * (-FIX_1_847759065)
    tmp3 = z1 + z2 * FIX_0_765366865

    tmp0 = (d[0][c] + d[4][c]) << IDCT_CONST_BITS
    tmp1 = (d[0][c] - d[4][c]) << IDCT_CONST_BITS
    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2

    z1 = d[7][c] + d[1][c];  z2 = d[5][c] + d[3][c]
    z3 = d[7][c] + d[3][c];  z4 = d[5][c] + d[1][c]
    z5 = (z3 + z4) * FIX_1_175875602
    z1 *= -FIX_0_899976223;  z2 *= -FIX_2_562915447
    z3 *= -FIX_1_961570560;  z4 *= -FIX_0_390180644
    z3 += z5;  z4 += z5

    r0 = d[7][c] * FIX_0_298631336 + z1 + z3
    r1 = d[5][c] * FIX_2_053119869 + z2 + z4
    r2 = d[3][c] * FIX_3_072711026 + z2 + z3
    r3 = d[1][c] * FIX_1_501321110 + z1 + z4

    sh = IDCT_CONST_BITS + IDCT_PASS1_BITS + 3  # = 17
    d[0][c] = idescale(tmp10 + r3, sh)
    d[7][c] = idescale(tmp10 - r3, sh)
    d[1][c] = idescale(tmp11 + r2, sh)
    d[6][c] = idescale(tmp11 - r2, sh)
    d[2][c] = idescale(tmp12 + r1, sh)
    d[5][c] = idescale(tmp12 - r1, sh)
    d[3][c] = idescale(tmp13 + r0, sh)
    d[4][c] = idescale(tmp13 - r0, sh)

def idct_row(d, r):
    row = d[r]
    z2 = row[2];  z3 = row[6]
    z1 = (z2 + z3) * FIX_0_541196100
    tmp2 = z1 + z3 * (-FIX_1_847759065)
    tmp3 = z1 + z2 * FIX_0_765366865

    tmp0 = (row[0] + row[4]) << IDCT_CONST_BITS
    tmp1 = (row[0] - row[4]) << IDCT_CONST_BITS
    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2

    z1 = row[7] + row[1];  z2 = row[5] + row[3]
    z3 = row[7] + row[3];  z4 = row[5] + row[1]
    z5 = (z3 + z4) * FIX_1_175875602
    z1 *= -FIX_0_899976223;  z2 *= -FIX_2_562915447
    z3 *= -FIX_1_961570560;  z4 *= -FIX_0_390180644
    z3 += z5;  z4 += z5

    r0 = row[7] * FIX_0_298631336 + z1 + z3
    r1 = row[5] * FIX_2_053119869 + z2 + z4
    r2 = row[3] * FIX_3_072711026 + z2 + z3
    r3 = row[1] * FIX_1_501321110 + z1 + z4

    sh = IDCT_CONST_BITS - IDCT_PASS1_BITS + 3  # = 15
    row[0] = idescale(tmp10 + r3, sh)
    row[7] = idescale(tmp10 - r3, sh)
    row[1] = idescale(tmp11 + r2, sh)
    row[6] = idescale(tmp11 - r2, sh)
    row[2] = idescale(tmp12 + r1, sh)
    row[5] = idescale(tmp12 - r1, sh)
    row[3] = idescale(tmp13 + r0, sh)
    row[4] = idescale(tmp13 - r0, sh)

def idct8(dqcoeff):
    d = [list(row) for row in dqcoeff]
    for c in range(8):
        idct_col(d, c)
    for r in range(8):
        idct_row(d, r)
    return d

# ---------------------------------------------------------------------------
# Bitstream reader
# ---------------------------------------------------------------------------
class BitstreamReader:
    def __init__(self, data: bytes):
        self.data = data
        self.byte_pos = 0
        self.bit_buf  = 0
        self.bit_count = 0
        self.total_bits = 0

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
        self.total_bits += 1
        return bit

    def read_bits(self, n: int) -> int:
        v = 0
        for _ in range(n):
            v = (v << 1) | self.read_bit()
        return v

    def read_ue(self) -> int:
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
        ue = self.read_ue()
        if ue == 0:
            return 0
        if ue & 1:
            return (ue + 1) >> 1
        else:
            return -(ue >> 1)

# ---------------------------------------------------------------------------
# Main diagnostic
# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) < 5:
        print("Usage: diag_hw.py <bs_out.bin> <width> <height> <qp>")
        sys.exit(1)

    bs_path = sys.argv[1]
    W  = int(sys.argv[2])
    H  = int(sys.argv[3])
    QP = int(sys.argv[4])

    step = quant_step(QP)
    recip = 65536 // step
    print(f"QP={QP}  step={step}  recip={recip}  dz_intra={step*3//8}")

    with open(bs_path, 'rb') as f:
        raw = f.read()
    print(f"Bitstream: {len(raw)} bytes\n")

    # --- Generate original test image ---
    orig = [[0]*W for _ in range(H)]
    for r in range(H):
        for c in range(W):
            orig[r][c] = (r * 4 + c * 2) % 256

    # --- Software encode (original pixels, VHDL-accurate) ---
    blk_cols = W // 8
    blk_rows = H // 8

    # Predictor state using ORIGINAL pixels (mirrors VHDL mb_buffer logic)
    # above_row_store[bx] = 8 pixels of row-7 from block above
    above_row_store = [[128]*8 for _ in range(blk_cols)]
    left_col_pix    = [128]*8   # 8 pixels of col-7 from left block
    first_strip = True

    MODE_DC, MODE_HORIZ, MODE_VERT = 0, 1, 2

    def sw_mode(has_above, has_left):
        if has_above:   return MODE_VERT
        if has_left:    return MODE_HORIZ
        return MODE_DC

    # Collect software-predicted quant coefficients + mode
    sw_qcoeff = {}   # (bx,by) -> 8x8 quant array
    sw_mode_d = {}   # (bx,by) -> mode

    print("=== SOFTWARE ENCODER (VHDL-accurate) ===")
    for by in range(blk_rows):
        left_col_pix = [128]*8
        for bx in range(blk_cols):
            px = bx * 8;  py = by * 8

            has_above = not first_strip
            has_left  = (bx > 0)
            mode      = sw_mode(has_above, has_left)

            if mode == MODE_VERT:
                # Residual = pixel - above_row_pixel[col]
                res = [[orig[py+r][px+c] - above_row_store[bx][c] for c in range(8)]
                       for r in range(8)]
                pred_dc = 0  # not a scalar pred in VERT mode
            elif mode == MODE_HORIZ:
                # Residual = pixel - left_col_pixel[row]
                res = [[orig[py+r][px+c] - left_col_pix[r] for c in range(8)]
                       for r in range(8)]
                pred_dc = 0
            else:
                pred_dc = 128
                res = [[orig[py+r][px+c] - pred_dc for c in range(8)] for r in range(8)]

            dct = fdct8(res)
            qc  = quantize_block(dct, step, is_intra=True)
            sw_qcoeff[(bx, by)] = qc
            sw_mode_d[(bx, by)] = mode

            # Update predictors with ORIGINAL pixels (row-7 and col-7)
            above_row_store[bx] = [orig[py+7][px+c] for c in range(8)]
            left_col_pix        = [orig[py+r][px+7] for r in range(8)]

            flat_zz = []
            last_nz = -1
            for z in range(64):
                nat = ZIGZAG_8x8[z]; r2 = nat >> 3; c2 = nat & 7
                v = qc[r2][c2]; flat_zz.append(v)
                if v != 0: last_nz = z

            n_nonzero = last_nz + 1
            if by < 2:
                mode_str = ['DC','HZ','VT'][mode]
                print(f"  blk({bx},{by}): mode={mode_str}  DC_quant={qc[0][0]:4d}  "
                      f"count={n_nonzero:2d}  "
                      f"avg_orig={sum(orig[py+r][px+c] for r in range(8) for c in range(8))/64:.1f}")

        first_strip = False

    print()

    # --- Parse hardware bitstream ---
    print("=== HARDWARE BITSTREAM DECODE ===")
    bs = BitstreamReader(raw)
    ftype = bs.read_bit()
    print(f"Frame type: {'P' if ftype else 'I'}")

    # Reconstructed-pixel prediction state
    above_row_recon = [[128]*8 for _ in range(blk_cols)]
    left_col_recon  = [128]*8

    Y_hw = [[128]*W for _ in range(H)]
    blk_errors = []
    prev_dc_hw = 0   # DC DPCM accumulator

    for blk_idx in range(blk_cols * blk_rows):
        bx = blk_idx % blk_cols
        by = blk_idx // blk_cols
        px = bx * 8;  py = by * 8

        if bx == 0:
            left_col_recon = [128]*8

        has_above = (by > 0)
        has_left  = (bx > 0)

        # Read 2-bit intra mode
        mode_bits = bs.read_bits(2)  # 0=DC, 1=HORIZ, 2=VERT
        mode = mode_bits

        # Read ue(count) then se coefficients with DC DPCM on first coeff
        count = bs.read_ue()
        hw_coeffs_flat = [0] * 64
        if count > 0:
            dc_diff = bs.read_se()
            dc_quant = dc_diff + prev_dc_hw
            prev_dc_hw = dc_quant
            hw_coeffs_flat[0] = dc_quant
            for i in range(1, count):
                hw_coeffs_flat[i] = bs.read_se()
        else:
            prev_dc_hw = 0

        # Unpack zigzag
        hw_qc = [[0]*8 for _ in range(8)]
        for z in range(64):
            nat = ZIGZAG_8x8[z]; r2 = nat >> 3; c2 = nat & 7
            hw_qc[r2][c2] = hw_coeffs_flat[z]

        # Compare SW vs HW for first two strips
        if by < 2:
            sw_qc   = sw_qcoeff[(bx, by)]
            sw_flat = []
            for z in range(64):
                nat = ZIGZAG_8x8[z]; r2 = nat >> 3; c2 = nat & 7
                sw_flat.append(sw_qc[r2][c2])
            sw_last_nz = max((i for i,v in enumerate(sw_flat) if v!=0), default=-1)
            hw_last_nz = max((i for i,v in enumerate(hw_coeffs_flat) if v!=0), default=-1)

            mismatches = [(z, sw_flat[z], hw_coeffs_flat[z])
                         for z in range(64) if sw_flat[z] != hw_coeffs_flat[z]]
            mismatch_str = f"{len(mismatches)} mismatch(es)" if mismatches else "EXACT MATCH"
            if mismatches and len(mismatches) <= 3:
                mismatch_str += " " + str(mismatches[:3])

            mode_str = ['DC','HZ','VT'][mode]
            print(f"  blk({bx},{by}): mode={mode_str}  DC={hw_qc[0][0]:4d}(hw) {sw_qcoeff[(bx,by)][0][0]:4d}(sw)"
                  f"  count={count:2d}(hw) {sw_last_nz+1:2d}(sw)  {mismatch_str}")

        # Dequantize + IDCT
        dqc   = [[hw_qc[r2][c2] * step for c2 in range(8)] for r2 in range(8)]
        resid = idct8(dqc)

        for r2 in range(8):
            for c2 in range(8):
                if mode == MODE_VERT:
                    pred_pix = above_row_recon[bx][c2]
                elif mode == MODE_HORIZ:
                    pred_pix = left_col_recon[r2]
                else:
                    pred_pix = 128
                Y_hw[py+r2][px+c2] = max(0, min(255, resid[r2][c2] + pred_pix))

        # Update reconstructed-pixel predictors
        above_row_recon[bx] = [Y_hw[py+7][px+c2] for c2 in range(8)]
        left_col_recon      = [Y_hw[py+r2][px+7] for r2 in range(8)]

        mse_blk = sum((Y_hw[py+r2][px+c2] - orig[py+r2][px+c2])**2
                       for r2 in range(8) for c2 in range(8)) / 64
        blk_errors.append((bx, by, mse_blk))

    print()

    # --- PSNR ---
    total_mse = sum(m for _,_,m in blk_errors) / len(blk_errors)
    if total_mse > 0:
        psnr = 10 * __import__('math').log10(255**2 / total_mse)
    else:
        psnr = float('inf')
    print(f"=== PSNR (HW bitstream + SW decode) = {psnr:.2f} dB  (MSE={total_mse:.1f}) ===")

    # --- Worst blocks ---
    print("\nWorst 10 blocks (highest MSE):")
    worst = sorted(blk_errors, key=lambda x: -x[2])[:10]
    for bx,by,mse in worst:
        px=bx*8; py=by*8
        psnr_blk = 10*__import__('math').log10(255**2/mse) if mse > 0 else float('inf')
        print(f"  blk({bx:2d},{by:2d}) px=({px},{py})  MSE={mse:6.1f}  PSNR={psnr_blk:.1f} dB"
              f"  DC_hw={0}  orig_avg={sum(orig[py+r][px+c] for r in range(8) for c in range(8))/64:.0f}")

    # --- Software-only round-trip PSNR (mirrors new VHDL encoder exactly) ---
    print("\n=== SOFTWARE ROUND-TRIP (for reference) ===")
    above_row_sw = [[128]*8 for _ in range(blk_cols)]
    left_col_sw  = [128]*8
    first_strip  = True
    Y_sw = [[0]*W for _ in range(H)]

    for by in range(blk_rows):
        left_col_sw = [128]*8
        for bx in range(blk_cols):
            px = bx * 8;  py = by * 8
            has_above = not first_strip
            has_left  = (bx > 0)
            mode      = sw_mode(has_above, has_left)

            if mode == MODE_VERT:
                res = [[orig[py+r][px+c] - above_row_sw[bx][c] for c in range(8)]
                       for r in range(8)]
            elif mode == MODE_HORIZ:
                res = [[orig[py+r][px+c] - left_col_sw[r] for c in range(8)]
                       for r in range(8)]
            else:
                res = [[orig[py+r][px+c] - 128 for c in range(8)] for r in range(8)]

            dct = fdct8(res)
            qc  = quantize_block(dct, step, is_intra=True)
            dqc = [[qc[r][c] * step for c in range(8)] for r in range(8)]
            rec = idct8(dqc)

            for r in range(8):
                for c in range(8):
                    if mode == MODE_VERT:
                        pred_pix = above_row_sw[bx][c]
                    elif mode == MODE_HORIZ:
                        pred_pix = left_col_sw[r]
                    else:
                        pred_pix = 128
                    Y_sw[py+r][px+c] = max(0, min(255, rec[r][c] + pred_pix))

            above_row_sw[bx] = [orig[py+7][px+c] for c in range(8)]
            left_col_sw      = [orig[py+r][px+7] for r in range(8)]

        first_strip = False

    mse_sw = sum((Y_sw[r][c] - orig[r][c])**2 for r in range(H) for c in range(W)) / (W*H)
    psnr_sw = 10 * __import__('math').log10(255**2 / mse_sw) if mse_sw > 0 else float('inf')
    print(f"SW round-trip PSNR = {psnr_sw:.2f} dB  (MSE={mse_sw:.2f})")
    print()
    print("Conclusion:")
    print(f"  SW round-trip PSNR = {psnr_sw:.2f} dB")
    print(f"  HW PSNR            = {psnr:.2f} dB  (delta {psnr-psnr_sw:+.2f} dB)")

if __name__ == '__main__':
    main()
