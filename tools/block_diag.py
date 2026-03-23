#!/usr/bin/env python3
"""
block_diag.py - Decode HW bitstream block-by-block and compare against SW DCT.

Reads bs_out.bin, decodes each block's quantized coefficients (exp-Golomb),
runs SW forward DCT on expected residuals, and shows non-zero coefficient
positions for problem blocks.
"""

import struct, math, sys, os

BS_PATH = r"C:/Users/kamin/Vivado_Projects/compression_test/compression_test.sim/sim_1/behav/xsim/bs_out.bin"
FRAME_W = 64
FRAME_H = 48
QP = 16

# ── QP step ROM ──────────────────────────────────────────────────────────────
BASE = [10, 11, 13, 14, 16, 18]
def qp_step(qp):
    return BASE[(qp-1) % 6] * (2 ** ((qp-1) // 6))

STEP = qp_step(QP)
print(f"QP={QP}  step={STEP}")

# ── Pixel formula (from tb_enc_top) ─────────────────────────────────────────
def pixel(row, col):
    i = row * FRAME_W + col
    return ((i // FRAME_W) * 4 + (i % FRAME_W) * 2) % 256

# ── Build full luma frame ────────────────────────────────────────────────────
frame = [[pixel(r, c) for c in range(FRAME_W)] for r in range(FRAME_H)]

# ── IJG 8x8 forward DCT (matches dct8_fwd.vhd) ──────────────────────────────
CONST_BITS = 13
PASS1_BITS = 2
FIX = lambda x: int(round(x * (1 << CONST_BITS)))

FIX_0_298631336 = FIX(0.298631336)
FIX_0_390180644 = FIX(0.390180644)
FIX_0_541196100 = FIX(0.541196100)
FIX_0_765366865 = FIX(0.765366865)
FIX_0_899976223 = FIX(0.899976223)
FIX_1_175875602 = FIX(1.175875602)
FIX_1_501321110 = FIX(1.501321110)
FIX_1_847759065 = FIX(1.847759065)
FIX_1_961570560 = FIX(1.961570560)
FIX_2_053119869 = FIX(2.053119869)
FIX_2_562915447 = FIX(2.562915447)
FIX_3_072711026 = FIX(3.072711026)

def descale(x, n):
    # arithmetic right shift with rounding
    if x >= 0:
        return (x + (1 << (n-1))) >> n
    else:
        return -( (-x + (1 << (n-1))) >> n )

def dct_row_pass(row):
    """Row pass of IJG 8x8 DCT, returns 8 fixed-point values scaled by 2^PASS1_BITS."""
    d = list(row)
    tmp0 = d[0] + d[7]
    tmp1 = d[1] + d[6]
    tmp2 = d[2] + d[5]
    tmp3 = d[3] + d[4]
    tmp4 = d[3] - d[4]
    tmp5 = d[2] - d[5]
    tmp6 = d[1] - d[6]
    tmp7 = d[0] - d[7]

    tmp10 = tmp0 + tmp3
    tmp11 = tmp1 + tmp2
    tmp12 = tmp1 - tmp2
    tmp13 = tmp0 - tmp3

    out = [0]*8
    out[0] = (tmp10 + tmp11) << PASS1_BITS
    out[4] = (tmp10 - tmp11) << PASS1_BITS

    z1 = (tmp12 + tmp13) * FIX_0_541196100
    out[2] = descale(z1 + tmp13 * FIX_0_765366865, CONST_BITS - PASS1_BITS)
    out[6] = descale(z1 + tmp12 * (-FIX_1_847759065), CONST_BITS - PASS1_BITS)

    z1 = tmp4 + tmp7
    z2 = tmp5 + tmp6
    z3 = tmp4 + tmp6
    z4 = tmp5 + tmp7
    z5 = (z3 + z4) * FIX_1_175875602

    tmp4 *= FIX_0_298631336
    tmp5 *= FIX_2_053119869
    tmp6 *= FIX_3_072711026
    tmp7 *= FIX_1_501321110

    z1 *= (-FIX_0_899976223)
    z2 *= (-FIX_2_562915447)
    z3 *= (-FIX_1_961570560)
    z4 *= (-FIX_0_390180644)

    z3 += z5; z4 += z5

    out[7] = descale(tmp4 + z1 + z3, CONST_BITS - PASS1_BITS)
    out[5] = descale(tmp5 + z2 + z4, CONST_BITS - PASS1_BITS)
    out[3] = descale(tmp6 + z2 + z3, CONST_BITS - PASS1_BITS)
    out[1] = descale(tmp7 + z1 + z4, CONST_BITS - PASS1_BITS)

    return out

def dct8x8(block):
    """Full 8x8 IJG DCT. block is list of 8 rows, each 8 ints."""
    # Row pass
    rows = [dct_row_pass(block[r]) for r in range(8)]
    # Column pass
    out = [[0]*8 for _ in range(8)]
    for c in range(8):
        col_in = [rows[r][c] for r in range(8)]
        d = col_in
        tmp0 = d[0] + d[7]
        tmp1 = d[1] + d[6]
        tmp2 = d[2] + d[5]
        tmp3 = d[3] + d[4]
        tmp4 = d[3] - d[4]
        tmp5 = d[2] - d[5]
        tmp6 = d[1] - d[6]
        tmp7 = d[0] - d[7]

        tmp10 = tmp0 + tmp3
        tmp11 = tmp1 + tmp2
        tmp12 = tmp1 - tmp2
        tmp13 = tmp0 - tmp3

        out[0][c] = descale(tmp10 + tmp11, PASS1_BITS + 3)
        out[4][c] = descale(tmp10 - tmp11, PASS1_BITS + 3)

        z1 = (tmp12 + tmp13) * FIX_0_541196100
        out[2][c] = descale(z1 + tmp13 * FIX_0_765366865, CONST_BITS + PASS1_BITS + 3)
        out[6][c] = descale(z1 + tmp12 * (-FIX_1_847759065), CONST_BITS + PASS1_BITS + 3)

        z1 = tmp4 + tmp7
        z2 = tmp5 + tmp6
        z3 = tmp4 + tmp6
        z4 = tmp5 + tmp7
        z5 = (z3 + z4) * FIX_1_175875602

        tmp4 *= FIX_0_298631336
        tmp5 *= FIX_2_053119869
        tmp6 *= FIX_3_072711026
        tmp7 *= FIX_1_501321110

        z1 *= (-FIX_0_899976223)
        z2 *= (-FIX_2_562915447)
        z3 *= (-FIX_1_961570560)
        z4 *= (-FIX_0_390180644)

        z3 += z5; z4 += z5

        out[7][c] = descale(tmp4 + z1 + z3, CONST_BITS + PASS1_BITS + 3)
        out[5][c] = descale(tmp5 + z2 + z4, CONST_BITS + PASS1_BITS + 3)
        out[3][c] = descale(tmp6 + z2 + z3, CONST_BITS + PASS1_BITS + 3)
        out[1][c] = descale(tmp7 + z1 + z4, CONST_BITS + PASS1_BITS + 3)

    return out

# ── Quantization (matches quant_enc.vhd) ─────────────────────────────────────
RECIP_TABLE = {}
def get_recip(step):
    if step not in RECIP_TABLE:
        RECIP_TABLE[step] = int(65536 / step + 0.5)  # round
    return RECIP_TABLE[step]

DZ_INTRA = STEP // 2 - 1  # dead zone for intra (approx)

def quantize(coeff, step):
    """Matches quant_enc.vhd: product = |coeff| * recip; q = product >> 16; restore sign."""
    recip = get_recip(step)
    av = abs(coeff)
    product = av * recip
    q = product >> 16
    return -q if coeff < 0 else q

# ── Zigzag LUT ────────────────────────────────────────────────────────────────
ZIGZAG = [
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
]
# INV_ZIGZAG: natural index → zigzag index
INV_ZIGZAG = [0]*64
for zz, nat in enumerate(ZIGZAG):
    INV_ZIGZAG[nat] = zz

def natural_to_zigzag(coeffs_2d):
    """coeffs_2d[row][col] → flat in zigzag order."""
    flat = [coeffs_2d[nat // 8][nat % 8] for nat in ZIGZAG]
    return flat

# ── Exponential-Golomb decoder ────────────────────────────────────────────────
class BitReader:
    def __init__(self, data):
        self.data = data
        self.pos = 0  # bit position

    def read_bit(self):
        byte_idx = self.pos >> 3
        bit_idx  = 7 - (self.pos & 7)
        self.pos += 1
        if byte_idx >= len(self.data):
            return 0
        return (self.data[byte_idx] >> bit_idx) & 1

    def read_bits(self, n):
        val = 0
        for _ in range(n):
            val = (val << 1) | self.read_bit()
        return val

    def read_exp_golomb_signed(self):
        """Signed exp-Golomb: k=0."""
        # Count leading zeros
        leading = 0
        while self.read_bit() == 0:
            leading += 1
            if leading > 32:
                return 0
        suffix = self.read_bits(leading)
        code = (1 << leading) - 1 + suffix
        # Signed mapping: 0→0, 1→1, 2→-1, 3→2, 4→-2, ...
        if code == 0:
            return 0
        elif code % 2 == 1:
            return (code + 1) // 2
        else:
            return -(code // 2)

    def bit_pos(self):
        return self.pos

# ── Decode one block from bitstream ──────────────────────────────────────────
def decode_block(br):
    """Decode one block: count + count signed exp-Golomb coefficients.
    Returns list of (zigzag_idx, q_val) pairs."""
    count = br.read_bits(7)
    coeffs = []
    for i in range(count):
        val = br.read_exp_golomb_signed()
        coeffs.append(val)
    return count, coeffs

# ── SW encode one block ───────────────────────────────────────────────────────
def sw_encode_block(bx, by, pred_dc):
    """Return (coeffs_zigzag, count_nz, dc_q) for SW encode of block (bx,by)."""
    r0 = by * 8
    c0 = bx * 8
    # Build residual block
    block = []
    for r in range(8):
        row = []
        for c in range(8):
            pix = frame[r0+r][c0+c]
            res = pix - pred_dc
            row.append(res)
        block.append(row)
    # Forward DCT
    dct = dct8x8(block)
    # Quantize in natural order, then zigzag
    coeffs_nat = [dct[r][c] for r in range(8) for c in range(8)]
    coeffs_zz = [quantize(coeffs_nat[ZIGZAG[zz]], STEP) for zz in range(64)]
    # Find last non-zero
    last_nz = -1
    for i in range(63, -1, -1):
        if coeffs_zz[i] != 0:
            last_nz = i
            break
    count = last_nz + 1
    return coeffs_zz, count, coeffs_zz[0]

# ── Main ──────────────────────────────────────────────────────────────────────
with open(BS_PATH, "rb") as f:
    bs_data = f.read()

print(f"Bitstream size: {len(bs_data)} bytes")

br = BitReader(bs_data)

BLOCKS_W = FRAME_W // 8  # 8
BLOCKS_H = FRAME_H // 8  # 6

# SW: track pred_dc per block (I-frame, predictor = avg of reconstructed block above)
# For simplicity in SW: use 128 as initial predictor (matches HW reset)
# Then update with avg of row 7 of reconstructed block

pred_dc_sw = 128

print(f"\n{'BLK':>8}  {'HW_cnt':>6}  {'SW_cnt':>6}  {'HW_DC':>7}  {'SW_DC':>7}  {'pred':>5}  STATUS")
print("-"*70)

hw_blocks = []
sw_blocks = []

for by in range(BLOCKS_H):
    for bx in range(BLOCKS_W):
        blk_id = f"({bx},{by})"

        # Decode HW block
        hw_count, hw_coeffs_raw = decode_block(br)
        # hw_coeffs_raw is the list of non-zero coeff values in zigzag order
        # Reconstruct full zigzag array
        hw_zz = [0]*64
        for i, v in enumerate(hw_coeffs_raw):
            if i < 64:
                hw_zz[i] = v
        hw_dc = hw_zz[0] if hw_count > 0 else 0

        # SW encode block
        sw_zz, sw_count, sw_dc = sw_encode_block(bx, by, pred_dc_sw)

        status = "OK" if hw_count == sw_count and hw_dc == sw_dc else "MISMATCH"

        print(f"{blk_id:>8}  {hw_count:>6}  {sw_count:>6}  {hw_dc:>7}  {sw_dc:>7}  {pred_dc_sw:>5}  {status}")

        if status == "MISMATCH" and (by <= 1 or bx >= 5):
            # Show first 10 coefficient differences
            print(f"         HW coeffs (first {min(hw_count,16)}): {hw_zz[:min(hw_count,16)]}")
            print(f"         SW coeffs (first {min(sw_count,16)}): {sw_zz[:min(sw_count,16)]}")

        # Update predictor: avg of reconstructed row 7
        # (For SW diagnostics, approximate with pixel avg of row 7)
        r7 = by*8 + 7
        row7_avg = sum(frame[r7][bx*8:bx*8+8]) // 8
        # In I-frame: pred_dc for next block in row = avg of last row of this block
        # But actually mb_buffer uses recon_col7avg (avg of col 7) as left predictor
        # and recon_row7avg (avg of row 7) as top predictor.
        # For simple strip-sequential order (left-to-right, top-to-bottom):
        # Use row7_avg as predictor for blocks in same strip's next block
        # But first block of each new strip uses top-block's row7 avg
        if bx == BLOCKS_W - 1:
            # End of strip: next strip's pred will come from row7 of this strip
            pred_dc_sw = row7_avg
        else:
            # Within strip: use col7 avg as left predictor? Or row7?
            # mb_buffer EMIT uses predictor=left_avg for all blocks in same strip
            # (left_avg set at start of strip from WAIT_RECON of previous strip's last block)
            # Actually predictor for next block comes from recon_col7avg of current block
            col7_avg = sum(frame[by*8+r][bx*8+7] for r in range(8)) // 8
            pred_dc_sw = col7_avg

print(f"\nEnd bit position: {br.bit_pos()} bits = {br.bit_pos()//8} bytes")
