#!/usr/bin/env python3
"""Reverse-engineer pixel row sums from bad block DCT coefficients."""
STEP = 56
IDCT_CONST_BITS = 13
IDCT_PASS1_BITS = 1
FIX_0_298631336=2446;FIX_0_390180644=3196;FIX_0_541196100=4433
FIX_0_765366865=6270;FIX_0_899976223=7373;FIX_1_175875602=9633
FIX_1_501321110=12299;FIX_1_847759065=15137;FIX_1_961570560=16069
FIX_2_053119869=16819;FIX_2_562915447=20995;FIX_3_072711026=25172

def ids(x,n): return (x+(1<<(n-1)))>>n
def idct1d(d_in):
    d=list(d_in)
    z2=d[2];z3=d[6]
    z1=(z2+z3)*FIX_0_541196100
    tmp2=z1+z3*(-FIX_1_847759065)
    tmp3=z1+z2*FIX_0_765366865
    tmp0=(d[0]+d[4])<<IDCT_CONST_BITS
    tmp1=(d[0]-d[4])<<IDCT_CONST_BITS
    tmp10=tmp0+tmp3;tmp13=tmp0-tmp3
    tmp11=tmp1+tmp2;tmp12=tmp1-tmp2
    z1=d[7]+d[1];z2=d[5]+d[3];z3=d[7]+d[3];z4=d[5]+d[1]
    z5=(z3+z4)*FIX_1_175875602
    z1*=-FIX_0_899976223;z2*=-FIX_2_562915447
    z3*=-FIX_1_961570560+z5;z4*=-FIX_0_390180644
    z3+=z5;z4+=z5
    r0=d[7]*FIX_0_298631336+z1+z3
    r1=d[5]*FIX_2_053119869+z2+z4
    r2=d[3]*FIX_3_072711026+z2+z3
    r3=d[1]*FIX_1_501321110+z1+z4
    sh=IDCT_CONST_BITS+IDCT_PASS1_BITS+3
    out=[0]*8
    out[0]=ids(tmp10+r3,sh);out[7]=ids(tmp10-r3,sh)
    out[1]=ids(tmp11+r2,sh);out[6]=ids(tmp11-r2,sh)
    out[2]=ids(tmp12+r1,sh);out[5]=ids(tmp12-r1,sh)
    out[3]=ids(tmp13+r0,sh);out[4]=ids(tmp13-r0,sh)
    return out

# Expected row-residual-sums for CORRECT blocks in the test image
# For block at bx, by with pred_dc:
# row_sum[r] = sum(pixel(by*8+r, bx*8+c) - pred_dc for c=0..7)
#            = sum(((by*8+r)*4 + (bx*8+c)*2) for c=0..7) - 8*pred_dc
#            = 8*(by*8+r)*4 + 2*(bx*8+0+...+bx*8+7) - 8*pred_dc
#            = 32*(by*8+r) + 2*(8*bx*8+28) - 8*pred_dc
#            = 32*by*8 + 32r + 16*bx*8 + 56 - 8*pred_dc

def expected_row_sums(bx, by, pred_dc):
    return [32*(by*8+r) + 16*bx*8 + 56 - 8*pred_dc for r in range(8)]

# For a block with row_sums = RS[r], the column-0 FDCT input (before column pass)
# = 4 * RS[r] (PASS1_BITS=2 shift in row pass DC output)
# The column-0 FDCT output (quantized) should match the bad block's col0.

# --- blk(6,0) analysis ---
print("=== blk(6,0) ===")
blk60_col0_q = [21, 2, 11, 9, 9, 6, 4, 2]  # quantized column-0 rows 0-7
blk60_col0_dq = [q*STEP for q in blk60_col0_q]
print(f"Dequantized col0: {blk60_col0_dq}")

# Expected for blk(6,0) with pred_dc=106:
rs60 = expected_row_sums(6, 0, 106)
print(f"Expected row sums (bx=6, pred_dc=106): {rs60}")
print(f"  Column-0 FDCT input: {[4*x for x in rs60]}")

# What block col sums give column-0 ≈ blk60_col0_dq?
# The FDCT maps 4*RS[r] → col0_coefficients. To find RS, apply IDCT to col0.
# IDCT maps col0_coefficients → 8 * RS[r] / (some normalization)
# Actually the column-pass of the FDCT:
# out[0] = descale(tmp10+tmp11, PASS1_BITS) where inputs are 4*RS[r]
# = (sum_all_4RS) / 4 = sum(RS) (approximately)
# = total_residual_sum, quantized → DC = Q

# Let's compute what the FDCT of 4*RS gives vs the bad block's col0
# SW expected for blk(6,0):
rs60_sw = expected_row_sums(6, 0, 108)  # SW pred_dc from diag_hw.py
print(f"\nExpected row sums (bx=6, pred_dc=108 SW): {rs60_sw}")

# --- What block has row sums that would produce blk(6,0) col0? ---
# The column-0 DCT coefficients depend only on RS[r] via a specific formula.
# By varying bx and pred_dc, find the match.
print("\n--- Searching for matching block ---")
for by_test in range(6):
    for bx_test in range(8):
        for pred_test in range(50, 200, 2):
            rs = expected_row_sums(bx_test, by_test, pred_test)
            # FDCT of 4*RS: rough DC check
            dc_approx = sum(rs)  # DC ≈ total_residual_sum
            q_dc = int(abs(dc_approx) * 1170 / 65536)  # quantize
            if q_dc == 21 and dc_approx > 0:  # match DC=21
                # Check other col0 terms
                # Rough: col0[1] corresponds to the "vertical k=1" frequency
                # For arithmetic RS[r] = A + B*r:
                if all(r0 == rs[0] + r*32 for r,r0 in enumerate(rs)):
                    print(f"  Arithmetic RS: bx={bx_test},by={by_test},pred_dc={pred_test}: RS={rs}")

# Direct SW check for all blocks
print("\n--- All blocks with DC quant ≈ 21 ---")
for by_test in range(6):
    for bx_test in range(8):
        for pred_test in range(0, 256, 2):
            rs = expected_row_sums(bx_test, by_test, pred_test)
            dc = sum(rs)
            if abs(dc)*1170//65536 == 21:
                print(f"  bx={bx_test},by={by_test},pred_dc={pred_test}: total_res={dc}, DC_q={abs(dc)*1170//65536}{'(neg)' if dc<0 else ''}")
                break
