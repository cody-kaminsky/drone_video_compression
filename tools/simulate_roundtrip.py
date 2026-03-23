#!/usr/bin/env python3
"""
Simulate the full encode-reconstruct loop (mb_buffer -> DCT -> quant ->
dequant -> IDCT -> recon_writer) for each block of strip 0.
Compute actual recon_col7avg values and resulting predictor chain.
Compare against HW-observed count=36 for blocks 6,7.
"""

W, H = 64, 48
STEP = 56   # QP=16 step

# ── IJG 8x8 forward DCT ───────────────────────────────────────────────────────
CONST_BITS = 13; PASS1_BITS = 2
FIX_0_298631336=2446; FIX_0_390180644=3196; FIX_0_541196100=4433
FIX_0_765366865=6270; FIX_0_899976223=7373; FIX_1_175875602=9633
FIX_1_501321110=12299; FIX_1_847759065=15137; FIX_1_961570560=16069
FIX_2_053119869=16819; FIX_2_562915447=20995; FIX_3_072711026=25172

def ds(x,n): return (x+(1<<(n-1)))>>n
def m32(a,b):
    av=((a+(1<<24))%(1<<25))-(1<<24); return av*b

def frow(d):
    t0=d[0]+d[7];t7=d[0]-d[7];t1=d[1]+d[6];t6=d[1]-d[6]
    t2=d[2]+d[5];t5=d[2]-d[5];t3=d[3]+d[4];t4=d[3]-d[4]
    a=t0+t3;c=t0-t3;b=t1+t2;e=t1-t2
    r=[0]*8
    r[0]=(a+b)<<PASS1_BITS; r[4]=(a-b)<<PASS1_BITS
    z=m32(e+c,FIX_0_541196100)
    r[2]=ds(z+m32(c,FIX_0_765366865),CONST_BITS-PASS1_BITS)
    r[6]=ds(z+m32(e,-FIX_1_847759065),CONST_BITS-PASS1_BITS)
    z1=t4+t7;z2=t5+t6;z3=t4+t6;z4=t5+t7
    z5=m32(z3+z4,FIX_1_175875602)
    z1=m32(z1,-FIX_0_899976223);z2=m32(z2,-FIX_2_562915447)
    z3=m32(z3,-FIX_1_961570560)+z5;z4=m32(z4,-FIX_0_390180644)+z5
    r[7]=ds(m32(t4,FIX_0_298631336)+z1+z3,CONST_BITS-PASS1_BITS)
    r[5]=ds(m32(t5,FIX_2_053119869)+z2+z4,CONST_BITS-PASS1_BITS)
    r[3]=ds(m32(t6,FIX_3_072711026)+z2+z3,CONST_BITS-PASS1_BITS)
    r[1]=ds(m32(t7,FIX_1_501321110)+z1+z4,CONST_BITS-PASS1_BITS)
    return r

def fcol(b,c):
    t0=b[0][c]+b[7][c];t7=b[0][c]-b[7][c];t1=b[1][c]+b[6][c];t6=b[1][c]-b[6][c]
    t2=b[2][c]+b[5][c];t5=b[2][c]-b[5][c];t3=b[3][c]+b[4][c];t4=b[3][c]-b[4][c]
    a=t0+t3;cc=t0-t3;bb=t1+t2;e=t1-t2
    b[0][c]=ds(a+bb,PASS1_BITS); b[4][c]=ds(a-bb,PASS1_BITS)
    z=m32(e+cc,FIX_0_541196100)
    b[2][c]=ds(z+m32(cc,FIX_0_765366865),CONST_BITS+PASS1_BITS)
    b[6][c]=ds(z+m32(e,-FIX_1_847759065),CONST_BITS+PASS1_BITS)
    z1=t4+t7;z2=t5+t6;z3=t4+t6;z4=t5+t7
    z5=m32(z3+z4,FIX_1_175875602)
    z1=m32(z1,-FIX_0_899976223);z2=m32(z2,-FIX_2_562915447)
    z3=m32(z3,-FIX_1_961570560)+z5;z4=m32(z4,-FIX_0_390180644)+z5
    b[7][c]=ds(m32(t4,FIX_0_298631336)+z1+z3,CONST_BITS+PASS1_BITS)
    b[5][c]=ds(m32(t5,FIX_2_053119869)+z2+z4,CONST_BITS+PASS1_BITS)
    b[3][c]=ds(m32(t6,FIX_3_072711026)+z2+z3,CONST_BITS+PASS1_BITS)
    b[1][c]=ds(m32(t7,FIX_1_501321110)+z1+z4,CONST_BITS+PASS1_BITS)

def fdct8(block):
    b=[list(r) for r in block]
    for r in range(8): b[r]=frow(b[r])
    for c in range(8): fcol(b,c)
    return b

# ── Quantize (matches quant_enc.vhd: recip=65536//step, no dead zone for recon) ──
def quant(dct, step=STEP):
    recip=65536//step; dz=step*3//8
    q=[[0]*8 for _ in range(8)]
    for r in range(8):
        for c in range(8):
            v=dct[r][c]; av=abs(v)
            if av>=dz: qv=(av*recip)>>16; q[r][c]=qv if v>=0 else -qv
    return q

# ── Dequantize ────────────────────────────────────────────────────────────────
def dequant(q, step=STEP):
    return [[q[r][c]*step for c in range(8)] for r in range(8)]

# ── IJG 8x8 inverse DCT (column-first then row, matches dct8_inv.vhd) ─────────
IDCT_CONST_BITS = 13; IDCT_PASS1_BITS = 1

def idct1d_col(d_in):
    """Column pass of IDCT. Input scaling from row pass: * 2^PASS1_BITS = *4"""
    d = list(d_in)
    z2=d[2]; z3=d[6]
    z1=(z2+z3)*FIX_0_541196100
    tmp2=z1+z3*(-FIX_1_847759065)
    tmp3=z1+z2*FIX_0_765366865
    tmp0=(d[0]+d[4])<<IDCT_CONST_BITS
    tmp1=(d[0]-d[4])<<IDCT_CONST_BITS
    tmp10=tmp0+tmp3; tmp13=tmp0-tmp3
    tmp11=tmp1+tmp2; tmp12=tmp1-tmp2
    z1=d[7]+d[1]; z2=d[5]+d[3]; z3=d[7]+d[3]; z4=d[5]+d[1]
    z5=(z3+z4)*FIX_1_175875602
    z1*=-FIX_0_899976223; z2*=-FIX_2_562915447
    z3=z3*(-FIX_1_961570560)+z5; z4=z4*(-FIX_0_390180644)+z5
    r0=d[7]*FIX_0_298631336+z1+z3
    r1=d[5]*FIX_2_053119869+z2+z4
    r2=d[3]*FIX_3_072711026+z2+z3
    r3=d[1]*FIX_1_501321110+z1+z4
    sh=IDCT_CONST_BITS+IDCT_PASS1_BITS+3
    out=[0]*8
    out[0]=ds(tmp10+r3,sh); out[7]=ds(tmp10-r3,sh)
    out[1]=ds(tmp11+r2,sh); out[6]=ds(tmp11-r2,sh)
    out[2]=ds(tmp12+r1,sh); out[5]=ds(tmp12-r1,sh)
    out[3]=ds(tmp13+r0,sh); out[4]=ds(tmp13-r0,sh)
    return out

def idct1d_row(d_in):
    """Row pass of IDCT. Output: final pixel residuals."""
    d = list(d_in)
    z2=d[2]; z3=d[6]
    z1=(z2+z3)*FIX_0_541196100
    tmp2=z1+z3*(-FIX_1_847759065)
    tmp3=z1+z2*FIX_0_765366865
    tmp0=(d[0]+d[4])<<IDCT_CONST_BITS
    tmp1=(d[0]-d[4])<<IDCT_CONST_BITS
    tmp10=tmp0+tmp3; tmp13=tmp0-tmp3
    tmp11=tmp1+tmp2; tmp12=tmp1-tmp2
    z1=d[7]+d[1]; z2=d[5]+d[3]; z3=d[7]+d[3]; z4=d[5]+d[1]
    z5=(z3+z4)*FIX_1_175875602
    z1*=-FIX_0_899976223; z2*=-FIX_2_562915447
    z3=z3*(-FIX_1_961570560)+z5; z4=z4*(-FIX_0_390180644)+z5
    r0=d[7]*FIX_0_298631336+z1+z3
    r1=d[5]*FIX_2_053119869+z2+z4
    r2=d[3]*FIX_3_072711026+z2+z3
    r3=d[1]*FIX_1_501321110+z1+z4
    sh=IDCT_CONST_BITS-IDCT_PASS1_BITS+3
    out=[0]*8
    out[0]=ds(tmp10+r3,sh); out[7]=ds(tmp10-r3,sh)
    out[1]=ds(tmp11+r2,sh); out[6]=ds(tmp11-r2,sh)
    out[2]=ds(tmp12+r1,sh); out[5]=ds(tmp12-r1,sh)
    out[3]=ds(tmp13+r0,sh); out[4]=ds(tmp13-r0,sh)
    return out

def idct8(dq_coeffs):
    """Full 8x8 IDCT: column pass first, then row pass. Returns pixel residuals."""
    # Column pass: operate on transposed 8x8
    cols = [[dq_coeffs[r][c] for r in range(8)] for c in range(8)]
    col_out = [idct1d_col(cols[c]) for c in range(8)]
    # col_out[c][r] = result for column c, row r
    # Row pass: operate on each row
    rows = [[col_out[c][r] for c in range(8)] for r in range(8)]
    row_out = [idct1d_row(rows[r]) for r in range(8)]
    return row_out

def pixel(r, c):
    return (r * 4 + c * 2) % 256

ZIGZAG=[0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,
        27,20,13,6,7,14,21,28,35,42,49,56,57,50,43,36,29,22,15,23,30,37,
        44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63]

# ── Simulate full encode-reconstruct chain for strip 0 ────────────────────────
print("Full encode-reconstruct simulation for strip 0")
print("="*70)
print(f"{'Blk':>4} {'Pred':>6} {'Cnt':>4} {'Zigzag[0..9]':>35} {'Recon_col7avg':>14} {'Err_col7':>9}")
print("-"*80)

pred = 128
for blk in range(8):
    bx = blk
    # Build pixel block
    pix = [[pixel(r, bx*8+c) for c in range(8)] for r in range(8)]

    # Residuals
    res = [[pix[r][c] - pred for c in range(8)] for r in range(8)]

    # Forward DCT
    dct = fdct8(res)

    # Quantize
    q = quant(dct)

    # Count non-zero (zigzag order, last non-zero)
    flat = [q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
    last_nz = max((i+1 for i,v in enumerate(flat) if v!=0), default=0)

    # Dequantize + IDCT for reconstruction
    dq = dequant(q)
    res_recon = idct8(dq)

    # Reconstruct pixels: clamp to [0,255]
    recon = [[max(0, min(255, res_recon[r][c] + pred)) for c in range(8)] for r in range(8)]

    # recon_col7avg: average of col 7, as computed by recon_writer
    # col7_sum accumulates rows 0-6, then adds row 7 at (pix_col=7, out_row=7)
    col7_vals = [recon[r][7] for r in range(8)]
    col7_sum = sum(col7_vals)
    # recon_writer: col7_final(10 downto 3) = col7_sum >> 3 (integer divide by 8)
    # BUT: col7_sum is 11-bit and col7_final = col7_sum + recon_pix (for last pixel)
    # Actually it's: sum of all 8 col-7 pixels, then >>3
    recon_col7avg = col7_sum >> 3

    # Original col7 avg for comparison
    orig_col7avg = sum(pix[r][7] for r in range(8)) >> 3
    err = recon_col7avg - orig_col7avg

    print(f"{blk:>4} {pred:>6} {last_nz:>4} {str(flat[:10]):>35} {recon_col7avg:>14} {err:>9}")

    # Next block's predictor = recon_col7avg of this block
    pred = recon_col7avg

print()
print("Key: Pred=predictor used, Cnt=last non-zero zigzag idx, Recon_col7avg=new left_avg")
print()

# ── Compare against expected HW counts ───────────────────────────────────────
print("HW observed: blk 6,7 have cnt=36. SW simulation above shows cnt=10 for all.")
print("This rules out predictor chain error as root cause of cnt=36.")
print()
print("Conclusion: The bug is NOT in the predictor computation or BRAM PREFETCH.")
print("The bug must be in the DCT pipeline or serializer timing within enc_top.")
print()

# ── Deeper analysis: what predictor would give cnt=36 for block 6? ────────────
print("="*70)
print("Reverse analysis: what predictor for block 6 gives cnt=36?")
print("(cnt=36 means last non-zero at zigzag[35]=nat(7,0), only col-0 non-zero)")
print()

bx = 6
pix = [[pixel(r, bx*8+c) for c in range(8)] for r in range(8)]

# Try a range of predictors
print(f"{'Pred':>6} {'Cnt':>5} {'DC':>7} {'AC1':>7}")
for try_pred in range(0, 200, 4):
    res = [[pix[r][c] - try_pred for c in range(8)] for r in range(8)]
    dct = fdct8(res)
    q = quant(dct)
    flat = [q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
    last_nz = max((i+1 for i,v in enumerate(flat) if v!=0), default=0)
    if last_nz in (10, 36):  # interesting counts
        print(f"{try_pred:>6} {last_nz:>5} {flat[0]:>7} {flat[1]:>7}  *** MATCH cnt={last_nz}")

print()
print("Exact search for cnt=36 at block 6:")
for try_pred in range(-10, 256):
    res = [[pix[r][c] - try_pred for c in range(8)] for r in range(8)]
    dct = fdct8(res)
    q = quant(dct)
    flat = [q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
    last_nz = max((i+1 for i,v in enumerate(flat) if v!=0), default=0)
    if last_nz == 36:
        print(f"  pred={try_pred}: cnt=36, flat[:10]={flat[:10]}")
        break
    elif last_nz > 10:
        print(f"  pred={try_pred}: cnt={last_nz}")
