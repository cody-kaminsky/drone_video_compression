#!/usr/bin/env python3
"""Identify which (bx, by, predictor) produces the bad block coefficients."""
W, H, QP = 64, 48, 16
STEP = 56
CONST_BITS = 13; PASS1_BITS = 2
FIX_0_298631336=2446; FIX_0_390180644=3196; FIX_0_541196100=4433
FIX_0_765366865=6270; FIX_0_899976223=7373; FIX_1_175875602=9633
FIX_1_501321110=12299; FIX_1_847759065=15137; FIX_1_961570560=16069
FIX_2_053119869=16819; FIX_2_562915447=20995; FIX_3_072711026=25172

def ds(x,n): return (x+(1<<(n-1)))>>n
def m32(a,b):
    av=((a+(1<<24))%(1<<25))-(1<<24)
    return av*b
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
def quant(dct,step):
    recip=65536//step; dz=step*3//8
    q=[[0]*8 for _ in range(8)]
    for r in range(8):
        for c in range(8):
            v=dct[r][c]; av=abs(v)
            if av>=dz: qv=(av*recip)>>16; q[r][c]=qv if v>=0 else -qv
    return q

ZIGZAG=[0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,
        27,20,13,6,7,14,21,28,35,42,49,56,57,50,43,36,29,22,15,23,30,37,
        44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63]

orig=[[((r*4+c*2)%256) for c in range(W)] for r in range(H)]

TARGET=[12,-5,-10,0,0,0,0,0,0,-1]  # bad block zigzag[0..9]

print("Searching for (bx, by, pred) that matches bad block [12,-5,-10,0,0,0,0,0,0,-1]...")
print()

# Also try arbitrary 8-row combos (shifted rows from different blocks)
# Check all normal blocks first
for by in range(H//8):
    for bx in range(W//8):
        for pred in range(256):
            py,px = by*8,bx*8
            res=[[orig[py+r][px+c]-pred for c in range(8)] for r in range(8)]
            dct=fdct8(res); q=quant(dct,STEP)
            flat=[q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
            if flat[:10]==TARGET and all(v==0 for v in flat[10:]):
                print(f"  MATCH: bx={bx} by={by} pred={pred}")

# Try shifted-row blocks: row_buf[0]=prev_row7, row_buf[1..7]=cur_rows[0..6]
# For strip 0, try: prev_row7 from block b-1, cur_rows from block b
print()
print("Searching shifted-row blocks (stale row 7 of prev + rows 0..6 of cur)...")
for prev_bx in range(W//8):
    for bx in range(W//8):
        prev_row7 = [orig[7][prev_bx*8+c] for c in range(8)]
        for pred in range(256):
            # Residuals: stale row 7, then rows 0-6 of bx (strip 0)
            rows = [prev_row7] + [[orig[r][bx*8+c] for c in range(8)] for r in range(7)]
            res = [[rows[r][c]-pred for c in range(8)] for r in range(8)]
            dct=fdct8(res); q=quant(dct,STEP)
            flat=[q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
            if flat[:10]==TARGET and all(v==0 for v in flat[10:]):
                print(f"  MATCH shifted: prev_bx={prev_bx} cur_bx={bx} pred={pred}")

# Also check: what if the 8 rows sent are all the same row?
print()
print("Searching single-row repeated (8 identical rows)...")
for by in range(H//8):
    for bx in range(W//8):
        for r_rep in range(8):
            row = [orig[by*8+r_rep][bx*8+c] for c in range(8)]
            for pred in range(256):
                res = [[row[c]-pred for c in range(8)] for r in range(8)]
                dct=fdct8(res); q=quant(dct,STEP)
                flat=[q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
                if flat[:10]==TARGET and all(v==0 for v in flat[10:]):
                    print(f"  MATCH repeated-row: bx={bx} by={by} r_rep={r_rep} pred={pred}")
