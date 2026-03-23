#!/usr/bin/env python3
"""Dump full coefficient sequences for all blocks. Shows HW vs SW."""
import math

BS_PATH = r"C:/Users/kamin/Vivado_Projects/compression_test/compression_test.sim/sim_1/behav/xsim/bs_out.bin"
W, H, QP = 64, 48, 16

CONST_BITS=13; PASS1_BITS=2
FIX_0_298631336=2446; FIX_0_390180644=3196; FIX_0_541196100=4433
FIX_0_765366865=6270; FIX_0_899976223=7373; FIX_1_175875602=9633
FIX_1_501321110=12299; FIX_1_847759065=15137; FIX_1_961570560=16069
FIX_2_053119869=16819; FIX_2_562915447=20995; FIX_3_072711026=25172

ZIGZAG=[0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,
        27,20,13,6,7,14,21,28,35,42,49,56,57,50,43,36,29,22,15,23,30,37,
        44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63]

QP_STEP_BASE=[10,11,13,14,16,18]
STEP=QP_STEP_BASE[(QP-1)%6]<<((QP-1)//6)

def ds(x,n): return (x+(1<<(n-1)))>>n
def m32(a,b): return ((a+(1<<24))%(1<<25)-(1<<24))*b
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
def quant(dct,step,intra=True):
    recip=65536//step; dz=step*3//8 if intra else step//4
    q=[[0]*8 for _ in range(8)]
    for r in range(8):
        for c in range(8):
            v=dct[r][c]; av=abs(v)
            if av>=dz: qv=(av*recip)>>16; q[r][c]=qv if v>=0 else -qv
    return q

class BS:
    def __init__(self,d): self.d=d; self.bp=0; self.bb=0; self.bc=0
    def rb(self):
        if self.bc==0:
            if self.bp>=len(self.d): return 0
            self.bb=self.d[self.bp]; self.bp+=1; self.bc=8
        b=(self.bb>>7)&1; self.bb=(self.bb<<1)&0xFF; self.bc-=1; return b
    def ue(self):
        l=0
        while self.rb()==0:
            l+=1
            if l>20: return 0
        s=0
        for _ in range(l): s=(s<<1)|self.rb()
        return (1<<l)-1+s
    def se(self):
        u=self.ue()
        if u==0: return 0
        return (u+1)>>1 if u&1 else -(u>>1)

# Build frame
orig=[[((r*4+c*2)%256) for c in range(W)] for r in range(H)]

# SW encode
bc=W//8; br=H//8
ab=[128]*bc; lf=128; fs=True; sw={}
for by in range(br):
    lf=128
    for bx in range(bc):
        aa=ab[bx]; ha=not fs; hl=(bx>0)
        if ha and hl: pd=(aa+lf+1)>>1
        elif ha: pd=aa
        elif hl: pd=lf
        else: pd=128
        px,py=bx*8,by*8
        res=[[orig[py+r][px+c]-pd for c in range(8)] for r in range(8)]
        dct=fdct8(res); q=quant(dct,STEP)
        flat=[]
        for z in range(64):
            nat=ZIGZAG[z]; flat.append(q[nat>>3][nat&7])
        sw[(bx,by)]=flat
        ab[bx]=sum(orig[py+7][px+c] for c in range(8))>>3
        lf=sum(orig[py+r][px+7] for r in range(8))>>3
    fs=False

# HW decode
with open(BS_PATH,'rb') as f: raw=f.read()
bs=BS(raw); _=bs.rb()  # frame type

# Decode ALL blocks and store
hw_all={}
for i in range(bc*br):
    bx=i%bc; by=i//bc
    cnt=bs.ue()
    coeffs=[bs.se() for _ in range(cnt)]+[0]*(64-cnt)
    hw_all[(bx,by)]=(cnt,coeffs)

# Print detailed comparison for strips 0 and 1
for by in range(2):
    print(f"\n{'='*80}")
    print(f"STRIP {by}:")
    for bx in range(bc):
        hw_cnt,hw_f=hw_all[(bx,by)]
        sw_f=sw[(bx,by)]
        sw_cnt=max((i+1 for i,v in enumerate(sw_f) if v!=0),default=0)
        match=(hw_f==sw_f)
        tag="OK" if match else f"MISMATCH cnt=HW:{hw_cnt}/SW:{sw_cnt}"
        print(f"\n  blk({bx},{by}): {tag}")
        if not match:
            # Show first max(hw_cnt,sw_cnt) positions
            n=max(hw_cnt,sw_cnt)
            diffs=[(z,sw_f[z],hw_f[z]) for z in range(n) if sw_f[z]!=hw_f[z]]
            print(f"    HW[0..{hw_cnt-1}]: {hw_f[:hw_cnt]}")
            print(f"    SW[0..{sw_cnt-1}]: {sw_f[:sw_cnt]}")
            print(f"    Diffs (pos,sw,hw): {diffs[:10]}")

# Check if any pairs of consecutive blocks have identical HW coefficients
print(f"\n{'='*80}")
print("CHECKING FOR IDENTICAL CONSECUTIVE HW BLOCKS:")
for by in range(br):
    for bx in range(bc-1):
        cnt0,f0=hw_all[(bx,by)]
        cnt1,f1=hw_all[(bx+1,by)]
        if f0==f1 and cnt0==cnt1 and cnt0>0:
            print(f"  blk({bx},{by}) == blk({bx+1},{by})  cnt={cnt0}")

print(f"\n{'='*80}")
print("ALL BLOCK COUNTS (HW vs SW):")
for by in range(br):
    row_hw=[hw_all[(bx,by)][0] for bx in range(bc)]
    row_sw=[max((i+1 for i,v in enumerate(sw[(bx,by)]) if v!=0),default=0) for bx in range(bc)]
    print(f"  strip {by} HW:{row_hw}")
    print(f"  strip {by} SW:{row_sw}")
