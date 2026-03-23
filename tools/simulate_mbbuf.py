#!/usr/bin/env python3
"""
Precise cycle-accurate simulation of mb_buffer.vhd PREFETCH/EMIT for strip 0.
Models the BRAM with 1-cycle read latency, FSM transitions, and row_regs loading.
Determines exactly what pixel data each block receives before emitting to the DCT.
"""

W, H = 64, 48
NBLK = W // 8  # 8 blocks per strip
ROW_STRIDE = 512

# ── Test image ────────────────────────────────────────────────────────────────
def pixel(r, c):
    return (r * 4 + c * 2) % 256

# ── BRAM model (flat, 1-cycle read latency) ───────────────────────────────────
lb = {}  # address → 8-pixel tuple

def lb_addr(row, col):
    return row * ROW_STRIDE + col

# ── Write strip 0 into bank 0 (wr_strip_sel=0) ───────────────────────────────
# mb_buffer writes: lb(wr_row, wr_waddr) = 8 pixels
for wr_row in range(8):
    for wr_waddr in range(8):
        pix = tuple(pixel(wr_row, wr_waddr * 8 + c) for c in range(8))
        lb[lb_addr(wr_row, wr_waddr)] = pix
        # Note: bank 0 (rd_strip_sel=0), rd_bank=0

print("BRAM contents (strip 0, bank 0):")
for r in range(8):
    for w in range(8):
        print(f"  lb[row={r}, waddr={w}] = {lb[lb_addr(r,w)]}")

# ── Simulate FSM for all 8 blocks ─────────────────────────────────────────────
# State: rd_fetch_row, rd_waddr, fetch_cnt, fsm, row_regs, rd_data
# Banks: rd_bank = 0 (rd_strip_sel=0 for strip 0)
RD_BANK = 0

print("\n" + "="*70)
print("Simulating PREFETCH sequences for all 8 blocks of strip 0")
print("="*70)

# After IDLE: rd_fetch_row=0, rd_waddr=0, fetch_cnt=0
# After WAIT_RECON→PREFETCH: rd_fetch_row=0, rd_waddr=N+1, fetch_cnt=0

def simulate_prefetch(blk, rd_waddr_start, rd_fetch_row_init=0):
    """
    Simulate PREFETCH for a block starting at rd_waddr=rd_waddr_start.
    Returns the 8 row_regs entries loaded.

    Timing model (VHDL signals update on rising edge, 1 cycle delayed):
      At clock T where fsm transitions to PREFETCH:
        - rd_fetch_row, rd_waddr, fetch_cnt are set (take effect next clock)
        - BRAM read uses OLD rd_fetch_row and OLD rd_waddr (wasted read)
        - rd_data at T+1 = BRAM[OLD fetch_row, OLD waddr]

      At T+1 (PREFETCH fc=0):
        - BRAM reads lb[rd_bank + rd_fetch_row=0, rd_waddr=rd_waddr_start]
        - row_regs NOT updated (fc>0 is FALSE)
        - rd_fetch_row <= 1, fetch_cnt <= 1
        - rd_data at T+2 = lb[0, rd_waddr_start]

      At T+2 (PREFETCH fc=1):
        - BRAM reads lb[rd_bank+1, rd_waddr_start]
        - row_regs(0) <= rd_data = lb[0, rd_waddr_start]  ← loads correctly
        - rd_fetch_row <= 2, fetch_cnt <= 2
        ...

      At T+8 (PREFETCH fc=7):
        - BRAM reads lb[rd_bank+7, rd_waddr_start]  (fc<7 is FALSE, rd_fetch_row stays 7)
        - row_regs(6) <= rd_data = lb[6, rd_waddr_start]
        - rd_fetch_row unchanged (7), fetch_cnt <= 8

      At T+9 (PREFETCH fc=8):
        - row_regs(7) <= rd_data = lb[7, rd_waddr_start]
        - fsm <= CALC
    """
    row_regs = [None] * 8
    rd_data = None  # BRAM output register (1-cycle latency)

    # Simulate what BRAM read happens at each fc
    # fc=0: BRAM reads [0, waddr_start] → rd_data available at fc=1
    # fc=k: BRAM reads [min(k,7), waddr_start] → rd_data available at fc=k+1
    # fc=k>0: row_regs(k-1) <= rd_data from previous cycle

    print(f"\n  Block {blk} (rd_waddr={rd_waddr_start}):")
    print(f"  {'fc':>3}  {'BRAM_read':>20}  {'rd_data used':>20}  {'row_regs updated':>20}")

    # Track what BRAM address is being read at each fc cycle
    bram_addr_history = []  # bram address read at clock T+1+fc (=fc-th PREFETCH clock)

    for fc in range(9):  # fc = current fetch_cnt value at start of this cycle
        # BRAM read at this cycle: uses rd_fetch_row = min(fc, 7) for fc>=0
        # (The transition clock already did a wasted read, here we start from fc=0)
        bram_row = min(fc, 7)  # rd_fetch_row at start of this cycle
        bram_addr = lb_addr(RD_BANK + bram_row, rd_waddr_start)
        bram_val = lb.get(bram_addr, None)
        bram_addr_history.append((bram_row, bram_addr, bram_val))

        # rd_data now has the value from PREVIOUS cycle's BRAM read
        if fc == 0:
            rd_data_now = None  # first cycle, rd_data is stale from transition
        else:
            prev_row, prev_addr, prev_val = bram_addr_history[fc - 1]
            rd_data_now = prev_val

        # Update row_regs: row_regs(fc-1) <= rd_data (if fc > 0)
        upd = ""
        if fc > 0 and rd_data_now is not None:
            row_regs[fc - 1] = rd_data_now
            upd = f"row_regs({fc-1}) = row {fc-1}"

        # Check fc=8: fsm <= CALC (row_regs(7) just loaded)
        if fc == 8:
            # row_regs(7) was loaded in this cycle
            upd = f"row_regs(7) = row 7 -> CALC"

        print(f"  {fc:>3}  BRAM[row={bram_row},w={rd_waddr_start}]={bram_val}  "
              f"rd_data={rd_data_now}  {upd}")

    print(f"\n  row_regs after PREFETCH:")
    all_ok = True
    for r in range(8):
        expected = tuple(pixel(r, rd_waddr_start * 8 + c) for c in range(8))
        match = "OK" if row_regs[r] == expected else "** MISMATCH **"
        if row_regs[r] != expected:
            all_ok = False
        print(f"    row_regs({r}) = {row_regs[r]}  expected={expected}  {match}")

    return row_regs, all_ok

# ── Block 0: IDLE → PREFETCH ──────────────────────────────────────────────────
print("\n--- Block 0: IDLE -> PREFETCH ---")
print("Note: IDLE sets rd_fetch_row=0, rd_waddr=0, fetch_cnt=0 at clock T.")
print("At T (IDLE), BRAM reads lb[0+rd_fetch_row_OLD, rd_waddr_OLD].")
print("After reset, rd_fetch_row=0, rd_waddr=0, so IDLE's wasted read = lb[0,0].")
print("Then PREFETCH starts at T+1 with rd_fetch_row=0, rd_waddr=0.")
row_regs_0, ok0 = simulate_prefetch(blk=0, rd_waddr_start=0)

# ── Blocks 1-7: WAIT_RECON → PREFETCH ────────────────────────────────────────
all_row_regs = [row_regs_0]
all_ok = ok0

for blk in range(1, 8):
    rr, ok = simulate_prefetch(blk=blk, rd_waddr_start=blk)
    all_row_regs.append(rr)
    all_ok = all_ok and ok

print("\n" + "="*70)
print(f"All PREFETCH data correct: {all_ok}")

# ── Now compute residuals and DCT for each block ──────────────────────────────
print("\n" + "="*70)
print("Computing DCT coefficients for each block")

CONST_BITS = 13
PASS1_BITS = 2
FIX_0_298631336=2446; FIX_0_390180644=3196; FIX_0_541196100=4433
FIX_0_765366865=6270; FIX_0_899976223=7373; FIX_1_175875602=9633
FIX_1_501321110=12299; FIX_1_847759065=15137; FIX_1_961570560=16069
FIX_2_053119869=16819; FIX_2_562915447=20995; FIX_3_072711026=25172

def ds(x, n):
    return (x + (1 << (n-1))) >> n

def m32(a, b):
    av = ((a + (1<<24)) % (1<<25)) - (1<<24)
    return av * b

def frow(d):
    t0=d[0]+d[7]; t7=d[0]-d[7]; t1=d[1]+d[6]; t6=d[1]-d[6]
    t2=d[2]+d[5]; t5=d[2]-d[5]; t3=d[3]+d[4]; t4=d[3]-d[4]
    a=t0+t3; c=t0-t3; b=t1+t2; e=t1-t2
    r=[0]*8
    r[0]=(a+b)<<PASS1_BITS; r[4]=(a-b)<<PASS1_BITS
    z=m32(e+c,FIX_0_541196100)
    r[2]=ds(z+m32(c,FIX_0_765366865),CONST_BITS-PASS1_BITS)
    r[6]=ds(z+m32(e,-FIX_1_847759065),CONST_BITS-PASS1_BITS)
    z1=t4+t7; z2=t5+t6; z3=t4+t6; z4=t5+t7
    z5=m32(z3+z4,FIX_1_175875602)
    z1=m32(z1,-FIX_0_899976223); z2=m32(z2,-FIX_2_562915447)
    z3=m32(z3,-FIX_1_961570560)+z5; z4=m32(z4,-FIX_0_390180644)+z5
    r[7]=ds(m32(t4,FIX_0_298631336)+z1+z3,CONST_BITS-PASS1_BITS)
    r[5]=ds(m32(t5,FIX_2_053119869)+z2+z4,CONST_BITS-PASS1_BITS)
    r[3]=ds(m32(t6,FIX_3_072711026)+z2+z3,CONST_BITS-PASS1_BITS)
    r[1]=ds(m32(t7,FIX_1_501321110)+z1+z4,CONST_BITS-PASS1_BITS)
    return r

def fcol(b, c):
    t0=b[0][c]+b[7][c]; t7=b[0][c]-b[7][c]
    t1=b[1][c]+b[6][c]; t6=b[1][c]-b[6][c]
    t2=b[2][c]+b[5][c]; t5=b[2][c]-b[5][c]
    t3=b[3][c]+b[4][c]; t4=b[3][c]-b[4][c]
    a=t0+t3; cc=t0-t3; bb=t1+t2; e=t1-t2
    b[0][c]=ds(a+bb,PASS1_BITS); b[4][c]=ds(a-bb,PASS1_BITS)
    z=m32(e+cc,FIX_0_541196100)
    b[2][c]=ds(z+m32(cc,FIX_0_765366865),CONST_BITS+PASS1_BITS)
    b[6][c]=ds(z+m32(e,-FIX_1_847759065),CONST_BITS+PASS1_BITS)
    z1=t4+t7; z2=t5+t6; z3=t4+t6; z4=t5+t7
    z5=m32(z3+z4,FIX_1_175875602)
    z1=m32(z1,-FIX_0_899976223); z2=m32(z2,-FIX_2_562915447)
    z3=m32(z3,-FIX_1_961570560)+z5; z4=m32(z4,-FIX_0_390180644)+z5
    b[7][c]=ds(m32(t4,FIX_0_298631336)+z1+z3,CONST_BITS+PASS1_BITS)
    b[5][c]=ds(m32(t5,FIX_2_053119869)+z2+z4,CONST_BITS+PASS1_BITS)
    b[3][c]=ds(m32(t6,FIX_3_072711026)+z2+z3,CONST_BITS+PASS1_BITS)
    b[1][c]=ds(m32(t7,FIX_1_501321110)+z1+z4,CONST_BITS+PASS1_BITS)

def fdct8(block):
    b = [list(r) for r in block]
    for r in range(8): b[r] = frow(b[r])
    for c in range(8): fcol(b, c)
    return b

def quant(dct, step=56):
    recip = 65536 // step
    dz = step * 3 // 8
    q = [[0]*8 for _ in range(8)]
    for r in range(8):
        for c in range(8):
            v = dct[r][c]; av = abs(v)
            if av >= dz:
                qv = (av * recip) >> 16
                q[r][c] = qv if v >= 0 else -qv
    return q

ZIGZAG = [0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,
          27,20,13,6,7,14,21,28,35,42,49,56,57,50,43,36,29,22,15,23,30,37,
          44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63]

# Expected HW predictor (from diagnostics: 2 below SW expected)
# SW predictors computed from pixel averages (not IDCT roundtrip)
def sw_pred(blk, pred_hist):
    if blk == 0:
        return 128
    else:
        return pred_hist[blk-1]  # left_avg from prev block

# Compute predictor chain as mb_buffer would (using raw pixel averages,
# since we're simulating the RTL which doesn't have quant error in sim)
def col7_avg(blk):
    """Average of column 7 of block bx in strip 0"""
    col = blk * 8 + 7
    return sum(pixel(r, col) for r in range(8)) // 8

print(f"\n{'Blk':>4} {'Pred':>6} {'Count':>6} {'Zigzag[0..9]':>30}")
print("-" * 55)

pred = 128
pred_hist = []
for blk in range(8):
    if blk == 0:
        pred = 128
    else:
        # has_left only (strip 0, no above)
        pred = pred_hist[-1]  # left_avg = col7_avg of prev block
    pred_hist.append(col7_avg(blk))

    # Build residual block from row_regs
    rr = all_row_regs[blk]
    res = []
    for r in range(8):
        if rr[r] is None:
            res.append([0]*8)
        else:
            row = [rr[r][c] - pred for c in range(8)]
            res.append(row)

    dct = fdct8(res)
    q = quant(dct)
    flat = [q[ZIGZAG[z]>>3][ZIGZAG[z]&7] for z in range(64)]
    last_nz = max((i+1 for i,v in enumerate(flat) if v!=0), default=0)
    print(f"{blk:>4} {pred:>6} {last_nz:>6} {flat[:10]}")

print()
print("Note: pred_hist (col7_avg used as left_avg for next block):", pred_hist)
print()

# Also show what the predictor WOULD be with correct SW computation
print("Showing SW predictor chain (corrected, using col7 of previous block):")
print(f"{'Blk':>4} {'SW_pred':>8} {'col7avg(blk)':>14}")
sw_pred_val = 128
for blk in range(8):
    print(f"{blk:>4} {sw_pred_val:>8} {col7_avg(blk):>14}")
    sw_pred_val = col7_avg(blk)

print()
print("HW observed count=36 for blocks 6,7 means residuals are MUCH LARGER.")
print("If BRAM data is correct (shown above), the issue must be in predictor or")
print("actual pixel data received by DCT (different from row_regs analysis).")
