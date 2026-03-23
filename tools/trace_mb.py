"""
trace_mb.py -- Cycle-accurate simulation of mb_buffer PREFETCH + BRAM write timing.

Answers:
  1. Does PREFETCH have an off-by-one where row_regs(1) == row_regs(0)?
  2. For strip 0 blocks 0/1 and strip 1 block 0, what does row_regs contain?
  3. What pred_dc does the VHDL compute for each block?

Frame: 64x48, Y = (r*4 + c*2) % 256, QP=28, I-frame.
"""

import numpy as np

# ---------------------------------------------------------------------------
# Frame data
# ---------------------------------------------------------------------------
W, H = 64, 48
frame = np.array([[(r*4 + c*2) % 256 for c in range(W)] for r in range(H)], dtype=np.int32)

ROW_STRIDE = 512

# ---------------------------------------------------------------------------
# BRAM model: lb[row_bram * ROW_STRIDE + word_col]
# row_bram wraps 0..7 (circular for each strip)
# ---------------------------------------------------------------------------
# We track not just values but WHEN each address was last written, to detect
# whether a PREFETCH read arrives before or after the overwrite.
lb_val  = [[None]*8 for _ in range(8)]   # lb_val[row_bram][word_col]
lb_time = [[-1]*8   for _ in range(8)]   # clock when last written

# Also track which FRAME row is stored at each BRAM address at any time
lb_frame_row = [[-1]*8 for _ in range(8)]  # which frame row (0..47) is stored

# ---------------------------------------------------------------------------
# Simulate the pixel write process
# Timing: T=0 is the clock when the first pixel (r=0, c=0) arrives with
#         s_tuser='1'.
#
# Write process notes from VHDL:
#  - s_tuser='1' resets wr_row=0, wr_waddr=0, wr_phase=0, line_cnt=0, y_active='1'
#    BUT y_active is only checked AFTER the tuser update (uses OLD value).
#    So pixel(0,0) is skipped if y_active was '0' before.
#    We assume y_active starts '0' (reset state), so pixel(0,0) is NOT stored.
#    At T=1, y_active='1', wr_phase=0 (reset at T=0). Pixel(0,1) goes to phase 0.
#
# Actually: at T=0 s_tuser='1' fires. In the VHDL:
#   if s_tuser='1' then y_active <= '1'; wr_phase <= 0; ...
#   if y_active='1' ...  -- checks OLD y_active
# If y_active was '0': pixel 0 is skipped. At T=1, y_active='1', wr_phase=0.
# Pixel(0,1) stored at wr_phase=0. So we're off by 1 pixel in the BRAM.
#
# BUT: this also means the LAST pixel in each row wraps around differently.
# For simplicity, let us assume y_active starts as '1' (i.e., this is not the
# first frame, or enc_enable was toggled). This is the typical case in simulation
# after reset releases. We'll check both cases.
#
# For now: assume y_active='1' at T=0 (pixel 0 IS stored). This is the cleaner case.
# ---------------------------------------------------------------------------

def simulate_writes(y_active_init=True):
    """
    Simulate BRAM writes, return list of (T, row_bram, word_col, frame_row, word_pixels)
    and list of strip_rdy events.
    """
    writes = []
    strip_rdys = []

    wr_row   = 0
    wr_waddr = 0
    wr_phase = 0
    line_cnt = 0
    y_active = y_active_init
    wr_buf   = [0]*8

    T = 0
    # Feed all pixels row by row
    for fr in range(H):
        for fc in range(W):
            s_tuser = (fr == 0 and fc == 0)
            s_tlast = (fc == W-1)
            s_tdata = frame[fr, fc]

            # s_tuser handling (updates take effect next cycle for y_active,
            # but in VHDL all in same process so same cycle)
            if s_tuser:
                # These assignments happen first, but y_active check below uses OLD y_active
                new_wr_row = 0; new_wr_waddr = 0; new_wr_phase = 0
                new_line_cnt = 0; new_y_active = True
            else:
                new_wr_row = wr_row; new_wr_waddr = wr_waddr
                new_wr_phase = wr_phase; new_line_cnt = line_cnt
                new_y_active = y_active

            # Pixel storage: uses OLD y_active (before s_tuser updates)
            if y_active and line_cnt < H:
                buf = list(wr_buf)
                buf[wr_phase] = s_tdata
                if wr_phase == 7:
                    # Write to BRAM
                    writes.append((T, wr_row, wr_waddr, fr, list(buf)))
                    lb_val[wr_row][wr_waddr]       = list(buf)
                    lb_time[wr_row][wr_waddr]       = T
                    lb_frame_row[wr_row][wr_waddr]  = fr
                    new_wr_phase = 0
                    if wr_waddr < ROW_STRIDE - 1:
                        new_wr_waddr = wr_waddr + 1
                    wr_buf = [0]*8
                else:
                    buf_new = list(buf)
                    new_wr_phase = wr_phase + 1
                    wr_buf = buf_new

            # s_tlast handling
            if s_tlast:
                new_wr_waddr = 0
                new_wr_phase = 0
                if y_active and line_cnt < H:
                    if wr_row == 7:
                        strip_rdys.append(T)
                        new_wr_row = 0
                    else:
                        new_wr_row = wr_row + 1
                new_line_cnt = line_cnt + 1
                if new_line_cnt == H:
                    new_y_active = False

            # Apply updates
            if s_tuser:
                wr_row = 0; wr_waddr = 0; wr_phase = 0
                line_cnt = 0; y_active = True
                wr_buf = [0]*8
            else:
                wr_row = new_wr_row; wr_waddr = new_wr_waddr
                wr_phase = new_wr_phase; line_cnt = new_line_cnt
                y_active = new_y_active

            if s_tlast:
                # Override from tlast
                wr_waddr = 0; wr_phase = 0
                if y_active and line_cnt <= H:  # note line_cnt already incremented
                    pass  # wr_row was set above
                wr_buf = [0]*8

            T += 1

    return writes, strip_rdys

# ---------------------------------------------------------------------------
# Simulate PREFETCH timing
# ---------------------------------------------------------------------------
def simulate_prefetch(strip_rdy_T, rd_waddr_start, strip_write_events):
    """
    Given the clock when strip_rdy fires, simulate the PREFETCH/CALC/EMIT FSM
    and return what data ends up in row_regs(0..7) for the given block column
    (rd_waddr_start).

    Also returns the timeline of when each row_regs is loaded (T_load) and
    from which BRAM address (row_bram, waddr).

    We model: IDLE→PREFETCH at T=strip_rdy_T+1 (since strip_pend becomes 1 at
    that clock and IDLE sees it at the next evaluation).

    Actually more carefully:
    - T=strip_rdy_T: strip_rdy='1', strip_pend goes from 0 to 1.
    - T=strip_rdy_T+1: IDLE sees strip_pend=1, transitions to PREFETCH,
      sets rd_fetch_row=0, fetch_cnt=0, rd_waddr=rd_waddr_start.
    - T=strip_rdy_T+2: first PREFETCH cycle (fsm=PREFETCH, fetch_cnt=0).
      BRAM reads lb(rd_fetch_row=0, rd_waddr) (using old rfr=0 set at IDLE).

    Parameters
    ----------
    strip_rdy_T : int
        Clock when strip_rdy='1'.
    rd_waddr_start : int
        Which block column (0=first 8 cols, 1=next 8 cols, ...).
    strip_write_events : list of (T, row_bram, word_col, frame_row, pixels)
        All BRAM write events so we can check if a read occurs after an overwrite.
    """
    # Build a lookup: for each (row_bram, waddr), the last write time BEFORE time T
    def last_write_before(row_bram, waddr, T):
        best = (-1, None, -1)  # (time, pixels, frame_row)
        for (wt, wr, wc, fr, pix) in strip_write_events:
            if wr == row_bram and wc == waddr and wt <= T:
                if wt > best[0]:
                    best = (wt, pix, fr)
        return best  # (time, pixels, frame_row)

    # IDLE→PREFETCH transition at T=strip_rdy_T+1
    T_prefetch_start = strip_rdy_T + 1  # fsm=PREFETCH takes effect at +2 actually

    # Signals (post-edge values):
    # At T=strip_rdy_T+1: IDLE sets rd_fetch_row=0, fetch_cnt=0, rd_waddr=rd_waddr_start
    #   fsm=PREFETCH
    # The BRAM at T=strip_rdy_T+1 reads lb(rd_fetch_row[T+1], rd_waddr[T+1])
    # rd_fetch_row[T+1] was set at T (which was IDLE: rfr=0).
    # Wait -- IDLE runs at T=strip_rdy_T+1 and sets rfr=0; this takes effect at T+2.
    # The BRAM at T=strip_rdy_T+1 uses rfr from T (= value set at T-1).

    # Let me track signals cycle by cycle.
    # State at start of each cycle T (= value after previous rising edge):
    rfr = 0          # rd_fetch_row (initialized to 0 before sim starts)
    rwa = rd_waddr_start
    fc  = 0          # fetch_cnt
    rd_data_cur = None  # current rd_data

    row_regs = [None]*8  # will be filled
    row_regs_time = [-1]*8  # when each was loaded
    row_regs_bram_addr = [(-1,-1)]*8  # which BRAM addr

    # We need rd_data to be initialized. Before PREFETCH starts, the BRAM
    # process has been running. What did it last read?
    # At T=strip_rdy_T: BRAM reads lb(rfr=0, rwa=rd_waddr_start) -- but rwa
    # was not set yet (IDLE transition happens at T=strip_rdy_T+1).
    # At T=strip_rdy_T, the FSM is still IDLE (strip_pend just became 1).
    # The BRAM at T=strip_rdy_T reads lb(rfr=whatever, rwa=whatever).
    # We conservatively say rd_data at T=strip_rdy_T+2 = lb(0, rwa_start)
    # since IDLE set rfr=0 and rwa at T=strip_rdy_T+1.

    # Let's be precise. Signal values at each clock:
    # T=srt+1: IDLE transitions. Sets rfr<-0, fc<-0, rwa<-rd_waddr_start, fsm<-PREFETCH.
    #   These take effect at T=srt+2.
    #   BRAM at T=srt+1: uses rfr[srt+1]=0 (from previous init), rwa[srt+1]=?
    #   Actually rwa was set at previous EMIT completion (rd_waddr was bumped).
    #   For block 0: rwa[srt+1]=0 (from IDLE init). BRAM reads lb(0, 0).
    #   rd_data[srt+2] = lb(0, 0).

    srt = strip_rdy_T

    # Reconstruct rd_data at T=srt+2 (= what BRAM outputs at that point)
    # The BRAM process at T=srt+1 reads lb(rfr=0, rwa=rd_waddr_start).
    # rd_data after T=srt+1 = lb(rfr[srt+1], rwa[srt+1]).
    # For IDLE initialization, rfr[srt+1] comes from IDLE at T=srt running
    # (strip_pend was 0 so IDLE did nothing special). So rfr = 0 (default).
    # And rwa[srt+1]: if this is the very first strip, rwa=0 (initial).
    # If this is strip N>0, rwa was set to rd_waddr_start at the previous IDLE.
    # For simplicity assume rwa=rd_waddr_start already.

    # Timeline: T=srt+2 is when PREFETCH begins executing (fetch_cnt=0).
    # rd_data at T=srt+2 = lb(0, rd_waddr_start) [from BRAM read at T=srt+1]

    bram_result_at = {}  # T -> (row_bram, waddr, value at read time)

    # Initialize rd_data to what BRAM computed at T=srt+1:
    # rd_fetch_row at T=srt+1 = 0 (set in IDLE at T=srt, no -- IDLE ran at T=srt+1)
    # Hmm let me just list the BRAM reads:

    # Actually the cleanest approach: track rfr[T] and rwa[T] cycle-by-cycle.
    # T=srt: IDLE, rfr unchanged (assume 0), rwa unchanged (assume rd_waddr_start)
    # T=srt+1: IDLE fires transition: rfr<-0, fc<-0, rwa<-rd_waddr_start, fsm<-PREFETCH
    #   rfr[srt+2]=0, fc[srt+2]=0, rwa[srt+2]=rd_waddr_start
    #   BRAM at T=srt+1: reads lb(rfr[srt+1], rwa[srt+1])
    #     rfr[srt+1]=0 (from before -- IDLE didn't change it at T=srt)
    #     rwa[srt+1]=rd_waddr_start
    #   rd_data[srt+2] = value at lb(0, rd_waddr_start) at T=srt+1

    # Let's define current_bram_state(row_bram, waddr, T) as lb value at time T
    def lb_at_time(row_bram, waddr, T):
        """Return (value, frame_row, write_time) for lb(row_bram, waddr) at time T."""
        best_t = -1; best_val = None; best_fr = -1
        for (wt, wr, wc, fr, pix) in strip_write_events:
            if wr == row_bram and wc == waddr and wt <= T:
                if wt > best_t:
                    best_t = wt; best_val = pix; best_fr = fr
        return best_val, best_fr, best_t

    # BRAM read pipeline: rfr[T] -> BRAM at T -> rd_data[T+1]
    # At T=srt, rfr = 0 (assuming IDLE state from end of previous block or init)
    # Let me just trace from T=srt+2 onwards (PREFETCH start).

    # We need rd_data[srt+2]:
    # BRAM at T=srt+1: rfr[srt+1]=0 (rfr was set in IDLE at T=srt+1, but we need
    # the value BEFORE that assignment, i.e., rfr as of T=srt).
    # For the very first strip: rfr was initialized to 0 in reset/IDLE. rfr[srt+1]=0.
    # For strip N>0 (after EMIT of last block): rfr was last set in PREFETCH of last block,
    # where it reached 7. Then IDLE sets rfr=0 again. So rfr[srt+1]=0 here too.
    # (Actually: IDLE transition happens at T=srt+1, so rfr[srt+2]=0. rfr[srt+1] could be 7.
    # But for strip_rdy_T+1 = first PREFETCH, IDLE ran at T=srt+1, setting rfr=0.
    # So rfr[srt+2]=0, and rfr[srt+1]=<old>.)

    # This is getting complicated. Let me just accept a small uncertainty at the
    # boundary and run the steady-state PREFETCH from T=srt+2.

    # At T=srt+2: fsm=PREFETCH, fc=0, rfr=0, rwa=rd_waddr_start.
    # BRAM at T=srt+2: reads lb(rfr[srt+2]=0, rwa[srt+2]=rd_waddr_start).
    #   rfr[srt+2] = 0 (set by IDLE at T=srt+1). ✓
    # rd_data[srt+3] = lb(0, rd_waddr_start) at T=srt+2.

    print(f"\n{'='*60}")
    print(f"PREFETCH simulation for block col {rd_waddr_start}")
    print(f"strip_rdy fired at T={srt}")
    print(f"PREFETCH starts at T={srt+2}")
    print(f"{'='*60}")

    # State at T=srt+2: fc=0, rfr=0, rwa=rd_waddr_start
    # BRAM read at T=srt+2 uses rfr[srt+2]=0 -> will produce rd_data[srt+3]
    # But we also need rd_data[srt+2] (from BRAM at T=srt+1).
    # For simplicity, assume rd_data[srt+2] = lb(0, rd_waddr_start) at T=srt+1
    # (worst case it's stale from previous block but we show what it would be).

    # Simpler: just track rfr[T] and compute what row_regs gets.
    rfr_seq = {}   # rfr[T] for T >= srt+2
    fc_seq  = {}   # fetch_cnt[T]

    rfr_seq[srt+2] = 0  # set by IDLE at T=srt+1
    fc_seq[srt+2]  = 0  # set by IDLE at T=srt+1

    for step in range(10):
        T_now = srt + 2 + step
        fc_now  = fc_seq.get(T_now, 0)
        rfr_now = rfr_seq.get(T_now, 0)

        # BRAM at T_now reads lb(rfr_now, rwa) -> result available at T_now+1
        bram_read_addr = (rfr_now, rd_waddr_start)
        bram_result_t = T_now + 1  # when rd_data gets this value
        bram_val, bram_fr, bram_wt = lb_at_time(rfr_now, rd_waddr_start, T_now)
        bram_result_at[bram_result_t] = (bram_read_addr, bram_val, bram_fr, bram_wt, T_now)

        # PREFETCH at T_now
        if fc_now > 0 and fc_now <= 8:
            # Store rd_data[T_now] into row_regs(fc_now-1)
            rd_data_now = bram_result_at.get(T_now, (None, None, None, None, None))
            rd_addr, rd_val, rd_fr, rd_wt, rd_issued_at = rd_data_now if rd_data_now[0] else ((None,None),None,None,None,None)
            row_regs[fc_now-1] = rd_val
            row_regs_time[fc_now-1] = T_now
            row_regs_bram_addr[fc_now-1] = rd_addr
            # Check for overwrite hazard
            overwrite = "OK"
            if rd_wt is not None:
                # Find if there's a LATER write to same address between rd_issued_at and T_now
                # (the BRAM was read at T=rd_issued_at with the state at T_now-1, result arrived at T_now)
                overwrite_after = [(wt, fr_w, pix_w) for (wt, wr, wc, fr_w, pix_w) in strip_write_events
                                   if wr == rd_addr[0] and wc == rd_addr[1] and rd_wt < wt <= T_now-1]
                if overwrite_after:
                    overwrite = f"OVERWRITE at T={overwrite_after[-1][0]} (frame row {overwrite_after[-1][1]})"
            print(f"  T={T_now}: fc={fc_now}, row_regs({fc_now-1}) <- lb({rfr_now if rd_addr else '?'},{rd_waddr_start})"
                  f" issued at T={rd_issued_at}, last written at T={rd_wt} (frame row {rd_fr}) => {overwrite}")

        if fc_now < 8:
            rfr_seq[T_now+1] = fc_now  # rd_fetch_row <- fetch_cnt (current)
            fc_seq[T_now+1]  = fc_now + 1
        else:
            # fsm -> CALC
            # Don't need to go further
            print(f"  T={T_now}: fc=8 -> CALC")
            break

    return row_regs, row_regs_time


# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------
print("Simulating BRAM writes (y_active starts '1')...")
writes_ya1, strip_rdys_ya1 = simulate_writes(y_active_init=True)

print(f"Strip rdy events at T={strip_rdys_ya1}")
if len(strip_rdys_ya1) >= 1:
    print(f"  Strip 0 rdy at T={strip_rdys_ya1[0]}")
if len(strip_rdys_ya1) >= 2:
    print(f"  Strip 1 rdy at T={strip_rdys_ya1[1]}")

# Show when lb(row, 0) and lb(row, 1) were written for the first 2 strips
print("\nBRAM write times for word_col=0 and word_col=1:")
for row_b in range(8):
    for wc in [0, 1]:
        relevant = [(wt, fr, pix) for (wt, wr, wc2, fr, pix) in writes_ya1 if wr==row_b and wc2==wc]
        if relevant:
            print(f"  lb({row_b},{wc}): writes at T={[wt for wt,fr,pix in relevant]} "
                  f"-> frame rows {[fr for wt,fr,pix in relevant]}")

# Simulate PREFETCH for strip 0, block 0 (rd_waddr=0)
if strip_rdys_ya1:
    srt0 = strip_rdys_ya1[0]
    rr00, rrt00 = simulate_prefetch(srt0, 0, writes_ya1)
    print(f"\nrow_regs for strip 0, block 0 (rd_waddr=0):")
    for i in range(8):
        if rr00[i] is not None:
            print(f"  row_regs({i}) = {rr00[i]} (loaded T={rrt00[i]})")

# For block 1 of strip 0, we need to estimate when PREFETCH starts.
# block 0 PREFETCH: T=srt0+2 to T=srt0+11 (10 cycles: 9 PREFETCH + 1 CALC)
# block 0 CALC: T=srt0+12
# block 0 EMIT: 8 rows, each takes 4 clocks (DCT FILL stages) = 32 clocks
#   T=srt0+13 to T=srt0+13+31 = T=srt0+44
# block 1 PREFETCH starts at T=srt0+45
# But we also need the EMIT FSM to fire: EMIT transitions to PREFETCH after rd_row=7.
# The timing depends on DCT back-pressure (m_tready). Assume no stalls.
# From previous analysis: each row takes 4 clocks (DCT has 4 fill stages).
# Actually the EMIT FSM advances when "m_tready='1' or m_tvalid_r='0'".
# At the start, m_tvalid_r='0' so it advances freely until DCT accepts.
# We'll estimate: each block takes PREFETCH(10) + CALC(1) + EMIT(8 rows * T_per_row) clocks.
# Per-row time depends on DCT. The DCT s_tready='1' only in fill_stage=0 (every 4 clocks).
# So each EMIT row waits for m_tready, which fires every 4 clocks.
# Approximate block 0 EMIT time: 8 rows * 4 clocks = 32 clocks.
# Block 1 PREFETCH start ~ srt0 + 10 + 1 + 32 + 1 = srt0 + 44.
# But EMIT also needs to wait 1 cycle for m_tvalid to become valid after CALC.
# Let's use srt0 + 10 + 1 + 1 + 32 = srt0 + 44.

T_blk1_prefetch = srt0 + 10 + 1 + 1 + 32  # estimate

print(f"\nEstimated block 1 PREFETCH start: T={T_blk1_prefetch}")
print(f"Strip 1 write of lb(0,1) started at T~{srt0 + 64 + 8} (row 8, word 1)")

# For the BRAM overwrite check: strip 1 starts writing lb(0,0) at T=srt0+8
# (row 8, word 0, arrives 8 pixels after strip_rdy which marks end of row 7).
# Strip 1 writes lb(0,1) (row 8, word 1) at T=srt0+16.
# Block 1 of strip 0 reads lb(row_b, 1) for row_b=0..7 during PREFETCH.
# If T_blk1_prefetch > T_BRAM_overwrite for lb(0,1), there's a hazard.

if strip_rdys_ya1:
    rr01, rrt01 = simulate_prefetch(srt0, 1, writes_ya1)
    print(f"\nrow_regs for strip 0, block 1 (rd_waddr=1):")
    for i in range(8):
        if rr01[i] is not None:
            print(f"  row_regs({i}) = {rr01[i]} (loaded T={rrt01[i]})")

# Now simulate strip 1, block 0
if len(strip_rdys_ya1) >= 2:
    srt1 = strip_rdys_ya1[1]
    rr10, rrt10 = simulate_prefetch(srt1, 0, writes_ya1)
    print(f"\nrow_regs for strip 1, block 0 (rd_waddr=0):")
    for i in range(8):
        if rr10[i] is not None:
            print(f"  row_regs({i}) = {rr10[i]} (loaded T={rrt10[i]})")

# ---------------------------------------------------------------------------
# Now compute what pred_dc the mb_buffer produces for strip 0 blocks 0 and 1
# using the ACTUAL left_avg and above_store logic.
# ---------------------------------------------------------------------------
print("\n\n--- pred_dc computation ---")
print("(For I-frame: predictor = f(left_avg, above_store, has_above, has_left))")

# Strip 0, block 0:
# has_above = (first_strip='0') = False (first strip)
# has_left  = (rd_blk > 0) = False (first block)
# predictor = 128
print("Strip 0, block 0: pred_dc = 128 (no above, no left)")

# Strip 0, block 1:
# has_above = False, has_left = True (rd_blk=1 > 0)
# left_avg = average of col 7 of block 0 (row_regs(7)(63:56) for each row)
# left_avg is accumulated in left_sum during block 0 EMIT, then avg = left_sum(10:3)
# col 7 of block 0 = column 7 of the first 8 columns = frame column 7
# Using CORRECT block 0 data (assume no PREFETCH bug for now):
# row_regs(i) for block 0 = frame row i, cols 0-7
# col 7 pixels for rows 0-7:
col7_blk0 = [frame[r, 7] for r in range(8)]
left_sum_blk1 = sum(col7_blk0)
left_avg_blk1 = left_sum_blk1 >> 3  # left_sum(10:3) = divide by 8
print(f"\nStrip 0, block 1:")
print(f"  col 7 pixels (block 0, rows 0-7): {col7_blk0}")
print(f"  left_sum = {left_sum_blk1}, left_avg = {left_avg_blk1}")
print(f"  pred_dc = left_avg = {left_avg_blk1} (has_left only)")

# BUT: if PREFETCH bug means row_regs(7) = lb(6,0) = frame row 6 instead of row 7:
# The col 7 pixels would be:
# Actually, in the EMIT FSM, col7_pix is taken from row_regs(rd_row)(63:56).
# If row_regs has the PREFETCH bug, the data is shifted.
# Let me compute left_avg with the potentially buggy row_regs.
if rr00[7] is not None:
    col7_actual = [rr00[i][7] for i in range(8) if rr00[i] is not None]  # col 7 = index 7
    if len(col7_actual) == 8:
        ls_actual = sum(col7_actual)
        la_actual = ls_actual >> 3
        print(f"\nWith PREFETCH-loaded row_regs(0..7) for block 0:")
        print(f"  col 7 pixels: {col7_actual}")
        print(f"  left_sum = {ls_actual}, left_avg = {la_actual}")
        pred_dc_blk1 = la_actual
        print(f"  pred_dc for block 1 = {pred_dc_blk1}")

# Also compute with BRAM-buggy row_regs for block 0
# According to my analysis: row_regs(0)=lb(0,0), row_regs(1)=lb(0,0), row_regs(2)=lb(1,0),...
# This means row_regs(1) duplicates row_regs(0), and row_regs(7) = lb(6,0) = frame row 6.
print("\n--- Summary of PREFETCH off-by-one analysis ---")
print("Expected row_regs assignment for block col 0:")
print("  row_regs(k) should = lb(k, waddr) = frame_row_k data")
print("  But PREFETCH actually assigns: row_regs(k) = lb(max(k-1,0), waddr)")
print("  => row_regs(0) = frame_row_0 (correct)")
print("  => row_regs(1) = frame_row_0 (WRONG, should be row_1)")
print("  => row_regs(2) = frame_row_1 (WRONG, should be row_2)")
print("  ...etc.")
print("  => row_regs(7) = frame_row_6 (WRONG, should be row_7)")
print()
print("Verify: does blk(strip=1, blk=0) still match SW if PREFETCH is buggy?")
print("Strip 1 block 0 with buggy PREFETCH:")
# With overwrite: lb(row_b, 0) has strip 1 data for all row_b (strip 0 all overwritten)
# row_regs(0) = lb(0, 0) = frame row 8 (correct strip 1, row 0)
# row_regs(1) = lb(0, 0) = frame row 8 (BUG: should be frame row 9)
# row_regs(2) = lb(1, 0) = frame row 9
# row_regs(3) = lb(2, 0) = frame row 10
# ...
# row_regs(7) = lb(6, 0) = frame row 14 (should be row 15)
buggy_rows = [8, 8, 9, 10, 11, 12, 13, 14]
correct_rows = [8, 9, 10, 11, 12, 13, 14, 15]
print(f"  Buggy row_regs frame rows: {buggy_rows}")
print(f"  Correct frame rows:        {correct_rows}")
print(f"  These differ at index 1 (8 vs 9) and index 7 (14 vs 15)")
print(f"  => HW and SW cannot match for strip 1 block 0 if PREFETCH is buggy")
print()
print("Conclusion: If blk(0,1) in diagnostic = (strip=1, blk=0) and it MATCHES,")
print("then my PREFETCH off-by-one analysis is WRONG (or the naming convention differs).")
print()
print("If blk(0,1) = (strip=0, blk=1) and it MATCHES, that is very surprising given")
print("the BRAM overwrite. Let me check if strip 0 blk 1 could match due to lucky overwrite...")
