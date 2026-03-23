/*
 * deblock_hw.c — hardware-pipeline deblocking filter
 *
 * One row per tick, 2-row output delay.
 *
 * Key design rule: y_wr advances on EVERY tick (valid or drain).
 * The line buffer is only written when in_valid=1, but the emit
 * counter always advances so the last 2 buffered rows are flushed
 * during the 2 drain ticks after the final valid input row.
 *
 * VHDL translation
 * ----------------
 * The 4-row line buffer maps to a dual-port BRAM18:
 *   Port A (write): stores the vertically-filtered row each clock.
 *   Port B (read): supplies p1/p0/q0/q1 for the horizontal filter.
 *
 * The horizontal and vertical filter comparisons + muxes replicate
 * across all W pixel positions in parallel.  At 3840 pixels that is
 * ~38 k LUTs; serialise to one pixel/clock and add a pixel counter
 * if LUT budget is tight (latency then scales to W cycles/row).
 */

#include "deblock_hw.h"
#include "../quant.h"
#include <string.h>

/* ----------------------------------------------------------------------- */
/* QP → filter thresholds (same formula as deblock.c)                     */
/* ----------------------------------------------------------------------- */
static void get_thresh(int qp, int is_chroma, int *alpha, int *beta)
{
    int step = quant_step(qp);
    *alpha = step + 4;
    *beta  = step / 2 + 2;
    if (*alpha > 128) *alpha = 128;
    if (*beta  >  32) *beta  = 32;
    if (is_chroma) { *alpha /= 2; *beta /= 2; }
}

/* ----------------------------------------------------------------------- */
/* 4-tap boundary filter (branch-free: maps to comparators + MUX in VHDL) */
/* ----------------------------------------------------------------------- */
static void filter_sample_hw(uint8_t *p1, uint8_t *p0,
                              uint8_t *q0, uint8_t *q1,
                              int alpha, int beta)
{
    int ip1 = *p1, ip0 = *p0, iq0 = *q0, iq1 = *q1;
    /* bitwise AND so all three conditions collapse to one MUX select */
    int do_filt = (ABS(ip0 - iq0) < alpha)
                & (ABS(ip1 - ip0) < beta)
                & (ABS(iq1 - iq0) < beta);
    int np0 = (2*ip1 + ip0 + iq1 + 2) >> 2;
    int nq0 = (2*iq1 + iq0 + ip1 + 2) >> 2;
    *p0 = (uint8_t)MUX2(do_filt, np0, ip0);
    *q0 = (uint8_t)MUX2(do_filt, nq0, iq0);
}

/* ======================================================================= */
/* Public API                                                               */
/* ======================================================================= */

void deblock_hw_init(DeblockHwState *s, int qp, int is_chroma,
                     int width, int height)
{
    memset(s, 0, sizeof(*s));
    s->width  = width;
    s->height = height;
    s->y_wr   = 0;
    get_thresh(qp, is_chroma, &s->alpha, &s->beta);
}

/*
 * deblock_hw_tick — one clock cycle.
 *
 * Four pipeline stages (register boundaries in VHDL):
 *
 *   A — vertical filter: apply 4-tap filter at column boundaries (x=8,16,…)
 *       Result stored in a registered row buffer (W flip-flops).
 *   B — write that buffer into line buffer slot [y_wr & 3].
 *       (Only when in_valid=1; drain ticks skip this write.)
 *   C — horizontal filter: if y_wr is a horizontal boundary (y_wr%8==0
 *       and y_wr >= 2), modify the p0 and q0 rows in the line buffer.
 *   D — emit line buffer row [y_wr - 2] as the fully-processed output.
 *       Valid when 0 <= (y_wr - 2) < height.
 *
 * y_wr advances unconditionally every tick so that the 2-tick drain
 * at end-of-frame naturally flushes the last 2 buffered rows.
 */
int deblock_hw_tick(DeblockHwState *s, const uint8_t *in_row, uint8_t in_valid)
{
    s->out_ctrl.valid = 0;
    s->out_ctrl.last  = 0;

    int W     = s->width;
    int H     = s->height;
    int alpha = s->alpha;
    int beta  = s->beta;
    int y     = s->y_wr;

    /* ------------------------------------------------------------------ */
    /* Stage A+B — vertical filter and write into line buffer              */
    /* Skipped on drain ticks (in_valid=0).                                */
    /* ------------------------------------------------------------------ */
    if (in_valid) {
        uint8_t vfilt[DEBLOCK_MAX_W];
        memcpy(vfilt, in_row, (size_t)W);

        /* Apply 4-tap filter at every column boundary x = 8, 16, 24 … */
        for (int bx = 8; bx < W; bx += 8) {
            if (bx < 2 || bx + 1 >= W) continue;
            filter_sample_hw(&vfilt[bx-2], &vfilt[bx-1],
                              &vfilt[bx],   &vfilt[bx+1],
                              alpha, beta);
        }

        /* Write vertically-filtered row into circular line buffer */
        memcpy(s->lb[y & 3], vfilt, (size_t)W);
    }

    /* ------------------------------------------------------------------ */
    /* Stage C — horizontal filter at row boundaries (y % 8 == 0)         */
    /* Reads p1=(y-2), p0=(y-1), q0=(y), uses q1=q0 approximation.        */
    /* ------------------------------------------------------------------ */
    if (in_valid && (y & 7) == 0 && y >= 2) {
        int sp1 = (y - 2) & 3;
        int sp0 = (y - 1) & 3;
        int sq0 = (y    ) & 3;
        int sq1 = sq0;   /* approximate: avoids 1-row extra pipeline delay */

        for (int x = 0; x < W; x++) {
            filter_sample_hw(&s->lb[sp1][x], &s->lb[sp0][x],
                              &s->lb[sq0][x], &s->lb[sq1][x],
                              alpha, beta);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Stage D — emit row y-2 (valid once we have 2 rows in the buffer)   */
    /* Advances every tick (including drain ticks).                        */
    /* ------------------------------------------------------------------ */
    s->y_wr++;   /* register update — happens AT the clock edge */

    int out_y = y - 2;
    if (out_y >= 0 && out_y < H) {
        memcpy(s->out_row, s->lb[out_y & 3], (size_t)W);
        s->out_ctrl.valid = 1;
        s->out_ctrl.last  = (out_y == H - 1) ? 1 : 0;
    }

    return (int)s->out_ctrl.valid;
}
