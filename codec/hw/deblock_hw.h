/*
 * deblock_hw.h — hardware-pipeline deblocking filter
 *
 * Processes one row of pixels per clock tick using a 4-row line buffer.
 * The 4 rows (p1, p0, q0, q1) are the four rows needed by the horizontal
 * boundary filter.  The vertical boundary filter operates on the current
 * row only (no inter-row lookahead needed).
 *
 * Pipeline timing
 * ---------------
 *   Ticks 0..1     : fill pipeline (output not yet valid — partial lines)
 *   Tick  N ≥ 2    : output row N-2 (fully filtered, both passes applied)
 *   Drain ticks    : flush remaining rows after frame ends
 *
 *   Output latency : 2 rows
 *   Throughput     : 1 row per tick
 *
 * FPGA resource estimate (one plane, 3840-pixel wide)
 * ----------------------------------------------------
 *   BRAM18  : 4  (4 × 3840 × 8-bit line buffer = ~15 KB)
 *   LUT     : ~200 (4-tap comparators + mux logic, replicated per pixel)
 *   DSP48E2 : 0  (only adds and shifts)
 *   Latency : 2 rows + vertical filter combinatorial delay (~3 ns)
 *
 * For a 4K frame at 30 fps: 2160 rows × 30 fps = 64 800 rows/sec.
 * At 200 MHz one row takes ≥ 3840 / 200e6 = 19.2 µs — comfortably met.
 */

#ifndef DRONE_CODEC_DEBLOCK_HW_H
#define DRONE_CODEC_DEBLOCK_HW_H

#include "common_hw.h"

/* Maximum row width the line buffer is sized for */
#define DEBLOCK_MAX_W  MAX_WIDTH

/* -----------------------------------------------------------------------
 * Deblocking filter pipeline state
 *
 * VHDL: declare as signals:
 *   signal lb       : lb_t;     -- BRAM: 4 × DEBLOCK_MAX_W × 8-bit
 *   signal wr_slot  : natural range 0 to 3;  -- write pointer (circular)
 *   signal y_wr     : natural;  -- absolute row being written
 *   signal out_row  : ub_t;     -- registered output bus
 *   signal out_valid: std_logic;
 * ----------------------------------------------------------------------- */
typedef struct {
    /* 4-row line buffer (circular, index = y_wr % 4) */
    uint8_t  lb[4][DEBLOCK_MAX_W];

    int      y_wr;       /* next row to be written into line buffer           */
    int      width;      /* active frame width  (≤ DEBLOCK_MAX_W)            */
    int      height;     /* active frame height                               */
    int      alpha;      /* boundary difference threshold                     */
    int      beta;       /* interior flatness threshold                       */

    /* Registered output (valid when out_ctrl.valid = 1) */
    uint8_t  out_row[DEBLOCK_MAX_W];
    PipeCtrl out_ctrl;
} DeblockHwState;

/*
 * Initialise state for one plane.
 *   qp         : quantisation parameter (determines filter thresholds)
 *   is_chroma  : 0 for luma, 1 for chroma (halves thresholds)
 *   width/height: active pixels per row/column
 */
void deblock_hw_init(DeblockHwState *s, int qp, int is_chroma,
                     int width, int height);

/*
 * deblock_hw_tick — one clock tick: process one input row.
 *
 *   in_row[width]  : incoming pixel row (valid when in_valid = 1)
 *   in_valid       : 1 when in_row contains valid data
 *
 * Returns 1 when state->out_row[] holds a fully filtered output row.
 * out_ctrl.last = 1 on the last output row of the frame.
 *
 * Call height + 2 times to drain the 2-row pipeline at end of frame
 * (pass in_valid = 0 for the drain ticks).
 *
 * VHDL process outline:
 *   -- Stage A: vertical filter on incoming row (column boundaries)
 *   --          runs combinatorially before the line-buffer write
 *   -- Stage B: write filtered row into line buffer slot wr_slot
 *   -- Stage C: if (y_wr % 8 == 0) and (y_wr >= 2):
 *   --             apply horizontal filter across lb[p1], lb[p0], lb[q0]
 *   --             (modifies the two registered rows in the line buffer)
 *   -- Stage D: output lb[(y_wr - 2) % 4] as the fully-processed row
 */
int deblock_hw_tick(DeblockHwState *s,
                    const uint8_t *in_row, uint8_t in_valid);

#endif /* DRONE_CODEC_DEBLOCK_HW_H */
