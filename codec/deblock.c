#include "deblock.h"
#include "quant.h"
#include "common.h"

/*
 * Thresholds derived from QP (following H.264 table 8-16 simplified):
 *   alpha: controls whether the boundary difference is large enough to filter
 *   beta:  controls whether the block interior is flat enough to be an artefact
 *
 * Both scale with step size so the filter adapts to the quantisation level —
 * at high QP (coarse quant) we filter more aggressively.
 */
static void get_thresholds(int qp, int *alpha, int *beta)
{
    int step = quant_step(qp);
    *alpha = step + 4;
    *beta  = step / 2 + 2;
    /* Cap to avoid over-smoothing at very high QP */
    if (*alpha > 128) *alpha = 128;
    if (*beta  > 32)  *beta  = 32;
}

/*
 * Filter one vertical boundary (pixels to left: p1,p0; to right: q0,q1).
 * Modifies p0 and q0 in-place if the boundary looks like a block artefact.
 *
 * FPGA: 4-tap window, 4 adds, 2 shifts — one pipeline stage per pixel.
 */
static void filter_sample(uint8_t *p1, uint8_t *p0, uint8_t *q0, uint8_t *q1,
                           int alpha, int beta)
{
    int ip1 = *p1, ip0 = *p0, iq0 = *q0, iq1 = *q1;

    if (ABS(ip0 - iq0) >= alpha) return;   /* strong edge — don't touch     */
    if (ABS(ip1 - ip0) >= beta)  return;   /* p-side not flat — real content */
    if (ABS(iq1 - iq0) >= beta)  return;   /* q-side not flat — real content */

    /* Weighted average — smooths artefact while preserving real edges */
    *p0 = (uint8_t)((2 * ip1 + ip0 + iq1 + 2) >> 2);
    *q0 = (uint8_t)((2 * iq1 + iq0 + ip1 + 2) >> 2);
}

/*
 * Filter all vertical block boundaries (between columns of 8x8 blocks).
 * For each boundary at x = 8, 16, 24, ..., filter pixels in that column.
 */
static void filter_vertical(Plane *p, int alpha, int beta)
{
    int w = p->width;
    int h = p->height;

    for (int bx = 8; bx < w; bx += 8) {
        for (int y = 0; y < h; y++) {
            uint8_t *row = p->data + y * p->stride;
            if (bx < 1 || bx + 1 >= w) continue;
            filter_sample(&row[bx - 2], &row[bx - 1],
                          &row[bx],     &row[bx + 1],
                          alpha, beta);
        }
    }
}

/*
 * Filter all horizontal block boundaries (between rows of 8x8 blocks).
 */
static void filter_horizontal(Plane *p, int alpha, int beta)
{
    int w = p->width;
    int h = p->height;

    for (int by = 8; by < h; by += 8) {
        uint8_t *p1_row = p->data + (by - 2) * p->stride;
        uint8_t *p0_row = p->data + (by - 1) * p->stride;
        uint8_t *q0_row = p->data + (by + 0) * p->stride;
        uint8_t *q1_row = p->data + (by + 1) * p->stride;

        if (by < 2 || by + 1 >= h) continue;

        for (int x = 0; x < w; x++) {
            filter_sample(&p1_row[x], &p0_row[x],
                          &q0_row[x], &q1_row[x],
                          alpha, beta);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                   */
/* -------------------------------------------------------------------------- */

void deblock_plane(Plane *p, int qp, int is_chroma)
{
    int alpha, beta;
    get_thresholds(qp, &alpha, &beta);

    /* Chroma is less sensitive — use tighter thresholds */
    if (is_chroma) {
        alpha = alpha / 2;
        beta  = beta  / 2;
    }

    filter_vertical(p, alpha, beta);
    filter_horizontal(p, alpha, beta);
}

void deblock_frame(Frame *f, int qp)
{
    deblock_plane(&f->y,  qp, 0);
    deblock_plane(&f->cb, qp, 1);
    deblock_plane(&f->cr, qp, 1);
}
