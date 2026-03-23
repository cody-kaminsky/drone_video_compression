#include "predict.h"
#include "common.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Intra prediction — 16x16 luma                                               */
/* -------------------------------------------------------------------------- */

void intra_predict_luma(uint8_t pred[16][16],
                        const uint8_t *above,
                        const uint8_t *left,
                        IntraMode mode)
{
    int r, c;

    switch (mode) {

    case INTRA_DC: {
        int sum = 0, count = 0;
        if (above) { for (c = 0; c < 16; c++) { sum += above[c]; count++; } }
        if (left)  { for (r = 0; r < 16; r++) { sum += left[r];  count++; } }
        uint8_t dc = (count > 0) ? (uint8_t)((sum + count/2) / count) : 128;
        for (r = 0; r < 16; r++)
            for (c = 0; c < 16; c++)
                pred[r][c] = dc;
        break;
    }

    case INTRA_HORIZ:
        for (r = 0; r < 16; r++) {
            uint8_t val = left ? left[r] : 128;
            for (c = 0; c < 16; c++)
                pred[r][c] = val;
        }
        break;

    case INTRA_VERT:
        for (r = 0; r < 16; r++)
            for (c = 0; c < 16; c++)
                pred[r][c] = above ? above[c] : 128;
        break;

    case INTRA_PLANAR: {
        /* Bilinear ramp from top-left to bottom-right */
        int tl = (above && left) ? (above[0]  + left[0]) / 2 :
                  above           ? above[0] :
                  left            ? left[0]  : 128;
        int tr = above ? above[15] : tl;
        int bl = left  ? left[15]  : tl;
        for (r = 0; r < 16; r++) {
            for (c = 0; c < 16; c++) {
                /* Bilinear blend */
                int v = ((16 - r - 1) * (16 - c - 1) * tl
                       + (16 - r - 1) * (c + 1)       * tr
                       + (r + 1)       * (16 - c - 1) * bl
                       + (r + 1)       * (c + 1)       * 128
                       + 128) >> 8;
                pred[r][c] = (uint8_t)CLIP8(v);
            }
        }
        break;
    }
    }
}

/* -------------------------------------------------------------------------- */
/* Intra prediction — 8x8 chroma                                               */
/* -------------------------------------------------------------------------- */

void intra_predict_chroma(uint8_t pred[8][8],
                          const uint8_t *above,
                          const uint8_t *left,
                          IntraMode mode)
{
    int r, c;

    switch (mode) {
    case INTRA_DC: {
        int sum = 0, count = 0;
        if (above) { for (c = 0; c < 8; c++) { sum += above[c]; count++; } }
        if (left)  { for (r = 0; r < 8; r++) { sum += left[r];  count++; } }
        uint8_t dc = (count > 0) ? (uint8_t)((sum + count/2) / count) : 128;
        for (r = 0; r < 8; r++)
            for (c = 0; c < 8; c++)
                pred[r][c] = dc;
        break;
    }
    case INTRA_HORIZ:
        for (r = 0; r < 8; r++) {
            uint8_t val = left ? left[r] : 128;
            for (c = 0; c < 8; c++)
                pred[r][c] = val;
        }
        break;
    case INTRA_VERT:
        for (r = 0; r < 8; r++)
            for (c = 0; c < 8; c++)
                pred[r][c] = above ? above[c] : 128;
        break;
    case INTRA_PLANAR: {
        int tl = (above && left) ? (above[0] + left[0]) / 2 :
                  above           ? above[0] :
                  left            ? left[0]  : 128;
        int tr = above ? above[7] : tl;
        int bl = left  ? left[7]  : tl;
        for (r = 0; r < 8; r++) {
            for (c = 0; c < 8; c++) {
                int v = ((8 - r - 1) * (8 - c - 1) * tl
                       + (8 - r - 1) * (c + 1)      * tr
                       + (r + 1)      * (8 - c - 1) * bl
                       + (r + 1)      * (c + 1)      * 128
                       + 32) >> 6;
                pred[r][c] = (uint8_t)CLIP8(v);
            }
        }
        break;
    }
    }
}

/* -------------------------------------------------------------------------- */
/* Intra mode selection                                                         */
/* -------------------------------------------------------------------------- */

static int sad16(const uint8_t a[16][16], const uint8_t b[16][16])
{
    int s = 0;
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            s += ABS((int)a[r][c] - (int)b[r][c]);
    return s;
}

IntraMode intra_pick_mode(const uint8_t src[16][16],
                          const uint8_t *above,
                          const uint8_t *left)
{
    IntraMode best = INTRA_DC;
    int best_sad = 0x7fffffff;
    uint8_t pred[16][16];

    for (int m = 0; m < NUM_INTRA_MODES; m++) {
        intra_predict_luma(pred, above, left, (IntraMode)m);
        int s = sad16(src, pred);
        if (s < best_sad) { best_sad = s; best = (IntraMode)m; }
    }
    return best;
}

/* -------------------------------------------------------------------------- */
/* Inter — integer-pixel SAD                                                    */
/* -------------------------------------------------------------------------- */

static int sad16_ref(const uint8_t src[16][16], const Plane *ref,
                     int rx, int ry)
{
    int s = 0;
    for (int r = 0; r < 16; r++) {
        int ry2 = ry + r;
        if (ry2 < 0 || ry2 >= ref->height) { s += 255 * 16; continue; }
        for (int c = 0; c < 16; c++) {
            int rx2 = rx + c;
            if (rx2 < 0 || rx2 >= ref->width) { s += 255; continue; }
            s += ABS((int)src[r][c] - (int)ref->data[ry2 * ref->stride + rx2]);
        }
    }
    return s;
}

MotionVector inter_search_integer(const uint8_t src[16][16],
                                  const Plane *ref,
                                  int mb_x, int mb_y,
                                  int search_range,
                                  int *best_sad)
{
    MotionVector best_mv = {0, 0};
    int bs = sad16_ref(src, ref, mb_x, mb_y);
    *best_sad = bs;

    for (int dy = -search_range; dy <= search_range; dy++) {
        for (int dx = -search_range; dx <= search_range; dx++) {
            int s = sad16_ref(src, ref, mb_x + dx, mb_y + dy);
            if (s < *best_sad) {
                *best_sad = s;
                best_mv.dx = (int16_t)dx;
                best_mv.dy = (int16_t)dy;
            }
        }
    }
    return best_mv;
}

/* -------------------------------------------------------------------------- */
/* Half-pixel interpolation (bilinear, FPGA maps to 2-tap FIR)                */
/* -------------------------------------------------------------------------- */

/*
 * Get a half-pixel sample from the reference plane.
 * hx, hy are in half-pixel units (0 = integer position, 1 = half-pixel).
 */
static uint8_t ref_halfpel(const Plane *ref, int x2, int y2)
{
    /* x2, y2 are in half-pixel units */
    int xi = x2 >> 1;
    int yi = y2 >> 1;
    int xf = x2 & 1;
    int yf = y2 & 1;

    xi = CLIP(xi, 0, ref->width  - 1);
    yi = CLIP(yi, 0, ref->height - 1);
    int xi1 = CLIP(xi + xf, 0, ref->width  - 1);
    int yi1 = CLIP(yi + yf, 0, ref->height - 1);

    int a = ref->data[yi  * ref->stride + xi];
    int b = ref->data[yi  * ref->stride + xi1];
    int c = ref->data[yi1 * ref->stride + xi];
    int d = ref->data[yi1 * ref->stride + xi1];

    if (!xf && !yf) return (uint8_t)a;
    if ( xf && !yf) return (uint8_t)((a + b + 1) >> 1);
    if (!xf &&  yf) return (uint8_t)((a + c + 1) >> 1);
    return (uint8_t)((a + b + c + d + 2) >> 2);
}

static int sad16_halfpel(const uint8_t src[16][16], const Plane *ref,
                         int mb_x2, int mb_y2)
{
    /* mb_x2, mb_y2 in half-pixel units */
    int s = 0;
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            s += ABS((int)src[r][c] -
                     (int)ref_halfpel(ref, mb_x2 + c*2, mb_y2 + r*2));
    return s;
}

MotionVector inter_refine_halfpel(const uint8_t src[16][16],
                                  const Plane *ref,
                                  int mb_x, int mb_y,
                                  MotionVector int_mv,
                                  int *best_sad)
{
    /* Convert integer MV to half-pixel units */
    int cx2 = (mb_x + int_mv.dx) * 2;
    int cy2 = (mb_y + int_mv.dy) * 2;

    MotionVector best = { (int16_t)(int_mv.dx * 2), (int16_t)(int_mv.dy * 2) };
    *best_sad = sad16_halfpel(src, ref, cx2, cy2);

    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int s = sad16_halfpel(src, ref, cx2 + dx, cy2 + dy);
            if (s < *best_sad) {
                *best_sad = s;
                best.dx = (int16_t)(int_mv.dx * 2 + dx);
                best.dy = (int16_t)(int_mv.dy * 2 + dy);
            }
        }
    }
    return best;
}

/* -------------------------------------------------------------------------- */
/* Motion compensation                                                          */
/* -------------------------------------------------------------------------- */

void inter_compensate(uint8_t dst[16][16],
                      const Plane *ref,
                      int mb_x, int mb_y,
                      MotionVector mv)
{
    int base_x2 = mb_x * 2 + mv.dx;
    int base_y2 = mb_y * 2 + mv.dy;
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            dst[r][c] = ref_halfpel(ref, base_x2 + c*2, base_y2 + r*2);
}

void inter_compensate_chroma(uint8_t dst[8][8],
                              const Plane *ref,
                              int mb_x, int mb_y,
                              MotionVector luma_mv)
{
    /* Chroma MV is luma MV / 2 (4:2:0), half-pixel units preserved */
    int base_x2 = (mb_x / 2) * 2 + (luma_mv.dx / 2);
    int base_y2 = (mb_y / 2) * 2 + (luma_mv.dy / 2);
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            dst[r][c] = ref_halfpel(ref, base_x2 + c*2, base_y2 + r*2);
}

/* -------------------------------------------------------------------------- */
/* Residual helpers                                                             */
/* -------------------------------------------------------------------------- */

void residual_compute_luma(const uint8_t src[16][16],
                            const uint8_t pred[16][16],
                            int16_t res[16][16])
{
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            res[r][c] = (int16_t)((int)src[r][c] - (int)pred[r][c]);
}

void residual_add_luma(const uint8_t pred[16][16],
                       const int16_t res[16][16],
                       uint8_t dst[16][16])
{
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            dst[r][c] = (uint8_t)CLIP8((int)pred[r][c] + (int)res[r][c]);
}

void residual_compute_chroma(const uint8_t src[8][8],
                              const uint8_t pred[8][8],
                              int16_t res[8][8])
{
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            res[r][c] = (int16_t)((int)src[r][c] - (int)pred[r][c]);
}

void residual_add_chroma(const uint8_t pred[8][8],
                         const int16_t res[8][8],
                         uint8_t dst[8][8])
{
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            dst[r][c] = (uint8_t)CLIP8((int)pred[r][c] + (int)res[r][c]);
}
