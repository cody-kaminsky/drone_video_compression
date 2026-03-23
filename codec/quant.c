#include "quant.h"
#include "common.h"

/*
 * Quantisation step sizes for QP 1–51.
 * Derived from H.264 Table 7-3: Qstep doubles every 6 steps.
 * Base values for QP % 6 = 0..5: {10, 11, 13, 14, 16, 18}
 */
static const int QP_STEP_BASE[6] = { 10, 11, 13, 14, 16, 18 };

int quant_step(int qp)
{
    if (qp < 1)  qp = 1;
    if (qp > 51) qp = 51;
    int shift = (qp - 1) / 6;
    int base  = QP_STEP_BASE[(qp - 1) % 6];
    return base << shift;
}

/*
 * Forward quantisation with dead-zone.
 *
 * Dead-zone: coefficients |c| < threshold are zeroed.
 * Intra uses a larger dead-zone (better rate-distortion for DC-heavy content).
 *
 * Division avoided: multiply by 2^QBITS / step, then shift.
 * On FPGA: reciprocal lookup table indexed by qp (51 entries).
 */
#define QBITS 16

void quant8(const int32_t in[8][8], int16_t out[8][8], int qp, int is_intra)
{
    int step = quant_step(qp);
    /* Dead-zone: half step for inter, 3/8 step for intra */
    int dz = is_intra ? (step * 3) / 8 : step / 2;

    /* Precompute reciprocal: (2^QBITS + step/2) / step */
    int32_t recip = (1 << QBITS) / step;

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            int32_t v = in[r][c];
            int32_t sign = (v >= 0) ? 1 : -1;
            int32_t av   = ABS(v);
            if (av < dz) {
                out[r][c] = 0;
            } else {
                int32_t q = (int32_t)(((int64_t)av * recip) >> QBITS);
                out[r][c] = (int16_t)(sign * q);
            }
        }
    }
}

/*
 * Dequantisation: multiply by step size.
 * No division needed — straightforward multiply.
 */
void dequant8(const int16_t in[8][8], int32_t out[8][8], int qp)
{
    int step = quant_step(qp);

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            out[r][c] = (int32_t)in[r][c] * step;
}
