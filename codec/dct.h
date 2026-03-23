/*
 * dct.h — 8x8 integer forward and inverse DCT
 *
 * Algorithm: IJG/JPEG-style scaled integer DCT (AAN-based factorization).
 * All arithmetic is fixed-point int32_t — no floating point, no division.
 * Scale factors are absorbed into the quantisation matrices in quant.h.
 *
 * FPGA note: each multiply by a constant maps to a shift-add chain.
 *            The two-pass (row then column) structure maps to a line buffer.
 */

#ifndef DRONE_CODEC_DCT_H
#define DRONE_CODEC_DCT_H

#include <stdint.h>

/*
 * Forward DCT:
 *   in  [8][8]  — residual values, range typically [-255, 255]
 *   out [8][8]  — scaled DCT coefficients (see quant.h for de-scaling)
 *
 * Output is scaled by FDCT_SCALE relative to true DCT, which is removed
 * during dequantisation.  Caller must not interpret raw values directly.
 */
void dct8_forward(const int16_t in[8][8], int32_t out[8][8]);

/*
 * Inverse DCT:
 *   in  [8][8]  — dequantised coefficients (already de-scaled by quant.h)
 *   out [8][8]  — reconstructed residuals, range [-255, 255]
 */
void dct8_inverse(const int32_t in[8][8], int16_t out[8][8]);

#endif /* DRONE_CODEC_DCT_H */
