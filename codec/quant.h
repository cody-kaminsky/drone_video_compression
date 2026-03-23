/*
 * quant.h — quantisation and dequantisation
 *
 * QP (quantisation parameter) 1–51 following H.264 convention.
 * The step size doubles every 6 QP steps.
 *
 * The quantisation matrices also absorb the DCT scale factors so
 * the IDCT path works on properly scaled coefficients.
 *
 * FPGA note: division replaced by multiply-by-reciprocal + shift.
 *            All multipliers are 16-bit, suitable for DSP48 blocks.
 */

#ifndef DRONE_CODEC_QUANT_H
#define DRONE_CODEC_QUANT_H

#include <stdint.h>

/* Dead-zone quantiser (asymmetric, as in H.264) */
void quant8(const int32_t in[8][8], int16_t out[8][8], int qp, int is_intra);

/* Dequantise: multiply by step size (and DCT scale factor) */
void dequant8(const int16_t in[8][8], int32_t out[8][8], int qp);

/* Return the quantisation step size for a given QP */
int quant_step(int qp);

#endif /* DRONE_CODEC_QUANT_H */
