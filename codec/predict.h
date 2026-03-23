/*
 * predict.h — intra and inter prediction
 *
 * Intra: 4 modes (DC, Horizontal, Vertical, Planar) applied to 16x16 MB.
 * Inter: half-pixel motion compensation using 6-tap bilinear filter.
 *        Motion search: full-pixel integer search + half-pixel refinement.
 *
 * FPGA note:
 *   - Intra prediction is a pure datapath (no branching on pixel values).
 *   - Half-pixel interpolation maps to a 6-tap FIR line buffer.
 *   - Motion search is the main area of parallelism on FPGA (multiple
 *     candidate positions evaluated simultaneously).
 */

#ifndef DRONE_CODEC_PREDICT_H
#define DRONE_CODEC_PREDICT_H

#include "common.h"

/* -------------------------------------------------------------------------- */
/* Intra prediction                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Compute intra prediction for a 16x16 luma block.
 *
 * pred     [16][16]  output prediction
 * above    [16]      top neighbour row (NULL if not available)
 * left     [16]      left neighbour column (NULL if not available)
 * mode     IntraMode
 */
void intra_predict_luma(uint8_t pred[16][16],
                        const uint8_t *above,
                        const uint8_t *left,
                        IntraMode mode);

/*
 * Same for 8x8 chroma block.
 */
void intra_predict_chroma(uint8_t pred[8][8],
                          const uint8_t *above,
                          const uint8_t *left,
                          IntraMode mode);

/*
 * Pick the best intra mode for a given 16x16 block by trying all 4 modes
 * and returning the one with lowest SAD vs original.
 */
IntraMode intra_pick_mode(const uint8_t src[16][16],
                          const uint8_t *above,
                          const uint8_t *left);

/* -------------------------------------------------------------------------- */
/* Inter prediction                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Integer-pixel motion search.
 * Searches a ±search_range pixel window (full search).
 *
 * src        16x16 block from current frame
 * ref_plane  full reference plane
 * mb_x, mb_y macroblock top-left in pixels
 * search_range  ±pixels to search (e.g. 16 or 32)
 *
 * Returns best integer MV (units: full pixels).
 * best_sad is set to the SAD of the best match.
 */
MotionVector inter_search_integer(const uint8_t src[16][16],
                                  const Plane *ref,
                                  int mb_x, int mb_y,
                                  int search_range,
                                  int *best_sad);

/*
 * Half-pixel refinement around a given integer MV.
 * Checks the 8 half-pixel positions surrounding the integer best.
 * Returns MV in half-pixel units (divide by 2 for actual offset).
 */
MotionVector inter_refine_halfpel(const uint8_t src[16][16],
                                  const Plane *ref,
                                  int mb_x, int mb_y,
                                  MotionVector int_mv,
                                  int *best_sad);

/*
 * Motion compensation: copy + interpolate a 16x16 block from the reference
 * plane using the given half-pixel MV.
 *
 * dst      [16][16]  output prediction block
 * ref      full reference plane
 * mb_x/y   macroblock position in pixels
 * mv       motion vector in half-pixel units
 */
void inter_compensate(uint8_t dst[16][16],
                      const Plane *ref,
                      int mb_x, int mb_y,
                      MotionVector mv);

/* Same for 8x8 chroma (MV scaled to chroma coordinates automatically) */
void inter_compensate_chroma(uint8_t dst[8][8],
                              const Plane *ref,
                              int mb_x, int mb_y,
                              MotionVector luma_mv);

/* -------------------------------------------------------------------------- */
/* Residual helpers                                                             */
/* -------------------------------------------------------------------------- */

/* Subtract prediction from source to get residual (16x16 luma) */
void residual_compute_luma(const uint8_t src[16][16],
                            const uint8_t pred[16][16],
                            int16_t res[16][16]);

/* Add prediction back to reconstructed residual (clamp to [0,255]) */
void residual_add_luma(const uint8_t pred[16][16],
                       const int16_t res[16][16],
                       uint8_t dst[16][16]);

/* 8x8 chroma versions */
void residual_compute_chroma(const uint8_t src[8][8],
                              const uint8_t pred[8][8],
                              int16_t res[8][8]);

void residual_add_chroma(const uint8_t pred[8][8],
                         const int16_t res[8][8],
                         uint8_t dst[8][8]);

#endif /* DRONE_CODEC_PREDICT_H */
