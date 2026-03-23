/*
 * deblock.h — simple deblocking filter
 *
 * Applied after frame reconstruction to smooth 8x8 block boundary artefacts.
 * Uses H.264-style boundary strength logic: only filters if the difference
 * across a boundary looks like a coding artefact rather than a real edge.
 *
 * FPGA note: processes one boundary pixel at a time using only neighbours
 * p1,p0|q0,q1 — maps directly to a 4-tap sliding window (line buffer).
 */

#ifndef DRONE_CODEC_DEBLOCK_H
#define DRONE_CODEC_DEBLOCK_H

#include "common.h"

/*
 * Apply deblocking filter to a full luma or chroma plane in-place.
 * qp     : quantisation parameter (used to derive filter thresholds)
 * is_chroma : 0 for luma (8x8 boundaries), 1 for chroma (8x8 boundaries)
 */
void deblock_plane(Plane *p, int qp, int is_chroma);

/*
 * Apply deblocking to all three planes of a frame.
 */
void deblock_frame(Frame *f, int qp);

#endif /* DRONE_CODEC_DEBLOCK_H */
