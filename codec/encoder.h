/*
 * encoder.h — frame encoder
 */

#ifndef DRONE_CODEC_ENCODER_H
#define DRONE_CODEC_ENCODER_H

#include "common.h"
#include "bitstream.h"

typedef struct {
    int qp;             /* quantisation parameter (1–51)         */
    int search_range;   /* motion search range in pixels (e.g. 16) */
    int gop_size;       /* I-frame interval: 1=intra-only, N=I+P   */
} EncoderConfig;

typedef struct {
    EncoderConfig cfg;
    Frame         recon;              /* reconstructed reference frame   */
    uint8_t       recon_y [MAX_HEIGHT][MAX_WIDTH];
    uint8_t       recon_cb[MAX_HEIGHT/2][MAX_WIDTH/2];
    uint8_t       recon_cr[MAX_HEIGHT/2][MAX_WIDTH/2];
    int           frame_count;
} Encoder;

void encoder_init(Encoder *enc, int width, int height, const EncoderConfig *cfg);

/*
 * Encode one raw YUV420p frame.
 * out_buf / out_cap: caller-provided output buffer.
 * Returns number of bytes written (frame header + bitstream).
 */
size_t encoder_encode_frame(Encoder *enc,
                             const Frame *src,
                             uint8_t *out_buf, size_t out_cap);

#endif /* DRONE_CODEC_ENCODER_H */
