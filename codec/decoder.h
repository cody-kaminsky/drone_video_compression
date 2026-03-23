/*
 * decoder.h — frame decoder
 */

#ifndef DRONE_CODEC_DECODER_H
#define DRONE_CODEC_DECODER_H

#include "common.h"
#include "bitstream.h"

typedef struct {
    Frame   recon;
    uint8_t recon_y [MAX_HEIGHT][MAX_WIDTH];
    uint8_t recon_cb[MAX_HEIGHT/2][MAX_WIDTH/2];
    uint8_t recon_cr[MAX_HEIGHT/2][MAX_WIDTH/2];
    int     width;
    int     height;
    int     qp;
} Decoder;

void decoder_init(Decoder *dec, int width, int height, int qp);

/*
 * Decode one frame from bitstream.
 * buf / size: raw frame bitstream (after FrameHeader has been stripped).
 * frame_type: FRAME_I or FRAME_P from FrameHeader.
 * out: decoded frame — caller must set out->y/cb/cr .data pointers.
 */
int decoder_decode_frame(Decoder *dec,
                          const uint8_t *buf, size_t size,
                          FrameType frame_type,
                          Frame *out);

#endif /* DRONE_CODEC_DECODER_H */
