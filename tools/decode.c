/*
 * decode.c — CLI decoder
 *
 * Usage:
 *   decode <input.drn> <output.yuv>
 *
 * Writes raw YUV420p which can be played with:
 *   ffplay -f rawvideo -pixel_format yuv420p -video_size WxH -framerate FPS output.yuv
 *
 * Or convert to MP4:
 *   ffmpeg -f rawvideo -pixel_format yuv420p -video_size WxH -framerate FPS \
 *          -i output.yuv -c:v libx264 output.mp4
 */

#include "../codec/common.h"
#include "../codec/decoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t bitstream[MAX_WIDTH * MAX_HEIGHT * 3];

static uint8_t y_buf [MAX_HEIGHT  ][MAX_WIDTH  ];
static uint8_t cb_buf[MAX_HEIGHT/2][MAX_WIDTH/2];
static uint8_t cr_buf[MAX_HEIGHT/2][MAX_WIDTH/2];

int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr, "Usage: decode <input.drn> <output.yuv>\n");
        return 1;
    }

    FILE *fin  = fopen(argv[1], "rb");
    FILE *fout = fopen(argv[2], "wb");
    if (!fin)  { perror(argv[1]); return 1; }
    if (!fout) { perror(argv[2]); fclose(fin); return 1; }

    /* Read stream header */
    StreamHeader sh;
    if (fread(&sh, sizeof(sh), 1, fin) != 1) {
        fprintf(stderr, "ERROR: failed to read stream header\n");
        return 1;
    }
    if (memcmp(sh.magic, CODEC_MAGIC, 4) != 0) {
        fprintf(stderr, "ERROR: not a DRN1 file\n");
        return 1;
    }

    int w   = sh.width;
    int h   = sh.height;
    int fps = sh.fps;
    int qp  = sh.qp;

    printf("Decoding %s\n  %dx%d @ %dfps  QP=%d\n",
           argv[1], w, h, fps, qp);
    printf("Play with:\n  ffplay -f rawvideo -pixel_format yuv420p"
           " -video_size %dx%d -framerate %d %s\n\n",
           w, h, fps, argv[2]);

    static Decoder dec;
    decoder_init(&dec, w, h, qp);

    Frame out;
    out.width  = w;  out.height = h;
    out.y.data   = y_buf[0];   out.y.width  = w;   out.y.height  = h;
    out.y.stride = MAX_WIDTH;
    out.cb.data  = cb_buf[0];  out.cb.width = w/2; out.cb.height = h/2;
    out.cb.stride = MAX_WIDTH/2;
    out.cr.data  = cr_buf[0];  out.cr.width = w/2; out.cr.height = h/2;
    out.cr.stride = MAX_WIDTH/2;

    int frame_count = 0;
    FrameHeader fh;

    while (fread(&fh, sizeof(fh), 1, fin) == 1) {
        if (fh.frame_size > sizeof(bitstream)) {
            fprintf(stderr, "ERROR: frame %d too large (%u bytes)\n",
                    frame_count, fh.frame_size);
            break;
        }
        if (fread(bitstream, 1, fh.frame_size, fin) != fh.frame_size) {
            fprintf(stderr, "ERROR: truncated frame %d\n", frame_count);
            break;
        }

        int ret = decoder_decode_frame(&dec, bitstream, fh.frame_size,
                                        (FrameType)fh.frame_type, &out);
        if (ret < 0) {
            fprintf(stderr, "WARNING: decode error on frame %d\n", frame_count);
        }

        /* Write YUV420p planar */
        for (int r = 0; r < h; r++)
            fwrite(y_buf[r], 1, w, fout);
        for (int r = 0; r < h/2; r++)
            fwrite(cb_buf[r], 1, w/2, fout);
        for (int r = 0; r < h/2; r++)
            fwrite(cr_buf[r], 1, w/2, fout);

        frame_count++;
        if (frame_count % 30 == 0) {
            printf("  %d frames decoded\r", frame_count);
            fflush(stdout);
        }
    }

    fclose(fin);
    fclose(fout);
    printf("\nDone: %d frames → %s\n", frame_count, argv[2]);
    return 0;
}
