/*
 * encode.c — CLI encoder
 *
 * Usage:
 *   encode <input.yuv> <width> <height> <fps> <qp> <gop> <output.drn> [search]
 *
 * Example:
 *   encode video.yuv 1920 1080 30 28 30 out.drn 4    (fast, search range 4)
 *   encode video.yuv 1920 1080 30 28 30 out.drn 16   (better, search range 16)
 *   encode video.yuv 3840 2160 30 32 60 out.drn 8
 *
 * Convert any video to raw YUV first:
 *   ffmpeg -i input.mp4 -pix_fmt yuv420p video.yuv
 */

#include "../codec/common.h"
#include "../codec/encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_BUF_BYTES (MAX_WIDTH * MAX_HEIGHT * 3)

static uint8_t yuv_raw  [MAX_WIDTH * MAX_HEIGHT * 3 / 2];
static uint8_t bitstream[FRAME_BUF_BYTES];

static uint8_t y_buf [MAX_HEIGHT  ][MAX_WIDTH  ];
static uint8_t cb_buf[MAX_HEIGHT/2][MAX_WIDTH/2];
static uint8_t cr_buf[MAX_HEIGHT/2][MAX_WIDTH/2];

int main(int argc, char *argv[])
{
    if (argc < 8) {
        fprintf(stderr,
            "Usage: encode <input.yuv> <w> <h> <fps> <qp> <gop> <output.drn>\n"
            "  qp  : 1–51 (lower = better quality, larger file)\n"
            "  gop : I-frame interval (1=intra-only, 30=I+P, ...)\n");
        return 1;
    }

    const char *in_path     = argv[1];
    int         w           = atoi(argv[2]);
    int         h           = atoi(argv[3]);
    int         fps         = atoi(argv[4]);
    int         qp          = atoi(argv[5]);
    int         gop         = atoi(argv[6]);
    const char *out_path    = argv[7];
    int         search      = (argc >= 9) ? atoi(argv[8]) : 4;

    if (w <= 0 || w > MAX_WIDTH || h <= 0 || h > MAX_HEIGHT) {
        fprintf(stderr, "Invalid resolution %dx%d (max %dx%d)\n",
                w, h, MAX_WIDTH, MAX_HEIGHT);
        return 1;
    }
    if (qp < QP_MIN || qp > QP_MAX) {
        fprintf(stderr, "QP must be %d–%d\n", QP_MIN, QP_MAX);
        return 1;
    }
    if (gop < 1) gop = 1;

    FILE *fin  = fopen(in_path,  "rb");
    FILE *fout = fopen(out_path, "wb");
    if (!fin)  { perror(in_path);  return 1; }
    if (!fout) { perror(out_path); fclose(fin); return 1; }

    /* Write stream header */
    StreamHeader sh;
    memcpy(sh.magic, CODEC_MAGIC, 4);
    sh.width      = (uint16_t)w;
    sh.height     = (uint16_t)h;
    sh.fps        = (uint8_t)fps;
    sh.qp         = (uint8_t)qp;
    sh.reserved[0] = sh.reserved[1] = 0;
    fwrite(&sh, sizeof(sh), 1, fout);

    EncoderConfig cfg = { qp, search, gop };
    static Encoder enc;
    encoder_init(&enc, w, h, &cfg);

    size_t frame_bytes = (size_t)(w * h * 3 / 2);
    int    frame_count = 0;
    size_t total_bytes = sizeof(StreamHeader);
    long   t0 = 0;

    printf("Encoding %s  %dx%d @ %dfps  QP=%d  GOP=%d\n",
           in_path, w, h, fps, qp, gop);

    while (fread(yuv_raw, 1, frame_bytes, fin) == frame_bytes) {
        /* Unpack planar YUV420p */
        uint8_t *yp  = yuv_raw;
        uint8_t *cbp = yuv_raw + w * h;
        uint8_t *crp = yuv_raw + w * h + (w/2)*(h/2);

        for (int r = 0; r < h; r++)
            memcpy(y_buf[r], yp + r * w, w);
        for (int r = 0; r < h/2; r++) {
            memcpy(cb_buf[r], cbp + r * (w/2), w/2);
            memcpy(cr_buf[r], crp + r * (w/2), w/2);
        }

        Frame src;
        src.width  = w;  src.height = h;
        src.y.data   = y_buf[0];  src.y.width  = w;   src.y.height  = h;
        src.y.stride = MAX_WIDTH;
        src.cb.data  = cb_buf[0]; src.cb.width = w/2; src.cb.height = h/2;
        src.cb.stride = MAX_WIDTH/2;
        src.cr.data  = cr_buf[0]; src.cr.width = w/2; src.cr.height = h/2;
        src.cr.stride = MAX_WIDTH/2;

        size_t n = encoder_encode_frame(&enc, &src, bitstream, sizeof(bitstream));
        fwrite(bitstream, 1, n, fout);
        total_bytes += n;
        frame_count++;

        if (frame_count % 30 == 0) {
            printf("  %d frames encoded  %.1f KB/frame  %.2f Mbps @ %dfps\n",
                   frame_count,
                   (double)total_bytes / frame_count / 1024.0,
                   (double)total_bytes * 8 / frame_count * fps / 1e6,
                   fps);
            fflush(stdout);
        }
    }

    fclose(fin);
    fclose(fout);

    double bpp = (double)(total_bytes - sizeof(StreamHeader)) * 8.0
                 / (w * h * (double)frame_count);
    printf("\nDone: %d frames, %zu bytes total, %.3f bpp, %.2f Mbps @ %dfps\n",
           frame_count, total_bytes, bpp,
           (double)(total_bytes - sizeof(StreamHeader)) * 8 * fps / frame_count / 1e6,
           fps);
    return 0;
}
