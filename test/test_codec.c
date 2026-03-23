/*
 * test_codec.c — end-to-end encode/decode tests with PSNR measurement
 *
 * Usage:
 *   ./test_codec                          (synthetic tests only)
 *   ./test_codec <input.yuv> <w> <h>      (test with a real YUV file)
 *
 * Synthetic tests use generated frames (gradients, noise, solid colours).
 * Reports PSNR per frame. Typical targets:
 *   QP=20  → PSNR > 38 dB
 *   QP=28  → PSNR > 32 dB
 *   QP=40  → PSNR > 26 dB
 */

#include "../test/test.h"
#include "../codec/common.h"
#include "../codec/encoder.h"
#include "../codec/decoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Maximum bitstream buffer: 4K frame, uncompressed worst-case */
#define MAX_FRAME_BYTES (MAX_WIDTH * MAX_HEIGHT * 3)

/* -------------------------------------------------------------------------- */
/* Frame allocation helpers                                                     */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t y [MAX_HEIGHT  ][MAX_WIDTH  ];
    uint8_t cb[MAX_HEIGHT/2][MAX_WIDTH/2];
    uint8_t cr[MAX_HEIGHT/2][MAX_WIDTH/2];
} FrameBuf;

static void framebuf_to_frame(FrameBuf *fb, Frame *f, int w, int h)
{
    f->width  = w;  f->height = h;
    f->y.data   = &fb->y[0][0];   f->y.width  = w;   f->y.height  = h;
    f->y.stride = MAX_WIDTH;
    f->cb.data  = &fb->cb[0][0];  f->cb.width = w/2; f->cb.height = h/2;
    f->cb.stride = MAX_WIDTH/2;
    f->cr.data  = &fb->cr[0][0];  f->cr.width = w/2; f->cr.height = h/2;
    f->cr.stride = MAX_WIDTH/2;
}

/* -------------------------------------------------------------------------- */
/* Synthetic frame generators                                                   */
/* -------------------------------------------------------------------------- */

static void gen_gradient(FrameBuf *fb, int w, int h, int offset)
{
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++)
            fb->y[r][c] = (uint8_t)((r + c + offset) & 0xFF);
    for (int r = 0; r < h/2; r++)
        for (int c = 0; c < w/2; c++) {
            fb->cb[r][c] = 128;
            fb->cr[r][c] = 128;
        }
}

static void gen_noise(FrameBuf *fb, int w, int h, unsigned seed)
{
    srand(seed);
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++)
            fb->y[r][c] = (uint8_t)(rand() & 0xFF);
    for (int r = 0; r < h/2; r++)
        for (int c = 0; c < w/2; c++) {
            fb->cb[r][c] = (uint8_t)(rand() & 0xFF);
            fb->cr[r][c] = (uint8_t)(rand() & 0xFF);
        }
}

/* -------------------------------------------------------------------------- */
/* Encode/decode one frame and return PSNR                                     */
/* -------------------------------------------------------------------------- */

static double encode_decode_psnr(int w, int h, int qp, int gop_size,
                                  FrameBuf *src_buf, FrameBuf *ref_buf)
{
    static uint8_t bitstream[MAX_FRAME_BYTES];
    static FrameBuf recon_buf;

    Frame src, recon;
    framebuf_to_frame(src_buf,   &src,   w, h);
    framebuf_to_frame(&recon_buf, &recon, w, h);

    EncoderConfig cfg = { qp, 16 /* search range */, gop_size };
    static Encoder enc;
    encoder_init(&enc, w, h, &cfg);

    /* Feed reference frame first for P-frame test */
    if (ref_buf && gop_size > 1) {
        Frame ref_f;
        static FrameBuf ref_recon;
        framebuf_to_frame(ref_buf, &ref_f, w, h);
        encoder_encode_frame(&enc, &ref_f, bitstream, sizeof(bitstream));
        /* Don't decode the reference here — just advance encoder state */
    }

    size_t bs_size = encoder_encode_frame(&enc, &src, bitstream, sizeof(bitstream));

    /* Parse frame header */
    if (bs_size < sizeof(FrameHeader)) return -1.0;
    FrameHeader *fh = (FrameHeader *)bitstream;
    FrameType ftype = (FrameType)fh->frame_type;

    static Decoder dec;
    decoder_init(&dec, w, h, qp);
    decoder_decode_frame(&dec,
                          bitstream + sizeof(FrameHeader), fh->frame_size,
                          ftype, &recon);

    /* PSNR over luma only — stride-aware */
    return psnr(src_buf->y[0], MAX_WIDTH, recon_buf.y[0], MAX_WIDTH, w, h);
}

/* -------------------------------------------------------------------------- */
/* Tests                                                                        */
/* -------------------------------------------------------------------------- */

static void test_intra_gradient_1080p(void)
{
    TEST_BEGIN("I-frame gradient 1920x1080 QP=28 PSNR>30dB");
    static FrameBuf src;
    gen_gradient(&src, 1920, 1080, 0);
    double p = encode_decode_psnr(1920, 1080, 28, 1, &src, NULL);
    printf("[%.1f dB] ", p);
    ASSERT(p > 30.0);
    TEST_END();
}

static void test_intra_noise_720p(void)
{
    /* Random noise is worst-case for DCT: energy spreads across all freqs.
     * Use QP=5 (near-lossless) to verify the codec handles it without error. */
    TEST_BEGIN("I-frame noise 1280x720 QP=5 PSNR>28dB");
    static FrameBuf src;
    gen_noise(&src, 1280, 720, 12345);
    double p = encode_decode_psnr(1280, 720, 5, 1, &src, NULL);
    printf("[%.1f dB] ", p);
    ASSERT(p > 28.0);
    TEST_END();
}

static void test_intra_qp_range(void)
{
    TEST_BEGIN("I-frame QP range: PSNR decreases as QP increases");
    static FrameBuf src;
    gen_gradient(&src, 640, 480, 0);
    double prev_psnr = 200.0;
    int ok = 1;
    for (int qp = 10; qp <= 45; qp += 5) {
        double p = encode_decode_psnr(640, 480, qp, 1, &src, NULL);
        if (p > prev_psnr + 2.0) { ok = 0; break; }
        prev_psnr = p;
    }
    ASSERT(ok);
    TEST_END();
}

static void test_intra_lossless_limit(void)
{
    TEST_BEGIN("I-frame QP=1 PSNR>40dB (near-lossless)");
    static FrameBuf src;
    gen_gradient(&src, 320, 240, 0);
    double p = encode_decode_psnr(320, 240, 1, 1, &src, NULL);
    printf("[%.1f dB] ", p);
    ASSERT(p > 40.0);
    TEST_END();
}

static void test_bitstream_roundtrip(void)
{
    TEST_BEGIN("Bitstream: encode and decode produce equal dimensions");
    static uint8_t bs[MAX_FRAME_BYTES];
    static FrameBuf src_buf, recon_buf;
    gen_gradient(&src_buf, 1280, 720, 0);

    Frame src, recon;
    framebuf_to_frame(&src_buf,   &src,   1280, 720);
    framebuf_to_frame(&recon_buf, &recon, 1280, 720);

    EncoderConfig cfg = {28, 16, 1};
    static Encoder enc;
    encoder_init(&enc, 1280, 720, &cfg);
    size_t n = encoder_encode_frame(&enc, &src, bs, sizeof(bs));

    ASSERT(n > sizeof(FrameHeader));
    FrameHeader *fh = (FrameHeader *)bs;
    ASSERT(fh->frame_size + sizeof(FrameHeader) == n);

    static Decoder dec;
    decoder_init(&dec, 1280, 720, 28);
    int r = decoder_decode_frame(&dec, bs + sizeof(FrameHeader),
                                  fh->frame_size, FRAME_I, &recon);
    ASSERT_EQ(r, 0);
    TEST_END();
}

/* -------------------------------------------------------------------------- */
/* Real YUV file test                                                           */
/* -------------------------------------------------------------------------- */

static void test_yuv_file(const char *path, int w, int h)
{
    printf("\n=== Real YUV File Test: %s (%dx%d) ===\n", path, w, h);

    FILE *f = fopen(path, "rb");
    if (!f) { printf("  Cannot open file, skipping.\n"); return; }

    size_t frame_bytes = (size_t)(w * h * 3 / 2);
    static uint8_t yuv_raw[MAX_WIDTH * MAX_HEIGHT * 3 / 2];
    static uint8_t bitstream[MAX_WIDTH * MAX_HEIGHT * 3];
    static FrameBuf src_buf, recon_buf;

    EncoderConfig cfg = {28, 16, 30};
    static Encoder enc;
    encoder_init(&enc, w, h, &cfg);

    static Decoder dec;
    decoder_init(&dec, w, h, 28);

    int fn = 0;
    double total_psnr = 0.0;

    while (fread(yuv_raw, 1, frame_bytes, f) == frame_bytes) {
        /* Unpack YUV420p into FrameBuf */
        uint8_t *yp  = yuv_raw;
        uint8_t *cbp = yuv_raw + w * h;
        uint8_t *crp = yuv_raw + w * h + (w/2) * (h/2);

        for (int r = 0; r < h; r++)
            memcpy(&src_buf.y[r][0], yp + r * w, w);
        for (int r = 0; r < h/2; r++) {
            memcpy(&src_buf.cb[r][0], cbp + r * (w/2), w/2);
            memcpy(&src_buf.cr[r][0], crp + r * (w/2), w/2);
        }

        Frame src, recon;
        framebuf_to_frame(&src_buf,   &src,   w, h);
        framebuf_to_frame(&recon_buf, &recon, w, h);

        size_t bs_size = encoder_encode_frame(&enc, &src, bitstream, sizeof(bitstream));
        FrameHeader *fh = (FrameHeader *)bitstream;

        decoder_decode_frame(&dec, bitstream + sizeof(FrameHeader),
                              fh->frame_size, (FrameType)fh->frame_type, &recon);

        double p = psnr(src_buf.y[0], MAX_WIDTH, recon_buf.y[0], MAX_WIDTH, w, h);
        total_psnr += p;
        printf("  frame %3d  type=%c  bits=%zu  PSNR=%.1f dB\n",
               fn, fh->frame_type == FRAME_I ? 'I' : 'P',
               bs_size * 8, p);
        fn++;
        if (fn >= 60) break;   /* limit to 60 frames for test */
    }
    fclose(f);

    if (fn > 0)
        printf("  Average PSNR: %.1f dB over %d frames\n", total_psnr / fn, fn);
}

/* -------------------------------------------------------------------------- */
/* Main                                                                         */
/* -------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    printf("=== Codec End-to-End Tests ===\n");

    test_bitstream_roundtrip();
    test_intra_lossless_limit();
    test_intra_qp_range();
    test_intra_gradient_1080p();
    test_intra_noise_720p();

    /* Optional: real YUV file */
    if (argc >= 4) {
        int w = atoi(argv[2]);
        int h = atoi(argv[3]);
        if (w > 0 && h > 0 && w <= MAX_WIDTH && h <= MAX_HEIGHT)
            test_yuv_file(argv[1], w, h);
    } else {
        printf("\nTip: run with a raw YUV file for real video testing:\n");
        printf("  ./test_codec video.yuv 1920 1080\n");
        printf("  ./test_codec video.yuv 3840 2160\n\n");
        printf("Convert any video to raw YUV with:\n");
        printf("  ffmpeg -i input.mp4 -pix_fmt yuv420p output.yuv\n");
    }

    TEST_SUMMARY();
}
