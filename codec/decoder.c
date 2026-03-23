#include "decoder.h"
#include "dct.h"
#include "quant.h"
#include "predict.h"
#include "deblock.h"
#include <string.h>

static void plane_put16(Plane *p, int px, int py, const uint8_t src[16][16])
{
    for (int r = 0; r < 16; r++) {
        if (py + r < 0 || py + r >= p->height) continue;
        for (int c = 0; c < 16; c++) {
            if (px + c < 0 || px + c >= p->width) continue;
            p->data[(py + r) * p->stride + px + c] = src[r][c];
        }
    }
}

static void plane_put8(Plane *p, int px, int py, const uint8_t src[8][8])
{
    for (int r = 0; r < 8; r++) {
        if (py + r < 0 || py + r >= p->height) continue;
        for (int c = 0; c < 8; c++) {
            if (px + c < 0 || px + c >= p->width) continue;
            p->data[(py + r) * p->stride + px + c] = src[r][c];
        }
    }
}

static void get_neighbours_luma(const Plane *recon, int px, int py,
                                 uint8_t above[16], uint8_t left[16],
                                 int *has_above, int *has_left)
{
    *has_above = (py > 0);
    *has_left  = (px > 0);
    if (*has_above)
        for (int c = 0; c < 16; c++)
            above[c] = recon->data[(py - 1) * recon->stride + px + c];
    if (*has_left)
        for (int r = 0; r < 16; r++)
            left[r] = recon->data[(py + r) * recon->stride + px - 1];
}

static void get_neighbours_chroma(const Plane *recon, int px, int py,
                                   uint8_t above[8], uint8_t left[8],
                                   int *has_above, int *has_left)
{
    *has_above = (py > 0);
    *has_left  = (px > 0);
    if (*has_above)
        for (int c = 0; c < 8; c++)
            above[c] = recon->data[(py - 1) * recon->stride + px + c];
    if (*has_left)
        for (int r = 0; r < 8; r++)
            left[r] = recon->data[(py + r) * recon->stride + px - 1];
}

/* Read and dequantise one 8x8 block from the bitstream */
static void decode_block8(BitstreamReader *bs, int qp,
                           int32_t dqcoeff[8][8])
{
    int16_t qcoeff[8][8];
    memset(qcoeff, 0, sizeof(qcoeff));

    uint32_t count = bs_read_ue(bs);
    if (count > 64) count = 64;

    for (uint32_t i = 0; i < count; i++) {
        int r = ZIGZAG_8x8[i] / 8;
        int c = ZIGZAG_8x8[i] % 8;
        qcoeff[r][c] = (int16_t)bs_read_se(bs);
    }
    dequant8(qcoeff, dqcoeff, qp);
}

/* Reconstruct 8x8 from prediction + dequantised coefficients */
static void reconstruct8(const uint8_t pred[8][8],
                          const int32_t dqcoeff[8][8],
                          uint8_t dst[8][8])
{
    int16_t idct[8][8];
    dct8_inverse(dqcoeff, idct);
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            dst[r][c] = (uint8_t)CLIP8((int)pred[r][c] + (int)idct[r][c]);
}

/* -------------------------------------------------------------------------- */
/* Init                                                                         */
/* -------------------------------------------------------------------------- */

void decoder_init(Decoder *dec, int width, int height, int qp)
{
    memset(dec, 0, sizeof(*dec));
    dec->width  = width;
    dec->height = height;
    dec->qp     = qp;

    dec->recon.width  = width;
    dec->recon.height = height;

    dec->recon.y.data   = &dec->recon_y[0][0];
    dec->recon.y.width  = width;
    dec->recon.y.height = height;
    dec->recon.y.stride = MAX_WIDTH;

    dec->recon.cb.data   = &dec->recon_cb[0][0];
    dec->recon.cb.width  = width / 2;
    dec->recon.cb.height = height / 2;
    dec->recon.cb.stride = MAX_WIDTH / 2;

    dec->recon.cr.data   = &dec->recon_cr[0][0];
    dec->recon.cr.width  = width / 2;
    dec->recon.cr.height = height / 2;
    dec->recon.cr.stride = MAX_WIDTH / 2;
}

/* -------------------------------------------------------------------------- */
/* Decode frame                                                                 */
/* -------------------------------------------------------------------------- */

int decoder_decode_frame(Decoder *dec,
                          const uint8_t *buf, size_t size,
                          FrameType frame_type,
                          Frame *out)
{
    BitstreamReader bs;
    bs_reader_init(&bs, buf, size);

    int qp      = dec->qp;
    int W       = dec->width;
    int H       = dec->height;
    int mb_cols = MB_COLS(W);
    int mb_rows = MB_ROWS(H);

    for (int mb_y = 0; mb_y < mb_rows; mb_y++) {
        for (int mb_x = 0; mb_x < mb_cols; mb_x++) {
            int px  = mb_x * MB_SIZE;
            int py  = mb_y * MB_SIZE;
            int cpx = px / 2;
            int cpy = py / 2;

            int skip = (int)bs_read_bits(&bs, 1);

            if (frame_type == FRAME_I || !skip) {

                uint8_t recon_luma[16][16];

                if (frame_type == FRAME_I) {
                    /* Intra */
                    IntraMode mode = (IntraMode)bs_read_bits(&bs, 2);

                    uint8_t above_l[16], left_l[16];
                    int has_above, has_left;
                    get_neighbours_luma(&dec->recon.y, px, py,
                                        above_l, left_l, &has_above, &has_left);

                    uint8_t pred_luma[16][16];
                    intra_predict_luma(pred_luma,
                                       has_above ? above_l : NULL,
                                       has_left  ? left_l  : NULL,
                                       mode);

                    for (int by = 0; by < 2; by++) {
                        for (int bx = 0; bx < 2; bx++) {
                            int32_t dqcoeff[8][8];
                            decode_block8(&bs, qp, dqcoeff);
                            uint8_t pred8[8][8], dst8[8][8];
                            for (int r = 0; r < 8; r++)
                                for (int c = 0; c < 8; c++)
                                    pred8[r][c] = pred_luma[by*8+r][bx*8+c];
                            reconstruct8(pred8, dqcoeff, dst8);
                            for (int r = 0; r < 8; r++)
                                for (int c = 0; c < 8; c++)
                                    recon_luma[by*8+r][bx*8+c] = dst8[r][c];
                        }
                    }
                    plane_put16(&dec->recon.y, px, py, recon_luma);

                    /* Chroma */
                    for (int comp = 0; comp < 2; comp++) {
                        Plane *rpl = comp ? &dec->recon.cr : &dec->recon.cb;
                        uint8_t above_c[8], left_c[8];
                        int hac, hlc;
                        get_neighbours_chroma(rpl, cpx, cpy,
                                              above_c, left_c, &hac, &hlc);
                        uint8_t pred_c[8][8];
                        intra_predict_chroma(pred_c,
                                             hac ? above_c : NULL,
                                             hlc ? left_c  : NULL,
                                             mode);
                        int32_t dqcoeff[8][8];
                        decode_block8(&bs, qp, dqcoeff);
                        uint8_t dst8[8][8];
                        reconstruct8(pred_c, dqcoeff, dst8);
                        plane_put8(rpl, cpx, cpy, dst8);
                    }

                } else {
                    /* Inter */
                    MotionVector mv;
                    mv.dx = (int16_t)bs_read_se(&bs);
                    mv.dy = (int16_t)bs_read_se(&bs);
                    int has_residual = (int)bs_read_bits(&bs, 1);

                    uint8_t comp_luma[16][16];
                    inter_compensate(comp_luma, &dec->recon.y, px, py, mv);

                    for (int by = 0; by < 2; by++) {
                        for (int bx = 0; bx < 2; bx++) {
                            uint8_t pred8[8][8];
                            for (int r = 0; r < 8; r++)
                                for (int c = 0; c < 8; c++)
                                    pred8[r][c] = comp_luma[by*8+r][bx*8+c];

                            if (has_residual) {
                                int32_t dqcoeff[8][8];
                                decode_block8(&bs, qp, dqcoeff);
                                uint8_t dst8[8][8];
                                reconstruct8(pred8, dqcoeff, dst8);
                                for (int r = 0; r < 8; r++)
                                    for (int c = 0; c < 8; c++)
                                        recon_luma[by*8+r][bx*8+c] = dst8[r][c];
                            } else {
                                for (int r = 0; r < 8; r++)
                                    for (int c = 0; c < 8; c++)
                                        recon_luma[by*8+r][bx*8+c] = pred8[r][c];
                            }
                        }
                    }
                    plane_put16(&dec->recon.y, px, py, recon_luma);

                    /* Chroma */
                    for (int comp = 0; comp < 2; comp++) {
                        Plane *rpl = comp ? &dec->recon.cr : &dec->recon.cb;
                        uint8_t pred8[8][8], dst8[8][8];
                        inter_compensate_chroma(pred8, rpl, px, py, mv);

                        if (has_residual) {
                            int32_t dqcoeff[8][8];
                            decode_block8(&bs, qp, dqcoeff);
                            reconstruct8(pred8, dqcoeff, dst8);
                        } else {
                            memcpy(dst8, pred8, sizeof(dst8));
                        }
                        plane_put8(rpl, cpx, cpy, dst8);
                    }
                }

            } else {
                /* Skip MB: copy from zero-MV reference */
                uint8_t comp_luma[16][16];
                inter_compensate(comp_luma, &dec->recon.y, px, py,
                                 (MotionVector){0, 0});
                plane_put16(&dec->recon.y, px, py, comp_luma);

                for (int comp = 0; comp < 2; comp++) {
                    Plane *rpl = comp ? &dec->recon.cr : &dec->recon.cb;
                    uint8_t dst8[8][8];
                    inter_compensate_chroma(dst8, rpl, px, py,
                                            (MotionVector){0, 0});
                    plane_put8(rpl, cpx, cpy, dst8);
                }
            }

            if (bs.error) return -1;
        }
    }

    /* Deblock before output and before storing as reference */
    deblock_frame(&dec->recon, dec->qp);

    /* Copy reconstructed frame to output */
    if (out) {
        if (out->y.data) {
            for (int r = 0; r < H; r++)
                memcpy(out->y.data + r * out->y.stride,
                       dec->recon.y.data + r * dec->recon.y.stride, W);
        }
        if (out->cb.data) {
            for (int r = 0; r < H/2; r++)
                memcpy(out->cb.data + r * out->cb.stride,
                       dec->recon.cb.data + r * dec->recon.cb.stride, W/2);
        }
        if (out->cr.data) {
            for (int r = 0; r < H/2; r++)
                memcpy(out->cr.data + r * out->cr.stride,
                       dec->recon.cr.data + r * dec->recon.cr.stride, W/2);
        }
    }
    return 0;
}
