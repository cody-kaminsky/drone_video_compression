#include "encoder.h"
#include "dct.h"
#include "quant.h"
#include "predict.h"
#include "deblock.h"
#include <string.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------- */
/* Helpers to copy a 16x16 block in/out of a Plane                            */
/* -------------------------------------------------------------------------- */

static void plane_get16(const Plane *p, int px, int py, uint8_t dst[16][16])
{
    for (int r = 0; r < 16; r++) {
        int row = CLIP(py + r, 0, p->height - 1);
        for (int c = 0; c < 16; c++) {
            int col = CLIP(px + c, 0, p->width - 1);
            dst[r][c] = p->data[row * p->stride + col];
        }
    }
}

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

static void plane_get8(const Plane *p, int px, int py, uint8_t dst[8][8])
{
    for (int r = 0; r < 8; r++) {
        int row = CLIP(py + r, 0, p->height - 1);
        for (int c = 0; c < 8; c++) {
            int col = CLIP(px + c, 0, p->width - 1);
            dst[r][c] = p->data[row * p->stride + col];
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

/* Get above/left neighbour arrays for intra prediction */
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

/* -------------------------------------------------------------------------- */
/* Transform + quantise one 8x8 block and write coefficients to bitstream     */
/* -------------------------------------------------------------------------- */

static void encode_block8(const int16_t res[8][8], int qp, int is_intra,
                            int16_t qcoeff[8][8],
                            int32_t dqcoeff[8][8],
                            BitstreamWriter *bs)
{
    int32_t dct_out[8][8];
    dct8_forward(res, dct_out);
    quant8(dct_out, qcoeff, qp, is_intra);
    dequant8(qcoeff, dqcoeff, qp);

    /* Write coefficients in zig-zag order as signed exp-golomb */
    /* Find last non-zero position for EOB */
    int last_nz = -1;
    for (int i = 0; i < 64; i++) {
        int r = ZIGZAG_8x8[i] / 8;
        int c = ZIGZAG_8x8[i] % 8;
        if (qcoeff[r][c] != 0) last_nz = i;
    }

    /* Write (last_nz + 1) count so decoder knows how many to read */
    bs_write_ue(bs, (uint32_t)(last_nz + 1));

    for (int i = 0; i <= last_nz; i++) {
        int r = ZIGZAG_8x8[i] / 8;
        int c = ZIGZAG_8x8[i] % 8;
        bs_write_se(bs, (int32_t)qcoeff[r][c]);
    }
}

/* -------------------------------------------------------------------------- */
/* Reconstruct one 8x8 block from dequantised coefficients                    */
/* -------------------------------------------------------------------------- */

static void reconstruct_block8(const uint8_t pred[8][8],
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
/* Initialise                                                                   */
/* -------------------------------------------------------------------------- */

void encoder_init(Encoder *enc, int width, int height, const EncoderConfig *cfg)
{
    memset(enc, 0, sizeof(*enc));
    enc->cfg = *cfg;

    enc->recon.width  = width;
    enc->recon.height = height;

    enc->recon.y.data   = &enc->recon_y[0][0];
    enc->recon.y.width  = width;
    enc->recon.y.height = height;
    enc->recon.y.stride = MAX_WIDTH;

    enc->recon.cb.data   = &enc->recon_cb[0][0];
    enc->recon.cb.width  = width / 2;
    enc->recon.cb.height = height / 2;
    enc->recon.cb.stride = MAX_WIDTH / 2;

    enc->recon.cr.data   = &enc->recon_cr[0][0];
    enc->recon.cr.width  = width / 2;
    enc->recon.cr.height = height / 2;
    enc->recon.cr.stride = MAX_WIDTH / 2;
}

/* -------------------------------------------------------------------------- */
/* Encode one frame                                                             */
/* -------------------------------------------------------------------------- */

size_t encoder_encode_frame(Encoder *enc,
                              const Frame *src,
                              uint8_t *out_buf, size_t out_cap)
{
    int is_intra = (enc->frame_count % enc->cfg.gop_size == 0);
    int qp       = enc->cfg.qp;
    int W        = src->width;
    int H        = src->height;
    int mb_cols  = MB_COLS(W);
    int mb_rows  = MB_ROWS(H);

    /* Leave space for frame header (5 bytes) at start of out_buf */
    BitstreamWriter bs;
    bs_writer_init(&bs, out_buf + sizeof(FrameHeader), out_cap - sizeof(FrameHeader));

    for (int mb_y = 0; mb_y < mb_rows; mb_y++) {
        for (int mb_x = 0; mb_x < mb_cols; mb_x++) {
            int px = mb_x * MB_SIZE;
            int py = mb_y * MB_SIZE;
            int cpx = px / 2;
            int cpy = py / 2;

            /* -- Fetch source blocks -- */
            uint8_t src_luma[16][16], src_cb[8][8], src_cr[8][8];
            plane_get16(&src->y,  px,  py,  src_luma);
            plane_get8 (&src->cb, cpx, cpy, src_cb);
            plane_get8 (&src->cr, cpx, cpy, src_cr);

            if (is_intra) {
                /* ---- I-frame macroblock ---- */
                bs_write_bits(&bs, 0, 1);   /* not skip (always 0 for I-frame) */

                /* Neighbour arrays */
                uint8_t above_l[16], left_l[16];
                int has_above, has_left;
                get_neighbours_luma(&enc->recon.y, px, py,
                                    above_l, left_l, &has_above, &has_left);

                IntraMode mode = intra_pick_mode(src_luma,
                                                  has_above ? above_l : NULL,
                                                  has_left  ? left_l  : NULL);
                bs_write_bits(&bs, (uint32_t)mode, 2);

                /* Predict + residual + transform for luma (four 8x8 blocks) */
                uint8_t pred_luma[16][16];
                intra_predict_luma(pred_luma,
                                   has_above ? above_l : NULL,
                                   has_left  ? left_l  : NULL,
                                   mode);

                uint8_t recon_luma[16][16];
                for (int by = 0; by < 2; by++) {
                    for (int bx = 0; bx < 2; bx++) {
                        /* Extract 8x8 sub-blocks */
                        int16_t res8[8][8];
                        uint8_t pred8[8][8], src8[8][8];
                        for (int r = 0; r < 8; r++)
                            for (int c = 0; c < 8; c++) {
                                src8[r][c]  = src_luma [by*8+r][bx*8+c];
                                pred8[r][c] = pred_luma[by*8+r][bx*8+c];
                                res8[r][c]  = (int16_t)(src8[r][c] - pred8[r][c]);
                            }
                        int16_t qcoeff[8][8];
                        int32_t dqcoeff[8][8];
                        encode_block8(res8, qp, 1, qcoeff, dqcoeff, &bs);

                        uint8_t dst8[8][8];
                        reconstruct_block8(pred8, dqcoeff, dst8);
                        for (int r = 0; r < 8; r++)
                            for (int c = 0; c < 8; c++)
                                recon_luma[by*8+r][bx*8+c] = dst8[r][c];
                    }
                }
                plane_put16(&enc->recon.y, px, py, recon_luma);

                /* Chroma — use same intra mode */
                uint8_t above_c[8], left_c[8];
                int hac, hlc;
                get_neighbours_chroma(&enc->recon.cb, cpx, cpy,
                                      above_c, left_c, &hac, &hlc);

                uint8_t pred_cb[8][8], pred_cr[8][8];
                intra_predict_chroma(pred_cb,
                                     hac ? above_c : NULL,
                                     hlc ? left_c  : NULL, mode);
                get_neighbours_chroma(&enc->recon.cr, cpx, cpy,
                                      above_c, left_c, &hac, &hlc);
                intra_predict_chroma(pred_cr,
                                     hac ? above_c : NULL,
                                     hlc ? left_c  : NULL, mode);

                for (int comp = 0; comp < 2; comp++) {
                    uint8_t (*src_c)[8]  = comp ? src_cr  : src_cb;
                    uint8_t (*pred_c)[8] = comp ? pred_cr : pred_cb;
                    Plane   *rpl         = comp ? &enc->recon.cr : &enc->recon.cb;

                    int16_t res8[8][8];
                    for (int r = 0; r < 8; r++)
                        for (int c = 0; c < 8; c++)
                            res8[r][c] = (int16_t)(src_c[r][c] - pred_c[r][c]);

                    int16_t qcoeff[8][8];
                    int32_t dqcoeff[8][8];
                    encode_block8(res8, qp, 1, qcoeff, dqcoeff, &bs);

                    uint8_t dst8[8][8];
                    reconstruct_block8(pred_c, dqcoeff, dst8);
                    plane_put8(rpl, cpx, cpy, dst8);
                }

            } else {
                /* ---- P-frame macroblock ---- */
                int int_sad;
                MotionVector int_mv = inter_search_integer(
                    src_luma, &enc->recon.y, px, py,
                    enc->cfg.search_range, &int_sad);

                int hp_sad;
                MotionVector mv = inter_refine_halfpel(
                    src_luma, &enc->recon.y, px, py, int_mv, &hp_sad);

                /* Compute residual SAD after compensation */
                uint8_t comp_luma[16][16];
                inter_compensate(comp_luma, &enc->recon.y, px, py, mv);

                /* Skip decision: if SAD is very low, don't code residual */
                int residual_sad = 0;
                for (int r = 0; r < 16; r++)
                    for (int c = 0; c < 16; c++)
                        residual_sad += ABS((int)src_luma[r][c] - (int)comp_luma[r][c]);

                int skip_threshold = 16 * 16 * 2; /* ~2 per pixel */
                int skip = (residual_sad < skip_threshold) ? 1 : 0;

                bs_write_bits(&bs, (uint32_t)skip, 1);

                if (!skip) {
                    bs_write_se(&bs, (int32_t)mv.dx);
                    bs_write_se(&bs, (int32_t)mv.dy);

                    /* Check if residual is worth coding */
                    int16_t res8[8][8];
                    int32_t dct_tmp[8][8];
                    int16_t q_tmp[8][8];
                    int has_residual = 0;

                    /* Quick check: quantise one block to see if any nonzero */
                    for (int r = 0; r < 8; r++)
                        for (int c = 0; c < 8; c++)
                            res8[r][c] = (int16_t)((int)src_luma[r][c]
                                                   - (int)comp_luma[r][c]);
                    dct8_forward(res8, dct_tmp);
                    quant8(dct_tmp, q_tmp, qp, 0);
                    for (int i = 0; i < 64; i++)
                        if (((int16_t*)q_tmp)[i]) { has_residual = 1; break; }

                    bs_write_bits(&bs, (uint32_t)has_residual, 1);

                    /* Luma — four 8x8 blocks */
                    uint8_t recon_luma[16][16];
                    for (int by = 0; by < 2; by++) {
                        for (int bx = 0; bx < 2; bx++) {
                            uint8_t pred8[8][8], src8[8][8];
                            int16_t r8[8][8];
                            for (int r = 0; r < 8; r++)
                                for (int c = 0; c < 8; c++) {
                                    src8[r][c]  = src_luma [by*8+r][bx*8+c];
                                    pred8[r][c] = comp_luma[by*8+r][bx*8+c];
                                    r8[r][c]    = (int16_t)(src8[r][c] - pred8[r][c]);
                                }
                            int16_t qcoeff[8][8];
                            int32_t dqcoeff[8][8];
                            if (has_residual) {
                                encode_block8(r8, qp, 0, qcoeff, dqcoeff, &bs);
                                uint8_t dst8[8][8];
                                reconstruct_block8(pred8, dqcoeff, dst8);
                                for (int rr = 0; rr < 8; rr++)
                                    for (int cc = 0; cc < 8; cc++)
                                        recon_luma[by*8+rr][bx*8+cc] = dst8[rr][cc];
                            } else {
                                memset(dqcoeff, 0, sizeof(dqcoeff));
                                for (int r = 0; r < 8; r++)
                                    for (int c = 0; c < 8; c++)
                                        recon_luma[by*8+r][bx*8+c] = pred8[r][c];
                            }
                        }
                    }
                    plane_put16(&enc->recon.y, px, py, recon_luma);

                    /* Chroma */
                    for (int comp = 0; comp < 2; comp++) {
                        const Plane *src_pl = comp ? &src->cr : &src->cb;
                        Plane   *rpl     = comp ? &enc->recon.cr : &enc->recon.cb;
                        uint8_t pred8[8][8], src8[8][8], dst8[8][8];

                        inter_compensate_chroma(pred8, comp ? &enc->recon.cr
                                                            : &enc->recon.cb,
                                                px, py, mv);
                        plane_get8(src_pl, cpx, cpy, src8);

                        if (has_residual) {
                            int16_t r8c[8][8];
                            for (int r = 0; r < 8; r++)
                                for (int c = 0; c < 8; c++)
                                    r8c[r][c] = (int16_t)(src8[r][c] - pred8[r][c]);
                            int16_t qcoeff[8][8];
                            int32_t dqcoeff[8][8];
                            encode_block8(r8c, qp, 0, qcoeff, dqcoeff, &bs);
                            reconstruct_block8(pred8, dqcoeff, dst8);
                        } else {
                            memcpy(dst8, pred8, sizeof(dst8));
                        }
                        plane_put8(rpl, cpx, cpy, dst8);
                    }

                } else {
                    /* Skip: copy reference directly to recon */
                    uint8_t comp_luma[16][16];
                    inter_compensate(comp_luma, &enc->recon.y, px, py,
                                     (MotionVector){0, 0});
                    plane_put16(&enc->recon.y, px, py, comp_luma);

                    for (int comp = 0; comp < 2; comp++) {
                        Plane *rpl = comp ? &enc->recon.cr : &enc->recon.cb;
                        uint8_t dst8[8][8];
                        inter_compensate_chroma(dst8, rpl, px, py,
                                                (MotionVector){0, 0});
                        plane_put8(rpl, cpx, cpy, dst8);
                    }
                }
            }
        }
    }

    bs_write_align(&bs);
    size_t bs_bytes = bs_writer_bytes(&bs);

    /* Deblock the reconstructed reference frame so P-frames predict
     * from a cleaner reference, improving quality over the GOP */
    deblock_frame(&enc->recon, enc->cfg.qp);

    /* Write frame header at start of buffer */
    FrameHeader *fh = (FrameHeader *)out_buf;
    fh->frame_type  = (uint8_t)(is_intra ? FRAME_I : FRAME_P);
    fh->frame_size  = (uint32_t)bs_bytes;

    enc->frame_count++;
    return sizeof(FrameHeader) + bs_bytes;
}
