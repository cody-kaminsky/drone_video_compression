/*
 * test_hw.c — verify the hardware-pipeline modules produce the same
 *             results as their software-reference counterparts.
 */

#include "test.h"
#include "../codec/dct.h"
#include "../codec/quant.h"
#include "../codec/deblock.h"
#include "../codec/hw/dct_hw.h"
#include "../codec/hw/quant_hw.h"
#include "../codec/hw/deblock_hw.h"
#include <string.h>

/* ----------------------------------------------------------------------- */
/* Helper: drive the DCT forward pipeline for one block, return row count  */
/* ----------------------------------------------------------------------- */
static int drive_fwd(DctFwdHwState *s, const int16_t in[8][8],
                     int32_t out[8][8])
{
    int16_t dummy[8] = {0};
    int out_row = 0;

    /* Push 8 input rows */
    for (int r = 0; r < 8 && out_row < 8; r++) {
        if (dct_fwd_hw_tick(s, in[r], 1) && out_row < 8) {
            for (int c = 0; c < 8; c++) out[out_row][c] = s->cur_out[c];
            out_row++;
        }
    }
    /* COL + EMIT ticks */
    for (int t = 0; t < 16 && out_row < 8; t++) {
        if (dct_fwd_hw_tick(s, dummy, 0) && out_row < 8) {
            for (int c = 0; c < 8; c++) out[out_row][c] = s->cur_out[c];
            out_row++;
        }
    }
    return out_row;
}

static int drive_inv(DctInvHwState *s, const int32_t in[8][8],
                     int16_t out[8][8])
{
    int32_t dummy[8] = {0};
    int out_row = 0;

    for (int r = 0; r < 8 && out_row < 8; r++) {
        if (dct_inv_hw_tick(s, in[r], 1) && out_row < 8) {
            for (int c = 0; c < 8; c++) out[out_row][c] = s->cur_out[c];
            out_row++;
        }
    }
    for (int t = 0; t < 16 && out_row < 8; t++) {
        if (dct_inv_hw_tick(s, dummy, 0) && out_row < 8) {
            for (int c = 0; c < 8; c++) out[out_row][c] = s->cur_out[c];
            out_row++;
        }
    }
    return out_row;
}

/* ----------------------------------------------------------------------- */
/* DCT forward: pipeline output == software reference                      */
/* ----------------------------------------------------------------------- */
static void test_dct_fwd_matches_sw(void)
{
    TEST_BEGIN("DCT forward pipeline matches software");

    int16_t in[8][8];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            in[r][c] = (int16_t)((r * 8 + c) * 3 - 96);

    int32_t ref[8][8];
    dct8_forward(in, ref);

    static DctFwdHwState fwd;
    dct_fwd_hw_init(&fwd);
    int32_t hw[8][8];
    int got = drive_fwd(&fwd, in, hw);
    ASSERT_EQ(got, 8);

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            ASSERT_EQ(hw[r][c], ref[r][c]);

    TEST_END();
}

/* ----------------------------------------------------------------------- */
/* DCT inverse: pipeline output == software reference                      */
/* ----------------------------------------------------------------------- */
static void test_dct_inv_matches_sw(void)
{
    TEST_BEGIN("DCT inverse pipeline matches software");

    int32_t in[8][8];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            in[r][c] = (r == 0 && c == 0) ? 512 : ((r + c) % 3 == 0 ? 64 : 0);

    int16_t ref[8][8];
    dct8_inverse(in, ref);

    static DctInvHwState inv;
    dct_inv_hw_init(&inv);
    int16_t hw[8][8];
    int got = drive_inv(&inv, in, hw);
    ASSERT_EQ(got, 8);

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            ASSERT_EQ(hw[r][c], ref[r][c]);

    TEST_END();
}

/* ----------------------------------------------------------------------- */
/* DCT fwd→inv roundtrip via pipeline                                      */
/* ----------------------------------------------------------------------- */
static void test_dct_pipeline_roundtrip(void)
{
    TEST_BEGIN("DCT fwd+inv pipeline roundtrip");

    int16_t orig[8][8];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            orig[r][c] = (int16_t)((r * 7 + c * 13) % 256 - 128);

    static DctFwdHwState fwd;  dct_fwd_hw_init(&fwd);
    static DctInvHwState inv;  dct_inv_hw_init(&inv);

    int32_t mid[8][8];
    int16_t recon[8][8];
    ASSERT_EQ(drive_fwd(&fwd, orig, mid),  8);
    ASSERT_EQ(drive_inv(&inv, mid,  recon), 8);

    /* Software reference roundtrip */
    int32_t mid_ref[8][8];
    int16_t recon_ref[8][8];
    dct8_forward(orig, mid_ref);
    dct8_inverse(mid_ref, recon_ref);

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            ASSERT_EQ(recon[r][c], recon_ref[r][c]);

    TEST_END();
}

/* ----------------------------------------------------------------------- */
/* Quantiser: pipeline matches software quant8()                           */
/* ----------------------------------------------------------------------- */
static void test_quant_matches_sw(void)
{
    TEST_BEGIN("Quant HW pipeline matches software (QP=28, intra)");

    int32_t in_block[8][8];
    int16_t sw_out[8][8];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            in_block[r][c] = (r * 8 + c) * 100 - 3200;

    quant8(in_block, sw_out, 28, 1);

    static QuantHwState qhw;
    quant_hw_init(&qhw, 28);

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            ASSERT_EQ(quant_hw_tick(&qhw, in_block[r][c], 1, 1), 1);
            ASSERT_EQ((int)qhw.cur_out, (int)sw_out[r][c]);
        }
    }
    TEST_END();
}

static void test_dequant_matches_sw(void)
{
    TEST_BEGIN("Dequant HW pipeline matches software (QP=28)");

    int16_t qcoeffs[8][8];
    int32_t sw_out[8][8];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            qcoeffs[r][c] = (int16_t)((r * 8 + c) - 32);

    dequant8(qcoeffs, sw_out, 28);

    static DequantHwState dqhw;
    dequant_hw_init(&dqhw, 28);

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            ASSERT_EQ(dequant_hw_tick(&dqhw, qcoeffs[r][c], 1), 1);
            ASSERT_EQ(dqhw.cur_out, sw_out[r][c]);
        }
    }
    TEST_END();
}

/* ----------------------------------------------------------------------- */
/* Deblocking pipeline: output matches software reference within 1 LSB     */
/* ----------------------------------------------------------------------- */
static void test_deblock_pipeline(void)
{
    TEST_BEGIN("Deblock HW pipeline matches software (<=1 LSB on boundaries)");

    enum { W = 32, H = 32 };

    /* Synthetic block-patterned plane */
    static uint8_t src[H][W];
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++)
            src[r][c] = (uint8_t)(((r / 8 + c / 8) & 1) ? 200 : 80);

    /* Software reference */
    static uint8_t ref[H][W];
    memcpy(ref, src, sizeof(ref));
    Plane rp = { &ref[0][0], W, H, W };
    deblock_plane(&rp, 28, 0);

    /* Hardware pipeline */
    static DeblockHwState dhw;
    deblock_hw_init(&dhw, 28, 0, W, H);

    static uint8_t hw_out[H][W];
    int out_row = 0;

    for (int r = 0; r < H + 2; r++) {
        const uint8_t *row = (r < H) ? src[r] : src[H-1];
        uint8_t in_valid   = (r < H) ? 1 : 0;
        if (deblock_hw_tick(&dhw, row, in_valid) && out_row < H) {
            memcpy(hw_out[out_row], dhw.out_row, W);
            out_row++;
        }
    }
    ASSERT_EQ(out_row, H);

    int bad = 0;
    for (int r = 0; r < H; r++)
        for (int c = 0; c < W; c++) {
            int d = (int)hw_out[r][c] - (int)ref[r][c];
            if (d < -1 || d > 1) bad++;
        }
    ASSERT_EQ(bad, 0);

    TEST_END();
}

/* ----------------------------------------------------------------------- */

int main(void)
{
    printf("=== Hardware pipeline module tests ===\n\n");

    test_dct_fwd_matches_sw();
    test_dct_inv_matches_sw();
    test_dct_pipeline_roundtrip();
    test_quant_matches_sw();
    test_dequant_matches_sw();
    test_deblock_pipeline();

    TEST_SUMMARY();
}
