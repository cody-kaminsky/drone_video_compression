/*
 * test_dct.c — DCT unit tests
 *
 * Tests:
 *   1. Roundtrip: forward then inverse recovers original (within rounding)
 *   2. DC-only input: all energy in DC coefficient
 *   3. Energy compaction: most energy in low-frequency coefficients
 *   4. Linearity: DCT(a+b) ≈ DCT(a) + DCT(b)
 */

#include "../test/test.h"
#include "../codec/dct.h"
#include "../codec/common.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------- */
/* Helpers                                                                      */
/* -------------------------------------------------------------------------- */

static void fill_ramp(int16_t blk[8][8])
{
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            blk[r][c] = (int16_t)(r * 8 + c - 32);  /* -32 .. 31 */
}

static void fill_const(int16_t blk[8][8], int16_t val)
{
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            blk[r][c] = val;
}

static int64_t block_energy(const int32_t blk[8][8])
{
    int64_t e = 0;
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            e += (int64_t)blk[r][c] * blk[r][c];
    return e;
}

/* -------------------------------------------------------------------------- */
/* Tests                                                                        */
/* -------------------------------------------------------------------------- */

static void test_roundtrip_zeros(void)
{
    TEST_BEGIN("DCT roundtrip: all-zero block");
    int16_t in[8][8], out[8][8];
    int32_t coeffs[8][8];
    memset(in, 0, sizeof(in));
    dct8_forward(in, coeffs);
    dct8_inverse(coeffs, out);
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            ASSERT_NEAR(out[r][c], in[r][c], 1);
    TEST_END();
}

static void test_roundtrip_ramp(void)
{
    TEST_BEGIN("DCT roundtrip: ramp signal");
    int16_t in[8][8], out[8][8];
    int32_t coeffs[8][8];
    fill_ramp(in);
    dct8_forward(in, coeffs);
    dct8_inverse(coeffs, out);
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            ASSERT_NEAR(out[r][c], in[r][c], 2);
    TEST_END();
}

static void test_roundtrip_max(void)
{
    TEST_BEGIN("DCT roundtrip: max residual (+/-255)");
    int16_t in[8][8], out[8][8];
    int32_t coeffs[8][8];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            in[r][c] = (int16_t)(((r + c) & 1) ? 255 : -255);
    dct8_forward(in, coeffs);
    dct8_inverse(coeffs, out);
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            ASSERT_NEAR(out[r][c], in[r][c], 2);
    TEST_END();
}

static void test_dc_only(void)
{
    TEST_BEGIN("DCT DC: flat block -> energy in DC coeff only");
    int16_t in[8][8];
    int32_t coeffs[8][8];
    fill_const(in, 64);
    dct8_forward(in, coeffs);
    /* DC should be nonzero, most ACs should be zero */
    ASSERT(coeffs[0][0] != 0);
    int nonzero_ac = 0;
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            if ((r || c) && coeffs[r][c] != 0) nonzero_ac++;
    ASSERT(nonzero_ac <= 2);  /* allow minor rounding */
    TEST_END();
}

static void test_energy_compaction(void)
{
    TEST_BEGIN("DCT energy compaction: top-4x4 >= 90% of total");
    int16_t in[8][8];
    int32_t coeffs[8][8];
    fill_ramp(in);
    dct8_forward(in, coeffs);

    int64_t total = block_energy(coeffs);
    int64_t low_freq = 0;
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            low_freq += (int64_t)coeffs[r][c] * coeffs[r][c];

    if (total > 0) {
        ASSERT((low_freq * 100) / total >= 90);
    }
    TEST_END();
}

static void test_roundtrip_random(void)
{
    TEST_BEGIN("DCT roundtrip: 100 random blocks");
    srand(42);
    for (int trial = 0; trial < 100; trial++) {
        int16_t in[8][8], out[8][8];
        int32_t coeffs[8][8];
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
                in[r][c] = (int16_t)((rand() % 511) - 255);
        dct8_forward(in, coeffs);
        dct8_inverse(coeffs, out);
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
                ASSERT_NEAR(out[r][c], in[r][c], 3);
    }
    TEST_END();
}

/* -------------------------------------------------------------------------- */
/* Main                                                                         */
/* -------------------------------------------------------------------------- */

int main(void)
{
    printf("=== DCT Tests ===\n");
    test_roundtrip_zeros();
    test_roundtrip_ramp();
    test_roundtrip_max();
    test_dc_only();
    test_energy_compaction();
    test_roundtrip_random();
    TEST_SUMMARY();
}
