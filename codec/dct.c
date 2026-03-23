/*
 * 8x8 integer DCT / IDCT
 *
 * Based on the Arai-Agui-Nakajima (AAN) factorization, adapted to
 * fixed-point integer arithmetic following the IJG JPEG implementation.
 *
 * CONST_BITS = 13  → constants multiplied by 2^13 = 8192
 * PASS1_BITS = 2   → extra precision carried through the first (row) pass
 *
 * The forward DCT output is scaled — scale factors are absorbed into
 * the quantisation step in quant.c so the IDCT path is exact.
 */

#include "dct.h"
#include "common.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                    */
/* -------------------------------------------------------------------------- */

#define CONST_BITS  13
#define PASS1_BITS  2

/* Right-shift with rounding */
#define DESCALE(x, n)  (((x) + (1 << ((n)-1))) >> (n))

/* Fixed-point versions of required cosine values (scaled by 2^13) */
#define FIX_0_298631336  2446
#define FIX_0_390180644  3196
#define FIX_0_541196100  4433
#define FIX_0_765366865  6270
#define FIX_0_899976223  7373
#define FIX_1_175875602  9633
#define FIX_1_501321110  12299
#define FIX_1_847759065  15137
#define FIX_1_961570560  16069
#define FIX_2_053119869  16819
#define FIX_2_562915447  20995
#define FIX_3_072711026  25172

/* -------------------------------------------------------------------------- */
/* Forward DCT — row pass (operates on one row of 8 int32_t values)           */
/* -------------------------------------------------------------------------- */

static void fdct_row(int32_t *d)
{
    int32_t tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
    int32_t tmp10, tmp11, tmp12, tmp13;
    int32_t z1, z2, z3, z4, z5;

    tmp0 = d[0] + d[7];  tmp7 = d[0] - d[7];
    tmp1 = d[1] + d[6];  tmp6 = d[1] - d[6];
    tmp2 = d[2] + d[5];  tmp5 = d[2] - d[5];
    tmp3 = d[3] + d[4];  tmp4 = d[3] - d[4];

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2;

    d[0] = (tmp10 + tmp11) << PASS1_BITS;
    d[4] = (tmp10 - tmp11) << PASS1_BITS;

    z1 = (tmp12 + tmp13) * FIX_0_541196100;
    d[2] = DESCALE(z1 + tmp13 * FIX_0_765366865,  CONST_BITS - PASS1_BITS);
    d[6] = DESCALE(z1 - tmp12 * FIX_1_847759065,  CONST_BITS - PASS1_BITS);

    z1 = tmp4 + tmp7;
    z2 = tmp5 + tmp6;
    z3 = tmp4 + tmp6;
    z4 = tmp5 + tmp7;
    z5 = (z3 + z4) * FIX_1_175875602;

    tmp4 *= FIX_0_298631336;
    tmp5 *= FIX_2_053119869;
    tmp6 *= FIX_3_072711026;
    tmp7 *= FIX_1_501321110;

    z1 *= -FIX_0_899976223;
    z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;
    z4 *= -FIX_0_390180644;

    z3 += z5;
    z4 += z5;

    d[7] = DESCALE(tmp4 + z1 + z3, CONST_BITS - PASS1_BITS);
    d[5] = DESCALE(tmp5 + z2 + z4, CONST_BITS - PASS1_BITS);
    d[3] = DESCALE(tmp6 + z2 + z3, CONST_BITS - PASS1_BITS);
    d[1] = DESCALE(tmp7 + z1 + z4, CONST_BITS - PASS1_BITS);
}

/* -------------------------------------------------------------------------- */
/* Forward DCT — column pass (operates on one column, stride=8)               */
/* -------------------------------------------------------------------------- */

static void fdct_col(int32_t *d)
{
    int32_t tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
    int32_t tmp10, tmp11, tmp12, tmp13;
    int32_t z1, z2, z3, z4, z5;

    tmp0 = d[0*8] + d[7*8];  tmp7 = d[0*8] - d[7*8];
    tmp1 = d[1*8] + d[6*8];  tmp6 = d[1*8] - d[6*8];
    tmp2 = d[2*8] + d[5*8];  tmp5 = d[2*8] - d[5*8];
    tmp3 = d[3*8] + d[4*8];  tmp4 = d[3*8] - d[4*8];

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2;

    d[0*8] = DESCALE(tmp10 + tmp11, PASS1_BITS);
    d[4*8] = DESCALE(tmp10 - tmp11, PASS1_BITS);

    z1 = (tmp12 + tmp13) * FIX_0_541196100;
    d[2*8] = DESCALE(z1 + tmp13 * FIX_0_765366865,  CONST_BITS + PASS1_BITS);
    d[6*8] = DESCALE(z1 - tmp12 * FIX_1_847759065,  CONST_BITS + PASS1_BITS);

    z1 = tmp4 + tmp7;
    z2 = tmp5 + tmp6;
    z3 = tmp4 + tmp6;
    z4 = tmp5 + tmp7;
    z5 = (z3 + z4) * FIX_1_175875602;

    tmp4 *= FIX_0_298631336;
    tmp5 *= FIX_2_053119869;
    tmp6 *= FIX_3_072711026;
    tmp7 *= FIX_1_501321110;

    z1 *= -FIX_0_899976223;
    z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;
    z4 *= -FIX_0_390180644;

    z3 += z5;
    z4 += z5;

    d[7*8] = DESCALE(tmp4 + z1 + z3, CONST_BITS + PASS1_BITS);
    d[5*8] = DESCALE(tmp5 + z2 + z4, CONST_BITS + PASS1_BITS);
    d[3*8] = DESCALE(tmp6 + z2 + z3, CONST_BITS + PASS1_BITS);
    d[1*8] = DESCALE(tmp7 + z1 + z4, CONST_BITS + PASS1_BITS);
}

/* -------------------------------------------------------------------------- */
/* Public forward DCT                                                           */
/* -------------------------------------------------------------------------- */

void dct8_forward(const int16_t in[8][8], int32_t out[8][8])
{
    int32_t tmp[8][8];

    /* Load into working buffer (level-shift by -128 for proper DC centering) */
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            tmp[r][c] = in[r][c];

    /* Row pass */
    for (int r = 0; r < 8; r++)
        fdct_row(tmp[r]);

    /* Column pass */
    for (int c = 0; c < 8; c++)
        fdct_col(&tmp[0][c]);

    /* Copy to output */
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            out[r][c] = tmp[r][c];
}

/* -------------------------------------------------------------------------- */
/* Inverse DCT — column pass                                                   */
/* -------------------------------------------------------------------------- */

#define IDCT_CONST_BITS  13
#define IDCT_PASS1_BITS  1

#define IDESCALE(x, n)  (((x) + (1 << ((n)-1))) >> (n))

static void idct_col(int32_t *d)
{
    int32_t tmp0, tmp1, tmp2, tmp3;
    int32_t tmp10, tmp11, tmp12, tmp13;
    int32_t z1, z2, z3, z4, z5;

    z2 = d[2*8];
    z3 = d[6*8];
    z1 = (z2 + z3) * FIX_0_541196100;
    tmp2 = z1 + z3 * (-FIX_1_847759065);
    tmp3 = z1 + z2 * FIX_0_765366865;

    z2 = d[0*8];
    z3 = d[4*8];
    tmp0 = (z2 + z3) << IDCT_CONST_BITS;
    tmp1 = (z2 - z3) << IDCT_CONST_BITS;

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2;

    z1 = d[7*8] + d[1*8];
    z2 = d[5*8] + d[3*8];
    z3 = d[7*8] + d[3*8];
    z4 = d[5*8] + d[1*8];
    z5 = (z3 + z4) * FIX_1_175875602;

    z1 *= -FIX_0_899976223;
    z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;
    z4 *= -FIX_0_390180644;
    z3 += z5;  z4 += z5;

    tmp0  = d[7*8] * FIX_0_298631336;
    tmp1  = d[5*8] * FIX_2_053119869;
    tmp2  = d[3*8] * FIX_3_072711026;
    tmp3  = d[1*8] * FIX_1_501321110;

    tmp0 += z1 + z3;
    tmp1 += z2 + z4;
    tmp2 += z2 + z3;
    tmp3 += z1 + z4;

    d[0*8] = IDESCALE(tmp10 + tmp3, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[7*8] = IDESCALE(tmp10 - tmp3, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[1*8] = IDESCALE(tmp11 + tmp2, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[6*8] = IDESCALE(tmp11 - tmp2, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[2*8] = IDESCALE(tmp12 + tmp1, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[5*8] = IDESCALE(tmp12 - tmp1, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[3*8] = IDESCALE(tmp13 + tmp0, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
    d[4*8] = IDESCALE(tmp13 - tmp0, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3);
}

/* -------------------------------------------------------------------------- */
/* Inverse DCT — row pass                                                      */
/* -------------------------------------------------------------------------- */

static void idct_row(int32_t *d)
{
    int32_t tmp0, tmp1, tmp2, tmp3;
    int32_t tmp10, tmp11, tmp12, tmp13;
    int32_t z1, z2, z3, z4, z5;

    z2 = d[2];
    z3 = d[6];
    z1 = (z2 + z3) * FIX_0_541196100;
    tmp2 = z1 + z3 * (-FIX_1_847759065);
    tmp3 = z1 + z2 * FIX_0_765366865;

    tmp0 = (d[0] + d[4]) << IDCT_CONST_BITS;
    tmp1 = (d[0] - d[4]) << IDCT_CONST_BITS;

    tmp10 = tmp0 + tmp3;  tmp13 = tmp0 - tmp3;
    tmp11 = tmp1 + tmp2;  tmp12 = tmp1 - tmp2;

    z1 = d[7] + d[1];
    z2 = d[5] + d[3];
    z3 = d[7] + d[3];
    z4 = d[5] + d[1];
    z5 = (z3 + z4) * FIX_1_175875602;

    z1 *= -FIX_0_899976223;
    z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;
    z4 *= -FIX_0_390180644;
    z3 += z5;  z4 += z5;

    tmp0  = d[7] * FIX_0_298631336;
    tmp1  = d[5] * FIX_2_053119869;
    tmp2  = d[3] * FIX_3_072711026;
    tmp3  = d[1] * FIX_1_501321110;

    tmp0 += z1 + z3;
    tmp1 += z2 + z4;
    tmp2 += z2 + z3;
    tmp3 += z1 + z4;

    d[0] = IDESCALE(tmp10 + tmp3, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[7] = IDESCALE(tmp10 - tmp3, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[1] = IDESCALE(tmp11 + tmp2, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[6] = IDESCALE(tmp11 - tmp2, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[2] = IDESCALE(tmp12 + tmp1, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[5] = IDESCALE(tmp12 - tmp1, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[3] = IDESCALE(tmp13 + tmp0, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
    d[4] = IDESCALE(tmp13 - tmp0, IDCT_CONST_BITS - IDCT_PASS1_BITS + 3);
}

/* -------------------------------------------------------------------------- */
/* Public inverse DCT                                                           */
/* -------------------------------------------------------------------------- */

void dct8_inverse(const int32_t in[8][8], int16_t out[8][8])
{
    int32_t tmp[8][8];

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            tmp[r][c] = in[r][c];

    /* Column pass first for IDCT */
    for (int c = 0; c < 8; c++)
        idct_col(&tmp[0][c]);

    /* Row pass */
    for (int r = 0; r < 8; r++)
        idct_row(tmp[r]);

    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            out[r][c] = (int16_t)CLIP(tmp[r][c], -32768, 32767);
}
