/*
 * dct_hw.c — pipelined 8x8 integer DCT / IDCT
 *
 * VHDL translation notes
 * ----------------------
 * Each *_tick() function body becomes:
 *
 *   process(clk)
 *   begin
 *     if rising_edge(clk) then
 *       -- all the combinatorial logic below, written as signal assignments
 *       -- e.g.: next_phase <= DCT_FWD_COL when (in_row_r = 7 and in_valid = '1') ...
 *       phase_r    <= next_phase;
 *       row_buf_r  <= next_row_buf;
 *       ...
 *     end if;
 *   end process;
 *
 * The butterfly arithmetic (fdct_row_hw / fdct_col_hw) becomes a separate
 * combinatorial process or an instantiated pipelined DSP chain.
 * Each multiply-add maps to one DSP48E2 primitive.
 */

#include "dct_hw.h"
#include <string.h>

/* ----------------------------------------------------------------------- */
/* Shared fixed-point constants (same as dct.c)                            */
/* ----------------------------------------------------------------------- */

#define CONST_BITS      13
#define PASS1_BITS       2
#define IDCT_CONST_BITS 13
#define IDCT_PASS1_BITS  1

#define DESCALE(x, n)   (((x) + (1 << ((n)-1))) >> (n))
#define IDESCALE(x, n)  (((x) + (1 << ((n)-1))) >> (n))

/* AAN cosine constants (scaled by 2^13) */
#define FIX_0_298631336   2446
#define FIX_0_390180644   3196
#define FIX_0_541196100   4433
#define FIX_0_765366865   6270
#define FIX_0_899976223   7373
#define FIX_1_175875602   9633
#define FIX_1_501321110  12299
#define FIX_1_847759065  15137
#define FIX_1_961570560  16069
#define FIX_2_053119869  16819
#define FIX_2_562915447  20995
#define FIX_3_072711026  25172

/* ----------------------------------------------------------------------- */
/* Forward DCT row butterfly                                                */
/*                                                                         */
/* VHDL: combinatorial process, 8 inputs → 8 outputs, ~5 DSP stages deep. */
/* All multiplies can share DSP48E2 blocks; adders use LUT carry-chains.   */
/* ----------------------------------------------------------------------- */
static void fdct_row_hw(int32_t *d)
{
    int32_t tmp0 = d[0]+d[7], tmp7 = d[0]-d[7];
    int32_t tmp1 = d[1]+d[6], tmp6 = d[1]-d[6];
    int32_t tmp2 = d[2]+d[5], tmp5 = d[2]-d[5];
    int32_t tmp3 = d[3]+d[4], tmp4 = d[3]-d[4];

    int32_t tmp10 = tmp0+tmp3, tmp13 = tmp0-tmp3;
    int32_t tmp11 = tmp1+tmp2, tmp12 = tmp1-tmp2;

    d[0] = (tmp10+tmp11) << PASS1_BITS;
    d[4] = (tmp10-tmp11) << PASS1_BITS;

    int32_t z1 = (tmp12+tmp13) * FIX_0_541196100;
    d[2] = DESCALE(z1 + tmp13 * FIX_0_765366865, CONST_BITS - PASS1_BITS);
    d[6] = DESCALE(z1 - tmp12 * FIX_1_847759065, CONST_BITS - PASS1_BITS);

    z1 = tmp4+tmp7;
    int32_t z2 = tmp5+tmp6, z3 = tmp4+tmp6, z4 = tmp5+tmp7;
    int32_t z5 = (z3+z4) * FIX_1_175875602;

    tmp4 *= FIX_0_298631336;  tmp5 *= FIX_2_053119869;
    tmp6 *= FIX_3_072711026;  tmp7 *= FIX_1_501321110;
    z1 *= -FIX_0_899976223;   z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;   z4 *= -FIX_0_390180644;
    z3 += z5;  z4 += z5;

    d[7] = DESCALE(tmp4+z1+z3, CONST_BITS-PASS1_BITS);
    d[5] = DESCALE(tmp5+z2+z4, CONST_BITS-PASS1_BITS);
    d[3] = DESCALE(tmp6+z2+z3, CONST_BITS-PASS1_BITS);
    d[1] = DESCALE(tmp7+z1+z4, CONST_BITS-PASS1_BITS);
}

/* ----------------------------------------------------------------------- */
/* Forward DCT column butterfly (stride=8 within row_buf)                  */
/*                                                                         */
/* VHDL: 8 of these run in parallel (one per column), each is an identical */
/* combinatorial block fed from a column slice of row_buf BRAM.            */
/* ----------------------------------------------------------------------- */
static void fdct_col_hw(int32_t *d)
{
    int32_t tmp0 = d[0*8]+d[7*8], tmp7 = d[0*8]-d[7*8];
    int32_t tmp1 = d[1*8]+d[6*8], tmp6 = d[1*8]-d[6*8];
    int32_t tmp2 = d[2*8]+d[5*8], tmp5 = d[2*8]-d[5*8];
    int32_t tmp3 = d[3*8]+d[4*8], tmp4 = d[3*8]-d[4*8];

    int32_t tmp10 = tmp0+tmp3, tmp13 = tmp0-tmp3;
    int32_t tmp11 = tmp1+tmp2, tmp12 = tmp1-tmp2;

    d[0*8] = DESCALE(tmp10+tmp11, PASS1_BITS);
    d[4*8] = DESCALE(tmp10-tmp11, PASS1_BITS);

    int32_t z1 = (tmp12+tmp13) * FIX_0_541196100;
    d[2*8] = DESCALE(z1 + tmp13*FIX_0_765366865, CONST_BITS+PASS1_BITS);
    d[6*8] = DESCALE(z1 - tmp12*FIX_1_847759065, CONST_BITS+PASS1_BITS);

    z1 = tmp4+tmp7;
    int32_t z2 = tmp5+tmp6, z3 = tmp4+tmp6, z4 = tmp5+tmp7;
    int32_t z5 = (z3+z4) * FIX_1_175875602;

    tmp4 *= FIX_0_298631336;  tmp5 *= FIX_2_053119869;
    tmp6 *= FIX_3_072711026;  tmp7 *= FIX_1_501321110;
    z1 *= -FIX_0_899976223;   z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;   z4 *= -FIX_0_390180644;
    z3 += z5;  z4 += z5;

    d[7*8] = DESCALE(tmp4+z1+z3, CONST_BITS+PASS1_BITS);
    d[5*8] = DESCALE(tmp5+z2+z4, CONST_BITS+PASS1_BITS);
    d[3*8] = DESCALE(tmp6+z2+z3, CONST_BITS+PASS1_BITS);
    d[1*8] = DESCALE(tmp7+z1+z4, CONST_BITS+PASS1_BITS);
}

/* ----------------------------------------------------------------------- */
/* Inverse DCT column butterfly                                             */
/* ----------------------------------------------------------------------- */
static void idct_col_hw(int32_t *d)
{
    int32_t z2 = d[2*8], z3 = d[6*8];
    int32_t z1 = (z2+z3) * FIX_0_541196100;
    int32_t tmp2 = z1 + z3 * (-FIX_1_847759065);
    int32_t tmp3 = z1 + z2 * FIX_0_765366865;

    z2 = d[0*8];  z3 = d[4*8];
    int32_t tmp0 = (z2+z3) << IDCT_CONST_BITS;
    int32_t tmp1 = (z2-z3) << IDCT_CONST_BITS;

    int32_t tmp10 = tmp0+tmp3, tmp13 = tmp0-tmp3;
    int32_t tmp11 = tmp1+tmp2, tmp12 = tmp1-tmp2;

    z1 = d[7*8]+d[1*8];  z2 = d[5*8]+d[3*8];
    z3 = d[7*8]+d[3*8];  int32_t z4 = d[5*8]+d[1*8];
    int32_t z5 = (z3+z4) * FIX_1_175875602;
    z1 *= -FIX_0_899976223;  z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;  z4 *= -FIX_0_390180644;
    z3 += z5;  z4 += z5;

    tmp0 = d[7*8]*FIX_0_298631336 + z1 + z3;
    tmp1 = d[5*8]*FIX_2_053119869 + z2 + z4;
    tmp2 = d[3*8]*FIX_3_072711026 + z2 + z3;
    tmp3 = d[1*8]*FIX_1_501321110 + z1 + z4;

    d[0*8] = IDESCALE(tmp10+tmp3, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[7*8] = IDESCALE(tmp10-tmp3, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[1*8] = IDESCALE(tmp11+tmp2, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[6*8] = IDESCALE(tmp11-tmp2, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[2*8] = IDESCALE(tmp12+tmp1, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[5*8] = IDESCALE(tmp12-tmp1, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[3*8] = IDESCALE(tmp13+tmp0, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
    d[4*8] = IDESCALE(tmp13-tmp0, IDCT_CONST_BITS+IDCT_PASS1_BITS+3);
}

/* ----------------------------------------------------------------------- */
/* Inverse DCT row butterfly                                                */
/* ----------------------------------------------------------------------- */
static void idct_row_hw(int32_t *d)
{
    int32_t z2 = d[2], z3 = d[6];
    int32_t z1 = (z2+z3) * FIX_0_541196100;
    int32_t tmp2 = z1 + z3 * (-FIX_1_847759065);
    int32_t tmp3 = z1 + z2 * FIX_0_765366865;

    int32_t tmp0 = (d[0]+d[4]) << IDCT_CONST_BITS;
    int32_t tmp1 = (d[0]-d[4]) << IDCT_CONST_BITS;

    int32_t tmp10 = tmp0+tmp3, tmp13 = tmp0-tmp3;
    int32_t tmp11 = tmp1+tmp2, tmp12 = tmp1-tmp2;

    z1 = d[7]+d[1];  z2 = d[5]+d[3];
    z3 = d[7]+d[3];  int32_t z4 = d[5]+d[1];
    int32_t z5 = (z3+z4) * FIX_1_175875602;
    z1 *= -FIX_0_899976223;  z2 *= -FIX_2_562915447;
    z3 *= -FIX_1_961570560;  z4 *= -FIX_0_390180644;
    z3 += z5;  z4 += z5;

    tmp0 = d[7]*FIX_0_298631336 + z1 + z3;
    tmp1 = d[5]*FIX_2_053119869 + z2 + z4;
    tmp2 = d[3]*FIX_3_072711026 + z2 + z3;
    tmp3 = d[1]*FIX_1_501321110 + z1 + z4;

    d[0] = IDESCALE(tmp10+tmp3, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[7] = IDESCALE(tmp10-tmp3, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[1] = IDESCALE(tmp11+tmp2, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[6] = IDESCALE(tmp11-tmp2, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[2] = IDESCALE(tmp12+tmp1, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[5] = IDESCALE(tmp12-tmp1, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[3] = IDESCALE(tmp13+tmp0, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
    d[4] = IDESCALE(tmp13-tmp0, IDCT_CONST_BITS-IDCT_PASS1_BITS+3);
}

/* ======================================================================= */
/* Forward DCT pipeline                                                     */
/* ======================================================================= */

void dct_fwd_hw_init(DctFwdHwState *s)
{
    memset(s, 0, sizeof(*s));
    s->phase          = DCT_FWD_FILL;
    s->in_ctrl.ready  = 1;
    s->out_ctrl.valid = 0;
}

/*
 * dct_fwd_hw_tick — one clock cycle of the forward DCT pipeline.
 *
 * VHDL process outline:
 *   process(clk)
 *   begin
 *     if rising_edge(clk) then
 *       case phase_r is
 *         when DCT_FWD_FILL =>
 *           if in_valid = '1' then
 *             row_buf_r(in_row_r) <= fdct_row(in_row);   -- row butterfly
 *             in_row_r <= in_row_r + 1;
 *             if in_row_r = 7 then phase_r <= DCT_FWD_COL; end if;
 *           end if;
 *         when DCT_FWD_COL =>
 *           for c in 0 to 7 loop                          -- 8 parallel col butterflies
 *             out_buf_r(*, c) <= fdct_col(row_buf_r(*, c));
 *           end loop;
 *           out_row_r <= 0;
 *           phase_r <= DCT_FWD_EMIT;
 *         when DCT_FWD_EMIT =>
 *           cur_out_r <= out_buf_r(out_row_r);
 *           out_valid_r <= '1';
 *           out_last_r  <= '1' when out_row_r = 7 else '0';
 *           out_row_r <= out_row_r + 1;
 *           if out_row_r = 7 then phase_r <= DCT_FWD_FILL; end if;
 *       end case;
 *     end if;
 *   end process;
 */
int dct_fwd_hw_tick(DctFwdHwState *s, const int16_t in_row[8], uint8_t in_valid)
{
    /* Default: no output this cycle */
    s->out_ctrl.valid = 0;
    s->out_ctrl.last  = 0;

    switch (s->phase) {

    case DCT_FWD_FILL:
        /* In_ready is always 1 while filling */
        s->in_ctrl.ready = 1;
        if (in_valid) {
            /* Row butterfly: combinatorial on FPGA (one DSP pipeline chain) */
            int32_t tmp[8];
            for (int c = 0; c < 8; c++) tmp[c] = (int32_t)in_row[c];
            fdct_row_hw(tmp);
            for (int c = 0; c < 8; c++) s->row_buf[s->in_row][c] = tmp[c];

            s->in_row++;
            if (s->in_row == 8) {
                s->in_row        = 0;
                s->in_ctrl.ready = 0;
                s->phase         = DCT_FWD_COL;
            }
        }
        break;

    case DCT_FWD_COL:
        /*
         * Column butterfly pass.
         * FPGA: 8 independent col-butterfly units run in parallel,
         *       each fed from a column slice of row_buf (BRAM read).
         *       This whole pass takes 1 clock cycle with parallel DSPs.
         * C:    serial loop over 8 columns (same arithmetic, different schedule).
         */
        for (int c = 0; c < 8; c++) fdct_col_hw(&s->row_buf[0][c]);
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
                s->out_buf[r][c] = s->row_buf[r][c];
        s->out_row = 0;
        s->phase   = DCT_FWD_EMIT;
        break;

    case DCT_FWD_EMIT:
        /* Drive one output row onto cur_out (registered output bus) */
        for (int c = 0; c < 8; c++) s->cur_out[c] = s->out_buf[s->out_row][c];
        s->out_ctrl.valid = 1;
        s->out_ctrl.last  = (s->out_row == 7) ? 1 : 0;

        s->out_row++;
        if (s->out_row == 8) {
            s->out_row       = 0;
            s->phase         = DCT_FWD_FILL;
            s->in_ctrl.ready = 1;
        }
        break;
    }

    return (int)s->out_ctrl.valid;
}

/* ======================================================================= */
/* Inverse DCT pipeline                                                     */
/* ======================================================================= */

void dct_inv_hw_init(DctInvHwState *s)
{
    memset(s, 0, sizeof(*s));
    s->phase          = DCT_INV_FILL;
    s->in_ctrl.ready  = 1;
    s->out_ctrl.valid = 0;
}

int dct_inv_hw_tick(DctInvHwState *s, const int32_t in_row[8], uint8_t in_valid)
{
    s->out_ctrl.valid = 0;
    s->out_ctrl.last  = 0;

    switch (s->phase) {

    case DCT_INV_FILL:
        s->in_ctrl.ready = 1;
        if (in_valid) {
            /* Store row (col pass runs after all 8 rows arrive) */
            for (int c = 0; c < 8; c++) s->row_buf[s->in_row][c] = in_row[c];
            s->in_row++;
            if (s->in_row == 8) {
                s->in_row        = 0;
                s->in_ctrl.ready = 0;
                s->phase         = DCT_INV_COL;
            }
        }
        break;

    case DCT_INV_COL:
        /* Column pass first for IDCT (same parallelism argument as forward) */
        for (int c = 0; c < 8; c++) idct_col_hw(&s->row_buf[0][c]);
        /* Row pass completes the transform */
        for (int r = 0; r < 8; r++) idct_row_hw(s->row_buf[r]);
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
                s->out_buf[r][c] = s->row_buf[r][c];
        s->out_row = 0;
        s->phase   = DCT_INV_EMIT;
        break;

    case DCT_INV_EMIT:
        for (int c = 0; c < 8; c++)
            s->cur_out[c] = (int16_t)CLIP(s->out_buf[s->out_row][c], -32768, 32767);
        s->out_ctrl.valid = 1;
        s->out_ctrl.last  = (s->out_row == 7) ? 1 : 0;

        s->out_row++;
        if (s->out_row == 8) {
            s->out_row       = 0;
            s->phase         = DCT_INV_FILL;
            s->in_ctrl.ready = 1;
        }
        break;
    }

    return (int)s->out_ctrl.valid;
}
