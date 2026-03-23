/*
 * quant_hw.c — hardware-pipeline quantisation / dequantisation
 *
 * One coefficient per clock cycle, 1-cycle registered latency.
 * No state machine needed — purely combinatorial with output register.
 */

#include "quant_hw.h"
#include <string.h>

/* QP step base table (same as quant.c) */
static const int QP_STEP_BASE[6] = { 10, 11, 13, 14, 16, 18 };

static int qp_step(int qp)
{
    if (qp < 1)  qp = 1;
    if (qp > 51) qp = 51;
    return QP_STEP_BASE[(qp-1) % 6] << ((qp-1) / 6);
}

/* ======================================================================= */
/* Quantiser                                                                */
/* ======================================================================= */

void quant_hw_init(QuantHwState *s, int qp)
{
    memset(s, 0, sizeof(*s));
    s->qp       = qp;
    s->step     = qp_step(qp);
    s->recip    = (1 << 16) / s->step;   /* ROM entry for this QP           */
    s->dz_intra = (s->step * 3) / 8;
    s->dz_inter = s->step / 2;
}

/*
 * quant_hw_tick — one rising clock edge.
 *
 * Arithmetic mapped to FPGA primitives:
 *   abs(coeff)          → unsigned magnitude (LUT/subtractor)
 *   abs_v * recip_r     → DSP48E2 multiply (18×18 → 36-bit product)
 *   product >> 16       → logical right-shift (free in VHDL)
 *   abs_v >= dz         → comparator (LUT carry-chain)
 *   MUX2(cond, q, 0)    → VHDL conditional signal assignment
 *   sign * q            → negate if sign bit set (subtractor, 1 LUT row)
 */
int quant_hw_tick(QuantHwState *s, int32_t coeff, uint8_t is_intra, uint8_t in_valid)
{
    s->out_ctrl.valid = in_valid;
    if (!in_valid) { s->cur_out = 0; return 0; }

    int32_t sign = (coeff >= 0) ? 1 : -1;
    int32_t av   = (coeff >= 0) ? coeff : -coeff;
    int     dz   = MUX2(is_intra, s->dz_intra, s->dz_inter);
    int32_t q    = (int32_t)(((int64_t)av * s->recip) >> 16);

    /* Dead-zone: zero if below threshold (branch-free mux) */
    s->cur_out = (int16_t)MUX2(av >= dz, sign * q, 0);
    return 1;
}

/* ======================================================================= */
/* Dequantiser                                                              */
/* ======================================================================= */

void dequant_hw_init(DequantHwState *s, int qp)
{
    memset(s, 0, sizeof(*s));
    s->step = qp_step(qp);
}

/*
 * dequant_hw_tick — one clock cycle.
 *
 * VHDL: cur_out_r <= qcoeff * step_r;  — single DSP48E2 multiply.
 */
int dequant_hw_tick(DequantHwState *s, int16_t qcoeff, uint8_t in_valid)
{
    s->out_ctrl.valid = in_valid;
    if (!in_valid) { s->cur_out = 0; return 0; }
    s->cur_out = (int32_t)qcoeff * s->step;
    return 1;
}
