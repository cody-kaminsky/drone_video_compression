/*
 * quant_hw.h — hardware-pipeline quantisation / dequantisation
 *
 * Quantisation is combinatorial: one coefficient in → one coefficient out
 * per clock cycle.  No buffering is needed.
 *
 * FPGA resource estimate (one forward quantiser)
 * -----------------------------------------------
 *   DSP48E2   : 1  (multiply by reciprocal + shift)
 *   LUT       : ~20 (dead-zone compare + sign extraction)
 *   BRAM      :  0
 *   Latency   :  1 clock cycle (registered output)
 *
 * Dequantisation is even simpler: 1 multiply, 1 DSP48E2, 1 cycle.
 *
 * The QP step and reciprocal are precomputed in QuantHwState so the
 * per-coefficient path is just: multiply → shift → clip.
 * On FPGA these would be loaded from a 51-entry ROM (one entry per QP).
 */

#ifndef DRONE_CODEC_QUANT_HW_H
#define DRONE_CODEC_QUANT_HW_H

#include "common_hw.h"

/* -----------------------------------------------------------------------
 * Quantiser state (pipeline registers)
 *
 * VHDL: these become registered signals updated at QP-change time,
 *       not every clock cycle — they act like configuration registers.
 *
 *   signal qp_r    : integer range 1 to 51;
 *   signal step_r  : integer;   -- quant_step(qp)
 *   signal recip_r : integer;   -- (2^16) / step
 *   signal dz_r    : integer;   -- dead-zone threshold
 * ----------------------------------------------------------------------- */
typedef struct {
    int      qp;       /* quantisation parameter (1..51)                    */
    int      step;     /* quant step size = quant_step(qp)                  */
    int32_t  recip;    /* (1<<16) / step  — for division-free quantisation  */
    int      dz_intra; /* dead-zone for intra (step * 3/8)                  */
    int      dz_inter; /* dead-zone for inter (step / 2)                    */
    /* Current coefficient output (valid when out_ctrl.valid = 1) */
    int16_t  cur_out;
    PipeCtrl out_ctrl;
} QuantHwState;

/*
 * Initialise the quantiser state for a given QP.
 * Call once per frame (or whenever QP changes).
 * VHDL: this is a ROM lookup that happens at QP-load time.
 */
void quant_hw_init(QuantHwState *s, int qp);

/*
 * quant_hw_tick — one clock cycle: quantise one DCT coefficient.
 *
 *   coeff     : dequantised DCT coefficient from the forward DCT pipeline
 *   is_intra  : 1 for I-frame (larger dead-zone), 0 for P-frame
 *   in_valid  : 1 when coeff carries meaningful data
 *
 * Output is in state->cur_out when the function returns 1.
 *
 * VHDL:
 *   process(clk)
 *   begin
 *     if rising_edge(clk) then
 *       if in_valid = '1' then
 *         abs_v     := abs(coeff);
 *         sign_v    := '1' when coeff >= 0 else '0';
 *         dz_sel    := dz_intra_r when is_intra = '1' else dz_inter_r;
 *         q         := to_integer(abs_v * recip_r) srl 16;
 *         cur_out_r <= (sign * q) when abs_v >= dz_sel else 0;
 *         out_valid <= in_valid;
 *       end if;
 *     end if;
 *   end process;
 */
int quant_hw_tick(QuantHwState *s, int32_t coeff, uint8_t is_intra, uint8_t in_valid);

/* -----------------------------------------------------------------------
 * Dequantiser state — even simpler: one multiply per coefficient
 * ----------------------------------------------------------------------- */
typedef struct {
    int      step;     /* quant_step(qp)                                    */
    int32_t  cur_out;
    PipeCtrl out_ctrl;
} DequantHwState;

void dequant_hw_init(DequantHwState *s, int qp);

/*
 * dequant_hw_tick — one clock cycle: dequantise one quantised coefficient.
 * Output in state->cur_out when returns 1.
 */
int dequant_hw_tick(DequantHwState *s, int16_t qcoeff, uint8_t in_valid);

#endif /* DRONE_CODEC_QUANT_HW_H */
