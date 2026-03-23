/*
 * dct_hw.h — pipelined 8x8 integer DCT / IDCT
 *
 * Models the transform as a 3-phase register pipeline so each
 * call to *_tick() translates to one FPGA clock cycle.
 *
 * Forward DCT phases
 * ------------------
 *   FILL (8 ticks) : accept one residual row per tick, run row butterflies
 *   COL  (1 tick)  : run 8 column butterfly chains in parallel (8 DSP chains)
 *   EMIT (8 ticks) : output one coefficient row per tick
 *
 *   Total latency  : 17 ticks
 *   Throughput     : 1 block per 64 ticks (back-to-back)
 *
 * FPGA resource estimate (one forward DCT block)
 * -----------------------------------------------
 *   DSP48E2   : ~40   (AAN butterfly stages, row + column pass)
 *   Registers : ~512  (two 8×8×32-bit register arrays)
 *   BRAM      :   0   (arrays fit in distributed RAM / FFs at this size)
 *   Latency   :  17 clock cycles
 *
 * Inverse DCT has identical structure and the same estimates.
 */

#ifndef DRONE_CODEC_DCT_HW_H
#define DRONE_CODEC_DCT_HW_H

#include "common_hw.h"

/* -----------------------------------------------------------------------
 * Forward DCT pipeline state
 *
 * VHDL: declare these as signals in the architecture body.
 *   signal phase    : std_logic_vector(1 downto 0);  -- FSM state
 *   signal in_row   : integer range 0 to 7;          -- row counter
 *   signal out_row  : integer range 0 to 7;          -- emit counter
 *   signal row_buf  : row_buf_t;   -- 8x8 int32 array (after row pass)
 *   signal out_buf  : row_buf_t;   -- 8x8 int32 array (after col pass)
 *   signal cur_out  : coeff_row_t; -- currently driven output bus
 * ----------------------------------------------------------------------- */
typedef enum { DCT_FWD_FILL = 0, DCT_FWD_COL = 1, DCT_FWD_EMIT = 2 } DctFwdPhase;

typedef struct {
    DctFwdPhase phase;
    uint8_t     in_row;          /* which row we're about to accept (0..7)  */
    uint8_t     out_row;         /* which row is on the output bus (0..7)   */
    int32_t     row_buf[8][8];   /* intermediate buffer: after row pass      */
    int32_t     out_buf[8][8];   /* output buffer: after col pass            */
    int32_t     cur_out[8];      /* registered output bus (valid when EMIT)  */
    PipeCtrl    in_ctrl;         /* upstream handshake (in_ctrl.ready = 1 in FILL) */
    PipeCtrl    out_ctrl;        /* downstream handshake (out_ctrl.valid = 1 in EMIT) */
} DctFwdHwState;

/* -----------------------------------------------------------------------
 * Inverse DCT pipeline state (mirror of forward)
 * ----------------------------------------------------------------------- */
typedef enum { DCT_INV_FILL = 0, DCT_INV_COL = 1, DCT_INV_EMIT = 2 } DctInvPhase;

typedef struct {
    DctInvPhase phase;
    uint8_t     in_row;
    uint8_t     out_row;
    int32_t     row_buf[8][8];   /* after column pass                        */
    int32_t     out_buf[8][8];   /* after row pass                           */
    int16_t     cur_out[8];      /* registered output bus (clamped to int16) */
    PipeCtrl    in_ctrl;
    PipeCtrl    out_ctrl;
} DctInvHwState;

/* -----------------------------------------------------------------------
 * API — one call = one clock tick
 *
 * dct_fwd_hw_tick
 *   in_row[8]  : 8 residual samples for the current row (int16 → signed 9-bit range)
 *   in_valid   : 1 if in_row carries valid data this tick
 *   Returns 1 when an output row is ready on state->cur_out[]
 *
 * dct_inv_hw_tick
 *   in_row[8]  : 8 dequantised coefficients for the current row
 *   in_valid   : 1 if in_row carries valid data this tick
 *   Returns 1 when an output row is ready on state->cur_out[]
 * ----------------------------------------------------------------------- */
void dct_fwd_hw_init(DctFwdHwState *s);
int  dct_fwd_hw_tick(DctFwdHwState *s, const int16_t in_row[8], uint8_t in_valid);

void dct_inv_hw_init(DctInvHwState *s);
int  dct_inv_hw_tick(DctInvHwState *s, const int32_t in_row[8], uint8_t in_valid);

#endif /* DRONE_CODEC_DCT_HW_H */
