/*
 * common_hw.h — hardware-pipeline types and VHDL translation conventions
 *
 * VHDL mapping cheat-sheet
 * ========================
 *   C construct                    | VHDL equivalent
 *   -------------------------------|------------------------------------------
 *   struct FooHwState              | architecture signal declarations (registers)
 *   *_tick(state, in, out) call    | process(clk) rising_edge register update
 *   uint8_t valid / ready          | std_logic port
 *   uint8_t phase (enum)           | std_logic_vector FSM state register
 *   int32_t buf[N]                 | signal array or BRAM (if N > ~32 words)
 *   MUX2(sel, a, b)                | a when sel='1' else b
 *   for (i=0; i<N; i++) ...        | generate / for-loop inside a process
 *
 * Pipeline protocol (AXI-Stream style)
 * =====================================
 *   upstream  asserts in_valid  when it has data ready
 *   this stage asserts in_ready when it can accept data
 *   transfer occurs only when BOTH in_valid AND in_ready are '1'
 *   this stage asserts out_valid when its output is meaningful
 *   downstream asserts out_ready when it can consume
 *
 * Clock-cycle model
 * =================
 *   Every call to *_tick() represents one rising clock edge.
 *   All reads of *HwState fields happen BEFORE the edge (combinatorial).
 *   All writes to *HwState fields happen AT the edge (register update).
 *   NEVER read a field that was written in the same tick — that would be
 *   combinatorial feedback (a latch/loop in VHDL).
 */

#ifndef DRONE_CODEC_COMMON_HW_H
#define DRONE_CODEC_COMMON_HW_H

#include <stdint.h>
#include "../common.h"

/* -----------------------------------------------------------------------
 * Branch-free 2-to-1 mux
 * Maps to VHDL: out <= a when sel='1' else b;
 * ----------------------------------------------------------------------- */
#define MUX2(sel, a, b)   ((sel) ? (a) : (b))

/* -----------------------------------------------------------------------
 * Pipeline handshake signals
 *
 * In VHDL these become individual std_logic ports:
 *   valid_i / ready_o  on the input side  (upstream  → this module)
 *   valid_o / ready_i  on the output side (this module → downstream)
 *   last_o             end-of-block / end-of-frame marker
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t valid;  /* 1 = data present on this cycle                       */
    uint8_t ready;  /* 1 = this stage can accept new data right now         */
    uint8_t last;   /* 1 = final item in the current block / frame / line   */
} PipeCtrl;

#endif /* DRONE_CODEC_COMMON_HW_H */
