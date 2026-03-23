-- =============================================================================
-- dct8_inv.vhd  --  Pipelined 8x8 inverse integer DCT
--
-- Translates decode_hw.py (_idct_col / _idct_row) to synthesisable VHDL.
-- Uses the same IJG fixed-point constants as dct8_fwd (IDCT_CONST_BITS=13,
-- IDCT_PASS1_BITS=1).  Column pass runs first; row pass runs per-row during
-- EMIT, producing one residual row per clock.
--
-- Interface
-- ---------
--   Input  : 8 rows of dequantised coefficients, one row per clock.
--            Each row is 8 x signed-32-bit packed as a 256-bit bus.
--            (Same bus format as dct8_fwd output so it can be fed directly
--             from a dequantiser that widens the 16-bit quant values.)
--   Output : one residual row per clock (8 x signed-16-bit = 128-bit bus).
--            Values fit in signed 9-bit; 16-bit output keeps alignment simple.
--
-- Pipeline stages per block
-- -------------------------
--   LOAD     : 8 clocks  (accept 8 rows into buf[])
--   COL_ADD  : 1 clock   (column butterfly additions, all 8 cols in parallel)
--   COL_MUL  : 1 clock   (column multiplications)
--   COL_FINAL: 1 clock   (column descale, sh=17, store in col_buf[])
--   ROW_EMIT : 8 x 3 = 24 clocks (row butterfly + emit, one row per 3 clocks)
--   Total    : 35 clocks per 8x8 block
--
-- Resource estimate (UltraScale+)
-- --------------------------------
--   DSP58E2 : ~40  (same multiply count as forward DCT, shared constants)
--   FF      : ~700
--   BRAM    :   0
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity dct8_inv is
  port (
    aclk     : in  std_logic;
    aresetn  : in  std_logic;

    -- Input: one row of 8 dequantised coefficients (8 x 32-bit = 256 bits)
    s_tdata  : in  std_logic_vector(255 downto 0);
    s_tvalid : in  std_logic;
    s_tready : out std_logic;

    -- Output: one residual row (8 x signed-16-bit = 128 bits)
    m_tdata  : out std_logic_vector(127 downto 0);
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;   -- '1' on row 7 of each block
    m_tready : in  std_logic
  );
end entity dct8_inv;

architecture rtl of dct8_inv is

  -- Descale with round-to-nearest (matches _idescale in decode_hw.py)
  function idescale(x : s32_t; n : integer) return s32_t is
    variable bias : s32_t;
  begin
    bias := to_signed(1, 32) sll (n - 1);
    return shift_right(x + bias, n);
  end function;

  function mul32(a : s32_t; b : integer) return s32_t is
    variable av : signed(24 downto 0);
    variable bv : signed(17 downto 0);
    variable p  : signed(42 downto 0);
  begin
    av := resize(a, 25);
    bv := to_signed(b, 18);
    p  := av * bv;
    return resize(p, 32);
  end function;

  -- Unpack 256-bit row into int32_row_t
  function unpack256(v : std_logic_vector(255 downto 0)) return int32_row_t is
    variable r : int32_row_t;
  begin
    for i in 0 to 7 loop
      r(i) := signed(v(i*32+31 downto i*32));
    end loop;
    return r;
  end function;

  -- IDCT butterfly — same structure for both column and row passes.
  -- sh: column pass = IDCT_CONST_BITS + IDCT_PASS1_BITS + 3 = 17
  --     row   pass = IDCT_CONST_BITS - IDCT_PASS1_BITS + 3 = 15
  -- For the column pass, inputs are pre-shifted by IDCT_CONST_BITS (done inline).
  -- This function receives the pre-shifted even-part sums and raw odd part.
  -- Returns descaled outputs [0..7].
  --
  -- All 11 multiplications are explicit so Vivado maps them to DSPs.

  -- -------------------------------------------------------------------------
  -- FSM
  -- -------------------------------------------------------------------------
  type state_t is (LOAD, COL_ADD, COL_MUL, COL_FINAL,
                   ROW_ADD, ROW_MUL, ROW_FINAL);
  signal state    : state_t := LOAD;
  signal load_row : integer range 0 to 7 := 0;
  signal emit_row : integer range 0 to 7 := 0;

  -- Input coefficient buffer (8x8 x 32-bit)
  signal in_buf   : block32_t;

  -- Column pass intermediate registers
  -- Even part
  signal c_tmp10, c_tmp11, c_tmp12, c_tmp13 : int32_row_t;
  -- Odd part: pre-sums for DSP
  signal c_z34    : int32_row_t;  -- z3+z4 sum
  signal c_z_add  : block32_t;    -- [0]=d7+d1, [1]=d5+d3, [2]=d7+d3, [3]=d5+d1
  signal c_d4711  : block32_t;    -- [0]=d7, [1]=d5, [2]=d3, [3]=d1 (for individual muls)
  -- DSP product registers
  signal c_dsp1   : block32_t;    -- [0]=z5, [1]=z1m, [2]=z2m, [3]=z3m, [4]=z4m, [5]=z1e(unused), [6]=even0, [7]=even4
  signal c_dsp2   : block32_t;    -- [0]=r0a, [1]=r1a, [2]=r2a, [3]=r3a

  -- Column pass output buffer
  signal col_buf  : block32_t;

  -- Row pass intermediate registers (for one row at a time)
  signal r_row    : int32_row_t;   -- current row from col_buf
  -- ROW_ADD: pure additions (no multiplies) → feeds ROW_MUL registers
  signal r_d04    : s32_t;         -- d(0) + d(4)
  signal r_d04d   : s32_t;         -- d(0) - d(4)
  signal r_d26    : s32_t;         -- d(2) + d(6)
  signal r_d6     : s32_t;         -- d(6) (for even FIX_1_847 multiply)
  signal r_d2     : s32_t;         -- d(2) (for even FIX_0_765 multiply)
  signal r_z34    : s32_t;
  signal r_z_add  : int32_row_t;   -- [0]=d7+d1, [1]=d5+d3, [2]=d7+d3, [3]=d5+d1
  signal r_d4711  : int32_row_t;   -- [0]=d7, [1]=d5, [2]=d3, [3]=d1
  -- ROW_MUL: all DSP products (even + odd) → feeds ROW_FINAL
  signal r_dsp1   : int32_row_t;   -- [0]=z5, [1]=z1m, [2]=z2m, [3]=z3m, [4]=z4m
  signal r_dsp2   : int32_row_t;   -- [0]=r0a, [1]=r1a, [2]=r2a, [3]=r3a
  -- Even part DSP products (computed in ROW_MUL, consumed in ROW_FINAL)
  signal r_e_base : s32_t;         -- (d0+d4) << CONST_BITS  (shift only, no DSP)
  signal r_e_diff : s32_t;         -- (d0-d4) << CONST_BITS
  signal r_e_mul0 : s32_t;         -- (d2+d6)*FIX_0_541  [shared z1_even base]
  signal r_e_mul1 : s32_t;         -- d6*(-FIX_1_847)
  signal r_e_mul2 : s32_t;         -- d2*FIX_0_765

  signal out_row  : int32_row_t;

begin

  s_tready <= '1' when state = LOAD else '0';

  process(aclk)
    variable d   : int32_row_t;
    variable tmp0, tmp1, tmp2, tmp3 : s32_t;
    variable z1, z2, z3, z4, z5    : s32_t;
    variable o   : int32_row_t;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state    <= LOAD;
        load_row <= 0;
        emit_row <= 0;
        m_tvalid <= '0';
        m_tlast  <= '0';
      else
        m_tvalid <= '0';
        m_tlast  <= '0';

        case state is

          -- ------------------------------------------------------------------
          -- LOAD: accept 8 rows of dequantised coefficients
          -- ------------------------------------------------------------------
          when LOAD =>
            if s_tvalid = '1' then
              in_buf(load_row) <= unpack256(s_tdata);
              if load_row = 7 then
                load_row <= 0;
                state    <= COL_ADD;
              else
                load_row <= load_row + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- COL_ADD: compute even/odd butterfly additions for all 8 columns
          -- Column pass shift = IDCT_CONST_BITS = 13 (applied to even DC/AC)
          -- ------------------------------------------------------------------
          when COL_ADD =>
            for c in 0 to 7 loop
              -- Even part: shift in[0..4] by IDCT_CONST_BITS
              tmp0 := shift_left(in_buf(0)(c) + in_buf(4)(c), IDCT_CONST_BITS);
              tmp1 := shift_left(in_buf(0)(c) - in_buf(4)(c), IDCT_CONST_BITS);
              tmp2 := mul32(in_buf(2)(c) + in_buf(6)(c), FIX_0_541196100);
              tmp3 := tmp2 + mul32(in_buf(6)(c), -FIX_1_847759065);
              tmp2 := tmp2 + mul32(in_buf(2)(c),  FIX_0_765366865);
              -- tmp10..13
              c_tmp10(c) <= tmp0 + tmp2;
              c_tmp13(c) <= tmp0 - tmp2;
              c_tmp11(c) <= tmp1 + tmp3;
              c_tmp12(c) <= tmp1 - tmp3;
              -- Odd part: precompute sums for DSP stage
              c_z_add(0)(c) <= in_buf(7)(c) + in_buf(1)(c);  -- z1 base
              c_z_add(1)(c) <= in_buf(5)(c) + in_buf(3)(c);  -- z2 base
              c_z_add(2)(c) <= in_buf(7)(c) + in_buf(3)(c);  -- z3 base
              c_z_add(3)(c) <= in_buf(5)(c) + in_buf(1)(c);  -- z4 base
              c_z34(c)      <= (in_buf(7)(c) + in_buf(3)(c))
                             + (in_buf(5)(c) + in_buf(1)(c));  -- z3+z4 for z5
              c_d4711(0)(c) <= in_buf(7)(c);
              c_d4711(1)(c) <= in_buf(5)(c);
              c_d4711(2)(c) <= in_buf(3)(c);
              c_d4711(3)(c) <= in_buf(1)(c);
            end loop;
            state <= COL_MUL;

          -- ------------------------------------------------------------------
          -- COL_MUL: all DSP multiplications (inputs are registered pre-sums)
          -- ------------------------------------------------------------------
          when COL_MUL =>
            for c in 0 to 7 loop
              c_dsp1(0)(c) <= mul32(c_z34(c),      FIX_1_175875602);   -- z5
              c_dsp1(1)(c) <= mul32(c_z_add(0)(c), -FIX_0_899976223);  -- z1
              c_dsp1(2)(c) <= mul32(c_z_add(1)(c), -FIX_2_562915447);  -- z2
              c_dsp1(3)(c) <= mul32(c_z_add(2)(c), -FIX_1_961570560);  -- z3
              c_dsp1(4)(c) <= mul32(c_z_add(3)(c), -FIX_0_390180644);  -- z4
              c_dsp2(0)(c) <= mul32(c_d4711(0)(c),  FIX_0_298631336);  -- r0a
              c_dsp2(1)(c) <= mul32(c_d4711(1)(c),  FIX_2_053119869);  -- r1a
              c_dsp2(2)(c) <= mul32(c_d4711(2)(c),  FIX_3_072711026);  -- r2a
              c_dsp2(3)(c) <= mul32(c_d4711(3)(c),  FIX_1_501321110);  -- r3a
            end loop;
            state <= COL_FINAL;

          -- ------------------------------------------------------------------
          -- COL_FINAL: add z5, combine r0..r3, descale(sh=17), store col_buf
          -- Output order matches _idct_col:
          --   out[0] = tmp10+r3, [7]=tmp10-r3, [1]=tmp11+r2, [6]=tmp11-r2
          --   out[2] = tmp12+r1, [5]=tmp12-r1, [3]=tmp13+r0, [4]=tmp13-r0
          -- ------------------------------------------------------------------
          when COL_FINAL =>
            for c in 0 to 7 loop
              z5 := c_dsp1(0)(c);
              z1 := c_dsp1(1)(c);
              z2 := c_dsp1(2)(c);
              z3 := c_dsp1(3)(c) + z5;
              z4 := c_dsp1(4)(c) + z5;
              -- r0..r3 assembled from DSP products + z corrections
              -- r0 = d7*FIX_0_298631336 + z1 + z3
              -- r1 = d5*FIX_2_053119869 + z2 + z4
              -- r2 = d3*FIX_3_072711026 + z2 + z3
              -- r3 = d1*FIX_1_501321110 + z1 + z4
              col_buf(0)(c) <= idescale(c_tmp10(c) + c_dsp2(3)(c) + z1 + z4, 17);
              col_buf(7)(c) <= idescale(c_tmp10(c) - c_dsp2(3)(c) - z1 - z4, 17);
              col_buf(1)(c) <= idescale(c_tmp11(c) + c_dsp2(2)(c) + z2 + z3, 17);
              col_buf(6)(c) <= idescale(c_tmp11(c) - c_dsp2(2)(c) - z2 - z3, 17);
              col_buf(2)(c) <= idescale(c_tmp12(c) + c_dsp2(1)(c) + z2 + z4, 17);
              col_buf(5)(c) <= idescale(c_tmp12(c) - c_dsp2(1)(c) - z2 - z4, 17);
              col_buf(3)(c) <= idescale(c_tmp13(c) + c_dsp2(0)(c) + z1 + z3, 17);
              col_buf(4)(c) <= idescale(c_tmp13(c) - c_dsp2(0)(c) - z1 - z3, 17);
            end loop;
            emit_row <= 0;
            state    <= ROW_ADD;

          -- ------------------------------------------------------------------
          -- ROW_ADD: row butterfly additions for current emit_row
          -- Row pass shift = IDCT_CONST_BITS - IDCT_PASS1_BITS + 3 = 15
          -- Inputs from col_buf[emit_row][0..7]
          -- ------------------------------------------------------------------
          -- ROW_ADD: pure additions only — no multiplications.
          -- Feeds registered signals into ROW_MUL so each DSP has a
          -- registered input and the critical path is: register → DSP → register.
          when ROW_ADD =>
            d := col_buf(emit_row);
            -- Even part pre-sums (register → ROW_MUL multiplies)
            r_d04  <= d(0) + d(4);
            r_d04d <= d(0) - d(4);
            r_d26  <= d(2) + d(6);
            r_d6   <= d(6);
            r_d2   <= d(2);
            -- Odd part pre-sums
            r_z_add(0) <= d(7) + d(1);
            r_z_add(1) <= d(5) + d(3);
            r_z_add(2) <= d(7) + d(3);
            r_z_add(3) <= d(5) + d(1);
            r_z34      <= (d(7) + d(3)) + (d(5) + d(1));
            r_d4711(0) <= d(7);
            r_d4711(1) <= d(5);
            r_d4711(2) <= d(3);
            r_d4711(3) <= d(1);
            state <= ROW_MUL;

          -- ------------------------------------------------------------------
          -- ROW_MUL: row DSP multiplications
          -- ------------------------------------------------------------------
          when ROW_MUL =>
            -- Even part: two shifts (no DSP) + three DSP multiplies
            r_e_base <= shift_left(r_d04,  IDCT_CONST_BITS);
            r_e_diff <= shift_left(r_d04d, IDCT_CONST_BITS);
            r_e_mul0 <= mul32(r_d26, FIX_0_541196100);
            r_e_mul1 <= mul32(r_d6,  -FIX_1_847759065);
            r_e_mul2 <= mul32(r_d2,   FIX_0_765366865);
            -- Odd part: unchanged
            r_dsp1(0) <= mul32(r_z34,      FIX_1_175875602);
            r_dsp1(1) <= mul32(r_z_add(0), -FIX_0_899976223);
            r_dsp1(2) <= mul32(r_z_add(1), -FIX_2_562915447);
            r_dsp1(3) <= mul32(r_z_add(2), -FIX_1_961570560);
            r_dsp1(4) <= mul32(r_z_add(3), -FIX_0_390180644);
            r_dsp2(0) <= mul32(r_d4711(0),  FIX_0_298631336);
            r_dsp2(1) <= mul32(r_d4711(1),  FIX_2_053119869);
            r_dsp2(2) <= mul32(r_d4711(2),  FIX_3_072711026);
            r_dsp2(3) <= mul32(r_d4711(3),  FIX_1_501321110);
            state <= ROW_FINAL;

          -- ------------------------------------------------------------------
          -- ROW_FINAL: complete row butterfly, descale(sh=15), emit
          -- ------------------------------------------------------------------
          when ROW_FINAL =>
            if m_tready = '1' then
              -- Reconstruct even-part butterfly from pipeline registers
              -- tmp2 = (d2+d6)*FIX_0_541 + d2*FIX_0_765
              -- tmp3 = (d2+d6)*FIX_0_541 + d6*(-FIX_1_847)
              tmp2 := r_e_mul0 + r_e_mul2;
              tmp3 := r_e_mul0 + r_e_mul1;
              -- tmp10 = (d0+d4)<<13 + tmp2,  tmp13 = (d0+d4)<<13 - tmp2
              -- tmp11 = (d0-d4)<<13 + tmp3,  tmp12 = (d0-d4)<<13 - tmp3
              tmp0 := r_e_base + tmp2;   -- tmp10
              tmp1 := r_e_base - tmp2;   -- tmp13
              -- Reuse tmp2/tmp3 as tmp11/tmp12
              tmp2 := r_e_diff + tmp3;   -- tmp11
              tmp3 := r_e_diff - tmp3;   -- tmp12
              z5 := r_dsp1(0);
              z1 := r_dsp1(1);
              z2 := r_dsp1(2);
              z3 := r_dsp1(3) + z5;
              z4 := r_dsp1(4) + z5;
              o(0) := idescale(tmp0 + r_dsp2(3) + z1 + z4, 15);
              o(7) := idescale(tmp0 - r_dsp2(3) - z1 - z4, 15);
              o(1) := idescale(tmp2 + r_dsp2(2) + z2 + z3, 15);
              o(6) := idescale(tmp2 - r_dsp2(2) - z2 - z3, 15);
              o(2) := idescale(tmp3 + r_dsp2(1) + z2 + z4, 15);
              o(5) := idescale(tmp3 - r_dsp2(1) - z2 - z4, 15);
              o(3) := idescale(tmp1 + r_dsp2(0) + z1 + z3, 15);
              o(4) := idescale(tmp1 - r_dsp2(0) - z1 - z3, 15);
              out_row <= o;
              m_tvalid <= '1';

              if emit_row = 7 then
                m_tlast  <= '1';
                emit_row <= 0;
                state    <= LOAD;
              else
                m_tlast  <= '0';
                emit_row <= emit_row + 1;
                state    <= ROW_ADD;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

  -- Pack output row
  gen_out : for i in 0 to 7 generate
    m_tdata(i*16+15 downto i*16) <= std_logic_vector(resize(out_row(i), 16));
  end generate;

end architecture rtl;
