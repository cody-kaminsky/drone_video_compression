-- =============================================================================
-- dct8_fwd.vhd  --  Pipelined 8×8 forward integer DCT
--
-- Translates dct_hw.c (DctFwdHwState / dct_fwd_hw_tick) to synthesisable VHDL.
--
-- Interface
-- ---------
--   Input  : one residual row per clock (8 × signed-9-bit packed as 72-bit bus)
--   Output : one DCT coefficient row per clock (8 × signed-32-bit = 256-bit bus)
--
-- Timing (200 MHz on UltraScale+)
-- --------------------------------
--   FILL phase : 8 clocks  (one row in per clock, row butterfly in same cycle)
--   COL phase  : 1 clock   (all 8 column butterflies run in parallel via DSP chains)
--   EMIT phase : 8 clocks  (one row out per clock)
--   Latency    : 17 clocks from first input row to first output row
--
-- Resource estimate
-- -----------------
--   DSP58E2 : ~40  (row + column butterfly multipliers)
--   FF      : ~600 (two 8×8×32-bit ping-pong register arrays + FSM)
--   BRAM    :   0  (arrays small enough for distributed RAM)
--
-- Synthesis hints
-- ---------------
--   The row/column butterfly logic is purely combinatorial.  Vivado will place
--   the multipliers into DSP58E2 blocks automatically; annotate with
--   (* use_dsp = "yes" *) if needed.  Enable register retiming
--   (-retiming true in synth_design) to balance pipeline stages automatically.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity dct8_fwd is
  port (
    aclk      : in  std_logic;
    aresetn   : in  std_logic;  -- synchronous active-low reset

    -- Input: one residual row (8 × 9-bit residuals packed into 72 bits)
    -- Bits [8:0]   = sample 0,  [17:9] = sample 1, ... [71:63] = sample 7
    s_tdata   : in  std_logic_vector(71 downto 0);
    s_tvalid  : in  std_logic;
    s_tready  : out std_logic;  -- '1' in FILL phase

    -- Output: one coefficient row (8 × 32-bit signed, 256 bits)
    -- Bits [31:0] = coeff 0,  [63:32] = coeff 1, ... [255:224] = coeff 7
    m_tdata   : out std_logic_vector(255 downto 0);
    m_tvalid  : out std_logic;
    m_tlast   : out std_logic;  -- '1' on final row (row 7)
    m_tready  : in  std_logic   -- backpressure from downstream
  );
end entity dct8_fwd;

architecture rtl of dct8_fwd is

  -- Arithmetic right-shift with round-to-nearest (DESCALE macro)
  function descale(x : s32_t; n : integer) return s32_t is
    variable rounded : signed(32 downto 0);
    variable bias    : signed(32 downto 0);
  begin
    bias    := to_signed(1, 33) sll (n - 1);
    rounded := resize(x, 33) + bias;
    return resize(shift_right(rounded, n), 32);
  end function;

  -- Signed multiply sized for a single DSP48E2 (25×18 → 43-bit product).
  -- All DCT constants fit in 18 bits; butterfly data fits in 25 bits.
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

  -- FSM
  type state_t is (FILL, COL_ADD, COL_MUL, COL_FINAL, EMIT);
  signal state   : state_t := FILL;

  -- Row / output counters
  signal in_row  : integer range 0 to 7 := 0;
  signal out_row : integer range 0 to 7 := 0;

  -- 4-stage FILL pipeline (4 clocks per input row):
  --   Stage 0: accept s_tdata → row_in_r
  --   Stage 1: butterfly additions → row_btfly_r + pre-sums
  --   Stage 2: DSP multiplications only → row_dsp1_r / row_dsp2_r
  --   Stage 3: final additions + descale → row_buf  (~3 ns: 3×CARRY8 only)
  signal row_in_r    : std_logic_vector(71 downto 0) := (others => '0');
  signal row_in_idx  : integer range 0 to 7 := 0;
  signal row_btfly_r : int32_row_t;   -- [tmp4,tmp5,tmp6,tmp7,tmp12,tmp13,r0,r4]
  signal fill_stage  : integer range 0 to 3 := 0;
  signal row_z34_r   : s32_t;
  signal row_z12_r   : s32_t;
  signal row_z_add_r : int32_row_t;   -- [0]=tmp4+tmp7,[1]=tmp5+tmp6,[2]=tmp4+tmp6,[3]=tmp5+tmp7
  -- Stage 2 DSP product registers → feed stage 3 with no preceding CARRY8
  --   row_dsp1_r: [z5, z1_mul, z2_mul, z3_mul, z4_mul, z1_even, r0, r4]
  --   row_dsp2_r: [r7_a, r5_a, r6_a, r1_a, r13_mul, r12_mul, -, -]
  signal row_dsp1_r  : int32_row_t;
  signal row_dsp2_r  : int32_row_t;

  -- Intermediate row buffer: holds row-pass output (8 rows × 8 × 32 bit)
  signal row_buf : block32_t;

  -- Column butterfly intermediate registers (COL_ADD → COL_MUL):
  --   col_btfly_r(slot)(col) where slot:
  --     0=tmp4, 1=tmp5, 2=tmp6, 3=tmp7, 4=tmp12, 5=tmp13, 6=r0, 7=r4
  --   col_z34_r(col) = (tmp4+tmp6)+(tmp5+tmp7) — z3+z4 precomputed so COL_MUL
  --                    feeds z5 DSP directly with no preceding CARRY8 adder.
  --   col_z12_r(col) = tmp12+tmp13 — similarly precomputed for z1_even DSP.
  signal col_btfly_r : block32_t;
  signal col_z34_r   : int32_row_t;  -- (tmp4+tmp6)+(tmp5+tmp7) per col
  signal col_z12_r   : int32_row_t;  -- tmp12+tmp13 per col
  -- z1-z4 pre-sums per column; removes CARRY8 stages before those DSPs in COL_MUL
  --   col_z_add_r(0)(c)=tmp4+tmp7, (1)=tmp5+tmp6, (2)=tmp4+tmp6, (3)=tmp5+tmp7
  signal col_z_add_r : block32_t;
  -- COL_MUL DSP product registers → feed COL_FINAL with no preceding CARRY8
  --   col_dsp1_r(slot)(col): [z5, z1_mul, z2_mul, z3_mul, z4_mul, z1_even, r0, r4]
  --   col_dsp2_r(slot)(col): [r7_a, r5_a, r6_a, r1_a, r13_mul, r12_mul, -, -]
  signal col_dsp1_r  : block32_t;
  signal col_dsp2_r  : block32_t;

  -- Output buffer: holds col-pass output (8 rows × 8 × 32 bit)
  signal out_buf : block32_t;

  -- Registered output row (valid in EMIT)
  signal cur_out : int32_row_t;

  -- -------------------------------------------------------------------------
  -- Row butterfly (direct translation of fdct_row in dct_hw.c)
  -- -------------------------------------------------------------------------
  function fdct_row(d : int32_row_t) return int32_row_t is
    variable r : int32_row_t;
    variable tmp0, tmp1, tmp2, tmp3   : s32_t;
    variable tmp4, tmp5, tmp6, tmp7   : s32_t;
    variable tmp10, tmp11, tmp12, tmp13 : s32_t;
    variable z1, z2, z3, z4, z5      : s32_t;
  begin
    tmp0 := d(0) + d(7);  tmp7 := d(0) - d(7);
    tmp1 := d(1) + d(6);  tmp6 := d(1) - d(6);
    tmp2 := d(2) + d(5);  tmp5 := d(2) - d(5);
    tmp3 := d(3) + d(4);  tmp4 := d(3) - d(4);

    tmp10 := tmp0 + tmp3;  tmp13 := tmp0 - tmp3;
    tmp11 := tmp1 + tmp2;  tmp12 := tmp1 - tmp2;

    r(0) := shift_left(tmp10 + tmp11, PASS1_BITS);
    r(4) := shift_left(tmp10 - tmp11, PASS1_BITS);

    z1   := mul32(tmp12 + tmp13, FIX_0_541196100);
    r(2) := descale(z1 + mul32(tmp13,  FIX_0_765366865), CONST_BITS - PASS1_BITS);
    r(6) := descale(z1 + mul32(tmp12, -FIX_1_847759065), CONST_BITS - PASS1_BITS);

    z1 := tmp4 + tmp7;
    z2 := tmp5 + tmp6;
    z3 := tmp4 + tmp6;
    z4 := tmp5 + tmp7;
    z5 := mul32(z3 + z4,  FIX_1_175875602);
    z1 := mul32(z1, -FIX_0_899976223);
    z2 := mul32(z2, -FIX_2_562915447);
    z3 := mul32(z3, -FIX_1_961570560);
    z4 := mul32(z4, -FIX_0_390180644);
    z3 := z3 + z5;
    z4 := z4 + z5;

    r(7) := descale(mul32(tmp4, FIX_0_298631336) + z1 + z3, CONST_BITS - PASS1_BITS);
    r(5) := descale(mul32(tmp5, FIX_2_053119869) + z2 + z4, CONST_BITS - PASS1_BITS);
    r(3) := descale(mul32(tmp6, FIX_3_072711026) + z2 + z3, CONST_BITS - PASS1_BITS);
    r(1) := descale(mul32(tmp7, FIX_1_501321110) + z1 + z4, CONST_BITS - PASS1_BITS);
    return r;
  end function;

  -- -------------------------------------------------------------------------
  -- Column butterfly: same arithmetic, strides through column c of block
  -- -------------------------------------------------------------------------
  function fdct_col(b : block32_t; c : integer) return block32_t is
    variable r : block32_t;
    variable tmp0, tmp1, tmp2, tmp3   : s32_t;
    variable tmp4, tmp5, tmp6, tmp7   : s32_t;
    variable tmp10, tmp11, tmp12, tmp13 : s32_t;
    variable z1, z2, z3, z4, z5      : s32_t;
  begin
    r := b;  -- copy; only column c is modified

    tmp0 := b(0)(c) + b(7)(c);  tmp7 := b(0)(c) - b(7)(c);
    tmp1 := b(1)(c) + b(6)(c);  tmp6 := b(1)(c) - b(6)(c);
    tmp2 := b(2)(c) + b(5)(c);  tmp5 := b(2)(c) - b(5)(c);
    tmp3 := b(3)(c) + b(4)(c);  tmp4 := b(3)(c) - b(4)(c);

    tmp10 := tmp0 + tmp3;  tmp13 := tmp0 - tmp3;
    tmp11 := tmp1 + tmp2;  tmp12 := tmp1 - tmp2;

    r(0)(c) := descale(tmp10 + tmp11, PASS1_BITS);
    r(4)(c) := descale(tmp10 - tmp11, PASS1_BITS);

    z1      := mul32(tmp12 + tmp13, FIX_0_541196100);
    r(2)(c) := descale(z1 + mul32(tmp13,  FIX_0_765366865), CONST_BITS + PASS1_BITS);
    r(6)(c) := descale(z1 + mul32(tmp12, -FIX_1_847759065), CONST_BITS + PASS1_BITS);

    z1 := tmp4 + tmp7;
    z2 := tmp5 + tmp6;
    z3 := tmp4 + tmp6;
    z4 := tmp5 + tmp7;
    z5 := mul32(z3 + z4,  FIX_1_175875602);
    z1 := mul32(z1, -FIX_0_899976223);
    z2 := mul32(z2, -FIX_2_562915447);
    z3 := mul32(z3, -FIX_1_961570560);
    z4 := mul32(z4, -FIX_0_390180644);
    z3 := z3 + z5;
    z4 := z4 + z5;

    r(7)(c) := descale(mul32(tmp4, FIX_0_298631336) + z1 + z3, CONST_BITS + PASS1_BITS);
    r(5)(c) := descale(mul32(tmp5, FIX_2_053119869) + z2 + z4, CONST_BITS + PASS1_BITS);
    r(3)(c) := descale(mul32(tmp6, FIX_3_072711026) + z2 + z3, CONST_BITS + PASS1_BITS);
    r(1)(c) := descale(mul32(tmp7, FIX_1_501321110) + z1 + z4, CONST_BITS + PASS1_BITS);
    return r;
  end function;

  -- -------------------------------------------------------------------------
  -- Helper: unpack 72-bit TDATA → int32_row_t
  -- -------------------------------------------------------------------------
  function unpack_row(v : std_logic_vector(71 downto 0)) return int32_row_t is
    variable r : int32_row_t;
  begin
    for i in 0 to 7 loop
      r(i) := resize(signed(v(i*9+8 downto i*9)), 32);
    end loop;
    return r;
  end function;

begin

  -- -------------------------------------------------------------------------
  -- Pipeline register process (one rising edge = one clock cycle)
  -- -------------------------------------------------------------------------
  process(aclk)
    variable tmp_blk                    : block32_t;
    variable d                          : int32_row_t;
    variable tmp0, tmp1, tmp2, tmp3     : s32_t;
    variable tmp4, tmp5, tmp6, tmp7     : s32_t;
    variable tmp10, tmp11, tmp12, tmp13 : s32_t;
    variable z1, z2, z3, z4, z5        : s32_t;
    variable r_out                      : int32_row_t;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state       <= FILL;
        in_row      <= 0;
        out_row     <= 0;
        m_tvalid    <= '0';
        m_tlast     <= '0';
        s_tready    <= '1';
        fill_stage  <= 0;
      else
        -- Default: output not valid
        m_tvalid <= '0';
        m_tlast  <= '0';

        case state is

          -- ---------------------------------------------------------------
          -- FILL: 3 clocks per row to break combinatorial timing paths.
          --
          --   fill_stage 0  — accept s_tdata into row_in_r (wires only).
          --   fill_stage 1  — butterfly additions → row_btfly_r (~2 ns).
          --                   Stores [tmp4,tmp5,tmp6,tmp7,tmp12,tmp13,r0,r4].
          --   fill_stage 2  — multiplications + descale → row_buf  (~4 ns).
          --
          -- Total FILL = 24 clocks; pipeline latency = 24+1+8 = 33 clocks.
          -- ---------------------------------------------------------------
          when FILL =>

            if fill_stage = 0 then
              -- Stage 0: wait for valid input, register raw bytes
              s_tready <= '1';
              if s_tvalid = '1' then
                row_in_r   <= s_tdata;
                row_in_idx <= in_row;
                fill_stage <= 1;
                s_tready   <= '0';
              end if;

            elsif fill_stage = 1 then
              -- Stage 1: butterfly additions only (no multiplications)
              d     := unpack_row(row_in_r);
              tmp0  := d(0) + d(7);  tmp7 := d(0) - d(7);
              tmp1  := d(1) + d(6);  tmp6 := d(1) - d(6);
              tmp2  := d(2) + d(5);  tmp5 := d(2) - d(5);
              tmp3  := d(3) + d(4);  tmp4 := d(3) - d(4);
              tmp10 := tmp0 + tmp3;  tmp13 := tmp0 - tmp3;
              tmp11 := tmp1 + tmp2;  tmp12 := tmp1 - tmp2;
              -- Pack intermediates into row_btfly_r for stage 2
              row_btfly_r(0) <= tmp4;
              row_btfly_r(1) <= tmp5;
              row_btfly_r(2) <= tmp6;
              row_btfly_r(3) <= tmp7;
              row_btfly_r(4) <= tmp12;
              row_btfly_r(5) <= tmp13;
              row_btfly_r(6) <= shift_left(tmp10 + tmp11, PASS1_BITS);  -- r(0)
              row_btfly_r(7) <= shift_left(tmp10 - tmp11, PASS1_BITS);  -- r(4)
              -- Precompute z-sums so fill_stage 2 DSPs have no preceding CARRY8
              row_z34_r      <= (tmp4 + tmp6) + (tmp5 + tmp7);
              row_z12_r      <= tmp12 + tmp13;
              row_z_add_r(0) <= tmp4 + tmp7;
              row_z_add_r(1) <= tmp5 + tmp6;
              row_z_add_r(2) <= tmp4 + tmp6;
              row_z_add_r(3) <= tmp5 + tmp7;
              fill_stage <= 2;

            elsif fill_stage = 2 then
              -- Stage 2: DSP multiplications only → row_dsp1_r / row_dsp2_r
              -- All inputs are registered pre-sums; no CARRY8 before any DSP.
              tmp4  := row_btfly_r(0);  tmp5  := row_btfly_r(1);
              tmp6  := row_btfly_r(2);  tmp7  := row_btfly_r(3);
              tmp12 := row_btfly_r(4);  tmp13 := row_btfly_r(5);
              row_dsp1_r(0) <= mul32(row_z34_r,      FIX_1_175875602);   -- z5
              row_dsp1_r(1) <= mul32(row_z_add_r(0), -FIX_0_899976223);  -- z1_mul
              row_dsp1_r(2) <= mul32(row_z_add_r(1), -FIX_2_562915447);  -- z2_mul
              row_dsp1_r(3) <= mul32(row_z_add_r(2), -FIX_1_961570560);  -- z3_mul
              row_dsp1_r(4) <= mul32(row_z_add_r(3), -FIX_0_390180644);  -- z4_mul
              row_dsp1_r(5) <= mul32(row_z12_r,       FIX_0_541196100);  -- z1_even
              row_dsp1_r(6) <= row_btfly_r(6);                           -- r(0)
              row_dsp1_r(7) <= row_btfly_r(7);                           -- r(4)
              row_dsp2_r(0) <= mul32(tmp4,  FIX_0_298631336);  -- r7_a
              row_dsp2_r(1) <= mul32(tmp5,  FIX_2_053119869);  -- r5_a
              row_dsp2_r(2) <= mul32(tmp6,  FIX_3_072711026);  -- r6_a
              row_dsp2_r(3) <= mul32(tmp7,  FIX_1_501321110);  -- r1_a
              row_dsp2_r(4) <= mul32(tmp13,  FIX_0_765366865); -- r13_mul
              row_dsp2_r(5) <= mul32(tmp12, -FIX_1_847759065); -- r12_mul
              fill_stage <= 3;

            else
              -- Stage 3: final additions + descale → row_buf
              -- All inputs are registered DSP products; critical path = 3×CARRY8 ~3 ns.
              r_out(0) := row_dsp1_r(6);
              r_out(4) := row_dsp1_r(7);
              z1 := row_dsp1_r(1);  -- z1_mul
              z2 := row_dsp1_r(2);  -- z2_mul
              z3 := row_dsp1_r(3) + row_dsp1_r(0);  -- z3_mul + z5
              z4 := row_dsp1_r(4) + row_dsp1_r(0);  -- z4_mul + z5
              r_out(2) := descale(row_dsp1_r(5) + row_dsp2_r(4), CONST_BITS - PASS1_BITS);
              r_out(6) := descale(row_dsp1_r(5) + row_dsp2_r(5), CONST_BITS - PASS1_BITS);
              r_out(7) := descale(row_dsp2_r(0) + z1 + z3, CONST_BITS - PASS1_BITS);
              r_out(5) := descale(row_dsp2_r(1) + z2 + z4, CONST_BITS - PASS1_BITS);
              r_out(3) := descale(row_dsp2_r(2) + z2 + z3, CONST_BITS - PASS1_BITS);
              r_out(1) := descale(row_dsp2_r(3) + z1 + z4, CONST_BITS - PASS1_BITS);
              row_buf(row_in_idx) <= r_out;
              fill_stage <= 0;

              if row_in_idx = 7 then
                in_row   <= 0;
                s_tready <= '0';
                state    <= COL_ADD;
              else
                in_row   <= row_in_idx + 1;
                s_tready <= '1';
              end if;
            end if;

          -- ---------------------------------------------------------------
          -- COL_ADD: butterfly additions for all 8 columns in parallel.
          --   Produces col_btfly_r [tmp4-tmp7, tmp12, tmp13, r0, r4] and
          --   precomputed sums col_z34_r, col_z12_r that remove one CARRY8
          --   from the COL_MUL critical path (z5 and z1_even DSPs start
          --   directly from a register, not through an adder).
          -- ---------------------------------------------------------------
          when COL_ADD =>
            s_tready <= '0';
            for c in 0 to 7 loop
              tmp0  := row_buf(0)(c) + row_buf(7)(c);  tmp7 := row_buf(0)(c) - row_buf(7)(c);
              tmp1  := row_buf(1)(c) + row_buf(6)(c);  tmp6 := row_buf(1)(c) - row_buf(6)(c);
              tmp2  := row_buf(2)(c) + row_buf(5)(c);  tmp5 := row_buf(2)(c) - row_buf(5)(c);
              tmp3  := row_buf(3)(c) + row_buf(4)(c);  tmp4 := row_buf(3)(c) - row_buf(4)(c);
              tmp10 := tmp0 + tmp3;  tmp13 := tmp0 - tmp3;
              tmp11 := tmp1 + tmp2;  tmp12 := tmp1 - tmp2;
              col_btfly_r(0)(c) <= tmp4;
              col_btfly_r(1)(c) <= tmp5;
              col_btfly_r(2)(c) <= tmp6;
              col_btfly_r(3)(c) <= tmp7;
              col_btfly_r(4)(c) <= tmp12;
              col_btfly_r(5)(c) <= tmp13;
              col_btfly_r(6)(c) <= descale(tmp10 + tmp11, PASS1_BITS);  -- r(0)
              col_btfly_r(7)(c) <= descale(tmp10 - tmp11, PASS1_BITS);  -- r(4)
              col_z34_r(c)   <= (tmp4 + tmp6) + (tmp5 + tmp7);  -- z3+z4 for z5 DSP
              col_z12_r(c)   <= tmp12 + tmp13;                  -- for z1_even DSP
              col_z_add_r(0)(c) <= tmp4 + tmp7;  -- z1_add for z1 DSP
              col_z_add_r(1)(c) <= tmp5 + tmp6;  -- z2_add for z2 DSP
              col_z_add_r(2)(c) <= tmp4 + tmp6;  -- z3_add for z3 DSP
              col_z_add_r(3)(c) <= tmp5 + tmp7;  -- z4_add for z4 DSP
            end loop;
            state <= COL_MUL;

          -- ---------------------------------------------------------------
          -- COL_MUL: DSP multiplications only → col_dsp1_r / col_dsp2_r.
          -- All DSP inputs are registered pre-sums; no CARRY8 before any DSP.
          -- ---------------------------------------------------------------
          when COL_MUL =>
            s_tready <= '0';
            for c in 0 to 7 loop
              tmp4  := col_btfly_r(0)(c);  tmp5  := col_btfly_r(1)(c);
              tmp6  := col_btfly_r(2)(c);  tmp7  := col_btfly_r(3)(c);
              tmp12 := col_btfly_r(4)(c);  tmp13 := col_btfly_r(5)(c);
              col_dsp1_r(0)(c) <= mul32(col_z34_r(c),      FIX_1_175875602);   -- z5
              col_dsp1_r(1)(c) <= mul32(col_z_add_r(0)(c), -FIX_0_899976223);  -- z1_mul
              col_dsp1_r(2)(c) <= mul32(col_z_add_r(1)(c), -FIX_2_562915447);  -- z2_mul
              col_dsp1_r(3)(c) <= mul32(col_z_add_r(2)(c), -FIX_1_961570560);  -- z3_mul
              col_dsp1_r(4)(c) <= mul32(col_z_add_r(3)(c), -FIX_0_390180644);  -- z4_mul
              col_dsp1_r(5)(c) <= mul32(col_z12_r(c),       FIX_0_541196100);  -- z1_even
              col_dsp1_r(6)(c) <= col_btfly_r(6)(c);                           -- r(0)
              col_dsp1_r(7)(c) <= col_btfly_r(7)(c);                           -- r(4)
              col_dsp2_r(0)(c) <= mul32(tmp4,  FIX_0_298631336);  -- r7_a
              col_dsp2_r(1)(c) <= mul32(tmp5,  FIX_2_053119869);  -- r5_a
              col_dsp2_r(2)(c) <= mul32(tmp6,  FIX_3_072711026);  -- r6_a
              col_dsp2_r(3)(c) <= mul32(tmp7,  FIX_1_501321110);  -- r1_a
              col_dsp2_r(4)(c) <= mul32(tmp13,  FIX_0_765366865); -- r13_mul
              col_dsp2_r(5)(c) <= mul32(tmp12, -FIX_1_847759065); -- r12_mul
            end loop;
            state <= COL_FINAL;

          -- ---------------------------------------------------------------
          -- COL_FINAL: final additions + descale → out_buf.
          -- All inputs are registered DSP products; critical path = 3×CARRY8 ~3 ns.
          -- ---------------------------------------------------------------
          when COL_FINAL =>
            s_tready <= '0';
            for c in 0 to 7 loop
              z1 := col_dsp1_r(1)(c);  -- z1_mul
              z2 := col_dsp1_r(2)(c);  -- z2_mul
              z3 := col_dsp1_r(3)(c) + col_dsp1_r(0)(c);  -- z3_mul + z5
              z4 := col_dsp1_r(4)(c) + col_dsp1_r(0)(c);  -- z4_mul + z5
              out_buf(0)(c) <= col_dsp1_r(6)(c);
              out_buf(4)(c) <= col_dsp1_r(7)(c);
              out_buf(2)(c) <= descale(col_dsp1_r(5)(c) + col_dsp2_r(4)(c), CONST_BITS + PASS1_BITS);
              out_buf(6)(c) <= descale(col_dsp1_r(5)(c) + col_dsp2_r(5)(c), CONST_BITS + PASS1_BITS);
              out_buf(7)(c) <= descale(col_dsp2_r(0)(c) + z1 + z3, CONST_BITS + PASS1_BITS);
              out_buf(5)(c) <= descale(col_dsp2_r(1)(c) + z2 + z4, CONST_BITS + PASS1_BITS);
              out_buf(3)(c) <= descale(col_dsp2_r(2)(c) + z2 + z3, CONST_BITS + PASS1_BITS);
              out_buf(1)(c) <= descale(col_dsp2_r(3)(c) + z1 + z4, CONST_BITS + PASS1_BITS);
            end loop;
            out_row <= 0;
            state   <= EMIT;

          -- ---------------------------------------------------------------
          -- EMIT: drive one output row per clock (AXI-S compliant).
          -- m_tvalid is held stable once asserted; only cleared after the
          -- consumer accepts the row (m_tready='1').  Without this hold the
          -- global default '0' would clear m_tvalid every cycle that m_tready
          -- is low, causing the serialiser to miss every odd-numbered row.
          -- ---------------------------------------------------------------
          when EMIT =>
            -- Override the global default: hold m_tvalid/m_tlast until consumed.
            m_tvalid <= m_tvalid;
            m_tlast  <= m_tlast;

            if m_tvalid = '0' then
              -- Slot empty: present the current row
              cur_out  <= out_buf(out_row);
              m_tvalid <= '1';
              m_tlast  <= '1' when out_row = 7 else '0';
            elsif m_tready = '1' then
              -- Handshake: consumer accepted the row; advance
              m_tvalid <= '0';
              m_tlast  <= '0';
              if out_row = 7 then
                out_row  <= 0;
                s_tready <= '1';
                state    <= FILL;
              else
                out_row  <= out_row + 1;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

  -- Pack cur_out into TDATA bus
  gen_tdata : for i in 0 to 7 generate
    m_tdata(i*32+31 downto i*32) <= std_logic_vector(cur_out(i));
  end generate;

end architecture rtl;
