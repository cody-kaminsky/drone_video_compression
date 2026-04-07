-- =============================================================================
-- dct8_fwd.vhd  --  Pipelined 8×8 forward integer DCT
--
-- DSP sharing strategy: all 12 multiplications live in a dedicated always-on
-- registered process (dsp_mul).  Each sh_dsp signal has exactly ONE driver in
-- the entire design, so Vivado infers exactly ONE DSP48E2 per line regardless
-- of synthesis settings.  The main FSM only loads the pre-mux inputs (dsp_a)
-- and uses the registered outputs two cycles later.
--
-- Interface (unchanged)
--   Input  : one residual row per clock (8 × signed-9-bit packed as 72-bit)
--   Output : one DCT coefficient row per clock (8 × signed-32-bit = 256-bit)
--
-- Timing
--   FILL phase : 10 clocks  (pipelined: 1 row/clock, 2-cycle DSP latency)
--   COL_PASS   : 10 clocks  (pipelined: 1 col/clock, 2-cycle DSP latency)
--   EMIT phase :  8 clocks
--   Total      : 28 clocks per 8×8 block
--
-- Pipeline structure (FILL and COL_PASS)
--   Cycle 0..7 : accept row/col butterfly → dsp_a, pass/col r0/r4
--   Cycle 1    : pipeline r0_p <= r0  (captures previous cycle's even-part DC)
--   Cycle 2..9 : FINAL fires using sh_dsp (2 cycles after dsp_a load) + r0_p
--   Total      : 8 loads + 2 drain = 10 cycles
--
-- Resource estimate
--   DSP58E2 : 12
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity dct8_fwd is
  port (
    aclk      : in  std_logic;
    aresetn   : in  std_logic;

    s_tdata   : in  std_logic_vector(71 downto 0);
    s_tvalid  : in  std_logic;
    s_tready  : out std_logic;

    m_tdata   : out std_logic_vector(255 downto 0);
    m_tvalid  : out std_logic;
    m_tlast   : out std_logic;
    m_tready  : in  std_logic
  );
end entity dct8_fwd;

architecture rtl of dct8_fwd is

  function descale(x : s32_t; n : integer) return s32_t is
    variable rounded : signed(32 downto 0);
    variable bias    : signed(32 downto 0);
  begin
    bias    := to_signed(1, 33) sll (n - 1);
    rounded := resize(x, 33) + bias;
    return resize(shift_right(rounded, n), 32);
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

  function unpack_row(v : std_logic_vector(71 downto 0)) return int32_row_t is
    variable r : int32_row_t;
  begin
    for i in 0 to 7 loop
      r(i) := resize(signed(v(i*9+8 downto i*9)), 32);
    end loop;
    return r;
  end function;

  -- -------------------------------------------------------------------------
  -- FSM
  -- -------------------------------------------------------------------------
  type state_t is (FILL, COL_PASS, EMIT);

  signal state : state_t := FILL;

  -- FILL pipeline counters (load 0..7, drain cycles push to 8..9)
  signal fill_load_idx : integer range 0 to 9 := 0;
  signal fill_emit_idx : integer range 0 to 7 := 0;

  -- COL_PASS pipeline counters
  signal col_load_idx  : integer range 0 to 9 := 0;
  signal col_emit_idx  : integer range 0 to 7 := 0;

  signal out_row : integer range 0 to 7 := 0;

  -- FILL even-part DC pipeline (1-cycle delay aligns with 2-cycle sh_dsp latency)
  signal pass_r0   : s32_t := (others => '0');
  signal pass_r4   : s32_t := (others => '0');
  signal pass_r0_p : s32_t := (others => '0');
  signal pass_r4_p : s32_t := (others => '0');

  -- COL_PASS even-part DC pipeline
  signal col_r0    : s32_t := (others => '0');
  signal col_r4    : s32_t := (others => '0');
  signal col_r0_p  : s32_t := (others => '0');
  signal col_r4_p  : s32_t := (others => '0');

  -- Row and column intermediate buffers
  signal row_buf : block32_t;
  signal out_buf : block32_t;

  -- -------------------------------------------------------------------------
  -- Pre-mux DSP A inputs
  -- -------------------------------------------------------------------------
  type dsp_in_t is array(0 to 11) of s32_t;
  signal dsp_a : dsp_in_t;

  -- -------------------------------------------------------------------------
  -- DSP output registers — one driver each → one DSP48E2 each.
  -- -------------------------------------------------------------------------
  signal sh_dsp1 : int32_row_t;   -- indices 0..5 used
  signal sh_dsp2 : int32_row_t;   -- indices 0..5 used

  signal cur_out : int32_row_t;

begin

  -- =========================================================================
  -- Dedicated multiply process — 12 DSP48E2 blocks, one per line.
  -- =========================================================================
  dsp_mul : process(aclk)
  begin
    if rising_edge(aclk) then
      sh_dsp1(0) <= mul32(dsp_a(0),   FIX_1_175875602);
      sh_dsp1(1) <= mul32(dsp_a(1),  -FIX_0_899976223);
      sh_dsp1(2) <= mul32(dsp_a(2),  -FIX_2_562915447);
      sh_dsp1(3) <= mul32(dsp_a(3),  -FIX_1_961570560);
      sh_dsp1(4) <= mul32(dsp_a(4),  -FIX_0_390180644);
      sh_dsp1(5) <= mul32(dsp_a(5),   FIX_0_541196100);
      sh_dsp2(0) <= mul32(dsp_a(6),   FIX_0_298631336);
      sh_dsp2(1) <= mul32(dsp_a(7),   FIX_2_053119869);
      sh_dsp2(2) <= mul32(dsp_a(8),   FIX_3_072711026);
      sh_dsp2(3) <= mul32(dsp_a(9),   FIX_1_501321110);
      sh_dsp2(4) <= mul32(dsp_a(10),  FIX_0_765366865);
      sh_dsp2(5) <= mul32(dsp_a(11), -FIX_1_847759065);
    end if;
  end process;

  -- =========================================================================
  -- Main FSM process
  -- =========================================================================
  fsm : process(aclk)
    variable d                          : int32_row_t;
    variable tmp0, tmp1, tmp2, tmp3     : s32_t;
    variable tmp4, tmp5, tmp6, tmp7     : s32_t;
    variable tmp10, tmp11, tmp12, tmp13 : s32_t;
    variable z1, z2, z3, z4            : s32_t;
    variable r_out                      : int32_row_t;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state         <= FILL;
        fill_load_idx <= 0;
        fill_emit_idx <= 0;
        col_load_idx  <= 0;
        col_emit_idx  <= 0;
        out_row       <= 0;
        m_tvalid      <= '0';
        m_tlast       <= '0';
        s_tready      <= '1';
      else
        m_tvalid <= '0';
        m_tlast  <= '0';

        case state is

          -- ------------------------------------------------------------------
          -- FILL: pipelined, 10 clocks total
          --
          -- Butterfly and dsp_a load happen in the same clock as input accept.
          -- pass_r0_p captures pass_r0 one cycle later (2-cycle total delay =
          -- matches DSP latency), so FINAL fires at fill_load_idx >= 2.
          --
          -- Timeline:
          --   cycle 0: accept row 0, butterfly → dsp_a, pass_r0
          --   cycle 1: accept row 1; pass_r0_p = row 0; sh_dsp computing row 0
          --   cycle 2: accept row 2; pass_r0_p = row 1; FINAL row 0 → row_buf(0)
          --   ...
          --   cycle 7: accept row 7 (last); FINAL row 5
          --   cycle 8: drain; FINAL row 6
          --   cycle 9: drain; FINAL row 7 → COL_PASS
          -- ------------------------------------------------------------------
          when FILL =>

            -- Stage 1 pipeline: even-part DC from previous cycle's butterfly
            pass_r0_p <= pass_r0;
            pass_r4_p <= pass_r4;

            -- Load: accept one input row per clock
            if fill_load_idx < 8 then
              if s_tvalid = '1' then
                d     := unpack_row(s_tdata);
                tmp0  := d(0) + d(7);  tmp7 := d(0) - d(7);
                tmp1  := d(1) + d(6);  tmp6 := d(1) - d(6);
                tmp2  := d(2) + d(5);  tmp5 := d(2) - d(5);
                tmp3  := d(3) + d(4);  tmp4 := d(3) - d(4);
                tmp10 := tmp0 + tmp3;  tmp13 := tmp0 - tmp3;
                tmp11 := tmp1 + tmp2;  tmp12 := tmp1 - tmp2;
                pass_r0   <= shift_left(tmp10 + tmp11, PASS1_BITS);
                pass_r4   <= shift_left(tmp10 - tmp11, PASS1_BITS);
                dsp_a(0)  <= (tmp4 + tmp6) + (tmp5 + tmp7);
                dsp_a(1)  <= tmp4 + tmp7;
                dsp_a(2)  <= tmp5 + tmp6;
                dsp_a(3)  <= tmp4 + tmp6;
                dsp_a(4)  <= tmp5 + tmp7;
                dsp_a(5)  <= tmp12 + tmp13;
                dsp_a(6)  <= tmp4;
                dsp_a(7)  <= tmp5;
                dsp_a(8)  <= tmp6;
                dsp_a(9)  <= tmp7;
                dsp_a(10) <= tmp13;
                dsp_a(11) <= tmp12;
                fill_load_idx <= fill_load_idx + 1;
                if fill_load_idx = 7 then
                  s_tready <= '0';
                end if;
              end if;
            end if;

            -- Emit FINAL: sh_dsp ready 2 cycles after dsp_a; pass_r0_p = 2-cycle delayed
            if fill_load_idx >= 2 then
              r_out(0) := pass_r0_p;
              r_out(4) := pass_r4_p;
              z1 := sh_dsp1(1);
              z2 := sh_dsp1(2);
              z3 := sh_dsp1(3) + sh_dsp1(0);
              z4 := sh_dsp1(4) + sh_dsp1(0);
              r_out(2) := descale(sh_dsp1(5) + sh_dsp2(4), CONST_BITS - PASS1_BITS);
              r_out(6) := descale(sh_dsp1(5) + sh_dsp2(5), CONST_BITS - PASS1_BITS);
              r_out(7) := descale(sh_dsp2(0) + z1 + z3,    CONST_BITS - PASS1_BITS);
              r_out(5) := descale(sh_dsp2(1) + z2 + z4,    CONST_BITS - PASS1_BITS);
              r_out(3) := descale(sh_dsp2(2) + z2 + z3,    CONST_BITS - PASS1_BITS);
              r_out(1) := descale(sh_dsp2(3) + z1 + z4,    CONST_BITS - PASS1_BITS);
              row_buf(fill_emit_idx) <= r_out;

              if fill_emit_idx = 7 then
                fill_load_idx <= 0;
                fill_emit_idx <= 0;
                col_load_idx  <= 0;
                col_emit_idx  <= 0;
                state         <= COL_PASS;
              else
                fill_emit_idx <= fill_emit_idx + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- COL_PASS: pipelined, 10 clocks total
          --
          -- Same 2-cycle pipeline pattern as FILL.
          -- col_r0_p captures col_r0 one cycle later, aligning with sh_dsp.
          --
          -- Timeline:
          --   cycle 0: butterfly col 0 → dsp_a, col_r0/r4
          --   cycle 1: butterfly col 1; col_r0_p = col 0
          --   cycle 2: butterfly col 2; FINAL col 0 → out_buf(:)(0)
          --   ...
          --   cycle 7: butterfly col 7 (last); FINAL col 5
          --   cycle 8: drain; FINAL col 6
          --   cycle 9: drain; FINAL col 7 → EMIT
          -- ------------------------------------------------------------------
          when COL_PASS =>

            -- Stage 1 pipeline: even-part DC from previous cycle's butterfly
            col_r0_p <= col_r0;
            col_r4_p <= col_r4;

            -- Load: butterfly one column per clock
            if col_load_idx < 8 then
              tmp0  := row_buf(0)(col_load_idx) + row_buf(7)(col_load_idx);
              tmp7  := row_buf(0)(col_load_idx) - row_buf(7)(col_load_idx);
              tmp1  := row_buf(1)(col_load_idx) + row_buf(6)(col_load_idx);
              tmp6  := row_buf(1)(col_load_idx) - row_buf(6)(col_load_idx);
              tmp2  := row_buf(2)(col_load_idx) + row_buf(5)(col_load_idx);
              tmp5  := row_buf(2)(col_load_idx) - row_buf(5)(col_load_idx);
              tmp3  := row_buf(3)(col_load_idx) + row_buf(4)(col_load_idx);
              tmp4  := row_buf(3)(col_load_idx) - row_buf(4)(col_load_idx);
              tmp10 := tmp0 + tmp3;  tmp13 := tmp0 - tmp3;
              tmp11 := tmp1 + tmp2;  tmp12 := tmp1 - tmp2;
              col_r0    <= descale(tmp10 + tmp11, PASS1_BITS);
              col_r4    <= descale(tmp10 - tmp11, PASS1_BITS);
              dsp_a(0)  <= (tmp4 + tmp6) + (tmp5 + tmp7);
              dsp_a(1)  <= tmp4 + tmp7;
              dsp_a(2)  <= tmp5 + tmp6;
              dsp_a(3)  <= tmp4 + tmp6;
              dsp_a(4)  <= tmp5 + tmp7;
              dsp_a(5)  <= tmp12 + tmp13;
              dsp_a(6)  <= tmp4;
              dsp_a(7)  <= tmp5;
              dsp_a(8)  <= tmp6;
              dsp_a(9)  <= tmp7;
              dsp_a(10) <= tmp13;
              dsp_a(11) <= tmp12;
              col_load_idx <= col_load_idx + 1;
            end if;

            -- Emit FINAL: sh_dsp ready 2 cycles after dsp_a load
            if col_load_idx >= 2 then
              z1 := sh_dsp1(1);
              z2 := sh_dsp1(2);
              z3 := sh_dsp1(3) + sh_dsp1(0);
              z4 := sh_dsp1(4) + sh_dsp1(0);
              out_buf(0)(col_emit_idx) <= col_r0_p;
              out_buf(4)(col_emit_idx) <= col_r4_p;
              out_buf(2)(col_emit_idx) <= descale(sh_dsp1(5) + sh_dsp2(4), CONST_BITS + PASS1_BITS);
              out_buf(6)(col_emit_idx) <= descale(sh_dsp1(5) + sh_dsp2(5), CONST_BITS + PASS1_BITS);
              out_buf(7)(col_emit_idx) <= descale(sh_dsp2(0) + z1 + z3, CONST_BITS + PASS1_BITS);
              out_buf(5)(col_emit_idx) <= descale(sh_dsp2(1) + z2 + z4, CONST_BITS + PASS1_BITS);
              out_buf(3)(col_emit_idx) <= descale(sh_dsp2(2) + z2 + z3, CONST_BITS + PASS1_BITS);
              out_buf(1)(col_emit_idx) <= descale(sh_dsp2(3) + z1 + z4, CONST_BITS + PASS1_BITS);

              if col_emit_idx = 7 then
                col_load_idx <= 0;
                col_emit_idx <= 0;
                out_row      <= 0;
                state        <= EMIT;
              else
                col_emit_idx <= col_emit_idx + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- EMIT: one output row per clock (AXI-S with backpressure).
          -- ------------------------------------------------------------------
          when EMIT =>
            m_tvalid <= m_tvalid;
            m_tlast  <= m_tlast;

            if m_tvalid = '0' then
              cur_out  <= out_buf(out_row);
              m_tvalid <= '1';
              m_tlast  <= '1' when out_row = 7 else '0';
            elsif m_tready = '1' then
              m_tvalid <= '0';
              m_tlast  <= '0';
              if out_row = 7 then
                out_row       <= 0;
                fill_load_idx <= 0;
                fill_emit_idx <= 0;
                s_tready      <= '1';
                state         <= FILL;
              else
                out_row <= out_row + 1;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

  gen_tdata : for i in 0 to 7 generate
    m_tdata(i*32+31 downto i*32) <= std_logic_vector(cur_out(i));
  end generate;

end architecture rtl;
