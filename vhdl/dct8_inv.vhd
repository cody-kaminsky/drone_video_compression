-- =============================================================================
-- dct8_inv.vhd  --  Pipelined 8×8 inverse integer DCT
--
-- DSP sharing strategy: all 12 multiplications live in a dedicated always-on
-- registered process (dsp_mul).  Each sh_dsp signal has exactly ONE driver,
-- so Vivado infers exactly ONE DSP48E2 per line.
--
-- Interface (unchanged)
--   Input  : one row of 8 dequantised coefficients (256-bit)
--   Output : one residual row (8 × signed-16-bit = 128-bit)
--
-- Timing
--   LOAD     :  8 clocks
--   COL_PASS : 10 clocks  (pipelined: 1 col/clock, 2-cycle DSP latency)
--   ROW_PASS : 10 clocks  (pipelined: 1 row/clock, 2-cycle DSP latency)
--   Total    : 28 clocks per 8×8 block
--
-- Pipeline structure (identical for COL and ROW):
--   Cycle 0..7 : load col/row butterfly → dsp_a + even-part DC signals
--   Cycle 1    : pipeline DC_p <= DC  (1-cycle delay for even-part)
--   Cycle 2..9 : FINAL fires: uses sh_dsp (2-cycle latency) + DC_p (1-cycle)
--   Total      : 8 loads + 2 drain = 10 cycles
--
-- Resource estimate
--   DSP58E2 : 12
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity dct8_inv is
  port (
    aclk     : in  std_logic;
    aresetn  : in  std_logic;

    s_tdata  : in  std_logic_vector(255 downto 0);
    s_tvalid : in  std_logic;
    s_tready : out std_logic;

    m_tdata  : out std_logic_vector(127 downto 0);
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;
    m_tready : in  std_logic
  );
end entity dct8_inv;

architecture rtl of dct8_inv is

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

  function unpack256(v : std_logic_vector(255 downto 0)) return int32_row_t is
    variable r : int32_row_t;
  begin
    for i in 0 to 7 loop
      r(i) := signed(v(i*32+31 downto i*32));
    end loop;
    return r;
  end function;

  -- -------------------------------------------------------------------------
  -- FSM
  -- -------------------------------------------------------------------------
  type state_t is (LOAD, COL_PASS, ROW_PASS);

  signal state    : state_t := LOAD;
  signal load_row : integer range 0 to 7 := 0;

  -- COL_PASS pipeline counters
  signal col_load_idx  : integer range 0 to 9 := 0;
  signal col_emit_idx  : integer range 0 to 7 := 0;

  -- ROW_PASS pipeline counters
  signal row_load_idx  : integer range 0 to 9 := 0;
  signal row_emit_idx  : integer range 0 to 7 := 0;

  signal in_buf  : block32_t;
  signal col_buf : block32_t;

  -- COL_PASS even-part DC pipeline (1-cycle delay)
  signal col_d0pd4   : s32_t := (others => '0');
  signal col_d0md4   : s32_t := (others => '0');
  signal col_d0pd4_p : s32_t := (others => '0');
  signal col_d0md4_p : s32_t := (others => '0');

  -- ROW_PASS even-part DC (r_d04/r_d04d) + 1-cycle pipeline
  signal r_d04        : s32_t := (others => '0');
  signal r_d04d       : s32_t := (others => '0');
  signal row_e_base_p : s32_t := (others => '0');
  signal row_e_diff_p : s32_t := (others => '0');

  -- -------------------------------------------------------------------------
  -- Pre-mux DSP A inputs
  -- -------------------------------------------------------------------------
  type dsp_in_t is array(0 to 11) of s32_t;
  signal dsp_a : dsp_in_t;

  -- -------------------------------------------------------------------------
  -- DSP output registers — one driver each → one DSP48E2 each.
  -- -------------------------------------------------------------------------
  signal sh_dsp1 : int32_row_t;
  signal sh_dsp2 : int32_row_t;

  signal out_row : int32_row_t;

begin

  s_tready <= '1' when state = LOAD else '0';

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
      sh_dsp1(6) <= mul32(dsp_a(6),  -FIX_1_847759065);
      sh_dsp1(7) <= mul32(dsp_a(7),   FIX_0_765366865);
      sh_dsp2(0) <= mul32(dsp_a(8),   FIX_0_298631336);
      sh_dsp2(1) <= mul32(dsp_a(9),   FIX_2_053119869);
      sh_dsp2(2) <= mul32(dsp_a(10),  FIX_3_072711026);
      sh_dsp2(3) <= mul32(dsp_a(11),  FIX_1_501321110);
    end if;
  end process;

  -- =========================================================================
  -- Main FSM process
  -- =========================================================================
  fsm : process(aclk)
    variable tmp0, tmp1, tmp2, tmp3 : s32_t;
    variable z1, z2, z3, z4, z5    : s32_t;
    variable o                      : int32_row_t;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state        <= LOAD;
        load_row     <= 0;
        col_load_idx <= 0;
        col_emit_idx <= 0;
        row_load_idx <= 0;
        row_emit_idx <= 0;
        m_tvalid     <= '0';
        m_tlast      <= '0';
      else
        m_tvalid <= '0';
        m_tlast  <= '0';

        case state is

          -- ------------------------------------------------------------------
          -- LOAD: accept 8 rows (256-bit each), 1 per clock
          -- ------------------------------------------------------------------
          when LOAD =>
            if s_tvalid = '1' then
              in_buf(load_row) <= unpack256(s_tdata);
              if load_row = 7 then
                load_row     <= 0;
                col_load_idx <= 0;
                col_emit_idx <= 0;
                state        <= COL_PASS;
              else
                load_row <= load_row + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- COL_PASS: pipelined, 10 clocks total
          --
          -- col_d0pd4/md4 computed alongside dsp_a load; col_d0pd4_p captures
          -- them 1 cycle later so they align with sh_dsp at FINAL time.
          --
          -- Timeline:
          --   cycle 0: butterfly col 0 → dsp_a, col_d0pd4/md4
          --   cycle 1: butterfly col 1; col_d0pd4_p = col 0 values
          --   cycle 2: butterfly col 2; FINAL col 0 → col_buf(:)(0)
          --   ...
          --   cycle 7: butterfly col 7 (last); FINAL col 5
          --   cycle 8: drain; FINAL col 6
          --   cycle 9: drain; FINAL col 7 → ROW_PASS
          -- ------------------------------------------------------------------
          when COL_PASS =>

            -- Stage 1 pipeline: capture even-part DC from previous cycle
            col_d0pd4_p <= col_d0pd4;
            col_d0md4_p <= col_d0md4;

            -- Load: butterfly one column per clock (cycles 0..7)
            if col_load_idx < 8 then
              col_d0pd4 <= shift_left(in_buf(0)(col_load_idx) + in_buf(4)(col_load_idx),
                                      IDCT_CONST_BITS);
              col_d0md4 <= shift_left(in_buf(0)(col_load_idx) - in_buf(4)(col_load_idx),
                                      IDCT_CONST_BITS);
              dsp_a(0)  <= (in_buf(7)(col_load_idx) + in_buf(3)(col_load_idx))
                         + (in_buf(5)(col_load_idx) + in_buf(1)(col_load_idx));
              dsp_a(1)  <= in_buf(7)(col_load_idx) + in_buf(1)(col_load_idx);
              dsp_a(2)  <= in_buf(5)(col_load_idx) + in_buf(3)(col_load_idx);
              dsp_a(3)  <= in_buf(7)(col_load_idx) + in_buf(3)(col_load_idx);
              dsp_a(4)  <= in_buf(5)(col_load_idx) + in_buf(1)(col_load_idx);
              dsp_a(5)  <= in_buf(2)(col_load_idx) + in_buf(6)(col_load_idx);
              dsp_a(6)  <= in_buf(6)(col_load_idx);
              dsp_a(7)  <= in_buf(2)(col_load_idx);
              dsp_a(8)  <= in_buf(7)(col_load_idx);
              dsp_a(9)  <= in_buf(5)(col_load_idx);
              dsp_a(10) <= in_buf(3)(col_load_idx);
              dsp_a(11) <= in_buf(1)(col_load_idx);
              col_load_idx <= col_load_idx + 1;
            end if;

            -- Emit FINAL: sh_dsp ready 2 cycles after dsp_a; col_d0pd4_p ready 2 cycles after load
            if col_load_idx >= 2 then
              tmp2 := sh_dsp1(5) + sh_dsp1(7);
              tmp3 := sh_dsp1(5) + sh_dsp1(6);
              tmp0 := col_d0pd4_p + tmp2;
              tmp1 := col_d0pd4_p - tmp2;
              tmp2 := col_d0md4_p + tmp3;
              tmp3 := col_d0md4_p - tmp3;
              z5 := sh_dsp1(0);
              z1 := sh_dsp1(1);
              z2 := sh_dsp1(2);
              z3 := sh_dsp1(3) + z5;
              z4 := sh_dsp1(4) + z5;
              col_buf(0)(col_emit_idx) <= idescale(tmp0 + sh_dsp2(3) + z1 + z4, 17);
              col_buf(7)(col_emit_idx) <= idescale(tmp0 - sh_dsp2(3) - z1 - z4, 17);
              col_buf(1)(col_emit_idx) <= idescale(tmp2 + sh_dsp2(2) + z2 + z3, 17);
              col_buf(6)(col_emit_idx) <= idescale(tmp2 - sh_dsp2(2) - z2 - z3, 17);
              col_buf(2)(col_emit_idx) <= idescale(tmp3 + sh_dsp2(1) + z2 + z4, 17);
              col_buf(5)(col_emit_idx) <= idescale(tmp3 - sh_dsp2(1) - z2 - z4, 17);
              col_buf(3)(col_emit_idx) <= idescale(tmp1 + sh_dsp2(0) + z1 + z3, 17);
              col_buf(4)(col_emit_idx) <= idescale(tmp1 - sh_dsp2(0) - z1 - z3, 17);

              if col_emit_idx = 7 then
                col_load_idx <= 0;
                col_emit_idx <= 0;
                row_load_idx <= 0;
                row_emit_idx <= 0;
                state        <= ROW_PASS;
              else
                col_emit_idx <= col_emit_idx + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- ROW_PASS: pipelined, 10 clocks total
          --
          -- r_d04/r_d04d set each cycle; row_e_base_p captures them 1 cycle
          -- later, aligning with sh_dsp (2-cycle DSP latency).
          --
          -- Timeline:
          --   cycle 0: load row 0 → dsp_a, r_d04/d
          --   cycle 1: load row 1; row_e_base_p = row 0 e_base
          --   cycle 2: load row 2; EMIT row 0
          --   ...
          --   cycle 7: load row 7; EMIT row 5
          --   cycle 8: drain; EMIT row 6
          --   cycle 9: drain; EMIT row 7 → LOAD
          -- ------------------------------------------------------------------
          when ROW_PASS =>

            -- Stage 1 pipeline: even-part DC from previous cycle's r_d04
            row_e_base_p <= shift_left(r_d04,  IDCT_CONST_BITS);
            row_e_diff_p <= shift_left(r_d04d, IDCT_CONST_BITS);

            -- Load: set dsp_a for row row_load_idx (cycles 0..7)
            if row_load_idx < 8 then
              r_d04  <= col_buf(row_load_idx)(0) + col_buf(row_load_idx)(4);
              r_d04d <= col_buf(row_load_idx)(0) - col_buf(row_load_idx)(4);
              dsp_a(0)  <= (col_buf(row_load_idx)(7) + col_buf(row_load_idx)(3))
                         + (col_buf(row_load_idx)(5) + col_buf(row_load_idx)(1));
              dsp_a(1)  <= col_buf(row_load_idx)(7) + col_buf(row_load_idx)(1);
              dsp_a(2)  <= col_buf(row_load_idx)(5) + col_buf(row_load_idx)(3);
              dsp_a(3)  <= col_buf(row_load_idx)(7) + col_buf(row_load_idx)(3);
              dsp_a(4)  <= col_buf(row_load_idx)(5) + col_buf(row_load_idx)(1);
              dsp_a(5)  <= col_buf(row_load_idx)(2) + col_buf(row_load_idx)(6);
              dsp_a(6)  <= col_buf(row_load_idx)(6);
              dsp_a(7)  <= col_buf(row_load_idx)(2);
              dsp_a(8)  <= col_buf(row_load_idx)(7);
              dsp_a(9)  <= col_buf(row_load_idx)(5);
              dsp_a(10) <= col_buf(row_load_idx)(3);
              dsp_a(11) <= col_buf(row_load_idx)(1);
              row_load_idx <= row_load_idx + 1;
            end if;

            -- Emit: sh_dsp ready 2 cycles after dsp_a load (cycles 2..9)
            if row_load_idx >= 2 then
              if m_tready = '1' then
                tmp2 := sh_dsp1(5) + sh_dsp1(7);
                tmp3 := sh_dsp1(5) + sh_dsp1(6);
                tmp0 := row_e_base_p + tmp2;
                tmp1 := row_e_base_p - tmp2;
                tmp2 := row_e_diff_p + tmp3;
                tmp3 := row_e_diff_p - tmp3;
                z5 := sh_dsp1(0);
                z1 := sh_dsp1(1);
                z2 := sh_dsp1(2);
                z3 := sh_dsp1(3) + z5;
                z4 := sh_dsp1(4) + z5;
                o(0) := idescale(tmp0 + sh_dsp2(3) + z1 + z4, 15);
                o(7) := idescale(tmp0 - sh_dsp2(3) - z1 - z4, 15);
                o(1) := idescale(tmp2 + sh_dsp2(2) + z2 + z3, 15);
                o(6) := idescale(tmp2 - sh_dsp2(2) - z2 - z3, 15);
                o(2) := idescale(tmp3 + sh_dsp2(1) + z2 + z4, 15);
                o(5) := idescale(tmp3 - sh_dsp2(1) - z2 - z4, 15);
                o(3) := idescale(tmp1 + sh_dsp2(0) + z1 + z3, 15);
                o(4) := idescale(tmp1 - sh_dsp2(0) - z1 - z3, 15);
                out_row  <= o;
                m_tvalid <= '1';
                m_tlast  <= '1' when row_emit_idx = 7 else '0';

                if row_emit_idx = 7 then
                  row_load_idx <= 0;
                  row_emit_idx <= 0;
                  col_load_idx <= 0;
                  col_emit_idx <= 0;
                  state        <= LOAD;
                else
                  row_emit_idx <= row_emit_idx + 1;
                end if;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

  gen_out : for i in 0 to 7 generate
    m_tdata(i*16+15 downto i*16) <= std_logic_vector(resize(out_row(i), 16));
  end generate;

end architecture rtl;
