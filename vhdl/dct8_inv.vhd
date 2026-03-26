-- =============================================================================
-- dct8_inv.vhd  --  Pipelined 8×8 inverse integer DCT
--
-- DSP sharing strategy: all 12 multiplications live in a dedicated always-on
-- registered process (dsp_mul).  Each sh_dsp signal has exactly ONE driver,
-- so Vivado infers exactly ONE DSP48E2 per line.  The main FSM loads dsp_a
-- in ADD phases and reads sh_dsp results two cycles later in FINAL phases.
--
-- Interface (unchanged)
--   Input  : one row of 8 dequantised coefficients (256-bit)
--   Output : one residual row (8 × signed-16-bit = 128-bit)
--
-- Timing
--   LOAD     :  8 clocks
--   COL_PASS : 24 clocks  (3 phases × 8 columns)
--   ROW_PASS : 24 clocks  (3 phases × 8 rows)
--   Total    : 56 clocks per 8×8 block
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
  type state_t  is (LOAD, COL_PASS, ROW_PASS);
  type col_ph_t is (COL_ADD_PH, COL_MUL_PH, COL_FINAL_PH);
  type row_ph_t is (ROW_ADD_PH, ROW_MUL_PH, ROW_FINAL_PH);

  signal state     : state_t  := LOAD;
  signal col_phase : col_ph_t := COL_ADD_PH;
  signal row_phase : row_ph_t := ROW_ADD_PH;
  signal col_idx   : integer range 0 to 7 := 0;
  signal row_idx   : integer range 0 to 7 := 0;
  signal load_row  : integer range 0 to 7 := 0;

  signal in_buf  : block32_t;
  signal col_buf : block32_t;

  -- Even-part DC values (shifts only, no DSP): set in ADD_PH, used in MUL_PH
  signal col_d0pd4 : s32_t;
  signal col_d0md4 : s32_t;
  signal r_d04     : s32_t;
  signal r_d04d    : s32_t;

  -- Even-part DC values registered into MUL_PH for use in FINAL_PH
  signal sh_e_base : s32_t;
  signal sh_e_diff : s32_t;

  -- -------------------------------------------------------------------------
  -- Pre-mux DSP A inputs.
  --
  --   dsp_a(0)  → FIX_1_175875602   z34
  --   dsp_a(1)  → -FIX_0_899976223  d7+d1
  --   dsp_a(2)  → -FIX_2_562915447  d5+d3
  --   dsp_a(3)  → -FIX_1_961570560  d7+d3
  --   dsp_a(4)  → -FIX_0_390180644  d5+d1
  --   dsp_a(5)  → FIX_0_541196100   d2+d6
  --   dsp_a(6)  → -FIX_1_847759065  d6
  --   dsp_a(7)  → FIX_0_765366865   d2
  --   dsp_a(8)  → FIX_0_298631336   d7
  --   dsp_a(9)  → FIX_2_053119869   d5
  --   dsp_a(10) → FIX_3_072711026   d3
  --   dsp_a(11) → FIX_1_501321110   d1
  -- -------------------------------------------------------------------------
  type dsp_in_t is array(0 to 11) of s32_t;
  signal dsp_a : dsp_in_t;

  -- -------------------------------------------------------------------------
  -- DSP output registers — one driver each → one DSP48E2 each.
  -- -------------------------------------------------------------------------
  signal sh_dsp1 : int32_row_t;   -- indices 0..7 used
  signal sh_dsp2 : int32_row_t;   -- indices 0..3 used

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
        state     <= LOAD;
        load_row  <= 0;
        col_idx   <= 0;
        row_idx   <= 0;
        col_phase <= COL_ADD_PH;
        row_phase <= ROW_ADD_PH;
        m_tvalid  <= '0';
        m_tlast   <= '0';
      else
        m_tvalid <= '0';
        m_tlast  <= '0';

        case state is

          -- ------------------------------------------------------------------
          -- LOAD
          -- ------------------------------------------------------------------
          when LOAD =>
            if s_tvalid = '1' then
              in_buf(load_row) <= unpack256(s_tdata);
              if load_row = 7 then
                load_row  <= 0;
                col_idx   <= 0;
                col_phase <= COL_ADD_PH;
                state     <= COL_PASS;
              else
                load_row <= load_row + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- COL_PASS: 3 phases × 8 columns = 24 clocks
          --   COL_ADD_PH  : additions → col_d0pd4/md4 + dsp_a(0..11)
          --   COL_MUL_PH  : wait (dsp_mul fires) + register sh_e_base/sh_e_diff
          --   COL_FINAL_PH: assemble, idescale(17) → col_buf
          -- ------------------------------------------------------------------
          when COL_PASS =>

            case col_phase is

              when COL_ADD_PH =>
                col_d0pd4 <= shift_left(in_buf(0)(col_idx) + in_buf(4)(col_idx),
                                        IDCT_CONST_BITS);
                col_d0md4 <= shift_left(in_buf(0)(col_idx) - in_buf(4)(col_idx),
                                        IDCT_CONST_BITS);
                dsp_a(0)  <= (in_buf(7)(col_idx) + in_buf(3)(col_idx))
                           + (in_buf(5)(col_idx) + in_buf(1)(col_idx));
                dsp_a(1)  <= in_buf(7)(col_idx) + in_buf(1)(col_idx);
                dsp_a(2)  <= in_buf(5)(col_idx) + in_buf(3)(col_idx);
                dsp_a(3)  <= in_buf(7)(col_idx) + in_buf(3)(col_idx);
                dsp_a(4)  <= in_buf(5)(col_idx) + in_buf(1)(col_idx);
                dsp_a(5)  <= in_buf(2)(col_idx) + in_buf(6)(col_idx);
                dsp_a(6)  <= in_buf(6)(col_idx);
                dsp_a(7)  <= in_buf(2)(col_idx);
                dsp_a(8)  <= in_buf(7)(col_idx);
                dsp_a(9)  <= in_buf(5)(col_idx);
                dsp_a(10) <= in_buf(3)(col_idx);
                dsp_a(11) <= in_buf(1)(col_idx);
                col_phase <= COL_MUL_PH;

              when COL_MUL_PH =>
                -- dsp_mul fires this cycle; register even-part DC shifts
                sh_e_base <= col_d0pd4;
                sh_e_diff <= col_d0md4;
                col_phase <= COL_FINAL_PH;

              when COL_FINAL_PH =>
                tmp2 := sh_dsp1(5) + sh_dsp1(7);
                tmp3 := sh_dsp1(5) + sh_dsp1(6);
                tmp0 := sh_e_base + tmp2;
                tmp1 := sh_e_base - tmp2;
                tmp2 := sh_e_diff + tmp3;
                tmp3 := sh_e_diff - tmp3;
                z5 := sh_dsp1(0);
                z1 := sh_dsp1(1);
                z2 := sh_dsp1(2);
                z3 := sh_dsp1(3) + z5;
                z4 := sh_dsp1(4) + z5;
                col_buf(0)(col_idx) <= idescale(tmp0 + sh_dsp2(3) + z1 + z4, 17);
                col_buf(7)(col_idx) <= idescale(tmp0 - sh_dsp2(3) - z1 - z4, 17);
                col_buf(1)(col_idx) <= idescale(tmp2 + sh_dsp2(2) + z2 + z3, 17);
                col_buf(6)(col_idx) <= idescale(tmp2 - sh_dsp2(2) - z2 - z3, 17);
                col_buf(2)(col_idx) <= idescale(tmp3 + sh_dsp2(1) + z2 + z4, 17);
                col_buf(5)(col_idx) <= idescale(tmp3 - sh_dsp2(1) - z2 - z4, 17);
                col_buf(3)(col_idx) <= idescale(tmp1 + sh_dsp2(0) + z1 + z3, 17);
                col_buf(4)(col_idx) <= idescale(tmp1 - sh_dsp2(0) - z1 - z3, 17);

                if col_idx = 7 then
                  col_idx   <= 0;
                  col_phase <= COL_ADD_PH;
                  row_idx   <= 0;
                  row_phase <= ROW_ADD_PH;
                  state     <= ROW_PASS;
                else
                  col_idx   <= col_idx + 1;
                  col_phase <= COL_ADD_PH;
                end if;

            end case;

          -- ------------------------------------------------------------------
          -- ROW_PASS: 3 phases × 8 rows = 24 clocks
          --   ROW_ADD_PH  : additions → r_d04/r_d04d + dsp_a(0..11)
          --   ROW_MUL_PH  : wait (dsp_mul fires) + register sh_e_base/sh_e_diff
          --   ROW_FINAL_PH: assemble, idescale(15), emit if m_tready
          -- ------------------------------------------------------------------
          when ROW_PASS =>

            case row_phase is

              when ROW_ADD_PH =>
                r_d04  <= col_buf(row_idx)(0) + col_buf(row_idx)(4);
                r_d04d <= col_buf(row_idx)(0) - col_buf(row_idx)(4);
                dsp_a(0)  <= (col_buf(row_idx)(7) + col_buf(row_idx)(3))
                           + (col_buf(row_idx)(5) + col_buf(row_idx)(1));
                dsp_a(1)  <= col_buf(row_idx)(7) + col_buf(row_idx)(1);
                dsp_a(2)  <= col_buf(row_idx)(5) + col_buf(row_idx)(3);
                dsp_a(3)  <= col_buf(row_idx)(7) + col_buf(row_idx)(3);
                dsp_a(4)  <= col_buf(row_idx)(5) + col_buf(row_idx)(1);
                dsp_a(5)  <= col_buf(row_idx)(2) + col_buf(row_idx)(6);
                dsp_a(6)  <= col_buf(row_idx)(6);
                dsp_a(7)  <= col_buf(row_idx)(2);
                dsp_a(8)  <= col_buf(row_idx)(7);
                dsp_a(9)  <= col_buf(row_idx)(5);
                dsp_a(10) <= col_buf(row_idx)(3);
                dsp_a(11) <= col_buf(row_idx)(1);
                row_phase <= ROW_MUL_PH;

              when ROW_MUL_PH =>
                -- dsp_mul fires this cycle; register even-part DC shifts
                sh_e_base <= shift_left(r_d04,  IDCT_CONST_BITS);
                sh_e_diff <= shift_left(r_d04d, IDCT_CONST_BITS);
                row_phase <= ROW_FINAL_PH;

              when ROW_FINAL_PH =>
                if m_tready = '1' then
                  tmp2 := sh_dsp1(5) + sh_dsp1(7);
                  tmp3 := sh_dsp1(5) + sh_dsp1(6);
                  tmp0 := sh_e_base + tmp2;
                  tmp1 := sh_e_base - tmp2;
                  tmp2 := sh_e_diff + tmp3;
                  tmp3 := sh_e_diff - tmp3;
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

                  if row_idx = 7 then
                    m_tlast   <= '1';
                    row_idx   <= 0;
                    col_idx   <= 0;
                    col_phase <= COL_ADD_PH;
                    row_phase <= ROW_ADD_PH;
                    state     <= LOAD;
                  else
                    m_tlast   <= '0';
                    row_idx   <= row_idx + 1;
                    row_phase <= ROW_ADD_PH;
                  end if;
                end if;

            end case;

        end case;
      end if;
    end if;
  end process;

  gen_out : for i in 0 to 7 generate
    m_tdata(i*16+15 downto i*16) <= std_logic_vector(resize(out_row(i), 16));
  end generate;

end architecture rtl;
