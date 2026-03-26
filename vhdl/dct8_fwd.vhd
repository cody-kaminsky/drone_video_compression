-- =============================================================================
-- dct8_fwd.vhd  --  Pipelined 8×8 forward integer DCT
--
-- DSP sharing strategy: all 12 multiplications live in a dedicated always-on
-- registered process (dsp_mul).  Each sh_dsp signal has exactly ONE driver in
-- the entire design, so Vivado infers exactly ONE DSP48E2 per line regardless
-- of synthesis settings.  The main FSM only loads the pre-mux inputs (dsp_a)
-- and uses the registered outputs one cycle later.
--
-- Interface (unchanged)
--   Input  : one residual row per clock (8 × signed-9-bit packed as 72-bit)
--   Output : one DCT coefficient row per clock (8 × signed-32-bit = 256-bit)
--
-- Timing
--   FILL phase : 32 clocks  (4 stages × 8 rows)
--   COL_PASS   : 24 clocks  (3 phases × 8 columns, serial)
--   EMIT phase :  8 clocks
--   Total      : 64 clocks per 8×8 block
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
  type state_t  is (FILL, COL_PASS, EMIT);
  type col_ph_t is (COL_ADD_PH, COL_MUL_PH, COL_FINAL_PH);

  signal state     : state_t  := FILL;
  signal col_phase : col_ph_t := COL_ADD_PH;
  signal col_idx   : integer range 0 to 7 := 0;
  signal in_row    : integer range 0 to 7 := 0;
  signal out_row   : integer range 0 to 7 := 0;

  signal fill_stage  : integer range 0 to 3 := 0;
  signal row_in_r    : std_logic_vector(71 downto 0) := (others => '0');
  signal row_in_idx  : integer range 0 to 7 := 0;

  -- Passthroughs (r0 and r4): set in stage 1 / COL_ADD_PH, read two cycles later
  signal pass_r0 : s32_t;
  signal pass_r4 : s32_t;
  signal col_r0  : s32_t;
  signal col_r4  : s32_t;

  -- Intermediate row-pass result buffer
  signal row_buf : block32_t;

  -- -------------------------------------------------------------------------
  -- Pre-mux DSP A inputs (driven by FSM, read by dsp_mul process).
  --
  --   dsp_a(0)  → FIX_1_175875602   z34
  --   dsp_a(1)  → -FIX_0_899976223  z_add(0)
  --   dsp_a(2)  → -FIX_2_562915447  z_add(1)
  --   dsp_a(3)  → -FIX_1_961570560  z_add(2)
  --   dsp_a(4)  → -FIX_0_390180644  z_add(3)
  --   dsp_a(5)  → FIX_0_541196100   z12
  --   dsp_a(6)  → FIX_0_298631336   tmp4
  --   dsp_a(7)  → FIX_2_053119869   tmp5
  --   dsp_a(8)  → FIX_3_072711026   tmp6
  --   dsp_a(9)  → FIX_1_501321110   tmp7
  --   dsp_a(10) → FIX_0_765366865   tmp13
  --   dsp_a(11) → -FIX_1_847759065  tmp12
  -- -------------------------------------------------------------------------
  type dsp_in_t is array(0 to 11) of s32_t;
  signal dsp_a : dsp_in_t;

  -- -------------------------------------------------------------------------
  -- DSP output registers.
  -- Each signal is driven by EXACTLY ONE assignment (in dsp_mul below).
  -- Vivado infers one DSP48E2 per line, unconditionally.
  -- -------------------------------------------------------------------------
  signal sh_dsp1 : int32_row_t;   -- indices 0..5 used
  signal sh_dsp2 : int32_row_t;   -- indices 0..5 used

  signal out_buf : block32_t;
  signal cur_out : int32_row_t;

begin

  -- =========================================================================
  -- Dedicated multiply process — 12 DSP48E2 blocks, one per line.
  -- No conditions, no case statements: each sh_dsp signal has exactly one
  -- driver in the entire design.  Vivado cannot split this into two DSPs.
  -- Outputs are available one cycle after dsp_a is loaded.
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
        state      <= FILL;
        fill_stage <= 0;
        in_row     <= 0;
        out_row    <= 0;
        col_phase  <= COL_ADD_PH;
        col_idx    <= 0;
        m_tvalid   <= '0';
        m_tlast    <= '0';
        s_tready   <= '1';
      else
        m_tvalid <= '0';
        m_tlast  <= '0';

        case state is

          -- ------------------------------------------------------------------
          -- FILL: 4 stages × 8 rows = 32 clocks
          --   Stage 0 : accept s_tdata
          --   Stage 1 : butterfly → dsp_a(0..11) + pass_r0/pass_r4
          --   Stage 2 : wait (dsp_mul fires, producing sh_dsp1/sh_dsp2)
          --   Stage 3 : final additions + descale → row_buf
          -- ------------------------------------------------------------------
          when FILL =>

            if fill_stage = 0 then
              s_tready <= '1';
              if s_tvalid = '1' then
                row_in_r   <= s_tdata;
                row_in_idx <= in_row;
                fill_stage <= 1;
                s_tready   <= '0';
              end if;

            elsif fill_stage = 1 then
              d     := unpack_row(row_in_r);
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
              fill_stage <= 2;

            elsif fill_stage = 2 then
              -- dsp_mul fires this cycle, sh_dsp1/sh_dsp2 ready next cycle
              fill_stage <= 3;

            else  -- fill_stage = 3
              r_out(0) := pass_r0;
              r_out(4) := pass_r4;
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
              row_buf(row_in_idx) <= r_out;
              fill_stage <= 0;

              if row_in_idx = 7 then
                in_row     <= 0;
                col_idx    <= 0;
                col_phase  <= COL_ADD_PH;
                s_tready   <= '0';
                state      <= COL_PASS;
              else
                in_row   <= row_in_idx + 1;
                s_tready <= '1';
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- COL_PASS: 3 phases × 8 columns = 24 clocks
          --   COL_ADD_PH  : butterfly → dsp_a(0..11) + col_r0/col_r4
          --   COL_MUL_PH  : wait (dsp_mul fires)
          --   COL_FINAL_PH: final additions + descale → out_buf(:)(col_idx)
          -- ------------------------------------------------------------------
          when COL_PASS =>
            s_tready <= '0';

            case col_phase is

              when COL_ADD_PH =>
                tmp0  := row_buf(0)(col_idx) + row_buf(7)(col_idx);
                tmp7  := row_buf(0)(col_idx) - row_buf(7)(col_idx);
                tmp1  := row_buf(1)(col_idx) + row_buf(6)(col_idx);
                tmp6  := row_buf(1)(col_idx) - row_buf(6)(col_idx);
                tmp2  := row_buf(2)(col_idx) + row_buf(5)(col_idx);
                tmp5  := row_buf(2)(col_idx) - row_buf(5)(col_idx);
                tmp3  := row_buf(3)(col_idx) + row_buf(4)(col_idx);
                tmp4  := row_buf(3)(col_idx) - row_buf(4)(col_idx);
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
                col_phase <= COL_MUL_PH;

              when COL_MUL_PH =>
                -- dsp_mul fires this cycle, sh_dsp1/sh_dsp2 ready next cycle
                col_phase <= COL_FINAL_PH;

              when COL_FINAL_PH =>
                z1 := sh_dsp1(1);
                z2 := sh_dsp1(2);
                z3 := sh_dsp1(3) + sh_dsp1(0);
                z4 := sh_dsp1(4) + sh_dsp1(0);
                out_buf(0)(col_idx) <= col_r0;
                out_buf(4)(col_idx) <= col_r4;
                out_buf(2)(col_idx) <= descale(sh_dsp1(5) + sh_dsp2(4), CONST_BITS + PASS1_BITS);
                out_buf(6)(col_idx) <= descale(sh_dsp1(5) + sh_dsp2(5), CONST_BITS + PASS1_BITS);
                out_buf(7)(col_idx) <= descale(sh_dsp2(0) + z1 + z3, CONST_BITS + PASS1_BITS);
                out_buf(5)(col_idx) <= descale(sh_dsp2(1) + z2 + z4, CONST_BITS + PASS1_BITS);
                out_buf(3)(col_idx) <= descale(sh_dsp2(2) + z2 + z3, CONST_BITS + PASS1_BITS);
                out_buf(1)(col_idx) <= descale(sh_dsp2(3) + z1 + z4, CONST_BITS + PASS1_BITS);

                if col_idx = 7 then
                  col_idx   <= 0;
                  col_phase <= COL_ADD_PH;
                  out_row   <= 0;
                  state     <= EMIT;
                else
                  col_idx   <= col_idx + 1;
                  col_phase <= COL_ADD_PH;
                end if;

            end case;

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
                out_row  <= 0;
                s_tready <= '1';
                state    <= FILL;
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
