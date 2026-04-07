-- =============================================================================
-- quant_enc.vhd  --  Forward quantiser, 8 coefficients per clock (one DCT row)
--
-- Accepts one full 8-coefficient DCT row per clock (256-bit) and quantises
-- all 8 values in parallel, emitting a 128-bit result on the next clock.
-- Replaces the original single-coefficient version to remove the pre-quant
-- DCT row serialiser bottleneck.
--
-- Operation (per coefficient)
-- ---------------------------
--   output = sign(in) * max(0, floor(|in| * recip >> 16))  if |in| >= dead_zone
--   output = 0                                               otherwise
--
--   recip = floor(2^16 / step)  — from precomputed ROM, no runtime division
--
-- Interface
-- ---------
--   Input  : 8 DCT coefficients per clock (256-bit, 8 × signed 32-bit)
--            bits [31:0]=coeff0, [63:32]=coeff1, ..., [255:224]=coeff7
--   Output : 8 quantised coefficients per clock (128-bit, 8 × signed 16-bit)
--            bits [15:0]=qcoeff0, [31:16]=qcoeff1, ..., [127:112]=qcoeff7
--   Latency: 1 clock cycle
--
-- Resource estimate
-- -----------------
--   DSP58E2 : 8  (one 32×16 multiply per coefficient, inferred in parallel)
--   LUT     : ~100 (dead-zone compares + sign muxes × 8)
--   Latency : 1 clock cycle
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity quant_enc is
  port (
    aclk      : in  std_logic;
    aresetn   : in  std_logic;

    -- QP configuration — update between frames (latency 1 cycle)
    qp        : in  unsigned(5 downto 0);   -- 1..51
    is_intra  : in  std_logic;              -- '1' = I-frame (smaller dead-zone)

    -- Input: one full DCT row (8 coefficients) per clock
    s_tdata   : in  std_logic_vector(255 downto 0);  -- 8 × signed 32-bit
    s_tvalid  : in  std_logic;
    s_tready  : out std_logic;  -- always '1'

    -- Output: 8 quantised coefficients, 1-cycle delayed
    m_tdata   : out std_logic_vector(127 downto 0);  -- 8 × signed 16-bit
    m_tvalid  : out std_logic
  );
end entity quant_enc;

architecture rtl of quant_enc is

  -- -------------------------------------------------------------------------
  -- QP step ROM: 51 entries, step = BASE[qp%6] << (qp/6)
  -- BASE = {10,11,13,14,16,18}
  -- -------------------------------------------------------------------------
  type step_rom_t is array(1 to 51) of integer range 0 to 65535;
  type int6_arr_t is array(0 to 5) of integer;

  function build_step_rom return step_rom_t is
    constant BASE : int6_arr_t := (10, 11, 13, 14, 16, 18);
    variable rom  : step_rom_t;
  begin
    for q in 1 to 51 loop
      rom(q) := BASE((q-1) mod 6) * (2 ** ((q-1)/6));
    end loop;
    return rom;
  end function;

  -- Precomputed reciprocals: floor(2^16 / step) — eliminates runtime division
  function build_recip_rom return step_rom_t is
    constant BASE : int6_arr_t := (10, 11, 13, 14, 16, 18);
    variable rom  : step_rom_t;
    variable step : integer;
  begin
    for q in 1 to 51 loop
      step   := BASE((q-1) mod 6) * (2 ** ((q-1)/6));
      rom(q) := 65536 / step;
    end loop;
    return rom;
  end function;

  constant STEP_ROM  : step_rom_t := build_step_rom;
  constant RECIP_ROM : step_rom_t := build_recip_rom;

  signal step_r    : integer range 1 to 65535 := 10;
  signal recip_r   : unsigned(15 downto 0)    := x"1999";  -- 2^16 / 10
  signal dz_intra  : integer range 0 to 32767 := 4;        -- step*3/8
  signal dz_inter  : integer range 0 to 32767 := 5;        -- step/2

  signal qp_prev   : unsigned(5 downto 0) := (others => '0');

begin

  s_tready <= '1';  -- always ready, no backpressure

  -- -------------------------------------------------------------------------
  -- Register process: update QP params and quantise 8 coefficients in parallel
  -- -------------------------------------------------------------------------
  process(aclk)
    variable coeff   : signed(31 downto 0);
    variable av      : unsigned(31 downto 0);
    variable product : unsigned(47 downto 0);
    variable q_val   : integer;
    variable out_val : signed(15 downto 0);
    variable qp_int  : integer range 1 to 51;
    variable dz_sel  : integer;
    variable s       : integer range 0 to 65535;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        m_tdata  <= (others => '0');
        m_tvalid <= '0';
        step_r   <= 10;
        recip_r  <= x"1999";
        dz_intra <= 4;
        dz_inter <= 5;
        qp_prev  <= (others => '0');
      else
        -- Update step/recip/dead-zone registers when QP changes
        if qp /= qp_prev then
          qp_int   := to_integer(qp);
          if qp_int < 1  then qp_int := 1;  end if;
          if qp_int > 51 then qp_int := 51; end if;
          s        := STEP_ROM(qp_int);
          step_r   <= s;
          recip_r  <= to_unsigned(RECIP_ROM(qp_int), 16);
          dz_intra <= (s * 3) / 8;
          dz_inter <= s / 2;
          qp_prev  <= qp;
        end if;

        m_tvalid <= s_tvalid;

        if s_tvalid = '1' then
          if is_intra = '1' then
            dz_sel := dz_intra;
          else
            dz_sel := dz_inter;
          end if;

          -- Quantise all 8 coefficients in parallel (synthesises as 8 DSPs)
          for i in 0 to 7 loop
            coeff   := signed(s_tdata(i*32+31 downto i*32));
            av      := unsigned(abs(coeff));
            product := av * recip_r;            -- 32×16 → 48-bit
            q_val   := to_integer(product(47 downto 16));  -- >> 16

            if to_integer(av) < dz_sel or q_val = 0 then
              out_val := (others => '0');
            elsif coeff >= 0 then
              out_val := to_signed(q_val, 16);
            else
              out_val := to_signed(-q_val, 16);
            end if;

            m_tdata(i*16+15 downto i*16) <= std_logic_vector(out_val);
          end loop;
        end if;

      end if;
    end if;
  end process;

end architecture rtl;
