-- =============================================================================
-- zigzag.vhd  --  Zigzag scan reorder buffer with trailing-zero suppression
--
-- Accepts one 8-coefficient row (128-bit) per clock; fills the 64-entry buffer
-- in 8 clocks instead of 64.  EMIT phase is unchanged: one coefficient per
-- clock in zigzag order, gated by m_tready.
--
-- Trailing-zero suppression
-- -------------------------
-- 1. FILL (8 clocks): accept rows; find last non-zero zigzag position (last_nz).
-- 2. COUNT (1 clock):  emit ue(last_nz+1) as count prefix (UE mode).
-- 3. EMIT (0..64 clocks): coefficients 0..last_nz in zigzag order (SE mode).
-- An all-zero block emits a single ue(0) token.
--
-- Interface
-- ---------
--   Input  : 8 quantised coefficients per clock (128-bit, natural row order)
--            bits [15:0]=coeff0, [31:16]=coeff1, ..., [127:112]=coeff7
--   Output : count prefix (UE, m_tmode='1') then 0..64 SE values (m_tmode='0')
--            m_tlast='1' on the final token of each block
--
-- Resource estimate
-- -----------------
--   LUTRAM : ~128 × 16-bit = 2 KB  (distributed RAM, no BRAM)
--   Latency: 8 clocks (fill) + 1 (count) + 0..64 (emit) per block
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity zigzag is
  port (
    aclk     : in  std_logic;
    aresetn  : in  std_logic;

    -- Input: 8 coefficients per clock (one DCT row, natural/row-major order)
    s_tdata  : in  std_logic_vector(127 downto 0);  -- 8 × signed 16-bit
    s_tvalid : in  std_logic;
    s_tready : out std_logic;

    -- Output: count prefix then zigzag-ordered coefficients
    m_tdata  : out std_logic_vector(15 downto 0);
    m_tvalid : out std_logic;
    m_tlast  : out std_logic;  -- '1' on last token of block
    m_tmode  : out std_logic;  -- '1' = UE count prefix, '0' = SE coefficient
    m_tready : in  std_logic
  );
end entity zigzag;

architecture rtl of zigzag is

  -- Zigzag position → natural index
  type zigzag_lut_t is array(0 to 63) of integer range 0 to 63;
  constant ZIGZAG_LUT : zigzag_lut_t := (
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
  );

  -- Inverse zigzag: natural position → zigzag index (for last_nz tracking)
  constant INV_ZIGZAG_LUT : zigzag_lut_t := (
     0,  1,  5,  6, 14, 15, 27, 28,
     2,  4,  7, 13, 16, 26, 29, 42,
     3,  8, 12, 17, 25, 30, 41, 43,
     9, 11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63
  );

  -- Coefficient buffer (64 × 16-bit, synthesises to LUTRAM)
  type coeff_buf_t is array(0 to 63) of signed(15 downto 0);
  signal buf : coeff_buf_t := (others => (others => '0'));

  -- FSM
  type state_t is (FILL, COUNT, EMIT);
  signal state   : state_t        := FILL;
  signal wr_ptr  : integer range 0 to 7  := 0;  -- row index (0..7)
  signal rd_ptr  : integer range 0 to 63 := 0;
  -- last non-zero zigzag index; -1 means all-zero block
  signal last_nz : integer range -1 to 63 := -1;

begin

  process(aclk)
    variable max_nz : integer range -1 to 63;
    variable coeff  : signed(15 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state    <= FILL;
        wr_ptr   <= 0;
        rd_ptr   <= 0;
        last_nz  <= -1;
        m_tvalid <= '0';
        m_tlast  <= '0';
        m_tmode  <= '0';
        s_tready <= '1';
      else
        m_tvalid <= '0';
        m_tlast  <= '0';
        m_tmode  <= '0';

        case state is

          -- ----------------------------------------------------------------
          -- FILL: accept one 128-bit row (8 coefficients) per clock.
          -- Stores all 8 into buf and tracks the last non-zero zigzag index.
          -- ----------------------------------------------------------------
          when FILL =>
            s_tready <= '1';
            if s_tvalid = '1' then
              -- Carry forward current last_nz as starting point for this row
              max_nz := last_nz;

              for i in 0 to 7 loop
                coeff := signed(s_tdata(i*16+15 downto i*16));
                buf(wr_ptr * 8 + i) <= coeff;

                -- Update max zigzag index among non-zero coefficients
                if coeff /= 0 then
                  if INV_ZIGZAG_LUT(wr_ptr * 8 + i) > max_nz then
                    max_nz := INV_ZIGZAG_LUT(wr_ptr * 8 + i);
                  end if;
                end if;
              end loop;

              last_nz <= max_nz;

              if wr_ptr = 7 then
                wr_ptr   <= 0;
                s_tready <= '0';
                state    <= COUNT;
              else
                wr_ptr <= wr_ptr + 1;
              end if;
            end if;

          -- ----------------------------------------------------------------
          -- COUNT: emit ue(last_nz + 1) as UE prefix
          -- ----------------------------------------------------------------
          when COUNT =>
            m_tdata  <= std_logic_vector(to_signed(last_nz + 1, 16));
            m_tvalid <= '1';
            m_tmode  <= '1';   -- UE mode

            if last_nz < 0 then
              -- All-zero block: single ue(0) token
              m_tlast  <= '1';
              last_nz  <= -1;
              s_tready <= '1';
              state    <= FILL;
            else
              m_tlast  <= '0';
              rd_ptr   <= 0;
              state    <= EMIT;
            end if;

          -- ----------------------------------------------------------------
          -- EMIT: output coefficients 0..last_nz in zigzag order
          -- ----------------------------------------------------------------
          when EMIT =>
            s_tready <= '0';
            if m_tready = '1' then
              m_tdata  <= std_logic_vector(buf(ZIGZAG_LUT(rd_ptr)));
              m_tvalid <= '1';
              m_tmode  <= '0';   -- SE mode
              m_tlast  <= '1' when rd_ptr = last_nz else '0';

              if rd_ptr = last_nz then
                rd_ptr   <= 0;
                last_nz  <= -1;
                s_tready <= '1';
                state    <= FILL;
              else
                rd_ptr <= rd_ptr + 1;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
