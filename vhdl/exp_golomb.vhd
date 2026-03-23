-- =============================================================================
-- exp_golomb.vhd  --  Exponential-Golomb entropy coder
--
-- Translates bitstream.c (bs_write_se / bs_write_ue) to synthesisable VHDL.
--
-- For each incoming signed coefficient this unit produces:
--   (codeword : std_logic_vector, length : integer)
-- The (codeword, length) pair is fed to bs_packer.vhd which shifts it into
-- the byte-aligned output stream.
--
-- Signed→Unsigned mapping (matches bitstream.c):
--   0 → 0,  1 → 1,  -1 → 2,  2 → 3,  -2 → 4 …
--   ue(v) where v = 2*|x| - 1 for x>0, 2*|x| for x≤0
--
-- Unsigned Exp-Golomb for value v:
--   M = floor(log2(v+1))          — number of prefix zeros
--   codeword = <M zeros> 1 <M LSBs of (v+1)>
--   total bits = 2*M + 1
--
-- Maximum coefficient value ±32767 → v ≤ 65534 → M ≤ 15 → max bits = 31
-- We output up to 32 bits per coefficient.
--
-- Resource estimate
-- -----------------
--   LUT  : ~100 (priority encoder + shift logic)
--   FF   : ~40  (registered output)
--   DSP  : 0
--   Latency : 1 clock cycle
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity exp_golomb is
  port (
    aclk      : in  std_logic;
    aresetn   : in  std_logic;

    -- Input: one token per clock
    s_tdata   : in  std_logic_vector(15 downto 0);  -- signed 16-bit
    s_tvalid  : in  std_logic;
    s_tready  : out std_logic;  -- always '1'
    s_tmode   : in  std_logic;  -- '0' = SE coefficient, '1' = UE count prefix

    -- Output: codeword + valid bit count (registered, 1-cycle latency)
    m_codeword : out std_logic_vector(31 downto 0);
    m_length   : out unsigned(5 downto 0);  -- 1..31 bits
    m_tvalid   : out std_logic
  );
end entity exp_golomb;

architecture rtl of exp_golomb is

  -- Count leading zeros of a 17-bit unsigned → position of MSB (0..16)
  function count_leading_zeros(v : unsigned(16 downto 0)) return integer is
  begin
    for i in 16 downto 0 loop
      if v(i) = '1' then
        return 16 - i;  -- number of leading zeros
      end if;
    end loop;
    return 17;  -- v = 0
  end function;

begin

  s_tready <= '1';

  process(aclk)
    variable coeff   : signed(15 downto 0);
    variable ue_val  : unsigned(16 downto 0);  -- unsigned-mapped value (0..65534)
    variable v1      : unsigned(16 downto 0);  -- v+1
    variable m       : integer range 0 to 16;  -- floor(log2(v+1))
    variable cw      : std_logic_vector(31 downto 0);
    variable len     : integer range 1 to 31;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        m_codeword <= (others => '0');
        m_length   <= (others => '0');
        m_tvalid   <= '0';
      else
        m_tvalid <= s_tvalid;
        if s_tvalid = '1' then
          coeff := signed(s_tdata);

          if s_tmode = '1' then
            -- UE mode: s_tdata is the unsigned count value (0..64)
            ue_val := resize(unsigned(s_tdata), 17);
          elsif coeff > 0 then
            -- SE positive: map to odd ue index
            ue_val := to_unsigned(2 * to_integer(coeff) - 1, 17);
          else
            -- SE zero/negative: map to even ue index
            ue_val := to_unsigned(2 * abs(to_integer(coeff)), 17);
          end if;

          v1  := ue_val + 1;                     -- v+1
          m   := 16 - count_leading_zeros(v1);   -- MSB position = floor(log2(v1))

          -- Build codeword: <m zeros> 1 <m LSBs of v1>  (left-aligned in 32-bit)
          len := 2 * m + 1;
          cw  := (others => '0');
          -- Place '1' at position (31 - m)
          cw(31 - m) := '1';
          -- Place m LSBs of v1 below the '1'
          for i in 0 to 15 loop
            if i < m then
              cw(31 - m - 1 - i) := v1(m - 1 - i);
            end if;
          end loop;

          m_codeword <= cw;
          m_length   <= to_unsigned(len, 6);
        end if;
      end if;
    end if;
  end process;

end architecture rtl;
