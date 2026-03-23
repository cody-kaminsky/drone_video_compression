-- =============================================================================
-- bs_packer.vhd  --  Variable-length bitstream packer → byte stream
--
-- Accepts (codeword, length) pairs from exp_golomb.vhd and shifts them into
-- a 128-bit accumulator.  Once 8 or more bits are accumulated, a byte is
-- emitted on the AXI-Stream output port.
--
-- Backpressure
-- ------------
--   cw_ready is de-asserted when fill > 32 OR m_tready = '0'.
--   The upstream zigzag should gate on cw_ready to avoid fill overflow.
--   With the 2-cycle zigzag→exp_golomb→bs_packer pipeline delay, at most one
--   extra codeword can arrive after cw_ready goes low; the 128-bit accumulator
--   provides enough headroom (worst-case fill ≤ 94 bits).
--
-- Drain
-- -----
--   One byte is emitted per clock whenever fill ≥ 8 and m_tready = '1',
--   regardless of whether a new codeword is also being inserted.
--   During flush the partial last byte (fill < 8) is zero-padded and emitted
--   with m_tlast = '1'.
--
-- Interface
-- ---------
--   cw_data   : 32-bit codeword (left-aligned)
--   cw_len    : number of valid bits in cw_data (1..32)
--   cw_valid  : strobe — one codeword per clock
--   cw_ready  : backpressure output — de-assert when accumulator is getting full
--   flush     : '1' for one clock at end of frame → emit remaining bits (zero-padded)
--   m_axis_*  : AXI-Stream byte output
--
-- Resource estimate
-- -----------------
--   FF  : ~160 (128-bit accumulator + fill count + state)
--   LUT : ~200 (shift + mux logic)
--   DSP : 0
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bs_packer is
  port (
    aclk      : in  std_logic;
    aresetn   : in  std_logic;

    -- Variable-length codeword input (from exp_golomb or direct)
    cw_data   : in  std_logic_vector(31 downto 0);  -- left-aligned codeword
    cw_len    : in  unsigned(5 downto 0);            -- valid bits (1..32)
    cw_valid  : in  std_logic;
    cw_ready  : out std_logic;  -- backpressure: '0' when accumulator nearly full

    -- End-of-frame flush: emit remaining bits zero-padded to byte boundary
    flush     : in  std_logic;

    -- AXI-Stream byte output
    m_tdata   : out std_logic_vector(7 downto 0);
    m_tvalid  : out std_logic;
    m_tlast   : out std_logic;  -- set with flush byte
    m_tready  : in  std_logic
  );
end entity bs_packer;

architecture rtl of bs_packer is

  -- 128-bit accumulator + fill level.
  -- Valid bits are left-aligned: acc[127 .. 128-fill] hold data, rest are 0.
  signal acc       : std_logic_vector(127 downto 0) := (others => '0');
  signal fill      : integer range 0 to 191 := 0;

  -- flushing: set by a flush pulse, cleared when fill reaches 0.
  signal flushing  : std_logic := '0';

  signal out_byte  : std_logic_vector(7 downto 0);
  signal out_valid : std_logic := '0';
  signal out_last  : std_logic := '0';

begin

  m_tdata  <= out_byte;
  m_tvalid <= out_valid;
  m_tlast  <= out_last;

  -- Backpressure: de-assert when fill is too high or downstream is stalled.
  -- Threshold 32: worst-case in-flight codeword (31 bits) after de-assert
  -- pushes fill to at most 94, well within the 128-bit accumulator.
  cw_ready <= '1' when fill <= 32 and m_tready = '1' else '0';

  process(aclk)
    -- new_acc = acc (128 bits) || 32 zero bits; codeword inserted into gap.
    variable new_acc  : std_logic_vector(159 downto 0);
    variable new_fill : integer range 0 to 191;
    variable shift    : integer range 0 to 63;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        acc       <= (others => '0');
        fill      <= 0;
        flushing  <= '0';
        out_valid <= '0';
        out_last  <= '0';
      else
        out_valid <= '0';
        out_last  <= '0';

        -- Latch flush request
        if flush = '1' and fill > 0 then
          flushing <= '1';
        end if;

        -- -------------------------------------------------------------------
        -- Insert new codeword (backpressure via cw_ready prevents overflow;
        -- one in-flight token may still arrive after cw_ready de-asserts)
        -- -------------------------------------------------------------------
        if cw_valid = '1' then
          shift    := to_integer(cw_len);
          -- Extend accumulator with 32 zero bits on the right
          new_acc  := acc & x"00000000";
          -- Place codeword left-aligned immediately after the current fill
          for i in 0 to 31 loop
            if i < shift then
              new_acc(159 - fill - i) := cw_data(31 - i);
            end if;
          end loop;
          new_fill := fill + shift;

          if new_fill >= 8 and m_tready = '1' then
            out_byte  <= new_acc(159 downto 152);
            acc       <= new_acc(151 downto 24);
            fill      <= new_fill - 8;
            out_valid <= '1';
          else
            -- No drain this cycle (either not enough bits or downstream busy).
            -- fill may temporarily exceed 32; cw_ready already guards upstream.
            acc  <= new_acc(159 downto 32);
            fill <= new_fill;
          end if;

        -- -------------------------------------------------------------------
        -- Drain: emit a byte whenever fill ≥ 8 and downstream is ready.
        -- Flushing path also handles the final partial byte.
        -- -------------------------------------------------------------------
        elsif flushing = '1' and fill > 0 and m_tready = '1' then
          out_byte  <= acc(127 downto 120);
          acc       <= acc(119 downto 0) & x"00";
          out_valid <= '1';
          if fill <= 8 then
            fill     <= 0;
            flushing <= '0';
            out_last <= '1';
          else
            fill <= fill - 8;
          end if;

        elsif fill >= 8 and m_tready = '1' then
          -- Normal mid-frame drain (complete bytes only)
          out_byte  <= acc(127 downto 120);
          acc       <= acc(119 downto 0) & x"00";
          fill      <= fill - 8;
          out_valid <= '1';

        end if;

      end if;
    end if;
  end process;

end architecture rtl;
