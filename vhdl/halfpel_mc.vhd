-- =============================================================================
-- halfpel_mc.vhd  --  Half-pixel bilinear motion compensation
--
-- Reads the reference search-window BRAM and produces one 8x8 block of
-- motion-compensated prediction pixels.  Bilinear interpolation matches
-- inter_compensate / ref_halfpel in predict.c.
--
-- Half-pixel representation
-- -------------------------
--   mv.dx, mv.dy are in half-pixel units (range -64..+63).
--   Integer MV = (mv.dx >> 1, mv.dy >> 1).
--   Sub-pixel flags: xf = mv.dx & 1, yf = mv.dy & 1.
--   Four cases:
--     xf=0, yf=0: integer-pel  → ref[y][x]
--     xf=1, yf=0: H half-pel  → (ref[y][x] + ref[y][x+1] + 1) >> 1
--     xf=0, yf=1: V half-pel  → (ref[y][x] + ref[y+1][x] + 1) >> 1
--     xf=1, yf=1: HV quarter  → (ref[y][x] + ref[y][x+1] + ref[y+1][x] + ref[y+1][x+1] + 2) >> 2
--
-- BRAM access
-- -----------
--   Reads from the search-window BRAM via rd_addr / rd_data (1-cycle latency).
--   For H half-pel (xf=1), both pixel[x] and pixel[x+1] may straddle a word
--   boundary; the module reads two adjacent words when necessary.
--
-- Output
-- ------
--   mc_row   : one row of 8 compensated pixels per clock (64-bit)
--   mc_valid : high during 8 output clocks per block
--   mc_last  : high on row 7
--
-- Latency: blk_start → first mc_valid = 3 clocks (2 BRAM reads + 1 interpolate)
--
-- Resource estimate
-- -----------------
--   LUT : ~200
--   FF  : ~150
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity halfpel_mc is
  port (
    aclk        : in  std_logic;
    aresetn     : in  std_logic;

    -- Frame geometry
    frame_width  : in unsigned(11 downto 0);
    frame_height : in unsigned(11 downto 0);

    -- Block start
    blk_start   : in  std_logic;
    blk_x       : in  unsigned(11 downto 0);   -- top-left pixel x
    blk_y       : in  unsigned(11 downto 0);   -- top-left pixel y
    mv          : in  mv_t;                     -- half-pixel MV

    -- Reference BRAM (1-cycle latency)
    ref_rd_addr : out std_logic_vector(14 downto 0);
    ref_rd_data : in  std_logic_vector(63 downto 0);

    -- Output
    mc_row      : out std_logic_vector(63 downto 0);
    mc_valid    : out std_logic;
    mc_last     : out std_logic
  );
end entity halfpel_mc;

architecture rtl of halfpel_mc is

  -- -----------------------------------------------------------------------
  -- State machine
  --
  -- For each of the 8 output rows we do up to 4 BRAM reads:
  --   FETCH_R0W0 : issue read for ref[row  ][x_word  ]
  --   FETCH_R0W1 : latch r0w0; issue read for ref[row  ][x_word+1]
  --   FETCH_R1W0 : latch r0w1; issue read for ref[row+1][x_word  ] (yf=1 only)
  --   FETCH_R1W1 : latch r1w0; issue read for ref[row+1][x_word+1] (yf=1 only)
  --   INTERPOLATE: latch final word; build 8 output pixels
  --
  -- When yf=0 we skip R1W0 / R1W1: FETCH_R0W1 → INTERPOLATE directly.
  -- This fixes the original bug where pixels beyond the first word boundary
  -- were zeroed when ref_x was not 8-byte aligned.
  -- -----------------------------------------------------------------------
  type state_t is (IDLE, FETCH_R0W0, FETCH_R0W1, FETCH_R1W0, FETCH_R1W1,
                   INTERPOLATE);
  signal state   : state_t := IDLE;

  signal row_cnt : integer range 0 to 7 := 0;
  signal xf, yf  : std_logic := '0';
  signal ref_x   : unsigned(11 downto 0);
  signal ref_y   : unsigned(11 downto 0);
  signal x_off   : integer range 0 to 7 := 0;  -- ref_x mod 8

  -- Four latched BRAM words: row0/row1 × word0/word1
  signal r0w0 : std_logic_vector(63 downto 0);
  signal r0w1 : std_logic_vector(63 downto 0);
  signal r1w0 : std_logic_vector(63 downto 0);
  -- r1w1 is taken directly from ref_rd_data in INTERPOLATE

  signal mc_row_r   : std_logic_vector(63 downto 0) := (others => '0');
  signal mc_valid_r : std_logic := '0';
  signal mc_last_r  : std_logic := '0';

  -- -----------------------------------------------------------------------
  -- Bilinear interpolate one output pixel.
  -- a00 = ref[y  ][x  ], a01 = ref[y  ][x+1]
  -- a10 = ref[y+1][x  ], a11 = ref[y+1][x+1]
  -- -----------------------------------------------------------------------
  function bilerp(a00, a01, a10, a11 : std_logic_vector(7 downto 0);
                  xhalf, yhalf : std_logic) return std_logic_vector is
    variable v00, v01, v10, v11 : unsigned(7 downto 0);
    variable sum2               : unsigned(8 downto 0);   -- 2x 8-bit + 1 = max 511
    variable sum4               : unsigned(9 downto 0);   -- 4x 8-bit + 2 = max 1022
  begin
    v00 := unsigned(a00);  v01 := unsigned(a01);
    v10 := unsigned(a10);  v11 := unsigned(a11);
    if xhalf = '0' and yhalf = '0' then
      return a00;
    elsif xhalf = '1' and yhalf = '0' then
      sum2 := ('0' & v00) + ('0' & v01) + 1;
      return std_logic_vector(sum2(8 downto 1));
    elsif xhalf = '0' and yhalf = '1' then
      sum2 := ('0' & v00) + ('0' & v10) + 1;
      return std_logic_vector(sum2(8 downto 1));
    else
      sum4 := ("00" & v00) + ("00" & v01) + ("00" & v10) + ("00" & v11) + 2;
      return std_logic_vector(sum4(9 downto 2));
    end if;
  end function;

  -- Extract byte i from a 64-bit BRAM word (byte 0 = bits 7:0)
  function get_byte(w : std_logic_vector(63 downto 0); i : integer)
    return std_logic_vector is
  begin
    return w(i*8+7 downto i*8);
  end function;

  -- Build one output pixel from the two 64-bit words covering positions
  -- [x_off .. x_off+8) of a row.  w0 covers bytes 0-7, w1 covers bytes 8-15.
  function pick_pixels(w0, w1 : std_logic_vector(63 downto 0);
                       off    : integer;
                       idx    : integer) return std_logic_vector is
    variable pos : integer;
  begin
    pos := off + idx;            -- absolute byte position in 16-byte span
    if pos < 8 then
      return get_byte(w0, pos);
    else
      return get_byte(w1, pos - 8);
    end if;
  end function;

begin

  mc_row   <= mc_row_r;
  mc_valid <= mc_valid_r;
  mc_last  <= mc_last_r;

  process(aclk)
    variable rx_int  : signed(12 downto 0);
    variable ry_int  : signed(12 downto 0);
    variable brow    : integer range 0 to 39;
    variable bcol    : integer range 0 to 479;
    variable out_row : std_logic_vector(63 downto 0);
    variable p00, p01, p10, p11 : std_logic_vector(7 downto 0);
    variable row0_w1 : std_logic_vector(63 downto 0);  -- r0w1 or ref_rd_data depending on yf
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state      <= IDLE;
        mc_valid_r <= '0';
        mc_last_r  <= '0';
      else
        mc_valid_r <= '0';
        mc_last_r  <= '0';

        case state is

          when IDLE =>
            if blk_start = '1' then
              rx_int := to_signed(to_integer(blk_x), 13)
                      + resize(shift_right(signed(mv.dx), 1), 13);
              ry_int := to_signed(to_integer(blk_y), 13)
                      + resize(shift_right(signed(mv.dy), 1), 13);
              xf <= mv.dx(0);
              yf <= mv.dy(0);
              -- Clip integer reference position to frame
              if rx_int < 0 then
                ref_x <= (others => '0');
              elsif rx_int >= signed('0' & frame_width) then
                ref_x <= frame_width - 1;
              else
                ref_x <= unsigned(rx_int(11 downto 0));
              end if;
              if ry_int < 0 then
                ref_y <= (others => '0');
              elsif ry_int >= signed('0' & frame_height) then
                ref_y <= frame_height - 1;
              else
                ref_y <= unsigned(ry_int(11 downto 0));
              end if;
              row_cnt <= 0;
              state   <= FETCH_R0W0;
            end if;

          -- Issue BRAM read for ref[row][x_word]
          when FETCH_R0W0 =>
            x_off  <= to_integer(ref_x) mod 8;
            brow   := to_integer(ref_y + to_unsigned(row_cnt, 12)) mod 40;
            bcol   := to_integer(ref_x) / 8;
            ref_rd_addr <= std_logic_vector(to_unsigned(brow, 6)) &
                           std_logic_vector(to_unsigned(bcol,  9));
            state <= FETCH_R0W1;

          -- Latch r0w0; issue read for ref[row][x_word+1]
          when FETCH_R0W1 =>
            r0w0  <= ref_rd_data;
            brow  := to_integer(ref_y + to_unsigned(row_cnt, 12)) mod 40;
            -- Adjacent x-word; clamp to last valid column to avoid OOB
            bcol  := to_integer(ref_x) / 8 + 1;
            if bcol >= to_integer(frame_width(11 downto 3)) then
              bcol := to_integer(frame_width(11 downto 3)) - 1;
            end if;
            ref_rd_addr <= std_logic_vector(to_unsigned(brow, 6)) &
                           std_logic_vector(to_unsigned(bcol,  9));
            if yf = '1' then
              state <= FETCH_R1W0;
            else
              state <= INTERPOLATE;
            end if;

          -- Latch r0w1; issue read for ref[row+1][x_word]
          when FETCH_R1W0 =>
            r0w1  <= ref_rd_data;
            brow  := (to_integer(ref_y + to_unsigned(row_cnt, 12)) + 1) mod 40;
            bcol  := to_integer(ref_x) / 8;
            ref_rd_addr <= std_logic_vector(to_unsigned(brow, 6)) &
                           std_logic_vector(to_unsigned(bcol,  9));
            state <= FETCH_R1W1;

          -- Latch r1w0; issue read for ref[row+1][x_word+1]
          when FETCH_R1W1 =>
            r1w0  <= ref_rd_data;
            brow  := (to_integer(ref_y + to_unsigned(row_cnt, 12)) + 1) mod 40;
            bcol  := to_integer(ref_x) / 8 + 1;
            if bcol >= to_integer(frame_width(11 downto 3)) then
              bcol := to_integer(frame_width(11 downto 3)) - 1;
            end if;
            ref_rd_addr <= std_logic_vector(to_unsigned(brow, 6)) &
                           std_logic_vector(to_unsigned(bcol,  9));
            state <= INTERPOLATE;

          -- Build 8 output pixels from the latched words.
          -- For yf=0: r0w0, r0w1 valid; ref_rd_data = r0w1 (already in r0w1
          --   for yf=0 path we latched it in FETCH_R0W1, not yet latched r0w1
          --   signal — use ref_rd_data directly as r0w1).
          -- For yf=1: r0w0, r0w1, r1w0, ref_rd_data=r1w1.
          when INTERPOLATE =>
            -- Select the correct second word for row 0
            if yf = '0' then
              row0_w1 := ref_rd_data;
            else
              row0_w1 := r0w1;
            end if;
            out_row := (others => '0');
            for i in 0 to 7 loop
              -- Current row: pick pixel at position x_off+i across w0/w1
              p00 := pick_pixels(r0w0, row0_w1, x_off, i);
              -- x+1 neighbor (for xf)
              if xf = '1' then
                p01 := pick_pixels(r0w0, row0_w1, x_off, i + 1);
              else
                p01 := x"00";
              end if;
              -- Row+1 (for yf)
              if yf = '1' then
                p10 := pick_pixels(r1w0, ref_rd_data, x_off, i);
                if xf = '1' then
                  p11 := pick_pixels(r1w0, ref_rd_data, x_off, i + 1);
                else
                  p11 := x"00";
                end if;
              else
                p10 := x"00";  p11 := x"00";
              end if;
              out_row(i*8+7 downto i*8) := bilerp(p00, p01, p10, p11, xf, yf);
            end loop;
            mc_row_r   <= out_row;
            mc_valid_r <= '1';
            if row_cnt = 7 then
              mc_last_r <= '1';
              state     <= IDLE;
            else
              mc_last_r <= '0';
              row_cnt   <= row_cnt + 1;
              state     <= FETCH_R0W0;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
