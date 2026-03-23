-- =============================================================================
-- enc_pkg.vhd  --  Shared types, constants and utility functions
--
-- All encoder components use this package.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package enc_pkg is

  -- ---------------------------------------------------------------------------
  -- Frame / block geometry
  -- ---------------------------------------------------------------------------
  constant MB_SIZE    : integer := 16;
  constant BLOCK_SIZE : integer := 8;
  constant MAX_WIDTH  : integer := 3840;
  constant MAX_HEIGHT : integer := 2160;
  constant MAX_MB_COLS : integer := MAX_WIDTH  / MB_SIZE;  -- 240
  constant MAX_MB_ROWS : integer := MAX_HEIGHT / MB_SIZE;  -- 135

  -- QP range
  constant QP_MIN : integer := 1;
  constant QP_MAX : integer := 51;

  -- ---------------------------------------------------------------------------
  -- AXI-Lite register map (byte offsets)
  -- ---------------------------------------------------------------------------
  constant REG_CTRL      : integer := 16#00#;  -- [0]=enable  [1]=soft-reset
  constant REG_STATUS    : integer := 16#04#;  -- [0]=busy    [1]=frame_done
  constant REG_WIDTH     : integer := 16#08#;  -- frame width  (12-bit)
  constant REG_HEIGHT    : integer := 16#0C#;  -- frame height (12-bit)
  constant REG_QP        : integer := 16#10#;  -- QP (6-bit, 1–51)
  constant REG_GOP       : integer := 16#14#;  -- GOP size (8-bit)
  constant REG_FRAME_CNT : integer := 16#18#;  -- frames encoded (RO)
  constant REG_BS_BYTES  : integer := 16#1C#;  -- bitstream bytes last frame (RO)

  -- ---------------------------------------------------------------------------
  -- Named subtypes (avoids Vivado parser issue with constrained types inline)
  -- ---------------------------------------------------------------------------
  subtype s32_t is signed(31 downto 0);
  subtype s16_t is signed(15 downto 0);
  subtype u8_t  is unsigned(7 downto 0);

  -- ---------------------------------------------------------------------------
  -- Composite array types
  -- ---------------------------------------------------------------------------
  -- One row of 8 samples (used inside DCT and prediction stages)
  type int32_row_t  is array(0 to 7) of s32_t;
  type int16_row_t  is array(0 to 7) of s16_t;
  type uint8_row_t  is array(0 to 7) of u8_t;

  -- 8×8 block arrays (used inside DCT/quantiser)
  type block32_t    is array(0 to 7) of int32_row_t;
  type block16_t    is array(0 to 7) of int16_row_t;

  -- Intra prediction mode (matches common.h)
  subtype intra_mode_t is std_logic_vector(1 downto 0);
  constant INTRA_DC     : intra_mode_t := "00";
  constant INTRA_HORIZ  : intra_mode_t := "01";
  constant INTRA_VERT   : intra_mode_t := "10";
  constant INTRA_PLANAR : intra_mode_t := "11";

  -- Frame type
  constant FRAME_I : std_logic := '0';
  constant FRAME_P : std_logic := '1';

  -- Motion search range (integer pixels; half-pixel range = 2 × this)
  constant SEARCH_RANGE : integer := 16;

  -- Motion vector (half-pixel units, ±32 range fits in signed 7-bit)
  type mv_t is record
    dx : signed(6 downto 0);   -- half-pixel MV x, range -64..+63
    dy : signed(6 downto 0);   -- half-pixel MV y, range -64..+63
  end record mv_t;

  constant MV_ZERO : mv_t := (dx => (others => '0'), dy => (others => '0'));

  -- ---------------------------------------------------------------------------
  -- Fixed-point DCT constants  (same values as dct.c / dct_hw.c)
  -- All scaled by 2^13 = 8192
  -- ---------------------------------------------------------------------------
  constant CONST_BITS      : integer := 13;
  constant PASS1_BITS      : integer := 2;
  constant IDCT_CONST_BITS : integer := 13;
  constant IDCT_PASS1_BITS : integer := 1;

  constant FIX_0_298631336 : integer :=  2446;
  constant FIX_0_390180644 : integer :=  3196;
  constant FIX_0_541196100 : integer :=  4433;
  constant FIX_0_765366865 : integer :=  6270;
  constant FIX_0_899976223 : integer :=  7373;
  constant FIX_1_175875602 : integer :=  9633;
  constant FIX_1_501321110 : integer := 12299;
  constant FIX_1_847759065 : integer := 15137;
  constant FIX_1_961570560 : integer := 16069;
  constant FIX_2_053119869 : integer := 16819;
  constant FIX_2_562915447 : integer := 20995;
  constant FIX_3_072711026 : integer := 25172;

  -- ---------------------------------------------------------------------------
  -- Utility functions
  -- ---------------------------------------------------------------------------

  -- Clip a signed value to [lo, hi]
  function clip_int(x : integer; lo : integer; hi : integer) return integer;

  -- Clip to unsigned 8-bit pixel range [0, 255]
  function clip8(x : s16_t) return u8_t;

  -- Zigzag position → (row, col) in 8×8 block
  -- Returns natural-order index 0..63 given zigzag index 0..63
  function zigzag_to_natural(z : integer) return integer;

end package enc_pkg;

-- =============================================================================
-- Package body needs its own context clause (separate VHDL design unit)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package body enc_pkg is

  function clip_int(x : integer; lo : integer; hi : integer) return integer is
  begin
    if    x < lo then return lo;
    elsif x > hi then return hi;
    else               return x;
    end if;
  end function;

  function clip8(x : s16_t) return u8_t is
  begin
    if    x < 0   then return x"00";
    elsif x > 255 then return x"FF";
    else               return unsigned(x(7 downto 0));
    end if;
  end function;

  -- Zigzag scan table (matches ZIGZAG_8x8 in common.h)
  type zigzag_table_t is array(0 to 63) of integer range 0 to 63;
  constant ZIGZAG_TBL : zigzag_table_t := (
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
  );

  function zigzag_to_natural(z : integer) return integer is
  begin
    return ZIGZAG_TBL(z);
  end function;

end package body enc_pkg;
