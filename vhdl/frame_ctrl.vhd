-- =============================================================================
-- frame_ctrl.vhd  --  GOP controller and frame-type signal
--
-- Tracks the encoded frame count and produces frame_type (FRAME_I / FRAME_P)
-- based on the configured GOP size.
--
-- frame_type = FRAME_I when (frame_count mod gop_size == 0) or gop_size == 1.
-- The signal is registered and stable for the full duration of each frame.
--
-- It also:
--   - Asserts frame_start for one clock when a new frame begins (detected from
--     s_tuser='1' on the video input), so the bitstream writer can insert the
--     1-bit frame-type header token.
--   - Asserts frame_end for one clock when the last block of the frame finishes
--     coding (fed back from enc_top's existing frame_done signal).
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity frame_ctrl is
  port (
    aclk        : in  std_logic;
    aresetn     : in  std_logic;

    -- Configuration
    gop_size    : in  unsigned(7 downto 0);   -- 1 = I-only, N = IPPPP...

    -- Trigger: s_tuser='1' on first video byte of new frame
    frame_sof   : in  std_logic;   -- start-of-frame pulse from video input

    -- Feedback: last block coded (from enc_top block_cnt logic)
    frame_done  : in  std_logic;   -- end-of-frame pulse

    -- Outputs
    frame_type  : out std_logic;   -- FRAME_I or FRAME_P (stable for full frame)
    frame_start : out std_logic;   -- 1-clock pulse at start of frame
    frame_end   : out std_logic;   -- mirrors frame_done
    frame_count : out unsigned(31 downto 0)
  );
end entity frame_ctrl;

architecture rtl of frame_ctrl is

  signal cnt_r      : unsigned(31 downto 0) := (others => '0');
  signal gop_pos    : unsigned(7 downto 0)  := (others => '0');
  signal ftype_r    : std_logic := FRAME_I;
  signal fstart_r   : std_logic := '0';

begin

  frame_type  <= ftype_r;
  frame_start <= fstart_r;
  frame_end   <= frame_done;
  frame_count <= cnt_r;

  process(aclk)
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        cnt_r    <= (others => '0');
        gop_pos  <= (others => '0');
        ftype_r  <= FRAME_I;
        fstart_r <= '0';
      else
        fstart_r <= '0';

        if frame_sof = '1' then
          fstart_r <= '1';
          -- Determine type of next frame
          if gop_pos = 0 or gop_size = 1 then
            ftype_r <= FRAME_I;
          else
            ftype_r <= FRAME_P;
          end if;
        end if;

        if frame_done = '1' then
          cnt_r <= cnt_r + 1;
          if gop_size = 1 then
            gop_pos <= (others => '0');
          elsif gop_pos = gop_size - 1 then
            gop_pos <= (others => '0');
          else
            gop_pos <= gop_pos + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

end architecture rtl;
