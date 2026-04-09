-- =============================================================================
-- me_engine.vhd  --  8x8 block motion estimator (integer-only diamond search)
--
-- Implements a 3-step diamond search (integer-pel only).
-- Half-pixel refinement has been removed to reduce LUT usage.
-- Output MV is expressed in half-pel units (LSB always 0) so that the
-- downstream halfpel_mc / enc_top interface is unchanged.
--
-- Search strategy
-- ---------------
--   Integer search — iterative diamond:
--     Start at (0,0).  At each step, test 5 points (centre + 4 orthogonal)
--     at the current stride (4, 2, 1 pixels).  Move to the best.
--     Total: ~25 SAD evaluations, ~280 clocks per 8x8 block.
--
-- Skip decision
-- -------------
--   After finding best MV, if best_sad < SKIP_THRESH (128 for 8x8 blocks),
--   the skip flag is asserted and no residual is transmitted.
--
-- Resource estimate
-- -----------------
--   LUT : ~800   (SAD accumulator, diamond FSM, BRAM address logic)
--   FF  : ~400
--   DSP : 0      (8-bit ABS differences fit in LUT)
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity me_engine is
  generic (
    SKIP_THRESH : integer := 128   -- SAD below this → skip (8x8: C uses 512/4=128)
  );
  port (
    aclk       : in  std_logic;
    aresetn    : in  std_logic;

    -- Frame geometry
    frame_width  : in unsigned(11 downto 0);
    frame_height : in unsigned(11 downto 0);

    -- Start new block ME
    blk_start  : in  std_logic;
    blk_x      : in  unsigned(11 downto 0);   -- pixel x (multiple of 8)
    blk_y      : in  unsigned(11 downto 0);   -- pixel y (multiple of 8)

    -- Current block pixel input (8 pixels/clock, 8 clocks after blk_start)
    cur_row    : in  std_logic_vector(63 downto 0);
    cur_valid  : in  std_logic;

    -- Reference BRAM read port (1-cycle latency)
    ref_rd_addr : out std_logic_vector(14 downto 0);
    ref_rd_data : in  std_logic_vector(63 downto 0);

    -- Results
    me_done    : out std_logic;
    mv_out     : out mv_t;
    skip_out   : out std_logic
  );
end entity me_engine;

architecture rtl of me_engine is

  -- ---------------------------------------------------------------------------
  -- Current block buffer (8x8 pixels)
  -- ---------------------------------------------------------------------------
  type blk8_t is array(0 to 7) of std_logic_vector(63 downto 0);
  signal cur_buf  : blk8_t;
  signal cur_load : integer range 0 to 7 := 0;

  -- ---------------------------------------------------------------------------
  -- FSM
  -- ---------------------------------------------------------------------------
  type state_t is (IDLE, LOAD, SEARCH_INIT, SAD_ISSUE, SAD_ACC, SAD_COMPARE, DONE);
  signal state : state_t := IDLE;

  -- ---------------------------------------------------------------------------
  -- Integer search state
  -- ---------------------------------------------------------------------------
  -- Diamond offsets: 5-point large diamond (stride), 5-point small diamond (1)
  -- We iterate over three strides: 4, 2, 1.
  type offset_t is record
    dx : integer range -4 to 4;
    dy : integer range -4 to 4;
  end record;
  type offsets5_t is array(0 to 4) of offset_t;
  -- 5-point diamond: center + 4 orthogonal
  constant DIAMOND5 : offsets5_t := (
    (dx =>  0, dy =>  0),
    (dx =>  1, dy =>  0),
    (dx => -1, dy =>  0),
    (dx =>  0, dy =>  1),
    (dx =>  0, dy => -1)
  );

  signal search_stride : integer range 1 to 4 := 4;
  signal cand_idx      : integer range 0 to 8 := 0;  -- index into current diamond
  signal best_mv_dx    : signed(5 downto 0) := (others => '0');
  signal best_mv_dy    : signed(5 downto 0) := (others => '0');
  signal best_sad      : unsigned(19 downto 0) := (others => '1');
  signal cur_cand_dx   : signed(5 downto 0) := (others => '0');
  signal cur_cand_dy   : signed(5 downto 0) := (others => '0');

  -- Base block position (registered on blk_start)
  signal blk_px   : unsigned(11 downto 0) := (others => '0');
  signal blk_py   : unsigned(11 downto 0) := (others => '0');

  -- Half-pixel refinement removed; integer MV is output in half-pel units (LSB=0).

  -- ---------------------------------------------------------------------------
  -- SAD accumulator
  -- ---------------------------------------------------------------------------
  signal sad_row   : integer range 0 to 7 := 0;   -- which row being summed
  signal sad_sum   : unsigned(19 downto 0) := (others => '0');
  signal ref_row_r : std_logic_vector(63 downto 0);  -- latched reference row
  signal ref_rd_r  : std_logic := '0';

  -- Reference row address calculation
  signal ref_y_int  : signed(12 downto 0) := (others => '0');
  signal ref_x_int  : signed(12 downto 0) := (others => '0');

  -- ---------------------------------------------------------------------------
  -- Helper functions
  -- ---------------------------------------------------------------------------
  function abs_diff(a : std_logic_vector(7 downto 0);
                    b : std_logic_vector(7 downto 0)) return unsigned is
    variable ai, bi, diff : integer;
  begin
    ai   := to_integer(unsigned(a));
    bi   := to_integer(unsigned(b));
    diff := ai - bi;
    if diff < 0 then diff := -diff; end if;
    return to_unsigned(diff, 8);
  end function;

  function row_sad(cur : std_logic_vector(63 downto 0);
                   ref : std_logic_vector(63 downto 0)) return unsigned is
    variable s : unsigned(11 downto 0) := (others => '0');
  begin
    for i in 0 to 7 loop
      s := s + resize(abs_diff(cur(i*8+7 downto i*8),
                               ref(i*8+7 downto i*8)), 12);
    end loop;
    return resize(s, 20);
  end function;

  signal me_done_r  : std_logic := '0';
  signal skip_out_r : std_logic := '0';

begin

  me_done  <= me_done_r;
  skip_out <= skip_out_r;

  process(aclk)
    variable cand_x, cand_y : signed(12 downto 0);
    variable ref_bram_row   : integer range 0 to 39;
    variable ref_bram_col   : integer range 0 to 479;
    variable row_sad_v      : unsigned(19 downto 0);
    variable new_sad        : unsigned(19 downto 0);
    variable dx_off, dy_off : integer;
    variable stride         : integer;
    variable fr_width       : integer;
    variable fr_height      : integer;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        state      <= IDLE;
        me_done_r  <= '0';
        skip_out_r <= '0';
        mv_out     <= MV_ZERO;
        ref_rd_r   <= '0';
      else
        me_done_r  <= '0';

        case state is

          -- ------------------------------------------------------------------
          when IDLE =>
            if blk_start = '1' then
              blk_px    <= blk_x;
              blk_py    <= blk_y;
              cur_load  <= 0;
              state     <= LOAD;
            end if;

          -- ------------------------------------------------------------------
          -- LOAD: buffer current block pixels (8 rows, one per clock)
          -- ------------------------------------------------------------------
          when LOAD =>
            if cur_valid = '1' then
              cur_buf(cur_load) <= cur_row;
              if cur_load = 7 then
                cur_load    <= 0;
                best_sad    <= (others => '1');
                best_mv_dx  <= (others => '0');
                best_mv_dy  <= (others => '0');
                state       <= SEARCH_INIT;
              else
                cur_load    <= cur_load + 1;
              end if;
            end if;

          -- ------------------------------------------------------------------
          -- SEARCH_INIT: set up first diamond iteration
          -- ------------------------------------------------------------------
          when SEARCH_INIT =>
            search_stride <= 4;
            cand_idx      <= 0;
            cur_cand_dx   <= best_mv_dx;
            cur_cand_dy   <= best_mv_dy;
            sad_row       <= 0;
            sad_sum       <= (others => '0');
            state         <= SAD_ISSUE;

          -- ------------------------------------------------------------------
          -- SAD_ISSUE: issue BRAM read for current row of current candidate
          -- Reference BRAM row = (blk_py + dy + sad_row) mod BRAM_ROWS
          -- Reference BRAM col = (blk_px + dx) / 8
          -- ------------------------------------------------------------------
          when SAD_ISSUE =>
            fr_width  := to_integer(frame_width);
            fr_height := to_integer(frame_height);
            -- Candidate position
            if cand_idx < 5 then
              dx_off := DIAMOND5(cand_idx).dx * search_stride;
              dy_off := DIAMOND5(cand_idx).dy * search_stride;
            else
              -- No more candidates in this diamond
              dx_off := 0; dy_off := 0;
            end if;
            cand_x := to_signed(to_integer(blk_px), 13)
                     + resize(best_mv_dx, 13) + to_signed(dx_off, 13);
            cand_y := to_signed(to_integer(blk_py), 13)
                     + resize(best_mv_dy, 13) + to_signed(dy_off, 13);
            -- Clip to frame bounds
            if cand_x < 0 then cand_x := to_signed(0, 13); end if;
            if cand_y < 0 then cand_y := to_signed(0, 13); end if;
            if cand_x > to_signed(fr_width - 8, 13) then
              cand_x := to_signed(fr_width - 8, 13);
            end if;
            if cand_y > to_signed(fr_height - 8, 13) then
              cand_y := to_signed(fr_height - 8, 13);
            end if;
            ref_y_int <= cand_y;
            ref_x_int <= cand_x;
            -- Issue BRAM read for row 0 of this candidate
            ref_bram_row := to_integer(cand_y(5 downto 0)) mod 40;
            ref_bram_col := to_integer(unsigned(cand_x(8 downto 0))) / 8;
            ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                             std_logic_vector(to_unsigned(ref_bram_col, 9));
            sad_row   <= 0;
            sad_sum   <= (others => '0');
            state     <= SAD_ACC;

          -- ------------------------------------------------------------------
          -- SAD_ACC: accumulate SAD over 8 rows
          -- Each iteration: compute row SAD from latched data, issue next read
          -- ------------------------------------------------------------------
          when SAD_ACC =>
            -- Row just read is available (1-cycle BRAM latency handled by
            -- issuing the read in SAD_ISSUE one cycle early)
            row_sad_v := row_sad(cur_buf(sad_row), ref_rd_data);
            sad_sum   <= sad_sum + row_sad_v;

            if sad_row = 7 then
              state <= SAD_COMPARE;
            else
              -- Issue next row read
              ref_bram_row := (to_integer(ref_y_int) + sad_row + 1) mod 40;
              ref_bram_col :=  to_integer(unsigned(ref_x_int(8 downto 0))) / 8;
              ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                               std_logic_vector(to_unsigned(ref_bram_col, 9));
              sad_row <= sad_row + 1;
            end if;

          -- ------------------------------------------------------------------
          -- SAD_COMPARE: compare accumulated SAD with best; advance search
          -- ------------------------------------------------------------------
          when SAD_COMPARE =>
            new_sad := sad_sum;
            if new_sad < best_sad then
              best_sad   <= new_sad;
              dx_off     := DIAMOND5(cand_idx).dx * search_stride;
              dy_off     := DIAMOND5(cand_idx).dy * search_stride;
              best_mv_dx <= best_mv_dx + to_signed(dx_off, 6);
              best_mv_dy <= best_mv_dy + to_signed(dy_off, 6);
            end if;

            if cand_idx = 4 then
              -- Finished all 5 candidates for this stride
              if search_stride = 1 then
                -- Integer search done; output result (MV in half-pel units, LSB=0)
                state <= DONE;
              else
                -- Reduce stride
                if search_stride = 4 then search_stride <= 2;
                else                       search_stride <= 1;
                end if;
                cand_idx <= 0;
                state    <= SAD_ISSUE;
              end if;
            else
              cand_idx <= cand_idx + 1;
              state    <= SAD_ISSUE;
            end if;

          -- ------------------------------------------------------------------
          when DONE =>
            -- Output integer MV in half-pel units (append '0' LSB so
            -- halfpel_mc always takes the integer-pel path, no interpolation).
            mv_out.dx  <= best_mv_dx & '0';
            mv_out.dy  <= best_mv_dy & '0';
            if best_sad < to_unsigned(SKIP_THRESH, 20) then
              skip_out_r <= '1';
            else
              skip_out_r <= '0';
            end if;
            me_done_r  <= '1';
            state      <= IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
