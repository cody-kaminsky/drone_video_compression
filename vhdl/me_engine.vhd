-- =============================================================================
-- me_engine.vhd  --  8x8 block motion estimator (diamond search + half-pixel)
--
-- Implements a 3-step diamond search followed by half-pixel refinement,
-- matching inter_search_integer + inter_refine_halfpel in predict.c.
--
-- Search strategy
-- ---------------
--   1. Integer search — iterative diamond:
--        Start at (0,0).  At each step, test 4 or 8 neighbours at the current
--        stride (4, 2, 1 pixels).  Move to the best.  Repeat at next stride.
--        Total: ~25 SAD evaluations.
--   2. Half-pixel refinement — test 8 half-pel positions around the best
--        integer MV.  Half-pel interpolation is bilinear (same as C encoder).
--        Total: 8 SAD evaluations.
--
-- SAD computation
-- ---------------
--   Current block (8x8 = 64 pixels) is buffered in registers.
--   Reference block is fetched from the search-window BRAM via rd_addr/rd_data.
--   One row of 8 pixels per clock is compared; SAD accumulates over 8 clocks
--   per candidate + 2 pipeline overhead = 10 clocks per candidate.
--   Sequential (single SAD pipeline), targeting ~720p@30fps throughput.
--   For 4K, instantiate 4-8 me_engine instances in parallel (one per MB-row
--   quadrant) — add parallelism externally without redesigning this module.
--
-- Skip decision
-- -------------
--   After finding best MV, if best_sad < SKIP_THRESH (= 16*16*2/4 = 128 for
--   8x8 blocks), the skip flag is asserted and no residual is transmitted.
--
-- Interface
-- ---------
--   blk_start : pulse — start ME for a new 8x8 block
--   cur_row   : current block pixels (one row of 8, presented during LOAD)
--   cur_valid : one per clock during LOAD (8 clocks)
--   blk_x/y   : top-left pixel coordinate of current block
--   Output    : mv, skip, me_done
--
-- Resource estimate
-- -----------------
--   LUT : ~1200  (SAD accumulator, diamond FSM, BRAM address logic)
--   FF  : ~600
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
  type state_t is (IDLE, LOAD, SEARCH_INIT, SAD_ISSUE, SAD_ACC, SAD_COMPARE,
                   HALFPEL_INIT, HP_ISSUE, HP_ACC, HP_COMPARE, DONE);
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

  -- ---------------------------------------------------------------------------
  -- Half-pixel refinement state
  -- ---------------------------------------------------------------------------
  -- 8 half-pixel offsets: (±1, 0), (0, ±1), (±1, ±1)
  type hp_offset_t is record
    dx : integer range -1 to 1;
    dy : integer range -1 to 1;
  end record;
  type hp_offsets8_t is array(0 to 7) of hp_offset_t;
  constant HP8 : hp_offsets8_t := (
    (dx =>  1, dy =>  0), (dx => -1, dy =>  0),
    (dx =>  0, dy =>  1), (dx =>  0, dy => -1),
    (dx =>  1, dy =>  1), (dx => -1, dy =>  1),
    (dx =>  1, dy => -1), (dx => -1, dy => -1)
  );
  signal hp_idx   : integer range 0 to 7 := 0;
  signal hp_mv_dx : signed(6 downto 0) := (others => '0');  -- half-pixel MV
  signal hp_mv_dy : signed(6 downto 0) := (others => '0');

  -- ---------------------------------------------------------------------------
  -- SAD accumulator
  -- ---------------------------------------------------------------------------
  signal sad_row   : integer range 0 to 7 := 0;   -- which row being summed
  signal sad_sum   : unsigned(19 downto 0) := (others => '0');  -- renamed: sad_sum clashes with SAD_ACC state
  signal sad_cur   : unsigned(19 downto 0);  -- running partial SAD
  signal ref_row_r : std_logic_vector(63 downto 0);  -- latched reference row
  signal ref_rd_r  : std_logic := '0';  -- BRAM read issued this cycle

  -- Reference row address calculation
  signal ref_y_int  : signed(12 downto 0) := (others => '0');  -- ref y (integer)
  signal ref_x_int  : signed(12 downto 0) := (others => '0');  -- ref x (integer)

  -- Half-pixel: bilinear interpolation registers
  -- hp_phase: 0=row_y/col0 just arrived  1=row_y/col1 arriving
  --           2=row_y+1/col0 arriving     3=row_y+1/col1 arriving
  signal hp_xf      : std_logic := '0';   -- x sub-pixel flag for current HP candidate
  signal hp_yf      : std_logic := '0';   -- y sub-pixel flag
  signal hp_x_off   : integer range 0 to 7 := 0;  -- ref_x mod 8 (byte offset in word)
  signal hp_x_col   : integer range 0 to 479 := 0; -- ref_x / 8 (base BRAM column)
  signal hp_w0      : std_logic_vector(63 downto 0);  -- row_y,   col0
  signal hp_w1      : std_logic_vector(63 downto 0);  -- row_y,   col1 (xf=1)
  signal hp_w2      : std_logic_vector(63 downto 0);  -- row_y+1, col0 (yf=1)
  signal hp_phase   : integer range 0 to 3 := 0;

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

  -- Extract byte at position (off+idx) from a 16-byte span across two BRAM words.
  -- w0 = bytes 0..7, w1 = bytes 8..15.
  function pick_byte(w0, w1 : std_logic_vector(63 downto 0);
                     off, idx : integer) return std_logic_vector is
    variable pos : integer;
  begin
    pos := off + idx;
    if pos < 8 then
      return w0(pos*8+7 downto pos*8);
    else
      return w1((pos-8)*8+7 downto (pos-8)*8);
    end if;
  end function;

  -- Compute SAD for one bilinear-interpolated row.
  -- w00/w01 = row_y  word0/word1; w10/w11 = row_y+1 word0/word1.
  -- x_off: byte offset of first pixel within word pair (= ref_x mod 8).
  -- xf/yf: sub-pixel flags.
  function hp_row_sad(cur             : std_logic_vector(63 downto 0);
                      w00, w01, w10, w11 : std_logic_vector(63 downto 0);
                      x_off           : integer;
                      xf, yf          : std_logic) return unsigned is
    variable s    : unsigned(11 downto 0) := (others => '0');
    variable p00, p01, p10, p11 : unsigned(7 downto 0);
    variable interp : unsigned(7 downto 0);
    variable sum2   : unsigned(8 downto 0);   -- 2x 8-bit + 1 = max 511, 9 bits
    variable sum4   : unsigned(9 downto 0);   -- 4x 8-bit + 2 = max 1022, 10 bits
  begin
    for i in 0 to 7 loop
      p00 := unsigned(pick_byte(w00, w01, x_off, i));
      if xf = '1' then
        p01 := unsigned(pick_byte(w00, w01, x_off, i + 1));
      else
        p01 := (others => '0');
      end if;
      if yf = '1' then
        p10 := unsigned(pick_byte(w10, w11, x_off, i));
        if xf = '1' then
          p11 := unsigned(pick_byte(w10, w11, x_off, i + 1));
        else
          p11 := (others => '0');
        end if;
      else
        p10 := (others => '0');
        p11 := (others => '0');
      end if;
      if xf = '1' and yf = '0' then
        sum2   := ('0' & p00) + ('0' & p01) + 1;
        interp := sum2(8 downto 1);
      elsif xf = '0' and yf = '1' then
        sum2   := ('0' & p00) + ('0' & p10) + 1;
        interp := sum2(8 downto 1);
      elsif xf = '1' and yf = '1' then
        sum4   := ("00" & p00) + ("00" & p01) + ("00" & p10) + ("00" & p11) + 2;
        interp := sum4(9 downto 2);
      else
        interp := p00;
      end if;
      s := s + resize(abs_diff(cur(i*8+7 downto i*8),
                               std_logic_vector(interp)), 12);
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
    variable hp_sum_x       : signed(6 downto 0);
    variable hp_sum_y       : signed(6 downto 0);
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
                -- Integer search done; start half-pixel
                hp_idx  <= 0;
                hp_mv_dx <= best_mv_dx & '0';  -- convert to half-pel units (x2)
                hp_mv_dy <= best_mv_dy & '0';
                state    <= HALFPEL_INIT;
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
          -- HALFPEL_INIT: set up half-pixel refinement
          -- ------------------------------------------------------------------
          when HALFPEL_INIT =>
            hp_idx    <= 0;
            sad_row   <= 0;
            sad_sum   <= (others => '0');
            hp_phase  <= 0;
            state     <= HP_ISSUE;

          -- ------------------------------------------------------------------
          -- HP_ISSUE: set up bilinear half-pixel SAD for one candidate.
          -- Computes xf/yf flags and issues BRAM read for row 0, word 0.
          -- ------------------------------------------------------------------
          when HP_ISSUE =>
            dx_off := HP8(hp_idx).dx;
            dy_off := HP8(hp_idx).dy;
            -- Integer pixel position = (half-pel MV + offset) >> 1
            cand_x := to_signed(to_integer(blk_px), 13)
                     + resize(shift_right(hp_mv_dx + to_signed(dx_off, 7), 1), 13);
            cand_y := to_signed(to_integer(blk_py), 13)
                     + resize(shift_right(hp_mv_dy + to_signed(dy_off, 7), 1), 13);
            -- Sub-pixel flags: LSB of total half-pel MV + offset
            hp_sum_x := hp_mv_dx + to_signed(dx_off, 7);
            hp_sum_y := hp_mv_dy + to_signed(dy_off, 7);
            hp_xf <= hp_sum_x(0);
            hp_yf <= hp_sum_y(0);
            fr_width  := to_integer(frame_width);
            fr_height := to_integer(frame_height);
            if cand_x < 0 then cand_x := to_signed(0, 13); end if;
            if cand_y < 0 then cand_y := to_signed(0, 13); end if;
            if cand_x > to_signed(fr_width - 9, 13) then
              cand_x := to_signed(fr_width - 9, 13);   -- leave room for x+1
            end if;
            if cand_y > to_signed(fr_height - 9, 13) then
              cand_y := to_signed(fr_height - 9, 13);  -- leave room for y+1
            end if;
            ref_y_int   <= cand_y;
            ref_x_int   <= cand_x;
            hp_x_off    <= to_integer(unsigned(cand_x(2 downto 0)));   -- mod 8
            hp_x_col    <= to_integer(unsigned(cand_x(11 downto 3)));  -- /8
            ref_bram_row := to_integer(cand_y(5 downto 0)) mod 40;
            ref_bram_col := to_integer(unsigned(cand_x(11 downto 3)));
            ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                             std_logic_vector(to_unsigned(ref_bram_col, 9));
            sad_row  <= 0;
            sad_sum  <= (others => '0');
            hp_phase <= 0;
            state    <= HP_ACC;

          -- ------------------------------------------------------------------
          -- HP_ACC: accumulate bilinear-interpolated SAD over 8 rows.
          --
          -- Always fetches 4 BRAM words per row (2 x-cols × 2 y-rows) to
          -- support arbitrary alignment (hp_x_off) and all xf/yf combinations.
          --
          -- hp_phase 0: row_y,   col0 arrives → latch hp_w0; issue row_y,   col1
          -- hp_phase 1: row_y,   col1 arrives → latch hp_w1; issue row_y+1, col0
          -- hp_phase 2: row_y+1, col0 arrives → latch hp_w2; issue row_y+1, col1
          -- hp_phase 3: row_y+1, col1 arrives → compute SAD; issue next row col0
          -- ------------------------------------------------------------------
          when HP_ACC =>
            case hp_phase is

              when 0 =>
                hp_w0 <= ref_rd_data;
                ref_bram_row := (to_integer(ref_y_int) + sad_row) mod 40;
                ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                                 std_logic_vector(to_unsigned(hp_x_col + 1, 9));
                hp_phase <= 1;

              when 1 =>
                hp_w1 <= ref_rd_data;
                ref_bram_row := (to_integer(ref_y_int) + sad_row + 1) mod 40;
                ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                                 std_logic_vector(to_unsigned(hp_x_col, 9));
                hp_phase <= 2;

              when 2 =>
                hp_w2 <= ref_rd_data;
                ref_bram_row := (to_integer(ref_y_int) + sad_row + 1) mod 40;
                ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                                 std_logic_vector(to_unsigned(hp_x_col + 1, 9));
                hp_phase <= 3;

              when 3 =>
                -- All 4 words latched: hp_w0=row_y/col0, hp_w1=row_y/col1,
                --                      hp_w2=row_y+1/col0, ref_rd_data=row_y+1/col1
                row_sad_v := hp_row_sad(cur_buf(sad_row),
                                        hp_w0, hp_w1, hp_w2, ref_rd_data,
                                        hp_x_off, hp_xf, hp_yf);
                sad_sum <= sad_sum + row_sad_v;
                if sad_row = 7 then
                  state <= HP_COMPARE;
                else
                  ref_bram_row := (to_integer(ref_y_int) + sad_row + 1) mod 40;
                  ref_rd_addr  <= std_logic_vector(to_unsigned(ref_bram_row, 6)) &
                                   std_logic_vector(to_unsigned(hp_x_col, 9));
                  sad_row  <= sad_row + 1;
                  hp_phase <= 0;
                end if;

            end case;

          when HP_COMPARE =>
            if sad_sum < best_sad then
              best_sad <= sad_sum;
              hp_mv_dx <= hp_mv_dx + to_signed(HP8(hp_idx).dx, 7);
              hp_mv_dy <= hp_mv_dy + to_signed(HP8(hp_idx).dy, 7);
            end if;
            if hp_idx = 7 then
              state <= DONE;
            else
              hp_idx <= hp_idx + 1;
              state  <= HP_ISSUE;
            end if;

          -- ------------------------------------------------------------------
          when DONE =>
            mv_out.dx  <= resize(hp_mv_dx, 7);
            mv_out.dy  <= resize(hp_mv_dy, 7);
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
