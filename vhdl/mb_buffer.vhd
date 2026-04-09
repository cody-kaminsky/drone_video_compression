-- =============================================================================
-- mb_buffer.vhd  --  Macroblock line buffer + intra predictor (YUV 4:2:0)
--
-- Handles three planes of a YUV 4:2:0 stream arriving in planar order:
--   1. Y  plane  (W × H   pixels, arriving first)
--   2. Cb plane  (W/2 × H/2 pixels, arriving second)
--   3. Cr plane  (W/2 × H/2 pixels, arriving third)
--
-- Chroma bytes are auto-detected from the luma y_active flag and a chroma
-- line counter — NOT from plane_sel.  plane_sel is used only to control
-- which plane the EMIT FSM is currently reading from.
--
-- BRAM layout  (24 rows × ROW_STRIDE words per plane, 64 bits wide):
--   Luma: bank 0 = rows 0..7, bank 1 = rows 8..15, bank 2 = rows 16..23
--   Cb:   bank 0 = rows 0..7, bank 1 = rows 8..15, bank 2 = rows 16..23
--   Cr:   bank 0 = rows 0..7, bank 1 = rows 8..15, bank 2 = rows 16..23
-- Triple-buffered: write can fill bank N+1 while read uses bank N,
-- eliminating strip-boundary stalls when the read side is faster.
--
-- Luma intra modes (per block):
--   first_strip=0     → INTRA_VERT  (predict from above row-7)
--   strip 0, bx > 0   → INTRA_HORIZ (predict from left col-7)
--   top-left block    → INTRA_DC    (predictor = 128)
-- Chroma: always INTRA_DC, predictor = 128.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity mb_buffer is
  port (
    aclk         : in  std_logic;
    aresetn      : in  std_logic;
    frame_width  : in  unsigned(11 downto 0);
    frame_height : in  unsigned(11 downto 0);
    enc_enable   : in  std_logic;
    frame_type   : in  std_logic;
    -- Plane selector driven by enc_top ("00"=Y, "01"=Cb, "10"=Cr).
    -- Controls EMIT side only; write side uses y_active / line counts.
    plane_sel    : in  std_logic_vector(1 downto 0);
    -- Chroma dimensions (frame_width/2 and frame_height/2 for 4:2:0)
    chroma_width  : in  unsigned(11 downto 0);
    chroma_height : in  unsigned(11 downto 0);
    -- One-clock pulse: reset chroma write-side counters at Cb→Cr boundary
    chroma_plane_rst : in  std_logic;
    s_tdata      : in  std_logic_vector(7 downto 0);
    s_tvalid     : in  std_logic;
    s_tready     : out std_logic;
    s_tlast      : in  std_logic;
    s_tuser      : in  std_logic;
    m_tdata      : out std_logic_vector(71 downto 0);
    m_tvalid     : out std_logic;
    m_tready     : in  std_logic;
    m_pred_dc    : out unsigned(7 downto 0);
    m_pred_valid : out std_logic;
    m_intra_mode : out intra_mode_t;
    m_above_row  : out std_logic_vector(63 downto 0);
    m_left_col   : out std_logic_vector(63 downto 0);
    strip_rdy_out    : out std_logic;
    cb_strip_rdy_out : out std_logic;
    cr_strip_rdy_out : out std_logic;
    mb_blk_start : out std_logic;
    mb_blk_x     : out unsigned(11 downto 0);
    mb_blk_y     : out unsigned(11 downto 0);
    recon_done   : in  std_logic;
    recon_row7   : in  std_logic_vector(63 downto 0);
    recon_col7   : in  std_logic_vector(63 downto 0)
  );
end entity mb_buffer;

architecture rtl of mb_buffer is

  -- =========================================================================
  -- BRAM: 24 rows × ROW_STRIDE words per plane, 64 bits (triple-buffered)
  -- =========================================================================
  constant ROW_STRIDE : integer := 512;
  constant LB_DEPTH   : integer := 24 * ROW_STRIDE;  -- per plane (3 banks × 8 rows = 24 rows)

  -- Split into three separate BRAMs so each has a single write port,
  -- allowing Vivado to infer block RAM cleanly.
  type lb_t is array(0 to LB_DEPTH-1) of std_logic_vector(63 downto 0);
  signal lb_y  : lb_t;
  signal lb_cb : lb_t;
  signal lb_cr : lb_t;
  attribute ram_style          : string;
  attribute ram_style of lb_y  : signal is "block";
  attribute ram_style of lb_cb : signal is "block";
  attribute ram_style of lb_cr : signal is "block";

  -- Row is 0-23 within each plane's array (bank 0=0..7, bank 1=8..15, bank 2=16..23)
  function lb_addr(row : integer range 0 to 23;
                   col : integer range 0 to ROW_STRIDE-1) return integer is
  begin
    return row * ROW_STRIDE + col;
  end function;

  -- =========================================================================
  -- Above-row predictor store (luma)
  -- =========================================================================
  type above_store_t is array(0 to MAX_MB_COLS*2-1) of std_logic_vector(63 downto 0);
  signal above_row_store : above_store_t := (others => x"8080808080808080");
  attribute ram_style of above_row_store : signal is "distributed";

  -- =========================================================================
  -- Luma write side
  -- =========================================================================
  signal wr_row    : integer range 0 to 7           := 0;
  signal wr_waddr  : integer range 0 to ROW_STRIDE-1 := 0;
  signal wr_phase  : integer range 0 to 7           := 0;
  signal wr_buf    : std_logic_vector(63 downto 0)  := (others => '0');
  signal line_cnt  : unsigned(11 downto 0)          := (others => '0');
  signal y_active      : std_logic := '0';
  signal strip_rdy     : std_logic := '0';
  signal wr_bank       : integer range 0 to 2 := 0;
  signal rd_bank       : integer range 0 to 2 := 0;
  signal strip_pend    : integer range 0 to 7 := 0;

  -- =========================================================================
  -- Chroma write side (auto-detects Cb/Cr from y_active + chr_line_cnt)
  -- =========================================================================
  -- chr_is_cr: '0' while writing Cb, '1' while writing Cr
  signal chr_is_cr         : std_logic := '0';
  signal chr_line_cnt      : unsigned(11 downto 0) := (others => '0');
  signal chr_active        : std_logic := '0';

  signal cb_wr_row         : integer range 0 to 7           := 0;
  signal cb_wr_waddr       : integer range 0 to ROW_STRIDE-1 := 0;
  signal cb_wr_phase       : integer range 0 to 7           := 0;
  signal cb_wr_buf         : std_logic_vector(63 downto 0)  := (others => '0');
  signal cb_wr_bank        : integer range 0 to 2 := 0;
  signal cb_rd_bank        : integer range 0 to 2 := 0;
  signal cb_strip_rdy_r    : std_logic := '0';
  signal cb_strip_pend     : integer range 0 to 7 := 0;

  signal cr_wr_row         : integer range 0 to 7           := 0;
  signal cr_wr_waddr       : integer range 0 to ROW_STRIDE-1 := 0;
  signal cr_wr_phase       : integer range 0 to 7           := 0;
  signal cr_wr_buf         : std_logic_vector(63 downto 0)  := (others => '0');
  signal cr_wr_bank        : integer range 0 to 2 := 0;
  signal cr_rd_bank        : integer range 0 to 2 := 0;
  signal cr_strip_rdy_r    : std_logic := '0';
  signal cr_strip_pend     : integer range 0 to 7 := 0;

  -- Internal readable copy of s_tready (s_tready is an 'out' port so it
  -- cannot be read back by this architecture's own processes).
  signal tready_i          : std_logic;

  -- =========================================================================
  -- Shared prefetch / row registers
  -- =========================================================================
  type row_reg_t is array(0 to 7) of std_logic_vector(63 downto 0);
  signal row_regs     : row_reg_t;
  signal rd_fetch_row : integer range 0 to 7           := 0;
  signal rd_waddr     : integer range 0 to ROW_STRIDE-1 := 0;
  signal fetch_cnt    : integer range 0 to 9           := 0;

  -- Separate registered outputs from each BRAM (enables block-RAM inference
  -- by giving each array a clean single-source read pattern).
  signal rd_data_y    : std_logic_vector(63 downto 0);
  signal rd_data_cb   : std_logic_vector(63 downto 0);
  signal rd_data_cr   : std_logic_vector(63 downto 0);
  signal rd_data      : std_logic_vector(63 downto 0);
  -- Plane selector registered 1 cycle (aligned with BRAM read latency)
  signal rd_sel_d1    : std_logic_vector(1 downto 0) := "00";

  -- =========================================================================
  -- Emit FSM (luma + chroma states)
  -- =========================================================================
  type fsm_t is (IDLE, PREFETCH, CALC, EMIT, WAIT_RECON,
                 CHR_IDLE, CHR_PREFETCH, CHR_CALC, CHR_EMIT, CHR_WAIT_RECON);
  signal fsm : fsm_t := IDLE;

  -- BRAM bank selection for current read operation
  signal use_chroma_rd : std_logic := '0';

  -- Luma emit state
  signal rd_blk       : integer range 0 to MAX_MB_COLS*2 := 0;
  signal n_blk_cols   : integer range 1 to MAX_MB_COLS*2 := 1;
  signal rd_row       : integer range 0 to 7 := 0;
  signal m_tvalid_r   : std_logic := '0';
  signal last_blk_done : std_logic := '0';

  -- Chroma emit state
  signal chr_rd_blk     : integer range 0 to MAX_MB_COLS := 0;
  signal chr_n_blk_cols : integer range 1 to MAX_MB_COLS := 1;
  signal chr_strip_y_r  : unsigned(11 downto 0) := (others => '0');
  signal chr_last_blk_done : std_logic := '0';

  -- =========================================================================
  -- Intra prediction registers (shared)
  -- =========================================================================
  signal predictor     : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal left_col_pix  : std_logic_vector(63 downto 0) := x"8080808080808080";
  signal intra_mode_r  : intra_mode_t := INTRA_DC;
  signal above_row_r   : std_logic_vector(63 downto 0) := (others => '0');
  signal first_strip   : std_logic := '1';
  signal pred_valid_r  : std_logic := '0';
  signal strip_y_r     : unsigned(11 downto 0) := (others => '0');
  signal mb_blk_start_r : std_logic := '0';
  signal mb_blk_x_r    : unsigned(11 downto 0) := (others => '0');
  signal mb_blk_y_r    : unsigned(11 downto 0) := (others => '0');

begin

  -- =========================================================================
  -- Concurrent assignments
  -- =========================================================================
  m_tvalid    <= m_tvalid_r;
  m_pred_dc   <= predictor;
  m_pred_valid <= pred_valid_r;
  m_intra_mode <= intra_mode_r;
  m_above_row  <= above_row_r;
  m_left_col   <= left_col_pix;
  n_blk_cols     <= to_integer(frame_width(11 downto 3));
  chr_n_blk_cols <= to_integer(chroma_width(11 downto 3));
  strip_rdy_out    <= strip_rdy;
  cb_strip_rdy_out <= cb_strip_rdy_r;
  cr_strip_rdy_out <= cr_strip_rdy_r;
  mb_blk_start <= mb_blk_start_r;
  mb_blk_x     <= mb_blk_x_r;
  mb_blk_y     <= mb_blk_y_r;

  use_chroma_rd <= '1' when (fsm = CHR_IDLE or fsm = CHR_PREFETCH or
                              fsm = CHR_CALC or fsm = CHR_EMIT or
                              fsm = CHR_WAIT_RECON) else '0';

  -- Back-pressure: gate by the pending count appropriate to the current byte type
  -- y_active='1'           → luma bytes  → gate by luma strip_pend
  -- y_active='0', not cr   → Cb bytes    → gate by cb_strip_pend
  -- y_active='0', cr       → Cr bytes    → gate by cr_strip_pend
  tready_i <= enc_enable when
                ((y_active = '1'  and strip_pend    < 2) or
                 (y_active = '0'  and chr_is_cr = '0' and cb_strip_pend < 2) or
                 (y_active = '0'  and chr_is_cr = '1' and cr_strip_pend < 2))
              else '0';
  s_tready <= tready_i;

  -- =========================================================================
  -- BRAM synchronous reads (1-cycle latency)
  -- Three separate processes, one per array, so Vivado sees each lb_y/cb/cr
  -- as an independent single-source read → enables block-RAM inference.
  -- All three read every cycle; the correct output is selected by rd_sel_d1.
  -- =========================================================================

  -- Luma BRAM read
  process(aclk)
  begin
    if rising_edge(aclk) then
      rd_data_y <= lb_y(lb_addr(rd_bank * 8 + rd_fetch_row, rd_waddr));
    end if;
  end process;

  -- Cb BRAM read
  process(aclk)
  begin
    if rising_edge(aclk) then
      rd_data_cb <= lb_cb(lb_addr(cb_rd_bank * 8 + rd_fetch_row, rd_waddr));
    end if;
  end process;

  -- Cr BRAM read
  process(aclk)
  begin
    if rising_edge(aclk) then
      rd_data_cr <= lb_cr(lb_addr(cr_rd_bank * 8 + rd_fetch_row, rd_waddr));
    end if;
  end process;

  -- Register plane selector in sync with BRAM read (1-cycle latency alignment)
  process(aclk)
  begin
    if rising_edge(aclk) then
      if use_chroma_rd = '0' then
        rd_sel_d1 <= "00";
      else
        rd_sel_d1 <= plane_sel;
      end if;
    end if;
  end process;

  -- Output mux: select correct BRAM output (combinatorial, after registered reads)
  rd_data <= rd_data_cb when rd_sel_d1 = "01" else
             rd_data_cr when rd_sel_d1 = "10" else
             rd_data_y;

  -- =========================================================================
  -- Combined BRAM write process (luma + chroma) — single driver for lb.
  -- Having two separate processes both assigning to lb elements causes
  -- multiple-driver resolution to 'U' in simulation even though they write
  -- to disjoint address ranges.
  -- =========================================================================
  process(aclk)
    variable buf : std_logic_vector(63 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' or enc_enable = '0' then
        -- Luma write-side
        wr_row       <= 0;  wr_waddr <= 0;  wr_phase <= 0;
        line_cnt     <= (others => '0');
        y_active     <= '0';
        strip_rdy    <= '0';
        wr_bank      <= 0;
        -- Chroma write-side
        chr_active      <= '0';
        chr_is_cr       <= '0';
        chr_line_cnt    <= (others => '0');
        cb_wr_row    <= 0;  cb_wr_waddr <= 0;  cb_wr_phase <= 0;
        cr_wr_row    <= 0;  cr_wr_waddr <= 0;  cr_wr_phase <= 0;
        cb_wr_bank   <= 0;
        cr_wr_bank   <= 0;
        cb_strip_rdy_r  <= '0';
        cr_strip_rdy_r  <= '0';
      else
        strip_rdy      <= '0';
        cb_strip_rdy_r <= '0';
        cr_strip_rdy_r <= '0';

        -- -----------------------------------------------------------------
        -- Luma write: bank 0 = rows 0-7, bank 1 = rows 8-15, bank 2 = rows 16-23
        -- -----------------------------------------------------------------
        if s_tvalid = '1' and tready_i = '1' and (y_active = '1' or s_tuser = '1') then

          if s_tuser = '1' then
            wr_row   <= 0;  wr_waddr <= 0;  wr_phase <= 0;
            line_cnt <= (others => '0');
            y_active <= '1';
            wr_bank  <= 0;
            -- Reset chroma write-side for new frame
            chr_active      <= '0';
            chr_is_cr       <= '0';
            chr_line_cnt    <= (others => '0');
            cb_wr_row    <= 0;  cb_wr_waddr <= 0;  cb_wr_phase <= 0;
            cr_wr_row    <= 0;  cr_wr_waddr <= 0;  cr_wr_phase <= 0;
            cb_wr_bank   <= 0;
            cr_wr_bank   <= 0;
          end if;

          if s_tuser = '1' or line_cnt < frame_height then
            buf := wr_buf;
            buf(wr_phase*8+7 downto wr_phase*8) := s_tdata;
            if wr_phase = 7 then
              lb_y(lb_addr(wr_bank * 8 + wr_row, wr_waddr)) <= buf;
              wr_phase <= 0;
              if wr_waddr < ROW_STRIDE - 1 then
                wr_waddr <= wr_waddr + 1;
              end if;
            else
              wr_buf   <= buf;
              wr_phase <= wr_phase + 1;
            end if;
          end if;

          if s_tlast = '1' then
            wr_waddr <= 0;
            wr_phase <= 0;
            if line_cnt < frame_height then
              if wr_row = 7 then
                wr_row       <= 0;
                strip_rdy    <= '1';
                if wr_bank = 2 then wr_bank <= 0; else wr_bank <= wr_bank + 1; end if;
              else
                wr_row <= wr_row + 1;
              end if;
            end if;
            line_cnt <= line_cnt + 1;
            if line_cnt + 1 = frame_height then
              y_active <= '0';
            end if;
          end if;

        end if;

        -- -----------------------------------------------------------------
        -- Optional external reset at Cb→Cr boundary
        -- -----------------------------------------------------------------
        if chroma_plane_rst = '1' then
          chr_is_cr       <= '1';
          chr_line_cnt    <= (others => '0');
          cr_wr_row       <= 0;  cr_wr_waddr <= 0;  cr_wr_phase <= 0;
          cr_wr_bank      <= 0;
        end if;

        -- -----------------------------------------------------------------
        -- Chroma write: Cb → rows 16-31, Cr → rows 32-47.
        -- Fires for all chroma bytes (y_active='0', s_tuser='0').
        -- chr_active is set here on the first byte so the first Cb byte
        -- is not lost due to the one-cycle activation delay in a separate
        -- process.
        -- -----------------------------------------------------------------
        if s_tvalid = '1' and tready_i = '1' and y_active = '0' and s_tuser = '0' then
          chr_active <= '1';

          if chr_is_cr = '0' then
            -- Writing Cb → bank 0/1/2 (rows 0-7, 8-15, 16-23 of lb_cb)
            if chr_line_cnt < chroma_height then
              buf := cb_wr_buf;
              buf(cb_wr_phase*8+7 downto cb_wr_phase*8) := s_tdata;
              if cb_wr_phase = 7 then
                lb_cb(lb_addr(cb_wr_bank * 8 + cb_wr_row, cb_wr_waddr)) <= buf;
                cb_wr_phase <= 0;
                if cb_wr_waddr < ROW_STRIDE - 1 then
                  cb_wr_waddr <= cb_wr_waddr + 1;
                end if;
              else
                cb_wr_buf   <= buf;
                cb_wr_phase <= cb_wr_phase + 1;
              end if;
            end if;

            if s_tlast = '1' then
              cb_wr_waddr <= 0;
              cb_wr_phase <= 0;
              if chr_line_cnt < chroma_height then
                if cb_wr_row = 7 then
                  cb_wr_row       <= 0;
                  cb_strip_rdy_r  <= '1';
                  if cb_wr_bank = 2 then cb_wr_bank <= 0; else cb_wr_bank <= cb_wr_bank + 1; end if;
                else
                  cb_wr_row <= cb_wr_row + 1;
                end if;
              end if;
              chr_line_cnt <= chr_line_cnt + 1;
              if chr_line_cnt + 1 = chroma_height then
                chr_is_cr    <= '1';
                chr_line_cnt <= (others => '0');  -- reset for Cr counting
              end if;
            end if;

          else
            -- Writing Cr → bank 0/1/2 (rows 0-7, 8-15, 16-23 of lb_cr)
            if chr_line_cnt < chroma_height then
              buf := cr_wr_buf;
              buf(cr_wr_phase*8+7 downto cr_wr_phase*8) := s_tdata;
              if cr_wr_phase = 7 then
                lb_cr(lb_addr(cr_wr_bank * 8 + cr_wr_row, cr_wr_waddr)) <= buf;
                cr_wr_phase <= 0;
                if cr_wr_waddr < ROW_STRIDE - 1 then
                  cr_wr_waddr <= cr_wr_waddr + 1;
                end if;
              else
                cr_wr_buf   <= buf;
                cr_wr_phase <= cr_wr_phase + 1;
              end if;
            end if;

            if s_tlast = '1' then
              cr_wr_waddr <= 0;
              cr_wr_phase <= 0;
              if chr_line_cnt < chroma_height then
                if cr_wr_row = 7 then
                  cr_wr_row       <= 0;
                  cr_strip_rdy_r  <= '1';
                  if cr_wr_bank = 2 then cr_wr_bank <= 0; else cr_wr_bank <= cr_wr_bank + 1; end if;
                else
                  cr_wr_row <= cr_wr_row + 1;
                end if;
              end if;
              chr_line_cnt <= chr_line_cnt + 1;
            end if;

          end if;
        end if;

      end if;
    end if;
  end process;

  -- =========================================================================
  -- Emit FSM (luma states + chroma states)
  -- =========================================================================
  process(aclk)
    variable pix      : unsigned(7 downto 0);
    variable pred_pix : unsigned(7 downto 0);
    variable res      : signed(8 downto 0);
    variable row_out  : std_logic_vector(71 downto 0);
    variable has_above : boolean;
    variable has_left  : boolean;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' or enc_enable = '0' then
        fsm            <= IDLE;
        rd_blk         <= 0;  rd_row <= 0;
        fetch_cnt      <= 0;  rd_fetch_row <= 0;
        m_tvalid_r     <= '0';
        strip_pend     <= 0;
        cb_strip_pend  <= 0;
        cr_strip_pend  <= 0;
        predictor      <= to_unsigned(128, 8);
        left_col_pix   <= x"8080808080808080";
        intra_mode_r   <= INTRA_DC;
        above_row_r    <= (others => '0');
        first_strip    <= '1';
        pred_valid_r   <= '0';
        strip_y_r      <= (others => '0');
        chr_strip_y_r  <= (others => '0');
        mb_blk_start_r <= '0';
        mb_blk_x_r     <= (others => '0');
        mb_blk_y_r     <= (others => '0');
        rd_bank         <= 0;
        cb_rd_bank      <= 0;
        cr_rd_bank      <= 0;
        chr_rd_blk      <= 0;
        last_blk_done      <= '0';
        chr_last_blk_done  <= '0';
      else

        -- -------------------------------------------------------------------
        -- Strip-pending counters
        -- -------------------------------------------------------------------
        if strip_rdy    = '1' then strip_pend   <= strip_pend   + 1; end if;
        if cb_strip_rdy_r = '1' then cb_strip_pend <= cb_strip_pend + 1; end if;
        if cr_strip_rdy_r = '1' then cr_strip_pend <= cr_strip_pend + 1; end if;

        case fsm is

          -- =================================================================
          -- LUMA STATES
          -- =================================================================
          when IDLE =>
            m_tvalid_r <= '0';
            if strip_pend > 0 then
              if strip_rdy = '1' then
                strip_pend <= strip_pend;
              else
                strip_pend <= strip_pend - 1;
              end if;
              rd_blk       <= 0;
              rd_waddr     <= 0;
              rd_fetch_row <= 0;
              fetch_cnt    <= 0;
              -- rd_bank already points to correct bank (advanced at end
              -- of previous strip, or reset to 0 at frame/reset boundary)
              left_col_pix <= x"8080808080808080";
              fsm          <= PREFETCH;
            -- Move to chroma FSM when luma done and plane_sel is chroma
            elsif plane_sel /= "00" then
              chr_strip_y_r <= (others => '0');
              cb_rd_bank    <= 0;
              cr_rd_bank    <= 0;
              fsm <= CHR_IDLE;
            end if;

          when PREFETCH =>
            if fetch_cnt > 0 then
              row_regs(fetch_cnt - 1) <= rd_data;
            end if;
            if fetch_cnt = 8 then
              fsm <= CALC;
            else
              if fetch_cnt < 7 then
                rd_fetch_row <= fetch_cnt + 1;
              end if;
              fetch_cnt <= fetch_cnt + 1;
            end if;

          when CALC =>
            mb_blk_start_r <= '1';
            mb_blk_x_r     <= to_unsigned(rd_blk * 8, 12);
            mb_blk_y_r     <= strip_y_r;
            has_above := (first_strip = '0');
            has_left  := (rd_blk > 0);
            above_row_r <= above_row_store(rd_blk);
            if frame_type = FRAME_P then
              intra_mode_r <= INTRA_DC;
              predictor    <= to_unsigned(0, 8);
            elsif has_above then
              intra_mode_r <= INTRA_VERT;
              predictor    <= to_unsigned(0, 8);
            elsif has_left then
              intra_mode_r <= INTRA_HORIZ;
              predictor    <= to_unsigned(0, 8);
            else
              intra_mode_r <= INTRA_DC;
              predictor    <= to_unsigned(128, 8);
            end if;
            pred_valid_r <= '1';
            rd_row <= 0;
            fsm    <= EMIT;

          when EMIT =>
            mb_blk_start_r <= '0';
            pred_valid_r <= '0';
            if m_tready = '1' or m_tvalid_r = '0' then
              row_out := (others => '0');
              for i in 0 to 7 loop
                pix := unsigned(row_regs(rd_row)(i*8+7 downto i*8));
                if intra_mode_r = INTRA_VERT then
                  pred_pix := unsigned(above_row_r(i*8+7 downto i*8));
                elsif intra_mode_r = INTRA_HORIZ then
                  pred_pix := unsigned(left_col_pix(rd_row*8+7 downto rd_row*8));
                else
                  pred_pix := predictor;
                end if;
                res := signed('0' & pix) - signed('0' & pred_pix);
                row_out(i*9+8 downto i*9) := std_logic_vector(res);
              end loop;
              m_tdata    <= row_out;
              m_tvalid_r <= '1';
              if rd_row = 7 then
                if rd_blk + 1 = n_blk_cols then
                  last_blk_done <= '1';
                else
                  last_blk_done <= '0';
                  -- Pre-issue BRAM read for next block: advance rd_waddr now so
                  -- the BRAM continuously reads row 0 of the next block throughout
                  -- WAIT_RECON.  Row 0 data arrives on PREFETCH cycle 0 (saves 1 cy).
                  rd_waddr     <= rd_waddr + 1;
                  rd_fetch_row <= 0;
                  fetch_cnt    <= 0;  -- reset for early-prefetch loop in WAIT_RECON
                end if;
                fsm <= WAIT_RECON;
              else
                rd_row <= rd_row + 1;
              end if;
            end if;

          when WAIT_RECON =>
            if m_tready = '1' then
              m_tvalid_r <= '0';
            end if;
            -- Early PREFETCH overlap for VERT / DC modes (no left_col dependency).
            -- rd_waddr and rd_fetch_row were pre-set in EMIT; fetch_cnt starts at 0.
            -- We run the same capture loop as PREFETCH so that by the time
            -- recon_done fires, some (or all) of the 8 rows are already loaded.
            if intra_mode_r /= INTRA_HORIZ and last_blk_done = '0' then
              if fetch_cnt < 8 then
                if fetch_cnt > 0 then
                  row_regs(fetch_cnt - 1) <= rd_data;
                end if;
                if fetch_cnt < 7 then
                  rd_fetch_row <= fetch_cnt + 1;
                end if;
                fetch_cnt <= fetch_cnt + 1;
              elsif fetch_cnt = 8 then
                row_regs(7) <= rd_data;  -- hold last-row capture stable
              end if;
            end if;
            if recon_done = '1' then
              above_row_store(rd_blk) <= recon_row7;
              left_col_pix            <= recon_col7;
              if last_blk_done = '1' then
                first_strip <= '0';
                strip_y_r   <= strip_y_r + 8;
                -- Advance rd_bank for next strip (triple-buffer)
                if rd_bank = 2 then rd_bank <= 0; else rd_bank <= rd_bank + 1; end if;
                fsm         <= IDLE;
              else
                rd_blk <= rd_blk + 1;
                if intra_mode_r /= INTRA_HORIZ then
                  -- VERT/DC: early prefetch was running; continue from
                  -- current fetch_cnt (rd_waddr & rd_fetch_row already set).
                  fsm <= PREFETCH;
                else
                  -- HORIZ: only the Step-2 single-cycle optimisation applies.
                  rd_fetch_row <= 1;
                  fetch_cnt    <= 1;
                  fsm          <= PREFETCH;
                end if;
              end if;
            end if;

          -- =================================================================
          -- CHROMA STATES (DC=128 prediction, always INTRA_DC)
          -- =================================================================
          when CHR_IDLE =>
            m_tvalid_r <= '0';
            -- Process pending strip for the currently selected plane
            if plane_sel = "01" and cb_strip_pend > 0 then
              if cb_strip_rdy_r = '1' then
                cb_strip_pend <= cb_strip_pend;
              else
                cb_strip_pend <= cb_strip_pend - 1;
              end if;
              -- cb_rd_bank already points to correct bank (advanced at
              -- end of previous chroma strip, or reset to 0 at phase start)
              chr_rd_blk   <= 0;
              rd_waddr     <= 0;
              rd_fetch_row <= 0;
              fetch_cnt    <= 0;
              fsm          <= CHR_PREFETCH;
            elsif plane_sel = "10" and cr_strip_pend > 0 then
              if cr_strip_rdy_r = '1' then
                cr_strip_pend <= cr_strip_pend;
              else
                cr_strip_pend <= cr_strip_pend - 1;
              end if;
              -- cr_rd_bank already points to correct bank
              chr_rd_blk   <= 0;
              rd_waddr     <= 0;
              rd_fetch_row <= 0;
              fetch_cnt    <= 0;
              fsm          <= CHR_PREFETCH;
            elsif plane_sel = "00" then
              -- Chroma phase ended; return to luma IDLE for next frame
              rd_bank <= 0;  -- reset for next frame's luma
              fsm <= IDLE;
            end if;

          when CHR_PREFETCH =>
            if fetch_cnt > 0 then
              row_regs(fetch_cnt - 1) <= rd_data;
            end if;
            if fetch_cnt = 8 then
              fsm <= CHR_CALC;
            else
              if fetch_cnt < 7 then
                rd_fetch_row <= fetch_cnt + 1;
              end if;
              fetch_cnt <= fetch_cnt + 1;
            end if;

          when CHR_CALC =>
            -- Chroma always uses INTRA_DC, predictor = 128
            mb_blk_start_r <= '1';
            mb_blk_x_r     <= to_unsigned(chr_rd_blk * 8, 12);
            mb_blk_y_r     <= chr_strip_y_r;
            intra_mode_r   <= INTRA_DC;
            predictor      <= to_unsigned(128, 8);
            pred_valid_r   <= '1';
            rd_row         <= 0;
            fsm            <= CHR_EMIT;

          when CHR_EMIT =>
            mb_blk_start_r <= '0';
            pred_valid_r   <= '0';
            if m_tready = '1' or m_tvalid_r = '0' then
              row_out := (others => '0');
              for i in 0 to 7 loop
                pix      := unsigned(row_regs(rd_row)(i*8+7 downto i*8));
                pred_pix := to_unsigned(128, 8);
                res      := signed('0' & pix) - signed('0' & pred_pix);
                row_out(i*9+8 downto i*9) := std_logic_vector(res);
              end loop;
              m_tdata    <= row_out;
              m_tvalid_r <= '1';
              if rd_row = 7 then
                if chr_rd_blk + 1 = chr_n_blk_cols then
                  chr_last_blk_done <= '1';
                else
                  chr_last_blk_done <= '0';
                end if;
                fsm <= CHR_WAIT_RECON;
              else
                rd_row <= rd_row + 1;
              end if;
            end if;

          when CHR_WAIT_RECON =>
            if m_tready = '1' then
              m_tvalid_r <= '0';
            end if;
            if recon_done = '1' then
              -- Chroma uses DC=128 only; no above/left update needed
              if chr_last_blk_done = '1' then
                chr_strip_y_r <= chr_strip_y_r + 8;
                -- Advance chroma rd_bank for next strip (triple-buffer)
                if plane_sel = "01" then
                  if cb_rd_bank = 2 then cb_rd_bank <= 0; else cb_rd_bank <= cb_rd_bank + 1; end if;
                else
                  if cr_rd_bank = 2 then cr_rd_bank <= 0; else cr_rd_bank <= cr_rd_bank + 1; end if;
                end if;
                fsm           <= CHR_IDLE;
              else
                chr_rd_blk   <= chr_rd_blk + 1;
                rd_waddr     <= rd_waddr + 1;
                rd_fetch_row <= 0;
                fetch_cnt    <= 0;
                fsm          <= CHR_PREFETCH;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
