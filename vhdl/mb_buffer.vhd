-- =============================================================================
-- mb_buffer.vhd  --  Macroblock line buffer + intra predictor
--
-- Intra prediction modes (per block):
--   has_above (strip >= 1) : INTRA_VERT  -- predict from above row-7 pixels
--   has_left only (strip 0, bx > 0) : INTRA_HORIZ -- predict from left col-7 pixels
--   neither (top-left block) : INTRA_DC with predictor = 128
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
    s_tdata      : in  std_logic_vector(7 downto 0);
    s_tvalid     : in  std_logic;
    s_tready     : out std_logic;
    s_tlast      : in  std_logic;
    s_tuser      : in  std_logic;
    m_tdata      : out std_logic_vector(71 downto 0);
    m_tvalid     : out std_logic;
    m_tready     : in  std_logic;
    -- Exposed DC predictor (used by recon_writer for DC-mode reconstruction)
    m_pred_dc    : out unsigned(7 downto 0);
    m_pred_valid : out std_logic;
    -- Intra mode and neighbour pixels (for enc_top / recon_writer)
    m_intra_mode : out intra_mode_t;
    m_above_row  : out std_logic_vector(63 downto 0);  -- above row-7 pixels (VERT)
    m_left_col   : out std_logic_vector(63 downto 0);  -- left col-7 pixels (HORIZ)
    -- Block-level outputs
    strip_rdy_out  : out std_logic;
    mb_blk_start   : out std_logic;
    mb_blk_x       : out unsigned(11 downto 0);
    mb_blk_y       : out unsigned(11 downto 0);
    -- Reconstructed pixels from recon_writer
    recon_done     : in  std_logic;
    recon_row7     : in  std_logic_vector(63 downto 0);  -- 8 reconstructed row-7 pixels
    recon_col7     : in  std_logic_vector(63 downto 0)   -- 8 reconstructed col-7 pixels
  );
end entity mb_buffer;

architecture rtl of mb_buffer is

  -- -------------------------------------------------------------------------
  -- Flat BRAM: depth = 16 rows x ROW_STRIDE words, width = 64 bits (8 pixels)
  -- -------------------------------------------------------------------------
  constant ROW_STRIDE : integer := 512;
  constant LB_DEPTH   : integer := 16 * ROW_STRIDE;

  type lb_t is array(0 to LB_DEPTH-1) of std_logic_vector(63 downto 0);
  signal lb : lb_t;
  attribute ram_style       : string;
  attribute ram_style of lb : signal is "block";

  function lb_addr(row : integer range 0 to 15;
                   col : integer range 0 to ROW_STRIDE-1) return integer is
  begin
    return row * ROW_STRIDE + col;
  end function;

  -- -------------------------------------------------------------------------
  -- Above-row predictor store: 8 pixels per block column (64 bits)
  -- Distributed RAM so reads are asynchronous (available same cycle as CALC)
  -- -------------------------------------------------------------------------
  type above_store_t is array(0 to MAX_MB_COLS*2-1) of std_logic_vector(63 downto 0);
  signal above_row_store : above_store_t := (others => x"8080808080808080");
  attribute ram_style of above_row_store : signal is "distributed";

  -- -------------------------------------------------------------------------
  -- Write side
  -- -------------------------------------------------------------------------
  signal wr_row    : integer range 0 to 7           := 0;
  signal wr_waddr  : integer range 0 to ROW_STRIDE-1 := 0;
  signal wr_phase  : integer range 0 to 7           := 0;
  signal wr_buf    : std_logic_vector(63 downto 0)  := (others => '0');
  signal line_cnt  : unsigned(11 downto 0)          := (others => '0');
  signal y_active      : std_logic                      := '0';
  signal strip_rdy     : std_logic                      := '0';
  signal wr_strip_sel  : std_logic                      := '0';
  signal rd_strip_sel  : std_logic                      := '0';

  -- -------------------------------------------------------------------------
  -- Prefetch
  -- -------------------------------------------------------------------------
  type row_reg_t is array(0 to 7) of std_logic_vector(63 downto 0);
  signal row_regs     : row_reg_t;
  signal rd_fetch_row : integer range 0 to 7           := 0;
  signal rd_waddr     : integer range 0 to ROW_STRIDE-1 := 0;
  signal rd_data      : std_logic_vector(63 downto 0);
  signal fetch_cnt    : integer range 0 to 9           := 0;

  -- -------------------------------------------------------------------------
  -- Emit FSM
  -- -------------------------------------------------------------------------
  type fsm_t is (IDLE, PREFETCH, CALC, EMIT, WAIT_RECON);
  signal fsm         : fsm_t  := IDLE;
  signal rd_blk      : integer range 0 to MAX_MB_COLS*2 := 0;
  signal n_blk_cols  : integer range 1 to MAX_MB_COLS*2 := 1;
  signal rd_row      : integer range 0 to 7 := 0;
  signal m_tvalid_r  : std_logic := '0';
  signal strip_pend  : integer range 0 to 7 := 0;

  -- -------------------------------------------------------------------------
  -- Intra prediction registers
  -- -------------------------------------------------------------------------
  signal predictor     : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal left_col_pix  : std_logic_vector(63 downto 0) := x"8080808080808080";
  signal intra_mode_r  : intra_mode_t := INTRA_DC;
  signal above_row_r   : std_logic_vector(63 downto 0) := (others => '0');
  signal last_blk_done : std_logic := '0';
  signal first_strip   : std_logic := '1';
  signal pred_valid_r  : std_logic := '0';
  signal strip_y_r     : unsigned(11 downto 0) := (others => '0');
  signal mb_blk_start_r : std_logic := '0';
  signal mb_blk_x_r    : unsigned(11 downto 0) := (others => '0');
  signal mb_blk_y_r    : unsigned(11 downto 0) := (others => '0');

begin

  m_tvalid       <= m_tvalid_r;
  s_tready       <= enc_enable when strip_pend = 0 else '0';
  m_pred_dc      <= predictor;
  m_pred_valid   <= pred_valid_r;
  m_intra_mode   <= intra_mode_r;
  m_above_row    <= above_row_r;
  m_left_col     <= left_col_pix;
  n_blk_cols     <= to_integer(frame_width(11 downto 3));
  strip_rdy_out  <= strip_rdy;
  mb_blk_start   <= mb_blk_start_r;
  mb_blk_x       <= mb_blk_x_r;
  mb_blk_y       <= mb_blk_y_r;

  -- -------------------------------------------------------------------------
  -- BRAM synchronous read (1-cycle latency)
  -- -------------------------------------------------------------------------
  process(aclk)
    variable rd_bank : integer range 0 to 8;
  begin
    if rising_edge(aclk) then
      if rd_strip_sel = '1' then rd_bank := 8; else rd_bank := 0; end if;
      rd_data <= lb(lb_addr(rd_bank + rd_fetch_row, rd_waddr));
    end if;
  end process;

  -- -------------------------------------------------------------------------
  -- Write: pack pixels into 64-bit words, write to BRAM
  -- -------------------------------------------------------------------------
  process(aclk)
    variable buf : std_logic_vector(63 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' or enc_enable = '0' then
        wr_row <= 0;  wr_waddr <= 0;  wr_phase <= 0;
        line_cnt <= (others => '0');
        y_active <= '0';  strip_rdy <= '0';
        wr_strip_sel <= '0';
      else
        strip_rdy <= '0';

        if s_tvalid = '1' then

          if s_tuser = '1' then
            wr_row <= 0;  wr_waddr <= 0;  wr_phase <= 0;
            line_cnt <= (others => '0');
            y_active <= '1';
          end if;

          if (y_active = '1' or s_tuser = '1') and
             (s_tuser = '1' or line_cnt < frame_height) then
            buf := wr_buf;
            buf(wr_phase*8+7 downto wr_phase*8) := s_tdata;

            if wr_phase = 7 then
              if wr_strip_sel = '1' then
                lb(lb_addr(8 + wr_row, wr_waddr)) <= buf;
              else
                lb(lb_addr(wr_row, wr_waddr)) <= buf;
              end if;
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
            if y_active = '1' and line_cnt < frame_height then
              if wr_row = 7 then
                wr_row       <= 0;
                strip_rdy    <= '1';
                wr_strip_sel <= not wr_strip_sel;
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
      end if;
    end if;
  end process;

  -- -------------------------------------------------------------------------
  -- Emit FSM with intra predictor
  -- -------------------------------------------------------------------------
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
        predictor      <= to_unsigned(128, 8);
        left_col_pix   <= x"8080808080808080";
        intra_mode_r   <= INTRA_DC;
        above_row_r    <= (others => '0');
        first_strip    <= '1';
        pred_valid_r   <= '0';
        strip_y_r      <= (others => '0');
        mb_blk_start_r <= '0';
        mb_blk_x_r     <= (others => '0');
        mb_blk_y_r     <= (others => '0');
        rd_strip_sel   <= '0';
      else
        if strip_rdy = '1' then
          strip_pend <= strip_pend + 1;
        end if;

        case fsm is

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
              rd_strip_sel <= not wr_strip_sel;
              left_col_pix <= x"8080808080808080";  -- reset left at strip start
              fsm          <= PREFETCH;
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

          -- Compute predictor mode; latch above row pixels
          when CALC =>
            mb_blk_start_r <= '1';
            mb_blk_x_r     <= to_unsigned(rd_blk * 8, 12);
            mb_blk_y_r     <= strip_y_r;
            has_above := (first_strip = '0');
            has_left  := (rd_blk > 0);

            -- Latch above row pixels (async read from distributed RAM)
            above_row_r <= above_row_store(rd_blk);

            if frame_type = FRAME_P then
              intra_mode_r <= INTRA_DC;
              predictor    <= to_unsigned(0, 8);
            elsif has_above then
              intra_mode_r <= INTRA_VERT;
              predictor    <= to_unsigned(0, 8);  -- not used in VERT
            elsif has_left then
              intra_mode_r <= INTRA_HORIZ;
              predictor    <= to_unsigned(0, 8);  -- not used in HORIZ
            else
              intra_mode_r <= INTRA_DC;
              predictor    <= to_unsigned(128, 8);
            end if;

            pred_valid_r <= '1';
            rd_row   <= 0;
            fsm      <= EMIT;

          when EMIT =>
            mb_blk_start_r <= '0';
            pred_valid_r <= '0';
            if m_tready = '1' or m_tvalid_r = '0' then
              row_out := (others => '0');

              for i in 0 to 7 loop
                pix := unsigned(row_regs(rd_row)(i*8+7 downto i*8));
                if intra_mode_r = INTRA_VERT then
                  -- Predict from pixel directly above (same column)
                  pred_pix := unsigned(above_row_r(i*8+7 downto i*8));
                elsif intra_mode_r = INTRA_HORIZ then
                  -- Predict from pixel directly to the left (same row)
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
            if recon_done = '1' then
              -- Store full 8-pixel row 7 and col 7 for next strip's prediction
              above_row_store(rd_blk) <= recon_row7;
              left_col_pix            <= recon_col7;
              if last_blk_done = '1' then
                first_strip <= '0';
                strip_y_r   <= strip_y_r + 8;
                fsm         <= IDLE;
              else
                rd_blk       <= rd_blk + 1;
                rd_waddr     <= rd_waddr + 1;
                rd_fetch_row <= 0;
                fetch_cnt    <= 0;
                fsm          <= PREFETCH;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
