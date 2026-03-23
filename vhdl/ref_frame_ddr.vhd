-- =============================================================================
-- ref_frame_ddr.vhd  --  Reference frame DDR manager + search-window BRAM
--
-- Manages the DDR reference frame (luma-only) via an AXI4 HP master port.
-- Provides two services:
--
--   1. WRITE: accepts reconstructed 8x8 blocks (64 pixels), writes them to
--      DDR at the correct luma address.  Address = base + y*stride + x, where
--      stride is round_up(frame_width, 8).
--
--   2. PREFETCH + SEARCH BRAM: maintains a 40-row x MAX_WIDTH search-window
--      BRAM that the ME engine and halfpel_mc read from.  As the encoder
--      advances strip-by-strip, the module prefetches the next strip of
--      reference rows from DDR into the BRAM circular buffer.
--
-- AXI4 HP port
-- ------------
--   Data width : 64 bits (HP port native)
--   Burst type : INCR
--   Max burst  : 16 beats (AWLEN/ARLEN = 15)
--   Addressing : byte addresses, word-aligned (3 LSBs = 0)
--
-- Search-window BRAM
-- ------------------
--   Depth  : BRAM_ROWS (40) x BRAM_COLS (= ceil(MAX_WIDTH/8) = 480) words
--   Width  : 64 bits (8 luma pixels per word)
--   Port A : DDR prefetch write (this module)
--   Port B : ME / MC read (external – separate rd_addr/rd_data ports)
--
-- Circular row mapping
-- --------------------
--   bram_row(y) = y mod BRAM_ROWS
--   The module always keeps rows [prefetch_top .. prefetch_top+BRAM_ROWS-1]
--   of the reference frame in the BRAM.
--
-- Resource estimate (UltraScale+, MAX_WIDTH=3840)
-- ------------------------------------------------
--   BRAM36  : ceil(40 x 480 x 64 / 36864) ≈ 42
--   LUT     : ~1100  (AXI state machine + address logic)
--   FF      : ~800
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity ref_frame_ddr is
  generic (
    BRAM_ROWS : integer := 40   -- rows of reference kept in BRAM (>= 2*SEARCH_RANGE+8)
  );
  port (
    aclk         : in  std_logic;
    aresetn      : in  std_logic;

    -- Encoder configuration
    frame_width  : in  unsigned(11 downto 0);   -- pixels
    frame_height : in  unsigned(11 downto 0);
    ref_base     : in  unsigned(31 downto 0);   -- DDR byte address of luma plane

    -- -------------------------------------------------------------------------
    -- Write port: reconstructed 8x8 block pixels → DDR
    -- -------------------------------------------------------------------------
    -- Caller drives blk_x/blk_y, then presents 64 pixels one per clock.
    wr_start     : in  std_logic;                       -- pulse: new block start
    wr_blk_x     : in  unsigned(11 downto 0);           -- pixel x of block top-left
    wr_blk_y     : in  unsigned(11 downto 0);           -- pixel y of block top-left
    wr_pixel     : in  std_logic_vector(7 downto 0);    -- one pixel per clock
    wr_pixel_v   : in  std_logic;                       -- pixel valid
    wr_done      : out std_logic;                       -- all 64 pixels accepted

    -- -------------------------------------------------------------------------
    -- Prefetch control: called at start of each new strip row
    -- -------------------------------------------------------------------------
    -- Caller pulses prefetch_start with the pixel y of the REFERENCE rows
    -- needed for the upcoming block row (typically ref_row_top = blk_y - SEARCH_RANGE).
    -- This module fetches BRAM_ROWS rows from DDR into the circular BRAM.
    prefetch_start : in  std_logic;
    prefetch_ref_y : in  unsigned(11 downto 0);    -- topmost ref row to prefetch
    prefetch_done  : out std_logic;                -- all rows loaded into BRAM

    -- -------------------------------------------------------------------------
    -- BRAM read port (for ME engine and halfpel_mc)
    -- Address: {row_in_bram[5:0], word_col[8:0]} = 15-bit
    -- row_in_bram = (ref_pixel_y mod BRAM_ROWS)
    -- word_col    = ref_pixel_x / 8
    -- -------------------------------------------------------------------------
    rd_addr      : in  std_logic_vector(14 downto 0);  -- {row[5:0], col[8:0]}
    rd_data      : out std_logic_vector(63 downto 0);  -- 8 pixels

    -- -------------------------------------------------------------------------
    -- AXI4 HP master (64-bit data)
    -- -------------------------------------------------------------------------
    -- Write address channel
    m_axi_awaddr  : out std_logic_vector(31 downto 0);
    m_axi_awlen   : out std_logic_vector(7 downto 0);
    m_axi_awsize  : out std_logic_vector(2 downto 0);
    m_axi_awburst : out std_logic_vector(1 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in  std_logic;
    -- Write data channel
    m_axi_wdata   : out std_logic_vector(63 downto 0);
    m_axi_wstrb   : out std_logic_vector(7 downto 0);
    m_axi_wlast   : out std_logic;
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in  std_logic;
    -- Write response channel
    m_axi_bresp   : in  std_logic_vector(1 downto 0);
    m_axi_bvalid  : in  std_logic;
    m_axi_bready  : out std_logic;
    -- Read address channel
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    -- Read data channel
    m_axi_rdata   : in  std_logic_vector(63 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rlast   : in  std_logic;
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic
  );
end entity ref_frame_ddr;

architecture rtl of ref_frame_ddr is

  -- ---------------------------------------------------------------------------
  -- Constants
  -- ---------------------------------------------------------------------------
  constant BRAM_COLS   : integer := MAX_WIDTH / 8;  -- 480 words per row at 4K

  -- ---------------------------------------------------------------------------
  -- BRAM: 40 rows x 480 words x 64-bit
  -- Vivado infers block RAM from this style.
  -- Port A: write (prefetch from DDR or reconstruct write)
  -- Port B: read  (ME / MC)
  -- ---------------------------------------------------------------------------
  constant BRAM_DEPTH  : integer := BRAM_ROWS * BRAM_COLS;
  type bram_t is array(0 to BRAM_DEPTH-1) of std_logic_vector(63 downto 0);
  signal bram : bram_t;
  attribute ram_style       : string;
  attribute ram_style of bram : signal is "block";

  -- Port A (write) — two sources muxed in the BRAM write process
  -- Pixel-write path (reconstructed pixels from recon_writer)
  signal wr_bram_en   : std_logic := '0';
  signal wr_bram_addr : integer range 0 to BRAM_DEPTH-1 := 0;
  signal wr_bram_din  : std_logic_vector(63 downto 0);
  -- Prefetch path (DDR read data)
  signal pf_bram_en   : std_logic := '0';
  signal pf_bram_addr : integer range 0 to BRAM_DEPTH-1 := 0;
  signal pf_bram_din  : std_logic_vector(63 downto 0);

  -- Port B (read) — registered output (1-cycle latency)
  signal bram_b_addr : integer range 0 to BRAM_DEPTH-1 := 0;
  signal bram_b_dout : std_logic_vector(63 downto 0);

  -- ---------------------------------------------------------------------------
  -- Write pixel buffer: collect 8 pixels into one 64-bit BRAM word
  -- ---------------------------------------------------------------------------
  signal wr_phase     : integer range 0 to 7 := 0;
  signal wr_word      : std_logic_vector(63 downto 0) := (others => '0');
  signal wr_px        : unsigned(11 downto 0) := (others => '0');
  signal wr_py        : unsigned(11 downto 0) := (others => '0');
  signal wr_row       : integer range 0 to 7 := 0;  -- row within 8x8 block
  signal wr_col_word  : integer range 0 to BRAM_COLS-1 := 0;
  signal wr_cnt       : integer range 0 to 64 := 0;
  signal wr_done_r    : std_logic := '0';

  -- AXI write FSM for reconstructed pixels → DDR
  type wr_axi_state_t is (WR_IDLE, WR_ADDR, WR_DATA, WR_RESP);
  signal wr_axi_state : wr_axi_state_t := WR_IDLE;
  -- Buffer one full row (8 words = 64 bytes) before issuing AXI burst
  type row64_t is array(0 to 7) of std_logic_vector(63 downto 0);
  signal wr_row_buf   : row64_t;
  signal wr_row_rdy   : std_logic := '0';  -- one row ready to write to DDR
  signal wr_row_idx   : integer range 0 to 7 := 0;
  signal wr_axi_row   : integer range 0 to 7 := 0;  -- row within block being written
  signal wr_axi_px    : unsigned(11 downto 0) := (others => '0');
  signal wr_axi_py    : unsigned(11 downto 0) := (others => '0');
  signal wr_burst_cnt : integer range 0 to 7 := 0;

  -- ---------------------------------------------------------------------------
  -- Prefetch FSM: read BRAM_ROWS rows from DDR into BRAM
  -- ---------------------------------------------------------------------------
  type pf_state_t is (PF_IDLE, PF_ADDR, PF_DATA, PF_NEXT_ROW);
  signal pf_state     : pf_state_t := PF_IDLE;
  signal pf_ref_y     : unsigned(11 downto 0) := (others => '0');
  signal pf_row       : integer range 0 to BRAM_ROWS-1 := 0;  -- row within prefetch set
  signal pf_col       : integer range 0 to BRAM_COLS-1 := 0;  -- word within row
  signal pf_col_max   : integer range 0 to BRAM_COLS-1 := 0;  -- ceil(width/8)-1
  signal pf_done_r    : std_logic := '0';
  signal pf_burst_rem : integer range 0 to 15 := 0;

  -- ---------------------------------------------------------------------------
  -- Helpers
  -- ---------------------------------------------------------------------------
  function bram_addr(row : integer; col : integer) return integer is
  begin
    return (row mod BRAM_ROWS) * BRAM_COLS + col;
  end function;

  function ddr_addr(base : unsigned(31 downto 0);
                    y    : unsigned(11 downto 0);
                    x    : unsigned(11 downto 0);
                    stride_words : unsigned(11 downto 0)) return unsigned is
    variable row_off : unsigned(31 downto 0);
    variable col_off : unsigned(31 downto 0);
    variable tmp64   : unsigned(63 downto 0);
  begin
    tmp64   := resize(y, 32) * resize(stride_words, 32);
    row_off := shift_left(tmp64(31 downto 0), 3);  -- *8 = <<3
    col_off := resize(x, 32);  -- x is already a byte offset
    return base + row_off + col_off;
  end function;

begin

  -- ---------------------------------------------------------------------------
  -- BRAM port A: synchronous write (both prefetch and wr_pixel_buf write here)
  -- ---------------------------------------------------------------------------
  -- BRAM port A write: pixel-write has priority; prefetch uses remaining cycles
  -- ---------------------------------------------------------------------------
  process(aclk)
  begin
    if rising_edge(aclk) then
      if wr_bram_en = '1' then
        bram(wr_bram_addr) <= wr_bram_din;
      elsif pf_bram_en = '1' then
        bram(pf_bram_addr) <= pf_bram_din;
      end if;
    end if;
  end process;

  -- BRAM port B: combinatorial address decode + registered data output (1-cycle latency).
  -- The previous version registered the address AND the data, giving 2-cycle latency
  -- which broke me_engine and halfpel_mc (both assume 1-cycle).
  bram_b_addr <= to_integer(unsigned(rd_addr(14 downto 9))) * BRAM_COLS
               + to_integer(unsigned(rd_addr(8 downto 0)));

  process(aclk)
  begin
    if rising_edge(aclk) then
      bram_b_dout <= bram(bram_b_addr);
    end if;
  end process;
  rd_data <= bram_b_dout;

  -- ---------------------------------------------------------------------------
  -- Write pixel buffer: collect pixels into 64-bit words, queue row to DDR
  -- ---------------------------------------------------------------------------
  wr_done <= wr_done_r;

  process(aclk)
    variable stride_words : unsigned(11 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        wr_phase   <= 0;
        wr_cnt     <= 0;
        wr_done_r  <= '0';
        wr_row_rdy <= '0';
        wr_bram_en <= '0';
      else
        wr_done_r  <= '0';
        wr_row_rdy <= '0';
        wr_bram_en <= '0';

        if wr_start = '1' then
          wr_px    <= wr_blk_x;
          wr_py    <= wr_blk_y;
          wr_phase <= 0;
          wr_row   <= 0;
          wr_cnt   <= 0;
          wr_col_word <= to_integer(wr_blk_x(11 downto 3));
        end if;

        if wr_pixel_v = '1' and wr_cnt < 64 then
          -- Pack pixel into current 64-bit word (byte 0 = leftmost)
          wr_word(wr_phase*8+7 downto wr_phase*8) <= wr_pixel;

          if wr_phase = 7 then
            -- Complete word: write to BRAM at reconstructed position
            wr_bram_en   <= '1';
            wr_bram_addr <= bram_addr(to_integer(wr_py) + wr_row, wr_col_word);
            wr_bram_din  <= wr_word(55 downto 0) & wr_pixel;  -- include last pixel

            -- Queue this word for DDR write as well
            wr_row_buf(wr_row) <= wr_word(55 downto 0) & wr_pixel;

            wr_phase <= 0;
            if wr_row = 7 then
              wr_row     <= 0;
              wr_done_r  <= '1';
              wr_row_rdy <= '1';
              wr_axi_px  <= wr_px;
              wr_axi_py  <= wr_py;
            else
              wr_row <= wr_row + 1;
            end if;
          else
            wr_word(wr_phase*8+7 downto wr_phase*8) <= wr_pixel;
            wr_phase <= wr_phase + 1;
          end if;
          wr_cnt <= wr_cnt + 1;
        end if;
      end if;
    end if;
  end process;

  -- ---------------------------------------------------------------------------
  -- AXI write FSM: write one 8-word row (64 bytes) per burst (AWLEN=7)
  -- Writes all 8 rows of the reconstructed 8x8 block sequentially.
  -- ---------------------------------------------------------------------------
  m_axi_awsize  <= "011";   -- 8 bytes per beat
  m_axi_awburst <= "01";    -- INCR
  m_axi_awlen   <= std_logic_vector(to_unsigned(0, 8));  -- 1-beat bursts (1 word = 8 pixels per row... wait, one row of 8x8 is 8 pixels = 1 word)
  m_axi_wstrb   <= (others => '1');
  m_axi_bready  <= '1';

  process(aclk)
    variable stride_bytes : unsigned(31 downto 0);
    variable row_addr     : unsigned(31 downto 0);
    variable tmp64        : unsigned(63 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        wr_axi_state  <= WR_IDLE;
        m_axi_awvalid <= '0';
        m_axi_wvalid  <= '0';
        m_axi_wlast   <= '0';
        wr_axi_row    <= 0;
      else
        case wr_axi_state is

          when WR_IDLE =>
            m_axi_awvalid <= '0';
            m_axi_wvalid  <= '0';
            if wr_row_rdy = '1' then
              wr_axi_row    <= 0;
              wr_axi_state  <= WR_ADDR;
            end if;

          when WR_ADDR =>
            -- Compute DDR address for row wr_axi_row of the block
            stride_bytes  := resize(frame_width, 32);
            -- Round stride up to 8-byte alignment (already multiple of 8 for valid widths)
            tmp64         := resize(wr_axi_py, 32) * resize(stride_bytes, 32);
            row_addr      := ref_base + tmp64(31 downto 0) + resize(wr_axi_px, 32);
            tmp64         := resize(to_unsigned(wr_axi_row, 32), 32) * resize(stride_bytes, 32);
            row_addr      := row_addr + tmp64(31 downto 0);
            m_axi_awaddr  <= std_logic_vector(row_addr);
            m_axi_awvalid <= '1';
            if m_axi_awready = '1' then
              m_axi_awvalid <= '0';
              m_axi_wdata   <= wr_row_buf(wr_axi_row);
              m_axi_wvalid  <= '1';
              m_axi_wlast   <= '1';
              wr_axi_state  <= WR_DATA;
            end if;

          when WR_DATA =>
            if m_axi_wready = '1' then
              m_axi_wvalid <= '0';
              m_axi_wlast  <= '0';
              wr_axi_state <= WR_RESP;
            end if;

          when WR_RESP =>
            if m_axi_bvalid = '1' then
              if wr_axi_row = 7 then
                wr_axi_state <= WR_IDLE;
              else
                wr_axi_row   <= wr_axi_row + 1;
                wr_axi_state <= WR_ADDR;
              end if;
            end if;

        end case;
      end if;
    end if;
  end process;

  -- ---------------------------------------------------------------------------
  -- Prefetch FSM: read BRAM_ROWS rows of reference frame from DDR into BRAM
  -- Reads one row at a time, using bursts of 16 beats (128 bytes = 16 words).
  -- ---------------------------------------------------------------------------
  prefetch_done <= pf_done_r;

  m_axi_arsize  <= "011";   -- 8 bytes per beat
  m_axi_arburst <= "01";    -- INCR

  process(aclk)
    variable stride_bytes : unsigned(31 downto 0);
    variable row_ddr_addr : unsigned(31 downto 0);
    variable cur_ref_y    : unsigned(11 downto 0);
    variable tmp64        : unsigned(63 downto 0);
    variable burst_words  : integer;
    variable words_left   : integer;
    variable col_words    : integer;  -- total words per row = ceil(width/8)
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        pf_state    <= PF_IDLE;
        pf_done_r   <= '0';
        m_axi_arvalid <= '0';
        m_axi_rready  <= '0';
      else
        pf_done_r   <= '0';
        m_axi_rready <= '0';
        pf_bram_en  <= '0';

        case pf_state is

          when PF_IDLE =>
            m_axi_arvalid <= '0';
            if prefetch_start = '1' then
              pf_ref_y   <= prefetch_ref_y;
              pf_row     <= 0;
              pf_col     <= 0;
              pf_col_max <= to_integer(frame_width(11 downto 3));  -- ceil(width/8)
              pf_state   <= PF_ADDR;
            end if;

          when PF_ADDR =>
            -- Issue read for current row at current column offset
            -- Use max burst of 16 words or remaining words in row
            col_words  := to_integer(frame_width(11 downto 3));
            words_left := col_words - pf_col;
            if words_left > 16 then
              burst_words := 16;
            else
              burst_words := words_left;
            end if;
            pf_burst_rem   <= burst_words - 1;
            stride_bytes   := resize(frame_width, 32);
            cur_ref_y      := pf_ref_y + to_unsigned(pf_row, 12);
            tmp64          := resize(cur_ref_y, 32) * resize(stride_bytes, 32);
            row_ddr_addr   := ref_base + tmp64(31 downto 0) + to_unsigned(pf_col * 8, 32);
            m_axi_araddr   <= std_logic_vector(row_ddr_addr);
            m_axi_arlen    <= std_logic_vector(to_unsigned(burst_words - 1, 8));
            m_axi_arvalid  <= '1';
            if m_axi_arready = '1' then
              m_axi_arvalid  <= '0';
              m_axi_rready   <= '1';
              pf_state       <= PF_DATA;
            end if;

          when PF_DATA =>
            m_axi_rready <= '1';
            if m_axi_rvalid = '1' then
              -- Write received word into BRAM at correct position
              pf_bram_en   <= '1';
              pf_bram_addr <= bram_addr(to_integer(pf_ref_y) + pf_row, pf_col);
              pf_bram_din  <= m_axi_rdata;

              if m_axi_rlast = '1' then
                m_axi_rready <= '0';
                -- Advance to next burst or next row
                col_words  := to_integer(frame_width(11 downto 3));
                if pf_col + pf_burst_rem + 1 >= col_words then
                  -- Row complete
                  pf_col <= 0;
                  pf_state <= PF_NEXT_ROW;
                else
                  pf_col   <= pf_col + pf_burst_rem + 1;
                  pf_state <= PF_ADDR;
                end if;
              else
                pf_col <= pf_col + 1;
              end if;
            end if;

          when PF_NEXT_ROW =>
            pf_bram_en <= '0';
            if pf_row = BRAM_ROWS - 1 then
              pf_done_r <= '1';
              pf_state  <= PF_IDLE;
            else
              pf_row   <= pf_row + 1;
              pf_col   <= 0;
              pf_state <= PF_ADDR;
            end if;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
