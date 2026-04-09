-- =============================================================================
-- enc_top.vhd  --  Top-level encoder IP  (I-frame + P-frame)
--
-- Port naming follows Xilinx conventions for Vivado IP Packager auto-inference.
--
-- Bitstream format (per frame)
-- ----------------------------
--   [1 bit] frame_type  (FRAME_I=0 / FRAME_P=1)
--   Per 8x8 luma block, raster order (left-to-right, strip-major):
--     I-frame: [2b mode] + ue(count) + se(dc_diff) + (count-1)*se(ac_coeff)
--              mode: 00=DC 01=HORIZ 10=VERT; dc_diff=dc_quant-prev_dc (reset at frame start)
--     P-frame: skip(1b); if skip=0: se(mv_dx) + se(mv_dy) + ue(count) + count*se(coeff)
--
-- P-frame per-block flow
-- ----------------------
--   1. mb_buffer emits 8 rows with pred=0 (raw pixels).
--   2. enc_top captures them into cur_blk_buf and simultaneously feeds
--      me_engine for LOAD.
--   3. me_engine performs diamond search + half-pixel refinement (~300 clocks).
--   4. enc_top injects skip/MV header into bitstream via bs_packer.
--   5. If not skip: halfpel_mc generates 8 MC rows; enc_top subtracts from
--      cur_blk_buf and feeds residuals to the DCT pipeline.
--   6. recon_writer reconstructs pixels → ref_frame_ddr.
--
-- AXI HP master connects to ref_frame_ddr for DDR reference frame storage.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity enc_top is
  port (
    aclk                    : in  std_logic;
    aresetn                 : in  std_logic;

    -- AXI-Stream video input
    s_axis_video_tdata      : in  std_logic_vector(7 downto 0);
    s_axis_video_tvalid     : in  std_logic;
    s_axis_video_tready     : out std_logic;
    s_axis_video_tlast      : in  std_logic;
    s_axis_video_tuser      : in  std_logic;

    -- AXI-Stream bitstream output
    m_axis_bitstream_tdata  : out std_logic_vector(7 downto 0);
    m_axis_bitstream_tvalid : out std_logic;
    m_axis_bitstream_tready : in  std_logic;
    m_axis_bitstream_tlast  : out std_logic;

    -- AXI-Lite control (6-bit address for 9 registers)
    s_axi_ctrl_awaddr       : in  std_logic_vector(5 downto 0);
    s_axi_ctrl_awvalid      : in  std_logic;
    s_axi_ctrl_awready      : out std_logic;
    s_axi_ctrl_wdata        : in  std_logic_vector(31 downto 0);
    s_axi_ctrl_wstrb        : in  std_logic_vector(3 downto 0);
    s_axi_ctrl_wvalid       : in  std_logic;
    s_axi_ctrl_wready       : out std_logic;
    s_axi_ctrl_bresp        : out std_logic_vector(1 downto 0);
    s_axi_ctrl_bvalid       : out std_logic;
    s_axi_ctrl_bready       : in  std_logic;
    s_axi_ctrl_araddr       : in  std_logic_vector(5 downto 0);
    s_axi_ctrl_arvalid      : in  std_logic;
    s_axi_ctrl_arready      : out std_logic;
    s_axi_ctrl_rdata        : out std_logic_vector(31 downto 0);
    s_axi_ctrl_rresp        : out std_logic_vector(1 downto 0);
    s_axi_ctrl_rvalid       : out std_logic;
    s_axi_ctrl_rready       : in  std_logic;

    -- AXI4 HP master (64-bit) for reference frame DDR
    m_axi_hp_awaddr         : out std_logic_vector(31 downto 0);
    m_axi_hp_awlen          : out std_logic_vector(7 downto 0);
    m_axi_hp_awsize         : out std_logic_vector(2 downto 0);
    m_axi_hp_awburst        : out std_logic_vector(1 downto 0);
    m_axi_hp_awvalid        : out std_logic;
    m_axi_hp_awready        : in  std_logic;
    m_axi_hp_wdata          : out std_logic_vector(63 downto 0);
    m_axi_hp_wstrb          : out std_logic_vector(7 downto 0);
    m_axi_hp_wlast          : out std_logic;
    m_axi_hp_wvalid         : out std_logic;
    m_axi_hp_wready         : in  std_logic;
    m_axi_hp_bresp          : in  std_logic_vector(1 downto 0);
    m_axi_hp_bvalid         : in  std_logic;
    m_axi_hp_bready         : out std_logic;
    m_axi_hp_araddr         : out std_logic_vector(31 downto 0);
    m_axi_hp_arlen          : out std_logic_vector(7 downto 0);
    m_axi_hp_arsize         : out std_logic_vector(2 downto 0);
    m_axi_hp_arburst        : out std_logic_vector(1 downto 0);
    m_axi_hp_arvalid        : out std_logic;
    m_axi_hp_arready        : in  std_logic;
    m_axi_hp_rdata          : in  std_logic_vector(63 downto 0);
    m_axi_hp_rresp          : in  std_logic_vector(1 downto 0);
    m_axi_hp_rlast          : in  std_logic;
    m_axi_hp_rvalid         : in  std_logic;
    m_axi_hp_rready         : out std_logic;

    irq                     : out std_logic
  );
end entity enc_top;

architecture rtl of enc_top is

  -- -------------------------------------------------------------------------
  -- Control register outputs
  -- -------------------------------------------------------------------------
  signal enc_enable    : std_logic;
  signal enc_reset     : std_logic;
  signal enc_width     : unsigned(11 downto 0);
  signal enc_height    : unsigned(11 downto 0);
  signal enc_qp        : unsigned(5 downto 0);
  signal enc_gop       : unsigned(7 downto 0);
  signal enc_ref_base  : unsigned(31 downto 0);
  signal int_resetn    : std_logic;

  -- -------------------------------------------------------------------------
  -- Frame control
  -- -------------------------------------------------------------------------
  signal frame_type    : std_logic;
  signal frame_sof     : std_logic;   -- s_tuser='1' detected
  signal frame_start   : std_logic;   -- one-clock pulse from frame_ctrl
  signal frame_done    : std_logic;

  -- -------------------------------------------------------------------------
  -- mb_buffer outputs
  -- -------------------------------------------------------------------------
  signal mb_dct_tdata  : std_logic_vector(71 downto 0);
  signal mb_dct_tvalid : std_logic;
  signal mb_dct_tready : std_logic;
  signal mb_pred_dc    : unsigned(7 downto 0);
  signal mb_pred_valid : std_logic;
  signal mb_strip_rdy  : std_logic;
  signal mb_blk_start  : std_logic;
  signal mb_blk_x      : unsigned(11 downto 0);
  signal mb_blk_y      : unsigned(11 downto 0);

  -- -------------------------------------------------------------------------
  -- P-frame block capture buffer (8 rows of 8 raw pixels = 64 bytes)
  -- -------------------------------------------------------------------------
  type blk_buf_t is array(0 to 7) of std_logic_vector(63 downto 0);
  signal cur_blk_buf   : blk_buf_t;
  signal cap_row       : integer range 0 to 7 := 0;
  signal blk_x_reg     : unsigned(11 downto 0) := (others => '0');
  signal blk_y_reg     : unsigned(11 downto 0) := (others => '0');

  -- -------------------------------------------------------------------------
  -- P-frame block FSM
  -- -------------------------------------------------------------------------
  type pblk_t is (PBLK_IDLE, PBLK_CAPTURE, PBLK_ME_WAIT,
                  PBLK_HDR_SKIP, PBLK_HDR_MV_X, PBLK_HDR_MV_Y,
                  PBLK_MC_EMIT, PBLK_SKIP_DONE);
  signal pblk_fsm      : pblk_t := PBLK_IDLE;

  -- me_engine I/O
  signal me_blk_start  : std_logic := '0';
  signal me_cur_row    : std_logic_vector(63 downto 0);
  signal me_cur_valid  : std_logic := '0';
  signal me_cur_idx    : integer range 0 to 7 := 0;
  signal me_done       : std_logic;
  signal me_mv         : mv_t;
  signal me_skip       : std_logic;
  signal me_ref_addr   : std_logic_vector(14 downto 0);

  -- halfpel_mc I/O
  signal mc_blk_start  : std_logic := '0';
  signal mc_row        : std_logic_vector(63 downto 0);
  signal mc_valid      : std_logic;
  signal mc_last       : std_logic;
  signal mc_ref_addr   : std_logic_vector(14 downto 0);
  signal mc_row_idx    : integer range 0 to 7 := 0;

  -- ref_frame_ddr BRAM read port mux
  signal ref_rd_addr   : std_logic_vector(14 downto 0);
  signal ref_rd_data   : std_logic_vector(63 downto 0);

  -- -------------------------------------------------------------------------
  -- DCT pipeline inputs (muxed I/P frame)
  -- -------------------------------------------------------------------------
  signal dct_in_data   : std_logic_vector(71 downto 0);
  signal dct_in_valid  : std_logic;
  signal dct_in_ready  : std_logic;

  -- dct8_fwd → quant_enc
  signal dct_q_tdata   : std_logic_vector(255 downto 0);
  signal dct_q_tvalid  : std_logic;
  signal dct_q_tready  : std_logic;
  signal dct_q_tlast   : std_logic;

  -- Post-quant row buffer: captures 8-wide quant output rows
  signal quant_out_data  : std_logic_vector(127 downto 0);  -- 8-wide quant output
  signal quant_out_valid : std_logic;
  type   qrow_buf_t      is array(0 to 7) of std_logic_vector(127 downto 0);
  signal qrow_buf        : qrow_buf_t;
  signal qrow_wr_idx     : integer range 0 to 7 := 0;
  type   qser_st_t       is (QSER_FILL, QSER_EMIT);
  signal qser_st         : qser_st_t := QSER_FILL;

  -- Wide path: qrow_buf → zigzag (128-bit, 8 rows)
  signal zz_in_data      : std_logic_vector(127 downto 0);
  signal zz_in_valid     : std_logic := '0';
  signal zz_in_tready    : std_logic;
  signal zz_emit_row     : integer range 0 to 7 := 0;
  signal zz_emit_done    : std_logic := '0';

  -- (Narrow path to recon_writer removed — recon_writer now fed directly from quant)

  -- zigzag → exp_golomb (muxed with MV SE injection)
  signal zz_eg_tdata   : std_logic_vector(15 downto 0);
  signal zz_eg_tvalid  : std_logic;
  signal zz_eg_tready  : std_logic := '1';
  signal zz_eg_tlast   : std_logic;
  signal zz_eg_tmode   : std_logic;

  -- exp_golomb input mux
  signal eg_in_data    : std_logic_vector(15 downto 0);
  signal eg_in_valid   : std_logic;
  signal eg_in_mode    : std_logic;

  -- exp_golomb → bs_packer (muxed with direct header injection)
  signal eg_cw_data    : std_logic_vector(31 downto 0);
  signal eg_cw_len     : unsigned(5 downto 0);
  signal eg_cw_valid   : std_logic;

  -- -------------------------------------------------------------------------
  -- Header injection into bs_packer
  -- -------------------------------------------------------------------------
  signal hdr_cw_data   : std_logic_vector(31 downto 0) := (others => '0');
  signal hdr_cw_len    : unsigned(5 downto 0) := (others => '0');
  signal hdr_cw_valid  : std_logic := '0';
  signal hdr_active    : std_logic := '0';  -- '1' blocks eg output to packer

  -- Pending header flags
  signal ftype_hdr_pend : std_logic := '0';  -- frame_type bit to inject
  signal skip_hdr_pend  : std_logic := '0';  -- skip bit to inject
  signal mv_x_pend      : std_logic := '0';
  signal mv_y_pend      : std_logic := '0';
  signal skip_val       : std_logic := '0';  -- current block skip value
  signal mv_reg         : mv_t;

  -- bs_packer inputs (muxed: header takes priority over eg)
  signal bs_cw_data    : std_logic_vector(31 downto 0);
  signal bs_cw_len     : unsigned(5 downto 0);
  signal bs_cw_valid   : std_logic;
  signal bs_flush      : std_logic := '0';
  signal bs_cw_ready   : std_logic;

  -- -------------------------------------------------------------------------
  -- recon_writer I/O
  -- -------------------------------------------------------------------------
  signal rw_blk_start  : std_logic := '0';
  signal rw_coeff      : std_logic_vector(127 downto 0);
  signal rw_coeff_v    : std_logic;
  signal rw_pred_row   : std_logic_vector(63 downto 0);
  signal rw_pred_row_v : std_logic := '0';
  signal rw_wr_start   : std_logic;
  signal rw_wr_blk_x   : unsigned(11 downto 0);
  signal rw_wr_blk_y   : unsigned(11 downto 0);
  signal rw_wr_pixel   : std_logic_vector(63 downto 0);
  signal rw_wr_pix_v   : std_logic;
  signal rw_recon_done : std_logic;
  signal rw_row7       : std_logic_vector(63 downto 0);
  signal rw_col7       : std_logic_vector(63 downto 0);

  -- -------------------------------------------------------------------------
  -- mb_buffer intra mode and neighbour pixel outputs
  -- -------------------------------------------------------------------------
  signal mb_intra_mode : intra_mode_t;
  signal mb_above_row  : std_logic_vector(63 downto 0);
  signal mb_left_col   : std_logic_vector(63 downto 0);

  -- -------------------------------------------------------------------------
  -- DC DPCM state (I-frame: encode dc_diff = dc_quant - prev_dc_quant)
  -- -------------------------------------------------------------------------
  signal prev_dc_q    : signed(15 downto 0) := (others => '0');
  signal await_dc     : std_logic := '0';
  signal dc_mod_data  : std_logic_vector(15 downto 0);

  -- -------------------------------------------------------------------------
  -- I-frame per-block 2-bit mode header injection
  -- -------------------------------------------------------------------------
  signal mode_hdr_pend : std_logic := '0';
  signal mode_hdr_val  : intra_mode_t := INTRA_DC;

  -- -------------------------------------------------------------------------
  -- ref_frame_ddr prefetch
  -- -------------------------------------------------------------------------
  signal prefetch_start : std_logic := '0';
  signal prefetch_ref_y : unsigned(11 downto 0) := (others => '0');
  signal prefetch_done  : std_logic;
  signal strip_prev_y   : unsigned(11 downto 0) := (others => '1');

  -- -------------------------------------------------------------------------
  -- Frame completion
  -- -------------------------------------------------------------------------
  signal enc_irq_s     : std_logic;
  signal frame_cnt     : unsigned(31 downto 0) := (others => '0');
  signal bs_byte_cnt   : unsigned(31 downto 0) := (others => '0');
  signal flush_d1      : std_logic := '0';
  signal flush_d2      : std_logic := '0';
  -- Count skip blocks (they don't produce zigzag tlast)
  signal skip_block_cnt : integer range 0 to 32767 := 0;

  -- Registered latch: captures zz_eg_tlast to give deterministic completion
  -- detection for mode-header injection (replaces eg_cw_valid polling).
  signal eg_last_latch  : std_logic := '0';

  -- -------------------------------------------------------------------------
  -- P-frame residual emission to DCT
  -- -------------------------------------------------------------------------
  signal p_res_row     : std_logic_vector(71 downto 0);  -- 8×9-bit residuals
  signal p_res_valid   : std_logic := '0';
  signal emit_row_idx  : integer range 0 to 7 := 0;

  -- -------------------------------------------------------------------------
  -- MV SE encoding (combinatorial)
  -- -------------------------------------------------------------------------
  signal inject_mv     : std_logic := '0';
  signal mv_se_val     : std_logic_vector(15 downto 0);

  -- -------------------------------------------------------------------------
  -- Encoding phase state machine (luma → Cb → Cr)
  -- -------------------------------------------------------------------------
  type enc_phase_t is (PHASE_LUMA, PHASE_WAIT_CB, PHASE_CB,
                       PHASE_WAIT_CR, PHASE_CR);
  signal enc_phase       : enc_phase_t := PHASE_LUMA;

  -- mb_buffer plane-select and chroma control
  signal mb_plane_sel    : std_logic_vector(1 downto 0) := "00";
  signal mb_cb_strip_rdy : std_logic;   -- cb_strip_rdy_out from mb_buffer
  signal mb_cr_strip_rdy : std_logic;   -- cr_strip_rdy_out from mb_buffer
  signal is_chroma_phase : std_logic;   -- '1' when enc_phase /= PHASE_LUMA

  -- Per-plane block counters
  signal luma_blk_cnt : integer range 0 to 32767 := 0;
  signal cb_blk_cnt   : integer range 0 to 8191  := 0;
  signal cr_blk_cnt   : integer range 0 to 8191  := 0;

begin

  int_resetn     <= aresetn and not enc_reset;
  irq            <= enc_irq_s;
  frame_sof      <= s_axis_video_tuser and s_axis_video_tvalid;
  is_chroma_phase <= '1' when enc_phase /= PHASE_LUMA else '0';

  -- =========================================================================
  -- AXI-Lite control registers
  -- =========================================================================
  u_ctrl : entity work.enc_ctrl
    port map (
      aclk           => aclk,
      aresetn        => aresetn,
      s_axi_awaddr   => s_axi_ctrl_awaddr,
      s_axi_awvalid  => s_axi_ctrl_awvalid,
      s_axi_awready  => s_axi_ctrl_awready,
      s_axi_wdata    => s_axi_ctrl_wdata,
      s_axi_wstrb    => s_axi_ctrl_wstrb,
      s_axi_wvalid   => s_axi_ctrl_wvalid,
      s_axi_wready   => s_axi_ctrl_wready,
      s_axi_bresp    => s_axi_ctrl_bresp,
      s_axi_bvalid   => s_axi_ctrl_bvalid,
      s_axi_bready   => s_axi_ctrl_bready,
      s_axi_araddr   => s_axi_ctrl_araddr,
      s_axi_arvalid  => s_axi_ctrl_arvalid,
      s_axi_arready  => s_axi_ctrl_arready,
      s_axi_rdata    => s_axi_ctrl_rdata,
      s_axi_rresp    => s_axi_ctrl_rresp,
      s_axi_rvalid   => s_axi_ctrl_rvalid,
      s_axi_rready   => s_axi_ctrl_rready,
      enc_enable     => enc_enable,
      enc_reset      => enc_reset,
      enc_width      => enc_width,
      enc_height     => enc_height,
      enc_qp         => enc_qp,
      enc_gop        => enc_gop,
      enc_ref_base   => enc_ref_base,
      enc_busy       => enc_enable,
      enc_frame_done => frame_done,
      enc_error      => '0',
      enc_frame_cnt  => frame_cnt,
      enc_bs_bytes   => bs_byte_cnt,
      enc_irq        => enc_irq_s
    );

  -- =========================================================================
  -- GOP / frame type controller
  -- =========================================================================
  u_fctrl : entity work.frame_ctrl
    port map (
      aclk        => aclk,
      aresetn     => int_resetn,
      gop_size    => enc_gop,
      frame_sof   => frame_sof,
      frame_done  => frame_done,
      frame_type  => frame_type,
      frame_start => frame_start,
      frame_end   => open,
      frame_count => open
    );

  -- =========================================================================
  -- Macroblock buffer
  -- =========================================================================
  u_mb : entity work.mb_buffer
    port map (
      aclk             => aclk,
      aresetn          => int_resetn,
      frame_width      => enc_width,
      frame_height     => enc_height,
      enc_enable       => enc_enable,
      frame_type       => frame_type,
      plane_sel        => mb_plane_sel,
      chroma_width     => '0' & enc_width(11 downto 1),
      chroma_height    => '0' & enc_height(11 downto 1),
      chroma_plane_rst => '0',
      s_tdata          => s_axis_video_tdata,
      s_tvalid         => s_axis_video_tvalid,
      s_tready         => s_axis_video_tready,
      s_tlast          => s_axis_video_tlast,
      s_tuser          => s_axis_video_tuser,
      m_tdata          => mb_dct_tdata,
      m_tvalid         => mb_dct_tvalid,
      m_tready         => mb_dct_tready,
      m_pred_dc        => mb_pred_dc,
      m_pred_valid     => mb_pred_valid,
      m_intra_mode     => mb_intra_mode,
      m_above_row      => mb_above_row,
      m_left_col       => mb_left_col,
      strip_rdy_out    => mb_strip_rdy,
      cb_strip_rdy_out => mb_cb_strip_rdy,
      cr_strip_rdy_out => mb_cr_strip_rdy,
      mb_blk_start     => mb_blk_start,
      mb_blk_x         => mb_blk_x,
      mb_blk_y         => mb_blk_y,
      recon_done       => rw_recon_done,
      recon_row7       => rw_row7,
      recon_col7       => rw_col7
    );

  -- =========================================================================
  -- Reference frame DDR manager + search-window BRAM
  -- =========================================================================
  u_ref : entity work.ref_frame_ddr
    port map (
      aclk           => aclk,
      aresetn        => int_resetn,
      frame_width    => enc_width,
      frame_height   => enc_height,
      ref_base       => enc_ref_base,
      wr_start       => rw_wr_start,
      wr_blk_x       => rw_wr_blk_x,
      wr_blk_y       => rw_wr_blk_y,
      wr_pixel       => rw_wr_pixel,
      wr_pixel_v     => rw_wr_pix_v,
      wr_done        => open,
      prefetch_start => prefetch_start,
      prefetch_ref_y => prefetch_ref_y,
      prefetch_done  => prefetch_done,
      rd_addr        => ref_rd_addr,
      rd_data        => ref_rd_data,
      m_axi_awaddr   => m_axi_hp_awaddr,
      m_axi_awlen    => m_axi_hp_awlen,
      m_axi_awsize   => m_axi_hp_awsize,
      m_axi_awburst  => m_axi_hp_awburst,
      m_axi_awvalid  => m_axi_hp_awvalid,
      m_axi_awready  => m_axi_hp_awready,
      m_axi_wdata    => m_axi_hp_wdata,
      m_axi_wstrb    => m_axi_hp_wstrb,
      m_axi_wlast    => m_axi_hp_wlast,
      m_axi_wvalid   => m_axi_hp_wvalid,
      m_axi_wready   => m_axi_hp_wready,
      m_axi_bresp    => m_axi_hp_bresp,
      m_axi_bvalid   => m_axi_hp_bvalid,
      m_axi_bready   => m_axi_hp_bready,
      m_axi_araddr   => m_axi_hp_araddr,
      m_axi_arlen    => m_axi_hp_arlen,
      m_axi_arsize   => m_axi_hp_arsize,
      m_axi_arburst  => m_axi_hp_arburst,
      m_axi_arvalid  => m_axi_hp_arvalid,
      m_axi_arready  => m_axi_hp_arready,
      m_axi_rdata    => m_axi_hp_rdata,
      m_axi_rresp    => m_axi_hp_rresp,
      m_axi_rlast    => m_axi_hp_rlast,
      m_axi_rvalid   => m_axi_hp_rvalid,
      m_axi_rready   => m_axi_hp_rready
    );

  -- =========================================================================
  -- Motion estimation engine
  -- =========================================================================
  u_me : entity work.me_engine
    port map (
      aclk         => aclk,
      aresetn      => int_resetn,
      frame_width  => enc_width,
      frame_height => enc_height,
      blk_start    => me_blk_start,
      blk_x        => blk_x_reg,
      blk_y        => blk_y_reg,
      cur_row      => me_cur_row,
      cur_valid    => me_cur_valid,
      ref_rd_addr  => me_ref_addr,
      ref_rd_data  => ref_rd_data,
      me_done      => me_done,
      mv_out       => me_mv,
      skip_out     => me_skip
    );

  -- =========================================================================
  -- Half-pixel motion compensation
  -- =========================================================================
  u_mc : entity work.halfpel_mc
    port map (
      aclk         => aclk,
      aresetn      => int_resetn,
      frame_width  => enc_width,
      frame_height => enc_height,
      blk_start    => mc_blk_start,
      blk_x        => blk_x_reg,
      blk_y        => blk_y_reg,
      mv           => mv_reg,
      ref_rd_addr  => mc_ref_addr,
      ref_rd_data  => ref_rd_data,
      mc_row       => mc_row,
      mc_valid     => mc_valid,
      mc_last      => mc_last
    );

  -- BRAM read-port mux: MC has priority during PBLK_MC_EMIT
  ref_rd_addr <= mc_ref_addr when pblk_fsm = PBLK_MC_EMIT else me_ref_addr;

  -- =========================================================================
  -- Reconstruction writer (dequant + IDCT + pred + DDR write)
  -- =========================================================================
  u_recon : entity work.recon_writer
    port map (
      aclk        => aclk,
      aresetn     => int_resetn,
      qp          => enc_qp,
      is_intra    => (not frame_type) or is_chroma_phase,
      coeff_in    => rw_coeff,    -- 128-bit: 8 quantised coefficients per clock
      coeff_valid => rw_coeff_v,
      blk_start   => rw_blk_start,
      pred_dc        => mb_pred_dc,
      pred_use_dc    => (not frame_type) or is_chroma_phase,
      pred_above_row => mb_above_row,
      pred_left_col  => mb_left_col,
      intra_mode     => mb_intra_mode,
      pred_row       => rw_pred_row,
      pred_row_v     => rw_pred_row_v,
      blk_x          => blk_x_reg,
      blk_y          => blk_y_reg,
      wr_start       => rw_wr_start,
      wr_blk_x       => rw_wr_blk_x,
      wr_blk_y       => rw_wr_blk_y,
      wr_pixel       => rw_wr_pixel,
      wr_pixel_v     => rw_wr_pix_v,
      recon_done     => rw_recon_done,
      recon_row7     => rw_row7,
      recon_col7     => rw_col7
    );

  -- Feed MC rows to recon_writer pred buffer during P-frame MC emission
  rw_pred_row   <= mc_row;
  rw_pred_row_v <= mc_valid when frame_type = FRAME_P else '0';

  -- =========================================================================
  -- DCT pipeline
  -- =========================================================================
  u_dct : entity work.dct8_fwd
    port map (
      aclk      => aclk,
      aresetn   => int_resetn,
      s_tdata   => dct_in_data,
      s_tvalid  => dct_in_valid,
      s_tready  => dct_in_ready,
      m_tdata   => dct_q_tdata,
      m_tvalid  => dct_q_tvalid,
      m_tlast   => dct_q_tlast,
      m_tready  => dct_q_tready
    );

  -- DCT input mux:
  --   I-frame luma or any chroma phase → mb_buffer residuals
  --   P-frame luma                     → computed residuals (p_res_row)
  dct_in_data  <= mb_dct_tdata  when (frame_type = FRAME_I or is_chroma_phase = '1')
                                else p_res_row;
  dct_in_valid <= mb_dct_tvalid when (frame_type = FRAME_I or is_chroma_phase = '1')
                                else p_res_valid;
  mb_dct_tready <= dct_in_ready when (frame_type = FRAME_I or is_chroma_phase = '1')
                                else '0';

  -- 8-wide quant accepts every DCT row immediately — no backpressure needed
  dct_q_tready <= '1';

  -- =========================================================================
  -- Post-quant buffer + zigzag EMIT
  --
  -- FILL phase (8 clocks): stores 8 × 128-bit quant rows into qrow_buf.
  --   Simultaneously, quant output is forwarded directly to recon_writer
  --   (128-bit/row, no buffering needed for that path).
  --
  -- EMIT phase: Wide path → zigzag (128-bit/row, 8 clocks, gated on tready).
  --   Returns to FILL when zigzag has accepted all 8 rows.
  -- =========================================================================
  process(aclk)
  begin
    if rising_edge(aclk) then
      if int_resetn = '0' then
        qrow_wr_idx  <= 0;
        qser_st      <= QSER_FILL;
        zz_in_valid  <= '0';
        zz_in_data   <= (others => '0');
        zz_emit_row  <= 0;
        zz_emit_done <= '0';
      else
        zz_in_valid <= '0';

        case qser_st is

          -- ----------------------------------------------------------------
          when QSER_FILL =>
            if quant_out_valid = '1' then
              qrow_buf(qrow_wr_idx) <= quant_out_data;
              if qrow_wr_idx = 7 then
                qrow_wr_idx  <= 0;
                zz_emit_row  <= 0;
                zz_emit_done <= '0';
                qser_st      <= QSER_EMIT;
              else
                qrow_wr_idx <= qrow_wr_idx + 1;
              end if;
            end if;

          -- ----------------------------------------------------------------
          when QSER_EMIT =>
            -- Wide path: emit one 128-bit row per clock to zigzag
            if zz_emit_done = '0' then
              if zz_in_tready = '1' then
                zz_in_data  <= qrow_buf(zz_emit_row);
                zz_in_valid <= '1';
                if zz_emit_row = 7 then
                  zz_emit_done <= '1';
                else
                  zz_emit_row <= zz_emit_row + 1;
                end if;
              end if;
            end if;

            -- Return to FILL when zigzag has accepted all 8 rows
            if zz_emit_done = '1' then
              qser_st <= QSER_FILL;
            end if;

        end case;
      end if;
    end if;
  end process;

  -- recon_writer fed directly from quant output (128-bit rows, no buffering)
  rw_coeff   <= quant_out_data;
  rw_coeff_v <= quant_out_valid;

  -- =========================================================================
  -- DC DPCM: replace DC coefficient with delta from previous block's DC
  -- When await_dc='1' and the current zigzag token is the DC SE coefficient,
  -- output (dc_quant - prev_dc_q) instead of dc_quant.
  -- =========================================================================
  dc_mod_data <= std_logic_vector(signed(zz_eg_tdata) - prev_dc_q)
                 when (await_dc = '1' and zz_eg_tmode = '0') else zz_eg_tdata;

  process(aclk)
  begin
    if rising_edge(aclk) then
      if int_resetn = '0' or frame_start = '1' or
       enc_phase = PHASE_WAIT_CB or enc_phase = PHASE_WAIT_CR then
        prev_dc_q <= (others => '0');
        await_dc  <= '0';
      elsif zz_eg_tvalid = '1' and zz_eg_tready = '1' then
        if zz_eg_tmode = '1' then          -- UE count token
          if unsigned(zz_eg_tdata) = 0 then
            prev_dc_q <= (others => '0');  -- dc_quant=0, update prev
            await_dc  <= '0';
          else
            await_dc  <= '1';              -- next SE is the DC coefficient
          end if;
        elsif await_dc = '1' then          -- SE DC token (tmode='0')
          prev_dc_q <= signed(zz_eg_tdata);  -- store actual DC (before diff)
          await_dc  <= '0';
        end if;
      end if;
    end if;
  end process;

  -- =========================================================================
  -- Quantiser
  -- =========================================================================
  u_quant : entity work.quant_enc
    port map (
      aclk      => aclk,
      aresetn   => int_resetn,
      qp        => enc_qp,
      is_intra  => not frame_type,
      s_tdata   => dct_q_tdata,
      s_tvalid  => dct_q_tvalid,
      s_tready  => open,
      m_tdata   => quant_out_data,
      m_tvalid  => quant_out_valid
    );

  -- =========================================================================
  -- Zigzag reorder
  -- =========================================================================
  u_zigzag : entity work.zigzag
    port map (
      aclk      => aclk,
      aresetn   => int_resetn,
      s_tdata   => zz_in_data,
      s_tvalid  => zz_in_valid,
      s_tready  => zz_in_tready,
      m_tdata   => zz_eg_tdata,
      m_tvalid  => zz_eg_tvalid,
      m_tlast   => zz_eg_tlast,
      m_tmode   => zz_eg_tmode,
      m_tready  => zz_eg_tready
    );

  -- Hold zigzag output when a header is being injected or packer is full
  zz_eg_tready <= bs_cw_ready when (hdr_active = '0' and inject_mv = '0') else '0';

  -- =========================================================================
  -- Exp-Golomb entropy coder input mux (residuals or MV SE)
  -- =========================================================================
  eg_in_data  <= mv_se_val    when inject_mv = '1' else dc_mod_data;
  eg_in_valid <= inject_mv    when inject_mv = '1' else zz_eg_tvalid;
  eg_in_mode  <= '0'          when inject_mv = '1' else zz_eg_tmode;

  u_eg : entity work.exp_golomb
    port map (
      aclk       => aclk,
      aresetn    => int_resetn,
      s_tdata    => eg_in_data,
      s_tvalid   => eg_in_valid,
      s_tready   => open,
      s_tmode    => eg_in_mode,
      m_codeword => eg_cw_data,
      m_length   => eg_cw_len,
      m_tvalid   => eg_cw_valid
    );

  -- =========================================================================
  -- Bitstream packer input mux (headers take priority)
  -- =========================================================================
  bs_cw_data  <= hdr_cw_data when hdr_active = '1' else eg_cw_data;
  bs_cw_len   <= hdr_cw_len  when hdr_active = '1' else eg_cw_len;
  bs_cw_valid <= hdr_cw_valid when hdr_active = '1' else
                 eg_cw_valid  when hdr_active = '0' else '0';

  u_packer : entity work.bs_packer
    port map (
      aclk      => aclk,
      aresetn   => int_resetn,
      cw_data   => bs_cw_data,
      cw_len    => bs_cw_len,
      cw_valid  => bs_cw_valid,
      cw_ready  => bs_cw_ready,
      flush     => bs_flush,
      m_tdata   => m_axis_bitstream_tdata,
      m_tvalid  => m_axis_bitstream_tvalid,
      m_tlast   => m_axis_bitstream_tlast,
      m_tready  => m_axis_bitstream_tready
    );

  -- =========================================================================
  -- Prefetch trigger: when a new strip row starts, load reference rows into BRAM
  -- Prefetch covers SEARCH_RANGE rows above the current strip.
  -- =========================================================================
  process(aclk)
    variable ref_top : signed(12 downto 0);
  begin
    if rising_edge(aclk) then
      if int_resetn = '0' then
        prefetch_start <= '0';
        prefetch_ref_y <= (others => '0');
        strip_prev_y   <= (others => '1');
      else
        prefetch_start <= '0';
        if mb_strip_rdy = '1' and mb_blk_y /= strip_prev_y then
          strip_prev_y   <= mb_blk_y;
          -- Load rows starting SEARCH_RANGE above current strip (clipped to 0)
          ref_top := to_signed(to_integer(mb_blk_y), 13)
                   - to_signed(SEARCH_RANGE, 13);
          if ref_top < 0 then
            prefetch_ref_y <= (others => '0');
          else
            prefetch_ref_y <= unsigned(ref_top(11 downto 0));
          end if;
          prefetch_start <= '1';
        end if;
      end if;
    end if;
  end process;

  -- =========================================================================
  -- P-frame block FSM: capture → ME → header inject → MC/emit residuals
  -- =========================================================================
  process(aclk)
    variable raw_pix : unsigned(7 downto 0);
    variable mc_pix  : unsigned(7 downto 0);
    variable res     : signed(8 downto 0);
    variable row_out : std_logic_vector(71 downto 0);
  begin
    if rising_edge(aclk) then
      if int_resetn = '0' then
        pblk_fsm      <= PBLK_IDLE;
        me_blk_start  <= '0';
        me_cur_valid  <= '0';
        me_cur_idx    <= 0;
        mc_blk_start  <= '0';
        p_res_valid   <= '0';
        hdr_cw_valid  <= '0';
        hdr_active    <= '0';
        inject_mv     <= '0';
        ftype_hdr_pend <= '0';
        skip_hdr_pend  <= '0';
        mv_x_pend      <= '0';
        mv_y_pend      <= '0';
        mode_hdr_pend  <= '0';
        eg_last_latch  <= '0';
        rw_blk_start   <= '0';
        emit_row_idx   <= 0;
        cap_row        <= 0;
      else
        -- Defaults
        me_blk_start  <= '0';
        me_cur_valid  <= '0';
        mc_blk_start  <= '0';
        p_res_valid   <= '0';
        hdr_cw_valid  <= '0';
        hdr_active    <= '0';
        inject_mv     <= '0';
        rw_blk_start  <= '0';

        -- Frame type header: inject 1-bit frame_type at start of each frame
        if ftype_hdr_pend = '1' then
          hdr_active    <= '1';
          hdr_cw_data   <= frame_type & (30 downto 0 => '0');
          hdr_cw_len    <= to_unsigned(1, 6);
          hdr_cw_valid  <= '1';
          ftype_hdr_pend <= '0';
        end if;

        if frame_start = '1' then
          ftype_hdr_pend <= '1';
          mode_hdr_pend  <= '0';  -- reset at new frame
          eg_last_latch  <= '0';
        end if;

        -- Latch zz_eg_tlast: set when the last zigzag token for a block is
        -- consumed by exp-golomb.  Provides a definitive "block done" marker
        -- that cannot false-trigger mid-block (unlike bare eg_cw_valid='0').
        if zz_eg_tvalid = '1' and zz_eg_tlast = '1' and zz_eg_tready = '1' then
          eg_last_latch <= '1';
        end if;

        -- I-frame block mode header: inject 2-bit intra mode before ue(count).
        -- The latch ensures we wait for the previous block's zigzag stream to
        -- fully enter exp-golomb; eg_cw_valid='0' confirms the last codeword
        -- has been accepted by bs_packer.  Together they give deterministic,
        -- race-free timing (eliminates the 0-2 cy jitter of the old poll).
        if mode_hdr_pend = '1' and ftype_hdr_pend = '0'
           and eg_cw_valid = '0' and eg_last_latch = '1' then
          hdr_active    <= '1';
          hdr_cw_data   <= mode_hdr_val & (29 downto 0 => '0');
          hdr_cw_len    <= to_unsigned(2, 6);
          hdr_cw_valid  <= '1';
          mode_hdr_pend <= '0';
          eg_last_latch <= '0';
        end if;

        -- ---------------------------------------------------------------
        -- P-frame block FSM
        -- ---------------------------------------------------------------
        case pblk_fsm is

          when PBLK_IDLE =>
            if frame_type = FRAME_P and mb_blk_start = '1' then
              blk_x_reg    <= mb_blk_x;
              blk_y_reg    <= mb_blk_y;
              cap_row      <= 0;
              rw_blk_start <= '1';
              -- Fire me_blk_start HERE (CALC state), so me_engine enters LOAD
              -- by the time mb_buffer starts EMIT next cycle.
              me_blk_start <= '1';
              pblk_fsm     <= PBLK_CAPTURE;
            elsif frame_type = FRAME_I and mb_blk_start = '1' then
              -- I-frame: register block position and queue 2-bit mode header
              blk_x_reg     <= mb_blk_x;
              blk_y_reg     <= mb_blk_y;
              rw_blk_start  <= '1';
              mode_hdr_pend <= '1';
              mode_hdr_val  <= mb_intra_mode;
            end if;

          when PBLK_CAPTURE =>
            -- Accept mb_buffer rows (raw pixels) and capture into cur_blk_buf.
            -- mb_dct_tvalid pulses once per row; DCT is stalled (mb_dct_tready='0'
            -- because frame_type=FRAME_P gates it off).
            -- me_engine is already in LOAD state (me_blk_start fired last cycle).
            if mb_dct_tvalid = '1' then
              -- Extract 8×8-bit pixels from 8×9-bit packed fields.
              -- In P-mode pred=0, so each 9-bit field's sign bit is always 0;
              -- bits [i*9+7 : i*9] are the raw pixel byte.
              for i in 0 to 7 loop
                cur_blk_buf(cap_row)(i*8+7 downto i*8) <=
                  mb_dct_tdata(i*9+7 downto i*9);
                me_cur_row(i*8+7 downto i*8) <=
                  mb_dct_tdata(i*9+7 downto i*9);
              end loop;
              me_cur_valid <= '1';
              if cap_row = 7 then
                cap_row  <= 0;
                pblk_fsm <= PBLK_ME_WAIT;
              else
                cap_row <= cap_row + 1;
              end if;
            end if;

          when PBLK_ME_WAIT =>
            if me_done = '1' then
              skip_val  <= me_skip;
              mv_reg    <= me_mv;
              pblk_fsm  <= PBLK_HDR_SKIP;
            end if;

          when PBLK_HDR_SKIP =>
            -- Inject 1-bit skip flag
            hdr_active   <= '1';
            hdr_cw_data  <= me_skip & (30 downto 0 => '0');
            hdr_cw_len   <= to_unsigned(1, 6);
            hdr_cw_valid <= '1';
            if me_skip = '1' then
              pblk_fsm <= PBLK_SKIP_DONE;
            else
              pblk_fsm <= PBLK_HDR_MV_X;
            end if;

          when PBLK_HDR_MV_X =>
            -- Inject se(mv_dx) via exp_golomb
            inject_mv  <= '1';
            mv_se_val  <= std_logic_vector(resize(mv_reg.dx, 16));
            pblk_fsm   <= PBLK_HDR_MV_Y;

          when PBLK_HDR_MV_Y =>
            inject_mv  <= '1';
            mv_se_val  <= std_logic_vector(resize(mv_reg.dy, 16));
            emit_row_idx <= 0;
            mc_blk_start <= '1';    -- start halfpel_mc
            pblk_fsm   <= PBLK_MC_EMIT;

          when PBLK_MC_EMIT =>
            -- When mc_valid: subtract mc from cur_blk_buf row, emit to DCT
            if mc_valid = '1' then
              row_out := (others => '0');
              for i in 0 to 7 loop
                raw_pix := unsigned(cur_blk_buf(emit_row_idx)(i*8+7 downto i*8));
                mc_pix  := unsigned(mc_row(i*8+7 downto i*8));
                res     := signed('0' & raw_pix) - signed('0' & mc_pix);
                row_out(i*9+8 downto i*9) := std_logic_vector(res);
              end loop;
              p_res_row   <= row_out;
              p_res_valid <= '1';
              if mc_last = '1' then
                emit_row_idx <= 0;
                pblk_fsm     <= PBLK_IDLE;
              else
                emit_row_idx <= emit_row_idx + 1;
              end if;
            end if;

          when PBLK_SKIP_DONE =>
            -- Skip block: no residual, count it and return to IDLE
            pblk_fsm <= PBLK_IDLE;

        end case;
      end if;
    end if;
  end process;

  -- =========================================================================
  -- recon_writer blk_start for I-frame: pulse from mb_blk_start
  -- (For P-frame, rw_blk_start is driven from the P-frame FSM above)
  -- =========================================================================

  -- =========================================================================
  -- Frame-done detector + IRQ + byte counter + encoding phase FSM
  -- All in one process to avoid multiple-driver conflicts on frame_done / flush.
  -- Phase order: LUMA → WAIT_CB → CB → WAIT_CR → CR → (LUMA next frame).
  -- frame_done fires after the last Cr block is fully encoded.
  -- =========================================================================
  process(aclk)
    variable n_luma   : integer range 1 to 32400;
    variable n_chroma : integer range 1 to 16384;
  begin
    if rising_edge(aclk) then
      if int_resetn = '0' then
        enc_phase      <= PHASE_LUMA;
        mb_plane_sel   <= "00";
        luma_blk_cnt   <= 0;
        cb_blk_cnt     <= 0;
        cr_blk_cnt     <= 0;
        skip_block_cnt <= 0;
        frame_done     <= '0';
        frame_cnt      <= (others => '0');
        bs_byte_cnt    <= (others => '0');
        flush_d1       <= '0';
        flush_d2       <= '0';
        bs_flush       <= '0';
      else
        frame_done    <= '0';
        bs_flush      <= flush_d2;
        flush_d2      <= flush_d1;
        flush_d1      <= '0';

        -- Reset phase at start of new frame
        if frame_start = '1' then
          enc_phase    <= PHASE_LUMA;
          mb_plane_sel <= "00";
        end if;

        if m_axis_bitstream_tvalid = '1' and m_axis_bitstream_tready = '1' then
          bs_byte_cnt <= bs_byte_cnt + 1;
        end if;

        n_luma   := to_integer(enc_width(11 downto 3)) *
                    to_integer(enc_height(11 downto 3));
        -- Chroma: (W/2)/8 × (H/2)/8 = W/16 × H/16
        n_chroma := to_integer(enc_width(11 downto 4)) *
                    to_integer(enc_height(11 downto 4));

        -- Steady-state phase output (overridden by transitions below)
        case enc_phase is
          when PHASE_WAIT_CB =>
            -- One dead cycle: advance to CB
            enc_phase    <= PHASE_CB;
            mb_plane_sel <= "01";
          when PHASE_WAIT_CR =>
            enc_phase    <= PHASE_CR;
            mb_plane_sel <= "10";
          when others => null;
        end case;

        -- Count completed residual blocks (zigzag tlast) and trigger transitions
        if zz_eg_tvalid = '1' and zz_eg_tlast = '1' then
          case enc_phase is
            when PHASE_LUMA =>
              if luma_blk_cnt + skip_block_cnt + 1 = n_luma then
                luma_blk_cnt   <= 0;
                skip_block_cnt <= 0;
                enc_phase      <= PHASE_WAIT_CB;
                mb_plane_sel   <= "01";
              else
                luma_blk_cnt <= luma_blk_cnt + 1;
              end if;
            when PHASE_CB =>
              if cb_blk_cnt + 1 = n_chroma then
                cb_blk_cnt    <= 0;
                enc_phase     <= PHASE_WAIT_CR;
                mb_plane_sel  <= "10";
              else
                cb_blk_cnt <= cb_blk_cnt + 1;
              end if;
            when PHASE_CR =>
              if cr_blk_cnt + 1 = n_chroma then
                cr_blk_cnt   <= 0;
                enc_phase    <= PHASE_LUMA;
                mb_plane_sel <= "00";
                frame_done   <= '1';
                frame_cnt    <= frame_cnt + 1;
                flush_d1     <= '1';
              else
                cr_blk_cnt <= cr_blk_cnt + 1;
              end if;
            when others => null;
          end case;
        end if;

        -- Skip blocks (P-frame luma only)
        if pblk_fsm = PBLK_SKIP_DONE then
          if luma_blk_cnt + skip_block_cnt + 1 = n_luma then
            luma_blk_cnt   <= 0;
            skip_block_cnt <= 0;
            enc_phase      <= PHASE_WAIT_CB;
            mb_plane_sel   <= "01";
          else
            skip_block_cnt <= skip_block_cnt + 1;
          end if;
        end if;

      end if;
    end if;
  end process;

  -- ===========================================================================
  -- Simulation-only: block throughput measurement
  -- Reports avg/min/max cycles between consecutive mb_blk_start pulses.
  -- Stripped out by synthesis (pragma translate_off/on).
  -- ===========================================================================
  -- pragma translate_off
  blk_timing : process(aclk)
    variable blk_count    : integer := 0;
    variable last_cycle   : integer := 0;
    variable cycle_cnt    : integer := 0;
    variable total_cycles : integer := 0;
    variable min_gap      : integer := 999999;
    variable max_gap      : integer := 0;
    variable gap          : integer;
  begin
    if rising_edge(aclk) then
      cycle_cnt := cycle_cnt + 1;
      if mb_blk_start = '1' then
        if blk_count > 0 then
          gap := cycle_cnt - last_cycle;
          total_cycles := total_cycles + gap;
          if gap < min_gap then min_gap := gap; end if;
          if gap > max_gap then max_gap := gap; end if;
        end if;
        last_cycle := cycle_cnt;
        blk_count  := blk_count + 1;
      end if;
      if frame_done = '1' and blk_count > 1 then
        report "BLK_THRU: n=" & integer'image(blk_count) &
               "  avg=" & integer'image(total_cycles / (blk_count - 1)) &
               "  min=" & integer'image(min_gap) &
               "  max=" & integer'image(max_gap) & " cy/blk"
               severity note;
        blk_count    := 0;
        total_cycles := 0;
        min_gap      := 999999;
        max_gap      := 0;
      end if;
    end if;
  end process blk_timing;
  -- pragma translate_on

end architecture rtl;
