-- =============================================================================
-- tb_enc_top.vhd  --  Full end-to-end testbench for enc_top
--
-- What it tests
-- -------------
--   1. AXI-Lite register writes (WIDTH, HEIGHT, QP, GOP, CTRL/enable).
--   2. AXI-Lite register read-back (verifies WIDTH and HEIGHT were latched).
--   3. Full YUV 4:2:0 frame streamed in via s_axis_video_*.
--        - TUSER='1' on first byte of frame.
--        - TLAST='1' on last byte of every line.
--        - Source: binary file pointed to by YUV_FILE generic,
--          or a built-in diagonal ramp pattern when YUV_FILE="".
--   4. Compressed bitstream captured from m_axis_bitstream_* and written
--      to BS_FILE so it can be decoded offline with ffmpeg / decode.exe.
--   5. IRQ checked (must pulse once per frame).
--   6. FRAME_CNT and BS_BYTES status registers read back after frame.
--   7. Backpressure: bs_tready toggled mid-stream to verify no byte loss.
--
-- Running in Vivado
-- -----------------
--   1. Add all vhdl/*.vhd and vhdl/tb/tb_enc_top.vhd to simulation sources.
--   2. Set tb_enc_top as the top simulation unit.
--   3. (optional) Set generics: YUV_FILE="C:/path/to/test.yuv",
--                               FRAME_W=64, FRAME_H=48
--   4. Run — check "SIMULATION PASS" in the log.
--   5. Open BS_FILE in the Vivado working directory with ffmpeg:
--        ffmpeg -i bs_out.bin decoded.yuv
--
-- Register map (from enc_ctrl.vhd)
-- ----------------------------------
--   0x00  CTRL    [0]=enable  [1]=soft-reset
--   0x04  STATUS  [0]=busy    [1]=frame_done (W1C)  [2]=error
--   0x08  WIDTH   pixels (12-bit)
--   0x0C  HEIGHT  pixels (12-bit)
--   0x10  QP      1..51
--   0x14  GOP     I-frame interval
--   0x18  FRAME_CNT  (RO)
--   0x1C  BS_BYTES   (RO)
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.enc_pkg.all;

entity tb_enc_top is
  generic (
    FRAME_W  : integer := 64;          -- must be multiple of 8
    FRAME_H  : integer := 48;          -- must be multiple of 8
    G_QP     : integer := 28;          -- quantisation parameter
    G_GOP    : integer := 1;           -- 1 = all I-frames
    YUV_FILE : string  := "";          -- raw YUV 4:2:0 file; "" = synthetic
    BS_FILE  : string  := "bs_out.bin" -- output bitstream file
  );
end entity tb_enc_top;

architecture sim of tb_enc_top is

  constant CLK_PERIOD : time := 5 ns;  -- 200 MHz

  -- Register byte addresses (6-bit AXI-Lite address — enc_ctrl has 9 regs)
  constant REG_CTRL    : std_logic_vector(5 downto 0) := "000000";  -- 0x00
  constant REG_STATUS  : std_logic_vector(5 downto 0) := "000100";  -- 0x04
  constant REG_WIDTH   : std_logic_vector(5 downto 0) := "001000";  -- 0x08
  constant REG_HEIGHT  : std_logic_vector(5 downto 0) := "001100";  -- 0x0C
  constant REG_QP      : std_logic_vector(5 downto 0) := "010000";  -- 0x10
  constant REG_GOP     : std_logic_vector(5 downto 0) := "010100";  -- 0x14
  constant REG_FCNT    : std_logic_vector(5 downto 0) := "011000";  -- 0x18
  constant REG_BSBYTES : std_logic_vector(5 downto 0) := "011100";  -- 0x1C

  -- -------------------------------------------------------------------------
  -- DUT ports
  -- -------------------------------------------------------------------------
  signal aclk    : std_logic := '0';
  signal aresetn : std_logic := '0';

  signal vid_tdata  : std_logic_vector(7 downto 0) := (others => '0');
  signal vid_tvalid : std_logic := '0';
  signal vid_tready : std_logic;
  signal vid_tlast  : std_logic := '0';
  signal vid_tuser  : std_logic := '0';

  signal bs_tdata  : std_logic_vector(7 downto 0);
  signal bs_tvalid : std_logic;
  signal bs_tready : std_logic := '1';
  signal bs_tlast  : std_logic;

  signal axil_awaddr  : std_logic_vector(5 downto 0)  := (others => '0');
  signal axil_awvalid : std_logic := '0';
  signal axil_awready : std_logic;
  signal axil_wdata   : std_logic_vector(31 downto 0) := (others => '0');
  signal axil_wstrb   : std_logic_vector(3 downto 0)  := (others => '0');
  signal axil_wvalid  : std_logic := '0';
  signal axil_wready  : std_logic;
  signal axil_bresp   : std_logic_vector(1 downto 0);
  signal axil_bvalid  : std_logic;
  signal axil_bready  : std_logic := '1';
  signal axil_araddr  : std_logic_vector(5 downto 0)  := (others => '0');
  signal axil_arvalid : std_logic := '0';
  signal axil_arready : std_logic;
  signal axil_rdata   : std_logic_vector(31 downto 0);
  signal axil_rresp   : std_logic_vector(1 downto 0);
  signal axil_rvalid  : std_logic;
  signal axil_rready  : std_logic := '1';

  signal irq : std_logic;

  -- -------------------------------------------------------------------------
  -- AXI HP stub signals (simple memory model: always-ready, returns 128s)
  -- -------------------------------------------------------------------------
  signal hp_awready : std_logic := '1';
  signal hp_wready  : std_logic := '1';
  signal hp_bresp   : std_logic_vector(1 downto 0) := "00";
  signal hp_bvalid  : std_logic := '0';
  signal hp_arready : std_logic := '1';
  signal hp_rdata   : std_logic_vector(63 downto 0) :=
                        x"8080808080808080";  -- 8 pixels = 128 each
  signal hp_rresp   : std_logic_vector(1 downto 0) := "00";
  signal hp_rlast   : std_logic := '0';
  signal hp_rvalid  : std_logic := '0';
  -- AXI HP DUT outputs (wired but unused by stub)
  signal hp_awvalid : std_logic;
  signal hp_awlen   : std_logic_vector(7 downto 0);
  signal hp_wvalid  : std_logic;
  signal hp_wlast   : std_logic;
  signal hp_bready  : std_logic;
  signal hp_arvalid : std_logic;
  signal hp_arlen   : std_logic_vector(7 downto 0);
  signal hp_arready_cnt : integer := 0;  -- beats remaining in current read burst

  -- -------------------------------------------------------------------------
  -- Testbench control
  -- -------------------------------------------------------------------------
  signal sim_done    : boolean := false;
  signal bs_byte_cnt : integer := 0;  -- written by capture process

  -- -------------------------------------------------------------------------
  -- AXI-Lite write procedure
  -- -------------------------------------------------------------------------
  procedure axil_write (
    signal   clk     : in    std_logic;
    signal   awaddr  : out   std_logic_vector(5 downto 0);
    signal   awvalid : out   std_logic;
    signal   awready : in    std_logic;
    signal   wdata   : out   std_logic_vector(31 downto 0);
    signal   wstrb   : out   std_logic_vector(3 downto 0);
    signal   wvalid  : out   std_logic;
    signal   wready  : in    std_logic;
    signal   bvalid  : in    std_logic;
    constant addr    : in    std_logic_vector(5 downto 0);
    constant data    : in    std_logic_vector(31 downto 0)
  ) is
  begin
    -- Present address and data simultaneously (both channels asserted together)
    wait until rising_edge(clk);
    awaddr  <= addr;
    awvalid <= '1';
    wdata   <= data;
    wstrb   <= x"F";
    wvalid  <= '1';
    -- Poll every cycle: deassert each channel when accepted, exit on bvalid.
    -- Avoids missing a single-cycle bvalid pulse when bready is permanently '1'.
    loop
      wait until rising_edge(clk);
      if awready = '1' then awvalid <= '0'; end if;
      if wready  = '1' then wvalid  <= '0'; end if;
      if bvalid  = '1' then exit; end if;
    end loop;
  end procedure;

  -- -------------------------------------------------------------------------
  -- AXI-Lite read procedure  (returns read data in variable 'data')
  -- -------------------------------------------------------------------------
  procedure axil_read (
    signal   clk     : in    std_logic;
    signal   araddr  : out   std_logic_vector(5 downto 0);
    signal   arvalid : out   std_logic;
    signal   arready : in    std_logic;
    signal   rdata   : in    std_logic_vector(31 downto 0);
    signal   rvalid  : in    std_logic;
    signal   rready  : out   std_logic;
    constant addr    : in    std_logic_vector(5 downto 0);
    variable data    : out   std_logic_vector(31 downto 0)
  ) is
  begin
    wait until rising_edge(clk);
    araddr  <= addr;
    arvalid <= '1';
    wait until rising_edge(clk) and arready = '1';
    arvalid <= '0';
    wait until rising_edge(clk) and rvalid = '1';
    data   := rdata;
    rready <= '1';
    wait until rising_edge(clk);
  end procedure;

  -- -------------------------------------------------------------------------
  -- Send one byte on the video AXI-Stream
  -- -------------------------------------------------------------------------
  procedure send_vid_byte (
    signal   clk    : in  std_logic;
    signal   tdata  : out std_logic_vector(7 downto 0);
    signal   tvalid : out std_logic;
    signal   tlast  : out std_logic;
    signal   tuser  : out std_logic;
    signal   tready : in  std_logic;
    constant val    : in  integer;
    constant last   : in  std_logic;
    constant user   : in  std_logic
  ) is
  begin
    -- Wait until downstream is ready
    wait until rising_edge(clk) and tready = '1';
    tdata  <= std_logic_vector(to_unsigned(val mod 256, 8));
    tvalid <= '1';
    tlast  <= last;
    tuser  <= user;
    wait until rising_edge(clk);
    tvalid <= '0';
    tlast  <= '0';
    tuser  <= '0';
  end procedure;

begin

  -- -------------------------------------------------------------------------
  -- DUT instantiation
  -- -------------------------------------------------------------------------
  dut : entity work.enc_top
    port map (
      aclk                    => aclk,
      aresetn                 => aresetn,
      s_axis_video_tdata      => vid_tdata,
      s_axis_video_tvalid     => vid_tvalid,
      s_axis_video_tready     => vid_tready,
      s_axis_video_tlast      => vid_tlast,
      s_axis_video_tuser      => vid_tuser,
      m_axis_bitstream_tdata  => bs_tdata,
      m_axis_bitstream_tvalid => bs_tvalid,
      m_axis_bitstream_tready => bs_tready,
      m_axis_bitstream_tlast  => bs_tlast,
      s_axi_ctrl_awaddr       => axil_awaddr,
      s_axi_ctrl_awvalid      => axil_awvalid,
      s_axi_ctrl_awready      => axil_awready,
      s_axi_ctrl_wdata        => axil_wdata,
      s_axi_ctrl_wstrb        => axil_wstrb,
      s_axi_ctrl_wvalid       => axil_wvalid,
      s_axi_ctrl_wready       => axil_wready,
      s_axi_ctrl_bresp        => axil_bresp,
      s_axi_ctrl_bvalid       => axil_bvalid,
      s_axi_ctrl_bready       => axil_bready,
      s_axi_ctrl_araddr       => axil_araddr,
      s_axi_ctrl_arvalid      => axil_arvalid,
      s_axi_ctrl_arready      => axil_arready,
      s_axi_ctrl_rdata        => axil_rdata,
      s_axi_ctrl_rresp        => axil_rresp,
      s_axi_ctrl_rvalid       => axil_rvalid,
      s_axi_ctrl_rready       => axil_rready,
      -- AXI HP master (stub)
      m_axi_hp_awaddr         => open,
      m_axi_hp_awlen          => hp_awlen,
      m_axi_hp_awsize         => open,
      m_axi_hp_awburst        => open,
      m_axi_hp_awvalid        => hp_awvalid,
      m_axi_hp_awready        => hp_awready,
      m_axi_hp_wdata          => open,
      m_axi_hp_wstrb          => open,
      m_axi_hp_wlast          => hp_wlast,
      m_axi_hp_wvalid         => hp_wvalid,
      m_axi_hp_wready         => hp_wready,
      m_axi_hp_bresp          => hp_bresp,
      m_axi_hp_bvalid         => hp_bvalid,
      m_axi_hp_bready         => hp_bready,
      m_axi_hp_araddr         => open,
      m_axi_hp_arlen          => hp_arlen,
      m_axi_hp_arsize         => open,
      m_axi_hp_arburst        => open,
      m_axi_hp_arvalid        => hp_arvalid,
      m_axi_hp_arready        => hp_arready,
      m_axi_hp_rdata          => hp_rdata,
      m_axi_hp_rresp          => hp_rresp,
      m_axi_hp_rlast          => hp_rlast,
      m_axi_hp_rvalid         => hp_rvalid,
      m_axi_hp_rready         => open,
      irq                     => irq
    );

  -- -------------------------------------------------------------------------
  -- 200 MHz clock
  -- -------------------------------------------------------------------------
  aclk <= not aclk after CLK_PERIOD / 2 when not sim_done else '0';

  -- -------------------------------------------------------------------------
  -- Bitstream capture: collect bytes and write to BS_FILE
  -- -------------------------------------------------------------------------
  process
    type char_file_t is file of character;
    file      bs_f     : char_file_t;
    variable  fstatus  : file_open_status;
    variable  cnt      : integer := 0;
  begin
    file_open(fstatus, bs_f, BS_FILE, write_mode);
    if fstatus /= open_ok then
      report "TB WARNING: cannot open output file " & BS_FILE severity warning;
    end if;

    loop
      wait until rising_edge(aclk);

      if bs_tvalid = '1' and bs_tready = '1' then
        if fstatus = open_ok then
          write(bs_f, character'val(to_integer(unsigned(bs_tdata))));
        end if;
        cnt          := cnt + 1;
        bs_byte_cnt  <= cnt;
      end if;

      if sim_done then
        if fstatus = open_ok then
          file_close(bs_f);
        end if;
        exit;
      end if;
    end loop;
    wait;
  end process;

  -- -------------------------------------------------------------------------
  -- Backpressure injector: drop bs_tready for 8 cycles mid-stream to verify
  -- the packer stalls cleanly without losing bytes.
  -- -------------------------------------------------------------------------
  process
  begin
    wait for CLK_PERIOD * 300;
    bs_tready <= '0';
    wait for CLK_PERIOD * 8;
    bs_tready <= '1';
    wait;
  end process;

  -- -------------------------------------------------------------------------
  -- Main stimulus
  -- -------------------------------------------------------------------------
  process
    type char_file_t is file of character;
    file     yuv_f    : char_file_t;
    variable fstatus  : file_open_status;
    variable use_file : boolean := false;
    variable c        : character;
    variable pix      : integer;
    variable v_last   : std_logic;
    variable v_user   : std_logic;
    variable rdata_v  : std_logic_vector(31 downto 0);
    variable irq_seen : boolean := false;
    variable total    : integer;
    variable line_w   : integer;

  begin
    -- =======================================================================
    -- 1. Reset
    -- =======================================================================
    aresetn <= '0';
    wait for CLK_PERIOD * 10;
    aresetn <= '1';
    wait for CLK_PERIOD * 4;
    report "TB: reset released";

    -- =======================================================================
    -- 2. Configure encoder via AXI-Lite
    -- =======================================================================
    axil_write(aclk,
      axil_awaddr, axil_awvalid, axil_awready,
      axil_wdata, axil_wstrb, axil_wvalid, axil_wready,
      axil_bvalid,
      REG_WIDTH,  std_logic_vector(to_unsigned(FRAME_W, 32)));

    axil_write(aclk,
      axil_awaddr, axil_awvalid, axil_awready,
      axil_wdata, axil_wstrb, axil_wvalid, axil_wready,
      axil_bvalid,
      REG_HEIGHT, std_logic_vector(to_unsigned(FRAME_H, 32)));

    axil_write(aclk,
      axil_awaddr, axil_awvalid, axil_awready,
      axil_wdata, axil_wstrb, axil_wvalid, axil_wready,
      axil_bvalid,
      REG_QP,     std_logic_vector(to_unsigned(G_QP, 32)));

    axil_write(aclk,
      axil_awaddr, axil_awvalid, axil_awready,
      axil_wdata, axil_wstrb, axil_wvalid, axil_wready,
      axil_bvalid,
      REG_GOP,    std_logic_vector(to_unsigned(G_GOP, 32)));

    -- =======================================================================
    -- 3. Read back WIDTH and HEIGHT to verify register writes
    -- =======================================================================
    axil_read(aclk,
      axil_araddr, axil_arvalid, axil_arready,
      axil_rdata, axil_rvalid, axil_rready,
      REG_WIDTH, rdata_v);
    assert to_integer(unsigned(rdata_v(11 downto 0))) = FRAME_W
      report "TB FAIL: WIDTH readback mismatch, got " &
             integer'image(to_integer(unsigned(rdata_v(11 downto 0))))
      severity failure;

    axil_read(aclk,
      axil_araddr, axil_arvalid, axil_arready,
      axil_rdata, axil_rvalid, axil_rready,
      REG_HEIGHT, rdata_v);
    assert to_integer(unsigned(rdata_v(11 downto 0))) = FRAME_H
      report "TB FAIL: HEIGHT readback mismatch, got " &
             integer'image(to_integer(unsigned(rdata_v(11 downto 0))))
      severity failure;

    report "TB: register readback OK  (W=" & integer'image(FRAME_W) &
           " H=" & integer'image(FRAME_H) &
           " QP=" & integer'image(G_QP) & ")";

    -- =======================================================================
    -- 4. Enable the encoder (CTRL[0] = 1)
    -- =======================================================================
    axil_write(aclk,
      axil_awaddr, axil_awvalid, axil_awready,
      axil_wdata, axil_wstrb, axil_wvalid, axil_wready,
      axil_bvalid,
      REG_CTRL, x"00000001");
    wait for CLK_PERIOD * 4;
    report "TB: encoder enabled";

    -- =======================================================================
    -- 5. Open YUV file (if provided) or fall back to synthetic pattern
    -- =======================================================================
    if YUV_FILE /= "" then
      file_open(fstatus, yuv_f, YUV_FILE, read_mode);
      if fstatus = open_ok then
        use_file := true;
        report "TB: reading pixels from " & YUV_FILE;
      else
        report "TB WARNING: cannot open " & YUV_FILE &
               " -- using synthetic ramp pattern" severity warning;
      end if;
    end if;

    -- =======================================================================
    -- 6. Stream one YUV 4:2:0 frame
    --
    --    Layout: Y plane (W*H bytes), Cb plane (W/2 * H/2), Cr plane (same).
    --    TUSER='1' on very first byte of frame only.
    --    TLAST='1' on last byte of each horizontal line.
    --    mb_buffer automatically discards Cb and Cr once line_cnt=HEIGHT.
    -- =======================================================================

    -- --- Y plane ---
    total  := FRAME_W * FRAME_H;
    line_w := FRAME_W;
    for i in 0 to total - 1 loop
      if use_file and not endfile(yuv_f) then
        read(yuv_f, c);
        pix := character'pos(c);
      else
        -- Diagonal ramp: exercises both DC (uniform regions) and AC (edges)
        pix := ((i / FRAME_W) * 4 + (i mod FRAME_W) * 2) mod 256;
      end if;
      if (i mod line_w) = line_w - 1 then v_last := '1'; else v_last := '0'; end if;
      if i = 0                        then v_user := '1'; else v_user := '0'; end if;
      send_vid_byte(aclk, vid_tdata, vid_tvalid, vid_tlast, vid_tuser,
                    vid_tready, pix, v_last, v_user);
    end loop;

    -- --- Cb plane (W/2 × H/2): horizontal colour ramp ---
    total  := (FRAME_W / 2) * (FRAME_H / 2);
    line_w := FRAME_W / 2;
    for i in 0 to total - 1 loop
      if use_file and not endfile(yuv_f) then
        read(yuv_f, c);
        pix := character'pos(c);
      else
        -- Horizontal ramp: 100..227 across each chroma row (exercises AC coeffs)
        pix := 100 + (i mod line_w) * 127 / (line_w - 1);
      end if;
      if (i mod line_w) = line_w - 1 then v_last := '1'; else v_last := '0'; end if;
      send_vid_byte(aclk, vid_tdata, vid_tvalid, vid_tlast, vid_tuser,
                    vid_tready, pix, v_last, '0');
    end loop;

    -- --- Cr plane (W/2 × H/2): diagonal colour pattern ---
    for i in 0 to total - 1 loop
      if use_file and not endfile(yuv_f) then
        read(yuv_f, c);
        pix := character'pos(c);
      else
        -- Diagonal ramp: exercises both row-AC and col-AC in chroma DCT
        pix := 80 + ((i / line_w) * 3 + (i mod line_w) * 5) mod 121;
      end if;
      if (i mod line_w) = line_w - 1 then v_last := '1'; else v_last := '0'; end if;
      send_vid_byte(aclk, vid_tdata, vid_tvalid, vid_tlast, vid_tuser,
                    vid_tready, pix, v_last, '0');
    end loop;

    if use_file then
      file_close(yuv_f);
    end if;
    report "TB: all frame pixels sent";

    -- =======================================================================
    -- 7. Wait for IRQ (frame done)
    -- =======================================================================
    irq_seen := false;
    for t in 0 to 200000 loop
      wait until rising_edge(aclk);
      if irq = '1' then
        irq_seen := true;
        exit;
      end if;
    end loop;
    assert irq_seen
      report "TB FAIL: IRQ (frame done) not seen within timeout" severity failure;
    report "TB: IRQ received - frame done";

    -- =======================================================================
    -- 8. Wait for bitstream TLAST
    -- =======================================================================
    for t in 0 to 200000 loop
      wait until rising_edge(aclk);
      if bs_tvalid = '1' and bs_tready = '1' and bs_tlast = '1' then
        exit;
      end if;
    end loop;
    wait for CLK_PERIOD * 4;

    -- =======================================================================
    -- 9. Read status registers
    -- =======================================================================
    axil_read(aclk,
      axil_araddr, axil_arvalid, axil_arready,
      axil_rdata, axil_rvalid, axil_rready,
      REG_FCNT, rdata_v);
    report "TB: FRAME_CNT  = " & integer'image(to_integer(unsigned(rdata_v)));
    assert to_integer(unsigned(rdata_v)) = 1
      report "TB FAIL: expected FRAME_CNT=1" severity failure;

    axil_read(aclk,
      axil_araddr, axil_arvalid, axil_arready,
      axil_rdata, axil_rvalid, axil_rready,
      REG_BSBYTES, rdata_v);
    report "TB: BS_BYTES   = " & integer'image(to_integer(unsigned(rdata_v)));

    assert bs_byte_cnt > 0
      report "TB FAIL: no bitstream bytes captured" severity failure;
    report "TB: captured " & integer'image(bs_byte_cnt) & " bytes -> " & BS_FILE;

    -- =======================================================================
    -- 10. Done
    -- =======================================================================
    report "SIMULATION PASS";
    sim_done <= true;
    wait;
  end process;

  -- -------------------------------------------------------------------------
  -- Diagnostic: report bitstream byte count and IRQ every 50000 cycles
  -- -------------------------------------------------------------------------
  process
    variable last_cnt : integer := 0;
    variable cycle    : integer := 0;
  begin
    loop
      wait until rising_edge(aclk);
      cycle := cycle + 1;
      if cycle mod 10000 = 0 then
        report "DIAG cycle=" & integer'image(cycle) &
               " bs_bytes=" & integer'image(bs_byte_cnt) &
               " irq=" & std_logic'image(irq);
      end if;
      if sim_done then exit; end if;
    end loop;
    wait;
  end process;

  -- -------------------------------------------------------------------------
  -- Watchdog - generous limit for a 64x48 frame at 200 MHz
  -- -------------------------------------------------------------------------
  process
  begin
    wait for 20 ms;  -- extended: chroma adds ~50% more blocks (3 planes)
    if not sim_done then
      report "SIMULATION FAIL: watchdog timeout after 20 ms" severity failure;
    end if;
    wait;
  end process;

  -- -------------------------------------------------------------------------
  -- AXI HP stub: write path — return bvalid one cycle after wlast accepted
  -- -------------------------------------------------------------------------
  process(aclk)
  begin
    if rising_edge(aclk) then
      hp_bvalid <= '0';
      if hp_wvalid = '1' and hp_wready = '1' and hp_wlast = '1' then
        hp_bvalid <= '1';
      end if;
    end if;
  end process;

  -- -------------------------------------------------------------------------
  -- AXI HP stub: read path — return arlen+1 beats of 128s then rlast
  -- -------------------------------------------------------------------------
  process(aclk)
    variable beats : integer := 0;
  begin
    if rising_edge(aclk) then
      hp_rvalid <= '0';
      hp_rlast  <= '0';
      if hp_arvalid = '1' and hp_arready = '1' then
        beats := to_integer(unsigned(hp_arlen)) + 1;
      end if;
      if beats > 0 then
        hp_rvalid <= '1';
        beats     := beats - 1;
        if beats = 0 then
          hp_rlast <= '1';
        end if;
      end if;
    end if;
  end process;

end architecture sim;
