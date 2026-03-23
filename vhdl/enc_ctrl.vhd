-- =============================================================================
-- enc_ctrl.vhd  --  AXI-Lite control register block
--
-- Provides 8 memory-mapped registers accessible from the Zynq PS over AXI-Lite.
-- Vivado's Package IP wizard will auto-infer the AXI-Lite interface from the
-- s_axi_* port naming convention.
--
-- Register map (32-bit word addresses, byte offset in parentheses)
-- ----------------------------------------------------------------
--   0x00  CTRL        [0]=enable  [1]=soft-reset (auto-clears)
--   0x04  STATUS      [0]=busy    [1]=frame_done (W1C)  [2]=error
--   0x08  WIDTH       frame width  in pixels (12-bit, 16..3840)
--   0x0C  HEIGHT      frame height in pixels (12-bit, 16..2160)
--   0x10  QP          quantisation parameter (6-bit, 1..51)
--   0x14  GOP_SIZE    I-frame interval (8-bit, 1=I-only)
--   0x18  FRAME_CNT   frames encoded since enable (RO, 32-bit)
--   0x1C  BS_BYTES    bitstream bytes in most recent frame (RO, 32-bit)
--   0x20  REF_BASE    DDR base address of reference frame buffer (RW, 32-bit)
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity enc_ctrl is
  port (
    aclk      : in  std_logic;
    aresetn   : in  std_logic;

    -- AXI-Lite slave (Vivado auto-infers interface from s_axi_ prefix)
    s_axi_awaddr  : in  std_logic_vector(5 downto 0);
    s_axi_awvalid : in  std_logic;
    s_axi_awready : out std_logic;
    s_axi_wdata   : in  std_logic_vector(31 downto 0);
    s_axi_wstrb   : in  std_logic_vector(3 downto 0);
    s_axi_wvalid  : in  std_logic;
    s_axi_wready  : out std_logic;
    s_axi_bresp   : out std_logic_vector(1 downto 0);
    s_axi_bvalid  : out std_logic;
    s_axi_bready  : in  std_logic;
    s_axi_araddr  : in  std_logic_vector(5 downto 0);
    s_axi_arvalid : in  std_logic;
    s_axi_arready : out std_logic;
    s_axi_rdata   : out std_logic_vector(31 downto 0);
    s_axi_rresp   : out std_logic_vector(1 downto 0);
    s_axi_rvalid  : out std_logic;
    s_axi_rready  : in  std_logic;

    -- Register outputs to encoder datapath
    enc_enable    : out std_logic;
    enc_reset     : out std_logic;
    enc_width     : out unsigned(11 downto 0);
    enc_height    : out unsigned(11 downto 0);
    enc_qp        : out unsigned(5 downto 0);
    enc_gop       : out unsigned(7 downto 0);
    enc_ref_base  : out unsigned(31 downto 0);  -- DDR base address for reference frame

    -- Status inputs from encoder datapath
    enc_busy      : in  std_logic;
    enc_frame_done: in  std_logic;
    enc_error     : in  std_logic;
    enc_frame_cnt : in  unsigned(31 downto 0);
    enc_bs_bytes  : in  unsigned(31 downto 0);

    -- IRQ output: mirrors STATUS[1] (frame_done W1C), cleared by CPU write
    enc_irq       : out std_logic
  );
end entity enc_ctrl;

architecture rtl of enc_ctrl is

  -- Registers
  signal reg_ctrl    : std_logic_vector(31 downto 0) := (others => '0');
  signal reg_status  : std_logic_vector(31 downto 0) := (others => '0');
  signal reg_width   : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(1920, 32));
  signal reg_height  : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(1080, 32));
  signal reg_qp      : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(28, 32));
  signal reg_gop     : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(30, 32));
  signal reg_ref_base: std_logic_vector(31 downto 0) := (others => '0');

  -- AXI-Lite handshake state
  signal aw_addr     : std_logic_vector(5 downto 0);
  signal aw_valid    : std_logic := '0';
  signal w_valid     : std_logic := '0';
  signal b_valid     : std_logic := '0';
  signal ar_addr     : std_logic_vector(5 downto 0);
  signal ar_valid    : std_logic := '0';
  signal r_valid     : std_logic := '0';
  signal r_data      : std_logic_vector(31 downto 0);

begin

  -- Outputs to datapath
  enc_enable   <= reg_ctrl(0);
  enc_reset    <= reg_ctrl(1);
  enc_width    <= unsigned(reg_width(11 downto 0));
  enc_height   <= unsigned(reg_height(11 downto 0));
  enc_irq      <= reg_status(1);
  enc_qp       <= unsigned(reg_qp(5 downto 0));
  enc_gop      <= unsigned(reg_gop(7 downto 0));
  enc_ref_base <= unsigned(reg_ref_base);

  -- AXI-Lite write channel
  s_axi_awready <= not aw_valid;
  s_axi_wready  <= not w_valid;
  s_axi_bresp   <= "00";
  s_axi_bvalid  <= b_valid;

  -- AXI-Lite read channel
  s_axi_arready <= not ar_valid;
  s_axi_rdata   <= r_data;
  s_axi_rresp   <= "00";
  s_axi_rvalid  <= r_valid;

  process(aclk)
    variable addr : integer range 0 to 31;
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        reg_ctrl     <= (others => '0');
        reg_width    <= std_logic_vector(to_unsigned(1920, 32));
        reg_height   <= std_logic_vector(to_unsigned(1080, 32));
        reg_qp       <= std_logic_vector(to_unsigned(28, 32));
        reg_gop      <= std_logic_vector(to_unsigned(30, 32));
        reg_ref_base <= (others => '0');
        aw_valid   <= '0';
        w_valid    <= '0';
        b_valid    <= '0';
        ar_valid   <= '0';
        r_valid    <= '0';
      else
        -- Auto-clear soft-reset bit
        reg_ctrl(1) <= '0';

        -- Update status register from datapath
        reg_status(0) <= enc_busy;
        reg_status(2) <= enc_error;
        if enc_frame_done = '1' then
          reg_status(1) <= '1';  -- W1C: set on frame done
        end if;

        -- ---------------------------------------------------------------
        -- Write address
        -- ---------------------------------------------------------------
        if s_axi_awvalid = '1' and aw_valid = '0' then
          aw_addr  <= s_axi_awaddr;
          aw_valid <= '1';
        end if;

        -- Write data
        if s_axi_wvalid = '1' and w_valid = '0' then
          w_valid <= '1';
        end if;

        -- Write register when both address and data are valid
        if aw_valid = '1' and w_valid = '1' then
          addr := to_integer(unsigned(aw_addr(5 downto 2)));
          case addr is
            when 0 => reg_ctrl     <= s_axi_wdata;
            when 1 =>
              -- W1C: clear frame_done if PS writes 1 to bit 1
              if s_axi_wdata(1) = '1' then reg_status(1) <= '0'; end if;
            when 2 => reg_width    <= s_axi_wdata;
            when 3 => reg_height   <= s_axi_wdata;
            when 4 => reg_qp       <= s_axi_wdata;
            when 5 => reg_gop      <= s_axi_wdata;
            when 8 => reg_ref_base <= s_axi_wdata;
            when others => null;
          end case;
          aw_valid <= '0';
          w_valid  <= '0';
          b_valid  <= '1';
        end if;

        if b_valid = '1' and s_axi_bready = '1' then
          b_valid <= '0';
        end if;

        -- ---------------------------------------------------------------
        -- Read address
        -- ---------------------------------------------------------------
        if s_axi_arvalid = '1' and ar_valid = '0' then
          ar_addr  <= s_axi_araddr;
          ar_valid <= '1';
        end if;

        if ar_valid = '1' and r_valid = '0' then
          addr := to_integer(unsigned(ar_addr(5 downto 2)));
          case addr is
            when 0 => r_data <= reg_ctrl;
            when 1 => r_data <= reg_status;
            when 2 => r_data <= reg_width;
            when 3 => r_data <= reg_height;
            when 4 => r_data <= reg_qp;
            when 5 => r_data <= reg_gop;
            when 6 => r_data <= std_logic_vector(enc_frame_cnt);
            when 7 => r_data <= std_logic_vector(enc_bs_bytes);
            when 8 => r_data <= reg_ref_base;
            when others => r_data <= (others => '0');
          end case;
          ar_valid <= '0';
          r_valid  <= '1';
        end if;

        if r_valid = '1' and s_axi_rready = '1' then
          r_valid <= '0';
        end if;

      end if;
    end if;
  end process;

end architecture rtl;
