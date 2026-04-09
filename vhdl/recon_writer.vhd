-- =============================================================================
-- recon_writer.vhd  --  Reconstruction loop: dequant -> IDCT -> add pred -> DDR
--
-- Change 3: Wide input/output.
--   coeff_in  : 128-bit (8 × 16-bit quantised coefficients, one full row/clock)
--   wr_pixel  : 64-bit  (8 pixels packed, one row per clock)
--
-- Dequant and pixel-output now process 8 values in parallel, reducing the
-- coefficient-fill phase from 64 to 8 clocks and the pixel-output phase from
-- 64 to 8 clocks.  recon_done fires 8 cycles after the IDCT emits its first
-- output row (i.e. 8 clocks to compute all 8 rows of the 8×8 block).
--
-- Supports three intra prediction modes driven by intra_mode port:
--   INTRA_DC    : add pred_dc scalar to every residual pixel
--   INTRA_VERT  : add pred_above_row[col] per column
--   INTRA_HORIZ : add pred_left_col[row] per row
-- P-frame: pred_use_dc='0', uses pred_buf (MC rows from halfpel_mc).
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.enc_pkg.all;

entity recon_writer is
  port (
    aclk       : in  std_logic;
    aresetn    : in  std_logic;

    qp         : in  unsigned(5 downto 0);
    is_intra   : in  std_logic;

    -- Quantised coefficient input (row-major order, one 8-coeff row per clock)
    coeff_in   : in  std_logic_vector(127 downto 0);  -- 8 × signed 16-bit
    coeff_valid: in  std_logic;
    blk_start  : in  std_logic;

    -- Prediction
    pred_dc       : in  unsigned(7 downto 0);
    pred_use_dc   : in  std_logic;
    pred_above_row: in  std_logic_vector(63 downto 0);
    pred_left_col : in  std_logic_vector(63 downto 0);
    intra_mode    : in  intra_mode_t;
    pred_row      : in  std_logic_vector(63 downto 0);
    pred_row_v    : in  std_logic;

    -- Block position
    blk_x      : in  unsigned(11 downto 0);
    blk_y      : in  unsigned(11 downto 0);

    -- DDR write port (64-bit: one 8-pixel row per clock)
    wr_start   : out std_logic;
    wr_blk_x   : out unsigned(11 downto 0);
    wr_blk_y   : out unsigned(11 downto 0);
    wr_pixel   : out std_logic_vector(63 downto 0);
    wr_pixel_v : out std_logic;

    -- Reconstructed pixel rows fed back to mb_buffer
    recon_done : out std_logic;
    recon_row7 : out std_logic_vector(63 downto 0);
    recon_col7 : out std_logic_vector(63 downto 0)
  );
end entity recon_writer;

architecture rtl of recon_writer is

  type step_rom_t is array(1 to 51) of integer range 0 to 65535;
  type int6_arr_t is array(0 to 5) of integer;
  function build_step_rom return step_rom_t is
    constant BASE : int6_arr_t := (10, 11, 13, 14, 16, 18);
    variable rom  : step_rom_t;
  begin
    for q in 1 to 51 loop
      rom(q) := BASE((q-1) mod 6) * (2 ** ((q-1)/6));
    end loop;
    return rom;
  end function;
  constant STEP_ROM : step_rom_t := build_step_rom;

  signal step_r  : integer range 1 to 65535 := 10;
  signal qp_prev : unsigned(5 downto 0) := (others => '0');

  signal coeff_row : integer range 0 to 7 := 0;

  signal deq_row   : int32_row_t;
  signal idct_din  : std_logic_vector(255 downto 0);
  signal idct_v    : std_logic := '0';

  signal idct_dout : std_logic_vector(127 downto 0);
  signal idct_dv   : std_logic;
  signal idct_last : std_logic;
  signal idct_rdy  : std_logic;

  type pred_buf_t is array(0 to 7) of std_logic_vector(63 downto 0);
  signal pred_buf    : pred_buf_t;
  signal pred_buf_wr : integer range 0 to 7 := 0;

  signal intra_mode_r : intra_mode_t := INTRA_DC;
  signal above_row_r  : std_logic_vector(63 downto 0) := (others => '0');
  signal left_col_r   : std_logic_vector(63 downto 0) := (others => '0');

  signal out_row    : integer range 0 to 7 := 0;
  signal wr_start_r : std_logic := '0';
  signal wr_pix_r   : std_logic_vector(63 downto 0) := (others => '0');
  signal wr_pix_v_r : std_logic := '0';
  signal blk_x_r    : unsigned(11 downto 0) := (others => '0');
  signal blk_y_r    : unsigned(11 downto 0) := (others => '0');

  -- col-7 pixel for each row (for recon_col7 feedback)
  type byte8_t is array(0 to 7) of std_logic_vector(7 downto 0);
  signal col7_pix : byte8_t := (others => x"80");

  signal recon_done_r : std_logic := '0';
  signal recon_row7_r : std_logic_vector(63 downto 0) := (others => '0');
  signal recon_col7_r : std_logic_vector(63 downto 0) := (others => '0');

begin

  wr_start   <= wr_start_r;
  wr_blk_x   <= blk_x_r;
  wr_blk_y   <= blk_y_r;
  wr_pixel   <= wr_pix_r;
  wr_pixel_v <= wr_pix_v_r;
  recon_done <= recon_done_r;
  recon_row7 <= recon_row7_r;
  recon_col7 <= recon_col7_r;

  gen_idct_in : for i in 0 to 7 generate
    idct_din(i*32+31 downto i*32) <= std_logic_vector(deq_row(i));
  end generate;

  -- IDCT is always ready: we process one row/clock and the pipeline is deep
  -- enough that backpressure never occurs between consecutive blocks.

  u_idct : entity work.dct8_inv
    port map (
      aclk     => aclk,
      aresetn  => aresetn,
      s_tdata  => idct_din,
      s_tvalid => idct_v,
      s_tready => idct_rdy,
      m_tdata  => idct_dout,
      m_tvalid => idct_dv,
      m_tlast  => idct_last,
      m_tready => '1'
    );

  -- ---------------------------------------------------------------------------
  -- Coefficient dequantisation — 8 coefficients per clock
  -- ---------------------------------------------------------------------------
  process(aclk)
    variable qp_int : integer range 1 to 51;
    variable coeff  : signed(15 downto 0);
    variable dq     : signed(31 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        coeff_row <= 0;
        idct_v    <= '0';
        step_r    <= 10;
        qp_prev   <= (others => '0');
      else
        idct_v <= '0';

        if qp /= qp_prev then
          qp_int := to_integer(qp);
          if qp_int < 1  then qp_int := 1;  end if;
          if qp_int > 51 then qp_int := 51; end if;
          step_r  <= STEP_ROM(qp_int);
          qp_prev <= qp;
        end if;

        if blk_start = '1' then
          coeff_row    <= 0;
          intra_mode_r <= intra_mode;
          above_row_r  <= pred_above_row;
          left_col_r   <= pred_left_col;
        end if;

        if coeff_valid = '1' then
          -- Dequantise all 8 coefficients in parallel
          for i in 0 to 7 loop
            coeff      := signed(coeff_in(i*16+15 downto i*16));
            dq         := coeff * to_signed(step_r, 16);
            deq_row(i) <= dq;
          end loop;
          idct_v <= '1';

          if coeff_row = 7 then
            coeff_row <= 0;
          else
            coeff_row <= coeff_row + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ---------------------------------------------------------------------------
  -- Prediction buffer write (P-frame MC rows)
  -- ---------------------------------------------------------------------------
  process(aclk)
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        pred_buf_wr <= 0;
      else
        if blk_start = '1' then
          pred_buf_wr <= 0;
        end if;
        if pred_row_v = '1' then
          pred_buf(pred_buf_wr) <= pred_row;
          if pred_buf_wr = 7 then pred_buf_wr <= 0;
          else                     pred_buf_wr <= pred_buf_wr + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ---------------------------------------------------------------------------
  -- Output: IDCT row -> add prediction -> clip -> 64-bit wr_pixel row
  -- One row per clock; recon_done fires after row 7.
  -- ---------------------------------------------------------------------------
  process(aclk)
    type pix8_t   is array(0 to 7) of unsigned(7 downto 0);
    variable pv       : pix8_t;
    variable residual : signed(15 downto 0);
    variable pred_pix : unsigned(7 downto 0);
    variable sum      : signed(16 downto 0);
    variable rr7_v    : std_logic_vector(63 downto 0);
    variable rc7_v    : std_logic_vector(63 downto 0);
  begin
    if rising_edge(aclk) then
      if aresetn = '0' then
        out_row      <= 0;
        wr_start_r   <= '0';
        wr_pix_v_r   <= '0';
        recon_done_r <= '0';
        blk_x_r      <= (others => '0');
        blk_y_r      <= (others => '0');
        recon_row7_r <= (others => '0');
        recon_col7_r <= (others => '0');
      else
        wr_start_r   <= '0';
        wr_pix_v_r   <= '0';
        recon_done_r <= '0';

        if blk_start = '1' then
          blk_x_r    <= blk_x;
          blk_y_r    <= blk_y;
          out_row    <= 0;
          wr_start_r <= '1';
        end if;

        if idct_dv = '1' then
          -- Compute all 8 pixels in parallel
          for i in 0 to 7 loop
            residual := signed(idct_dout(i*16+15 downto i*16));

            if pred_use_dc = '1' then
              case intra_mode_r is
                when INTRA_VERT =>
                  pred_pix := unsigned(above_row_r(i*8+7 downto i*8));
                when INTRA_HORIZ =>
                  pred_pix := unsigned(left_col_r(out_row*8+7 downto out_row*8));
                when others =>
                  pred_pix := pred_dc;
              end case;
            else
              pred_pix := unsigned(pred_buf(out_row)(i*8+7 downto i*8));
            end if;

            sum := resize(residual, 17) + resize(signed('0' & pred_pix), 17);
            if sum < 0 then
              pv(i) := (others => '0');
            elsif sum > 255 then
              pv(i) := (others => '1');
            else
              pv(i) := unsigned(sum(7 downto 0));
            end if;
          end loop;

          -- Pack 8 pixels into 64-bit row output
          for i in 0 to 7 loop
            wr_pix_r(i*8+7 downto i*8) <= std_logic_vector(pv(i));
          end loop;
          wr_pix_v_r <= '1';

          -- Record col-7 pixel for this row
          col7_pix(out_row) <= std_logic_vector(pv(7));

          if out_row = 7 then
            -- Build recon_row7: all 8 pixels of row 7
            for i in 0 to 7 loop
              rr7_v(i*8+7 downto i*8) := std_logic_vector(pv(i));
            end loop;
            recon_row7_r <= rr7_v;

            -- Build recon_col7: col-7 pixel from each row
            for i in 0 to 6 loop
              rc7_v(i*8+7 downto i*8) := col7_pix(i);
            end loop;
            rc7_v(63 downto 56) := std_logic_vector(pv(7));
            recon_col7_r <= rc7_v;

            recon_done_r <= '1';
            out_row <= 0;
          else
            out_row <= out_row + 1;
          end if;
        end if;

      end if;
    end if;
  end process;

end architecture rtl;
