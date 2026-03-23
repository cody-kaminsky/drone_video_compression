# =============================================================================
# enc_top.xdc  --  Timing constraints for enc_top IP
# Target: Zynq UltraScale+ @ 200 MHz
# =============================================================================

# Primary clock on aclk port (200 MHz = 5 ns period)
create_clock -period 5.000 -name aclk -waveform {0.000 2.500} [get_ports aclk]

# All logic in this design is synchronous to aclk
set_clock_groups -asynchronous -group [get_clocks aclk]
