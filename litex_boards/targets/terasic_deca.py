#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2019 msloniewski <marcin.sloniewski@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use:
# ./terasic_deca.py --uart-name jtag_uart --build --load
# litex_term --jtag-config ../prog/openocd_usb_blaster2.cfg jtag

from migen import *
from litex_boards.platforms import terasic_deca

from litex.gen import *

from litex.soc.cores.clock import Max10PLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.video import VideoDVIPHY
from litex.soc.cores.led import LedChaser

from litedram.modules import AS4C128M16
from litedram.phy import DecaDdr3Phy

from BrianHG_DDR3 import Ddr3


from liteeth.phy.mii import LiteEthPHYMII

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_usb_pll=False):
        self.rst     = Signal()
        self.cd_sys  = ClockDomain()
        self.cd_hdmi = ClockDomain()
        self.cd_usb  = ClockDomain()

        # # #

        # Clk / Rst.
        clk50 = platform.request("clk50")

        # PLL
        self.pll = pll = Max10PLL(speedgrade="-6")
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys,  sys_clk_freq)
        pll.create_clkout(self.cd_hdmi, 40e6)

        # USB PLL.
        if with_usb_pll:
            ulpi  = platform.request("ulpi")
            self.comb += ulpi.cs.eq(1) # Enable ULPI chip to enable the ULPI clock.
            self.usb_pll = pll = Max10PLL(speedgrade="-6")
            self.comb += pll.reset.eq(self.rst)
            pll.register_clkin(ulpi.clk, 60e6)
            pll.create_clkout(self.cd_usb, 60e6, phase=-120) # -120° from DECA's example (also validated with LUNA).

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=50e6,
        with_led_chaser     = True,
        with_video_terminal = False,
        with_spi_sdcard     = False,
        with_ethernet       = False,
        with_etherbone      = False,
        with_dram           = False,
        eth_ip              = "192.168.1.50",
        eth_dynamic_ip      = False,
        **kwargs):
        self.platform = platform = terasic_deca.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = self.crg = _CRG(platform, sys_clk_freq, with_usb_pll=False)

        # SoCCore ----------------------------------------------------------------------------------
        # Defaults to JTAG-UART since no hardware UART.
        real_uart_name = kwargs["uart_name"]
        if real_uart_name == "serial":
            if kwargs["with_jtagbone"]:
                kwargs["uart_name"] = "crossover"
            else:
                kwargs["uart_name"] = "jtag_uart"
        if kwargs.get("with_uartbone"):
            kwargs["uart_name"] = "crossover"
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Terasic DECA", **kwargs)

        # Ethernet ---------------------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.platform.toolchain.additional_sdc_commands += [
                'create_clock -name eth_rx_clk -period 40.0 [get_ports {eth_clocks_rx}]',
                'create_clock -name eth_tx_clk -period 40.0 [get_ports {eth_clocks_tx}]',
                'set_false_path -from [get_clocks {sys_clk}] -to [get_clocks {eth_rx_clk}]',
                'set_false_path -from [get_clocks {sys_clk}] -to [get_clocks {eth_tx_clk}]',
                'set_false_path -from [get_clocks {eth_rx_clk}] -to [get_clocks {eth_tx_clk}]',
            ]
            self.ethphy = LiteEthPHYMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip)

        # Video ------------------------------------------------------------------------------------
        if with_video_terminal:
            self.videophy = VideoDVIPHY(platform.request("hdmi"), clock_domain="hdmi")
            self.add_video_terminal(phy=self.videophy, timings="800x600@60Hz", clock_domain="hdmi")

        # SPI SD card ------------------------------------------------------------------------------
        if with_spi_sdcard:
            self.add_spi_sdcard()

            sd_aux = self.platform.request("spisdcard_aux")

            # Set the SD card supply to 3.3V
            self.comb += sd_aux.sel.eq(0)

            # Set the direction of the level shifter (0 = SD to FPGA; 1 = FPGA to SD)
            self.comb += sd_aux.cmd_dir.eq(1)
            self.comb += sd_aux.d0_dir.eq(0)
            self.comb += sd_aux.d123_dir.eq(1)

            # Keep the unused data lines high
            self.comb += sd_aux.dat1.eq(1)
            self.comb += sd_aux.dat2.eq(1)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)
        
        if with_dram:
            ddr3 = Ddr3(platform)
            ddr3_pads = platform.request("ddram")
            ddr3_reset = Signal()
            ddr3_reset.eq(0)
            ddr_clk = Signal()
            ddr_clk_rdq = Signal()
            ddr_clk_wdq = Signal()
            ddr_clk_50 = Signal()
            ddr_clk_25 = Signal()
            ddr_cmd_clk = Signal()
            ddr_pll_locked = Signal()
            ddr_pll_phase_step = Signal()
            ddr_pll_phase_updn = Signal()
            ddr_pll_tune_phase_done = Signal()
            dram_pll = Instance("BrianHG_DDR3_PLL",
                i_RST_IN = ddr3_reset,
                o_DDR3_CLK = ddr_clk,
                o_DDR3_CLK_WDQ = ddr_clk_wdq,
                o_DDR3_CLK_RDQ = ddr_clk_rdq,
                o_DDR3_CLK_50 = ddr_clk_50,
                o_DDR3_CLK_25 = ddr_clk_25,
                o_CMD_CLK = ddr_cmd_clk,
                o_PLL_LOCKED = ddr_pll_locked,
                i_phase_step = ddr_pll_phase_step,
                i_phase_updn = ddr_pll_phase_updn,
                i_phase_sclk = ddr_clk_25,
                o_phase_done = ddr_pll_tune_phase_done,
                )
            self.dram = Instance(ddr3.ddr(),
                o_DDR3_RESET_n = ddr3_pads.reset_n,
                o_DDR3_CK_p = ddr3_pads.clk_p,
                o_DDR3_CK_n = ddr3_pads.clk_n,
                o_DDR3_CKE = ddr3_pads.cke,
                o_DDR3_CS_n = ddr3_pads.cs_n,
                o_DDR3_RAS_n = ddr3_pads.ras_n,
                o_DDR3_CAS_n = ddr3_pads.cas_n,
                o_DDR3_WE_n = ddr3_pads.we_n,
                o_DDR3_ODT = ddr3_pads.odt,
                o_DDR3_A = ddr3_pads.a,
                o_DDR3_BA = ddr3_pads.ba,
                io_DDR3_DM = ddr3_pads.dm,
                io_DDR3_DQ = ddr3_pads.dq,
                io_DDR3_DQS_p = ddr3_pads.dqs_p,
                io_DDR3_DQS_n = ddr3_pads.dqs_n,
                i_RST_IN = ddr3_reset,
                i_DDR_CLK = ddr_clk,
                i_DDR_CLK_RDQ = ddr_clk_rdq,
                i_DDR_CLK_WDQ = ddr_clk_wdq,
                i_DDR_CLK_50 = ddr_clk_50,
                i_DDR_CLK_25 = ddr_clk_25,
                i_CLK_IN = main_clock,
                i_CMD_CLK = ddr_cmd_clk,
                i_SEQ_CMD_ENA_t = ddr_seq_cmd_ena_t,
                i_SEQ_WRITE_ENA = ddr_seq_write_ena,
                i_SEQ_ADDR = ddr_seq_addr,
                i_SEQ_WDATA = ddr_seq_wdata,
                i_SEQ_WMASK = ddr_seq_wmask,
                i_SEQ_RDATA_VECT_IN = ddr_seq_rdata_vect_in,
                i_SEQ_refresh_hold = ddr_seq_refresh_hold,
                o_SEQ_BUSY_t = ddr_seq_busy_t,
                o_SEQ_RDATA_RDY_t = ddr_seq_rdata_rdy_t,
                o_SEQ_RDATA = ddr_seq_rdata,
                o_SEQ_RDATA_VECT_OUT = ddr_seq_rdata_vect_out,
                o_SEQ_refresh_queue = ddr_seq_refresh_queue,
                i_phase_done = ddr_pll_tune_phase_done,
                o_phase_step = ddr_pll_phase_step,
                o_phase_updn = ddr_pll_phase_updn,
                o_RDCAL_data = ddr_pll_rdcal_data,
            )
            self.specials += [self.dram]

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=terasic_deca.Platform, description="LiteX SoC on DECA.")
    parser.add_target_argument("--sys-clk-freq", default=50e6, type=float, help="System clock frequency.")
    ethopts = parser.target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",             action="store_true",    help="Enable Ethernet support.")
    ethopts.add_argument("--with-etherbone",            action="store_true",    help="Enable Etherbone support.")
    parser.add_target_argument("--eth-ip",              default="192.168.1.50", help="Ethernet/Etherbone IP address.")
    parser.add_target_argument("--eth-dynamic-ip",      action="store_true",    help="Enable dynamic Ethernet IP addresses setting.")
    parser.add_target_argument("--with-video-terminal", action="store_true",    help="Enable Video Terminal (VGA).")
    parser.add_target_argument("--with-spi-sdcard",     action="store_true",    help="Enable SPI SD card controller.")
    parser.add_target_argument("--with-dram",           action="store_true",    help="Enable DRAM support.")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq        = args.sys_clk_freq,
        with_ethernet       = args.with_ethernet,
        with_etherbone      = args.with_etherbone,
        eth_ip              = args.eth_ip,
        eth_dynamic_ip      = args.eth_dynamic_ip,
        with_video_terminal = args.with_video_terminal,
        with_spi_sdcard     = args.with_spi_sdcard,
        with_dram           = args.with_dram,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
