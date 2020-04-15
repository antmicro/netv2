#!/usr/bin/env python3

import argparse

from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone
from litex.soc.cores.uart import *

from litevideo.output import VideoOut

from litedram import modules as litedram_modules
from litedram.phy.model import SDRAMPHYModel
from litex.tools.litex_sim import sdram_module_nphases, get_sdram_phy_settings

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid", Pins(1)),
        Subsignal("sink_ready", Pins(1)),
        Subsignal("sink_data",  Pins(8)),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# NeTV2 --------------------------------------------------------------------------------------------

class NeTV2(SoCSDRAM):
    def __init__(self,
        init_memories         = True,
        with_sdram            = True,
        sdram_module          = "MT48LC16M16",
        sdram_data_width      = 32,
        sdram_verbosity       = 0,
        with_ethernet         = False):
        platform     = Platform()
        sys_clk_freq = int(1e6)

        main_ram_init = get_mem_data({
            "firmware/firmware.bin": "0x00000000",
            }, "little")

        # SoCSDRAM ----------------------------------------------------------------------------------
        SoCSDRAM.__init__(self, platform, sys_clk_freq,
            cpu_type                 = "vexriscv",
            cpu_variant              = "lite",
            l2_reverse               = False,
            csr_data_width           = 32,
            with_uart                = False,
            max_sdram_size           = 0x10000000, # Limit mapped SDRAM to 1GB.
            integrated_rom_size      = 0x8000,
            integrated_sram_size     = 0x4000,
            ident                    = "NeTV2 LiteX SoC",
            ident_version            = True
        )
        self.add_constant("SIM", None)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("sys_clk"))

        # UART -------------------------------------------------------------------------------------
        self.submodules.uart_phy = RS232PHYModel(platform.request("serial"))
        self.submodules.uart = UART(self.uart_phy)
        self.add_csr("uart")
        self.add_interrupt("uart")

        # SDRAM ------------------------------------------------------------------------------------
        if with_sdram:
            sdram_clk_freq   = int(100e6) # FIXME: use 100MHz timings
            sdram_module_cls = getattr(litedram_modules, sdram_module)
            sdram_rate       = "1:{}".format(sdram_module_nphases[sdram_module_cls.memtype])
            sdram_module     = sdram_module_cls(sdram_clk_freq, sdram_rate)
            phy_settings     = get_sdram_phy_settings(
                memtype    = sdram_module.memtype,
                data_width = sdram_data_width,
                clk_freq   = sdram_clk_freq)
            self.submodules.sdrphy = SDRAMPHYModel(
                module    = sdram_module,
                settings  = phy_settings,
                clk_freq  = sdram_clk_freq,
                verbosity = sdram_verbosity,
                init      = main_ram_init)
            self.register_sdram(
                self.sdrphy,
                sdram_module.geom_settings,
                sdram_module.timing_settings)
            # FIXME: skip memtest to avoid corrupting memory
            self.add_constant("MEMTEST_BUS_SIZE",  0)
            self.add_constant("MEMTEST_ADDR_SIZE", 0)
            self.add_constant("MEMTEST_DATA_SIZE", 0)

        # hdmi out
        self.submodules.hdmi_out0 = VideoOut(
            device      = "sim",
            pads        = None,
            dram_port   = self.sdram.crossbar.get_port(
                mode         = "read",
                data_width   = 16,
                clock_domain = "sys",
                reverse      = True),
            mode        = "ycbcr422",
            fifo_depth  = 512)
        self.add_csr("hdmi_out0")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="NeTV2 HDMI2PCIe simulation")
    parser.add_argument("--with-sdram",           action="store_true",     help="enable SDRAM support")
    parser.add_argument("--sdram-module",         default="MT48LC16M16",   help="Select SDRAM chip")
    parser.add_argument("--sdram-data-width",     default=32,              help="Set SDRAM chip data width")
    parser.add_argument("--sdram-verbosity",      default=0,               help="Set SDRAM checker verbosity")
    parser.add_argument("--trace",                action="store_true",     help="enable VCD tracing")
    parser.add_argument("--trace-start",          default=0,               help="cycle to start VCD tracing")
    parser.add_argument("--trace-end",            default=-1,              help="cycle to end VCD tracing")
    parser.add_argument("--opt-level",            default="O3",            help="compilation optimization level")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")

    soc = NeTV2(
        init_memories         = False,
        with_sdram            = args.with_sdram,
        sdram_module          = args.sdram_module,
        sdram_data_width      = int(args.sdram_data_width),
        sdram_verbosity       = int(args.sdram_verbosity),
        with_ethernet         = False)
    board_name = "sim"
    build_dir = os.path.join("build", board_name)
    builder = Builder(soc, output_dir=build_dir,
        compile_gateware = True,
        csr_json         = os.path.join(build_dir, "csr.json"))
    builder.build(sim_config=sim_config,
        run         = True,
        opt_level   = args.opt_level,
        trace       = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end))


if __name__ == "__main__":
    main()
