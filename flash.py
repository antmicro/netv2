#!/usr/bin/env python3
import sys
if "vivado" in sys.argv[1:]:
    from litex.build.xilinx.programmer import VivadoProgrammer
    prog = VivadoProgrammer(flash_part="mx25l6433f-spi-x1_x2_x4")
    prog.flash(0, "build/gateware/top.bin")
else:
    from litex.build.openocd import OpenOCD
    prog = OpenOCD("openocd/openocd.cfg",
            flash_proxy_basename="openocd/bscan_spi_xc7a35t.bit")
    prog.set_flash_proxy_dir(".")
    prog.flash(0x0, "build/gateware/top.bin")