NETV2_DIR=../build/sim

include $(NETV2_DIR)/software/include/generated/variables.mak
include ../../litex/litex/soc/software/common.mak
#include $(SOC_DIRECTORY)/software/common.mak

UIPDIR=uip
LIBUIPDIR=libuip
OBJECTS=\
        bist.o \
        ci.o \
        config.o \
        edid.o \
        encoder.o \
        etherbone.o \
        ethernet.o \
        fx2.o \
        hdmi_in0.o \
        hdmi_out0.o \
        hdmi_out1.o \
        heartbeat.o \
        i2c.o \
        isr.o \
        main.o \
        mdio.o \
        mmcm.o \
        oled.o \
        opsis_eeprom.o \
        pattern.o \
        pll.o \
        processor.o \
        reboot.o \
        stdio_wrap.o \
        tofe_eeprom.o \
        uptime.o \
        version.o \
        pcie.o \
	version_data.o

#$OBJECTS=isr.o \
#$		config.o \
#$		ethernet.o \
#$		etherbone.o \
#$		processor.o \
#$		hdmi_in0.o \
#$		hdmi_in1.o \
#$		hdmi_out0.o \
#$		pattern.o \
#$		edid.o \
#$		mmcm.o \
#$		ci.o \
#$		encoder.o \
#$		i2c.o \
#$		main.o \
#$		bist.o \
#$		dump.o \
#$		stdio_wrap.o

CFLAGS += -I. -I$(UIPDIR) -I$(LIBUIPDIR)

all: firmware.bin

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	chmod -x $@
	cp $@ boot.bin

firmware.elf: $(OBJECTS)
	$(LD) $(LDFLAGS) \
		-T linker.ld \
		-N -o $@ \
		 $(NETV2_DIR)/software/libbase/crt0-$(CPU)-ctr.o \
		$(OBJECTS) \
		-L$(NETV2_DIR)/software/libbase \
		-L$(NETV2_DIR)/software/libcompiler_rt \
		-lbase-nofloat -lcompiler_rt
	chmod -x $@

main.o: main.c
	$(compile)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

libs:
	$(MAKE) -C $(LIBUIPDIR)

load: firmware.bin
	litex_term --kernel firmware.bin COM8

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) firmware.elf firmware.bin .*~ *~

.PHONY: all main.o clean load
