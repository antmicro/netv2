IP_ADDR ?= 10.0.0.2
SERIAL_PORT ?= /dev/ttyUSB1

default: module/build

all: build module/build load

build: gateware/build firmware/build

load: gateware/load firmware/load

software.tar.gz: $(wildcard software/kernel/*)
	tar czfv $@ software

module/copy: software.tar.gz
	scp $< antmicro@$(IP_ADDR):/home/antmicro

module/extract: module/copy
	ssh antmicro@$(IP_ADDR) "tar xzfv software.tar.gz"

module/build: module/extract
	ssh antmicro@$(IP_ADDR) "cd software/kernel; make; sudo cp litepcie.ko /lib/modules/4.19.0-6-amd64"

firmware/firmware.bin:
	make -C firmware

firmware/build: firmware/firmware.bin

firmware/load: firmware/firmware.bin
	lxterm --speed 115200 --kernel $< --serial-boot $(SERIAL_PORT)

gateware/build: netv2.py
	./netv2.py --build

gateware/load: gateware/build
	./netv2.py --load

host/gst:
	ssh antmicro@$(IP_ADDR) "DISPLAY=:0 gst-launch-1.0 -v --gst-debug=3 v4l2src device=/dev/video0 ! videoconvert ! fpsdisplaysink video-sink=\"ximagesink\" sync=false"

clean:
	git clean -xfd
