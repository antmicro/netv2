IP_ADDR ?= 10.0.0.2
LOGIN ?= antmicro
HOST := $(LOGIN)@$(IP_ADDR)
SERIAL_PORT ?= /dev/ttyUSB1

default: all

### GENERAL TARGETS ###
all: build gateware/load host/reload

build: gateware/build firmware/build module/build host/reload

load: host/reload gateware/load firmware/load

target: firmware/load

clean: firmware/clean gateware/clean module/clean
	rm -rf data data.yuv


### GATEWARE ###
build/gateware/top.bit: netv2.py
	./netv2.py --build

gateware/build: build/gateware/top.bit

gateware/load: build/gateware/top.bit
	./netv2.py --load

gateware/clean:
	rm -rf build test/csr.csv


### FIRMWARE ###
firmware/firmware.bin: $(wildcard firmware/*) build/gateware/top.bit
	(cd firmware && make)

firmware/build: firmware/firmware.bin

firmware/load: firmware/firmware.bin
	lxterm --speed 115200 --kernel $< --serial-boot $(SERIAL_PORT)

firmware/clean:
	(cd firmware && make clean)


### KERNEL MODULE ###
software.tar.gz: $(wildcard software/kernel/*)
	tar czf $@ software

module/copy: software.tar.gz
	scp $< $(HOST):/home/$(LOGIN)

module/extract: module/copy
	ssh $(HOST) "tar --overwrite -xzf software.tar.gz"

module/build: module/extract
	ssh $(HOST) -t "cd software/kernel; make; sudo cp litepcie.ko /lib/modules/\`uname -r\`"

module/clean:
	rm -f software.tar.gz
	ssh $(HOST) "sudo rm /lib/modules/\`uname -r\`/litepcie.ko"
	ssh $(HOST) "rm -rf software software.tar.gz"


### HOST COMMANDS ###
host/reload:
	ssh $(HOST) "sudo modprobe -r litepcie"
	ssh $(HOST) "echo 1 | sudo tee /sys/bus/pci/devices/0000\:02\:00.0/remove"
	ssh $(HOST) "echo 1 | sudo tee /sys/bus/pci/rescan"

host/gst:
	ssh $(HOST) "DISPLAY=:0 gst-launch-1.0 -v --gst-debug=3 v4l2src device=/dev/video0 ! videoconvert ! fpsdisplaysink video-sink=\"ximagesink\" sync=false"

host/out:
	ssh $(HOST) "DISPLAY=:0 gst-launch-1.0 -v --gst-debug=3 videotestsrc ! video/x-raw,width=1280,height=720 ! v4l2sink device=/dev/video1 sync=false"

host/stop:
	ssh $(HOST) "killall -q gst-launch-1.0"

host/frames:
	ssh $(HOST) "DISPLAY=:0 gst-launch-1.0 -v --gst-debug=3 v4l2src device=/dev/video0 ! videoconvert ! filesink location=data.yuv \
							 & sleep 1s; killall -9 gst-launch-1.0"
	scp $(HOST):/home/$(LOGIN)/data.yuv .
	mkdir -p data
	convert -size 1280x720 -sampling-factor 4:2:2 -depth 8 data.yuv -colorspace RGB data/data.png
	feh $$(ls -vd data/*.png)
