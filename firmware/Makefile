all: debug.dfu.bin release.dfu.bin

release.bare.bin:
	cargo build --target=thumbv7m-none-eabi --release --features=debugprint_version
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/midikraken $@

debug.bare.bin:
	cargo build --target=thumbv7m-none-eabi --features=debugprint_version,debugprint_basic,debugprint_verbose
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/debug/midikraken $@

release.dfu.bin:
	cargo build --target=thumbv7m-none-eabi --release --features=bootloader,debugprint_version
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/midikraken $@

debug.dfu.bin:
	cargo build --target=thumbv7m-none-eabi --features=bootloader,debugprint_basic,debugprint_verbose
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/debug/midikraken $@

flash.bare.%: %.bare.bin
	stm32flash -R /dev/ttyUSB0 -b115200 -w $<

flash.dfu.%: %.dfu.bin reboot
	dfu-util -R -D $<

reboot: sboot/bootloader.syx
	bash -c 'PORT=`amidi -l | grep "Midikraken.*" | head -n 1 | grep -o "hw:\S*"`; if [ x$$PORT != x ]; then amidi -p $$PORT -s $<; sleep 1.5; echo "Ok"; else echo "No device connected"; fi'
	sleep 0.5

.PHONY: debug.bare.bin debug.dfu.bin release.bare.bin release.dfu.bin

tty:
	python /usr/lib/python3.9/site-packages/serial/tools/miniterm.py /dev/ttyUSB0 38400
