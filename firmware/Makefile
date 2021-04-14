all: debug.bin release.bin

release.bare.bin:
	cargo build --target=thumbv7m-none-eabi --release
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/midikraken $@

debug.bare.bin:
	cargo build --target=thumbv7m-none-eabi
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/debug/midikraken $@

release.dfu.bin:
	cargo build --target=thumbv7m-none-eabi --release --features=bootloader
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/midikraken $@

debug.dfu.bin:
	cargo build --target=thumbv7m-none-eabi --features=bootloader
	arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/debug/midikraken $@

flash.bare.%: %.bare.bin
	stm32flash /dev/ttyUSB0 -b115200 -w $<

flash.dfu.%: %.dfu.bin
	dfu-util -U $@

.PHONY: debug.bare.bin debug.dfu.bin release.bare.bin release.dfu.bin

tty:
	python /usr/lib/python3.9/site-packages/serial/tools/miniterm.py /dev/ttyUSB0 115200
