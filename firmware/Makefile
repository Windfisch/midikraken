all: debug.bin release.bin

%.bin: target/thumbv7m-none-eabi/%/midikraken
	arm-none-eabi-objcopy -O binary $< $@

flash.%: %.bin
	stm32flash /dev/ttyUSB0 -b115200 -w $<

.PHONY: target/thumbv7m-none-eabi/debug/midikraken
.PHONY: target/thumbv7m-none-eabi/release/midikraken
target/thumbv7m-none-eabi/debug/midikraken:
	cargo build --target=thumbv7m-none-eabi --features=benchmark
target/thumbv7m-none-eabi/release/midikraken:
	cargo build --release --target=thumbv7m-none-eabi --features=benchmark

tty:
	python /usr/lib/python3.9/site-packages/serial/tools/miniterm.py /dev/ttyUSB0 115200
