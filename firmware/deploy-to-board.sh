#!/bin/bash

# Usage:
# cat >> /etc/udev/rules.d/50-dfu.rules
# SUBSYSTEMS=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="2e7f", GROUP="uucp", MODE="0666"
# ^D
# Then connect a fresh bluepill via USB to your computer, and connect it to a black magic probe (for JTAG/SWD).
# Execute the script to automatically flash the bootloader, the firmware, reboot into the firmware, reboot into
# the bootloader and test flashing.

set -e

if amidi -l | grep -q Midikraken; then
	echo "Error: Remove any midikrakens first"
	exit 1
fi

if [ ! -e sboot/sboot_stm32/build/firmware.elf ]; then
	echo "You need to read the instructions in sboot/README.md first"
fi

if [ ! -e target/thumbv7m-none-eabi/release/midikraken ]; then
	make release.dfu.bin
fi

for file in sboot/sboot_stm32/build/firmware.elf target/thumbv7m-none-eabi/release/midikraken; do
	if [ ! -e $file ]; then
		echo "Error: $file is missing"
		exit 1
	fi
	echo "Flashing $file"
	arm-none-eabi-gdb -n -batch -ex "target extended-remote /dev/ttyACM0" -ex "mon swdp_scan" -ex "attach 1" -ex "load" $file
done

echo
echo now reset and press enter

read foo

sleep 2

if ! amidi -l | grep -q Midikraken; then
	echo "Error: No midikraken found"
fi

make reboot

if dfu-util -R -D release.dfu.bin; then
	echo
	echo "All done :)"
else
	echo
	echo "Rebooting and re-flashing failed :("
	exit 1
fi

sleep 3

if ! amidi -l | grep Midikraken; then
	echo "Error: No midikraken found"
else
	echo "Success, midikraken found!"
fi

