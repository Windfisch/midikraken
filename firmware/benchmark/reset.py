#!/usr/bin/env python3

from time import sleep
from sys import argv
import libm2k

RESET_PIN=0
BOOT0_PIN=1

bootloader = False

def usage(name):
	print("Usage: %s [-b|--boot|--bootloader]", argv[0])
	exit(1)


if len(argv) == 2:
	if argv[1] in ['-b', '--boot', '--bootloader']:
		bootloader = True
	else:
		usage()
elif len(argv) == 1:
	bootloader = False
else:
	usage()

dev = libm2k.m2kOpen()

def reset(dev, bootloader):
	io = dev.getDigital()

	io.setDirection(RESET_PIN, libm2k.DIO_OUTPUT)
	io.setOutputMode(RESET_PIN, libm2k.DIO_OPENDRAIN)
	io.setDirection(BOOT0_PIN, libm2k.DIO_OUTPUT)
	io.setOutputMode(BOOT0_PIN, libm2k.DIO_PUSHPULL)

	io.setValueRaw(BOOT0_PIN, 1 if bootloader else 0)
	io.setValueRaw(RESET_PIN, 0)
	sleep(0.1)
	io.setValueRaw(RESET_PIN, 1)
	sleep(0.1)

	io.setDirection(RESET_PIN, libm2k.DIO_INPUT)
	io.setDirection(BOOT0_PIN, libm2k.DIO_INPUT)

print("resetting into %s mode" % ( "bootloader" if bootloader else "normal"))
reset(dev, bootloader)
print("ok")
