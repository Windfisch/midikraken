Midikraken
==========

An open-source, open-hardware MIDI-USB-interface supporting up to 16
(and maybe beyond?) MIDI ports. The hardware is based on a STM32F103
*"blue pill"* board which can be cheaply sourced from your favourite chinese
seller ([note about quality differences](https://github.com/Windfisch/analog-synth/blob/master/bluepill.md)),
plus other garden variety components such as 6N137 opto couplers, some LEDs
and resistors.

Future features may include sophisticated MIDI routing, clock division,
a hardware arpeggiator etc.

This is currently in an early development stage, but should still be functional
as a MIDI-USB-interface.

How it works
------------

Since the STM32F103 only offers 3 hardware UARTs, we need to emulate the UART
protocol in software. For this, a timer interrupt oversamples the UART input pins
with thrice the baudrate in order to always read a stable bit at a non-edge.
(Guaranteed 1/3 * bit time distance from any edge, so the signal should be quite
stable until then.)

Flashing without a bootloader
-----------------------------

  - Connect RX/TX of your uart/usb-adapter to PA9/PA8.
  - Put the STM32 into download mode by setting BOOT0 high and resetting.
  - `cd firmware`
  - `make flash.bare.release` (assuming that `ttyUSB0` is your uart adapter)

Flashing and using a bootloader
-------------------------------

### Preparing the bootloader

Building with the `--features=bootloader` flag enabled will relocate the
program entry point to `FLASH + 0x2000`, allowing to use a bootloader.

Follow [these steps](firmware/sboot/README.md) in order to build and
initially flash the bootloader. You only need to perform this once.

### Flashing the firmware

Bootloader mode can be entered by sending the `firmware/sboot/bootloader.syx` sysex to
the zeroth output of the midikraken (e.g. using `amidi -p hw:1,0,0 -s bootloader.syx`).

If that does not work (e.g. because the firmware is not present or corrupt), pull
PB12 low and reset.

  - Connect the Midikraken to USB
  - (in case of a defective firmware, pull PB12 low and reset)
  - `cd firmware`
  - `make flash.dfu.release` (will send the reset sysex automatically)

Hardware
--------

Schematics for the first prototype board were created with KiCAD and are
located in [schem/perfboard](schem/perfboard).

A 3D-printed enclosure (which needs some adjustments to fit well) was
designed in FreeCAD and resides in [cad/](cad/).

Debugging
---------

printf-debugging can be seen on PA9/PA10 with 38400 baud. In the default
configuration, you should see only

```
========================================================
midikraken @ 6b3d42c5bf369a8da5d348a39342b91c2a3f7e80
      built on 2021-04-09T18:50:50.306707469+00:00
========================================================
```

You can enable more debug information with the `debugprint` flag.

Note that this degrades performance close to being unusable.

Benchmarking
------------

The software UART timer interrupt fires 93750x per second. This leaves
only 768 cpu cycles per call to the ISR.

To measure the performance of the software UART in the worst-case scenario
(all receivers are receiving their last bit and thus are finishing, and all
senders fetch their next byte and start transmitting it), enable the
`benchmark feature flag`. Note that this will send and receive garbage data.

