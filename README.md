Midikraken
==========

An open-source, open-hardware MIDI-USB-interface supporting up to 16
(and maybe beyond?) MIDI ports. The hardware is based on a STM32F103
*"blue pill"* board which can be cheaply sourced from your favourite chinese
seller ([note about quality differences](https://github.com/Windfisch/analog-synth/blob/master/bluepill.md)),
plus other garden variety components such as HC2630 opto couplers (that's just
two 6N137 in one package), shift registers, some LEDs and resistors.

Future features may include sophisticated MIDI routing, clock division,
a hardware arpeggiator etc.

This project is still in development stage, but already works fine as a flexible
as a MIDI-USB-interface.

![Example of a Midikraken configuration](hardware/img/midikraken_trs_din_ui.jpg)

How it works
------------

Since the STM32F103 only offers 3 hardware UARTs, we need to emulate the UART
protocol in software. For this, a timer interrupt oversamples the UART input pins
with thrice the baudrate in order to always read a stable bit at a non-edge.
(Guaranteed 1/3 * bit time distance from any edge, so the signal should be quite
stable until then.)

[Benchmarks show](firmware/benchmark/README.md) that this can be done fast
enough for up to 16 software UARTs (and likely even more), while leaving
plenty of cpu cycles (25% for 16 UARTs) free for other work.

Hardware
--------

Midikraken consists of two different PCBs that can be stacked (almost)
arbitrarily: [The DIN board](hardware/din5_pcb) has four traditional MIDI
in/out port pairs, and can house the master microcontroller.

[The TRS board](hardware/trs_pcb) gives you eight TRS MIDI pairs that support
both TRS-A and TRS-B. (Inputs are automatic, outputs need to be configured
in software). It can not house the microcontroller and thus cannot be used
without a DIN board.

The boards can be chained using their master/slave connectors and are
stackable using M3 screws / spacers. Currently, the firmware must be
adapted and recompiled according to the stack configuration. Currently,
a maximum of 16 port pairs is supported.

For information on how to build the hardware, the revision history and errata,
see [the hardware documentation](hardware/README.md).


Firmware
--------

The firmware is written in Rust. It can be used either bare metal or with a
bootloader.

### Flashing without a bootloader

  - Connect RX/TX of your uart/usb-adapter to PA9/PA8.
  - Put the STM32 into download mode by setting BOOT0 high and resetting.
  - `cd firmware`
  - `make flash.bare.release` (assuming that `ttyUSB0` is your uart adapter)

### Flashing and using a bootloader

#### Preparing the bootloader

Building with the `--features=bootloader` flag enabled will relocate the
program entry point to `FLASH + 0x2000`, allowing to use a bootloader.

Follow [these steps](firmware/sboot/README.md) in order to build and
initially flash the bootloader. You only need to perform this once.

#### Flashing the firmware

Bootloader mode can be entered by sending the `firmware/sboot/bootloader.syx` sysex to
the zeroth output of the midikraken (e.g. using `amidi -p hw:1,0,0 -s bootloader.syx`).

If that does not work (e.g. because the firmware is not present or corrupt), pull
PB12 low and reset.

  - Connect the Midikraken to USB
  - (in case of a defective firmware, pull PB12 low and reset)
  - `cd firmware`
  - `make flash.dfu.release` (will send the reset sysex automatically)

### Debugging

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

### Benchmarking

The software UART timer interrupt fires 93750x per second. This leaves
only 768 cpu cycles per call to the ISR.

To measure the performance of the software UART in the worst-case scenario
(all receivers are receiving their last bit and thus are finishing, and all
senders fetch their next byte and start transmitting it), enable the
`benchmark feature flag`. Note that this will send and receive garbage data.

License
-------

The software in the [firmware](firmware) directory can be redistributed
and/or modified under the terms of the
[GNU General Public License Version 3](gpl3.txt). Note this does not apply
to the [bootloader](firmware/sboot/sboot_stm32) which is under its own
license.

The hardware designs in the [hardware](hardware) directory, excluding
[hardware/lib/3d](hardware/lib/3d) to which own conditions apply, can be
licensed under the terms of the [CERN OHL Version 2 license](cern_ohl_s_v2.txt).
This specifically includes (but is not limited to) schematics, PCB designs and
gerber files.

Contributing
------------

I am happy about contributions, just open a pull request!

**Note:** Even though Midikraken is licensed under GPLv3 and OHL-S-v2 (the exact
versions), any contributions / pull requests must be licensed under the
"GPLv3 *or any later version*" or "OHL-S-v2 *or any later version*", respectively.
This is because on the one hand, I want to be able to update the project to new
versions of these licenses, but on the other hand, I don't like licensing my work
under a license (e.g. the hypothetical GPLv4) that I haven't even read yet. If such
a license is released, I will likely adopt it after revising it.
