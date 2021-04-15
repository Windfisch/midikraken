# Building the bootloader

Ensure that you have checked out the `sboot_stm32` submodule. If you
have not `git clone`d with the `--recursive` option, execute
`git submodule update --init` now.

  - `git submodule update --init`
  - `cd sboot_stm32`
  - `make prerequisites`
  - `make DFU_USER_CONFIG=../userconfig.h stm32f103x8`
  - flash `build/firmware.bin` to your device using one of the following ways:
    - using a ST-LINK
    - or set the *BOOT0* jumper to *high*, reset the board, connect a USB-UART
      adapter as follows: RX -> PA9, TX -> PA10, GND -> GND. Then use e.g.
      [stm32flash](https://sourceforge.net/projects/stm32flash/).
