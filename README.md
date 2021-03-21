Midikraken
==========

Currently, only a proof-of-concept software UART.

Flashing: Connect RX/TX of your uart/usb-adapter to PA9/PA8.

PA0/PB0 is the first software UART running at 31250 baud, while PA9/PA8 is the hardware uart running at 38400 baud used
for flashing and debug messages.

Connect your MIDI opto-coupler to PB0 and watch the output on PA9/8.
