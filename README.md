Midikraken
==========

Currently, only a proof-of-concept software UART.

Flashing: Connect RX/TX of your uart/usb-adapter to PA9/PA8.

Running: Connect RX/TX of your uart/usb-adapter to PA9/PB0, set the baud rate
to 38400 8N1 (8 data bits, no parity, 1 stop bit). Then type some letters. You
should receive

```
byte received!
#0, has_byte = Some(97)
...
```

Or connect RX/TX of your uart adapter to PA0/PB0 with 38400 baud 8N1 and type
some letters. You should receive one `abcd` for every byte sent.

PA0/PB0 is the first software UART, while PA9/PA8 is the hardware uart used
for flashing and debug messages.
