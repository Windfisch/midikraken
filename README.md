Midikraken
==========

Currently, only a proof-of-concept software UART.

Flashing: Connect RX/TX of your uart/usb-adapter to PA9/PA8.

Running: Connect RX/TX of your uart/usb-adapter to PA9/PB0. Then type some letters. You should receive

```
byte received!
#0, has_byte = Some(97)
...
```
