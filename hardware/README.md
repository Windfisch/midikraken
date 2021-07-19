Midikraken hardware
===================

Currently, two different hardware revisions exist. They are described here from most recent to
oldest.

Revision 01
-----------

![Photo of Midikraken](img/midikraken_trs_din_ui.jpg)

The photo shows an (almost) fully equipped Midikraken including:

- 1 DIN connector board (the large MIDI connectors).
- 1 TRS connector board (MIDI via headphone jacks).
- The main microcontroller
- The UI consisting of a display and a rotary encoder.

Note that the UI is not implemented in software yet and thus has no function.

Midikraken boards can be stacked almost arbitrarily; the only limit is
(currently) a maximum of 16 MIDI in/out port pairs.

DIN boards may or may not house the main microcontroller, so every stack needs
one. Note that the DIN board can be used to house only the microcontroller, if
desired. No actual MIDI ports need to be equipped, making a TRS-only stack
possible.

TRS boards are not able to house a microcontroller and thus can not serve as
master.

### Errata

During powerup, the MIDI ports send garbage bits. To fix this,
you likely need to disconnect the 74HC595s' *output enable* pins from GND
using a sharp knife (and some wire to reconnect all GNDs that should stay
connected), and connecting them like this instead:

```
+5V --- Capacitor 10µF --- OE pin --- Resistor 10k --- GND
```

This delays the output enable enough such that the shift registers can
be filled with proper data after the microcontroller has booted up.

### Eurorack compatibility

Please note that Midikraken was designed as a standalone adapter box. However,
where possible, efforts were made to make Midikraken as Eurorack compatible as
possible:

- The PCBs are 110mm in height, fitting tightly into any Eurorack setup.
- The boards are prepared for upright connectors. This is sort of hacky, but it
  might work. The TRS board can hold PJ392 sockets that are usually meant to
  have wires soldered to their legs. The DIN board can hold DIN sockets similar
  to the NYS325, with a hack: The outermost two solder tail must be removed, and
  the remaining three must be bent so that they fit into the PCB.
- Alternatively, any socket can be used and wires can be soldered to the PCB.

Ghettokraken on perfboard
-------------------------

Schematics for the first prototype board were created with KiCAD and are
located in [perfboard](perfboard).

A 3D-printed enclosure (which needs some adjustments to fit well) was
designed in FreeCAD and resides in [perfboard/cad/](perfboard/cad/).
The enclose provides the DIN ports with additional stability which the
usual perfboards lack.

**Note**: You need to use the firmware from 7072a0c6eb or earlier due
different pin assignment; or you reassign the pins.

