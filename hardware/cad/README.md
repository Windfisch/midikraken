# Enclosures

Midikraken can optionally be put in an enclosure to protect it from dust and metallic parts that
could get inside.  Alternatively, it can be left half-open.

## Modular closed solution

For the closed solution, 3D-print the `display_shell`, `trs_back_shell` and `din5_shell`. You need
4x 22mm M3 spacers, 4x M3 screws and 4x M3 nuts to put everything together.

Place the nuts in the appropriate holes in `trs_back_shell`, then place the TRS board on top and
screw in the spacers.  Put the DIN5 shell onto the DIN5 board, connect the data cable and place the
DIN5 board on top of the TRS board. Now, mount the display on the display shell and carefully place
it onto the DIN5 board. Ensure that the display connector actually meets the receptacle. Then put in
the screws so everything holds together.

## Half-open solution

You will need one extra TRS or DIN5 board which has no parts soldered on. Also, you will need either
a 3D-printed `display_plate`, or the same thing fabricated from PCB material (preferred).
Additionally, 4x 22mm and 8x 16mm spacers are needed.

Basically, you create a sandwich from the empty PCB (used as bottom plate), then the TRS board, the
DIN5 board, and the display plate (with the display already mounted). These are connected by the
spacers.

It is recommended to use brass spacers here, for looks and for mechanical stability.
