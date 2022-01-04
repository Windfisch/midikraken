#!/bin/bash

set -e

VERSION="$1"

if [ x$VERSION == x ]; then
	echo Missing version
	exit 1
fi

rm -rf tmp
rm -rf out
mkdir tmp
mkdir out

# din5 schematic
eeschema_do export din5_pcb/din5_pcb.sch tmp
mv tmp/din5_pcb.pdf out/schematic_din5.pdf

# trs schematic
cp trs_pcb/pcb-cache.lib trs_pcb/trsmidi_input_8x-cache.lib
cp trs_pcb/pcb-cache.lib trs_pcb/trs_midi_output_8x-cache.lib
eeschema_do export trs_pcb/pcb.sch tmp
eeschema_do export trs_pcb/trsmidi_input_8x.sch tmp
eeschema_do export trs_pcb/trs_midi_output_8x.sch tmp
gs -o out/schematic_trs.pdf -sDEVICE=pdfwrite tmp/pcb.pdf tmp/trsmidi_input_8x.pdf tmp/trs_midi_output_8x.pdf

# din5 PCB
pcbnew_do export din5_pcb/din5_pcb.kicad_pcb tmp F.Silkscreen Edge.Cuts User.Comments F.Fab
mv tmp/printed.pdf tmp/front.pdf
pcbnew_do export din5_pcb/din5_pcb.kicad_pcb tmp B.Silkscreen Edge.Cuts B.Fab
mv tmp/printed.pdf tmp/back.pdf
gs -o out/boardview_din5.pdf -sDEVICE=pdfwrite tmp/front.pdf -dAutoRotatePages=/None -c "<</Install{595 0 translate -1 1 scale}>>setpagedevice" -f tmp/back.pdf

# trs PCB
pcbnew_do export trs_pcb/pcb.kicad_pcb tmp F.Silkscreen Edge.Cuts User.Comments F.Fab
mv tmp/printed.pdf tmp/front.pdf
pcbnew_do export trs_pcb/pcb.kicad_pcb tmp B.Silkscreen Edge.Cuts B.Fab
mv tmp/printed.pdf tmp/back.pdf
gs -o out/boardview_trs.pdf -sDEVICE=pdfwrite tmp/front.pdf -dAutoRotatePages=/None -c "<</Install{595 0 translate -1 1 scale}>>setpagedevice" -f tmp/back.pdf

mv out midikraken-hardware-drawings-"$VERSION"
zip -r midikraken-hardware-drawings-"$VERSION".zip midikraken-hardware-drawings-"$VERSION"/
