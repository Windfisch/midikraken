set termoption lw 3
set key bottom right

set yrange [0 : 800]
set xlabel 'number of software UARTs'
set ylabel 'worst-case cpu cycles used'

offset=0

plot 'data.txt' u 1:3 w lines title 'baseline' ls 1,\
	'data.txt' u 1:9 w lines title '0: unoptimized' ls 2,\
	'data.txt' u 1:($15+offset) w lines notitle ls 3 dt '-',\
	NaN w lines title '1: cache recv\_buffers' ls 3,\
	'data.txt' u 1:21 w lines title '2: cache send\_buffers' ls 4,\
	'data.txt' u 1:27 w lines title '3: cache recv\_active' ls 5,\
	'data.txt' u 1:33 w lines title '4: cache send\_workbuf[i]' ls 6,\
	'data.txt' u 1:($39+offset) w lines notitle ls 7 dt '-',\
	NaN w lines title '5: use generic\_array' ls 7,\
	768 title 'deadline' lc 'magenta'

pause -1
