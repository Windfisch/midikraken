#!/bin/bash

FIRMWARE=../
RESET="`pwd`/reset.py"
prefix="$1"
TTY=/dev/ttyUSB0

for N_UART in `seq 1 16`; do
	pushd "$FIRMWARE"
	sed -i 's/\(const\s*N_UART\s*:\s*usize\s*=\s*\)[0-9]\+/\1'${N_UART}'/' src/*.rs
	sed -i 's/\(type NumUarts = software_uart::typenum::U\)[0-9]\+;/\1'${N_UART}';/' src/*.rs
	$RESET -b
	make flash.release || ( sleep 1; $RESET -b; make flash.release || ( sleep 1; $RESET -b; make flash.release ) ) || ( echo "Error!" ; exit 1 )
	popd

	#stty 38400 -F $TTY
	stty 0:0:cbf:0:3:1c:7f:15:4:0:0:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0 -F $TTY
	tail -f $TTY | tee ${prefix}_${N_UART}.txt &
	TAILPID=$!
	$RESET
	sleep 5
	kill -s SIGINT $TAILPID; sleep 0.5; kill $TAILPID
done
