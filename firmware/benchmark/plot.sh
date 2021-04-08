#!/bin/bash

echo -n "plot "

for i in `seq 1 $#`; do
	echo -n "'data.txt' u 1:$[6*i - 3] w lines title '$1', "
	shift 1
done
echo 0
echo pause -1
