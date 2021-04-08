#!/bin/bash

for N in `seq 1 16`; do
	echo -n "$N          "
	for prefix in 82b57efa1 88f58aba4 242d2b9bd 725a3f8f1 f68a41072 opt4 generic_arrays; do
		for phase in 0 1 2; do
			numbers="$(cat ${prefix}_${N}.txt | grep 'cycle# '$phase | sed 's/.*-> //g')"
			min=999999
			max=0
			for num in $numbers; do
				if [ $num -gt $max ]; then max=$num; fi
				if [ $num -lt $min ]; then min=$num; fi
			done

			echo -n "$min $max    "
		done
		echo -n "    "
	done
	echo
done
