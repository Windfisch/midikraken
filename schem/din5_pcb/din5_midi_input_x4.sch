EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Isolator:HCPL-2630 U?
U 1 1 60B467F2
P 2025 1400
AR Path="/60B467F2" Ref="U?"  Part="1" 
AR Path="/60B37C5E/60B467F2" Ref="U201"  Part="1" 
AR Path="/60B6A74D/60B467F2" Ref="U201"  Part="1" 
F 0 "U201" H 2325 975 50  0000 C CNN
F 1 "HCPL-2630" H 2400 875 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 2125 680 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-0940EN" H 1625 1750 50  0001 C CNN
	1    2025 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60B467F8
P 1575 1700
AR Path="/60B467F8" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60B467F8" Ref="R201"  Part="1" 
AR Path="/60B6A74D/60B467F8" Ref="R201"  Part="1" 
F 0 "R201" V 1475 1700 50  0000 C CNN
F 1 "220" V 1575 1700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1505 1700 50  0001 C CNN
F 3 "~" H 1575 1700 50  0001 C CNN
	1    1575 1700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60B46812
P 2025 1900
AR Path="/60B46812" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60B46812" Ref="#PWR0209"  Part="1" 
AR Path="/60B6A74D/60B46812" Ref="#PWR0209"  Part="1" 
F 0 "#PWR0209" H 2025 1650 50  0001 C CNN
F 1 "GND" H 2030 1727 50  0000 C CNN
F 2 "" H 2025 1900 50  0001 C CNN
F 3 "" H 2025 1900 50  0001 C CNN
	1    2025 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60B46818
P 2400 975
AR Path="/60B46818" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60B46818" Ref="R205"  Part="1" 
AR Path="/60B6A74D/60B46818" Ref="R205"  Part="1" 
F 0 "R205" V 2300 975 50  0000 C CNN
F 1 "1k" V 2400 975 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2330 975 50  0001 C CNN
F 3 "~" H 2400 975 50  0001 C CNN
	1    2400 975 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60B4681E
P 2600 975
AR Path="/60B4681E" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60B4681E" Ref="R209"  Part="1" 
AR Path="/60B6A74D/60B4681E" Ref="R209"  Part="1" 
F 0 "R209" V 2700 975 50  0000 C CNN
F 1 "220" V 2600 975 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2530 975 50  0001 C CNN
F 3 "~" H 2600 975 50  0001 C CNN
	1    2600 975 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2325 1200 2400 1200
Wire Wire Line
	2400 1125 2400 1200
$Comp
L Device:LED D?
U 1 1 60B4682F
P 2600 675
AR Path="/60B4682F" Ref="D?"  Part="1" 
AR Path="/60B37C5E/60B4682F" Ref="D201"  Part="1" 
AR Path="/60B6A74D/60B4682F" Ref="D201"  Part="1" 
F 0 "D201" V 2639 557 50  0000 R CNN
F 1 "LED" V 2548 557 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 2600 675 50  0001 C CNN
F 3 "~" H 2600 675 50  0001 C CNN
	1    2600 675 
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60B46E5A
P 1425 725
AR Path="/60B46E5A" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60B46E5A" Ref="#PWR0201"  Part="1" 
AR Path="/60B6A74D/60B46E5A" Ref="#PWR0201"  Part="1" 
F 0 "#PWR0201" H 1425 575 50  0001 C CNN
F 1 "+5V" H 1440 898 50  0000 C CNN
F 2 "" H 1425 725 50  0001 C CNN
F 3 "" H 1425 725 50  0001 C CNN
	1    1425 725 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C201
U 1 1 60B477D8
P 1575 725
F 0 "C201" V 1400 875 50  0000 C CNN
F 1 "100n" V 1500 875 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1613 575 50  0001 C CNN
F 3 "~" H 1575 725 50  0001 C CNN
	1    1575 725 
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60B484B1
P 1725 725
AR Path="/60B484B1" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60B484B1" Ref="#PWR0205"  Part="1" 
AR Path="/60B6A74D/60B484B1" Ref="#PWR0205"  Part="1" 
F 0 "#PWR0205" H 1725 475 50  0001 C CNN
F 1 "GND" H 1730 552 50  0000 C CNN
F 2 "" H 1725 725 50  0001 C CNN
F 3 "" H 1725 725 50  0001 C CNN
	1    1725 725 
	1    0    0    -1  
$EndComp
Text HLabel 2600 1200 2    50   Input ~ 0
in0
Wire Wire Line
	2600 1125 2600 1200
Wire Wire Line
	2600 1200 2400 1200
Connection ~ 2400 1200
$Comp
L power:+5V #PWR?
U 1 1 60B46827
P 2025 525
AR Path="/60B46827" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60B46827" Ref="#PWR0213"  Part="1" 
AR Path="/60B6A74D/60B46827" Ref="#PWR0213"  Part="1" 
F 0 "#PWR0213" H 2025 375 50  0001 C CNN
F 1 "+5V" H 1875 600 50  0000 C CNN
F 2 "" H 2025 525 50  0001 C CNN
F 3 "" H 2025 525 50  0001 C CNN
	1    2025 525 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 825  2400 525 
Wire Wire Line
	2025 525  2400 525 
Wire Wire Line
	2025 525  2025 900 
Connection ~ 2400 525 
Wire Wire Line
	2400 525  2600 525 
$Comp
L Device:R R?
U 1 1 60BCEBC4
P 3250 975
AR Path="/60BCEBC4" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60BCEBC4" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60BCEBC4" Ref="R?"  Part="1" 
F 0 "R?" V 3150 975 50  0000 C CNN
F 1 "1k" V 3250 975 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3180 975 50  0001 C CNN
F 3 "~" H 3250 975 50  0001 C CNN
	1    3250 975 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60BCEE8E
P 3450 975
AR Path="/60BCEE8E" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60BCEE8E" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60BCEE8E" Ref="R?"  Part="1" 
F 0 "R?" V 3550 975 50  0000 C CNN
F 1 "220" V 3450 975 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3380 975 50  0001 C CNN
F 3 "~" H 3450 975 50  0001 C CNN
	1    3450 975 
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 60BCEE9A
P 3450 675
AR Path="/60BCEE9A" Ref="D?"  Part="1" 
AR Path="/60B37C5E/60BCEE9A" Ref="D?"  Part="1" 
AR Path="/60B6A74D/60BCEE9A" Ref="D?"  Part="1" 
F 0 "D?" V 3489 557 50  0000 R CNN
F 1 "LED" V 3398 557 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 3450 675 50  0001 C CNN
F 3 "~" H 3450 675 50  0001 C CNN
	1    3450 675 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3250 825  3250 525 
Connection ~ 3250 525 
Wire Wire Line
	3250 525  3450 525 
Wire Wire Line
	2600 525  3250 525 
Connection ~ 2600 525 
Connection ~ 2025 525 
Wire Wire Line
	3250 1125 3250 1600
Wire Wire Line
	2325 1600 3250 1600
Wire Wire Line
	3450 1600 3250 1600
Wire Wire Line
	3450 1125 3450 1600
Connection ~ 3250 1600
$Comp
L Device:R R?
U 1 1 60BF5D21
P 1575 1100
AR Path="/60BF5D21" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60BF5D21" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60BF5D21" Ref="R?"  Part="1" 
F 0 "R?" V 1475 1100 50  0000 C CNN
F 1 "220" V 1575 1100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1505 1100 50  0001 C CNN
F 3 "~" H 1575 1100 50  0001 C CNN
	1    1575 1100
	0    1    1    0   
$EndComp
NoConn ~ 875  1050
Text HLabel 3450 1600 2    50   Input ~ 0
in1
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CC8557
P 875 750
F 0 "J?" H 725 975 50  0000 C CNN
F 1 "DIN-5_180degree" H 875 500 50  0000 C CNN
F 2 "" H 875 750 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 875 750 50  0001 C CNN
	1    875  750 
	-1   0    0    1   
$EndComp
Wire Wire Line
	1175 850  1175 1100
Wire Wire Line
	1175 1100 1425 1100
Wire Wire Line
	575  1300 1725 1300
Wire Wire Line
	575  850  575  1300
NoConn ~ 1175 750 
NoConn ~ 575  750 
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CCF2F3
P 875 1600
F 0 "J?" H 725 1825 50  0000 C CNN
F 1 "DIN-5_180degree" H 875 2050 50  0000 C CNN
F 2 "" H 875 1600 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 875 1600 50  0001 C CNN
	1    875  1600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1175 1700 1425 1700
Wire Wire Line
	575  1700 575  1975
Wire Wire Line
	575  1975 1325 1975
Wire Wire Line
	1325 1975 1325 1500
Wire Wire Line
	1325 1500 1725 1500
NoConn ~ 875  1900
NoConn ~ 1175 1600
NoConn ~ 575  1600
$Comp
L Isolator:HCPL-2630 U?
U 1 1 60CDE5F0
P 2025 3325
AR Path="/60CDE5F0" Ref="U?"  Part="1" 
AR Path="/60B37C5E/60CDE5F0" Ref="U?"  Part="1" 
AR Path="/60B6A74D/60CDE5F0" Ref="U?"  Part="1" 
F 0 "U?" H 2325 2900 50  0000 C CNN
F 1 "HCPL-2630" H 2400 2800 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 2125 2605 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-0940EN" H 1625 3675 50  0001 C CNN
	1    2025 3325
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60CDE7F0
P 1575 3625
AR Path="/60CDE7F0" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60CDE7F0" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60CDE7F0" Ref="R?"  Part="1" 
F 0 "R?" V 1475 3625 50  0000 C CNN
F 1 "220" V 1575 3625 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1505 3625 50  0001 C CNN
F 3 "~" H 1575 3625 50  0001 C CNN
	1    1575 3625
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60CDE7FA
P 2025 3825
AR Path="/60CDE7FA" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60CDE7FA" Ref="#PWR?"  Part="1" 
AR Path="/60B6A74D/60CDE7FA" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2025 3575 50  0001 C CNN
F 1 "GND" H 2030 3652 50  0000 C CNN
F 2 "" H 2025 3825 50  0001 C CNN
F 3 "" H 2025 3825 50  0001 C CNN
	1    2025 3825
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60CDE804
P 2400 2900
AR Path="/60CDE804" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60CDE804" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60CDE804" Ref="R?"  Part="1" 
F 0 "R?" V 2300 2900 50  0000 C CNN
F 1 "1k" V 2400 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2330 2900 50  0001 C CNN
F 3 "~" H 2400 2900 50  0001 C CNN
	1    2400 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60CDE80E
P 2600 2900
AR Path="/60CDE80E" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60CDE80E" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60CDE80E" Ref="R?"  Part="1" 
F 0 "R?" V 2700 2900 50  0000 C CNN
F 1 "220" V 2600 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2530 2900 50  0001 C CNN
F 3 "~" H 2600 2900 50  0001 C CNN
	1    2600 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2325 3125 2400 3125
Wire Wire Line
	2400 3050 2400 3125
$Comp
L Device:LED D?
U 1 1 60CDE81A
P 2600 2600
AR Path="/60CDE81A" Ref="D?"  Part="1" 
AR Path="/60B37C5E/60CDE81A" Ref="D?"  Part="1" 
AR Path="/60B6A74D/60CDE81A" Ref="D?"  Part="1" 
F 0 "D?" V 2639 2482 50  0000 R CNN
F 1 "LED" V 2548 2482 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 2600 2600 50  0001 C CNN
F 3 "~" H 2600 2600 50  0001 C CNN
	1    2600 2600
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60CDE824
P 1425 2650
AR Path="/60CDE824" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60CDE824" Ref="#PWR?"  Part="1" 
AR Path="/60B6A74D/60CDE824" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 1425 2500 50  0001 C CNN
F 1 "+5V" H 1440 2823 50  0000 C CNN
F 2 "" H 1425 2650 50  0001 C CNN
F 3 "" H 1425 2650 50  0001 C CNN
	1    1425 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60CDE82E
P 1575 2650
F 0 "C?" V 1400 2800 50  0000 C CNN
F 1 "100n" V 1500 2800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1613 2500 50  0001 C CNN
F 3 "~" H 1575 2650 50  0001 C CNN
	1    1575 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60CDE838
P 1725 2650
AR Path="/60CDE838" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60CDE838" Ref="#PWR?"  Part="1" 
AR Path="/60B6A74D/60CDE838" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 1725 2400 50  0001 C CNN
F 1 "GND" H 1730 2477 50  0000 C CNN
F 2 "" H 1725 2650 50  0001 C CNN
F 3 "" H 1725 2650 50  0001 C CNN
	1    1725 2650
	1    0    0    -1  
$EndComp
Text HLabel 2600 3125 2    50   Input ~ 0
in0
Wire Wire Line
	2600 3050 2600 3125
Wire Wire Line
	2600 3125 2400 3125
Connection ~ 2400 3125
$Comp
L power:+5V #PWR?
U 1 1 60CDE846
P 2025 2450
AR Path="/60CDE846" Ref="#PWR?"  Part="1" 
AR Path="/60B37C5E/60CDE846" Ref="#PWR?"  Part="1" 
AR Path="/60B6A74D/60CDE846" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2025 2300 50  0001 C CNN
F 1 "+5V" H 1875 2525 50  0000 C CNN
F 2 "" H 2025 2450 50  0001 C CNN
F 3 "" H 2025 2450 50  0001 C CNN
	1    2025 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2750 2400 2450
Wire Wire Line
	2025 2450 2400 2450
Wire Wire Line
	2025 2450 2025 2825
Connection ~ 2400 2450
Wire Wire Line
	2400 2450 2600 2450
$Comp
L Device:R R?
U 1 1 60CDE855
P 3250 2900
AR Path="/60CDE855" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60CDE855" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60CDE855" Ref="R?"  Part="1" 
F 0 "R?" V 3150 2900 50  0000 C CNN
F 1 "1k" V 3250 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3180 2900 50  0001 C CNN
F 3 "~" H 3250 2900 50  0001 C CNN
	1    3250 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60CDE85F
P 3450 2900
AR Path="/60CDE85F" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60CDE85F" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60CDE85F" Ref="R?"  Part="1" 
F 0 "R?" V 3550 2900 50  0000 C CNN
F 1 "220" V 3450 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3380 2900 50  0001 C CNN
F 3 "~" H 3450 2900 50  0001 C CNN
	1    3450 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 60CDE869
P 3450 2600
AR Path="/60CDE869" Ref="D?"  Part="1" 
AR Path="/60B37C5E/60CDE869" Ref="D?"  Part="1" 
AR Path="/60B6A74D/60CDE869" Ref="D?"  Part="1" 
F 0 "D?" V 3489 2482 50  0000 R CNN
F 1 "LED" V 3398 2482 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 3450 2600 50  0001 C CNN
F 3 "~" H 3450 2600 50  0001 C CNN
	1    3450 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3250 2750 3250 2450
Connection ~ 3250 2450
Wire Wire Line
	3250 2450 3450 2450
Wire Wire Line
	2600 2450 3250 2450
Connection ~ 2600 2450
Connection ~ 2025 2450
Wire Wire Line
	3250 3050 3250 3525
Wire Wire Line
	2325 3525 3250 3525
Wire Wire Line
	3450 3525 3250 3525
Wire Wire Line
	3450 3050 3450 3525
Connection ~ 3250 3525
$Comp
L Device:R R?
U 1 1 60CDE87E
P 1575 3025
AR Path="/60CDE87E" Ref="R?"  Part="1" 
AR Path="/60B37C5E/60CDE87E" Ref="R?"  Part="1" 
AR Path="/60B6A74D/60CDE87E" Ref="R?"  Part="1" 
F 0 "R?" V 1475 3025 50  0000 C CNN
F 1 "220" V 1575 3025 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1505 3025 50  0001 C CNN
F 3 "~" H 1575 3025 50  0001 C CNN
	1    1575 3025
	0    1    1    0   
$EndComp
NoConn ~ 875  2975
Text HLabel 3450 3525 2    50   Input ~ 0
in1
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CDE88A
P 875 2675
F 0 "J?" H 725 2900 50  0000 C CNN
F 1 "DIN-5_180degree" H 875 2425 50  0000 C CNN
F 2 "" H 875 2675 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 875 2675 50  0001 C CNN
	1    875  2675
	-1   0    0    1   
$EndComp
Wire Wire Line
	1175 2775 1175 3025
Wire Wire Line
	1175 3025 1425 3025
Wire Wire Line
	575  3225 1725 3225
Wire Wire Line
	575  2775 575  3225
NoConn ~ 1175 2675
NoConn ~ 575  2675
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CDE89A
P 875 3525
F 0 "J?" H 725 3750 50  0000 C CNN
F 1 "DIN-5_180degree" H 875 3975 50  0000 C CNN
F 2 "" H 875 3525 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 875 3525 50  0001 C CNN
	1    875  3525
	-1   0    0    1   
$EndComp
Wire Wire Line
	1175 3625 1425 3625
Wire Wire Line
	575  3625 575  3900
Wire Wire Line
	575  3900 1325 3900
Wire Wire Line
	1325 3900 1325 3425
Wire Wire Line
	1325 3425 1725 3425
NoConn ~ 875  3825
NoConn ~ 1175 3525
NoConn ~ 575  3525
$EndSCHEMATC
