EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 1000 1625 0    50   Input ~ 0
out0
$Comp
L Device:LED D301
U 1 1 60C8764B
P 1150 1350
F 0 "D301" V 1175 1050 50  0000 L CNN
F 1 "yellow" V 1100 1025 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 1150 1350 50  0001 C CNN
F 3 "~" H 1150 1350 50  0001 C CNN
	1    1150 1350
	0    1    -1   0   
$EndComp
$Comp
L Device:R R301
U 1 1 60C88923
P 1150 1050
F 0 "R301" H 850 1100 50  0000 L CNN
F 1 "1k" V 1150 1000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1080 1050 50  0001 C CNN
F 3 "~" H 1150 1050 50  0001 C CNN
	1    1150 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1500 1150 1625
Wire Wire Line
	1150 1625 1000 1625
Connection ~ 1150 1625
Wire Wire Line
	1150 1625 1550 1625
$Comp
L Device:R R310
U 1 1 60C91870
P 1875 1200
F 0 "R310" V 1975 1050 50  0000 L CNN
F 1 "220" V 1875 1125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1805 1200 50  0001 C CNN
F 3 "~" H 1875 1200 50  0001 C CNN
	1    1875 1200
	0    1    1    0   
$EndComp
$Comp
L Device:R R309
U 1 1 60C8ACC4
P 1700 1625
F 0 "R309" V 1600 1525 50  0000 L CNN
F 1 "220" V 1700 1550 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1630 1625 50  0001 C CNN
F 3 "~" H 1700 1625 50  0001 C CNN
	1    1700 1625
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 900  1150 925 
$Comp
L power:+5V #PWR0301
U 1 1 60C8A300
P 1150 925
F 0 "#PWR0301" H 1150 775 50  0001 C CNN
F 1 "+5V" H 1275 1000 50  0000 C CNN
F 2 "" H 1150 925 50  0001 C CNN
F 3 "" H 1150 925 50  0001 C CNN
	1    1150 925 
	1    0    0    -1  
$EndComp
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CA1AE4
P 2325 1300
F 0 "J?" H 2275 1225 50  0000 C CNN
F 1 "DIN-5_180degree" H 2325 1050 50  0000 C CNN
F 2 "" H 2325 1300 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 2325 1300 50  0001 C CNN
	1    2325 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60CA7296
P 1725 1200
F 0 "#PWR?" H 1725 1050 50  0001 C CNN
F 1 "+5V" H 1725 1350 50  0000 C CNN
F 2 "" H 1725 1200 50  0001 C CNN
F 3 "" H 1725 1200 50  0001 C CNN
	1    1725 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60CA7BCC
P 2525 1000
F 0 "#PWR?" H 2525 750 50  0001 C CNN
F 1 "GND" H 2675 925 50  0000 C CNN
F 2 "" H 2525 1000 50  0001 C CNN
F 3 "" H 2525 1000 50  0001 C CNN
	1    2525 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 1000 2325 1000
NoConn ~ 2025 1300
Wire Wire Line
	2625 1200 2725 1200
Wire Wire Line
	2725 1200 2725 1625
Wire Wire Line
	1850 1625 2725 1625
NoConn ~ 2625 1300
Text HLabel 1000 2625 0    50   Input ~ 0
out1
$Comp
L Device:LED D?
U 1 1 60CB11ED
P 1150 2350
F 0 "D?" V 1175 2050 50  0000 L CNN
F 1 "yellow" V 1100 2025 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 1150 2350 50  0001 C CNN
F 3 "~" H 1150 2350 50  0001 C CNN
	1    1150 2350
	0    1    -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 60CB11F7
P 1150 2050
F 0 "R?" H 850 2100 50  0000 L CNN
F 1 "1k" V 1150 2000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1080 2050 50  0001 C CNN
F 3 "~" H 1150 2050 50  0001 C CNN
	1    1150 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 2500 1150 2625
Wire Wire Line
	1150 2625 1000 2625
Connection ~ 1150 2625
Wire Wire Line
	1150 2625 1550 2625
$Comp
L Device:R R?
U 1 1 60CB1205
P 1875 2200
F 0 "R?" V 1975 2050 50  0000 L CNN
F 1 "220" V 1875 2125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1805 2200 50  0001 C CNN
F 3 "~" H 1875 2200 50  0001 C CNN
	1    1875 2200
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60CB120F
P 1700 2625
F 0 "R?" V 1600 2525 50  0000 L CNN
F 1 "220" V 1700 2550 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1630 2625 50  0001 C CNN
F 3 "~" H 1700 2625 50  0001 C CNN
	1    1700 2625
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 1900 1150 1925
$Comp
L power:+5V #PWR?
U 1 1 60CB121A
P 1150 1925
F 0 "#PWR?" H 1150 1775 50  0001 C CNN
F 1 "+5V" H 1275 2000 50  0000 C CNN
F 2 "" H 1150 1925 50  0001 C CNN
F 3 "" H 1150 1925 50  0001 C CNN
	1    1150 1925
	1    0    0    -1  
$EndComp
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CB1224
P 2325 2300
F 0 "J?" H 2275 2225 50  0000 C CNN
F 1 "DIN-5_180degree" H 2325 2050 50  0000 C CNN
F 2 "" H 2325 2300 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 2325 2300 50  0001 C CNN
	1    2325 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60CB122E
P 1725 2200
F 0 "#PWR?" H 1725 2050 50  0001 C CNN
F 1 "+5V" H 1725 2350 50  0000 C CNN
F 2 "" H 1725 2200 50  0001 C CNN
F 3 "" H 1725 2200 50  0001 C CNN
	1    1725 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60CB1238
P 2525 2000
F 0 "#PWR?" H 2525 1750 50  0001 C CNN
F 1 "GND" H 2675 1925 50  0000 C CNN
F 2 "" H 2525 2000 50  0001 C CNN
F 3 "" H 2525 2000 50  0001 C CNN
	1    2525 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 2000 2325 2000
NoConn ~ 2025 2300
Wire Wire Line
	2625 2200 2725 2200
Wire Wire Line
	2725 2200 2725 2625
Wire Wire Line
	1850 2625 2725 2625
NoConn ~ 2625 2300
Text HLabel 1000 3625 0    50   Input ~ 0
out2
$Comp
L Device:LED D?
U 1 1 60CB6CFC
P 1150 3350
F 0 "D?" V 1175 3050 50  0000 L CNN
F 1 "yellow" V 1100 3025 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 1150 3350 50  0001 C CNN
F 3 "~" H 1150 3350 50  0001 C CNN
	1    1150 3350
	0    1    -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 60CB6D06
P 1150 3050
F 0 "R?" H 850 3100 50  0000 L CNN
F 1 "1k" V 1150 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1080 3050 50  0001 C CNN
F 3 "~" H 1150 3050 50  0001 C CNN
	1    1150 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 3500 1150 3625
Wire Wire Line
	1150 3625 1000 3625
Connection ~ 1150 3625
Wire Wire Line
	1150 3625 1550 3625
$Comp
L Device:R R?
U 1 1 60CB6D14
P 1875 3200
F 0 "R?" V 1975 3050 50  0000 L CNN
F 1 "220" V 1875 3125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1805 3200 50  0001 C CNN
F 3 "~" H 1875 3200 50  0001 C CNN
	1    1875 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60CB6D1E
P 1700 3625
F 0 "R?" V 1600 3525 50  0000 L CNN
F 1 "220" V 1700 3550 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1630 3625 50  0001 C CNN
F 3 "~" H 1700 3625 50  0001 C CNN
	1    1700 3625
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 2900 1150 2925
$Comp
L power:+5V #PWR?
U 1 1 60CB6D29
P 1150 2925
F 0 "#PWR?" H 1150 2775 50  0001 C CNN
F 1 "+5V" H 1275 3000 50  0000 C CNN
F 2 "" H 1150 2925 50  0001 C CNN
F 3 "" H 1150 2925 50  0001 C CNN
	1    1150 2925
	1    0    0    -1  
$EndComp
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CB6D33
P 2325 3300
F 0 "J?" H 2275 3225 50  0000 C CNN
F 1 "DIN-5_180degree" H 2325 3050 50  0000 C CNN
F 2 "" H 2325 3300 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 2325 3300 50  0001 C CNN
	1    2325 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60CB6D3D
P 1725 3200
F 0 "#PWR?" H 1725 3050 50  0001 C CNN
F 1 "+5V" H 1725 3350 50  0000 C CNN
F 2 "" H 1725 3200 50  0001 C CNN
F 3 "" H 1725 3200 50  0001 C CNN
	1    1725 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60CB6D47
P 2525 3000
F 0 "#PWR?" H 2525 2750 50  0001 C CNN
F 1 "GND" H 2675 2925 50  0000 C CNN
F 2 "" H 2525 3000 50  0001 C CNN
F 3 "" H 2525 3000 50  0001 C CNN
	1    2525 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 3000 2325 3000
NoConn ~ 2025 3300
Wire Wire Line
	2625 3200 2725 3200
Wire Wire Line
	2725 3200 2725 3625
Wire Wire Line
	1850 3625 2725 3625
NoConn ~ 2625 3300
Text HLabel 1000 4625 0    50   Input ~ 0
out3
$Comp
L Device:LED D?
U 1 1 60CB6D58
P 1150 4350
F 0 "D?" V 1175 4050 50  0000 L CNN
F 1 "yellow" V 1100 4025 50  0000 L CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O6.35mm_Z2.0mm" H 1150 4350 50  0001 C CNN
F 3 "~" H 1150 4350 50  0001 C CNN
	1    1150 4350
	0    1    -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 60CB6D62
P 1150 4050
F 0 "R?" H 850 4100 50  0000 L CNN
F 1 "1k" V 1150 4000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1080 4050 50  0001 C CNN
F 3 "~" H 1150 4050 50  0001 C CNN
	1    1150 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 4500 1150 4625
Wire Wire Line
	1150 4625 1000 4625
Connection ~ 1150 4625
Wire Wire Line
	1150 4625 1550 4625
$Comp
L Device:R R?
U 1 1 60CB6D70
P 1875 4200
F 0 "R?" V 1975 4050 50  0000 L CNN
F 1 "220" V 1875 4125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1805 4200 50  0001 C CNN
F 3 "~" H 1875 4200 50  0001 C CNN
	1    1875 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 60CB6D7A
P 1700 4625
F 0 "R?" V 1600 4525 50  0000 L CNN
F 1 "220" V 1700 4550 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1630 4625 50  0001 C CNN
F 3 "~" H 1700 4625 50  0001 C CNN
	1    1700 4625
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 3900 1150 3925
$Comp
L power:+5V #PWR?
U 1 1 60CB6D85
P 1150 3925
F 0 "#PWR?" H 1150 3775 50  0001 C CNN
F 1 "+5V" H 1275 4000 50  0000 C CNN
F 2 "" H 1150 3925 50  0001 C CNN
F 3 "" H 1150 3925 50  0001 C CNN
	1    1150 3925
	1    0    0    -1  
$EndComp
$Comp
L Connector:DIN-5_180degree J?
U 1 1 60CB6D8F
P 2325 4300
F 0 "J?" H 2275 4225 50  0000 C CNN
F 1 "DIN-5_180degree" H 2325 4050 50  0000 C CNN
F 2 "" H 2325 4300 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 2325 4300 50  0001 C CNN
	1    2325 4300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60CB6D99
P 1725 4200
F 0 "#PWR?" H 1725 4050 50  0001 C CNN
F 1 "+5V" H 1725 4350 50  0000 C CNN
F 2 "" H 1725 4200 50  0001 C CNN
F 3 "" H 1725 4200 50  0001 C CNN
	1    1725 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60CB6DA3
P 2525 4000
F 0 "#PWR?" H 2525 3750 50  0001 C CNN
F 1 "GND" H 2675 3925 50  0000 C CNN
F 2 "" H 2525 4000 50  0001 C CNN
F 3 "" H 2525 4000 50  0001 C CNN
	1    2525 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 4000 2325 4000
NoConn ~ 2025 4300
Wire Wire Line
	2625 4200 2725 4200
Wire Wire Line
	2725 4200 2725 4625
Wire Wire Line
	1850 4625 2725 4625
NoConn ~ 2625 4300
$EndSCHEMATC