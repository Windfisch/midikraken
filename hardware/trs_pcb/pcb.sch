EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title "Midikraken TRS"
Date ""
Rev "01"
Comp "Windfisch"
Comment1 "the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.txt)."
Comment2 "You may redistribute and modify this source and make products using it under "
Comment3 "This source describes Open Hardware and is licensed under the CERN-OHL-S v2."
Comment4 "Copyright (c) 2021 Florian Jung"
$EndDescr
$Comp
L 74xx:74HC595 U103
U 1 1 60AADCC5
P 8850 1575
F 0 "U103" H 9125 800 50  0000 C CNN
F 1 "74HC595" H 9100 900 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 8850 1575 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8850 1575 50  0001 C CNN
	1    8850 1575
	1    0    0    -1  
$EndComp
Text GLabel 8450 1175 0    50   Input ~ 0
MOSI_MIDI_5
Text GLabel 8450 1375 0    50   Input ~ 0
SCK_MIDI_5
Text GLabel 8450 1675 0    50   Input ~ 0
STROBE_MIDI_5
Wire Wire Line
	8450 1475 7900 1475
Wire Wire Line
	7900 1475 7900 975 
Wire Wire Line
	7900 975  8850 975 
Wire Wire Line
	8450 1775 8450 2275
Wire Wire Line
	8450 2275 8850 2275
$Comp
L power:GND #PWR0112
U 1 1 60B26AB2
P 8850 2275
F 0 "#PWR0112" H 8850 2025 50  0001 C CNN
F 1 "GND" H 8855 2102 50  0000 C CNN
F 2 "" H 8850 2275 50  0001 C CNN
F 3 "" H 8850 2275 50  0001 C CNN
	1    8850 2275
	1    0    0    -1  
$EndComp
Connection ~ 8850 2275
$Comp
L power:+5V #PWR0111
U 1 1 60B27D55
P 8850 975
F 0 "#PWR0111" H 8850 825 50  0001 C CNN
F 1 "+5V" H 8865 1148 50  0000 C CNN
F 2 "" H 8850 975 50  0001 C CNN
F 3 "" H 8850 975 50  0001 C CNN
	1    8850 975 
	1    0    0    -1  
$EndComp
Connection ~ 8850 975 
Text GLabel 3675 2825 0    50   Input ~ 0
SCK_MIDI_5
Text GLabel 3675 2625 0    50   Input ~ 0
STROBE_MIDI_5
Text GLabel 4675 1625 2    50   Input ~ 0
MISO_MIDI
Wire Wire Line
	3675 2925 3675 3225
Wire Wire Line
	3675 3225 4175 3225
$Comp
L 74xx:74HC165 U102
U 1 1 60AAE484
P 4175 2225
F 0 "U102" H 4300 2200 50  0000 C CNN
F 1 "74HC165" H 4275 2050 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 4175 2225 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/74HC_HCT165.pdf" H 4175 2225 50  0001 C CNN
	1    4175 2225
	1    0    0    -1  
$EndComp
NoConn ~ 4675 1725
$Comp
L power:+5V #PWR0115
U 1 1 60B34A09
P 9500 5225
F 0 "#PWR0115" H 9500 5075 50  0001 C CNN
F 1 "+5V" H 9515 5398 50  0000 C CNN
F 2 "" H 9500 5225 50  0001 C CNN
F 3 "" H 9500 5225 50  0001 C CNN
	1    9500 5225
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 60B3513A
P 10000 5425
F 0 "#PWR0117" H 10000 5175 50  0001 C CNN
F 1 "GND" H 10150 5350 50  0000 C CNN
F 2 "" H 10000 5425 50  0001 C CNN
F 3 "" H 10000 5425 50  0001 C CNN
	1    10000 5425
	1    0    0    -1  
$EndComp
Text GLabel 9500 5325 0    50   Input ~ 0
MIDI_MOSI_CHAIN
Text GLabel 3675 1475 0    50   Input ~ 0
MIDI_MISO_CHAIN
$Comp
L Device:R R101
U 1 1 60B3B958
P 3675 1275
F 0 "R101" H 3745 1321 50  0000 L CNN
F 1 "10k" H 3745 1230 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3605 1275 50  0001 C CNN
F 3 "~" H 3675 1275 50  0001 C CNN
	1    3675 1275
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 60B3D5B0
P 3550 1125
F 0 "#PWR0108" H 3550 875 50  0001 C CNN
F 1 "GND" H 3555 952 50  0000 C CNN
F 2 "" H 3550 1125 50  0001 C CNN
F 3 "" H 3550 1125 50  0001 C CNN
	1    3550 1125
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 1125 3675 1125
$Comp
L power:+5V #PWR0109
U 1 1 60B3DFFB
P 4175 1325
F 0 "#PWR0109" H 4175 1175 50  0001 C CNN
F 1 "+5V" H 4190 1498 50  0000 C CNN
F 2 "" H 4175 1325 50  0001 C CNN
F 3 "" H 4175 1325 50  0001 C CNN
	1    4175 1325
	1    0    0    -1  
$EndComp
Text GLabel 10000 5225 2    50   Input ~ 0
MIDI_MISO_CHAIN
Text GLabel 10000 5325 2    50   Input ~ 0
STROBE_MIDI_5
Text GLabel 9500 5425 0    50   Input ~ 0
SCK_MIDI_5
Text Notes 9475 5625 0    50   ~ 0
chain to slave
$Comp
L power:+5V #PWR0116
U 1 1 60B50985
P 9500 5950
F 0 "#PWR0116" H 9500 5800 50  0001 C CNN
F 1 "+5V" H 9515 6123 50  0000 C CNN
F 2 "" H 9500 5950 50  0001 C CNN
F 3 "" H 9500 5950 50  0001 C CNN
	1    9500 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 60B5098B
P 10000 6150
F 0 "#PWR0118" H 10000 5900 50  0001 C CNN
F 1 "GND" H 10150 6075 50  0000 C CNN
F 2 "" H 10000 6150 50  0001 C CNN
F 3 "" H 10000 6150 50  0001 C CNN
	1    10000 6150
	1    0    0    -1  
$EndComp
Text GLabel 10000 6050 2    50   Input ~ 0
STROBE_MIDI_5
Text GLabel 9500 6150 0    50   Input ~ 0
SCK_MIDI_5
Text Notes 9400 6375 0    50   ~ 0
chain from master
Text GLabel 9500 6050 0    50   Input ~ 0
MOSI_MIDI_5
Text GLabel 10000 5950 2    50   Input ~ 0
MISO_MIDI
$Sheet
S 2650 1675 525  800 
U 60B37C5E
F0 "8x TRS Midi input" 50
F1 "trsmidi_input_8x.sch" 50
F2 "in0" I R 3175 1725 50 
F3 "in1" I R 3175 1825 50 
F4 "in2" I R 3175 1925 50 
F5 "in3" I R 3175 2025 50 
F6 "in4" I R 3175 2125 50 
F7 "in5" I R 3175 2225 50 
F8 "in6" I R 3175 2325 50 
F9 "in7" I R 3175 2425 50 
$EndSheet
Wire Wire Line
	3675 1625 3675 1425
Wire Wire Line
	3175 1725 3675 1725
Wire Wire Line
	3675 1825 3175 1825
Wire Wire Line
	3175 1925 3675 1925
Wire Wire Line
	3675 2025 3175 2025
Wire Wire Line
	3175 2125 3675 2125
Wire Wire Line
	3175 2225 3675 2225
Wire Wire Line
	3675 2325 3175 2325
Wire Wire Line
	3175 2425 3675 2425
$Comp
L power:GND #PWR0110
U 1 1 60C5C9F3
P 4175 3225
F 0 "#PWR0110" H 4175 2975 50  0001 C CNN
F 1 "GND" H 4180 3052 50  0000 C CNN
F 2 "" H 4175 3225 50  0001 C CNN
F 3 "" H 4175 3225 50  0001 C CNN
	1    4175 3225
	1    0    0    -1  
$EndComp
Connection ~ 4175 3225
$Sheet
S 10050 1725 525  1600
U 60C86B49
F0 "8x TRS Midi output" 50
F1 "trs_midi_output_8x.sch" 50
F2 "out0a" I L 10050 1775 50 
F3 "out0b" I L 10050 1875 50 
F4 "out1a" I L 10050 1975 50 
F5 "out1b" I L 10050 2075 50 
F6 "out2a" I L 10050 2175 50 
F7 "out2b" I L 10050 2275 50 
F8 "out3a" I L 10050 2375 50 
F9 "out3b" I L 10050 2475 50 
F10 "out4a" I L 10050 2575 50 
F11 "out4b" I L 10050 2675 50 
F12 "out5a" I L 10050 2775 50 
F13 "out5b" I L 10050 2875 50 
F14 "out6a" I L 10050 2975 50 
F15 "out6b" I L 10050 3075 50 
F16 "out7a" I L 10050 3175 50 
F17 "out7b" I L 10050 3275 50 
$EndSheet
Text GLabel 8450 3175 0    50   Input ~ 0
SCK_MIDI_5
Text GLabel 8450 3475 0    50   Input ~ 0
STROBE_MIDI_5
Wire Wire Line
	8450 3275 7900 3275
Wire Wire Line
	7900 3275 7900 2775
Wire Wire Line
	7900 2775 8850 2775
Wire Wire Line
	8450 3575 8450 4075
Wire Wire Line
	8450 4075 8850 4075
$Comp
L power:GND #PWR0114
U 1 1 60D31571
P 8850 4075
F 0 "#PWR0114" H 8850 3825 50  0001 C CNN
F 1 "GND" H 8855 3902 50  0000 C CNN
F 2 "" H 8850 4075 50  0001 C CNN
F 3 "" H 8850 4075 50  0001 C CNN
	1    8850 4075
	1    0    0    -1  
$EndComp
Connection ~ 8850 4075
$Comp
L power:+5V #PWR0113
U 1 1 60D3157C
P 8850 2775
F 0 "#PWR0113" H 8850 2625 50  0001 C CNN
F 1 "+5V" H 8865 2948 50  0000 C CNN
F 2 "" H 8850 2775 50  0001 C CNN
F 3 "" H 8850 2775 50  0001 C CNN
	1    8850 2775
	1    0    0    -1  
$EndComp
Connection ~ 8850 2775
Wire Wire Line
	8450 2975 8350 2975
Wire Wire Line
	8350 2975 8350 2525
Wire Wire Line
	8350 2525 9350 2525
Wire Wire Line
	9350 2525 9350 2075
Wire Wire Line
	9350 2075 9250 2075
Wire Wire Line
	10000 1175 10000 1775
Wire Wire Line
	10000 1775 10050 1775
Wire Wire Line
	9250 1275 9975 1275
Wire Wire Line
	9975 1275 9975 1975
Wire Wire Line
	9975 1975 10050 1975
Wire Wire Line
	9250 1375 9950 1375
Wire Wire Line
	9950 1375 9950 2175
Wire Wire Line
	9950 2175 10050 2175
Wire Wire Line
	10050 2375 9925 2375
Wire Wire Line
	9925 2375 9925 1475
Wire Wire Line
	9925 1475 9250 1475
Wire Wire Line
	9250 1575 9875 1575
Wire Wire Line
	9875 1575 9875 1875
Wire Wire Line
	9875 1875 10050 1875
Wire Wire Line
	9250 1675 9850 1675
Wire Wire Line
	9850 1675 9850 2075
Wire Wire Line
	9850 2075 10050 2075
Wire Wire Line
	9250 1775 9825 1775
Wire Wire Line
	9825 1775 9825 2275
Wire Wire Line
	9825 2275 10050 2275
Wire Wire Line
	9250 1875 9800 1875
Wire Wire Line
	9800 1875 9800 2475
Wire Wire Line
	9800 2475 10050 2475
Wire Wire Line
	9250 3675 10000 3675
Wire Wire Line
	10000 3675 10000 3275
Wire Wire Line
	10000 3275 10050 3275
Wire Wire Line
	10050 3075 9975 3075
Wire Wire Line
	9975 3075 9975 3575
Wire Wire Line
	9975 3575 9250 3575
Wire Wire Line
	10050 2875 9950 2875
Wire Wire Line
	9250 3475 9950 3475
Wire Wire Line
	9950 2875 9950 3475
Wire Wire Line
	9250 3375 9925 3375
Wire Wire Line
	9925 3375 9925 2675
Wire Wire Line
	9925 2675 10050 2675
Wire Wire Line
	10050 3175 9875 3175
Wire Wire Line
	9875 3175 9875 3275
Wire Wire Line
	9875 3275 9250 3275
Wire Wire Line
	9250 3175 9850 3175
Wire Wire Line
	9850 3175 9850 2975
Wire Wire Line
	9850 2975 10050 2975
Wire Wire Line
	9250 3075 9825 3075
Wire Wire Line
	9825 3075 9825 2775
Wire Wire Line
	9825 2775 10050 2775
Wire Wire Line
	9250 2975 9800 2975
Wire Wire Line
	9800 2975 9800 2575
Wire Wire Line
	9800 2575 10050 2575
Text GLabel 9525 3875 2    50   Input ~ 0
MIDI_MOSI_CHAIN
Wire Wire Line
	8350 2525 8175 2525
Wire Wire Line
	7700 2525 7700 4375
Wire Wire Line
	7700 4375 9400 4375
Connection ~ 8350 2525
$Comp
L Jumper:SolderJumper_2_Open JP102
U 1 1 60D9961B
P 8025 2525
F 0 "JP102" H 8025 2730 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 8025 2639 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 8025 2525 50  0001 C CNN
F 3 "~" H 8025 2525 50  0001 C CNN
	1    8025 2525
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3875 9525 3875
Wire Wire Line
	9400 3875 9250 3875
Connection ~ 9400 3875
Wire Wire Line
	7875 2525 7700 2525
Wire Wire Line
	9400 3875 9400 4375
$Comp
L 74xx:74HC595 U104
U 1 1 60D3133D
P 8850 3375
F 0 "U104" H 9125 2600 50  0000 C CNN
F 1 "74HC595" H 9100 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 8850 3375 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8850 3375 50  0001 C CNN
	1    8850 3375
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 1175 10000 1175
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J102
U 1 1 60C0CBEA
P 9700 6050
F 0 "J102" H 9750 6275 50  0000 C CNN
F 1 "WSL 6G (box connector 6 pin)" H 9750 6276 50  0001 C CNN
F 2 "Connector_IDC:IDC-Header_2x03_P2.54mm_Vertical" H 9700 6050 50  0001 C CNN
F 3 "~" H 9700 6050 50  0001 C CNN
	1    9700 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J101
U 1 1 60C0AEA7
P 9700 5325
F 0 "J101" H 9750 5550 50  0000 C CNN
F 1 "WSL 6G (box connector 6 pin)" H 9750 5551 50  0001 C CNN
F 2 "Connector_IDC:IDC-Header_2x03_P2.54mm_Vertical" H 9700 5325 50  0001 C CNN
F 3 "~" H 9700 5325 50  0001 C CNN
	1    9700 5325
	1    0    0    -1  
$EndComp
Text Notes 550  7725 0    50   ~ 0
Copyright (c) 2021 Florian Jung.\n\nThis source describes Open Hardware and is licensed under the CERN-OHL-S v2.\n\nYou may redistribute and modify this source and make products using it under\nthe terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.txt).\n\nThis source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,\nINCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A\nPARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.\n\nSource location: https://github.com/Windfisch/midikraken\n\nAs per CERN-OHL-S v2 section 4, should You produce hardware based on this\nsource, You must where practicable maintain the Source Location visible\non the external case of the Gizmo or other products you make using this\nsource.\n
$EndSCHEMATC
