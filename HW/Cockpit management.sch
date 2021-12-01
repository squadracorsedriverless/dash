EESchema Schematic File Version 4
LIBS:Dash-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 3 4
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
L Device:R R6
U 1 1 5BD087B3
P 2550 1850
F 0 "R6" H 2480 1804 50  0000 R CNN
F 1 "100k" H 2480 1895 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2480 1850 50  0001 C CNN
F 3 "~" H 2550 1850 50  0001 C CNN
	1    2550 1850
	-1   0    0    1   
$EndComp
Connection ~ 2550 1700
Wire Wire Line
	3300 1500 3300 1200
$Comp
L power:GND #PWR04
U 1 1 5BD0881D
P 2550 2100
F 0 "#PWR04" H 2550 1850 50  0001 C CNN
F 1 "GND" H 2555 1927 50  0000 C CNN
F 2 "" H 2550 2100 50  0001 C CNN
F 3 "" H 2550 2100 50  0001 C CNN
	1    2550 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 2100 3300 1900
Connection ~ 2550 2100
Wire Wire Line
	2550 2100 3300 2100
Wire Wire Line
	2550 2100 2550 2000
Text HLabel 3300 1200 2    50   Input ~ 0
IMD_LED
Text HLabel 1850 1700 0    50   Input ~ 0
IMD_LED_CMD
Wire Wire Line
	3300 2650 3300 2350
$Comp
L power:GND #PWR05
U 1 1 5BD08C6B
P 2550 3250
F 0 "#PWR05" H 2550 3000 50  0001 C CNN
F 1 "GND" H 2555 3077 50  0000 C CNN
F 2 "" H 2550 3250 50  0001 C CNN
F 3 "" H 2550 3250 50  0001 C CNN
	1    2550 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3250 3300 3050
Connection ~ 2550 3250
Wire Wire Line
	2550 3250 3300 3250
Wire Wire Line
	2550 3250 2550 3150
Text HLabel 3300 2350 2    50   Input ~ 0
BMS_LED
Text HLabel 1850 2850 0    50   Input ~ 0
BMS_LED_CMD
Wire Wire Line
	3300 3950 3300 3650
$Comp
L power:GND #PWR06
U 1 1 5BD08E1E
P 2700 4550
F 0 "#PWR06" H 2700 4300 50  0001 C CNN
F 1 "GND" H 2705 4377 50  0000 C CNN
F 2 "" H 2700 4550 50  0001 C CNN
F 3 "" H 2700 4550 50  0001 C CNN
	1    2700 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 4550 3300 4350
Text HLabel 3300 3650 2    50   Input ~ 0
RTD_LED
Text HLabel 1850 4150 0    50   Input ~ 0
RTD_LED_CMD
Wire Wire Line
	3300 5100 3300 4800
$Comp
L power:GND #PWR07
U 1 1 5BD092CF
P 2550 5700
F 0 "#PWR07" H 2550 5450 50  0001 C CNN
F 1 "GND" H 2555 5527 50  0000 C CNN
F 2 "" H 2550 5700 50  0001 C CNN
F 3 "" H 2550 5700 50  0001 C CNN
	1    2550 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5700 3300 5500
Connection ~ 2550 5700
Wire Wire Line
	2550 5700 3300 5700
Wire Wire Line
	2550 5700 2550 5600
Text HLabel 3300 4800 2    50   Input ~ 0
BUZZER
Text HLabel 1850 5300 0    50   Input ~ 0
BUZZER_CMD
Wire Wire Line
	3300 6200 3300 5900
$Comp
L power:GND #PWR08
U 1 1 5BD09A0B
P 2550 6800
F 0 "#PWR08" H 2550 6550 50  0001 C CNN
F 1 "GND" H 2555 6627 50  0000 C CNN
F 2 "" H 2550 6800 50  0001 C CNN
F 3 "" H 2550 6800 50  0001 C CNN
	1    2550 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 6800 3300 6600
Connection ~ 2550 6800
Wire Wire Line
	2550 6800 3300 6800
Wire Wire Line
	2550 6800 2550 6700
Text HLabel 3300 5900 2    50   Input ~ 0
NO_HV_LED
Text HLabel 1850 6400 0    50   Input ~ 0
NO_HV_LED_CMD
Wire Wire Line
	3300 7450 3300 7150
$Comp
L power:GND #PWR09
U 1 1 5BD0B1D1
P 2550 8050
F 0 "#PWR09" H 2550 7800 50  0001 C CNN
F 1 "GND" H 2555 7877 50  0000 C CNN
F 2 "" H 2550 8050 50  0001 C CNN
F 3 "" H 2550 8050 50  0001 C CNN
	1    2550 8050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 8050 3300 7850
Connection ~ 2550 8050
Wire Wire Line
	2550 8050 3300 8050
Wire Wire Line
	2550 8050 2550 7950
Text HLabel 3700 7150 2    50   Input ~ 0
PUMP_AUTO
Text HLabel 1850 7650 0    50   Input ~ 0
PUMP_CMD
Wire Wire Line
	3300 7150 3700 7150
Wire Wire Line
	3300 8700 3300 8400
$Comp
L power:GND #PWR010
U 1 1 5BD0D351
P 2550 9300
F 0 "#PWR010" H 2550 9050 50  0001 C CNN
F 1 "GND" H 2555 9127 50  0000 C CNN
F 2 "" H 2550 9300 50  0001 C CNN
F 3 "" H 2550 9300 50  0001 C CNN
	1    2550 9300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 9300 3300 9100
Connection ~ 2550 9300
Wire Wire Line
	2550 9300 3300 9300
Wire Wire Line
	2550 9300 2550 9200
Text HLabel 3700 8400 2    50   Input ~ 0
BP_FAN_AUTO
Text HLabel 1850 8900 0    50   Input ~ 0
BP_FAN_CMD
Wire Wire Line
	3300 8400 3700 8400
Wire Wire Line
	3300 9800 3300 9500
$Comp
L power:GND #PWR011
U 1 1 5BD0E295
P 2700 10400
F 0 "#PWR011" H 2700 10150 50  0001 C CNN
F 1 "GND" H 2705 10227 50  0000 C CNN
F 2 "" H 2700 10400 50  0001 C CNN
F 3 "" H 2700 10400 50  0001 C CNN
	1    2700 10400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 10400 3300 10200
Text HLabel 3700 9500 2    50   Input ~ 0
RAD_FAN_AUTO
Text HLabel 1850 10000 0    50   Input ~ 0
RAD_FAN_CMD
Wire Wire Line
	3300 9500 3700 9500
$Comp
L Device:R R7
U 1 1 5BD0FED0
P 2550 3000
F 0 "R7" H 2480 2954 50  0000 R CNN
F 1 "100k" H 2480 3045 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2480 3000 50  0001 C CNN
F 3 "~" H 2550 3000 50  0001 C CNN
	1    2550 3000
	-1   0    0    1   
$EndComp
Connection ~ 2550 2850
$Comp
L Device:R R8
U 1 1 5BD0FFFD
P 2700 4300
F 0 "R8" H 2630 4254 50  0000 R CNN
F 1 "100k" H 2630 4345 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2630 4300 50  0001 C CNN
F 3 "~" H 2700 4300 50  0001 C CNN
	1    2700 4300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R9
U 1 1 5BD1134F
P 2550 5450
F 0 "R9" H 2480 5404 50  0000 R CNN
F 1 "100k" H 2480 5495 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2480 5450 50  0001 C CNN
F 3 "~" H 2550 5450 50  0001 C CNN
	1    2550 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R10
U 1 1 5BD12662
P 2550 6550
F 0 "R10" H 2480 6504 50  0000 R CNN
F 1 "100k" H 2480 6595 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2480 6550 50  0001 C CNN
F 3 "~" H 2550 6550 50  0001 C CNN
	1    2550 6550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R16
U 1 1 5BD1C86B
P 2550 7800
F 0 "R16" H 2480 7754 50  0000 R CNN
F 1 "100k" H 2480 7845 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2480 7800 50  0001 C CNN
F 3 "~" H 2550 7800 50  0001 C CNN
	1    2550 7800
	-1   0    0    1   
$EndComp
$Comp
L Device:R R17
U 1 1 5BD1DAE2
P 2550 9050
F 0 "R17" H 2480 9004 50  0000 R CNN
F 1 "100k" H 2480 9095 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2480 9050 50  0001 C CNN
F 3 "~" H 2550 9050 50  0001 C CNN
	1    2550 9050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R18
U 1 1 5BD1ECD9
P 2700 10150
F 0 "R18" H 2630 10104 50  0000 R CNN
F 1 "100k" H 2630 10195 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 2630 10150 50  0001 C CNN
F 3 "~" H 2700 10150 50  0001 C CNN
	1    2700 10150
	-1   0    0    1   
$EndComp
Connection ~ 2550 8900
Connection ~ 2550 7650
Connection ~ 2550 5300
Connection ~ 2550 6400
Wire Wire Line
	2700 4450 2700 4550
Connection ~ 2700 4550
Wire Wire Line
	2700 4550 3300 4550
Wire Wire Line
	2700 4150 3000 4150
Connection ~ 2700 4150
Wire Wire Line
	2550 1700 3000 1700
Wire Wire Line
	2550 2850 3000 2850
Wire Wire Line
	2550 6400 3000 6400
Wire Wire Line
	2550 7650 3000 7650
Wire Wire Line
	2550 8900 3000 8900
Wire Wire Line
	2700 10300 2700 10400
Connection ~ 2700 10400
Wire Wire Line
	2700 10400 3300 10400
Wire Wire Line
	2700 10000 3000 10000
Connection ~ 2700 10000
$Comp
L Device:R R37
U 1 1 5C000234
P 5950 2850
F 0 "R37" H 5880 2804 50  0000 R CNN
F 1 "100k" H 5880 2895 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 5880 2850 50  0001 C CNN
F 3 "~" H 5950 2850 50  0001 C CNN
	1    5950 2850
	-1   0    0    1   
$EndComp
$Comp
L Device:C C32
U 1 1 5C001C47
P 6650 3150
F 0 "C32" H 6765 3196 50  0000 L CNN
F 1 "100n" H 6765 3105 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6688 3000 50  0001 C CNN
F 3 "~" H 6650 3150 50  0001 C CNN
	1    6650 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 3000 6800 3000
Wire Wire Line
	7100 3300 7100 3200
Connection ~ 6650 3300
Wire Wire Line
	6650 3300 7100 3300
$Comp
L Device:R R36
U 1 1 5C005716
P 6350 3000
F 0 "R36" H 6280 2954 50  0000 R CNN
F 1 "15k" H 6280 3045 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 6280 3000 50  0001 C CNN
F 3 "~" H 6350 3000 50  0001 C CNN
	1    6350 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 2700 5950 2550
$Comp
L power:+3V3 #PWR0122
U 1 1 5C00DB19
P 5950 2550
F 0 "#PWR0122" H 5950 2400 50  0001 C CNN
F 1 "+3V3" H 5965 2723 50  0000 C CNN
F 2 "" H 5950 2550 50  0001 C CNN
F 3 "" H 5950 2550 50  0001 C CNN
	1    5950 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0123
U 1 1 5C00DB65
P 7100 2400
F 0 "#PWR0123" H 7100 2250 50  0001 C CNN
F 1 "+3V3" H 7115 2573 50  0000 C CNN
F 2 "" H 7100 2400 50  0001 C CNN
F 3 "" H 7100 2400 50  0001 C CNN
	1    7100 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R38
U 1 1 5C00DBAA
P 7100 2650
F 0 "R38" H 7030 2604 50  0000 R CNN
F 1 "15k" H 7030 2695 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 7030 2650 50  0001 C CNN
F 3 "~" H 7100 2650 50  0001 C CNN
	1    7100 2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	7100 2400 7100 2500
$Comp
L power:GND #PWR0124
U 1 1 5C01152C
P 6650 3300
F 0 "#PWR0124" H 6650 3050 50  0001 C CNN
F 1 "GND" H 6655 3127 50  0000 C CNN
F 2 "" H 6650 3300 50  0001 C CNN
F 3 "" H 6650 3300 50  0001 C CNN
	1    6650 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q14
U 1 1 5C091CBE
P 7000 3000
F 0 "Q14" H 7205 3046 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7205 2955 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 7200 3100 50  0001 C CNN
F 3 "~" H 7000 3000 50  0001 C CNN
	1    7000 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 5C094274
P 3200 1700
F 0 "Q1" H 3405 1746 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 1655 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 1800 50  0001 C CNN
F 3 "~" H 3200 1700 50  0001 C CNN
	1    3200 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q2
U 1 1 5C094365
P 3200 2850
F 0 "Q2" H 3405 2896 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 2805 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 2950 50  0001 C CNN
F 3 "~" H 3200 2850 50  0001 C CNN
	1    3200 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q3
U 1 1 5C09443F
P 3200 4150
F 0 "Q3" H 3405 4196 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 4105 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 4250 50  0001 C CNN
F 3 "~" H 3200 4150 50  0001 C CNN
	1    3200 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q4
U 1 1 5C0944F6
P 3200 5300
F 0 "Q4" H 3405 5346 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 5255 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 5400 50  0001 C CNN
F 3 "~" H 3200 5300 50  0001 C CNN
	1    3200 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q5
U 1 1 5C094580
P 3200 6400
F 0 "Q5" H 3405 6446 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 6355 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 6500 50  0001 C CNN
F 3 "~" H 3200 6400 50  0001 C CNN
	1    3200 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q6
U 1 1 5C094658
P 3200 7650
F 0 "Q6" H 3405 7696 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 7605 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 7750 50  0001 C CNN
F 3 "~" H 3200 7650 50  0001 C CNN
	1    3200 7650
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q7
U 1 1 5C094736
P 3200 8900
F 0 "Q7" H 3405 8946 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 8855 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 9000 50  0001 C CNN
F 3 "~" H 3200 8900 50  0001 C CNN
	1    3200 8900
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q8
U 1 1 5C094E27
P 3200 10000
F 0 "Q8" H 3405 10046 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 3405 9955 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 3400 10100 50  0001 C CNN
F 3 "~" H 3200 10000 50  0001 C CNN
	1    3200 10000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 5300 3000 5300
Wire Wire Line
	7100 2800 7500 2800
Connection ~ 7100 2800
Text HLabel 7500 2800 2    50   Input ~ 0
RTD_IN
Text HLabel 5400 3000 0    50   Input ~ 0
RTD
Wire Wire Line
	5950 3000 6200 3000
Wire Wire Line
	6500 3000 6650 3000
Connection ~ 6650 3000
Wire Wire Line
	5400 3000 5950 3000
Connection ~ 5950 3000
$Comp
L Device:R R40
U 1 1 5C1306A0
P 6050 4350
F 0 "R40" H 5980 4304 50  0000 R CNN
F 1 "100k" H 5980 4395 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 5980 4350 50  0001 C CNN
F 3 "~" H 6050 4350 50  0001 C CNN
	1    6050 4350
	-1   0    0    1   
$EndComp
$Comp
L Device:C C18
U 1 1 5C1306A7
P 6750 4650
F 0 "C18" H 6865 4696 50  0000 L CNN
F 1 "100n" H 6865 4605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6788 4500 50  0001 C CNN
F 3 "~" H 6750 4650 50  0001 C CNN
	1    6750 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 4500 6900 4500
Wire Wire Line
	7200 4800 7200 4700
Connection ~ 6750 4800
Wire Wire Line
	6750 4800 7200 4800
$Comp
L Device:R R41
U 1 1 5C1306B2
P 6450 4500
F 0 "R41" H 6380 4454 50  0000 R CNN
F 1 "15k" H 6380 4545 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 6380 4500 50  0001 C CNN
F 3 "~" H 6450 4500 50  0001 C CNN
	1    6450 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 4200 6050 4050
$Comp
L power:+3V3 #PWR0128
U 1 1 5C1306BA
P 6050 4050
F 0 "#PWR0128" H 6050 3900 50  0001 C CNN
F 1 "+3V3" H 6065 4223 50  0000 C CNN
F 2 "" H 6050 4050 50  0001 C CNN
F 3 "" H 6050 4050 50  0001 C CNN
	1    6050 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0129
U 1 1 5C1306C0
P 7200 3900
F 0 "#PWR0129" H 7200 3750 50  0001 C CNN
F 1 "+3V3" H 7215 4073 50  0000 C CNN
F 2 "" H 7200 3900 50  0001 C CNN
F 3 "" H 7200 3900 50  0001 C CNN
	1    7200 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R42
U 1 1 5C1306C6
P 7200 4150
F 0 "R42" H 7130 4104 50  0000 R CNN
F 1 "15k" H 7130 4195 50  0000 R CNN
F 2 "Resistors_SMD:R_0603" V 7130 4150 50  0001 C CNN
F 3 "~" H 7200 4150 50  0001 C CNN
	1    7200 4150
	-1   0    0    1   
$EndComp
Wire Wire Line
	7200 3900 7200 4000
$Comp
L power:GND #PWR0130
U 1 1 5C1306CE
P 6750 4800
F 0 "#PWR0130" H 6750 4550 50  0001 C CNN
F 1 "GND" H 6755 4627 50  0000 C CNN
F 2 "" H 6750 4800 50  0001 C CNN
F 3 "" H 6750 4800 50  0001 C CNN
	1    6750 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q15
U 1 1 5C1306D4
P 7100 4500
F 0 "Q15" H 7305 4546 50  0000 L CNN
F 1 "Q_NMOS_GSD" H 7305 4455 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70" H 7300 4600 50  0001 C CNN
F 3 "~" H 7100 4500 50  0001 C CNN
	1    7100 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4300 7600 4300
Connection ~ 7200 4300
Text HLabel 5500 4500 0    50   Input ~ 0
Spare_Button_In
Wire Wire Line
	6050 4500 6300 4500
Wire Wire Line
	6600 4500 6750 4500
Connection ~ 6750 4500
Wire Wire Line
	5500 4500 6050 4500
Connection ~ 6050 4500
Text HLabel 7600 4300 2    50   Input ~ 0
Spare_Button
Wire Wire Line
	1850 2850 2550 2850
Wire Wire Line
	1850 1700 2550 1700
Wire Wire Line
	1850 4150 2700 4150
Wire Wire Line
	1850 5300 2550 5300
Wire Wire Line
	1850 6400 2550 6400
Wire Wire Line
	1850 7650 2550 7650
Wire Wire Line
	1850 8900 2550 8900
Wire Wire Line
	1850 10000 2700 10000
$EndSCHEMATC
