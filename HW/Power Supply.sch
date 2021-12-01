EESchema Schematic File Version 4
LIBS:Dash-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 1050 1250 0    50   Input ~ 0
+24v
Text HLabel 1750 2100 0    50   Input ~ 0
GND
$Comp
L Dash-rescue:LMZ14202TZE-ADJ_NOPB-LMZ14202TZE-ADJ_NOPB U5
U 1 1 5BCBC45C
P 3750 1800
F 0 "U5" H 4350 2065 50  0000 C CNN
F 1 "LMZ14202TZE-ADJ_NOPB" H 4350 1974 50  0000 C CNN
F 2 "footprint:TO-PMOD-7" H 4800 1900 50  0001 L CNN
F 3 "http://uk.rs-online.com/web/p/products/7981195P" H 4800 1800 50  0001 L CNN
F 4 "Voltage Regulators - Switching Regulators 2A pwr Module w/ 42V Max Input Vtg" H 4800 1700 50  0001 L CNN "Description"
F 5 "" H 4800 1600 50  0001 L CNN "Height"
F 6 "7981195P" H 4800 1500 50  0001 L CNN "RS Part Number"
F 7 "Texas Instruments" H 4800 1400 50  0001 L CNN "Manufacturer_Name"
F 8 "LMZ14202TZE-ADJ/NOPB" H 4800 1300 50  0001 L CNN "Manufacturer_Part_Number"
	1    3750 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R29
U 1 1 5BCBC53D
P 2700 1400
F 0 "R29" H 2770 1446 50  0000 L CNN
F 1 "68.1k" H 2770 1355 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 2630 1400 50  0001 C CNN
F 3 "~" H 2700 1400 50  0001 C CNN
	1    2700 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R30
U 1 1 5BCBC5FB
P 2700 1950
F 0 "R30" H 2770 1996 50  0000 L CNN
F 1 "11.8k" H 2770 1905 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 2630 1950 50  0001 C CNN
F 3 "~" H 2700 1950 50  0001 C CNN
	1    2700 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R31
U 1 1 5BCBC617
P 3200 1400
F 0 "R31" H 3270 1446 50  0000 L CNN
F 1 "100k" H 3270 1355 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 3130 1400 50  0001 C CNN
F 3 "~" H 3200 1400 50  0001 C CNN
	1    3200 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R34
U 1 1 5BCBC633
P 6050 1900
F 0 "R34" H 6120 1946 50  0000 L CNN
F 1 "5.62k" H 6120 1855 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 5980 1900 50  0001 C CNN
F 3 "~" H 6050 1900 50  0001 C CNN
	1    6050 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R32
U 1 1 5BCBC6C9
P 5600 2050
F 0 "R32" H 5670 2096 50  0000 L CNN
F 1 "1.07k" H 5670 2005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 5530 2050 50  0001 C CNN
F 3 "~" H 5600 2050 50  0001 C CNN
	1    5600 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C21
U 1 1 5BCBC70B
P 6050 1350
F 0 "C21" V 5798 1350 50  0000 C CNN
F 1 "22n" V 5889 1350 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 6088 1200 50  0001 C CNN
F 3 "~" H 6050 1350 50  0001 C CNN
	1    6050 1350
	0    1    1    0   
$EndComp
$Comp
L Device:C C22
U 1 1 5BCBC7EC
P 6450 2050
F 0 "C22" H 6565 2096 50  0000 L CNN
F 1 "1u" H 6565 2005 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6488 1900 50  0001 C CNN
F 3 "~" H 6450 2050 50  0001 C CNN
	1    6450 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 5BCBC818
P 6900 2050
F 0 "C23" H 7015 2096 50  0000 L CNN
F 1 "100u" H 7015 2005 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210" H 6938 1900 50  0001 C CNN
F 3 "~" H 6900 2050 50  0001 C CNN
	1    6900 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5BCBC87E
P 2200 1650
F 0 "C6" H 2315 1696 50  0000 L CNN
F 1 "10u" H 2315 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2238 1500 50  0001 C CNN
F 3 "~" H 2200 1650 50  0001 C CNN
	1    2200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5BCBCB60
P 1750 1650
F 0 "C5" H 1865 1696 50  0000 L CNN
F 1 "1u" H 1865 1605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1788 1500 50  0001 C CNN
F 3 "~" H 1750 1650 50  0001 C CNN
	1    1750 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5BCBCBAA
P 5100 1400
F 0 "C19" H 5215 1446 50  0000 L CNN
F 1 "22n" H 5215 1355 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5138 1250 50  0001 C CNN
F 3 "~" H 5100 1400 50  0001 C CNN
	1    5100 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1800 5100 1800
Wire Wire Line
	5100 1800 5100 1550
Wire Wire Line
	4950 1900 5600 1900
Wire Wire Line
	5600 1900 5900 1900
Connection ~ 5600 1900
Wire Wire Line
	5900 1350 5600 1350
Wire Wire Line
	5600 1350 5600 1900
Wire Wire Line
	6200 1900 6450 1900
Connection ~ 6450 1900
Wire Wire Line
	6200 1350 6450 1350
Wire Wire Line
	6450 1350 6450 1900
Wire Wire Line
	4950 2100 5150 2100
Wire Wire Line
	5150 2100 5150 2200
Wire Wire Line
	5150 2200 5600 2200
Connection ~ 5600 2200
Wire Wire Line
	5600 2200 6050 2200
Connection ~ 6450 2200
Wire Wire Line
	6450 2200 6800 2200
Wire Wire Line
	3200 1550 3200 1900
Wire Wire Line
	3200 1900 3750 1900
Wire Wire Line
	3750 1800 3550 1800
Wire Wire Line
	3550 1250 3200 1250
Wire Wire Line
	1750 1250 1750 1500
Wire Wire Line
	3550 1250 3550 1800
Connection ~ 2700 1250
Wire Wire Line
	2700 1250 2200 1250
Connection ~ 3200 1250
Wire Wire Line
	3200 1250 2700 1250
Connection ~ 2200 1250
Wire Wire Line
	2200 1250 1750 1250
Wire Wire Line
	2200 1250 2200 1500
Wire Wire Line
	2700 1550 2700 1700
Wire Wire Line
	3750 2000 3050 2000
Wire Wire Line
	3050 2000 3050 1700
Wire Wire Line
	3050 1700 2700 1700
Connection ~ 2700 1700
Wire Wire Line
	2700 1700 2700 1800
Wire Wire Line
	2200 2100 2700 2100
Wire Wire Line
	2200 1800 2200 2100
Connection ~ 2700 2100
Wire Wire Line
	1750 1800 1750 2100
Wire Wire Line
	1750 2100 2200 2100
Connection ~ 2200 2100
$Comp
L power:GND #PWR015
U 1 1 5BCC5A79
P 2700 2100
F 0 "#PWR015" H 2700 1850 50  0001 C CNN
F 1 "GND" H 2705 1927 50  0000 C CNN
F 2 "" H 2700 2100 50  0001 C CNN
F 3 "" H 2700 2100 50  0001 C CNN
	1    2700 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5BCC5AF1
P 6050 2200
F 0 "#PWR017" H 6050 1950 50  0001 C CNN
F 1 "GND" H 6055 2027 50  0000 C CNN
F 2 "" H 6050 2200 50  0001 C CNN
F 3 "" H 6050 2200 50  0001 C CNN
	1    6050 2200
	1    0    0    -1  
$EndComp
Connection ~ 6050 2200
Wire Wire Line
	6050 2200 6450 2200
$Comp
L power:GND #PWR016
U 1 1 5BCC5BB9
P 5100 1250
F 0 "#PWR016" H 5100 1000 50  0001 C CNN
F 1 "GND" H 5105 1077 50  0000 C CNN
F 2 "" H 5100 1250 50  0001 C CNN
F 3 "" H 5100 1250 50  0001 C CNN
	1    5100 1250
	-1   0    0    1   
$EndComp
$Comp
L Device:D D1
U 1 1 5BCF3289
P 1350 1250
F 0 "D1" H 1350 1034 50  0000 C CNN
F 1 "D" H 1350 1125 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 1350 1250 50  0001 C CNN
F 3 "~" H 1350 1250 50  0001 C CNN
	1    1350 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	1500 1250 1750 1250
Connection ~ 1750 1250
Wire Wire Line
	1200 1250 1050 1250
$Comp
L Device:C C15
U 1 1 5BD2345C
P 2700 3700
F 0 "C15" H 2815 3746 50  0000 L CNN
F 1 "1u" H 2815 3655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2738 3550 50  0001 C CNN
F 3 "~" H 2700 3700 50  0001 C CNN
	1    2700 3700
	1    0    0    -1  
$EndComp
Connection ~ 2700 3550
Wire Wire Line
	2700 3550 2450 3550
Wire Wire Line
	2700 3850 2700 4050
Wire Wire Line
	3850 3550 3950 3550
Wire Wire Line
	4550 3550 4550 3650
Connection ~ 4550 3550
Wire Wire Line
	4550 3950 4550 4050
Connection ~ 4550 4050
Wire Wire Line
	4550 4050 5000 4050
Text HLabel 5450 3550 2    50   Input ~ 0
+3.3v
Text Label 7250 1850 0    50   ~ 0
+5v
Text Label 2450 3550 2    50   ~ 0
+5v
Wire Wire Line
	2700 2100 3750 2100
Wire Notes Line
	6150 3000 6150 4350
Wire Notes Line
	6150 4350 2100 4350
Wire Notes Line
	2100 4350 2100 3000
Wire Notes Line
	2100 3000 6150 3000
Connection ~ 6900 1900
Wire Wire Line
	7250 1900 7250 1850
Text Label 4950 2000 0    50   ~ 0
+5v
Wire Wire Line
	6900 1900 7250 1900
Wire Wire Line
	6450 1900 6900 1900
$Comp
L Device:R R35
U 1 1 5C09D9C3
P 7400 2050
F 0 "R35" H 7470 2096 50  0000 L CNN
F 1 "1.2k" H 7470 2005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 7330 2050 50  0001 C CNN
F 3 "~" H 7400 2050 50  0001 C CNN
	1    7400 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1900 7400 1900
Connection ~ 7250 1900
$Comp
L Device:LED D3
U 1 1 5C09F054
P 7400 2350
F 0 "D3" V 7438 2233 50  0000 R CNN
F 1 "LED" V 7347 2233 50  0000 R CNN
F 2 "Diodes_SMD:D_0805" H 7400 2350 50  0001 C CNN
F 3 "~" H 7400 2350 50  0001 C CNN
	1    7400 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7400 2500 6800 2500
Wire Wire Line
	6800 2500 6800 2200
Connection ~ 6800 2200
Wire Wire Line
	6800 2200 6900 2200
$Comp
L Device:R R33
U 1 1 5C0A1E4A
P 5400 3700
F 0 "R33" H 5470 3746 50  0000 L CNN
F 1 "1.2k" H 5470 3655 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 5330 3700 50  0001 C CNN
F 3 "~" H 5400 3700 50  0001 C CNN
	1    5400 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5C0A1E51
P 5400 4000
F 0 "D2" V 5438 3883 50  0000 R CNN
F 1 "LED" V 5347 3883 50  0000 R CNN
F 2 "Diodes_SMD:D_0805" H 5400 4000 50  0001 C CNN
F 3 "~" H 5400 4000 50  0001 C CNN
	1    5400 4000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5000 4050 5000 4150
Wire Wire Line
	5000 4150 5400 4150
Connection ~ 5400 3550
Wire Wire Line
	5400 3550 5450 3550
Wire Notes Line
	650  2650 7850 2650
Wire Notes Line
	7850 850  7850 2650
Wire Notes Line
	650  850  7850 850 
Wire Notes Line
	650  850  650  2650
Text Label 4550 3550 0    50   ~ 0
+3.3v
Wire Wire Line
	2700 4050 3550 4050
Wire Wire Line
	2700 3550 3250 3550
Connection ~ 3550 4050
Wire Wire Line
	3550 4050 4100 4050
$Comp
L power:GND #PWR0125
U 1 1 5BF80FC3
P 3550 4050
F 0 "#PWR0125" H 3550 3800 50  0001 C CNN
F 1 "GND" H 3555 3877 50  0000 C CNN
F 2 "" H 3550 4050 50  0001 C CNN
F 3 "" H 3550 4050 50  0001 C CNN
	1    3550 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3850 3550 4050
$Comp
L Dash-rescue:LM1117-3.3-Regulator_Linear U3
U 1 1 5BFC2690
P 3550 3550
F 0 "U3" H 3550 3792 50  0000 C CNN
F 1 "LM1117-3.3" H 3550 3701 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 3550 3550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 3550 3550 50  0001 C CNN
F 4 "LM1117MP-3.3/NOPB" H 0   0   50  0001 C CNN "Manufacturer_Part_Number"
	1    3550 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3550 5400 3550
$Comp
L Device:C C33
U 1 1 5BFBE3F0
P 4100 3800
F 0 "C33" H 4215 3846 50  0000 L CNN
F 1 "100n" H 4215 3755 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4138 3650 50  0001 C CNN
F 3 "~" H 4100 3800 50  0001 C CNN
	1    4100 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 3650 3950 3650
Wire Wire Line
	3950 3650 3950 3550
Connection ~ 3950 3550
Wire Wire Line
	3950 3550 4100 3550
Wire Wire Line
	4100 3650 4100 3550
Connection ~ 4100 3550
Wire Wire Line
	4100 3550 4550 3550
Wire Wire Line
	4100 3950 4100 4050
Connection ~ 4100 4050
Wire Wire Line
	4100 4050 4550 4050
$Comp
L Device:C C34
U 1 1 5C147131
P 4550 3800
F 0 "C34" H 4665 3846 50  0000 L CNN
F 1 "10u" H 4665 3755 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4588 3650 50  0001 C CNN
F 3 "~" H 4550 3800 50  0001 C CNN
	1    4550 3800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
