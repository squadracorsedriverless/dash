EESchema Schematic File Version 4
LIBS:Dash-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1800 2700 1800 1900
U 5BD043C1
F0 "MCU" 50
F1 "MCU.sch" 50
F2 "BMS_Led_Cmd" I R 3600 3000 50 
F3 "Imd_Err_Led_Cmd" I R 3600 2850 50 
F4 "Rtd_Led" I R 3600 3150 50 
F5 "Fan_Rad_Cmd" I R 3600 3900 50 
F6 "Pump_Cmd" I R 3600 3600 50 
F7 "Fan_Bp_Cmd" I R 3600 3750 50 
F8 "Buzzer" I R 3600 3300 50 
F9 "No_Hv_Led" I R 3600 3450 50 
F10 "RTD_Button" I R 3600 4450 50 
F11 "CAN_H" O R 3600 4100 50 
F12 "CAN_L" O R 3600 4250 50 
F13 "+3.3VD" I L 1800 2850 50 
F14 "spare_button" I R 3600 4550 50 
$EndSheet
$Sheet
S 5400 2700 1900 1650
U 5BD043E3
F0 "Cockpit management" 50
F1 "Cockpit management.sch" 50
F2 "IMD_LED" I R 7300 2850 50 
F3 "IMD_LED_CMD" I L 5400 2850 50 
F4 "BMS_LED" I R 7300 3000 50 
F5 "BMS_LED_CMD" I L 5400 3000 50 
F6 "RTD_LED" I R 7300 3150 50 
F7 "RTD_LED_CMD" I L 5400 3150 50 
F8 "BUZZER" I R 7300 3300 50 
F9 "BUZZER_CMD" I L 5400 3300 50 
F10 "NO_HV_LED" I R 7300 3450 50 
F11 "NO_HV_LED_CMD" I L 5400 3450 50 
F12 "PUMP_AUTO" I R 7300 3600 50 
F13 "PUMP_CMD" I L 5400 3600 50 
F14 "BP_FAN_AUTO" I R 7300 3750 50 
F15 "BP_FAN_CMD" I L 5400 3750 50 
F16 "RAD_FAN_AUTO" I R 7300 3900 50 
F17 "RAD_FAN_CMD" I L 5400 3900 50 
F18 "RTD_IN" I L 5400 4100 50 
F19 "RTD" I R 7300 4100 50 
F20 "Spare_Button_In" I R 7300 4250 50 
F21 "Spare_Button" I L 5400 4250 50 
$EndSheet
Wire Wire Line
	5400 2850 3600 2850
Wire Wire Line
	5400 3000 3600 3000
Wire Wire Line
	5400 3150 3600 3150
Wire Wire Line
	5400 3300 3600 3300
Wire Wire Line
	5400 3450 3600 3450
Wire Wire Line
	5400 3600 3600 3600
Wire Wire Line
	5400 3750 3600 3750
Wire Wire Line
	5400 3900 3600 3900
$Sheet
S 6000 1350 550  600 
U 5BD39470
F0 "Power Supply" 50
F1 "Power Supply.sch" 50
F2 "+24v" I R 6550 1450 50 
F3 "GND" I R 6550 1550 50 
F4 "+3.3v" I R 6550 1700 50 
$EndSheet
Wire Wire Line
	6550 1450 7250 1450
Wire Wire Line
	6550 1550 7250 1550
Wire Wire Line
	6550 1700 7250 1700
Wire Wire Line
	7250 1700 7250 2400
Wire Wire Line
	7250 2400 1600 2400
Wire Wire Line
	1600 2400 1600 2850
Wire Wire Line
	1600 2850 1800 2850
Wire Wire Line
	3600 4100 3850 4100
Wire Wire Line
	3600 4250 3850 4250
Text Label 3850 4100 0    50   ~ 0
CAN_H
Text Label 3850 4250 0    50   ~ 0
CAN_L
Text Label 9100 4800 2    50   ~ 0
CAN_H
Text Label 9100 4900 2    50   ~ 0
CAN_L
Text Label 7250 1450 0    50   ~ 0
+24v
Text Label 7250 1550 0    50   ~ 0
GND
Text Label 9100 4200 2    50   ~ 0
GND
Wire Wire Line
	4800 4450 4800 4100
Wire Wire Line
	4800 4100 5400 4100
Wire Wire Line
	3600 4450 4800 4450
Text Label 9100 4500 2    50   ~ 0
+24v
Wire Wire Line
	3600 4550 4900 4550
Wire Wire Line
	4900 4550 4900 4250
Wire Wire Line
	4900 4250 5400 4250
Wire Wire Line
	7300 2850 7450 2850
Wire Wire Line
	7300 3000 7450 3000
Wire Wire Line
	7300 3150 7450 3150
Wire Wire Line
	7300 3300 7450 3300
Wire Wire Line
	7300 3450 7450 3450
Wire Wire Line
	7300 3600 7450 3600
Wire Wire Line
	7300 3750 7450 3750
Wire Wire Line
	7300 3900 7450 3900
Wire Wire Line
	7300 4100 7450 4100
Wire Wire Line
	7300 4250 7450 4250
Text Label 7450 4250 0    50   ~ 0
Spare_butt
Text Label 7450 4100 0    50   ~ 0
RTD
Text Label 7450 3900 0    50   ~ 0
RAD
Text Label 9100 4300 2    50   ~ 0
FAN
Text Label 7450 3600 0    50   ~ 0
PMP
Text Label 7450 3450 0    50   ~ 0
HV_LED
Text Label 7450 3300 0    50   ~ 0
BUZ
Text Label 7450 3150 0    50   ~ 0
RTD_LED
Text Label 7450 2850 0    50   ~ 0
IMD_LED
Text Label 7450 3000 0    50   ~ 0
BMS_LED
$Comp
L Dash-rescue:souriau18pin-Connector J2
U 1 1 5C155455
P 9350 4100
F 0 "J2" H 9350 4225 50  0000 C CNN
F 1 "souriau18pin" H 9350 4134 50  0000 C CNN
F 2 "footprint:8STA1418" H 9350 4100 50  0001 C CNN
F 3 "" H 9350 4100 50  0001 C CNN
	1    9350 4100
	1    0    0    -1  
$EndComp
Text Label 9600 4600 0    50   ~ 0
RAD
Text Label 7450 3750 0    50   ~ 0
FAN
Text Label 9100 4400 2    50   ~ 0
PMP
Text Label 9100 4700 2    50   ~ 0
RTD
Text Label 9600 5000 0    50   ~ 0
BUZ
Text Label 9600 4500 0    50   ~ 0
BMS_LED
Text Label 9600 4300 0    50   ~ 0
IMD_LED
Text Label 9600 4400 0    50   ~ 0
HV_LED
Text Label 9600 4800 0    50   ~ 0
CAN_H
Text Label 9600 4900 0    50   ~ 0
CAN_L
Text Label 9600 4200 0    50   ~ 0
RTD_LED
Text Label 9100 5000 2    50   ~ 0
Spare_butt
$EndSCHEMATC
