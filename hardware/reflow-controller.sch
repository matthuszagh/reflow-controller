EESchema Schematic File Version 4
LIBS:reflow-controller-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L MCU_Atmel_ATMEGA:ATMEGA8U2-AU U3
U 1 1 5B196B05
P 7850 3900
F 0 "U3" H 7850 5478 50  0000 C CNN
F 1 "ATMEGA8U2-AU" H 7850 5387 50  0000 C CNN
F 2 "TQFP-32" H 7700 3950 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc7799.pdf" H 7950 2450 50  0001 C CNN
F 4 "ATMEGA8U2-AU" H 7850 3900 50  0001 C CNN "manf#"
	1    7850 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Specialized:USB_B_Mini J1
U 1 1 5B196DD2
P 2650 4250
F 0 "J1" H 2705 4717 50  0000 C CNN
F 1 "USB_B_Mini" H 2705 4626 50  0000 C CNN
F 2 "" H 2800 4200 50  0001 C CNN
F 3 "~" H 2800 4200 50  0001 C CNN
F 4 "10033526-N3212LF" H 2650 4250 50  0001 C CNN "manf#"
	1    2650 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4250 3200 4250
$Comp
L Device:R R1
U 1 1 5B197482
P 3350 4250
F 0 "R1" V 3143 4250 50  0000 C CNN
F 1 "22" V 3234 4250 50  0000 C CNN
F 2 "" V 3280 4250 50  0001 C CNN
F 3 "~" H 3350 4250 50  0001 C CNN
F 4 "5%" V 3350 4250 50  0001 C CNN "Tolerance"
	1    3350 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 4250 3650 4250
Text Label 3650 4250 2    50   ~ 0
D+
Wire Wire Line
	6750 4300 6350 4300
Text Label 6350 4300 0    50   ~ 0
D+
$Comp
L Device:R R2
U 1 1 5B1975AE
P 3750 4350
F 0 "R2" V 3543 4350 50  0000 C CNN
F 1 "22" V 3634 4350 50  0000 C CNN
F 2 "" V 3680 4350 50  0001 C CNN
F 3 "~" H 3750 4350 50  0001 C CNN
F 4 "5%" V 3750 4350 50  0001 C CNN "Tolerance"
	1    3750 4350
	0    -1   1    0   
$EndComp
Wire Wire Line
	2950 4350 3600 4350
Wire Wire Line
	3900 4350 4050 4350
Wire Wire Line
	6750 4400 6350 4400
Text Label 6350 4400 0    50   ~ 0
D-
Wire Wire Line
	6750 4500 6500 4500
Wire Wire Line
	6500 4500 6500 4700
$Comp
L power:GND #PWR0101
U 1 1 5B1979AB
P 6500 4700
F 0 "#PWR0101" H 6500 4450 50  0001 C CNN
F 1 "GND" H 6505 4527 50  0000 C CNN
F 2 "" H 6500 4700 50  0001 C CNN
F 3 "" H 6500 4700 50  0001 C CNN
	1    6500 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5300 7850 5500
$Comp
L power:GND #PWR0102
U 1 1 5B197A36
P 7850 5500
F 0 "#PWR0102" H 7850 5250 50  0001 C CNN
F 1 "GND" H 7855 5327 50  0000 C CNN
F 2 "" H 7850 5500 50  0001 C CNN
F 3 "" H 7850 5500 50  0001 C CNN
	1    7850 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4650 2650 4900
$Comp
L power:GND #PWR0103
U 1 1 5B197B2B
P 2650 4900
F 0 "#PWR0103" H 2650 4650 50  0001 C CNN
F 1 "GND" H 2655 4727 50  0000 C CNN
F 2 "" H 2650 4900 50  0001 C CNN
F 3 "" H 2650 4900 50  0001 C CNN
	1    2650 4900
	1    0    0    -1  
$EndComp
Text Label 3950 4350 0    50   ~ 0
D-
NoConn ~ 2950 4450
Wire Wire Line
	6750 4100 6100 4100
Wire Wire Line
	6100 4100 6100 4250
$Comp
L Device:C C6
U 1 1 5B19B1C7
P 6100 4400
F 0 "C6" H 6215 4446 50  0000 L CNN
F 1 "1u" H 6215 4355 50  0000 L CNN
F 2 "" H 6138 4250 50  0001 C CNN
F 3 "~" H 6100 4400 50  0001 C CNN
	1    6100 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5B19B228
P 6100 4700
F 0 "#PWR0104" H 6100 4450 50  0001 C CNN
F 1 "GND" H 6105 4527 50  0000 C CNN
F 2 "" H 6100 4700 50  0001 C CNN
F 3 "" H 6100 4700 50  0001 C CNN
	1    6100 4700
	1    0    0    -1  
$EndComp
Text Label 6100 4100 0    50   ~ 0
VCC
Wire Wire Line
	6750 3300 6100 3300
Text Label 6100 3300 0    50   ~ 0
VCC
Wire Wire Line
	7850 2500 7200 2500
Text Label 7200 2500 0    50   ~ 0
VCC
Wire Wire Line
	6750 4200 6350 4200
Text Label 6350 4200 0    50   ~ 0
VCC
$Comp
L Device:Crystal Y1
U 1 1 5B19B7FB
P 6350 3550
F 0 "Y1" V 6304 3681 50  0000 L CNN
F 1 "8MHz" V 6395 3681 50  0000 L CNN
F 2 "" H 6350 3550 50  0001 C CNN
F 3 "~" H 6350 3550 50  0001 C CNN
F 4 "ABLS-8.000MHZ-B4-T" V 6350 3550 50  0001 C CNN "manf#"
	1    6350 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 3400 6350 3400
Wire Wire Line
	6750 3500 6750 3700
Wire Wire Line
	6750 3700 6350 3700
Wire Wire Line
	6350 3400 5900 3400
Connection ~ 6350 3400
$Comp
L Device:C C4
U 1 1 5B19BD77
P 5750 3400
F 0 "C4" V 5498 3400 50  0000 C CNN
F 1 "26p" V 5589 3400 50  0000 C CNN
F 2 "" H 5788 3250 50  0001 C CNN
F 3 "~" H 5750 3400 50  0001 C CNN
	1    5750 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	6350 3700 6150 3700
Connection ~ 6350 3700
$Comp
L Device:C C5
U 1 1 5B19BFEE
P 6000 3700
F 0 "C5" V 5748 3700 50  0000 C CNN
F 1 "26p" V 5839 3700 50  0000 C CNN
F 2 "" H 6038 3550 50  0001 C CNN
F 3 "~" H 6000 3700 50  0001 C CNN
	1    6000 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 3400 5600 3700
Wire Wire Line
	5850 3700 5600 3700
Connection ~ 5600 3700
Wire Wire Line
	5600 3700 5600 3850
$Comp
L power:GND #PWR0105
U 1 1 5B19CA69
P 5600 3850
F 0 "#PWR0105" H 5600 3600 50  0001 C CNN
F 1 "GND" H 5605 3677 50  0000 C CNN
F 2 "" H 5600 3850 50  0001 C CNN
F 3 "" H 5600 3850 50  0001 C CNN
	1    5600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4550 6100 4700
$Comp
L Sensor_Temperature:MAX31855KASA U1
U 1 1 5B1A75E4
P 4650 2500
F 0 "U1" H 4650 3078 50  0000 C CNN
F 1 "MAX31855KASA" H 4650 2987 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5650 2150 50  0001 C CIN
F 3 "http://datasheets.maximintegrated.com/en/ds/MAX31855.pdf" H 4650 2500 50  0001 C CNN
F 4 "MAX31855KASA+T" H 4650 2500 50  0001 C CNN "manf#"
	1    4650 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2100 5150 2100
Text Label 5000 2100 0    50   ~ 0
VCC
Wire Wire Line
	5050 2300 5400 2300
Text Label 5250 2300 0    50   ~ 0
SCK
Wire Wire Line
	8950 2900 9450 2900
Text Label 9300 2900 0    50   ~ 0
SCK
Wire Wire Line
	5050 2400 5400 2400
Text Label 5200 2400 0    50   ~ 0
MISO
Wire Wire Line
	8950 3100 9450 3100
Text Label 9250 3100 0    50   ~ 0
MISO
Wire Wire Line
	5050 2600 5400 2600
Text Label 5300 2600 0    50   ~ 0
SS
Wire Wire Line
	8950 2800 9450 2800
Text Label 9350 2800 0    50   ~ 0
SS
Wire Wire Line
	4650 2900 4650 3200
$Comp
L power:GND #PWR0106
U 1 1 5B1A977A
P 4650 3200
F 0 "#PWR0106" H 4650 2950 50  0001 C CNN
F 1 "GND" H 4655 3027 50  0000 C CNN
F 2 "" H 4650 3200 50  0001 C CNN
F 3 "" H 4650 3200 50  0001 C CNN
	1    4650 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5B1A9863
P 5300 2100
F 0 "C3" V 5048 2100 50  0000 C CNN
F 1 "0.1u" V 5139 2100 50  0000 C CNN
F 2 "" H 5338 1950 50  0001 C CNN
F 3 "~" H 5300 2100 50  0001 C CNN
	1    5300 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 2100 5750 2100
Wire Wire Line
	5750 2100 5750 2300
$Comp
L power:GND #PWR0107
U 1 1 5B1A9EA9
P 5750 2300
F 0 "#PWR0107" H 5750 2050 50  0001 C CNN
F 1 "GND" H 5755 2127 50  0000 C CNN
F 2 "" H 5750 2300 50  0001 C CNN
F 3 "" H 5750 2300 50  0001 C CNN
	1    5750 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5B1AA059
P 3300 2550
F 0 "J2" H 3220 2225 50  0000 C CNN
F 1 "Conn_01x02" H 3220 2316 50  0000 C CNN
F 2 "" H 3300 2550 50  0001 C CNN
F 3 "~" H 3300 2550 50  0001 C CNN
	1    3300 2550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4250 2400 3900 2400
Wire Wire Line
	3900 2400 3900 2450
Wire Wire Line
	3900 2450 3500 2450
Wire Wire Line
	4250 2600 3900 2600
Wire Wire Line
	3900 2600 3900 2550
Wire Wire Line
	3900 2550 3500 2550
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J3
U 1 1 5B1AB2E1
P 7650 1650
F 0 "J3" H 7700 1967 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 7700 1876 50  0000 C CNN
F 2 "" H 7650 1650 50  0001 C CNN
F 3 "~" H 7650 1650 50  0001 C CNN
	1    7650 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 1550 7100 1550
Text Label 7100 1550 0    50   ~ 0
MISO
Wire Wire Line
	7450 1650 7100 1650
Text Label 7100 1650 0    50   ~ 0
SCK
Wire Wire Line
	7450 1750 7100 1750
Text Label 7100 1750 0    50   ~ 0
RST
Wire Wire Line
	7950 1550 8300 1550
Text Label 8150 1550 0    50   ~ 0
VCC
Wire Wire Line
	7950 1650 8300 1650
Text Label 8100 1650 0    50   ~ 0
MOSI
Wire Wire Line
	7950 1750 8300 1750
Text Label 8150 1750 0    50   ~ 0
GND
Wire Wire Line
	8950 3000 9450 3000
Text Label 9250 3000 0    50   ~ 0
MOSI
Wire Wire Line
	6750 3200 6550 3200
Wire Wire Line
	6100 3200 6100 2900
Text Label 6100 3050 0    50   ~ 0
RST
$Comp
L Device:R R3
U 1 1 5B1C9481
P 6100 2750
F 0 "R3" H 6170 2796 50  0000 L CNN
F 1 "4.7k" H 6170 2705 50  0000 L CNN
F 2 "" V 6030 2750 50  0001 C CNN
F 3 "~" H 6100 2750 50  0001 C CNN
	1    6100 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2600 6100 2300
Text Label 6100 2400 0    50   ~ 0
VCC
$Comp
L Device:D D1
U 1 1 5B1CE015
P 6550 2750
F 0 "D1" V 6504 2829 50  0000 L CNN
F 1 "D" V 6595 2829 50  0000 L CNN
F 2 "" H 6550 2750 50  0001 C CNN
F 3 "~" H 6550 2750 50  0001 C CNN
	1    6550 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 3200 6550 2900
Connection ~ 6550 3200
Wire Wire Line
	6550 3200 6100 3200
Wire Wire Line
	6550 2600 6550 2300
Text Label 6550 2400 0    50   ~ 0
VCC
$Comp
L Regulator_Linear:AP2127K-3.3 U2
U 1 1 5B1D00AB
P 4800 5400
F 0 "U2" H 4800 5742 50  0000 C CNN
F 1 "AP2127K-3.3" H 4800 5651 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4800 5725 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/AP2127.pdf" H 4800 5500 50  0001 C CNN
F 4 "AP2127K-3.3TRG1" H 4800 5400 50  0001 C CNN "manf#"
	1    4800 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4050 3100 4050
Wire Wire Line
	3100 4050 3100 3250
Text Label 3650 3250 0    50   ~ 0
VBUS
Wire Wire Line
	4500 5300 4150 5300
Text Label 4150 5300 0    50   ~ 0
VBUS
Wire Wire Line
	3100 3250 3550 3250
Wire Wire Line
	3550 3250 3550 3350
Connection ~ 3550 3250
Wire Wire Line
	3550 3250 3850 3250
$Comp
L Device:C C1
U 1 1 5B1D63DC
P 3550 3500
F 0 "C1" H 3665 3546 50  0000 L CNN
F 1 "10u" H 3665 3455 50  0000 L CNN
F 2 "" H 3588 3350 50  0001 C CNN
F 3 "~" H 3550 3500 50  0001 C CNN
	1    3550 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5B1DB937
P 3550 3750
F 0 "#PWR0108" H 3550 3500 50  0001 C CNN
F 1 "GND" H 3555 3577 50  0000 C CNN
F 2 "" H 3550 3750 50  0001 C CNN
F 3 "" H 3550 3750 50  0001 C CNN
	1    3550 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3650 3550 3750
Wire Wire Line
	4800 5700 4800 5900
$Comp
L power:GND #PWR0109
U 1 1 5B1E58E9
P 4800 5900
F 0 "#PWR0109" H 4800 5650 50  0001 C CNN
F 1 "GND" H 4805 5727 50  0000 C CNN
F 2 "" H 4800 5900 50  0001 C CNN
F 3 "" H 4800 5900 50  0001 C CNN
	1    4800 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 5300 5250 5300
Text Label 5350 5300 0    50   ~ 0
VCC
Wire Wire Line
	5250 5300 5250 5450
Connection ~ 5250 5300
Wire Wire Line
	5250 5300 5500 5300
$Comp
L Device:C C2
U 1 1 5B1E81B4
P 5250 5600
F 0 "C2" H 5365 5646 50  0000 L CNN
F 1 "1u" H 5365 5555 50  0000 L CNN
F 2 "" H 5288 5450 50  0001 C CNN
F 3 "~" H 5250 5600 50  0001 C CNN
	1    5250 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 5750 5250 5900
$Comp
L power:GND #PWR0110
U 1 1 5B1EAA50
P 5250 5900
F 0 "#PWR0110" H 5250 5650 50  0001 C CNN
F 1 "GND" H 5255 5727 50  0000 C CNN
F 2 "" H 5250 5900 50  0001 C CNN
F 3 "" H 5250 5900 50  0001 C CNN
	1    5250 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 5400 4150 5400
Text Label 4150 5400 0    50   ~ 0
VBUS
Wire Wire Line
	8950 4900 9550 4900
Text Label 9300 4900 0    50   ~ 0
RELAY
$Comp
L Transistor_BJT:BC817 Q1
U 1 1 5B1EDCE0
P 9300 5750
F 0 "Q1" H 9491 5796 50  0000 L CNN
F 1 "BC817" H 9491 5705 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9500 5675 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC817.pdf" H 9300 5750 50  0001 L CNN
F 4 "BC817-40Q-13-F" H 9300 5750 50  0001 C CNN "manf#"
	1    9300 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 5750 8600 5750
Text Label 8600 5750 0    50   ~ 0
RELAY
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5B1EF3C5
P 10100 5450
F 0 "J4" H 10180 5442 50  0000 L CNN
F 1 "Conn_01x02" H 10180 5351 50  0000 L CNN
F 2 "" H 10100 5450 50  0001 C CNN
F 3 "~" H 10100 5450 50  0001 C CNN
	1    10100 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5950 9400 6100
$Comp
L power:GND #PWR0111
U 1 1 5B1F9C1E
P 9400 6100
F 0 "#PWR0111" H 9400 5850 50  0001 C CNN
F 1 "GND" H 9405 5927 50  0000 C CNN
F 2 "" H 9400 6100 50  0001 C CNN
F 3 "" H 9400 6100 50  0001 C CNN
	1    9400 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5550 9900 5550
Wire Wire Line
	9900 5450 9400 5450
Text Label 9400 5450 0    50   ~ 0
VCC
Wire Wire Line
	8950 3900 9450 3900
Text Label 9350 3900 0    50   ~ 0
RX
Wire Wire Line
	8950 4000 9450 4000
Text Label 9350 4000 0    50   ~ 0
TX
$Comp
L Connector_Generic:Conn_01x04 J5
U 1 1 5B201493
P 10350 3900
F 0 "J5" H 10430 3892 50  0000 L CNN
F 1 "Conn_01x04" H 10430 3801 50  0000 L CNN
F 2 "" H 10350 3900 50  0001 C CNN
F 3 "~" H 10350 3900 50  0001 C CNN
	1    10350 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 3800 9900 3800
Text Label 9900 3800 0    50   ~ 0
RX
Wire Wire Line
	10150 3900 9900 3900
Text Label 9900 3900 0    50   ~ 0
TX
Wire Wire Line
	10150 4000 9900 4000
Text Label 9900 4000 0    50   ~ 0
VBUS
Wire Wire Line
	10150 4100 9900 4100
Wire Wire Line
	9900 4100 9900 4350
$Comp
L power:GND #PWR0112
U 1 1 5B207F1D
P 9900 4350
F 0 "#PWR0112" H 9900 4100 50  0001 C CNN
F 1 "GND" H 9905 4177 50  0000 C CNN
F 2 "" H 9900 4350 50  0001 C CNN
F 3 "" H 9900 4350 50  0001 C CNN
	1    9900 4350
	1    0    0    -1  
$EndComp
NoConn ~ 8950 3200
NoConn ~ 8950 3300
NoConn ~ 8950 3400
NoConn ~ 8950 3500
NoConn ~ 8950 3700
NoConn ~ 8950 3800
NoConn ~ 8950 4100
NoConn ~ 8950 4200
NoConn ~ 8950 4300
NoConn ~ 8950 4400
NoConn ~ 8950 4600
NoConn ~ 8950 4700
NoConn ~ 8950 4800
NoConn ~ 8950 5000
NoConn ~ 2550 4650
$EndSCHEMATC
