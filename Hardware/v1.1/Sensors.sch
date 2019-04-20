EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:switches
LIBS:MPU-9250
LIBS:local
LIBS:gps
LIBS:GPS_Diamond-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title "GPS Diamond"
Date "2017-06-07"
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L SAM-M8Q U3
U 1 1 58E775B2
P 2750 6000
F 0 "U3" H 2450 6550 60  0000 C CNN
F 1 "SAM-M8Q" H 2750 5450 60  0000 C CNN
F 2 "local:SAM-M8Q" H 2750 6000 60  0001 C CNN
F 3 "" H 2750 6000 60  0000 C CNN
	1    2750 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 58E7780F
P 2100 6850
F 0 "#PWR019" H 2100 6600 50  0001 C CNN
F 1 "GND" H 2100 6700 50  0000 C CNN
F 2 "" H 2100 6850 50  0001 C CNN
F 3 "" H 2100 6850 50  0001 C CNN
	1    2100 6850
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR020
U 1 1 58E778F5
P 3300 5300
F 0 "#PWR020" H 3300 5150 50  0001 C CNN
F 1 "+3.3V" H 3300 5440 50  0000 C CNN
F 2 "" H 3300 5300 50  0001 C CNN
F 3 "" H 3300 5300 50  0001 C CNN
	1    3300 5300
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR021
U 1 1 58E77949
P 1950 5350
F 0 "#PWR021" H 1950 5200 50  0001 C CNN
F 1 "+BATT" H 1950 5490 50  0000 C CNN
F 2 "" H 1950 5350 50  0001 C CNN
F 3 "" H 1950 5350 50  0001 C CNN
	1    1950 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 6450 2250 6450
Wire Wire Line
	2100 5550 2100 6850
Wire Wire Line
	2250 5550 2100 5550
Connection ~ 2100 6450
Wire Wire Line
	2250 5850 2100 5850
Connection ~ 2100 5850
Wire Wire Line
	2250 5950 2100 5950
Connection ~ 2100 5950
Wire Wire Line
	2250 6050 2100 6050
Connection ~ 2100 6050
Connection ~ 2100 6650
Wire Wire Line
	3400 6450 3250 6450
Wire Wire Line
	3400 5550 3400 6650
Wire Wire Line
	3400 6650 2100 6650
Wire Wire Line
	3250 6050 3400 6050
Connection ~ 3400 6450
Wire Wire Line
	3250 5950 3400 5950
Connection ~ 3400 6050
Wire Wire Line
	3250 5550 3400 5550
Connection ~ 3400 5950
Wire Wire Line
	3300 5850 3250 5850
Wire Wire Line
	3300 5300 3300 5850
Wire Wire Line
	2250 5650 2200 5650
Wire Wire Line
	2200 5650 2200 5350
Wire Wire Line
	2200 5350 3300 5350
Connection ~ 3300 5350
Wire Wire Line
	2250 5750 1950 5750
Wire Wire Line
	1950 5750 1950 5350
NoConn ~ 3250 5650
NoConn ~ 3250 6350
NoConn ~ 2250 6250
NoConn ~ 2250 6150
NoConn ~ 2250 6350
Text HLabel 3550 6150 2    60   Input ~ 0
GPS_RX
Wire Wire Line
	3250 6150 3550 6150
Text HLabel 3550 6250 2    60   Output ~ 0
GPS_TX
Wire Wire Line
	3250 6250 3550 6250
Text HLabel 1200 3700 0    60   Input ~ 0
IMU_SDA
Text HLabel 3550 3700 2    60   Output ~ 0
IMU_INT1
$Comp
L GND #PWR022
U 1 1 58E7D88D
P 2150 4600
F 0 "#PWR022" H 2150 4350 50  0001 C CNN
F 1 "GND" H 2150 4450 50  0000 C CNN
F 2 "" H 2150 4600 50  0001 C CNN
F 3 "" H 2150 4600 50  0001 C CNN
	1    2150 4600
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 58E7DA51
P 1600 4150
F 0 "C12" H 1625 4250 50  0000 L CNN
F 1 "100nF" H 1625 4050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1638 4000 50  0001 C CNN
F 3 "" H 1600 4150 50  0001 C CNN
	1    1600 4150
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 58E7DD9D
P 7550 3550
F 0 "C11" H 7575 3650 50  0000 L CNN
F 1 "100nF" H 7575 3450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402_NoSilk" H 7588 3400 50  0001 C CNN
F 3 "" H 7550 3550 50  0001 C CNN
	1    7550 3550
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 58E7DE3F
P 1300 4150
F 0 "C10" H 1325 4250 50  0000 L CNN
F 1 "100nF" H 1325 4050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 1338 4000 50  0001 C CNN
F 3 "" H 1300 4150 50  0001 C CNN
	1    1300 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 58E83A8A
P 7400 3850
F 0 "#PWR023" H 7400 3600 50  0001 C CNN
F 1 "GND" H 7400 3700 50  0000 C CNN
F 2 "" H 7400 3850 50  0001 C CNN
F 3 "" H 7400 3850 50  0001 C CNN
	1    7400 3850
	1    0    0    -1  
$EndComp
Text HLabel 6300 3500 0    60   Input ~ 0
IMU_SDA
Text HLabel 6300 3600 0    60   Input ~ 0
IMU_SCL
Wire Wire Line
	6300 3500 6500 3500
Wire Wire Line
	6300 3600 6500 3600
NoConn ~ 3250 5750
Text HLabel 1200 3500 0    60   Input ~ 0
IMU_SCL
Text Notes 7300 6900 0    79   ~ 0
Copyright 2017 Brian Webb\nReleased under GNU General Public License V3.0
$Comp
L MS5637 U5
U 1 1 590444DD
P 6900 3550
F 0 "U5" H 6850 3750 50  0000 L CNN
F 1 "MS5637" H 7050 3350 50  0000 R CNN
F 2 "local:MS5637" H 6900 3250 50  0001 C CIN
F 3 "" H 6900 3550 50  0000 C CNN
	1    6900 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3500 7300 3500
Wire Wire Line
	7300 3600 7400 3600
Wire Wire Line
	7400 3600 7400 3850
$Comp
L +3.3V #PWR024
U 1 1 5904C6D9
P 7400 3250
F 0 "#PWR024" H 7400 3100 50  0001 C CNN
F 1 "+3.3V" H 7400 3390 50  0000 C CNN
F 2 "" H 7400 3250 50  0001 C CNN
F 3 "" H 7400 3250 50  0001 C CNN
	1    7400 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3250 7400 3500
$Comp
L +3.3V #PWR025
U 1 1 5904C9E8
P 3400 2950
F 0 "#PWR025" H 3400 2800 50  0001 C CNN
F 1 "+3.3V" H 3400 3090 50  0000 C CNN
F 2 "" H 3400 2950 50  0001 C CNN
F 3 "" H 3400 2950 50  0001 C CNN
	1    3400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3400 7550 3350
Wire Wire Line
	7550 3350 7400 3350
Connection ~ 7400 3350
Wire Wire Line
	7550 3700 7550 3750
Wire Wire Line
	7550 3750 7400 3750
Connection ~ 7400 3750
Wire Wire Line
	2000 3300 1600 3300
Wire Wire Line
	1600 3300 1600 4000
Wire Wire Line
	2000 3200 1300 3200
Wire Wire Line
	1300 3200 1300 4000
Wire Wire Line
	2000 3400 1900 3400
Wire Wire Line
	2000 3500 1200 3500
Wire Wire Line
	2000 3700 1200 3700
Wire Wire Line
	2000 3600 1900 3600
Connection ~ 1900 3600
$Comp
L C C1
U 1 1 5904D72E
P 2150 4300
F 0 "C1" H 2175 4400 50  0000 L CNN
F 1 "100nF" H 2175 4200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 2188 4150 50  0001 C CNN
F 3 "" H 2150 4300 50  0001 C CNN
	1    2150 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3900 1950 3900
Wire Wire Line
	1950 3900 1950 4100
Wire Wire Line
	1950 4100 2150 4100
Wire Wire Line
	2150 4100 2150 4150
Wire Wire Line
	2150 4450 2150 4600
Wire Wire Line
	1300 4500 3700 4500
Wire Wire Line
	1300 4500 1300 4300
Connection ~ 2150 4500
Wire Wire Line
	1600 4300 1600 4500
Connection ~ 1600 4500
Wire Wire Line
	1900 3400 1900 4500
Connection ~ 1900 4500
Wire Wire Line
	2000 3800 1900 3800
Connection ~ 1900 3800
Text Notes 3200 4250 2    60   ~ 0
I2C Address: 0x1E
$Comp
L C C2
U 1 1 5904DA3C
P 3400 4150
F 0 "C2" H 3425 4250 50  0000 L CNN
F 1 "4.7uF" H 3425 4050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3438 4000 50  0001 C CNN
F 3 "" H 3400 4150 50  0001 C CNN
	1    3400 4150
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 5904DAAE
P 3700 4150
F 0 "C13" H 3725 4250 50  0000 L CNN
F 1 "100nF" H 3725 4050 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 3738 4000 50  0001 C CNN
F 3 "" H 3700 4150 50  0001 C CNN
	1    3700 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3400 3400 3400
Wire Wire Line
	3400 2950 3400 4000
Wire Wire Line
	3400 3950 3700 3950
Wire Wire Line
	3700 3950 3700 4000
Connection ~ 3400 3950
Wire Wire Line
	3400 4500 3400 4300
Wire Wire Line
	3700 4500 3700 4300
Connection ~ 3400 4500
Wire Wire Line
	3200 3600 3300 3600
Wire Wire Line
	3300 3200 3300 4500
Connection ~ 3300 4500
Wire Wire Line
	3200 3500 3300 3500
Connection ~ 3300 3600
Wire Wire Line
	3200 3200 3300 3200
Connection ~ 3300 3500
Wire Wire Line
	3200 3700 3550 3700
Text HLabel 3550 3900 2    60   Output ~ 0
IMU_INT2
Wire Wire Line
	3200 3900 3550 3900
Wire Wire Line
	3200 3800 3300 3800
Connection ~ 3300 3800
Connection ~ 3400 3400
Wire Wire Line
	3400 2950 1900 2950
Wire Wire Line
	1900 2950 1900 3200
Connection ~ 1900 3200
$Comp
L FXOS8700CQ U4
U 1 1 5904FEF7
P 2600 3550
F 0 "U4" H 2600 4050 60  0000 C CNN
F 1 "FXOS8700CQ" H 2600 3050 60  0000 C CNN
F 2 "local:FXOS8700CQ" H 2600 2950 60  0001 C CNN
F 3 "" H 2600 3550 60  0001 C CNN
	1    2600 3550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
