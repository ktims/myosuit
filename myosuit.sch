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
LIBS:special
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
LIBS:w_connectors
LIBS:misc
LIBS:myosuit-cache
EELAYER 24 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title "Myoelectrosuit Limbo"
Date "13 Jan 2014"
Rev "1"
Comp "Keenan Tims"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RJ45 P103
U 1 1 52A7EEFD
P 2050 1650
F 0 "P103" H 2250 2150 60  0000 C CNN
F 1 "BUS_IN" H 1750 2150 60  0000 C CNN
F 2 "" H 2050 1650 60  0000 C CNN
F 3 "" H 2050 1650 60  0000 C CNN
F 4 "54602-908LF" H 2050 1650 60  0001 C CNN "Part#"
	1    2050 1650
	0    -1   1    0   
$EndComp
Text Notes 1600 1000 0    60   ~ 0
Twisted Pairs:\n1-2 - V+\n3-6 - Serial MOSI\n4-5 - Serial SIMO\n7-8 - V-
Text Label 2650 1500 0    39   ~ 0
BUS_RX-
Text Label 2650 1600 0    39   ~ 0
BUS_TX+
Text Label 2650 1700 0    39   ~ 0
BUS_TX-
Text Label 2650 1800 0    39   ~ 0
BUS_RX+
$Comp
L GND #PWR01
U 1 1 52A807A2
P 4250 2300
F 0 "#PWR01" H 4250 2300 30  0001 C CNN
F 1 "GND" H 4250 2230 30  0001 C CNN
F 2 "" H 4250 2300 60  0000 C CNN
F 3 "" H 4250 2300 60  0000 C CNN
	1    4250 2300
	1    0    0    -1  
$EndComp
Text Label 2600 1300 0    39   ~ 0
VIN
$Comp
L SP491E U102
U 1 1 52A807EB
P 3850 2800
F 0 "U102" H 3600 2400 40  0000 C CNN
F 1 "SP491E" H 4100 2400 40  0000 C CNN
F 2 "SO8" H 3850 2800 35  0001 C CIN
F 3 "" H 3850 2800 60  0000 C CNN
	1    3850 2800
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 52A8085E
P 3850 3400
F 0 "#PWR02" H 3850 3400 30  0001 C CNN
F 1 "GND" H 3850 3330 30  0001 C CNN
F 2 "" H 3850 3400 60  0000 C CNN
F 3 "" H 3850 3400 60  0000 C CNN
	1    3850 3400
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 52A80A2E
P 3850 2200
F 0 "#PWR03" H 3850 2290 20  0001 C CNN
F 1 "+5V" H 3850 2290 30  0000 C CNN
F 2 "" H 3850 2200 60  0000 C CNN
F 3 "" H 3850 2200 60  0000 C CNN
	1    3850 2200
	1    0    0    -1  
$EndComp
$Comp
L C C103
U 1 1 52A80A5F
P 4050 2250
F 0 "C103" H 4050 2350 40  0000 L CNN
F 1 "100nF" H 4056 2165 40  0000 L CNN
F 2 "" H 4088 2100 30  0000 C CNN
F 3 "" H 4050 2250 60  0000 C CNN
	1    4050 2250
	0    -1   -1   0   
$EndComp
$Comp
L R R102
U 1 1 52A80B1A
P 3300 2300
F 0 "R102" V 3380 2300 40  0000 C CNN
F 1 "120" V 3307 2301 40  0000 C CNN
F 2 "" V 3230 2300 30  0000 C CNN
F 3 "" H 3300 2300 30  0000 C CNN
	1    3300 2300
	1    0    0    -1  
$EndComp
$Comp
L R R101
U 1 1 52A80B2E
P 3050 2300
F 0 "R101" V 3130 2300 40  0000 C CNN
F 1 "120" V 3057 2301 40  0000 C CNN
F 2 "" V 2980 2300 30  0000 C CNN
F 3 "" H 3050 2300 30  0000 C CNN
	1    3050 2300
	1    0    0    -1  
$EndComp
Text Notes 2150 2450 0    39   ~ 0
Optional Bus Termination R
$Comp
L NCP1117ST50T3G U101
U 1 1 52A80F98
P 2250 3500
F 0 "U101" H 2250 3750 40  0000 C CNN
F 1 "1117" H 2250 3700 40  0000 C CNN
F 2 "" H 2250 3500 60  0000 C CNN
F 3 "" H 2250 3500 60  0000 C CNN
F 4 "NCP1117ST50T3G" H 2250 3500 60  0001 C CNN "Part#"
	1    2250 3500
	1    0    0    -1  
$EndComp
$Comp
L CP1 C101
U 1 1 52A81013
P 1750 3700
F 0 "C101" H 1800 3800 50  0000 L CNN
F 1 "47uF" H 1800 3600 50  0000 L CNN
F 2 "c_1210" H 1750 3700 60  0001 C CNN
F 3 "" H 1750 3700 60  0000 C CNN
F 4 "C1210C476M4PACTU" H 1750 3700 60  0001 C CNN "Part#"
	1    1750 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 52A81027
P 1750 3950
F 0 "#PWR04" H 1750 3950 30  0001 C CNN
F 1 "GND" H 1750 3880 30  0001 C CNN
F 2 "" H 1750 3950 60  0000 C CNN
F 3 "" H 1750 3950 60  0000 C CNN
	1    1750 3950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 52A810BF
P 2250 3800
F 0 "#PWR05" H 2250 3800 30  0001 C CNN
F 1 "GND" H 2250 3730 30  0001 C CNN
F 2 "" H 2250 3800 60  0000 C CNN
F 3 "" H 2250 3800 60  0000 C CNN
	1    2250 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 52A81179
P 2750 3950
F 0 "#PWR06" H 2750 3950 30  0001 C CNN
F 1 "GND" H 2750 3880 30  0001 C CNN
F 2 "" H 2750 3950 60  0000 C CNN
F 3 "" H 2750 3950 60  0000 C CNN
	1    2750 3950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR07
U 1 1 52A811E0
P 2750 3400
F 0 "#PWR07" H 2750 3490 20  0001 C CNN
F 1 "+5V" H 2750 3490 30  0000 C CNN
F 2 "" H 2750 3400 60  0000 C CNN
F 3 "" H 2750 3400 60  0000 C CNN
	1    2750 3400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 52A816FA
P 2850 3150
F 0 "#PWR08" H 2850 3150 30  0001 C CNN
F 1 "GND" H 2850 3080 30  0001 C CNN
F 2 "" H 2850 3150 60  0000 C CNN
F 3 "" H 2850 3150 60  0000 C CNN
	1    2850 3150
	1    0    0    -1  
$EndComp
Text Label 2600 2000 0    39   ~ 0
GNDIN
$Comp
L ATTINY841-SSU U103
U 1 1 52A8256F
P 6700 3600
F 0 "U103" H 5850 4350 40  0000 C CNN
F 1 "ATTINY841-SSU" H 7350 2850 40  0000 C CNN
F 2 "SO14" H 6700 3400 35  0000 C CIN
F 3 "" H 6700 3600 60  0000 C CNN
	1    6700 3600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR09
U 1 1 52A82583
P 5600 2950
F 0 "#PWR09" H 5600 3040 20  0001 C CNN
F 1 "+5V" H 5600 3040 30  0000 C CNN
F 2 "" H 5600 2950 60  0000 C CNN
F 3 "" H 5600 2950 60  0000 C CNN
	1    5600 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 52A825F2
P 5600 4250
F 0 "#PWR010" H 5600 4250 30  0001 C CNN
F 1 "GND" H 5600 4180 30  0001 C CNN
F 2 "" H 5600 4250 60  0000 C CNN
F 3 "" H 5600 4250 60  0000 C CNN
	1    5600 4250
	1    0    0    -1  
$EndComp
$Comp
L C C104
U 1 1 52A82663
P 5600 3600
F 0 "C104" H 5600 3700 40  0000 L CNN
F 1 "100nF" H 5606 3515 40  0000 L CNN
F 2 "" H 5638 3450 30  0000 C CNN
F 3 "" H 5600 3600 60  0000 C CNN
	1    5600 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 52A82B27
P 4400 3100
F 0 "#PWR011" H 4400 3100 30  0001 C CNN
F 1 "GND" H 4400 3030 30  0001 C CNN
F 2 "" H 4400 3100 60  0000 C CNN
F 3 "" H 4400 3100 60  0000 C CNN
	1    4400 3100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR012
U 1 1 52A82C2B
P 7750 1100
F 0 "#PWR012" H 7750 1190 20  0001 C CNN
F 1 "+5V" H 7750 1190 30  0000 C CNN
F 2 "" H 7750 1100 60  0000 C CNN
F 3 "" H 7750 1100 60  0000 C CNN
	1    7750 1100
	1    0    0    -1  
$EndComp
$Comp
L AVR-ISP-6 P102
U 1 1 52A82CC7
P 7400 1250
F 0 "P102" H 7320 1490 50  0000 C CNN
F 1 "ISP" H 7160 1020 50  0000 L BNN
F 2 "AVR-ISP-6" V 6880 1290 50  0001 C CNN
F 3 "" H 7400 1250 60  0000 C CNN
	1    7400 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 52A82E46
P 7750 1400
F 0 "#PWR013" H 7750 1400 30  0001 C CNN
F 1 "GND" H 7750 1330 30  0001 C CNN
F 2 "" H 7750 1400 60  0000 C CNN
F 3 "" H 7750 1400 60  0000 C CNN
	1    7750 1400
	1    0    0    -1  
$EndComp
$Comp
L FDC6305N Q101
U 2 1 52A8330C
P 9000 3100
F 0 "Q101" V 9200 3150 60  0000 R CNN
F 1 "FDC6305N" V 8900 3050 39  0000 R CNN
F 2 "" H 9000 3100 60  0000 C CNN
F 3 "" H 9000 3100 60  0000 C CNN
	2    9000 3100
	0    -1   -1   0   
$EndComp
$Comp
L FDC6305N Q101
U 1 1 52A8331E
P 9600 3100
F 0 "Q101" V 9800 3150 60  0000 R CNN
F 1 "FDC6305N" V 9500 3050 39  0000 R CNN
F 2 "" H 9600 3100 60  0000 C CNN
F 3 "" H 9600 3100 60  0000 C CNN
	1    9600 3100
	0    -1   -1   0   
$EndComp
$Comp
L FDC6305N Q102
U 2 1 52A83330
P 10200 3100
F 0 "Q102" V 10400 3150 60  0000 R CNN
F 1 "FDC6305N" V 10100 3050 39  0000 R CNN
F 2 "" H 10200 3100 60  0000 C CNN
F 3 "" H 10200 3100 60  0000 C CNN
	2    10200 3100
	0    -1   -1   0   
$EndComp
$Comp
L FDC6305N Q102
U 1 1 52A83342
P 10800 3100
F 0 "Q102" V 11000 3150 60  0000 R CNN
F 1 "FDC6305N" V 10700 3050 39  0000 R CNN
F 2 "" H 10800 3100 60  0000 C CNN
F 3 "" H 10800 3100 60  0000 C CNN
	1    10800 3100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR014
U 1 1 52A83739
P 11150 3000
F 0 "#PWR014" H 11150 3000 30  0001 C CNN
F 1 "GND" H 11150 2930 30  0001 C CNN
F 2 "" H 11150 3000 60  0000 C CNN
F 3 "" H 11150 3000 60  0000 C CNN
	1    11150 3000
	1    0    0    -1  
$EndComp
$Comp
L CONN_5 P101
U 1 1 52A839AA
P 9900 1700
F 0 "P101" V 9850 1700 50  0000 C CNN
F 1 "LEDS" V 9950 1700 50  0000 C CNN
F 2 "" H 9900 1700 60  0000 C CNN
F 3 "" H 9900 1700 60  0000 C CNN
	1    9900 1700
	0    -1   -1   0   
$EndComp
Text Label 9550 2150 0    39   ~ 0
VIN
Text Label 9800 2250 1    39   ~ 0
LED0
Text Label 9900 2250 1    39   ~ 0
LED1
Text Label 10000 2250 1    39   ~ 0
LED2
Text Label 10100 2250 1    39   ~ 0
LED3
$Sheet
S 6000 5150 1200 1200
U 52A8F66D
F0 "analog input" 50
F1 "analog input.sch" 50
F2 "IN0+" I L 6000 5350 60 
F3 "IN0-" I L 6000 5250 60 
F4 "IN1+" I L 6000 5500 60 
F5 "IN1-" I L 6000 5600 60 
F6 "IN2+" I L 6000 5850 60 
F7 "IN2-" I L 6000 5750 60 
F8 "IN3+" I L 6000 6000 60 
F9 "IN3-" I L 6000 6100 60 
F10 "3Vref" I R 7200 5700 60 
F11 "OUT0" I R 7200 5550 60 
F12 "OUT1" I R 7200 5450 60 
F13 "OUT2" I R 7200 5250 60 
F14 "OUT3" I R 7200 5350 60 
F15 "BGND" I L 6000 6250 60 
$EndSheet
$Comp
L RJ45 P104
U 1 1 52A9915B
P 4000 5350
F 0 "P104" H 4200 5850 60  0000 C CNN
F 1 "LIMB_U" H 3700 5850 60  0000 C CNN
F 2 "" H 4000 5350 60  0000 C CNN
F 3 "" H 4000 5350 60  0000 C CNN
F 4 "54602-908LF" H 4000 5350 60  0001 C CNN "Part#"
	1    4000 5350
	0    -1   1    0   
$EndComp
$Comp
L RJ45 P105
U 1 1 52A99238
P 4000 6600
F 0 "P105" H 4200 7100 60  0000 C CNN
F 1 "LIMB_L" H 3700 7100 60  0000 C CNN
F 2 "" H 4000 6600 60  0000 C CNN
F 3 "" H 4000 6600 60  0000 C CNN
F 4 "54602-908LF" H 4000 6600 60  0001 C CNN "Part#"
	1    4000 6600
	0    -1   1    0   
$EndComp
Text Label 4550 5300 0    39   ~ 0
VIN
Text Label 4550 5400 0    39   ~ 0
LED0
Text Label 4550 5600 0    39   ~ 0
VIN
Text Label 4550 5700 0    39   ~ 0
LED1
Text Label 4550 6550 0    39   ~ 0
VIN
Text Label 4550 6650 0    39   ~ 0
LED2
Text Label 4550 6850 0    39   ~ 0
VIN
Text Label 4550 6950 0    39   ~ 0
LED3
$Comp
L TRANSFO CM101
U 1 1 52AA08B9
P 2350 2850
F 0 "CM101" H 2350 3100 39  0000 C CNN
F 1 " " H 2350 2600 39  0000 C CNN
F 2 "" H 2350 2850 60  0000 C CNN
F 3 "" H 2350 2850 60  0000 C CNN
F 4 "SRF2012-361" H 2350 2850 60  0001 C CNN "Part#"
	1    2350 2850
	1    0    0    1   
$EndComp
$Comp
L C C102
U 1 1 52AA1CCA
P 2750 3700
F 0 "C102" H 2750 3800 40  0000 L CNN
F 1 "10uF" H 2756 3615 40  0000 L CNN
F 2 "c_1206" H 2788 3550 30  0001 C CNN
F 3 "" H 2750 3700 60  0000 C CNN
	1    2750 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 52CE7798
P 4800 7350
F 0 "#PWR015" H 4800 7350 30  0001 C CNN
F 1 "GND" H 4800 7280 30  0001 C CNN
F 2 "" H 4800 7350 60  0000 C CNN
F 3 "" H 4800 7350 60  0000 C CNN
	1    4800 7350
	1    0    0    -1  
$EndComp
$Comp
L C C105
U 1 1 52CE78C1
P 4800 5850
F 0 "C105" H 4800 5950 40  0000 L CNN
F 1 "10uF" H 4806 5765 40  0000 L CNN
F 2 "" H 4838 5700 30  0000 C CNN
F 3 "" H 4800 5850 60  0000 C CNN
F 4 "CL31A106KAHNNNE" H 4800 5850 60  0001 C CNN "Part#"
	1    4800 5850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 52CE7B9C
P 4800 6100
F 0 "#PWR016" H 4800 6100 30  0001 C CNN
F 1 "GND" H 4800 6030 30  0001 C CNN
F 2 "" H 4800 6100 60  0000 C CNN
F 3 "" H 4800 6100 60  0000 C CNN
	1    4800 6100
	1    0    0    -1  
$EndComp
$Comp
L C C106
U 1 1 52CE76A8
P 4800 7100
F 0 "C106" H 4800 7200 40  0000 L CNN
F 1 "10uF" H 4806 7015 40  0000 L CNN
F 2 "" H 4838 6950 30  0000 C CNN
F 3 "" H 4800 7100 60  0000 C CNN
F 4 "CL31A106KAHNNNE" H 4800 7100 60  0001 C CNN "Part#"
	1    4800 7100
	1    0    0    -1  
$EndComp
NoConn ~ 3650 5900
NoConn ~ 3650 7150
NoConn ~ 1700 2200
$Comp
L DIODESCH D101
U 1 1 52D21F98
P 1600 3250
F 0 "D101" H 1600 3350 40  0000 C CNN
F 1 "MBR0540" H 1600 3150 40  0000 C CNN
F 2 "sod123" H 1600 3250 60  0001 C CNN
F 3 "" H 1600 3250 60  0000 C CNN
	1    1600 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 2200 4250 2300
Wire Wire Line
	3800 3300 3800 3350
Wire Wire Line
	3800 3350 3900 3350
Wire Wire Line
	3850 3350 3850 3400
Wire Wire Line
	3900 3350 3900 3300
Connection ~ 3850 3350
Wire Wire Line
	3300 2550 3450 2550
Wire Wire Line
	3200 2650 3450 2650
Wire Wire Line
	3150 2950 3450 2950
Wire Wire Line
	2950 3050 3450 3050
Wire Wire Line
	2950 1150 2950 3050
Wire Wire Line
	3850 2200 3850 2300
Connection ~ 4250 2250
Connection ~ 3850 2250
Wire Wire Line
	2950 2050 3050 2050
Connection ~ 2950 2050
Wire Wire Line
	3050 2550 3150 2550
Connection ~ 3150 2550
Connection ~ 3400 2550
Wire Wire Line
	1750 3950 1750 3900
Wire Wire Line
	1750 3500 1750 3450
Connection ~ 1750 3450
Wire Wire Line
	2250 3800 2250 3750
Wire Wire Line
	2650 3450 2950 3450
Wire Wire Line
	2750 3400 2750 3500
Wire Wire Line
	2750 3950 2750 3900
Connection ~ 2750 3450
Wire Wire Line
	1600 3450 1850 3450
Wire Wire Line
	2600 1150 2600 1400
Wire Wire Line
	2600 1150 1350 1150
Wire Wire Line
	1350 1150 1350 2650
Wire Wire Line
	1350 2650 1950 2650
Connection ~ 2600 2000
Wire Wire Line
	1950 3050 1600 3050
Wire Wire Line
	5650 3000 5600 3000
Wire Wire Line
	5600 2950 5600 3400
Wire Wire Line
	5600 3800 5600 4250
Wire Wire Line
	5600 4200 5650 4200
Connection ~ 5600 3000
Connection ~ 5600 4200
Wire Wire Line
	7750 3000 8000 3000
Wire Wire Line
	4250 2600 5250 2600
Wire Wire Line
	5250 2600 5250 4600
Wire Wire Line
	5250 4600 8100 4600
Wire Wire Line
	8100 4600 8100 4100
Wire Wire Line
	8100 4100 7750 4100
Wire Wire Line
	7750 3700 8200 3700
Wire Wire Line
	8200 3700 8200 4700
Wire Wire Line
	8200 4700 5100 4700
Wire Wire Line
	5100 4700 5100 3000
Wire Wire Line
	5100 3000 4250 3000
Wire Wire Line
	7750 4000 8150 4000
Wire Wire Line
	8150 4000 8150 4650
Wire Wire Line
	8150 4650 5200 4650
Wire Wire Line
	5200 4650 5200 2850
Wire Wire Line
	5200 2850 4250 2850
Wire Wire Line
	4250 2750 4400 2750
Wire Wire Line
	4400 2750 4400 3100
Wire Wire Line
	7750 1150 7750 1100
Wire Wire Line
	7525 1150 7750 1150
Wire Wire Line
	7525 1350 7750 1350
Wire Wire Line
	7750 1350 7750 1400
Wire Wire Line
	7750 3600 10800 3600
Wire Wire Line
	8200 1250 8200 3600
Wire Wire Line
	7525 1250 8200 1250
Wire Wire Line
	7750 3500 10200 3500
Wire Wire Line
	8150 3500 8150 950 
Wire Wire Line
	8150 950  6950 950 
Wire Wire Line
	6950 950  6950 1150
Wire Wire Line
	6950 1150 7275 1150
Wire Wire Line
	7750 3400 9600 3400
Wire Wire Line
	8100 3400 8100 1500
Wire Wire Line
	8100 1500 6950 1500
Wire Wire Line
	6950 1500 6950 1250
Wire Wire Line
	6950 1250 7275 1250
Wire Wire Line
	7150 1350 7275 1350
Wire Wire Line
	7150 1350 7150 1550
Wire Wire Line
	7150 1550 8050 1550
Wire Wire Line
	8050 1550 8050 4200
Wire Wire Line
	8050 4200 7750 4200
Wire Wire Line
	10800 3600 10800 3300
Connection ~ 8200 3600
Wire Wire Line
	10200 3500 10200 3300
Connection ~ 8150 3500
Wire Wire Line
	9600 3400 9600 3300
Connection ~ 8100 3400
Wire Wire Line
	9000 3300 8050 3300
Connection ~ 8050 3300
Wire Wire Line
	9200 3000 9200 2850
Wire Wire Line
	9200 2850 11150 2850
Wire Wire Line
	11150 2850 11150 3000
Wire Wire Line
	11000 3000 11000 2850
Connection ~ 11000 2850
Wire Wire Line
	10400 3000 10400 2850
Connection ~ 10400 2850
Wire Wire Line
	9800 3000 9800 2850
Connection ~ 9800 2850
Wire Wire Line
	9700 2100 9700 2150
Wire Wire Line
	9700 2150 9550 2150
Wire Wire Line
	8800 3000 8800 2600
Wire Wire Line
	8800 2600 9800 2600
Wire Wire Line
	9800 2600 9800 2100
Wire Wire Line
	9400 3000 9400 2650
Wire Wire Line
	9400 2650 9900 2650
Wire Wire Line
	9900 2650 9900 2100
Wire Wire Line
	10000 3000 10000 2100
Wire Wire Line
	10100 2100 10100 2650
Wire Wire Line
	10100 2650 10600 2650
Wire Wire Line
	10600 2650 10600 3000
Wire Wire Line
	7200 5700 8000 5700
Wire Wire Line
	8000 5700 8000 3000
Wire Wire Line
	7800 5250 7800 3900
Wire Wire Line
	7800 3900 7750 3900
Wire Wire Line
	7850 3300 7750 3300
Wire Wire Line
	7900 3200 7900 5450
Wire Wire Line
	7900 3200 7750 3200
Wire Wire Line
	7950 3100 7950 5550
Wire Wire Line
	7950 3100 7750 3100
Wire Wire Line
	7950 5550 7200 5550
Wire Wire Line
	7800 5250 7200 5250
Wire Wire Line
	7900 5450 7200 5450
Wire Wire Line
	7850 5350 7200 5350
Wire Wire Line
	7850 5350 7850 3300
Wire Wire Line
	2500 2000 2850 2000
Wire Wire Line
	2600 1900 2500 1900
Wire Wire Line
	2600 1400 2500 1400
Wire Wire Line
	2600 1300 2500 1300
Connection ~ 2600 1300
Wire Wire Line
	4450 5000 5700 5000
Wire Wire Line
	5700 5000 5700 5250
Wire Wire Line
	5700 5250 6000 5250
Wire Wire Line
	4450 5100 5650 5100
Wire Wire Line
	5650 5100 5650 5350
Wire Wire Line
	5650 5350 6000 5350
Wire Wire Line
	4450 5200 5600 5200
Wire Wire Line
	5600 5200 5600 5500
Wire Wire Line
	5600 5500 6000 5500
Wire Wire Line
	4450 5500 5550 5500
Wire Wire Line
	5550 5500 5550 5600
Wire Wire Line
	5550 5600 6000 5600
Wire Wire Line
	4450 5300 4800 5300
Wire Wire Line
	4550 5400 4450 5400
Wire Wire Line
	4450 5600 4800 5600
Wire Wire Line
	4550 5700 4450 5700
Wire Wire Line
	4450 6250 5550 6250
Wire Wire Line
	5550 6250 5550 5750
Wire Wire Line
	5550 5750 6000 5750
Wire Wire Line
	6000 5850 5600 5850
Wire Wire Line
	5600 5850 5600 6350
Wire Wire Line
	5600 6350 4450 6350
Wire Wire Line
	4450 6450 5650 6450
Wire Wire Line
	5650 6450 5650 6000
Wire Wire Line
	5650 6000 6000 6000
Wire Wire Line
	6000 6100 5700 6100
Wire Wire Line
	5700 6100 5700 6750
Wire Wire Line
	5700 6750 4450 6750
Wire Wire Line
	4550 6650 4450 6650
Wire Wire Line
	4550 6950 4450 6950
Wire Wire Line
	2850 2000 2850 2650
Wire Wire Line
	2600 1900 2600 2000
Wire Wire Line
	4800 7350 4800 7300
Wire Wire Line
	4800 5300 4800 5650
Connection ~ 4800 5600
Wire Wire Line
	4800 6100 4800 6050
Wire Wire Line
	4800 6550 4800 6900
Wire Wire Line
	4800 6850 4450 6850
Wire Wire Line
	4450 6550 4800 6550
Connection ~ 4800 6850
Wire Wire Line
	2850 3050 2850 3150
Wire Wire Line
	2850 2650 2750 2650
Wire Wire Line
	2750 3050 2850 3050
Wire Wire Line
	3200 1500 3200 2650
Wire Wire Line
	3300 2050 3200 2050
Connection ~ 3200 2050
Wire Wire Line
	2950 1600 2500 1600
Wire Wire Line
	2500 1700 3250 1700
Wire Wire Line
	3150 1700 3150 2950
Wire Wire Line
	3200 1500 2500 1500
Wire Wire Line
	2500 1800 3400 1800
Wire Wire Line
	3400 1800 3400 2550
Wire Wire Line
	6000 6250 5750 6250
Wire Wire Line
	5750 6250 5750 6950
Wire Wire Line
	5750 6950 5950 6950
$Comp
L CONN_1 P106
U 1 1 52E22C60
P 6100 6950
F 0 "P106" H 6180 6950 40  0000 L CNN
F 1 "BODYGND" H 6100 7005 30  0001 C CNN
F 2 "" H 6100 6950 60  0000 C CNN
F 3 "" H 6100 6950 60  0000 C CNN
	1    6100 6950
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P107
U 1 1 52EB61B8
P 2600 1000
F 0 "P107" H 2680 1000 40  0000 L CNN
F 1 "VIN" H 2600 1055 30  0001 C CNN
F 2 "" H 2600 1000 60  0000 C CNN
F 3 "" H 2600 1000 60  0000 C CNN
	1    2600 1000
	0    -1   -1   0   
$EndComp
Connection ~ 2600 1150
$Comp
L CONN_1 P108
U 1 1 52EB6305
P 2600 2150
F 0 "P108" H 2680 2150 40  0000 L CNN
F 1 "GNDIN" H 2600 2205 30  0001 C CNN
F 2 "" H 2600 2150 60  0000 C CNN
F 3 "" H 2600 2150 60  0000 C CNN
	1    2600 2150
	0    1    1    0   
$EndComp
$Comp
L CONN_1 P109
U 1 1 52EB64FD
P 2950 1000
F 0 "P109" H 3030 1000 40  0000 L CNN
F 1 "TX+" H 2950 1055 30  0001 C CNN
F 2 "" H 2950 1000 60  0000 C CNN
F 3 "" H 2950 1000 60  0000 C CNN
	1    2950 1000
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P111
U 1 1 52EB6565
P 3050 1000
F 0 "P111" H 3130 1000 40  0000 L CNN
F 1 "RX+" H 3050 1055 30  0001 C CNN
F 2 "" H 3050 1000 60  0000 C CNN
F 3 "" H 3050 1000 60  0000 C CNN
	1    3050 1000
	0    -1   -1   0   
$EndComp
Connection ~ 2950 1600
Wire Wire Line
	3050 1800 3050 1150
Connection ~ 3050 1800
$Comp
L CONN_1 P110
U 1 1 52EB6CC9
P 2950 3300
F 0 "P110" H 3030 3300 40  0000 L CNN
F 1 "5V" H 2950 3355 30  0001 C CNN
F 2 "" H 2950 3300 60  0000 C CNN
F 3 "" H 2950 3300 60  0000 C CNN
	1    2950 3300
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P112
U 1 1 52EB77F6
P 3150 1000
F 0 "P112" H 3230 1000 40  0000 L CNN
F 1 "RX-" H 3150 1055 30  0001 C CNN
F 2 "" H 3150 1000 60  0000 C CNN
F 3 "" H 3150 1000 60  0000 C CNN
	1    3150 1000
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P113
U 1 1 52EB780A
P 3250 1000
F 0 "P113" H 3330 1000 40  0000 L CNN
F 1 "TX-" H 3250 1055 30  0001 C CNN
F 2 "" H 3250 1000 60  0000 C CNN
F 3 "" H 3250 1000 60  0000 C CNN
	1    3250 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3150 1150 3150 1500
Connection ~ 3150 1500
Wire Wire Line
	3250 1700 3250 1150
Connection ~ 3150 1700
$EndSCHEMATC
