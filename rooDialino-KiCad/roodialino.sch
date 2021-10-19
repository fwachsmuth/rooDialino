EESchema Schematic File Version 4
EELAYER 30 0
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
L MCU_Microchip_ATmega:ATmega328P-AU U1
U 1 1 5E110A1E
P 1400 2300
F 0 "U1" H 1400 2350 50  0000 C CNN
F 1 "ATmega328P" H 1700 850 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 1400 2300 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 1400 2300 50  0001 C CNN
F 4 "C14877" H 1400 2300 50  0001 C CNN "LCSC"
	1    1400 2300
	1    0    0    -1  
$EndComp
NoConn ~ 800  1300
NoConn ~ 800  1400
Wire Wire Line
	700  1100 800  1100
Wire Wire Line
	2700 1700 2700 1800
Wire Wire Line
	2000 1800 2150 1800
Wire Wire Line
	2150 1800 2150 1900
$Comp
L Device:C_Small C4
U 1 1 5E11A53A
P 2600 1700
F 0 "C4" V 2450 1700 50  0000 C CNN
F 1 "22p" V 2550 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2600 1700 50  0001 C CNN
F 3 "~" H 2600 1700 50  0001 C CNN
F 4 "C1555" V 2600 1700 50  0001 C CNN "LCSC"
	1    2600 1700
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5E11B014
P 2600 1900
F 0 "C5" V 2750 1900 50  0000 C CNN
F 1 "22p" V 2650 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2600 1900 50  0001 C CNN
F 3 "~" H 2600 1900 50  0001 C CNN
F 4 "C1555" V 2600 1900 50  0001 C CNN "LCSC"
	1    2600 1900
	0    -1   1    0   
$EndComp
$Comp
L power:GNDD #PWR010
U 1 1 5E123E9B
P 2850 1850
F 0 "#PWR010" H 2850 1600 50  0001 C CNN
F 1 "GNDD" H 2900 1700 50  0000 C CNN
F 2 "" H 2850 1850 50  0001 C CNN
F 3 "" H 2850 1850 50  0001 C CNN
	1    2850 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1800 2850 1800
Connection ~ 2700 1800
Wire Wire Line
	2700 1800 2700 1900
$Comp
L power:GNDD #PWR01
U 1 1 5E124BB9
P 700 1400
F 0 "#PWR01" H 700 1150 50  0001 C CNN
F 1 "GNDD" H 700 1250 50  0000 C CNN
F 2 "" H 700 1400 50  0001 C CNN
F 3 "" H 700 1400 50  0001 C CNN
	1    700  1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 800  1500 800 
$Comp
L power:+5V #PWR04
U 1 1 5E132CDE
P 1300 800
F 0 "#PWR04" H 1300 650 50  0001 C CNN
F 1 "+5V" H 1150 850 50  0000 C CNN
F 2 "" H 1300 800 50  0001 C CNN
F 3 "" H 1300 800 50  0001 C CNN
	1    1300 800 
	1    0    0    -1  
$EndComp
Connection ~ 1400 800 
Wire Wire Line
	1400 800  1300 800 
$Comp
L power:GNDD #PWR05
U 1 1 5E133B8F
P 1400 3900
F 0 "#PWR05" H 1400 3650 50  0001 C CNN
F 1 "GNDD" H 1400 3750 50  0000 C CNN
F 2 "" H 1400 3900 50  0001 C CNN
F 3 "" H 1400 3900 50  0001 C CNN
	1    1400 3900
	1    0    0    -1  
$EndComp
Text Label 2150 2600 0    50   ~ 0
~RST~
Wire Wire Line
	2000 1100 2150 1100
Wire Wire Line
	2000 3000 2150 3000
Wire Wire Line
	2000 3100 2150 3100
Wire Wire Line
	2000 2600 2350 2600
$Comp
L power:+5V #PWR09
U 1 1 5E13905F
P 2800 2600
F 0 "#PWR09" H 2800 2450 50  0001 C CNN
F 1 "+5V" H 2815 2773 50  0000 C CNN
F 2 "" H 2800 2600 50  0001 C CNN
F 3 "" H 2800 2600 50  0001 C CNN
	1    2800 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5E13D4FB
P 2500 2600
F 0 "R1" V 2450 2750 50  0000 C CNN
F 1 "10k" V 2500 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 2430 2600 50  0001 C CNN
F 3 "~" H 2500 2600 50  0001 C CNN
F 4 "C25744" V 2500 2600 50  0001 C CNN "LCSC"
	1    2500 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	2650 2600 2800 2600
$Comp
L Device:C_Small C1
U 1 1 5E11B384
P 700 1200
F 0 "C1" H 700 1100 50  0000 R CNN
F 1 "100n" H 800 1400 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 700 1200 50  0001 C CNN
F 3 "~" H 700 1200 50  0001 C CNN
F 4 "C14663" H 700 1200 50  0001 C CNN "LCSC"
	1    700  1200
	1    0    0    -1  
$EndComp
Text Label 2150 1400 0    50   ~ 0
MOSI5
Text Label 2150 1500 0    50   ~ 0
MISO5
Text Label 2150 1600 0    50   ~ 0
SCK5
Wire Wire Line
	2000 1400 2150 1400
Wire Wire Line
	2000 1500 2150 1500
Wire Wire Line
	2000 1600 2150 1600
$Comp
L Connector:AVR-ISP-6 J2
U 1 1 5E20BA79
P 3350 3500
F 0 "J2" H 3020 3596 50  0000 R CNN
F 1 "ICSP" H 3650 3150 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" V 3100 3550 50  0001 C CNN
F 3 " ~" H 2075 2950 50  0001 C CNN
	1    3350 3500
	-1   0    0    -1  
$EndComp
Text Label 2900 3300 2    50   ~ 0
MISO5
Text Label 2900 3400 2    50   ~ 0
MOSI5
Text Label 2900 3500 2    50   ~ 0
SCK5
Text Label 2900 3600 2    50   ~ 0
~RST~
$Comp
L power:+5V #PWR012
U 1 1 5E2151A9
P 3450 2950
F 0 "#PWR012" H 3450 2800 50  0001 C CNN
F 1 "+5V" H 3300 3000 50  0000 C CNN
F 2 "" H 3450 2950 50  0001 C CNN
F 3 "" H 3450 2950 50  0001 C CNN
	1    3450 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2950 3450 3000
Wire Wire Line
	2000 3400 2150 3400
Wire Wire Line
	2000 2000 2150 2000
Wire Wire Line
	2000 2100 2150 2100
Wire Wire Line
	2000 2200 2150 2200
Wire Wire Line
	2000 3300 2150 3300
Text Label 2150 2800 0    50   ~ 0
RX5
Text Label 2150 2900 0    50   ~ 0
TX5
Wire Wire Line
	2900 3300 2950 3300
Wire Wire Line
	2900 3400 2950 3400
Wire Wire Line
	2900 3500 2950 3500
Wire Wire Line
	2900 3600 2950 3600
$Comp
L Connector_Generic:Conn_01x06 J3
U 1 1 5E178CC1
P 3500 1050
F 0 "J3" H 3580 1042 50  0000 L CNN
F 1 "FTDI" H 3500 650 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3500 1050 50  0001 C CNN
F 3 "~" H 3500 1050 50  0001 C CNN
	1    3500 1050
	1    0    0    -1  
$EndComp
Text Label 3250 850  2    50   ~ 0
GNDD
Text Label 3250 950  2    50   ~ 0
GNDD
Text Label 3250 1050 2    50   ~ 0
5V
Text Label 3250 1150 2    50   ~ 0
RX5
Text Label 3250 1250 2    50   ~ 0
TX5
$Comp
L Device:C_Small C6
U 1 1 5E1C6352
P 3150 1350
F 0 "C6" V 3200 1500 50  0000 R CNN
F 1 "100n" V 3300 1550 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3150 1350 50  0001 C CNN
F 3 "~" H 3150 1350 50  0001 C CNN
F 4 "C14663" V 3150 1350 50  0001 C CNN "LCSC"
	1    3150 1350
	0    1    1    0   
$EndComp
Text Label 3000 1350 2    50   ~ 0
~RST~
Wire Wire Line
	3000 1350 3050 1350
Wire Wire Line
	3250 1350 3300 1350
Wire Wire Line
	3250 1250 3300 1250
Wire Wire Line
	3250 1150 3300 1150
Wire Wire Line
	3250 1050 3300 1050
Wire Wire Line
	3250 950  3300 950 
Wire Wire Line
	3250 850  3300 850 
Wire Wire Line
	2000 3200 2150 3200
Wire Wire Line
	1400 3800 1400 3900
Wire Wire Line
	2000 1200 2150 1200
Text Label 2150 1300 0    50   ~ 0
LEDRSTATE
Text Label 2150 3000 0    50   ~ 0
VOL+5
$Comp
L Device:R R12
U 1 1 6169CA12
P 5800 6700
F 0 "R12" V 5700 6700 50  0000 C CNN
F 1 "10k" V 5900 6700 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5730 6700 50  0001 C CNN
F 3 "~" H 5800 6700 50  0001 C CNN
F 4 "C25744" V 5800 6700 50  0001 C CNN "LCSC"
	1    5800 6700
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C11
U 1 1 6169CEA6
P 6050 6900
F 0 "C11" H 6150 7000 50  0000 C CNN
F 1 "47n" H 5950 7000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6050 6900 50  0001 C CNN
F 3 "~" H 6050 6900 50  0001 C CNN
F 4 "C1622" H 6050 6900 50  0001 C CNN "LCSC"
	1    6050 6900
	-1   0    0    1   
$EndComp
$Comp
L power:GNDD #PWR023
U 1 1 6169EC89
P 6050 7150
F 0 "#PWR023" H 6050 6900 50  0001 C CNN
F 1 "GNDD" H 6050 7000 50  0000 C CNN
F 2 "" H 6050 7150 50  0001 C CNN
F 3 "" H 6050 7150 50  0001 C CNN
	1    6050 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 6700 6050 6700
Wire Wire Line
	6050 6700 6050 6800
$Comp
L Switch:SW_Push SW1
U 1 1 616A88A9
P 5550 6900
F 0 "SW1" H 5550 7185 50  0000 C CNN
F 1 "SW_Push" H 5550 7094 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 5550 7100 50  0001 C CNN
F 3 "~" H 5550 7100 50  0001 C CNN
F 4 "C318884" H 5550 6900 50  0001 C CNN "LCSC"
	1    5550 6900
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDD #PWR020
U 1 1 616AA0AF
P 5550 7150
F 0 "#PWR020" H 5550 6900 50  0001 C CNN
F 1 "GNDD" H 5550 7000 50  0000 C CNN
F 2 "" H 5550 7150 50  0001 C CNN
F 3 "" H 5550 7150 50  0001 C CNN
	1    5550 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 6700 5650 6700
Wire Wire Line
	6050 7000 6050 7150
Wire Wire Line
	5550 7150 5550 7100
$Comp
L Device:LED D1
U 1 1 616C64CA
P 3050 5300
F 0 "D1" V 3100 5200 50  0000 R CNN
F 1 "LED" V 3000 5200 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3050 5300 50  0001 C CNN
F 3 "~" H 3050 5300 50  0001 C CNN
F 4 "C72041" V 3050 5300 50  0001 C CNN "LCSC"
	1    3050 5300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 616D2936
P 3400 5300
F 0 "D2" V 3450 5200 50  0000 R CNN
F 1 "LED" V 3350 5200 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3400 5300 50  0001 C CNN
F 3 "~" H 3400 5300 50  0001 C CNN
F 4 "C72041" V 3400 5300 50  0001 C CNN "LCSC"
	1    3400 5300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D3
U 1 1 616D3A9E
P 3750 5300
F 0 "D3" V 3800 5200 50  0000 R CNN
F 1 "LED" V 3700 5200 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3750 5300 50  0001 C CNN
F 3 "~" H 3750 5300 50  0001 C CNN
F 4 "C72041" V 3750 5300 50  0001 C CNN "LCSC"
	1    3750 5300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D4
U 1 1 616D45D8
P 4100 5300
F 0 "D4" V 4150 5200 50  0000 R CNN
F 1 "LED" V 4050 5200 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 4100 5300 50  0001 C CNN
F 3 "~" H 4100 5300 50  0001 C CNN
F 4 "C84256" V 4100 5300 50  0001 C CNN "LCSC"
	1    4100 5300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D5
U 1 1 616D5262
P 4450 5300
F 0 "D5" V 4500 5200 50  0000 R CNN
F 1 "LED" V 4400 5200 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 4450 5300 50  0001 C CNN
F 3 "~" H 4450 5300 50  0001 C CNN
F 4 "C2297" V 4450 5300 50  0001 C CNN "LCSC"
	1    4450 5300
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDD #PWR011
U 1 1 616D66B9
P 3050 5600
F 0 "#PWR011" H 3050 5350 50  0001 C CNN
F 1 "GNDD" H 3050 5450 50  0000 C CNN
F 2 "" H 3050 5600 50  0001 C CNN
F 3 "" H 3050 5600 50  0001 C CNN
	1    3050 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 5450 3050 5550
Wire Wire Line
	3400 5450 3400 5550
Wire Wire Line
	3400 5550 3050 5550
Connection ~ 3050 5550
Wire Wire Line
	3050 5550 3050 5600
Wire Wire Line
	3750 5450 3750 5550
Wire Wire Line
	3750 5550 3400 5550
Connection ~ 3400 5550
Wire Wire Line
	4100 5450 4100 5550
Wire Wire Line
	4100 5550 3750 5550
Connection ~ 3750 5550
Wire Wire Line
	4450 5450 4450 5550
Wire Wire Line
	4450 5550 4100 5550
Connection ~ 4100 5550
$Comp
L Device:R R2
U 1 1 616EF7E8
P 3050 4900
F 0 "R2" H 2900 4850 50  0000 C CNN
F 1 "470" H 2900 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2980 4900 50  0001 C CNN
F 3 "~" H 3050 4900 50  0001 C CNN
F 4 "C17710" H 3050 4900 50  0001 C CNN "LCSC"
	1    3050 4900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 616F0D98
P 3400 4900
F 0 "R3" H 3250 4850 50  0000 C CNN
F 1 "470" H 3250 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3330 4900 50  0001 C CNN
F 3 "~" H 3400 4900 50  0001 C CNN
F 4 "C17710" H 3400 4900 50  0001 C CNN "LCSC"
	1    3400 4900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 616F1158
P 3750 4900
F 0 "R4" H 3600 4850 50  0000 C CNN
F 1 "470" H 3600 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3680 4900 50  0001 C CNN
F 3 "~" H 3750 4900 50  0001 C CNN
F 4 "C17710" H 3750 4900 50  0001 C CNN "LCSC"
	1    3750 4900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 616F14AA
P 4100 4900
F 0 "R5" H 3950 4850 50  0000 C CNN
F 1 "470" H 3950 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4030 4900 50  0001 C CNN
F 3 "~" H 4100 4900 50  0001 C CNN
F 4 "C17710" H 4100 4900 50  0001 C CNN "LCSC"
	1    4100 4900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 616F1CC8
P 4450 4900
F 0 "R6" H 4300 4850 50  0000 C CNN
F 1 "470" H 4300 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4380 4900 50  0001 C CNN
F 3 "~" H 4450 4900 50  0001 C CNN
F 4 "C17710" H 4450 4900 50  0001 C CNN "LCSC"
	1    4450 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 5050 3050 5150
Wire Wire Line
	3400 5050 3400 5150
Wire Wire Line
	3750 5050 3750 5150
Wire Wire Line
	4100 5050 4100 5150
Wire Wire Line
	4450 5050 4450 5150
$Comp
L Transistor_FET:2N7002 Q1
U 1 1 617835F2
P 5300 2100
F 0 "Q1" H 5504 2146 50  0000 L CNN
F 1 "2N7002" H 5504 2055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5500 2025 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 5300 2100 50  0001 L CNN
F 4 "C8545" H 5300 2100 50  0001 C CNN "LCSC"
	1    5300 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 6178B6AC
P 4750 2100
F 0 "R9" V 4650 2100 50  0000 C CNN
F 1 "10" V 4550 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4680 2100 50  0001 C CNN
F 3 "~" H 4750 2100 50  0001 C CNN
	1    4750 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 6178C156
P 4300 2350
F 0 "R8" H 4150 2400 50  0000 C CNN
F 1 "22k" H 4150 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4230 2350 50  0001 C CNN
F 3 "~" H 4300 2350 50  0001 C CNN
	1    4300 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2100 4300 2200
Wire Wire Line
	4300 2500 4300 2550
$Comp
L power:GNDD #PWR017
U 1 1 6179F800
P 4300 2650
F 0 "#PWR017" H 4300 2400 50  0001 C CNN
F 1 "GNDD" H 4300 2500 50  0000 C CNN
F 2 "" H 4300 2650 50  0001 C CNN
F 3 "" H 4300 2650 50  0001 C CNN
	1    4300 2650
	1    0    0    -1  
$EndComp
Connection ~ 4300 2550
Text Label 3950 2100 0    50   ~ 0
IRTX
Connection ~ 4300 2100
$Comp
L Device:LED D7
U 1 1 617A7846
P 4850 1700
F 0 "D7" H 4900 1800 50  0000 R CNN
F 1 "LED" H 4900 1900 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm_IRBlack" H 4850 1700 50  0001 C CNN
F 3 "~" H 4850 1700 50  0001 C CNN
	1    4850 1700
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D8
U 1 1 617D1BAB
P 5200 1700
F 0 "D8" H 5250 1800 50  0000 R CNN
F 1 "LED" H 5250 1900 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm_Horizontal_O1.27mm_Z3.0mm_IRBlack" H 5200 1700 50  0001 C CNN
F 3 "~" H 5200 1700 50  0001 C CNN
	1    5200 1700
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D6
U 1 1 617DFD32
P 4500 1700
F 0 "D6" H 4550 1800 50  0000 R CNN
F 1 "LED" H 4550 1900 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm_Horizontal_O1.27mm_Z3.0mm_IRBlack" H 4500 1700 50  0001 C CNN
F 3 "~" H 4500 1700 50  0001 C CNN
	1    4500 1700
	-1   0    0    1   
$EndComp
$Comp
L Device:R R7
U 1 1 617E0549
P 4300 1300
F 0 "R7" H 4150 1350 50  0000 C CNN
F 1 "5R1" H 4150 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4230 1300 50  0001 C CNN
F 3 "~" H 4300 1300 50  0001 C CNN
F 4 "C17724" H 4300 1300 50  0001 C CNN "LCSC"
	1    4300 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR016
U 1 1 6183BAF7
P 4300 850
F 0 "#PWR016" H 4300 700 50  0001 C CNN
F 1 "+5V" H 4150 900 50  0000 C CNN
F 2 "" H 4300 850 50  0001 C CNN
F 3 "" H 4300 850 50  0001 C CNN
	1    4300 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 4750 3050 4750
Wire Wire Line
	2900 4650 3400 4650
Wire Wire Line
	3400 4650 3400 4750
Wire Wire Line
	2900 4550 3750 4550
Wire Wire Line
	3750 4550 3750 4750
Wire Wire Line
	2900 4450 4100 4450
Wire Wire Line
	4100 4450 4100 4750
Wire Wire Line
	2900 4350 4450 4350
Wire Wire Line
	4450 4350 4450 4750
$Comp
L Device:CP1_Small C8
U 1 1 6187E589
P 4700 1050
F 0 "C8" H 4791 1096 50  0000 L CNN
F 1 "100u" H 4750 950 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.8" H 4700 1050 50  0001 C CNN
F 3 "https://lcsc.com/product-detail/Aluminum-Electrolytic-Capacitors---SMD_PANASONIC-EEEFT1C470AR_C178589.html" H 4700 1050 50  0001 C CNN
F 4 "C178589" H 4700 1050 50  0001 C CNN "LCSC"
	1    4700 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR018
U 1 1 6188544C
P 4700 1250
F 0 "#PWR018" H 4700 1000 50  0001 C CNN
F 1 "GNDD" H 4700 1100 50  0000 C CNN
F 2 "" H 4700 1250 50  0001 C CNN
F 3 "" H 4700 1250 50  0001 C CNN
	1    4700 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 900  4700 950 
Wire Wire Line
	4700 900  5050 900 
Wire Wire Line
	5050 900  5050 950 
Connection ~ 4700 900 
Wire Wire Line
	3950 2100 4300 2100
Wire Wire Line
	4700 1150 4700 1200
Wire Wire Line
	5050 1150 5050 1200
Wire Wire Line
	5050 1200 4700 1200
Connection ~ 4700 1200
Wire Wire Line
	4700 1200 4700 1250
$Comp
L Device:R R11
U 1 1 61918749
P 5400 1300
F 0 "R11" H 5500 1350 50  0000 C CNN
F 1 "33" H 5500 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5330 1300 50  0001 C CNN
F 3 "~" H 5400 1300 50  0001 C CNN
F 4 "C17634" H 5400 1300 50  0001 C CNN "LCSC"
	1    5400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 900  5400 900 
Wire Wire Line
	5400 900  5400 1150
Connection ~ 5050 900 
$Comp
L Interface_Optical:TSOP312xx U2
U 1 1 61922DF7
P 3600 6950
F 0 "U2" H 3588 7375 50  0000 C CNN
F 1 "TSOP312xx" H 3588 7284 50  0000 C CNN
F 2 "OptoDevice:Vishay_MINICAST-3Pin" H 3550 6575 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82492/tsop312.pdf" H 4250 7250 50  0001 C CNN
	1    3600 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR015
U 1 1 619248C3
P 4000 7250
F 0 "#PWR015" H 4000 7000 50  0001 C CNN
F 1 "GNDD" H 4000 7100 50  0000 C CNN
F 2 "" H 4000 7250 50  0001 C CNN
F 3 "" H 4000 7250 50  0001 C CNN
	1    4000 7250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR014
U 1 1 6192500D
P 4000 6600
F 0 "#PWR014" H 4000 6450 50  0001 C CNN
F 1 "+5V" H 3850 6650 50  0000 C CNN
F 2 "" H 4000 6600 50  0001 C CNN
F 3 "" H 4000 6600 50  0001 C CNN
	1    4000 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 6600 4000 6750
Wire Wire Line
	4000 7150 4000 7250
Text Label 4150 6600 0    50   ~ 0
IRRCV
Wire Wire Line
	4300 7050 4300 7150
Wire Wire Line
	4300 7150 4000 7150
Connection ~ 4000 7150
Wire Wire Line
	4300 6850 4300 6750
Connection ~ 4000 6750
Wire Wire Line
	4000 6950 4150 6950
Wire Wire Line
	4000 6750 4300 6750
Wire Wire Line
	4150 6600 4150 6950
Connection ~ 4150 6950
Wire Wire Line
	4150 6950 4300 6950
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 61933FD4
P 4500 6950
F 0 "J4" H 4500 7150 50  0000 C CNN
F 1 "IR Receiver" H 4650 6700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4500 6950 50  0001 C CNN
F 3 "~" H 4500 6950 50  0001 C CNN
	1    4500 6950
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_02x20_Odd_Even J1
U 1 1 61952DD9
P 1450 5750
F 0 "J1" H 1500 6867 50  0000 C CNN
F 1 "Raspi Connector" H 1500 4600 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical" H 1450 5750 50  0001 C CNN
F 3 "~" H 1450 5750 50  0001 C CNN
	1    1450 5750
	1    0    0    -1  
$EndComp
Text Label 1100 5350 2    50   ~ 0
VOL+3
Text Label 1100 5450 2    50   ~ 0
VOL-3
$Comp
L power:+5V #PWR06
U 1 1 619D1271
P 1800 4750
F 0 "#PWR06" H 1800 4600 50  0001 C CNN
F 1 "+5V" H 1650 4800 50  0000 C CNN
F 2 "" H 1800 4750 50  0001 C CNN
F 3 "" H 1800 4750 50  0001 C CNN
	1    1800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4950 1800 4950
Wire Wire Line
	1800 4950 1800 4850
Wire Wire Line
	1750 4850 1800 4850
Connection ~ 1800 4850
$Comp
L power:GNDD #PWR08
U 1 1 619DDC30
P 2350 5150
F 0 "#PWR08" H 2350 4900 50  0001 C CNN
F 1 "GNDD" H 2350 5000 50  0000 C CNN
F 2 "" H 2350 5150 50  0001 C CNN
F 3 "" H 2350 5150 50  0001 C CNN
	1    2350 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C2
U 1 1 619F1DEA
P 2100 4950
F 0 "C2" H 2050 5150 50  0000 L CNN
F 1 "47u" H 2150 4850 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.8" H 2100 4950 50  0001 C CNN
F 3 "https://lcsc.com/product-detail/Aluminum-Electrolytic-Capacitors---SMD_PANASONIC-EEEFT1C470AR_C178589.html" H 2100 4950 50  0001 C CNN
F 4 "C178589" H 2100 4950 50  0001 C CNN "LCSC"
	1    2100 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 619F4890
P 2350 4950
F 0 "C3" H 2400 5150 50  0000 R CNN
F 1 "100n" H 2600 4850 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2350 4950 50  0001 C CNN
F 3 "~" H 2350 4950 50  0001 C CNN
F 4 "C14663" H 2350 4950 50  0001 C CNN "LCSC"
	1    2350 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4850 2100 4850
Connection ~ 2100 4850
Wire Wire Line
	2350 5050 2350 5100
Wire Wire Line
	2350 5100 2100 5100
Wire Wire Line
	2100 5100 2100 5050
$Comp
L Device:C_Small C9
U 1 1 61A1841A
P 5050 1050
F 0 "C9" H 5250 1100 50  0000 R CNN
F 1 "100n" H 5300 950 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5050 1050 50  0001 C CNN
F 3 "~" H 5050 1050 50  0001 C CNN
F 4 "C14663" H 5050 1050 50  0001 C CNN "LCSC"
	1    5050 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 61A1AF2A
P 5600 1600
F 0 "J5" H 5680 1592 50  0000 L CNN
F 1 "IR LED" H 5680 1501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5600 1600 50  0001 C CNN
F 3 "~" H 5600 1600 50  0001 C CNN
	1    5600 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2550 5400 2300
Wire Wire Line
	4300 2100 4600 2100
Wire Wire Line
	4900 2100 5100 2100
Wire Wire Line
	4300 2550 5400 2550
Wire Wire Line
	4300 2550 4300 2650
Wire Wire Line
	4650 1700 4700 1700
Wire Wire Line
	5000 1700 5050 1700
Wire Wire Line
	5350 1700 5400 1700
Wire Wire Line
	5400 1700 5400 1900
Connection ~ 5400 1700
Wire Wire Line
	5400 1600 5400 1450
Wire Wire Line
	4300 1450 4300 1700
Wire Wire Line
	4300 1700 4350 1700
Wire Wire Line
	4300 1150 4300 900 
Wire Wire Line
	4300 900  4400 900 
Connection ~ 4300 900 
Wire Wire Line
	4300 900  4300 850 
Wire Wire Line
	1800 4850 1900 4850
Wire Wire Line
	1750 5050 2100 5050
Connection ~ 2100 5050
Wire Wire Line
	1750 5450 1800 5450
Wire Wire Line
	1800 5450 1800 5750
Wire Wire Line
	1800 5750 1750 5750
Wire Wire Line
	1750 6250 1800 6250
Wire Wire Line
	1800 6250 1800 5750
Connection ~ 1800 5750
$Comp
L power:GNDD #PWR07
U 1 1 61AEC0BC
P 1950 5750
F 0 "#PWR07" H 1950 5500 50  0001 C CNN
F 1 "GNDD" H 1950 5600 50  0000 C CNN
F 2 "" H 1950 5750 50  0001 C CNN
F 3 "" H 1950 5750 50  0001 C CNN
	1    1950 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 6050 1200 6050
Text Label 1900 5350 0    50   ~ 0
MISO3
Text Label 1900 5550 0    50   ~ 0
MOSI3
Text Label 1900 5650 0    50   ~ 0
SCK3
Text Label 1900 6350 0    50   ~ 0
RST3
Text Label 2150 3500 0    50   ~ 0
IRRCV
Text Label 2150 1100 0    50   ~ 0
IRTX
Wire Wire Line
	2000 3500 2150 3500
Text Label 2150 3100 0    50   ~ 0
VOL-5
Text Label 2150 3200 0    50   ~ 0
BTTN
Wire Wire Line
	2000 1300 2150 1300
Text Label 2150 3300 0    50   ~ 0
LEDVOL-
Text Label 2150 3400 0    50   ~ 0
LEDVOL+
Text Label 2150 2100 0    50   ~ 0
LEDRON
Text Label 2150 2200 0    50   ~ 0
LEDROFF
Text Label 2900 4750 2    50   ~ 0
LEDRON
Text Label 2900 4650 2    50   ~ 0
LEDROFF
Text Label 2900 4550 2    50   ~ 0
LEDRSTATE
Text Label 2900 4450 2    50   ~ 0
LEDVOL-
Text Label 2900 4350 2    50   ~ 0
LEDVOL+
Text Label 2150 1200 0    50   ~ 0
D9
Text Label 2150 2000 0    50   ~ 0
A0
Text Label 1900 5150 0    50   ~ 0
TX3
Text Label 1900 5250 0    50   ~ 0
RX3
$Comp
L power:GNDD #PWR02
U 1 1 619DED5D
P 1050 5850
F 0 "#PWR02" H 1050 5600 50  0001 C CNN
F 1 "GNDD" H 1050 5700 50  0000 C CNN
F 2 "" H 1050 5850 50  0001 C CNN
F 3 "" H 1050 5850 50  0001 C CNN
	1    1050 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5250 1200 5250
Wire Wire Line
	1100 5450 1250 5450
Wire Wire Line
	1100 5350 1250 5350
Wire Wire Line
	1750 6350 1900 6350
Wire Wire Line
	1900 5650 1750 5650
Wire Wire Line
	1900 5350 1750 5350
Wire Wire Line
	1900 5550 1750 5550
Wire Wire Line
	1750 5150 1900 5150
Wire Wire Line
	1750 5250 1900 5250
NoConn ~ 1250 4950
NoConn ~ 1250 5050
NoConn ~ 1250 5150
NoConn ~ 1250 5550
NoConn ~ 1250 6150
NoConn ~ 1250 6450
NoConn ~ 1250 6550
NoConn ~ 1250 6650
NoConn ~ 1250 5750
NoConn ~ 1250 5850
NoConn ~ 1250 5950
NoConn ~ 1750 5850
NoConn ~ 1750 5950
NoConn ~ 1750 6050
NoConn ~ 1750 6150
NoConn ~ 1750 6550
NoConn ~ 1750 6650
NoConn ~ 1750 6750
Wire Wire Line
	1800 5750 1950 5750
Text Label 6150 6700 0    50   ~ 0
BTTN
Wire Wire Line
	6150 6700 6050 6700
Connection ~ 6050 6700
Text Label 1100 6250 2    50   ~ 0
GPIO5
Text Label 1100 6350 2    50   ~ 0
GPIO6
Wire Wire Line
	1100 6250 1250 6250
Wire Wire Line
	1100 6350 1250 6350
Wire Wire Line
	2000 2900 2150 2900
Wire Wire Line
	2000 2800 2150 2800
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 61DED3B8
P 5400 3400
F 0 "Q2" V 5649 3400 50  0000 C CNN
F 1 "2N7002" V 5740 3400 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5600 3325 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 5400 3400 50  0001 L CNN
F 4 "C8545" V 5400 3400 50  0001 C CNN "LCSC"
	1    5400 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 61DED3C2
P 5100 3300
F 0 "R13" H 4950 3350 50  0000 C CNN
F 1 "10k" H 4950 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5030 3300 50  0001 C CNN
F 3 "~" H 5100 3300 50  0001 C CNN
F 4 "C25744" H 5100 3300 50  0001 C CNN "LCSC"
	1    5100 3300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R15
U 1 1 61DED3CC
P 5650 3300
F 0 "R15" H 5500 3350 50  0000 C CNN
F 1 "10k" H 5500 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5580 3300 50  0001 C CNN
F 3 "~" H 5650 3300 50  0001 C CNN
F 4 "C25744" H 5650 3300 50  0001 C CNN "LCSC"
	1    5650 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	5100 3150 5100 3100
Wire Wire Line
	5100 3100 5400 3100
Wire Wire Line
	5400 3100 5400 3200
Wire Wire Line
	5600 3500 5650 3500
Wire Wire Line
	5650 3500 5650 3450
Wire Wire Line
	5200 3500 5100 3500
Wire Wire Line
	5100 3500 5100 3450
$Comp
L power:+5V #PWR032
U 1 1 61DED3DD
P 5650 3050
F 0 "#PWR032" H 5650 2900 50  0001 C CNN
F 1 "+5V" H 5500 3100 50  0000 C CNN
F 2 "" H 5650 3050 50  0001 C CNN
F 3 "" H 5650 3050 50  0001 C CNN
	1    5650 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3050 5650 3150
Wire Wire Line
	5100 3100 5100 3050
Connection ~ 5100 3100
$Comp
L power:+3.3V #PWR027
U 1 1 61DED3EA
P 5100 3050
F 0 "#PWR027" H 5100 2900 50  0001 C CNN
F 1 "+3.3V" H 4900 3100 50  0000 C CNN
F 2 "" H 5100 3050 50  0001 C CNN
F 3 "" H 5100 3050 50  0001 C CNN
	1    5100 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3500 5050 3500
Connection ~ 5100 3500
Wire Wire Line
	5650 3500 5700 3500
Connection ~ 5650 3500
$Comp
L Transistor_FET:2N7002 Q3
U 1 1 61DED3F8
P 6700 3400
F 0 "Q3" V 6949 3400 50  0000 C CNN
F 1 "2N7002" V 7040 3400 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6900 3325 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 6700 3400 50  0001 L CNN
F 4 "C8545" V 6700 3400 50  0001 C CNN "LCSC"
	1    6700 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 61DED402
P 6400 3300
F 0 "R14" H 6250 3350 50  0000 C CNN
F 1 "10k" H 6250 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6330 3300 50  0001 C CNN
F 3 "~" H 6400 3300 50  0001 C CNN
F 4 "C25744" H 6400 3300 50  0001 C CNN "LCSC"
	1    6400 3300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R16
U 1 1 61DED40C
P 6950 3300
F 0 "R16" H 6800 3350 50  0000 C CNN
F 1 "10k" H 6800 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6880 3300 50  0001 C CNN
F 3 "~" H 6950 3300 50  0001 C CNN
F 4 "C25744" H 6950 3300 50  0001 C CNN "LCSC"
	1    6950 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	6400 3150 6400 3100
Wire Wire Line
	6400 3100 6700 3100
Wire Wire Line
	6700 3100 6700 3200
Wire Wire Line
	6900 3500 6950 3500
Wire Wire Line
	6950 3500 6950 3450
Wire Wire Line
	6500 3500 6400 3500
Wire Wire Line
	6400 3500 6400 3450
$Comp
L power:+5V #PWR033
U 1 1 61DED41D
P 6950 3050
F 0 "#PWR033" H 6950 2900 50  0001 C CNN
F 1 "+5V" H 6800 3100 50  0000 C CNN
F 2 "" H 6950 3050 50  0001 C CNN
F 3 "" H 6950 3050 50  0001 C CNN
	1    6950 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 3050 6950 3150
Wire Wire Line
	6400 3100 6400 3050
Connection ~ 6400 3100
$Comp
L power:+3.3V #PWR028
U 1 1 61DED42A
P 6400 3050
F 0 "#PWR028" H 6400 2900 50  0001 C CNN
F 1 "+3.3V" H 6200 3100 50  0000 C CNN
F 2 "" H 6400 3050 50  0001 C CNN
F 3 "" H 6400 3050 50  0001 C CNN
	1    6400 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3500 6350 3500
Connection ~ 6400 3500
Wire Wire Line
	6950 3500 7000 3500
Connection ~ 6950 3500
Text Label 5650 5250 2    50   ~ 0
VOL-3
Text Label 5650 5550 2    50   ~ 0
RX3
Text Label 5650 5650 2    50   ~ 0
TX3
Text Label 5050 3500 2    50   ~ 0
GPIO5
Text Label 6350 3500 2    50   ~ 0
GPIO6
NoConn ~ 2000 2300
Text Label 5700 3500 0    50   ~ 0
D9
Text Label 7000 3500 0    50   ~ 0
A0
Text Label 6550 5550 0    50   ~ 0
RX5
Text Label 6550 5650 0    50   ~ 0
TX5
Text Label 6550 5250 0    50   ~ 0
VOL-5
Text Label 6550 5450 0    50   ~ 0
VOL+5
$Comp
L power:+3.3V #PWR03
U 1 1 61ECC860
P 1200 4750
F 0 "#PWR03" H 1200 4600 50  0001 C CNN
F 1 "+3.3V" H 1000 4800 50  0000 C CNN
F 2 "" H 1200 4750 50  0001 C CNN
F 3 "" H 1200 4750 50  0001 C CNN
	1    1200 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5250 1200 5750
Wire Wire Line
	1250 5650 1150 5650
Wire Wire Line
	1150 5650 1150 4950
Wire Wire Line
	1050 5850 1050 5750
Wire Wire Line
	1050 5750 1200 5750
Connection ~ 1200 5750
Wire Wire Line
	1200 5750 1200 6050
$Comp
L Logic_LevelTranslator:TXS0108EPW U3
U 1 1 61EF8601
P 6100 5250
F 0 "U3" H 6450 4600 50  0000 C CNN
F 1 "TXS0108EPW" H 6450 4500 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 6100 4500 50  0001 C CNN
F 3 "www.ti.com/lit/ds/symlink/txs0108e.pdf" H 6100 5150 50  0001 C CNN
F 4 "C17206" H 6100 5250 50  0001 C CNN "LCSC"
	1    6100 5250
	1    0    0    -1  
$EndComp
Text Label 5650 5450 2    50   ~ 0
VOL+3
Text Label 5650 5350 2    50   ~ 0
MISO3
Text Label 5650 5150 2    50   ~ 0
MOSI3
Text Label 5650 5050 2    50   ~ 0
SCK3
Text Label 5650 4950 2    50   ~ 0
RST3
Text Label 6550 4950 0    50   ~ 0
~RST~
Text Label 6550 5050 0    50   ~ 0
SCK5
Text Label 6550 5150 0    50   ~ 0
MOSI5
Text Label 6550 5350 0    50   ~ 0
MISO5
$Comp
L power:GNDD #PWR024
U 1 1 61F065BE
P 6100 6000
F 0 "#PWR024" H 6100 5750 50  0001 C CNN
F 1 "GNDD" H 6100 5850 50  0000 C CNN
F 2 "" H 6100 6000 50  0001 C CNN
F 3 "" H 6100 6000 50  0001 C CNN
	1    6100 6000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR025
U 1 1 61F07825
P 6200 4150
F 0 "#PWR025" H 6200 4000 50  0001 C CNN
F 1 "+5V" H 6200 4300 50  0000 C CNN
F 2 "" H 6200 4150 50  0001 C CNN
F 3 "" H 6200 4150 50  0001 C CNN
	1    6200 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4850 5650 4850
Wire Wire Line
	5650 4850 5650 4550
Wire Wire Line
	5650 4550 6000 4550
Connection ~ 6000 4550
Wire Wire Line
	5700 4950 5650 4950
Wire Wire Line
	5700 5050 5650 5050
Wire Wire Line
	5700 5150 5650 5150
Wire Wire Line
	5700 5250 5650 5250
Wire Wire Line
	5700 5350 5650 5350
Wire Wire Line
	5700 5450 5650 5450
Wire Wire Line
	5700 5550 5650 5550
Wire Wire Line
	5700 5650 5650 5650
Wire Wire Line
	6500 4950 6550 4950
Wire Wire Line
	6500 5050 6550 5050
Wire Wire Line
	6500 5150 6550 5150
Wire Wire Line
	6500 5250 6550 5250
Wire Wire Line
	6500 5350 6550 5350
Wire Wire Line
	6500 5450 6550 5450
Wire Wire Line
	6500 5550 6550 5550
Wire Wire Line
	6500 5650 6550 5650
$Comp
L Device:R R10
U 1 1 61FCB007
P 5500 4550
F 0 "R10" V 5600 4550 50  0000 C CNN
F 1 "10k" V 5400 4550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5430 4550 50  0001 C CNN
F 3 "~" H 5500 4550 50  0001 C CNN
F 4 "C25744" V 5500 4550 50  0001 C CNN "LCSC"
	1    5500 4550
	0    -1   -1   0   
$EndComp
Connection ~ 5650 4550
$Comp
L power:GNDD #PWR019
U 1 1 61FCD646
P 5300 4600
F 0 "#PWR019" H 5300 4350 50  0001 C CNN
F 1 "GNDD" H 5300 4450 50  0000 C CNN
F 2 "" H 5300 4600 50  0001 C CNN
F 3 "" H 5300 4600 50  0001 C CNN
	1    5300 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4600 5300 4550
Wire Wire Line
	5300 4550 5350 4550
Wire Wire Line
	6100 5950 6100 6000
$Comp
L power:+3.3V #PWR022
U 1 1 61FF5EB6
P 6000 4150
F 0 "#PWR022" H 6000 4000 50  0001 C CNN
F 1 "+3.3V" H 5950 4300 50  0000 C CNN
F 2 "" H 6000 4150 50  0001 C CNN
F 3 "" H 6000 4150 50  0001 C CNN
	1    6000 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 61FF7511
P 6300 4250
F 0 "C12" H 6500 4300 50  0000 R CNN
F 1 "100n" H 6550 4200 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6300 4250 50  0001 C CNN
F 3 "~" H 6300 4250 50  0001 C CNN
F 4 "C14663" H 6300 4250 50  0001 C CNN "LCSC"
	1    6300 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 61FF86BA
P 5900 4250
F 0 "C10" H 5800 4300 50  0000 R CNN
F 1 "100n" H 5850 4200 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5900 4250 50  0001 C CNN
F 3 "~" H 5900 4250 50  0001 C CNN
F 4 "C14663" H 5900 4250 50  0001 C CNN "LCSC"
	1    5900 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR021
U 1 1 61FF8BDE
P 5900 4350
F 0 "#PWR021" H 5900 4100 50  0001 C CNN
F 1 "GNDD" H 5850 4200 50  0000 C CNN
F 2 "" H 5900 4350 50  0001 C CNN
F 3 "" H 5900 4350 50  0001 C CNN
	1    5900 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4150 6000 4550
Wire Wire Line
	6200 4150 6200 4550
Wire Wire Line
	6300 4150 6200 4150
Connection ~ 6200 4150
Wire Wire Line
	5900 4150 6000 4150
Connection ~ 6000 4150
$Comp
L power:GNDD #PWR026
U 1 1 6203ECEC
P 6300 4350
F 0 "#PWR026" H 6300 4100 50  0001 C CNN
F 1 "GNDD" H 6350 4200 50  0000 C CNN
F 2 "" H 6300 4350 50  0001 C CNN
F 3 "" H 6300 4350 50  0001 C CNN
	1    6300 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  1400 700  1300
Text Label 4250 900  2    50   ~ 0
5V
Wire Wire Line
	4300 900  4250 900 
Wire Wire Line
	2350 5100 2350 5150
Connection ~ 2350 5100
$Comp
L power:PWR_FLAG #FLG03
U 1 1 620D1518
P 1900 4850
F 0 "#FLG03" H 1900 4925 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 5023 50  0000 C CNN
F 2 "" H 1900 4850 50  0001 C CNN
F 3 "~" H 1900 4850 50  0001 C CNN
	1    1900 4850
	1    0    0    -1  
$EndComp
Connection ~ 1900 4850
Wire Wire Line
	1900 4850 2100 4850
$Comp
L power:PWR_FLAG #FLG02
U 1 1 620D2193
P 1150 4950
F 0 "#FLG02" H 1150 5025 50  0001 C CNN
F 1 "PWR_FLAG" V 1150 5077 50  0000 L CNN
F 2 "" H 1150 4950 50  0001 C CNN
F 3 "~" H 1150 4950 50  0001 C CNN
	1    1150 4950
	0    -1   -1   0   
$EndComp
Connection ~ 1150 4950
Wire Wire Line
	1150 4950 1150 4850
Wire Wire Line
	1800 4750 1800 4850
Wire Wire Line
	1150 4850 1200 4850
Wire Wire Line
	1200 4750 1200 4850
Connection ~ 1200 4850
Wire Wire Line
	1200 4850 1250 4850
Wire Wire Line
	2850 1850 2850 1800
$Comp
L power:GNDD #PWR013
U 1 1 620F8036
P 3450 3950
F 0 "#PWR013" H 3450 3700 50  0001 C CNN
F 1 "GNDD" H 3450 3800 50  0000 C CNN
F 2 "" H 3450 3950 50  0001 C CNN
F 3 "" H 3450 3950 50  0001 C CNN
	1    3450 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3900 3450 3950
$Comp
L power:PWR_FLAG #FLG01
U 1 1 62118275
P 1050 5750
F 0 "#FLG01" H 1050 5825 50  0001 C CNN
F 1 "PWR_FLAG" V 1050 5877 50  0000 L CNN
F 2 "" H 1050 5750 50  0001 C CNN
F 3 "~" H 1050 5750 50  0001 C CNN
	1    1050 5750
	0    -1   -1   0   
$EndComp
Connection ~ 1050 5750
$Comp
L Device:C_Small C7
U 1 1 621450A9
P 4400 1050
F 0 "C7" H 4600 1100 50  0000 R CNN
F 1 "47u" H 4650 950 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4400 1050 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/lcsc/1810301816_Samsung-Electro-Mechanics-CL31A107MQHNNNE_C15008.pdf" H 4400 1050 50  0001 C CNN
F 4 "C15008" H 4400 1050 50  0001 C CNN "LCSC"
	1    4400 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 950  4400 900 
Connection ~ 4400 900 
Wire Wire Line
	4400 900  4700 900 
Wire Wire Line
	4400 1150 4400 1200
Wire Wire Line
	4400 1200 4700 1200
Wire Wire Line
	1750 6450 1800 6450
Wire Wire Line
	1800 6450 1800 6250
Connection ~ 1800 6250
Wire Wire Line
	1200 6050 1200 6750
Wire Wire Line
	1200 6750 1250 6750
Connection ~ 1200 6050
NoConn ~ 2000 2400
NoConn ~ 2000 2500
NoConn ~ 2600 2700
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 617082D2
P 2350 1800
F 0 "Y1" V 2400 1600 50  0000 L CNN
F 1 "16 MHz" V 2500 1700 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 2350 1800 50  0001 C CNN
F 3 "~" H 2350 1800 50  0001 C CNN
F 4 "C13738" V 2350 1800 50  0001 C CNN "LCSC"
	1    2350 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 1900 2350 1900
Wire Wire Line
	2000 1700 2350 1700
Connection ~ 2350 1700
Wire Wire Line
	2350 1700 2500 1700
Connection ~ 2350 1900
Wire Wire Line
	2350 1900 2500 1900
Wire Wire Line
	2700 1800 2450 1800
Wire Wire Line
	2250 1800 2250 1650
Wire Wire Line
	2250 1650 2450 1650
Wire Wire Line
	2450 1650 2450 1800
Connection ~ 2450 1800
$EndSCHEMATC
