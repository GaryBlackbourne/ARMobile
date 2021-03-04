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
L Driver_Motor:L293D U?
U 1 1 60413715
P 2250 1750
F 0 "U?" H 1900 2700 50  0000 C CNN
F 1 "L293D" H 2550 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 2500 1000 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/l293.pdf" H 1950 2450 50  0001 C CNN
	1    2250 1750
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 60417EC5
P 3250 1200
F 0 "M?" H 3408 1196 50  0000 L CNN
F 1 "Motor_DC" H 3408 1105 50  0000 L CNN
F 2 "" H 3250 1110 50  0001 C CNN
F 3 "~" H 3250 1110 50  0001 C CNN
	1    3250 1200
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 604183B5
P 3250 3950
F 0 "M?" H 3408 3946 50  0000 L CNN
F 1 "Motor_DC" H 3408 3855 50  0000 L CNN
F 2 "" H 3250 3860 50  0001 C CNN
F 3 "~" H 3250 3860 50  0001 C CNN
	1    3250 3950
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 604185E9
P 3250 1950
F 0 "M?" H 3408 1946 50  0000 L CNN
F 1 "Motor_DC" H 3408 1855 50  0000 L CNN
F 2 "" H 3250 1860 50  0001 C CNN
F 3 "~" H 3250 1860 50  0001 C CNN
	1    3250 1950
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 60418833
P 3250 4700
F 0 "M?" H 3408 4696 50  0000 L CNN
F 1 "Motor_DC" H 3408 4605 50  0000 L CNN
F 2 "" H 3250 4610 50  0001 C CNN
F 3 "~" H 3250 4610 50  0001 C CNN
	1    3250 4700
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:L293D U?
U 1 1 6041B097
P 2250 4500
F 0 "U?" H 1900 5450 50  0000 C CNN
F 1 "L293D" H 2550 5450 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 2500 3750 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/l293.pdf" H 1950 5200 50  0001 C CNN
	1    2250 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR?
U 1 1 60425D0B
P 2650 800
F 0 "#PWR?" H 2650 650 50  0001 C CNN
F 1 "+6V" H 2665 973 50  0000 C CNN
F 2 "" H 2650 800 50  0001 C CNN
F 3 "" H 2650 800 50  0001 C CNN
	1    2650 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR?
U 1 1 60426C22
P 2650 3500
F 0 "#PWR?" H 2650 3350 50  0001 C CNN
F 1 "+6V" H 2665 3673 50  0000 C CNN
F 2 "" H 2650 3500 50  0001 C CNN
F 3 "" H 2650 3500 50  0001 C CNN
	1    2650 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3500 2350 3500
Wire Wire Line
	2350 3500 2150 3500
Connection ~ 2350 3500
$Comp
L power:GND #PWR?
U 1 1 604286A4
P 2650 2550
F 0 "#PWR?" H 2650 2300 50  0001 C CNN
F 1 "GND" H 2655 2377 50  0000 C CNN
F 2 "" H 2650 2550 50  0001 C CNN
F 3 "" H 2650 2550 50  0001 C CNN
	1    2650 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2550 2450 2550
Connection ~ 2150 2550
Wire Wire Line
	2150 2550 2050 2550
Connection ~ 2350 2550
Wire Wire Line
	2350 2550 2150 2550
Connection ~ 2450 2550
Wire Wire Line
	2450 2550 2350 2550
$Comp
L power:GND #PWR?
U 1 1 60428E13
P 2650 5300
F 0 "#PWR?" H 2650 5050 50  0001 C CNN
F 1 "GND" H 2655 5127 50  0000 C CNN
F 2 "" H 2650 5300 50  0001 C CNN
F 3 "" H 2650 5300 50  0001 C CNN
	1    2650 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 5300 2450 5300
Connection ~ 2150 5300
Wire Wire Line
	2150 5300 2050 5300
Connection ~ 2350 5300
Wire Wire Line
	2350 5300 2150 5300
Connection ~ 2450 5300
Wire Wire Line
	2450 5300 2350 5300
Wire Wire Line
	3250 4500 2750 4500
Wire Wire Line
	2750 4700 2950 4700
Wire Wire Line
	2950 4700 2950 5000
Wire Wire Line
	2950 5000 3250 5000
Wire Wire Line
	2750 1950 2950 1950
Wire Wire Line
	2950 1950 2950 2250
Wire Wire Line
	2950 2250 3250 2250
Wire Wire Line
	3250 1750 2750 1750
Wire Wire Line
	2750 1350 3000 1350
Wire Wire Line
	3000 1350 3000 1500
Wire Wire Line
	3000 1500 3250 1500
Wire Wire Line
	3000 1150 3000 1000
Wire Wire Line
	3000 1000 3250 1000
Wire Wire Line
	2750 1150 3000 1150
Wire Wire Line
	2750 3900 3000 3900
Wire Wire Line
	3000 3900 3000 3750
Wire Wire Line
	3000 3750 3250 3750
Wire Wire Line
	3250 4250 3000 4250
Wire Wire Line
	3000 4250 3000 4100
Wire Wire Line
	3000 4100 2750 4100
$Comp
L Device:Battery BT?
U 1 1 60456B75
P 4500 1200
F 0 "BT?" H 4608 1246 50  0000 L CNN
F 1 "Battery" H 4608 1155 50  0000 L CNN
F 2 "" V 4500 1260 50  0001 C CNN
F 3 "~" V 4500 1260 50  0001 C CNN
	1    4500 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60457EAB
P 5000 1200
F 0 "C?" H 5115 1246 50  0000 L CNN
F 1 "330n" H 5115 1155 50  0000 L CNN
F 2 "" H 5038 1050 50  0001 C CNN
F 3 "~" H 5000 1200 50  0001 C CNN
	1    5000 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60458376
P 6000 1200
F 0 "C?" H 6115 1246 50  0000 L CNN
F 1 "100n" H 6115 1155 50  0000 L CNN
F 2 "" H 6038 1050 50  0001 C CNN
F 3 "~" H 6000 1200 50  0001 C CNN
	1    6000 1200
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L78L33_TO92 U?
U 1 1 60458CAC
P 5500 1050
F 0 "U?" H 5500 1292 50  0000 C CNN
F 1 "L78L33_TO92" H 5500 1201 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5500 1275 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/15/55/e5/aa/23/5b/43/fd/CD00000446.pdf/files/CD00000446.pdf/jcr:content/translations/en.CD00000446.pdf" H 5500 1000 50  0001 C CNN
	1    5500 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1050 5200 1050
Wire Wire Line
	5800 1050 6000 1050
Wire Wire Line
	6000 1350 5500 1350
Connection ~ 5500 1350
Wire Wire Line
	5500 1350 5000 1350
Wire Wire Line
	4500 1000 5000 1000
Wire Wire Line
	5000 1000 5000 1050
Connection ~ 5000 1050
Wire Wire Line
	5000 1350 5000 1400
Wire Wire Line
	5000 1400 4500 1400
Connection ~ 5000 1350
$EndSCHEMATC