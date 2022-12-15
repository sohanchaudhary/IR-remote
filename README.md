# IR-Blaster
This driver is being developed for continuous monitoring of the IR signals and transmitting as per the need.

## Supported Protocols
1. NEC  
2. RC5
3. Samsung (for TV and Projector)
4. LGTV 
5. LGAC 
6. Sony (SIRCS) all 12 bits, 15 bits, 20 bits
7. GREE 
8. PANASONIC
9. DISH
10. JVC
11. SHARP (In Progress)
12. LEGO
13. SAMSUNG AC
14. TOSHIBA_AC (50, 72 bit Supported)
15. TOSHIBA_TV
16. EPSON projector IR Remote
17. AIRTON AC
18. AIWA for DVD

## Steps to Run this project

1. Clone the project from the link
2. Configure the project using idf.py menuconfig command
3. Select the IR protocol and configure RX and TX pin from the Example configuration list
4. Build the project using the idf.py build command
5. Flash the firmware to the esp-32 S3 using the following command
    idf.py -p COM -b baudrate flash
6. Monitor the logs using the following command
    idf.py -p COM -b 115200 monitor
