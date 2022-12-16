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

##** Steps to Run this project**   
Steps to configure the project  
1. clone IR-Blaster repository from github  
    "git clone https://github.com/ezloteam/Ezlo_Pi"  
2. If you have already installed the esp-idf then go to step 3. If not install esp-idf
3. Configure the project using "idf.py menuconfig" command you will see  
    ![Image](/Ir-blaster_configuration.png)
4. Goto Example Configuration  
5. Select the Protocol in Infrored Protocol
    ![Image](/IR-protocol.png)
    ![Image](/IR-protocol-selection.png)
6. Configure the RX and TX pin  
    ![Image](/IR-protocol-Rx-Tx-pin.png)
7. Save the configuration
8. Build using "idf.py build" command
9. Flash the firmware using "idf.py -p <COM Port Name> -b <baud> flash"
10. Monitor using "idf.py -p <COM Port Name> monitor"