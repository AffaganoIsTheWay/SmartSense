## SmartSense
***
Table of contents:
- Overview
- Finite State Machine
- Pre-requisits
- Project Layout
- Getting Started
- Illustrations
- Powerpoint Presentation & Video link

***
Overview

SmartSense is a technology aimed at detecting and displaying the Carbon Dioxide level (ppm), humidity and temperature. Developed using NordicThingy nrf52, NordicThingy:52 and BoosterpackXL kits in C language, it can be deployed by any operating system. SmartSense is great for people who like to keep a constant update of their environment's condition. The bluetooth sensor of the NordicThingy:52 allows us to actively update the state of the environment according to time of request. The display and joystick from the BoosterpackXL kit displays them in a clear and precise form, with the joystick allowing to shuffle to the parameter the user wants to see. 
***
Finite State Machines

blob:https://web.whatsapp.com/643e0dc2-505e-4529-aaf9-52e66428890d

Pre-requisits

  Hardware
  
    1. Texas Instruments MSP432P401R microcontroller
    2. NordicThingy:52 IOT sensor kit
    3. NordicThingy nRF52 development kit
    4. BOOSTXL-EDUMKII Educational BoosterPack
    5. 2 MtF jumper wires
    
  Software
  
    1. J-link software from https://www.segger.com/downloads/jlink/
    2. nRF desktop application (SDK version)
    3. Code composer studio
    4. TI driver library
    5. Visual Studio Code
***
Project Layout

    SmartSense
    |
    |── Nordic               #Folder for the Nordic nRF52-DK project
    |     |
    |     |── src
    |     |     |
    |     |     |── main.c   #Main file for Nordic nRF52-DK
    |     |
    |     |──CMakeLists. txt
    |     |──prj.conf        #File that contain which Library the project shold contain
    |     |──sample.yaml
    |
    |──Texas_Instruments
    |     |
    |     |──launches
    |     |──settings
    |     |──LcdDriver      #Driver needed for the LCS screen
    |     |──targetConfigs
    |     |──.ccsproject
    |     |──.cproject
    |     |──.project
    |     |──main.c        #Main file for MSP432P401R
    |     |──msp432p401r.cmd
    |     |──startup_msp432p401r_ccs.c
    |     |──startup_msp432p401r.c
    |
    |── .gitignore
    |── PowerPointPresentation.pptx
    |── README.md

***
Getting Started

1. Clone the repo using

```
git clone https://github.com/AffaganoIsTheWay/SmartSense.git
```

For the Texas Instrument board
1. Import the Texas_Instrument folder in your local CCS Workspace
2. Downloand and unzip TI driver Library (https://drive.google.com/file/d/1_5TsECed3wNJpIpllxYYdD06aFbkk7Fc/view)
3. Open CSS and left click on Project Folder to select Properties
4. Select CSS Build
5. Click ARM Compiler and then Include Opt
6. Add "simplelink_msp432p4_sdk_3_40_01_02/source" directory to "Add dir to #include search path" window
7. Click ARM Linker and File Search Path
8. Add "simplelink_msp432p4_sdk_3_40_01_02/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib" to "Include library file..." window
9. Add "simplelink_msp432p4_sdk_3_40_01_02/source/ti/grlib/lib/ccs/m4/grlib.a" to "Include library file..." window
10. Import the Texas_Instrument folder in your local CCS Workspace
11. Build and flash on the TI board

For the Nordic Board
1. Install the nRF Connect for VS Code Extension Pack (https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-extension-pack)
2. From the nRF connect tab install toolchain and SDK
3. Download and install Segger j-link
4. Open the nordic folder with VS code
5. Build and Flash on the Nordic board

For the Hardware
1. Connect the Educational Boosterpack to the MSP432P401R
2. Ports P0.08 and P0.06 of nRF52 are connected to ports J1.4 and J1.3 of the BoosterpackXL board respectively using the wires

***
Illustrations

![WhatsApp Image 2025-01-25 at 17 58 55](https://github.com/user-attachments/assets/e2e87625-18a6-4f4d-934e-8101017b1eb6)
![WhatsApp Image 2025-01-25 at 17 58 54(3)](https://github.com/user-attachments/assets/35f27e1b-5c13-4a1b-b196-42a63d383394)
![WhatsApp Image 2025-01-25 at 17 58 54(2)](https://github.com/user-attachments/assets/d4d73d66-21f5-448d-89d1-df2ad340629d)
![WhatsApp Image 2025-01-25 at 17 58 54(1)](https://github.com/user-attachments/assets/984bb302-fd9f-4567-9c96-3622faf180af)
![WhatsApp Image 2025-01-25 at 17 58 54](https://github.com/user-attachments/assets/bf45d9d5-d4f8-447c-9cda-a076ba398405)
****
Powerpoint Presentation & Video link

-Powerpoint: 
-Video: 
****
Colaboratores

- Giovanni Vitiello
- Trisha Agrawal
- Christian Li Sivertsen
- Frederico Gasperi
