## SmartSense
***
Table of contents:
- Overview
- Finite State Machine Diagram
- Pre-requisites
- Project Layout
- Getting Started
- Illustrations
- Powerpoint Presentation & Video link

***
Overview

SmartSense is a technology aimed at detecting and displaying the Carbon Dioxide concentration (ppm), humidity and temperature. Developed using Nordic nrf52 development kit, Nordic Thingy:52 and TI Educational BoosterPack MKII kits in C language, it can be deployed by any operating system. SmartSense is great for people who like to keep a constant update of their environment's condition. The bluetooth sensor of the Nordic Thingy:52 allows us to actively update the state of the environment according to time of request. The display and joystick from the BoosterPack kit displays them in a clear and intuitive way, with the joystick allowing the user to cycle through the values the user wants to see. 
***
Finite State Machine Diagram


Working of MSP432 Boosterpack Developmental Kit:

![WhatsApp Image 2025-02-04 at 19 39 52](https://github.com/user-attachments/assets/30cd4935-e904-4707-91aa-a64bb792935b)

Working of NordicThingy 52 and Nordic nrf 52:

![Screenshot 2025-02-04 at 19-26-02 Claude](https://github.com/user-attachments/assets/32d1d6ea-f3de-45eb-8db7-9ebb8b076546)

***
Pre-requisites

  Hardware
  
    1. Texas Instruments MSP432P401R LaunchPad Development Kit
    2. Nordic Thingy:52 IOT sensor kit
    3. Nordic nRF52 development kit
    4. Texas Instruments Educational BoosterPack MKII
    5. 2 MtF jumper wires
    6. Micro usb power supply for MSP432
    7. Battery or micro usb power supply for the nrf52dk
    
    
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
    |──Texas_Instruments     #Folder for MSP432p401R and BoosterPack MKII project
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

For the Texas Instruments MSP432 board
1. Import the Texas_Instrument folder in your local CCS Workspace
2. Downloand and unzip TI driver Library (https://drive.google.com/file/d/1_5TsECed3wNJpIpllxYYdD06aFbkk7Fc/view)
3. Open CSS and left click on Project Folder to select Properties
4. Select CSS Build
5. Click ARM Compiler and then Include Opt
6. Add "simplelink_msp432p4_sdk_3_40_01_02/source" directory to "Add dir to #include search path" window
7. Click ARM Linker and File Search Path
8. Add "simplelink_msp432p4_sdk_3_40_01_02/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib" to "Include library file..." window
9. Add "simplelink_msp432p4_sdk_3_40_01_02/source/ti/grlib/lib/ccs/m4/grlib.a" to "Include library file..." window
10. Build and flash on the TI board

For the Nordic nrf52dk Board
1. Install the nRF Connect for VS Code Extension Pack (https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-extension-pack)
2. From the nRF connect tab install toolchain and SDK
3. Download and install Segger j-link
4. Open the nordic folder with VS code
5. Build and Flash on the Nordic board

For the Hardware
1. Connect the Educational BoosterPack to the MSP432
2. Connect ports P0.08 and P0.06 of nRF52 to ports J1.4 and J1.3 of the BoosterPack board respectively using the wires
3. Turn on the Nordic Thingy:52
4. Turn on the Nordic nrf52dk. Make sure either it has a battery with power or is connected to a power supply via the micro usb port.
5. Connect a power supply to the MSP432 board
6. Ready to use!

***
Illustrations

![WhatsApp Image 2025-01-25 at 17 58 55](https://github.com/user-attachments/assets/e2e87625-18a6-4f4d-934e-8101017b1eb6)
![WhatsApp Image 2025-01-25 at 17 58 54(3)](https://github.com/user-attachments/assets/35f27e1b-5c13-4a1b-b196-42a63d383394)
![WhatsApp Image 2025-01-25 at 17 58 54(2)](https://github.com/user-attachments/assets/d4d73d66-21f5-448d-89d1-df2ad340629d)
![WhatsApp Image 2025-01-25 at 17 58 54(1)](https://github.com/user-attachments/assets/984bb302-fd9f-4567-9c96-3622faf180af)
![WhatsApp Image 2025-01-25 at 17 58 54](https://github.com/user-attachments/assets/bf45d9d5-d4f8-447c-9cda-a076ba398405)
****
Powerpoint Presentation & Video link

- Demonstration video: https://drive.google.com/file/d/1XCggXwoIA0TfUid-i-VX_US3aubdrWZ0/view?usp=share_link
- Power Point: https://github.com/AffaganoIsTheWay/SmartSense/blob/873adc31c0c6759e1cb17bd6ead0c28609d2d425/SmartSense.pdf
****
Colaboratores

- Giovanni Vitiello
- Trisha Agrawal
- Christian Li Sivertsen
- Federico Gasperi
