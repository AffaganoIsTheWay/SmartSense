## SmartSense
***
Table of contents:
- Overview
- Pre-requisits
- Project Layout
- Getting Started
- Illustrations
- Powerpoint Presentation & Video link

***
Overview

SmartSense is a technology aimed at detecting and displaying the Carbon Dioxide level (ppm), humidity and temperature. Developed using NordicThingy nrf52, NordicThingy:52 and BoosterpackXL kits in C language, it can be deployed by any operating system. SmartSense is great for people who like to keep a constant update of their environment's condition. The bluetooth sensor of the NordicThingy:52 allows us to actively update the state of the environment according to time of request. The display and joystick from the BoosterpackXL kit displays them in a clear and precise form, with the joystick allowing to shuffle to the parameter the user wants to see. 
***
Pre-requisits

  Hardware
  
    1. NordicThingy:52 IOT sensor kit
    2. NordicThingy nRF52 development kit
    3. BOOSTXL-EDUMKII Educational BoosterPack
    
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

1. 
2. Ports P0.08 and P0.06 of NordicThingy: 52 are connected to ports J1.4 and J1.3 of the BoosterpackXL board respectively.
3. Run nordic.c, texas.c
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
