.. _bluetooth_central:

SmartSense
##################

Overview
********

SmartSense is a technology aimed at detecting and displaying the Carbon Dioxide level (ppm), humidity and temperature. Developed using NordicThingy nrf52, NordicThingy:52 and BoosterpackXL kits in C language, it can be deployed by any operating system. SmartSense is great for people who like to keep a constant update of their environment's condition. The bluetooth sensor of the NordicThingy:52 allows us to actively update the state of the environment according to time of request. The display and joystick from the BoosterpackXL kit displays them in a clear and precise form, with the joystick allowing to shuffle to the parameter the user wants to see. 

Pre-requisits
************
1. NordicThingy:52 IOT sensor kit
2. NordicThingy nRF52 development kit
3. BOOSTXL-EDUMKII Educational BoosterPack
4. J-link software from https://www.segger.com/downloads/jlink/
5. nRF desktop application (SDK version)
6. Code composer studio
7. TI driver library
8. Visual Studio Code

Getting Started
********************

Connect NordicThingy: 52 and the BoosterpackXL boards to a power supply.
Ports P0.08 and P0.06 of NordicThingy: 52 are connected to ports J1.4 and J1.3 of the BoosterpackXL board respectively.
Run nordic.c, texas.c


