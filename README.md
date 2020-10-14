# STM32WB55 MCU project

The goal of this project is to implement a sensor MCU-based network, where the MCU collects and then sends the data via Radio to a processing system (Cloud or Embedded system).

<p align="center">
  <img src="Doc/Images/pic1.png" width="600">
</p>

## Hardware
  - MCU -> STM32WB55xx based MCU (Nucleo 68 Board)
  - Radio - Onboard radio transceiver (MAC 802.15.4)
  - Camera -> OV7670 (I2C and Parallel Data)
  - Temperature sensor -> KY028 (Serial)
  - Vibration/Tilt sensors -> KY020 / KY002 (Serial)
  - Light sensor -> KY018 (Serial)

## Summary
In this folder we have all the apps that implement one part of the project. So for every app we have the following:
  - MCU_Base_App has the non-RF implementation of the project where the drivers for the sensors and the camera are implemented
  - MAC_802_15_4_RFD_App containts the implementation of the Transmitter part of the RF(RFD node)
  - MAC_802_15_4_FFD_App containts the implementation of the Receiver part of the RF(FFD node)

So for the MCU_Base_App a diagram of operation:
<p align="center">
  <img src="Doc/Images/pic3.png" width="600">
</p>

While for the RFD (client) and FFD (server) apps we have:
<p align="center">
  <img src="Doc/Images/pic2.png" width="600">
</p>
 
#### MCU GPIOs - Sensors/Camera Pins

| MCU GPIOs | Function | Sensors/Camera Pins |
| ------ | ------ | ------ |
| ------ | **MCU** | ------ |
| PB6 | UART Transmit (TX) | ------ |
| PB7 | UART Transmit (RX) | ------ |
| ------ | **CAMERA** | ------ |
| PA0-PA7 | Camera Parallel Output | Data0-Data7 |
| PB4 | Vertical sync | VSYNC |
| PC3 | Horizontal sync | HSYNC |
| PC1 | Pixel clock | PXCLK |
| PC10 | Camera Reset | RET |
| PC12 | Camera Power Down | PWDN |
| PA8 | Camera Input clock | XCLK |
| PB8 | I2C SCL | SIO_C |
| PB9 | I2C SDA | SIO_D |
| ------ | **SENSORS** | ------ |
| PB2 | Digital serial KY-002 | D0 |
| PC0 | Analog serial KY-018 | A0 |
| PE4 | Digital serial KY-020 | D0 |
| PC2 | Analog serial KY-028 | A0 |
| PC5 | Digital serial KY-028 | D0 |

Right above is the table with the GPIO connections used for the project (RFD_App and Base_App). A detailed table is also included at the start of the main.c source file.
Power for the camera is given from the 3.3V arduino supply pin on the board and for the sensors from the IOREF pin (also a 3.3V supply).

## Source code
### MCU_Base_App
In the **/Core/src** sub-folder the main logic is implemented, namely:
- **Drivers/sensor_lib.c** -> The sensor's routines and necessary data structures
- **Drivers/OV7670.c** -> Has the routines to setup, configure, get frames and transfer from the camera module
- **Drivers/uart.c** -> UART expansion driver, routines that imitate printf() function call
- **main.c** -> Assembles the drivers and uses them to print data to the serial terminal (frame or sensor data)

Every .c file (of the above) has its corresponding header file that has sets of global variables, external routines, etc..

- For the **Drivers/sensor_lib.h** , user can toggle ON/OFF with a simple define every sensor
- For the **Drivers/0V7670.h** , user can change the default values chosen for every camera register, as well as change key parameters of the camera itself

All of the logic is implemented in **Core/main.c** file
### MAC_802_15_4_RFD_App
Has all of the above, including RF functionality which is taken from ST's RF examples, namely:
- **app_rfd_mac_802_15_4.c** contains the main utilities used to use the RF (in the **/App** folder)
- All the other drivers are left inside **/Core/Src** folder

Generally small changes have been made in order for the main implementation to be usable inside ST's example project, some of those are:
- **HW_UART_Transmit()** function call instead of **HAL_UART_Transmit()** usage
- Minor additions to the clock initialization
- Added EXTI interrupts for push-buttons
- Changes in the **APP_RFD_MAC_802_15_4_SendData()** function call (Reads in binary format)

All of the logic is implemented in **Core/Src/main.c** file.

### MAC_802_15_4_FFD_App
For the receiver part we made changes to:
- Clock initialization (same with RFD app)
- Receive data callback works differently (checks for sensor/camera packets and removed XOR checking)
- Added only the **transfer_UART_frame()** routine from **OV7670.c** so we can transfer the frame after we receive it via RF

All of the logic is implemented in App/app_rfd_mac_802_15_4_process.c .

### ST App/Project examples
[STM32WB firmware] -> The link is the ST's firmware driver for STM32WB55xx based boards.

Our RF implementation is based on the MAC 802.15.4 applications made by ST. The original RFD and FFD apps transmitted a simple message from one to another.

MCU_Base_App is written completely by us (only the initial code was generated with STM32CubeMX).

In order to see the original work (RFD/FFD Apps) press this [[link]].


## Install
Tools that are needed for the installation or were used to create it:

* [Termite] - Serial terminal used to print messages from UART (one of the many choices)
* [STM32IDE] - STM's IDE used for the building of the source code
* [STM32CubeMX] - Create the code base used for the source code

In order to build the project, user must first install STM32CubeIDE, then:
  - Clone the app folder of their choice
  - Double click the .project file in the app folder
  - IDE should build the project and have it in the file explorer of the IDE
  - Go to menu **Project->Build** to compile the code
  - Then go to menu **Run->Run** to upload the binary to the board


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [Termite]: <https://www.compuphase.com/software_termite.htm>
   [STM32IDE]: <https://www.st.com/en/development-tools/stm32cubeide.html>
   [STM32CubeMX]: <https://www.st.com/en/development-tools/stm32cubemx.html>
   [STM32WB firmware]: <https://github.com/STMicroelectronics/STM32CubeWB>
   [link]: <https://github.com/STMicroelectronics/STM32CubeWB/tree/master/Projects/P-NUCLEO-WB55.Nucleo/Applications/Mac_802_15_4>
