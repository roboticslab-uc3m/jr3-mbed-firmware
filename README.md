# LacqueyFetch Firmware

Simple MBED 2 application to receive float values via CAN, interpret them as PWM signals (ranging from -100.0 to 100.0) and command a connected LacqueyFetch device.

## Installation

Since Mbed Online Compiler has been discontinued, development and compilation should be carried out at https://studio.keil.arm.com/. Make sure you select "mbed LPC1768" as the compilation target. Once build, plug in the MBED to a USB port of your PC and drag-and-drop the downloaded .bin file into it.

## See also
- [legacy-PIC.tar.gz](https://github.com/roboticslab-uc3m/yarp-devices/blob/c4730b862bed212f73c7dbdc4660b2ed0e8b1dc7/firmware/LacqueyFetch/legacy-PIC.tar.gz): legacy code and binary hex of the former PIC controller
