# JR3 Mbed firmware

An MBED 2 application that performs data acquisition from a JR3 force-torque sensor and streams it through a CAN channel.

Incidentally, support was added for sending simple PWM commands to a Lacquey fetch hand through a secondary optional CAN channel.

## Installation

Since Mbed Online Compiler has been discontinued, development and compilation should be carried out at https://studio.keil.arm.com/. Make sure you select "mbed LPC1768" as the compilation target. Once build, plug in the MBED to a USB port of your PC and drag-and-drop the downloaded .bin file into it.

## See also

- Alberto López Esteban, *Diseño y desarrollo de un módulo de conexión a CANopen de un sensor comercial fuerza/par*, bachelor's thesis, Universidad Carlos III de Madrid, 2011
- Carlos de la Hoz Najarro, *Puesta en marcha del sensor fuerza/par JR3*, bachelor's thesis, Universidad Carlos III de Madrid, 2011
- Javier Berrocal, *Design and implementation of a data acquisition system for force/torque sensors*, master's thesis, Universidad Carlos III de Madrid, 2019
- https://github.com/roboticslab-uc3m/yarp-devices/issues/263
- https://github.com/roboticslab-uc3m/jr3pci-linux/
