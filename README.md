# JR3 Mbed firmware

An MBED 2 application that performs data acquisition from a JR3 force-torque sensor and streams it through a CAN channel.

Incidentally, support was added for sending simple PWM commands to a Lacquey fetch hand through a secondary optional CAN channel.

## Installation

Since Mbed Online Compiler has been discontinued, development and compilation should be carried out at https://studio.keil.arm.com/. Make sure you select "mbed LPC1768" as the compilation target. Once build, plug in the MBED to a USB port of your PC and drag-and-drop the downloaded .bin file into it.

## CAN protocol

| command             | op code | direction | payload<br>(bytes) | details                                                                                           |
|---------------------|:-------:|:---------:|:------------------:|---------------------------------------------------------------------------------------------------|
| sync                |  0x080  |     in    |          0         |                                                                                                   |
| bootup              |  0x100  |    out    |          0         |                                                                                                   |
| acknowledge         |  0x180  |    out    |          1         | 0x00: sensor ready<br>0x01: not initialized                                                       |
| **start sync**      |  0x200  |     in    |          2         | low-pass filter cutoff frequency in 0.1*Hz (integer)<br>(e.g. 102 = 10.2 Hz)                      |
| **start async**     |  0x280  |     in    |          6         | 2 LSB bytes: cutoff frequency (as above)<br>4 MSB bytes: period in us (integer)                   |
| **stop**            |  0x300  |     in    |          0         |                                                                                                   |
| **zero offsets**    |  0x380  |     in    |          0         |                                                                                                   |
| **set filter**      |  0x400  |     in    |          2         | cutoff frequency (as above)                                                                       |
| **get full scales** |  0x480  |     in    |          0         | sends "force data" and "moment data" (see below)<br>with full scales instead of live measurements |
| **reset**           |  0x500  |     in    |          0         |                                                                                                   |
| force data          |  0x580  |    out    |          8         | (3x) 2 LSB bytes: Fx, Fy, Fz (integer, signed)<br>2 MSB bytes: integrity counter                  |
| moment data         |  0x600  |    out    |          8         | (3x) 2 LSB bytes: Mx, My, Mz (integer, signed)<br>2 MSB bytes: integrity counter                  |
| gripper PWM         |  0x780  |     in    |          4         | PWM command between -100.0 and 100.0 (float)                                                      |

Bolded incoming commands imply that the Mbed will respond with an acknowledge message (following other generated response messages, if any, as in "get full scales").

## See also

- Alberto López Esteban, *Diseño y desarrollo de un módulo de conexión a CANopen de un sensor comercial fuerza/par*, bachelor's thesis, Universidad Carlos III de Madrid, 2011
- Carlos de la Hoz Najarro, *Puesta en marcha del sensor fuerza/par JR3*, bachelor's thesis, Universidad Carlos III de Madrid, 2011
- Javier Berrocal, *Design and implementation of a data acquisition system for force/torque sensors*, master's thesis, Universidad Carlos III de Madrid, 2019
- https://github.com/roboticslab-uc3m/yarp-devices/issues/263
- https://github.com/roboticslab-uc3m/jr3pci-linux/
