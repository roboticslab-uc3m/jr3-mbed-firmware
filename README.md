# JR3 Mbed firmware

An Arm Mbed OS library that performs data acquisition from a JR3 force-torque sensor. This repository covers the JR3-Mbed communication layer. For the interface layer with an external PC, refer to [jr3-mbed-firmware-usb](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-usb) for USB and [jr3-mbed-firmware-can](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-can) for CAN.

## Usage

On bootup, the calibration matrix and full scales are queried from the sensor and stored for later use. A failure means that there is no connection to the sensor. Re-initialization may be requested during normal operation through the "reset" command. If the initialization succeeds, the JR3 controller is in "ready" state, otherwise it remains in "not initialized" state. All acknowledge messages carry this state information in their payload. The "get state" command is a no-op that can be used to ping the controller.

The JR3 sensor operates in two modes: synchronous and asynchronous. Both entail that a background thread will be performing data acquisition, decoupling, offset removal and filtering at full sensor bandwidth (8 KHz per channel).

- Synchronous ("start sync" command): for compatibility with the CiA 402 standard (a.k.a. CANopen), a SYNC message is meant to be broadcast over the CAN network by a producer node so that consumers act upon (e.g. send sensor data, accept motion commands). An Mbed application may process the incoming SYNC message and send the latest force and moment data in return.
- Asynchronous ("start async" command): an additional thread is spawned to query latest forces and moments at the specified fixed rate (tested at 1 ms).

The sensor is **not** calibrated by default. Use the "zero offsets" command to capture the current offset and substract it from subsequent filtered results. This command can be issued at any time.

It is highly recommended to enable raw data filtering by specifying the desired cutoff frequency to either start command. This firmware implements a simple first-order low-pass IIR filter, also known as an exponential moving average (see [Wikipedia article](https://w.wiki/7Er6)). Its cutoff frequency can be modified through the "set filter" command.

Outgoing force and moment data requires post-processing on the receiver's side. These signed integer values should be multiplied by the corresponding full scale and divided by a factor of 16384 (=2^14) for forces and 16384\*10 for moments. The resulting values will be expressed in Newtons and Newton*meters, respectively. Use the "get full scales" command to query the sensor full scales.

## Citation

If you found this project useful, please consider citing the following work:

Łukawski, B., Rodríguez-Sanz, A., Victores, J., & Balaguer, C. (2024). An open-source implementation of a force-torque sensor data acquisition device for the humanoid robot TEO. In Actas del Simposio de Robótica, Bioingeniería y Visión por Computador (pp. 79–84). Universidad de Extremadura.

```bibtex
@inproceedings{lukawski2024srbv,
    author={{\L}ukawski, Bartek and Rodríguez-Sanz, Alberto and Victores, Juan G. and Balaguer, Carlos},
    title={{An open-source implementation of a force-torque sensor data acquisition device for the humanoid robot TEO}},
    booktitle={Actas del Simposio de Robótica, Bioingeniería y Visión por Computador},
    year={2024},
    pages={79--84},
    publisher={Universidad de Extremadura},
    url={http://hdl.handle.net/10662/21260},
}
```

## Dependencies

This project depends on the following libraries developed and kindly shared by other MBED users. Their code has been embedded into this repository for convenience.

- <https://os.mbed.com/users/MultipleMonomials/code/AccurateWaiter/>
- <https://os.mbed.com/users/sravet/code/fixedpoint/>

## See also

- Alberto López Esteban, *Diseño y desarrollo de un módulo de conexión a CANopen de un sensor comercial fuerza/par*, bachelor's thesis, Universidad Carlos III de Madrid, 2011
- Carlos de la Hoz Najarro, *Puesta en marcha del sensor fuerza/par JR3*, bachelor's thesis, Universidad Carlos III de Madrid, 2011
- Javier Berrocal, *Design and implementation of a data acquisition system for force/torque sensors*, master's thesis, Universidad Carlos III de Madrid, 2019
- [roboticslab-uc3m/yarp-devices#263](https://github.com/roboticslab-uc3m/yarp-devices/issues/263)
- [roboticslab-uc3m/jr3-mbed-firmware-can](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-can)
- [roboticslab-uc3m/jr3-mbed-firmware-usb](https://github.com/roboticslab-uc3m/jr3-mbed-firmware-usb)
- [roboticslab-uc3m/jr3pci-linux](https://github.com/roboticslab-uc3m/jr3pci-linux)
