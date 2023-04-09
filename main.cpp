/**
 * Copyright: 2023 (C) Universidad Carlos III de Madrid
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE
 **/

#include "mbed.h"

PinName sensorClockPin = p9;
PinName sensorDataPin = p10;

class SensorInterruptHandler
{
public:
    SensorInterruptHandler(PinName clockPin, PinName dataPin, Thread & _thread)
        : interrupt(clockPin),
          data(dataPin),
          thread(_thread),
          index(FRAME_LENGTH),
          buffer(SIGNAL_BIT)
    {
        interrupt.rise(callback(this, &SensorInterruptHandler::onClockRisingEdge));
    }

    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

private:
    void onClockRisingEdge()
    {
        buffer |= data << --index;

        if (index == 0)
        {
            thread.flags_set(buffer);
            index = FRAME_LENGTH;
            buffer = SIGNAL_BIT;
        }
    }

    InterruptIn interrupt;
    DigitalIn data;
    Thread & thread;
    volatile int index;
    volatile uint32_t buffer;
};

enum SensorChannel : uint8_t
{
    VOLTAGE_LEVEL = 0,
    FX, FY, FZ,
    MX, MY, MZ,
    CALIBRATION
};

void sensorWorker(SensorInterruptHandler * handler)
{
    uint32_t raw;

    while (true)
    {
        raw = ThisThread::flags_wait_all(SensorInterruptHandler::SIGNAL_BIT) & 0x000FFFFF; // 20 bits

        uint8_t rawChannel = raw >> 16; // 4 MSB
        uint16_t rawData = raw & 0x0000FFFF; // two's complement representation of a signed 16-bit number

        int16_t data = static_cast<int16_t>(rawData);

        switch (rawChannel)
        {
        case VOLTAGE_LEVEL:
            printf("Voltage level: %d\n", data);
            break;
        case FX:
            printf("Fx: %d\n", data);
            break;
        case FY:
            printf("Fy: %d\n", data);
            break;
        case FZ:
            printf("Fz: %d\n", data);
            break;
        case MX:
            printf("Mx: %d\n", data);
            break;
        case MY:
            printf("My: %d\n", data);
            break;
        case MZ:
            printf("Mz: %d\n", data);
            break;
        case CALIBRATION:
            printf("Calibration: %d\n", data);
            break;
        default:
            printf("Unknown channel: %d\n", data);
            break;
        }
    }
}

int main()
{
    Thread sensorThread;
    SensorInterruptHandler sensorInterrupt(sensorClockPin, sensorDataPin, sensorThread);

    sensorThread.start(callback(sensorWorker, &sensorInterrupt));

    while (true)
    {}

    sensorThread.join();
}
