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
    SensorInterruptHandler(PinName clockPin, PinName dataPin, EventFlags & _flags)
        : interrupt(clockPin),
          data(dataPin),
          flags(_flags),
          index(HIGHEST_INDEX)
    {
        interrupt.rise(callback(this, &SensorInterruptHandler::accept));
    }

    static const int HIGHEST_INDEX = 20;

private:
    void accept()
    {
        if (data)
        {
            flags.set(1UL << index);
        }

        if (index == 0)
        {
            // signal the blocked thread to resume execution
            flags.set(1UL << HIGHEST_INDEX);
            index = HIGHEST_INDEX;
        }

        index--;
    }

    InterruptIn interrupt;
    DigitalIn data;
    EventFlags & flags;
    volatile int index;
};

enum SensorChannel : uint8_t
{
    VOLTAGE_LEVEL = 0,
    FX, FY, FZ,
    MX, MY, MZ,
    CALIBRATION
};

int main()
{
    EventFlags sensorFlags;
    SensorInterruptHandler sensorInterrupt(sensorClockPin, sensorDataPin, sensorFlags);

    uint32_t raw;

    while (true)
    {
        raw = sensorFlags.wait_all(1UL << SensorInterruptHandler::HIGHEST_INDEX);

        if (raw >> 31 == 0) // no error, see osFlagsError otherwise
        {
            raw &= 0x000FFFFF; // 20 bits
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
}
