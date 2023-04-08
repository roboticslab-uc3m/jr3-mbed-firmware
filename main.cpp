/**
 * Copyright: 2023 (C) Universidad Carlos III de Madrid
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE
 **/

#include "mbed.h"

class SensorInterruptHandler
{
public:
    SensorInterruptHandler(PinName clockPin, PinName dataPin, EventFlags & _flags)
        : interrupt(clockPin),
          data(dataPin),
          flags(_flags),
          index(0)
    {
        interrupt.rise(callback(this, &SensorInterruptHandler::accept));
    }

    static const int HIGHEST_INDEX = 20;

private:
    void accept()
    {
        unsigned int value = data.read();

        if (value)
        {
            flags.set(1 << index);
        }

        if (index == 0)
        {
            // signal the blocked thread to resume execution
            flags.set(1 << HIGHEST_INDEX);
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
    CALIBRATION,
    INVALID
};

int main()
{
    EventFlags sensorFlags;
    SensorInterruptHandler sensorInterrupt(p10, p9, sensorFlags);

    while (true)
    {
        uint32_t raw = sensorFlags.wait_all(1 << SensorInterruptHandler::HIGHEST_INDEX);

        if (raw >> 31 == 0) // no error, see osFlagsError otherwise
        {
            raw &= 0x000FFFFF; // 20 bits
            uint8_t rawChannel = raw >> 16;
            uint16_t rawData = raw & 0x0000FFFF;

            switch (rawChannel)
            {
            case VOLTAGE_LEVEL:
                break;
            case FX:
                break;
            case FY:
                break;
            case FZ:
                break;
            case MX:
                break;
            case MY:
                break;
            case MZ:
                break;
            case CALIBRATION:
                break;
            default:
                break;
            }
        }
    }
}
