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
        : clock(clockPin),
          data(dataPin),
          thread(_thread),
          isTransmittingFrame(false),
          isStartPulse(false),
          index(FRAME_LENGTH),
          buffer(SIGNAL_BIT)
    {
        clock.rise(callback(this, &SensorInterruptHandler::onClockRisingEdge));
        data.fall(callback(this, &SensorInterruptHandler::onDataFallingEdge));
        data.rise(callback(this, &SensorInterruptHandler::onDataRisingEdge));
    }

    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

private:
    void onClockRisingEdge()
    {
        if (isTransmittingFrame)
        {
            buffer |= data << --index;

            if (index == 0)
            {
                isTransmittingFrame = false;
                thread.flags_set(buffer);
                index = FRAME_LENGTH;
                buffer = SIGNAL_BIT;
            }
        }
        else if (isStartPulse)
        {
            // invalidate, we are amid a frame transmission (which is not being accounted)
            isStartPulse = false;
        }
    }

    void onDataFallingEdge()
    {
        if (!isTransmittingFrame && clock == 1)
        {
            isStartPulse = true;
        }
    }

    void onDataRisingEdge()
    {
        if (!isTransmittingFrame && isStartPulse && clock == 1)
        {
            isStartPulse = false;
            isTransmittingFrame = true;
        }
    }

    InterruptIn clock;
    InterruptIn data;
    Thread & thread;

    volatile bool isTransmittingFrame;
    volatile bool isStartPulse; // pulse high-low-high on DATA while DLCK is high
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
