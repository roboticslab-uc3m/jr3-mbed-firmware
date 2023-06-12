/**
 * Copyright: 2023 (C) Universidad Carlos III de Madrid
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE
 **/

#include "mbed.h"
#include "LPC17xx.h"

PinName sensorClockPin = p9;
PinName sensorDataPin = p10;

// DigitalOut led(LED1);

class SensorInterruptHandler
{
public:
    SensorInterruptHandler(PinName clockPin, PinName dataPin, Thread & _thread)
        : clock(clockPin),
          data(dataPin),
          clockIn(clockPin),
          dataIn(dataPin),
          thread(_thread),
          isTransmittingFrame(false),
          isStartPulse(false),
          clockState(true),
          dataState(true),
          index(FRAME_LENGTH),
          buffer(SIGNAL_BIT)
    {
        clock.fall(callback(this, &SensorInterruptHandler::onClockFallingEdge));
        clock.rise(callback(this, &SensorInterruptHandler::onClockRisingEdge));

        data.fall(callback(this, &SensorInterruptHandler::onDataFallingEdge));
        data.rise(callback(this, &SensorInterruptHandler::onDataRisingEdge));
    }

    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

private:
    void onClockFallingEdge()
    {
        clockState = false;
    }

    void onClockRisingEdge()
    {
        clockState = true;

        if (isTransmittingFrame)
        {
            index--;

            if (!dataState)
            {
                buffer = buffer | (1UL << index);
            }
            else
            {
                buffer = buffer | ((index % 2 == 0) << index);
            }

            if (index == 0)
            {
                isTransmittingFrame = false;
                thread.flags_set(buffer);
                index = FRAME_LENGTH;
                // led = 0;
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
        dataState = false;

        if (!isTransmittingFrame && clockState)
        {
            isStartPulse = true;
        }
    }

    void onDataRisingEdge()
    {
        dataState = true;

        if (!isTransmittingFrame && isStartPulse && clockState)
        {
            isStartPulse = false;
            isTransmittingFrame = true;
            // led = 1;
            // buffer = SIGNAL_BIT;
        }

        // buffer |= (dataState ? 1 : 0) << --index;

        // if (index == 0)
        // {
        //     thread.flags_set(buffer);
        //     index = FRAME_LENGTH;
        //     buffer = SIGNAL_BIT;
        // }
    }

    InterruptIn clock;
    InterruptIn data;
    DigitalIn clockIn;
    DigitalIn dataIn;
    Thread & thread;

    volatile bool isTransmittingFrame;
    volatile bool isStartPulse; // pulse high-low-high on DATA while DLCK is high
    volatile bool clockState;
    volatile bool dataState;
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

        printf("0x%08x 0x%08x 0x%02x\n", rawData, data, rawChannel);

        // switch (rawChannel)
        // {
        // case VOLTAGE_LEVEL:
        //     printf("Voltage level: %d\n", data);
        //     break;
        // case FX:
        //     printf("Fx: %d\n", data);
        //     break;
        // case FY:
        //     printf("Fy: %d\n", data);
        //     break;
        // case FZ:
        //     printf("Fz: %d\n", data);
        //     break;
        // case MX:
        //     printf("Mx: %d\n", data);
        //     break;
        // case MY:
        //     printf("My: %d\n", data);
        //     break;
        // case MZ:
        //     printf("Mz: %d\n", data);
        //     break;
        // case CALIBRATION:
        //     printf("Calibration: %d\n", data);
        //     break;
        // default:
        //     printf("Unknown channel: %d\n", data);
        //     break;
        // }
    }
}

int main()
{
    Thread sensorThread;
    SensorInterruptHandler sensorInterrupt(sensorClockPin, sensorDataPin, sensorThread);

    sensorThread.start(callback(sensorWorker, &sensorInterrupt));

    while (true)
    {
        printf("loop\n");
        ThisThread::sleep_for(1000);
    }

    sensorThread.join();
}
