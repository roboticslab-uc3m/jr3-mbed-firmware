/*
 * Copyright (c) 2017-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

// Semaphore sem(1);
// Thread thread;

class Counter {
public:
    Counter(PinName clockPin, PinName dataPin)
        : _clockInterrupt(clockPin),
          _dataInterrupt(dataPin),
          isTransmittingFrame(false),
          isStartPulse(false),
          clockState(true),
          dataState(true),
          index(FRAME_LENGTH),
          buffer(SIGNAL_BIT),
          frames(0)
    {
        _clockInterrupt.rise(callback(this, &Counter::clockRise));
        _clockInterrupt.fall(callback(this, &Counter::clockFall));
        _dataInterrupt.rise(callback(this, &Counter::dataRise));
        _dataInterrupt.fall(callback(this, &Counter::dataFall));
        // sem.acquire();
    }

    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

    void clockRise()
    {
        _countClock++;

        clockState = true;

        if (isTransmittingFrame)
        {
            // index--;

            // if (!dataState)
            // {
            //     buffer = buffer | (1UL << index);
            // }
            // else
            // {
            //     buffer = buffer | ((index % 2 == 0) << index);
            // }

            buffer |= (dataState ? 1 : 0) << --index;
            // buffer |= (index % 2) << --index;

            if (index == 0)
            {
                isTransmittingFrame = false;
                frames++;
                // thread.flags_set(buffer);
                // sem.release();
                index = FRAME_LENGTH;
            }
        }
        else if (isStartPulse)
        {
            // invalidate, we are amid a frame transmission (which is not being accounted)
            isStartPulse = false;
        }
    }

    void clockFall()
    {
        clockState = false;
    }

    int readClock()
    {
        return _countClock;
    }

    void dataRise()
    {
        _countData++;

        dataState = true;

        if (!isTransmittingFrame && isStartPulse && clockState)
        {
            isStartPulse = false;
            isTransmittingFrame = true;
            buffer = SIGNAL_BIT;
        }
    }

    void dataFall()
    {
        dataState = false;

        if (!isTransmittingFrame && clockState)
        {
            isStartPulse = true;
        }
    }

    int readData()
    {
        return _countData;
    }

    uint32_t readBuffer() const
    { return buffer; }

    int getFrames() const
    { return frames; }

private:
    InterruptIn _clockInterrupt;
    InterruptIn _dataInterrupt;

    volatile int _countClock;
    volatile int _countData;

    volatile bool isTransmittingFrame;
    volatile bool isStartPulse; // pulse high-low-high on DATA while DLCK is high
    volatile bool clockState;
    volatile bool dataState;
    volatile int index;
    volatile uint32_t buffer;
    volatile int frames;
};

Counter counter(p9, p10);

void sensorWorker()
{
    uint32_t raw;

    while (true)
    {
        raw = ThisThread::flags_wait_all(Counter::SIGNAL_BIT) & 0x000FFFFF; // 20 bits

        uint8_t rawChannel = raw >> 16; // 4 MSB
        uint16_t rawData = raw & 0x0000FFFF; // two's complement representation of a signed 16-bit number
        int16_t data = static_cast<int16_t>(rawData);

        // printf("0x%08x 0x%08x 0x%02x\n", rawData, data, rawChannel);
        printf("Count so far: %d %d 0x%08X\n", counter.readClock(), counter.readData(), counter.readBuffer());
    }
}

int main()
{
    // thread.start(callback(sensorWorker));

    while (1)
    {
        // printf("loop\n");
        // ThisThread::sleep_for(1000);
        // sem.acquire();
        printf("Count so far: cticks=%d dticks=%d frames=%d data=0x%08X\n", counter.readClock(), counter.readData(), counter.getFrames(), counter.readBuffer());
        ThisThread::sleep_for(2000);
        // sem.acquire();
    }
}
