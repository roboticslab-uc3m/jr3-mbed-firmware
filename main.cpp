#include "mbed.h"
#include "FastIO.h"

// FastIn<p9> clk;
// FastIn<p10> data;

// volatile bool stateClock = true;
// volatile bool stateData = true;

// volatile int clockTicks = 0;
// volatile int dataTicks = 0;

#define CLOCK_PIN p9
#define DATA_PIN p10

class SensorFrameHandler
{
public:
    SensorFrameHandler(Thread & _thread)
    // SensorFrameHandler()
        :
          thread(_thread),
          clockTickCounter(0),
          dataTickCounter(0),
          frameCounter(0),
          isTransmittingFrame(false),
          isStartPulse(false),
          clockState(true),
          dataState(true),
          index(FRAME_LENGTH),
          buffer(SIGNAL_BIT)
    {}

    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

    void worker()
    {
        bool _clk, _data;
        int counter = 0;
        const int timeout = 10;

        while (true)
        {
            _clk = clk.read() == 1;
            _data = data.read() == 1;

            if (_clk == clockState) {
                counter++;
                continue;
            }

            if (_clk) // clock level high
            {
                if (!clockState) // clock rising edge
                {
                    clockTickCounter++;
                    clockState = true;

                    if (isTransmittingFrame)
                    {
                        buffer |= (dataState ? 1 : 0) << --index;

                        if (index == 0)
                        {
                            frameCounter++;
                            isTransmittingFrame = false;
                            thread.flags_set(buffer);
                            index = FRAME_LENGTH;
                        }
                    }
                    else if (isStartPulse)
                    {
                        // invalidate, we are amid a frame transmission (which is not being accounted)
                        isStartPulse = false;
                    }
                }
            }
            else // clock level low
            {
                if (clockState) // clock falling edge
                {
                    clockState = false;
                    isStartPulse = false;
                }
            }

            if (_data) // data level high
            {
                if (!dataState) // data rising edge
                {
                    dataTickCounter++;
                    dataState = true;

                    if (!isTransmittingFrame && isStartPulse && clockState)
                    {
                        isStartPulse = false;
                        isTransmittingFrame = true;
                        buffer = SIGNAL_BIT;
                    }
                }
            }
            else // data level low
            {
                if (dataState) // data falling edge
                {
                    dataState = false;

                    if (!isTransmittingFrame && clockState)
                    {
                        isStartPulse = true;
                    }
                }
            }
        }
    }

// private:
    FastIn<p9> clk;
    FastIn<p10> data;

    Thread & thread;

    volatile unsigned long clockTickCounter;
    volatile unsigned long dataTickCounter;
    volatile unsigned long frameCounter;
    volatile bool isTransmittingFrame;
    volatile bool isStartPulse; // pulse high-low-high on DATA while DLCK is high
    volatile bool clockState;
    volatile bool dataState;
    volatile int index;
    volatile uint32_t buffer;
};

void worker(SensorFrameHandler * sensor)
{
    uint32_t raw;

    while (true)
    {
        raw = ThisThread::flags_wait_all(SensorFrameHandler::SIGNAL_BIT) & 0x000FFFFF; // 20 bits
        raw = sensor->buffer;

        uint8_t rawChannel = raw >> 16; // 4 MSB
        uint16_t rawData = raw & 0x0000FFFF; // two's complement representation of a signed 16-bit number

        int16_t data = static_cast<int16_t>(rawData);

        printf("0x%08x 0x%08x 0x%02x\n", rawData, data, rawChannel);
    }
}

int main()
{
    Thread thread;
    SensorFrameHandler sensor(thread);
    // SensorFrameHandler sensor;

    thread.start(callback(&worker, &sensor));
    // thread.start(callback(&sensor, &SensorFrameHandler::worker));

    sensor.worker();

    // while (true)
    // {
    //     printf("cticks = %lu, dticks = %lu, frames = %lu\n", sensor.clockTickCounter, sensor.dataTickCounter, sensor.frameCounter);
    //     ThisThread::sleep_for(1000);
    // }
}
