#include "mbed.h"

#define GPIO_PORT Port0
#define CLOCK_PIN (1UL << 0) // P0.0
#define DATA_PIN (1UL << 1) // P0.1

class SensorFrameHandler
{
public:
    SensorFrameHandler(PortName portName, uint32_t portMask)
        : port(portName, portMask),
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
        int value;

        while (true)
        {
            value = port.read();

            if ((value & CLOCK_PIN) == 1) // clock level high
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
                            // thread.flags_set(buffer);
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
                }
            }

            if ((value & DATA_PIN) == 1) // data level high
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

private:
    PortIn port;

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

int main()
{
    SensorFrameHandler sensorHandler(GPIO_PORT, 0x00000003);//CLOCK_PIN | DATA_PIN);

    Thread thread;
    thread.start(callback(&sensorHandler, &SensorFrameHandler::worker));

    while (true)
    {
        printf("clockTickCounter: %lu, dataTickCounter: %lu\n", sensorHandler.clockTickCounter, sensorHandler.dataTickCounter);
        ThisThread::sleep_for(1000);
    }

    thread.join();
    return 0;
}
