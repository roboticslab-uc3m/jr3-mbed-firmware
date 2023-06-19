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

// Semaphore sem(1);

class SensorFrameHandler
{
public:
    // SensorFrameHandler(Thread & _thread)
    SensorFrameHandler()
        :
        //   thread(_thread),
        //   port(Port0, 0x00000003),
          clockTickCounter(0),
          dataTickCounter(0),
          frameCounter(0),
          isTransmittingFrame(false),
          isStartPulse(false),
          clockState(true),
          dataState(true),
          index(FRAME_LENGTH),
          buffer(SIGNAL_BIT)
    {
        // sem.acquire();
    }

    static const unsigned int STORAGE_SIZE = 1000;
    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

    void worker()
    {
        bool clkLevel, dataLevel;
        int storageIndex = 0;
        int counter = 0;
        int counter2 = 0;
        const int timeout = 50;
        int resets = 0;
        bool clockValues[STORAGE_SIZE];
        bool dataValues[STORAGE_SIZE];
        int valueCounter = 0;
        // int pins;
        bool wasFirstRead = false;

        while (true)
        {
            dataLevel = data.read() == 1;
            clkLevel = clk.read() == 1;

            if (!wasFirstRead)
            {
                if (!clkLevel)
                {
                    continue;
                }
                else
                {
                    wasFirstRead = true;
                }
            }

            // dataLevel = ((((LPC_GPIO_TypeDef*)(p10 & ~0x1F))->FIOPIN & data.container.mask));
            // clkLevel = ((((LPC_GPIO_TypeDef*)(p9 & ~0x1F))->FIOPIN & clk.container.mask));

            // pins = port.read();
            // clkLevel = pins & 0x00000001;
            // dataLevel = pins & 0x00000002;

            clockValues[valueCounter] = clkLevel;
            dataValues[valueCounter] = dataLevel;

            if (valueCounter == STORAGE_SIZE)
            {
                printf("[DCLK] ");

                for (int i = 0; i < STORAGE_SIZE; i++)
                {
                    printf("%d", clockValues[i]);
                }

                printf("\n[DATA] ");

                for (int i = 0; i < STORAGE_SIZE; i++)
                {
                    printf("%d", dataValues[i]);
                }

                printf("\n");

                valueCounter = 0;
            }

            valueCounter++;

            // if (clkLevel && clockState)
            // {
            //     counter2++;

            //     if (++counter == timeout)
            //     {
            //         counter = 0;
            //         isTransmittingFrame = false;
            //     }
            // }
            // else
            // {
            //     counter = 0;

            //     if (clkLevel) // clock level high
            //     {
            //         if (!clockState) // clock rising edge
            //         {
            //             clockTickCounter++;
            //             clockState = true;

            //             if (isTransmittingFrame)
            //             {
            //                 buffer |= (dataLevel ? 1 : 0) << --index;

            //                 if (index == 0)
            //                 {
            //                     storage[storageIndex++] = buffer;
            //                     frameCounter++;

            //                     if (storageIndex == STORAGE_SIZE)
            //                     {
            //                         printf("cticks = %lu, dticks = %lu, frames = %lu, resets = %d, counter2 = %d\n",
            //                                clockTickCounter, dataTickCounter, frameCounter, resets, counter2);

            //                         for (int i = 0; i < STORAGE_SIZE; i++)
            //                         {
            //                             printf("[%03d] 0x%08x\n", i + 1, storage[i] & 0x000FFFFF);
            //                         }

            //                         storageIndex = 0;
            //                         resets = 0;
            //                     }

            //                     isTransmittingFrame = false;
            //                     // thread.flags_set(buffer);
            //                 }
            //             }

			// 			if (isStartPulse) // isTransmittingFrame is already false
            //             {
            //                 // invalidate, we are amid a frame transmission (which is not being accounted)
            //                 isTransmittingFrame = isStartPulse = false;
            //             }
            //         }
            //     }
            //     else // clock level low
            //     {
            //         if (clockState) // clock falling edge
            //         {
            //             clockState = false;

            //             if (isStartPulse)
            //             {
            //                 // printf("resets = %d, isTransmittingFrame = %s\n", resets, isTransmittingFrame ? "true" : "false");
            //                 isTransmittingFrame = isStartPulse = false; // invalidate
            //                 resets++;
            //             }
            //         }
            //     }
            // }

            // if (dataLevel) // data level high
            // {
            //     if (!dataState) // data rising edge
            //     {
            //         dataTickCounter++;
            //         dataState = true;

            //         if (!isTransmittingFrame && isStartPulse && clkLevel)
            //         {
            //             isStartPulse = false;
            //             isTransmittingFrame = true;
			// 			index = FRAME_LENGTH;
            //             buffer = SIGNAL_BIT;
            //             // counter = 0; // FIXME: is this enough?
            //         }
            //     }
            // }
            // else // data level low
            // {
            //     if (dataState) // data falling edge
            //     {
            //         dataState = false;

            //         if (!isTransmittingFrame && clkLevel)
            //         {
            //             isStartPulse = true;
            //         }
            //     }
            // }
        }
    }

// private:
    FastIn<p9> clk;
    FastIn<p10> data;
    // PortIn port;

    // Thread & thread;

    volatile unsigned long clockTickCounter;
    volatile unsigned long dataTickCounter;
    volatile unsigned long frameCounter;
    volatile bool isTransmittingFrame;
    volatile bool isStartPulse; // pulse high-low-high on DATA while DLCK is high
    volatile bool clockState;
    volatile bool dataState;
    volatile int index;
    volatile uint32_t buffer;
    volatile uint32_t storage[STORAGE_SIZE];
};

// void worker()
// {
//     uint32_t raw;

//     while (true)
//     {
//         raw = ThisThread::flags_wait_all(SensorFrameHandler::SIGNAL_BIT) & 0x000FFFFF; // 20 bits

//         uint8_t rawChannel = raw >> 16; // 4 MSB
//         uint16_t rawData = raw & 0x0000FFFF; // two's complement representation of a signed 16-bit number

//         int16_t data = static_cast<int16_t>(rawData);

//         printf("0x%08x 0x%08x 0x%02x\n", rawData, data, rawChannel);
//     }
// }

int main()
{
    // Thread thread;
    // SensorFrameHandler sensor(thread);
    SensorFrameHandler sensor;

    // thread.start(callback(&worker));
    // thread.start(callback(&sensor, &SensorFrameHandler::worker));

    printf("worker start\n");
    sensor.worker();

    // while (true)
    // {
        printf("cticks = %lu, dticks = %lu, frames = %lu\n", sensor.clockTickCounter, sensor.dataTickCounter, sensor.frameCounter);
    //     ThisThread::sleep_for(1000);
    // }
}
