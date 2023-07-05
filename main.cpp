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
    SensorFrameHandler(Serial & pc)
        :
        //   thread(_thread),
        //   port(Port0, 0x00000003),
          pc(pc),
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

    static const unsigned int STORAGE_SIZE = 2000;
    static const unsigned FRAME_LENGTH = 20;
    static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

    void worker()
    {
        bool clkLevel, dataLevel;
        // int storageIndex = 0;
        // int counter = 0;
        // int counter2 = 0;
        // const int timeout = 50;
        // int resets = 0;
        bool clockValues[STORAGE_SIZE];
        bool dataValues[STORAGE_SIZE];
        int valueCounter = 0;
        // int pins;
        bool wasFirstRead = false;
        const int offset = 100000;
        int offsetCount = 0;

        while (true)
        {
            dataLevel = data.read() == 1;
            clkLevel = clk.read() == 1;

            if (offsetCount++ < offset)
            {
                continue;
            }

            // pc.printf("SystemCoreClock = %d Hz\n", SystemCoreClock);

            // if (!wasFirstRead)
            // {
            //     if (!clkLevel)
            //     {
            //         continue;
            //     }
            //     else
            //     {
            //         wasFirstRead = true;
            //     }
            // }

            // dataLevel = ((((LPC_GPIO_TypeDef*)(p10 & ~0x1F))->FIOPIN & data.container.mask));
            // clkLevel = ((((LPC_GPIO_TypeDef*)(p9 & ~0x1F))->FIOPIN & clk.container.mask));

            // pins = port.read();
            // clkLevel = pins & 0x00000001;
            // dataLevel = pins & 0x00000002;

            clockValues[valueCounter] = clkLevel;
            dataValues[valueCounter] = dataLevel;

            if (valueCounter == STORAGE_SIZE)
            {
                pc.printf("[DCLK] ");

                for (int i = 0; i < STORAGE_SIZE; i++)
                {
                    pc.printf("%d", clockValues[i]);
                }

                pc.printf("\n[DATA] ");

                for (int i = 0; i < STORAGE_SIZE; i++)
                {
                    pc.printf("%d", dataValues[i]);
                }

                pc.printf("\n");

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
    Serial & pc;
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
    printf("start\n");

    wait(5);

    const int Fin = 12000000; // 12MHz XTAL

    printf("PLL Registers:\n");
    printf(" - PLL0CFG = 0x%08X\n", LPC_SC->PLL0CFG);
    printf(" - CLKCFG  = 0x%08X\n", LPC_SC->CCLKCFG);

    int M = (LPC_SC->PLL0CFG & 0xFFFF) + 1;
    int N = (LPC_SC->PLL0CFG >> 16) + 1;
    int CCLKDIV = LPC_SC->CCLKCFG + 1;

    printf("Clock Variables:\n");
    printf(" - Fin = %d\n", Fin);
    printf(" - M   = %d\n", M);
    printf(" - N   = %d\n", N);
    printf(" - CCLKDIV = %d\n", CCLKDIV);

    int Fcco = (2 * M * Fin) / N;
    int CCLK = Fcco / CCLKDIV;

    printf("Clock Results:\n");
    printf(" - Fcco = %d\n", Fcco);
    printf(" - CCLK = %d\n", CCLK);

    printf("Initialising ...\n");
    printf("\n");
    LPC_SC->PLL0CON   = 0x00;             /* PLL0 Disable                    */
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;

    // LPC_SC->CCLKCFG   = 0x00000003;       /* Select Clock Divisor = 4        */
    // LPC_SC->CCLKCFG   = 0x00000001;       /* Select Clock Divisor = 2        */
    LPC_SC->CCLKCFG   = 0x00000002;       /* Select Clock Divisor = 3        */
    // LPC_SC->PLL0CFG   = 0x00020031;       /* configure PLL0                  */
    // LPC_SC->PLL0CFG   = 0x00000009;       /* configure PLL0                  */
    LPC_SC->PLL0CFG   = 0x0000000F;       /* configure PLL0                  */
    LPC_SC->PLL0FEED  = 0xAA;             /* divide by 3 then multiply by 50 */
    LPC_SC->PLL0FEED  = 0x55;             /* PLL0 frequency = 400,000,000    */

    LPC_SC->PLL0CON   = 0x01;             /* PLL0 Enable                     */
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;
    // printf("waiting ...\n");
    while (!(LPC_SC->PLL0STAT & (1<<26)));/* Wait for PLOCK0                 */

    // setSystemFrequency(0x3, 0x1, 15, 1);

    LPC_SC->PLL0CON   = 0x03;             /* PLL0 Enable & Connect           */
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;
    // printf("waiting ...\n");
    while (!(LPC_SC->PLL0STAT & ((1<<25) | (1<<24))));/* Wait for PLLC0_STAT & PLLE0_STAT */

    SystemCoreClockUpdate();
    Serial pc(USBTX, USBRX); // tx, rx
    pc.printf("System clock = %d\n", SystemCoreClock);

    // setSystemFrequency(0x3, 0x1, 15, 1);

    // printf("a\n");
    // ThisThread::sleep_for(5s);
    // printf("a\n");

    // printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
    // printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
    // printf("cccccccccccccccccccccccccccccccccccccccccc\n");
    // printf("dddddddddddddddddddddddddddddddddddddddddd\n");

    int test = (Fin * 2 /
                      (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1) *
                      ((LPC_SC->PLL0STAT & 0x7FFF) + 1)  /
                      ((LPC_SC->CCLKCFG & 0xFF)+ 1));
    pc.printf("System clock = %d\n", test);

    // SystemCoreClockUpdate();
    // pc.printf("System clock = %d\n", SystemCoreClock);

    pc.printf("PLL Registers:\n");
    pc.printf(" - PLL0CFG = 0x%08X\n", LPC_SC->PLL0CFG);
    pc.printf(" - CLKCFG  = 0x%08X\n", LPC_SC->CCLKCFG);

    M = (LPC_SC->PLL0CFG & 0xFFFF) + 1;
    N = (LPC_SC->PLL0CFG >> 16) + 1;
    CCLKDIV = LPC_SC->CCLKCFG + 1;

    pc.printf("Clock Variables:\n");
    pc.printf(" - Fin = %d\n", Fin);
    pc.printf(" - M   = %d\n", M);
    pc.printf(" - N   = %d\n", N);
    pc.printf(" - CCLKDIV = %d\n", CCLKDIV);

    Fcco = (2 * M * Fin) / N;
    CCLK = Fcco / CCLKDIV;

    pc.printf("Clock Results:\n");
    pc.printf(" - Fcco = %d\n", Fcco);
    pc.printf(" - CCLK = %d\n", CCLK);

    // Thread thread;
    // SensorFrameHandler sensor(thread);
    // Serial pc(USBTX, USBRX); // tx, rx
    SensorFrameHandler sensor(pc);

    // thread.start(callback(&worker));
    // thread.start(callback(&sensor, &SensorFrameHandler::worker));

    pc.printf("worker start\n");
    sensor.worker();

    // while (true)
    // {
        pc.printf("cticks = %lu, dticks = %lu, frames = %lu\n", sensor.clockTickCounter, sensor.dataTickCounter, sensor.frameCounter);
    //     ThisThread::sleep_for(1000);
    // }
}
