#include "mbed.h"
#include "FastIO.h"

void setSystemFrequency(int clkDiv, int M, int N)
{
    // see M/N values at https://os.mbed.com/users/no2chem/notebook/mbed-clock-control--benchmarks/
    // this code was mostly borrowed from https://os.mbed.com/forum/mbed/topic/229/?page=2#comment-8566
    // see also https://os.mbed.com/users/no2chem/code/ClockControl//file/b5d3bd64d2dc/main.cpp/
    // manual entry: Chapter 4: LPC17xx CLocking and power control

    // current clock variables:
    // Fin = 12000000; // 12 MHz XTAL
    // M = (LPC_SC->PLL0CFG & 0xFFFF) + 1;
    // N = (LPC_SC->PLL0CFG >> 16) + 1;
    // CCLKDIV = LPC_SC->CCLKCFG + 1;
    // Fcco = (2 * M * Fin) / N;
    // CCLK = Fcco / CCLKDIV; // this should yield 96 MHz

    LPC_SC->PLL0CON   = 0x00; // PLL0 Disable
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;

    LPC_SC->CCLKCFG   = clkDiv - 1; // Select Clock Divisor
    LPC_SC->PLL0CFG   = (((unsigned int)N - 1) << 16) | (M - 1); // configure PLL0
    LPC_SC->PLL0FEED  = 0xAA;

    LPC_SC->PLL0CON   = 0x01; // PLL0 Enable
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;

    while (!(LPC_SC->PLL0STAT & (1<<26))) {} // Wait for PLOCK0

    LPC_SC->PLL0CON   = 0x03; // PLL0 Enable & Connect
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;

    while (!(LPC_SC->PLL0STAT & ((1<<25) | (1<<24)))) {} // Wait for PLLC0_STAT & PLLE0_STAT

    SystemCoreClockUpdate(); // updates the SystemCoreClock global variable, should be 128 MHz now

    // this is how SystemCoreClock is obtained:
    // SystemCoreClock = (Fin * 2 /
    //                              (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1) *
    //                              ((LPC_SC->PLL0STAT & 0x7FFF) + 1) /
    //                              ((LPC_SC->CCLKCFG & 0xFF) + 1));
}

int main()
{
    wait(5);

    // overclock CPU from 96 MHz to 128 MHz
    setSystemFrequency(3, 16, 1);

    Serial pc(USBTX, USBRX); // tx, rx
    pc.printf("System clock = %d\n", SystemCoreClock);

    wait(5);

    // PortIn port(Port0, 0x00000003);
    FastIn<p9> clock;
    FastIn<p10> data;

    const unsigned int STORAGE_SIZE = 100;
    const unsigned int FRAME_LENGTH = 20;
    const unsigned int CLOCK_TIMEOUT = 5;
    const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

    unsigned long clockTickCounter = 0;
    // unsigned long dataTickCounter = 0;
    unsigned long frameCounter = 0;

    bool isTransmittingFrame = false;
    bool isStartPulse = false; // pulse high-low-high on DATA while DLCK is high
    bool clockState = true;
    bool dataState = true;

    int index = FRAME_LENGTH;
    uint32_t buffer = SIGNAL_BIT;
    bool buffBool[FRAME_LENGTH];
    uint32_t storage[STORAGE_SIZE];

    bool clkLevel, dataLevel;

    int pins = 0;
    int storageIndex = 0;
    int idleCounter = 0;
    // int resets = 0;

    int idleStorage[50];
    int idleCtr2 = 0;
    int idleCtr3 = 0;

    // bool wasFirstRead = false;

    while (true)
    {
        // pins = port.read();
        // dataLevel = pins & 0x00000002;
        // clkLevel = pins & 0x00000001;
        clkLevel = clock.read();
        dataLevel = data.read();

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

        // if (clkLevel && clockState)
        // {
        //     // idleCtr2++;

        //     if (++idleCounter == CLOCK_TIMEOUT)
        //     {
        //         // pc.printf("idle\n");
        //         idleCounter = 0;
        //         isTransmittingFrame = false;

        //         // if (!clkLevel)
        //         // {
        //         //     // pc.printf("idle and low level\n");
        //         //     continue; // disregard data readings
        //         // }
        //     }
        // }
        // else
        {
            // idleStorage[idleCtr3++] = idleCtr2;
            // idleCtr2 = 0;

            // if (idleCtr3 == 50)
            // {
            //     for (int i = 0; i < 50; i++)
            //     {
            //         pc.printf("[%02d] %d\n", i, idleStorage[i]);
            //     }

            //     idleCtr3 = 0;
            // }

            // pc.printf("in clock read clause\n");
            // idleCounter = 0;

            if (clkLevel) // clock level high
            {
                if (!clockState) // clock rising edge
                {
                    // clockTickCounter++;
                    clockState = true;

                    if (isTransmittingFrame)
                    {
                        // pc.printf("reading %d\n", dataLevel);
                        // buffer |= (dataLevel ? 1 : 0) << --index;
                        buffBool[--index] = dataLevel;

                        if (index == 0)
                        {
                            // pc.printf("read: 0x%08x\n", buffer);
                            buffer = 0;

                            for (int i = 0; i < FRAME_LENGTH; i++)
                            {
                                buffer |= (buffBool[i] ? 1 : 0) << i;
                            }

                            storage[storageIndex++] = buffer;

                            frameCounter++;

                            if (storageIndex == STORAGE_SIZE)
                            {
                                // pc.printf("clockTickCounter = %d\n", clockTickCounter);

                                for (int i = 0; i < STORAGE_SIZE; i++)
                                {
                                    pc.printf("[%03d] 0x%08x\n", i + 1, storage[i] & 0x000FFFFF);
                                }

                                storageIndex = 0;
                                // resets = 0;
                            }

                            isTransmittingFrame = false;
                        }
                    }
                    else if (isStartPulse) // isTransmittingFrame is already false
                    {
                        // pc.printf("resetting flags A\n");
                        // invalidate, we are amid a frame transmission (which is not being accounted)
                        isTransmittingFrame = isStartPulse = false;
                    }
                }
            }
            else // clock level low
            {
                if (clockState) // clock falling edge
                {
                    clockState = false;

                    if (isStartPulse)
                    {
                        // pc.printf("resetting flags B\n");
                        isTransmittingFrame = isStartPulse = false; // invalidate
                        // resets++;
                    }
                }
            }
        }

        if (dataLevel) // data level high
        {
            if (!dataState) // data rising edge
            {
                // dataTickCounter++;
                dataState = true;

                if (!isTransmittingFrame && isStartPulse && clkLevel)
                {
                    // pc.printf("setting transmit flag to true\n");
                    isStartPulse = false;
                    isTransmittingFrame = true;
                    index = FRAME_LENGTH;
                    buffer = SIGNAL_BIT;
                    // counter = 0; // FIXME: is this enough?
                }
            }
        }
        else // data level low
        {
            if (dataState) // data falling edge
            {
                dataState = false;

                if (!isTransmittingFrame && clkLevel)
                {
                    // pc.printf("setting start pulse to true\n");
                    isStartPulse = true;
                }
            }
        }
    }
}
