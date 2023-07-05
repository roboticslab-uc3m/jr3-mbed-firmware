#include "mbed.h"

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

    PortIn port(Port0, 0x00000003);

    const unsigned int STORAGE_SIZE = 10;
    const unsigned int FRAME_LENGTH = 20;
    const unsigned int CLOCK_TIMEOUT = 7;
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
    uint32_t storage[STORAGE_SIZE];

    bool clkLevel, dataLevel;

    int pins = 0;
    int storageIndex = 0;
    int idleCounter = 0;
    // int resets = 0;

    const unsigned int BLARG = 100;
    int idleStorage[BLARG];
    int idleStorage2[BLARG];
    int idleCtr2 = 0;
    int idleCtr3 = 0;

    // bool wasFirstRead = false;

    while (true)
    {
        pins = port.read();
        // dataLevel = pins & 0x00000002;
        clkLevel = pins & 0x00000001;

        if (clkLevel && clockState)
        {
            idleCtr2++;
        }
        else
        {
            idleStorage[idleCtr3++] = idleCtr2;
            idleStorage2[idleCtr3 - 1] = clkLevel;
            idleCtr2 = 0;

            if (idleCtr3 == BLARG)
            {
                for (int i = 0; i < BLARG; i++)
                {
                    pc.printf("[%03d] %d %d\n", i, idleStorage[i], idleStorage2[i]);
                }

                idleCtr3 = 0;
            }

            clockState = clkLevel;
        }
    }
}
