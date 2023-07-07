#include "mbed.h"
#include "FastIO.h"
#include <cstring>

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

inline int getDiff(int val1, int val2)
{
    int diff = val2 - val1;

    if (diff <= 0)
    {
        return val2 + 8 - val1;
    }
    else
    {
        return diff;
    }
}

int main()
{
    wait(5);

    // overclock CPU from 96 MHz to 128 MHz
    // setSystemFrequency(3, 16, 1);

    Serial pc(USBTX, USBRX); // tx, rx
    pc.printf("System clock = %d\n", SystemCoreClock);

    wait(5);

    pc.printf("ready\n");

    // PortIn port(Port0, 0x00000003);
    FastIn<p9> clock;
    FastIn<p10> data;

    bool clkLevel, dataLevel;
    bool clockState = true;

    const unsigned int BLARG = 500;
    int idleStorage[BLARG];
    int idleStorage2[BLARG];
    int idleCtr2 = 1;
    int idleCtr3 = 0;

    const static int BLORG = 1000;
    bool storage[BLORG];
    bool storage2[BLORG];
    int i = 0;
    int idle = 0;
    const unsigned int FRAMES = 500;
    uint32_t frames[FRAMES];
    int frame_n = 0;

    bool lastBit = true;
    int index = 20;
    uint32_t frame = 0;

    int j;

    int pins;

    CAN can(p30, p29); // rd, td
    can.frequency(1000000);
    can.reset();

    CANMessage msg;
    int refId1 = 0;
    int refId2 = 0;
    msg.id = 0x01;

    uint64_t msg_64 = 0;

    // bool wasFirstRead = false;

    while (true)
    {
        // pins = port.read();
        // storage[i] = pins & 0x00000001;
        // storage2[i] = pins & 0x00000002;

        storage[i] = clkLevel = clock.read();
        storage2[i] = data.read();

        i++;

        if (clkLevel)
        {
            idle++;
        }
        else
        {
            idle = 0;
        }

        if (idle == 10)
        {
            if (i > 45)
            {
                lastBit = true;
                index = 20;
                frame = 0;

                for (j = 0; j < i - 9; j++)
                {
                    if (storage[j] != lastBit)
                    {
                        lastBit = storage[j];

                        if (lastBit)
                        {
                            frame |= (storage2[j] ? 1 : 0) << --index;

                            if (index < 0)
                            {
                                break;
                            }
                        }
                    }
                }

                if (index == 0)
                {
                    frames[frame_n++] = frame;
                    // pc.printf("0x%08x\n", frame);

                    // for (j = 0; j < i - 9; j++)
                    // {
                    //     pc.printf("%d", storage2[j]);
                    // }

                    // pc.printf("\n");
                }

                if (frame_n == 4)
                {
                    // CANMessage msg;
                    // msg.id = 0x01;
                    msg_64 = 0;
                    // msg_64 |= static_cast<uint64_t>(frames[0] & 0x0005FFFF);
                    // msg_64 |= static_cast<uint64_t>(frames[1] & 0x0005FFFF) << 20;
                    // msg_64 |= static_cast<uint64_t>(frames[2] & 0x0005FFFF) << 40;
                    msg_64 |= static_cast<uint64_t>(frames[0] & 0x0000FFFF);
                    msg_64 |= static_cast<uint64_t>(frames[1] & 0x0000FFFF) << 16;
                    msg_64 |= static_cast<uint64_t>(frames[2] & 0x0000FFFF) << 32;
                    msg_64 |= static_cast<uint64_t>(frames[3] & 0x0000FFFF) << 48;
                    memcpy(msg.data, &msg_64, sizeof(msg_64));

                    msg.id = refId1 = (frames[0] & 0x00070000) >> 16;
                    refId2 = ((frames[1] & 0x00070000) >> 16);
                    msg.id |= (getDiff(refId1, refId2) & 0x00000003) << 3;
                    refId1 = refId2;
                    refId2 = ((frames[2] & 0x00070000) >> 16);
                    msg.id |= (getDiff(refId1, refId2) & 0x00000003) << 5;
                    refId1 = refId2;
                    refId2 = ((frames[3] & 0x00070000) >> 16);
                    msg.id |= (getDiff(refId1, refId2) & 0x00000003) << 7;

                    can.write(msg);
                    frame_n = 0;
                }
            }

            i = idle = 0;
        }

        // if (idle == 10)
        // {
        //     pc.printf("[CLCK] ");

        //     for (int j = 0; j < i; j++)
        //     {
        //         pc.printf("%d", storage[j]);
        //     }

        //     pc.printf("\n[DATA] ");

        //     for (int j = 0; j < i; j++)
        //     {
        //         pc.printf("%d", storage2[j]);
        //     }

        //     pc.printf("\n");

        //     i = idle = 0;
        // }

        // if (i == BLORG)
        // {
        //     pc.printf("[CLCK] ");

        //     for (int j = 0; j < BLORG; j++)
        //     {
        //         pc.printf("%d", storage[j]);
        //     }

        //     pc.printf("\n[DATA] ");

        //     for (int j = 0; j < BLORG; j++)
        //     {
        //         pc.printf("%d", storage2[j]);
        //     }

        //     pc.printf("\n");

        //     i = idle = 0;
        // }

        // if (clkLevel && clockState)
        // {
        //     idleCtr2++;
        // }
        // else
        // {
        //     idleStorage[idleCtr3++] = idleCtr2;
        //     idleStorage2[idleCtr3 - 1] = clkLevel;
        //     idleCtr2 = 1;

        //     if (idleCtr3 == BLARG)
        //     {
        //         for (int i = 0; i < BLARG; i++)
        //         {
        //             pc.printf("[%03d] %d %d\n", i, idleStorage[i], idleStorage2[i]);
        //         }

        //         idleCtr3 = 0;
        //     }

        //     clockState = clkLevel;
        // }
    }
}
