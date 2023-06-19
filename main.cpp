#include "mbed.h"

PortIn port(Port0, 0x0000003F);

volatile bool stateClock = true;
volatile bool stateData = true;

volatile int clockTicks = 0;
volatile int dataTicks = 0;

Thread thread;

void worker()
{
    int value;

    while (true)
    {
        value = port.read();

        if ((value & 0x00000001) == 1)
        {
            if (!stateClock)
            {
                clockTicks++;
                stateClock = true;
            }
        }
        else
        {
            if (stateClock)
            {
                stateClock = false;
            }
        }

        if ((value & 0x00000002) == 1)
        {
            if (!stateData)
            {
                dataTicks++;
                stateData = true;
            }
        }
        else
        {
            if (stateData)
            {
                stateData = false;
            }
        }
    }
}

int main()
{
    thread.start(callback(&worker));

    while (true)
    {
        printf("cticks = %d, dticks = %d\n", clockTicks, dataTicks);
        ThisThread::sleep_for(1000);
    }
}
