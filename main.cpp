#include "mbed.h"
#include "Jr3.hpp"

#define CLOCK_PIN p9
#define DATA_PIN p10

int main()
{
    // overclock CPU from 96 MHz to 128 MHz
    // setSystemFrequency(3, 16, 1);

    Jr3<Port0, CLOCK_PIN, DATA_PIN> jr3;

    const int STORAGE_SIZE = 500;
    int storage[STORAGE_SIZE];
    int storageIndex = 0;

    wait(5);

    printf("System clock = %d\n", SystemCoreClock);

    wait(5);

    printf("ready\n");

    while (true)
    {
        storage[storageIndex++] = jr3.readFrame();

        if (storageIndex == STORAGE_SIZE)
        {
            for (int i = 0; i < STORAGE_SIZE; i++)
            {
                printf("[%03d] [%d] 0x%04X\n", i + 1, (storage[i] & 0x000F0000) >> 16, storage[i] & 0x0000FFFF);
            }

            storageIndex = 0;
        }
    }
}
