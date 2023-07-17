#include "mbed.h"
#include "Jr3.hpp"

#define CLOCK_PIN p9
#define DATA_PIN p10

enum jr3_channel {
    VOLTAGE = 0,
    FORCE_X,
    FORCE_Y,
    FORCE_Z,
    TORQUE_X,
    TORQUE_Y,
    TORQUE_Z,
    CALIBRATION
};

float parseMantissa(uint16_t mantissa)
{
    float value = 0.0f;

    for (int i = 0; i < 16; i++)
    {
        value += ((mantissa & (1U << (15 - i))) >> (15 - i)) * (2 ^ -i);

        if (i == 0)
        {
            // in signed Q1.15 format, the leftmost bit determines the sign
            value = -value;
        }
    }

    return value;
}

inline float parseCoefficient(int8_t exponent, uint16_t mantissa)
{
    return parseMantissa(mantissa) * (2 ^ exponent);
}

int main()
{
    // overclock CPU from 96 MHz to 128 MHz
    // setSystemFrequency(3, 16, 1);

    Jr3<Port0, CLOCK_PIN, DATA_PIN> jr3;

    const int STORAGE_SIZE = 500;
    int storage[STORAGE_SIZE];
    int storageIndex = 0;

    uint8_t calibration[256];
    uint8_t calibrationIndex = 0;
    int calibrationCounter = 0;
    bool calibrationDone = false;

    float calibrationCoeffs[36];

    wait(5);

    printf("System clock = %d\n", SystemCoreClock);

    wait(5);

    printf("ready\n");

    uint32_t frame;

    while (true)
    {
        frame = jr3.readFrame();

        if (!calibrationDone)
        {
            if ((frame & 0x000F0000) >> 16 == CALIBRATION)
            {
                uint8_t address = (frame & 0x0000FF00) >> 8;
                uint8_t value = frame & 0x000000FF;

                if (calibrationCounter == 0 || address == calibrationIndex)
                {
                    calibration[address] = value;
                    calibrationIndex = address + 1;
                    calibrationCounter++;
                }

                if (calibrationCounter == 256)
                {
                    printf("\nEEPROM contents:\n\n");

                    for (int i = 0; i < 256; i += 8)
                    {
                        printf("[%02X] %02X %02X %02X %02X %02X %02X %02X %02X\n",
                               i,
                               calibration[i], calibration[i + 1], calibration[i + 2], calibration[i + 3],
                               calibration[i + 4], calibration[i + 5], calibration[i + 6], calibration[i + 7]);
                    }

                    printf("\ncalibration matrix:\n\n");

                    for (int i = 0; i < 6; i++)
                    {
                        for (int j = 0; j < 6; j++)
                        {
                            uint8_t mantissa_lsb = calibration[10 + (i * 20) + (j * 3)];
                            uint8_t mantissa_msb = calibration[10 + (i * 20) + (j * 3) + 1];
                            int8_t  exponent     = calibration[10 + (i * 20) + (j * 3) + 2];

                            calibrationCoeffs[(i * 6) + j] = parseCoefficient(exponent, (((uint16_t)mantissa_msb) << 8) | mantissa_lsb);
                        }

                        printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n",
                               calibrationCoeffs[i * 6],
                               calibrationCoeffs[i * 6 + 1],
                               calibrationCoeffs[i * 6 + 2],
                               calibrationCoeffs[i * 6 + 3],
                               calibrationCoeffs[i * 6 + 4],
                               calibrationCoeffs[i * 6 + 5]);
                    }

                    printf("\nfull scales:\n\n");

                    for (int i = 0; i < 6; i++)
                    {
                        uint8_t fullScales_lsb = calibration[28 + (i * 20)];
                        uint8_t fullScales_msb = calibration[28 + (i * 20) + 1];
                        int16_t fullScales = (((uint16_t)fullScales_msb) << 8) | fullScales_lsb;

                        printf("%d\n", fullScales);
                    }

                    printf("\ncalibration ready\n\n");

                    calibrationDone = true;
                }
            }
        }
        else
        {
            storage[storageIndex++] = frame;

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
}
