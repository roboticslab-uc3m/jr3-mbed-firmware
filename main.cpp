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

float convertToFloat(uint16_t mantissa)
{
    float value = 0.0f;

    for (int i = 0; i < 16; i++)
    {
        value += ((mantissa & (1U << (15 - i))) >> (15 - i)) * powf(2, -i);

        if (i == 0)
        {
            // in signed Q1.15 format, the leftmost bit determines the sign
            value = -value;
        }
    }

    return value;
}

inline float convertToFloat(int8_t exponent, uint16_t mantissa)
{
    return convertToFloat(mantissa) * powf(2, exponent);
}

uint16_t convertToInteger(float value)
{
    int16_t result = 0;
    float integer = 0;
    float absolute = fabsf(value);

    for (int i = 0; i < 15; i++)
    {
        modff((absolute * powf(2, 15 - i)), &integer);
        result |= (int)integer << i;
    }

    return value < 0.0f ? result * -1 : result;
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

    ThisThread::sleep_for(5s);

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
                            uint32_t coefficient = 0;
                            memcpy(&coefficient, calibration + 10 + (i * 20) + (j * 3), 3);
                            calibrationCoeffs[(i * 6) + j] = convertToFloat(coefficient >> 16, coefficient & 0x0000FFFF);
                        }

                        printf("%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n",
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
                        int16_t fullScales = 0;
                        memcpy(&fullScales, calibration + 28 + (i * 20), 2);
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
