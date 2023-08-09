#include "mbed.h"
#include "Jr3.hpp"

#define CLOCK_PIN p9
#define DATA_PIN p10

#define CAN_RD_PIN p30
#define CAN_TD_PIN p29
#define CAN_BAUDRATE 1000000
#define CAN_ID 0x01

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

float fixedToIEEE754(int8_t exponent, uint16_t mantissa)
{
    uint32_t temp = 0;

    if (mantissa >> 15)
    {
        temp |= (((~mantissa & 0x3FFF) + 1U) << 9) | (1U << 31);
    }
    else
    {
        temp |= (mantissa & 0x3FFF) << 9;
    }

    temp |= (exponent + 126) << 23;

    float f;
    memcpy(&f, &temp, 4);
    return f;
}

inline float fixedToIEEE754(uint16_t mantissa)
{
    // beware of integral promotion! https://stackoverflow.com/a/30474166
    int8_t exponent = __CLZ((mantissa >> 15 ? ~mantissa : mantissa) & 0x0000FFFF) - 17;
    return fixedToIEEE754(-exponent, mantissa << exponent);
}

uint16_t fixedFromIEEE754(float f)
{
    uint32_t temp;
    memcpy(&temp, &f, 4);

    int8_t exponent = ((temp & 0x7F800000) >> 23) - 127;
    uint16_t mantissa = (temp & 0x007FFFFF) >> (8 - exponent);

    if (temp >> 31)
    {
        return ~((mantissa - 1U) | (1U << (15 + exponent)));
    }
    else
    {
        return mantissa | (1U << (15 + exponent));
    }
}

Mutex mutex;
ConditionVariable cv(mutex);

Jr3<Port0, CLOCK_PIN, DATA_PIN> jr3;

const int STORAGE_SIZE = 500;
int storage[STORAGE_SIZE];
int storageIndex = 0;

uint8_t calibration[256];
uint8_t calibrationIndex = 0;
int calibrationCounter = 0;

float calibrationCoeffs[36];

uint32_t frame;

float raw_fx, raw_fy, raw_fz;
float raw_mx, raw_my, raw_mz;

uint16_t decoupled_fx, decoupled_fy, decoupled_fz;
uint16_t decoupled_mx, decoupled_my, decoupled_mz;

uint16_t offset_fx = 0, offset_fy = 0, offset_fz = 0;
uint16_t offset_mx = 0, offset_my = 0, offset_mz = 0;

bool offsetsAcquired = false;
bool dataReady = false;

void worker()
{
    uint16_t temp_fx, temp_fy, temp_fz;
    uint16_t temp_mx, temp_my, temp_mz;

    while (true)
    {
        frame = jr3.readFrame();

        // storage[storageIndex++] = frame;

        switch ((frame & 0x000F0000) >> 16)
        {
        case FORCE_X:
            raw_fx = fixedToIEEE754(frame & 0x0000FFFF);
            break;
        case FORCE_Y:
            raw_fy = fixedToIEEE754(frame & 0x0000FFFF);
            break;
        case FORCE_Z:
            raw_fz = fixedToIEEE754(frame & 0x0000FFFF);
            break;
        case TORQUE_X:
            raw_mx = fixedToIEEE754(frame & 0x0000FFFF);
            break;
        case TORQUE_Y:
            raw_my = fixedToIEEE754(frame & 0x0000FFFF);
            break;
        case TORQUE_Z:
            raw_mz = fixedToIEEE754(frame & 0x0000FFFF);

            temp_fx = fixedFromIEEE754(
                               calibrationCoeffs[0] * raw_fx + calibrationCoeffs[1] * raw_fy + calibrationCoeffs[2] * raw_fz +
                               calibrationCoeffs[3] * raw_mx + calibrationCoeffs[4] * raw_my + calibrationCoeffs[5] * raw_mz -
                               offset_fx
                           );

            temp_fy = fixedFromIEEE754(
                               calibrationCoeffs[6] * raw_fx + calibrationCoeffs[7] * raw_fy + calibrationCoeffs[8] * raw_fz +
                               calibrationCoeffs[9] * raw_mx + calibrationCoeffs[10] * raw_my + calibrationCoeffs[11] * raw_mz -
                               offset_fy
                           );

            temp_fz = fixedFromIEEE754(
                               calibrationCoeffs[12] * raw_fx + calibrationCoeffs[13] * raw_fy + calibrationCoeffs[14] * raw_fz +
                               calibrationCoeffs[15] * raw_mx + calibrationCoeffs[16] * raw_my + calibrationCoeffs[17] * raw_mz -
                               offset_fz
                           );

            temp_mx = fixedFromIEEE754(
                               calibrationCoeffs[18] * raw_fx + calibrationCoeffs[19] * raw_fy + calibrationCoeffs[20] * raw_fz +
                               calibrationCoeffs[21] * raw_mx + calibrationCoeffs[22] * raw_my + calibrationCoeffs[23] * raw_mz -
                               offset_mx
                           );

            temp_my = fixedFromIEEE754(
                               calibrationCoeffs[24] * raw_fx + calibrationCoeffs[25] * raw_fy + calibrationCoeffs[26] * raw_fz +
                               calibrationCoeffs[27] * raw_mx + calibrationCoeffs[28] * raw_my + calibrationCoeffs[29] * raw_mz -
                               offset_my
                           );

            temp_mz = fixedFromIEEE754(
                               calibrationCoeffs[30] * raw_fx + calibrationCoeffs[31] * raw_fy + calibrationCoeffs[32] * raw_fz +
                               calibrationCoeffs[33] * raw_mx + calibrationCoeffs[34] * raw_my + calibrationCoeffs[35] * raw_mz -
                               offset_mz
                           );

            if (!offsetsAcquired)
            {
                offset_fx = temp_fx;
                offset_fy = temp_fy;
                offset_fz = temp_fz;
                offset_mx = temp_mx;
                offset_my = temp_my;
                offset_mz = temp_mz;

                offsetsAcquired = true;
            }

            mutex.lock();

            decoupled_fx = temp_fx;
            decoupled_fy = temp_fy;
            decoupled_fz = temp_fz;
            decoupled_mx = temp_mx;
            decoupled_my = temp_my;
            decoupled_mz = temp_mz;

            dataReady = true;

            cv.notify_all();
            mutex.unlock();

            break;
        }

        // if (storageIndex == STORAGE_SIZE)
        // {
        //     for (int i = 0; i < STORAGE_SIZE; i++)
        //     {
        //         printf("[%03d] [%d] 0x%04X\n", i + 1, (storage[i] & 0x000F0000) >> 16, storage[i] & 0x0000FFFF);
        //     }

        //     storageIndex = 0;
        // }
    }
}

int main()
{
    // overclock CPU from 96 MHz to 128 MHz
    // setSystemFrequency(3, 16, 1);

    ThisThread::sleep_for(5s);

    printf("ready\n");

    while (true)
    {
        frame = jr3.readFrame();

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
                        calibrationCoeffs[(i * 6) + j] = fixedToIEEE754(coefficient >> 16, coefficient & 0x0000FFFF);
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

                printf("\ncalibration done\n\n");
                break;
            }
        }
    }

    Thread thread;
    thread.start(callback(worker));

    CAN can(CAN_RD_PIN, CAN_TD_PIN);
    can.frequency(CAN_BAUDRATE);
    can.reset();

    CANMessage msg;
    msg.len = 6;

    uint64_t msg_64 = 0;

    uint16_t temp_fx, temp_fy, temp_fz;
    uint16_t temp_mx, temp_my, temp_mz;

    while (true)
    {
        mutex.lock();

        while (!dataReady)
        {
            cv.wait();
        }

        temp_fx = decoupled_fx;
        temp_fy = decoupled_fy;
        temp_fz = decoupled_fz;
        temp_mx = decoupled_mx;
        temp_my = decoupled_my;
        temp_mz = decoupled_mz;

        dataReady = false;

        mutex.unlock();

        msg_64 = 0;
        msg_64 |= static_cast<uint64_t>(temp_fx);
        msg_64 |= static_cast<uint64_t>(temp_fy) << 16;
        msg_64 |= static_cast<uint64_t>(temp_fz) << 32;
        memcpy(msg.data, &msg_64, 6);

        msg.id = 0x180 + CAN_ID;
        can.write(msg);

        // printf("fx: %04X fy: %04X fz: %04X\n", temp_fx, temp_fy, temp_fz);

        // wait_ns(100);

        msg_64 = 0;
        msg_64 |= static_cast<uint64_t>(temp_mx);
        msg_64 |= static_cast<uint64_t>(temp_my) << 16;
        msg_64 |= static_cast<uint64_t>(temp_mz) << 32;
        memcpy(msg.data, &msg_64, 6);

        msg.id = 0x280 + CAN_ID;
        can.write(msg);

        // printf("mx: %04X my: %04X mz: %04X\n", temp_mx, temp_my, temp_mz);

        wait_us(1);
    }
}
