#include "mbed.h"
#include "Jr3.hpp"

#define JR3_PORT Port0
#define CLOCK_PIN p9
#define DATA_PIN p10

#define CAN_RD_PIN p30
#define CAN_TD_PIN p29
#define CAN_BAUDRATE 1000000
#define CAN_ID 0x01

#define M_PI 3.14159265358979323846

#define DBG 0

enum jr3_channel : uint8_t {
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

Jr3<JR3_PORT, CLOCK_PIN, DATA_PIN> jr3;

float calibrationCoeffs[36];
float shared[6];

bool dataReady = false;

const double samplingPeriod = 15.625e-6; // [s], TODO: verify this
const double cutOffFrequency = 500; // [Hz]

// https://w.wiki/7Er6
const float smoothingFactor = samplingPeriod / (samplingPeriod + 1.0 / (2.0 * M_PI * cutOffFrequency));

void initialize()
{
    uint8_t calibration[256];
    uint8_t calibrationIndex = 0;
    int calibrationCounter = 0;

    while (calibrationCounter < 256)
    {
        uint32_t frame = jr3.readFrame();

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
        }
    }

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
            uint16_t mantissa;
            int8_t exponent;

            memcpy(&mantissa, calibration + 10 + (i * 20) + (j * 3), 2);
            memcpy(&exponent, calibration + 12 + (i * 20) + (j * 3), 1);

            calibrationCoeffs[(i * 6) + j] = fixedToIEEE754(exponent, mantissa);
        }

        printf("%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n",
                calibrationCoeffs[(i * 6)],
                calibrationCoeffs[(i * 6) + 1],
                calibrationCoeffs[(i * 6) + 2],
                calibrationCoeffs[(i * 6) + 3],
                calibrationCoeffs[(i * 6) + 4],
                calibrationCoeffs[(i * 6) + 5]);
    }

    printf("\nfull scales:\n\n");

    for (int i = 0; i < 6; i++)
    {
        uint16_t fullScales;
        memcpy(&fullScales, calibration + 28 + (i * 20), 2);
        printf("%d\n", fullScales);
    }

    printf("\ncalibration done\n\n");
}

void worker()
{
    uint32_t frame;
    uint8_t address;

    float raw[6], offset[6], decoupled[6], filtered[6];

    memset(raw, 0, sizeof(raw));
    memset(offset, 0, sizeof(offset));
    memset(decoupled, 0, sizeof(decoupled));
    memset(filtered, 0, sizeof(filtered));

    bool offsetsAcquired = false;

#if DBG
    const int STORAGE_SIZE = 500;
    int storage[STORAGE_SIZE];
    int storageIndex = 0;
#endif

    // block until the first processed frame in the next loop is FORCE_X
    while ((jr3.readFrame() & 0x000F0000) >> 16 != VOLTAGE) {}

    while (true)
    {
        frame = jr3.readFrame();
        address = (frame & 0x000F0000) >> 16;

#if DBG
        storage[storageIndex++] = frame;
#endif

        if (address >= FORCE_X && address <= TORQUE_Z)
        {
            raw[address - 1] = fixedToIEEE754(frame & 0x0000FFFF);
        }

        if (address != TORQUE_Z)
        {
            continue; // keep reading frames until we get all six axis values
        }

        for (int i = 0; i < 6; i++)
        {
            decoupled[i] = 0.0f;

            for (int j = 0; j < 6; j++)
            {
                decoupled[i] += calibrationCoeffs[(i * 6) + j] * raw[j];
            }

            // first-order low-pass IIR filter (as an exponential moving average)
            filtered[i] += smoothingFactor * (decoupled[i] - offset[i] - filtered[i]);
        }

        if (!offsetsAcquired)
        {
            memcpy(offset, decoupled, sizeof(decoupled));
            offsetsAcquired = true;
        }

        mutex.lock();
        memcpy(shared, filtered, sizeof(filtered));
        dataReady = true;
        cv.notify_all();
        mutex.unlock();

#if DBG
        if (storageIndex == STORAGE_SIZE)
        {
            for (int i = 0; i < STORAGE_SIZE; i++)
            {
                printf("[%03d] [%d] 0x%04X\n", i + 1, (storage[i] & 0x000F0000) >> 16, storage[i] & 0x0000FFFF);
            }

            storageIndex = 0;
        }
#endif
    }
}

int main()
{
    // overclock CPU from 96 MHz to 128 MHz
    // setSystemFrequency(3, 16, 1);

    ThisThread::sleep_for(5s);

    printf("ready\n");

    initialize();

    Thread thread;
    thread.start(callback(worker));

    CAN can(CAN_RD_PIN, CAN_TD_PIN, CAN_BAUDRATE);
    can.reset();

    CANMessage msg;
    msg.len = 6;

    float temp[6];
    uint16_t fixed[6];

    while (true)
    {
        mutex.lock();

        if (!dataReady)
        {
            cv.wait();
        }

        memcpy(temp, shared, sizeof(shared));
        dataReady = false;
        mutex.unlock();

        for (int i = 0; i < 6; i++)
        {
            fixed[i] = fixedFromIEEE754(temp[i]);
        }

        memcpy(msg.data, fixed, 6); // fx, fy, fz
        msg.id = 0x180 + CAN_ID;
        can.write(msg);

        // wait_ns(100);

        memcpy(msg.data, fixed + 3, 6); // mx, my, mz
        msg.id = 0x280 + CAN_ID;
        can.write(msg);

        wait_us(1);
    }
}
