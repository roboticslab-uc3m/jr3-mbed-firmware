#ifndef __JR3_CONTROLLER_HPP__
#define __JR3_CONTROLLER_HPP__

#include "mbed.h"
#include "atomic_bool.h"
#include "utils.hpp"

#define DBG 0

constexpr float M_PI = 3.14159265358979323846;

template <typename ReaderT>
class Jr3Controller
{
public:
    void start()
    {
        if (threadRunning)
        {
            // TODO: set filter
        }
        else
        {
            initialize();
            threadRunning = true;
            thread.start(callback(this, &Jr3Controller::worker));
        }
    }

    void stop()
    {
        if (threadRunning)
        {
            threadRunning = false;
            thread.join();
        }
    }

    void calibrate()
    {
        if (threadRunning)
        {
            mutex.lock();
            zeroOffsets = true;
            mutex.unlock();
        }
    }

    bool acquire(uint16_t * data)
    {
        if (threadRunning)
        {
            float temp[6];

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
                data[i] = fixedFromIEEE754(temp[i]);
            }

            return true;
        }

        return false;
    }

    void initialize();
    void worker();

private:
    enum jr3_channel : uint8_t
    {
        VOLTAGE = 0,
        FORCE_X, FORCE_Y, FORCE_Z,
        MOMENT_X, MOMENT_Y, MOMENT_Z,
        CALIBRATION
    };

    ReaderT jr3;
    Thread thread;
    Mutex mutex;
    ConditionVariable cv {mutex};

    float calibrationCoeffs[36];
    float shared[6];

    AtomicBool threadRunning {false};
    bool zeroOffsets {false};
    bool dataReady {false};

    static constexpr double samplingPeriod = 15.625e-6; // [s], TODO: verify this
    static constexpr double cutOffFrequency = 500; // [Hz]

    // https://w.wiki/7Er6
    const float smoothingFactor = samplingPeriod / (samplingPeriod + 1.0 / (2.0 * M_PI * cutOffFrequency));
};

template <typename ReaderT>
inline void Jr3Controller<ReaderT>::initialize()
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

template <typename ReaderT>
inline void Jr3Controller<ReaderT>::worker()
{
    uint32_t frame;
    uint8_t address;

    float raw[6], offset[6], decoupled[6], filtered[6];

    memset(raw, 0, sizeof(raw));
    memset(offset, 0, sizeof(offset));
    memset(decoupled, 0, sizeof(decoupled));
    memset(filtered, 0, sizeof(filtered));

#if DBG
    constexpr int STORAGE_SIZE = 500;
    int storage[STORAGE_SIZE];
    int storageIndex = 0;
#endif

    // block until the first processed frame in the next loop is FORCE_X
    while ((jr3.readFrame() & 0x000F0000) >> 16 != VOLTAGE) {}

    while (threadRunning)
    {
        frame = jr3.readFrame();
        address = (frame & 0x000F0000) >> 16;

#if DBG
        storage[storageIndex++] = frame;
#endif

        if (address >= FORCE_X && address <= MOMENT_Z)
        {
            raw[address - 1] = fixedToIEEE754(frame & 0x0000FFFF);
        }

        if (address != MOMENT_Z)
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

        mutex.lock();

        if (zeroOffsets)
        {
            memcpy(offset, decoupled, sizeof(decoupled));
            zeroOffsets = false;
        }

        dataReady = true;
        cv.notify_all();
        memcpy(shared, filtered, sizeof(filtered));
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

#endif // __JR3_CONTROLLER_HPP__
