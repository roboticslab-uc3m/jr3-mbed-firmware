#ifndef __JR3_CONTROLLER_HPP__
#define __JR3_CONTROLLER_HPP__

#include "mbed.h"
#include "utils.hpp"
#include "AccurateWaiter.h"

#define DBG 0

constexpr double M_PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;

template <typename ReaderT>
class Jr3Controller
{
public:
    void startSync()
    {
        startSensorThread();
    }

    void startAsync(Callback<void(uint16_t *)> cb, float delay)
    {
        if (!asyncCallback || asyncCallback != cb)
        {
            // to replace an existing callback, we need to shutdown both threads first
            stop();
            asyncCallback = cb;
        }

        mutex.lock();
        asyncDelay = delay;
        mutex.unlock();

        startSensorThread();

        if (!asyncThreadRunning)
        {
            asyncThread.start({this, &Jr3Controller::doAsyncWork});
            asyncThreadRunning = true;
        }
    }

    void stop()
    {
        if (asyncThreadRunning)
        {
            mutex.lock();
            stopRequested = true;
            mutex.unlock();

            asyncThread.join();
            asyncThreadRunning = false;
        }

        if (sensorThreadRunning)
        {
            mutex.lock();
            stopRequested = true;
            mutex.unlock();

            sensorThread.join();
            sensorThreadRunning = false;
        }
    }

    void calibrate()
    {
        mutex.lock();
        zeroOffsets = true;
        mutex.unlock();
    }

    void setFilter(float cutOffFrequency)
    {
        mutex.lock();

        if (cutOffFrequency > 0.0f)
        {
            // https://w.wiki/7Er6
            smoothingFactor = samplingPeriod / (samplingPeriod + 1.0 / (2.0 * M_PI * cutOffFrequency));
        }
        else
        {
            smoothingFactor = 1.0f; // unfiltered
        }

        mutex.unlock();
    }

    bool acquire(uint16_t * data)
    {
        if (sensorThreadRunning)
        {
            acquireInternal(data);
            return true;
        }

        return false;
    }

private:
    enum jr3_channel : uint8_t
    {
        VOLTAGE = 0,
        FORCE_X, FORCE_Y, FORCE_Z,
        MOMENT_X, MOMENT_Y, MOMENT_Z,
        CALIBRATION
    };

    void startSensorThread()
    {
        calibrate();

        if (!sensorThreadRunning)
        {
            initialize();
            sensorThreadRunning = true;
            stopRequested = false;
            sensorThread.start({this, &Jr3Controller::doSensorWork});
        }
    }

    void initialize();
    void acquireInternal(uint16_t * data);
    void doSensorWork();
    void doAsyncWork();

    ReaderT jr3;
    Thread sensorThread;
    Thread asyncThread;
    Mutex mutex;
    Callback<void(uint16_t *)> asyncCallback;
    AccurateWaiter waiter;

    float calibrationCoeffs[36];
    float shared[6];
    float asyncDelay {0.0f}; // TODO

    bool sensorThreadRunning {false};
    bool asyncThreadRunning {false};
    bool stopRequested {false};
    bool zeroOffsets {false};

    float smoothingFactor {1.0f}; // unfiltered

    static constexpr double samplingPeriod = 15.625e-6; // [s], TODO: verify this
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

            calibrationCoeffs[(i * 6) + j] = jr3FloatToIEEE754(exponent, mantissa);
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
inline void Jr3Controller<ReaderT>::acquireInternal(uint16_t * data)
{
    float temp[6];

    mutex.lock();
    memcpy(temp, shared, sizeof(shared));
    mutex.unlock();

    for (int i = 0; i < 6; i++)
    {
        data[i] = jr3FixedFromIEEE754(temp[i]);
    }
}

template <typename ReaderT>
inline void Jr3Controller<ReaderT>::doSensorWork()
{
    uint32_t frame;
    uint8_t address;

    float raw[6], offset[6], decoupled[6], filtered[6];

    memset(raw, 0, sizeof(raw));
    memset(offset, 0, sizeof(offset));
    memset(decoupled, 0, sizeof(decoupled));
    memset(filtered, 0, sizeof(filtered));

    mutex.lock();
    float localSmoothingFactor = smoothingFactor;
    bool localStopRequested = stopRequested;
    mutex.unlock();

#if DBG
    constexpr int STORAGE_SIZE = 500;
    int storage[STORAGE_SIZE];
    int storageIndex = 0;
#endif

    jr3_channel expectedChannel = FORCE_X;

    while (!localStopRequested)
    {
        frame = jr3.readFrame();
        address = (frame & 0x000F0000) >> 16;

#if DBG
        storage[storageIndex++] = frame;
#else
        if (address != expectedChannel) // in case any channel is skipped
        {
            expectedChannel = FORCE_X;
            continue;
        }
#endif

#if DBG
        if (address >= FORCE_X && address <= MOMENT_Z)
#endif
        {
            raw[address - 1] = jr3FixedToIEEE754(frame & 0x0000FFFF);
        }

        if (address != MOMENT_Z)
        {
            expectedChannel = static_cast<jr3_channel>(expectedChannel + 1);
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
            filtered[i] += localSmoothingFactor * (decoupled[i] - offset[i] - filtered[i]);
        }

        mutex.lock();

        if (zeroOffsets)
        {
            memcpy(offset, decoupled, sizeof(decoupled));
            zeroOffsets = false;
        }

        memcpy(shared, filtered, sizeof(filtered));
        localSmoothingFactor = smoothingFactor;
        localStopRequested = stopRequested;
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

        expectedChannel = FORCE_X;
    }
}

template <typename ReaderT>
inline void Jr3Controller<ReaderT>::doAsyncWork()
{
    uint16_t data[6];

    mutex.lock();
    bool localStopRequested = stopRequested;
    float localAsyncDelay = asyncDelay;
    mutex.unlock();

    while (!localStopRequested)
    {
        acquireInternal(data);
        asyncCallback(data);
        waiter.wait_for(localAsyncDelay);

        mutex.lock();
        localStopRequested = stopRequested;
        localAsyncDelay = asyncDelay;
        mutex.unlock();
    }
}

#endif // __JR3_CONTROLLER_HPP__
