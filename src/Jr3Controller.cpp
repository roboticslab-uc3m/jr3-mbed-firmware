#include "Jr3Controller.hpp"
#include "utils.hpp"

#define DBG 0

#define CHECK_STATE() do { if (state != READY) { printf("not in ready state\n"); return; } } while (0);

constexpr float M_PI = 3.14159265358979323846f;

#if DBG
namespace
{
    constexpr int STORAGE_SIZE = 500;
    int storage[STORAGE_SIZE];
    int storageIndex = 0;
}
#endif

Jr3Controller::Jr3Controller(mbed::Callback<uint32_t()> cb)
    : readerCallback(cb)
{}

void Jr3Controller::startSync(uint16_t cutOffFrequency)
{
    CHECK_STATE();
    stopAsyncThread();
    setFilter(cutOffFrequency);
    startSensorThread();
}

void Jr3Controller::startAsync(mbed::Callback<void(uint16_t *)> cb, uint16_t cutOffFrequency, uint32_t periodUs)
{
    CHECK_STATE();

    if (!asyncCallback || asyncCallback != cb)
    {
        // to replace an existing callback, we need to shutdown both threads first
        stop();
        asyncCallback = cb;
    }

    printf("using a period of %d us\n", periodUs);

    mutex.lock();
    asyncPeriodUs = std::chrono::microseconds(periodUs);
    mutex.unlock();

    setFilter(cutOffFrequency);
    startSensorThread();
    startAsyncThread();
}

void Jr3Controller::startSensorThread()
{
    if (!sensorThread)
    {
        mutex.lock();
        sensorStopRequested = false;
        mutex.unlock();

        sensorThread = new rtos::Thread(osPriorityNormal);
        sensorThread->start({this, &Jr3Controller::doSensorWork});
    }
}

void Jr3Controller::startAsyncThread()
{
    if (!asyncThread)
    {
        mutex.lock();
        asyncStopRequested = false;
        mutex.unlock();

        // increased priority, see AccurateWaiter::wait_for
        asyncThread = new rtos::Thread(osPriorityAboveNormal);
        asyncThread->start({this, &Jr3Controller::doAsyncWork});
    }
}

void Jr3Controller::stop()
{
    CHECK_STATE();
    stopAsyncThread();
    stopSensorThread();
}

void Jr3Controller::stopSensorThread()
{
    if (sensorThread)
    {
        mutex.lock();
        sensorStopRequested = true;
        mutex.unlock();

        sensorThread->join();
        delete sensorThread;
        sensorThread = nullptr;

        mutex.lock();
        memset(shared, 0, sizeof(shared));
        frameCounter = 0;
        mutex.unlock();
    }
}

void Jr3Controller::stopAsyncThread()
{
    if (asyncThread)
    {
        mutex.lock();
        asyncStopRequested = true;
        mutex.unlock();

        asyncThread->join();
        delete asyncThread;
        asyncThread = nullptr;
    }
}

void Jr3Controller::calibrate()
{
    CHECK_STATE();

    mutex.lock();
    zeroOffsets = true;
    mutex.unlock();
}

void Jr3Controller::setFilter(uint16_t cutOffFrequency)
{
    CHECK_STATE();

    // the input cutoff frequency is expressed in [0.01*Hz]
    printf("setting new cutoff frequency: %.1f Hz\n", cutOffFrequency * 0.01f);

    mutex.lock();

    if (cutOffFrequency != 0)
    {
        // https://w.wiki/7Er6
        smoothingFactor = samplingPeriod / (samplingPeriod + 1.0f / (2.0f * M_PI * cutOffFrequency * 0.01f));
    }
    else
    {
        smoothingFactor = 1.0f; // unfiltered
    }

    mutex.unlock();

    printf("smoothing factor: %0.6f\n", static_cast<float>(smoothingFactor));
}

void Jr3Controller::getFullScales(uint16_t * data) const
{
    CHECK_STATE();
    memcpy(data, fullScales, sizeof(fullScales));
}

bool Jr3Controller::acquire(uint16_t * data) const
{
    if (state == READY && sensorThread)
    {
        acquireInternal(data);
        return true;
    }

    return false;
}

Jr3Controller::jr3_state Jr3Controller::getState() const
{
    return state;
}

void Jr3Controller::initialize()
{
    // in case a re-initialization was requested
    stopAsyncThread();
    stopSensorThread();

    state = UNINITIALIZED;

    uint8_t calibration[256];
    uint8_t calibrationIndex = 0;
    int calibrationCounter = 0;

    while (calibrationCounter < 256)
    {
        uint32_t frame = readerCallback();

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

            memcpy(&mantissa, calibration + 10 + (i * 20) + (j * 3), sizeof(uint16_t));
            memcpy(&exponent, calibration + 12 + (i * 20) + (j * 3), sizeof(int8_t));

            calibrationCoeffs[(i * 6) + j] = jr3ToFixedPoint(mantissa, exponent);
        }

        printf("%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n",
               static_cast<float>(calibrationCoeffs[(i * 6)]),
               static_cast<float>(calibrationCoeffs[(i * 6) + 1]),
               static_cast<float>(calibrationCoeffs[(i * 6) + 2]),
               static_cast<float>(calibrationCoeffs[(i * 6) + 3]),
               static_cast<float>(calibrationCoeffs[(i * 6) + 4]),
               static_cast<float>(calibrationCoeffs[(i * 6) + 5]));
    }

    printf("\nfull scales:\n\n");

    for (int i = 0; i < 6; i++)
    {
        uint16_t fullScale;
        memcpy(&fullScale, calibration + 28 + (i * 20), sizeof(uint16_t));
        fullScales[i] = fullScale;
        printf("%d ", fullScale);
    }

    printf("\n\ninitialization done\n\n");

    state = READY;
}

void Jr3Controller::acquireInternal(uint16_t * data) const
{
    fixed_t temp[6];

    mutex.lock();
    memcpy(temp, shared, sizeof(shared));
    data[6] = frameCounter;
    mutex.unlock();

    for (int i = 0; i < 6; i++)
    {
        data[i] = jr3FromFixedPoint(temp[i]);
    }
}

void Jr3Controller::doSensorWork()
{
    printf("starting sensor thread\n");

    uint32_t frame;
    uint8_t address;

    fixed_t raw[6], offset[6], decoupled[6], filtered[6], normalized[6];

    memset(raw, 0, sizeof(raw));
    memset(offset, 0, sizeof(offset));
    memset(decoupled, 0, sizeof(decoupled));
    memset(filtered, 0, sizeof(filtered));
    memset(normalized, 0, sizeof(normalized));

    mutex.lock();
    fixed_t localSmoothingFactor = smoothingFactor;
    bool localStopRequested = sensorStopRequested;
    mutex.unlock();

    jr3_channel expectedChannel = FORCE_X;

    while (!localStopRequested)
    {
        frame = readerCallback();
        address = (frame & 0x000F0000) >> 16;

#if DBG
        storage[storageIndex++] = frame;

        if (storageIndex == STORAGE_SIZE)
        {
            for (int i = 0; i < STORAGE_SIZE; i++)
            {
                printf("[%03d] [%d] 0x%04X\n", i + 1, (storage[i] & 0x000F0000) >> 16, storage[i] & 0x0000FFFF);
            }

            storageIndex = 0;
        }
#endif

        if (address != expectedChannel) // in case any channel is skipped
        {
            expectedChannel = FORCE_X;
            continue;
        }

        raw[address - 1] = jr3ToFixedPoint(frame & 0x0000FFFF);

        if (address != MOMENT_Z)
        {
            expectedChannel = static_cast<jr3_channel>(expectedChannel + 1);
            continue; // keep reading frames until we get all six axis values
        }

        for (int i = 0; i < 6; i++)
        {
            decoupled[i] = fixedpoint::multiply_accumulate(6, calibrationCoeffs + (i * 6), raw);

            // first-order low-pass IIR filter (as an exponential moving average)
            filtered[i] += localSmoothingFactor * (decoupled[i] - filtered[i]);

            normalized[i] = filtered[i] - offset[i];
        }

        mutex.lock();

        if (zeroOffsets)
        {
            memcpy(offset, filtered, sizeof(filtered));
            memset(shared, 0, sizeof(shared));
            zeroOffsets = false;
        }
        else
        {
            memcpy(shared, normalized, sizeof(normalized));
        }

        localSmoothingFactor = smoothingFactor;
        localStopRequested = sensorStopRequested;
        frameCounter++;
        mutex.unlock();

        expectedChannel = FORCE_X;
    }

    printf("quitting sensor thread\n");
}

void Jr3Controller::doAsyncWork()
{
    printf("starting async thread\n");

    uint16_t data[7]; // fx, fy, fz, mx, my, mz, frame counter

    mutex.lock();
    bool localStopRequested = asyncStopRequested;
    std::chrono::microseconds localAsyncPeriodUs = asyncPeriodUs;
    mutex.unlock();

    while (!localStopRequested)
    {
        acquireInternal(data);
        asyncCallback(data);
        waiter.wait_for(localAsyncPeriodUs);

        mutex.lock();
        localStopRequested = asyncStopRequested;
        localAsyncPeriodUs = asyncPeriodUs;
        mutex.unlock();
    }

    printf("quitting async thread\n");
}
