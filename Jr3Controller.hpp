#ifndef __JR3_CONTROLLER_HPP__
#define __JR3_CONTROLLER_HPP__

#include "mbed.h"
#include "AccurateWaiter.h"

class Jr3Controller
{
public:
    Jr3Controller(Callback<uint32_t()> cb);
    void startSync();
    void startAsync(Callback<void(uint16_t *)> cb, uint32_t delayUs);
    void stop();
    void calibrate();
    void setFilter(uint16_t cutOffFrequency);
    bool acquire(uint16_t * data);

private:
    enum jr3_channel : uint8_t
    {
        VOLTAGE = 0,
        FORCE_X, FORCE_Y, FORCE_Z,
        MOMENT_X, MOMENT_Y, MOMENT_Z,
        CALIBRATION
    };

    void startSensorThread();
    void initialize();
    void acquireInternal(uint16_t * data);
    void doSensorWork();
    void doAsyncWork();

    Thread * sensorThread {nullptr};
    Thread * asyncThread {nullptr};
    Mutex mutex;
    Callback<uint32_t()> readerCallback;
    Callback<void(uint16_t *)> asyncCallback;
    AccurateWaiter waiter;

    float calibrationCoeffs[36];
    float shared[6];
    std::chrono::microseconds asyncDelayUs {0us};

    bool stopRequested {false};
    bool zeroOffsets {false};

    float smoothingFactor {1.0f}; // unfiltered

    static constexpr double samplingPeriod = 15.625e-6; // [s], TODO: verify this
};

#endif // __JR3_CONTROLLER_HPP__
