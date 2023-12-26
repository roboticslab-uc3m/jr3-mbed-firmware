#ifndef __JR3_CONTROLLER_HPP__
#define __JR3_CONTROLLER_HPP__

#include "mbed.h"
#include "chrono"
#include "AccurateWaiter.h"
#include "utils.hpp"

class Jr3Controller
{
public:
    enum jr3_state
    { UNINITIALIZED, READY };

    Jr3Controller(mbed::Callback<uint32_t()> cb);
    void initialize();
    void startSync();
    void startAsync(mbed::Callback<void(uint16_t *)> cb, uint32_t periodUs);
    void stop();
    void calibrate();
    void setFilter(uint16_t cutOffFrequency);
    void getFullScales(uint16_t * data) const;
    bool acquire(uint16_t * data) const;
    jr3_state getState() const;

private:
    enum jr3_channel : uint8_t
    {
        VOLTAGE = 0,
        FORCE_X, FORCE_Y, FORCE_Z,
        MOMENT_X, MOMENT_Y, MOMENT_Z,
        CALIBRATION
    };

    void startSensorThread();
    void startAsyncThread();
    void stopSensorThread();
    void stopAsyncThread();
    void acquireInternal(uint16_t * data) const;
    void doSensorWork();
    void doAsyncWork();

    rtos::Thread * sensorThread {nullptr};
    rtos::Thread * asyncThread {nullptr};
    mutable rtos::Mutex mutex;
    mbed::Callback<uint32_t()> readerCallback;
    mbed::Callback<void(uint16_t *)> asyncCallback;
    AccurateWaiter waiter;
    jr3_state state {UNINITIALIZED};

    fixed_t calibrationCoeffs[36] {}; // value initialization to zero
    fixed_t shared[6] {}; // value initialization to zero
    uint16_t fullScales[6] {}; // value initialization to zero
    std::chrono::microseconds asyncPeriodUs {0us};

    bool sensorStopRequested {false};
    bool asyncStopRequested {false};
    bool zeroOffsets {false};

    fixed_t smoothingFactor {1.0f}; // default: unfiltered

    static constexpr float samplingPeriod = 128.5e-6f; // [s]
};

#endif // __JR3_CONTROLLER_HPP__
