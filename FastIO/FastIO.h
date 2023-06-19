#ifndef __FAST_IO_H
#define __FAST_IO_H

#include "FastIO_LPC1768.h"
#include "FastIO_LPC11UXX.h"
#include "FastIO_LPC11U6X.h"
#include "FastIO_LPC81X.h"
#include "FastIO_KLXX.h"
#include "FastIO_K20D50M_KPSDK.h"
#include "FastIO_STM32F4.h"
#include "FastIO_STM32L073XX.h"
#include "FastIO_NUCLEO_F030.h"
#include "FastIO_LPC11XX.h"
#include "FastIO_EFM32.h"
#include "FastIO_LPC43XX.h"
#include "FastIO_NRF51822.h"

#ifndef INIT_PIN
#warning Target is not supported by FastIO
#warning Reverting to regular DigitalInOut
#include "FastIO_Unsupported.h"
#endif

#include "mbed.h"

/**
 * Faster alternative compared to regular DigitalInOut
 *
 * Except the constructor it is compatible with regular DigitalInOut.
 * Code is based on Igor Skochinsky's code (http://mbed.org/users/igorsk/code/FastIO/)
 */
template <PinName pin> class FastInOut
{
public:
    /**
     * Construct new FastInOut object
     *
     * @code
     * FastInOut<LED1> led1;
     * @endcode
     *
     * No initialization is done regarding input/output mode,
     * FastIn/FastOut can be used if that is required
     *
     * @param pin pin the FastOut object should be used for
     */
    FastInOut() {
        INIT_PIN;
    }

    ~FastInOut() {
        DESTROY_PIN;
    }

    void write(int value) {
        if ( value )
            WRITE_PIN_SET;
        else
            WRITE_PIN_CLR;
    }
    int read() {
        return READ_PIN;
    }

    void mode(PinMode pull) {
        SET_MODE(pull);
    }

    void output() {
        SET_DIR_OUTPUT;
    }

    void input() {
        SET_DIR_INPUT;
    }

    FastInOut& operator= (int value) {
        this->write(value);
        return *this;
    };
    FastInOut& operator= (FastInOut& rhs) {
        return this->write(rhs.read());
    };
    operator int() {
        return read();
    };

    protected:
    fastio_vars container;
};

/**
 * Faster alternative compared to regular DigitalOut
 *
 * Except the constructor it is compatible with regular DigitalOut. Aditionally all
 * functions from DigitalInOut are also available (only initialization is different)
 * Code is based on Igor Skochinsky's code (http://mbed.org/users/igorsk/code/FastIO/)
 */
template <PinName pin, int initial = 0> class FastOut : public FastInOut<pin>
{
public:
    /**
     * Construct new FastOut object
     *
     * @code
     * FastOut<LED1> led1;
     * @endcode
     *
     * @param pin pin the FastOut object should be used for
     * @param initial (optional) initial state of the pin after construction: default is 0 (low)
     */
    FastOut() : FastInOut<pin>::FastInOut() {
        this->write(initial);
        SET_DIR_OUTPUT;
    }

    FastOut& operator= (int value) {
        this->write(value);
        return *this;
    };
    FastOut& operator= (FastOut& rhs) {
        return this->write(rhs.read());
    };
    operator int() {
        return this->read();
    };
};

/**
 * Faster alternative compared to regular DigitalIn
 *
 * Except the constructor it is compatible with regular DigitalIn. Aditionally all
 * functions from DigitalInOut are also available (only initialization is different)
 * Code is based on Igor Skochinsky's code (http://mbed.org/users/igorsk/code/FastIO/)
 */
template <PinName pin, PinMode pinmode = PullDefault> class FastIn : public FastInOut<pin>
{
public:
    /**
     * Construct new FastIn object
     *
     * @code
     * FastIn<LED1> led1;
     * @endcode
     *
     * @param pin pin the FastIn object should be used for
     * @param pinmode (optional) initial mode of the pin after construction: default is PullDefault
     */
    FastIn() : FastInOut<pin>::FastInOut() {
        SET_MODE(pinmode);
        SET_DIR_INPUT;
    }

    FastIn& operator= (int value) {
        this->write(value);
        return *this;
    };
    FastIn& operator= (FastIn& rhs) {
        return this->write(rhs.read());
    };
    operator int() {
        return this->read();
    };
};

#endif
