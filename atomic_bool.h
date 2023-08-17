/* (C) 2014 Richard Thompson (Tomo2k)
This software is Apache 2.0 licenced.
*/
#pragma once

//! Atomic Boolean.
//! Also provides test-and-set and test-and-clear methods.
//! Interchangable with bool.\n
//! Works by disabling then re-enabling interrupts.\n
//! Do not use during ISR or when interrupts should remain disabled.
class AtomicBool
{
public:
    //! Create the boolean and set initial value from bool
    //! @warning Construction is not guaranteed atomic! Interrupts may occur before construction completes.
    AtomicBool(const bool initialValue) :
        m_value(initialValue) {}

    //! Create and set initial value from another AtomicBool.
    //! This method guaranteed atomic.
    AtomicBool(const AtomicBool &rhs) {
        __disable_irq();
        m_value = rhs.m_value;
        __enable_irq();
    }

    // Assign to bool
    void operator=(const bool &rhs) {
        __disable_irq();
        m_value = rhs;
        __enable_irq();
    }

    // Assign to own type
    void operator=(const AtomicBool &rhs) {
        __disable_irq();
        m_value = rhs.m_value;
        __enable_irq();
    }

    inline bool operator==(const bool &rhs) const {
        __disable_irq();
        bool result(m_value == rhs);
        __enable_irq();
        return result;
    }

    inline bool operator!=(const bool &rhs) const {
        __disable_irq();
        bool result(m_value != rhs);
        __enable_irq();
        return result;
    }

    //! Test and set the bool
    //! @returns true if bool was false before calling
    inline bool testAndSet() {
        __disable_irq();
        bool result(!m_value);
        m_value = true;
        __enable_irq();
        return result;
    }

    //! Test and clear the bool
    //! @returns true if bool was true before calling
    inline bool testAndClear() {
        __disable_irq();
        bool result(m_value);
        m_value = false;
        __enable_irq();
        return result;
    }

    inline operator bool() const {
        __disable_irq();
        bool result(m_value);
        __enable_irq();
        return result;
    }

    bool operator==(const AtomicBool &rhs) const {
        __disable_irq();
        bool result(m_value == rhs.m_value);
        __enable_irq();
        return result;
    }

    bool operator!=(const AtomicBool &rhs) const {
        __disable_irq();
        bool result(m_value != rhs.m_value);
        __enable_irq();
        return result;
    }

private:
    bool m_value;
};
