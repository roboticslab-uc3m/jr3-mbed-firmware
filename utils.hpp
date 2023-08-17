#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include "mbed.h"

float jr3FloatToIEEE754(int8_t exponent, uint16_t mantissa);

inline float jr3FixedToIEEE754(uint16_t fixed)
{
    // beware of integral promotion! https://stackoverflow.com/a/30474166
    int8_t exponent = __CLZ((fixed >> 15 ? ~fixed : fixed) & 0x0000FFFF) - 17;
    return jr3FloatToIEEE754(-exponent, fixed << exponent);
}

uint16_t jr3FixedFromIEEE754(float f);

#endif // __UTILS_HPP__
