#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include "mbed.h"

float fixedToIEEE754(int8_t exponent, uint16_t mantissa);

inline float fixedToIEEE754(uint16_t mantissa)
{
    // beware of integral promotion! https://stackoverflow.com/a/30474166
    int8_t exponent = __CLZ((mantissa >> 15 ? ~mantissa : mantissa) & 0x0000FFFF) - 17;
    return fixedToIEEE754(-exponent, mantissa << exponent);
}

uint16_t fixedFromIEEE754(float f);

#endif // __UTILS_HPP__
