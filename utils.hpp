#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include "cstdint"
#include "fixed_class.h"

constexpr int JR3_PRECISION = 15;
constexpr int FIXED_PRECISION = 31;

using fixed_t = fixedpoint::fixed_point<FIXED_PRECISION>; // (-1, 1]

inline fixed_t jr3ToFixedPoint(uint16_t mantissa, int8_t exponent = 0x00)
{
    fixed_t f;
    // beware of integral promotion! https://stackoverflow.com/a/30474166
    f.intValue = ((~mantissa + 1U) & (mantissa >> JR3_PRECISION ? 0x0000FFFF : 0xFFFFFFFF)) << (FIXED_PRECISION - JR3_PRECISION + exponent);
    return f;
}

inline uint16_t jr3FromFixedPoint(fixed_t f)
{
    uint16_t temp = (~f.intValue + 1U) >> (FIXED_PRECISION - JR3_PRECISION);
    return temp;
}

#endif // __UTILS_HPP__
