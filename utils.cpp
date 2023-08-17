#include "utils.hpp"

float jr3FloatToIEEE754(int8_t exponent, uint16_t mantissa)
{
    uint32_t temp = 0;

    if (mantissa >> 15)
    {
        temp |= (((~mantissa & 0x3FFF) + 1U) << 9) | (1U << 31);
    }
    else
    {
        temp |= (mantissa & 0x3FFF) << 9;
    }

    temp |= (exponent + 126) << 23;

    float f;
    memcpy(&f, &temp, 4);
    return f;
}

uint16_t jr3FixedFromIEEE754(float f)
{
    uint32_t temp;
    memcpy(&temp, &f, 4);

    int8_t exponent = ((temp & 0x7F800000) >> 23) - 127;
    uint16_t mantissa = (temp & 0x007FFFFF) >> (8 - exponent);

    if (temp >> 31)
    {
        return ~((mantissa - 1U) | (1U << (15 + exponent)));
    }
    else
    {
        return mantissa | (1U << (15 + exponent));
    }
}
