

#include "frame_analysis.h"


uint8_t xor_check(uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;
    uint8_t x = 0;

    for(; i < len; i++)
    {
        x = x ^ (*(buf + i));
    }

    return x;
}

