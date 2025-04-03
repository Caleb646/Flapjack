#include "common.h"

int32_t clipi32(int32_t v, int32_t lower, int32_t upper)
{
    if(v < lower) return lower;
    else if (v > upper) return upper;
    else return v;
}

float clipf32(float v, float lower, float upper)
{
    if(v < lower) return lower;
    else if (v > upper) return upper;
    else return v;
}