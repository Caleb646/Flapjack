#include <math.h>
#include <string.h>
#include "filter.h"

int8_t FilterMadgwick(FilterMadgwickContext *pContext, Vec3 gyro, Vec3 accel, Vec3f *pOutputAttitude)
{
    return 0;
}

int8_t FilterMadgwickMagnetometer(FilterMadgwickContext *pContext, Vec3 gyro, Vec3 accel, Vec3 magno, Vec3f *pOutputAttitude)
{
    return 0;
}

int8_t FilterMadgwickInit(FilterMadgwickContext *pContext)
{
    // Initialization values: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf
    memset((void*)pContext, 0, sizeof(FilterMadgwickContext));
    pContext->dt = 0.001f;
    pContext->gyroMeasureError = M_PI * (5.0f / 180.0f);
    pContext->beta = sqrt(3.0f / 4.0f) * pContext->gyroMeasureError;
    pContext->seq1 = 1.0f;
    pContext->seq2 = 0.0f;
    pContext->seq3 = 0.0f;
    pContext->seq4 = 0.0f;
    return 0;
}