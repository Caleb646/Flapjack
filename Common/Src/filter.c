#include <math.h>
#include "filter.h"

int8_t FilterMadgwick(FilterMadgwickContext *pContext, Vec3 gyro, Vec3 accel)
{
    return 0;
}
int8_t FilterMadgwickwMagnetometer(FilterMadgwickContext *pContext, Vec3 gyro, Vec3 accel, Vec3 magno)
{
    return 0;
}
int8_t FilterMadgwickInit(FilterMadgwickContext *pContext)
{
    // Initialization values source: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf
    pContext->dt = 0.001f;
    pContext->gyroMeasureError = M_PI * (5.0f / 180.0f);
    pContext->beta = sqrt(3.0f / 4.0f) * pContext->gyroMeasureError;
    pContext->seq1 = 1.0f;
    pContext->seq2 = 0.0f;
    pContext->seq3 = 0.0f;
    pContext->seq4 = 0.0f;
    return 0;
}