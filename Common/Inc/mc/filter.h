#ifndef MOTION_CONTROL_FILTER_H
#define MOTION_CONTROL_FILTER_H

#include "common.h"
#include "device/imu/imu.h"
#include <stdbool.h>
#include <stdint.h>

typedef uint8_t eFILTER_ID_t;
enum {
    eFILTER_MADGWICK_6DOF = (1U << 1U),
    eFILTER_MADGWICK_9DOF = (1U << 2U),
    eFILTER_LOWPASS       = (1U << 3U)
};

typedef struct {
    float cutoffFreq;
    float sampleRate;
} FilterLowPassInitConf_t;

typedef struct {
    uint8_t unused;
} FilterLowPass_t;

typedef struct {
    float gyroMeasureErrorDegs;
    float gyroMeasureDriftDegs;
} FilterMadgwickInitConf_t;

typedef struct {
    // estimated orientation quaternion elements with initial conditions
    Vec4f qEst;
    // gyro bias error
    Vec3f gbias;
    // Gamma_t (γt)
    float beta;
    float zeta;
    // reference direction of flux in earth frame
    float bx;
    float bz;
} FilterMadgwick_t;

typedef struct {
    FilterLowPassInitConf_t lowpassConf;
    FilterMadgwickInitConf_t madgwickConf;
} FilterInitConf_t;

typedef struct Filter_s {
    FilterLowPass_t lowpass;
    FilterMadgwick_t madgwick;
    uint32_t msLastUpdateTime;
    bool isInitialized;
} Filter_t;

typedef Filter_t vFilter_t;

// clang-format off
#ifdef UNIT_TEST

bool FilterMadgwickInit (FilterMadgwickInitConf_t conf, FilterMadgwick_t* pOut);
bool FilterMadgwickUpdate (FilterMadgwick_t* pFilter,Vec3f const* pAccel,Vec3f const* pGyro,Vec3f const* pMag,float dt,Vec3f* pOutAttitude);
bool FilterMadgwickUpdate_6DOF (FilterMadgwick_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt);
bool FilterMadgwickUpdate_9DOF (FilterMadgwick_t* pFilter,Vec3f const* pAccel,Vec3f const* pGyro,Vec3f const* pMag, float dt);

#endif

eSTATUS_t FilterInit (FilterInitConf_t conf, Filter_t* pOut);
eSTATUS_t FilterStart (vFilter_t* pFilter, uint32_t warmUpIterations, Vec3f* pOutAttitude);
eSTATUS_t FilterStop (vFilter_t* pFilter);
eSTATUS_t FilterUpdate (vFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutput);
vFilter_t const* FilterGetActiveFilter (void);
vFilter_t* FilterGetMutableActiveFilter (void);

// clang-format on

#define FILTER_INIT(pSTATUS)                                              \
    do {                                                                  \
        FilterInitConf_t conf                  = { 0 };                   \
        conf.madgwickConf.gyroMeasureErrorDegs = 5.0f;                    \
        conf.madgwickConf.gyroMeasureDriftDegs = 0.2f;                    \
        *(pSTATUS)                             = FilterInit (conf, NULL); \
    } while (0)

#endif // MOTION_CONTROL_FILTER_H
