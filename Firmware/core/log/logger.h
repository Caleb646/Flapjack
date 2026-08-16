#ifndef LOG_H
#define LOG_H

#include "hal.h"
#include "target.h"

#include "core/core_shared.h"
#include "core/log/format.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>


// Macro to extract filename from __FILE__ at compile time
#ifdef _WIN32
#define __FILENAME__ (strrchr (__FILE__, '\\') ? strrchr (__FILE__, '\\') + 1 : __FILE__)
#else
#define __FILENAME__ (strrchr (__FILE__, '/') ? strrchr (__FILE__, '/') + 1 : __FILE__)
#endif

// clang-format off

#ifdef CORE_CM4
#define __CORE_NAME__ "CM4"
#elif defined CORE_CM7
#define __CORE_NAME__ "CM7"
#else
#define __CORE_NAME__ "Unknown"
#endif

#ifndef UNIT_TEST
#define PRINT tfp_printf
#else
#define PRINT printf
#endif

/* < and > delimits a message */
#define LOG_(lvl, ...)                                                                                                  \
    do {                                                                                                                \
        PRINT ("<{\"type\":\"debug\",\"lvl\":\"%s\",\"core\":\"%s\",\"function\":\"%s\",\"line\":%u,\"msg\":\"", lvl, __CORE_NAME__, __func__, __LINE__); \
        PRINT (__VA_ARGS__);                                                                                           \
        PRINT ("\"}>\r\n");                                                                                            \
    } while (0)

/*
 * Compiled-out stand-in, selected by CFG_LOG_LEVEL (target/cfgs/cfg.h).
 *
 * The `if (0)` is deliberate and must stay: it keeps the arguments referenced,
 * so a variable used ONLY inside log calls does not become -Wunused, and it
 * keeps the format string type-checked. GCC drops the branch during
 * gimplification, so nothing reaches the image even at -O0 (Debug's setting).
 *
 * Only the log statement is replaced. The `return` in RETURN_IF and the `goto`
 * in GOTO_IF sit outside it and still fire at every level, including NONE.
 */
#define LOG_NOOP(...)                       \
    do {                                    \
        if (0) {                            \
            PRINT (__VA_ARGS__);            \
        }                                   \
    } while (0)

#if CFG_LOG_LEVEL >= CFG_LOG_LEVEL_ERROR
#define LOG_ERROR(...)          LOG_ ("[ERROR]", __VA_ARGS__)
#else
#define LOG_ERROR(...)          LOG_NOOP (__VA_ARGS__)
#endif

#if CFG_LOG_LEVEL >= CFG_LOG_LEVEL_WARN
#define LOG_WARN(...)           LOG_ ("[WARN]", __VA_ARGS__)
#else
#define LOG_WARN(...)           LOG_NOOP (__VA_ARGS__)
#endif

#if CFG_LOG_LEVEL >= CFG_LOG_LEVEL_INFO
#define LOG_INFO(...)           LOG_ ("[INFO]", __VA_ARGS__)
#else
#define LOG_INFO(...)           LOG_NOOP (__VA_ARGS__)
#endif

#if CFG_LOG_LEVEL >= CFG_LOG_LEVEL_DEBUG
#define LOG_DEBUG(...)          LOG_ ("[DEBUG]", __VA_ARGS__)
#else
#define LOG_DEBUG(...)          LOG_NOOP (__VA_ARGS__)
#endif

#define LOG_ERROR_IF(cond, ...) \
    do {                                 \
        if ((cond)) {                     \
            LOG_ERROR (__VA_ARGS__);    \
        }                                \
    } while (0)

#define LOG_ERROR_IF_NOT(cond, ...) LOG_ERROR_IF (!(cond), __VA_ARGS__)
#define LOG_ERROR_IF_NULL(ptr, ...) LOG_ERROR_IF ((ptr) == NULL, __VA_ARGS__)

#define RETURN_IF(cond, retval, ...)                \
    do {                                    \
        if ((cond)) {               \
            LOG_ERROR (__VA_ARGS__);        \
            return (retval);                \
        }                                   \
    } while (0)
#define RETURN_IF_NOT(cond, retval, ...) RETURN_IF (!(cond), (retval), __VA_ARGS__)
#define RETURN_IF_NULL(ptr, retval, ...) RETURN_IF ((ptr) == NULL, (retval), __VA_ARGS__)

#define GOTO_IF(cond, label, ...)          \
    do {                                    \
        if ((cond)) {               \
            LOG_ERROR (__VA_ARGS__);        \
            goto label;                     \
        }                                   \
    } while (0)

#define GOTO_IF_NOT(cond, label, ...) GOTO_IF (!(cond), label, __VA_ARGS__)
#define GOTO_IF_NULL(ptr, label, ...) GOTO_IF ((ptr) == NULL, label, __VA_ARGS__)

/*
 * Data streams (and every LOG_DATA_* / LOG_n_FLOATS below, which expand to
 * this) gate at DEBUG. They carry no severity of their own, they are the
 * heaviest users of the link, and they are what the GUI plots - so a build
 * below DEBUG stops those plots updating.
 */
// Example usage: LOG_DATA("imu", "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}", roll, pitch, yaw);
#if CFG_LOG_LEVEL >= CFG_LOG_LEVEL_DEBUG
#define LOG_DATA(type, fmt, ...)                      \
    do {                                              \
        PRINT ("<{\"type\":\"%s\",\"core\":\"%s\",\"data\":", type, __CORE_NAME__); \
        PRINT (fmt, __VA_ARGS__);                      \
        PRINT ("}>\r\n");                            \
    } while (0)
#else
#define LOG_DATA(type, fmt, ...)                      \
    do {                                              \
        if (0) {                                      \
            PRINT ("%s", type);                       \
            PRINT (fmt, __VA_ARGS__);                 \
        }                                             \
    } while (0)
#endif


#define _LOG_JSON_2(key1, val1) "\"%s\":%s", #key1, #val1
#define _LOG_JSON_4(key1, val1, key2, val2) "\"%s\":%s,\"%s\":%s", #key1, #val1, #key2, #val2
#define _LOG_JSON_6(key1, val1, key2, val2, key3, val3) "\"%s\":%s,\"%s\":%s,\"%s\":%s", #key1, #val1, #key2, #val2, #key3, #val3
#define _LOG_JSON_8(key1, val1, key2, val2, key3, val3, key4, val4) "\"%s\":%s,\"%s\":%s,\"%s\":%s,\"%s\":%s", #key1, #val1, #key2, #val2, #key3, #val3, #key4, #val4

#define LOG_4_FLOATS(type, k1, v1, k2, v2, k3, v3, k4, v4) \
    LOG_DATA (type, "{" _LOG_JSON_8 (k1, "%d", k2, "%d", k3, "%d", k4, "%d") "}", \
              (int16_t)((v1) * 1000.0F), (int16_t)((v2) * 1000.0F), (int16_t)((v3) * 1000.0F), (int16_t)((v4) * 1000.0F))

#define LOG_8_FLOATS(type, k1, v1, k2, v2, k3, v3, k4, v4, k5, v5, k6, v6, k7, v7, k8, v8) \
    LOG_DATA (type, "{" _LOG_JSON_8 (k1, "%d", k2, "%d", k3, "%d", k4, "%d") "," _LOG_JSON_8 (k5, "%d", k6, "%d", k7, "%d", k8, "%d") "}", \
              (int16_t)((v1) * 1000.0F), (int16_t)((v2) * 1000.0F), (int16_t)((v3) * 1000.0F), (int16_t)((v4) * 1000.0F), \
              (int16_t)((v5) * 1000.0F), (int16_t)((v6) * 1000.0F), (int16_t)((v7) * 1000.0F), (int16_t)((v8) * 1000.0F))

#define LOG_IMU_DATA(accel, gyro) LOG_8_FLOATS ("imu_data", ax, accel.x, ay, accel.y, az, accel.z, aw, 0.0F, gx, gyro.x, gy, gyro.y, gz, gyro.z, gw, 0.0F)

#define LOG_DATA_IMU_CALIB(calib) \
    LOG_DATA (LOG_DATA_TYPE_IMU_CALIB, "{\"rslt\":%d,\"err\":%d}", (calib).result, (calib).err, (calib).gyro.z)

/* accel data is in meters per second squared and gyro is in degrees per second */
#define LOG_DATA_IMU_DATA(accel, gyro) \
    LOG_DATA (LOG_DATA_TYPE_IMU_DATA, "{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d}", \
              (int16_t)(accel.x * 1000.0F), (int16_t)(accel.y * 1000.0F), (int16_t)(accel.z * 1000.0F), \
              (int16_t)(gyro.x * 1000.0F), (int16_t)(gyro.y * 1000.0F), (int16_t)(gyro.z * 1000.0F))
              
/* attitude is in degrees */
#define LOG_DATA_CURRENT_ATTITUDE(attitude) \
    LOG_DATA (LOG_DATA_TYPE_ATTITUDE, "{\"roll\":%d,\"pitch\":%d,\"yaw\":%d}", \
              (int16_t)((attitude).roll), (int16_t)((attitude).pitch), (int16_t)((attitude).yaw))

/* pid attitude is in degrees */
#define LOG_DATA_CURRENT_PID_ATTITUDE(PID) \
    LOG_DATA (LOG_DATA_TYPE_PID_ATTITUDE, "{\"roll\":%d,\"pitch\":%d,\"yaw\":%d}", \
              (int16_t)((PID).roll), (int16_t)((PID).pitch), (int16_t)((PID).yaw))

#define LOG_DATA_ACTUATORS_DATA(motorName, motor, servoName, servo) \
    LOG_DATA (LOG_DATA_TYPE_ACTUATORS, "{\"%s\":{\"type\":\"motor\",\"throttle\":%d,\"target_throttle\":%d},\"%s\":{\"type\":\"servo\",\"angle\":%d,\"target_angle\":%d}}", \
              motorName, (int16_t)((motor).curThrottle * 100.0F), (int16_t)((motor).curTargetThrottle * 100.0F), \
              servoName, (int16_t)((servo).curAngle), (int16_t)((servo).curTargetAngle))

// clang-format on

typedef void (*LoggerWriteToSink_t) (uint8_t const* pData, uint32_t len);

eSTATUS_t LoggerInit (void);
eSTATUS_t LoggerAddSink (LoggerWriteToSink_t sink);
eSTATUS_t LoggerRemoveSink (LoggerWriteToSink_t sink);
// void LogStackTrace (void);

#endif // LOG_H
