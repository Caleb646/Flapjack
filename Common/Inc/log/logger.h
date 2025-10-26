#ifndef LOG_H
#define LOG_H

#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/format.h"
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
        PRINT ("<{\"type\":\"debug\",\"lvl\":\"%s\",\"core\":\"%s\",\"file\":\"%s\",\"function\":\"%s\",\"line\":%u,\"msg\":\"", lvl, __CORE_NAME__, __FILENAME__, __func__, __LINE__); \
        PRINT (__VA_ARGS__);                                                                                           \
        PRINT ("\"}>\r\n");                                                                                            \
    } while (0)
#define LOG_INFO(...)           LOG_ ("[INFO]", __VA_ARGS__)
#define LOG_DEBUG(...)          LOG_ ("[DEBUG]", __VA_ARGS__)
#define LOG_WARN(...)           LOG_ ("[WARN]", __VA_ARGS__)
#define LOG_ERROR(...)          LOG_ ("[ERROR]", __VA_ARGS__)

#define LOG_ERROR_IF(cond, ...) \
    do {                                 \
        if ((cond) == true) {                     \
            LOG_ERROR (__VA_ARGS__);    \
        }                                \
    } while (0)

#define LOG_ERROR_IF_NOT(cond, ...) LOG_ERROR_IF (!(cond), __VA_ARGS__)
#define LOG_ERROR_IF_NULL(ptr, ...) LOG_ERROR_IF ((ptr) == NULL, __VA_ARGS__)

#define RETURN_IF(cond, retval, ...)                \
    do {                                    \
        if ((cond) == true) {               \
            LOG_ERROR (__VA_ARGS__);        \
            return (retval);                \
        }                                   \
    } while (0)
#define RETURN_IF_NOT(cond, retval, ...) RETURN_IF (!(cond), (retval), __VA_ARGS__)
#define RETURN_IF_NULL(ptr, retval, ...) RETURN_IF ((ptr) == NULL, (retval), __VA_ARGS__)

#define GOTO_IF(cond, label, ...)          \
    do {                                    \
        if ((cond) == true) {               \
            LOG_ERROR (__VA_ARGS__);        \
            goto label;                     \
        }                                   \
    } while (0)

#define GOTO_IF_NOT(cond, label, ...) GOTO_IF (!(cond), label, __VA_ARGS__)
#define GOTO_IF_NULL(ptr, label, ...) GOTO_IF ((ptr) == NULL, label, __VA_ARGS__)

// Example usage: LOG_DATA("imu", "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}", roll, pitch, yaw);
#define LOG_DATA(type, fmt, ...)                      \
    do {                                              \
        PRINT ("<{\"type\":\"%s\",\"core\":\"%s\",\"data\":", type, __CORE_NAME__); \
        PRINT (fmt, __VA_ARGS__);                      \
        PRINT ("}>\r\n");                            \
    } while (0)

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
