#ifndef LOG_H
#define LOG_H

#include "common.h"
#include "hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Macro to extract filename from __FILE__ at compile time
#ifdef _WIN32
#define __FILENAME__ \
    (strrchr (__FILE__, '\\') ? strrchr (__FILE__, '\\') + 1 : __FILE__)
#else
#define __FILENAME__ \
    (strrchr (__FILE__, '/') ? strrchr (__FILE__, '/') + 1 : __FILE__)
#endif

// clang-format off

#ifdef CORE_CM4
#define __CORE_NAME__ "CM4"
#elif defined CORE_CM7
#define __CORE_NAME__ "CM7"
#else
#define __CORE_NAME__ "Unknown"
#endif

/* < and > delimits a message */
#define LOG_(lvl, ...)                                                                                                  \
    do {                                                                                                                \
        printf ("<{\"type\":\"debug\",\"lvl\":\"%s\",\"core\":\"%s\",\"file\":\"%s\",\"function\":\"%s\",\"line\":%u,\"msg\":\"", lvl, __CORE_NAME__, __FILENAME__, __func__, __LINE__); \
        printf (__VA_ARGS__);                                                                                           \
        printf ("\"}>\r\n");                                                                                            \
    } while (0)
#define LOG_INFO(...)           LOG_ ("[INFO]", __VA_ARGS__)
#define LOG_DEBUG(...)          LOG_ ("[DEBUG]", __VA_ARGS__)
#define LOG_WARN(...)           LOG_ ("[WARN]", __VA_ARGS__)
#define LOG_ERROR(...)          LOG_ ("[ERROR]", __VA_ARGS__)

#define LOG_DATA_TYPE_ATTITUDE  "attitude"
#define LOG_DATA_TYPE_IMU_CALIB "imu_calib"
#define LOG_DATA_TYPE_IMU_DATA  "imu_data"
#define LOG_DATA_TYPE_RAW_IMU_DATA  "imu_raw_data"
// Example usage: LOG_DATA("imu", "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}", roll, pitch, yaw);
#define LOG_DATA(type, fmt, ...)                      \
    do {                                              \
        printf ("<{\"type\":\"%s\",\"core\":\"%s\",\"data\":", type, __CORE_NAME__); \
        printf (fmt, __VA_ARGS__);                      \
        printf ("}>\r\n");                            \
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
              (int16_t)(attitude.roll), (int16_t)(attitude.pitch), (int16_t)(attitude.yaw))

#define LOG_DATA_ACTUATORS_DATA(motorName, motor, servoName, servo) \
    LOG_DATA ("actuators", "{\"%s\":{\"type\":\"motor\",\"throttle\":%d,\"target_throttle\":%d},\"%s\":{\"type\":\"servo\",\"angle\":%d,\"target_angle\":%d}}", \
              motorName, (int16_t)((motor).desc.curThrottle * 100.0F), (int16_t)((motor).desc.curTargetThrottle * 100.0F), \
              servoName, (int16_t)((servo).desc.curAngle), (int16_t)((servo).desc.curTargetAngle))

// clang-format on

eSTATUS_t LoggerInit (void);
UART_HandleTypeDef* LoggerGetUARTHandle (void);
// void LogStackTrace (void);


#endif // LOG_H
