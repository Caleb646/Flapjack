#ifndef LOG_H
#define LOG_H

#include "common.h"
#include "hal.h"
#include <stdint.h>
#include <stdio.h>

// clang-format off
/* < and > delimit a message */
#define LOG_(lvl, ...)                                                                                                  \
    do {                                                                                                                \
        printf ("<{\"type\":\"debug\",\"lvl\":\"%s\",\"file\":\"%s\",\"line\":%u,\"msg\":\"", lvl, __FILE__, __LINE__); \
        printf (__VA_ARGS__);                                                                                           \
        printf ("\"}>\r\n");                                                                                            \
    } while (0)
#define LOG_INFO(...)           LOG_ ("[INFO]", __VA_ARGS__)
#define LOG_DEBUG(...)          LOG_ ("[DEBUG]", __VA_ARGS__)
#define LOG_WARN(...)           LOG_ ("[WARN]", __VA_ARGS__)
#define LOG_ERROR(...)          LOG_ ("[ERROR]", __VA_ARGS__)

// #define LOG_ARRAY(arr, len, fmt)                    \
//     do {                                                             \
//         printf("<{\"type\":\"debug\",\"lvl\":\"%s\",\"array\":\"%s\",\"values\":[", "[DEBUG]", #arr);             \
//         for (uint32_t i = 0; i < (len); ++i) {                      \
//             printf(fmt, (arr)[i]);                                   \
//             if (i < (len) - 1) printf(",");                          \
//         }                                                            \
//         printf("]}>\r\n");                                           \
//     } while (0)

#define LOG_DATA_TYPE_ATTITUDE  "attitude"
#define LOG_DATA_TYPE_IMU_CALIB "imu_calib"
#define LOG_DATA_TYPE_IMU_DATA  "imu_data"
#define LOG_DATA_TYPE_RAW_IMU_DATA  "imu_raw_data"
// Example usage: LOG_DATA("imu", "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}", roll, pitch, yaw);
#define LOG_DATA(type, fmt, ...)                      \
    do {                                              \
        printf ("<{\"type\":\"%s\",\"data\":", type); \
        printf (fmt, __VA_ARGS__);                      \
        printf ("}>\r\n");                            \
    } while (0)

// clang-format on

STATUS_TYPE LoggerInit (UART_HandleTypeDef* pUART);


#endif // LOG_H
