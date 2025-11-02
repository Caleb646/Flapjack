#ifndef CONF_H
#define CONF_H

#include "conf/board.h"
#include "conf/ids.h"
#include "hal.h"

// #define CONF_USE_MY_BOARD

#ifdef CONF_USE_MY_BOARD
#define BOARD_CONF_INIT() BoardConfInit_MyBoard ()
#else
#define BOARD_CONF_INIT() BoardConfInit_DevBoard ()
#endif

#define PRIMARY_LOGGER_ROLE                CM4_CPUID
// #define PRIMARY_LOGGER_ROLE                CM7_CPUID
#define LOGGER_SHOULD_BLOCK_ON_OVERWRITE   1U
#define MS_PER_LOG_DATA_UPDATE             250U // 250 ms data update interval
#define LOG_DATA_TYPE_ATTITUDE             "attitude"
#define LOG_DATA_TYPE_PID_ATTITUDE         "pid_attitude"
#define LOG_DATA_TYPE_IMU_CALIB            "imu_calib"
#define LOG_DATA_TYPE_IMU_DATA             "imu_data"
#define LOG_DATA_TYPE_RAW_IMU_DATA         "imu_raw_data"
#define LOG_DATA_TYPE_ACTUATORS            "actuators"

#define FJ_LOOP_UPDATE_RATE_HZ             200U // main control loop update rate in Hz
#define FJ_SENSOR_UPDATE_MODE_IS_INTERRUPT 0U

#define PID_MIN_VALUE                      0.0F
#define PID_MAX_VALUE                      5.0F
#define PID_STARTING_ROLL_P                0.2F
#define PID_STARTING_ROLL_I                0.3F
#define PID_STARTING_ROLL_D                0.05F
#define PID_STARTING_PITCH_P               0.2F
#define PID_STARTING_PITCH_I               0.3F
#define PID_STARTING_PITCH_D               0.05F
#define PID_STARTING_YAW_P                 0.3F
#define PID_STARTING_YAW_I                 0.05F
#define PID_STARTING_YAW_D                 0.00015F
#define PID_STARTING_INTEGRAL_LIMIT        25.0F

#define MOTOR_MIN_THROTTLE                 0.20F // 20% throttle
#define MOTOR_MAX_THROTTLE                 0.40F // 40% throttle
#define MOTOR_STARTUP_THROTTLE             0.25F // 25% throttle

/*
 * PID mixing magnitudes for the Motors. They determine how much each PID axis contributes to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID pitch value of 0.5 is received the motor should
 * increase its throttle to maintain the current altitude while allowing the drone to pitch up.
 *
 *
 * PID mixing directions for the Motors. They determine the direction of each PID axis contribution to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID roll value of 0.5
 * is received the left motor should increase its throttle and the right motor should decrease its throttle.
 *
 * NOTE: A PID pitch value should always increase the throttle of both motors regardless of the sign of the PID pitch value.
 */
#endif // CONF_H