#ifndef CFGS_CFG_H
#define CFGS_CFG_H

// to disable redefinition warnings for default target macros
#pragma GCC system_header

#define CFG_PRIMARY_LOGGER                   CM4_CPUID
// #define CFG_PRIMARY_LOGGER                   CM7_CPUID
#define CFG_LOGGER_SHOULD_BLOCK_ON_OVERWRITE 1U
#define LOG_DATA_TYPE_ATTITUDE               "attitude"
#define LOG_DATA_TYPE_PID_ATTITUDE           "pid_attitude"
#define LOG_DATA_TYPE_IMU_CALIB              "imu_calib"
#define LOG_DATA_TYPE_IMU_DATA               "imu_data"
#define LOG_DATA_TYPE_RAW_IMU_DATA           "imu_raw_data"
#define LOG_DATA_TYPE_ACTUATORS              "actuators"

// main control loop update rate in Hz
#define CFG_LOOP_UPDATE_RATE_HZ              200U

#define CFG_PID_MIN_VALUE                    0.0F
#define CFG_PID_MAX_VALUE                    5.0F
#define CFG_PID_ROLL_P                       0.2F
#define CFG_PID_ROLL_I                       0.3F
#define CFG_PID_ROLL_D                       0.05F
#define CFG_PID_PITCH_P                      0.2F
#define CFG_PID_PITCH_I                      0.3F
#define CFG_PID_PITCH_D                      0.05F
#define CFG_PID_YAW_P                        0.3F
#define CFG_PID_YAW_I                        0.05F
#define CFG_PID_YAW_D                        0.00015F
#define CFG_PID_THROTTLE_P                   0.1F
#define CFG_PID_THROTTLE_I                   0.2F
#define CFG_PID_THROTTLE_D                   0.05F
#define CFG_PID_INTEGRAL_LIMIT               25.0F

#define CFG_MOTOR_MIN_THROTTLE               0.20F // 20% throttle
#define CFG_MOTOR_MAX_THROTTLE               0.40F // 40% throttle
#define CFG_MOTOR_STARTUP_THROTTLE           0.25F // 25% throttle

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
#endif // CFGS_CFG_H