#ifndef CONF_H
#define CONF_H
#include "hal.h"

// #define USE_SERVOS_ONLY

#define MS_PER_LOG_DATA_UPDATE           250U // 250 ms data update interval

#define HZ_SENSOR_UPDATE_RATE            200U // 200 Hz sensor update rate

#define PRIMARY_LOGGER_ROLE              CM4_CPUID
#define LOGGER_SHOULD_BLOCK_ON_OVERWRITE 1

#define PID_MIN_VALUE                    0.0F
#define PID_MAX_VALUE                    5.0F
#define PID_STARTING_ROLL_P              0.2F
#define PID_STARTING_ROLL_I              0.3F
#define PID_STARTING_ROLL_D              0.05F
#define PID_STARTING_PITCH_P             0.2F
#define PID_STARTING_PITCH_I             0.3F
#define PID_STARTING_PITCH_D             0.05F
#define PID_STARTING_YAW_P               0.3F
#define PID_STARTING_YAW_I               0.05F
#define PID_STARTING_YAW_D               0.00015F
#define PID_STARTING_INTEGRAL_LIMIT      25.0F

#define LEFT_MOTOR_PWM_TIMER             TIM8
#define LEFT_MOTOR_PWM_CHANNEL           TIM_CHANNEL_1
#define LEFT_MOTOR_DMA_STREAM            DMA1_Stream0
#define LEFT_MOTOR_DMA_REQUEST           DMA_REQUEST_TIM8_CH1

#define LEFT_SERVO_PWM_TIMER             TIM13
#define LEFT_SERVO_PWM_CHANNEL           TIM_CHANNEL_1
#define LEFT_SERVO_PWM_FREQUENCY         50 // 50 Hz

#define MOTOR_MAX_THROTTLE               0.40F // 40% throttle
#define MOTOR_STARTUP_THROTTLE           0.25F // 25% throttle
#define MOTOR_MIN_THROTTLE               0.20F // 20% throttle

/*
 * PID mixing magnitudes for the Motors. They determine how much each PID axis contributes to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID pitch value of 0.5 is received the motor should
 * increase its throttle to maintain the current altitude while allowing the drone to pitch up.
 */
#define MOTOR_PID_ROLL_MIX               0.0F
#define MOTOR_PID_PITCH_MIX              1.0F
#define MOTOR_PID_YAW_MIX                0.0F

/*
 * PID mixing directions for the Motors. They determine the direction of each PID axis contribution to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID roll value of 0.5
 * is received the left motor should increase its throttle and the right motor should decrease its throttle.
 *
 * NOTE: A PID pitch value should always increase the throttle of both motors regardless of the sign of the PID pitch value.
 */
#define LEFT_MOTOR_PID_ROLL_MIX_DIR      (1.0F)
#define LEFT_MOTOR_PID_PITCH_MIX_DIR     (1.0F)
#define LEFT_MOTOR_PID_YAW_MIX_DIR       (1.0F)

#define RIGHT_MOTOR_PID_ROLL_MIX_DIR     (1.0F)
#define RIGHT_MOTOR_PID_PITCH_MIX_DIR    (1.0F)
#define RIGHT_MOTOR_PID_YAW_MIX_DIR      (1.0F)

#define SERVO_PID_ROLL_MIX               0.0F
#define SERVO_PID_PITCH_MIX              1.0F
#define SERVO_PID_YAW_MIX                0.0F

#define LEFT_SERVO_PID_ROLL_MIX_DIR      (1.0F)
#define LEFT_SERVO_PID_PITCH_MIX_DIR     (1.0F)
#define LEFT_SERVO_PID_YAW_MIX_DIR       (1.0F)

#define RIGHT_SERVO_PID_ROLL_MIX_DIR     (1.0F)
#define RIGHT_SERVO_PID_PITCH_MIX_DIR    (1.0F)
#define RIGHT_SERVO_PID_YAW_MIX_DIR      (1.0F)

#endif // CONF_H