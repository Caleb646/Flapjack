#ifndef CONF_H
#define CONF_H
#include "hal.h"

#define LEFT_MOTOR_PWM_TIMER          TIM8
#define LEFT_MOTOR_PWM_CHANNEL        TIM_CHANNEL_1
#define LEFT_MOTOR_DMA_STREAM         DMA1_Stream0
#define LEFT_MOTOR_DMA_REQUEST        DMA_REQUEST_TIM8_CH1

#define LEFT_SERVO_PWM_TIMER          TIM13
#define LEFT_SERVO_PWM_CHANNEL        TIM_CHANNEL_1
#define LEFT_SERVO_PWM_FREQUENCY      50 // 50 Hz

#define MOTOR_MAX_THROTTLE            0.25F // 25% throttle
#define MOTOR_STARTUP_THROTTLE        0.10F // 10% throttle
#define MOTOR_MIN_THROTTLE            0.05F // 5% throttle

#define SERVO_PID_ROLL_MIX            0.0F
#define SERVO_PID_PITCH_MIX           1.0F
#define SERVO_PID_YAW_MIX             0.0F

#define LEFT_SERVO_PID_ROLL_MIX_DIR   1.0F
#define LEFT_SERVO_PID_PITCH_MIX_DIR  (-1.0F)
#define LEFT_SERVO_PID_YAW_MIX_DIR    1.0F

#define RIGHT_SERVO_PID_ROLL_MIX_DIR  1.0F
#define RIGHT_SERVO_PID_PITCH_MIX_DIR 1.0F
#define RIGHT_SERVO_PID_YAW_MIX_DIR   1.0F

// #define USE_SERVOS_ONLY

#endif // CONF_H