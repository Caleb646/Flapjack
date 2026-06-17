// Generated with umsg_gen on 2026-06-16
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs

typedef enum
{
    TUNE_PID_GAIN_KP,
    TUNE_PID_GAIN_KI,
    TUNE_PID_GAIN_KD,
    TUNE_PID_GAIN_INTEGRAL_LIMIT
} umsg_tune_gain_t;

typedef struct
{
    uint8_t axis;
    umsg_tune_gain_t gain;
    float value;
} umsg_tune_pid_t;

// api function headers
umsg_sub_handle_t umsg_tune_pid_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_tune_pid_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_tune_pid_publish(umsg_tune_pid_t* data);
void umsg_tune_pid_publish_ch(umsg_tune_pid_t* data, uint8_t channel);
uint8_t umsg_tune_pid_receive(umsg_sub_handle_t queue, umsg_tune_pid_t* data, uint32_t timeout);
uint8_t umsg_tune_pid_peek(umsg_tune_pid_t* data);


#ifdef __cplusplus
}
#endif

