// Generated with umsg_gen - do not edit by hand
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs

typedef enum
{
    CONTROL_INIT_MOTORS = (1 << 0),
    CONTROL_INIT_SERVOS = (1 << 1),
    CONTROL_INIT_MIXER  = (1 << 2),
    CONTROL_INIT_PID    = (1 << 3)
} umsg_control_init_t;

typedef enum
{
    CONTROL_MOTORS_DISARMED,
    CONTROL_MOTORS_ARMING,
    CONTROL_MOTORS_ARMED
} umsg_control_motor_state_t;

typedef struct
{
    uint8_t init_ok;
    umsg_control_motor_state_t motor_state;
    uint32_t motor_writes;
    uint32_t servo_writes;
} umsg_control_status_t;

// api function headers
umsg_sub_handle_t umsg_control_status_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_control_status_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_control_status_publish(umsg_control_status_t* data);
void umsg_control_status_publish_ch(umsg_control_status_t* data, uint8_t channel);
uint8_t umsg_control_status_receive(umsg_sub_handle_t queue, umsg_control_status_t* data, uint32_t timeout);
// Running total of publishes; wraps, so diff two reads as unsigned. Only exact
// while the topic has a single publisher (one task, no ISR): see umsg_publish.
uint32_t umsg_control_status_count(void);


#ifdef __cplusplus
}
#endif

