// Generated with umsg_gen on 2026-06-14
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs
typedef struct
{
    float quat[4];
    float euler[3];
    float gyro[3];
    float pos_ned[3];
    float vel_ned[3];
    float alt;
    uint8_t valid;
} umsg_nav_state_t;

// api function headers
umsg_sub_handle_t umsg_nav_state_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_nav_state_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_nav_state_publish(umsg_nav_state_t* data);
void umsg_nav_state_publish_ch(umsg_nav_state_t* data, uint8_t channel);
uint8_t umsg_nav_state_receive(umsg_sub_handle_t queue, umsg_nav_state_t* data, uint32_t timeout);
uint8_t umsg_nav_state_peek(umsg_nav_state_t* data);


#ifdef __cplusplus
}
#endif

