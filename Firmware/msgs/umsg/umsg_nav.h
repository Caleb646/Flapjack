// Generated with umsg_gen on 2026-08-15
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs

typedef enum
{
    NAV_VALID_ATTITUDE = (1 << 0),
    NAV_VALID_POSITION = (1 << 1),
    NAV_VALID_VELOCITY = (1 << 2),
    NAV_VALID_BARO_ALT = (1 << 3)
} umsg_nav_valid_t;

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


#ifdef __cplusplus
}
#endif

