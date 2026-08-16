// Generated with umsg_gen - do not edit by hand
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs
typedef struct
{
    float w[3];
    float vel_b[3];
    float quat[4];
} umsg_guidance_setpoints_t;

// api function headers
umsg_sub_handle_t umsg_guidance_setpoints_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_guidance_setpoints_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_guidance_setpoints_publish(umsg_guidance_setpoints_t* data);
void umsg_guidance_setpoints_publish_ch(umsg_guidance_setpoints_t* data, uint8_t channel);
uint8_t umsg_guidance_setpoints_receive(umsg_sub_handle_t queue, umsg_guidance_setpoints_t* data, uint32_t timeout);


#ifdef __cplusplus
}
#endif

