// Generated with umsg_gen on 2026-05-30
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs
typedef struct
{
    uint32_t channels[16];
    uint8_t rssi;
    uint8_t link_quality;
} umsg_rc_input_t;

// api function headers
umsg_sub_handle_t umsg_rc_input_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_rc_input_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_rc_input_publish(umsg_rc_input_t* data);
void umsg_rc_input_publish_ch(umsg_rc_input_t* data, uint8_t channel);
uint8_t umsg_rc_input_receive(umsg_sub_handle_t queue, umsg_rc_input_t* data, uint32_t timeout);
uint8_t umsg_rc_input_peek(umsg_rc_input_t* data);


#ifdef __cplusplus
}
#endif

