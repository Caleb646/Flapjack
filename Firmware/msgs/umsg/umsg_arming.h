// Generated with umsg_gen on 2026-08-15
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs
typedef struct
{
    uint8_t arm;
} umsg_arming_request_t;

// api function headers
umsg_sub_handle_t umsg_arming_request_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_arming_request_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_arming_request_publish(umsg_arming_request_t* data);
void umsg_arming_request_publish_ch(umsg_arming_request_t* data, uint8_t channel);
uint8_t umsg_arming_request_receive(umsg_sub_handle_t queue, umsg_arming_request_t* data, uint32_t timeout);
uint8_t umsg_arming_request_peek(umsg_arming_request_t* data);


#ifdef __cplusplus
}
#endif

