// Generated with umsg_gen on 2026-05-30
#include <umsg.h>
#include <umsg_guidance.h>

// msg instances
static umsg_msg_metadata_t msg_guidance_setpoints = {.name = "guidance_setpoints"};

// msg api's
// guidance_setpoints
umsg_sub_handle_t umsg_guidance_setpoints_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_guidance_setpoints, prescaler, sizeof(umsg_guidance_setpoints_t), length, 0);
}
umsg_sub_handle_t umsg_guidance_setpoints_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_guidance_setpoints, prescaler, sizeof(umsg_guidance_setpoints_t), length, channel);
}
void umsg_guidance_setpoints_publish(umsg_guidance_setpoints_t* data)
{
    umsg_publish(&msg_guidance_setpoints, data, 0);
}
void umsg_guidance_setpoints_publish_ch(umsg_guidance_setpoints_t* data, uint8_t channel)
{
    umsg_publish(&msg_guidance_setpoints, data, channel);
}
uint8_t umsg_guidance_setpoints_receive(umsg_sub_handle_t queue, umsg_guidance_setpoints_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
uint8_t umsg_guidance_setpoints_peek(umsg_guidance_setpoints_t* data)
{
    return umsg_peek(&msg_guidance_setpoints, data, sizeof(umsg_guidance_setpoints_t));
}

