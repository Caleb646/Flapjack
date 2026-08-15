// Generated with umsg_gen on 2026-08-15
#include <umsg.h>
#include <umsg_tune.h>

// msg instances
static umsg_msg_metadata_t msg_tune_pid = {.name = "tune_pid"};

// msg api's
// tune_pid
umsg_sub_handle_t umsg_tune_pid_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_tune_pid, prescaler, sizeof(umsg_tune_pid_t), length, 0);
}
umsg_sub_handle_t umsg_tune_pid_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_tune_pid, prescaler, sizeof(umsg_tune_pid_t), length, channel);
}
void umsg_tune_pid_publish(umsg_tune_pid_t* data)
{
    umsg_publish(&msg_tune_pid, data, 0);
}
void umsg_tune_pid_publish_ch(umsg_tune_pid_t* data, uint8_t channel)
{
    umsg_publish(&msg_tune_pid, data, channel);
}
uint8_t umsg_tune_pid_receive(umsg_sub_handle_t queue, umsg_tune_pid_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
