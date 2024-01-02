#ifndef CALLBACKS_HPP_
#define CALLBACKS_HPP_

#include <components.hpp>
#include <racs_services/srv/control.h>
#include <racs_services/srv/setup.h>
#include <racs_services/msg/feedback.h>
#include <racs_services/msg/direct_access.h>
#include <rcl/rcl.h>
#include <rcl/timer.h>
#include <utils.hpp>

extern Robot ScorBot;
extern rcl_publisher_t feedback_publisher;

#ifdef __cplusplus
extern "C" {
#endif

void pwm_callback(const void* msgin);
void control_callback(const void* request_msg, void* response_msg);
void setup_callback(const void* request_msg, void* response_msg);
void feedback_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void robot_timer_callback(rcl_timer_t* timer, int64_t last_call_time);

#ifdef __cplusplus
}
#endif

#endif
