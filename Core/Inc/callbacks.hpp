#ifndef CALLBACKS_HPP_
#define CALLBACKS_HPP_

#include <components.hpp>
#include <racs_services/srv/control.h>
#include <racs_services/srv/setup.h>
#include <utils.hpp>

extern Robot ScorBot;

#ifdef __cplusplus
extern "C" {
#endif

void control_callback(const void* request_msg, void* response_msg);
void setup_callback(const void* request_msg, void* response_msg);

#ifdef __cplusplus
}
#endif

#endif
