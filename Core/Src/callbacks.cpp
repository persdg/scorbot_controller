#include <callbacks.hpp>

void pwm_callback(const void* msgin) {
	const racs_services__msg__DirectAccess* pwm_msg =
		(const racs_services__msg__DirectAccess*) msgin;

	if(pwm_msg->num_motors != ScorBot.getSize()) {
	    ScorBot.setStatus(Robot::Status::Idle, true);
	    return;
	}

	ScorBot.setPWMs(pwm_msg->pwms);
	ScorBot.setStatus(Robot::Status::DAQ, false);
}

void control_callback(const void* request_msg, void* response_msg){
		racs_services__srv__Control_Request* req_in =
				(racs_services__srv__Control_Request*) request_msg;
		racs_services__srv__Control_Response* res_in =
				(racs_services__srv__Control_Response*) response_msg;
		ScorBot.lastEvent = getCurrentTime();
		ScorBot.rcvCtrl(req_in);
		ScorBot.sndCtrl(res_in);
	}

void setup_callback(const void* request_msg, void* response_msg){
		racs_services__srv__Setup_Request* req_in =
				(racs_services__srv__Setup_Request*) request_msg;
		racs_services__srv__Setup_Response* res_in =
				(racs_services__srv__Setup_Response*) response_msg;
		ScorBot.lastEvent = getCurrentTime();
		ScorBot.rcvSetup(req_in);
		ScorBot.sndSetup(res_in);
	}

void feedback_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	//UNUSED(timer);
	UNUSED(last_call_time);

	if (timer != NULL) {
		racs_services__msg__Feedback feedback;
		rcl_ret_t rc;

		int size = ScorBot.getSize();
		feedback.num_motors = size;
		for(uint8_t i = 0; i < size; i++) {
			feedback.encoders[i] = ScorBot.getEncoder(i);
		}

		for(uint8_t i = 6; i > size; i--) {
			feedback.encoders[i] = 0;
		}
		rc = rcl_publish(&feedback_publisher, &feedback, NULL);
		if (rc != RCL_RET_OK) return;
	}
}

void robot_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	//UNUSED(timer);
	UNUSED(last_call_time);
	if (timer != NULL) {
		ScorBot.cycle(0);	//0 è un numero qualunque, probabilmente cycle verrà
							//cambiata e non accetterà più input
	}

}
