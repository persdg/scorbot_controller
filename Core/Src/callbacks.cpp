#include <callbacks.hpp>

void encoder_callback(const void* msgin) { //5 ms di ricezione
	const racs_services__msg__Encoder* enc_msg =
			(const racs_services__msg__Encoder*) msgin;

	ScorBot.toggle(true);
	ScorBot.setEncoders(enc_msg->encoders);
	ScorBot.setStatus(Robot::Status::PID, false);
	ScorBot.toggle(false);
}

void pwm_callback(const void* msgin) {
	const racs_services__msg__DirectAccess* pwm_msg =
		(const racs_services__msg__DirectAccess*) msgin;

	ScorBot.setPWMs(pwm_msg->pwms);
	ScorBot.setStatus(Robot::Status::DAQ, false);
}

void control_callback(const void* request_msg, void* response_msg){
		racs_services__srv__Control_Request* req_in =
				(racs_services__srv__Control_Request*) request_msg;
		racs_services__srv__Control_Response* res_in =
				(racs_services__srv__Control_Response*) response_msg;
		ScorBot.rcvCtrl(req_in);
		ScorBot.sndCtrl(res_in);
	}

void setup_callback(const void* request_msg, void* response_msg){
		racs_services__srv__Setup_Request* req_in =
				(racs_services__srv__Setup_Request*) request_msg;
		racs_services__srv__Setup_Response* res_in =
				(racs_services__srv__Setup_Response*) response_msg;
		ScorBot.rcvSetup(req_in);
		ScorBot.sndSetup(res_in);
	}

void feedback_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	//UNUSED(timer);
	UNUSED(last_call_time);

	if (timer != NULL) {
		racs_services__msg__Feedback feedback;
		racs_services__msg__Debug debug;
		rcl_ret_t rc;

		for(uint8_t i = 0; i < 5; i++) {
			feedback.encoders[i] = ScorBot.getEncoder(i);
		}

		rc = rcl_publish(&feedback_publisher, &feedback, NULL);
		if (rc != RCL_RET_OK) return;

		for(uint8_t i = 0; i < 1; i++) {
			ScorBot.getPID(0)->show(i, debug);
		}
		rc = rcl_publish(&debug_publisher, &debug, NULL);
		if (rc != RCL_RET_OK) return;
	}
}

void robot_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	UNUSED(last_call_time);
	if (timer != NULL) {
		ScorBot.cycle();
	}
} // 70 us

void encoder_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	UNUSED(last_call_time);
	racs_services__msg__Encoder enc_msg;
	rcl_ret_t rc;
	static uint16_t i = 0;
	enc_msg.encoders[0] = encs[i];
	i = (i >= 1999) ? 0 : i+1;
	for (int i = 1; i < 5; i++) {
		enc_msg.encoders[i] = 0;
	}
	if (timer != NULL) {
		rc = rcl_publish(&encoder_publisher, &enc_msg, NULL);
		if (rc != RCL_RET_OK) return;
	}
} // 3 ms
