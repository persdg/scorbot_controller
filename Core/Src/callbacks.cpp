#include <callbacks.hpp>

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
