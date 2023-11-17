#include <callbacks.hpp>

void control_callback(const void* request_msg, void* response_msg){
		racs_services__srv__Control_Request* req_in =
				(racs_services__srv__Control_Request*) request_msg;
		racs_services__srv__Control_Response* res_in =
				(racs_services__srv__Control_Response*) response_msg;
		//logica
	}

void setup_callback(const void* request_msg, void* response_msg){
		racs_services__srv__Setup_Request* req_in =
				(racs_services__srv__Setup_Request*) request_msg;
		racs_services__srv__Setup_Response* res_in =
				(racs_services__srv__Setup_Response*) response_msg;
		ScorBot.rcvSetup(req_in);
		ScorBot.sndSetup(res_in);
	}
