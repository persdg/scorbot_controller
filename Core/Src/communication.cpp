#include "communication.hpp"


// ==================================================
// Communication
// ==================================================
/*
 * RCVctrl:
 * Classe che gestisce il ricevimento di un messaggio di controllo
 */
Communication::RCVctrl::RCVctrl(uint8_t n){
	this->num = n;
	for (uint8_t i = 0; i < n; i++) {
	  values.push_back(0);
	}
}

Communication::RCVctrl::~RCVctrl(){

}
/*
 * RCVctrl:
 * Classe che gestisce l'invio di un messaggio di controllo
 */
Communication::SNDctrl::SNDctrl(unsigned char n){
  this->num = n;
  for (uint8_t i = 0; i < n; i++) {
    encoders.push_back(0);
  }
}

Communication::SNDctrl::~SNDctrl(){

}
/*
 * RCVctrl:
 * Classe che gestisce il ricevimento di un messaggio di setup
 */
Communication::RCVsetup::RCVsetup(){

}

Communication::RCVsetup::~RCVsetup(){

}
/*
 * RCVctrl:
 * Classe che gestisce l'invio di un messaggio di setup
 */
Communication::SNDsetup::SNDsetup(){

}

Communication::SNDsetup::~SNDsetup(){

}

/* FORSE NON SERVE
  static Communication::Next Communication::peek(){
  int res = (Serial.available() > 0) ? Serial.peek() : -1;

  if(res == -1) {
    return Next::None;
  } else {
    uint8_t byte = ((uint8_t) res) & 0b00011111;
    if(byte == 0 || byte == 1 || byte == 2){
      return Next::Ctrl;
    } else if(byte == 3) {
      return Next::Setup;
    } else {
      return Next::Error;
    }
  }
}*/

void Communication::rcv(RCVctrl *rcv_ctrl){
  /*
  byte = Serial.read();

  rcv_ctrl->num = (byte >> 5) + 1;
  rcv_ctrl->command = byte & 0b00011111;

  while(Serial.available() < 1);
  byte = Serial.read();

  for (int i = 0; i < rcv_ctrl->num; i++) {
    while(Serial.available() < 1);
    rcv_ctrl->values[i] = (1 - 2 *((byte >> i) & 0b00000001)) * ((short) Serial.read());
  }*/
}

void Communication::snd(SNDctrl *snd_ctrl){
  /*buffer[0] = (((uint8_t) (snd_ctrl->num - 1)) << 5) | (snd_ctrl->status & 0b00011111);
  buffer[1] = 0b00000000;
  buffer[2] = 0b00000000;

  for(int i = 0; i < snd_ctrl->num; i++) {
    buffer[1] = buffer[1] | (snd_ctrl->switches[i] ? (0b00000001 << i) : 0b00000000);
    buffer[2] = buffer[2] | ((snd_ctrl->values[i] < 0) ? (0b00000001 << i) : 0b00000000);
    buffer[3+i] = (uint8_t) min(abs(snd_ctrl->values[i]), 255);
  }

  Serial.write(buffer, 3+snd_ctrl->num);*/
}

void Communication::rcv(RCVsetup *rcv_setup){
  /*uint8_t byte;
  while(Serial.available() < 1);
  byte = Serial.read();

  rcv_setup->num = (byte >> 5);
  rcv_setup->command = byte & 0b00011111;

  for (int i = 0; i < 6; i++){
    rcv_setup->values[i] = 0b0;
    uint8_t bytes[4] = {0b0, 0b0, 0b0, 0b0};
    for (int j = 0; j < 4; j++){
      while(Serial.available() < 1);
      bytes[j] = Serial.read();
    }
    rcv_setup->values[i] = *((float *) bytes);
  }*/
}

void Communication::snd(SNDsetup *snd_setup){
  /*uint8_t byte = snd_setup->status & 0b00011111;
  Serial.write(byte);*/
}
