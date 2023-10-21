/*
 * COSE DA FARE:
 * Tutta la roba commentata de sotto
 * l'attributo error_div deve essere rimosso
 */

#include "components.hpp"

#include "utils.hpp"
#include <algorithm>


// ==================================================
// PWMfreq
// ==================================================
// ==================================================
// PinControl
// ==================================================

PinControl::PinControl() {

}

PinControl::PinControl(GPIO_TypeDef* port, uint16_t pin){
  this->port = port;
  this->pin = pin;
  //tolta la linea per impostare il pin ad output perché STM32 lo fa da sé, se settato correttamente
  setLimits(0.0, 0.0);
}

PinControl::PinControl(GPIO_TypeDef* port, uint16_t pin, float v1, float v2){
  this->port = port;
  this->pin = pin;
  setLimits(v1, v2);
}

void PinControl::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

void PinControl::set(bool state){
  HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * Probabilmente non serve, occorre fare qualcosa di diverso per controllare le pwm
 * Forse questa cosa da fare va aggiunta alla classe Motor
 *
 void PinControl::pwm(uint8_t pwm){
  switch(pwm) {
    case 1:
      TIM1->CCR1 = pwm;
      break;
    case 2:
      TIM1->CCR2 = pwm;
      break;
    case 3:
      TIM1->CCR3 = pwm;
      break;
    case 4:
      TIM1->CCR4 = pwm;
      break;
    case 5:
      TIM9->CCR1 = pwm;
      break;
    case 6:
      TIM9->CCR2 = pwm;
      break;
  }
}
void PinControl::control(float value){
  pwm(remap(value, v1, v2, 0l, 255l, true));
}
*/
#if defined(PIN_EXTRA_FEATURES)
/*
void PinControl::feedback(float error){
  control(pid->evolve(error));
}
*/
void PinControl::setPID(PID* pid){
  this->pid = pid;
}

PID* PinControl::getPID(){
  return this->pid;
}
#endif


// ==================================================
// PinMeasure
// ==================================================

PinMeasure::PinMeasure() {

}

PinMeasure::PinMeasure(GPIO_TypeDef* port, uint16_t pin){
  this->port = port;
  this->pin = pin;
  setLimits(0.0, 0.0);
}

PinMeasure::PinMeasure(GPIO_TypeDef* port, uint16_t pin, float v1, float v2){
  //rimossa l'opzione per fare i pin pullup (mi serviva?)
  this->port = port;
  this->pin = pin;
  setLimits(v1, v2);
}

void PinMeasure::setLimits(float v1, float v2){
  this->v1 = v1;
  this->v2 = v2;
}

bool PinMeasure::state(){
  return HAL_GPIO_ReadPin(port, pin);
}
/*
uint16_t PinMeasure::value(){
  //return analogRead(pin); DA CAMBIARE
}

float PinMeasure::measure(){
  //return remap((long) value(), 0l, 1023l, v1, v2); CAPIRE COSA FA E SE DEVO MODIFICARLO
}

#if defined(PIN_CONTROL_FEATURES)
float PinMeasure::filter(){
	//CAPIRE COSA FA STA ROBA QUA SOTTO
  return (fil != NULL) ? fil->evolve(measure()) : measure();
}

void PinMeasure::setFilter(Filter *filter){
	//NO DAVVERO COSA SONO QUESTI FILTRI?
  this->fil = filter;
}

Filter* PinMeasure::getFilter(){
	//IIIIIIIIIIIIIIIH
  return this->fil;
}
#endif
*/

// ==================================================
// Motor
// ==================================================

Motor::Motor(PinControl &INA, PinControl &INB,
			 TIM_HandleTypeDef* htimPWM, uint8_t CCRx,
			 PinMeasure &END)
  : pin_INA(INA), pin_INB(INB), htimPWM(htimPWM), CCRx(CCRx), pin_END(END) {}

Motor::Motor(PinControl &INA, PinControl &INB,
			 TIM_HandleTypeDef* htimPWM, uint8_t CCRx,
			 TIM_HandleTypeDef* htimENC, PinMeasure &END)
  : pin_INA(INA), pin_INB(INB), htimPWM(htimPWM), CCRx(CCRx), htimENC(htimENC), pin_END(END) {}

Motor::~Motor() {}

/*void Motor::invertEncoder(bool invert){
  this->encoder_invert = invert;
}*/

long Motor::getEncoder(){
  return htimENC->Instance->CNT;
}

/*void Motor::setEncoder(long value){
  this->encoder = value;
}

void Motor::readEncoder(){
  enc_A = pin_CHA.state();
  enc_B = pin_CHB.state();
}

void Motor::updateEncoder(){
  bool old_A = enc_A;
  readEncoder();
  if(old_A != enc_A){
    if(enc_B == enc_A) {
      encoder_invert ? encoder-- : encoder++;
    } else {
      encoder_invert ? encoder++ : encoder--;
    }
  }
}
*/
void Motor::invertMotor(bool invert){
  this->motor_invert = invert;
}
void Motor::driveMotor(int16_t spwm){
  OperatingMode mode = OperatingMode::BRAKE_GND;
  spwm = std::min(std::max(spwm, (int16_t)-255), (int16_t)255);

  if(spwm > 0) {
    mode = motor_invert ? OperatingMode::SPIN_CCW : OperatingMode::SPIN_CW;
  } else if (spwm < 0) {
    mode = motor_invert ? OperatingMode::SPIN_CW : OperatingMode::SPIN_CCW;
  } else {
    mode = OperatingMode::BRAKE_GND;
  }

  switch(mode){
    case OperatingMode::BRAKE_GND:
      pin_INA.set(false);
      pin_INB.set(false);
      break;
    case OperatingMode::SPIN_CCW:
      pin_INA.set(false);
      pin_INB.set(true);
      break;
    case OperatingMode::SPIN_CW:
      pin_INA.set(true);
      pin_INB.set(false);
      break;
    case OperatingMode::BRAKE_VCC:
      pin_INA.set(true);
      pin_INB.set(true);
      break;
  }
  switch(CCRx) {
    case 1:
      htimPWM->Instance->CCR1 = (uint16_t) abs(spwm);
      break;
    case 2:
      htimPWM->Instance->CCR2 = (uint16_t) abs(spwm);
      break;
    case 3:
      htimPWM->Instance->CCR3 = (uint16_t) abs(spwm);
      break;
    case 4:
      htimPWM->Instance->CCR4 = (uint16_t) abs(spwm);
      break;
    case 5:
      htimPWM->Instance->CCR5 = (uint16_t) abs(spwm);
      break;
    case 6:
      htimPWM->Instance->CCR6 = (uint16_t) abs(spwm);
      break;

  }
}

bool Motor::isInEndStop(){
  return pin_END.state();
}


// ==================================================
// Robot
// ==================================================

Robot::Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors, float *encs_div)
  : pin_enable(enable), pin_toggle(toggle) {
  this->ts = ts_ms;
  this->motors = 		(Motor**)	malloc(size * sizeof(Motor*));
  this->pids = 			(PID*)		malloc(size * sizeof(PID));
  this->switches = 		(bool*) 	malloc(size * sizeof(bool));
  this->motors_pwm = 	(int16_t*) 	malloc(size * sizeof(int16_t));
  this->encoders_rcv = 	(long*)		malloc(size * sizeof(long));
  this->encoders_snd = 	(long*) 	malloc(size * sizeof(long));
  this->error_div = 	(float*) 	malloc(size * sizeof(float));

  this->size = size;
  this->status = Status::Idle;

  for(int i = 0; i < size; i++){
    this->switches[i] = false;
    this->motors_pwm[i] = 0;
    this->encoders_snd[i] = 0;
    this->encoders_rcv[i] = 0;
    this->error_div[i] = 0.0;
  }

  this->snd_ctrl = new Communication::SNDctrl(size);
  this->rcv_ctrl = new Communication::RCVctrl(size);
  this->snd_setup = new Communication::SNDsetup();
  this->rcv_setup = new Communication::RCVsetup();

  if(motors != NULL){
    for(int i = 0; i < size; i++){
      if(encs_div != NULL){
        setMotor(i, motors[i], encs_div[i]);
      } else {
        setMotor(i, motors[i]);
      }
    }
  }

  timer.setup(ts_ms);
  update();
}

Robot::Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors)
  : Robot(enable, toggle,ts_ms, size, motors, NULL) {}

Robot::Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size)
  : Robot(enable, toggle,ts_ms, size, NULL, NULL) {}

Robot::~Robot() {
  free(this->motors);
  free(this->pids);
  free(this->switches);
  free(this->motors_pwm);
  free(this->encoders_rcv);
  free(this->encoders_snd);
  free(this->error_div);
  delete this->snd_ctrl;
  delete this->rcv_ctrl;
  delete this->snd_setup;
  delete this->rcv_setup;
}

int Robot::getSize(){
  return this->size;
}

Robot::Status Robot::getStatus(){
  return this->status;
}

void Robot::setStatus(Status status, bool reset = false){
  if(this->status != status || reset){
    resetPWMs();
    resetPIDs();
    this->status = status;
  }
}

Motor * Robot::getMotor(uint8_t index){
  return this->motors[index];
}

void Robot::setMotor(uint8_t index, Motor * motor){
  this->motors[index] = motor;
}

void Robot::setMotor(uint8_t index, Motor * motor, float enc_div){
  this->motors[index] = motor;
  this->error_div[index] = enc_div;
}

void Robot::setEncoderDivider(uint8_t index, float enc_div){
  this->error_div[index] = enc_div;
}

PID * Robot::getPID(uint8_t index){
  return &(this->pids[index]);
}

void Robot::initPIDs(float ts, float pole, float sat, bool bumpless){
  for(int i = 0; i < size; i++){
    getPID(i)->init(ts, pole, sat, bumpless);
  }
}

void Robot::setupPIDs(float kp, float ki, float kd){
  for(int i = 0; i < size; i++){
    getPID(i)->setup(kp, ki, kd);
  }
}

void Robot::resetPIDs(){
  for(int i = 0; i < size; i++){
    getPID(i)->reset();
  }
}

/*void Robot::updateEncoders(){
  for(int i = 0; i < size; i++){
    getMotor(i)->updateEncoder();
  }
}*/

/*void Robot::setEncoders(long *values){
  for(int i = 0; i < size; i++){
    setEncoder(i, values[i]);
  }
}

void Robot::setEncoder(uint8_t index, long value){
  getMotor(index)->setEncoder(value);
}

void Robot::resetEncoders(){
  for(int i = 0; i < size; i++){
    setEncoder(i, 0);
  }
}*/


void Robot::setPWMs(int16_t *pwms){
  for(int i = 0; i < size; i++){
    setPWM(i, pwms[i]);
  }
}

void Robot::setPWM(uint8_t index, int16_t pwm){
  motors_pwm[index] = pwm;

}

void Robot::resetPWMs(){
  for(int i = 0; i < size; i++){
    motors_pwm[i] = 0;
  }
}

/* DA RIFARE
 * void Robot::enableMotors(){
  setStatus(Status::Idle, true);
  pin_enable.set(true);
}

void Robot::disableMotors(){
  setStatus(Status::Idle, true);
  pin_enable.set(false);
}*/

/*Communication::Next Robot::peek(){
  return Communication::peek();
}*/

void Robot::rcvCtrl(racs_services__srv__Control_Request* request){

  if(request->command > 2 || request->num_motors != size) {
    setStatus(Status::Idle, true);
    return;
  }

  switch(request->command){
    case (unsigned char) Command::Idle:
      for(int i = 0; i < size; i++) {
        motors_pwm[i] = 0;
        encoders_rcv[i] = 0;
      }
      break;
    case (unsigned char) Command::DAQ:
      for(int i = 0; i < size; i++) {
        motors_pwm[i] = request->values.data[i];
        encoders_rcv[i] = 0;
      }
      break;
    case (unsigned char) Command::PID:
      for(int i = 0; i < size; i++) {
        motors_pwm[i] = 0;
        encoders_rcv[i] += request->values.data[i];
      }
      break;
  }

  setStatus((Status) request->command);
}

void Robot::sndCtrl(racs_services__srv__Control_Response* response){
  response->num_motors = size;
  response->status = (uint8_t) status;
  response->ends = 0;
  response->enc_directions = 0;

  for(int i = 0; i < size; i++) {
    response->ends |= switches[i] * (1 << i);
    response->encoders.data[i] = std::min(abs(getMotor(i)->getEncoder() - encoders_snd[i]), 255L);
    encoders_snd[i] += response->encoders.data[i];
    response->enc_directions |= sgn(encoders_snd[i]) * (1 << i);
  }

  //Communication::snd(snd_ctrl);
}

void Robot::rcvSetup(racs_services__srv__Setup_Request* request){
  //Communication::rcv(rcv_setup);

  setEncoderDivider(request->motor_index, request->encoder_error_divider);
  getPID(request->motor_index)->reset();
  getPID(request->motor_index)->init((float) ts/1000.0, request->tau, request->sat, true);
  getPID(request->motor_index)->setup(request->p, request->i, request->d);

  setStatus(Status::Idle, true);
}

void Robot::sndSetup(racs_services__srv__Setup_Response* response){
  response->response = (unsigned char) Status::Setup;
  //Communication::snd(snd_setup);
}

void Robot::update(){
  switch(status){
    case Status::Idle:
      resetPWMs();
      break;

    case Status::DAQ:
      break;

    case Status::PID:
      for(int i = 0; i < size; i++){
    	//RIMUOVERE ERROR_DIV
        //float err = (float) (getMotor(i)->getEncoder() - encoders_rcv[i]) / ((error_div[i] == 0) ? 1.0 : error_div[i]);
    	float err = (float) (getMotor(i)->getEncoder() - encoders_rcv[i]);
    	motors_pwm[i] = (short) std::min(std::max(getPID(i)->evolve(err), (float) -255.0), (float) 255.0);
      }
      break;

    default:
      resetPWMs();
      resetPIDs();
      status = Status::Idle;
      break;
  }

  for(int i = 0; i < size; i++){
    switches[i] = getMotor(i)->isInEndStop();
  }
}

void Robot::actuate(){
  for(int i = 0; i < size; i++){
    getMotor(i)->driveMotor(motors_pwm[i]);
  }
}

/*void Robot::cycle(unsigned long time_ms){
  updateEncoders();
  //Communication::Next next = peek(); da cancellare?

  if(next == Communication::Next::Setup){
    //rcvSetup();
    //sndSetup();
    timer.reset(time_ms);
  } else if (next == Communication::Next::Error) {
    //Serial.flush(); da cambiare, non c'è il seriale
    setStatus(Robot::Status::Idle, true);
    timer.reset(time_ms);
  } else {
    if(getStatus() == Robot::Status::Idle && next == Communication::Next::Ctrl){
      pin_toggle.set(true);
      //rcvCtrl(); buggone?
      update();
      actuate();
      pin_toggle.set(false);
      timer.reset(time_ms);
      if(getStatus() == Robot::Status::Idle) {
        //sndCtrl();
      }
    } else if (getStatus() != Robot::Status::Idle) {
      if (timer.check(time_ms)){
        pin_toggle.set(true);
        //sndCtrl();
        //rcvCtrl();
        update();
        actuate();
        pin_toggle.set(false);
        if(getStatus() == Robot::Status::Idle){
          //sndCtrl();
        }
      }
    }
  }
}*/

