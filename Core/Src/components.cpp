/*
 * COSE DA FARE:
 * Tutta la roba commentata de sotto
 * l'attributo error_div deve essere rimosso
 */

#include <components.hpp>

//#include <algorithm>


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
  //this->encoders_rcv = 	(int32_t*)	malloc(size * sizeof(int32_t));
  //this->encoders_snd = 	(int32_t*) 	malloc(size * sizeof(int32_t));
  this->encoders =		(int32_t*)	malloc(size * sizeof(int32_t));
  this->error_div = 	(float*) 	malloc(size * sizeof(float));

  this->size = size;
  this->status = Status::Idle;

  for(int i = 0; i < size; i++){
    this->switches[i] = false;
    this->motors_pwm[i] = 0;
    //this->encoders_snd[i] = 0;
    //this->encoders_rcv[i] = 0;
    this->encoders[i] = 0;
    this->error_div[i] = 0.0;
  }

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
  //free(this->encoders_rcv);
  //free(this->encoders_snd);
  free(this->encoders);
  free(this->error_div);
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

void Robot::rcvCtrl(racs_services__srv__Control_Request* request){

  if(request->command > 2 || request->num_motors != size) {
    setStatus(Status::Idle, true);
    return;
  }

  switch(request->command){
    case (unsigned char) Command::Idle:
      for(int i = 0; i < size; i++) {
        motors_pwm[i] = 0;
        //encoders_rcv[i] = 0;
        //encoders[i] = 0;
      }
      break;
    case (unsigned char) Command::DAQ:
      for(int i = 0; i < size; i++) {
        motors_pwm[i] = request->values.data[i];
        //encoders_rcv[i] = 0;
        encoders[i] = 0;
      }
      break;
    case (unsigned char) Command::PID:
      for(int i = 0; i < size; i++) {
        motors_pwm[i] = 0;
        //encoders_rcv[i] += request->values.data[i];
        encoders[i] = request->values.data[i];
      }
      break;
  }

  setStatus((Status) request->command);
}

void Robot::sndCtrl(racs_services__srv__Control_Response* response){

	response->response = (uint8_t) status;
}

void Robot::rcvSetup(racs_services__srv__Setup_Request* request){

  setEncoderDivider(request->motor_index, request->eed);
  getPID(request->motor_index)->reset();
  getPID(request->motor_index)->init((float) ts/1000.0, request->tau, request->sat, true);
  getPID(request->motor_index)->setup(request->p, request->i, request->d);

  setStatus(Status::Idle, true);
}

void Robot::sndSetup(racs_services__srv__Setup_Response* response){
  response->response = (uint8_t) Status::Setup;
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
        float err = (float) (getMotor(i)->getEncoder() - encoders[i]) / ((error_div[i] == 0) ? 1.0 : error_div[i]);
    	motors_pwm[i] = (int16_t) std::min(std::max((float) 0, getPID(i)->evolve(err)), (float) MAX_PWM) - HALF_PWM;
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

Robot create_robot() {

	PinControl mot1_ina = PinControl(MOTOR1_INA_GPIO_Port, MOTOR1_INA_Pin);
	PinControl mot1_inb = PinControl(MOTOR1_INB_GPIO_Port, MOTOR1_INB_Pin);
	PinMeasure mot1_end = PinMeasure(MOTOR1_END_GPIO_Port, MOTOR1_END_Pin);

	PinControl mot2_ina = PinControl(MOTOR2_INA_GPIO_Port, MOTOR2_INA_Pin);
	PinControl mot2_inb = PinControl(MOTOR2_INB_GPIO_Port, MOTOR2_INB_Pin);
	PinMeasure mot2_end = PinMeasure(MOTOR2_END_GPIO_Port, MOTOR2_END_Pin);

	PinControl mot3_ina = PinControl(MOTOR3_INA_GPIO_Port, MOTOR3_INA_Pin);
	PinControl mot3_inb = PinControl(MOTOR3_INB_GPIO_Port, MOTOR3_INB_Pin);
	PinMeasure mot3_end = PinMeasure(MOTOR3_END_GPIO_Port, MOTOR3_END_Pin);

	PinControl mot4_ina = PinControl(MOTOR4_INA_GPIO_Port, MOTOR4_INA_Pin);
	PinControl mot4_inb = PinControl(MOTOR4_INB_GPIO_Port, MOTOR4_INB_Pin);
	PinMeasure mot4_end = PinMeasure(MOTOR4_END_GPIO_Port, MOTOR4_END_Pin);

	PinControl mot5_ina = PinControl(MOTOR5_INA_GPIO_Port, MOTOR5_INA_Pin);
	PinControl mot5_inb = PinControl(MOTOR5_INB_GPIO_Port, MOTOR5_INB_Pin);
	PinMeasure mot5_end = PinMeasure(MOTOR5_END_GPIO_Port, MOTOR5_END_Pin);

	PinControl mot6_ina = PinControl(MOTOR6_INA_GPIO_Port, MOTOR6_INA_Pin);
	PinControl mot6_inb = PinControl(MOTOR6_INB_GPIO_Port, MOTOR6_INB_Pin);
	PinMeasure mot6_end = PinMeasure(MOTOR6_END_GPIO_Port, MOTOR6_END_Pin);

	PinControl enable = PinControl(MOTORS_EN_GPIO_Port, MOTORS_EN_Pin);
	PinControl toggle = PinControl(PIN_TOGGLE_GPIO_Port, PIN_TOGGLE_Pin);

	Motor motor1 = Motor(mot1_ina, mot1_inb, &htim1, 1, &htim2, mot1_end);
	Motor motor2 = Motor(mot2_ina, mot2_inb, &htim1, 2, &htim3, mot2_end);
	Motor motor3 = Motor(mot3_ina, mot3_inb, &htim1, 3, &htim4, mot3_end);
	Motor motor4 = Motor(mot4_ina, mot4_inb, &htim1, 4, &htim5, mot4_end);
	Motor motor5 = Motor(mot5_ina, mot5_inb, &htim9, 1, &htim8, mot5_end);
	Motor motor6 = Motor(mot6_ina, mot6_inb, &htim9, 2, 		mot6_end);

	Motor** motors = (Motor**) malloc(sizeof(Motor*)*6);
	float* encs_div = (float*) malloc(sizeof(float)*6);

	motors[0] = &motor1; motors[1] = &motor2; motors[2] = &motor3; motors[3] = &motor4; motors[4] = &motor5; motors[5] = &motor6;
	encs_div[0] = 1; encs_div[1] = 1; encs_div[2] = 1; encs_div[3] = 1; encs_div[4] = 1; encs_div[5] = 1;

	Robot myRobot = Robot(enable, toggle, 10, 6, motors, encs_div);

	return myRobot;
}

Robot ScorBot = create_robot();
