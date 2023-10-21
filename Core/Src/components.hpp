/*
 * COSE DA FARE:
 * -Includere utils.h e control.h appena li faccio
 */

#ifndef COMPONENTS_H
#define COMPONENTS_H

//#define PIN_EXTRA_FEATURES

#include <cstdint>
#include <math.h>
#include "utils.hpp"
#include "control.hpp"
#include "communication.hpp"
#include "main.h"
#include <racs_services/srv/control.h>
#include <racs_services/srv/setup.h>

class PinControl {
public:
  PinControl();
  PinControl(GPIO_TypeDef* port, uint16_t pin);
  PinControl(GPIO_TypeDef* port, uint16_t pin, float v1, float v2);

  void setLimits(float v1, float v2);

  void set(bool state);
  void pwm(uint8_t pwm);
  void control(float value);

  #if defined(PIN_EXTRA_FEATURES)
  void setPID(PID* pid);
  PID* getPID();
  void feedback(float error);
  #endif


private:
  GPIO_TypeDef* port;
  uint8_t pin;
  float v1;
  float v2;
  #if defined(PIN_EXTRA_FEATURES)
  PID* pid;
  #endif
};


class PinMeasure {
public:
  PinMeasure();
  PinMeasure(GPIO_TypeDef* port, uint16_t pin);
  PinMeasure(GPIO_TypeDef* port, uint16_t pin, float v1, float v2);

  void setLimits(float v1, float v2);

  bool state();
  uint16_t value();
  float measure();
  #if defined(PIN_EXTRA_FEATURES)
  float filter();

  void setFilter(Filter *filter);
  Filter* getFilter();
  #endif

private:
  GPIO_TypeDef* port;
  uint16_t pin;

  float v1;
  float v2;
  #if defined(PIN_EXTRA_FEATURES)
  Filter* fil;
  #endif
};


// DC Motor with Encoder
class Motor{
public:
  Motor(PinControl &INA, PinControl &INB,
		TIM_HandleTypeDef* htimPWM, uint8_t CCRx,
		PinMeasure &END);
  Motor(PinControl &INA, PinControl &INB,
  		TIM_HandleTypeDef* htimPWM, uint8_t CCRx,
		TIM_HandleTypeDef* htimENC, PinMeasure &END);
  ~Motor();

  // Motor operating modes
  enum class OperatingMode {
    BRAKE_GND,
    SPIN_CCW,
    SPIN_CW,
    BRAKE_VCC
  };

  //void invertEncoder(bool invert);  // Invert encoder ticks counting direction
  long getEncoder();                // Return encoder ticks
  //void setEncoder(long value);      // Set/reset encoder ticks
  //void readEncoder();               // Read encoder pins
  //void updateEncoder();             // Update encoder ticks

  void invertMotor(bool invert);    // Invert physical spin direction of the motor
  void driveMotor(short spwm);      // Assign pwm with sign for spin direction

  bool isInEndStop();               // Check if motor is at endstop

private:
  PinControl &pin_INA;          // A pin
  PinControl &pin_INB;          // B pin
  TIM_HandleTypeDef* htimPWM;	// Handle del Timer relativo al motore
  uint8_t CCRx;					// Indice del Timer relativo al motore
  TIM_HandleTypeDef* htimENC;	// Handle del Timer relativo all'encoder
  PinMeasure &pin_END;          // endstop switch

  /*
  bool enc_A = false;           // Encoder channel A state
  bool enc_B = false;           // Encoder channel B state

  bool encoder_invert = false;  // If invert encoder counting direction
  long encoder = 0;             // Encoder ticks count
  */
  bool motor_invert = false;    // If invert motor physical spin direction
};


// 1-8 DoF Robot with DC Motors
class Robot {
public:
  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors, float *encs_div);
  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors);
  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size);
  ~Robot();

  // Robot possible commands
  enum class Command : uint8_t {
    Idle = 0,
    DAQ = 1,
    PID = 2,
    Setup = 3
  };

  // Robot possible statuses
  enum class Status : uint8_t {
    Idle = 0,
    DAQ = 1,
    PID = 2,
    Setup = 3
  };

  int getSize();                                      // Return number of motors
  Status getStatus();                                 // Return robot status
  void setStatus(Status status, bool reset);  // Set robot internal status

  Motor * getMotor(uint8_t index);                              // Return pointer to motor of specified index
  void setMotor(uint8_t index, Motor * motor);                  // Set robot's motor of specified index
  void setMotor(uint8_t index, Motor * motor, float enc_div);   // Set robot's motor of specified index and relative encoder divider
  void setEncoderDivider(uint8_t index, float enc_div);         // Set motor's encoder divider for specified index

  PID * getPID(uint8_t index);                                    // Return pointer to motor's PID specified by index
  void initPIDs(float ts, float pole, float sat, bool bumpless);  // Initialize all PIDs with following parameters
  void setupPIDs(float kp, float ki, float kd);                   // Initialize all PIDs with following coefficients
  void resetPIDs();                                               // Reset all PIDs state

  void updateEncoders();                        // Update all motors' encoders
  void setEncoders(long *values);               // Set encoders' values
  void setEncoder(uint8_t index, long value);   // Set encoder value for motor specified by index
  void resetEncoders();                         // Reset all motors' encoders

  void setPWMs(short *pwms);                    // Set motors' PWMs values
  void setPWM(uint8_t index, int16_t pwm);     // Set PWM value for motor specified by index
  void resetPWMs();                             // Reset all motors' PWMs values to zero

  void enableMotors();                          // Enable all motors
  void disableMotors();                         // Disable all motors

//  Communication::Next peek();         						// Check next message type
  void rcvCtrl(racs_services__srv__Control_Request* request);     // Receive and process a control message from serial communication
  void sndCtrl(racs_services__srv__Control_Response* response);	// Make and send a control response to serial communication
  void rcvSetup(racs_services__srv__Setup_Request* request);                    							// Receive and process a setup message from serial communication
  void sndSetup(racs_services__srv__Setup_Response* response);                    							// Make and send a setup response to serial communication

  void update();                      // Update robot internal state according to status and received data
  void actuate();                     // Apply computed PWMs
  void cycle(unsigned long time_ms);  // Looping function


private:
  PinControl &pin_enable; // Pin for motors enabling/disabling
  PinControl &pin_toggle; // Pin for motors enabling/disabling

  unsigned long ts;       // Time sampling in milliseconds
  unsigned char size;     // Number of motors
  Motor **motors;         // Pointers array to Motors
  PID *pids;              // Pointer to motors' PIDs

  Status status;          // Robot status
  Timer timer;            // Robot cycle timer

  bool *switches;         // Motors' endstops switches values
  int16_t *motors_pwm;      // Motors' PWMs current values

  long *encoders_snd;     // Cumulative encoders values sent
  long *encoders_rcv;     // Cumulative encoders values received
  float *error_div;       // Encoders error dividers

  Communication::SNDctrl *snd_ctrl;     // Control message data
  Communication::RCVctrl *rcv_ctrl;     // Control response data
  Communication::SNDsetup *snd_setup;   // Setup message data
  Communication::RCVsetup *rcv_setup;   // Setup response data
};


#endif  // COMPONENTS_H
