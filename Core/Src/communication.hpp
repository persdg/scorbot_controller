#ifndef COMMUNICATION_H
#define COMMUNICATION_H


//#include <Arduino.h> no
#include <math.h>
#include <vector>
#include "utils.hpp"
#include <cstdint>

// Serial Communication Protocol (static)
class Communication {
public:
  Communication() = delete;
  ~Communication() = delete;
  
  // Store received control message data
  struct RCVctrl {
    RCVctrl() = delete;
    RCVctrl(uint8_t n);
    ~RCVctrl();

    uint8_t num;
    uint8_t command;
    uint8_t directions;
    std::vector<uint8_t> values;
  };

  // Store sending control response data
  struct SNDctrl {
    SNDctrl() = delete;
    SNDctrl(uint8_t n);
    ~SNDctrl();

    uint8_t num;
    uint8_t status;
    uint8_t switches;
    uint8_t enc_directions;
    std::vector<uint8_t> encoders;
  };

  // Store received setup message data
  struct RCVsetup {
    RCVsetup();
    ~RCVsetup();

    uint8_t motor_index;
    float encoder_error_divider;
    float p;
    float i;
    float d;
    float tau;
    float sat;
  };

  // Store sending setup response data
  struct SNDsetup {
    SNDsetup();
    ~SNDsetup();

    unsigned char status;
  };
  
  // Describe next serial bytes kind of message
  enum class Next : int {
    Error = -1,
    None  = 0,
    Ctrl  = 1,
    Setup = 2
  };

//  static Next peek();                     // Check next byte without removing from serial buffer to return next message type

  static void rcv(RCVctrl *rcv_ctrl);     // Fill rcv_ctrl with incoming control message bytes
  static void snd(SNDctrl *snd_ctrl);     // Send snd_ctrl control response data through serial
  static void rcv(RCVsetup *rcv_setup);   // Fill rcv_setup with incoming setup message bytes
  static void snd(SNDsetup *snd_setup);   // Send snd_setup setup response data through serial

private:
  //inline static unsigned char buffer[16];
};


#endif  // COMMUNICATION_H
