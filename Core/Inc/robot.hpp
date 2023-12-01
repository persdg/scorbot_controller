#ifndef COMPONENTS_HPP_
#define COMPONENTS_HPP_

#include "components.hpp"

PinControl INA1;
PinControl INA2, INA3, INA4, INA5, INA6;
PinControl INB1, INB2, INB3, INB4, INB5, INB6;
PinMeasure END1, END2, END3, END4, END5;

TIM_HandleTypeDef* htimPW1, htimPW2, htimPW3, htimPW4, htimPW5, htimPW6;
TIM_HandleTypeDef* htimENC1, htimENC2, htimENC3, htimENC4, htimENC5;
uint8_t CCRx1, CCRx2, CCRx3, CCRx4, CCRx5, CCRx6;

GPIO_TypeDef* Port1A, Port2A, Port3A, Port4A, Port5A, Port6A;
GPIO_TypeDef* Port1B, Port2B, Port3B, Port4B, Port5B, Port6B;

uint16_t Pin1A, Pin2A, Pin3A, Pin4A, Pin5A, Pin6A;
uint16_t Pin1B, Pin2B, Pin3B, Pin4B, Pin5B, Pin6B;


#endif
