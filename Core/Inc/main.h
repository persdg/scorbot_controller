/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR5_PWM_Pin GPIO_PIN_5
#define MOTOR5_PWM_GPIO_Port GPIOE
#define MOTOR6_PWM_Pin GPIO_PIN_6
#define MOTOR6_PWM_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define MOTOR4_CHA_Pin GPIO_PIN_0
#define MOTOR4_CHA_GPIO_Port GPIOA
#define MOTOR4_CHB_Pin GPIO_PIN_1
#define MOTOR4_CHB_GPIO_Port GPIOA
#define MOTOR1_CHA_Pin GPIO_PIN_5
#define MOTOR1_CHA_GPIO_Port GPIOA
#define MOTOR2_CHA_Pin GPIO_PIN_6
#define MOTOR2_CHA_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define MOTOR1_PWM_Pin GPIO_PIN_9
#define MOTOR1_PWM_GPIO_Port GPIOE
#define MOTOR2_PWM_Pin GPIO_PIN_11
#define MOTOR2_PWM_GPIO_Port GPIOE
#define MOTOR3_PWM_Pin GPIO_PIN_13
#define MOTOR3_PWM_GPIO_Port GPIOE
#define MOTOR4_PWM_Pin GPIO_PIN_14
#define MOTOR4_PWM_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define MOTOR3_CHA_Pin GPIO_PIN_12
#define MOTOR3_CHA_GPIO_Port GPIOD
#define MOTOR3_CHB_Pin GPIO_PIN_13
#define MOTOR3_CHB_GPIO_Port GPIOD
#define MOTOR5_INA_Pin GPIO_PIN_14
#define MOTOR5_INA_GPIO_Port GPIOD
#define MOTOR5_INB_Pin GPIO_PIN_15
#define MOTOR5_INB_GPIO_Port GPIOD
#define MOTOR6_INA_Pin GPIO_PIN_2
#define MOTOR6_INA_GPIO_Port GPIOG
#define MOTOR6_INB_Pin GPIO_PIN_3
#define MOTOR6_INB_GPIO_Port GPIOG
#define PIN_TOGGLE_Pin GPIO_PIN_5
#define PIN_TOGGLE_GPIO_Port GPIOG
#define MOTORS_EN_Pin GPIO_PIN_6
#define MOTORS_EN_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define MOTOR5_CHA_Pin GPIO_PIN_6
#define MOTOR5_CHA_GPIO_Port GPIOC
#define MOTOR5_CHB_Pin GPIO_PIN_7
#define MOTOR5_CHB_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MOTOR1_INA_Pin GPIO_PIN_0
#define MOTOR1_INA_GPIO_Port GPIOD
#define MOTOR1_INB_Pin GPIO_PIN_1
#define MOTOR1_INB_GPIO_Port GPIOD
#define MOTOR2_INA_Pin GPIO_PIN_2
#define MOTOR2_INA_GPIO_Port GPIOD
#define MOTOR2_INB_Pin GPIO_PIN_3
#define MOTOR2_INB_GPIO_Port GPIOD
#define MOTOR3_INA_Pin GPIO_PIN_4
#define MOTOR3_INA_GPIO_Port GPIOD
#define MOTOR3_INB_Pin GPIO_PIN_5
#define MOTOR3_INB_GPIO_Port GPIOD
#define MOTOR4_INA_Pin GPIO_PIN_6
#define MOTOR4_INA_GPIO_Port GPIOD
#define MOTOR4_INB_Pin GPIO_PIN_7
#define MOTOR4_INB_GPIO_Port GPIOD
#define MOTOR1_END_Pin GPIO_PIN_9
#define MOTOR1_END_GPIO_Port GPIOG
#define MOTOR2_END_Pin GPIO_PIN_10
#define MOTOR2_END_GPIO_Port GPIOG
#define MOTOR3_END_Pin GPIO_PIN_11
#define MOTOR3_END_GPIO_Port GPIOG
#define MOTOR4_END_Pin GPIO_PIN_12
#define MOTOR4_END_GPIO_Port GPIOG
#define MOTOR5_END_Pin GPIO_PIN_13
#define MOTOR5_END_GPIO_Port GPIOG
#define MOTOR6_END_Pin GPIO_PIN_14
#define MOTOR6_END_GPIO_Port GPIOG
#define MOTOR1_CHB_Pin GPIO_PIN_3
#define MOTOR1_CHB_GPIO_Port GPIOB
#define MOTOR2_CHB_Pin GPIO_PIN_5
#define MOTOR2_CHB_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// ============================================================
// Pins
// ============================================================

// DC Motors PINs
/*#define MOTORS_EN      		// Motors enabler

#define PORT_MOTOR_1_INA GPIOE      // Motor 1 spin direction
#define MOTOR_1_INB 23      // Motor 1 spin direction
#define MOTOR_1_PWM 5       // Motor 1 spin pwm
#define MOTOR_1_CHA A8      // Motor 1 encoder A channel
#define MOTOR_1_CHB A14     // Motor 1 encoder B channel
#define MOTOR_1_END 37      // Motor 1 endstop switch

#define MOTOR_2_INA 24      // Motor 2 spin direction
#define MOTOR_2_INB 25      // Motor 2 spin direction
#define MOTOR_2_PWM 2       // Motor 2 spin pwm
#define MOTOR_2_CHA A9      // Motor 2 encoder A channel
#define MOTOR_2_CHB A15     // Motor 2 encoder B channel
#define MOTOR_2_END 36      // Motor 2 endstop switch

#define MOTOR_3_INA 26      // Motor 3 spin direction
#define MOTOR_3_INB 27      // Motor 3 spin direction
#define MOTOR_3_PWM 3       // Motor 3 spin pwm
#define MOTOR_3_CHA A10     // Motor 3 encoder A channel
#define MOTOR_3_CHB 10      // Motor 3 encoder B channel
#define MOTOR_3_END 35      // Motor 3 endstop switch

#define MOTOR_4_INA 28      // Motor 4 spin direction
#define MOTOR_4_INB 29      // Motor 4 spin direction
#define MOTOR_4_PWM 6       // Motor 4 spin pwm
#define MOTOR_4_CHA A11     // Motor 4 encoder A channel
#define MOTOR_4_CHB 11      // Motor 4 encoder B channel
#define MOTOR_4_END 34      // Motor 4 endstop switch

#define MOTOR_5_INA 49      // Motor 5 spin direction
#define MOTOR_5_INB 48      // Motor 5 spin direction
#define MOTOR_5_PWM 7       // Motor 5 spin pwm
#define MOTOR_5_CHA A12     // Motor 5 encoder A channel
#define MOTOR_5_CHB 12      // Motor 5 encoder B channel
#define MOTOR_5_END 33      // Motor 5 endstop switch

#define MOTOR_6_INA 47      // Motor 6 spin direction
#define MOTOR_6_INB 46      // Motor 6 spin direction
#define MOTOR_6_PWM 8       // Motor 6 spin pwm
#define MOTOR_6_CHA A13     // Motor 6 encoder A channel
#define MOTOR_6_CHB 13      // Motor 6 encoder B channel
#define MOTOR_6_END 32      // Motor 6 endstop switch*/

// Other pins
// #define PIN_TOGGLE  52

// Control parameters

void control_callback(const void* request_msg, void* response_msg);
void setup_callback(const void* request_msg, void* response_msg);

#define MAX_ENC 65535
#define MAX_PWM 65535
#define HALF_ENC MAX_ENC/2
#define HALF_PWM MAX_PWM/2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
