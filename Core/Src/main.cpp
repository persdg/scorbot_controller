/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "components.hpp"

#include <sys/time.h>
#include <dma_transport.h>

#include <racs_services/srv/control.h>
#include <racs_services/srv/setup.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef __cplusplus
 extern "C" {
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
#ifdef __cplusplus
}
#endif
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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


		res_in->response = 0b00000011;
	}

Robot create_robot() {

	PinControl mot1_ina = PinControl(MOTOR1_INA_GPIO_Port, MOTOR1_INA_Pin);
	PinControl mot1_inb = PinControl(MOTOR1_INB_GPIO_Port, MOTOR1_INB_Pin);
//	PinControl mot1_pwm = PinControl(MOTOR1_PWM_GPIO_Port, MOTOR1_PWM_Pin);
//	PinMeasure mot1_cha = PinMeasure(MOTOR1_CHA_GPIO_Port, MOTOR1_CHA_Pin);
//	PinMeasure mot1_chb = PinMeasure(MOTOR1_CHB_GPIO_Port, MOTOR1_CHB_Pin);
	PinMeasure mot1_end = PinMeasure(MOTOR1_END_GPIO_Port, MOTOR1_END_Pin);

	PinControl mot2_ina = PinControl(MOTOR2_INA_GPIO_Port, MOTOR2_INA_Pin);
	PinControl mot2_inb = PinControl(MOTOR2_INB_GPIO_Port, MOTOR2_INB_Pin);
//	PinControl mot2_pwm = PinControl(MOTOR2_PWM_GPIO_Port, MOTOR2_PWM_Pin);
//	PinMeasure mot2_cha = PinMeasure(MOTOR2_CHA_GPIO_Port, MOTOR2_CHA_Pin);
//	PinMeasure mot2_chb = PinMeasure(MOTOR2_CHB_GPIO_Port, MOTOR2_CHB_Pin);
	PinMeasure mot2_end = PinMeasure(MOTOR2_END_GPIO_Port, MOTOR2_END_Pin);

	PinControl mot3_ina = PinControl(MOTOR3_INA_GPIO_Port, MOTOR3_INA_Pin);
	PinControl mot3_inb = PinControl(MOTOR3_INB_GPIO_Port, MOTOR3_INB_Pin);
//	PinControl mot3_pwm = PinControl(MOTOR3_PWM_GPIO_Port, MOTOR3_PWM_Pin);
//	PinMeasure mot3_cha = PinMeasure(MOTOR3_CHA_GPIO_Port, MOTOR3_CHA_Pin);
//	PinMeasure mot3_chb = PinMeasure(MOTOR3_CHB_GPIO_Port, MOTOR3_CHB_Pin);
	PinMeasure mot3_end = PinMeasure(MOTOR3_END_GPIO_Port, MOTOR3_END_Pin);

	PinControl mot4_ina = PinControl(MOTOR4_INA_GPIO_Port, MOTOR4_INA_Pin);
	PinControl mot4_inb = PinControl(MOTOR4_INB_GPIO_Port, MOTOR4_INB_Pin);
//	PinControl mot4_pwm = PinControl(MOTOR4_PWM_GPIO_Port, MOTOR4_PWM_Pin);
//	PinMeasure mot4_cha = PinMeasure(MOTOR4_CHA_GPIO_Port, MOTOR4_CHA_Pin);
//	PinMeasure mot4_chb = PinMeasure(MOTOR4_CHB_GPIO_Port, MOTOR4_CHB_Pin);
	PinMeasure mot4_end = PinMeasure(MOTOR4_END_GPIO_Port, MOTOR4_END_Pin);

	PinControl mot5_ina = PinControl(MOTOR5_INA_GPIO_Port, MOTOR5_INA_Pin);
	PinControl mot5_inb = PinControl(MOTOR5_INB_GPIO_Port, MOTOR5_INB_Pin);
//	PinControl mot5_pwm = PinControl(MOTOR5_PWM_GPIO_Port, MOTOR5_PWM_Pin);
//	PinMeasure mot5_cha = PinMeasure(MOTOR5_CHA_GPIO_Port, MOTOR5_CHA_Pin);
//	PinMeasure mot5_chb = PinMeasure(MOTOR5_CHB_GPIO_Port, MOTOR5_CHB_Pin);
	PinMeasure mot5_end = PinMeasure(MOTOR5_END_GPIO_Port, MOTOR5_END_Pin);

	PinControl mot6_ina = PinControl(MOTOR6_INA_GPIO_Port, MOTOR6_INA_Pin);
	PinControl mot6_inb = PinControl(MOTOR6_INB_GPIO_Port, MOTOR6_INB_Pin);
//	PinControl mot6_pwm = PinControl(MOTOR6_PWM_GPIO_Port, MOTOR6_PWM_Pin);
//	PinMeasure mot6_cha = PinMeasure(MOTOR6_CHA_GPIO_Port, MOTOR6_CHA_Pin);
//	PinMeasure mot6_chb = PinMeasure(MOTOR6_CHB_GPIO_Port, MOTOR6_CHB_Pin);
	PinMeasure mot6_end = PinMeasure(MOTOR6_END_GPIO_Port, MOTOR6_END_Pin);

	PinControl enable = PinControl(MOTORS_EN_GPIO_Port, MOTORS_EN_Pin);
	PinControl toggle = PinControl(PIN_TOGGLE_GPIO_Port, PIN_TOGGLE_Pin);

	Motor motor1 = Motor(mot1_ina, mot1_inb, &htim1, 1, &htim2, mot1_end);
	Motor motor2 = Motor(mot2_ina, mot2_inb, &htim1, 2, &htim3, mot2_end);
	Motor motor3 = Motor(mot3_ina, mot3_inb, &htim1, 3, &htim4, mot3_end);
	Motor motor4 = Motor(mot4_ina, mot4_inb, &htim1, 4, &htim5, mot4_end);
	Motor motor5 = Motor(mot5_ina, mot5_inb, &htim9, 1, &htim8, mot5_end);
	Motor motor6 = Motor(mot6_ina, mot6_inb, &htim9, 2, mot6_end);

	Motor** motors = (Motor**) malloc(sizeof(Motor)*6);
	float* encs_div = (float*) malloc(sizeof(float)*6);

	motors[0] = &motor1; motors[1] = &motor2; motors[2] = &motor3; motors[3] = &motor4; motors[4] = &motor5; motors[5] = &motor6;
	encs_div[0] = 1; encs_div[1] = 1; encs_div[2] = 1; encs_div[3] = 1; encs_div[4] = 1; encs_div[5] = 1;

	Robot myRobot = Robot(enable, toggle, 10, 6, motors, encs_div);

//  Robot(PinControl &enable, PinControl &toggle, unsigned long ts_ms, uint8_t size, Motor **motors, float *encs_div);

	return myRobot;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
