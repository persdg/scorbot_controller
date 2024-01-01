/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <racs_services/srv/control.h>
#include <racs_services/srv/setup.h>
#include <racs_services/msg/feedback.h>
#include <racs_services/msg/direct_access.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/transport.h>

#include <stdbool.h>
#include "usart.h"
#include <dma_transport.h>
#include <micro_ros_allocators.h>
#include <components.hpp>
#include <callbacks.hpp>
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
/* USER CODE BEGIN Variables */
extern Robot ScorBot;
rcl_publisher_t feedback_publisher;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cycleRobot */
osThreadId_t cycleRobotHandle;
const osThreadAttr_t cycleRobot_attributes = {
  .name = "cycleRobot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void cycleRobotTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of cycleRobot */
  cycleRobotHandle = osThreadNew(cycleRobotTask, NULL, &cycleRobot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	rmw_uros_set_custom_transport(
		true,
		(void *) &huart3,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) return;

	rcl_ret_t rc;
	rcl_node_t node; // nodo;
	rcl_timer_t timer;
	const unsigned int timer_period = RCL_MS_TO_NS(1000);
	//rcl_publisher_t feedback_publisher; // publisher
	rcl_subscription_t subscriber;
	rcl_service_t setup_service, control_service; //servizi

	const char* feedback_publisher_name = "/feedback";	//publisher
	const char* pwm_subscriber_name = "/pwm"			//subscriber
	const char* setup_service_name = "/setup";			//servizi
	const char* control_service_name = "/control";

	const rosidl_message_type_support_t* feedback_type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(racs_services, msg, Feedback);
	const rosidl_message_type_support_t * pwm_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(racs_services, msg, DirectAccess);
	const rosidl_service_type_support_t* setup_type_support =
		ROSIDL_GET_SRV_TYPE_SUPPORT(racs_services, srv, Setup);
	const rosidl_service_type_support_t* control_type_support =
		ROSIDL_GET_SRV_TYPE_SUPPORT(racs_services, srv, Control);
	rclc_support_t support;// support_p;
	rcl_allocator_t allocator;// allocator_p;

	racs_services__msg__DirectAccess pwm_msg;

	racs_services__srv__Setup_Request req_setup;
	racs_services__srv__Setup_Response res_setup;
	racs_services__srv__Control_Request req_control;
	racs_services__srv__Control_Response res_control;

	allocator = rcl_get_default_allocator();

	rc = rclc_support_init(&support, 0, NULL, &allocator);
	if (rc != RCL_RET_OK) return;

	rc = rclc_timer_init_default(&timer, &support, timer_period, timer_callback);
	if (rc != RCL_RET_OK) return;

	rc = rclc_node_init_default(&node, "STM32_node", "", &support);
	if (rc != RCL_RET_OK) return;

	rc = rclc_publisher_init_best_effort(
	  &feedback_publisher, &node, feedback_type_support, feedback_publisher_name);
	if (rc != RCL_RET_OK) return;

	rc = rclc_subscription_init_best_effort(
	  &subscriber, &node, pwm_type_support, pwm_subscriber_name);
	if (rc != RCL_RET_OK) return;

	rc = rclc_service_init_default(
		&setup_service, &node, setup_type_support, setup_service_name);
	if (rc != RCL_RET_OK) return;

	rc = rclc_service_init_default(
		&control_service, &node, control_type_support, control_service_name);
	if (rc != RCL_RET_OK) return;

	rclc_executor_t executor;
	executor = rclc_executor_get_zero_initialized_executor();
	unsigned int num_handles = 3; //2 servizi e 1 timer
	rclc_executor_init(&executor, &support.context, num_handles, &allocator);

	rc = rclc_executor_add_timer(&executor, &timer);
	if (rc != RCL_RET_OK) return;

	rc = rclc_executor_add_subscription(
	  &executor, &subscriber, &pwm_msg,
	  &pwm_callback, ON_NEW_DATA);
	if (rc != RCL_RET_OK) return;

	rc = rclc_executor_add_service(
		&executor, &setup_service, &req_setup,
		&res_setup, setup_callback);
	if (rc != RCL_RET_OK) return;

	rc = rclc_executor_add_service(
		&executor, &control_service, &req_control,
		&res_control, control_callback);
	if (rc != RCL_RET_OK) return;

	rclc_executor_spin(&executor);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_cycleRobotTask */
/**
* @brief Function implementing the cycleRobot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cycleRobotTask */
void cycleRobotTask(void *argument)
{
  /* USER CODE BEGIN cycleRobotTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END cycleRobotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

