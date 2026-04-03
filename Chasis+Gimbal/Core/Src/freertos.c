/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "reg.h"
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

/* USER CODE END Variables */
/* Definitions for RC_Task */
osThreadId_t RC_TaskHandle;
const osThreadAttr_t RC_Task_attributes = {
  .name = "RC_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Gimbal_Task */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Vision_Task */
osThreadId_t Vision_TaskHandle;
const osThreadAttr_t Vision_Task_attributes = {
  .name = "Vision_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CANRXQueue */
osMessageQueueId_t CANRXQueueHandle;
const osMessageQueueAttr_t CANRXQueue_attributes = {
  .name = "CANRXQueue"
};
/* Definitions for USBRXQueue */
osMessageQueueId_t USBRXQueueHandle;
const osMessageQueueAttr_t USBRXQueue_attributes = {
  .name = "USBRXQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRC_Task(void *argument);
void StartGimbal_Task(void *argument);
void StartChassis_Task(void *argument);
void StartVision_Task(void *argument);
void StartINSTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* Create the queue(s) */
  /* creation of CANRXQueue */
  CANRXQueueHandle = osMessageQueueNew (16, sizeof(CanRxFrame_t), &CANRXQueue_attributes);

  /* creation of USBRXQueue */
  USBRXQueueHandle = osMessageQueueNew (16, sizeof(UsbRxFrame_t), &USBRXQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RC_Task */
  RC_TaskHandle = osThreadNew(StartRC_Task, NULL, &RC_Task_attributes);

  /* creation of Gimbal_Task */
  Gimbal_TaskHandle = osThreadNew(StartGimbal_Task, NULL, &Gimbal_Task_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(StartChassis_Task, NULL, &Chassis_Task_attributes);

  /* creation of Vision_Task */
  Vision_TaskHandle = osThreadNew(StartVision_Task, NULL, &Vision_Task_attributes);

  /* creation of INSTask */
  INSTaskHandle = osThreadNew(StartINSTask, NULL, &INSTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRC_Task */
/**
  * @brief  Function implementing the RC_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRC_Task */
__weak void StartRC_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartRC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRC_Task */
}

/* USER CODE BEGIN Header_StartGimbal_Task */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbal_Task */
__weak void StartGimbal_Task(void *argument)
{
  /* USER CODE BEGIN StartGimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGimbal_Task */
}

/* USER CODE BEGIN Header_StartChassis_Task */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassis_Task */
__weak void StartChassis_Task(void *argument)
{
  /* USER CODE BEGIN StartChassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassis_Task */
}

/* USER CODE BEGIN Header_StartVision_Task */
/**
* @brief Function implementing the Vision_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVision_Task */
__weak void StartVision_Task(void *argument)
{
  /* USER CODE BEGIN StartVision_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartVision_Task */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the INSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTask */
__weak void StartINSTask(void *argument)
{
  /* USER CODE BEGIN StartINSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

