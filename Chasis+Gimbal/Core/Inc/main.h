/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  extern osThreadId_t INSTaskHandle;
  extern osThreadId_t Vision_TaskHandle;
  extern osThreadId_t Chassis_TaskHandle;
  extern osThreadId_t Gimbal_TaskHandle;
  extern osThreadId_t RC_TaskHandle;
  extern osMessageQueueId_t CANRXQueueHandle;
  extern osMessageQueueId_t USBRXQueueHandle;
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
  void Delay_us(uint16_t nus);
  void Delays_Init(void);
  void Delay_ms(uint16_t nms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOC
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_ACCEL_Pin GPIO_PIN_0
#define INT1_ACCEL_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_1
#define CS1_GYRO_GPIO_Port GPIOB
#define TEMP_088_Pin GPIO_PIN_10
#define TEMP_088_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
