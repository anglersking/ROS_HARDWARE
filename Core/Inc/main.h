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
#include "stm32f4xx_hal.h"

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
#define CONVERT4_Pin GPIO_PIN_7
#define CONVERT4_GPIO_Port GPIOE
#define CONVERT1_Pin GPIO_PIN_8
#define CONVERT1_GPIO_Port GPIOE
#define CONVERT2_Pin GPIO_PIN_9
#define CONVERT2_GPIO_Port GPIOE
#define BREAK3_Pin GPIO_PIN_10
#define BREAK3_GPIO_Port GPIOE
#define CONVERT3_Pin GPIO_PIN_12
#define CONVERT3_GPIO_Port GPIOE
#define BREAK4_Pin GPIO_PIN_13
#define BREAK4_GPIO_Port GPIOE
#define BREAK1_Pin GPIO_PIN_14
#define BREAK1_GPIO_Port GPIOE
#define BREAK2_Pin GPIO_PIN_15
#define BREAK2_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOC
#define PWM4_Pin GPIO_PIN_9
#define PWM4_GPIO_Port GPIOC
#define MPU6050_SDA_Pin GPIO_PIN_11
#define MPU6050_SDA_GPIO_Port GPIOC
#define MPU6050_SCL_Pin GPIO_PIN_0
#define MPU6050_SCL_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
