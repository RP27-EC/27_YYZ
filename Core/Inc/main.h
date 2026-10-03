/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define PWR_OUT2_EN_Pin GPIO_PIN_13
#define PWR_OUT2_EN_GPIO_Port GPIOC
#define PWR_OUT1_EN_Pin GPIO_PIN_14
#define PWR_OUT1_EN_GPIO_Port GPIOC
#define PWR_5V_EN_Pin GPIO_PIN_15
#define PWR_5V_EN_GPIO_Port GPIOC
#define CS1_ACCEL_Pin GPIO_PIN_0
#define CS1_ACCEL_GPIO_Port GPIOC
#define BMI_MOSI_Pin GPIO_PIN_1
#define BMI_MOSI_GPIO_Port GPIOC
#define BMI_MISO_Pin GPIO_PIN_2
#define BMI_MISO_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_3
#define CS1_GYRO_GPIO_Port GPIOC
#define GIMB_PITCH_PWM_Pin GPIO_PIN_0
#define GIMB_PITCH_PWM_GPIO_Port GPIOA
#define PUMP_EN1_Pin GPIO_PIN_2
#define PUMP_EN1_GPIO_Port GPIOA
#define RGB_SCK_Pin GPIO_PIN_5
#define RGB_SCK_GPIO_Port GPIOA
#define RGB_MOSI_Pin GPIO_PIN_7
#define RGB_MOSI_GPIO_Port GPIOA
#define BMI_TEMP_PWM_Pin GPIO_PIN_1
#define BMI_TEMP_PWM_GPIO_Port GPIOB
#define PUMP_EN2_Pin GPIO_PIN_9
#define PUMP_EN2_GPIO_Port GPIOE
#define BMI_ACCEL_INT_Pin GPIO_PIN_10
#define BMI_ACCEL_INT_GPIO_Port GPIOE
#define BMI_ACCEL_INT_EXTI_IRQn EXTI15_10_IRQn
#define BMI_GYRO_INT_Pin GPIO_PIN_12
#define BMI_GYRO_INT_GPIO_Port GPIOE
#define BMI_GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define PUMP_EN3_Pin GPIO_PIN_13
#define PUMP_EN3_GPIO_Port GPIOE
#define BMI_SCK_Pin GPIO_PIN_13
#define BMI_SCK_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define KEY_INT_Pin GPIO_PIN_15
#define KEY_INT_GPIO_Port GPIOA
#define DBUS_RX_Pin GPIO_PIN_2
#define DBUS_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
