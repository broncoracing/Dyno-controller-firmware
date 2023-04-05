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
#include "stm32f1xx_hal.h"

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

#define HX711_CAL_CONSTANT 1000.0f

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEG_B_Pin GPIO_PIN_13
#define SEG_B_GPIO_Port GPIOC
#define SEG_F_Pin GPIO_PIN_14
#define SEG_F_GPIO_Port GPIOC
#define SEG_A_Pin GPIO_PIN_15
#define SEG_A_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define HX_SCK_Pin GPIO_PIN_3
#define HX_SCK_GPIO_Port GPIOA
#define HX_DT_Pin GPIO_PIN_4
#define HX_DT_GPIO_Port GPIOA
#define STABLE_LED_Pin GPIO_PIN_5
#define STABLE_LED_GPIO_Port GPIOA
#define SEG_E_Pin GPIO_PIN_0
#define SEG_E_GPIO_Port GPIOB
#define SEG_D_Pin GPIO_PIN_1
#define SEG_D_GPIO_Port GPIOB
#define SEG_DP_Pin GPIO_PIN_2
#define SEG_DP_GPIO_Port GPIOB
#define SEG_C_Pin GPIO_PIN_10
#define SEG_C_GPIO_Port GPIOB
#define SEG_G_Pin GPIO_PIN_11
#define SEG_G_GPIO_Port GPIOB
#define PULSE_Pin GPIO_PIN_9
#define PULSE_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_10
#define DIR_GPIO_Port GPIOA
#define DIG_0_Pin GPIO_PIN_4
#define DIG_0_GPIO_Port GPIOB
#define DIG_1_Pin GPIO_PIN_5
#define DIG_1_GPIO_Port GPIOB
#define DIG_2_Pin GPIO_PIN_6
#define DIG_2_GPIO_Port GPIOB
#define DIG_3_Pin GPIO_PIN_7
#define DIG_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t ready;
extern uint32_t current_rpm;
extern uint32_t target_rpm;

#define KP 1.0f
#define KI 0.0f

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
