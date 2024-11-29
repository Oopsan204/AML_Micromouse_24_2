/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIT_SW_0_Pin GPIO_PIN_2
#define BIT_SW_0_GPIO_Port GPIOE
#define BIT_SW_1_Pin GPIO_PIN_3
#define BIT_SW_1_GPIO_Port GPIOE
#define BIT_SW_2_Pin GPIO_PIN_4
#define BIT_SW_2_GPIO_Port GPIOE
#define BIT_SW_3_Pin GPIO_PIN_5
#define BIT_SW_3_GPIO_Port GPIOE
#define BIT_SW_4_Pin GPIO_PIN_6
#define BIT_SW_4_GPIO_Port GPIOE
#define EN_A_R_Pin GPIO_PIN_0
#define EN_A_R_GPIO_Port GPIOA
#define EN_B_L_Pin GPIO_PIN_1
#define EN_B_L_GPIO_Port GPIOA
#define BUTTON_0_Pin GPIO_PIN_2
#define BUTTON_0_GPIO_Port GPIOA
#define BUTTON_1_Pin GPIO_PIN_3
#define BUTTON_1_GPIO_Port GPIOA
#define SS_FF_Pin GPIO_PIN_6
#define SS_FF_GPIO_Port GPIOA
#define SS_FR_Pin GPIO_PIN_7
#define SS_FR_GPIO_Port GPIOA
#define SS_FL_Pin GPIO_PIN_4
#define SS_FL_GPIO_Port GPIOC
#define SS_R_Pin GPIO_PIN_5
#define SS_R_GPIO_Port GPIOC
#define SS_L_Pin GPIO_PIN_1
#define SS_L_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_14
#define buzzer_GPIO_Port GPIOD
#define LED_0_Pin GPIO_PIN_15
#define LED_0_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_7
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_8
#define LED_3_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_9
#define LED_4_GPIO_Port GPIOC
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define EN_A_L_Pin GPIO_PIN_15
#define EN_A_L_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_6
#define BIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_7
#define BIN1_GPIO_Port GPIOD
#define EN_B_LB3_Pin GPIO_PIN_3
#define EN_B_LB3_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_5
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_6
#define AIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
