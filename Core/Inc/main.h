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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Pos_A_Pin GPIO_PIN_0
#define Pos_A_GPIO_Port GPIOA
#define Pos_B_Pin GPIO_PIN_1
#define Pos_B_GPIO_Port GPIOA
#define Pos_C_Pin GPIO_PIN_2
#define Pos_C_GPIO_Port GPIOA
#define Pos_D_Pin GPIO_PIN_3
#define Pos_D_GPIO_Port GPIOA
#define Pos_E_Pin GPIO_PIN_4
#define Pos_E_GPIO_Port GPIOA
#define Pos_F_Pin GPIO_PIN_5
#define Pos_F_GPIO_Port GPIOA
#define Pos_G_Pin GPIO_PIN_6
#define Pos_G_GPIO_Port GPIOA
#define Point_Pin GPIO_PIN_7
#define Point_GPIO_Port GPIOA
#define T_CLK_Pin GPIO_PIN_0
#define T_CLK_GPIO_Port GPIOB
#define Ind_1_Pin GPIO_PIN_10
#define Ind_1_GPIO_Port GPIOB
#define Ind_2_Pin GPIO_PIN_11
#define Ind_2_GPIO_Port GPIOB
#define Ind_3_Pin GPIO_PIN_12
#define Ind_3_GPIO_Port GPIOB
#define Ind_4_Pin GPIO_PIN_13
#define Ind_4_GPIO_Port GPIOB
#define T_SCLK_Pin GPIO_PIN_10
#define T_SCLK_GPIO_Port GPIOA
#define T_SDA_Pin GPIO_PIN_11
#define T_SDA_GPIO_Port GPIOA
#define T_RST_Pin GPIO_PIN_12
#define T_RST_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
