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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define KB_Row4_Pin GPIO_PIN_6
#define KB_Row4_GPIO_Port GPIOA
#define KB_Row7_Pin GPIO_PIN_7
#define KB_Row7_GPIO_Port GPIOA
#define KB_Col1_Pin GPIO_PIN_4
#define KB_Col1_GPIO_Port GPIOC
#define KB_Col6_Pin GPIO_PIN_10
#define KB_Col6_GPIO_Port GPIOB
#define KB_Col7_Pin GPIO_PIN_13
#define KB_Col7_GPIO_Port GPIOB
#define KB_Row1_Pin GPIO_PIN_7
#define KB_Row1_GPIO_Port GPIOC
#define KB_Row3_Pin GPIO_PIN_9
#define KB_Row3_GPIO_Port GPIOC
#define KB_Col0_Pin GPIO_PIN_8
#define KB_Col0_GPIO_Port GPIOA
#define KB_Row0_Pin GPIO_PIN_9
#define KB_Row0_GPIO_Port GPIOA
#define KB_Col2_Pin GPIO_PIN_10
#define KB_Col2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define KB_Restore_Pin GPIO_PIN_12
#define KB_Restore_GPIO_Port GPIOC
#define KB_Col3_Pin GPIO_PIN_3
#define KB_Col3_GPIO_Port GPIOB
#define KB_Col5_Pin GPIO_PIN_4
#define KB_Col5_GPIO_Port GPIOB
#define KB_Col4_Pin GPIO_PIN_5
#define KB_Col4_GPIO_Port GPIOB
#define KB_Row2_Pin GPIO_PIN_6
#define KB_Row2_GPIO_Port GPIOB
#define KB_Row6_Pin GPIO_PIN_8
#define KB_Row6_GPIO_Port GPIOB
#define KB_Row5_Pin GPIO_PIN_9
#define KB_Row5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
