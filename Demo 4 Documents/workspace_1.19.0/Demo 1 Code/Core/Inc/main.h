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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define SPI3_CS_Pin GPIO_PIN_3
#define SPI3_CS_GPIO_Port GPIOC
#define S1_Button_Pin GPIO_PIN_0
#define S1_Button_GPIO_Port GPIOA
#define S1_Button_EXTI_IRQn EXTI0_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define S3_Button_Pin GPIO_PIN_7
#define S3_Button_GPIO_Port GPIOA
#define S3_Button_EXTI_IRQn EXTI9_5_IRQn
#define D5_LED_Pin GPIO_PIN_4
#define D5_LED_GPIO_Port GPIOC
#define Column_3_Pin GPIO_PIN_5
#define Column_3_GPIO_Port GPIOC
#define S2_Button_Pin GPIO_PIN_1
#define S2_Button_GPIO_Port GPIOB
#define S2_Button_EXTI_IRQn EXTI1_IRQn
#define Column_2_Pin GPIO_PIN_2
#define Column_2_GPIO_Port GPIOB
#define Row_1_Pin GPIO_PIN_12
#define Row_1_GPIO_Port GPIOB
#define Row_2_Pin GPIO_PIN_13
#define Row_2_GPIO_Port GPIOB
#define Row_3_Pin GPIO_PIN_6
#define Row_3_GPIO_Port GPIOC
#define S5_Button_Pin GPIO_PIN_9
#define S5_Button_GPIO_Port GPIOA
#define S5_Button_EXTI_IRQn EXTI9_5_IRQn
#define D4_LED_Pin GPIO_PIN_10
#define D4_LED_GPIO_Port GPIOA
#define Column_1_Pin GPIO_PIN_11
#define Column_1_GPIO_Port GPIOA
#define Row_4_Pin GPIO_PIN_12
#define Row_4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define D3_LED_Pin GPIO_PIN_4
#define D3_LED_GPIO_Port GPIOB
#define D2_LED_Pin GPIO_PIN_5
#define D2_LED_GPIO_Port GPIOB
#define S4_Button_Pin GPIO_PIN_6
#define S4_Button_GPIO_Port GPIOB
#define S4_Button_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
