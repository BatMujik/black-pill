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
#include "stm32f4xx_hal.h"
#include "spi_rpc.h"

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
#define INT_Pin GPIO_PIN_13
#define INT_GPIO_Port GPIOC
#define MCU_MUX_SEL_0_Pin GPIO_PIN_14
#define MCU_MUX_SEL_0_GPIO_Port GPIOC
#define MCU_MUX_SEL_1_Pin GPIO_PIN_15
#define MCU_MUX_SEL_1_GPIO_Port GPIOC
#define MCU_PWM_0_Pin GPIO_PIN_0
#define MCU_PWM_0_GPIO_Port GPIOA
#define MCU_PWM_1_Pin GPIO_PIN_1
#define MCU_PWM_1_GPIO_Port GPIOA
#define MCU_TX_2_Pin GPIO_PIN_2
#define MCU_TX_2_GPIO_Port GPIOA
#define MCU_RX_2_Pin GPIO_PIN_3
#define MCU_RX_2_GPIO_Port GPIOA
#define ADC_1_Pin GPIO_PIN_4
#define ADC_1_GPIO_Port GPIOA
#define ADC_2_Pin GPIO_PIN_5
#define ADC_2_GPIO_Port GPIOA
#define ADC_3_Pin GPIO_PIN_6
#define ADC_3_GPIO_Port GPIOA
#define ADC_0_Pin GPIO_PIN_7
#define ADC_0_GPIO_Port GPIOA
#define MCU_TX_1_Pin GPIO_PIN_9
#define MCU_TX_1_GPIO_Port GPIOA
#define MCU_RX_1_Pin GPIO_PIN_10
#define MCU_RX_1_GPIO_Port GPIOA
#define JET_CS0_Pin GPIO_PIN_15
#define JET_CS0_GPIO_Port GPIOA
#define JET_SCK_Pin GPIO_PIN_3
#define JET_SCK_GPIO_Port GPIOB
#define JET_MISO_Pin GPIO_PIN_4
#define JET_MISO_GPIO_Port GPIOB
#define JET_MOSI_Pin GPIO_PIN_5
#define JET_MOSI_GPIO_Port GPIOB
#define MCU_ENC_0_Pin GPIO_PIN_6
#define MCU_ENC_0_GPIO_Port GPIOB
#define MCU_ENC_1_Pin GPIO_PIN_7
#define MCU_ENC_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
