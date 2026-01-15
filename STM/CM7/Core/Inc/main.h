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

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

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
#define DCMI_D4_Pin GPIO_PIN_4
#define DCMI_D4_GPIO_Port GPIOE
#define DCMI_D6_Pin GPIO_PIN_5
#define DCMI_D6_GPIO_Port GPIOE
#define DCMI_D7_Pin GPIO_PIN_6
#define DCMI_D7_GPIO_Port GPIOE
#define PIXCLK_Pin GPIO_PIN_6
#define PIXCLK_GPIO_Port GPIOF
#define PIXCLK_EXTI_IRQn EXTI9_5_IRQn
#define HREF_Pin GPIO_PIN_7
#define HREF_GPIO_Port GPIOF
#define HREF_EXTI_IRQn EXTI9_5_IRQn
#define VSYNC_Pin GPIO_PIN_8
#define VSYNC_GPIO_Port GPIOF
#define VSYNC_EXTI_IRQn EXTI9_5_IRQn
#define DCMI_HSYNC_Pin GPIO_PIN_4
#define DCMI_HSYNC_GPIO_Port GPIOA
#define DCMI_PIXCLK_Pin GPIO_PIN_6
#define DCMI_PIXCLK_GPIO_Port GPIOA
#define CAM_PWDN_Pin GPIO_PIN_11
#define CAM_PWDN_GPIO_Port GPIOE
#define BNO_RST_Pin GPIO_PIN_10
#define BNO_RST_GPIO_Port GPIOB
#define BNO_ADR_Pin GPIO_PIN_11
#define BNO_ADR_GPIO_Port GPIOB
#define DCMI_D0_Pin GPIO_PIN_6
#define DCMI_D0_GPIO_Port GPIOC
#define DCMI_D1_Pin GPIO_PIN_7
#define DCMI_D1_GPIO_Port GPIOC
#define DCMI_D2_Pin GPIO_PIN_8
#define DCMI_D2_GPIO_Port GPIOC
#define DCMI_D3_Pin GPIO_PIN_9
#define DCMI_D3_GPIO_Port GPIOC
#define DCMI_D5_Pin GPIO_PIN_3
#define DCMI_D5_GPIO_Port GPIOD
#define DCMI_VSYNC_Pin GPIO_PIN_9
#define DCMI_VSYNC_GPIO_Port GPIOG
#define CAM_RST_Pin GPIO_PIN_12
#define CAM_RST_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
