/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/*
 * variable for fir filter
 */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_3
#define LCD_RST_GPIO_Port GPIOE
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOE
#define PRESSURE_4_Pin GPIO_PIN_0
#define PRESSURE_4_GPIO_Port GPIOC
#define PULSE_1_Pin GPIO_PIN_3
#define PULSE_1_GPIO_Port GPIOA
#define PULSE_2_Pin GPIO_PIN_4
#define PULSE_2_GPIO_Port GPIOA
#define PULSE_3_Pin GPIO_PIN_5
#define PULSE_3_GPIO_Port GPIOA
#define PULSE_4_Pin GPIO_PIN_6
#define PULSE_4_GPIO_Port GPIOA
#define PRESSURE_1_Pin GPIO_PIN_7
#define PRESSURE_1_GPIO_Port GPIOA
#define PRESSURE_2_Pin GPIO_PIN_0
#define PRESSURE_2_GPIO_Port GPIOB
#define PRESSURE_3_Pin GPIO_PIN_1
#define PRESSURE_3_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_10
#define LCD_SCK_GPIO_Port GPIOB
#define PUMP_4_Pin GPIO_PIN_8
#define PUMP_4_GPIO_Port GPIOD
#define PUMP_3_Pin GPIO_PIN_9
#define PUMP_3_GPIO_Port GPIOD
#define PUMP_2_Pin GPIO_PIN_10
#define PUMP_2_GPIO_Port GPIOD
#define PUMP_1_Pin GPIO_PIN_11
#define PUMP_1_GPIO_Port GPIOD
#define VALVE_1_Pin GPIO_PIN_6
#define VALVE_1_GPIO_Port GPIOC
#define VLAVE_2_Pin GPIO_PIN_7
#define VLAVE_2_GPIO_Port GPIOC
#define VLAVE_3_Pin GPIO_PIN_8
#define VLAVE_3_GPIO_Port GPIOC
#define VALVE_4_Pin GPIO_PIN_9
#define VALVE_4_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
