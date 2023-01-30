/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define batarya_Pin GPIO_PIN_1
#define batarya_GPIO_Port GPIOA
#define deployment_2_Pin GPIO_PIN_0
#define deployment_2_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_1
#define buzzer_GPIO_Port GPIOB
#define BMP280_SCL_Pin GPIO_PIN_10
#define BMP280_SCL_GPIO_Port GPIOB
#define BMP280_SDA_Pin GPIO_PIN_11
#define BMP280_SDA_GPIO_Port GPIOB
#define led_blue_Pin GPIO_PIN_13
#define led_blue_GPIO_Port GPIOB
#define led_red_Pin GPIO_PIN_15
#define led_red_GPIO_Port GPIOB
#define LORA_TX_Pin GPIO_PIN_9
#define LORA_TX_GPIO_Port GPIOA
#define LORA_RX_Pin GPIO_PIN_10
#define LORA_RX_GPIO_Port GPIOA
#define eeprom_erase_ping_Pin GPIO_PIN_11
#define eeprom_erase_ping_GPIO_Port GPIOA
#define eeprom_erase_Pin GPIO_PIN_12
#define eeprom_erase_GPIO_Port GPIOA
#define deployment_1_Pin GPIO_PIN_15
#define deployment_1_GPIO_Port GPIOA
#define eeprom_read_ping_Pin GPIO_PIN_3
#define eeprom_read_ping_GPIO_Port GPIOB
#define eeprom_read_Pin GPIO_PIN_4
#define eeprom_read_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_6
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
