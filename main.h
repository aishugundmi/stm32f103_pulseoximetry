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
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define GPIO_SCL_Pin GPIO_PIN_6
#define GPIO_SCL_GPIO_Port GPIOB
#define GPIO_SDA_Pin GPIO_PIN_7
#define GPIO_SDA_GPIO_Port GPIOB


// FIFO registers
#define MAX30100_FIFO_W_POINTER      0x02
#define MAX30100_OVF_COUNTER         0x03
#define MAX30100_FIFO_R_POINTER      0x04
#define MAX30100_FIFO_DATA_REG       0x05

// configuration registers
#define MAX30100_MODE_CONFIG         0x06
#define MAX30100_SPO2_CONFIG         0x07
#define MAX30100_LED_CONFIG          0x09

// PART ID registers
#define MAX30100_PART_ID             0xFF

// MAX30100 I2C addresses
#define MAX30100_WADDRESS        0xAE  // 8bit address converted to 7bit + W
#define MAX30100_RADDRESS        0xAF  // 8bit address converted to 7bit + R


#define UPPER_TH					40
#define LOWER_TH					10

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
