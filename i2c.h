/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __I2C_H
#define __I2C_H

#define SDA_HIGH()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,1)
#define SDA_LOW()    	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,0)

#define SCL_HIGH()   	{	\
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);	\
							do{	\
								;	\
							}while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)== 0);	\
						}

#define SCL_LOW()  		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);


void i2c_delay();
void i2c_stop(void);

int read_max101_register(uint8_t reg, uint8_t *data, int size);
int write_max101_register(uint8_t reg, uint8_t data);

#endif // __I2C_H
