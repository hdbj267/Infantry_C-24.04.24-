/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOC
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define IMU_HEAT_PWM_Pin GPIO_PIN_6
#define IMU_HEAT_PWM_GPIO_Port GPIOF
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOD
#define User_key_Pin GPIO_PIN_0
#define User_key_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define MAGAZINE_CONTROL_Pin GPIO_PIN_9
#define MAGAZINE_CONTROL_GPIO_Port GPIOE
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/******* device ********/
#include "dma.h"
#include "gpio.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "flash.h"
/******* app ********/
#include "start_task.h"
#include "remote_app.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "can1_app.h"
#include "can2_app.h"
/******* FreeRTOS ********/
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
