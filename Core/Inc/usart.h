/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#define CALI_DATA_PACKAGE_SIZE (40u)
extern uint8_t usart1_rx_cali_data[CALI_DATA_PACKAGE_SIZE];
extern uint8_t usart1_tx_cali_data[CALI_DATA_PACKAGE_SIZE];
#define DBUS_RX_BUF_NUM 36
extern uint8_t usart3_remote_data[DBUS_RX_BUF_NUM];
#define VISION_DATA_PACKAGE_SIZE (40u)
extern uint8_t usart6_rx_vision_data[VISION_DATA_PACKAGE_SIZE];
extern uint8_t usart6_tx_vision_data[VISION_DATA_PACKAGE_SIZE];
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void usart_Receive_DMA(void);
void Transmit_To_vision(uint8_t enemy);
/**********USART1**********/
void usart1_send_data(uint8_t *tx_data_package);
/***********USART3******************/
void usart3_reset(void);
/************USART6**********************/

void TX2_RX_INT(void);

typedef __packed struct 
{ 
	uint8_t frame_seq;
	uint8_t shoot_mode;
	uint8_t shoot_speed;
	float pitch_dev;
	float yaw_dev;
	int16_t rail_speed;
	uint8_t gimbal_mode;
	
}_tx2_feedback_data;

typedef __packed struct 
{

  volatile uint8_t recog_flag;
	volatile uint8_t shoot_flag;
	uint8_t yaw_sign;
	uint8_t view_control_flag;
	uint16_t yaw_raw;
	float 	 yaw_dev;
	uint8_t  pitch_sign;
	uint16_t pitch_raw;
	float 	 pitch_dev;
	float yaw_old_dev;
	float pitch_old_dev;
	uint16_t Target_distance;
	float forward_speed;
	float zuoyou_speed;
	float rotate_speed;
}vision_control_data_t;

uint8_t VISION_MES(vision_control_data_t *control_data);


typedef struct
{
	
	uint8_t check_top_byte;
	uint8_t check_bottom_byte;
	uint8_t pid_type_data;

	
	uint8_t receive_success_flag; 
	uint8_t beep_flag;            
	
} vision_t;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
