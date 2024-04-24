/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia
 * @Teammate
 * @Version: V1.0
 * @Date: 2021.4.13
 * @Description:     CAN1Ӧ�ã������������Ŀ��ƣ�           
 * @Note:            ��cubeMX���ɺ󣬵ײ�CAN�����ò����ᱻ�޸ģ��ǵøĻ�ȥ
 * 						 hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
 						 hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;    
 * @Others: 
**/
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can1_app.h"
#include "can.h"
#include "monitor_task.h"
#include "semphr.h"

extern CAN_RxHeaderTypeDef can1_rx_header;
extern uint8_t can1_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can1_tx_header;
extern uint8_t can1_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan1;

motor_msg_t fric_motor1_msg = {0};
motor_msg_t fric_motor2_msg = {0};
motor_msg_t trigger_motor_msg = {0};

/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    Ĭ�ϵ����can����Ƶ��Ϊ1KHZ       
  */
static void shoot_motor_msg_process(motor_msg_t *m, uint8_t aData[])
{
	int16_t i;
	m->encoder.filter_rate_sum = 0;//��������
	if(m->encoder.raw_value !=  (aData[0]<<8|aData[1]))
	{
		m->encoder.last_raw_value = m->encoder.raw_value; 
	}
	if(m->encoder.start_flag==0)//�ϵ�ɼ�ԭʼ�Ƕ�
	{
		m->encoder.ecd_bias = (aData[0]<<8)|aData[1];//��ʼλ��
		m->encoder.last_raw_value = (aData[0]<<8)|aData[1];
		m->encoder.raw_value = m->encoder.last_raw_value;
		m->encoder.start_flag = 1;
	}
	else
	{
		m->encoder.raw_value = (aData[0]<<8)|aData[1];
	}
	
	m->encoder.diff = m->encoder.raw_value - m->encoder.last_raw_value;
	if(m->encoder.diff < -6000)//���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�                         
	{                          //7500���ݿ������ڿɵ�������֤һ������������ ת�ӻ�е�Ƕȱ仯С��8191-7500=691���� 
		m->encoder.round_cnt ++;
		m->encoder.ecd_raw_rate = m->encoder.diff + 8192;
	}
	else if(m->encoder.diff > 6000)
	{
		m->encoder.round_cnt --;
		m->encoder.ecd_raw_rate = m->encoder.diff - 8192;
	}
	else
	{
		m->encoder.ecd_raw_rate = m->encoder.diff;
	}
	//����õ��Ƕ�ֵ����Χ���������
	m->encoder.ecd_angle = (float)(m->encoder.raw_value - m->encoder.ecd_bias)*360/8192  \
								   + m->encoder.round_cnt * 360;
	
	m->encoder.rate_buf[m->encoder.buf_count++] = m->encoder.ecd_raw_rate;
	if(m->encoder.buf_count == RATE_BUF_SIZE)
	{
		m->encoder.buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		m->encoder.filter_rate_sum += m->encoder.rate_buf[i];
	}
	m->encoder.filter_rate = (int32_t)(m->encoder.filter_rate_sum/RATE_BUF_SIZE);	
	/*---------------------�Ǳ���������------------------------*/
	m->speed_rpm = (uint16_t)(aData[2] << 8 | aData[3]);     				
	m->given_current = (uint16_t)(aData[4] << 8 | aData[5]); 				
	m->temperate = aData[6];         
}

/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void can1_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	
	switch(pHeader->StdId)
	{
		//get shoot control 
		case CAN1_TRIGGER_MOTOR_STD_ID:
		{
			shoot_motor_msg_process(&trigger_motor_msg ,aData);   
			monitor.trigger_motor.time = xTaskGetTickCount();			
		}break;
		case CAN1_FRICTION_MOTOR1_STD_ID:
		{
			shoot_motor_msg_process(&fric_motor1_msg ,aData);
			monitor.fric1_motor.time = xTaskGetTickCount();			
		}break;
		case CAN1_FRICTION_MOTOR2_STD_ID:
		{
			shoot_motor_msg_process(&fric_motor2_msg ,aData);   
			monitor.fric2_motor.time = xTaskGetTickCount();			
		}break;
	}
}
/**
  * @brief          ���͵���ֵ�����
  * @author         
  * @param[in] 
  * @retval	
  * @note         
  */
void set_shoot_behaviour(int16_t fric1_iq,   \			
						 int16_t fric2_iq,	 \
						 int16_t trigger_iq)  
{
	//can1
	can1_tx_header.StdId = CAN1_SHOOT_STD_ID;
	can1_tx_header.IDE = CAN_ID_STD;
	can1_tx_header.RTR = CAN_RTR_DATA;
	can1_tx_header.DLC = 0x08;
    
    can1_tx_data[0] = (uint8_t)(fric1_iq >> 8);
    can1_tx_data[1] = (uint8_t)fric1_iq;
    can1_tx_data[2] = (uint8_t)(fric2_iq >> 8);
    can1_tx_data[3] = (uint8_t)fric2_iq;
		can1_tx_data[4] = (uint8_t)(trigger_iq >> 8);
    can1_tx_data[5] = (uint8_t)trigger_iq;
    can1_tx_data[6] = (uint8_t)(0 >> 8);
    can1_tx_data[7] = (uint8_t)0;
	extern SemaphoreHandle_t hcan1_mutex;
	xSemaphoreTake(hcan1_mutex, 100);
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_header, can1_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
	xSemaphoreGive(hcan1_mutex);
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
void set_shoot_stop(void)
{
	//can1
	can1_tx_header.StdId = CAN1_SHOOT_STD_ID;
	can1_tx_header.IDE = CAN_ID_STD;
	can1_tx_header.RTR = CAN_RTR_DATA;
	can1_tx_header.DLC = 0x08;
    
    can1_tx_data[0] = (uint8_t)(0 >> 8);
    can1_tx_data[1] = (uint8_t)0;
    can1_tx_data[2] = (uint8_t)(0 >> 8);
    can1_tx_data[3] = (uint8_t)0;
    can1_tx_data[4] = (uint8_t)(0 >> 8);
    can1_tx_data[5] = (uint8_t)0;
    can1_tx_data[6] = (uint8_t)(0 >> 8);
    can1_tx_data[7] = (uint8_t)0;
	extern SemaphoreHandle_t hcan1_mutex;
	xSemaphoreTake(hcan1_mutex, 100);
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_header, can1_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
	xSemaphoreGive(hcan1_mutex);
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
motor_msg_t *get_trigger_motor_msg_point(void)
{
	return &trigger_motor_msg;
}
motor_msg_t *get_fric_motor1_msg_point(void)
{
	return &fric_motor1_msg;
}
motor_msg_t *get_fric_motor2_msg_point(void)
{
	return &fric_motor2_msg;
}
