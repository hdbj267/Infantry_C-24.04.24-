#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

typedef struct
{
	uint32_t time;            //ģ��ˢ��ʱ��ϵͳʱ��
	uint8_t enable_flag;      //�Ƿ�ʹ�ܼ�ر�־λ
	uint8_t *error_msg;       //oled��ӡ�Ĵ�����Ϣ
	uint8_t lost_flag;        //���߱�־λ
	uint16_t timeout;         //���г�ʱʱ��

//	uint16_t error_oled_row;  //������ʾ��oled�ϵ���
//	uint8_t display_flag;
}error_t;

typedef struct //��Ҫ��ص�ģ��
{
	error_t remote;
	error_t yaw_motor;
	error_t pitch_motor;
	error_t trigger_motor;
	error_t fric1_motor;
	error_t fric2_motor;
	
	uint32_t system_current_time;//ϵͳ��ǰʱ��
	uint16_t error_cnt;//��������
	uint8_t exist_error_flag;

}monitor_t;

extern monitor_t monitor;

#endif
