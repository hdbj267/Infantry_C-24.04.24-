
#ifndef CAN1_APP_H
#define CAN1_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define RATE_BUF_SIZE 6  //����ٶȻ����˲����ڴ�С

typedef struct 
{
	struct //�����ṹ�嶨��ṹ�����    *hyj
	{                               
		int32_t raw_value;          //����ֵ
		int32_t last_raw_value;     //��һ����ֵ
		int32_t diff;               //ǰ��������
		int32_t round_cnt;          //Ȧ��
		int32_t ecd_raw_rate;       //
		int32_t ecd_bias;           //��ʼλ��
		int32_t filter_rate;        //���ٶ�ƽ��ֵ
		uint8_t start_flag;         //��ʼλ�ñ�־
		int32_t buf_count;          
		int32_t filter_rate_sum;    //
		int32_t rate_buf[RATE_BUF_SIZE];
		float ecd_angle;            //�Ƕ�  
	}encoder;//����һ����ΪEncoder�Ľṹ��       
	
	int32_t speed_rpm;
	int32_t given_current;
	int32_t temperate;
}motor_msg_t;

typedef enum
{
	CAN1_SHOOT_STD_ID = 0x200,//�����

	CAN1_TRIGGER_MOTOR_STD_ID = 0x203,
	CAN1_FRICTION_MOTOR1_STD_ID = 0x201,
	CAN1_FRICTION_MOTOR2_STD_ID = 0x202,
	
} can1_msg_id_e;

extern motor_msg_t trigger_motor_msg;
extern motor_msg_t fric_motor1_msg;
extern motor_msg_t fric_motor2_msg;

void can1_message_progress( CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);

void set_shoot_behaviour(int16_t trigger_iq, \
						 int16_t fric1_iq,   \
						 int16_t fric2_iq);
void set_shoot_stop(void);

motor_msg_t *get_trigger_motor_msg_point(void);
motor_msg_t *get_fric_motor1_msg_point(void);
motor_msg_t *get_fric_motor2_msg_point(void);




#endif

