#ifndef CAN2_APP_H
#define CAN2_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"

//#define RATE_BUF_SIZE 6  //����ٶȻ����˲����ڴ�С

typedef struct
{
	
	uint8_t robot_id;                           //������ID
	//�������
	uint16_t shooter_id1_17mm_cooling_rate;     //ÿ����ȴֵ0��17mm��  1��42mm��  
	uint16_t shooter_id1_17mm_cooling_limit;    //ǹ����������
	uint16_t shooter_id1_17mm_speed_limit;      //�ٶ�����
	uint8_t mains_power_shooter_output ;        //��Դ�Ƿ�������������
	float bullet_speed;                         //��ǰ����
	uint16_t shooter_id1_17mm_cooling_heat;     //17mm��ǰǹ������
	uint8_t hurt_type : 4;                      //0x2 �����ٿ�Ѫ��0x3 ��ǹ��������Ѫ��

} ext_Judge_data_t;                     //ʵʱ������Ϣ   *hyj

typedef enum
{	
	CAN2_GIMBAL_STD_ID = 0x1FF,
	CAN2_YAW_MOTOR_STD_ID = 0x205,	// yaw���
	CAN2_PITCH_MOTOR_STD_ID = 0x206,	//pitch���
	//can2��
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x200,
	CAN2_CONNECT_CM_GYRO_STD_ID = 0x208,
	CAN2_CONNECT_UIFLAG_STD_ID  = 0x209,
	//can2��
	CAN2_SHOOT_17mm_ID = 0x020B,         //17mm�������������Ϣ
	CAN2_SHOOT_JUDGE_ID = 0x020C,        //�������������Ϣ
	
} can2_msg_id_e;

extern motor_msg_t yaw_motor_msg;
extern motor_msg_t pitch_motor_msg;

void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
                             
void shoot_17mm_mag(ext_Judge_data_t *Judge_data, uint8_t aData[]);
void shoot_judge_process(ext_Judge_data_t *Judge_data, uint8_t aData[]);

void set_gimbal_behaviour(int16_t yaw_iq, int16_t pitch_iq);
void set_gimbal_stop(void);

motor_msg_t *get_yaw_motor_msg_point(void);
motor_msg_t *get_pitch_motor_msg_point(void);


#endif

