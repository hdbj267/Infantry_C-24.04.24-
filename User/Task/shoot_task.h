#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "remote_app.h"
#include "gimbal_task.h"

#define SHOOT_TASK_TIME_1MS         (1u)

// #define FRIC_NORMAL_SPPED_VALUE     (300.0f)//Ħ���������ٶ�ֵ
#define FRIC_TEXT_SPEED           (600.0f)

#define FRIC_ALLOW_SPEED_OFFSET     (300.0f)//Ħ�����ٶ�����ƫ��
#define TRIGGER_INIT_POSITION_ECD   (2000)//���ֵ����ʼλ��

#define TRIGGER_GRID          	    (12) //���̸���
#define SINGLE_BULLET_ECD_INC 	    (24576.0f)//ת���ı���ֵ 
#define SINGLE_BULLET_ANGLE_INC     (45.0f)//����������Ҫת���ĽǶ�
#define TRIGGER_ECD_TO_ANGLE  	    (0.001220703125f)//��������ֵת�������Ƕ� ���Ǽ��ٱ�36:1

#define SHOOT_TASK_INIT_TIME        (3010)//��ʼǰ��ʱһ��ʱ�� �ȴ�����ģ����յ�����

#define PRESS_LONG_TIME             (400)//����ֵ
#define RC_CH4_MAX                  (660)

#define MAGAZINE_CONTROL            KEY_PRESSED_OFFSET_R         //���ֿ��� 
#define SHOOT_CONTROL               KEY_PRESSED_OFFSET_CTRL      //�����������
#define VISION_BUFF                 KEY_PRESSED_OFFSET_F         //�Ӿ����
#define VISION_BUFF_OVER            KEY_PRESSED_OFFSET_G         //ȡ���Ӿ����
typedef enum
{
	SHOOT_STOP = 0,
	SHOOT_READY,
	SHOOT_BULLET,
}shoot_mode_e;

typedef enum 
{
	FRIC_WHEEL_NORMAL = 0,  //������
	FRIC_WHEEL_INSTABLE,    //�쳣��
	FRIC_WHEEL_OFF,					//δ����
}fric_mode_e;

typedef struct
{
	cascade_pid_t trigger_pid;
	pid_t fric_1_pid;
	pid_t fric_2_pid;
}shoot_pid_t;

typedef struct
{
	RC_ctrl_t *rc_ctrl;
	motor_msg_t *trigger_motor_msg;
	motor_msg_t *fric_motor1_msg;
	motor_msg_t *fric_motor2_msg;	

	given_current_t given_current;
	
	uint8_t single_bullet_flag;
	uint8_t revolution_bullet_flag;
	float trigger_position_angle_set;
	float trigger_position_angle_fdb;
	float trigger_continuous_set;
	float trigger_continyous_fdb;

	
	float fric1_set;
	float fric1_fdb;
	
	float fric2_set;
	float fric2_fdb;
	
	//���+ң����ch4              
	uint16_t press_left_time;//�������ʱ��ͳ��
	uint16_t press_right_time;
	uint8_t mouse_left_long_press_flag;      //�󳤰�   
	uint8_t mouse_right_long_press_flag;     //�ҳ���   ������ģʽ��
	uint8_t mouse_left_single_click_flag;    //�󵥻�   ������ģʽ��
	uint8_t mouse_right_single_click_flag;   //�ҵ��� 

	uint8_t magazine_control_flag;      //1:������      0:�ص���
	uint8_t shoot_control_flag;         //�����������
	uint8_t shoot_vision_flag;          //�Ӿ�ģʽ
}shoot_control_data_t;

uint8_t get_shoot_mode(void);
uint8_t get_fric_mode(void);
#endif
