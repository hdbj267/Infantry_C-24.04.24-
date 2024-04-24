/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate��
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description:   �����������ڲ鿴�����ϵͳ����״̬
 * @Note:       
 * @Others: 
**/
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "oled.h"
#include "gpio.h"
#include "remote_app.h"
#include "bmi088driver.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "stdio.h"
#include "string.h"
#include "shoot_task.h"
#include "can.h"
#include "tim.h"
#include "monitor_task.h"
#include "connect_task.h"
#include "flash.h"
#include "gpio.h"
#include <stdlib.h>
#include <stdio.h>

extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern INS_t INS;
float yaw_angle_max = 0.0;
float yaw_angle_speed_max = 0.0;
extern uint8_t start_time_flag;
extern int32_t start_time_us;
extern int32_t current_time_us;
extern int16_t pitch_motor_can_set_current;

extern robot_work_mode_e robot_work_mode;
extern robot_control_mode_e robot_control_mode;
extern gimbal_control_data_t gimbal_control_data;
extern gimbal_work_mode_e gimbal_work_mode;
extern shoot_control_data_t shoot_control_data;
uint8_t can2_buff[8] = {10, 11, 10, 11, 10, 11, 10, 11,};
extern uint8_t can2_rx_data[8];


#define MAX_PSC             1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

uint16_t psc = 0;
uint16_t pwm = MIN_BUZZER_PWM;
char ch[400] = {0};
extern gimbal_pid_t gimbal_pid;
volatile uint8_t flash_flag = 0;
volatile uint32_t flash_data = 0;


const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//���鳤��	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0x080C0000 	//����FLASH �����ַ(����Ϊ4�ı���������������,Ҫ���ڱ�������ռ�õ�������.

u8 datatemp[SIZE];
extern TaskHandle_t Gimbal_Task_Handler;
extern TaskHandle_t GUI_Task_Handler;
extern RC_ctrl_t rc_ctrl_data;

extern QueueHandle_t test_queue_handler;

extern shoot_control_data_t shoot_control_data;
extern shoot_pid_t shoot_pid;
extern float shoot_abs(float value);

extern RC_ctrl_t rc_ctrl_data;

void test_task(void *argument)
{

    while(1)
    {	
		
//		LED_P6x8Str(0, 0, (uint8_t *)"p_set");
//		LED_PrintValueI(40, 0, shoot_control_data.trigger_position_angle_set);

/**
* @brief ����������ʱ�����
* @note    
*/
#if 1 //0   *hyj
		memset(ch,0,400);				//��Ϣ����������
		vTaskGetRunTimeStats(ch);		//��ȡ��������ʱ����Ϣ
		// printf("#������\t\t\t����ʱ��\t������ռ�ٷֱ�\r\n");   //�����ݸ�����   *hyj
		// printf("%s\r\n", ch);
		
/**
* @brief ����������״̬���
* @note    
*/	
#elif 0 		
		memset(ch,0,400);				//��Ϣ����������
		vTaskList(ch);
		printf("task_name\ttask_state\tpriority\tstack\ttasK_num\r\n");
		printf("%s\r\n", ch);
/**
* @brief vTaskGetInfo()
* @note    
*/
#elif 0
		TaskHandle_t TaskHandle;	
		TaskStatus_t TaskStatus;

		TaskHandle=xTaskGetHandle("GUI_task");		//������������ȡ��������

		vTaskGetInfo((TaskHandle_t	)TaskHandle, 		//������
					 (TaskStatus_t*	)&TaskStatus, 		//������Ϣ�ṹ��
					 (BaseType_t	)pdTRUE,			//����ͳ�������ջ��ʷ��Сʣ���С
					 (eTaskState	)eInvalid);			//�����Լ���ȡ��������״̬

		printf("������:                %s\r\n",TaskStatus.pcTaskName);
		printf("������:              %d\r\n",(int)TaskStatus.xTaskNumber);
		printf("����״̬:              %d\r\n",TaskStatus.eCurrentState);
		printf("����ǰ���ȼ�:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
		printf("��������ȼ�:          %d\r\n",(int)TaskStatus.uxBasePriority);
		printf("����ʱ�����:          %d\r\n",(int)TaskStatus.ulRunTimeCounter);
		printf("�����ջ����ַ:        %#x\r\n",(int)TaskStatus.pxStackBase);
		printf("�����ջ��ʷʣ����Сֵ:%d\r\n",TaskStatus.usStackHighWaterMark);	
	    printf("/**************************����***************************/\r\n");	
#endif		
		vTaskDelay(100);
		
    }
}
