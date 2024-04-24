/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe
 * @Teammate��
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description:          �������ȷ������ģ���Ƿ�����
 * @Others:               Ŀǰ�����⣺oledˢ�������ᵼ��gui����������������һ˲�������
						  ���죬�����ǻ��⵽���ߣ����⣩���Ƿ���Ҫ���������߼���
						  �д�˼����
**/
#include "monitor_task.h"
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "remote_app.h"
#include "can.h"
#include "tim.h"
#include "gimbal_task.h"
#include "timers.h"

extern TaskHandle_t Gimbal_Task_Handler;
extern TaskHandle_t Shoot_Task_Handler;
extern TaskHandle_t Connect_Task_Handler;
extern TaskHandle_t INS_Task_Handler;
extern TaskHandle_t GUI_Task_Handler;
extern TaskHandle_t Test_Task_Handler;
extern TaskHandle_t Cali_Task_Handler;

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
monitor_t monitor = 
{
	.remote.enable_flag = 1, //ʹ�ܵ�ģ���������
	.remote.error_msg = (uint8_t *)"remote error",
	.remote.timeout = 500,//��ʱʱ������
	
	.yaw_motor.enable_flag = 1,
	.yaw_motor.error_msg = (uint8_t *)"yaw_motor error",
	.yaw_motor.timeout = 500,
	
	.pitch_motor.enable_flag = 1,
	.pitch_motor.error_msg = (uint8_t *)"pitch_motor error",
	.pitch_motor.timeout = 500,
	
	.trigger_motor.enable_flag = 1,
	.trigger_motor.error_msg = (uint8_t *)"trigger_motor error",
	.trigger_motor.timeout = 4000,
	
	.fric1_motor.enable_flag = 1,
	.fric1_motor.error_msg = (uint8_t *)"fric1_motor error",
	.fric1_motor.timeout = 4000,
	
	.fric2_motor.enable_flag = 1,
	.fric2_motor.error_msg = (uint8_t *)"fric2_motor error",
	.fric2_motor.timeout = 4000,
};
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void is_lost(monitor_t *monitor, error_t *error)
{
	if(error->enable_flag == 1)
	{
		if(monitor->system_current_time > error->time + error->timeout)
		{
			if(monitor->error_cnt < 8 && error->lost_flag == 0)//����8������ʾ
			{
				error->lost_flag = 1;
//				error->error_oled_row = monitor->error_cnt;//���浱ǰ��������ʾ��oled��
				monitor->error_cnt ++;//����������
			}
		}
		else
		{
			if(error->lost_flag == 1)//�ָ���
			{
				error->lost_flag = 0;//��ʧ��־λ��λ
				monitor->error_cnt --;//������1
//				monitor->single_error_resume_flag = 1;
//				monitor->single_error_resume_row = error->error_oled_row;//����֮ǰ��oledλ��
//				error->error_oled_row = 0;//��λ
//				LED_Fill(0x00);//����˲���⵽ģ�����ߣ����е����ִ����������oledGUI���������쳣
			}
		}
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void error_list_display(monitor_t *monitor, error_t *error)
{
	static uint16_t error_row = 0;
	if(error->enable_flag == 1)
	{
		if(error->lost_flag == 1)
		{
	//		if(monitor->single_error_resume_flag)//����һ������ָ�
	//		{
	//			if(error->error_oled_row > monitor->single_error_resume_row)//������ʾ��ǰ���ж�
	//			{
	//				if(monitor->error_cnt == error->error_oled_row)//����oled���һ��errorλ��
	//				{
	//					monitor->single_error_resume_flag = 0;//ֹͣ�ƶ�
	//				}
	//				error->error_oled_row -= 1;
	//			}
	//		}
			
	//		if(error->display_flag == 0)
	//		{
	//			error->error_oled_row = error_row;//��������oled��
	//			error_row ++;
	//			error->display_flag = 1;
	//		}
			//����pc_cali��ʱ�ر�
			if(get_robot_control_mode() != GUI_CALI_MODE)
			{
				LED_P6x8Str(0, error_row, (uint8_t *)error->error_msg);
			}
			error_row ++;
			if(error_row == monitor->error_cnt)
			{
				error_row = 0;
			}
			
		}
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void monitor_process(monitor_t *monitor)
{
	is_lost(monitor, &monitor->remote);
	is_lost(monitor, &monitor->yaw_motor);
	is_lost(monitor, &monitor->pitch_motor);
	is_lost(monitor, &monitor->trigger_motor);
	is_lost(monitor, &monitor->fric1_motor);
	is_lost(monitor, &monitor->fric2_motor);	
	
	error_list_display(monitor, &monitor->remote);
	error_list_display(monitor, &monitor->yaw_motor);
	error_list_display(monitor, &monitor->pitch_motor);
	error_list_display(monitor, &monitor->trigger_motor);
	error_list_display(monitor, &monitor->fric1_motor);
	error_list_display(monitor, &monitor->fric2_motor);
	
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
TimerHandle_t Timer_100MS_Task_Handler;
void Timer_100MS_Task(void);
uint8_t task_suspend_flag = 0;
void monitor_task(void *argument)
{
	
	vTaskDelay(200);
	//���������ʱ��
	Timer_100MS_Task_Handler = xTimerCreate((char *)"Timer_100MS_Task",
							           (TickType_t)300,//1s
							           (UBaseType_t)pdTRUE,//����
							           (void *)1,//��ʱ�����
							           (TimerCallbackFunction_t)Timer_100MS_Task);//���ú��� 						
	while(1)
	{
		monitor.system_current_time = xTaskGetTickCount();
		monitor_process(&monitor);
		
		if(monitor.error_cnt != 0)//��һ��ģ�鲻���� ���ǹ������������ ���߽����ٽ��� while(1)
		{
//			vTaskSuspendAll();
//			vTaskResume(NULL);
			monitor.exist_error_flag = 1;//��������ģ������־��1��ʹϵͳ��ģʽ��Ϊֹͣ
			if(task_suspend_flag == 0)
			{			
				task_suspend_flag = 1;
				//LED_Fill(0x00);
				xTimerStart(Timer_100MS_Task_Handler, 0);
			}
			
		}
		else
		{
			task_suspend_flag = 0;
			monitor.exist_error_flag = 0;
			xTimerStop(Timer_100MS_Task_Handler, 0);
			BUZZER_OFF();//�͵���cali��ʾ�г�ͻ
		}
		vTaskDelay(10);                     //10msһ��
	}	
}

void Timer_100MS_Task(void)
{
	static uint8 cnt = 0;
	cnt ++;
	
	if(cnt%4 == 0)
	{
		BUZZER_ON();
		cnt = 0;
	}
	else 
	{
		BUZZER_OFF();
	}
	
}


