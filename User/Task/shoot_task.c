/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate��
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: ���ڷ�������Ŀ���
 * @Note:       
 * @Others: 
**/
#include "shoot_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "can2_app.h"
#include "pid.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "rule.h"

fric_mode_e fric_mode;

shoot_mode_e shoot_mode;
shoot_pid_t shoot_pid;

shoot_control_data_t shoot_control_data;
extern shoot_real_mag shoot_mag;
uint16_t shoot_speed = 0;

uint8_t get_shoot_mode(void)
{
	return shoot_mode;
}
void set_shoot_mode(shoot_mode_e mode)
{
	shoot_mode = mode;
}

void set_fric_mode(fric_mode_e mode)//����Ħ����״̬
{
	fric_mode = mode;
}
uint8_t get_fric_mode(void)
{
	return fric_mode;
}
/**
  * @brief        ��ȡ����ϵĽ�ֵ��Ϣ����λ��ر�־λ
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void get_mouse_press_msg(shoot_control_data_t *shoot)
{
	//left right singel_click
	if(shoot->rc_ctrl->mouse.last_press_l == 0 && shoot->rc_ctrl->mouse.press_l == 1)
	{
		shoot->mouse_left_single_click_flag = 1;//ֵ��˲��仯��//����
	}
	else
	{
		shoot->mouse_left_single_click_flag = 0;
	}
	
	if(shoot->rc_ctrl->mouse.last_press_r == 0 && shoot->rc_ctrl->mouse.press_r == 1)
	{
		shoot->mouse_right_single_click_flag = 1;
	}
	else
	{
		shoot->mouse_right_single_click_flag = 0;
	}
	shoot->rc_ctrl->mouse.last_press_l = shoot->rc_ctrl->mouse.press_l;
	shoot->rc_ctrl->mouse.last_press_r = shoot->rc_ctrl->mouse.press_r;
	
	//left right long_press
	if(shoot->rc_ctrl->mouse.press_l)
	{
		if(shoot->press_left_time < PRESS_LONG_TIME)
		{
			shoot->press_left_time ++;
		}
		else if(shoot->press_left_time == PRESS_LONG_TIME)
		{
			shoot->mouse_left_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_left_time = 0;
		shoot->mouse_left_long_press_flag = 0;
	}
	
	if(shoot->rc_ctrl->mouse.press_r)
	{
		if(shoot->press_right_time < PRESS_LONG_TIME)
		{
			shoot->press_right_time ++;
		}
		else if(shoot->press_right_time == PRESS_LONG_TIME)
		{
			shoot->mouse_right_long_press_flag = 1;						//����
		}
	}
	else
	{
		shoot->press_right_time = 0;
		shoot->mouse_right_long_press_flag = 0;
	}
}
/**
  * @brief        ң��ģ�������� ʹ����ͬ��־λ ͨ����ͬ�Ŀ���״̬�л�
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void get_rc_control_msg(shoot_control_data_t *shoot)
{
	static int16_t rc_ch4;
	static int16_t rc_ch4_last;
	
	rc_ch4 = shoot->rc_ctrl->rc.ch4;
	
	if(rc_ch4_last < RC_CH4_MAX && rc_ch4 == RC_CH4_MAX)//���� 
	{
		shoot->mouse_left_single_click_flag = 1;
	}		
	else
	{
		shoot->mouse_left_single_click_flag = 0;
	}
	if(rc_ch4_last > -RC_CH4_MAX && rc_ch4 == -RC_CH4_MAX)//����
	{
		shoot->mouse_right_single_click_flag = 1;
	}		
	else
	{
		shoot->mouse_right_single_click_flag = 0;
	}
	rc_ch4_last = rc_ch4;
	
	//left right long_press
	if(rc_ch4 == RC_CH4_MAX)//����
	{
		if(shoot->press_left_time < PRESS_LONG_TIME)
		{
			shoot->press_left_time ++;
		}
		else if(shoot->press_left_time == PRESS_LONG_TIME)
		{
			shoot->mouse_left_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_left_time = 0;
		shoot->mouse_left_long_press_flag = 0;
	}
	
	if(rc_ch4 == -RC_CH4_MAX)//����
	{
		if(shoot->press_right_time < PRESS_LONG_TIME)
		{
			shoot->press_right_time ++;
		}
		else if(shoot->press_right_time == PRESS_LONG_TIME)
		{
			shoot->mouse_right_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_right_time = 0;
		shoot->mouse_right_long_press_flag = 0;
	}
}
/**
  * @brief      ����robot��ͬ�Ŀ��Ʒ�ʽ��ȡ��ر�־λ���л����ߵ���ʱ��ձ�־λ
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void get_shoot_related_flags(shoot_control_data_t *shoot)
{
	static uint8_t control_mode;
	static uint8_t control_mode_last;
	
	control_mode = get_robot_control_mode();
	
	if(control_mode == KEY_MOUSE_MODE)				//����
	{
		get_mouse_press_msg(shoot);
	}
	else if(control_mode == REMOTE_MODE)			//ң��
	{
		get_rc_control_msg(shoot);
	}
	//ģʽ�л� clear related flags 
	if((control_mode_last == KEY_MOUSE_MODE && control_mode == REMOTE_MODE) || \
	   (control_mode_last == REMOTE_MODE && control_mode == KEY_MOUSE_MODE) || \
	   (control_mode == GUI_CALI_MODE) || shoot->shoot_control_flag || shoot->magazine_control_flag )
	{
		shoot->press_right_time = 0;
		shoot->mouse_right_long_press_flag = 0;
		shoot->mouse_right_single_click_flag = 0;
		
		shoot->press_left_time = 0;
		shoot->mouse_left_long_press_flag = 0;
		shoot->mouse_left_single_click_flag = 0;
		
		shoot->shoot_control_flag = 0;
		set_shoot_mode(SHOOT_STOP);//����ر�
	    set_fric_mode(FRIC_WHEEL_OFF);//Ħ���ֹر�
//		BLUELED_ON();
	}
	control_mode_last = control_mode;
}
/**
  * @brief         
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float shoot_abs(float value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief         Ħ���ֺͷ���ģʽ���� 
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_mode_update(shoot_control_data_t *shoot, shoot_pid_t *shoot_pid)
{
	//set_fric_mode(FRIC_WHEEL_NORMAL); //���Բſ���                                      
	if(shoot->mouse_left_single_click_flag == 1)                                           
	{
		if(get_fric_mode() == FRIC_WHEEL_OFF)//�ر�����Ϊδ�ȶ�ģʽ                        
			set_fric_mode(FRIC_WHEEL_INSTABLE);//��������
	}
	
	//Ħ����״̬			
	if(get_fric_mode() == FRIC_WHEEL_NORMAL)
	{
		if(/*(shoot_abs(shoot_pid->fric_1_pid.fdb -  shoot_pid->fric_1_pid.set)  \         
			> FRIC_ALLOW_SPEED_OFFSET) ||								   \
		   (shoot_abs(shoot_pid->fric_2_pid.fdb -  shoot_pid->fric_2_pid.set)  \
		    > FRIC_ALLOW_SPEED_OFFSET)*/0)
		{
			set_fric_mode(FRIC_WHEEL_INSTABLE);//��������
//			set_fric_mode(FRIC_WHEEL_NORMAL);//���Բſ���Ĭ��Ϊ�ȶ�
		}

	}
	else if(get_fric_mode() == FRIC_WHEEL_INSTABLE)//������Щ���⣬��ʼ��fdb��set��Ϊ0 ��ͨ��
	{
		if(/*(shoot_abs(shoot_pid->fric_1_pid.fdb - FRIC_MIDDLE_SPEED)         \             
			< FRIC_ALLOW_SPEED_OFFSET) &&									 \
		   (shoot_abs(shoot_pid->fric_2_pid.fdb - shoot_pid->fric_2_pid.set) \
		    < FRIC_ALLOW_SPEED_OFFSET)*/1)
		{
			set_fric_mode(FRIC_WHEEL_NORMAL);
		}		
	}
	
	switch(get_shoot_mode())
	{
		case SHOOT_STOP:						//��ֹ
		{	
			if(get_fric_mode() == FRIC_WHEEL_NORMAL)
			{
				set_shoot_mode(SHOOT_READY); 
			}
		}break;
		case SHOOT_READY:						//	׼��
		{
			if((shoot->mouse_left_single_click_flag == 1)||   \
			   (shoot->mouse_right_long_press_flag == 1) )     //�������ߵ�������������ģʽ
			{
				set_shoot_mode(SHOOT_BULLET);
			}

			if(get_fric_mode() == FRIC_WHEEL_INSTABLE)
			{
				set_shoot_mode(SHOOT_STOP);	
			}
		}break;
		case SHOOT_BULLET:					//����
		{
			if(get_fric_mode() == FRIC_WHEEL_INSTABLE)
			{
				set_shoot_mode(SHOOT_STOP);
			}	
		}break;
	}
	
	if(get_shoot_mode() == SHOOT_STOP)//����ָʾ��ʾĦ�����ٶ����� ���Է���
	{
		LASER_OFF();
	}
	else
	{
		LASER_ON();
	}
}
/**
  * @brief        ���½���pid��set��fdb
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
uint16_t SINGLE_BULLET_ANGLE=30;

static void shoot_set_and_fdb_update(shoot_control_data_t *shoot)//���ֵ����Ҫ��ʼ����Ŀ��Ƕ�
{
	shoot->trigger_position_angle_fdb = (shoot->trigger_motor_msg->encoder.raw_value +      \
										 shoot->trigger_motor_msg->encoder.round_cnt*8192 - \
										 shoot->trigger_motor_msg->encoder.ecd_bias	)*      \
										 TRIGGER_ECD_TO_ANGLE;
	//shoot_bullet
	if(get_shoot_mode() == SHOOT_BULLET)//����
	{
		if(shoot->single_bullet_flag == 0)
		{
			shoot->trigger_position_angle_set += SINGLE_BULLET_ANGLE_INC;	
			shoot->single_bullet_flag = 1;//���ñ�־λ��ֹ������	
		}
//		if(shoot->revolution_bullet_flag==0)			
//		{
//			shoot->trigger_position_angle_set -= SINGLE_BULLET_ANGLE_INC;	
//			shoot->revolution_bullet_flag = 1;//���ñ�־λ��ֹ������
//		}
		if(shoot_abs(shoot->trigger_position_angle_fdb - shoot->trigger_position_angle_set) < 15.0f)//0.2f
		{
			shoot->single_bullet_flag = 0;
			if(shoot->mouse_right_long_press_flag == 0)//�ж��Ƿ���Ҫ����
			{
				set_shoot_mode(SHOOT_READY);//����Ҫ����תΪ׼��ģʽ
				
			}
		}
		else shoot->revolution_bullet_flag=0;
		
		
		
	}
	//fric
	if(get_fric_mode() != FRIC_WHEEL_OFF)
	{
		shoot->fric1_set =  -shoot_mag.allow_fri_speed;
		shoot->fric2_set =  shoot_mag.allow_fri_speed;
		// shoot->fric1_set =  -FRIC_TEXT_SPEED;//����
		// shoot->fric2_set =  FRIC_TEXT_SPEED;
	}
	else
	{
		shoot->fric1_set = 0.0;
		shoot->fric2_set = 0.0;
	}

	shoot->fric1_fdb = (float)shoot->fric_motor1_msg->encoder.filter_rate;//����ٶ�
	shoot->fric2_fdb = (float)shoot->fric_motor2_msg->encoder.filter_rate;
}
/**
  * @brief        pid����
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_cascade_pid_calculate(shoot_pid_t *shoot_pid, shoot_control_data_t *shoot)
{
	//trigger pid
	shoot_pid->trigger_pid.position_pid.set = shoot->trigger_position_angle_set;
	shoot_pid->trigger_pid.position_pid.fdb = shoot->trigger_position_angle_fdb;
	shoot_pid->trigger_pid.position_pid.Calc(&shoot_pid->trigger_pid.position_pid);							//λ�û�
	
	shoot_pid->trigger_pid.speed_pid.set = shoot_pid->trigger_pid.position_pid.output;
	shoot_pid->trigger_pid.speed_pid.fdb = shoot->trigger_motor_msg->encoder.filter_rate;
	shoot_pid->trigger_pid.speed_pid.Calc(&shoot_pid->trigger_pid.speed_pid);										//�ٶȻ�
	
	shoot->given_current.trigger_motor = shoot_pid->trigger_pid.speed_pid.output;
	//fric1 pid
	shoot_pid->fric_1_pid.set = shoot->fric1_set;
	shoot_pid->fric_1_pid.fdb = shoot->fric1_fdb;
	shoot_pid->fric_1_pid.Calc(&shoot_pid->fric_1_pid);																			  	//λ�û�
	                                                                                            
	shoot->given_current.fric1_motor = shoot_pid->fric_1_pid.output;                            
	//fric2 pid                                                                                 
	shoot_pid->fric_2_pid.set = shoot->fric2_set;                                               //�ٶȻ�
	shoot_pid->fric_2_pid.fdb = shoot->fric2_fdb;
	shoot_pid->fric_2_pid.Calc(&shoot_pid->fric_2_pid);																				
	
	shoot->given_current.fric2_motor = shoot_pid->fric_2_pid.output;
}
/**
  * @brief         can��������ѭ��
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_control_loop(shoot_control_data_t *shoot)
{
	// if(get_robot_control_mode() == GUI_CALI_MODE)            
	// {
	//  	set_shoot_stop();
	// }
	// else //���Է��� û��ʹ��ָ����Ϊ��ڲ��� ���������޸�can��صĺ���
	// {
		set_shoot_behaviour(shoot->given_current.fric1_motor,    \
							shoot->given_current.fric2_motor,   \
							shoot->given_current.trigger_motor  ); 
	
	//}
}
/**
  * @brief         �������pid��ʼ���Ӻ���
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}
/**
  * @brief         ���������ʼ������
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_init(shoot_pid_t *shoot_pid, cali_shoot_t *cali_pid, shoot_control_data_t *shoot)
{
	shoot->rc_ctrl = get_rc_data_point();
	shoot->trigger_motor_msg = get_trigger_motor_msg_point();
	shoot->fric_motor1_msg = get_fric_motor1_msg_point();
	shoot->fric_motor2_msg = get_fric_motor2_msg_point();
	
	shoot->trigger_position_angle_set = (shoot->trigger_motor_msg->encoder.ecd_bias)*  \
										 TRIGGER_ECD_TO_ANGLE;//��ʼ��λ��
	//shoot->trigger_position_angle_set = 2000;
	//trigger 
	shoot_pid_init(&shoot_pid->trigger_pid.position_pid, &cali_pid->trigger_pid.position);
	shoot_pid_init(&shoot_pid->trigger_pid.speed_pid, &cali_pid->trigger_pid.speed);
	//fric
	shoot_pid_init(&shoot_pid->fric_1_pid, &cali_pid->fric1_pid);
	shoot_pid_init(&shoot_pid->fric_2_pid, &cali_pid->fric2_pid);
	
	set_shoot_mode(SHOOT_STOP);
	set_fric_mode(FRIC_WHEEL_OFF);
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			������������ֿ��ؿ���        
  * @note           
  */
void shoot_magazine_control(shoot_control_data_t *shoot)
{
	if(shoot->rc_ctrl->rc.s2 == RC_SW_UP )                 //����ģʽ   
	{
		if(shoot->rc_ctrl->key.v  & MAGAZINE_CONTROL)      
		{
			shoot->magazine_control_flag = 1; 
		}
		if (shoot->rc_ctrl->key.v & SHOOT_CONTROL)         
		{
			shoot->shoot_control_flag = 1;
		}
		if (shoot->rc_ctrl->key.v & VISION_BUFF)
		{
			shoot->shoot_vision_flag = 1;
		}
		if (shoot->rc_ctrl->key.v & VISION_BUFF_OVER)
		{
			shoot->shoot_vision_flag = 0;
		}
	}
	if(shoot->rc_ctrl->rc.s2 == RC_SW_DOWN )                 //ң��ģʽ    
	{
		if(shoot->rc_ctrl->rc.ch4== RC_CH4_MAX )	
		{
			shoot->magazine_control_flag = 1; 
		}
		else if(shoot->rc_ctrl->rc.ch4== -RC_CH4_MAX ) 
		{
			shoot->magazine_control_flag = 0;
		}
	}
	
	if(shoot->magazine_control_flag )
	{
		pwm1_on(3894);					//���ƶ����
	}
	else 
	{
		pwm1_on(1850);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void shoot_task(void *argument)
{
	
	TickType_t current_time = 0;
	
	vTaskDelay(SHOOT_TASK_INIT_TIME);
	shoot_init(&shoot_pid, &cali_shoot_pid, &shoot_control_data);
	while(1)
	{	
		current_time = xTaskGetTickCount();                     // ��ǰϵͳʱ�� 
		shoot_magazine_control(&shoot_control_data);			// �����������  
		shoot_limit();                                          // ��������
		get_shoot_related_flags(&shoot_control_data);						//��ȡ����
		shoot_mode_update(&shoot_control_data, &shoot_pid);			//ģʽ����
		shoot_set_and_fdb_update(&shoot_control_data);					//pidֵ����
		shoot_cascade_pid_calculate(&shoot_pid, &shoot_control_data);			//pid����
		shoot_control_loop(&shoot_control_data); 								//�����������ֵ                   
		
		vTaskDelayUntil(&current_time, SHOOT_TASK_TIME_1MS);        //1msһ��       *hyj
	}	
}

