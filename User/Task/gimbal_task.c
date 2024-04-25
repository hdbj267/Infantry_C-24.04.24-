/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: 关于云台的控制
 * @Note:       
 * @Others: 
**/
#include "gimbal_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "pid.h"
#include "INS_task.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "monitor_task.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "start_task.h"
#include "gimbal_task.h"
#include "can2_app.h"
#include "flash.h"
#include "usart.h"
#include "shoot_task.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "vision.h"
#include "stm32f4xx_hal_cortex.h"
#include "math.h"
uint8_t view_control_flag=0;
extern uint8_t gimbal_init_ok_flag;

extern vision_control_data_t control_data;
extern TaskHandle_t INS_Task_Handler;
extern monitor_t monitor;

gimbal_pid_t gimbal_pid;
gimbal_control_data_t gimbal_control_data;
robot_work_mode_e robot_work_mode;
robot_control_mode_e robot_control_mode;
gimbal_work_mode_e gimbal_work_mode;
extern shoot_control_data_t shoot_control_data;

extern ext_Judge_data_t Judge_data;
extern shoot_control_data_t shoot_control_data;
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//机器人工作模式 
void set_robot_control_mode(robot_control_mode_e mode)	//***
{
	robot_control_mode = mode;
}
uint8_t get_robot_control_mode(void)
{
	return robot_control_mode;
}
void robot_control_mode_update(RC_ctrl_t *rc_s)	
{
	//control mode
	switch(rc_s->rc.s2) 
	{
		case RC_SW_UP:
			set_robot_control_mode(KEY_MOUSE_MODE);break;
		case RC_SW_MID:
			set_robot_control_mode(REMOTE_MODE);break;
		case RC_SW_DOWN:
			set_robot_control_mode(GUI_CALI_MODE);break;
	}
//	 if(monitor.exist_error_flag == 1)//发生严重错误时，不受遥控指挥强行转为调试模式
//	 {
//	 	set_robot_control_mode(GUI_CALI_MODE);
//	 }
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//uint8_t start_time_flag = 0;
//int32_t start_time_us = 0;
//int32_t current_time_us = 0;
uint32_t time_tick_ms = 0;
uint8_t gimbal_position_init_finish_flag = 0;

//robot_work_mode
void set_robot_work_mode(robot_work_mode_e mode)
{
	robot_work_mode = mode;
}
uint8_t get_robot_work_mode(void)
{
	return robot_work_mode;
}
void robot_work_mode_update(RC_ctrl_t *rc_s)  // **
{
	//work mode
	if(gimbal_position_init_finish_flag == 0 && get_robot_control_mode() != GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_INIT_MODE);	//初始化	
		gimbal_position_init_finish_flag = 1;
	}
	else if(get_robot_control_mode() == REMOTE_MODE)	//遥控器控制工作模式
	{
		switch(rc_s->rc.s1)
		{
			case RC_SW_UP:
				set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);break;
			case RC_SW_MID:
				set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);break;
			case RC_SW_DOWN:
				set_robot_work_mode(ROBOT_COMMON_MODE);break;
		}
	}
	else if(get_robot_control_mode() == KEY_MOUSE_MODE)	//键鼠控制工作模式EQV
	{
		if(rc_s->key.v & ROBOT_ROTATE_STOP_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);
		}
		else if(rc_s->key.v & ROBOT_ROTATE_MOTION_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);
		}
		else if(rc_s->key.v & ROBOT_COMMON_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_COMMON_MODE);
			shoot_control_data.magazine_control_flag = 0; 
		}
	}
	else if(get_robot_control_mode() == GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_CALI_MODE);	//调试模式
		gimbal_position_init_finish_flag = 0;
	}	
	
//	if(get_robot_work_mode() == ROBOT_INIT_MODE)
//	{
//		GREEDLED_ON();
//	}
//	else 
//	{
//		GREEDLED_OFF();
//	}
}	
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//云台工作模式
void set_gimbal_work_mode(gimbal_work_mode_e mode)
{
	gimbal_work_mode = mode;
}
uint8_t get_gimbal_work_mode(void)
{
	return gimbal_work_mode;
}

void gimbal_work_mode_update(RC_ctrl_t *rc_s, gimbal_control_data_t *gimbal_control_data)
{
	robot_control_mode_update(rc_s);
	robot_work_mode_update(rc_s);

	if(get_robot_work_mode() == ROBOT_CALI_MODE)
	{
		set_gimbal_work_mode(GIMBAL_CALI_MODE);
	}
	else if(get_robot_work_mode() == ROBOT_INIT_MODE)
	{
		set_gimbal_work_mode(GIMBAL_RELATIVE_ANGLE_MODE);		
	}
	else 
	{
		set_gimbal_work_mode(GIMBAL_ABSOLUTE_ANGLE_MODE);		
	}
	
	switch(get_gimbal_work_mode())
	{
		case GIMBAL_RELATIVE_ANGLE_MODE:         //编码值   *hyj
		{
			gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
			gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
		}break;
		case GIMBAL_ABSOLUTE_ANGLE_MODE:         //陀螺仪   *hyj
		{
			gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
			gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
		}break;
	}
}	
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
#define MOUSE_SINGLE_TIME_ALLOW_VALUE  38   //150
void gimbal_val_limit(int16_t *val, int16_t min, int16_t max)
{
	if(*val < min)
	{
		*val = min;
	}
	else if(*val > max)
	{
		*val = max;
	}
}
void mouse_sensit_cali(RC_ctrl_t *rc_ctrl)
{
	if( rc_ctrl->mouse.x < 20)
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT;
	}
	
	if ( rc_ctrl->mouse.y < 20)
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT;
	}
}


float abs_fun(float a)
{
	if(a < 0) a = -a;
	return a;
}

float vision_K = 1;
int ROBOT_ROTATE_MOTION_MODE_flag;


/*目前步兵用的是三串级pid。实际测试时发现，虽然控制得比较稳定（不怎么抖），但是响应太慢，
抗干扰能力也不行，对面一开小陀螺就跟不上了，还是得改成和哨兵一样的双串级pid*/

pid_t delta_yaw= 
{
	.kp = 0.0035,//0.007
	.ki = 0,//0.00005
	.kd = 0.2,//0.2
	.ioutMax = 100,
	.outputMax = 5000,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};
pid_t delta_pitch= 
{
	.kp = 0.003,//0.65
	.ki = 0,//0.006
	.kd = 0.15,
	.ioutMax = 100,
	.outputMax = 500,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};

int WINDOW_SIZE=2000;
float sum = 0;
float output;
int i;
float slidingWindowFilter(float input) {
    sum += input;
		i++;
		if(i>(WINDOW_SIZE-1))
		{
    output = sum / WINDOW_SIZE;
		sum=0;
		i=0;
		}
		return output;
		
}

float delta_yaw_dev,delta_pitch_dev,delta_pitch_dev_y1=-1.0,delta_pitch_dev_y2=3.0,gyroy_aver,last_gyroy_aver,final_gyroy_aver,numnum=0.1;
uint8_t delta_yaw_dev_flag=0,enemy_rotate,rotate_deduce,right_num,left_num,break_rotate,time1=1000,key_trigger_num;
int left_time,right_time;
int8_t vision_flag=-1;

void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,
							   robot_control_mode_e robot_control_mode,							   
							   vision_control_data_t control_data)	//*****
{	
/*根据yaw轴的运动判断对方是否开启小陀螺，写这个是为了做自瞄补偿用，当时时间紧没用上*/
	switch(rotate_deduce)
	{
		case 0:
			if(gimbal_control_data->gimbal_INS->gyro_y>5)
			{
			rotate_deduce=1;
			left_num+=1;
			}break;
		case 1:
			if(gimbal_control_data->gimbal_INS->gyro_y<-5)
			{
			rotate_deduce=0;
			right_num+=1;
			}
			break;
		default:break;
	}
		if(gyroy_aver<20)
	{
	enemy_rotate=0;
	}
	else if((right_num>3&&left_num>3)&&(gyroy_aver>20))
	{
	enemy_rotate=1;
	right_num--;
	left_num--;
	}
	
		/*下面这一部分的代码是尝试进行做视觉预测的部分，基本逻辑：由于自瞄时，云台的运动基本是和敌方装甲板的运动绑定的
	所以对云台的yaw轴角速度进行长时间的取平均，得到比较平滑的角速度，该部分目前未实践，只是写了个接口*/
	gyroy_aver=slidingWindowFilter(abs_fun(gimbal_control_data->gimbal_INS->gyro_y));
	if(delta_yaw_dev_flag)
	delta_yaw_dev=gyroy_aver*numnum;
	else
	delta_yaw_dev=0;	
	
	
	

	if(gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO)//mpu
	{
		
	//f按键自瞄，按一次开，按一次关	
			switch(key_trigger_num)
	{
		case 0:
			if(gimbal_control_data->rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)
			{
				vision_flag=-vision_flag;
				key_trigger_num=1;
			}break;
		case 1:
			if(!(gimbal_control_data->rc_ctrl->key.v & KEY_PRESSED_OFFSET_F))
			{
				key_trigger_num=0;
			}break;
		default: break;
	}

//步兵的自瞄打得比较准，	delta_yaw_dev和delta_pitch_dev是做敌方移动补偿用的，如果视觉后面预测做得比较好应该就不需要了
		if(robot_control_mode == KEY_MOUSE_MODE ) //KEY_MOUSE_MODE
		{	
			if(vision_flag==1)     //按键视觉自瞄     *hyj  
			{
				if(control_data.recog_flag==1 )
			{
				//delta_pitch_dev=-(control_data.Target_distance-5000.0)/4000.0*(delta_pitch_dev_y1-delta_pitch_dev_y2)+delta_pitch_dev_y2;//y=(x-x2)/(x1-x2)*(y1-y2)+y2  (1000,-1) (5000,3)
				delta_yaw.set=0;
				if(control_data.yaw_sign)
					delta_yaw.fdb=control_data.yaw_dev+delta_yaw_dev;
				else
					delta_yaw.fdb=-control_data.yaw_dev-delta_yaw_dev;
				delta_yaw.Calc(&delta_yaw);
				gimbal_control_data->gimbal_yaw_set += delta_yaw.output;
				
				delta_pitch.set=0;
				if(control_data.pitch_sign)
					delta_pitch.fdb=control_data.pitch_dev-delta_pitch_dev;
				else
					delta_pitch.fdb=-control_data.pitch_dev-delta_pitch_dev;
				delta_pitch.Calc(&delta_pitch);
				gimbal_control_data->gimbal_pitch_set += delta_pitch.output;
				control_data.recog_flag=0;//视觉标志位清零，防止重复计算pid
			}
			else
			{
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.x, \
								-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
								MOUSE_SINGLE_TIME_ALLOW_VALUE);
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.y, \
								-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
								MOUSE_SINGLE_TIME_ALLOW_VALUE);
				mouse_sensit_cali(gimbal_control_data->rc_ctrl);

				gimbal_control_data->gimbal_yaw_set += (gimbal_control_data->rc_ctrl->mouse.x) *   gimbal_control_data->rc_ctrl->yaw_sensit;
				gimbal_control_data->gimbal_pitch_set += (gimbal_control_data->rc_ctrl->mouse.y) * gimbal_control_data->rc_ctrl->pitch_sensit;
			}
			}

			else
			{
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.x, \
								-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
								MOUSE_SINGLE_TIME_ALLOW_VALUE);
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.y, \
								-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
								MOUSE_SINGLE_TIME_ALLOW_VALUE);
				mouse_sensit_cali(gimbal_control_data->rc_ctrl);

				gimbal_control_data->gimbal_yaw_set += (gimbal_control_data->rc_ctrl->mouse.x) *   gimbal_control_data->rc_ctrl->yaw_sensit;
				gimbal_control_data->gimbal_pitch_set += (gimbal_control_data->rc_ctrl->mouse.y) * gimbal_control_data->rc_ctrl->pitch_sensit;
			}
		}
		
		else if(robot_control_mode != KEY_MOUSE_MODE) //REMOTE_MODE		调试视觉用
		 //else if(robot_control_mode == REMOTE_MODE ) //REMOTE_MODE
		{
			//视觉处理     *hyj
			if(control_data.recog_flag==1 )
			{
				//delta_pitch_dev=-(control_data.Target_distance-5000.0)/4000.0*(delta_pitch_dev_y1-delta_pitch_dev_y2)+delta_pitch_dev_y2;//y=(x-x2)/(x1-x2)*(y1-y2)+y2  (1000,-1) (5000,3)
				delta_yaw.set=0;
				if(control_data.yaw_sign)
					delta_yaw.fdb=control_data.yaw_dev+delta_yaw_dev;
				else
					delta_yaw.fdb=-control_data.yaw_dev-delta_yaw_dev;
				delta_yaw.Calc(&delta_yaw);
				gimbal_control_data->gimbal_yaw_set += delta_yaw.output;
				
				delta_pitch.set=0;
				if(control_data.pitch_sign)
					delta_pitch.fdb=control_data.pitch_dev-delta_pitch_dev;
				else
					delta_pitch.fdb=-control_data.pitch_dev-delta_pitch_dev;
				delta_pitch.Calc(&delta_pitch);
				gimbal_control_data->gimbal_pitch_set += delta_pitch.output;
				control_data.recog_flag=0;
			}
			else
			{
				delta_pitch.output=0;
				delta_pitch.iout=0;
				delta_yaw.output=0;
				delta_yaw.iout=0;
				gimbal_control_data->gimbal_yaw_set += (-gimbal_control_data->rc_ctrl->rc.ch0) *    \
													 GAMBAL_YAW_MAX_ANGLE_SET_FACT;
				gimbal_control_data->gimbal_pitch_set += (-gimbal_control_data->rc_ctrl->rc.ch1) *  \
													 GAMBAL_PITCH_MAX_ANGLE_SET_FACT;//ch1
			}
			
		}	
		gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;
		gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->roll_angle;		
	}
	else if(gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_ENCONDE)//初始化时的编码模式 set 可修改
	{
		//yaw
		gimbal_control_data->gimbal_yaw_set =  gimbal_control_data->gimbal_INS->yaw_angle\
		+((float)(GAMBAL_YAW_INIT_ENCODE_VALUE-gimbal_control_data->gimbal_yaw_motor_msg->encoder.raw_value))/GAMBAL_YAW_angle_VALUE;
		gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;	
		//pitch
		gimbal_control_data->gimbal_pitch_set = gimbal_control_data->gimbal_INS->roll_angle\
		-((float)(GAMBAL_PITCH_INIT_ENCODE_VALUE-gimbal_control_data->gimbal_pitch_motor_msg->encoder.raw_value))/GAMBAL_PITCH_angle_VALUE;	
		gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->roll_angle;

	}
	if(control_data.recog_flag==1)//
	{
	if(gimbal_control_data->gimbal_pitch_set>-80)//
	{
		gimbal_control_data->gimbal_pitch_set=-80;
	}
	else if(gimbal_control_data->gimbal_pitch_set<-101)//自瞄的话头就不低太下了，防止开小陀螺时打到自己灯条
		gimbal_control_data->gimbal_pitch_set=-101;
}
	else
	{
	if(gimbal_control_data->gimbal_pitch_set>-80)//
	{
		gimbal_control_data->gimbal_pitch_set=-80;
	}
	else if(gimbal_control_data->gimbal_pitch_set<-117)//不开自瞄最低角度开大点，利于打近战
		gimbal_control_data->gimbal_pitch_set=-117;
	}
}


/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_cascade_pid_calculate(gimbal_pid_t *gimbal_pid,       //****
					              gimbal_control_data_t *gimbal_control_data)
{
//	if(!vision_flag)
//	{
	//yaw轴角度pid计算
	gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.fdb = gimbal_control_data->gimbal_yaw_fdb;
	gimbal_pid->yaw_pid.position_pid.Calc(&gimbal_pid->yaw_pid.position_pid);
	
	//yaw轴速度pid计算
	gimbal_pid->yaw_pid.speed_pid.set = gimbal_pid->yaw_pid.position_pid.output;
	gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y;
	gimbal_pid->yaw_pid.speed_pid.Calc(&gimbal_pid->yaw_pid.speed_pid);
	
	//pitch轴角度pid计算
	gimbal_pid->pitch_pid.position_pid.set = -gimbal_control_data->gimbal_pitch_set;
	gimbal_pid->pitch_pid.position_pid.fdb = -gimbal_control_data->gimbal_pitch_fdb; 
	gimbal_pid->pitch_pid.position_pid.Calc(&gimbal_pid->pitch_pid.position_pid);
	//pitch轴速度pid计算
	gimbal_pid->pitch_pid.speed_pid.set = gimbal_pid->pitch_pid.position_pid.output;
	gimbal_pid->pitch_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_x;
	gimbal_pid->pitch_pid.speed_pid.Calc(&gimbal_pid->pitch_pid.speed_pid);	
	//}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_control_loop(gimbal_pid_t *gimbal_pid,       //****
					     gimbal_control_data_t *gimbal_control_data)
{

	//这里改为了正值
	gimbal_control_data->given_current.yaw_motor = gimbal_pid->yaw_pid.speed_pid.output;//g6020反向安装需要加负号
	gimbal_control_data->given_current.pitch_motor = gimbal_pid->pitch_pid.speed_pid.output;

	//看了下底盘，没接受CAN2_GIMBAL_STD_ID
	if(get_robot_control_mode() == GUI_CALI_MODE )//if(get_gimbal_work_mode() == GIMBAL_CALI_MODE)
	{
		set_gimbal_stop();
	}
	else 
	{
		set_gimbal_behaviour(gimbal_control_data->given_current.yaw_motor,   \
							gimbal_control_data->given_current.pitch_motor); 
	}
	
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	pid->a = cali_pid->a;
	
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_init(gimbal_pid_t *gimbal_pid,   \
				 cali_gimbal_t *cali_pid,    \
				 gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->rc_ctrl = get_rc_data_point();
	gimbal_control_data->gimbal_INS = get_INS_point();
	gimbal_control_data->gimbal_yaw_motor_msg = get_yaw_motor_msg_point();
	gimbal_control_data->gimbal_pitch_motor_msg = get_pitch_motor_msg_point();
	//yaw cascade pid
	gimbal_pid_init(&gimbal_pid->yaw_pid.position_pid, &cali_pid->yaw_pid.position);
	gimbal_pid_init(&gimbal_pid->yaw_pid.speed_pid, &cali_pid->yaw_pid.speed);
	//pitch cascade pid
	gimbal_pid_init(&gimbal_pid->pitch_pid.position_pid, &cali_pid->pitch_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch_pid.speed_pid, &cali_pid->pitch_pid.speed);
	//GIMBAL_InitArgument();

	set_robot_control_mode(GUI_CALI_MODE);
	set_robot_work_mode(ROBOT_CALI_MODE);
	set_gimbal_work_mode(GIMBAL_CALI_MODE);

}
/**
  * @brief        运行时挂起GUI任务
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */


/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note  		1.gimbal_task 跑不了oled程序 md不知道什么问题  
				2.while循环的前面两个函数消耗4%(其中第一个0.3%) 整个while消耗15%
  */
void gimbal_task(void *argument)
{
	TickType_t current_time = 0;
	
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	gimbal_init(&gimbal_pid, &cali_gimbal_pid, &gimbal_control_data);
	while(1)
	{
		
		current_time = xTaskGetTickCount();                         //当前系统时间       *hyj
		send_gyro_data_to_chassis();																//发送陀螺仪数据给底盘（底盘实现跟随云台）
		gimbal_work_mode_update(&rc_ctrl_data, &gimbal_control_data);//更新云台状态
		gimbal_set_and_fdb_update(&gimbal_control_data, robot_control_mode, control_data );//set fdb数据更新
		gimbal_cascade_pid_calculate(&gimbal_pid, &gimbal_control_data);//串级pid计算
		gimbal_control_loop(&gimbal_pid, &gimbal_control_data);//控制循环
		vTaskDelayUntil(&current_time, GIMBAL_TASK_TIME_1MS);       //1ms一次         *hyj     
	}	
}
