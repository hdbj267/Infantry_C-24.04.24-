/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate��
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: ����AC��֮�����Ϣ����
 * @Note:       
 * @Others: 
**/
#include "connect_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "remote_app.h"
#include "can1_app.h"
#include "gimbal_task.h"
#include "can.h"
#include "can2_app.h"
#include "tim.h"
#include "semphr.h"
#include"GUI_task.h"

extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl_data;
extern gimbal_control_data_t gimbal_control_data;
extern ext_Judge_data_t Judge_data;
extern INS_t INS;
extern vision_control_data_t control_data;
connect_t connect_data;
_tx2_tx_data vision_tx_data;



/**
* @brief        can2��ȡrc���� ����������
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void can2_get_rc_data(connect_t *connect_data)							
{
	connect_data->can2_rc_ctrl.control_mode = get_robot_control_mode();
	connect_data->can2_rc_ctrl.work_mode = get_robot_work_mode();
	connect_data->can2_rc_ctrl.rc.ch2 = connect_data->rc_ctrl->rc.ch2 + \
													  RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch3 = connect_data->rc_ctrl->rc.ch3 + \
													  RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.mouse.key = connect_data->rc_ctrl->key.v;
}
void send_rc_to_chassis(can2_rc_ctrl_t *can2_rc_ctrl)  
{
	can2_tx_header.StdId = CAN2_CONNECT_RC_CTRL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(can2_rc_ctrl->control_mode);
    can2_tx_data[1] = (uint8_t)(can2_rc_ctrl->work_mode);
    can2_tx_data[2] = (uint8_t)(can2_rc_ctrl->rc.ch2 >> 8);
    can2_tx_data[3] = (uint8_t)(can2_rc_ctrl->rc.ch2);
    can2_tx_data[4] = (uint8_t)(can2_rc_ctrl->rc.ch3 >> 8);
    can2_tx_data[5] = (uint8_t)(can2_rc_ctrl->rc.ch3);
    can2_tx_data[6] = (uint8_t)(can2_rc_ctrl->mouse.key >> 8);
    can2_tx_data[7] = (uint8_t)(can2_rc_ctrl->mouse.key);
	extern SemaphoreHandle_t hcan2_mutex;
	xSemaphoreTake(hcan2_mutex, 100);
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
	xSemaphoreGive(hcan2_mutex);
}

extern int8_t vision_flag;

void send_uiflag_to_chassis(void)  
{
	can2_tx_header.StdId = CAN2_CONNECT_UIFLAG_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x02;
    
		uint8_t ui_data[8];
    ui_data[0] = (uint8_t)(control_data.recog_flag);		//���յ�С�������ݵı�־λ
    ui_data[1] = (int8_t)(vision_flag);									//С���Կ����ı�־λ
    ui_data[2] = 0;
    ui_data[3] = 0;
    ui_data[4] = 0;
    ui_data[5] = 0;
    ui_data[6] = 0;
    ui_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, ui_data, (uint32_t *) CAN_TX_MAILBOX1  );
}

/**
* @brief        can2��ȡGYRO���������ݣ�����������
  * @author         
  * @param[in]      
  * @retval			
  * @note           
*/
typedef union float_char_
{
	float f;
	uint8_t c[4];
}float_char;
extern gimbal_control_data_t gimbal_control_data;
void send_gyro_data_to_chassis(void)  
{
	can2_tx_header.StdId = CAN2_CONNECT_CM_GYRO_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
	float_char f_c;
	f_c.f = (gimbal_control_data.gimbal_yaw_set*10);
	can2_tx_data[0] = f_c.c[0];
	can2_tx_data[1] = f_c.c[1];
	can2_tx_data[2] = f_c.c[2];
	can2_tx_data[3] = f_c.c[3];
	f_c.f = (gimbal_control_data.gimbal_INS->yaw_angle*10);
	can2_tx_data[4] = f_c.c[0];
	can2_tx_data[5] = f_c.c[1];
	can2_tx_data[6] = f_c.c[2];
	can2_tx_data[7] = f_c.c[3];
	extern SemaphoreHandle_t hcan2_mutex;
	xSemaphoreTake(hcan2_mutex, 100);
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
	xSemaphoreGive(hcan2_mutex);
}

uint8_t tx_data[22];
 communication_t communication;

void vision_tx(_tx2_tx_data *vision_tx_data,INS_t *INS)
{
		if(Judge_data.robot_id < 10) //������ɫ
		vision_tx_data->robot_color=0;
		else
		vision_tx_data->robot_color=1;
		if(connect_data.can2_rc_ctrl.work_mode==4||connect_data.can2_rc_ctrl.work_mode==5)
		vision_tx_data->robot_mode=4;
		else
		vision_tx_data->robot_mode=0;
		vision_tx_data->robot_id=2;
		vision_tx_data->gyro_x=INS->gyro_y;
		vision_tx_data->gyro_y=INS->gyro_x ;
		vision_tx_data->yaw=INS->yaw_angle ;
		vision_tx_data->pitch=INS->roll_angle;
		//tx_data.gyro_accle
		vision_tx_data->bullet_speed=Judge_data.bullet_speed;
		tx_data[0]='S';
		tx_data[1]=vision_tx_data->robot_color;
		tx_data[2]=vision_tx_data->robot_mode;
		tx_data[3]=vision_tx_data->robot_id;
		unsigned char* ptr = (unsigned char*)&(vision_tx_data->yaw);
		tx_data[4] = *(ptr + 0);
		tx_data[5] = *(ptr + 1);
    tx_data[6] = *(ptr + 2);  // ��ȡ��λ�ֽ�
    tx_data[7] = *(ptr + 3);
		ptr = (unsigned char*)&(vision_tx_data->pitch);
		tx_data[8] =  *(ptr +  0);
		tx_data[9] =  *(ptr +  1);
    tx_data[10] = *(ptr + 2);  // ��ȡ��λ�ֽ�
    tx_data[11] = *(ptr + 3);		
//		tx_data[12]=(vision_tx_data->gyro_accle);
//		tx_data[13]=(vision_tx_data->gyro_accle<<8);
		communication.aaa = (float)(vision_tx_data->gyro_x);
    tx_data[12] =  communication.bbb[0];  // ��ȡ��λ�ֽ�
    tx_data[13] =  communication.bbb[1];
		tx_data[14] =  communication.bbb[2];
		tx_data[15] =  communication.bbb[3];
		communication.aaa = (float)(vision_tx_data->gyro_y);
    tx_data[12] =  communication.bbb[0];  // ��ȡ��λ�ֽ�
    tx_data[13] =  communication.bbb[1];
		tx_data[14] =  communication.bbb[2];
		tx_data[15] =  communication.bbb[3];
		tx_data[20]=Judge_data.bullet_speed;
		tx_data[21]='E';
		HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 1000);
}


/**
  * @brief          ���ӳ�ʼ��
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void connect_init(connect_t *connect_data)
{
	connect_data->rc_ctrl = get_rc_data_point();
	
//	vTaskSuspendAll();//�������������
//	check_connect(connect_data);//���δ���ӽ��޷��˳�ѭ������̨Ҳ���ᷢ��rc����
//	xTaskResumeAll();
}
/**
  * @brief         ���ӵ�������
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
SemaphoreHandle_t hcan2_mutex;
SemaphoreHandle_t hcan1_mutex;
void connect_task(void *argument)
{	
	// TickType_t current_time = 0;
	hcan2_mutex = xSemaphoreCreateMutex(); 
	hcan1_mutex = xSemaphoreCreateMutex();
	connect_init(&connect_data);
	while(1)
	{
		// current_time = xTaskGetTickCount();                         //��ǰϵͳʱ��       *hyj
		can2_get_rc_data(&connect_data);										//��ȡң�������ݣ�����
		
		send_rc_to_chassis(&connect_data.can2_rc_ctrl);			//����ң��������|��������������
		vision_tx(&vision_tx_data,&INS);										//����һ������̨��̬������С����
		send_uiflag_to_chassis();														//�����Ӿ���־λ�����̣����ڻ�UI
		vTaskDelay(50);                         						// 10msһ��
		   
	}
}


connect_t *get_connect_data_point(void)
{
	return &connect_data;
}
