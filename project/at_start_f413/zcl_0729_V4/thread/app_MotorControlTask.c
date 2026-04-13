#include "string.h"
#include "stdlib.h"

#include "at32f413_conf.h"
#include "delay.h"

#include "gpio_port.h"
#include "at32f413_board.h"
#include "app_MotorControlTask.h"

#include "customer_control.h"
#include "para_list.h"

#include "usart_port.h"
#include "usart_motor_bsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"	
#ifdef WDT_ENABLE
#include "event_groups.h"	
extern EventGroupHandle_t  WDTEventGroup;
#endif
//QueueHandle_t  xQueueMotorControl;	

static uint8_t foc_flag;
static uint8_t u_motor_cmd_busy_flag=0;
static unsigned int motorErrHeatBeat;
 
extern MotorStatus_TypeDef motor_status;
extern MotorSettings_TypeDef motor_settings;

extern union Param_Union  sys_param_un; 
extern union Motor_Para_Union  motor_param_un;

#define USART_RX_CMD            0x05//gc=30,��⵽��
#define USART_RX_1CMD           0x06//5~27 gc=18 //�������
#define USART_RX_2CMD           0x07//1~4 gc=3 //����,��������
#define USART_RX_3CMD           0x08//<=1  gc=1 //�������
#define USART_RX_4CMD           0x09//<=0  gc=0 //��������

typedef struct   
{
	unsigned char       cmdSource;             //������Դ�����������ڵȣ�
	unsigned char       motorCmd_Type;         //��������
	unsigned char       motorDeviceId;        //������
	unsigned char       controlCode;                //����ָ��ID	
	unsigned int        Data;                              //��������
}MotorCtl_TypeDef;

/**
  * @brief MOTOR_150US_Callback
  * @param  none
  * @retval none
  */
void MOTOR_150US_Callback(void)//100usĿǰ�е����⣬��Ϊ150us
{	
		
	motorErrHeatBeat=0;		
}
/**
  * @brief motor default param
  * @param  none
  * @retval none
  */
void MotorParamInit(void)
{	
	unsigned short int useNum=0;
	stop();	
//	motor_settings.mode =EndoModeSpeedForward;// 1; //���ó���Ҫ��ģʽ
//	motor_settings.autorev_mode =AutoReverseMode1;// 1; //����ת��ģʽ���Զ���תģʽ
//	motor_settings.forward_position = 30; //������ת�Ƕ�
//	motor_settings.reverse_position = -150; // ���÷�ת�Ƕ�
//	motor_settings.forward_speed = 50;//500; // �����ٶ�ģʽ����ת�ٶ�
//	motor_settings.reverse_speed = -50;//500;// �����ٶ�ģʽ�·�ת�ٶȣ��ø�����
//	motor_settings.upper_threshold = 4.0;//4.0; // ����ת�ر�����(n.cm)
//	motor_settings.lower_threshold =2.4;// 2.4; // ����ת��ȡ��������60%���ֵ
//	motor_settings.toggle_mode_speed = 500; //��������ģʽ�ٶ�
//		MenuMotorParamUpdate(sys_param_un.device_param.use_num,0);
//  MOTOR_SETTING_UPDATE	
	useNum=sys_param_un.device_param.use_p_num;	
	motor_settings.autorev_mode=AutoReverseMode1;//AutoReverseMode1;
	if(motor_param_un.system_motor_pattern[useNum].dir==EndoModeTorqueATC)
	{
		motor_settings.mode=EndoModeSpeedForward;
	}
	else
	{
		motor_settings.mode=motor_param_un.system_motor_pattern[useNum].dir;
	}	
	if(motor_param_un.system_motor_pattern[useNum].motorSpeedNum >MAX_spd_Rpm_num) motor_param_un.system_motor_pattern[useNum].motorSpeedNum=MAX_spd_Rpm_num;	
	if(motor_param_un.system_motor_pattern[useNum].toggleSpeedNum >spd600_Rpm_num) motor_param_un.system_motor_pattern[useNum].toggleSpeedNum=spd600_Rpm_num;			
	motor_settings.forward_speed=speed_list[motor_param_un.system_motor_pattern[useNum].motorSpeedNum];
	motor_settings.reverse_speed=-motor_settings.forward_speed;			
	motor_settings.toggle_mode_speed=speed_list[motor_param_un.system_motor_pattern[useNum].toggleSpeedNum];//
	motor_settings.forward_position=motor_param_un.system_motor_pattern[useNum].forwardPosition;
	motor_settings.reverse_position=motor_param_un.system_motor_pattern[useNum].reversePosition;			
	if(motor_param_un.system_motor_pattern[useNum].dir==EndoModeTorqueATC)
	{			
		motor_settings.upper_threshold=torque_list[torque40_Ncm]*0.1f;//torque_list[MAX_torque_42_Ncm]*0.1;
		if(motor_param_un.system_motor_pattern[useNum].atcTorqueThresholdNum>torque20_Ncm)
		{
			motor_settings.lower_threshold=torque_list[torque20_Ncm]*0.1f;
		}
		else
		{			
			motor_settings.lower_threshold=torque_list[motor_param_un.system_motor_pattern[useNum].atcTorqueThresholdNum]*0.10f;		
		}			
	}
	else if(motor_param_un.system_motor_pattern[useNum].dir==EndoModePositionToggle)
	{		
		motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[useNum].recTorqueThresholdNum]*0.10f;
		motor_settings.lower_threshold=motor_settings.upper_threshold*0.6f;//*0.6
	}
	else 
	{					
		motor_settings.upper_threshold=torque_list[motor_param_un.system_motor_pattern[useNum].torqueThresholdNum]*0.10f;			
		motor_settings.lower_threshold=motor_settings.upper_threshold*0.6f;//60%		
	}				
	update_settings(&motor_settings);	
}
/**
  * @brief  motor run status monitor
  * @param   
  * @retval none
  */
static void MotorStatusMonitor(unsigned short int perTimeMs)
{	 
	
}

/**
  * @brief  motor  run in certain mode,ָ���·�
  * @param  uint8_t run_flag,run mode 
  * @retval none
  */
unsigned char App_MotorControl(unsigned char cmd)
{
	unsigned char err;
	err=0;
	if(cmd>MOTOR_SETTING_ERR) return err;				
	//cmd 0,stop,1 update,2 start
	u_motor_cmd_busy_flag=1;	
	switch(cmd)
	{	
		case MOTOR_MODE_STOP:	
			if(foc_flag!=0)
			{	
				stop();	
				foc_flag=0;	
			}
												
		#ifdef LED_INDICATE_ENABLE		
			LedFunctionSet( LED_B ,500,LED_T_HIGH_PRIORITY,LED_OFF);
		#endif			
			break;		
		case MOTOR_MODE_START:			
			if(foc_flag==0)		
			{	
				start();
				foc_flag=1;							
			}	
			vTaskDelay(5);//
			#ifdef LED_INDICATE_ENABLE		
				LedFunctionSet(LED_B ,500,LED_T_HIGH_PRIORITY,LED_KEEP_ON);		
			#endif		
			break;
		case MOTOR_SETTING_UPDATE:		
				update_settings(&motor_settings);	
			break;	
		case MOTOR_MODE_SEARCH_ANGLE:	
				stop();	
				foc_flag=0;
				vTaskDelay(5);						
				app_u_motor_angle_cali();//???????
				//delay_ms(4000);
				#ifdef WDT_ENABLE
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);
				xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
				vTaskDelay(MAX_WDT_FEED_TIME_MS);				
				#endif
				app_u_motor_angle_cali_next();
				#ifdef DEBUG_RTT
				SEGGER_RTT_printf(0, "motor_angle%d\r\n", err);	
				#endif						
				#ifdef LED_INDICATE_ENABLE
					LedFunctionSet( LED_B ,500,LED_T_HIGH_PRIORITY,LED_OFF);
				#endif	
		break;
		case MOTOR_MODE_RESTART:
			stop();				
			if(foc_flag!=0)
			{
				foc_flag=0;								
			}
			update_settings(&motor_settings);				
			MotorStatusMonitor(0);	       // status monitor 20ms periodic
	
			vTaskDelay(15);
			foc_flag=1;	
			start();	
		break;			
		case MOTOR_SETTING_ERR:		
			stop();//err handle	
			foc_flag=0;			
			vTaskDelay(5);//20ms
		#ifdef DEBUG_RTT
			SEGGER_RTT_printf(0,"motor control err ID%d\r\n", motorErrHeatBeat);	
		#endif		
			break;		
		default:
			stop();
			foc_flag=0;
			break;
	}
	return err;	
}
/*=======================motor control task===================================
	* @brief:  motor control task.
	* @param:  None
	* @retval: None
==============================================================================*/
/*
unsigned char head1;
	unsigned char head2;
	unsigned char len;
	unsigned char id;	//head
	unsigned char device_enable;
	unsigned char fault_type;
	unsigned char motor_state;
	float current;
	float speed;
	float position;
	unsigned char check_sum;	
	*/
void vAppMotorControlTask( void * pvParameters )
{	
	unsigned int count,countS;
	for(;;)	
	{		
		count+=10;
		if(countS>=20)
		{
			countS=0;					
			SEGGER_RTT_printf(0, "1s iq=%.2fmA t=%d spd=%.2f\r\n", u_motor_sta_replay.sta.current*1000,GetRealTorque(),u_motor_sta_replay.sta.speed);				
		}
		if(count>=50)
		{
			countS++;
			count=0;
		   	if(u_motor_bus_idle_sta.idlesta==0)
			{	
				app_u_motor_get_sta_req();
				u_motor_bus_idle_sta.timeout=0;
			}
			else 
			{
				u_motor_bus_idle_sta.timeout++;
				if(u_motor_bus_idle_sta.timeout>20) u_motor_bus_idle_sta.idlesta=0;//timeout
			}
		}
		vTaskDelay(5);//5ms
		app_u_motor_rec_data();
	 	MotorStatusMonitor(10 );	// status monitor 20ms periodic	
		if(foc_flag) customer_control();	
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
		#endif
		vTaskDelay(5);//5ms		
	}
}
