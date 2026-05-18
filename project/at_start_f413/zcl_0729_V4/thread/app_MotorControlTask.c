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
	motor_settings.set_cali_index=0;			
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
	unsigned char err=0;
	U_MOTOR_CTR_MESSAGE u_ctr_tx_msg;
	BaseType_t q_tx_sta;	
	if(cmd>MOTOR_SETTING_ERR) return err;				
	//cmd 0,stop,1 update,2 start	
	u_ctr_tx_msg.msg.cmdCode=cmd;
	u_ctr_tx_msg.msg.cmdLen=0;
	if(cmd==MOTOR_MODE_STOP)
	{
		#ifdef LED_INDICATE_ENABLE		
		LedFunctionSet( LED_B ,500,LED_T_HIGH_PRIORITY,LED_OFF);
		#endif	
	}
	else if(cmd==MOTOR_MODE_START)
	{
		#ifdef LED_INDICATE_ENABLE		
		LedFunctionSet(LED_B ,500,LED_T_HIGH_PRIORITY,LED_KEEP_ON);		
		#endif	
	}	
	else if(cmd==MOTOR_MODE_RESTART)
	{
		stop();
		vTaskDelay(10);

	}
	else if(cmd==MOTOR_SETTING_ERR)
	{	
		DEBUG_PRINTF("motor control err ID%d\r\n", motorErrHeatBeat);	
		
	}
	q_tx_sta=xQueueSend(xQueueMotorControlMessage, u_ctr_tx_msg.mBuff, 0);
	if(q_tx_sta!=pdTRUE)		
	{//resend
		vTaskDelay(10);
		xQueueSend(xQueueMotorControlMessage, u_ctr_tx_msg.mBuff, 0);
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
extern short forward_speed;
extern short reverse_speed;
extern unsigned short upper_threshold;		//iq upper threshold 
void vAppMotorControlTask( void * pvParameters )
{	
	unsigned int countMs,countS,cali_start_s;
	unsigned char buff[6]={0};
	U_MOTOR_CTR_MESSAGE u_ctr_rx_msg;
	U_MOTOR_CTR_MESSAGE p_tx_msg;
	BaseType_t m_ctr_sta;
	BaseType_t m_ctr_tx_sta;
	unsigned char m_run_sta=0,mStaLen;
	unsigned char sendKeyMessage=run_button_release_signal;

	motor_settings.set_cali_index=0;
	for(;;)	
	{	
		vTaskDelay(10);	
		countMs+=10;	
		mStaLen=app_u_motor_rec_data();
		if(mStaLen!=0)
		{//正常通讯
			if(countMs>=1000)
			{
				countMs=0;
				countS++;											
				DEBUG_PRINTF("1s iq=%.3fA  spd=%.2f T=%d\r\n", u_motor_sta_replay.sta.current,u_motor_sta_replay.sta.speed,GetRealTorque());
			}	
		}
		else{
			if(countMs>2000&&m_run_sta!=m_run_cali_angle)
			{			
				countMs=0;
				countS+=2;
				DEBUG_PRINTF("disconnnect timeout\r\n");
				if(m_run_sta!=m_run_stop&&motor_settings.set_cali_index==0)
				{
					stop();
					DEBUG_PRINTF("stop motor\r\n");
					p_tx_msg.msg.cmdCode=MOTOR_MODE_STOP;
					p_tx_msg.msg.cmdLen=0;	
					m_ctr_tx_sta=xQueueSend(xQueueMotorControlMessage, p_tx_msg.mBuff, 0);
					if(m_ctr_tx_sta!=pdTRUE)		
					{//resend
						vTaskDelay(10);
						xQueueSend(xQueueMotorControlMessage, p_tx_msg.mBuff, 10);
					}	
					app_u_motor_reset();
				}				
			}
		}		
		m_ctr_sta = xQueueReceive(xQueueMotorControlMessage, buff, 0);
		if(m_ctr_sta==pdTRUE)
		{
			memcpy(u_ctr_rx_msg.mBuff,buff,sizeof(MOTOR_CTR_MESSAGE));
			switch(u_ctr_rx_msg.msg.cmdCode)
			{
				case MOTOR_MODE_STOP:
					{
						stop();
						m_run_sta=m_run_stop;
						if(motor_settings.set_cali_index!=0){
							motor_settings.set_cali_index=0;//异常停止
						}
					}
				break;	
				case MOTOR_MODE_START:
					if(m_run_sta==m_run_stop)
					{ 						
						m_run_sta=m_run_genara;
					}
					start();
					break;
				case MOTOR_SETTING_UPDATE:				
					update_settings(&motor_settings);
					break;
				case MOTOR_MODE_SEARCH_ANGLE:
					{
						stop();	
						vTaskDelay(5);//5ms						
						#if 1
						app_u_motor_angle_cali();
						#else 
						AppUsartMotorTransmit(U_MOTOR_ID_PREPARE_MODE,&sendBuff,0);//静默
						vTaskDelay(10); 
						AppUsartMotorTransmit(U_MOTOR_ID_ENCODE_CALI,&sendBuff,0);
						#endif	
						m_run_sta=m_run_cali_angle;	
						cali_start_s=countS;	
					}
					break;
				case MOTOR_MODE_RESTART:					 
					{
						update_settings(&motor_settings);
						vTaskDelay(5);//5ms
						start();
						m_run_sta=m_run_genara;							
					}	
					break;
				case MOTOR_USART_GC_CONTROL:			
					DEBUG_PRINTF( "gc ctrl motor \r\n");								
					break;
				case MOTOR_SETTING_ERR:
					stop();
					break;
				default:
					stop();
					break;
			}					
		}
		else
		{
			if(countMs%50==0) app_u_motor_get_sta_req();
		}			
		if(m_run_sta==m_run_stop)
		{
				
		}	
		else if(m_run_sta==m_run_genara)
		{
			customer_control();	
			if(motor_settings.set_cali_index!=0)	
			{//100~2200RPM
				GetRealTorque();//自动存储
				if(countS>cali_start_s+3)
				{//运行4S
					//save current,					
					if(motor_settings.set_cali_index<=MAX_spd_Rpm_num)
					{
						DEBUG_PRINTF("cali spd=%d cur=%d\r\n",speed_list[motor_settings.set_cali_index-1],sys_param_un.device_param.m_noload_curretnRef[motor_settings.set_cali_index-1]);
						p_tx_msg.msg.cmdCode=MOTOR_MODE_START;
						p_tx_msg.msg.cmdLen=0;				
						forward_speed=speed_list[motor_settings.set_cali_index-1];
						reverse_speed=-speed_list[motor_settings.set_cali_index-1];
						upper_threshold=360;
						motor_status.mode=EndoModeSpeedForward;						
						m_ctr_tx_sta=xQueueSend(xQueueMotorControlMessage, p_tx_msg.mBuff, 0);
						if(m_ctr_tx_sta!=pdTRUE)		
						{//resend
							vTaskDelay(10);
							xQueueSend(xQueueMotorControlMessage, p_tx_msg.mBuff, 0);
						}	
						cali_start_s=countS;
						motor_settings.set_cali_index++;							
					}
					else {
						stop();
                        //恢复数据						
						update_settings(&motor_settings);
						vTaskDelay(10);
						//停止转动
						 sendKeyMessage=run_button_press_signal;		
						xQueueSend(xQueueKeyMessage, &sendKeyMessage, 0);					
						motor_settings.set_cali_index=0;
						m_run_sta=m_run_stop;
					}	
				}			
			}	
			
		}
		else if(m_run_sta==m_run_cali_angle)
		{	
			if(countMs>=1000)
			{
				countMs=0;
				countS++;											
				DEBUG_PRINTF("1s iq=%.3fA  spd=%.2f T=%d\r\n", u_motor_sta_replay.sta.current,u_motor_sta_replay.sta.speed,GetRealTorque());
			}	
			if(countS>cali_start_s+3)
			{//4秒
				cali_start_s=countS;
				app_u_motor_angle_cali_next();
				stop();
				motor_settings.set_cali_index=1;	
				vTaskDelay(10);
				p_tx_msg.msg.cmdCode=MOTOR_MODE_START;
				p_tx_msg.msg.cmdLen=0;				
				forward_speed=speed_list[0];
				reverse_speed=-speed_list[0];
				upper_threshold=360;
				motor_status.mode=EndoModeSpeedForward;	
				motor_status.mode=EndoModeSpeedForward;
				vTaskDelay(10);			
				m_ctr_tx_sta=xQueueSend(xQueueMotorControlMessage, p_tx_msg.mBuff, 0);
				if(m_ctr_tx_sta!=pdTRUE)		
				{//resend
					vTaskDelay(10);
					xQueueSend(xQueueMotorControlMessage, p_tx_msg.mBuff, 0);
				}
				xSemaphoreGive(xSemaphoreCaliFinish);	
				m_run_sta=m_run_stop;
			}	
		}		
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,MOTOR_CONTROL_TASK_EVENT_BIT);
		#endif			
	}
}
