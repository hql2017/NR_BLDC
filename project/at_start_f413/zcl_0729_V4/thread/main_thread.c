
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "para_list.h"
#include "at32f413_board.h"
#include "oled_port.h"
#include "gpio_port.h"
#include "buzz_port.h"
#include "apex_gc_port.h"
#include "adc_port.h"

#include "app_MotorControlTask.h"
#include "customer_control.h"
#include "macros.h"
#include "delay.h"

#include "key.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#ifdef WDT_ENABLE
#include "event_groups.h"	
EventGroupHandle_t  WDTEventGroup;
#endif

#define KEY_SCAN_STACK_DEPTH  64
#define KEY_SCAN_TASK_PRIORITY 	TASK_PRIORITY_HIGH
TaskHandle_t  keyScanTask_Handle;
	
#define PERIODIC_STACK_DEPTH 256
#define PERIODIC_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
TaskHandle_t  periodicTask_Handle; //周期任务	
	
#define APEX_GC_STACK_DEPTH  128
#define APEX_GC_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
TaskHandle_t  ApexGC_Task_Handle; //
	
#define MENU_MANAGE_STACK_DEPTH 256
#define MENU_MANAGE_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
TaskHandle_t  menuManageTask_Handle; //周期任务	

#define MOTOR_CONTROL_STACK_DEPTH 128
#define MOTOR_CONTROL_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE
TaskHandle_t  motorControlTask_Handle;
	
#define SECONDARY_BEEP_STACK_DEPTH  32
#define SECONDARY_BEEP_TASK_PRIORITY 	TASK_PRIORITY_MIDDLE//(TASK_PRIORITY_LOWEST-1)
TaskHandle_t  beepTemporaryTask_Handle;	
	
#define WDT_MANAGE_STACK_DEPTH  32
#define WDT_MANAGE_TASK_PRIORITY 	(TASK_PRIORITY_MIDDLE)
TaskHandle_t  WDT_ManageTask_Handle;

#ifdef BLE_FUN
	#define BLE_MANAGE_STACK_DEPTH  256
	#define BLE_MANAGE_TASK_PRIORITY 	(TASK_PRIORITY_MIDDLE)
	TaskHandle_t  AppBLETaskTask_Handle;			
#endif
	
QueueHandle_t  xQueueKeyMessage;//key
QueueHandle_t  xQueueBeepMode;//beep
QueueHandle_t  xQueueMenuValue;//menu	
QueueHandle_t  xQueueBatValue;//batValue	
	
SemaphoreHandle_t xSemaphoreDispRfresh;			
SemaphoreHandle_t xSemaphorePowerOff;

#ifdef  APEX_FUNCTION_EBABLE
SemaphoreHandle_t xSemaphoreApexAutoCalibration;
#endif
	
extern void vAppPeriodicTask( void * pvParameters );	
extern void vAppMotorControlTask( void * pvParameters );
extern void vAppMenuManageTask( void * pvParameters );
extern void MenuDevicePowerOff(unsigned char feedDogFlag);
	
#if configUSE_TIMERS
TimerHandle_t xTimer01;
#endif

#ifdef  APEX_FUNCTION_EBABLE
#define MIN_GC_PWM_VALID_TIME  40//Minimum switching time Ms(40Hz>30  8000>20)
#define GC_ADC_DELAY_TIME  3//采集延迟

uint16_t   apex_ValueBuff[8]={0};  //根尖参考值
/**
  * @brief 	GC_PWM_SwitchTimeManage
  * @param  systemTime
  * @retval none
  */
static error_status  GC_PWM_SwitchTimeManage(unsigned int systemTime,unsigned int targetTime)
{	
	error_status err;	
	static  unsigned int recTicks;
	err=ERROR;
	if(targetTime==0||recTicks>systemTime) 
	{
		recTicks=systemTime;		
	}
	else
	{
		if(recTicks+targetTime<systemTime)
		{
			recTicks=systemTime;
			err=SUCCESS;
		}		
	}	
	return err;
}

#endif

#ifdef BLE_FUN
/**
  * @brief 	BLE_Reset
  * @param  systemTime
  * @retval none
  */
static void  BLE_Reset(void)
{	
	BLE_power_off(0);
	BLE_rst(0)	;
	vTaskDelay(10);  //10ms
	BLE_power_off(1);
	#ifdef WDT_ENABLE
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(250);  //100ms
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	#endif
	BLE_rst(1);	
	vTaskDelay(10);  //10ms
}
/**
  * @brief 	BLE_masterConfig
  * @param  systemTime
  * @retval none
  */
static void  BLE_masterConfig(void)
{	
	unsigned short int timeOutMs;
	static  AT_CMD_NUMBER cmd;
	#ifdef WDT_ENABLE
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);	
	#endif
	BLE_transmit(AT_MODE_ENABLE,TYPE_AT_HANDLE,NULL);	
	while(MCU_at_cmd_analysis(AT_MODE_ENABLE)!=AT_ONLY_ACK)
	{		
		vTaskDelay(5);  //1ms
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);			
		#endif		
	}
	#ifdef WDT_ENABLE
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	#endif	
	BLE_transmit(AT_MAC,TYPE_AT_SEARCH,NULL);
	while( MCU_at_cmd_analysis(AT_MAC)!=AT_MAC)	
	{
		vTaskDelay(5);  //1ms
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif	
	}	
	#ifdef WDT_ENABLE
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	#endif	
	cmd=AT_NULL;
	BLE_transmit(AT_NAME,TYPE_AT_SEARCH,NULL);
	while(cmd==AT_NULL)
	{
		cmd=MCU_at_cmd_analysis(AT_NAME);
		vTaskDelay(5);  //1ms
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif
	}		
	#ifdef WDT_ENABLE
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	#endif	
	if(cmd==AT_LOCAL_NAME_ERR)//出厂重新配置
	{	
		
		BLE_transmit(AT_ROLE,TYPE_AT_SET,NULL);
		while( MCU_at_cmd_analysis(AT_ROLE)!=AT_ONLY_ACK)		
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif				
		}		
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif				
		BLE_transmit(AT_MSERVICE,TYPE_AT_SET,NULL);	
		while( MCU_at_cmd_analysis(AT_MSERVICE)!=AT_ONLY_ACK)					
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif
		}
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif			
		BLE_transmit(AT_POWER,TYPE_AT_SET,NULL);	
		while( MCU_at_cmd_analysis(AT_POWER)!=AT_ONLY_ACK)		
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif		
		}	
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif		
		BLE_transmit(AT_OBSERVER,TYPE_AT_SET,NULL);	
		while( MCU_at_cmd_analysis(AT_OBSERVER)!=AT_ONLY_ACK)					
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif
		}
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif			
		BLE_transmit(AT_SCAN_SET,TYPE_AT_SET,NULL);	
		while( MCU_at_cmd_analysis(AT_SCAN_SET)!=AT_ONLY_ACK)		
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif		
		}	
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif	
		//	获取从机UUID							
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
		#endif	
		BLE_transmit(AT_UUID_SCAN,TYPE_AT_HANDLE,NULL);
		while( MCU_at_cmd_analysis(AT_UUID_SCAN)!=AT_ONLY_ACK)
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif
		}	
		//改名						
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
		#endif	
		BLE_transmit(AT_NAME_SET,TYPE_AT_SET,NULL);
		while( MCU_at_cmd_analysis(AT_NAME_SET)!=AT_ONLY_ACK)	
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif		
		}	
			#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif		
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		#endif			
		BLE_transmit(AT_RESTART,TYPE_AT_HANDLE,NULL);	
		while( MCU_at_cmd_analysis(AT_RESTART)!=AT_ONLY_ACK)
		{
			vTaskDelay(1);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif					
		}
//		BLE_Reset();//掉电保存		
		while( MCU_at_cmd_analysis(AT_DEVICE_START_HINT)!=AT_DEVICE_START_HINT)//等待重启成功
		{
			vTaskDelay(1);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif
			
		}	
		#ifdef WDT_ENABLE//wait >=2.4S
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
		vTaskDelay(MAX_WDT_FEED_TIME_MS);  //1ms	
		xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);	
		#else
		vTaskDelay(2400);  //2.4s 		
		#endif	
		BLE_transmit(AT_MODE_ENABLE,TYPE_AT_HANDLE,NULL);
		timeOutMs=0;
		while( MCU_at_cmd_analysis(AT_MODE_ENABLE)!=AT_ONLY_ACK)//AT mode	
		{
			vTaskDelay(5);  //1ms
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif
			timeOutMs++;
			if(timeOutMs>300)	
			{						
				timeOutMs=0;//超时重启
				BLE_transmit(AT_MODE_ENABLE,TYPE_AT_HANDLE,NULL);
				break;
			}				
		}	
	}  
}


 
/*==================================BLE task===================================
			* @brief:  BLE task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppBLETask( void * pvParameters )
{ 
	static unsigned char BLE_state=BLE_MASTER_CONFIG;	
	task_notify_enum sendMessage=null_signal;
	static unsigned int timeOut;
	TickType_t heatTime=xTaskGetTickCount();
	TickType_t iDletime=xTaskGetTickCount();
	AT_CMD_NUMBER cmdN;
	BLE_2340C2_gpio_init();
	#ifdef WDT_ENABLE
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(MAX_WDT_FEED_TIME_MS);  //10ms
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	vTaskDelay(MAX_WDT_FEED_TIME_MS);  //10ms
	xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
	ble_package.Len=0;
	#else 
	vTaskDelay(1000);  //1s
	#endif

	for(;;)
	{			
      #ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
			#endif	
			if(sys_param_un.device_param.BLE_Connect == BLE_OFF&&BLE_state!=BLE_MASTER_CONFIG&&BLE_state!=BLE_POWER_OFF)	//off					
			{
				BLE_Reset();			
				BLE_state=BLE_POWER_OFF;	//off			
			}						
			if(sys_param_un.device_param.BLE_Connect == BLE_CONNECT_TRX)
			{
				if(iDletime>xTaskGetTickCount())
				{
					iDletime=xTaskGetTickCount();
				}
				else
				{
					if(iDletime+500<xTaskGetTickCount())
					{						
						iDletime=xTaskGetTickCount();
						sendMessage= only_standby_signal;		
						xQueueSend(xQueueKeyMessage, &sendMessage, 0);
					}
				}	
			}	
			switch(BLE_state)
			{
				case BLE_MASTER_CONFIG:		
					ble_package.Len=0;	
					BLE_Reset();						
					#ifdef WDT_ENABLE
					xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
					vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
					xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);		
					#endif
					while(MCU_at_cmd_analysis(AT_DEVICE_START_HINT)!=AT_DEVICE_START_HINT)
					{
						vTaskDelay(1);  //1ms
						#ifdef WDT_ENABLE
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
						#endif 					
					}				
					BLE_masterConfig();	
						
					#ifdef WDT_ENABLE
					xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
					vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
					xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
					#endif	
						if(sys_param_un.device_param.BLE_Connect == BLE_OFF)
						{
							BLE_state =	BLE_POWER_OFF;//进入BLE关闭状态							
							cmdN=AT_NULL;
						}
						else 
						{
							BLE_state =	BLE_SCAN_STATUS;//进入扫描状态
							BLE_transmit(AT_SCAN,TYPE_AT_HANDLE,NULL);
							cmdN=AT_NULL;
						}
					break;
				case BLE_SCAN_STATUS:							
						vTaskDelay(5);  //1ms
						#ifdef WDT_ENABLE
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
						#endif 
						timeOut+=5;
						if(timeOut%50==0)
						{
							cmdN=MCU_at_cmd_analysis(AT_SCAN);													
						}						
						if(cmdN!=AT_SCAN)
					{					
						if(timeOut>2200)//2.5s间隔重发
						{
							timeOut=0;							
							BLE_transmit(AT_SCAN,TYPE_AT_HANDLE,NULL);
						}							
						if(cmdN!=AT_NULL&&ble_package.Len!=0) ble_package.Len=0;	
						if(cmdN==UNVARNISHED_BLE_DATA_REV)	//还未彻底断开
						{
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
							#endif
							BLE_state =	BLE_CONNECT_MANAGE;//进入连接状态		
							BLE_transmit(AT_CONNECT,TYPE_AT_HANDLE,NULL);	
							cmdN=AT_NULL;
							timeOut=0;
						}								
					}						
					else
					{									
						#ifdef WDT_ENABLE
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
						vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
						#endif
						BLE_state =	BLE_CONNECT_MANAGE;//进入连接状态		
						BLE_transmit(AT_CONNECT,TYPE_AT_HANDLE,NULL);	
						cmdN=AT_NULL;
						timeOut=0;
					}
				break;
				case BLE_CONNECT_MANAGE: 
						cmdN=MCU_at_cmd_analysis(AT_CONNECT);						
						if(cmdN==CONNECTED_SUCCESS)						
						{		
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
							#endif	
							BLE_transmit(AT_TRX_CHAN,TYPE_AT_HANDLE,NULL);
							while( MCU_at_cmd_analysis(AT_TRX_CHAN)!=AT_ONLY_ACK&&MCU_at_cmd_analysis(AT_TRX_CHAN)!=AT_ERR_FAIL)
							{
								vTaskDelay(5);  //1ms
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
								#endif	
								//AT_ERR_FAIL	//已完成配置								
							}	
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
							#endif						
							BLE_transmit(AT_TTM_HANDLE,TYPE_AT_HANDLE,NULL);
							while( MCU_at_cmd_analysis(AT_TTM_HANDLE)!=AT_ONLY_ACK)
							{
								vTaskDelay(5);  //1ms
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
								#endif
							}	
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
							#endif	
							BLE_transmit(AT_MODE_DISABLE,TYPE_AT_HANDLE,NULL);
							timeOut=0;
							while( MCU_at_cmd_analysis(AT_MODE_DISABLE)!=AT_ONLY_ACK)
							{
								vTaskDelay(1);  //1ms
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
								#endif
								timeOut++;
								if(timeOut>2000)
								{
									timeOut=0;
									break;
								}
							}			
							BLE_state=BLE_APPLICATION;//进入应用数据处理状态	
						}					
					   else 
						{
							vTaskDelay(5);  //10ms
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							#endif	
							timeOut+=5;			
							if(timeOut>2000||cmdN==CONNECTED_TIMEOUT)//5s间隔重发//超时重发	
							{						
								timeOut=0;									
								BLE_state =	BLE_SCAN_STATUS;//进入扫描状态							
								cmdN=AT_NULL;							
							}	
							else if(cmdN==DISCONNECTED_HINT)	
							{
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
								vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
								#endif
								BLE_transmit(AT_CONNECT,TYPE_AT_HANDLE,NULL);//重连
							}	
					    if(cmdN!=AT_NULL&&ble_package.Len!=0)ble_package.Len=0;
							if(cmdN==UNVARNISHED_BLE_DATA_REV)	//还未彻底断开
							{
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
								vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);						
								#endif	
								BLE_transmit(AT_MODE_DISABLE,TYPE_AT_HANDLE,NULL);
								timeOut=0;
								while( MCU_at_cmd_analysis(AT_MODE_DISABLE)!=AT_ONLY_ACK)
								{
									vTaskDelay(1);  //1ms
									#ifdef WDT_ENABLE
									xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
									#endif
									timeOut++;
									if(timeOut>2000)
									{
										timeOut=0;
										break;
									}
								}														
								BLE_state=BLE_APPLICATION;//进入应用数据处理状态	
							}								
						}							
						break;
				case BLE_APPLICATION://透传模式	
						cmdN=MCU_at_cmd_analysis(UNVARNISHED_BLE_DATA_REV);
					 if(cmdN==UNVARNISHED_BLE_DATA_REV)//收到应用数据
					{						//250ms后回复标准数据包
						if(sys_param_un.device_param.BLE_Connect!=BLE_ON) 
						{
							sys_param_un.device_param.BLE_Connect=BLE_CONNECT_TRX ;					
						}
						timeOut=0;	
						ble_package.Len=0;
					}
          else if(cmdN==DISCONNECTED_HINT)//断开连接
					{
							if(sys_param_un.device_param.BLE_Connect==BLE_CONNECT_TRX)
							{								
								sys_param_un.device_param.BLE_Connect=BLE_ON ;		
							}								
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);	
							#endif
							BLE_transmit(AT_MODE_ENABLE,TYPE_AT_HANDLE,NULL);	
							while(MCU_at_cmd_analysis(AT_MODE_ENABLE)!=AT_ONLY_ACK)
							{		
								vTaskDelay(5);  //1ms
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);			
								#endif	
								timeOut++;								
							}	
							timeOut=0;									
							BLE_state =	BLE_SCAN_STATUS;								
							BLE_transmit(AT_SCAN,TYPE_AT_HANDLE,NULL);
							cmdN=AT_NULL;		
					}										
						else //if(cmdN==CONNECTED_SUCCESS)
					{
						timeOut++;
            if(timeOut>2500)//2.5s  .			
						{
							if(sys_param_un.device_param.BLE_Connect==BLE_CONNECT_TRX) 
							{
								sys_param_un.device_param.BLE_Connect=BLE_ON;
							}
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);	
							#endif
							BLE_transmit(AT_MODE_ENABLE,TYPE_AT_HANDLE,NULL);	
							while(MCU_at_cmd_analysis(AT_MODE_ENABLE)!=AT_ONLY_ACK)
							{		
								vTaskDelay(5);  //1ms
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);			
								#endif	
								timeOut+=5;
								if(timeOut>10000) break;								
							}	
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);	
							#endif
							BLE_transmit(AT_DISCONNECT,TYPE_AT_HANDLE,NULL);	
							while(MCU_at_cmd_analysis(AT_DISCONNECT)!=AT_ONLY_ACK)
							{		
								vTaskDelay(5);  //1ms
								#ifdef WDT_ENABLE
								xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);			
								#endif	
								timeOut+=5; 
								if(timeOut>10000) break;
							}	
							if(timeOut>3500)
							{		
								ble_package.Len=0;									
								BLE_state =	BLE_MASTER_CONFIG;
							}
							else
							{
								BLE_state =	BLE_SCAN_STATUS;
								BLE_transmit(AT_SCAN,TYPE_AT_HANDLE,NULL);								
							}
							timeOut=0;		
							cmdN=AT_NULL;			
						}							
						else						
						{					
							if(sys_param_un.device_param.BLE_Connect==BLE_ON)
							{
								sys_param_un.device_param.BLE_Connect=BLE_CONNECT_TRX;			
							}							
							if(heatTime>xTaskGetTickCount())
							{
								BLE_app_data_tx_buf[0]=0x96;
								BLE_app_data_tx_buf[1]=(uint8_t)sys_param_un.device_param.ref_tine;									
								BLE_app_data_tx_buf[2]=GC_depth_vlaue(0, 1);//read	;
								if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
								{
									BLE_app_data_tx_buf[3]=(uint8_t)abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum]/10);//spd
								}
								else
								{
									BLE_app_data_tx_buf[3]=(uint8_t)abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum]/10);//spd
								}								
								if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
								{
									BLE_app_data_tx_buf[4]=(uint8_t)(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]);
								}
								else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
								{
									BLE_app_data_tx_buf[4]=(uint8_t)(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
								}
								else
								{
									BLE_app_data_tx_buf[4]=(uint8_t)(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
								}
//								BLE_app_data_tx_buf[5]=ble_device_info_list[0].mac[13];
//								BLE_app_data_tx_buf[6]=ble_device_info_list[0].mac[15];
//								BLE_app_data_tx_buf[7]=ble_device_info_list[0].mac[16];
								BLE_transmit(UNVARNISHED_BLE_DATA_REV,BLE_APPLICATION_DATA_LENGTH,BLE_app_data_tx_buf);
								heatTime=xTaskGetTickCount();
						}
							else 
							{
							if(xTaskGetTickCount()>heatTime+250)//250ms发送数据
							{								
								BLE_app_data_tx_buf[0]=0x96;
								BLE_app_data_tx_buf[1]=(uint8_t)sys_param_un.device_param.ref_tine;									
								BLE_app_data_tx_buf[2]=GC_depth_vlaue(0, 1);//read	;
								if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
								{
									BLE_app_data_tx_buf[3]=(uint8_t)abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].toggleSpeedNum]/10);//spd
								}
								else
								{
									BLE_app_data_tx_buf[3]=(uint8_t)abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].motorSpeedNum]/10);//spd
								}
								
								if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModePositionToggle)
								{
									BLE_app_data_tx_buf[4]=(uint8_t)(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].recTorqueThresholdNum]);
								}
								else if(motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].dir==EndoModeTorqueATC)
								{
									BLE_app_data_tx_buf[4]=(uint8_t)(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].atcTorqueThresholdNum]);
								}
								else
								{
									BLE_app_data_tx_buf[4]=(uint8_t)(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_p_num].torqueThresholdNum]);
								}
//								BLE_app_data_tx_buf[5]=ble_device_info_list[0].mac[13];
//								BLE_app_data_tx_buf[6]=ble_device_info_list[0].mac[15];
//								BLE_app_data_tx_buf[7]=ble_device_info_list[0].mac[16];
								BLE_transmit(UNVARNISHED_BLE_DATA_REV,BLE_APPLICATION_DATA_LENGTH,BLE_app_data_tx_buf);
								heatTime=xTaskGetTickCount();
							}
							}
						}	
					}
					vTaskDelay(1);  //10ms					
					break;		
			case BLE_POWER_OFF://	
					if(sys_param_un.device_param.BLE_Connect==BLE_OFF)
					{
						vTaskDelay(5);  //10ms
					}
					else //重新打开
					{
						ble_package.Len=0;	
						BLE_Reset();						
						#ifdef WDT_ENABLE
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
						vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);		
						#endif
						timeOut=0;
						while(MCU_at_cmd_analysis(AT_DEVICE_START_HINT)!=AT_DEVICE_START_HINT)
						{
							vTaskDelay(5);  //1ms
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
							#endif	
							timeOut+=5;
							if(timeOut>3000)
							{
								timeOut=0;
								ble_package.Len=0;
								break;
//							BLE_Reset();					
							}							
						}				
						#ifdef WDT_ENABLE
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);
						vTaskDelay(BLE_COMMUNICATION_GAP_TIME_MS);  //1ms	
						xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);	
						#endif
						BLE_transmit(AT_MODE_ENABLE,TYPE_AT_HANDLE,NULL);	
						while(MCU_at_cmd_analysis(AT_MODE_ENABLE)!=AT_ONLY_ACK)
						{		
							vTaskDelay(5);  //1ms
							#ifdef WDT_ENABLE
							xEventGroupSetBits(WDTEventGroup, BLE_TASK_EVENT_BIT);			
							#endif								
						}					
						timeOut=0;												
						BLE_state =	BLE_SCAN_STATUS;//进入扫描状态									
						BLE_transmit(AT_SCAN,TYPE_AT_HANDLE,NULL);
						cmdN=AT_NULL;		
					}
					break;		
				default:
					vTaskDelay(3);  //10ms	
					break;
		}
	}
}
#endif
/*==================================GC task===================================
			* @brief:  GC task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vApexGC_Task( void * pvParameters )
{ 	
	uint32_t countTimeMs=0;
	unsigned  int timeOut;
	unsigned  char i,sendBuff;	
	#ifdef  APEX_FUNCTION_EBABLE
	static uint8_t apexGC_Flag=1;	
	static unsigned short int GC_Frequency=8000;
	unsigned  int GC_8k_AdcValue=0;
	unsigned  int GC_400_AdcValue=1;
	unsigned  short int  rec_gc_rate;  //当前rate
	static 	int  GC_depth=30;//初始值
	gc_in_or_exit(APEX_GC_IN);
	#endif		
	for(;;)
	{		
		countTimeMs++;	
		#ifdef DEBUG_RTT
		if(countTimeMs>200)//1s
		{
			countTimeMs=0;			
			SEGGER_RTT_printf(0, "gc8k= %d gc400=%d GC_depth=%d\r\n",GC_8k_AdcValue, GC_400_AdcValue,GC_depth);
		}			
		#endif
		#ifdef  APEX_FUNCTION_EBABLE		
		if(xSemaphoreTake(xSemaphoreApexAutoCalibration,0))
		{	
			apexGC_Flag=2;
		}		
		if(apexGC_Flag==1)//开机
		{
			gc_in_or_exit(APEX_GC_IN);
			GC_Frequency=8000;
			apex_frequency_set(GC_Frequency);
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			vTaskDelay(4);
			for(i=0;i<8;i++)//i<8;i++)
			{
				vTaskDelay(MIN_GC_PWM_VALID_TIME);
				restart_adc_sampling();
				timeOut=0;						
				while(fresh_adc_value()==0)	//wait collect finished
				{
					timeOut++;
					if(timeOut%48000==0)  taskYIELD();//exit task
					if(timeOut>144000) break;//3ms	
				}
				fresh_adc_value();//get latest value				
				if(GC_Frequency==8000)
				{
					GC_8k_AdcValue=get_gc_8k_value();	
					apex_ValueBuff[i] =	GC_8k_AdcValue;			
					GC_Frequency=400;
					apex_frequency_set(GC_Frequency);	
				}
				else // if(GC_Frequency==400)
				{					
					GC_400_AdcValue   = get_gc_400_value();								
					apex_ValueBuff[i] =	GC_400_AdcValue;							
					GC_Frequency=8000;
					apex_frequency_set(GC_Frequency);
				}				
				#ifdef WDT_ENABLE
				xEventGroupSetBits(WDTEventGroup,APEX_TASK_EVENT_BIT);
				#endif				
			}
			GC_8k_AdcValue=0;
			GC_400_AdcValue=0;			
			for(i=0;i<4;i++)	
			{
				GC_8k_AdcValue+=apex_ValueBuff[2*i]	;
				GC_400_AdcValue+=apex_ValueBuff[2*i+1]	;
			}		
			GC_400_AdcValue = GC_400_AdcValue>>2;
			GC_8k_AdcValue = (GC_8k_AdcValue>>2);	
     		if(GC_400_AdcValue==0)	 GC_400_AdcValue=1;		
			rec_gc_rate=(GC_8k_AdcValue*1000/GC_400_AdcValue);
			if((rec_gc_rate+5)<sys_param_un.device_param.gc_ref_rate||rec_gc_rate>5+sys_param_un.device_param.gc_ref_rate)  sys_param_un.device_param.gc_ref_rate=rec_gc_rate;//更新参考值	
			else 
			{				
				if(sys_param_un.device_param.empty_rate<sys_param_un.device_param.gc_ref_rate+180)
				{				
					sys_param_un.device_param.empty_rate=1000;//1000;
					//sys_param_un.device_param.gc_ref_rate=480;//680;			
				}
			}
			gc_list_init((int)sys_param_un.device_param.empty_rate,(int)(sys_param_un.device_param.gc_ref_rate-10));	//半格		
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			restart_adc_sampling();
			gc_in_or_exit(APEX_GC_EXIT);	
			#ifdef DEBUG_RTT
			SEGGER_RTT_printf(0, "ref-rate%d\r\n", sys_param_un.device_param.gc_ref_rate);	
			#endif
			apexGC_Flag=0;
		}
		else if(apexGC_Flag==2)//空载校准
		{
			gc_in_or_exit(APEX_GC_EXIT);
			vTaskDelay(3);	
			GC_Frequency=8000;
			apex_frequency_set(GC_Frequency);
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			vTaskDelay(4);
			for(i=0;i<8;i++)//i<8;i++)
			{
				vTaskDelay(MIN_GC_PWM_VALID_TIME);
				restart_adc_sampling();
				timeOut=0;						
				while(fresh_adc_value()==0)	//wait collect finished
				{
					timeOut++;
					if(timeOut%48000==0)  taskYIELD();//exit task
					if(timeOut>144000) break;//3ms	
				}
				fresh_adc_value();//get latest value				
				if(GC_Frequency==8000)
				{
					GC_8k_AdcValue=get_gc_8k_value();	
					apex_ValueBuff[i]		=	GC_8k_AdcValue;			
					GC_Frequency=400;
					apex_frequency_set(GC_Frequency);	
				}
				else if(GC_Frequency==400)
				{
					GC_400_AdcValue=get_gc_400_value();								
					apex_ValueBuff[i]		=	GC_400_AdcValue;							
					GC_Frequency=8000;
					apex_frequency_set(GC_Frequency);
				}				
				#ifdef WDT_ENABLE
				xEventGroupSetBits(WDTEventGroup,APEX_TASK_EVENT_BIT);
				#endif				
			}
			GC_8k_AdcValue=0;
			GC_400_AdcValue=0;			
			for(i=0;i<4;i++)	
			{
				GC_8k_AdcValue+=apex_ValueBuff[2*i]	;
				GC_400_AdcValue+=apex_ValueBuff[2*i+1]	;
			}		
			GC_400_AdcValue=	GC_400_AdcValue>>2;
			GC_8k_AdcValue=(GC_8k_AdcValue>>2);	
      		if(GC_400_AdcValue==0)	 GC_400_AdcValue=1;	
			rec_gc_rate=(GC_8k_AdcValue*1000/GC_400_AdcValue);			
		  	if(sys_param_un.device_param.apex_tine_400Value+5<GC_400_AdcValue||sys_param_un.device_param.apex_tine_400Value>5+GC_400_AdcValue)
			{				
				sys_param_un.device_param.apex_tine_400Value = GC_400_AdcValue;
				sys_param_un.device_param.empty_rate=rec_gc_rate;//保存参考值
				if(sys_param_un.device_param.empty_rate<sys_param_un.device_param.gc_ref_rate+150) sys_param_un.device_param.empty_rate=1000 ;//1000;//使用默认值
				gc_list_init((int)sys_param_un.device_param.empty_rate,(int)(sys_param_un.device_param.gc_ref_rate-10));
			}				
		//too low ,error value 不更新
			GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart	
			restart_adc_sampling();	
			#ifdef DEBUG_RTT
			SEGGER_RTT_printf(0, "400Va=%d\r\n", sys_param_un.device_param.apex_tine_400Value);	
			#endif
			apexGC_Flag=0;
		}
		else 
		{	
       if(get_gc_state() == RESET)
			{
				if(sys_param_un.device_param.apexFunctionLoad==0)
				{//beep	            						
					sendBuff=BUZZER_MODE_BEEP;
					xQueueSend(xQueueBeepMode, &sendBuff, 0);	
					GC_Frequency=8000;	
					apex_frequency_set(GC_Frequency);
					GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart
					restart_adc_sampling();
				}
				sys_param_un.device_param.apexFunctionLoad=1;	
				//*****************sampling manage****************************
				if(GC_PWM_SwitchTimeManage(xTaskGetTickCount(),MIN_GC_PWM_VALID_TIME))			
				{							
					timeOut=0;				
					while(fresh_adc_value()==0)	//wait collect finished
					{
						timeOut++;
						if(timeOut%48000)  taskYIELD();//exit task
						if(timeOut>144000) break;//3ms	
					}						
					if(GC_Frequency==8000)
					{
						GC_8k_AdcValue=get_gc_8k_value();						
						GC_Frequency=400;
						apex_frequency_set(GC_Frequency);	
					}		
					else //if(GC_Frequency==400)
					{
						GC_400_AdcValue=get_gc_400_value();					
						GC_Frequency=8000;
						apex_frequency_set(GC_Frequency);	
						//handle
						if(GC_400_AdcValue==0) GC_400_AdcValue=1;// 6k阻抗 GC_400_AdcValue=1100;
						GC_depth=apex_depth_calculate(GC_8k_AdcValue,GC_400_AdcValue);	
//						OLED_ShowNum(100,32,GC_8k_AdcValue,4,16,1);	//test
//						OLED_ShowNum(100,48,GC_depth,4,16,1);							
					}										
					GC_PWM_SwitchTimeManage(xTaskGetTickCount(),0);//restart									
				}
				else
				{										
//				fresh_adc_value();	
					timeOut=0;						
					while(fresh_adc_value()==0)	//wait collect finished
					{
						timeOut++;
						if(timeOut%48000)  taskYIELD();//exit task
						if(timeOut>144000) 
						{
							timeOut=0;
							break;//3ms	
						}
					}				
				}					 
			}	
			else
			{
				if(GC_Frequency!=0)
				{
					GC_Frequency=0;
					apex_frequency_set(0);//关闭
				}	
				if(sys_param_un.device_param.apexFunctionLoad==1)
				{//beep						
					sendBuff=BUZZER_MODE_BEEP;
					xQueueSend(xQueueBeepMode, &sendBuff, 0);
				}					
				sys_param_un.device_param.apexFunctionLoad=0;					
				GC_depth=30;
				timeOut=0;						
				while(fresh_adc_value()==0)	//wait collect finished
				{
					timeOut++;
					if(timeOut%48000)  taskYIELD();//exit task
					if(timeOut>144000) 
					{
						timeOut=0;
						break;//3ms	
					}
				}				
			}
        //send ramp
		}	    		
		#else
		//usart   only motor
		fresh_adc_value();
		//AppGetUsartData(&usartCmd);
		if(usartCmd>=USART_RX_CMD)
		{	
			if(sys_param_un.device_param.apexFunctionLoad==0)
			{//beep						
				sendBuff=BUZZER_MODE_BEEP;
				xQueueSend(xQueueBeepMode, &sendBuff, 0);
			}
			sys_param_un.device_param.apexFunctionLoad=1;
		}	
		else 
		{
			if(sys_param_un.device_param.apexFunctionLoad==1)
				{//beep						
					sendBuff=BUZZER_MODE_BEEP;
					xQueueSend(xQueueBeepMode, &sendBuff, 0);
				}
				sys_param_un.device_param.apexFunctionLoad=0;
		}
		if(countTimeMs>15)
		{
			countTimeMs=0;
			usartSendBuff[0]=0xa2;
			usartSendBuff[1]=0x92;
			usartSendBuff[2]=((abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].motorSpeedNum]*0.1)>>8)&0xFF)+5;
			usartSendBuff[3]=(abs(speed_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].motorSpeedNum]*0.1)&0xFF)+5; 
			usartSendBuff[4]=0x93;
			usartSendBuff[5]=(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].torqueThresholdNum]>>8) +5;
			usartSendBuff[6]=(torque_list[motor_param_un.system_motor_pattern[sys_param_un.device_param.use_num].torqueThresholdNum]&0xFF) +5;
			AppUsartTransmit(usartSendBuff,7);	
		}	
		GC_depth=30;
		#endif
		GC_depth_vlaue(GC_depth, 0);//wrtite
		
		#ifdef WDT_ENABLE
		xEventGroupSetBits(WDTEventGroup,APEX_TASK_EVENT_BIT);
		#endif
		if(countTimeMs>5000) countTimeMs=1;
		vTaskDelay(5);  //10ms
	}
}

/*==================================key task===================================
			* @brief:  key task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppKeyTask( void * pvParameters )
{ 
	static uint8_t sendKeyMessage=null_signal,buttonLock=1;	
	
	for(;;)
	{		
		sendKeyMessage= ButtonScan(10);//10ms,//get key value			
		if(buttonLock!=0)	//button lock	
		{	
			if(sendKeyMessage!=null_signal||get_insert_state()==SET||crm_flag_get(CRM_WDT_RESET_FLAG) != RESET)	
			{
				buttonLock=0;//unlock				
			}
			sendKeyMessage=null_signal;			
		}
		//handle  or send to key message queue		
		if(sendKeyMessage!=null_signal)
		{		
			xQueueSend(xQueueKeyMessage, &sendKeyMessage, 0);			
			if(sendKeyMessage!=power_off_signal)	
			{
				sendKeyMessage= BUZZER_MODE_BEEP;		
				xQueueSend(xQueueBeepMode, &sendKeyMessage, 0);
			}		
			sendKeyMessage=null_signal;
		}
		#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,KEY_TASK_EVENT_BIT);
		#endif
		//handle  or send to key message queue
		vTaskDelay(10);  //10ms
	}
}
/*==================================vAppBeepTemporaryTask  task===================================
			* @brief:  vAppBeepTemporaryTask task.
			* @param:  None
			* @retval: None
==============================================================================*/
void vAppBeepTemporaryTask( void * pvParameters )
{ 
	static unsigned char  buzzerOp=BUZZER_MODE_POWER_ON_HINT_VOL;//开机提示音
	unsigned short int freq=4000;//		
	for(;;)
	{		
		if(buzzerOp==BUZZER_MODE_POWER_ON_HINT_VOL)
		{
			buzzerOp=BUZZER_MODE_MUTE;	
		}
		else if(buzzerOp==BUZZER_MODE_MOTOR_REVERSE)
		{
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);                                              
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(50);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off							
			buzzerOp=BUZZER_MODE_MUTE;			
		}
		else if(buzzerOp==BUZZER_MODE_MOTOR_REVERSE_LONG)
		{
			if(	freq!=1800)
			{	
				freq=1800;
				buzz_frequency_set(freq,0);                                              
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(100);//	80ms							
			BuzzrSimpleSwitch(0);//off							
			buzzerOp=BUZZER_MODE_MUTE;			
		}
		else if(buzzerOp==BUZZER_MODE_GC_APEX)
		{
			if(	freq!=1800)
			{	
				freq=1800;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(150);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(50);		
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(100);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(100);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(100);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(100);
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(100);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(100);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			buzzerOp=BUZZER_MODE_MUTE;						
		}	
		else if(buzzerOp==BUZZER_MODE_GC_ZREO_APEX)
		{			
			if(	freq!=1800)
			{	
				freq=1800;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(75);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(25);		
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(50);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(50);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(50);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(50);
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);						
			vTaskDelay(50);					
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(50);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(200);			
			#endif	
			buzzerOp=BUZZER_MODE_MUTE;				
		}	
		else if(buzzerOp==BUZZER_MODE_GC_OVER)
		{			
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(80);//	80ms        				
			BuzzrSimpleSwitch(0);//off				
			#ifdef WDT_ENABLE		
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			vTaskDelay(40);				
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#else
			vTaskDelay(100);			
			#endif	
			buzzerOp=BUZZER_MODE_MUTE;				
		}	
		else if(buzzerOp==BUZZER_MODE_GC_CONNECT_TEST)
		{
			if(	freq!=4000)
			{	
				freq=4000;
				buzz_frequency_set(freq,0);
			}
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(50);//	80ms	
			#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
			#endif				
			BuzzrSimpleSwitch(0);//off
			vTaskDelay(50);
			buzzerOp=BUZZER_MODE_MUTE;				
		}
		buzzerOp=BUZZER_MODE_MUTE;	//清除旧值			
		xQueueReceive(xQueueBeepMode, &buzzerOp, 0);//获取新值				
		if(buzzerOp==BUZZER_MODE_BEEP)
		{
			if(freq!=4000)
			{
				freq=4000;				
				buzz_frequency_set(freq,0);
			}	
			BuzzrSimpleSwitch(1);//on
			vTaskDelay(60);//	80ms		
			BuzzrSimpleSwitch(0);//off	
			buzzerOp=BUZZER_MODE_MUTE;
		}		
		#ifdef WDT_ENABLE
			xEventGroupSetBits(WDTEventGroup,BEEP_TASK_EVENT_BIT);
		#endif		
		vTaskDelay(5);
	}	
}
/*==================================vSysWDT_ManageTask  ===================================
			* @brief:  vSysWDT_ManageTask.
			* @param:  None
			* @retval: None
==============================================================================*/
#ifdef WDT_ENABLE
void vSysWDT_ManageTask( void * pvParameters )
{ 	
	EventBits_t uxBits;
	BaseType_t retFlag;	
	for(;;)
	{
		vTaskDelay(MAX_WDT_FEED_TIME_MS);		
		uxBits =xEventGroupWaitBits(WDTEventGroup,EVENT_BITS_ALL,pdFALSE,pdTRUE,50);
		if((uxBits&EVENT_BITS_ALL)==EVENT_BITS_ALL)
		{			
			wdt_counter_reload();//feed dog
			xEventGroupClearBits(WDTEventGroup,EVENT_BITS_ALL);
			uxBits=0;
		}
		else
		{							
			if((uxBits&KEY_TASK_EVENT_BIT)!=KEY_TASK_EVENT_BIT)//key task err
			{
				vTaskDelete( keyScanTask_Handle);	
				retFlag=xTaskCreate( vAppKeyTask, "key scan", KEY_SCAN_STACK_DEPTH, NULL, KEY_SCAN_TASK_PRIORITY, &keyScanTask_Handle );	
				if(retFlag!=pdPASS)
				{				
					MenuDevicePowerOff(0);	
				}					
			}
			else if((uxBits&MENU_TASK_EVENT_BIT)!=MENU_TASK_EVENT_BIT)
			{	
				vTaskDelete( menuManageTask_Handle);	
				retFlag=xTaskCreate( vAppMenuManageTask, "menu manage", MENU_MANAGE_STACK_DEPTH, NULL, MENU_MANAGE_TASK_PRIORITY, &menuManageTask_Handle );
				if(retFlag!=pdPASS)
				{
					App_MotorControl(MOTOR_MODE_STOP);	
				}	
			}
			else if((uxBits&MOTOR_CONTROL_TASK_EVENT_BIT)!=MOTOR_CONTROL_TASK_EVENT_BIT)
			{									
				vTaskDelete( motorControlTask_Handle);	
				retFlag=xTaskCreate( vAppMotorControlTask, "motor control", MOTOR_CONTROL_STACK_DEPTH, NULL, MOTOR_CONTROL_TASK_PRIORITY, &motorControlTask_Handle );
				if(retFlag!=pdPASS)
				{
					App_MotorControl(MOTOR_MODE_STOP);	
				}					
			}
			else if((uxBits&PERIODIC_TASK_EVENT_BIT)!=PERIODIC_TASK_EVENT_BIT)
			{								
				vTaskDelete( periodicTask_Handle);	
				retFlag=xTaskCreate( vAppPeriodicTask, "periodic", PERIODIC_STACK_DEPTH, NULL, PERIODIC_TASK_PRIORITY, &periodicTask_Handle );	
				if(retFlag!=pdPASS)
				{
					App_MotorControl(MOTOR_MODE_STOP);	
				}						
			}	
			else if((uxBits&BEEP_TASK_EVENT_BIT)!=BEEP_TASK_EVENT_BIT){					
				vTaskDelete( beepTemporaryTask_Handle);
				xTaskCreate( vAppBeepTemporaryTask, "beep", SECONDARY_BEEP_STACK_DEPTH, NULL, SECONDARY_BEEP_TASK_PRIORITY, &beepTemporaryTask_Handle );			
			}
			else if((uxBits&APEX_TASK_EVENT_BIT)!=APEX_TASK_EVENT_BIT){					
				vTaskDelete( ApexGC_Task_Handle);
				xTaskCreate( vApexGC_Task, "apex", APEX_GC_STACK_DEPTH, NULL, APEX_GC_TASK_PRIORITY, &ApexGC_Task_Handle );			
			}
			#ifdef BLE_FUN
			else if((uxBits&BLE_TASK_EVENT_BIT)!=BLE_TASK_EVENT_BIT){					
				vTaskDelete( AppBLETaskTask_Handle);
				xTaskCreate( vAppBLETask, "BLE", BLE_MANAGE_STACK_DEPTH, NULL, BLE_MANAGE_TASK_PRIORITY, &AppBLETaskTask_Handle );			
			}
			#endif							
		}	
	}		
}
#endif

/*==================================start task===================================
			* @brief:  start task.
			* @param:  None
			* @retval: None
====================================================================================*/
void vTaskStart( void * pvParameters )
{		
		taskENTER_CRITICAL();
		Resume_RTOS_stick();//start
	 //queue
		xQueueKeyMessage =xQueueCreate(3,sizeof(uint16_t));	
		xQueueBeepMode =	xQueueCreate(1,sizeof(uint8_t));
		xQueueBatValue =	xQueueCreate(1,sizeof(unsigned short int));	
		xQueueMenuValue = xQueueCreate(1,sizeof(uint8_t));
	//Semaphore		
		xSemaphorePowerOff=xSemaphoreCreateBinary() ; //power off		
		xSemaphoreDispRfresh=xSemaphoreCreateBinary();
	#ifdef  APEX_FUNCTION_EBABLE
		xSemaphoreApexAutoCalibration=xSemaphoreCreateBinary();
	#endif
	
	#ifdef WDT_ENABLE
		WDTEventGroup=xEventGroupCreate();
	#endif		
		
		xTaskCreate( vAppPeriodicTask, "periodic", PERIODIC_STACK_DEPTH, NULL, PERIODIC_TASK_PRIORITY, &periodicTask_Handle );						
		xTaskCreate( vAppMotorControlTask, "motor control", MOTOR_CONTROL_STACK_DEPTH, NULL, MOTOR_CONTROL_TASK_PRIORITY, &motorControlTask_Handle );
		xTaskCreate( vAppKeyTask, "key scan", KEY_SCAN_STACK_DEPTH, NULL, KEY_SCAN_TASK_PRIORITY, &keyScanTask_Handle );
		xTaskCreate( vAppBeepTemporaryTask, "beep", SECONDARY_BEEP_STACK_DEPTH, NULL, SECONDARY_BEEP_TASK_PRIORITY, &beepTemporaryTask_Handle );
		#if configUSE_TIMERS == 1//软件定时器
//	xTimer01=xTimerCreate("xTimer01",100,pdTRUE,&1,NULL);		
		#endif		
		xTaskCreate( vApexGC_Task, "APEX", APEX_GC_STACK_DEPTH, NULL, APEX_GC_TASK_PRIORITY, &ApexGC_Task_Handle );		
		xTaskCreate( vAppMenuManageTask, "menu manage", MENU_MANAGE_STACK_DEPTH, NULL, MENU_MANAGE_TASK_PRIORITY, &menuManageTask_Handle );	
	#ifdef WDT_ENABLE	
		if(WDTEventGroup!=NULL)
		{
			xTaskCreate( vSysWDT_ManageTask, "WDT", WDT_MANAGE_STACK_DEPTH, NULL, WDT_MANAGE_TASK_PRIORITY, &WDT_ManageTask_Handle );
		}
	#endif
	#ifdef BLE_FUN
		xTaskCreate( vAppBLETask, "BLE", BLE_MANAGE_STACK_DEPTH, NULL, BLE_MANAGE_TASK_PRIORITY, &AppBLETaskTask_Handle );	
	#endif
		vTaskDelete( NULL);
		taskEXIT_CRITICAL();  			
}
