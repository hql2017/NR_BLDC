
#include "stdint.h"
#include "app_user_storage.h"
#include "para_list.h"
#include "flash.h"
#include "at32f413_board.h"
#include "at32f413.h"

#define DIR_FW_AND_RV   0
#define DIR_FW   1
#define DIR_RV   2
#define DIR_ATC  3

union Param_Union  sys_param_un; 
union Motor_Para_Union  motor_param_un; 

union Param_Union default_sys_param_un; 
union Motor_Para_Union default_mt_un; 

unsigned char const DataStr[]=__DATE__;
unsigned char const TimeStr[]=__TIME__;


extern void flash_read(unsigned  int read_addr, unsigned short int *p_buffer, unsigned short int num_read);
extern error_status flash_write(unsigned  int write_addr, unsigned short int *p_buffer, unsigned short int num_write);
error_status buffer_compare(uint16_t* p_buffer1, uint16_t* p_buffer2, uint16_t buffer_length);	

void default_para_write_buff(void)
{	
	sys_param_un.device_param.W_flag=40722;//DataStr[4]*256+DataStr[5]; 
	sys_param_un.device_param.use_hand=right_hand;  
	sys_param_un.device_param.use_p_num=0;   
	sys_param_un.device_param.apical_action_flag=1;//0单马达,1根尖自动反转，2，根尖停 
	sys_param_un.device_param.auto_start_flag=1; 
	sys_param_un.device_param.auto_stop_flag=1; 
	sys_param_un.device_param.motor_max_current=2800;
	 sys_param_un.device_param.BLE_Connect=BLE_OFF;
	sys_param_un.device_param.ref_tine=0;//3;	
	sys_param_un.device_param.apexFunctionLoad=0;
	if(sys_param_un.device_param.apex_tine_400Value==0||sys_param_un.device_param.apex_tine_400Value==0xFFFF)//
	{
	sys_param_un.device_param.gc_ref_rate=690;//480;//670;//
	sys_param_un.device_param.empty_rate=1000;//720;//1000;//no care
	sys_param_un.device_param.apex_tine_400Value=2400;
	}  
	motor_param_un.system_motor_pattern[0].pNum=0;	
	motor_param_un.system_motor_pattern[0].dir=DIR_FW;
	motor_param_un.system_motor_pattern[0].motorSpeedNum=spd300_Rpm_num;
	motor_param_un.system_motor_pattern[0].forwardPosition=30;
	motor_param_un.system_motor_pattern[0].reversePosition=-150;
	motor_param_un.system_motor_pattern[0].torqueThresholdNum=torque20_Ncm;//	
	motor_param_un.system_motor_pattern[0].toggleSpeedNum=spd300_Rpm_num;	
	motor_param_un.system_motor_pattern[0].atcTorqueThresholdNum=torque12_Ncm;//low = up *0.6
	motor_param_un.system_motor_pattern[0].recTorqueThresholdNum=torque40_Ncm;//	

   motor_param_un.system_motor_pattern[1].pNum=1;
	motor_param_un.system_motor_pattern[1].dir=DIR_FW;
	motor_param_un.system_motor_pattern[1].motorSpeedNum=spd350_Rpm_num;
	motor_param_un.system_motor_pattern[1].forwardPosition=30;
	motor_param_un.system_motor_pattern[1].reversePosition=-150;
	motor_param_un.system_motor_pattern[1].torqueThresholdNum=torque26_Ncm;
	motor_param_un.system_motor_pattern[1].toggleSpeedNum=spd350_Rpm_num;
	motor_param_un.system_motor_pattern[1].atcTorqueThresholdNum=torque16_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[1].recTorqueThresholdNum=torque40_Ncm;//
	
	motor_param_un.system_motor_pattern[2].pNum=2;
	motor_param_un.system_motor_pattern[2].dir=DIR_FW_AND_RV;//往复
	motor_param_un.system_motor_pattern[2].motorSpeedNum=spd350_Rpm_num;
	motor_param_un.system_motor_pattern[2].forwardPosition=30;
	motor_param_un.system_motor_pattern[2].reversePosition=-150;
	motor_param_un.system_motor_pattern[2].torqueThresholdNum=torque40_Ncm;//
	motor_param_un.system_motor_pattern[2].toggleSpeedNum=spd350_Rpm_num;
	motor_param_un.system_motor_pattern[2].atcTorqueThresholdNum=torque20_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[2].recTorqueThresholdNum=torque40_Ncm;//

	motor_param_un.system_motor_pattern[3].pNum=3;
	motor_param_un.system_motor_pattern[3].dir=DIR_FW;
	motor_param_un.system_motor_pattern[3].motorSpeedNum=spd500_Rpm_num;
	motor_param_un.system_motor_pattern[3].forwardPosition=30;
	motor_param_un.system_motor_pattern[3].reversePosition=-150;
	motor_param_un.system_motor_pattern[3].torqueThresholdNum=torque26_Ncm;
	motor_param_un.system_motor_pattern[3].toggleSpeedNum=spd500_Rpm_num;
	motor_param_un.system_motor_pattern[3].atcTorqueThresholdNum=torque16_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[3].recTorqueThresholdNum=torque40_Ncm;//
	
	motor_param_un.system_motor_pattern[4].pNum=4;
	motor_param_un.system_motor_pattern[4].dir=DIR_FW_AND_RV;//往复
	motor_param_un.system_motor_pattern[4].motorSpeedNum=spd400_Rpm_num;
	motor_param_un.system_motor_pattern[4].forwardPosition=110;
	motor_param_un.system_motor_pattern[4].reversePosition=-30;
	motor_param_un.system_motor_pattern[4].torqueThresholdNum=torque40_Ncm;//MAX_torque_42_Ncm;//MAX
	motor_param_un.system_motor_pattern[4].toggleSpeedNum=spd400_Rpm_num;
	motor_param_un.system_motor_pattern[4].atcTorqueThresholdNum=torque20_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[4].recTorqueThresholdNum=torque40_Ncm;//
	
	motor_param_un.system_motor_pattern[5].pNum=5;
	motor_param_un.system_motor_pattern[5].dir=DIR_FW;//顺时针
	motor_param_un.system_motor_pattern[5].motorSpeedNum=spd400_Rpm_num;
	motor_param_un.system_motor_pattern[5].forwardPosition=30;
	motor_param_un.system_motor_pattern[5].reversePosition=-150;
	motor_param_un.system_motor_pattern[5].torqueThresholdNum=torque35_Ncm;
	motor_param_un.system_motor_pattern[5].toggleSpeedNum=spd400_Rpm_num;
	motor_param_un.system_motor_pattern[5].atcTorqueThresholdNum=torque20_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[5].recTorqueThresholdNum=torque40_Ncm;//
	
	motor_param_un.system_motor_pattern[6].pNum=6;
	motor_param_un.system_motor_pattern[6].dir=DIR_FW;//顺时针
	motor_param_un.system_motor_pattern[6].motorSpeedNum=spd550_Rpm_num;
	motor_param_un.system_motor_pattern[6].forwardPosition=90;
	motor_param_un.system_motor_pattern[6].reversePosition=-30;
	motor_param_un.system_motor_pattern[6].torqueThresholdNum=torque35_Ncm;
	motor_param_un.system_motor_pattern[6].toggleSpeedNum=spd550_Rpm_num;
	motor_param_un.system_motor_pattern[6].atcTorqueThresholdNum=torque20_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[6].recTorqueThresholdNum=torque40_Ncm;//
	
	motor_param_un.system_motor_pattern[7].pNum=7;
	motor_param_un.system_motor_pattern[7].dir=DIR_FW;//顺时针
	motor_param_un.system_motor_pattern[7].motorSpeedNum=spd800_Rpm_num;
	motor_param_un.system_motor_pattern[7].forwardPosition=30;
	motor_param_un.system_motor_pattern[7].reversePosition=-150;
	motor_param_un.system_motor_pattern[7].torqueThresholdNum=torque10_Ncm;
	motor_param_un.system_motor_pattern[7].toggleSpeedNum=spd600_Rpm_num;
	motor_param_un.system_motor_pattern[7].atcTorqueThresholdNum=torque06_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[7].recTorqueThresholdNum=torque40_Ncm;//
	
	motor_param_un.system_motor_pattern[8].pNum=8;	
	motor_param_un.system_motor_pattern[8].dir=DIR_FW;//顺时针
	motor_param_un.system_motor_pattern[8].motorSpeedNum=spd250_Rpm_num;
	motor_param_un.system_motor_pattern[8].forwardPosition=270;
	motor_param_un.system_motor_pattern[8].reversePosition=-60;
	motor_param_un.system_motor_pattern[8].torqueThresholdNum=torque14_Ncm;
	motor_param_un.system_motor_pattern[8].toggleSpeedNum=spd250_Rpm_num;	
	motor_param_un.system_motor_pattern[8].atcTorqueThresholdNum=torque08_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[8].recTorqueThresholdNum=torque40_Ncm;//
	
 	motor_param_un.system_motor_pattern[9].pNum=9;	
	motor_param_un.system_motor_pattern[9].dir=DIR_FW;//顺时针
	motor_param_un.system_motor_pattern[9].motorSpeedNum=spd500_Rpm_num;
	motor_param_un.system_motor_pattern[9].forwardPosition=180;
	motor_param_un.system_motor_pattern[9].reversePosition=-70;
	motor_param_un.system_motor_pattern[9].torqueThresholdNum=torque30_Ncm;
	motor_param_un.system_motor_pattern[9].toggleSpeedNum=spd500_Rpm_num;
	motor_param_un.system_motor_pattern[9].atcTorqueThresholdNum=torque18_Ncm;//low = up *0.6	
	motor_param_un.system_motor_pattern[9].recTorqueThresholdNum=torque40_Ncm;//
	
   motor_param_un.system_motor_pattern[10].pNum=10;
	motor_param_un.system_motor_pattern[10].dir=DIR_FW;//顺时针
	motor_param_un.system_motor_pattern[10].motorSpeedNum=spd100_Rpm_num;
	motor_param_un.system_motor_pattern[10].forwardPosition=150;
	motor_param_un.system_motor_pattern[10].reversePosition=-30;
	motor_param_un.system_motor_pattern[10].torqueThresholdNum=torque40_Ncm;
	motor_param_un.system_motor_pattern[10].toggleSpeedNum=spd100_Rpm_num;
	motor_param_un.system_motor_pattern[10].atcTorqueThresholdNum=torque20_Ncm;//low = up *0.6
	motor_param_un.system_motor_pattern[10].recTorqueThresholdNum=torque40_Ncm;//	
//100,150,200,250,300,350,400,450,500,550,600,800,1000,1200,1500,1800,2000,2200,2500
}

void para_print(void)
{

	#ifdef DEBUG_RTT	
	SEGGER_RTT_printf(0,"*******sys_para******\r\n",0);
	SEGGER_RTT_printf(0,"W_flag %d , err_num %d , sys_num %d ,\r\n empty_rate %d , use_hand %d , use_num %d ,\r\n vol %d , apical_action_flag %d , auto_start_flag %d ,\r\n mt_i %d , auto_stop_flag %d , ref_rate %d , ref_tine %d\r\n", 

	sys_param_un.device_param.W_flag,
	sys_param_un.device_param.empty_rate,  
	sys_param_un.device_param.use_hand,  
	sys_param_un.device_param.use_p_num,   
	sys_param_un.device_param.apical_action_flag,
	sys_param_un.device_param.auto_start_flag,
	sys_param_un.device_param.motor_max_current,
	sys_param_un.device_param.auto_stop_flag,
	sys_param_un.device_param.ref_tine);
	#endif 	
	#ifdef DEBUG_RTT
//	SEGGER_RTT_printf(0,"************************\r\n",0);
//	SEGGER_RTT_printf(0,"*******patttern_para******\r\n",0);	
//	for(i=0;i<10;i++)
//	{		
//		SEGGER_RTT_printf(0,"PAT%d apex %d dir %d speed %d fv %d rv %d torque %d toogle spd %d\r\n ",	
//		motor_param_un.system_motor_pattern[i].pNum,
//		motor_param_un.system_motor_pattern[i].apexEnable,
//		motor_param_un.system_motor_pattern[i].dir,
//		motor_param_un.system_motor_pattern[i].motorSpeedNum,
//		motor_param_un.system_motor_pattern[i].forwardPosition,
//		motor_param_un.system_motor_pattern[i].reversePosition,	
//		motor_param_un.system_motor_pattern[i].torqueThresholdNum,
//		motor_param_un.system_motor_pattern[i].toggle_mode_speed);		
//	}
	#endif 
   
}
void start_para_write_read(void)
{
  flash_read(FLASH_SYSTEM_PARAM_ADDR, sys_param_un.para_buff, sizeof(Device_Param_Def));
	#ifdef DEBUG_RTT
	SEGGER_RTT_printf(0,"first read w_flag is %d \r\n",sys_param_un.device_param.W_flag);
	#endif
  if(sys_param_un.device_param.W_flag==40530)//(DataStr[4]*256+DataStr[5]))
  {
	#ifdef DEBUG_RTT
		SEGGER_RTT_printf(0,"NO DO WRITE\r\n",0);	
	#endif   
    flash_read(FLASH_MOTOR_PARAM_ADDR,(uint16_t *)&motor_param_un.system_motor_pattern,11*sizeof(SYSTEM_MOTOR_PARAM));     
  }
  else
  {   
   default_para_write_buff();
   flash_write(FLASH_SYSTEM_PARAM_ADDR, sys_param_un.para_buff, sizeof(Device_Param_Def)); 
   flash_write(FLASH_MOTOR_PARAM_ADDR,(uint16_t *)&motor_param_un.system_motor_pattern,11*sizeof(SYSTEM_MOTOR_PARAM));   
	#ifdef DEBUG_RTT
		 SEGGER_RTT_printf(0,"DO ONE WRITE\r\n",0);
	#endif		
   flash_read(FLASH_SYSTEM_PARAM_ADDR, sys_param_un.para_buff, sizeof(Device_Param_Def));   
   flash_read(FLASH_MOTOR_PARAM_ADDR,(uint16_t *)&motor_param_un.system_motor_pattern,11*sizeof(SYSTEM_MOTOR_PARAM));
	#ifdef DEBUG_RTT
		 SEGGER_RTT_printf(0,"DO ONE READ\r\n",0);
	#endif       
  }
   para_copy_default();
   para_print();
}

void para_copy_default(void)//将开机后使用参数拷贝到默认参数为下次存储参数做比较
{
  uint16_t i=0;
  for(i=0;i<sizeof(Device_Param_Def);i++)
  {
    default_sys_param_un.para_buff[i]=sys_param_un.para_buff[i];
  }
  for(i=0;i<11*sizeof(SYSTEM_MOTOR_PARAM);i++)
  {
    default_mt_un.pattern_buff[i] = motor_param_un.pattern_buff[i];
  }  
}

void write_para_judge(void)
{
  error_status err;
  err=buffer_compare(default_sys_param_un.para_buff,sys_param_un.para_buff,sizeof(Device_Param_Def));
  if(ERROR==err) //参数已经被篡改
  {
     flash_write(FLASH_SYSTEM_PARAM_ADDR, sys_param_un.para_buff, sizeof(Device_Param_Def));
  }
  else
  {
	#ifdef DEBUG_RTT
		SEGGER_RTT_printf(0,"PARA NO CHANGE\r\n",0);
	#endif   
  } 
  err=buffer_compare(default_mt_un.pattern_buff,motor_param_un.pattern_buff,11*sizeof(SYSTEM_MOTOR_PARAM));
  if(ERROR==err) //参数已经被篡改
  {
   flash_write(FLASH_MOTOR_PARAM_ADDR,motor_param_un.pattern_buff,11*sizeof(SYSTEM_MOTOR_PARAM));
  }
  else
  {
	#ifdef DEBUG_RTT
		SEGGER_RTT_printf(0,"MOTOR PARA NO CHANGE\r\n",0);
	#endif      
  }
}
error_status buffer_compare(uint16_t* p_buffer1, uint16_t* p_buffer2, uint16_t buffer_length)
{
  while(buffer_length--)
  {
    if(*p_buffer1 != *p_buffer2)
    {
      return ERROR;
    }
    p_buffer1++;
    p_buffer2++;
  }
  return SUCCESS;
}

