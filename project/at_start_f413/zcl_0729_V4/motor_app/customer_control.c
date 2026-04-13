#include "customer_control.h"
#include "delay.h"
#include "usart_motor_bsp.h"

#include "gpio_port.h"
#include "math.h"


#ifdef DEBUG_RTT
#include "SEGGER_RTT.h"
#endif
//************AT32f413*****************
#ifdef  ZHX
#define NOT_DIRECTION_REVERSE//����ת��������		
#endif

#define CURRENT_REVERSE_PARAM     1.041//�������������forward=1;
extern uint8_t menu_motor_run_mode; //�������ģʽ,0������1����2����ת���Ƕȵ��ڣ�



//*****************************
unsigned short status_mp6570 = 0;
 short forward_speed; 
 short reverse_speed;
 short toggle_speed;
int	forward_position;
int reverse_position;
unsigned short upper_threshold;		//iq upper threshold
unsigned short lower_threshold;
unsigned short threshold_times = 3000;   //delay counter x * 100us
unsigned short reach_upper_times = 0;
unsigned short reach_lower_times = 0;
MotorStatus_TypeDef motor_status;
MotorSettings_TypeDef motor_settings;
//变量
static float iq;

#define m_gear_ratio	6		//gearbox ratio, set to 1 if no gearbox is used.
unsigned short update_command ;


unsigned char torque_reach(void);
unsigned char toggle_torque_reach(void);
void set_torque_limit(float upper_limit, float lower_limit); //set torque limit in speed mode 
void set_speed(int forward, int reverse);//set forward and backward speed
void set_position(int forward, int reverse);//set forward and backward position
void set_toggle_mode_speed(int speed);
void mode_select(enum EndoMode mode);//set working mode. 0:speed mode, 1: position back and forth mode;

/*****************************set working mode********************************************
  * @brief  Set working mode.
  * @param  mode: EndoModePositionToggle or EndoModeSpeedAutoReverse
  * @retval None
*****************************************************************************************/
void mode_select(enum EndoMode mode)
{
	if(mode==EndoModePositionToggle)
	{
		motor_status.mode = mode;						
	}
	else 
	{		
		motor_status.mode = mode;		
	}
}

/*****************************start the motor********************************************
  * @brief  start the motor.
  * @param  None
  * @retval None
*****************************************************************************************/
void start()
{
		motor_status.status = Status_START;
		/*		
		if(motor_status.mode==EndoModePositionToggle)
		{//position mode				
			app_u_motor_start(2,forward_speed,(upper_threshold)*10.0);
		}	
		else  if(motor_status.mode==EndoModeSpeedForward)
		{//torque mode
			app_u_motor_start(0, forward_speed,(upper_threshold)*10.0);
		}
		else  if(motor_status.mode==EndoModeSpeedReverse)
		{
			app_u_motor_start(0, reverse_speed,(upper_threshold)*10.0);
		}
		else  if(motor_status.mode==EndoModeTorqueATC)
		{
			app_u_motor_start(0, forward_speed,(upper_threshold)*10.0);
		}
		*/			
}
/*****************************stop the motor********************************************
  * @brief  stop the motor.
  * @param  None
  * @retval None
*****************************************************************************************/
void stop()
{	
	motor_status.status = Status_STOP;	
	app_u_motor_stop();		
}

/*****************************set torque limit in speed mode*****************************
  * @brief  set torque limit in speed mode.
  * @param  upper_limit: when the motor torque higher than this value, motor start to turn backward
	* @Param	lower_limit: when the motor torque lower than this value, motor start to turn forward
  * @retval None
*****************************************************************************************/
void set_torque_limit(float upper_limit, float lower_limit)
{
	if(lower_limit<0.4)		lower_limit = 0.4;	
	upper_threshold = upper_limit *90;
	lower_threshold = lower_limit *90;
}
void set_toggle_mode_speed(int speed)
{
	toggle_speed = (speed * m_gear_ratio);	
}
/*****************************set forward and backward speed******************************
  * @brief  set forward and backward speed
	* @param  forward: forward speed. 
	* @param  reverse: reverse speed. 
*****************************************************************************************/
void set_speed(int forward, int reverse)
{
	forward_speed = (forward );//* m_gear_ratio);
	reverse_speed = (reverse );//* m_gear_ratio);	
}

/*****************************set forward and backward speed******************************
  * @brief  set speed ramp slope.
	* @param  rpmPerMs: speed slope(unit rpm/ms), range 1-91rpm/ms. 
  * @retval None
*****************************************************************************************/
void set_speed_slope(int rpmPerMs)
{
	u_pos_set.p_set.freq=rpmPerMs;
}

/*****************************set forward and backward position******************************
  * @brief  set forward and backward position
	* @param  forward: forward position. 
	* @param  reverse: reverse position. 
  * @retval None
*****************************************************************************************/
void set_position(int forward, int reverse)
{	
	//forward_position = forward * 182 * m_gear_ratio;
	//reverse_position = reverse * 182 * m_gear_ratio;	
	u_pos_set.p_set.position_ref1=forward;	
	u_pos_set.p_set.position_ref2=reverse;	
	forward_position = forward ;
	reverse_position = reverse ;	
}
/**
  * @brief  Obtain the motor speed.
  * @param  None.
  * @retval the motor speed in rpm.
  */
int get_motor_speed(void)
{
  	int32_t speed_fbk;	
	speed_fbk=(int32_t)u_motor_sta_replay.sta.speed;	
	
	return speed_fbk;
}

/**
  * @brief  Obtain the motor round.
  * @param  None.
  * @retval the motor angle.
  */
int16_t get_postion_round(void)
{
	int16_t position;
	position=(int16_t)u_motor_sta_replay.sta.position;
	return position;
}

/**
  * @brief  Obtain the motor angle.
  * @param  None.
  * @retval the motor angle in degree.
  */
uint16_t get_position_angle(void)
{
	int16_t position_angle;
	position_angle=(int16_t)u_motor_sta_replay.sta.position;
	return position_angle;
}

/*****************************customer control logic******************************
  * @brief  customer control logic function
						Note: Don't add delay in this function and any function called by this function should not
						has delay.
	* @param  None. 
  * @retval None
*****************************************************************************************/

void customer_control()
{
	static int delay_cnt = 0;	//
	iq=u_motor_sta_replay.sta.current*1000;//mA
	if(motor_settings.mode==EndoModePositionToggle)
	{	
		motor_status.reach_torque = toggle_torque_reach();			
	}
	else motor_status.reach_torque = torque_reach();				//whether motor reach target torque	
	//往复模式	
	if(motor_status.mode==EndoModePositionToggle)
	{
		if(motor_status.status == Status_START)	{					
			if(forward_position+reverse_position<0){
				motor_status.status = Status_REVERSE;
				app_u_motor_start(2, -toggle_speed,upper_threshold);
			}
			else {
				motor_status.status = Status_FORWARD;
				app_u_motor_start(2, toggle_speed,upper_threshold);
			}
		}
		if(motor_status.status == Status_FORWARD)	{			
				if(motor_status.reach_torque == 2){					
					if(forward_position+reverse_position<0)
					{
						motor_status.status = Status_REVERSE;
						app_u_motor_start(0, -toggle_speed,upper_threshold);
					}					
				}
				else if(motor_status.reach_torque == 1){	
					if(forward_position+reverse_position>0){
						motor_status.status = Status_REVERSE;
						app_u_motor_start(0, -toggle_speed,upper_threshold);
					}					
				}			
		}
		else if(motor_status.status == Status_REVERSE)	{			
				if(motor_status.reach_torque == 2){					
					if(forward_position+reverse_position>0)
					{
						motor_status.status = Status_FORWARD;
						app_u_motor_start(0, toggle_speed,upper_threshold);
					}					
				}
				else if(motor_status.reach_torque == 1){	
					if(forward_position+reverse_position<0){
						motor_status.status = Status_FORWARD;
						app_u_motor_start(0, toggle_speed,upper_threshold);
					}					
				}			
		}
	}		
	else if(motor_status.mode == EndoModeSpeedReverse)
	{	//speed Reverse	
		if(motor_status.status == Status_START)	{	
			motor_status.status = Status_REVERSE;
			app_u_motor_start(0, reverse_speed,upper_threshold);	
		}
		else if(motor_status.status == Status_REVERSE)
		{			
			if(motor_status.reach_torque == 1)   {											
				motor_status.status = Status_FORWARD;	
				app_u_motor_start(0, -reverse_speed,upper_threshold);			
			}	
		}
		else if(motor_status.status == Status_FORWARD)
		{				
			if(motor_status.reach_torque == 2)   {									
				motor_status.status = Status_REVERSE;	
				app_u_motor_start(0, reverse_speed,upper_threshold);					
			}	
		}						
	}	
	else if(motor_status.mode==EndoModeSpeedForward)
	{	 //speed Forward
		if(motor_status.status == Status_START)	{	
			motor_status.status = Status_FORWARD;
			app_u_motor_start(0, forward_speed,upper_threshold);	
		}
		else if(motor_status.status == Status_FORWARD)
		{			
			if(motor_status.reach_torque == 1)   {											
					motor_status.status = Status_REVERSE;	
					app_u_motor_start(0, -forward_speed,upper_threshold);				
			}	
		}
		else if(motor_status.status == Status_REVERSE)
		{				
			if(motor_status.reach_torque == 2)   {									
					motor_status.status = Status_FORWARD;	
					app_u_motor_start(0, forward_speed,upper_threshold);					
			}	
		}							
	}
	else if(motor_status.mode == EndoModeKeepForward)
	{	//keep Forward
		if(motor_status.status == Status_START)	{	
			motor_status.status = Status_FORWARD;
			app_u_motor_start(0, forward_speed,upper_threshold);	
		}
		else if(motor_status.status == Status_FORWARD)
		{			
			if(motor_status.reach_torque == 1)   {
				stop();	
			}
		}		
	}
	else if(motor_status.mode == EndoModeKeepReverse)
	{	//keep Reverse
		if(motor_status.status == Status_START)	{	
			motor_status.status = Status_REVERSE;
			app_u_motor_start(0, reverse_speed,upper_threshold);	
		}
		else if(motor_status.status == Status_REVERSE)
		{			
			if(motor_status.reach_torque == 1)   {
				stop();	
			}
		}		
	}		
}		
void update_settings(MotorSettings_TypeDef *setting)
{
	mode_select(setting->mode);	
	set_speed(setting->forward_speed, setting->reverse_speed);
	set_position(setting->forward_position, setting->reverse_position);	
	set_torque_limit(setting->upper_threshold, setting->lower_threshold);
	set_toggle_mode_speed(setting->toggle_mode_speed);		
}

unsigned char torque_reach(void)
{	
	u8 status = 0; 							// 1- reach_upper  2- reach_lower  0: middle-state
	threshold_times = 3000;
	if ((iq > upper_threshold)||(iq < -(upper_threshold*CURRENT_REVERSE_PARAM)))
	{
		reach_upper_times ++;
		if(reach_upper_times >=threshold_times)
		{
			reach_upper_times = threshold_times;
			reach_lower_times = 0;
			status = 1; 
		}
	}
	if((iq < lower_threshold)&&(iq > -(lower_threshold*CURRENT_REVERSE_PARAM)))
	{
		reach_lower_times ++;
		if(reach_lower_times >= threshold_times)
		{
			reach_lower_times = threshold_times;
			reach_upper_times = 0;
			status = 2;
		}
	}
	return status;
}

unsigned char toggle_torque_reach(void)
{	
	u8 status = 0; 							// 1- reach_upper  2- reach_lower  0: middle-state
	u8 check_torque = 0;
	threshold_times = 350;
	if(((motor_settings.forward_position > - motor_settings.reverse_position)&&(motor_status.status == Status_FORWARD)) || ((motor_settings.forward_position < - motor_settings.reverse_position)&&(motor_status.status == Status_REVERSE)))
		check_torque = 1;
	else
		check_torque = 0;
	if(check_torque)
	{
		if ((iq > upper_threshold)||(iq < -(upper_threshold*CURRENT_REVERSE_PARAM)))
		{
			reach_upper_times ++;
			if(reach_upper_times >=threshold_times)
			{
				reach_upper_times = threshold_times;
				reach_lower_times = 0;
				status = 1; 
			}
		}
		if((iq < lower_threshold)&&(iq > -(lower_threshold*CURRENT_REVERSE_PARAM)))
		{
			reach_lower_times ++;
			if(reach_lower_times >= threshold_times)
			{
				reach_lower_times = threshold_times;
				reach_upper_times = 0;
				status = 2;
			}
		}
		
	}
	return status;
}
void init_registers()
{
	iq=u_motor_sta_replay.sta.current*1000;//mA
}

/**
  * @brief  MotorDeviceReset
  * @param  none
  * @retval none
  */
void MotorDeviceReset(void)
{
	delay_1ms(100);				//delay_1ms 100ms wait for sensor to be stable	
//LOOPTIMER_ENABLE; //enable loop timer
}
/**
  * @brief  GetRealTorque
  * @param  none
  * @retval none
  */
unsigned short int GetRealTorque(void)
{
	unsigned  int retValue;
  	unsigned int torqueValue;
	static 	unsigned short int torqueValueBuff[4]={0};
	static unsigned char num;
	num++;
	num%=4;
 	torqueValueBuff[num]=(unsigned short int)(fabsf(iq)); 		
	#ifdef ZHX
	torqueValue=(torqueValueBuff[0]+torqueValueBuff[1]+torqueValueBuff[2]+torqueValueBuff[3])/36;//ZHX;10/(93*4);
	#else
	torqueValue=(torqueValueBuff[0]+torqueValueBuff[1]+torqueValueBuff[2]+torqueValueBuff[3])/51;//10/(130*4);
	#endif
	retValue=torqueValue;	
	return retValue;
}
