#ifndef __APP_MOTORCONTROLTASK
#define __APP_MOTORCONTROLTASK

typedef struct{
    unsigned char cmdCode;
    unsigned char cmdLen;
    float fData;
}__attribute__((packed))MOTOR_CTR_MESSAGE;
typedef union{
    MOTOR_CTR_MESSAGE msg;
    unsigned char mBuff[6];
}U_MOTOR_CTR_MESSAGE;

//#define MOTOR_MODE_CW            	0 //顺时针，达到力矩反转，
//#define MOTOR_MODE_CCW            1 //逆时针，达到力矩反转，
//#define MOTOR_MODE_REC          	2 //往复运动，可调节角度，达到力矩反转
//#define MOTOR_MODE_TORPUE_REV     3  //扭力自适应（ATC）可调
////#define MOTOR_MODE_ATC          4 //扭力自适应
#define MOTOR_MODE_STOP         	0
#define MOTOR_MODE_START        	1
#define MOTOR_MODE_SEARCH_ANGLE   5  //找初始角度
#define MOTOR_SETTING_UPDATE   6  //配置更新
#define MOTOR_MODE_RESTART        	7
#define MOTOR_USART_GC_CONTROL      8//单马达串口根测控制
#define MOTOR_SETTING_ERR   9  //err

#define m_run_stop         	0
#define m_run_genara       	1
#define m_run_cali_angle    2  //找初始角度
#define m_run_cali_current  4  //找空载电流

#define MOTOR_MINI_DELAY_TIME_MS  50

extern void MotorParamInit(void);

extern unsigned char App_MotorControl(unsigned char cmd);	

#endif
