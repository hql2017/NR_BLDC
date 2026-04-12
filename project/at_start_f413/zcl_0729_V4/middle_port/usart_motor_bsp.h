#ifndef __USART_MOTOR_BSP_
#define __USART_MOTOR_BSP_

#define U_MOTOR_CODE_ID_MASK         0xFE
#define U_MOTOR_ID_RESTORE           0x00
#define U_MOTOR_ID_GENE_WORK_ENABLE    0x02
#define U_MOTOR_ID_PREPARE_MODE      0x04 
#define U_MOTOR_ID_ENCODE_CALI       0x06
#define U_MOTOR_ID_SPEED_MODE   0x0A
#define U_MOTOR_ID_POSITION_MODE 0x0C
#define U_MOTOR_ID_STATUS_INFO_REQ   0x0E
#define U_MOTOR_ID_SOFT_RESET_REQ    0x10
#define U_MOTOR_ID_STANSBY_MODE      0x12 
#define U_MOTOR_ID_EMERGE_STOP       0x14
#define U_MOTOR_ID_TORQUE_MODE_SET       0x1C
extern void app_uart_motor_init(void);

typedef struct
{
	unsigned char head1;
	unsigned char head2;
	unsigned char len;
	unsigned char id;//4byte
	int speed_ref; // speed_ref: 4-byte�� ת��ָ� ��λ�� rpm, 6��1 ���ٺ���ٶ�
	float iq_ref; // iq_ref: 4-byte�� Ť�ص��� iq,������ iq ����
	unsigned char check_sum; // 1-byte

}__attribute__((packed)) set_speed_tq_limit_struct;
typedef union{
	set_speed_tq_limit_struct tq_set;
	unsigned char data[sizeof(set_speed_tq_limit_struct)];
}un_set_speed_tq_limit_struct;

typedef struct
{
	unsigned char head1;
	unsigned char head2;
	unsigned char len;
	unsigned char id;
	float position_ref1;
	float position_ref2;
	float freq;//0~10Hz
	unsigned char  mode;
	unsigned char check_sum;
}__attribute__((packed)) set_position_ref_request_struct;

typedef union{
	set_position_ref_request_struct p_set;
	unsigned char data[sizeof(set_position_ref_request_struct)];
}un_motor_positon_set;
extern un_motor_positon_set u_pos_set;

typedef struct
	{
		unsigned char id;//����id,ack,(bit0=1��ack����
		unsigned char idStatus;	//1�ȴ�ack,0����
	
}U_MOTOR_PACK_STATUS;
	
typedef struct
{
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
	
}__attribute__((packed)) get_status_reply_struct;

typedef union{
	get_status_reply_struct sta;
	unsigned char data[sizeof(get_status_reply_struct)];
}un_motor_status;

extern un_motor_status u_motor_sta_replay;
void app_u_motor_run_enable(void);
void app_u_motor_param_config_init(void );
void app_u_motor_reset(void );
void app_u_motor_angle_cali(void );
void app_u_motor_angle_cali_next(void );
void app_u_motor_get_sta_req(void );
void app_u_motor_position_set(un_motor_positon_set* p_set);
unsigned char  app_u_motor_rec_data(void );

void AppUsartMotorTransmit(unsigned char id ,unsigned char *buf,unsigned short int len);

#endif
