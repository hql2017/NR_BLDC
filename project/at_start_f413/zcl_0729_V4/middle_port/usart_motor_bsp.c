
#include "stdio.h"
#include "string.h"
#include "at32f413_usart.h"
#include "at32f413_gpio.h"
#include "at32f413_board.h"

#include "usart_motor_bsp.h"

 

#define MAX_U_MOTOR_FIFO_LEN 64
uint8_t uart_motor_tx_buf[MAX_U_MOTOR_FIFO_LEN+1];
uint8_t uart_motor_rx_buf[MAX_U_MOTOR_FIFO_LEN+1];
static unsigned char receiveLen=0;
uint8_t TX_CheckSum(uint8_t *buf, uint8_t len) ;//bufΪ���飬lenΪ���鳤��


un_motor_status u_motor_sta_replay;
un_motor_positon_set u_pos_set;

typedef union{
	float f_value;
	unsigned char data[4];
}u_f_in;//������ת���ֽ�����

static u_f_in u_f_uni;


static void config_u_motor_uart3_dma(void)
{
  dma_init_type dma_init_struct;

  /* enable dma1 clock */
  crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
  /* dma1 channel2 for usart1 tx configuration */
  dma_reset(DMA2_CHANNEL3);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = MAX_U_MOTOR_FIFO_LEN+1;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)uart_motor_tx_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL3, &dma_init_struct);

  /* config flexible dma for usart3 tx */
	usart_dma_transmitter_enable(USART3,TRUE);//DMA��usart TX������DMA
  dma_flexible_config(DMA2, FLEX_CHANNEL3, DMA_FLEXIBLE_UART3_TX);
  dma_channel_enable(DMA2_CHANNEL3, TRUE); /* usart3 tx begin dma transmitting */	
	dma_interrupt_enable(DMA2_CHANNEL3,DMA_FDT_INT,TRUE);
	nvic_irq_enable(DMA2_Channel3_IRQn, 2, 0);


}

void app_uart_motor_init(void)
{
	/******************gpio**************************/
	gpio_init_type gpio_init_struct;

  #if defined (__GNUC__) && !defined (__clang__)
  setvbuf(stdout, NULL, _IONBF, 0);
  #endif

  /* enable the uart and gpio clock */
  crm_periph_clock_enable(USART3_CRM_CLK,TRUE);
  crm_periph_clock_enable(USART3TX_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  
  /* configure the usart tx pin  PB10*/
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = USART3TX_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(USART3TX_PORT, &gpio_init_struct);

  /* configure the usart rx pin pB11 */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = USART3RX_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(USART3RX_PORT, &gpio_init_struct);

  /* configure uart param */
  usart_init(USART3, 9600, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);	
	
	
	usart_receiver_enable(USART3, TRUE); 
	nvic_irq_enable(USART3_IRQn, 4, 0);
	usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
	usart_flag_clear (USART3, USART_IDLEF_FLAG);	
//	usart_interrupt_enable(USART3, USART_RDBF_INT|USART_IDLE_INT, TRUE);
//	usart_flag_clear (USART3, USART_IDLEF_FLAG);	
	usart_enable(USART3, TRUE);	 
	//DMA
	config_u_motor_uart3_dma();		
}

/**
  * @brief  usart3_send_data
  * @param  u8 *buf, u16 len
  * @retval none
  */
void u_motor_send_data(u8 *buf, u16 len)
{  	
//while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET){;} 
	
	while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET){;} //	
	memcpy(uart_motor_tx_buf, buf, len); 	
  dma_channel_enable(DMA2_CHANNEL3, FALSE); 	 	 
  dma_data_number_set(DMA2_CHANNEL3, len);  
  dma_channel_enable(DMA2_CHANNEL3, TRUE);	 
}

 
/**
  * @brief AppUsartMotorTransmit
  * @param   
  * @retval none
  */
 void AppUsartMotorTransmit(unsigned char id ,unsigned char *buf,unsigned short int len)
{	
	uart_motor_tx_buf[0]=0x55;
	uart_motor_tx_buf[1]=len+5;	
	uart_motor_tx_buf[2]=id;
	if(len!=0) memcpy(&uart_motor_tx_buf[3],buf,len);	
	uart_motor_tx_buf[len+3]=TX_CheckSum(uart_motor_tx_buf, len+3);
	uart_motor_tx_buf[len+4]=0xAA;
	u_motor_send_data(buf, len+5);
}

//*****************APP ***************************//
//参数配置
void app_u_motor_param_config_init(void )
{

}
//状态查询
void app_u_motor_get_sta_req(void )
{		
	unsigned char sendBuff=0;
	AppUsartMotorTransmit(U_MOTOR_ID_STATUS_INFO_REQ,&sendBuff,0);		
}

//位置设置
void app_u_motor_position_set(un_motor_positon_set* p_set)
{		
	AppUsartMotorTransmit(U_MOTOR_ID_POSITION_MODE_SET,&(p_set->data[4]),13);		
}
//
void app_u_motor_angle_cali(void )
{		
	unsigned char sendBuff=0;
	AppUsartMotorTransmit(U_MOTOR_ID_ENCODE_CALI,&sendBuff,0);
	//osDelay(2000);
}
void app_u_motor_run_enable(void )
{		//id 0x02	
	if(u_motor_sta_replay.sta.device_enable==0)
	{
		AppUsartMotorTransmit(U_MOTOR_ID_GENE_WORK_ENABLE,uart_motor_tx_buf,0);	
	}		
}
//������
// spd >0 ˳ʱ�룬spd <0 ��ʱ��
void app_u_motor_start(unsigned char s_t_mode, int spd,float torqueI)
{	
	if(s_t_mode==0)	//速度模式
	{
		u_f_uni.f_value=spd*1.0;	
		AppUsartMotorTransmit(U_MOTOR_ID_SPEED_MODE_SET,u_f_uni.data,sizeof(float));
	}
	else//扭矩模式
	{
		un_set_speed_tq_limit_struct u_p_str;
		u_p_str.tq_set.iq_ref=torqueI;
		u_p_str.tq_set.speed_ref=spd*1.0;
		AppUsartMotorTransmit(U_MOTOR_ID_TORQUE_MODE_SET,&(u_p_str.data[4]),8);			
	}
}
//软件停止
void app_u_motor_stop(void )
{	//iD=0x12	
	AppUsartMotorTransmit(U_MOTOR_ID_STANSBY_MODE,uart_motor_tx_buf,0);			
}
//软件复位
void app_u_motor_reset(void )
{

}
//�������ݴ���
void app_u_motor_handle(unsigned char *data,unsigned char packLen )
{				
	unsigned char codeId;
	codeId=data[2]&U_MOTOR_CODE_ID_MASK;		
	if(codeId==U_MOTOR_ID_STATUS_INFO_REQ)
	{
		if((data[2]&0x01)!=0)
		{
			memcpy(u_motor_sta_replay.data,&data[3],sizeof(get_status_reply_struct));	
        			
		}			
	}		
}  

//-----����У��-----------------------
uint8_t TX_CheckSum(uint8_t *buf, uint8_t len) //bufΪ���飬lenΪ���鳤��
{
	uint8_t i, ret = 0;
	for (i = 0; i<len; i++)
	{
	ret += *(buf++);
	} 
	return ret; // ����У������
}

/**
 * @brief �����������ݲ�У��Э���
 * @param data �������ݻ��������ᱻ�޸����ڻ������ڣ�
 * @param checkLen ��ǰ��������Ч���ݳ���
 * @return ���ص�һ���Ϸ����ĳ��ȣ������򷵻�0
 */
unsigned char app_u_rec_check(unsigned char *data, unsigned char checkLen)
{
    
unsigned char calculated_crc ; // �ų�CRC����
        unsigned char received_crc ;                // CRCλ��ĩβ
unsigned char remaining;
    unsigned char readlen = 0;
    unsigned char retLen = 0;
	unsigned char packLen;
    const unsigned char minPackSize = 5; // ��С����С��֡ͷ(2)+����(1)+CRC(1)+����(>=1)
if (!data || checkLen < 5) return 0;
    while ((readlen + 1) < checkLen)
    {
        // ����֡ͷ 0x55 0xAA
        if (data[readlen] != 0x55 || data[readlen + 1] != 0xAA)
        {
            readlen++;
            continue;
        }

        // ��ȡ�������ֶ�
        if (readlen + 2 >= checkLen) break;
         packLen = data[readlen + 2];

        // �жϰ����Ⱥ�����
        if (packLen < minPackSize || packLen > (checkLen - readlen))
        {
            readlen++;
            continue;
        }

        // У��CRC
         calculated_crc = TX_CheckSum(&data[readlen], packLen - 1); // �ų�CRC����
      received_crc = data[readlen + packLen - 1];                // CRCλ��ĩβ

        if (calculated_crc == received_crc)
        {
            app_u_motor_handle(&data[readlen], packLen);
            retLen = packLen;
            readlen += packLen;
            break; // �ҵ��׸���ȷ�����˳�ѭ��
        }
        else
        {
            readlen++; // CRCʧ��Ҳǰ��һλ���²���֡ͷ
        }
    }

    // ���ݻ��������ƶ�δ�����Ĳ�����ͷ��
     remaining = checkLen - readlen;
    if (remaining > 0 && readlen > 0)
    {
        memmove(data, &data[readlen], remaining);
    }

    return retLen;
}

/**
  * @brief  USART3_IRQHandler
  * @param  unsigned char cmdNum,unsigned char *str
  * @retval none
  */
void USART3_IRQHandler(void)
{		
	uint16_t data;
	if(usart_flag_get(USART3, USART_RDBF_FLAG) != RESET)               
	{
		usart_flag_clear (USART3, USART_RDBF_FLAG);
		data =	usart_data_receive (USART3);
		receiveLen%=(MAX_U_MOTOR_FIFO_LEN+1);		
		uart_motor_rx_buf[receiveLen]=data&0xFF;
		receiveLen++;
	}	
	if(usart_flag_get(USART3, USART_IDLEF_FLAG) != RESET)       
	{		
		usart_flag_clear (USART3, USART_IDLEF_FLAG);
		
	}	
}




