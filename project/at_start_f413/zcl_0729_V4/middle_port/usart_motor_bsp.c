
#include "stdio.h"
#include "string.h"
#include "at32f413_usart.h"
#include "at32f413_gpio.h"
#include "at32f413_board.h"

#include "usart_motor_bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"	
 

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

U_MOTOR_PACK_STATUS u_motor_bus_idle_sta;
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
  usart_init(USART3, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);	
	
	
	usart_receiver_enable(USART3, TRUE); 
	nvic_irq_enable(USART3_IRQn, 4, 0);
	usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
	usart_flag_clear (USART3, USART_IDLEF_FLAG);	

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
	memcpy(buf, buf, len); 	
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
  uart_motor_tx_buf[1]=0xAA;
	uart_motor_tx_buf[2]=len+5;	
	uart_motor_tx_buf[3]=id;
  if(id!=U_MOTOR_ID_STATUS_INFO_REQ) u_motor_bus_idle_sta.idlesta=1;
	if(len!=0) memcpy(&uart_motor_tx_buf[4],buf,len);	
  
	uart_motor_tx_buf[len+4]=TX_CheckSum(uart_motor_tx_buf, len+4);
	
	u_motor_send_data(uart_motor_tx_buf, len+5);
}

//*****************APP ***************************//
//参数配置
void app_u_motor_param_config_init(void )
{
  u_motor_bus_idle_sta.idlesta=0;
  u_motor_bus_idle_sta.timeout=0;
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
	AppUsartMotorTransmit(U_MOTOR_ID_POSITION_MODE,&(p_set->data[4]),13);		
}
//
void app_u_motor_angle_cali(void )
{		
	unsigned char sendBuff=0;

  AppUsartMotorTransmit(U_MOTOR_ID_PREPARE_MODE,&sendBuff,0);//静默
  vTaskDelay(10); 
	AppUsartMotorTransmit(U_MOTOR_ID_ENCODE_CALI,&sendBuff,0);
}
void app_u_motor_angle_cali_next(void )
{
  unsigned char sendBuff=0;
  sendBuff=0x06;
  AppUsartMotorTransmit(0x08,&sendBuff,1);//转速比6：1
  vTaskDelay(10);
  AppUsartMotorTransmit(0x1A,&sendBuff,0);//电角度自动微调
  vTaskDelay(100);
  AppUsartMotorTransmit(U_MOTOR_ID_GENE_WORK_ENABLE,&sendBuff,0);//回复正常模式
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
    u_pos_set.p_set.mode=0;
		u_f_uni.f_value=spd*1.0;    
		AppUsartMotorTransmit(U_MOTOR_ID_SPEED_MODE,u_f_uni.data,sizeof(float));
		
		//un_set_speed_tq_limit_struct u_p_str;
		//u_p_str.tq_set.iq_ref=1.500;//torqueI;
		//u_p_str.tq_set.speed_ref=spd*1.0;
		//AppUsartMotorTransmit(U_MOTOR_ID_TORQUE_MODE_SET,&(u_p_str.data[4]),8);	
	}
  else if(s_t_mode==2)//往复运动
  {
    //位置设置  
    SEGGER_RTT_WriteString(0, "psi start\r\n");
    u_pos_set.p_set.mode =1;
    u_pos_set.p_set.position_ref1=150;
    u_pos_set.p_set.position_ref2=-150;
    u_pos_set.p_set.freq=2;                                  
    app_u_motor_position_set(&u_pos_set);
  }
	else //扭矩模式
	{
		un_set_speed_tq_limit_struct u_p_str;
		u_p_str.tq_set.iq_ref=torqueI;
		u_p_str.tq_set.speed_ref=spd*1.0;
		AppUsartMotorTransmit(U_MOTOR_ID_TORQUE_MODE_SET,&(u_p_str.data[4]),8);			
	}
}
//软件停止
void app_u_motor_stop(void )
{	
  //iD=0x12	
  unsigned char sendBuff=0;  
  AppUsartMotorTransmit(U_MOTOR_ID_STANSBY_MODE,&sendBuff,0);
}
//软件复位
void app_u_motor_reset(void )
{	
  unsigned char sendBuff=0;
    AppUsartMotorTransmit(U_MOTOR_ID_SOFT_RESET_REQ,&sendBuff,0);	
}
//�������ݴ���
void app_u_motor_handle(unsigned char *data,unsigned char packLen )
{				
	unsigned char codeId,dataLen=0;
  unsigned char i=0;
  if(data[2]>5)  dataLen=data[2]-5;
	codeId=data[3]&U_MOTOR_CODE_ID_MASK;
  
	if(codeId==U_MOTOR_ID_STATUS_INFO_REQ)
	{
    #if 0
    SEGGER_RTT_WriteString(0, "u_pack\r\n");
    for( i=0;i<packLen;i++)
    {
      SEGGER_RTT_printf(0, " %02x\r\n",data[i]);
    }
    SEGGER_RTT_printf(0, " -len=\r\n",packLen);
    #endif		
		if((data[3]&0x01)!=0)
		{     
      //  SEGGER_RTT_WriteString(0, "sta r\r\n");
			memcpy(u_motor_sta_replay.data,data,sizeof(get_status_reply_struct));	      

		}			
	}	
  else if(codeId==U_MOTOR_ID_SPEED_MODE)
  {
    u_motor_bus_idle_sta.idlesta=0;
    if((data[3]&0x01)!=0)
		{         
    // SEGGER_RTT_printf(0, "spd l=%d\r\n",packLen);
    }
  }	
  else if(codeId==U_MOTOR_ID_POSITION_MODE)
  {
    u_motor_bus_idle_sta.idlesta=0;
    if((data[3]&0x01)!=0)
		{ 
     //SEGGER_RTT_printf(0, "posi l=%d\r\n",packLen);
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
* @brief 协议帧数据校验处理函数
* @param data 输入的数据缓冲区，可能会被修改用于调整未解析部分的位置
* @param checkLen 当前传入的有效数据长度
* @return 返回第一个合法包的长度，若无则返回0
*/
unsigned char app_u_rec_check(unsigned char *data, unsigned char checkLen)
{
    unsigned char calculated_crc ; // 计算出的CRC值
    unsigned char received_crc ;   // 接收到的CRC值
    unsigned char remaining;
    unsigned char readlen = 0;
    unsigned char retLen = 0;
    unsigned char packLen;
    const unsigned char minPackSize = 5; // 最小包长=帧头(2)+长度(1)+CRC(1)+负载(>=1)

    if (!data || checkLen < minPackSize) return 0;

    while ((readlen + 1) < checkLen)
    {
        // 判断帧头 0x55 0xAA
        if (data[readlen] == 0x55 && data[readlen + 1] == 0xAA)
        {
            // 获取长度字段
            if (readlen + 2 >= checkLen) break;
            packLen = data[readlen + 2];

            // 判断包长度有效性
            if (packLen >= minPackSize && packLen <= (checkLen - readlen))
            {
                // 校验CRC
                calculated_crc = TX_CheckSum(&data[readlen], packLen - 1); // 不包括CRC本身
                received_crc = data[readlen + packLen - 1];               // CRC位于末尾

                if (calculated_crc == received_crc)
                {                  
                    app_u_motor_handle(&data[readlen], packLen);
                    retLen = packLen;
                    readlen += packLen;
                    break; // 找到正确包退出循环
                }
                else
                {
                    readlen++; // CRC失败也前进一位重新查找帧头
                }
            }
            else
            {
                readlen++; // 长度无效继续向前找下一可能帧头位置
            }
        }
        else
        {
            readlen++;     // 帧头不符合继续向前找下一可能帧头位置
        }
    }

    // 将剩余未能解析的部分移动至缓存起始处
    remaining = checkLen - readlen;
    if (remaining > 0 && readlen > 0)
    {
      memmove(data, &data[readlen], remaining);
    }

    return retLen;
}
unsigned char  app_u_motor_rec_data(void )
{
  unsigned char retlen=0;
  unsigned char peekLen=receiveLen;
  if(receiveLen>=5)
  {
    retlen=app_u_rec_check(uart_motor_rx_buf, peekLen);
    if(receiveLen>retlen)  receiveLen-=retlen;
  }
   
  return retlen;
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




