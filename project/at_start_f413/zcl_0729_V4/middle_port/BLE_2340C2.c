
#include "at32f413_board.h" 
#include "BLE_2340C2.h"
#include "usart_port.h"
#include "string.h"
#include "stdio.h"

uint8_t uart3_tx_buf[USART3_MAX_FIFO_LEN];
//uint8_t uart3_rx_buf[USART3_MAX_FIFO_LEN];

uint8_t uart3_tx_dma_status=1 ;
static unsigned char MAC_LOCAL_DEVICE[BLE_MAC_ASCII_LENGTH]={0};//"F1:76:7A:CA:67:10"};//当前设备MAC
static unsigned char LOCAL_DEVICE_ADV_NANE[BLE_DEVICE_NAME_LENGTH]={'n','D','M','x','6','7','1','0','\r','\n'};//本机设备名称匹配“EDMx” 
//static unsigned char SLAVER_ADV_NANE[BLE_SLAVE_DEVICE_NAME_LENGTH]={'R','N','G','C','6','7','1','0'};//本机设备名称匹配“RNGC” 0x520x4E0x470x43
BLE_RECEIVE_DATA_PACKAGE	ble_package;

typedef struct {
	unsigned char  head;	//头
	unsigned char  tail;//尾
	unsigned char  fullFlag;//满标志
	unsigned char  receiveBuff[USART3_MAX_FIFO_LEN];	
}BLE_RECEIVED_FIFO;
BLE_RECEIVED_FIFO ble_received_fifo;

typedef struct {
	unsigned char  blank0;	//0x20,格式数据
	unsigned char  num;	//返回的序列号
	unsigned char  blank1;	//0x20,格式数据
	unsigned char  mac[BLE_MAC_ASCII_LENGTH];	
	unsigned char  blank2;	
	unsigned char  rssi[3];//长度会有3个字节的波动，RSSI<=-100db丢弃，RSSI限定在-10db到-99db，固定长度,
	unsigned char  blank3;	
	unsigned char  deviceName[BLE_SLAVE_DEVICE_NAME_LENGTH];
	unsigned char  blank4;//‘\r’结束符0x0D便于存储	
	unsigned char  blank5;//‘\n’结束符0x0A便于存储	
	unsigned char  blank6;//保留	,可变位
}BLE_DEVICE_INFO_LIST;//36byte

//union  BLE_device_info_Union//所有参数均按照16位定义 10个记忆模式
//{
//	BLE_DEVICE_INFO_LIST  ble_device_info;//4个设备列表
//  unsigned char p_buff[36];//[sizeof(BLE_DEVICE_INFO_LIST)]
//};

unsigned char BLE_app_data_tx_buf[BLE_APPLICATION_DATA_LENGTH]={0};//固定数据包
 BLE_DEVICE_INFO_LIST  ble_device_info_list[BLE_MAX_DEVICE_NUM];
//static unsigned char DEVICE_CONNECTED_SUCCESS[29]={"F1:76:7A:CA:67:10 CONNECTED 0"};//
//static unsigned char MAC_AUTO_CONNECT_List[4][BLE_MAC_ASCII_LENGTH]={"F1:76:7A:CA:67:10","F1:76:7A:CA:67:10","F1:76:7A:CA:67:10","F1:76:7A:CA:67:10"};//
//static AT_CMD_NUMBER  MCU_at_cmd_analysis(AT_CMD_NUMBER atCmd);

flag_status get_ble_bcts_state(void);
#define PRINT_UART  USART3	
#if (__ARMCC_VERSION > 6000000)
  __asm (".global __use_no_semihosting\n\t");
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
  FILE __stdout;
#else
 #ifdef __CC_ARM
  #pragma import(__use_no_semihosting)
  struct __FILE
  {
    int handle;
  };
  FILE __stdout;
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
 #endif
#endif
#if defined (__GNUC__) && !defined (__clang__)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE  int  fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
#if 0
	while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
	usart_data_transmit(PRINT_UART, ch);
#else
	static uint32_t status;
	//while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
	do
	{
	status = PRINT_UART->sts;
	}while((status & USART_TDBE_FLAG) == RESET);

	//usart_data_transmit(PRINT_UART, ch);
	PRINT_UART->dt = (ch & 0x01FF);
#endif
  return ch;
}
void config_uart3_dma(void)
{
  dma_init_type dma_init_struct;

  /* enable dma1 clock */
  crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
  /* dma1 channel2 for usart1 tx configuration */
  dma_reset(DMA2_CHANNEL3);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = USART3_MAX_FIFO_LEN;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)uart3_tx_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL3, &dma_init_struct);

  /* config flexible dma for usart3 tx */
	usart_dma_transmitter_enable(USART3,TRUE);//DMA，usart TX开发送DMA
  dma_flexible_config(DMA2, FLEX_CHANNEL3, DMA_FLEXIBLE_UART3_TX);
  dma_channel_enable(DMA2_CHANNEL3, TRUE); /* usart3 tx begin dma transmitting */	
	dma_interrupt_enable(DMA2_CHANNEL3,DMA_FDT_INT,TRUE);
	nvic_irq_enable(DMA2_Channel3_IRQn, 2, 0);
  /* dma2 channel4 for usart3 rx configuration */
//	dma_reset(DMA2_CHANNEL4);
//  dma_default_para_init(&dma_init_struct);
//  dma_init_struct.buffer_size = USART3_MAX_FIFO_LEN;
//  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
//  dma_init_struct.memory_base_addr = (uint32_t)uart3_rx_buf;
//  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
//  dma_init_struct.memory_inc_enable = TRUE;
//  dma_init_struct.peripheral_base_addr = (uint32_t)&USART3->dt;
//  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
//  dma_init_struct.peripheral_inc_enable = TRUE;
//  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
//  dma_init_struct.loop_mode_enable = TRUE;
//  dma_init(DMA2_CHANNEL4, &dma_init_struct);

//  /* config flexible dma for usart3 tx */
//  dma_flexible_config(DMA2, FLEX_CHANNEL4, DMA_FLEXIBLE_UART3_RX);
//  dma_channel_enable(DMA2_CHANNEL4, TRUE); /* usart3 rx begin dma transmitting */	

}
/**
  * @brief  initialize uart gpio
  * @param  baudrate: uart baudrate
  * @retval none
  */
void uart3_gpio_init(uint32_t baudrate)
{
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
  usart_init(USART3, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);	
	
	
	usart_receiver_enable(USART3, TRUE); 
	nvic_irq_enable(USART3_IRQn, 4, 0);
	usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
	usart_flag_clear (USART3, USART_IDLEF_FLAG);	
//	usart_interrupt_enable(USART3, USART_RDBF_INT|USART_IDLE_INT, TRUE);
//	usart_flag_clear (USART3, USART_IDLEF_FLAG);	
	usart_enable(USART3, TRUE);	 
	config_uart3_dma();		
}

/**
  * @brief  usart3_send_data
  * @param  u8 *buf, u16 len
  * @retval none
  */
void usart3_send_data(u8 *buf, u16 len)
{  	
//while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET){;} 
	
	while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET){;} //	
	memcpy(uart3_tx_buf, buf, len); 	
  dma_channel_enable(DMA2_CHANNEL3, FALSE); 	 	 
  dma_data_number_set(DMA2_CHANNEL3, len);  
  dma_channel_enable(DMA2_CHANNEL3, TRUE);	 
}
/**
  * @brief  BLE_2340C2_gpio_init
  * @param  none
  * @retval none
  */
void BLE_2340C2_gpio_init(void)
{
  gpio_init_type gpio_init_struct;
	//TRANSMIT FLAG,上升沿表示模块发送给MCU数据完成
  crm_periph_clock_enable(BLE_2340C2_TRANS_FINISHED_CLOCK,TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins =BLE_2340C2_TRANS_FINISHED_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(BLE_2340C2_TRANS_FINISHED_PORT, &gpio_init_struct); 
	
//	nvic_irq_enable(EXINT15_10_IRQn, 3, 0);
//	exint_interrupt_enable(EXINT15_10_IRQn, TRUE);	
//	exint_event_enable(EXINT15_10_IRQn, TRUE);	
	
	//POWER GPIO
//  crm_periph_clock_enable(BLE_2340C2_POWER_CLOCK,TRUE);
//  gpio_default_para_init(&gpio_init_struct);
//  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
//  gpio_init_struct.gpio_pins = BLE_2340C2_POWER_IO;
//	gpio_init_struct.gpio_out_type=GPIO_OUTPUT_PUSH_PULL;
//  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
//  gpio_init(BLE_2340C2_POWER_PORT, &gpio_init_struct);  
//RESET GPIO
  crm_periph_clock_enable(BLE_2340C2_RESET_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_init_struct.gpio_out_type=GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pins = BLE_2340C2_RESET_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(BLE_2340C2_RESET_PORT, &gpio_init_struct);
	
//CONNECT GPIO
  crm_periph_clock_enable(BLE_2340C2_CONNECT_CLOCK,TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = BLE_2340C2_CONNECT_IO;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(BLE_2340C2_CONNECT_PORT, &gpio_init_struct);  	
	
	 gpio_bits_reset(BLE_2340C2_RESET_PORT, BLE_2340C2_RESET_IO);
//	 gpio_bits_reset(BLE_2340C2_POWER_PORT, BLE_2340C2_POWER_IO);
	 
	uart3_gpio_init(115200);
}
/**
  * @brief  EXINT15_10_IRQHandler
  * @param  void
  * @retval none
  */
void EXINT15_10_IRQHandler(void)//模块数据接收完成
{
  if(exint_flag_get(EXINT15_10_IRQn))
  {
//	xSemaphoreGiveFromISR(xSemaphoreDispRfresh,0);//信号量，数据接收完成
    exint_flag_clear(EXINT15_10_IRQn);  
  }
}
/**
  * @brief 	MCU_BLE_data_Process
  * @param  systemTime
  * @retval none
  */
static AT_CMD_NUMBER  MCU_BLE_app_data_handle(unsigned char *pDat,unsigned char Len)
{
	AT_CMD_NUMBER ret;
	ret=AT_NULL;
	if(pDat[0]==0xA0)//来自定位仪
	{
		ret=UNVARNISHED_BLE_DATA_REV;
	}
//	else if(pDat[0]==0xB0)//来自脚踏
//	{ 
//	}
	return ret;
}
/**
  * @brief 	MCU_BLE_cmd_data_handle
  * @param  systemTime
  * @retval none
  */
 AT_CMD_NUMBER  MCU_BLE_cmd_data_handle(unsigned char *pDat,AT_CMD_NUMBER AtCmd)
{
	AT_CMD_NUMBER ret;
	ret=AT_NULL;
	if(AtCmd==AT_NAME)
	{
		if(memcmp(pDat,CMD_AT_NAME_CMP,strlen(CMD_AT_NAME_CMP))==0)//匹配成功
		{
			ret=AT_NAME;
		}
		else
		{				
			LOCAL_DEVICE_ADV_NANE[0]='E';//名称前4位
			LOCAL_DEVICE_ADV_NANE[1]='D';
			LOCAL_DEVICE_ADV_NANE[2]='M';
			LOCAL_DEVICE_ADV_NANE[3]='x';
			LOCAL_DEVICE_ADV_NANE[4]=MAC_LOCAL_DEVICE[12];//名称后4位
			LOCAL_DEVICE_ADV_NANE[5]=MAC_LOCAL_DEVICE[13];
			LOCAL_DEVICE_ADV_NANE[6]=MAC_LOCAL_DEVICE[15];
			LOCAL_DEVICE_ADV_NANE[7]=MAC_LOCAL_DEVICE[16];
			ret = AT_LOCAL_NAME_ERR;//	
		}				
	}	
	else if(AtCmd==AT_MAC)
	{
		memcpy(MAC_LOCAL_DEVICE,&pDat[7],BLE_MAC_ASCII_LENGTH);
		LOCAL_DEVICE_ADV_NANE[4]=MAC_LOCAL_DEVICE[12];//名称后4位
		LOCAL_DEVICE_ADV_NANE[5]=MAC_LOCAL_DEVICE[13];
		LOCAL_DEVICE_ADV_NANE[6]=MAC_LOCAL_DEVICE[15];
		LOCAL_DEVICE_ADV_NANE[7]=MAC_LOCAL_DEVICE[16];	
		ret=AT_MAC;
	}
	else // if(AtCmd==AT_UUID_SCAN)
	{
		ret=AtCmd;
	}
  return ret;
}
/**
  * @brief  MCU_usart3_read_bytes
  * @param  unsigned char data
  * @retval none
  */
unsigned char  MCU_usart3_read_bytes(AT_CMD_NUMBER atMode)
{	
	unsigned char ret;	
	ret=ble_package.Len;
	if(get_ble_bcts_state())//传输结束
	{		
		if(ble_received_fifo.fullFlag!=0)//缓存满已提取
		{
			ble_package.Len=USART3_MAX_FIFO_LEN;
			if(ble_received_fifo.head!=ble_received_fifo.tail)
			{
				ble_package.Len=USART3_MAX_FIFO_LEN+ble_received_fifo.head-ble_received_fifo.tail;//>USART3_MAX_FIFO_LEN
				ble_received_fifo.tail=ble_received_fifo.head;
			}					
			ble_received_fifo.fullFlag = 0;					
		}
		else 
		{
			if(ble_received_fifo.head<ble_received_fifo.tail)
			{				
				ble_package.Len=ble_received_fifo.head+USART3_MAX_FIFO_LEN-ble_received_fifo.tail;
				ble_package.Len%=USART3_MAX_FIFO_LEN;				
				memcpy(ble_package.dataBuff,&ble_received_fifo.receiveBuff[ble_received_fifo.tail%USART3_MAX_FIFO_LEN],USART3_MAX_FIFO_LEN-ble_received_fifo.tail);//前半段
				if(ble_received_fifo.head!=0) memcpy(&ble_package.dataBuff[USART3_MAX_FIFO_LEN-ble_received_fifo.tail],ble_received_fifo.receiveBuff,ble_received_fifo.head);//后半段						
				ble_received_fifo.tail = ble_received_fifo.head;
				ret=ble_package.Len;
			}
			else if(ble_received_fifo.head>ble_received_fifo.tail)
			{	
				ble_package.Len=ble_received_fifo.head-ble_received_fifo.tail;
				memcpy(ble_package.dataBuff,&ble_received_fifo.receiveBuff[ble_received_fifo.tail],ble_package.Len);	
				ble_received_fifo.tail=ble_received_fifo.head;
				ret=ble_package.Len;
			}		
		}
		 if(ble_received_fifo.head==ble_received_fifo.tail&&ble_package.Len!=0)
	 {
		 ret=ble_package.Len;
	 }
	}
		return ret;//NULL
}
/**
  * @brief  usart3_receive_bytes
  * @param  unsigned char data
  * @retval none
  */
void usart3_receive_bytes(unsigned char data)
{				
		ble_received_fifo.head%=	USART3_MAX_FIFO_LEN;
		ble_received_fifo.receiveBuff[ble_received_fifo.head]=data;		
		if((ble_received_fifo.head+1)%USART3_MAX_FIFO_LEN==ble_received_fifo.tail)//buff 满了 			
		{	
			ble_received_fifo.fullFlag=1;//标记一次//+=1;
			if(ble_package.Len==0)//buff满，提取数据，马上处理
			{
				ble_package.Len=USART3_MAX_FIFO_LEN;			
				memcpy(ble_package.dataBuff,&ble_received_fifo.receiveBuff[ble_received_fifo.tail],USART3_MAX_FIFO_LEN-ble_received_fifo.tail);//前半段
				if(ble_received_fifo.head!=0) memcpy(&ble_package.dataBuff[USART3_MAX_FIFO_LEN-ble_received_fifo.tail],ble_received_fifo.receiveBuff,ble_received_fifo.head);//后半段															
				ble_received_fifo.head=0;//提取结束归0
				ble_received_fifo.tail = ble_received_fifo.head;	
			}	
			else //处理上一包数据还未处理，丢弃
			{
				ble_received_fifo.head=0;
				ble_received_fifo.tail=ble_received_fifo.head;
				ble_received_fifo.fullFlag=0;
			}	
		}
		ble_received_fifo.head+=1;
		ble_received_fifo.head%=	USART3_MAX_FIFO_LEN;		
		//xSemaphoreGiveFromISR(xSemaphoreDispRfresh,0);//信号量
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
		usart3_receive_bytes( data);	
	}	
	if(usart_flag_get(USART3, USART_IDLEF_FLAG) != RESET)       
	{		
		usart_flag_clear (USART3, USART_IDLEF_FLAG);
//		dma_channel_enable(DMA2_CHANNEL4, FALSE);//close rx dma 
//len=dma_data_number_get(DMA2_CHANNEL4);		//copy data
		//		usart3_receive_bytes(0);
//		dma_channel_enable(DMA2_CHANNEL4, TRUE);//start rx dma
		
	}
	
}
/**
  * @brief  usart3_send_bytes
  * @param  unsigned char cmdNum,unsigned char *str
  * @retval none
  */
void usart3_send_bytes(unsigned char *str,unsigned short int len)
{	
	unsigned char package=0;
	if(len>USART3_MAX_FIFO_LEN)//分包
	{
			package=len/USART3_MAX_FIFO_LEN;					
			do
			{
				while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET){;} //
//				while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET){;} //等待缓存非空				
				memcpy(uart3_tx_buf,str,USART3_MAX_FIFO_LEN);
				uart3_tx_dma_status=0;
				dma_channel_enable(DMA2_CHANNEL3, FALSE); 	 	 
				dma_data_number_set(DMA2_CHANNEL3, USART3_MAX_FIFO_LEN);  
				dma_channel_enable(DMA2_CHANNEL3, TRUE);
				str+=USART3_MAX_FIFO_LEN;			
				package--;
			}while(package==0);
			if(len%USART3_MAX_FIFO_LEN!=0)
			{
				while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET){;} //	
				memcpy(uart3_tx_buf,str,len%USART3_MAX_FIFO_LEN);
				uart3_tx_dma_status=0;
				dma_channel_enable(DMA2_CHANNEL3, FALSE); 	 	 
				dma_data_number_set(DMA2_CHANNEL3, len%USART3_MAX_FIFO_LEN);  
				dma_channel_enable(DMA2_CHANNEL3, TRUE);	
			}
	}
	else
	{
		while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET){;} //	
		memcpy(uart3_tx_buf,str,len);
		uart3_tx_dma_status=0;
		dma_channel_enable(DMA2_CHANNEL3, FALSE); 	 	 
		dma_data_number_set(DMA2_CHANNEL3, len);  
		dma_channel_enable(DMA2_CHANNEL3, TRUE);	
	}
}
/**
  * @brief  BLE power off
  * @param  none
  * @retval none
  */
void BLE_power_off(unsigned char OnOff)//still on
{	
//	if(OnOff==0)//off
//	{
//		gpio_bits_set(BLE_2340C2_POWER_PORT, BLE_2340C2_POWER_IO);
//	}
//	else gpio_bits_reset(BLE_2340C2_POWER_PORT, BLE_2340C2_POWER_IO);
}
/**
  * @brief  BLE_RESET
  * @param  none
  * @retval none
  */
void BLE_rst(unsigned char  setOrReset)
{
	if(setOrReset==0)//RESET
	{
		gpio_bits_reset(BLE_2340C2_RESET_PORT, BLE_2340C2_RESET_IO);
	}
	else 	gpio_bits_set(BLE_2340C2_RESET_PORT, BLE_2340C2_RESET_IO);
}

/**input
  * @brief  get_ble_bcts_state
  * @param  GPIO level
	* @retval  status: SET or RESET
  */
flag_status get_ble_bcts_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(BLE_2340C2_TRANS_FINISHED_PORT,BLE_2340C2_TRANS_FINISHED_IO);
   return re;
}

/**input
  * @brief  get_ble_connect_state
  * @param  GPIO level
	* @retval  status: SET or RESET
  */
flag_status get_ble_connect_state(void)
{
   flag_status re;
   re=gpio_input_data_bit_read(BLE_2340C2_CONNECT_PORT,BLE_2340C2_CONNECT_IO);
   return re;
}
/*****************************BLE****************************/ 


/**
  * @brief  BLE_transmit
  * @param  unsigned char cmdNum,unsigned char *str
  * @retval none
  */

void BLE_transmit(AT_CMD_NUMBER cmdNum,unsigned char typeOrLen,unsigned char *str)
{
	ble_package.Len=0;	
	if(cmdNum==UNVARNISHED_BLE_DATA_REV)//only data，unvarnished transmission
	{
		usart3_send_bytes(str,typeOrLen);	
	}
	else if(cmdNum==AT_MODE_ENABLE)//AT mode in
	{
		printf(CMD_AT_MODE_ENABLE);		//only set	
	} 
	else if(cmdNum==AT_MAC)//AT mode in
	{
		printf(CMD_AT_MAC_INQUIRE);		//only ?
	}
	else if(cmdNum==AT_NAME)//AT mode in
	{
		printf(CMD_AT_NAME_SERTCH);		//SEARCH		
	}
	else if(cmdNum==AT_ROLE)//AT mode exit
	{		
	 	printf(CMD_AT_ROLE_ONLY_MASTER_SET);	
	} 
	else if(cmdNum==AT_POWER)//AT POWER
	{
		printf(CMD_AT_POWER);		//tx power
	}
	else if(cmdNum==AT_NAME_SET)//AT mode in
	{
		printf(CMD_AT_ADV_NAME_SET);		//SET
		usart3_send_bytes(LOCAL_DEVICE_ADV_NANE,BLE_DEVICE_NAME_LENGTH);			
	}
	else if(cmdNum==AT_MSERVICE)//AT UUID
	{
		if(typeOrLen==TYPE_AT_SEARCH)
		{
			printf(CMD_AT_MASTER_SERVER_INQUIRE);	
		}
		else if(typeOrLen==TYPE_AT_SET)
		{
			printf(CMD_AT_MASTER_UUID_SET);	
		}		
	}
	else if(cmdNum==AT_OBSERVER)//AT observe
	{
		printf(CMD_AT_OBSERVER);		//observe	
	}	
	
	else if(cmdNum==AT_MODE_DISABLE)//AT mode exit
	{		
	 	printf(CMD_AT_MODE_DISABLE);		
	} 
		else if(cmdNum==AT_SCAN)// AT_SCAN
	{		
	 	printf(CMD_AT_SCAN);			
	} 
		else if(cmdNum==AT_SCAN_SET)// AT_SCAN_SET
	{		
	 	printf(CMD_AT_SCAN_SET);			
	} 
	else if(cmdNum==AT_CONNECT)//AT connect
	{		
	 	printf(CMD_AT_CONNECT);	
//	usart3_send_bytes(&ble_device_info_list[0].num,1);//num
		usart3_send_data(&ble_device_info_list[0].num, 1);
		printf("\r\n");
	} 
	else if(cmdNum==AT_DISCONNECT)//AT connect
	{		
	 	printf( CMD_AT_DISCONNECT);			
	} 	
	else if(cmdNum==AT_TTM_HANDLE)//AT TTM
	{		
	 	printf(CMD_AT_TTM_HANDLE);			
	} 	
	else if(cmdNum==AT_TRX_CHAN)//AT rtx
	{		
	 	printf(CMD_AT_TRX_CHAN);			
	} 
	else if(cmdNum==AT_AUTO_CNT)
	{
		printf(CMD_AT_AUTO_CONNECT);//ALL
//		usart3_send_bytes(ble_device_info_list[0].mac,BLE_MAC_ASCII_LENGTH);//num
//		printf("\r\n");
	}	
	else if(cmdNum==AT_DEL_AUTO_CONNECT_ALL)
	{
		printf(CMD_AT_DEL_AUTO_CONNECT_ALL);
	}
	else if(cmdNum==AT_UUID_SCAN)
	{
		printf(CMD_AT_UUID_SCAN);
	}
	else if(cmdNum==AT_RESTART)
	{
		printf(CMD_AT_RESTART);
	}
	else if(cmdNum==AT_SLEEP)
	{
		printf(CMD_AT_SLEEP);
	}	
	else if(cmdNum==AT_RESET)
	{
		printf(CMD_AT_RESET);
	}	
		else if(cmdNum== AT_VERSION)
	{
		printf( CMD_AT_VERSION);
	}	
}

/**
  * @brief 	At_scan_rsp_data_handle
  * @param  unsigned char *pDat,unsigned short int packLen
  * @retval unsigned char
  */
static unsigned char  At_scan_rsp_data_handle(unsigned char *pDat,unsigned short int packLen)
{
	unsigned char ret=0;
	unsigned short int len=0;
	unsigned short int read=0;
	//handle=pDat[1]
	while(read+1<packLen)//strlen(CMD_AT_OK)=4;
	{
		if(pDat[read]=='\r'&&pDat[read+1]=='\n')
		{			
			if(memcmp(&pDat[read-8+len],BLE_DEVICE_GC_NAME_FILTER,4)==0)
			{
				memcpy(&ble_device_info_list[0],&pDat[len],sizeof(BLE_DEVICE_INFO_LIST));//handle=0
				BLE_app_data_tx_buf[5]=ble_device_info_list[0].mac[13];
				BLE_app_data_tx_buf[6]=ble_device_info_list[0].mac[15];
				BLE_app_data_tx_buf[7]=ble_device_info_list[0].mac[16];
				ret=1;//end			
				break;
			}	
			len+=read+1+1;			
		}
		read++;		 
	}
	return ret;
}

/**
  * @brief 	MCU_at_cmd_analysis
  * @param  systemTime
  * @retval none
  */
 AT_CMD_NUMBER MCU_at_cmd_analysis(AT_CMD_NUMBER atCmd)
{
		AT_CMD_NUMBER ret=AT_NULL;		
		if(MCU_usart3_read_bytes(atCmd)==0)	 return ret;		
		if(atCmd==AT_MODE_ENABLE)
		{	
			if(memcmp(ble_package.dataBuff,CMD_AT_ACK,strlen(CMD_AT_ACK))==0)//“OK\r\n”，
			{
				ret=AT_ONLY_ACK;				
			}
		}
		else if(atCmd==AT_DEVICE_START_HINT)
		{	
			if(memcmp(ble_package.dataBuff,CMD_DEVICE_START_HINT,strlen(CMD_DEVICE_START_HINT)))
			{
				ret=AT_DEVICE_START_HINT;			
			}
		}		
		else if(atCmd==AT_SCAN)
		{
			 if(memcmp(ble_package.dataBuff,CMD_AT_ACK,strlen(CMD_AT_ACK))==0)//“ok\r\n”
			 {
					if(ble_package.Len==4)
					{
						ret=AT_ONLY_ACK;		//timeout			
					}
					else
					{
						if(ble_package.dataBuff[strlen(CMD_AT_ACK)]==' ')
						{	
							if(At_scan_rsp_data_handle(&ble_package.dataBuff[strlen(CMD_AT_ACK)],ble_package.Len-strlen(CMD_AT_ACK))!=0)
							{
								ret = AT_SCAN;							
							}
              else 	ret = AT_ERR_ERROR;//ee
						}
					}
			 }			
			  else
				{
						if(ble_package.dataBuff[0]==' ')
						{	
							if(At_scan_rsp_data_handle(ble_package.dataBuff,ble_package.Len)!=0)
							{
								ret = AT_SCAN;								
							}	
							else 	ret = AT_ERR_FAIL;	//tt						
						}
						 else if(memcmp(ble_package.dataBuff,CMD_AT_BLE_DATA_REV,strlen(CMD_AT_BLE_DATA_REV))==0)//
						 {
							 ret=MCU_BLE_app_data_handle(&ble_package.dataBuff[strlen(CMD_AT_BLE_DATA_REV)],BLE_APPLICATION_DATA_LENGTH);
						 }
				}		
		}
		else if(atCmd==AT_CONNECT)
		{				
			if(memcmp(ble_package.dataBuff,CMD_AT_ACK,strlen(CMD_AT_ACK))==0)//“OK\r\n”，
			{		
				if(ble_package.Len==4)			
				{
					ret=AT_ONLY_ACK;            						
				}
				else
				{
					if(memcmp(&ble_package.dataBuff[strlen(CMD_AT_ACK)],ble_device_info_list[0].mac,BLE_MAC_ASCII_LENGTH)==0)//连接MAC地址匹配，只用第一个
					{				
						if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH+strlen(CMD_AT_ACK)],BLE_DISCONNECTED_HINT,strlen(BLE_DISCONNECTED_HINT))==0)
						{
							ret=DISCONNECTED_HINT;
						}							
						 else	if(memcmp(ble_package.dataBuff,ERROR_BLE_CONNECT_ALREADY_FAIL,strlen(ERROR_BLE_CONNECT_ALREADY_FAIL))==0)
						 {
							 ret=CONNECTED_SUCCESS;//CONNECT ready						
						 }
						else if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH+strlen(CMD_AT_ACK)],BLE_CONNECTED_HINT,strlen(BLE_CONNECTED_HINT))==0)
						{
							if(ble_package.Len>USART3_MAX_FIFO_LEN)	//chan，UUID数据
							{		
								ble_package.Len=ble_package.Len-USART3_MAX_FIFO_LEN;
								if(memcmp(&ble_received_fifo.receiveBuff[ble_package.Len-strlen(BLE_SCAN_SLAVER_UUID_SUCCESS)],BLE_SCAN_SLAVER_UUID_SUCCESS,strlen(BLE_SCAN_SLAVER_UUID_SUCCESS))==0)//success
								{
									ret=CONNECTED_SUCCESS;
								}		
							}	
						
						}
							else if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH+strlen(CMD_AT_ACK)],BLE_CONNECTED_FAIL,strlen(BLE_CONNECTED_FAIL))==0)
						 {
								ret = CONNECTED_FAIL;	
						 }
							else if(memcmp(ble_package.dataBuff,CMD_AT_BLE_DATA_REV,strlen(CMD_AT_BLE_DATA_REV))==0)//
						 {
							 ret=MCU_BLE_app_data_handle(&ble_package.dataBuff[strlen(CMD_AT_BLE_DATA_REV)],BLE_APPLICATION_DATA_LENGTH);
								if(ret==UNVARNISHED_BLE_DATA_REV)
								{							
									ret=CONNECTED_SUCCESS;	
								}
						 }
					}
				}			
			}
			else
			{
				if(memcmp(ble_package.dataBuff,ble_device_info_list[0].mac,BLE_MAC_ASCII_LENGTH)==0)//连接MAC地址匹配，只用第一个
				{				
					if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_DISCONNECTED_HINT,strlen(BLE_DISCONNECTED_HINT))==0)
					{
						ret=DISCONNECTED_HINT;
					}
					else	if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_CONNECTED_TIMEOUT,strlen(BLE_CONNECTED_TIMEOUT))==0)
					{
					 ret=CONNECTED_TIMEOUT;						
					}				
					else if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_CONNECTED_HINT,strlen(BLE_CONNECTED_HINT))==0)
					{
						if(ble_package.Len>USART3_MAX_FIFO_LEN)	//chan，UUID数据
						{
							ble_package.Len=ble_package.Len-USART3_MAX_FIFO_LEN;
							if(memcmp(&ble_received_fifo.receiveBuff[ble_package.Len-strlen(BLE_SCAN_SLAVER_UUID_SUCCESS)],BLE_SCAN_SLAVER_UUID_SUCCESS,strlen(BLE_SCAN_SLAVER_UUID_SUCCESS))==0)//success
							{
								ret=CONNECTED_SUCCESS;
							}	
						}	
//					  else
//						{
//							if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH+strlen(BLE_CONNECTED_HINT)],BLE_SCAN_SLAVER_UUID_INFO,strlen(BLE_SCAN_SLAVER_UUID_INFO))!=0)//新设备
//							{
//								ret=CONNECTED_SUCCESS_FIRST;
//							}
//						}
					}
					else if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_CONNECTED_FAIL,strlen(BLE_CONNECTED_FAIL))==0)
					{
						ret = CONNECTED_FAIL;		
					}
				}			
			   else if(memcmp(ble_package.dataBuff,CMD_AT_BLE_DATA_REV,strlen(CMD_AT_BLE_DATA_REV))==0)//
				 {
					ret=MCU_BLE_app_data_handle(&ble_package.dataBuff[strlen(CMD_AT_BLE_DATA_REV)],BLE_APPLICATION_DATA_LENGTH);						
				 }
				  else if(memcmp(ble_package.dataBuff,BLE_SCAN_SLAVER_UUID_SUCCESS,strlen(BLE_SCAN_SLAVER_UUID_SUCCESS))==0)//success
				 {
					ret=CONNECTED_SUCCESS;						
				 }
//				 else
//				{
//					if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH+strlen(BLE_CONNECTED_HINT)],BLE_SCAN_SLAVER_UUID_INFO,strlen(BLE_SCAN_SLAVER_UUID_INFO))!=0)//新设备
//					{
//						ret=CONNECTED_SUCCESS_FIRST;
//					}
//				}
			}
							
		}	
	
		else if(atCmd==AT_DISCONNECT)
		{				
			if(memcmp(ble_package.dataBuff,CMD_AT_ACK,strlen(CMD_AT_ACK))==0)//“OK\r\n”，
			{		
				if(ble_package.Len==4)			
				{
					ret=AT_ONLY_ACK;            						
				}
				else
				{
					if(memcmp(&ble_package.dataBuff[strlen(CMD_AT_ACK)],ble_device_info_list[0].mac,BLE_MAC_ASCII_LENGTH)==0)//连接MAC地址匹配，只用第一个
					{
						ret=AT_DISCONNECT;   
					}
				}
			}
			else
			{
				 if(memcmp(ble_package.dataBuff,CMD_AT_BLE_DATA_REV,strlen(CMD_AT_BLE_DATA_REV))==0)//
				 {
					ret=MCU_BLE_app_data_handle(&ble_package.dataBuff[strlen(CMD_AT_BLE_DATA_REV)],BLE_APPLICATION_DATA_LENGTH);						
				 }
			}
		}	
		else  if(atCmd==UNVARNISHED_BLE_DATA_REV)//透传模式
		{
			 if(memcmp(ble_package.dataBuff,ble_device_info_list[0].mac,BLE_MAC_ASCII_LENGTH)==0)//连接MAC地址匹配，只用第一个
			{
				if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_DISCONNECTED_HINT,strlen(BLE_DISCONNECTED_HINT))==0)
				{
					ret=DISCONNECTED_HINT;
				}
				else if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_CONNECTED_HINT,strlen(BLE_CONNECTED_HINT))==0)
				{
					 if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_CONNECTED_TIMEOUT,strlen(BLE_CONNECTED_TIMEOUT))==0)
					 {
						 ret=CONNECTED_TIMEOUT;						
					 }
					 else if(memcmp(&ble_package.dataBuff[BLE_MAC_ASCII_LENGTH],BLE_CONNECTED_FAIL,strlen(BLE_CONNECTED_FAIL))==0)
					 {
							ret= CONNECTED_FAIL;	
					 }					 
				}					 
			}	
			else
			{
				ret=MCU_BLE_app_data_handle(ble_package.dataBuff,ble_package.Len);//透传数据
			}					
		}	
		else
		{
			if(memcmp(ble_package.dataBuff,CMD_AT_HEAD1,strlen(CMD_AT_HEAD1))==0)
			{
				if(memcmp(&ble_package.dataBuff[ble_package.Len-strlen(CMD_AT_ACK)],CMD_AT_ACK,strlen(CMD_AT_ACK))==0)
				{
					ret=MCU_BLE_cmd_data_handle(ble_package.dataBuff,atCmd);				
				}				
			}	
			else if(memcmp(ble_package.dataBuff,CMD_AT_ACK,strlen(CMD_AT_ACK))==0)//“OK\r\n”，
			{
				ret=AT_ONLY_ACK;				
			}	
			else if(memcmp(ble_package.dataBuff,CMD_AT_BLE_DATA_REV,strlen(CMD_AT_BLE_DATA_REV))==0)//
			{
				ret=MCU_BLE_app_data_handle(ble_package.dataBuff,ble_package.Len);//透传数据
			}	
			else if(memcmp(ble_package.dataBuff,ERROR_AT_ERR_BUSY,strlen(ERROR_AT_ERR_BUSY))==0)
			{
				ret=AT_ERR_BUSY;		
			}else if(memcmp(ble_package.dataBuff,ERROR_AT_ERR_FAIL,strlen(ERROR_AT_ERR_FAIL))==0)
			{
				ret=AT_ERR_FAIL;		
			}
			else if(memcmp(ble_package.dataBuff,ERROR_AT_ERR_ERROR,strlen(ERROR_AT_ERR_ERROR))==0)
			{
				ret=AT_ERR_ERROR;		
			}							
		}
		return ret;
		
}
/*************************end file*****************************/

