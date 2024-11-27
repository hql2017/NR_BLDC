#ifndef __BLE_2340C2
#define __BLE_2340C2

#define BLE_COMMUNICATION_GAP_TIME_MS 10// 50//MAX_WDT_FEED_TIME_MS    300


#define USART3_MAX_FIFO_LEN  253//128
//UUID
#define LBS_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}

#define LBS_UUID_SERVICE     0x1523
#define LBS_UUID_BUTTON_CHAR 0x1524
#define LBS_UUID_LED_CHAR    0x1525		
															
#define  BLE_MASTER_CONFIG    0   //出厂配置状态
#define  BLE_SCAN_STATUS       1		//扫描状态
#define  BLE_CONNECT_MANAGE   2	// 连接管理状态															
#define  BLE_APPLICATION       3 //通信管理状态															
#define  BLE_WAIT_ACK    4
#define  BLE_POWER_OFF       5 //蓝牙模块关闭
		
//
#define  BLE_MAX_DEVICE_NUM   4	
#define  BLE_MAC_ASCII_LENGTH   17 //ASCII形式带：
#define  BLE_DEVICE_NAME_LENGTH  10//8 
#define  BLE_SLAVE_DEVICE_NAME_LENGTH  8 		//RNGC6710													
#define  BLE_APPLICATION_DATA_LENGTH   8 		//蓝牙应用数据包长													
//	

#define TYPE_AT_SEARCH      0//查询
#define TYPE_AT_PARAM_RANGE 1//参数范围
#define TYPE_AT_SET         2//设置
#define TYPE_AT_HANDLE      3//执行	

#define  BLE_DEVICE_GC_NAME_FILTER         "RNGC"//名称过滤	
#define  BLE_DEVICE_JT_NAME_FILTER         "RNJT"//名称过滤	

#define  CMD_TYPE_AT_SEARCH          "?\r\n"//查询指令
#define  CMD_TYPE_AT_PARAM_RANGE     "=?\r\n"//参数范围
#define  CMD_TYPE_AT_SET             '='//设置指令
#define  CMD_TYPE_AT_HANDLE          "\r\n"//执行指令														
															
#define  ERROR_AT_ERR_BUSY           "BUSY\r\n"//指令操作忙
#define  ERROR_AT_ERR_ERROR          "ERROR\r\n"//指令操作错误
#define  ERROR_AT_ERR_FAIL            "FAIL\r\n"//指令操作失败	
#define  ERROR_BLE_CONNECT_ALREADY_FAIL   "DEVICE ALREADY CONNECTED\r\nFAIL\r\n" //该设备已连接

#define  CMD_DEVICE_START_HINT          "DEVICE START\r\n"//启动提示															
#define  BLE_DISCONNECTED_HINT       " DISCONNECTED\r\n"//断开连接提示
#define  BLE_CONNECTED_HINT        " CONNECTED 0\r\n"//链接成功,只用列表第一个链接，1对多重新设计
#define  BLE_CONNECTED_TIMEOUT        " CONNECTED TIMEOUT\r\n"//链接超时
#define  BLE_SCAN_SLAVER_UUID_INFO         "-CHAR:"//从机UUID信息
#define  BLE_SCAN_SLAVER_UUID_SUCCESS        "SUCCESS\r\n"//连接成功数据包结束
#define  BLE_CONNECTED_FAIL        " CONNECTED FAIL\r\n"//连接失败，配置不对

#define  CMD_AT_BLE_DATA_REV      "+RECEIVED:0,8\r\nOUTPUT_BLE_DATA\r\n"//AT模式下来自蓝牙透传数据,牙根尖定位仪固定8字节长度设备识别码0xA0开头
#define  CMD_AT_HEAD1             "AT+"//包头1
#define  CMD_AT_ACK              	"OK\r\n"//包尾，回应
#define  CMD_AT_SCAN_RSP          " 0 E1:2B:0A:AD:55:15 -75 RNGC5515\r\n" //扫描回应包格式

#define  CMD_AT_SCAN_SET         "AT+SCAN=0,2,1,2\r\n"//扫描2秒时间//10,1,3\r\n" //限定时长扫描,扫描10秒时间，3秒后启动链接
#define  CMD_AT_NAME_SERTCH        "AT+NAME?\r\n"// seartch
#define  CMD_AT_NAME_CMP         "AT+NAME=0,EDMx"//名称前四个字节//9527\r\n" //设备名称比较,名称过滤	(EDMx)
//#define  CMD_AT_NAME_CMP         "AT+NAME=0,Tv700u-D55564\r\nOK\r\n"//名称前四个字节//9527\r\n" //设备名称比较,名称过滤	(EDMx)	
#define  CMD_AT_MODE_DISABLE     	"AT+EXIT\r\n"
#define  CMD_AT_MODE_ENABLE     	"+++"
#define  CMD_AT_RESTART           "AT+RESTART\r\n"
#define  CMD_AT_SCAN             "AT+SCAN=1,2,1\r\n"//限定2s时长扫描\r\n"// 启动一次扫描
#define  CMD_AT_RESET              "AT+RESET\r\n"// 恢复出厂
#define  CMD_AT_MAC_INQUIRE      	      "AT+MAC?\r\n"
#define  CMD_AT_ADV_NAME_SET          	"AT+NAME=0,"//0,ASCII码
#define  CMD_AT_ROLE_ONLY_MASTER_SET    "AT+ROLE=1\r\n"//0,单从，1单主，2主从一体

#define  CMD_AT_POWER                  "AT+POWER=4\r\n"//默认为0dBm发射功率 =[-20, -18, -15, -12, -10, -9, -6, -5, -3, 0, 1, 2, 3, 4, 5, 8]db
#define  CMD_AT_MASTER_SERVER_INQUIRE  "AT+MSERVICE?\r\n"
#define  CMD_AT_MASTER_UUID_SET        "AT+MSERVICE=2,000015231212FEED1523785FEABCD123,000015231212FEED1523785FEABCD123,000015241212FEED1523785FEABCD123,000015251212FEED1523785FEABCD123\r\n"
//#define  CMD_AT_OBSERVER               "AT+OBSERVER=0,02,,,,,524E4743\r\n"//名称过滤,串口不打印信息 //"AT+OBSERVER=1,02,,,,,524E4743\r\n"//名称过滤,立即开启串口打印信息
#define  CMD_AT_OBSERVER               "AT+OBSERVER=0,2,00:00:00:00:00:00,RNGC,0,0000,524E4743\r\n"//名称过滤,串口不打印信息 //"AT+OBSERVER=1,02,,,,,524E4743\r\n"//名称过滤,立即开启串口打印信息

#define  CMD_AT_UUID_SCAN            		"AT+UUID_SCAN=1\r\n"//获取链接从机UUID通道、属性选择不打印,0，不打印，1打印
#define  CMD_AT_TRX_CHAN                "AT+TRX_CHAN=0,5,4,1\r\n"//UUID主机数据传输通道，固定
#define  CMD_AT_CONNECT               "AT+CONNECT="//0\r\n"//UUID主机数据传输通道，固定
#define  CMD_AT_DISCONNECT               "AT+DISCONNECT\r\n"//断开所有连接=1,0"//0\r\n"//断开主角色下连接，固定
	#define CMD_AT_VERSION                  "AT+VERSION\r\n" //版本查询

#define  CMD_AT_AUTO_CONNECT           	 "AT+AUTO_CNT=1\r\n"//all AUTO connect//F1:76:7A:CA:67:10\r\n"
#define  CMD_AT_TTM_HANDLE             	 "AT+TTM_HANDLE=0\r\n"
#define  CMD_AT_DELETE_AUTO_CONNECT      "AT+DEV_DEL=F1:76:7A:CA:67:10\r\n"
#define  CMD_AT_DEL_AUTO_CONNECT_ALL     "AT+DEV_DEL=ALL\r\n"
#define  CMD_AT_SLEEP                    "AT+SLEEP=1,1\r\n"//打开串口，关闭BLE功能

typedef enum {	
	AT_NULL=0,//idle wait
	UNVARNISHED_BLE_DATA_REV,		//透传模式下数据	
	AT_BLE_DATA_REV,//AT模式下来自其他蓝牙透传数据	
	AT_BLE_CMD_DATA,//AT模式下指令回复数据
	AT_DEVICE_START_HINT,
	AT_ONLY_ACK,
	AT_MODE_ENABLE,//指令
	AT_NAME,
	AT_MAC,
	AT_ROLE,
	AT_POWER,
	AT_ADS,//查询从角色广播参数
	AT_EN_PIN,
	AT_ADV_DATA,	
	AT_RSP_DATA,	
	AT_BEACON,
	AT_SCAN,	
  AT_SEND,	
	AT_CONNECT,
	AT_CNT_LIST,	
	AT_DISCONNECT,
	AT_AUTO_CNT,	
	AT_DEV_DEL,	
	AT_CNT_INTERVAL,	
	AT_TTM_HANDLE,
	AT_SERVICE ,//从角色UUID
	AT_MSERVICE ,//主角色UUID
	AT_UUID_SCAN ,//扫描打印从机UUID信息
	AT_TRX_CHAN ,
	AT_READ_UUID ,
	AT_OBSERVER ,	//
	AT_SLEEP,
	AT_RESTART,
	AT_RESET, //恢复出厂并复位	
	AT_VERSION, //版本查询	
	AT_MODE_DISABLE,//exit at_MODE
	DISCONNECTED_HINT,//断开连接	
	CONNECTED_SUCCESS,//连接陈工
	CONNECTED_TIMEOUT,
	CONNECTED_FAIL,//连接失败
	AT_SCANNING_STAUS,//正在扫描
	AT_DEL_AUTO_CONNECT_ALL,//删除自动重连列表
	AT_SCAN_SET,	
	AT_NAME_SET,	
	AT_ERR_BUSY,             
	AT_ERR_ERROR ,            
	AT_ERR_FAIL ,   //45
	AT_LOCAL_NAME_ERR , //设备名称没有改过
	
}AT_CMD_NUMBER;	

typedef struct {
	unsigned short int  Len;	
	unsigned char  dataBuff[USART3_MAX_FIFO_LEN];	
}BLE_RECEIVE_DATA_PACKAGE;	

extern unsigned char BLE_app_data_tx_buf[BLE_APPLICATION_DATA_LENGTH];//固定数据包
extern BLE_RECEIVE_DATA_PACKAGE	ble_package;	

extern AT_CMD_NUMBER MCU_at_cmd_analysis(AT_CMD_NUMBER atCmd);
unsigned char  MCU_usart3_read_bytes(AT_CMD_NUMBER atMode);//数据提取，数据包检索
void BLE_2340C2_gpio_init(void);
void BLE_rst(unsigned char  setOrReset);
void BLE_power_off(unsigned char OnOff);
void BLE_transmit(AT_CMD_NUMBER cmdNum,unsigned char typeOrLen,unsigned char *str);
 AT_CMD_NUMBER  MCU_BLE_cmd_data_handle(unsigned char *pDat,AT_CMD_NUMBER atCmd);
															
#endif
