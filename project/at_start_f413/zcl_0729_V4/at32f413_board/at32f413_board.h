/**
  **************************************************************************
  * @file     at32f415_board.h
  * @version  v2.0.3
  * @date     2022-02-11
  * @brief    header file for at-start board. set of firmware functions to 
  *           manage leds and push-button. initialize delay function.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef  __AT32F413_BOARD_H
#define  __AT32F413_BOARD_H


#ifdef __cplusplus
extern "C" {
#endif
#include "at32f413.h"

#define TASK_PRIORITY_HIGHEST   6
#define TASK_PRIORITY_HIGH   5
#define TASK_PRIORITY_MIDDLE   4
#define TASK_PRIORITY_LOWEST   3

#ifndef LED_INDICATE_ENABLE
#define LED_INDICATE_ENABLE
#endif

#ifdef DEBUG_RTT
#define DEBUG_RTT
#include "SEGGER_RTT.h"
#endif 
#ifndef BLE_FUN
#define BLE_FUN
#include "BLE_2340C2.h"
#endif 
#ifndef WDT_ENABLE	
	#define WDT_ENABLE
	#define EVENT_BIT0  (1<<0)
	#define EVENT_BIT1  (1<<1)
	#define EVENT_BIT2  (1<<2)
	#define EVENT_BIT3  (1<<3)
	#define EVENT_BIT4  (1<<4)
	#define EVENT_BIT5  (1<<5)
  #ifdef BLE_FUN
	#define EVENT_BIT6  (1<<6)
	#define EVENT_BITS_ALL  (EVENT_BIT0|EVENT_BIT1|EVENT_BIT2|EVENT_BIT3|EVENT_BIT4|EVENT_BIT5|EVENT_BIT6)
  #else
  #define EVENT_BITS_ALL  (EVENT_BIT0|EVENT_BIT1|EVENT_BIT2|EVENT_BIT3|EVENT_BIT4|EVENT_BIT5)
  #endif
	
	#define 	KEY_TASK_EVENT_BIT               EVENT_BIT0 
	#define   MENU_TASK_EVENT_BIT              EVENT_BIT1  
	#define   MOTOR_CONTROL_TASK_EVENT_BIT      EVENT_BIT2 
	#define 	PERIODIC_TASK_EVENT_BIT           EVENT_BIT3 	 
	#define 	BEEP_TASK_EVENT_BIT           EVENT_BIT4
	#define 	APEX_TASK_EVENT_BIT           EVENT_BIT5

  #ifdef BLE_FUN
	#define 	BLE_TASK_EVENT_BIT           EVENT_BIT6
  #endif	
	
	#define MAX_WDT_FEED_TIME_MS    300
#endif 

#define SYSTEM_CLOCK_FREQUENCY  200000000U//系统时钟频率
#define SYSTEM_TIME_BASE_MS 5U//系统时钟计数单位ms

#ifndef  APEX_FUNCTION_EBABLE //二合一PDO-pro
#define  APEX_FUNCTION_EBABLE  
#define  APEX_GC_IN    0
#define  APEX_GC_EXIT    1

#define  APEX_RELEY_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define  APEX_RELEY_PORT   GPIOA
#define  APEX_RELEY_IO     GPIO_PINS_12

#define  APEX_INSERT_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define  APEX_INSERT_PORT   GPIOA
#define  APEX_INSERT_IO     GPIO_PINS_10

#define  APEX_PWM_CLOCK  	 CRM_GPIOA_PERIPH_CLOCK
#define  APEX_PWM_PORT   		GPIOA
#define  APEX_PWM_IO    		GPIO_PINS_0

#define  APEX_8K_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define  APEX_8K_PORT   GPIOA
#define  APEX_8K_IO     GPIO_PINS_1

#define  APEX_400_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define  APEX_400_PORT   GPIOA
#define  APEX_400_IO     GPIO_PINS_2

#endif
/** @defgroup BOARD_pins_definition 
  * @{
  */
#define LCDSPI    SPI2
#define LCDSPI_CLOCK  CRM_SPI2_PERIPH_CLOCK

#define LCD_RST_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LCD_RST_PORT   GPIOB
#define LCD_RST_IO     GPIO_PINS_7

#define LCD_DC_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LCD_DC_PORT   GPIOB
#define LCD_DC_IO     GPIO_PINS_6

#define LCD_CS_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LCD_CS_PORT   GPIOB
#define LCD_CS_IO     GPIO_PINS_9

#define LCD_CLK_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LCD_CLK_PORT   GPIOB
#define LCD_CLK_IO     GPIO_PINS_3

//#define LCD_MISO_CLOCK  CRM_GPIOB_PERIPH_CLOCK
//#define LCD_MISO_PORT   GPIOB
//#define LCD_MISO_IO     GPIO_PINS_4

#define LCD_MOSI_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LCD_MOSI_PORT   GPIOB
#define LCD_MOSI_IO     GPIO_PINS_5

#define LEDB_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LEDB_PORT   GPIOB
#define LEDB_IO      GPIO_PINS_15

#define LEDY_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define LEDY_PORT   GPIOB
#define LEDY_IO      GPIO_PINS_14

#define SYS_ON_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define SYS_ON_PORT   GPIOA
#define SYS_ON_IO     GPIO_PINS_15

#define PWR_SW_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define PWR_SW_PORT   GPIOB
#define PWR_SW_IO     GPIO_PINS_4

#define KEY_S_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define KEY_S_PORT   GPIOA
#define KEY_S_IO     GPIO_PINS_8

#define KEY_A_CLOCK  CRM_GPIOC_PERIPH_CLOCK
#define KEY_A_PORT   GPIOC
#define KEY_A_IO     GPIO_PINS_14

#define KEY_D_CLOCK  CRM_GPIOA_PERIPH_CLOCK
#define KEY_D_PORT   GPIOA
#define KEY_D_IO     GPIO_PINS_11

#define INSERT_CLOCK  CRM_GPIOF_PERIPH_CLOCK
#define INSERT_PORT   GPIOF
#define INSERT_IO     GPIO_PINS_7

#define CHR_CLOCK  CRM_GPIOF_PERIPH_CLOCK  //charge
#define CHR_PORT   GPIOF
#define CHR_IO     GPIO_PINS_6

//BLE_2340C2
#define USART3_CRM_CLK  CRM_USART3_PERIPH_CLOCK
#define USART3TX_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define USART3TX_PORT   GPIOB
#define USART3TX_IO     GPIO_PINS_10

#define USART3RX_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define USART3RX_PORT   GPIOB
#define USART3RX_IO     GPIO_PINS_11

#define BLE_2340C2_RESET_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define BLE_2340C2_RESET_PORT   GPIOB
#define BLE_2340C2_RESET_IO     GPIO_PINS_12

#define BLE_2340C2_TRANS_FINISHED_CLOCK  CRM_GPIOC_PERIPH_CLOCK
#define BLE_2340C2_TRANS_FINISHED_PORT   GPIOC
#define BLE_2340C2_TRANS_FINISHED_IO     GPIO_PINS_15

#define BLE_2340C2_CONNECT_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define BLE_2340C2_CONNECT_PORT   GPIOB
#define BLE_2340C2_CONNECT_IO     GPIO_PINS_13

//beep
#define BUZZ_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define BUZZ_PORT   GPIOB
#define BUZZ_IO     GPIO_PINS_8
//beep music
#define BUZZER_MODE_MUTE             0//静音
#define BUZZER_MODE_BEEP             1//提示音
#define BUZZER_MODE_MOTOR_REVERSE    2 
#define BUZZER_MODE_GC_APEX          3
#define BUZZER_MODE_GC_OVER          4
#define BUZZER_MODE_GC_CONNECT_TEST  5
#define BUZZER_MODE_MOTOR_REVERSE_LONG   6 //长提示音
#define BUZZER_MODE_POWER_ON_HINT_VOL   7 //开机提示音
#define BUZZER_MODE_GC_ZREO_APEX          8//零点提示音
//key
#define KEY_RUN_NUM 0
#define KEY_S_NUM 1
#define KEY_ADD_NUM 2
#define KEY_SUB_NUM 3

#define MAX_KEY_NUM 4
//led

#define LED_Y     0
#define LED_B     1
#define LED_OFF 0
#define LED_KEEP_ON  0xFFFF  

#define LED_T_IDLE   0
#define LED_T_HIGH_PRIORITY   1//

/** @defgroup BOARD_exported_functions
  * @{
  */
/******************** functions ********************/

/* delay function */
void delay_init(unsigned int tickFrequency);
void delay_us(unsigned int nus);
void delay_ms(unsigned short int nms);
void delay_sec(unsigned short int sec);

extern void Suspend_RTOS_stick(void);
extern void Resume_RTOS_stick(void);

#ifdef __cplusplus
}
#endif

#endif




