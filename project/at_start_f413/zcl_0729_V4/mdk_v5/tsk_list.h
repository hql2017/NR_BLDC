#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern void vAppPeriodicTask( void * pvParameters );	
extern void vAppMotorControlTask( void * pvParameters );
extern void vAppMenuManageTask( void * pvParameters );
extern void MenuDevicePowerOff(unsigned char feedDogFlag);



extern TaskHandle_t  beepTemporaryTask_Handle;
extern QueueHandle_t  xQueueMotorControlMessage;


extern QueueHandle_t   xQueueMenuValue;
extern QueueHandle_t  xQueueKeyMessage;//key
extern QueueHandle_t  xQueueBeepMode;//beep
extern QueueHandle_t  xQueueBatValue;//batValue
extern QueueHandle_t  xQueueMotorControlMessage;
	
extern SemaphoreHandle_t xSemaphorePowerOff;
extern SemaphoreHandle_t xSemaphoreCaliFinish;
extern SemaphoreHandle_t xSemaphoreDispRfresh;