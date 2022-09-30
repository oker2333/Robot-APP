/* Firmware includes. */
#include "gd32f30x_libopt.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Hardware includes. */
#include "bsp.h"

/* Sensor Drivers */
#include "vl6180x_sample_plat.h"

#include <limits.h>

#include "fifo.h"

#include "fms.h"

#include "app_config.h"

#include "print.h"

SemaphoreHandle_t VL6180xSemaphore;
SemaphoreHandle_t CommunicationSemaphore;

#define LOG_BUFFER_SIZE 1024
FIFO_BUFFER log_queue;
FIFO_BUFFER *Queue_log = &log_queue;
static uint8_t LOG_Buffer[LOG_BUFFER_SIZE] = {0};

#define COMMUNICATE_RX_BUFFER_SIZE 1024
FIFO_BUFFER communicate_rx_queue;
FIFO_BUFFER *Queue_Communicate_RX = &communicate_rx_queue;
static uint8_t Communicate_RX_Buffer[COMMUNICATE_RX_BUFFER_SIZE] = {0};

#define COMMUNICATE_TX_BUFFER_SIZE 1024
FIFO_BUFFER communicate_tx_queue;
FIFO_BUFFER *Queue_Communicate_TX = &communicate_tx_queue;
static uint8_t Communicate_TX_Buffer[COMMUNICATE_TX_BUFFER_SIZE] = {0};

/* Task Configure. */
#define LED_TEST_TASK_PRIORITY 1
#define LED_TEST_TASK_STK_SIZE 130
TaskHandle_t LedTestTaskHanle;
static void ledTestTask( void *pvParameters );

#define LOG_TASK_PRIORITY 2
#define LOG_TASK_STK_SIZE 130
TaskHandle_t LogTaskHanle;
static void LogTask( void *pvParameters );

#define EMERGENCY_TASK_PRIORITY 3
#define EMERGENCY_TASK_STK_SIZE 130
TaskHandle_t EmergencyTaskHanle;
static void EmergencyTask( void *pvParameters );

#define VL6180x_TASK_PRIORITY 4
#define VL6180x_TASK_STK_SIZE 130
TaskHandle_t VL6180xTaskHanle;
static void VL6180xTask( void *pvParameters );

#define COMMUNICATION_TASK_PRIORITY 5
#define COMMUNICATION_TASK_STK_SIZE 130
TaskHandle_t CommunicationTaskHanle;
static void CommunicationTask( void *pvParameters );

int main(void)
{
	  __enable_irq();
	  
		nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
	
	  FIFO_Callback_Init(Queue_log,usart0_dma_send,NULL);
		FIFO_Init(Queue_log,LOG_Buffer,LOG_BUFFER_SIZE);

	  FIFO_Callback_Init(Queue_Communicate_RX,NULL,usart1_dma_recv);
		FIFO_Init(Queue_Communicate_RX,Communicate_RX_Buffer,COMMUNICATE_RX_BUFFER_SIZE);	
	
	  FIFO_Callback_Init(Queue_Communicate_TX,usart1_dma_send,NULL);
		FIFO_Init(Queue_Communicate_TX,Communicate_TX_Buffer,COMMUNICATE_TX_BUFFER_SIZE);
		
		dma_usart1_init(115200);
		bsp_usart_init(460800);
		bsp_iic_init(I2C0);
	
		/* led gpio init */
		rcu_periph_clock_enable(RCU_GPIOC);
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);
		GPIO_BOP(GPIOC) = GPIO_PIN_13;
	
		/* vl6180x gpio0 init */
		rcu_periph_clock_enable(RCU_GPIOB);
		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
		GPIO_BC(GPIOB) = GPIO_PIN_8;
	
		printf("APP Current Address = 0x%x\n",*((volatile uint32_t*)APP_ADDR_ADDRESS));
		
		VL6180xSemaphore = xSemaphoreCreateBinary();
		CommunicationSemaphore = xSemaphoreCreateBinary();
	
		xTaskCreate(ledTestTask, "ledTestTask", LED_TEST_TASK_STK_SIZE, NULL, LED_TEST_TASK_PRIORITY, &LedTestTaskHanle);
		xTaskCreate(VL6180xTask, "VL6180xTask", VL6180x_TASK_STK_SIZE, NULL, VL6180x_TASK_PRIORITY, &VL6180xTaskHanle);
		xTaskCreate(LogTask, "LogTask", LOG_TASK_STK_SIZE, NULL, LOG_TASK_PRIORITY, &LogTaskHanle);
		xTaskCreate(EmergencyTask, "EmergencyTask", EMERGENCY_TASK_STK_SIZE, NULL, EMERGENCY_TASK_PRIORITY, &EmergencyTaskHanle);
		xTaskCreate(CommunicationTask, "CommunicationTask", COMMUNICATION_TASK_STK_SIZE, NULL, COMMUNICATION_TASK_PRIORITY, &CommunicationTaskHanle);
	
		vTaskStartScheduler();

		/* Should not get here */
		while(1);
}

static void ledTestTask( void *pvParameters )
{
		while(1)
		{
				GPIO_BC(GPIOC) = GPIO_PIN_13;
				vTaskDelay(pdMS_TO_TICKS(1000));
				GPIO_BOP(GPIOC) = GPIO_PIN_13;
				vTaskDelay(pdMS_TO_TICKS(1000));
		}
}

static void CommunicationTask( void *pvParameters )
{
		while(1)
		{
			 xSemaphoreTake(CommunicationSemaphore, portMAX_DELAY);
			 FIFO_Recv(Queue_Communicate_RX);
			 DataFrame_Handle();
		}
}

static void VL6180xTask( void *pvParameters )
{
	 VL6180x_RangeData_t RangeData;
	 Sample_Interrupt();
	 bsp_gpio_exti_init();
	 
	 while(1)
	 {
		    xSemaphoreTake(VL6180xSemaphore, portMAX_DELAY);
        VL6180x_RangeGetMeasurement(theVL6180xDev, &RangeData);
        if( RangeData.errorStatus == 0){
            MyDev_ShowRange(theVL6180xDev, RangeData.range_mm, 0);
					  print_info("RangeData.range_mm = %d\n",RangeData.range_mm);
        }
        else{
            MyDev_ShowErr(theVL6180xDev, RangeData.errorStatus);
        }
				VL6180x_RangeClearInterrupt(theVL6180xDev);
				VL6180x_INT_Enable();
	 }
}

static void LogTask(void *pvParameters)
{
	  while(1)
		{
			 print_logs();
			 vTaskDelay(pdMS_TO_TICKS(100));
		}
}

static void EmergencyTask(void *pvParameters)
{
	  while(1)
		{
			 //Reset the Breakdown Sensors
			 
			 vTaskDelay(pdMS_TO_TICKS(1000));
		}
}
