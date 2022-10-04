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

/* global variable */
SemaphoreHandle_t VL6180xSemaphore;
SemaphoreHandle_t Usart1TxSemaphore;
SemaphoreHandle_t Usar0TxSemaphore;

/* FIFO Configure */
#define LOG_BUFFER_SIZE 1024
FIFO_BUFFER log_queue;
FIFO_BUFFER *Queue_log = &log_queue;
static uint8_t LOG_Buffer[LOG_BUFFER_SIZE] = {0};

#define USART1_RX_BUFFER_SIZE 1024
FIFO_BUFFER communicate_rx_queue;
FIFO_BUFFER *Queue_Usart1_RX = &communicate_rx_queue;
static uint8_t Usart1_RX_Buffer[USART1_RX_BUFFER_SIZE] = {0};

#define USART1_TX_BUFFER_SIZE 1024
FIFO_BUFFER usart1_tx_queue;
FIFO_BUFFER *Queue_Usart1_TX = &usart1_tx_queue;
static uint8_t Usart1_TX_Buffer[USART1_TX_BUFFER_SIZE] = {0};

/* Task Configure. */
#define INIT_TASK_PRIORITY 1
#define INIT_TASK_STK_SIZE 100
TaskHandle_t InitTaskHanle;
static void InitTask( void *pvParameters );

#define LOG_TASK_PRIORITY 2
#define LOG_TASK_STK_SIZE 100
TaskHandle_t LogTaskHanle;
static void LogTask( void *pvParameters );

#define EMERGENCY_TASK_PRIORITY 3
#define EMERGENCY_TASK_STK_SIZE 100
TaskHandle_t EmergencyTaskHanle;
static void EmergencyTask( void *pvParameters );

#define VL6180x_TASK_PRIORITY 4
#define VL6180x_TASK_STK_SIZE 100
TaskHandle_t VL6180xTaskHanle;
static void VL6180xTask( void *pvParameters );

#define COMMUNICATION_TASK_PRIORITY 5
#define COMMUNICATION_TASK_STK_SIZE 1024
TaskHandle_t CommunicationTaskHanle;
static void CommunicationTask( void *pvParameters );

int main(void)
{
	  __enable_irq();
	  
		nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
			
		VL6180xSemaphore = xSemaphoreCreateBinary();
		Usart1TxSemaphore = xSemaphoreCreateBinary();
	  Usar0TxSemaphore = xSemaphoreCreateBinary();

		xTaskCreate(InitTask, "InitTask", INIT_TASK_STK_SIZE, NULL, INIT_TASK_PRIORITY, &InitTaskHanle);	
	
		vTaskStartScheduler();

		/* Should not get here */
		while(1);
}

static void InitTask( void *pvParameters )
{
	  FIFO_Callback_Init(Queue_log,usart0_dma_send,NULL);
		FIFO_Init(Queue_log,LOG_Buffer,LOG_BUFFER_SIZE);

	  FIFO_Callback_Init(Queue_Usart1_RX,NULL,usart1_dma_recv);
		FIFO_Init(Queue_Usart1_RX,Usart1_RX_Buffer,USART1_RX_BUFFER_SIZE);	
	
	  FIFO_Callback_Init(Queue_Usart1_TX,usart1_dma_send,NULL);
		FIFO_Init(Queue_Usart1_TX,Usart1_TX_Buffer,USART1_TX_BUFFER_SIZE);
		
		dma_usart1_init(460800);
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

		xTaskCreate(VL6180xTask, "VL6180xTask", VL6180x_TASK_STK_SIZE, NULL, VL6180x_TASK_PRIORITY, &VL6180xTaskHanle);
		xTaskCreate(LogTask, "LogTask", LOG_TASK_STK_SIZE, NULL, LOG_TASK_PRIORITY, &LogTaskHanle);
		xTaskCreate(EmergencyTask, "EmergencyTask", EMERGENCY_TASK_STK_SIZE, NULL, EMERGENCY_TASK_PRIORITY, &EmergencyTaskHanle);
		xTaskCreate(CommunicationTask, "CommunicationTask", COMMUNICATION_TASK_STK_SIZE, NULL, COMMUNICATION_TASK_PRIORITY, &CommunicationTaskHanle);

		printf("APP is Running,Start Address = 0x%x,Stack Address = 0x%x\r\n",*((volatile uint32_t*)APP_ADDR_ADDRESS),*((volatile uint32_t*)(*((volatile uint32_t*)APP_ADDR_ADDRESS))));

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
			 DataFrame_Handle();
			 DataFrame_Transmit();
			 vTaskDelay(pdMS_TO_TICKS(20));
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
