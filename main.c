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

#include "app_config.h"

#include "print.h"

#define FIFO_BUFFER_SIZE 1024
FIFO_BUFFER FIFO_Queue;
FIFO_BUFFER *Queue_log = &FIFO_Queue;
static uint8_t FIFO_Buffer[FIFO_BUFFER_SIZE] = {0};

/* Task Configure. */
#define LED_TEST_TASK_PRIORITY 1
#define LED_TEST_TASK_STK_SIZE 130
TaskHandle_t LedTestTaskHanle;
static void ledTestTask( void *pvParameters );

#define VL6180x_TASK_PRIORITY 0
#define VL6180x_TASK_STK_SIZE 130
TaskHandle_t VL6180xTaskHanle;
static void VL6180xTask( void *pvParameters );

/*
	FLASH:256KB,start:0x8000000,size:0x40000
	RAM:48KB,start:0x20000000,size:0xC000
	CPU:120MHz
*/

int main(void)
{
	  nvic_vector_table_set(NVIC_VECTTAB_FLASH, APP_OFFSET);
	  __enable_irq();
	  
	  FIFO_Callback_Init(Queue_log,usart0_dma_send,usart0_dma_recv);
		FIFO_Init(Queue_log,FIFO_Buffer,FIFO_BUFFER_SIZE);
		
		bsp_usart_init(115200);
		bsp_iic_init(I2C0);
	
		/* led gpio init */
		rcu_periph_clock_enable(RCU_GPIOC);
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);
		GPIO_BOP(GPIOC) = GPIO_PIN_13;
	
		/* vl6180x gpio0 init */
		rcu_periph_clock_enable(RCU_GPIOB);
		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
		GPIO_BC(GPIOB) = GPIO_PIN_8;
	
		xTaskCreate(ledTestTask, "ledTestTask", LED_TEST_TASK_STK_SIZE, NULL, LED_TEST_TASK_PRIORITY, &LedTestTaskHanle);
		xTaskCreate(VL6180xTask, "VL6180xTask", VL6180x_TASK_STK_SIZE, NULL, VL6180x_TASK_PRIORITY, &VL6180xTaskHanle);
	
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

SemaphoreHandle_t VL6180xSemaphore;

static void VL6180xTask( void *pvParameters )
{
	 VL6180x_RangeData_t RangeData;
	 VL6180xSemaphore = xSemaphoreCreateBinary();
	 Sample_Interrupt();
	 bsp_gpio_exti_init();
	 
	 while(1)
	 {
		    xSemaphoreTake(VL6180xSemaphore, portMAX_DELAY);
        VL6180x_RangeGetMeasurement(theVL6180xDev, &RangeData);
        if( RangeData.errorStatus == 0){
            MyDev_ShowRange(theVL6180xDev, RangeData.range_mm, 0);
						printf("range_mm = %d\r\n",RangeData.range_mm);
        }
        else{
            MyDev_ShowErr(theVL6180xDev, RangeData.errorStatus);
        } 
	 }
}
