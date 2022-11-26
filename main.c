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

#include "shell.h"

#include "console.h"

#include "fms.h"

#include "ota.h"

#include "timing.h"

#include "timer.h"

#include "pid.h"

#include "switch.h"

#include "IRremote.h"

#include "app_config.h"

#include "print.h"

#if JSON
#include "cJSON.h"
#endif

#include "driver_mpu6050_register_test.h"
#include "driver_mpu6050_read_test.h"
#include "driver_mpu6050_fifo_test.h"
#include "driver_mpu6050_dmp_read_test.h"
#include "driver_mpu6050_dmp_tap_orient_motion_test.h"
#include "driver_mpu6050_dmp_pedometer_test.h"
#include "driver_mpu6050_basic.h"
#include "driver_mpu6050_fifo.h"
#include "driver_mpu6050_dmp.h"

/* global variable */
SemaphoreHandle_t VL6180xSemaphore;
SemaphoreHandle_t Usart1TxSemaphore;
SemaphoreHandle_t Usar0TxSemaphore;

/* FIFO Configure */
#if FIFO_DEBUG
#define LOG_BUFFER_SIZE 1024
FIFO_BUFFER log_queue;
FIFO_BUFFER *Queue_log = &log_queue;
static uint8_t LOG_Buffer[LOG_BUFFER_SIZE] = {0};
#endif

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

#if FIFO_DEBUG
#define LOG_TASK_PRIORITY 2
#define LOG_TASK_STK_SIZE 100
TaskHandle_t LogTaskHanle;
static void LogTask( void *pvParameters );
#endif

#define MPU6050_TASK_PRIORITY 2
#define MPU6050_TASK_STK_SIZE 1024
TaskHandle_t MPU6050TaskHanle;
static void MPU6050Task( void *pvParameters );

#define REMOTE_CONTROL_TASK_PRIORITY 3
#define REMOTE_CONTROL_TASK_STK_SIZE 100
TaskHandle_t RemoteControlTaskHanle;
static void RemoteControlTask( void *pvParameters );

#define SPEED_TASK_PRIORITY 4
#define SPEED_TASK_STK_SIZE 1024
TaskHandle_t SpeedTaskHanle;
static void VelocityMeasurementTask( void *pvParameters );

#define VL6180x_TASK_PRIORITY 5
#define VL6180x_TASK_STK_SIZE 100
TaskHandle_t VL6180xTaskHanle;
static void VL6180xTask( void *pvParameters );

#define TIMING_UPLOAD_TASK_PRIORITY 6
#define TIMING_UPLOAD_TASK_STK_SIZE 1024
TaskHandle_t TimingUploadTaskHanle;
static void TimingUploadTask( void *pvParameters );

#define COMMUNICATION_TASK_PRIORITY 7
#define COMMUNICATION_TASK_STK_SIZE 1024
TaskHandle_t CommunicationTaskHanle;
static void CommunicationTask( void *pvParameters );

#define ACTIVE_UPLOAD_TASK_PRIORITY 8
#define ACTIVE_UPLOAD_TASK_STK_SIZE 1024
TaskHandle_t ActiveUploadTaskHanle;
static void ActiveUploadTask( void *pvParameters );

#define CONSOLE_TASK_PRIORITY 9
#define CONSOLE_TASK_STK_SIZE 512
TaskHandle_t ConsoleTaskHanle;
static void ConsoleTask( void *pvParameters );

// printf("/*value:%f:%f*/\r\n",value1,value2);
/*
	Serial Studio json conf
*/

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
		while(pdTRUE);
}

static void InitTask( void *pvParameters )
{
		#if JSON
	  cJSON_InitHooks(NULL);
	  #endif
	  
	  #if FIFO_DEBUG
	  FIFO_Callback_Init(Queue_log,usart0_dma_send,NULL);
		FIFO_Init(Queue_log,LOG_Buffer,LOG_BUFFER_SIZE);
	  #endif

	  FIFO_Callback_Init(Queue_Usart1_RX,NULL,usart1_dma_recv);
		FIFO_Init(Queue_Usart1_RX,Usart1_RX_Buffer,USART1_RX_BUFFER_SIZE);	
	
	  FIFO_Callback_Init(Queue_Usart1_TX,usart1_dma_send,NULL);
		FIFO_Init(Queue_Usart1_TX,Usart1_TX_Buffer,USART1_TX_BUFFER_SIZE);
		
	  #if FIFO_DEBUG
	  dma_usart0_init(3000000);
	  #else
	  usart0_init(3000000);
	  #endif
		dma_usart1_init(460800);
		
		printf("APP %s is running,welcome to robot console.\n",BINARY_VERSION);
		
		bsp_iic_init(I2C0);
		
		download_address_update();
	
		switch_init();
	  
		IncPIDInit();
	 
		encoder_init();
	  
		motor_init();
		
		ir_rx_init();
	
		/* shell init && register handler fuction */
    shell_init();
    shell_register("robot", console);
	
		/* led gpio init */
		rcu_periph_clock_enable(RCU_GPIOC);
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);
		GPIO_BOP(GPIOC) = GPIO_PIN_13;
	
		/* vl6180x gpio0 init */
		rcu_periph_clock_enable(RCU_GPIOB);
		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);
		GPIO_BC(GPIOB) = GPIO_PIN_8;

		taskENTER_CRITICAL();
		
		xTaskCreate(VelocityMeasurementTask, "VelocityMeasurementTask", SPEED_TASK_STK_SIZE, NULL, SPEED_TASK_PRIORITY, &SpeedTaskHanle);
		xTaskCreate(VL6180xTask, "VL6180xTask", VL6180x_TASK_STK_SIZE, NULL, VL6180x_TASK_PRIORITY, &VL6180xTaskHanle);
		xTaskCreate(MPU6050Task, "MPU6050Task", MPU6050_TASK_STK_SIZE, NULL, MPU6050_TASK_PRIORITY, &MPU6050TaskHanle);
		#if FIFO_DEBUG
		xTaskCreate(LogTask, "LogTask", LOG_TASK_STK_SIZE, NULL, LOG_TASK_PRIORITY, &LogTaskHanle);
		#endif
		xTaskCreate(RemoteControlTask, "RemoteControlTask", REMOTE_CONTROL_TASK_STK_SIZE, NULL, REMOTE_CONTROL_TASK_PRIORITY, &RemoteControlTaskHanle);
		xTaskCreate(CommunicationTask, "CommunicationTask", COMMUNICATION_TASK_STK_SIZE, NULL, COMMUNICATION_TASK_PRIORITY, &CommunicationTaskHanle);
		xTaskCreate(TimingUploadTask, "TimingUploadTask", TIMING_UPLOAD_TASK_STK_SIZE, NULL, TIMING_UPLOAD_TASK_PRIORITY, &TimingUploadTaskHanle);
		xTaskCreate(ActiveUploadTask, "ActiveUploadTask", ACTIVE_UPLOAD_TASK_STK_SIZE, NULL, ACTIVE_UPLOAD_TASK_PRIORITY, &ActiveUploadTaskHanle);
		xTaskCreate(ConsoleTask, "ConsoleTask", CONSOLE_TASK_STK_SIZE, NULL, CONSOLE_TASK_PRIORITY, &ConsoleTaskHanle);
		
		taskEXIT_CRITICAL();

		while(pdTRUE)
		{
				GPIO_BC(GPIOC) = GPIO_PIN_13;
				vTaskDelay(pdMS_TO_TICKS(500));
				GPIO_BOP(GPIOC) = GPIO_PIN_13;
				vTaskDelay(pdMS_TO_TICKS(500));
		}
}

int16_t gs_accel_raw[MPU6050_BUFFER_SIZE][3];
float gs_accel_g[MPU6050_BUFFER_SIZE][3];
int16_t gs_gyro_raw[MPU6050_BUFFER_SIZE][3];
float gs_gyro_dps[MPU6050_BUFFER_SIZE][3];
int32_t gs_quat[MPU6050_BUFFER_SIZE][4];
float gs_pitch[MPU6050_BUFFER_SIZE];
float gs_roll[MPU6050_BUFFER_SIZE];
float gs_yaw[MPU6050_BUFFER_SIZE];

static void MPU6050Task( void *pvParameters )
{

		uint8_t res;
		uint16_t len;
		mpu6050_address_t addr = MPU6050_ADDRESS_AD0_LOW;
	
		if (mpu6050_dmp_init(addr, a_receive_callback, 
												 a_dmp_tap_callback, a_dmp_orient_callback) != 0)
		{
				return;
		}
		
		while(pdTRUE)
		{
				len = MPU6050_BUFFER_SIZE;

				if(mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
														 gs_gyro_raw, gs_gyro_dps, 
														 gs_quat,
														 gs_pitch, gs_roll, gs_yaw,
														 &len) != 0)
				{
					 continue;
				}

				printf("\033[1;1Hmpu6050: fifo %d.\n", len);
				printf("mpu6050: pitch[%d] is %0.2fdps. \n", len-1, gs_pitch[len-1]);
				printf("mpu6050: roll[%d] is %0.2fdps. \n", len-1, gs_roll[len-1]);
				printf("mpu6050: yaw[%d] is %0.2fdps. \n", len-1, gs_yaw[len-1]);
				printf("mpu6050: acc x[%d] is %0.2fg. \n", len-1, gs_accel_g[len-1][0]);
				printf("mpu6050: acc y[%d] is %0.2fg. \n", len-1, gs_accel_g[len-1][1]);
				printf("mpu6050: acc z[%d] is %0.2fg. \n", len-1, gs_accel_g[len-1][2]);
				printf("mpu6050: gyro x[%d] is %0.2fdps. \n", len-1, gs_gyro_dps[len-1][0]);
				printf("mpu6050: gyro y[%d] is %0.2fdps. \n", len-1, gs_gyro_dps[len-1][1]);
				printf("mpu6050: gyro z[%d] is %0.2fdps. \n\033[m", len-1, gs_gyro_dps[len-1][2]);

				vTaskDelay(pdMS_TO_TICKS(10));
		}
}

static void CommunicationTask( void *pvParameters )
{
		while(pdTRUE)
		{
			 DataFrame_Handle();
			 DataFrame_Transmit();
			 vTaskDelay(pdMS_TO_TICKS(10));
		}
}

uint16_t tof_mm = 0;

static void VL6180xTask( void *pvParameters )
{
	 VL6180x_RangeData_t RangeData;
	 Sample_Interrupt();
	 bsp_gpio_exti_init();
	 
	 while(pdTRUE)
	 {
		    xSemaphoreTake(VL6180xSemaphore, portMAX_DELAY);
        VL6180x_RangeGetMeasurement(theVL6180xDev, &RangeData);
        if( RangeData.errorStatus == 0){
					  tof_mm = RangeData.range_mm;
        }
        else{
            tof_mm = -1;
        }
				VL6180x_RangeClearInterrupt(theVL6180xDev);
				VL6180x_INT_Enable();
	 }
}

#if FIFO_DEBUG
static void LogTask(void *pvParameters)
{
	  while(pdTRUE)
		{
			 print_logs();
			 vTaskDelay(pdMS_TO_TICKS(100));
		}
}
#endif

uint8_t ir_value = NONE;

static void RemoteControlTask(void *pvParameters)
{
	 int32_t VELOCITY = 80;

	  while(pdTRUE)
		{
			 ir_value = IR_Key_Obtain();
			 
			 switch(ir_value)
			 {
				 case LEFT:
					 pid_motor_control(-VELOCITY,VELOCITY);
				 break;
				 
				 case RIGHT:
					 pid_motor_control(VELOCITY,-VELOCITY);
				 break;
				 
				 case UP:
					 pid_motor_control(VELOCITY,VELOCITY);
				 break;

				 case DOWN:
					 pid_motor_control(-VELOCITY,-VELOCITY);
				 break;
				 
				 case PAUSE:
					 pid_motor_control(0,0);
				 break;
				 
				 case ZERO:
					 VELOCITY -= 20;
				   printf("Remote Control Set Velocity %d mm/s\n",VELOCITY);
				 break;
				 
				 case C:
					 VELOCITY += 20;
				   printf("Remote Control Set Velocity %d mm/s\n",VELOCITY);
				 break;
				 
				 default:
					 printf("unsupported ir key type\n");
				 break;
			 }			 
		}
}

static void VelocityMeasurementTask(void *pvParameters)
{
	  while(pdTRUE)
		{
			 left_velocity_measurement(VELOCITY_MEASUREMENT_INTERVAL);
			 right_velocity_measurement(VELOCITY_MEASUREMENT_INTERVAL);
			 
			 PID_Controller();

			 vTaskDelay(pdMS_TO_TICKS(VELOCITY_MEASUREMENT_INTERVAL));

       #if JSON
			 char* cJSON_Str = NULL;
			 cJSON *cJSON_Velocity = cJSON_CreateObject();
			 cJSON_AddNumberToObject(cJSON_Velocity, "left_velocity", get_left_velocity());
			 cJSON_AddNumberToObject(cJSON_Velocity, "right_velocity", get_right_velocity());
			
			 cJSON_Str = cJSON_Print(cJSON_Velocity);
			 printf("%s\r\n",cJSON_Str);
			
			 cJSON_Delete(cJSON_Velocity);
			 free(cJSON_Str);
		   #endif
		}
}

static void TimingUploadTask(void *pvParameters)
{
	 while(pdTRUE)
	 {
		  timing_uploader();
		  vTaskDelay(pdMS_TO_TICKS(10));
	 }
}

static void ActiveUploadTask(void *pvParameters)
{
	 while(pdTRUE)
	 {
		  
		  vTaskDelay(pdMS_TO_TICKS(10));
	 }
}

static void ConsoleTask( void *pvParameters )
{
	 while(pdTRUE)
	 {
		  uint8_t len = console_header - console_tail;
		  if(len)
			{
				 shell_parse((char *)&console_buffer[console_tail%CONSOLE_BUFFER_LEN],len);
				 console_tail += len;
			}
		  vTaskDelay(pdMS_TO_TICKS(300));
	 }
}
