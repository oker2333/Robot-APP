#ifndef IAP_CONFIG_H_
#define IAP_CONFIG_H_

#include "fifo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define FMC_PAGE_NUM					((uint16_t)0x80U)
#define FMC_PAGE_SIZE					((uint32_t)0x00000800U)

#define FLASH_TOTAL_SIZE 			((uint32_t)0x40000U)

#define APP_FLASH_SIZE         APP_A_FLASH_SIZE

#define FLASH_BASE_ADDR 			((uint32_t)0x08000000U)

#define IAP_ADDRESS 					((uint32_t)0x08000000U)
#define APP_A_ADDRESS 				((uint32_t)0x08004000U)
#define APP_B_ADDRESS 				((uint32_t)0x08021000U)
#define APP_ADDR_ADDRESS			((uint32_t)0x0803E000U)
#define INFO_ADDRESS 					((uint32_t)0x0803E800U)
#define FLASH_END_ADDR 				((uint32_t)0x0803FFFFU)

#define IAP_FLASH_SIZE 				((uint32_t)(APP_A_ADDRESS - IAP_ADDRESS))
#define APP_A_FLASH_SIZE 			((uint32_t)(APP_B_ADDRESS - APP_A_ADDRESS))
#define APP_B_FLASH_SIZE 			((uint32_t)(APP_ADDR_ADDRESS - APP_B_ADDRESS))
#define APP_ADDR_FLASH_SIZE 	((uint32_t)(INFO_ADDRESS - APP_ADDR_ADDRESS))
#define INFO_FLASH_SIZE 			((uint32_t)(FLASH_END_ADDR - INFO_ADDRESS + 1))

extern FIFO_BUFFER *Queue_Communicate_RX;
extern FIFO_BUFFER *Queue_log;
extern FIFO_BUFFER *Queue_Communicate_RX;
extern SemaphoreHandle_t CommunicationSemaphore;
extern SemaphoreHandle_t VL6180xSemaphore;

#endif
