#ifndef HANDLER_H_
#define HANDLER_H_

#include <stdint.h>

#define CALLBACK_NUM 6

#define COMAND_MASK 0x0F00U

#define SUCCEED 0X00
#define FAIL		0X01

typedef void (*msg_handler_t)(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength);

typedef enum{
	
	/* OTA */
	OTA_START = 0x0500,
	OTA_ACK = 0x8500,
	OTA_END = 0x0502,
	OTA_FRAME = 0x0503
}msg_cmd_t;



#endif
