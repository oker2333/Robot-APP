#ifndef HANDLER_H_
#define HANDLER_H_

#include <stdint.h>
#include <stdbool.h>

#include "synchronous.h"

#define CALLBACK_NUM 6

#define COMAND_MASK 0x0F00U

#define SUCCEED 0X00
#define FAIL		0X01

typedef void (*msg_handler_t)(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength);

typedef enum{
	/* ONLINE */
	ONLINE_HEARTBEAT = 0x0002,
	ONLINE_HEARTBEAT_ACK = 0x8002,
	
	/* CONTROL */
	CONTROL_SPEED = 0x0202,
	CONTROL_ACK = 0x8200,
	
	/* UPLOAD */
	UPLOAD_KEY_TYPE = 0x0407,
	UPLOAD_ACK = 0x8400,
	
	/* OTA */
	OTA_START = 0x0500,
	OTA_ACK = 0x8500,
	OTA_END = 0x0502,
	OTA_FRAME = 0x0503
}msg_cmd_t;

bool datalink_frame_send(msg_cmd_t cmd,Sensor_Id_t id,uint8_t* buffer,uint16_t len);


#endif
